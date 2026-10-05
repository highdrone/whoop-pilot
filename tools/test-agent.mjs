// The Claude agent loop (app/js/agent.js) against a fake Claude API: prompt caching stays set up
// (a breakpoint on the fixed tools+system prefix plus automatic caching for the conversation), the
// fixed prefix is byte-identical across commands, history is append-only, and usage/cost add up.
// With a house map: the house summary rides in the cached prefix (byte-identical for the same house), the tools are the
// mission tools (and only those run when Claude asks), and the position goes in the messages.
import assert from "node:assert/strict";
import { Agent, SYSTEM_PROMPT, HOUSE_PROMPT } from "../app/js/agent.js";
import { TOOL_DEFS, HOUSE_TOOL_DEFS, houseSummary, ToolBox } from "../app/js/tools.js";

const bodies = [];
let replies = [];
globalThis.fetch = async (url, init) => {
  bodies.push(JSON.parse(init.body));
  const reply = replies.shift();
  return new Response(JSON.stringify({ id: `msg_${bodies.length}`, type: "message", role: "assistant", model: "claude-opus-5", ...reply }), {
    status: 200,
    headers: { "content-type": "application/json", "request-id": `req_${bodies.length}` },
  });
};

const toolUse = { stop_reason: "tool_use", content: [{ type: "text", text: "Hovering." }, { type: "tool_use", id: "toolu_1", name: "hover", input: { seconds: 1 } }] };
const done = { stop_reason: "end_turn", content: [{ type: "text", text: "Done." }] };
const usage = (input, write, read, output) => ({ usage: { input_tokens: input, cache_creation_input_tokens: write, cache_read_input_tokens: read, output_tokens: output } });

const HOUSE = {
  id: "h1", name: "Test flat",
  rooms: [
    { id: "k", name: "Kitchen", floorZ: 0, outline: [[0, 0], [3, 0], [3, 3], [0, 3]] },
    { id: "l", name: "Lounge", floorZ: 0.2, outline: [[3.1, 0], [7, 0], [7, 4], [3.1, 4]] },
  ],
  doors: [{ id: "d1", rooms: ["k", "l"], a: [3, 1], b: [3, 2], width: 1.0, passable: true }],
  landmarks: [{ name: "sofa", room: "l", x: 5, y: 2, source: "roomplan" }, { name: "cat bed", room: "k", x: 1, y: 1, source: "user" }],
  keepouts: [{ kind: "fan", x: 5, y: 3, r: 1, room: "l" }],
  home: { x: 1, y: 2, yaw: 0, source: "user" },
};

function makeAgent(house = null) {
  const settings = { values: { apiKey: "test-key", model: "claude-opus-5", effort: "low", autonomy: "copilot", homeNotes: "" } };
  settings.get = (k) => settings.values[k];
  settings.all = () => settings.values;
  const tools = {
    describe: () => "Heading 0°, 0 detections.",
    perception: { snapshot: () => "AAAA" },
    call: async () => ({ text: "Hovered 1 s.", image: "BBBB" }),
    ...(house && { house, map: null, missions: { whereAmI: () => "In the Kitchen, 1.0 m from the home pad." }, defs: () => HOUSE_TOOL_DEFS }),
  };
  const ctl = { tel: { vbat: 4.1, lq: 99 }, isFlying: () => true, blocker: () => null, imageHeading: () => 0 };
  const agent = new Agent({ tools, settings, ctl });
  const events = { usage: [], say: [] };
  agent.on("usage", (u) => events.usage.push(u));
  agent.on("say", (t) => events.say.push(t));
  agent.on("error", (m) => {
    throw new Error(m);
  });
  return { agent, events };
}

const tests = {
  async "every request caches the fixed tools+system prefix and the growing conversation"() {
    bodies.length = 0;
    replies = [{ ...toolUse, ...usage(1200, 2600, 0, 60) }, { ...done, ...usage(40, 1500, 2600, 20) }];
    const { agent } = makeAgent();
    await agent.run("hover for a second");
    assert.equal(bodies.length, 2);
    for (const b of bodies) {
      assert.deepEqual(b.cache_control, { type: "ephemeral" }, "top-level automatic caching");
      assert.deepEqual(b.system, [{ type: "text", text: SYSTEM_PROMPT, cache_control: { type: "ephemeral" } }], "breakpoint on the fixed prefix");
      assert.deepEqual(b.tools, TOOL_DEFS);
      const markers = JSON.stringify(b).match(/"cache_control"/g).length;
      assert.ok(markers <= 4, `${markers} breakpoints (max 4)`);
    }
    // append-only: the second request starts with the first one's messages, unchanged
    assert.deepEqual(bodies[1].messages.slice(0, bodies[0].messages.length), bodies[0].messages);
    assert.equal(bodies[1].messages.at(-1).content[0].type, "tool_result");
  },

  async "the fixed prefix is byte-identical from one command to the next"() {
    bodies.length = 0;
    replies = [{ ...done, ...usage(900, 2600, 0, 10) }, { ...done, ...usage(900, 0, 2600, 10) }];
    const { agent } = makeAgent();
    await agent.run("look around");
    await agent.run("turn left 90");
    const prefix = (b) => JSON.stringify([b.model, b.tools, b.system, b.thinking, b.output_config]);
    assert.equal(prefix(bodies[0]), prefix(bodies[1]));
    assert.notDeepEqual(bodies[0].messages, bodies[1].messages, "per-command context lives in messages");
    assert.ok(!/\d{4}-\d\d-\d\dT/.test(JSON.stringify(bodies[0].system)), "no timestamps in the system prompt");
  },

  async "with a house map: its summary is in the cached prefix, byte-identical across commands, with the mission tools"() {
    bodies.length = 0;
    replies = [{ ...done, ...usage(900, 3000, 0, 10) }, { ...done, ...usage(900, 0, 3000, 10) }];
    const { agent } = makeAgent(HOUSE);
    await agent.run("go to the lounge");
    await agent.run("find the cat");
    const summary = houseSummary(HOUSE);
    for (const b of bodies) {
      assert.deepEqual(b.system, [{ type: "text", text: SYSTEM_PROMPT }, { type: "text", text: HOUSE_PROMPT + summary, cache_control: { type: "ephemeral" } }]);
      assert.deepEqual(b.tools, HOUSE_TOOL_DEFS);
      assert.ok(JSON.stringify(b).match(/"cache_control"/g).length <= 4);
      assert.match(b.messages[0].content.at(-1).text, /Position: In the Kitchen, 1\.0 m from the home pad\./);
    }
    const prefix = (b) => JSON.stringify([b.model, b.tools, b.system, b.thinking, b.output_config]);
    assert.equal(prefix(bodies[0]), prefix(bodies[1]), "same house, same bytes");
    for (const t of ["Kitchen", "Lounge", "Kitchen and Lounge (1.0 m wide)", "cat bed (saved)", "sofa", "ceiling fan (Lounge)", "floor 0.20 m higher"]) assert.ok(summary.includes(t), t);
    assert.ok(!summary.includes("1.0, 2.0") && !/\d\.\d{3}/.test(summary), "no coordinates");
    replies = [{ ...done, ...usage(900, 3000, 0, 10) }];
    const renamed = { ...HOUSE, rooms: HOUSE.rooms.map((r) => (r.id === "l" ? { ...r, name: "Living room" } : r)) };
    await makeAgent(renamed).agent.run("look around");
    assert.notEqual(prefix(bodies[2]), prefix(bodies[0]), "a renamed room changes the prefix");
    assert.equal(houseSummary(structuredClone(HOUSE)), summary, "deterministic");
  },

  async "without a map the tools and system prompt are the plain ones"() {
    bodies.length = 0;
    replies = [{ ...done, ...usage(900, 2600, 0, 10) }];
    await makeAgent().agent.run("hover");
    assert.deepEqual(bodies[0].tools, TOOL_DEFS);
    assert.equal(bodies[0].system.length, 1);
    assert.ok(!/Position:/.test(bodies[0].messages[0].content.at(-1).text));
  },

  async "with a map, a hand-flying tool Claude asks for anyway is refused and nothing flies"() {
    bodies.length = 0;
    replies = [{ stop_reason: "tool_use", content: [{ type: "tool_use", id: "toolu_t", name: "turn", input: { degrees: 90 } }], ...usage(900, 0, 3000, 10) }, { ...done, ...usage(40, 0, 3000, 5) }];
    const settings = { values: { apiKey: "test-key", model: "claude-opus-5", effort: "low", autonomy: "full", homeNotes: "" } };
    settings.get = (k) => settings.values[k];
    settings.all = () => settings.values;
    const flown = [];
    const ctl = { tel: { vbat: 4.1 }, autonomy: "full", isFlying: () => true, blocker: () => null, imageHeading: () => 0, run: async (b) => (flown.push(b), { ok: true, text: "done" }) };
    const tools = new ToolBox({ ctl, perception: { latest: { detections: [] }, snapshot: () => null }, settings, speak: () => {} });
    const map = { rooms: [], room: [], o: { cell: 0.05 }, roomAt: () => null, keepouts: [] };
    tools.setHouse({ house: HOUSE, map, missions: { whereAmI: () => "In the Kitchen." } });
    await new Agent({ tools, settings, ctl }).run("turn right");
    const result = bodies[1].messages.at(-1).content[0];
    assert.equal(result.is_error, true);
    assert.match(result.content[0].text, /"turn" isn't available while the house map is active/);
    assert.equal(flown.length, 0, "no hand flying");
  },

  async "usage adds up per command, with cache share and an Opus 5 cost estimate"() {
    bodies.length = 0;
    replies = [{ ...toolUse, ...usage(1200, 2600, 0, 60) }, { ...done, ...usage(40, 1500, 2600, 20) }];
    const { agent, events } = makeAgent();
    await agent.run("hover for a second");
    const u = events.usage[0];
    assert.deepEqual([u.requests, u.input, u.cacheWrite, u.cacheRead, u.output], [2, 1240, 4100, 2600, 80]);
    assert.equal(u.cachedShare.toFixed(3), (2600 / (1240 + 4100 + 2600)).toFixed(3));
    const cost = (1240 * 5 + 80 * 25 + 2600 * 0.5 + 4100 * 6.25) / 1e6;
    assert.equal(u.cost.toFixed(6), cost.toFixed(6));
    assert.equal(u.session.commands, 1);
    replies = [{ ...done, ...usage(10, 0, 4000, 5) }];
    await agent.run("stop");
    assert.equal(events.usage[1].session.commands, 2);
    assert.equal(events.usage[1].session.cost.toFixed(6), (cost + (10 * 5 + 5 * 25 + 4000 * 0.5) / 1e6).toFixed(6));
  },
};

let failed = 0;
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name}\n      ${e.stack?.split("\n").slice(0, 3).join("\n      ")}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed`);
process.exit(failed ? 1 : 0);
