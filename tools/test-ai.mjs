// Claude's eyes (app/js/ai/*) against a fake Claude API and a fake twin on the capture fixture: the survey's requests
// (pictures, the room's plan, a JSON-schema answer, the cached system prompt), lifting boxes to 3D with the twin's depth,
// merging duplicates, the cost estimate against the usage reported after (through the agent's usage event), budgets,
// consent (no picture ever leaves without it), the change and detection checks (and alerts.js waiting for them), the new
// Claude tools (recall, what_changed, survey_house, where_am_i) and the agent keeping memory out of the cached prefix;
// and wired together as main.js will (memory, Inspector watching it, alerts, nav/changes.js): one check per change, no
// consent prompt in the air, what Claude may resolve on its own.
// The fake twin is a ray caster over a few spheres (things in the house) between the room's floor and ceiling, with a far
// background; the fake Claude boxes those spheres in the views it is sent, as a careful model would.
// Usage: cd tools && node test-ai.mjs
process.env.TZ = "UTC";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";

const I = await import("../app/js/house/import.js");
const { Claude, imageTokens } = await import("../app/js/ai/claude.js");
const S = await import("../app/js/ai/survey.js");
const { Inspector } = await import("../app/js/ai/inspect.js");
const { HouseMemory } = await import("../app/js/memory/memory.js");
const { Alerts } = await import("../app/js/alerts.js");
const { ToolBox, HOUSE_TOOL_DEFS } = await import("../app/js/tools.js");
const { Agent, SYSTEM_PROMPT, costOf } = await import("../app/js/agent.js");
const { pinholeLens, intrinsics, unproject, project } = await import("../app/js/twin/lens.js");
const { droneCamera } = await import("../app/js/twin/pose.js");
const { Emitter } = await import("../app/js/util.js");
const { ChangeDetector } = await import("../app/js/nav/changes.js");
const { askMemory, summarizeFlight } = await import("../app/js/ai/recall.js");

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const fixture = () => I.importCapture(fsSource(FIXTURE));
const { house, map } = await fixture();
const jpeg = (n = 4) => Buffer.from([0xff, 0xd8, ...Array(n).fill(9), 0xff, 0xd9]).toString("base64"); // no size in it
// A JPEG whose header says w x h (an SOF0 segment).
const jpegOf = (w, h, n = 4) => Buffer.from([0xff, 0xd8, 0xff, 0xc0, 0, 17, 8, h >> 8, h & 255, w >> 8, w & 255, 3, 1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1, ...Array(n).fill(9), 0xff, 0xd9]).toString("base64");
const settle = () => new Promise((r) => setTimeout(r, 30));

// ---- the fake world: spheres { name | kind, c: [x, y, z], r, rooms: [rooms whose survey reports it], confidence } ----
const THINGS = [
  { name: "sofa", c: [2.5, 4.2, 0.6], r: 0.4, rooms: ["r1"], confidence: 0.9 },
  { kind: "ceiling-fan", c: [2.93, 2.9, 2.4], r: 0.35, rooms: ["r1"], confidence: 0.8, why: "fan blades at head height" },
  { name: "cat tree", c: [-0.4, 4.2, 0.9], r: 0.3, rooms: ["r1", "r3"], confidence: 0.6 },
  { kind: "plant", c: [0.3, 0.6, 1.1], r: 0.25, rooms: ["r2"], confidence: 0.7, why: "leafy plant on a stand" }, // a level camera 1.3 m up sees the floor from 1.7 m out
  { name: "tree", c: [-3.0, 1.0, 1.5], r: 0.4, rooms: ["r2"], confidence: 0.5 }, // through the window: outside every room
];
const ROOM_ANSWERS = { r1: ["living room", "Lounge", 0.9], r2: ["entrance", "Entrance hall", 0.8], r3: ["hallway", "Stair hall", 0.7] };

function raySphere(o, d, s) {
  const oc = [o[0] - s.c[0], o[1] - s.c[1], o[2] - s.c[2]], b = oc[0] * d[0] + oc[1] * d[1] + oc[2] * d[2], c = oc[0] ** 2 + oc[1] ** 2 + oc[2] ** 2 - s.r * s.r, q = b * b - c;
  return q < 0 ? Infinity : -b - Math.sqrt(q) > 0 ? -b - Math.sqrt(q) : Infinity;
}
const camDir = (cam, v) => [0, 1, 2].map((i) => cam.R[i][0] * v[0] + cam.R[i][1] * v[1] + cam.R[i][2] * v[2]);
const rendered = []; // every picture any fake twin rendered: a room's request carries its last ones, in order
class FakeTwin {
  calls = [];
  constructor({ floors = true, override = null } = {}) {
    Object.assign(this, { floors, override });
  }
  async pixels(pose, view) {
    this.calls.push({ op: "pixels", pose, view });
    rendered.push(pose);
    return { width: view.width, height: view.height, data: new Uint8ClampedArray(4) };
  }
  async depth(pose, view) {
    this.calls.push({ op: "depth", pose, view });
    const K = intrinsics(view.lens, view.width, view.height), cam = droneCamera(pose, view.lens.uptiltDeg ?? 0), out = new Float32Array(view.width * view.height);
    const F = map.floorAt(pose.x, pose.y) ?? 0, C = map.ceilingAt(pose.x, pose.y) ?? F + 2.5;
    for (let v = 0; v < view.height; v++)
      for (let u = 0; u < view.width; u++) {
        const d = camDir(cam, unproject(K, u + 0.5, v + 0.5));
        const plane = !this.floors ? Infinity : d[2] < -1e-6 ? (F - cam.p[2]) / d[2] : d[2] > 1e-6 ? (C - cam.p[2]) / d[2] : Infinity;
        out[v * view.width + u] = Math.min(6, plane, ...THINGS.map((s) => raySphere(cam.p, d, s)));
      }
    this.override?.(pose, view, out);
    return out;
  }
}
let encoded = 0;
const encode = () => Buffer.from(`view-${++encoded}`).toString("base64");

// ---- the fake Claude ----
const bodies = [], heads = [];
let respond = () => ({});
const fakeFetch = async (url, init) => {
  const body = JSON.parse(init.body);
  bodies.push(body);
  heads.push(new Headers(init.headers));
  const reply = respond(body, bodies.length);
  return new Response(JSON.stringify({ id: `msg_${bodies.length}`, type: "message", role: "assistant", model: body.model, stop_reason: "end_turn", ...reply }), {
    status: 200, headers: { "content-type": "application/json", "request-id": `req_${bodies.length}` },
  });
};
const answer = (data, usage) => ({ content: [{ type: "thinking", thinking: "", signature: "x" }, { type: "text", text: JSON.stringify(data) }], usage });
const imagesOf = (b) => b.messages[0].content.filter((x) => x.type === "image");
const textOf = (b) => b.messages[0].content.filter((x) => x.type === "text").map((x) => x.text).join("\n");

// What a careful model answers for a room: the things it is told about, boxed in the view that shows them best (of the
// pictures in the request: the last ones rendered).
function surveyAnswer(body, n) {
  const room = house.rooms.find((r) => textOf(body).startsWith(`This is ${r.name} (`));
  const views = rendered.slice(-imagesOf(body).length), lens = pinholeLens(S.SURVEY.hfov), W = S.SURVEY.width, H = S.SURVEY.height, K = intrinsics(lens, W, H);
  // The silhouette's box: the sphere's surface points projected (the outline under perspective is wider than r·f/z).
  const toCam = (cam, P) => { const rel = [0, 1, 2].map((k) => P[k] - cam.p[k]); return [0, 1, 2].map((k) => cam.R[0][k] * rel[0] + cam.R[1][k] * rel[1] + cam.R[2][k] * rel[2]); };
  const box = (s) => {
    let best = null;
    views.forEach((pose, i) => {
      const cam = droneCamera(pose, 0), v = toCam(cam, s.c), px = project(K, v);
      if (!px || v[2] < 0.3) return;
      const pts = [];
      for (let a = 0; a < 24; a++) for (let e = -6; e <= 6; e++) {
        const [th, ph] = [(a * Math.PI) / 12, (e * Math.PI) / 12], q = toCam(cam, [s.c[0] + s.r * Math.cos(ph) * Math.cos(th), s.c[1] + s.r * Math.cos(ph) * Math.sin(th), s.c[2] + s.r * Math.sin(ph)]);
        if (q[2] > 0.05) pts.push(project(K, q));
      }
      const b = [Math.min(...pts.map((p) => p[0])), Math.min(...pts.map((p) => p[1])), Math.max(...pts.map((p) => p[0])), Math.max(...pts.map((p) => p[1]))];
      const margin = Math.min(px[0], W - px[0], px[1], H - px[1]);
      if (margin > 10 && (!best || margin > best.margin)) best = { view: i + 1, margin, box: b.map((x, k) => Math.round(Math.max(0, Math.min(x, k % 2 ? H : W)))) };
    });
    return best;
  };
  const mine = THINGS.filter((s) => s.rooms.includes(room.id)).map((s) => ({ s, b: box(s) })).filter((x) => x.b);
  const [kind, name, confidence] = ROOM_ANSWERS[room.id];
  const images = imagesOf(body).length;
  return answer({
    room: { kind, name, confidence },
    landmarks: mine.filter((x) => x.s.name).map(({ s, b }) => ({ name: s.name, view: b.view, box: b.box, confidence: s.confidence })),
    hazards: mine.filter((x) => x.s.kind).map(({ s, b }) => ({ kind: s.kind, view: b.view, box: b.box, confidence: s.confidence, why: s.why })),
  }, { input_tokens: images * 590 + 380, cache_creation_input_tokens: n === 1 ? 560 : 0, cache_read_input_tokens: n === 1 ? 0 : 560, output_tokens: 1100 });
}

function settingsWith(values = {}) {
  const s = { values: { apiKey: "test-key", model: "claude-opus-5", effort: "low", aiVision: "on", aiBudget: 0.5, ...values } };
  s.get = (k) => s.values[k];
  s.set = (k, v) => (s.values[k] = v);
  s.all = () => s.values;
  return s;
}
function claudeWith(values, agent = null) {
  const settings = settingsWith(values);
  return { settings, claude: new Claude({ settings, agent, fetch: fakeFetch }) };
}
const survey = (claude, o = {}) => S.surveyHouse({ house, map, twin: new FakeTwin(), settings: claude.settings, claude, encode, ...o });

const tests = {
  async "survey: one request per room with its 4-8 level views (and ones down at the floor, up at the ceiling) as JPEGs, the room's plan, a JSON-schema answer and one cached system prompt"() {
    bodies.length = 0;
    respond = surveyAnswer;
    const { claude } = claudeWith();
    const twin = new FakeTwin(), res = await S.surveyHouse({ house, map, twin, settings: claude.settings, claude, encode });
    assert.ok(res.ok, res.stopped);
    assert.equal(bodies.length, house.rooms.length, "one call per room");
    bodies.forEach((b, i) => {
      const room = house.rooms[i], views = S.surveyViews(map, room.id), imgs = imagesOf(b), level = views.filter((v) => v.look === "level").length;
      assert.ok(level >= 4 && level <= 8 && views.length - level <= S.SURVEY.maxExtra, `${room.name}: ${level} level views and ${views.length - level} more`);
      assert.ok(res.rooms[i].more <= S.SURVEY.maxMore);
      assert.equal(imgs.length, views.length + res.rooms[i].more, "the planned views, and any from other spots the measured floor or ceiling asked for");
      assert.ok(imgs.every((x) => x.source.type === "base64" && x.source.media_type === "image/jpeg" && !("width" in x)), "plain image blocks");
      assert.deepEqual(b.output_config.format, { type: "json_schema", schema: S.SURVEY_SCHEMA }, "structured output");
      assert.equal(b.output_config.effort, "low");
      assert.deepEqual(b.thinking, { type: "adaptive" });
      assert.equal(b.tool_choice, undefined, "no forced tool (refused by Opus 5.5, Sonnet 5.5, Fable 5.1)");
      assert.equal(b.tools, undefined);
      assert.deepEqual(b.system[0].cache_control, { type: "ephemeral" });
      assert.deepEqual(b.system, bodies[0].system, "the same system prompt for every room: cached");
      assert.match(textOf(b), new RegExp(`^This is ${room.name} \\(\\d+ m², floor to ceiling about \\d\\.\\d m\\)\\. Doorways to: `));
      assert.match(textOf(b), /View 1 of \d+: facing /);
      if (views.some((v) => v.look === "down")) assert.match(textOf(b), /: facing .*, looking down at the floor\./);
      assert.equal(b.model, "claude-opus-5");
    });
    assert.match(bodies[0].system[0].text, /The home: Living room \(\d+ m², ceiling about 3\.0 m\); Room 2/);
    const renders = twin.calls.filter((c) => c.op === "pixels");
    assert.ok(renders.every((c) => c.view.actors === false && c.view.props === false && c.view.width === 768 && c.view.height === 576 && c.view.lens.model === "pinhole" && c.view.lens.hfovDeg === 90),
      "768x576 pinhole 90°, no actors or props");
    const pitch = { level: 0, down: S.SURVEY.down, up: -S.SURVEY.up };
    assert.ok(renders.every((c) => Math.abs(c.pose.pitch - pitch[c.pose.look] * (Math.PI / 180)) < 1e-3 && c.pose.z - map.floorAt(c.pose.x, c.pose.y) <= 1.31), "level, or pitched down or up, about 1.3 m up");
    assert.ok(renders.some((c) => c.pose.look === "down") && renders.some((c) => c.pose.look === "up"));
    assert.deepEqual(res.rooms.map((r) => [r.id, r.kind, r.suggestedName]), [["r1", "living room", "Lounge"], ["r2", "entrance", "Entrance hall"], ["r3", "hallway", "Stair hall"]]);
    assert.equal(res.views.length, bodies.reduce((a, b) => a + imagesOf(b).length, 0), "the views come back for the review");
  },

  async "survey: boxes land in 3D on the things (twin depth), duplicates across rooms merge, outside the rooms is dropped, known ones are marked"() {
    bodies.length = 0;
    respond = surveyAnswer;
    const { claude } = claudeWith(), res = await survey(claude);
    const near = (it, s) => Math.hypot(it.x - s.c[0], it.y - s.c[1], it.z - s.c[2]);
    const sofa = res.landmarks.find((l) => l.name === "sofa"), tree = res.landmarks.find((l) => l.name === "cat tree");
    if (process.env.DEBUG) for (const it of [...res.landmarks, ...res.hazards]) {
      const s = THINGS.find((t) => (t.name ?? t.kind) === (it.name ?? it.kind));
      console.log(`      ${it.name ?? it.kind}: ${near(it, s).toFixed(3)} m from the centre (r ${s.r}), r ${it.r}, z ${it.zMin}..${it.zMax} (true ${(s.c[2] - s.r).toFixed(2)}..${(s.c[2] + s.r).toFixed(2)}), views ${it.views.length}`);
    }
    assert.ok(near(sofa, THINGS[0]) <= THINGS[0].r + 0.05, `sofa ${near(sofa, THINGS[0]).toFixed(3)} m from its centre (radius 0.4)`);
    assert.equal(sofa.room, "r1");
    assert.ok(Math.abs(sofa.r - 0.4) < 0.1, `sofa radius ${sofa.r}`);
    assert.ok(near(tree, THINGS[2]) <= THINGS[2].r + 0.05, "cat tree placed");
    assert.equal(tree.views.length, 2, "seen from the Living room and from Room 3: one landmark");
    assert.deepEqual(tree.views.map((v) => v.room).sort(), ["r1", "r3"]);
    assert.equal(tree.confidence, 0.84, "1 - (1 - 0.6)²");
    assert.equal(tree.room, "r3");
    assert.ok(!res.landmarks.some((l) => l.name === "tree"), "outside every room: dropped");
    assert.equal(res.unplaced, 1);
    const fan = res.hazards.find((h) => h.kind === "ceiling-fan"), plant = res.hazards.find((h) => h.kind === "plant");
    assert.equal(fan.known, "fan", "the map already has this fan");
    assert.ok(near(plant, THINGS[3]) <= 0.3);
    assert.equal(plant.room, "r2");
    assert.ok(plant.zMin < 0.95 && plant.zMax > 1.25 && plant.zMax - plant.zMin <= 0.5, `plant spans ${plant.zMin}..${plant.zMax} (the sphere 0.85..1.35, seen from above)`);
    assert.equal(plant.why, "leafy plant on a stand");
    const sofaRoomplan = house.landmarks.find((l) => l.name === "sofa");
    assert.ok(!sofa.known || Math.hypot(sofaRoomplan.x - sofa.x, sofaRoomplan.y - sofa.y) < 1, "known only when the scan's sofa is within 1 m");
    assert.match(S.surveySummary(res, house), /^Surveyed 3 rooms for \$0\.\d{3} \(estimated \$0\.\d{3}\)\. Rooms: Living room: living room, "Lounge" \(90%\); .* Hazards: ceiling fan in the Living room \(already on the map\), plant in Room 2\. .*Settings → House\.$/);
    // Applying what the user accepts.
    const { house: h2, map: m2 } = await fixture(), before = h2.keepouts.length;
    const done = S.applySurvey(h2, res, { rooms: ["r2"], landmarks: [res.landmarks.indexOf(tree)], hazards: [res.hazards.indexOf(plant)] }, { map: m2 });
    assert.deepEqual(done, { rooms: 1, landmarks: 1, keepouts: 1 });
    assert.equal(h2.rooms.find((r) => r.id === "r2").name, "Entrance hall");
    assert.equal(h2.rooms.find((r) => r.id === "r2").nameSource, "claude");
    assert.equal(h2.landmarks.at(-1).source, "claude");
    const ko = h2.keepouts.at(-1);
    assert.equal(h2.keepouts.length, before + 1);
    assert.deepEqual([ko.kind, ko.source, ko.zMin < plant.zMin, ko.zMax > plant.zMax], ["plant", "claude", true, true]);
    m2.finalize();
    assert.ok(!m2.free(plant.x, plant.y, plant.z), "the planner avoids it");
    h2.rooms[0].nameSource = "user";
    S.applySurvey(h2, res, { rooms: ["r1"] });
    assert.notEqual(h2.rooms[0].name, "Lounge", "a name the user gave stays");
  },

  async "survey again after accepting it: what was accepted is known (hyphenated kinds too), and accepting it again replaces it, never doubles it"() {
    bodies.length = 0;
    respond = (body, n) => {
      const a = surveyAnswer(body, n), d = JSON.parse(a.content[1].text);
      for (const h of d.hazards) if (h.kind === "plant") h.kind = "hanging-lamp";
      return { ...a, content: [a.content[0], { type: "text", text: JSON.stringify(d) }] };
    };
    const { claude } = claudeWith(), { house: h2, map: m2 } = await fixture(), all = (list) => list.map((_, i) => i);
    const run = () => S.surveyHouse({ house: h2, map: m2, twin: new FakeTwin(), settings: claude.settings, claude, encode });
    const first = await run(), lamp = first.hazards.find((h) => h.kind === "hanging-lamp"), tree = first.landmarks.find((l) => l.name === "cat tree");
    assert.ok(lamp && !lamp.known && tree && !tree.known, "new the first time");
    S.applySurvey(h2, first, { hazards: all(first.hazards), landmarks: all(first.landmarks) }, { map: m2 });
    const n = [h2.keepouts.length, h2.landmarks.length], ko = h2.keepouts.find((k) => k.source === "claude" && k.kind === "hanging lamp");
    assert.ok(ko, "stored as a \"hanging lamp\" keep-out");
    ko.x += 0.6; // the next survey's estimate lands 0.6 m off (a box drawn differently)
    h2.landmarks.find((l) => l.source === "claude" && l.name === "cat tree").y += 0.6;
    m2.finalize();
    const again = await run();
    assert.deepEqual(again.hazards.filter((h) => !h.known).map((h) => h.kind), [], "every hazard accepted before is known");
    assert.deepEqual(again.landmarks.filter((l) => !l.known).map((l) => l.name), [], "every landmark too");
    assert.equal(again.hazards.find((h) => h.kind === "hanging-lamp").known, "hanging lamp");
    S.applySurvey(h2, again, { hazards: all(again.hazards), landmarks: all(again.landmarks) }, { map: m2 });
    assert.deepEqual([h2.keepouts.length, h2.landmarks.length], n, "accepted again: replaced, not doubled");
    assert.ok(!h2.keepouts.includes(ko), "the old estimate is gone");
  },

  async "lifting: a point projected and lifted back is the same point; box sizes; boxes given as fractions"() {
    const pose = { x: 1.2, y: -0.4, z: 1.1, yaw: 0.7, pitch: 0.1, roll: -0.05 };
    for (const lens of [pinholeLens(90), pinholeLens(90, { uptiltDeg: 20 })]) {
      const W = 768, H = 576, K = intrinsics(lens, W, H), cam = droneCamera(pose, lens.uptiltDeg), P = [2.9, 1.1, 0.6];
      const rel = P.map((v, k) => v - cam.p[k]), v = [0, 1, 2].map((k) => cam.R[0][k] * rel[0] + cam.R[1][k] * rel[1] + cam.R[2][k] * rel[2]);
      const [u, w] = project(K, v), d = Math.hypot(...rel);
      const depth = new Float32Array(W * H).fill(d);
      const p = S.liftBox({ pose, lens, width: W, height: H, depth, dw: W, dh: H }, [u - 2, w - 2, u + 2, w + 2]);
      assert.ok(Math.hypot(p.x - P[0], p.y - P[1], p.z - P[2]) < 0.01, `lifted within 1 cm (uptilt ${lens.uptiltDeg})`);
      const wide = S.liftBox({ pose, lens, width: W, height: H, depth, dw: W, dh: H }, [u - (0.25 * K.fx) / v[2], w - 10, u + (0.25 * K.fx) / v[2], w + 10]);
      assert.ok(Math.abs(wide.width - 0.5) < 0.02, `a 0.5 m wide box: ${wide.width.toFixed(3)} m`);
    }
    const lens = pinholeLens(90), depth = new Float32Array(4 * 3).fill(2);
    const a = S.liftBox({ pose, lens, width: 768, height: 576, depth, dw: 4, dh: 3 }, [0.4, 0.4, 0.6, 0.6]), b = S.liftBox({ pose, lens, width: 768, height: 576, depth, dw: 4, dh: 3 }, [307.2, 230.4, 460.8, 345.6]);
    assert.ok(Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z) < 1e-9, "fractions are read as fractions; the depth may be smaller than the picture");
    assert.equal(S.liftBox({ pose, lens, width: 768, height: 576, depth: new Float32Array(12).fill(NaN), dw: 4, dh: 3 }, [10, 10, 100, 100]), null, "no depth: no place");
    // Something nearer across a third of the box (a chair back in front of the table): the thing is what fills most of it.
    const view = (fill) => ({ pose, lens, width: 768, height: 576, depth: fill, dw: 768, dh: 576 }), img = (f) => { const d = new Float32Array(768 * 576); for (let v = 0; v < 576; v++) for (let u = 0; u < 768; u++) d[v * 768 + u] = f(u, v); return d; };
    const inBox = (u, v) => u >= 250 && u < 550 && v >= 200 && v < 400;
    const occ = img((u, v) => (u >= 300 && u < 380 ? 1.2 : inBox(u, v) ? 3.0 : 5.0));
    const t = S.liftBox(view(occ), [250, 200, 550, 400]);
    assert.ok(t.d >= 3.0 && t.d <= 3.41, `the largest cluster (3 m), not the nearer thing across it, moved back toward its middle: ${t.d.toFixed(2)}`);
    // A chair in front of a wall: the box shows mostly the wall (which goes on outside the box); the chair is the nearer part.
    const chair = img((u, v) => (inBox(u, v) && ((u - 400) ** 2 / 90 ** 2 + (v - 300) ** 2 / 70 ** 2 < 1) ? 2.0 : 4.0));
    const c = S.liftBox(view(chair), [250, 200, 550, 400]);
    assert.ok(c.d >= 2.0 && c.d <= 2.41, `the chair (2 m), not the wall behind it (4 m): ${c.d.toFixed(2)}`);
    // A cabinet against the wall (as deep as the wall around it) behind a door frame that goes on above and below the box:
    // the cabinet, not the frame.
    const cab = S.liftBox(view(img((u, v) => (u >= 300 && u < 380 ? 2.5 : inBox(u, v) ? 4.6 : 4.9))), [250, 200, 550, 400]);
    assert.ok(cab.d >= 4.6 && cab.d <= 4.9, `the cabinet (4.6 m), not the door frame in front: ${cab.d.toFixed(2)}`);
    // A fan under the ceiling: thin, a third of the box, the ceiling all around it: the fan.
    const fan = S.liftBox(view(img((u, v) => ((u - 400) ** 2 / 70 ** 2 + (v - 300) ** 2 / 25 ** 2 < 1 ? 2.3 : 3.1))), [250, 200, 550, 400]);
    assert.ok(fan.d >= 2.3 && fan.d <= 2.71, `the fan (2.3 m, moved back toward its middle), not the ceiling (3.1 m): ${fan.d.toFixed(2)}`);
    // A sofa 5-6 m away seen at a slant (a metre deep across its box) through a doorway whose frame crosses a third of it.
    const sofa = S.liftBox(view(img((u, v) => (u >= 300 && u < 375 ? 1.5 : inBox(u, v) ? 4.6 + (2 * (u - 250)) / 300 : 7.0))), [250, 200, 550, 400]);
    assert.ok(sofa.d > 4.5, `the sofa is one thing, not split behind the frame: ${sofa.d.toFixed(2)}`);
    const flat = S.liftBox(view(img(() => 3.0)), [250, 200, 550, 400]);
    assert.ok(Math.abs(flat.d - 3.0) < 0.01, "flat on the wall: not moved back");
  },

  async "survey: a view with a wall in its face is swapped for the room's next best one"() {
    bodies.length = 0;
    respond = surveyAnswer;
    const { claude } = claudeWith(), twin = new FakeTwin(), depth = twin.depth.bind(twin);
    twin.depth = async (pose, view) => (pose.room === "r2" && Math.abs(pose.yaw) < 0.01 ? new Float32Array(view.width * view.height).fill(0.5) : depth(pose, view));
    const res = await S.surveyHouse({ house, map, twin, settings: claude.settings, claude, encode, rooms: ["r2"] });
    const shown = twin.calls.filter((c) => c.op === "pixels").map((c) => c.pose);
    assert.ok(shown.every((p) => Math.abs(p.yaw) > 0.01), "the wall view isn't sent");
    assert.equal(shown.length, S.surveyViews(map, "r2").length, "a spare view takes its place");
    assert.equal(imagesOf(bodies[0]).length, shown.length);
    assert.ok(res.ok);
    const narrow = new FakeTwin();
    narrow.depth = async (pose, view) => new Float32Array(view.width * view.height).fill(0.4 + 0.1 * Math.abs(pose.yaw));
    await S.surveyHouse({ house, map, twin: narrow, settings: claude.settings, claude, encode, rooms: ["r2"] });
    assert.equal(narrow.calls.filter((c) => c.op === "pixels" && c.pose.look === "level").length, 4, "nothing open anywhere: the 4 most open level views still go");
  },

  async "survey: the floor and ceiling a level camera misses get views pitched down and up; what no view saw is measured from their depth and said"() {
    for (const r of house.rooms) {
      const level = S.surveyViews(map, r.id, { maxExtra: 0 })[0].planned, all = S.surveyViews(map, r.id)[0].planned;
      assert.ok(all.floor >= Math.min(S.SURVEY.floorCoverage, level.floor + 0.2) && all.ceiling >= Math.min(S.SURVEY.ceilingCoverage, level.ceiling), `${r.name}: floor ${level.floor} -> ${all.floor}, ceiling ${level.ceiling} -> ${all.ceiling}`);
    }
    assert.ok(S.surveyViews(map, "r2", { maxExtra: 0 })[0].planned.floor < 0.1, "Room 2's floor is all within 1.7 m of its one spot: a level camera sees none of it");
    const est = S.estimateSurvey({ house, map });
    assert.equal(est.total.images, house.rooms.reduce((a, r) => a + S.surveyViews(map, r.id).length, 0), "the estimate counts the pitched pictures");
    bodies.length = 0;
    respond = surveyAnswer;
    const { claude } = claudeWith(), res = await survey(claude);
    for (const r of res.rooms) assert.ok(r.unseen.floorShare < 0.3 && r.seen.floor === Math.round(100 * (1 - r.unseen.floorShare)) / 100, `${r.name}: ${Math.round(r.unseen.floorShare * 100)}% of the floor unseen (${r.unseen.text || "nothing to say"})`);
    assert.ok(res.views.every((v) => !("planned" in v.pose) && !("coverage" in v.pose)), "no planned shares on the returned poses: rooms[].seen is what the depth showed");
    const blindTwin = new FakeTwin({ floors: false }), blind = await S.surveyHouse({ house, map, twin: blindTwin, settings: claude.settings, claude, encode, rooms: ["r2"] });
    assert.ok(blind.rooms[0].unseen.floorShare > 0.9, "a scan that shows no floor: the floor is unseen");
    const tried = blindTwin.calls.filter((c) => c.op === "depth" && c.pose.more).length;
    assert.ok(tried >= 1 && tried <= S.SURVEY.moreTries && blind.rooms[0].more === 0, `other spots tried (${tried}), none kept: their depth showed no more floor`);
    assert.match(S.surveySummary(blind, house), /Not seen in the pictures \(hazards there may be missing\): Room 2: (about [\d.]+ m² of (ceiling|floor) [a-z ]+, )*about [\d.]+ m² of floor (near the|below the|below where|along|in the)/);
  },

  async "survey: floor the planned spot can't see (furniture in the way) gets views from other spots, kept only when their depth shows more of it"() {
    const spot = S.surveyViews(map, "r1")[0], far = (P) => P[0] > spot.x + 0.6; // from the planned spot, everything past this line is hidden
    const twin = new FakeTwin({ override: (pose, view, out) => {
      if (pose.x !== spot.x || pose.y !== spot.y) return;
      const K = intrinsics(view.lens, view.width, view.height), cam = droneCamera(pose, view.lens.uptiltDeg ?? 0);
      for (let v = 0; v < view.height; v++)
        for (let u = 0; u < view.width; u++) {
          const d = camDir(cam, unproject(K, u + 0.5, v + 0.5)), t = out[v * view.width + u];
          if (d[2] < 0 && far([cam.p[0] + t * d[0], cam.p[1] + t * d[1]])) out[v * view.width + u] = NaN;
        }
    } });
    bodies.length = 0;
    respond = surveyAnswer;
    const { claude } = claudeWith(), progress = [], res = await S.surveyHouse({ house, map, twin, settings: claude.settings, claude, encode, rooms: ["r1"], onProgress: (p) => progress.push(p.text) });
    const r = res.rooms[0], extra = res.views.filter((v) => v.pose.more);
    const planned = S.surveyViews(map, "r1").length;
    assert.ok(r.more >= 1 && r.more <= S.SURVEY.maxMore && extra.length === r.more, `${r.more} more views`);
    assert.equal(imagesOf(bodies[0]).length, planned + r.more, "they go to Claude with the others");
    assert.ok(extra.every((v) => v.pose.look === "down" && Math.hypot(v.pose.x - spot.x, v.pose.y - spot.y) >= S.SURVEY.spotStep - 1e-9 && map.roomAt(v.pose.x, v.pose.y)?.id === "r1"), "pitched down, from other spots in the room");
    assert.ok(extra.every((v) => v.pose.x > spot.x), "from the side that sees what was hidden");
    const tried = twin.calls.filter((c) => c.op === "depth" && c.pose.more).length;
    assert.ok(tried <= S.SURVEY.moreTries, `${tried} tried`);
    assert.ok(progress.some((t) => /^\d more views? of the Living room from other spots, for floor or ceiling the first ones missed: \d+% of the floor/.test(t)), progress.join(" | "));
    const before = S.SURVEY.maxMore;
    S.SURVEY.maxMore = 0;
    try {
      const none = await S.surveyHouse({ house, map, twin, settings: claude.settings, claude, encode, rooms: ["r1"] });
      assert.ok(none.rooms[0].seen.floor < r.seen.floor - 0.1, `without them: ${none.rooms[0].seen.floor} of the floor seen, with them ${r.seen.floor}`);
      if (process.env.DEBUG) console.log(`      floor seen ${none.rooms[0].seen.floor} -> ${r.seen.floor} with ${r.more} more views (${tried} tried)`);
    } finally {
      S.SURVEY.maxMore = before;
    }
  },

  async "survey: a mirror (the scan shows a room behind it) is placed on its wall, not behind it, and keeps out as a panel along the wall"() {
    const v1 = S.surveyViews(map, "r1")[0], dir = [Math.cos(v1.yaw), Math.sin(v1.yaw)];
    let wall = 0.3;
    while (!map.wall[map.idx(v1.x + wall * dir[0], v1.y + wall * dir[1])] && wall < 12) wall += 0.025;
    const box = [334, 238, 434, 338];
    const twin = new FakeTwin({ override: (pose, view, out) => {
      if (pose.look !== "level" || pose.x !== v1.x || pose.y !== v1.y || pose.yaw !== v1.yaw) return;
      const k = view.width / S.SURVEY.width, K = intrinsics(view.lens, view.width, view.height), [u0, v0, u1, w1] = box.map((x) => Math.round(x * k)), e = Math.round(0.12 * (u1 - u0));
      for (let v = v0; v < w1; v++)
        for (let u = u0; u < u1; u++) out[v * view.width + u] = (wall - 0.02) / unproject(K, u + 0.5, v + 0.5)[2] + (u < u0 + e || u >= u1 - e || v < v0 + e || v >= w1 - e ? 0 : 2.5); // frame on the wall, reflection behind
    } });
    bodies.length = 0;
    respond = (body, n) => (textOf(body).startsWith("This is Living room (") ? answer({ room: { kind: "living room", name: "Lounge", confidence: 0.9 }, landmarks: [], hazards: [{ kind: "mirror", view: 1, box, confidence: 0.8, why: "a tall mirror" }] }, { input_tokens: 5000, output_tokens: 900 }) : surveyAnswer(body, n));
    const { claude } = claudeWith(), res = await S.surveyHouse({ house, map, twin, settings: claude.settings, claude, encode, rooms: ["r1"] });
    const mir = res.hazards.find((h) => h.kind === "mirror"), W = [v1.x + wall * dir[0], v1.y + wall * dir[1]];
    assert.ok(mir, `placed (${res.unplaced} unplaced)`);
    assert.ok(Math.hypot(mir.x - W[0], mir.y - W[1]) < 0.3, `on its wall: ${Math.hypot(mir.x - W[0], mir.y - W[1]).toFixed(2)} m from where the view meets it`);
    const L = Math.hypot(mir.line[1][0] - mir.line[0][0], mir.line[1][1] - mir.line[0][1]);
    assert.ok(Math.abs(L - (100 / 384) * wall) < 0.1, `the panel is as wide as the mirror: ${L.toFixed(2)} m`);
    const ko = S.hazardKeepout(mir), { house: h2, map: m2 } = await fixture();
    assert.equal(ko.polygon.length, 4);
    S.applySurvey(h2, res, { hazards: [res.hazards.indexOf(mir)] });
    m2.finalize();
    assert.ok(!m2.free(mir.x, mir.y, 1.0), "the planner keeps off it");
  },

  async "dedupe: the same name close by merges; far apart or another name stays apart"() {
    const it = (name, x, c, room = "r1") => ({ name, x, y: 1, z: 0.5, r: 0.2, zMin: 0.3, zMax: 0.7, confidence: c, views: [{ room }] });
    const out = S.dedupe([it("sofa", 1, 0.5), it("Sofa", 1.3, 0.5, "r3"), it("sofa", 4, 0.9), it("table", 1.1, 0.8)], (o) => o.name.toLowerCase());
    assert.equal(out.length, 3);
    const merged = out.find((o) => o.views.length === 2);
    assert.equal(merged.confidence, 0.75);
    assert.ok(Math.abs(merged.x - 1.15) < 1e-9, "confidence-weighted position");
  },

  async "cost: the estimate (590 tokens a picture, per room and in all) is close to what is reported after, through the agent's usage event and session"() {
    bodies.length = 0;
    respond = surveyAnswer;
    const settings = settingsWith(), agent = new Agent({ tools: { defs: () => [] }, settings, ctl: {} }), usage = [];
    agent.on("usage", (u) => usage.push(u));
    const claude = new Claude({ settings, agent, fetch: fakeFetch });
    const est = S.estimateSurvey({ house, map, settings });
    assert.equal(est.perImage, 590, "768x576 = 442368 px / 750");
    assert.equal(imageTokens(768, 576, "claude-haiku-4-5"), 590);
    assert.equal(imageTokens(4000, 3000, "claude-haiku-4-5"), 1600, "older models: capped");
    assert.equal(imageTokens(4000, 3000, "claude-opus-5"), 4784, "high-res models: capped higher");
    assert.deepEqual(est.rooms.map((r) => r.views), house.rooms.map((r) => S.surveyViews(map, r.id).length));
    assert.match(est.text, /^3 rooms, \d+ pictures of the 3D scan \(768x576, about 590 tokens each\): about [\d.]+K tokens in and 4\.5K out, about \$0\.\d\d with claude-opus-5 \(at most \d+ pictures and \$0\.\d\d if the floor or ceiling needs pictures from more spots\)\.$/);
    assert.equal(est.total.maxImages, est.total.images + 3 * S.SURVEY.maxMore);
    const res = await survey(claude, { estimate: est });
    const want = bodies.length && usage.reduce((a, u) => a + u.cost, 0);
    assert.equal(usage.length, 3);
    assert.ok(usage.every((u) => u.purpose === "survey" && u.requests === 1));
    assert.ok(Math.abs(res.cost.cost - want) < 1e-4, `reported ${res.cost.cost} = Σ usage ${want}`);
    assert.equal(res.cost.input, bodies.reduce((a, b) => a + imagesOf(b).length * 590 + 380, 0));
    assert.ok(Math.abs(agent.session.cost - want) < 1e-9 && agent.session.commands === 0, "in the session's cost, not its command count");
    const off = Math.abs(est.total.cost - res.cost.cost) / res.cost.cost;
    assert.ok(off < 0.3, `estimate $${est.total.cost.toFixed(3)} vs $${res.cost.cost.toFixed(3)} (${Math.round(off * 100)}% off)`);
    assert.ok(res.cost.cost <= est.total.maxCost, "within the most the estimate said");
    assert.equal(res.cost.estimate, Math.round(est.total.cost * 1e4) / 1e4);
  },

  async "budget: a survey stops at its limit, a flight's checks at aiBudget; a new flight starts afresh"() {
    bodies.length = 0;
    respond = surveyAnswer;
    const { claude } = claudeWith(), est = S.estimateSurvey({ house, map, settings: claude.settings });
    const res = await survey(claude, { limit: est.rooms[0].cost + 0.3 * Math.min(est.rooms[1].cost, est.rooms[2].cost) });
    assert.equal(bodies.length, 1, "the second room would go over");
    assert.match(res.stopped, /budget for this job is used up/);
    assert.ok(res.ok && res.rooms[1].error);
    bodies.length = 0;
    respond = () => answer({ verdict: "real-change", kind: "obstacle", what: "box", confidence: 0.9, why: "a box" }, { input_tokens: 1300, output_tokens: 500 });
    const c2 = claudeWith({ aiBudget: 0.05 }).claude, ins = new Inspector({ claude: c2 });
    const ch = { kind: "obstacle", x: 1, y: 2, size: 0.4, evidence: { live: jpeg(), expected: jpeg() } };
    const r = [await ins.confirmChange(ch), await ins.confirmChange(ch), await ins.confirmChange(ch)];
    assert.deepEqual(r.map((x) => x.status), ["answered", "answered", "unconfirmed"]);
    assert.match(r[2].reason, /AI budget for this flight is used up \(\$0\.04 of \$0\.05/);
    assert.equal(bodies.length, 2);
    c2.newFlight("f2");
    assert.equal((await ins.confirmChange(ch)).status, "answered", "a new flight, a new budget");
  },

  async "consent: off never sends a picture; ask asks once (yes is remembered, not now isn't asked again, once is once); the survey's OK counts"() {
    bodies.length = 0;
    respond = () => answer({ present: true, actual: "a person", confidence: 0.9, why: "standing" }, { input_tokens: 700, output_tokens: 100 });
    const off = claudeWith({ aiVision: "off" }).claude, ins = new Inspector({ claude: off });
    const s = await survey(off);
    assert.ok(!s.ok && /off in Settings/.test(s.stopped));
    assert.equal((await ins.confirmDetection(jpeg(), "person", 0.4)).status, "unconfirmed");
    assert.equal((await ins.confirmChange({ kind: "obstacle", x: 0, y: 0, evidence: { live: jpeg(), expected: jpeg() } })).status, "unconfirmed");
    assert.equal(bodies.length, 0, "nothing sent");
    const { claude: ask, settings } = claudeWith({ aiVision: "ask" }), asked = [];
    const insA = new Inspector({ claude: ask });
    assert.match((await insA.confirmDetection(jpeg(), "person", 0.4)).reason, /need your OK first/, "no way to ask: no");
    ask.setConsent(async (q) => (asked.push(q.purpose), false));
    await insA.confirmDetection(jpeg(), "person", 0.4);
    await insA.confirmDetection(jpeg(), "person", 0.4);
    assert.deepEqual(asked, ["detection"], "not now: not asked again this session");
    assert.equal(bodies.length, 0);
    ask.setConsent(async (q) => (asked.push(q.purpose), "once"));
    assert.equal((await insA.confirmDetection(jpeg(), "person", 0.4)).status, "confirmed");
    assert.equal(settings.get("aiVision"), "ask", "once isn't remembered");
    ask.setConsent(async (q) => (asked.push(q.purpose), true));
    await insA.confirmDetection(jpeg(), "person", 0.4);
    await insA.confirmDetection(jpeg(), "person", 0.4);
    assert.equal(settings.get("aiVision"), "on", "yes is remembered");
    assert.equal(asked.length, 3);
    assert.equal(bodies.length, 3);
    assert.ok(bodies.every((b) => imagesOf(b).length === 1));
    bodies.length = 0;
    respond = surveyAnswer;
    const fresh = claudeWith({ aiVision: "ask" });
    const ok = await survey(fresh.claude, { approved: true });
    assert.ok(ok.ok && bodies.length === 3, "the survey dialog's OK is consent for that survey");
    assert.equal(fresh.settings.get("aiVision"), "ask");
    bodies.length = 0;
    const nokey = claudeWith({ apiKey: "" }).claude;
    assert.match((await survey(nokey)).stopped, /no Claude API key/);
    assert.equal((await new Inspector({ claude: nokey }).confirmDetection(jpeg(), "cat", 0.3)).status, "unconfirmed");
    assert.equal(bodies.length, 0);
  },

  async "changes: Claude compares and suggests; on its own it only confirms a new blocker it names, on pictures big enough; a person or pet keeps blocking a while; freeing and waving away wait for the user"() {
    const { house: h, map: m } = await fixture();
    const memory = await HouseMemory.open(h.id, { indexedDB: null });
    memory.setHouse({ house: h, map: m });
    const { claude } = claudeWith(), ins = new Inspector({ claude, memory }), usage = [];
    claude.on("usage", (u) => usage.push(u.purpose));
    const big = { live: jpegOf(640, 480), expected: jpegOf(640, 480, 6) };
    const add = ([x, y], kind = "obstacle", evidence = big) => memory.addChange({ kind, x, y, z: 0.4, size: 0.4, evidence });
    const say = (verdict, confidence, kind = "obstacle", what = "cardboard box") => (respond = () => answer({ verdict, kind, what, confidence, why: "seen in both" }, { input_tokens: 1300, output_tokens: 200 }));
    const check = async (c) => [await ins.confirmChange(c, { apply: true }), c];
    bodies.length = 0;
    say("real-change", 0.9);
    const [ra, a] = await check(add([0.9, 2.4]));
    assert.deepEqual([ra.status, ra.suggest, ra.small, a.status, a.by], ["answered", "confirmed", false, "confirmed", "claude"]);
    assert.ok(h.keepouts.some((k) => k.change === a.id), "applied to the house");
    const b0 = bodies[0], imgs = imagesOf(b0);
    assert.deepEqual(imgs.map((x) => x.source.data), [big.live, big.expected], "live, then expected");
    assert.equal(b0.output_config.format.type, "json_schema");
    assert.match(textOf(b0), /^Live camera:\nWhat the 3D scan expected from the same spot:\nThe change detector says: obstacle in the Living room, about 40 cm across, 0\.4 m above the floor\. Is it real\?$/);
    assert.match(b0.system[0].text, /slightly misaligned with each other, differently exposed or coloured, blurred or compressed, and the live one may carry on-screen text/);
    const [rt, tiny] = await check(add([1.5, 2.4], "obstacle", { live: jpegOf(38, 30), expected: jpegOf(38, 30) }));
    assert.deepEqual([rt.suggest, rt.small, tiny.status], ["confirmed", true, "suspected"], "a 38 px crop: a suggestion only");
    const [, nosize] = await check(add([2.1, 2.4], "obstacle", { live: jpeg(), expected: jpeg(6) }));
    assert.equal(nosize.status, "suspected", "pictures of unknown size: a suggestion only");
    say("real-change", 0.9, "door-closed");
    const [, other] = await check(add([2.7, 2.4]));
    assert.equal(other.status, "suspected", "Claude names another kind of change: the user decides");
    say("real-change", 0.95, "gone", "the armchair");
    const before = h.keepouts.length, [rg, g] = await check(add([3.3, 2.4], "gone"));
    assert.deepEqual([rg.suggest, g.status, h.keepouts.length], ["confirmed", "suspected", before], "nothing is freed on Claude's word");
    say("no-change", 0.95);
    const [rb, b] = await check(add([3.9, 2.4]));
    assert.deepEqual([rb.suggest, b.status], ["dismissed", "suspected"], "a blocker isn't waved away by Claude, however sure");
    say("no-change", 0.95, "none", "");
    const [, g2] = await check(add([3.9, 3.6], "gone"));
    assert.deepEqual([g2.status, g2.by], ["dismissed", "claude"], "a change that would free space, dismissed: the scan's version stays");
    say("person-or-pet", 0.92, "none", "the dog");
    const [rc, c] = await check(add([3.0, 3.6]));
    assert.deepEqual([c.status, rc.passing, m.temps.get(c.id)?.kind], ["suspected", true, "person"], "a person or pet: still an obstacle, for a while");
    assert.ok(!m.free(3.0, 3.6, 1.0));
    say("unclear", 0.4, "none", "");
    const [rd, d] = await check(add([2.1, 3.6]));
    assert.deepEqual([rd.suggest, d.status], [null, "suspected"]);
    assert.equal(usage.length, 9);
    assert.ok(usage.every((u) => u === "change"));
    assert.match(memory.whatChanged(), /Not confirmed yet: .* Confirmed: a new cardboard box in the Living room.*Claude: cardboard box, 90% sure/);
    const none = new Inspector({ claude: claudeWith({ apiKey: "" }).claude, memory }), e = add([1.2, 3.6]);
    const re = await none.confirmChange(e);
    assert.equal(re.status, "unconfirmed");
    assert.equal(e.claude.status, "unconfirmed", "noted on the change");
    assert.equal((await ins.confirmChange({ kind: "obstacle", x: 1, y: 1 })).reason, "no pictures to compare");
  },

  async "a weak suspicion (seen only where the drone's own height error could make one) is dropped when Claude calls it no change at 0.6 or more; a well-evidenced one never is; under 0.6, unclear or a person: as before"() {
    const { house: h, map: m } = await fixture();
    const memory = await HouseMemory.open(`${h.id}:weak`, { indexedDB: null });
    memory.setHouse({ house: h, map: m });
    memory.startFlight({ kind: "patrol" });
    const { claude } = claudeWith(), ins = new Inspector({ claude, memory }), big = { live: jpegOf(640, 480), expected: jpegOf(640, 480, 6) };
    const add = ([x, y], weak = true) => memory.addChange({ kind: "obstacle", x, y, z: 1.1, size: 0.4, weak, evidence: big });
    const say = (verdict, confidence) => (respond = () => answer({ verdict, kind: verdict === "real-change" ? "obstacle" : "none", what: "the sideboard's top", confidence, why: "the scan's smear" }, { input_tokens: 1300, output_tokens: 200 }));
    const check = async (c) => [await ins.confirmChange(c, { apply: true }), c];
    say("no-change", 0.7);
    const [ra, a] = await check(add([0.9, 2.4]));
    assert.deepEqual([ra.suggest, a.status, a.by, m.temps.has(a.id)], ["dismissed", "dismissed", "claude", false], "weak, no change at 0.7: dropped, off the map");
    assert.match(a.note, /^Claude: the sideboard's top \(70% sure\)/);
    const [, b] = await check(add([2.1, 2.4], false));
    assert.deepEqual([b.status, m.temps.has(b.id)], ["suspected", true], "well-evidenced: the same answer leaves it to the user");
    say("no-change", 0.5);
    const [, c] = await check(add([3.3, 2.4]));
    assert.equal(c.status, "suspected", "under 0.6: kept");
    say("unclear", 0.8);
    const [, d] = await check(add([3.9, 2.4]));
    assert.equal(d.status, "suspected", "unclear: kept");
    say("person-or-pet", 0.92);
    const [rp, p] = await check(add([3.0, 3.6]));
    assert.deepEqual([p.status, rp.passing], ["suspected", true], "a person or pet: blocks a while, as any");
    say("real-change", 0.9);
    const [, r] = await check(add([2.1, 3.6]));
    assert.deepEqual([r.status, r.by], ["confirmed", "claude"], "Claude seeing a real thing confirms it, weak or not");
    memory.endFlight("Landed.");
    for (const id of [...m.temps.keys()]) m.removeTemp(id);
  },

  async "detections: an uncertain alert waits for Claude, never for the consent prompt; confirmed or likely says so; a person Claude doubts still goes out, a pet it doubts is only logged"() {
    const memory = await HouseMemory.open(house.id, { indexedDB: null });
    memory.setHouse({ house, map });
    memory.addSighting({ label: "person", x: 2.0, y: 3.0, t: Date.now() - 36e5 }); // seen on an earlier day
    const { claude } = claudeWith(), ins = new Inspector({ claude, memory });
    const pushed = [], said = [];
    const settings = { get: (k) => ({ ntfyTopic: "whoop-t", speak: true })[k] };
    const alerts = new Alerts({ settings, speak: (t) => said.push(t), fetch: async (u, init) => (pushed.push(init), new Response("{}")), Notification: undefined, indexedDB: undefined, memory, inspector: ins });
    const ev = (o) => ({ text: "A person in the Living room.", image: jpeg(), room: "r1", urgency: "high", kind: "finding", label: "person", score: 0.45, x: 2, y: 3, ...o });
    const seen = (present, confidence, actual) => (respond = () => answer({ present, actual, confidence, why: "looked" }, { input_tokens: 800, output_tokens: 80 }));
    bodies.length = 0;
    seen(true, 0.9, "a person standing");
    const a = await alerts.notify(ev());
    assert.deepEqual([a.confirmation, a.pushed, a.spoke], ["confirmed", true, true]);
    assert.equal(said[0], "A person in the Living room.", "spoken as is");
    let log = await alerts.events();
    assert.equal(log[0].text, "A person in the Living room. (confirmed by Claude)");
    assert.equal(log[0].confirmed, true);
    assert.equal(memory.sightings.at(-1).claude, "confirmed");
    assert.match(textOf(bodies[0]), /^The detector says: person \(score 45%\)\. Is there really a person in this picture\?$/);
    seen(true, 0.6, "probably a person");
    const l = await alerts.notify(ev({ x: 3, y: 4.5 }));
    assert.deepEqual([l.confirmation, l.pushed], ["likely", true]);
    assert.equal((await alerts.events())[0].text, "A person in the Living room. (Claude: likely)", "50-80% sure is not \"confirmed\"");
    seen(false, 0.72, "A coat on a chair");
    const b = await alerts.notify(ev({ x: 4, y: 5, box: { x: 0.4, y: 0.2, w: 0.1, h: 0.5 } }));
    assert.deepEqual([b.confirmation, b.pushed, b.spoke], ["rejected", true, true], "one doubtful look never silences an intruder alert");
    log = await alerts.events();
    assert.deepEqual([log[0].text, log[0].urgency], ["A person in the Living room. (Claude thinks it's a coat on a chair)", "default"]);
    assert.equal(pushed.at(-1).headers.Priority, "3", "pushed at default priority");
    assert.match(textOf(bodies.at(-1)), /Its box, as fractions of the picture from the top left: x 0\.40, y 0\.20, width 0\.10, height 0\.50\./, "Node can't crop: the box is described");
    seen(false, 0.9, "a cushion");
    const n0 = pushed.length, cat = await alerts.notify(ev({ label: "cat", urgency: "default", score: 0.5, text: "The cat in the Living room." }));
    assert.deepEqual([cat.confirmation, cat.pushed, cat.logged, pushed.length], ["rejected", false, true, n0], "a pet Claude is sure isn't one: only logged");
    assert.equal((await alerts.events())[0].urgency, "low");
    const n = bodies.length;
    const c = await alerts.notify(ev({ score: 0.9 }));
    assert.equal(bodies.length, n, "a sure detection of a label this house has seen before: no check");
    assert.equal(c.confirmation, undefined);
    seen(true, 0.85, "a dog");
    memory.addSighting({ label: "dog", x: 1, y: 4, score: 0.9 }); // missions records the finding first, then alerts
    await alerts.notify(ev({ label: "dog", score: 0.9, text: "The dog in the Living room." }));
    assert.equal(bodies.length, n + 1, "a label new to this house is checked, though its finding was just recorded");
    const blind = new Alerts({ settings, speak: () => {}, fetch: async () => new Response("{}"), Notification: undefined, indexedDB: undefined, inspector: new Inspector({ claude: claudeWith({ apiKey: "" }).claude }) });
    const d = await blind.notify(ev());
    assert.deepEqual([d.confirmation, d.pushed], ["unconfirmed", true]);
    assert.equal((await blind.events())[0].text, "A person in the Living room. (unconfirmed)");
    // Consent never answered (nobody home): the alert doesn't wait for it, and no prompt is raised for it.
    const asking = claudeWith({ aiVision: "ask" }).claude, prompts = [], needed = [];
    asking.setConsent(() => (prompts.push(1), new Promise(() => {})));
    asking.on("consent-needed", (e) => needed.push(e.purpose));
    const away = new Alerts({ settings, speak: () => {}, fetch: async () => new Response("{}"), Notification: undefined, indexedDB: undefined, inspector: new Inspector({ claude: asking }) });
    const t0 = performance.now(), e = await away.notify(ev());
    assert.deepEqual([e.confirmation, e.pushed, prompts.length, needed], ["unconfirmed", true, 0, ["detection"]]);
    assert.ok(performance.now() - t0 < 1000, "at once");
  },

  async "tools: recall (with the sighting's picture once picture checks are allowed), what_changed, survey_house only on the ground and through the pilot's OK, where_am_i with where it has been and every stop still ahead"() {
    const settings = settingsWith();
    let flying = true;
    const ctl = { tel: {}, autonomy: "full", isFlying: () => flying, blocker: () => null, est: { heading: 0 }, imageHeading: () => 0 };
    const tools = new ToolBox({ ctl, perception: { latest: { detections: [] }, snapshot: () => null }, settings, speak: () => {} });
    const memory = await HouseMemory.open(house.id, { indexedDB: null });
    memory.setHouse({ house, map });
    class Missions extends Emitter {
      busy = true;
      whereAmI = () => "In the Living room, near the sofa, 3.4 m from the home pad.";
      battery = () => ({ vbat: 3.8, secondsLeft: 120 });
    }
    const missions = new Missions(), pose = { x: 1.0, y: 1.5, z: 1, yaw: 0, sigma: 0.1, status: "ok", room: "r1" };
    tools.setHouse({ house, map, missions, localizer: { pose: () => pose }, memory });
    memory.startFlight({ kind: "goTo", mission: { kind: "goTo", target: "Room 3" } });
    for (const [dt, p] of [[20000, { ...pose, x: -0.9, y: 1.0, room: "r2" }], [19000, { ...pose, x: -0.9, y: 1.1, room: "r2" }], [1500, pose], [1000, pose]]) memory.samplePose(p, Date.now() - dt);
    memory.addSighting({ label: "cat", x: 2.0, y: 3.0, score: 0.8, snapshot: jpeg(5) });
    missions.emit("path", { path: [[1.0, 1.5, 1], [0.8, 2.6, 1], [-0.5, 3.5, 1], [-1.2, 3.6, 1]] });
    missions.emit("status", { phase: "fly", text: "Flying to Room 3 (3.6 m)." });
    const where = await tools.call("where_am_i", {}, null, { from: "claude" });
    assert.match(where.text, /^In the Living room, near the sofa, 3\.4 m from the home pad\. This flight: \d+ s so far, [\d.]+ m flown, took off in Room 2, now in the Living room\. Going: Flying to Room 3; 3\.4 m to go \(about 11 s\), into Room 3; about 120 s of battery left\.$/);
    missions.emit("plan", { stops: [{ room: "r3" }, { room: "r2" }, { room: "r1", name: "Living room" }], home: true, seconds: 70 });
    assert.match((await tools.call("where_am_i")).text, /Going: Flying to Room 3; 3\.4 m to go \(about 11 s\), into Room 3, then Room 2 and the Living room, then home; about 70 s in all, about 120 s of battery left\.$/, "every stop still ahead, the way home, the battery");
    assert.match(tools.memoryContext(), /^Memory: This flight: .*now in the Living room\./);
    missions.emit("done", {});
    assert.ok(!/Going/.test((await tools.call("where_am_i")).text), "no mission: nothing ahead");
    const r = await tools.call("recall", { question: "where did you last see the cat?" }, null, { from: "claude" });
    assert.match(r.text, /^I last saw the cat in the Living room/);
    assert.equal(r.image, jpeg(5), "with its picture");
    settings.set("aiVision", "off");
    const off = await tools.call("recall", { question: "where did you last see the cat?" });
    assert.ok(!off.image && /Its picture isn't sent: Claude's picture checks are off in Settings/.test(off.text), "picture checks off: no stored picture leaves either");
    settings.set("aiVision", "ask");
    const needed = [], asking = new Claude({ settings, fetch: fakeFetch });
    asking.on("consent-needed", (e) => needed.push(e.purpose));
    tools.setAI({ claude: asking });
    const ask = await tools.call("recall", { question: "where did you last see the cat?" }, null, { from: "claude" });
    assert.ok(!ask.image && /Its picture isn't sent until you allow Claude's picture checks/.test(ask.text) && needed[0] === "recall", "not before the user allows it: the banner asks");
    tools.setAI(null);
    settings.set("aiVision", "on");
    memory.addChange({ kind: "obstacle", x: 0.9, y: 2.4, size: 0.3 });
    assert.match((await tools.call("what_changed", { since: "today" }, null, { from: "claude" })).text, /^1 change since today\. Not confirmed yet: a new obstacle/);
    assert.match(tools.memoryContext(), /^Memory: This flight: .* 1 possible change from the 3D scan not confirmed yet \(what_changed lists them\)\.$/);
    for (const n of ["recall", "what_changed", "survey_house"]) assert.ok(HOUSE_TOOL_DEFS.some((d) => d.name === n), n);
    // survey_house: on the ground, and the pilot's OK first.
    bodies.length = 0;
    respond = surveyAnswer;
    const claude = new Claude({ settings, fetch: fakeFetch }), seen = [], twins = [], released = [];
    let reviewed = null;
    tools.setAI({ claude, twin: async () => (twins.push(new FakeTwin()), twins.at(-1)), releaseTwin: (t) => released.push(t),
      confirmSurvey: async (est, o) => (seen.push([est, o]), false), reviewSurvey: (res) => (reviewed = res) });
    assert.match((await tools.call("survey_house", {}, null, { from: "claude" })).text, /^Land first/, "not in the air");
    assert.equal(seen.length, 0, "no dialog in the air");
    flying = false;
    const ac = new AbortController(), no = await tools.call("survey_house", {}, ac.signal, { from: "claude" });
    assert.match(no.text, /^The pilot didn't approve the survey \(estimated 3 rooms, \d+ pictures/);
    assert.equal(seen[0][1].signal, ac.signal, "the dialog gets the stop signal");
    assert.deepEqual([bodies.length, twins.length], [0, 0], "declined: nothing sent, and the 3D scan was never loaded for it");
    tools.ai.confirmSurvey = async () => true;
    const yes = await tools.call("survey_house", {}, null, { from: "claude" });
    assert.match(yes.text, /^Surveyed 3 rooms for \$/);
    assert.equal(reviewed.rooms.length, 3, "the result goes to the UI's review");
    assert.deepEqual(released, twins, "the twin is released once the survey is done");
    const stop = new AbortController();
    tools.ai.twin = async () => (stop.abort(), twins.push(new FakeTwin()), twins.at(-1));
    assert.match((await tools.call("survey_house", {}, stop.signal)).text, /^Stopped by the pilot/);
    tools.ai.twin = async () => (twins.push({ pixels: async () => { throw new Error("out of GPU memory"); }, depth: async () => { throw new Error("out of GPU memory"); } }), twins.at(-1));
    assert.match((await tools.call("survey_house")).text, /^The survey failed: out of GPU memory\.$/);
    assert.deepEqual([twins.length, released], [3, twins], "stopped or failed: released too");
    settings.set("aiVision", "off");
    assert.match((await tools.call("survey_house")).text, /picture checks are off in Settings/);
  },

  async "together as main.js wires them: one Claude check per change, no consent prompt in the air but after landing; no key never loops; a banner's once lets the waiting checks go"() {
    const { house: h, map: m } = await fixture();
    const memory = await HouseMemory.open(h.id, { indexedDB: null });
    memory.setHouse({ house: h, map: m });
    let flying = true;
    const { claude, settings } = claudeWith({ aiVision: "ask" }), prompts = [], needed = [];
    claude.setConsent(async (q) => (prompts.push(q.purpose), true), { canAsk: () => !flying });
    claude.on("consent-needed", (e) => needed.push(e.purpose));
    const ins = new Inspector({ claude, memory, watch: true }), checks = [], check = ins.checkChange.bind(ins);
    ins.checkChange = (...a) => (checks.push(a[0].id), check(...a));
    const det = new ChangeDetector({ map: m, house: h, memory });
    det.evidence = async () => ({ live: jpegOf(640, 480), expected: jpegOf(640, 480, 6) });
    bodies.length = 0;
    respond = () => answer({ verdict: "real-change", kind: "obstacle", what: "a box", confidence: 0.9, why: "new" }, { input_tokens: 1300, output_tokens: 200 });
    memory.startFlight({ kind: "patrol" });
    await det.report({ near: true, x: 0.9, y: 2.4, z: 0.4, size: 0.4, zMin: 0, zMax: 0.8, room: "r1", n: 6 }, 0);
    await settle();
    const ch = memory.changes[0];
    assert.deepEqual([checks.length, bodies.length, prompts.length, needed], [1, 0, 0, ["change"]], "checked once, no prompt in the air");
    assert.ok(ch.claude.waiting && ch.status === "suspected" && m.temps.has(ch.id), "waiting, and an obstacle meanwhile");
    memory.addChange({ kind: "obstacle", x: 0.95, y: 2.4, z: 0.4, size: 0.4 });
    memory.annotateChange(ch.id, { note: "x" });
    await settle();
    assert.equal(checks.length, 1, "seen again or annotated: no second check");
    flying = false;
    memory.endFlight("Landed.");
    await settle();
    assert.deepEqual([prompts, bodies.length, ch.status, settings.get("aiVision")], [["change"], 1, "confirmed", "on"], "landed: asked, checked, confirmed");
    assert.ok(!det.reported.some((r) => r.memoryId === ch.id || r.id === ch.id), "nav/changes.js let go of it");
    assert.ok(!m.temps.has(ch.id) && h.keepouts.some((k) => k.change === ch.id));
    // No key: unconfirmed, once, and the event loop goes on.
    const { house: h2, map: m2 } = await fixture(), mem2 = await HouseMemory.open(h2.id, { indexedDB: null });
    mem2.setHouse({ house: h2, map: m2 });
    const ins2 = new Inspector({ claude: claudeWith({ apiKey: "" }).claude, memory: mem2, watch: true }), calls2 = [], c2 = ins2.checkChange.bind(ins2);
    ins2.checkChange = (...a) => (calls2.push(1), c2(...a));
    let ticked = false;
    setTimeout(() => (ticked = true), 0);
    mem2.addChange({ kind: "obstacle", x: 1, y: 2, size: 0.4, evidence: { live: jpegOf(640, 480), expected: jpegOf(640, 480) } });
    await settle();
    assert.deepEqual([calls2.length, ticked, mem2.changes[0].claude.status], [1, true, "unconfirmed"]);
    // The banner (no prompt hook): "once" lets the waiting checks go, that once.
    const { house: h3, map: m3 } = await fixture(), mem3 = await HouseMemory.open(h3.id, { indexedDB: null });
    mem3.setHouse({ house: h3, map: m3 });
    const { claude: c3, settings: s3 } = claudeWith({ aiVision: "ask" });
    new Inspector({ claude: c3, memory: mem3, watch: true });
    bodies.length = 0;
    const w = mem3.addChange({ kind: "obstacle", x: 1, y: 2, size: 0.4, evidence: { live: jpegOf(640, 480), expected: jpegOf(640, 480) } });
    await settle();
    assert.ok(w.claude.waiting && bodies.length === 0);
    c3.answerConsent("once");
    await settle();
    assert.deepEqual([bodies.length, s3.get("aiVision"), w.status], [1, "ask", "confirmed"]);
  },

  async "a person or pet Claude names, with the real nav/changes.js (which never reports a spot twice): blocking while the camera has it in view and a minute after, then let go"() {
    const { house: h, map: m } = await fixture();
    let t = 1_000_000;
    const memory = await HouseMemory.open(h.id, { indexedDB: null, now: () => t }), adds = [], add = memory.addChange.bind(memory);
    memory.setHouse({ house: h, map: m });
    memory.addChange = (c, ...a) => (adds.push(c.kind), add(c, ...a));
    new Inspector({ claude: claudeWith().claude, memory, watch: true });
    const det = new ChangeDetector({ map: m, house: h, memory });
    det.evidence = async () => ({ live: jpegOf(640, 480), expected: jpegOf(640, 480, 6) });
    bodies.length = 0;
    respond = () => answer({ verdict: "person-or-pet", kind: "none", what: "the dog", confidence: 0.92, why: "a dog lying down" }, { input_tokens: 1300, output_tokens: 200 });
    memory.startFlight({ kind: "patrol" });
    const cand = () => ({ near: true, x: 0.9, y: 2.4, z: 0.4, size: 0.5, zMin: 0, zMax: 0.8, room: "r1", n: 6 });
    const pose = (yaw) => memory.samplePose({ x: 0.0, y: 2.4, z: 1, yaw, sigma: 0.05, status: "ok" });
    await det.report(cand(), null, 0);
    await settle();
    const ch = memory.changes[0];
    assert.ok(ch.passing && m.temps.get(ch.id)?.kind === "person", "Claude: a person or pet, an obstacle meanwhile");
    for (let s = 1; s <= 150; s++) {
      t += 1000;
      await det.report(cand(), null, s * 1000); // the detector still sees it: a reported spot, nothing new
      pose(0.2);
    }
    assert.deepEqual([adds.length, bodies.length], [1, 1], "reported once, checked once");
    assert.equal(ch.status, "suspected", "150 s in view: still blocking");
    assert.ok(!m.free(0.9, 2.4, 1.0) && det.reported.length === 1);
    for (let s = 1; s <= 50; s++) (t += 1000), pose(Math.PI);
    assert.equal(ch.status, "suspected", "50 s looking away");
    assert.ok(memory.seen(ch.id), "the detector's own sign (AUTONOMY's wire): a minute more");
    for (let s = 1; s <= 59; s++) (t += 1000), pose(Math.PI);
    assert.equal(ch.status, "suspected");
    for (let s = 1; s <= 2; s++) (t += 1000), pose(Math.PI);
    assert.deepEqual([ch.status, ch.transient, ch.by], ["dismissed", true, "passing"], "a minute out of view: gone");
    assert.ok(m.free(0.9, 2.4, 1.0) && !det.reported.length, "off the map, and nav/changes.js let go of it");
    await det.report(cand(), null, 400000);
    await settle();
    assert.deepEqual([adds.length, memory.changes.length, bodies.length], [2, 2, 2], "there again later: a new change, checked again");
    memory.noteUnchanged({ x: 0.9, y: 2.4, z: 0.4, r: 0.25 });
    assert.deepEqual([memory.changes[1].status, memory.changes[1].by], ["dismissed", "detector"], "the spot looks as the scan expects: gone at once");
  },

  async "Claude for real: structured outputs with adaptive thinking (never a forced tool), refusal fallbacks, priced by the model that answered, estimates that learn"() {
    const check = (claude) => new Inspector({ claude }).confirmDetection(jpeg(), "cat", 0.4);
    const ok = { present: true, actual: "a cat", confidence: 0.9, why: "on the sofa" };
    bodies.length = heads.length = 0;
    respond = () => answer(ok, { input_tokens: 900, output_tokens: 120 });
    const opus = claudeWith({ aiModel: "claude-opus-5-5" }).claude;
    assert.equal((await check(opus)).status, "confirmed");
    const b = bodies[0];
    assert.deepEqual([b.model, b.max_tokens, b.thinking, b.output_config.effort, b.output_config.format.type, b.tool_choice, b.tools, b.fallbacks],
      ["claude-opus-5-5", 16000, { type: "adaptive" }, "low", "json_schema", undefined, undefined, "default"]);
    assert.match(heads[0].get("anthropic-beta") ?? "", /server-side-fallback-2026-07-01/, "refusal fallbacks, routed by category");
    assert.ok(JSON.stringify(b.output_config.format.schema).includes('"additionalProperties":false') && !/"(minimum|maximum|minLength|maxLength)"/.test(JSON.stringify(b.output_config.format.schema)), "a schema structured outputs take");
    // Haiku 4.5: structured outputs, no adaptive thinking; Sonnet 4.6 (no structured outputs): the schema in the prompt, JSON read from the text
    await check(claudeWith({ aiModel: "claude-haiku-4-5" }).claude);
    assert.ok(bodies[1].output_config.format && !bodies[1].thinking && !bodies[1].fallbacks);
    respond = () => ({ content: [{ type: "text", text: `Here it is: ${JSON.stringify(ok)}` }], usage: { input_tokens: 900, output_tokens: 120 } });
    assert.equal((await check(claudeWith({ aiModel: "claude-sonnet-4-6" }).claude)).status, "confirmed");
    assert.ok(!bodies[2].output_config?.format && /matches this JSON schema/.test(bodies[2].messages[0].content.at(-1).text));
    // a refusal: said, not parsed; a fallback (usage.iterations): each attempt at its own model's price, served by the one that answered
    respond = () => ({ stop_reason: "refusal", stop_details: { type: "refusal", category: "cyber" }, content: [], usage: { input_tokens: 900, output_tokens: 0 } });
    const no = await opus.call({ purpose: "detection", system: "x", content: [], schema: { type: "object" } });
    assert.deepEqual([no.ok, no.refused, no.reason], [false, true, "Claude declined to answer (cyber)"]);
    respond = () => ({ model: "claude-opus-4-8", ...answer(ok, { input_tokens: 900, output_tokens: 120, iterations: [
      { type: "message", model: "claude-opus-5-5", input_tokens: 900, output_tokens: 0 }, { type: "fallback_message", model: "claude-opus-4-8", input_tokens: 900, output_tokens: 120 }] }) });
    const fb = await opus.call({ purpose: "detection", system: "x", content: [], schema: { type: "object" } });
    assert.equal(fb.cost.toFixed(6), ((900 * 4 + 900 * 5 + 120 * 25) / 1e6).toFixed(6));
    assert.deepEqual([fb.model, fb.note], ["claude-opus-4-8", "answered by claude-opus-4-8 after claude-opus-5-5 declined"]);
    // estimates learn: answers three times as long as expected make the next budget check ask for more
    const learn = claudeWith({ aiModel: "claude-opus-5-5" }).claude, before = learn.estimate({ system: "x", content: [], expectOut: 300 }).cost;
    respond = () => answer(ok, { input_tokens: 2, output_tokens: 900 });
    for (let i = 0; i < 4; i++) await learn.call({ purpose: "detection", system: "x", content: [], schema: { type: "object" }, expectOut: 300 });
    console.log(`      opus 5.5: $${before.toFixed(5)} estimated, $${(900 * 20 / 1e6).toFixed(5)} real; the detection estimate now runs x${learn.accuracy().detection}`);
    assert.ok(learn.accuracy().detection > 2);
    learn.settings.set("aiBudget", learn.flight.spent + 1.5 * before);
    assert.match((await learn.call({ purpose: "detection", system: "x", content: [], schema: { type: "object" }, expectOut: 300 })).reason ?? "", /budget for this flight is used up/);
  },

  async "Claude on the memory: a question the rules don't understand, and a note after landing; text only, within the budget, never with Claude's checks off"() {
    const { house: h, map: m } = await fixture();
    let t = 1.77e12;
    const memory = await HouseMemory.open(`${h.id}:ask`, { indexedDB: null, now: () => t });
    memory.setHouse({ house: h, map: m });
    memory.startFlight({ kind: "patrol" });
    memory.addSighting({ label: "dog", x: 1.8, y: 3.0, t });
    memory.endFlight("Patrolled.", (t += 60000));
    bodies.length = 0;
    respond = (body) => answer(/after their small indoor drone landed/.test(body.system[0].text) ? { summary: "A quiet patrol; the dog was in the Living room." } : { answer: "Yes, the dog was in the Living room a minute ago.", found: true }, { input_tokens: 800, output_tokens: 60 });
    const { claude } = claudeWith();
    const a = await askMemory({ claude, memory, question: "has the pup been about?" });
    assert.ok(a.ok && a.found && /dog/.test(a.text) && /dog in the Living room/.test(a.local ?? ""));
    assert.ok(!imagesOf(bodies[0]).length && /Records, newest first:\n- [\s\S]*Saw the dog in the Living room/.test(textOf(bodies[0])), "the memory as text, no pictures");
    const s = await summarizeFlight({ claude, memory });
    assert.equal(s.text, "A quiet patrol; the dog was in the Living room.");
    assert.match(textOf(bodies[1]), /^Flight report: Flight: 1 min, /);
    claude.settings.set("aiVision", "off");
    const off = await askMemory({ claude, memory, question: "has the pup been about?" });
    assert.ok(!off.ok && bodies.length === 2 && off.text === a.local, "off: not sent, the rules' answer stays");
  },

  async "agent: memory rides in the messages; the cached prefix stays byte-identical while memory changes"() {
    const sent = [];
    const old = globalThis.fetch;
    globalThis.fetch = async (url, init) => (sent.push(JSON.parse(init.body)),
      new Response(JSON.stringify({ id: "m", type: "message", role: "assistant", model: "claude-opus-5", stop_reason: "end_turn", content: [{ type: "text", text: "Done." }], usage: { input_tokens: 10, output_tokens: 2 } }), { status: 200, headers: { "content-type": "application/json" } }));
    try {
      const settings = settingsWith({ autonomy: "full", homeNotes: "" });
      const ctl = { tel: { vbat: 4 }, autonomy: "full", isFlying: () => true, blocker: () => null, est: { heading: 0 }, imageHeading: () => 0 };
      const tools = new ToolBox({ ctl, perception: { latest: { detections: [] }, snapshot: () => null }, settings, speak: () => {} });
      const memory = await HouseMemory.open(house.id, { indexedDB: null });
      memory.setHouse({ house, map });
      tools.setHouse({ house, map, missions: { whereAmI: () => "In the Living room." }, localizer: { pose: () => ({ status: "lost" }) }, memory });
      const agent = new Agent({ tools, settings, ctl });
      await agent.run("where did you see the cat?");
      memory.startFlight({ kind: "manual" });
      memory.addChange({ kind: "obstacle", x: 1, y: 2, size: 0.3 });
      await agent.run("what changed?");
      const prefix = (b) => JSON.stringify([b.model, b.tools, b.system, b.thinking, b.output_config]);
      assert.equal(prefix(sent[0]), prefix(sent[1]), "byte-identical prefix");
      assert.ok(!/Memory:|This flight/.test(JSON.stringify(sent[1].system)), "nothing from memory in the system prompt");
      assert.match(sent[1].messages[0].content.at(-1).text, /\nMemory: This flight: .*1 possible change from the 3D scan not confirmed yet/);
      assert.ok(!/Memory:/.test(sent[0].messages[0].content.at(-1).text), "nothing to say before a flight");
      assert.match(sent[0].system[1].text, /recall answers questions about them/);
      assert.equal(sent[0].system[0].text, SYSTEM_PROMPT);
    } finally {
      globalThis.fetch = old;
    }
  },
};

let failed = 0;
const t0 = performance.now();
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name}\n      ${e.message.split("\n").slice(0, 8).join("\n      ")}\n      ${e.stack?.split("\n").find((l) => l.includes("test-ai.mjs:"))?.trim() ?? ""}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed (${((performance.now() - t0) / 1000).toFixed(1)} s)`);
process.exit(failed ? 1 : 0);
