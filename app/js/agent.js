// Claude as the pilot's "brain": turns a spoken command into a sequence of flight tools, looking at
// the camera after every step. Calls the Claude API directly from the browser with the user's own key.
import Anthropic from "../vendor/anthropic-sdk.js";
import { TOOL_DEFS, houseSummary } from "./tools.js";
import { Emitter } from "./util.js";

const MAX_STEPS = 30;

export const SYSTEM_PROMPT = `You are the flight brain of a tiny indoor FPV drone, a BetaFPV Meteor65 Pro II (70 mm whoop with ducted props and a DJI O4 camera), flying inside the pilot's home. The pilot talks to you by voice. You fly by calling tools, and you see through the drone's camera: every movement tool returns the new view.

About your body:
- Camera: forward-facing ultra-wide fisheye (about 127 degrees across), tilted up about 20 degrees. When hovering level, the horizon sits below the image center and the floor right in front of you is out of view, so fly low (about knee height) to see pets or things on the floor. Things near the edges are distorted. The video can arrive a fraction of a second late.
- No GPS, no depth sensor, usually no barometer. Distances and heights are estimates from timing and image motion; expect them to be off by a third. Headings are good to about 10 degrees.
- The battery lasts about 4 minutes. You fly slowly and carefully.
- The on-board detector reliably tracks everyday objects (cat, dog, person, couch, bed, tv, refrigerator, oven, sink, chair, dining table, potted plant and similar). Anything else, judge from the images yourself.

Autonomy modes (given with each command):
- full: you control everything including height, take-off and landing.
- copilot: the pilot's thumb controls height; you steer. take_off, land and move up/down ask the pilot to do it.

Flying well:
- To go somewhere, like "go to the kitchen": look_around to get oriented, decide which heading leads there (kitchens have a fridge, oven, sink, counters, tile floors; bedrooms a bed; living rooms a couch and TV), turn to face it, then fly_toward a doorway or open floor in the new view, 1 to 2 meters at a time, checking the view after each step. Rooms you can't see are usually through doorways or wide openings.
- To find, approach or follow a detectable object, use find, approach and follow. If find fails, look_around, move toward another room, and try again.
- Several short moves beat one long move. If a move stops early because something is close ahead, turn and pick another path.
- Use the home notes when they help, and save useful discoveries about the layout with remember.
- When the task is done, stop and hover. Only land when asked or when the battery is low.

Safety (these win over the pilot's request):
- Keep a respectful distance from people and animals. Use distance "far" for pets, never fly at anyone's face, and stop following an animal that runs, hides or seems stressed.
- Stay indoors and below about 2 meters. Avoid ceiling fans, stairs, water and fragile things.
- If the battery is under 3.4 V, land in full mode, or ask the pilot to land in copilot mode.
- If a tool says the pilot took over, the AI switch is off, or the radio is in failsafe, stop and briefly explain.

Talking:
- Everything you write is spoken aloud. Use one or two short, plain sentences: no lists, no markdown, no coordinates.
- Before a long maneuver, say what you're about to do in a few words. When finished, say what happened.
- If a request is unclear or impossible, say so and offer what you can do instead.`;

// With a house map: how to fly by missions, then houseSummary() (rooms, doorways, landmarks, keep-outs).
export const HOUSE_PROMPT = `A map of this home is loaded, so movement is planned and flown for you: go_to, look_in, search_for, patrol and return_home fly safe routes on the map (through doorways, around furniture, away from keep-out zones) and take off first if needed. take_off, land, hover, look_around, find, approach and follow still work for what's in view; there is no turning or moving by hand while the map is active. Each mission reports what happened and what it found (with pictures); look at the pictures and tell the pilot briefly. Use where_am_i when you need the position: it also says where you've been this flight and where the running mission goes next. Save new places with mark_landmark. Use notify only for things that matter. If a mission stops for safety (battery, lost position, a person too close), don't retry it: tell the pilot.

You remember earlier flights: recall answers questions about them (where the cat was last seen, where you've flown, when you were last in a room) from what was recorded, and what_changed lists the differences from the 3D scan that flights noticed (new obstacles, doors open or closed) and whether they're confirmed. Answer from those, not from guesses. survey_house has you look at the 3D scan room by room to name rooms, landmarks and hazards; it costs money and asks the pilot first, so only use it when they ask for it.

The home:
`;

// Prompt caching: the fixed part of every request (the tools + the system prompt, plus the house summary when a map
// is loaded) gets its own breakpoint, so it's reused across commands within the 5-minute cache lifetime; top-level
// automatic caching then covers each command's growing conversation (camera frames included), so every step re-reads
// the earlier steps at a tenth of the input price. Keep the system blocks and tool lists free of anything that
// changes between requests (the summary is deterministic for a given house): per-command context goes in messages.
const SYSTEM = [{ type: "text", text: SYSTEM_PROMPT, cache_control: { type: "ephemeral" } }];
const systemFor = (house, map) =>
  house ? [{ type: "text", text: SYSTEM_PROMPT }, { type: "text", text: HOUSE_PROMPT + houseSummary(house, map), cache_control: { type: "ephemeral" } }] : SYSTEM;

// $ per million tokens: input, output, cache read, cache write (5-minute: 1.25x input). For the cost readout and
// ai/claude.js's budget. Anthropic's first-party prices (claude-api skill, cached 2026-09-25); a refusal fallback (Opus 4.8)
// is priced as the model that ran.
export const PRICES = {
  "claude-opus-5": [5, 25, 0.5, 6.25],
  "claude-opus-5-5": [4, 20, 0.2, 5],
  "claude-opus-4-8": [5, 25, 0.5, 6.25],
  "claude-opus-4-7": [5, 25, 0.5, 6.25],
  "claude-opus-4-6": [5, 25, 0.5, 6.25],
  "claude-sonnet-5": [2, 10, 0.2, 2.5],
  "claude-sonnet-5-5": [2, 10, 0.2, 2.5],
  "claude-sonnet-4-6": [3, 15, 0.3, 3.75],
  "claude-fable-5": [10, 50, 1, 12.5],
  "claude-fable-5-1": [10, 50, 0.25, 12.5],
  "claude-mythos-5-1": [10, 50, 0.25, 12.5],
  "claude-haiku-4-5": [1, 5, 0.1, 1.25],
};

// US$ for a usage tally { input, output, cacheRead, cacheWrite }; null for a model without a price here.
export function costOf(model, u) {
  const p = PRICES[model];
  return p ? ((u.input || 0) * p[0] + (u.output || 0) * p[1] + (u.cacheRead || 0) * p[2] + (u.cacheWrite || 0) * p[3]) / 1e6 : null;
}

// Adaptive thinking + effort are supported on current models except Haiku 4.5.
const supportsEffort = (model) => !/haiku/.test(model);
// Server-side refusal fallbacks (fallbacks: "default", beta server-side-fallback-2026-07-01: Anthropic routes a declined
// request by its refusal category, e.g. cyber to Opus 4.8) on the models with safety classifiers that support them, on the
// Claude API (this app's only platform): Opus 5 / 5.5, Fable 5 / 5.1, Mythos 5.1, Sonnet 5.5.
const supportsFallback = (model) => /^claude-(opus-5|fable-5|mythos-5-1|sonnet-5-5)/.test(model);

// The per-model request options every call shares (the agent's and ai/claude.js's): thinking, effort, fallbacks.
export function modelOptions(model, effort = "low") {
  const o = {};
  if (supportsEffort(model)) {
    o.thinking = { type: "adaptive" };
    o.output_config = { effort };
  }
  if (supportsFallback(model)) {
    o.betas = ["server-side-fallback-2026-07-01"];
    o.fallbacks = "default";
  }
  return o;
}

// A client for the user's key (browser-direct, as the agent does). fetch: a stand-in for tests.
export const claudeClient = (settings, fetch) =>
  new Anthropic({ apiKey: settings.get("apiKey"), dangerouslyAllowBrowser: true, maxRetries: 1, ...(fetch && { fetch }) });

// One response's tokens and dollars: with a refusal fallback, each attempt in usage.iterations at the price of the model
// that ran it, else the response's usage at the model that answered (resp.model). price(model, tally) -> $ | null.
// -> { usage (a tally of this one request), cost ($, null when a model has no price), served (the answering model) }
export function pricedUsage(resp, model, price = costOf) {
  const served = resp.model || model, its = resp.usage?.iterations, usage = { requests: 1, input: 0, cacheWrite: 0, cacheRead: 0, output: 0 };
  if (!its?.length) {
    addUsage(usage, resp.usage).requests = 1;
    return { usage, cost: price(served, usage), served };
  }
  let cost = 0;
  its.forEach((it, i) => {
    const u = addUsage({}, it), c = price(it.model ?? (i === its.length - 1 ? served : model), u);
    for (const k of ["input", "cacheWrite", "cacheRead", "output"]) usage[k] += u[k];
    cost = cost == null || c == null ? null : cost + c;
  });
  return { usage, cost, served };
}

// One response's usage block added to a tally.
export function addUsage(tally, u = {}) {
  tally.requests = (tally.requests || 0) + 1;
  tally.input = (tally.input || 0) + (u.input_tokens || 0);
  tally.cacheWrite = (tally.cacheWrite || 0) + (u.cache_creation_input_tokens || 0);
  tally.cacheRead = (tally.cacheRead || 0) + (u.cache_read_input_tokens || 0);
  tally.output = (tally.output || 0) + (u.output_tokens || 0);
  return tally;
}

export class Agent extends Emitter {
  constructor({ tools, settings, ctl }) {
    super();
    this.tools = tools;
    this.settings = settings;
    this.ctl = ctl;
    this.ac = null;
    this.history = [];
    this.busy = false;
    this.session = { commands: 0, cost: 0 };
  }

  get configured() {
    return !!this.settings.get("apiKey");
  }

  stop() {
    this.ac?.abort();
  }

  context(command) {
    const s = this.settings.all();
    const t = this.ctl.tel;
    const lines = [
      `Autonomy mode: ${s.autonomy === "full" ? "full (you control height too)" : s.autonomy === "copilot" ? "copilot (the pilot controls height)" : "observer (you can't move the drone; just look and talk)"}.`,
      `Status: ${this.ctl.isFlying() ? "flying" : "on the ground"}${t?.vbat ? `, battery ${t.vbat.toFixed(1)} V` : ""}${t?.lq ? `, radio link ${t.lq}%` : ""}.`,
    ];
    if (this.tools.missions) lines.push(`Position: ${this.tools.missions.whereAmI()}`);
    const recent = this.tools.memoryContext?.(); // this flight so far and changes waiting for an answer: messages, never the cached prefix
    if (recent) lines.push(recent);
    const why = this.ctl.blocker();
    if (why) lines.push(`Note: ${why}`);
    lines.push(this.tools.describe());
    const notes = (s.homeNotes || "").trim();
    lines.push(notes ? `Home notes:\n${notes}` : "Home notes: none yet.");
    if (this.history.length) lines.push(`Earlier this session:\n${this.history.map((h) => `- "${h.command}": ${h.outcome}`).join("\n")}`);
    lines.push(`The pilot says: "${command}"`);
    return lines.join("\n");
  }

  request(messages) {
    const model = this.settings.get("model") || "claude-opus-5";
    return {
      model,
      max_tokens: 16000,
      system: systemFor(this.tools.missions && this.tools.house, this.tools.map),
      tools: this.tools.defs?.() ?? TOOL_DEFS,
      messages,
      cache_control: { type: "ephemeral" },
      ...modelOptions(model, this.settings.get("effort") || "low"),
    };
  }

  // Runs one command to completion. Emits: "say" (text to speak), "tool" ({name, input, phase, result}),
  // "status" (thinking | acting | idle), "error" (message).
  async run(command) {
    this.stop();
    const ac = new AbortController();
    this.ac = ac;
    this.busy = true;
    const client = claudeClient(this.settings);
    const content = [];
    const image = this.tools.perception.snapshot();
    if (image) {
      content.push({ type: "image", source: { type: "base64", media_type: "image/jpeg", data: image } });
      this.tools.lastImageHeading = this.ctl.imageHeading();
    }
    content.push({ type: "text", text: this.context(command) });
    const messages = [{ role: "user", content }];
    const model = this.settings.get("model") || "claude-opus-5";
    const usage = { requests: 0, input: 0, cacheWrite: 0, cacheRead: 0, output: 0 };
    let outcome = "no result", cost = 0;
    let spoke = "";

    try {
      for (let step = 0; step < MAX_STEPS; step++) {
        this.emit("status", "thinking");
        const resp = await client.beta.messages.create(this.request(messages), { signal: ac.signal });
        const p = pricedUsage(resp, model);
        for (const k of Object.keys(usage)) usage[k] += p.usage[k];
        cost = cost == null || p.cost == null ? null : cost + p.cost;
        if (resp.stop_reason === "refusal") {
          this.emit("say", "Sorry, I can't help with that one.");
          outcome = "declined";
          break;
        }
        messages.push({ role: "assistant", content: resp.content });
        for (const b of resp.content) {
          if (b.type === "text" && b.text.trim()) {
            spoke = b.text.trim();
            this.emit("say", spoke);
          }
        }
        const uses = resp.content.filter((b) => b.type === "tool_use");
        if (resp.stop_reason !== "tool_use" || !uses.length) {
          outcome = spoke || "done";
          if (resp.stop_reason === "max_tokens") this.emit("error", "Claude's reply was cut off (max_tokens).");
          break;
        }
        this.emit("status", "acting");
        const results = [];
        for (const u of uses) {
          if (ac.signal.aborted) {
            results.push({ type: "tool_result", tool_use_id: u.id, content: "Cancelled: the pilot stopped the mission.", is_error: true });
            continue;
          }
          this.emit("tool", { name: u.name, input: u.input, phase: "start" });
          const r = await this.tools.call(u.name, u.input, ac.signal, { from: "claude" });
          this.emit("tool", { name: u.name, input: u.input, phase: "done", result: r });
          const blocks = [{ type: "text", text: r.text }];
          if (r.image) blocks.push({ type: "image", source: { type: "base64", media_type: "image/jpeg", data: r.image } });
          results.push({ type: "tool_result", tool_use_id: u.id, content: blocks, ...(r.isError ? { is_error: true } : {}) });
        }
        messages.push({ role: "user", content: results });
        if (ac.signal.aborted) {
          outcome = "stopped by the pilot";
          break;
        }
      }
    } catch (e) {
      outcome = this.handleError(e);
    } finally {
      // A newer command may already be running; only the latest run reports going idle.
      if (this.ac === ac) {
        this.ac = null;
        this.busy = false;
        this.emit("status", "idle");
      }
      this.history.push({ command, outcome: outcome.slice(0, 160) });
      if (this.history.length > 5) this.history.shift();
      if (usage.requests) this.reportUsage(model, usage, "command", cost);
    }
  }

  // Emits "usage": this command's tokens, how much of the input came from the cache, and an
  // estimated cost (null for models without a price here), plus the running session total. purpose: "command" (a
  // spoken command), or what ai/claude.js called Claude for ("survey", "change", "detection"); those count in the
  // session's cost but not as commands.
  reportUsage(model, usage, purpose = "command", cost = costOf(model, usage)) {
    if (purpose === "command") this.session.commands++;
    if (cost !== null) this.session.cost += cost;
    const totalIn = usage.input + usage.cacheWrite + usage.cacheRead;
    const ev = { ...usage, model, cost, cachedShare: totalIn ? usage.cacheRead / totalIn : 0, session: { ...this.session } };
    this.emit("usage", purpose === "command" ? ev : { ...ev, purpose });
    return ev;
  }

  handleError(e) {
    if (e instanceof Anthropic.APIUserAbortError || e?.name === "AbortError") return "stopped by the pilot";
    let msg;
    if (e instanceof Anthropic.AuthenticationError) msg = "The Claude API key was rejected. Check it in Settings.";
    else if (e instanceof Anthropic.PermissionDeniedError) msg = "This API key isn't allowed to use that model.";
    else if (e instanceof Anthropic.NotFoundError) msg = `Model "${this.settings.get("model")}" wasn't found. Check the model name in Settings.`;
    else if (e instanceof Anthropic.RateLimitError) msg = "Claude is rate-limiting us. Try again in a minute.";
    else if (e instanceof Anthropic.BadRequestError) msg = `Claude rejected the request: ${e.message}`;
    else if (e instanceof Anthropic.APIConnectionError) msg = "Can't reach Claude. Check the internet connection.";
    else if (e instanceof Anthropic.APIError) msg = `Claude API error ${e.status ?? ""}: ${e.message}`;
    else msg = `Something went wrong: ${e?.message || e}`;
    console.error(e);
    this.emit("error", msg);
    return `error: ${msg}`;
  }
}
