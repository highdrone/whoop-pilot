// The drone's abilities, described for Claude (TOOL_DEFS) and implemented on top of the flight
// controller (ToolBox). The offline command parser uses the same ToolBox. With a house map (setHouse) Claude gets
// HOUSE_TOOL_DEFS instead: missions (go_to, look_in, search_for, patrol, return_home) planned on the map and flown by
// missions.js, and no raw motion (turn, move, fly_toward): Claude picks where to go and reads the pictures, it never
// commands velocities while a map is active.
import { Hold, Turn, Move, TakeOff, Land, Search, Approach, LookAround, WaitFor } from "./behaviors.js";
import { COCO_LABELS, toDetectorLabel } from "./detector.js";
import { DEG, clamp } from "./util.js";
import { theRoom, MISSION } from "./missions.js";
import { estimateSurvey, surveyHouse, surveySummary } from "./ai/survey.js";
import { toJpegBase64 } from "./ai/claude.js";

const NO_INPUT = { type: "object", properties: {}, additionalProperties: false };
const DISTANCE = { type: "string", enum: ["close", "medium", "far"], description: "How close to get. Use far for pets." };
const HOME_LABELS = COCO_LABELS.filter((l) => !["car", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "skis", "snowboard", "surfboard"].includes(l));

export const TOOL_DEFS = [
  {
    name: "take_off",
    description: "Take off and hover about 1 m up. In co-pilot mode the human pilot controls height, so this asks them to take off and waits until the quad is flying.",
    input_schema: NO_INPUT,
  },
  {
    name: "land",
    description: "Land where we are. In co-pilot mode this asks the pilot to land.",
    input_schema: NO_INPUT,
  },
  {
    name: "hover",
    description: "Hold position (damping drift and keeping the heading) for a number of seconds.",
    input_schema: { type: "object", properties: { seconds: { type: "number", description: "1 to 30" } }, required: ["seconds"], additionalProperties: false },
  },
  {
    name: "turn",
    description: "Rotate in place. Positive degrees turn right (clockwise), negative turn left. Accurate to roughly 10 degrees. Returns the new view.",
    input_schema: { type: "object", properties: { degrees: { type: "number", description: "-180 to 180" } }, required: ["degrees"], additionalProperties: false },
  },
  {
    name: "move",
    description: "Fly in a direction relative to where the camera points, for an approximate distance (estimated from time and speed; there is no rangefinder). Forward moves stop early if something is very close ahead. In co-pilot mode up/down ask the pilot to change height. Returns the new view.",
    input_schema: {
      type: "object",
      properties: {
        direction: { type: "string", enum: ["forward", "back", "left", "right", "up", "down"] },
        meters: { type: "number", description: "0.2 to 4" },
      },
      required: ["direction", "meters"],
      additionalProperties: false,
    },
  },
  {
    name: "look_around",
    description: "Turn a full circle in 8 steps and return one labeled contact-sheet image of what the camera saw at each heading (0 = the heading we started at, +45 = 45 degrees to the right, ... +315 = 45 degrees to the left), plus the objects the detector found at each heading. Ends facing the original direction. Use it to get oriented or to pick which way a room is.",
    input_schema: NO_INPUT,
  },
  {
    name: "fly_toward",
    description: "Turn toward a point in the most recent single camera image you were shown (not a contact sheet) and fly forward about the given distance. Use it for things the detector can't track: a doorway, a hallway opening, a spot on the floor. x and y are normalized image coordinates: (0,0) is top-left, (1,1) is bottom-right.",
    input_schema: {
      type: "object",
      properties: {
        x: { type: "number", description: "0 to 1, left to right" },
        y: { type: "number", description: "0 to 1, top to bottom" },
        meters: { type: "number", description: "0.3 to 4" },
      },
      required: ["x", "y", "meters"],
      additionalProperties: false,
    },
  },
  {
    name: "find",
    description: `Rotate in place until the fast on-board detector sees an object, then face it. Only works for these detector labels: ${HOME_LABELS.join(", ")}. For anything else (rooms, doorways, specific items), use look_around and fly_toward and judge the images yourself.`,
    input_schema: { type: "object", properties: { object: { type: "string", description: "A detector label, e.g. cat" } }, required: ["object"], additionalProperties: false },
  },
  {
    name: "approach",
    description: "Fly up to a detectable object (same labels as find), keeping it centered, and stop at a polite distance. Searches for it first if it isn't in view.",
    input_schema: { type: "object", properties: { object: { type: "string" }, distance: DISTANCE }, required: ["object"], additionalProperties: false },
  },
  {
    name: "follow",
    description: "Keep a detectable object (cat, dog, person, ...) centered at a steady distance while it moves, for up to the given seconds. If it is lost, searches toward where it was last seen. Ends early if the target is lost for good or the pilot says stop.",
    input_schema: {
      type: "object",
      properties: { object: { type: "string" }, seconds: { type: "number", description: "5 to 180" }, distance: DISTANCE },
      required: ["object", "seconds"],
      additionalProperties: false,
    },
  },
  {
    name: "snapshot",
    description: "Get the current camera image and detector results without moving.",
    input_schema: NO_INPUT,
  },
  {
    name: "say",
    description: "Say something out loud to the pilot right now (text-to-speech) and keep going. One short sentence.",
    input_schema: { type: "object", properties: { text: { type: "string" } }, required: ["text"], additionalProperties: false },
  },
  {
    name: "remember",
    description: "Save a short note about this home for future flights, e.g. 'The kitchen is through the wide opening to the left of the TV.' Saved notes are included with every command.",
    input_schema: { type: "object", properties: { note: { type: "string" } }, required: ["note"], additionalProperties: false },
  },
];

const OPTIONAL_ROOMS = { type: "array", items: { type: "string" }, description: "Room names; leave out for every room" };

// Missions on the house map, plus the old tools that don't fly a course by hand.
export const HOUSE_TOOL_DEFS = [
  {
    name: "where_am_i",
    description: "Where the drone is on the house map: the room, a nearby landmark, how far the home pad is, how sure the position is, the height, battery and flying time left. Doesn't move.",
    input_schema: NO_INPUT,
  },
  {
    name: "go_to",
    description: "Fly to a place: a room by name, a landmark (one in the house summary or saved with mark_landmark, e.g. 'sofa'), or 'home'. The route is planned on the map and flown automatically, through doorways and around furniture, with safety checks; takes off first if needed. Returns the view on arrival.",
    input_schema: { type: "object", properties: { place: { type: "string" } }, required: ["place"], additionalProperties: false },
  },
  {
    name: "look_in",
    description: "Look into a room: fly to its doorway and look around (going further in only if the doorway sees too little). Returns a contact sheet of what the camera saw in each direction and any people or pets the detector found, with where they are.",
    input_schema: { type: "object", properties: { room: { type: "string" } }, required: ["room"], additionalProperties: false },
  },
  {
    name: "search_for",
    description: "Search rooms, nearest first, for 'person' (stops at the first), 'people' (checks every room), 'cat', 'dog', 'pets' or another detector label, from spots that see each whole room (low for pets). Reports the room and nearby landmark of each find, with a picture. For things the detector can't recognize it returns views of the rooms for you to judge.",
    input_schema: { type: "object", properties: { target: { type: "string" }, rooms: OPTIONAL_ROOMS }, required: ["target"], additionalProperties: false },
  },
  {
    name: "patrol",
    description: "Check every room (or the listed ones) in the shortest loop, then fly home and land. Reports the people and pets it saw; a person also sends an alert.",
    input_schema: { type: "object", properties: { rooms: OPTIONAL_ROOMS }, additionalProperties: false },
  },
  {
    name: "return_home",
    description: "Fly back to the home pad and land (in co-pilot mode: hover over it and ask the pilot to land).",
    input_schema: NO_INPUT,
  },
  {
    name: "set_home",
    description: "Make the drone's current spot the home pad, where missions start, return to and land. Best done while it sits on the pad.",
    input_schema: NO_INPUT,
  },
  {
    name: "mark_landmark",
    description: "Save the drone's current spot on the map under a name (e.g. 'cat bed', 'front door'), so go_to can fly there later.",
    input_schema: { type: "object", properties: { name: { type: "string" } }, required: ["name"], additionalProperties: false },
  },
  {
    name: "recall",
    description: "Answer a question about earlier flights from the flight memory: where you last saw someone or a pet (\"where did you last see the cat?\"), where you've flown, when you were last in a room, what flights found. Plain facts from the records, with the picture of the sighting when there is one. Doesn't move.",
    input_schema: { type: "object", properties: { question: { type: "string" } }, required: ["question"], additionalProperties: false },
  },
  {
    name: "what_changed",
    description: "What flights noticed is different from the 3D scan (a new obstacle, a door open or closed, something moved or gone), whether each is confirmed, dismissed or not confirmed yet, and what Claude's check of it said. Doesn't move.",
    input_schema: {
      type: "object",
      properties: { since: { type: "string", description: "Optional: \"yesterday\", \"today\", \"last flight\", \"2 hours\"; leave out for everything" } },
      additionalProperties: false,
    },
  },
  {
    name: "survey_house",
    description: "Look at the house's 3D scan room by room (pictures rendered from the scan, sent to Claude) to suggest room names, landmarks and hazards for the drone (fans, hanging lamps, plants, cables, glass). Asks the pilot to approve the cost first; nothing goes on the map until they accept the suggestions in Settings → House. Only when the pilot asks for it.",
    input_schema: NO_INPUT,
  },
  {
    name: "notify",
    description: "Alert the pilot: spoken, a notification on the Mac and, if set up, a push to their phone, with the current camera picture. Only for things that matter (someone home who shouldn't be, a pet in trouble, a fall).",
    input_schema: {
      type: "object",
      properties: { text: { type: "string" }, urgency: { type: "string", enum: ["low", "default", "high"] } },
      required: ["text"],
      additionalProperties: false,
    },
  },
  ...TOOL_DEFS.filter((t) => !["turn", "move", "fly_toward"].includes(t.name)),
];

// The house in a few deterministic lines for the cached system prompt: rooms, doorways, the home pad, landmarks and
// keep-outs. Same house in, same bytes out (no coordinates, no times), so the prompt cache keeps hitting.
export function houseSummary(house, map = null) {
  const rooms = house.rooms ?? [], name = (id) => rooms.find((r) => r.id === id)?.name ?? "outside";
  // Floor area: the map's cells of the room when there is a map (outlines can overlap), else the outline's.
  const area = (r) => {
    const i = map?.rooms.findIndex((q) => q.id === r.id) ?? -1;
    if (i >= 0) return map.room.reduce((n, v) => n + (v === i ? 1 : 0), 0) * map.o.cell ** 2;
    const o = r.outline;
    return Math.abs(o.reduce((a, [x, y], k) => a + x * o[(k + 1) % o.length][1] - o[(k + 1) % o.length][0] * y, 0) / 2);
  };
  const low = Math.min(...rooms.map((r) => r.floorZ ?? 0));
  const lines = [`House: ${house.name ?? house.id}.`];
  lines.push(`Rooms: ${rooms.map((r) => `${r.name} (${Math.round(area(r))} m²${(r.floorZ ?? 0) - low > 0.02 ? `, floor ${(r.floorZ - low).toFixed(2)} m higher` : ""})`).join("; ")}.`);
  const pairs = new Map();
  for (const d of house.doors ?? []) {
    if (!d.passable || !d.rooms?.[1] || d.rooms[0] === d.rooms[1]) continue;
    const key = [name(d.rooms[0]), name(d.rooms[1])].sort().join(" and ");
    pairs.set(key, [...(pairs.get(key) ?? []), `${(d.width ?? Math.hypot(d.b[0] - d.a[0], d.b[1] - d.a[1])).toFixed(1)} m`]);
  }
  if (pairs.size) lines.push(`Doorways: ${[...pairs].map(([k, w]) => `${k} (${w.join(", ")} wide)`).join("; ")}.`);
  if (house.home) lines.push(`Home pad: in ${theRoom(map?.roomAt(house.home.x, house.home.y)?.name ?? "house")}${house.home.source === "user" ? ", set by the pilot" : ""}.`);
  const byRoom = new Map(rooms.map((r) => [r.id, { counts: new Map(), named: [] }]));
  for (const l of house.landmarks ?? []) {
    const g = byRoom.get(l.room) ?? byRoom.get(map?.roomAt(l.x, l.y)?.id);
    if (!g) continue;
    const n = l.name.length > 40 ? `${l.name.slice(0, 38)}…` : l.name;
    if (l.source === "roomplan") g.counts.set(n.toLowerCase(), (g.counts.get(n.toLowerCase()) ?? 0) + 1);
    else if (!g.named.includes(n)) g.named.push(l.source === "user" ? `${n} (saved)` : n);
  }
  const things = rooms.map((r) => {
    const g = byRoom.get(r.id), objs = [...g.counts].map(([n, c]) => (c > 1 ? `${c} ${n}s` : n));
    const all = [...g.named.filter((n) => n.endsWith("(saved)")), ...objs, ...g.named.filter((n) => !n.endsWith("(saved)"))].slice(0, 14);
    return all.length ? `${r.name}: ${all.join(", ")}` : null;
  }).filter(Boolean);
  if (things.length) lines.push(`Landmarks: ${things.join(". ")}.`);
  const kos = (map?.keepouts ?? house.keepouts ?? []).map((k) => {
    const [x, y] = k.polygon ? k.polygon.reduce(([a, b], p) => [a + p[0] / k.polygon.length, b + p[1] / k.polygon.length], [0, 0]) : [k.x, k.y];
    const r = map?.roomAt(x, y)?.name ?? rooms.find((q) => q.id === k.room)?.name;
    return `${k.kind === "fan" ? "ceiling fan" : k.kind}${r ? ` (${r})` : ""}`;
  });
  if (kos.length) lines.push(`Keep-out zones the planner avoids: ${[...new Set(kos)].join(", ")}.`);
  return lines.join("\n");
}

const SIZES = {
  cat: { close: 0.3, medium: 0.22, far: 0.15 },
  dog: { close: 0.36, medium: 0.26, far: 0.18 },
  person: { close: 0.7, medium: 0.5, far: 0.35 },
  default: { close: 0.5, medium: 0.35, far: 0.22 },
};

export class ToolBox {
  constructor({ ctl, perception, settings, speak, alerts = null }) {
    this.ctl = ctl;
    this.perception = perception;
    this.settings = settings;
    this.speak = speak;
    this.alerts = alerts;
    this.lastImageHeading = null;
    this.house = this.map = this.missions = this.localizer = this.saveHouse = this.memory = this.vox = this.splat = null;
    this.ai = null; // { claude, confirmSurvey(estimate, { signal }) -> bool, reviewSurvey(result), twin() -> Twin, releaseTwin?(twin), onProgress? } (setAI)
    this.route = null; // the running mission's path, for where_am_i
    this.plan = null; // the running mission's stops still ahead ({ stops, home, seconds, t }: its "plan" event)
    this.unwatch = null;
  }

  // A house map makes missions available (and the raw motion tools unavailable). null: back to plain tools.
  // save(house): persist user edits (home pad, landmarks). memory: the house's HouseMemory (memory/memory.js); vox: its
  // VoxelMap; splat: nav/splatloc.js's SplatLocalizer (where_am_i matches the camera view to the 3D scan with it).
  setHouse({ house = null, map = null, missions = null, localizer = null, save = null, memory = null, vox = null, splat = null } = {}) {
    const runner = house && map ? missions : null;
    if (runner !== this.missions) this.watch(runner);
    Object.assign(this, { house, map, missions: runner, localizer, saveHouse: save, memory: house ? memory : null, vox: house ? vox : null, splat: house ? splat : null });
  }

  // Claude's eyes for the survey: claude (ai/claude.js) and the UI's hooks. twin() is asked for only once the pilot said
  // yes, and releaseTwin(twin) always follows (declined, stopped, failed or done): dispose one loaded just for this.
  setAI(ai = null) {
    this.ai = ai;
  }

  // Where the running mission is going (its "path" event: this leg; "plan": the stops after it), for where_am_i.
  watch(missions) {
    this.unwatch?.();
    this.route = this.plan = null;
    if (!missions?.on) return (this.unwatch = null);
    const offs = [
      missions.on("path", ({ path }) => (this.route = { path, text: this.route?.text ?? null })),
      missions.on("status", ({ phase, text }) => phase !== "done" && (this.route = { path: this.route?.path ?? null, text })),
      missions.on("plan", (p) => (this.plan = p?.stops ? { ...p, t: Date.now() } : null)),
      missions.on("done", () => (this.route = this.plan = null)),
    ];
    this.unwatch = () => offs.forEach((f) => f?.());
  }

  // "Going: Flying to the Kitchen; 4.2 m to go (about 14 s), through Room 2 into the Kitchen, then Room 3 and Room 4,
  // then home; about 70 s in all, about 120 s of battery left."
  goingText(P = this.localizer?.pose()) {
    if (!this.route || !this.missions?.busy) return null;
    const { path, text } = this.route, head = text ? `Going: ${text.replace(/\s*\([\d.]+ m\)\.?$/, "").replace(/\.$/, "")}` : "Going";
    if (!path?.length || !P || P.status === "lost") return `${head}${this.planText()}.`;
    let k = 0, bd = Infinity, at = path[0];
    for (let i = 0; i < path.length - 1; i++) {
      const [a, b] = [path[i], path[i + 1]], ax = b[0] - a[0], ay = b[1] - a[1], L2 = ax * ax + ay * ay || 1;
      const t = Math.max(0, Math.min(1, ((P.x - a[0]) * ax + (P.y - a[1]) * ay) / L2)), q = [a[0] + t * ax, a[1] + t * ay], d = Math.hypot(q[0] - P.x, q[1] - P.y);
      if (d < bd) [bd, k, at] = [d, i, q];
    }
    const rest = [at, ...path.slice(k + 1)], names = [];
    let left = 0;
    for (let i = 0; i < rest.length; i++) {
      if (i) left += Math.hypot(rest[i][0] - rest[i - 1][0], rest[i][1] - rest[i - 1][1]);
      const r = this.map?.roomAt(rest[i][0], rest[i][1])?.name;
      if (r && r !== names.at(-1)) names.push(r);
    }
    const here = this.map?.roomAt(P.x, P.y)?.name, via = names.filter((n, i) => !(i === 0 && n === here));
    const rooms = via.length > 1 ? `, through ${via.slice(0, -1).map(theRoom).join(", ")} into ${theRoom(via.at(-1))}` : via.length ? `, into ${theRoom(via[0])}` : "";
    return `${head}; ${left.toFixed(1)} m to go (about ${Math.round(left / MISSION.cruise)} s)${rooms}${this.planText(left / MISSION.cruise)}.`;
  }

  // ", then Room 3 and Room 4, then home; about 70 s in all, about 120 s of battery left" (the mission's "plan").
  planText(leg = 0) {
    const p = this.plan, b = this.missions?.battery?.()?.secondsLeft;
    if (!p) return Number.isFinite(b) ? `; about ${Math.round(b)} s of battery left` : "";
    const name = (s) => theRoom(s.name ?? this.map?.rooms?.find((r) => r.id === s.room)?.name ?? s.room ?? "a spot on the map");
    const next = p.stops.slice(1).map(name), then = [next.length ? andList(next) : null, p.home ? "home" : null].filter(Boolean);
    const total = Number.isFinite(p.seconds) ? Math.max(leg, p.seconds - (Date.now() - p.t) / 1000) : null;
    return `${then.length ? `, then ${then.join(", then ")}` : ""}${total != null ? `; about ${Math.round(total)} s in all` : ""}${Number.isFinite(b) ? `${total != null ? "," : ";"} about ${Math.round(b)} s of battery left` : ""}`;
  }

  // For the agent's per-command context (messages, never the cached prefix): this flight so far and open changes.
  memoryContext() {
    if (!this.memory) return null;
    const m = this.memory, bits = m.flight ? [m.flightSoFar()] : [], { open, stale } = m.openChanges?.() ?? { open: m.changes.filter((c) => c.status === "suspected").length, stale: 0 };
    if (open) bits.push(`${open} possible change${open > 1 ? "s" : ""} from the 3D scan not confirmed yet (what_changed lists them)${stale ? `; ${stale} not seen again for a minute, so not blocking the way` : ""}.`);
    const ins = m.flight && m.inspection?.(), seen = ins?.measured && m.inspectionText(ins);
    if (seen) bits.push(`So far this flight: ${seen.replace(/^The camera saw/, "the camera has seen")}`);
    return bits.length ? `Memory: ${bits.join(" ")}` : null;
  }

  // The tool list for Claude (stable objects, so the cached prefix stays the same while the house does).
  defs() {
    return this.missions ? HOUSE_TOOL_DEFS : TOOL_DEFS;
  }

  // Returns { text, image?, isError? }. from "claude": only the tools defs() offers (no hand flying with a map), whatever
  // the model asks for; the pilot's own offline commands may use them all.
  async call(name, input = {}, signal, { from = "pilot" } = {}) {
    try {
      const fn = this[`tool_${name}`];
      if (!fn) return { text: `Unknown tool "${name}".`, isError: true };
      if (from === "claude" && !this.defs().some((d) => d.name === name))
        return { text: `"${name}" isn't available${this.missions ? " while the house map is active: use go_to, look_in or search_for" : " right now"}.`, isError: true };
      return await fn.call(this, input || {}, signal);
    } catch (e) {
      if (e?.name === "AbortError") return { text: "Stopped by the pilot.", isError: true };
      console.error(e);
      return { text: `Tool failed: ${e.message}`, isError: true };
    }
  }

  // ---- helpers ----

  describe(dets = this.perception.latest.detections) {
    if (!dets.length) return "Detector sees nothing it recognizes.";
    const items = dets
      .filter((d) => d.score >= 0.4)
      .sort((a, b) => b.box.w * b.box.h - a.box.w * a.box.h)
      .slice(0, 8)
      .map((d) => {
        const bearing = this.ctl.bearingOf(d.box.x + d.box.w / 2) / DEG;
        const where = bearing < -40 ? "far left" : bearing < -12 ? "left" : bearing <= 12 ? "ahead" : bearing <= 40 ? "right" : "far right";
        const size = d.box.h > 0.45 ? "very close" : d.box.h > 0.25 ? "close" : d.box.h > 0.12 ? "mid-distance" : "far";
        return `${d.label} (${where}, ${size}, ${Math.round(d.score * 100)}%)`;
      });
    return items.length ? `Detector sees: ${items.join("; ")}.` : "Detector sees nothing it's confident about.";
  }

  state() {
    const t = this.ctl.tel;
    const bits = [];
    if (t?.vbat) bits.push(`battery ${t.vbat.toFixed(1)} V`);
    bits.push(this.ctl.isFlying() ? "flying" : "on the ground");
    return bits.join(", ");
  }

  // Text + the current camera image. Waits out the video delay so the image shows the result.
  async view(text) {
    if (this.ctl.videoDelay) await sleepMs(this.ctl.videoDelay + 60);
    const image = this.perception.snapshot();
    if (image) this.lastImageHeading = this.ctl.imageHeading();
    return { text: `${text}\n${this.describe()} (${this.state()})`, image };
  }

  ready({ needFlying = true } = {}) {
    const why = this.ctl.blocker();
    if (why) return why;
    if (needFlying && !this.ctl.isFlying()) return "Not flying yet. Call take_off first.";
    return null;
  }

  async fly(behavior, signal) {
    const res = await this.ctl.run(behavior, signal);
    if (signal?.aborted) return { text: "Stopped by the pilot.", isError: true };
    return res;
  }

  // ---- tools ----

  async tool_take_off(_, signal) {
    const why = this.ready({ needFlying: false });
    if (why) return { text: why, isError: true };
    if (this.ctl.isFlying()) return this.view("Already flying.");
    if (this.ctl.autonomy === "full") {
      const r = await this.fly(new TakeOff(), signal);
      return r.isError ? r : this.view(r.text);
    }
    this.ctl.askPilot("takeoff", "Please take off and hover about a meter up. I'll steer once we're flying.");
    const r = await this.fly(new WaitFor("Waiting for the pilot to take off", (c) => c.isFlying(), 25, "The pilot took off; we're flying.", "The pilot didn't take off within 25 s."), signal);
    if (r.ok) await sleepMs(1200); // let the pilot settle into a hover
    return r.isError ? r : r.ok ? this.view(r.text) : { text: r.text, isError: true };
  }

  async tool_land(_, signal) {
    if (!this.ctl.isFlying()) return { text: "Already on the ground." };
    if (this.ctl.autonomy === "full" && !this.ctl.blocker()) {
      const r = await this.fly(new Land(), signal);
      return r.isError ? r : { text: r.text, isError: !r.ok };
    }
    this.ctl.askPilot("land", "Please land now.");
    const r = await this.fly(new WaitFor("Waiting for the pilot to land", (c) => !c.isFlying(), 25, "Landed.", "Still flying after 25 s."), signal);
    return { text: r.text, isError: !r.ok };
  }

  async tool_hover({ seconds = 3 }, signal) {
    const why = this.ready();
    if (why) return { text: why, isError: true };
    const r = await this.fly(new Hold(clamp(Number(seconds) || 3, 1, 30)), signal);
    return r.isError ? r : this.view(r.text);
  }

  async tool_turn({ degrees }, signal) {
    const why = this.ready();
    if (why) return { text: why, isError: true };
    const deg = clamp(Number(degrees) || 0, -180, 180);
    if (Math.abs(deg) < 2) return this.view("Already facing that way.");
    const r = await this.fly(new Turn(deg), signal);
    return r.isError ? r : this.view(r.text);
  }

  async tool_move({ direction, meters }, signal) {
    const why = this.ready();
    if (why) return { text: why, isError: true };
    const dir = String(direction || "").toLowerCase();
    if (!["forward", "back", "left", "right", "up", "down"].includes(dir)) return { text: `Unknown direction "${direction}".`, isError: true };
    const m = clamp(Number(meters) || 1, 0.2, 4);
    if ((dir === "up" || dir === "down") && this.ctl.autonomy !== "full") {
      this.ctl.askPilot(dir === "up" ? "higher" : "lower", `Please take us ${dir === "up" ? "higher" : "lower"}, about ${m.toFixed(1)} meters.`);
      await sleepMs(2500, signal);
      return this.view(`Asked the pilot to go ${dir}.`);
    }
    const r = await this.fly(new Move(dir, m, this.settings.get("speed")), signal);
    return r.isError ? r : this.view(r.text);
  }

  async tool_look_around(_, signal) {
    const why = this.ready();
    if (why) return { text: why, isError: true };
    const r = await this.fly(new LookAround(8), signal);
    if (r.isError) return r;
    const frames = r.data?.frames || [];
    const lines = frames.map((f) => {
      const d = f.detections.filter((x) => x.score >= 0.4).map((x) => x.label);
      return `${headingName(f.angle)}: ${d.length ? [...new Set(d)].join(", ") : "nothing detected"}`;
    });
    const image = await contactSheet(frames);
    return { text: `${r.text}\n${lines.join("\n")}\n(${this.state()})`, image };
  }

  async tool_fly_toward({ x, y, meters }, signal) {
    const why = this.ready();
    if (why) return { text: why, isError: true };
    const px = clamp(Number(x), 0, 1);
    const m = clamp(Number(meters) || 1, 0.3, 4);
    if (!Number.isFinite(px)) return { text: "x must be a number between 0 and 1.", isError: true };
    // The point is relative to the heading the image was taken at.
    const base = this.lastImageHeading ?? this.ctl.est.heading;
    const deg = (base + this.ctl.bearingOf(px) - this.ctl.est.heading) / DEG;
    if (Math.abs(deg) > 3) {
      const t = await this.fly(new Turn(clamp(deg, -180, 180)), signal);
      if (t.isError) return t;
    }
    const r = await this.fly(new Move("forward", m, this.settings.get("speed")), signal);
    return r.isError ? r : this.view(`Turned ${Math.round(deg)}° toward the point. ${r.text}`);
  }

  async tool_find({ object }, signal) {
    const why = this.ready();
    if (why) return { text: why, isError: true };
    const label = toDetectorLabel(object);
    if (!label) return { text: `The on-board detector can't recognize "${object}". Use look_around and fly_toward instead.`, isError: true };
    const r = await this.fly(new Search(label), signal);
    return r.isError ? r : { ...(await this.view(r.text)), isError: !r.ok };
  }

  async tool_approach({ object, distance = "medium" }, signal) {
    return this.servo(object, distance, false, 45, signal);
  }

  async tool_follow({ object, seconds = 30, distance = "medium" }, signal) {
    return this.servo(object, distance, true, clamp(Number(seconds) || 30, 5, 180), signal);
  }

  async servo(object, distance, follow, duration, signal) {
    const why = this.ready();
    if (why) return { text: why, isError: true };
    const label = toDetectorLabel(object);
    if (!label) return { text: `The on-board detector can't recognize "${object}". Use look_around and fly_toward instead.`, isError: true };
    const seen = this.perception.find(label, 0.4)[0];
    let trackId = seen?.trackId; // lock on to this one (the detector's tracks keep their id)
    if (!seen) {
      const s = await this.fly(new Search(label), signal);
      if (s.isError) return s;
      if (!s.ok) return { ...(await this.view(s.text)), isError: true };
      trackId = s.data?.trackId;
    }
    const sizes = SIZES[label] || SIZES.default;
    const size = sizes[distance] || sizes.medium;
    const r = await this.fly(new Approach(label, { size, follow, duration, maxSpeed: Math.min(0.6, this.settings.get("speed") + 0.1), trackId }), signal);
    return r.isError ? r : { ...(await this.view(r.text)), isError: !r.ok };
  }

  async tool_snapshot() {
    return this.view("Current view.");
  }

  async tool_say({ text }) {
    if (text) this.speak(String(text).slice(0, 300));
    return { text: "Said it." };
  }

  // ---- missions on the house map ----

  async mission(m, signal, { image = "view" } = {}) {
    if (!this.missions) return { text: "No house map is loaded, so this isn't available.", isError: true };
    if (this.ctl.autonomy === "observer") return { text: "Autonomy is set to Observer, so I only watch and talk.", isError: true };
    const stop = () => this.missions.stop("the pilot stopped the mission");
    signal?.addEventListener("abort", stop, { once: true });
    try {
      const r = await this.missions.run(m);
      if (signal?.aborted) return { text: "Stopped by the pilot.", isError: true };
      const found = r.findings.map((f) => `- ${f.label} in ${theRoom(f.roomName)}${f.near ? `, near the ${f.near}` : ""} (${Math.round(f.score * 100)}%)`);
      const text = [r.summary, ...(found.length ? ["Found:", ...found] : []), `(${this.state()}, mission took ${r.seconds} s)`].join("\n");
      if (image === "sheet" && r.frames?.length) return { text, image: await contactSheet(r.frames.slice(-8).map((f, i) => ({ ...f, angle: this.viewName(f, i) }))), isError: !r.ok };
      const shot = r.findings.find((f) => f.snapshot)?.snapshot;
      if (shot) return { text, image: shot, isError: !r.ok };
      return r.ok ? this.view(text) : { text, isError: true };
    } finally {
      signal?.removeEventListener("abort", stop);
    }
  }

  // "2: toward the Living room" (the room a ray 2.5 m along the view lands in).
  viewName(f, i) {
    const r = this.map?.roomAt(f.x + 2.5 * Math.cos(f.yaw), f.y + 2.5 * Math.sin(f.yaw));
    return `${i + 1}: toward ${r ? theRoom(r.name) : "a wall"}`;
  }

  async tool_where_am_i(_, signal) {
    if (!this.missions) return { text: "No house map is loaded.", isError: true };
    return { text: [await this.visionText(signal), this.missions.whereAmI(), this.memory?.flightSoFar(), this.goingText()].filter(Boolean).join(" ") };
  }

  // With vision localization on and the position unknown, unsure or in conflict: the camera view matched to the 3D scan
  // now (an answer within about 2 s; never a database build), and how the fixes have been going.
  async visionText(signal) {
    const s = this.splat, L = this.localizer, P = L?.pose();
    if (!s?.enabled || !P) return null;
    const q = L.fixQuality?.(), quality = q && q.used ? ` Vision fixes: ${q.rate.toFixed(1)}/s, the last ${Number.isFinite(q.age) ? `${(q.age / 1000).toFixed(1)} s ago` : "a while ago"}.` : "";
    if (P.status === "ok" && !P.conflict) return quality.trim() || null;
    const r = await s.locateNow({ signal }).catch((e) => ({ ok: false, reason: e.message }));
    const said = r.disagrees ? "The camera view disagrees with my position."
      : r.ok && r.advisory ? "The camera view matches the 3D scan here (not used for flying until the pad check passes)."
      : r.ok && r.checked ? "The camera view agrees with my position."
      : r.ok ? "I matched the camera view to the 3D scan."
      : `I couldn't match the camera view to the 3D scan (${r.reason ?? "no match"}).`;
    return `${said}${quality}`;
  }

  // The sighting's stored picture (a person or pet in the house) goes with the answer only once Claude's picture checks
  // are allowed (aiVision "on"); under "ask" the consent banner is asked for (claude.permit, never a prompt mid-flight).
  async tool_recall({ question }) {
    if (!this.memory) return { text: "There's no flight memory for this house yet.", isError: true };
    const a = this.memory.answer(String(question ?? "").slice(0, 300)), shot = a.sighting && (await (this.memory.picture?.(a.sighting) ?? a.sighting.snapshot));
    if (!shot) return { text: a.text };
    const vision = this.settings.get("aiVision");
    if (vision === "off") return { text: `${a.text} (Its picture isn't sent: Claude's picture checks are off in Settings.)` };
    if (vision !== "on" && !(await this.ai?.claude?.permit?.("recall", { ask: false }))?.ok) return { text: `${a.text} (Its picture isn't sent until you allow Claude's picture checks.)` };
    const image = await toJpegBase64(shot).catch(() => null);
    return image ? { text: a.text, image } : { text: a.text };
  }

  async tool_what_changed({ since } = {}) {
    if (!this.memory) return { text: "There's no flight memory for this house yet.", isError: true };
    return { text: this.memory.whatChanged(since ? String(since).slice(0, 60) : null) };
  }

  // On the ground only: estimate -> the pilot's OK (the UI's dialog, which says pictures of the house go to Claude; a stop
  // closes it) -> the twin -> survey -> the UI's review -> the twin released. Claude gets the summary; nothing is applied.
  async tool_survey_house(_, signal) {
    const ai = this.ai;
    if (!this.house || !this.map) return { text: "No house map is loaded.", isError: true };
    if (this.ctl.isFlying()) return { text: "Land first: the survey renders the 3D scan room by room and takes a few minutes, longer than a battery lasts.", isError: true };
    if (!ai?.claude?.configured) return { text: "The survey needs a Claude API key (Settings → Brain).", isError: true };
    if (this.settings.get("aiVision") === "off") return { text: "Claude's picture checks are off in Settings, so I can't survey the house (voice commands still send the current camera view). The pilot can turn them on in Settings.", isError: true };
    const estimate = estimateSurvey({ house: this.house, map: this.map, settings: this.settings, model: ai.claude.model });
    if (!(await ai.confirmSurvey?.(estimate, { signal }))) return { text: signal?.aborted ? "Stopped by the pilot." : `The pilot didn't approve the survey (estimated ${estimate.text}).`, isError: !!signal?.aborted };
    let twin = null;
    try {
      twin = await ai.twin?.();
      if (signal?.aborted) return { text: "Stopped by the pilot.", isError: true };
      if (!twin) return { text: "The house's 3D scan isn't loaded, so there's nothing to look at. Use the capture again in Settings → House.", isError: true };
      const res = await surveyHouse({ house: this.house, map: this.map, vox: this.vox, twin, settings: this.settings, claude: ai.claude, estimate, approved: true, signal, onProgress: ai.onProgress });
      ai.reviewSurvey?.(res);
      return { text: surveySummary(res, this.house), isError: !res.ok };
    } catch (e) {
      return { text: `The survey failed: ${e?.message ?? e}.`, isError: true };
    } finally {
      if (twin) ai.releaseTwin?.(twin);
    }
  }

  async tool_go_to({ place }, signal) {
    return this.mission({ kind: "goTo", target: place }, signal);
  }

  async tool_look_in({ room }, signal) {
    return this.mission({ kind: "lookIn", room }, signal, { image: "sheet" });
  }

  async tool_search_for({ target, rooms }, signal) {
    return this.mission({ kind: "searchFor", target, rooms }, signal, { image: toDetectorLabel(target) || /^(people|person|pets?)$/i.test(String(target).trim()) ? "view" : "sheet" });
  }

  async tool_patrol({ rooms } = {}, signal) {
    return this.mission({ kind: "patrol", rooms }, signal);
  }

  async tool_return_home(_, signal) {
    return this.mission({ kind: "returnHome" }, signal);
  }

  async tool_set_home() {
    const p = this.localizer?.pose();
    if (!this.house || !p || p.status === "lost") return { text: "I don't know where I am, so I can't set the home pad here.", isError: true };
    this.house.home = { x: +p.x.toFixed(3), y: +p.y.toFixed(3), yaw: +p.yaw.toFixed(3), source: "user" };
    await this.saveHouse?.(this.house);
    return { text: `Home pad set here, in ${theRoom(this.map.roomAt(p.x, p.y)?.name ?? "house")}.` };
  }

  async tool_mark_landmark({ name }) {
    const n = String(name || "").trim().slice(0, 40), p = this.localizer?.pose();
    if (!n) return { text: "The landmark needs a name.", isError: true };
    if (!this.house || !p || p.status === "lost") return { text: "I don't know where I am, so I can't mark this spot.", isError: true };
    const room = this.map.roomAt(p.x, p.y)?.id ?? null;
    this.house.landmarks = [...(this.house.landmarks ?? []).filter((l) => !(l.source === "user" && l.name.toLowerCase() === n.toLowerCase())),
      { name: n, room, x: +p.x.toFixed(3), y: +p.y.toFixed(3), z: +p.z.toFixed(2), source: "user" }];
    await this.saveHouse?.(this.house);
    return { text: `Saved "${n}" here, in ${theRoom(this.map.roomAt(p.x, p.y)?.name ?? "house")}.` };
  }

  async tool_notify({ text, urgency = "default" }) {
    const t = String(text || "").trim().slice(0, 300);
    if (!t) return { text: "Nothing to send.", isError: true };
    if (!this.alerts) {
      this.speak(t);
      return { text: "Said it (alerts aren't set up)." };
    }
    const p = this.localizer?.pose();
    const r = await this.alerts.notify({ text: t, urgency, image: this.perception.snapshot({ maxWidth: 640 }), room: p?.room ?? null, pose: p?.status !== "lost" ? p : null });
    const how = [r.spoke && "spoken", r.notified && "Mac notification", r.pushed && "phone push"].filter(Boolean);
    return { text: `Alert sent${how.length ? ` (${how.join(", ")})` : ""}.` };
  }

  async tool_remember({ note }) {
    const n = String(note || "").trim().slice(0, 300);
    if (!n) return { text: "Empty note.", isError: true };
    const notes = (this.settings.get("homeNotes") || "").trim();
    this.settings.set("homeNotes", `${notes ? notes + "\n" : ""}- ${n}`.slice(-3000));
    return { text: "Saved to home notes." };
  }
}

const andList = (xs) => (xs.length > 1 ? `${xs.slice(0, -1).join(", ")} and ${xs.at(-1)}` : xs[0] ?? "");

function headingName(a) {
  if (a === 0) return "0° (straight ahead)";
  if (a === 180) return "180° (behind)";
  return a < 180 ? `+${a}° (${a}° right)` : `+${a}° (${360 - a}° left)`;
}

async function contactSheet(frames) {
  const valid = frames.filter((f) => f.image);
  if (!valid.length) return null;
  const images = await Promise.all(valid.map(async (f) => {
    const img = new Image();
    img.src = `data:image/jpeg;base64,${f.image}`;
    await img.decode();
    return img;
  }));
  const tw = 320;
  const th = Math.round((tw * images[0].naturalHeight) / images[0].naturalWidth) || 240; // the frames' own shape (4:3 or 16:9)
  const cols = 4;
  const rows = Math.ceil(valid.length / cols);
  const c = document.createElement("canvas");
  c.width = tw * cols;
  c.height = th * rows;
  const g = c.getContext("2d");
  g.fillStyle = "#000";
  g.fillRect(0, 0, c.width, c.height);
  for (let i = 0; i < valid.length; i++) {
    const img = images[i];
    const x = (i % cols) * tw;
    const y = Math.floor(i / cols) * th;
    g.drawImage(img, x, y, tw, th);
    g.strokeStyle = "#000";
    g.lineWidth = 3;
    g.strokeRect(x, y, tw, th);
    const label = typeof valid[i].angle === "string" ? valid[i].angle : headingName(valid[i].angle);
    g.font = "bold 16px system-ui, sans-serif";
    const w = g.measureText(label).width + 12;
    g.fillStyle = "rgba(0,0,0,0.7)";
    g.fillRect(x + 4, y + 4, w, 24);
    g.fillStyle = "#ffd84d";
    g.fillText(label, x + 10, y + 22);
  }
  return c.toDataURL("image/jpeg", 0.75).split(",")[1];
}

function sleepMs(ms, signal) {
  return new Promise((resolve) => {
    const t = setTimeout(resolve, ms);
    signal?.addEventListener("abort", () => {
      clearTimeout(t);
      resolve();
    });
  });
}
