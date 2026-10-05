// Deterministic missions on the house map (docs/HOME-DRONE.md, "Wave B contracts"): Claude (or the offline parser) picks
// one, this flies it with the planner and the map behaviors. run(m), m one of
//   { kind: "goTo", target }        a room (id or name), a landmark name, "home" or { x, y }
//   { kind: "lookIn", room }        from the doorway first, further in only if the doorway sees too little
//   { kind: "patrol", rooms? }      one scan per room, home, land
//   { kind: "searchFor", target, rooms? }   "person", "cat", "dog", "pets" or any text (then: views for Claude)
//   { kind: "checkOnPets" }         cat and dog: their favourite spots first, low
//   { kind: "returnHome" }
//   { kind: "calibrate" }           full auto: measure how the drone brakes (settings "brake", for nav/avoid.js)
// -> { ok, summary, findings: [{ label, room, x, y, score, snapshot }], frames? }. Events: "status" { phase, text },
// "path" { path }, "plan" { stops: [{ room, name, x?, y? }] (stops[0]: the one flown to now), home, seconds } (where a
// patrol, search or pet check will be, then whether it lands home, about how long all of it takes; null on the way
// home), "finding", "done". Every leg is checked against the battery (leg + 1.3 x the way home + 10 s); a
// mission that can't afford its next leg flies home instead. A blocked way (a person's no-fly zone, something new the
// live depth found, a suspected change: temporary obstacles on the map) is planned again around it; with no way around
// it waits up to `patience` s for it to clear, then says what blocks it (only something on the way: the plan without the
// temporary obstacles passes it; a place no plan reaches is refused, on the ground without taking off). A patrol skips a
// room it can't get to and says why; a patrol, search or pet check that stops early in full auto flies home. Going home
// never gives up while the battery allows (behaviors.js ReturnHome: wider berths, waits, then a clear spot nearby). A
// room counts as looked at only from scans the person detector saw (ScanAt: enough fresh frames per heading with it
// working); summaries say which rooms weren't, and why, and how long the drone flew slowly without live depth (nav/avoid.js
// eyes()). On the real drone a mission doesn't start on video that isn't the camera's picture. Sightings count once confirmed on 3
// frames; findings with the same label are one when they share a track, lie within 1 m, or look like the detector
// renumbering one person (the old track gone when the new one appeared, never both in one frame, within 2.5 m, about the
// same size), or (people) one walking on (seen again within 30 s, no faster than 1 m/s): one finding, one alert. A spot
// off every room is no room ("somewhere past the Living room"), and no high-urgency alert. Full auto doesn't take off
// while someone in the picture may be within 1.5 m (it waits up to `patience` s, then checks everything again).
// Co-pilot: the pilot holds height (and takes off and lands when asked), the mission steers. Without a
// position on the ground, a mission starts from the home pad only if the drone hasn't flown since its position was last
// set (setting "startOnPad"). With a house memory (memory/memory.js) findings become sightings, and every flight (take-off
// to landing, missions or not) is recorded with its pose trail at 2 Hz.
// Ready to fly: preflightCheck({ mode }) lists what a mission needs and what is missing (the shell's "Ready to fly?"); a
// mission refuses on the first "block" item, in the air too, before anything moves (the mission flying goes on; nothing
// flies home for a refusal), except that return home in the air needs only what flying home needs. The real drone (mode
// "real", its position never the simulator's truth) needs the 3D map, the camera's picture, vision localization set up
// and running (calibrated, the position database, the pad check, a fix under 1 s old), a machine that keeps up, the person
// detector, live depth (full auto) and room around the pad: voice moves still work without them. Time budgets use the
// speed the drone really flies (nav/avoid.js cruise()) and scans as long as they really take.
// A false change mustn't wreck a sortie: a way blocked by a suspected change first gets a closer look (the drone faces it
// for MISSION.relook s, so the change detector can see the spot as the scan has it and drop it); still there, one Claude
// called no change (at least MISSION.passConfidence sure) is passed slowly, watching with live depth, by any mission; one
// not seen again for MISSION.unseen ms of this flight comes off the map while the drone can see (back on when seen again,
// after MISSION.blindBack ms blind unless the drone is at it, and after landing). What only the live depth says blocks the
// way (a doorway it sees closed, a wedge it sees ahead) gets a look from about 1 m in front of it, on the way home too
// (closer()). A room counts as patrolled or looked into only from inside it, or seen whole from outside
// through open doorways (a doorway that looks closed is a wall): else the summary says what wasn't checked and why; a
// patrol says where it really landed.
import { Emitter, clamp } from "./util.js";
import { plan, viewpoints, roomCenter, nearestFree, patrolRoute } from "./house/planner.js";
import { TakeOff, Land, WaitFor, Hold, PathFollow, DoorTransit, ScanAt, ReturnHome, BrakeTest, locate, HOME_TRY, SCAN } from "./behaviors.js";
import { toDetectorLabel } from "./detector.js";
import { blockers, freeRun, polyDist, cruiseSpeed, staticClearance, withoutTemps, stopDistance, brakeKey, AVOID } from "./nav/avoid.js";
import { pictureProblem, pictureOf } from "./vision/depth.js";
import { coverageReport } from "./house/coverage.js";
import { SAFETY } from "./safety.js";

export const MISSION = {
  alt: 1.0, petAlt: 0.6, vmax: 0.55, doorTime: 3, scanTime: 10, reserve: 10, budget: 1.3, landTime: 8,
  cruise: AVOID.cruise, // nominal only: budgets use cruise() (the speed the drone really flies)
  narrowDoor: 0.25, // m of spare clearance below which a doorway gets the full DoorTransit
  confirm: 3, near: 1.0, window: 2000,
  patience: 20, waitStep: 2, // s waiting for a blocked way to clear, in steps
  doorTries: 3, // failed passes of one doorway (couldn't line up, blocked) before a leg gives up on it
  renumber: { gate: 2.5, within: 15000, size: 0.35 }, // a new track id for the same one: m apart, ms after, size ratio
  walk: 1.0, walkWithin: 30, // a person seen again elsewhere within this many s, no faster than this (m/s): the same one
  covered: 0.75, dense: 0.6, gain: 0.08, // a room seen past furniture (seenShare "mid") below `dense` after one scan per room gets
  // the denser scans, each adding at least `gain`, until `covered`
  sample: 500, // ms between pose samples for the flight record
  relook: 2, // s facing a suspected change that blocks the way before planning again (once per change and mission)
  unseen: 60000, // ms (the memory's clock) a suspected change may go unseen in flight before it comes off the map
  blindBack: 3000, // ms blind (avoid.eyes()) before a lifted one goes back on (fix and depth-frame gaps last a second or so)
  eyesWait: 10, // s a full-auto take-off on the real drone hovers for live depth to see (avoid.eyes()) before the first leg
  passConfidence: 0.8, // how sure Claude must be that a suspected change is no change for return home to pass it
  minSeconds: 60, // s of flying the battery must have left for a mission to start (on the ground)
  fixAge: 1000, // ms: the real drone's last vision fix must be newer for a mission to start
  paceFps: 15, paceFixes: 3, // the real drone: video frames and vision fixes a second the machine must keep up
  padWarn: 0.5, padBlock: 0.35, // m clear around the take-off spot under which the checklist warns (the real drone: refuses)
  coverageWarn: 70, // % of the house known (house/coverage.js score) under which the checklist warns
  planSigma: 0.17, planMargin: 0.08, // m: the least σ plans are sized for; what they add to twice the position's σ
  brakeRuns: 3, brakeRoom: 1.6, brakeAgree: 3, brakeMin: 0.2, brakeReact: 0.8, // braking: runs (each a stop; the simulator's own
  // identical runs stop 0.13-0.21 m from 0.45 m/s, so the fit takes the most careful), m of corridor each needs, measurements
  // in a row that must agree (within 30%) before a braking harder than the simulator's is trusted, the least m/s² and the
  // longest s of reaction that make sense
};
const HOME_NEEDS = new Set(["house", "radio", "safety", "position"]); // checklist items return home in the air waits on
const PETS = ["cat", "dog"];
const PET_SPOTS = /sofa|couch|bed|chair|armchair|cushion|basket|rug|window/i;
const norm = (t) => String(t ?? "").toLowerCase().replace(/[^a-z0-9 ]/g, " ").replace(/\b(the|my|our|a|an|to|in|into)\b/g, " ").replace(/\s+/g, " ").trim();
const words = (t) => norm(t).split(" ").filter(Boolean);
const where = (p) => `(${p[0].toFixed(1)}, ${p[1].toFixed(1)})`;
// A reason as a clause inside a sentence: no final stop, no "(Events: ...)", lower case (but "I").
const clause = (t) => String(t ?? "").replace(/\.$/, "").replace(/ \(Events[^)]*\)$/, "").replace(/^(?!I\b)./, (q) => q.toLowerCase());
// "the Living room", but "Room 2" (a generic name takes no article).
export const theRoom = (name) => (/^(room|bedroom|bathroom)\s*\d/i.test(name) ? name : `the ${name}`);

class Stop extends Error {
  constructor(text, more = {}) {
    super(text);
    Object.assign(this, more);
  }
}

// Rooms, landmarks and places by what people call them.
export function findRoom(house, q) {
  const n = norm(q);
  if (!n) return null;
  const rooms = house?.rooms ?? [];
  return rooms.find((r) => r.id === q || norm(r.name) === n) ?? rooms.find((r) => n.includes(norm(r.name)) || norm(r.name).includes(n)) ?? null;
}
export function findLandmark(house, q, near = null) {
  const n = norm(q), qs = words(q).map((w) => ({ couch: "sofa", tv: "television", telly: "television" })[w] ?? w);
  if (!n) return null;
  const score = (l) => {
    const ws = words(l.name);
    if (norm(l.name) === n) return 3 + (l.source === "user" ? 1 : 0);
    const hit = qs.filter((w) => ws.includes(w)).length;
    return hit ? hit / Math.max(qs.length, 1) + hit / ws.length / 2 + (l.source === "user" ? 0.5 : 0) : 0;
  };
  const best = (house?.landmarks ?? []).map((l) => ({ l, s: score(l) })).filter((c) => c.s >= 0.6);
  if (!best.length) return null;
  const top = Math.max(...best.map((c) => c.s));
  const ties = best.filter((c) => c.s === top).map((c) => c.l);
  return near ? ties.reduce((a, b) => (Math.hypot(a.x - near.x, a.y - near.y) <= Math.hypot(b.x - near.x, b.y - near.y) ? a : b)) : ties[0];
}

// Share of a room a 360° look from p would see (standing people; walls and tall furniture hide, 6 m range).
export const coverageFrom = (map, roomId, p, o) => seenShare(map, roomId, [p], o);

// Share of a room's floor seen from any of the spots [[x, y]]: hides "tall" (standing people: walls and tall furniture
// hide) or "mid" (seated ones: anything at 0.6 m hides too), 6 m range; doorways in `shut` (map doors that look closed)
// hide as walls do.
export function seenShare(map, roomId, spots, { range = 6, hides = "tall", shut = [] } = {}) {
  const ri = map.rooms.findIndex((r) => r.id === roomId), occ = map.occ[map.bandOf(hides === "mid" ? 0.6 : 1.4)], step = 4;
  let n = 0, seen = 0;
  for (let r = 0; r < map.H; r += step)
    for (let c = 0; c < map.W; c += step) {
      const k = r * map.W + c;
      if (map.room[k] !== ri || map.wall[k] || occ[k]) continue;
      n++;
      const [x, y] = map.center(c, r);
      seen += spots.some((p) => {
        const L = Math.hypot(x - p[0], y - p[1]);
        if (L > range || shut.some((d) => crosses(p, [x, y], d.a, d.b))) return false;
        for (let i = 1, m = Math.ceil(L / map.o.cell); i < m; i++) {
          const q = map.idx(p[0] + ((x - p[0]) * i) / m, p[1] + ((y - p[1]) * i) / m);
          if (q < 0 || map.room[q] < 0 || map.wall[q] || occ[q]) return false;
        }
        return true;
      }) ? 1 : 0;
    }
  return n ? seen / n : 0;
}

// Do segments p-q and a-b cross?
function crosses(p, q, a, b) {
  const side = (o, u, v) => Math.sign((u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0]));
  return side(p, q, a) * side(p, q, b) < 0 && side(a, b, p) * side(a, b, q) < 0;
}

// Sightings -> findings: the same label near the same spot on `confirm` frames within `window` ms (or the same track).
class Spotter {
  constructor(o) {
    this.o = o;
    this.clusters = [];
  }
  add(s) {
    const { confirm, near, window } = this.o;
    let c = this.clusters.find((c) => c.label === s.label && s.t - c.t < window && ((s.trackId != null && s.trackId === c.trackId) || Math.hypot(c.x - s.x, c.y - s.y) < near));
    if (!c) this.clusters.push((c = { label: s.label, trackId: s.trackId, x: s.x, y: s.y, w: 0, hits: 0, score: 0, t: s.t, t0: s.t, size: 0 }));
    const w = (s.cut ? 0.2 : 1) / Math.max(0.5, s.range) ** 2; // near, whole views place it best
    [c.x, c.y, c.size] = [(c.x * c.w + s.x * w) / (c.w + w), (c.y * c.w + s.y * w) / (c.w + w), (c.size * c.w + (s.size ?? 0) * w) / (c.w + w)];
    Object.assign(c, { w: c.w + w, hits: c.hits + 1, score: Math.max(c.score, s.score), t: s.t, trackId: s.trackId ?? c.trackId, box: s.box, cut: (c.cut ?? true) && !!s.cut });
    if (c.hits >= confirm && !c.confirmed) return (c.confirmed = true), c;
    return null;
  }
}

export class MissionRunner extends Emitter {
  // depth: vision/depth.js LiveDepth (for the checklist: its model loaded); readiness: () => more checklist items (e.g. the
  // vision track's model downloads), merged by id.
  constructor({ ctl, map, house, localizer, perception, alerts = null, settings, memory = null, splat = null, depth = null, readiness = null, options = {} }) {
    super();
    Object.assign(this, { ctl, map, house, loc: localizer, perception, alerts, settings, memory, splat, depth, readiness });
    this.o = { ...MISSION, ...options };
    this.ac = null;
    this.spotting = null;
    this.lastFrame = 0;
    this.tracks = new Map(); // trackId -> { t0, t, label } (when the detector saw it first and last)
    this.together = new Set(); // "a|b": tracks seen in one frame (two of them, not one renumbered)
    this.flight = null; // { t0, notes: [] } while flying (the memory's flight record)
    this.lifted = new Map(); // suspected changes taken off the map in flight (unseen): id -> { last (seen then), shape (as the map had it) }
    this.blindAt = null; // since when the drone can't see what the scan lacks (liftStale)
    this.liftAt = 0;
    this.liftSaid = new Set(); // lifted changes already said (lifted again after the camera's view came back)
    this.unsub = ctl.on("tick", ({ now } = {}) => (this.spot(), this.record(now ?? performance.now()), this.liftStale(now ?? performance.now()), this.hookHome()));
    // the safety layer's own way home (battery low, found again) plans as missions do: their σ, kept out of its buffer where
    // that fits, facing what blocks it, a closer look at what only the live depth says blocks it, passing a change Claude
    // called no change
    this.homeHooks = () => {
      const h = this.house?.home;
      return h ? { alt: this.o.alt, vmax: this.o.vmax, sigma: () => this.planSigma(), comfy: () => this.comfySigma(), blocker: () => this.blocker([h.x, h.y])?.text,
        lookAt: () => this.blocker([h.x, h.y])?.at, passable: () => this.passableChanges([h.x, h.y]), closer: () => ((w) => (w?.live ? this.closer(w) : null))(this.blocker([h.x, h.y])) } : {};
    };
    this.hookHome();
    // the real drone's video: learn where the camera's picture is in it (black bars), for preflight
    this.unsubFrame = perception?.on?.("frame", () => this.settings?.get?.("mode") === "real" && pictureOf(perception.source));
  }

  setHouse({ house, map, memory = this.memory, splat = this.splat, depth = this.depth }) {
    if (map !== this.map) this.liftStale(0, true);
    Object.assign(this, { house, map, memory, splat, depth });
    this.hookHome();
  }
  hookHome() {
    const s = this.ctl.safety;
    if (s && s.homeOptions !== this.homeHooks) s.homeOptions = this.homeHooks;
  }

  // The real drone's vision localization (nav/splatloc.js), on the ground and in the air: on, the camera calibrated for this
  // framing, the house's position database, the pad check passed with this lens and framing, and a fix in the last
  // MISSION.fixAge ms. -> why not | "".
  visionProblem() {
    return this.preflightCheck().find((c) => c.level === "block" && c.vision)?.text ?? "";
  }

  // The "Ready to fly?" list: [{ id, ok, level: "block" | "warn" | "info", text, fix?: { label, action }, when, vision? }].
  // ok items are "info"; a mission refuses on the first "block". when: "before" (can be done before the battery goes in)
  // or "battery" (needs the drone on and the radio linked). fix.action (the shell's buttons): import-house, build-3d,
  // radio, pad, home (set the home pad on the map), crop-video, vision-on, calibrate-camera, build-db, models,
  // measure-braking, charge, ai-settings. Ids shared with the vision side's readiness() (nav/readiness.js: picture, calib,
  // posdb, webgpu, padcheck) are replaced by its richer items when it is wired. mode "real": the real drone (unless its position is the simulator's truth: a
  // rehearsal of the real caps). depth, vox: the shell's, when this runner wasn't given them.
  preflightCheck({ mode = this.settings?.get?.("mode") === "real" ? "real" : "sim", now = performance.now(), depth = this.depth } = {}) {
    const out = [], add = (id, ok, level, text, more = {}) => out.push({ id, ok: !!ok, level: ok ? "info" : level, text, when: "before", ...more });
    const real = mode === "real", realPos = real && this.loc?.source !== "truth", s = this.splat, vox = this.map?.vox;
    add("house", this.map && this.house, "block", this.house ? `House: ${this.house.name ?? "your house"}, ${this.house.rooms?.length ?? 0} rooms.` : "No house map is loaded.", { fix: { label: "Add your house", action: "import-house" } });
    if (!this.map || !this.house) return this.merged(out);
    const radio = this.ctl.blocker(), t = this.ctl.tel, switchOff = !!radio && !!this.ctl.radio && !!this.ctl.telemetryFresh && this.ctl.autonomy !== "observer" && !!t && !t.engaged && !t.failsafe;
    add("radio", !radio, switchOff ? "warn" : "block", switchOff ? "The AI switch on the radio is off: flip it on to hand me control (a mission asks for it as it starts)." : radio || "Radio connected and the AI switch on.",
      { when: "battery", ...(!switchOff && { fix: { label: "Radio help", action: "radio" } }) });
    if (this.ctl.safety?.takeover) add("safety", false, "block", `The safety layer has control: ${this.ctl.safety.takeover.reason}.`, { when: "battery" });
    // the real drone's video: running (frames under a second old), and the camera's picture found in it (perception.js
    // finds it by itself, the goggles' or a capture's; a box dragged on the video when its shape isn't the camera's)
    const video = this.perception?.frameAge < 1000;
    if (real) {
      const pic = video ? pictureProblem(this.perception) : "", goggles = this.perception?.source?.kind === "goggles", crop = /crop the video to it/.test(pic);
      add("picture", video && !pic, "block", !video ? "No video from the drone yet: goggles on and plugged in, battery in." : !pic ? "Video: the camera's picture is found."
        : /^checking/.test(pic) ? `Still finding the camera picture in the ${goggles ? "goggles' " : ""}video (a few seconds): missions wait for it.`
        : /too dark/.test(pic) ? "The video is too dark to find the camera picture in it: point the camera at a lit room for a few seconds."
        : `I won't fly a mission on this video: ${pic}${goggles ? " (the goggles' picture is found by itself; if this stays, drag a box around the picture on the video)" : " (drag a box around the picture on the video)"}.`,
        { when: "battery", ...((!video || crop) && { fix: !video ? { label: "Connect goggles", action: "goggles" } : { label: "Crop the video", action: "crop-video" } }) });
    }
    const P = this.loc.pose(), flying = this.ctl.isFlying();
    if (P.status === "lost" && (flying || this.loc.flown || !this.house.home || this.settings?.get?.("startOnPad") === false))
      add("position", false, "block", this.loc.flown && !flying ? `I don't know where I am. Put me on the home pad and press "The drone is on its home pad".` : "I don't know where I am. Put me on the home pad first.", { when: "battery", fix: { label: "The drone is on its pad", action: "pad" } });
    else add("position", true, "info", P.status === "lost" ? "I'll start from the home pad." : `Position: in ${theRoom(this.roomName(P.room))}, good to about ${Math.max(0.05, P.sigma).toFixed(2)} m.`, { when: "battery" });
    if (realPos) {
      const vision = { vision: true };
      add("vision", s?.enabled, "block", s?.enabled ? "Vision localization is on." : "Vision localization is off: missions on the real drone need the position from the camera (voice moves still work).", { ...vision, fix: { label: "Turn it on", action: "vision-on" } });
      if (s?.enabled) {
        const backend = s.features?.info?.backend;
        add("calib", s.calibOk, "block", s.calibOk ? "Camera calibrated for this video framing." : "Calibrate the camera first (Settings → House → Calibrate the camera, with the drone on its pad, or from a recorded flight).", { ...vision, fix: { label: "Calibrate the camera", action: "calibrate-camera" } });
        add("posdb", s.db, "block", s.db ? "Position database ready." : "The position database for this house isn't built yet (Settings → House).", { ...vision, fix: { label: "Build it", action: "build-db" } });
        add("webgpu", backend !== "wasm", "block", backend === "wasm" ? "Vision runs on the slow fallback without WebGPU (about 2 fixes a second): too slow to trust for missions. Use Chrome with WebGPU on." : backend ? "Vision runs on the GPU (WebGPU)." : "The vision model loads with the first video.", { ...vision, fix: backend === "wasm" ? undefined : { label: "Load the models", action: "models" } });
        const verified = s.trust?.() === "verified";
        add("padcheck", verified, "block", verified ? `Pad check passed${s.pad?.text ? `: ${s.pad.text.replace(/\.$/, "")}` : ""}.` : `${s.pad?.text ?? "The camera hasn't been checked on the pad yet."} Put the drone on its home pad and press the pad button.`, { ...vision, when: "battery", fix: { label: "The drone is on its pad", action: "pad" } });
        const q = this.loc.fixQuality?.(now), fresh = q?.visionAge < this.o.fixAge;
        if (!video) add("fix", false, "info", "Camera position fixes start once the video shows.", { ...vision, when: "battery" });
        else add("fix", fresh, "block", fresh ? `Vision fixes: ${q.rate.toFixed(1)} a second.` : `No position fix from the camera in the last second${q?.rate ? ` (${q.rate.toFixed(1)} a second lately)` : ""}: keep the camera's view on the room (not a blank wall or the floor), and check the 3D scan is loaded for vision.`, { ...vision, when: "battery" });
      }
      // a machine too busy to keep up: the video's frames and the camera's position fixes (once the video runs)
      const fps = this.perception?.fps ?? 0, rate = s?.enabled ? this.loc.fixQuality?.(now)?.rate ?? 0 : Infinity;
      if (!video) add("pace", false, "info", "Whether this computer keeps up is checked once the video runs.", { when: "battery" });
      else if (fps > 0) add("pace", fps >= this.o.paceFps && rate >= this.o.paceFixes, "block", fps >= this.o.paceFps && rate >= this.o.paceFixes ? `The computer keeps up: ${fps} frames a second${Number.isFinite(rate) ? `, ${rate.toFixed(1)} position fixes a second` : ""}.`
        : `The computer is too busy to fly safely (${fps} frames a second${Number.isFinite(rate) ? `, ${rate.toFixed(1)} position fixes a second` : ""}; I need ${this.o.paceFps} and ${this.o.paceFixes}): close other tabs and apps.`, { when: "battery" });
    }
    // room around the take-off spot (the pad, or where the drone is on the ground): take-offs drift, and a wall within reach
    // of that drift touches the props
    if (!flying) {
      const h = this.house.home, at = P.status !== "lost" ? P : h, room = at ? this.padRoom(at.x, at.y) : null, pad = !!h && !!at && Math.hypot(at.x - h.x, at.y - h.y) < 0.3;
      if (room != null) add("pad", room >= this.o.padWarn, realPos && room < this.o.padBlock ? "block" : "warn", room >= this.o.padWarn ? `Room to take off: ${room.toFixed(1)} m clear around ${pad ? "the pad" : "me"}.`
        : `${pad ? "Your pad is" : "I'm"} only ${room.toFixed(2)} m from a wall or furniture: take-offs drift and the props could touch it. ${pad ? `Move the pad at least ${this.o.padWarn} m out, then set home again with the Home tool on the map (click where the pad is now)` : `Put me on the pad, or somewhere at least ${this.o.padWarn} m from anything`}.`,
        room < this.o.padWarn && pad ? { fix: { label: "Set home again", action: "home" } } : {});
    }
    const cov = vox && this.coverage(), age = vox?.built?.at ? Math.round((Date.now() - vox.built.at) / 864e5) : null;
    add("map3d", vox, realPos ? "block" : "warn", vox ? `3D map ready${cov ? `: ${Math.round(cov.score)}% of the house known` : ""}${age > 0 ? `, built ${age} day${age > 1 ? "s" : ""} ago` : ""}.`
      : realPos ? "The 3D map isn't built: missions on the real drone need it (space the scan never saw must count as unknown, never free)." : "No 3D map yet: the simulator flies on the flat map.", { fix: { label: "Build the 3D map", action: "build-3d" } });
    if (cov && cov.score < this.o.coverageWarn) add("coverage", false, "warn", `Only ${Math.round(cov.score)}% of the house is known: the drone won't fly where the scan didn't see (Settings → House → coverage).`, { fix: { label: "See the gaps", action: "build-3d" } });
    if (this.memory?.needsRebuild) add("rebuild", false, "warn", "A door changed since the map was built: it is rebuilt after landing.");
    // the real drone waits for the person detector to run (the fallback counts); in the simulator a warning
    const per = this.perception, det = per?.detector, truthBoxes = !det || per?.source?.detections;
    add("detector", truthBoxes || det.ready, real ? "block" : "warn", truthBoxes ? "Person detector: ready." : det.ready ? `Person detector: ready${det.backend === "mediapipe" ? " (the fallback, which misses more)" : ""}.`
      : per?.detectorFailed ? "The person detector failed to load: I can't keep away from people I can't see." : `The person detector is still loading${real ? ": missions on the real drone wait for it (I can't keep away from people I can't see)" : ""}.`, { fix: { label: "Load the models", action: "models" } });
    if (realPos) {
      // live depth: on the ground, what keeps it from starting (off, no 3D twin, its model failed to load, not loaded, or
      // still loading: not a failure, but nothing to fly by yet): its state there ("not started", "stopped", the last
      // flight's) changes only in the air, so it says nothing (a full-auto take-off waits for it: awaitEyes()); in the air,
      // that state and frames coming. Full auto needs it (it sees what the scan lacks), co-pilot may fly without it, slowly
      const on = this.settings?.get?.("avoid") !== false, d = depth, av = this.ctl.safety?.avoid, br = av?.brake?.(), cloud = av?.cloud;
      const failed = /^the depth model didn't load/.test(d?.why ?? "") && d.why, loading = !!d?.estimator && !d.loaded;
      const why = !on ? "live depth is off in Settings" : !flying ? (!d?.twin ? (d ? "no 3D twin of the house is loaded" : "the live depth model isn't loaded") : d.loaded ? ""
        : loading ? "its model is still loading" : failed || "the live depth model isn't loaded")
        : !d?.estimator ? failed || "the live depth model isn't loaded" : av?.depthWhy || (!(now - (cloud?.t ?? -Infinity) < AVOID.depthAge) ? "live depth frames aren't coming" : "");
      const needs = this.full ? `Full-auto missions on the real drone need it to see what the scan lacks; co-pilot missions still work, at most ${AVOID.blind} m/s` : `The real drone flies at most ${AVOID.blind} m/s and can't see things the scan lacks`;
      add("depth", !why, this.full ? "block" : "warn", !why ? (flying ? "Live depth: running." : "Live depth: the model is loaded; it starts once I'm flying (a full-auto mission waits for it over the pad).")
        : loading && !flying ? `Live depth's model is still loading: ${this.full ? "full-auto missions on the real drone wait for it" : `until it runs the real drone flies at most ${AVOID.blind} m/s`}.`
        : `Live depth isn't running: ${why}. ${needs}.`,
        { fix: on ? { label: "Load the models", action: "models" } : { label: "Turn it on", action: "vision-on" } });
      add("braking", br?.measured, "warn", br?.measured ? "Braking measured." : `Braking not measured: at most ${AVOID.realVmax} m/s until it is (an open room, full autonomy).`, { when: "battery", fix: { label: "Measure braking", action: "measure-braking" } });
    }
    // too little battery to start (on the ground; in the air each leg is checked against the way home instead)
    const b = this.battery(), enough = b && b.secondsLeft >= this.o.minSeconds;
    if (b) add("battery", enough, flying ? "warn" : "block", enough ? `Battery ${b.vbat.toFixed(2)} V: about ${Math.round(b.secondsLeft)} s of flying (a sortie is about 3 minutes; it flies home with a margin).`
      : `The battery is too low for a mission (${b.vbat.toFixed(2)} V, about ${Math.round(b.secondsLeft)} s left): ${flying ? "I'll only fly what still leaves enough to get home" : "charge or swap it"}.`, { when: "battery", fix: { label: "Charge", action: "charge" } });
    else add("battery", false, "warn", "No battery reading yet (it comes with the radio's telemetry once the battery is in).", { when: "battery" });
    add("memory", this.memory, "warn", !this.memory ? "No flight memory: where it flew and what it saw won't be remembered." : this.memory.stored === false ? "Flight memory lasts until the page closes (this browser can't save it)." : "Flight memory is on.");
    const ai = this.settings?.get?.("aiVision") ?? "ask", budget = Number(this.settings?.get?.("aiBudget") ?? 0.5);
    add("ai", true, "info", ai === "off" ? "Claude's picture checks are off." : `${ai === "on" ? "Claude may check" : "Claude will ask before checking"} pictures of your house (sent to Anthropic's API with your key), up to $${budget.toFixed(2)} a flight.`, { fix: { label: "Change", action: "ai-settings" } });
    return this.merged(out);
  }
  // readiness() items replace the ones with their id (never a "block" of ours with something milder: a model downloaded
  // isn't a model running), or come after.
  merged(out) {
    for (const it of this.readiness?.() ?? []) {
      const i = out.findIndex((c) => c.id === it.id);
      if (i < 0) out.push(it);
      else if (out[i].level !== "block" || it.level === "block") out[i] = { ...out[i], ...it };
    }
    return out;
  }
  // Static clearance (m) around a take-off spot, 0.2-1.0 m over the floor (walls, furniture, unknown space), or null off the map.
  padRoom(x, y) {
    const f = this.map.floorAt(x, y);
    return f == null ? null : Math.min(...[0.2, 0.4, 0.6, 0.8, 1.0].map((h) => staticClearance(this.map, x, y, f + h)));
  }
  // The coverage report (house/coverage.js) for the 3D map as it is, cached until it changes.
  coverage() {
    const vox = this.map.vox;
    if (this.cov?.vox !== vox || this.cov.ver !== vox.version) {
      let report = null;
      try {
        report = coverageReport({ house: this.house, map: this.map, vox });
      } catch {}
      this.cov = { vox, ver: vox.version, report };
    }
    return this.cov.report;
  }

  // Suspected changes not seen again for MISSION.unseen ms of this flight (the memory's clock, counted from the take-off
  // at the earliest: one reported yesterday is still there at first) come off the map while flying and the drone can see
  // what the scan lacks (avoid.eyes(): live depth would find a real one again). A phantom mustn't block the sortie; a real
  // one is seen again as the drone comes near: then, and after landing, it goes back on. Blind (no new ones lifted), a
  // lifted one stays off through the gaps between fixes and depth frames (a second or so: the drone is often flying
  // through the very spot it freed); blind for MISSION.blindBack ms it goes back on, but not while the drone is within
  // AVOID.passNear of it or the leg flown now passes through it (avoid.lift: a creep near it while blind instead).
  // Checked once a second; restore: put every lifted one back now.
  liftStale(now, restore = false) {
    const mem = this.memory, map = this.map, flying = !restore && this.ctl.isFlying(), avoid = this.ctl.safety?.avoid;
    if (!mem?.changes || !map?.temps) return;
    const t = mem.now?.() ?? Date.now();
    this.takeoff = flying ? this.takeoff ?? t : null;
    if (!restore && now - this.liftAt < 1000 && flying) return;
    this.liftAt = now;
    const blind = avoid?.eyes?.(now).ok === false;
    this.blindAt = flying && blind ? this.blindAt ?? now : null;
    const long = this.blindAt != null && now - this.blindAt >= this.o.blindBack;
    for (const [id, l] of this.lifted) {
      const r = mem.changes.find((c) => c.id === id);
      if (r?.status === "suspected" && flying && (r.last ?? r.t) <= l.last && (!long || this.inReach(l.shape))) continue;
      this.lifted.delete(id);
      avoid?.unlift?.(id);
      const tp = r?.status === "suspected" && !map.temps.has(id) && mem.blocking?.(r) !== false && mem.tempOf?.(r); // not one the memory has off the map itself
      if (tp) map.addTemp({ ...tp, ...(r.passing && { kind: "person" }), id });
    }
    if (!flying || blind) return;
    for (const r of mem.changes) {
      if (r.status !== "suspected" || r.passing || !map.temps.has(r.id) || t - Math.max(r.last ?? r.t, this.takeoff) < this.o.unseen || this.lifted.has(r.id)) continue;
      const shape = map.temps.get(r.id);
      this.lifted.set(r.id, { last: r.last ?? r.t, shape });
      map.removeTemp(r.id);
      avoid?.lift?.(r.id, shape);
      if (!this.liftSaid.has(r.id)) this.status("fly", `Something the scan doesn't have in ${theRoom(this.roomName(r.room))} hasn't been seen again for a minute: no longer keeping clear of it unless I see it again.`);
      this.liftSaid.add(r.id);
    }
  }
  // Is the drone within AVOID.passNear of a temporary obstacle's shape, or does what it flies now (a path: PathFollow, or
  // return home's) pass within reach of it ahead?
  inReach(t) {
    const P = this.loc.pose(), d = (x, y) => (t.polygon ? polyDist(x, y, t.polygon) : Math.hypot(x - t.x, y - t.y) - (t.r ?? 0));
    if (d(P.x, P.y) < AVOID.passNear) return true;
    const b = this.ctl.behavior, f = b?.path ? b : b?.child?.path ? b.child : null, need = this.map.lethal(this.planSigma()) + 0.1;
    return !!f && f.path.slice(f.i ?? 0).some((q) => d(q[0], q[1]) < need);
  }

  // Memory calls (IndexedDB, maybe async) never hold up or break a flight.
  remember(op, ...args) {
    try {
      Promise.resolve(this.memory?.[op]?.(...args)).catch((e) => console.warn(`memory.${op}: ${e.message}`));
    } catch (e) {
      console.warn(`memory.${op}: ${e.message}`);
    }
  }

  // The flight record: from take-off to landing, the pose every MISSION.sample ms (a lost one is a marker), and what each
  // mission said.
  record(now) {
    if (!this.memory) return;
    const flying = this.ctl.isFlying();
    if (flying && !this.flight) {
      this.flight = { t0: now, notes: [], sampled: -Infinity };
      this.remember("startFlight", { kind: this.mission?.kind ?? "manual", mission: this.mission ?? null });
    }
    if (!this.flight) return;
    if (!flying) {
      const notes = this.flight.notes, secs = Math.round((now - this.flight.t0) / 1000);
      this.flight = null;
      return this.remember("endFlight", notes.length ? `${notes.join(" ")} (${secs} s in the air)` : `Flew for ${secs} s without a mission.`);
    }
    if (now - this.flight.sampled < this.o.sample) return;
    this.flight.sampled = now;
    const P = this.loc.pose();
    this.remember("samplePose", { x: P.x, y: P.y, z: P.z, yaw: P.yaw, sigma: P.sigma, room: P.room, status: P.status });
  }

  dispose() {
    this.stop("the house map was closed");
    this.unsub();
    this.unsubFrame?.();
  }

  get busy() {
    return !!this.ac;
  }

  stop(reason = "stopped") {
    if (!this.ac) return;
    this.ac.abort(reason);
    this.ctl.abort(reason);
  }

  status(phase, text) {
    this.emit("status", { phase, text });
  }

  async run(m) {
    // refused before anything moves: a refused command neither cancels the mission flying now nor flies home by itself
    const why = this.preflight(m);
    if (why) {
      const res = { ok: false, summary: why, findings: [], refused: true, seconds: 0 };
      this.flight?.notes.push(why);
      if (!this.busy) (this.status("done", why), this.emit("done", res));
      return res;
    }
    this.stop("a new mission started");
    const ac = (this.ac = new AbortController()), avoid = this.ctl.safety?.avoid, blind0 = avoid?.blindMs ?? 0;
    Object.assign(this, { findings: [], frames: [], t0: performance.now(), spotting: null, mission: m, unlooked: new Map(), detectNote: "" });
    let res;
    try {
      const summary = await this.dispatch(m);
      res = { ok: true, summary, findings: this.findings };
    } catch (e) {
      if (!(e instanceof Stop)) throw e;
      res = { ok: false, summary: e.message + this.sofar(), findings: this.findings };
      // a sortie meant to run on its own (patrol, search, pet check) that stops early comes home in full auto; co-pilot
      // keeps the hold for the pilot. Not after a stop, a newer command, a safety takeover or a refusal (a check that failed
      // before take-off after a wait: it says why and doesn't fly).
      const sortie = /^(patrol|searchFor|checkOnPets)$/.test(m?.kind) && this.full && this.ctl.isFlying() && !e.aborted && !e.safety && !e.refused;
      if ((e.home || sortie) && !ac.signal.aborted) {
        const h = await this.returnHome().then((t) => ` ${t}`, (e2) => ` ${e2.message}`);
        res.summary += h;
      }
    } finally {
      this.spotting = null;
      if (this.ac === ac) [this.ac, this.mission] = [null, null];
    }
    if (this.frames.length) res.frames = this.frames;
    if (this.detectNote) res.summary += ` (I looked ${this.detectNote}.)`;
    const blind = ((avoid?.blindMs ?? 0) - blind0) / 1000;
    if (blind >= 2) res.summary += ` I flew ${Math.round(blind)} s slowly without live depth (${avoid.last.blind || avoid.eyes().why || "it came back"}).`;
    res.seconds = Math.round((performance.now() - this.t0) / 1000);
    this.flight?.notes.push(res.summary);
    this.status("done", res.summary);
    this.emit("done", res);
    return res;
  }

  // A mission that stops early still says what it found (a search held back by the person it found, say).
  sofar() {
    const seen = [...new Set(this.findings.map((f) => `${f.label === "person" ? "a person" : `the ${f.label}`} ${f.where}`))];
    return seen.length ? ` Before that I found ${seen.join(", ")}.` : "";
  }

  dispatch(m) {
    switch (m?.kind) {
      case "goTo": return this.goTo(m.target);
      case "lookIn": return this.lookIn(m.room);
      case "patrol": return this.patrol(m.rooms);
      case "searchFor": return /^pets?$/i.test(String(m.target).trim()) ? this.checkOnPets(m.rooms) : this.searchFor(m.target, m.rooms, m);
      case "checkOnPets": return this.checkOnPets(m.rooms);
      case "returnHome": return this.returnHome();
      case "calibrate": return this.calibrate();
      default: throw new Stop(`Unknown mission "${m?.kind}".`);
    }
  }

  // Why mission m can't start now (preflightCheck's first "block", or the radio's warning that the AI switch is off: the
  // list only warns, the page asks for the switch as a mission starts), in the air too (after a manual take-off as well), or null.
  // Going home in the air needs only what flying home needs (HOME_NEEDS: the house, the radio, no safety takeover, a
  // position): a low battery, an old vision fix or a slow machine are reasons to go home, not to refuse it.
  // No position on the ground: the home pad, if nothing says the drone has left it (it hasn't flown since the last reset,
  // e.g. a fresh start). After flying, it may have landed anywhere (lost, or the pilot landed it).
  preflight(m = this.mission) {
    const home = m?.kind === "returnHome" && this.ctl.isFlying();
    const first = () => this.preflightCheck().find((c) => (c.level === "block" || (c.id === "radio" && !c.ok)) && (!home || HOME_NEEDS.has(c.id)))?.text ?? null, why = first();
    if (why || this.loc.pose().status !== "lost" || this.ctl.isFlying()) return why;
    const h = this.house.home;
    this.loc.reset({ x: h.x, y: h.y, yaw: h.yaw ?? Math.PI / 2 });
    this.status("start", "Starting from the home pad.");
    return first();
  }

  // ---- flying ----

  get full() {
    return this.ctl.autonomy === "full";
  }

  async fly(behavior, { soft = false } = {}) {
    const signal = this.ac?.signal;
    if (signal?.aborted) throw new Stop("Stopped.");
    const r = await this.ctl.run(behavior, signal);
    if (this.found) throw this.found;
    if (r.aborted || signal?.aborted) throw new Stop(r.safety ? r.text : signal?.reason ? `Stopped: ${signal.reason}.` : r.text, { aborted: true, safety: !!r.safety });
    if (!r.ok && !soft) throw new Stop(r.text, { blocked: !!r.blocked, person: !!r.person, held: !!r.held });
    return r;
  }

  // Full auto on the real drone (or rehearsing it): live depth runs only in the air, so after the take-off the mission hovers until the drone
  // can see what the scan lacks (avoid.eyes(): depth frames lined up with the scan, fixes to place them), at most
  // MISSION.eyesWait s; then it comes home (the pad below) rather than fly the mission blind.
  async awaitEyes() {
    const avoid = this.ctl.safety?.avoid;
    if (!avoid?.real || this.loc?.source === "truth" || !avoid.eyes || avoid.eyes().ok) return;
    this.status("wait", "Waiting for live depth to start before I fly on.");
    const r = await this.fly(new WaitFor("Waiting for live depth", (c) => !!c.safety?.avoid?.eyes?.().ok, this.o.eyesWait, "Live depth is running.", ""), { soft: true });
    if (!r.ok) throw new Stop(`Live depth didn't start once I was up (${avoid.eyes().why}), so I won't fly this mission without it.`, { home: true, eyes: true });
  }

  async ensureFlying() {
    if (this.ctl.isFlying()) return;
    if (this.full && this.ctl.safety?.personClose?.()) {
      this.status("wait", "Someone is within 1.5 m of me: I'll take off once they step back.");
      const r = await this.fly(new WaitFor("Waiting for room to take off", (c) => !c.safety?.personClose?.(), this.o.patience, "Clear.", ""), { soft: true });
      if (!r.ok) throw new Stop(`Someone stayed within 1.5 m of me, so I didn't take off (I waited ${this.o.patience} s).`);
      // the checks again after the wait: someone may have moved the drone (a fresh vision fix must agree with where it is),
      // the battery sagged, the video stalled
      const why = this.preflight();
      if (why) throw new Stop(`I didn't take off: ${clause(why)}.`, { refused: true });
    }
    const P = this.loc.pose(), above = this.full && this.ctl.safety?.avoid?.columnClear?.(P.x, P.y, this.o.alt + 0.1);
    if (above) throw new Stop(`I won't take off here: ${above}.`);
    this.status("takeoff", "Taking off.");
    if (this.full) (await this.fly(new TakeOff()), await this.awaitEyes());
    else {
      this.ctl.askPilot("takeoff", "Please take off and hover about a meter up. I'll steer once we're flying.");
      await this.fly(new WaitFor("Waiting for the pilot to take off", (c) => c.isFlying(), 25, "Flying.", "The pilot didn't take off within 25 s."));
    }
  }

  // The σ plans are sized for: twice the floor the fixes keep (not a peak between two of them) plus MISSION.planMargin, at
  // least MISSION.planSigma. (With vision's σ about 0.045 m, the old 2σ + 0.13 m planned with 0.22 m and found no way
  // through the house's 0.27 m doorway; the geofence keeps its own live 2σ and buffer.)
  planSigma(p = this.loc.pose()) {
    return clamp(2 * Math.min(p.sigma, this.loc.sigmaFloor?.() ?? p.sigma) + this.o.planMargin, this.o.planSigma, 0.4);
  }

  // Paths kept out of the safety layer's buffer where they fit (twice the σ now, SAFETY.buffer and 2 cm for the σ's rise
  // between fixes: its geofence pushes inside that, and a path along it ended "pushed off the way" for minutes), else
  // as tight as planSigma() (a narrow doorway).
  comfySigma(p = this.loc.pose()) {
    return clamp(2 * p.sigma + SAFETY.buffer + 0.02, this.planSigma(p), 0.4);
  }
  plan(from, to, { alt = this.o.alt, map = this.map } = {}) {
    const tight = this.planSigma(), comfy = this.comfySigma(), g = Array.isArray(to) ? to : [to.x, to.y], go = (sigma) => plan(map, from, to, { alt, sigma, climb: this.full });
    const p = comfy > tight + 0.005 ? go(comfy) : null;
    return p?.ok && Math.hypot(p.to[0] - g[0], p.to[1] - g[1]) < 0.1 ? p : go(tight);
  }

  battery() {
    return this.ctl.safety?.battery?.() ?? null;
  }

  // The speed (m/s) legs are budgeted at: what the drone really flies (nav/avoid.js cruise(): its caps, the speed flown).
  cruise() {
    return cruiseSpeed(this.ctl);
  }

  // Seconds a look around (`headings` of them, evenly spread) takes: with vision the position source, turns at
  // SAFETY.yawVision in SCAN.step steps with a dwell for a fix after each, then the look; at least MISSION.scanTime, and no
  // less than the scans of this flight took a heading (noteScan).
  scanSeconds(headings = 4) {
    const turn = (2 * Math.PI) / headings, vision = this.loc.visionActive?.(), rate = vision ? SAFETY.yawVision : ((this.ctl.settings?.get?.("maxYawRate") ?? 120) * Math.PI) / 180;
    const per = turn / rate + (vision ? Math.ceil(turn / SCAN.step) * SCAN.dwell : 0) + SCAN.look + 0.3 + (this.ctl.videoDelay ?? 0) / 1000;
    return Math.max(this.o.scanTime, headings * Math.max(per, this.scanPer ?? 0));
  }
  noteScan(seconds, headings) {
    if (headings > 0 && seconds > 0) this.scanPer = this.scanPer == null ? seconds / headings : this.scanPer + (seconds / headings - this.scanPer) * 0.5;
  }

  // Can we fly `length` m (and `scans`) to p and still get home with the margin?
  afford(p, length, { doors = 0, scans = 0 } = {}) {
    const b = this.battery(), h = this.house.home, v = this.cruise();
    if (!b || !h) return true;
    const back = this.plan(p, [h.x, h.y]), home = (back.ok ? back.length : 2 * Math.hypot(h.x - p[0], h.y - p[1])) / v + this.o.landTime;
    const need = (this.need = length / v + doors * this.o.doorTime + scans * this.scanSeconds() + this.o.budget * home + this.o.reserve);
    return b.secondsLeft >= need;
  }

  // What temporary obstacle blocks the way to goal, in words: { text, person, wait (it may clear by itself), at: [x, y],
  // id, relook (a suspected change: a closer look may clear it), door (the doorway that looks closed), live (only the live
  // depth says so: a door leaf no change report backs, a way-ahead wedge, something it placed in free space) } | null. Only
  // one on the way counts (the plan without temporary obstacles passes it; a suspected change first, else the first along
  // that way); null when nothing temporary is in the way, or when even that plan fails (the place itself can't be
  // reached). A suspected change (a closed door, a new obstacle) stays until someone confirms or dismisses it in Events,
  // or the camera sees the spot as the scan has it, so waiting doesn't help.
  blocker(goal, { alt = this.o.alt } = {}) {
    const P = this.loc.pose(), on = (goal && blockers(this.map, [P.x, P.y], goal, { alt, sigma: this.planSigma(), climb: this.full })?.on) ?? [];
    const t = on.find((q) => q.kind === "change") ?? on[0]; // something that stays (a suspected change) first: waiting won't help
    if (!t) return null;
    const at = Number.isFinite(t.x) ? [t.x, t.y] : [0, 1].map((a) => t.polygon.reduce((s, q) => s + q[a], 0) / t.polygon.length), room = theRoom(this.roomName(this.map.roomAt(...at)?.id));
    const between = (d) => (d?.rooms[1] ? ` between ${theRoom(this.roomName(d.rooms[0]))} and ${theRoom(this.roomName(d.rooms[1]))}` : ` in ${room}`), base = { at, id: t.id };
    if (t.kind === "person") return { ...base, person: true, wait: true, text: `A person in ${room} is in the way` };
    if (t.kind !== "change") {
      // a door leaf (nav/avoid.js), or what the live depth found across a doorway (at its quarter, middle and three-quarter
      // points): a closed door (the change detector may not have seen it from enough places yet); waiting won't open it
      const near = (x, y) => (t.polygon ? polyDist(x, y, t.polygon) : Math.hypot(x - t.x, y - t.y) - t.r) < 0.2;
      const shut = t.source === "door leaf" ? this.map.doors.find((d) => d.id === t.door) : t.source === "live depth" && this.map.doors.find((d) => [0.25, 0.5, 0.75].every((u) => near(d.a[0] + u * (d.b[0] - d.a[0]), d.a[1] + u * (d.b[1] - d.a[1]))));
      // live: only the live depth says so (no closed door the change detector reported there, Claude didn't call it no change)
      const said = shut && this.memory?.changes?.some((c) => c.door === shut.id && /^door-closed/.test(c.kind) && c.status !== "dismissed" && c.claude?.verdict !== "no-change");
      if (shut) return { ...base, door: shut, live: !said, wait: false, text: `The doorway${between(shut)} looks closed` };
      return { ...base, live: /^(live depth|way ahead)$/.test(t.source ?? ""), wait: Number.isFinite(t.until), text: `Something that isn't on the map is in the way in ${room}` };
    }
    const c = this.memory?.changes?.find((q) => q.id === t.id), id = c?.door ?? (String(t.id).startsWith("door:") ? String(t.id).slice(5) : null), d = id && this.map.doors.find((q) => q.id === id);
    const relook = c?.status === "suspected";
    if (/^door/.test(t.change ?? c?.kind ?? "") || d) return { ...base, door: d, relook, wait: false, text: `The doorway${between(d)} looks closed (Events: confirm or dismiss it)` };
    return { ...base, relook, wait: false, text: `Something new in ${room} blocks the way (Events: confirm or dismiss it)` };
  }

  // No way to goal and nothing temporary on it to blame: a doorway into its room that looks closed (what the 3D map learnt in
  // flight can close one before anything else says so), else the planner's reason. A room a patrol, search or look-in
  // can't reach is skipped (blocked), not the end of the mission.
  noWay(goal, reason) {
    const room = this.map.roomAt(...goal)?.id, d = room && this.closedDoors().find((q) => q.rooms.includes(room));
    const between = d?.rooms[1] ? ` between ${theRoom(this.roomName(d.rooms[0]))} and ${theRoom(this.roomName(d.rooms[1]))}` : "";
    return new Stop(d ? `The doorway${between} looks closed.` : `I can't find a way there: ${this.plain(reason)}.`, { blocked: true, noWay: true });
  }

  // Suspected changes on the way to goal that Claude called no change, at least MISSION.passConfidence sure, when nothing
  // else temporary is on it: return home may pass them (slowly, watching with live depth). -> ids
  passableChanges(goal, { alt = this.o.alt } = {}) {
    const P = this.loc.pose(), on = blockers(this.map, [P.x, P.y], goal, { alt, sigma: this.planSigma(), climb: this.full })?.on ?? [];
    const ok = (t) => {
      const c = t.kind === "change" && this.memory?.changes?.find((q) => q.id === t.id);
      return c?.status === "suspected" && !c.passing && c.claude?.verdict === "no-change" && c.claude.confidence >= this.o.passConfidence;
    };
    return on.length && on.every(ok) ? on.map((t) => t.id) : [];
  }

  // Doorways that look closed now: a door leaf (nav/avoid.js), a suspected or confirmed closed door on the map, or one the
  // house marks not passable. -> [map door]
  closedDoors() {
    const ids = new Set(this.map.doors.filter((d) => d.passable === false).map((d) => d.id));
    for (const t of this.map.temps?.values() ?? []) {
      if (t.source === "door leaf" && t.door) ids.add(t.door);
      else if (t.kind === "change" && /^door-closed/.test(t.change ?? "")) ids.add(this.memory?.changes?.find((q) => q.id === t.id)?.door ?? (String(t.id).startsWith("door:") ? String(t.id).slice(5) : null));
    }
    return this.map.doors.filter((d) => ids.has(d.id));
  }

  // How a room was seen from the drone's spot P (after a scan there that began at `since`, Date.now() ms): { inside, share
  // (of its floor, through open doorways only; no more than the camera really saw of it since, where the 3D map keeps
  // that), cam (that share, or null), shut (a doorway into it looks closed) }.
  roomView(roomId, P = this.loc.pose(), since = null) {
    const shut = this.closedDoors(), inside = this.map.roomAt(P.x, P.y)?.id === roomId, cam = inside ? null : this.camSeen(roomId, since);
    return { inside, share: inside ? 1 : Math.min(seenShare(this.map, roomId, [[P.x, P.y]], { shut }), cam ?? 1), cam, shut: shut.some((d) => d.rooms.includes(roomId)) };
  }
  // Share of a room's known-free space (0.6 and 1.0 m over its floor, every 20 cm) the drone camera saw since `since`
  // (vox.markSeen, Date.now() ms); null without those marks.
  camSeen(roomId, since) {
    const m = this.map, vox = m.vox, ri = m.rooms.findIndex((r) => r.id === roomId);
    if (!vox?.seen || !vox.seenAt || ri < 0 || since == null) return null;
    let n = 0, seen = 0;
    for (let r = 0; r < m.H; r += 4)
      for (let c = 0; c < m.W; c += 4) {
        const k = r * m.W + c;
        if (m.room[k] !== ri || m.wall[k] || m.floorZ[k] == null) continue;
        const [x, y] = m.center(c, r);
        for (const h of [0.6, 1.0]) if (vox.state(x, y, m.floorZ[k] + h) === 1) (n++, (seen += vox.seenAt(x, y, m.floorZ[k] + h) >= since ? 1 : 0)); // FREE
      }
    return n >= 10 ? seen / n : null;
  }

  // To an H point over the planner's path; narrow doorways with DoorTransit. A blocked way is planned again around what
  // blocks it (now on the map); with no way around, it hovers in steps of waitStep s while that may clear, up to
  // `patience` s of waiting in all, then says what blocks it; a doorway it failed to pass doorTries times (the plan keeps
  // choosing it) ends the leg too. A suspected change in the way gets a closer look first; one that is still there after it
  // and that Claude called no change (MISSION.passConfidence sure) is passed, slowly, while the drone can see what the scan
  // lacks (live depth then stops it at a real thing; a door leaf names a closed door), as return home does. What only the
  // live depth says is in the way gets one look from in front of it per leg (closer()). On the ground
  // it plans before taking off, so a place it can't reach (unknown space, off the map) is refused without flying.
  async flyTo(goal, { alt = this.o.alt, label = "Flying", scans = 0, budget = true, patience = this.o.patience } = {}) {
    if (!this.ctl.isFlying()) {
      const P = this.loc.pose(), p = this.plan([P.x, P.y], goal, { alt }), why = !p.ok && this.blocker(goal, { alt });
      if (!p.ok && !why) throw this.noWay(goal, p.reason);
      if (why && !why.wait && !(why.relook && this.passableChanges(goal, { alt }).length)) throw new Stop(`${why.text}.`, { blocked: true });
    }
    await this.ensureFlying();
    let waited = 0, held = 0, passing = [];
    const doorFails = new Map(), relooked = new Set(), avoid = this.ctl.safety?.avoid;
    try {
      for (let replans = 0; ; replans++) {
        const map = passing.length ? withoutTemps(this.map, passing) : this.map;
        const P = this.loc.pose(), p = this.plan([P.x, P.y], goal, { alt, map });
        if (!p.ok) {
          const why = this.blocker(goal, { alt });
          if (!why) throw this.noWay(goal, p.reason);
          if (why.relook && !relooked.has(why.id)) { // a suspected change: a look at it first (a false one is dropped)
            relooked.add(why.id);
            this.status("wait", `${why.text.replace(/ \(Events.*\)$/, "")}: taking a closer look.`);
            await this.look(why.at, this.o.relook);
            continue;
          }
          if (why.live && !relooked.has(why.id)) { // only the live depth says so: a look from in front of it (a false one is seen through)
            relooked.add(why.id);
            const c = this.closer(why, alt);
            this.status("wait", `${why.text}: taking a closer look${c.spot ? " from in front of it" : ""}.`);
            await this.lookCloser(c, alt);
            continue;
          }
          const ids = why.relook && !passing.length && avoid?.eyes().ok ? this.passableChanges(goal, { alt }) : [];
          if (ids.length) {
            passing = ids;
            for (const id of ids) avoid.pass(id);
            this.status("fly", `${why.text.replace(/ \(Events.*\)$/, "")}, but Claude saw no change there: passing it slowly, watching with live depth.`);
            continue;
          }
          if (!why.wait) throw new Stop(`${why.text}.`, { blocked: true, door: why.door });
          if (waited >= patience) throw new Stop(`${why.text} and I can't get around it; I waited ${Math.round(waited)} s.`, { blocked: true, person: why.person });
          this.status("wait", `${why.text}; waiting for the way to clear.`);
          await this.fly(new Hold(this.o.waitStep), { soft: true });
          waited += this.o.waitStep;
          continue;
        }
        if (budget && !this.afford(p.to, p.length, { doors: p.doors.length, scans })) {
          const left = Math.floor(this.battery().secondsLeft);
          throw new Stop(`The battery is too low for that (about ${left} s of flying left; that leg and the way home need about ${Math.max(left + 1, Math.ceil(this.need))} s at the ${this.cruise().toFixed(2)} m/s I can fly now).`, { home: true });
        }
        this.emit("path", { path: p.path, doors: p.doors });
        this.status("fly", `${label} (${p.length.toFixed(1)} m).`);
        try {
          const t0 = performance.now();
          await this.legs(p, alt, label, map, passing.length ? HOME_TRY.passVmax : this.o.vmax);
          this.ctl.safety?.avoid?.noteLeg?.(p.length, (performance.now() - t0) / 1000); // for budgets at the speed really flown
          return p;
        } catch (e) {
          if (!(e instanceof Stop) || !e.blocked || replans >= 30) throw e;
          if (e.door && doorFails.set(e.door.id, (doorFails.get(e.door.id) ?? 0) + 1).get(e.door.id) >= this.o.doorTries)
            throw new Stop(`I couldn't get through the doorway to ${theRoom(this.roomName(e.door.rooms[1]))} (${clause(e.message)}, ${this.o.doorTries} tries).`, { blocked: true });
          if (e.held) {
            // held still by something the map lacks: a look either side (the change detector places it from two
            // directions), then plan again; after a few, say so
            if (++held > 4) throw e;
            const P = this.loc.pose();
            await this.fly(new ScanAt({ x: P.x, y: P.y, headings: [P.yaw + 0.45, P.yaw - 0.45, P.yaw] }, { localizer: this.loc, map: this.map, labels: [], look: 0.3, snapshots: false, label: "Looking at what is in the way" }), { soft: true });
          }
          if (!e.person) {
            this.status("fly", `${e.message} Finding another way.`);
            continue;
          }
          // Held back by someone the way passes (the hold goes by the nearest they could be): a moment, then plan again.
          const why = this.blocker(goal, { alt }) ?? { text: "A person is in the way" };
          if (waited >= patience) throw new Stop(`${why.text} and I can't get around it; I waited ${Math.round(waited)} s.`, { blocked: true, person: true });
          this.status("wait", `${why.text}; waiting a moment before trying again.`);
          await this.fly(new Hold(this.o.waitStep), { soft: true });
          waited += this.o.waitStep;
        }
      }
    } finally {
      for (const id of passing) avoid?.unpass(id);
    }
  }

  async legs(p, alt, label, map = this.map, vmax = this.o.vmax) {
    const need = map.lethal(this.planSigma()), b = map.bandOf(alt);
    const narrow = p.doors.filter((d) => (map.doors.find((x) => x.id === d.id)?.clearance?.[b] ?? 0) < need + this.o.narrowDoor);
    const follow = (path, doors) => new PathFollow(path, { localizer: this.loc, map, vmax, doors, need, label });
    if (!narrow.length) return this.fly(follow(p.path, p.doors)).catch(this.blocked);
    let from = p.from;
    for (const d of [...narrow, null]) {
      const to = d ? d.pre : p.to, leg = this.plan(from, to, { alt, map });
      if (!leg.ok) throw new Stop(`I can't find a way there: ${this.plain(leg.reason)}.`);
      await this.fly(follow(leg.path, leg.doors)).catch(this.blocked);
      if (!d) return;
      this.status("door", `Going through the doorway to ${this.roomName(d.rooms[1])}.`);
      await this.fly(new DoorTransit(d, { localizer: this.loc, map })).catch((e) => this.blocked(Object.assign(e, { door: d })));
      from = d.post;
    }
  }

  blocked = (e) => {
    throw e instanceof Stop && /way|blocked|doorway/i.test(e.message) ? Object.assign(e, { blocked: true }) : e;
  };

  async face(yaw, look = 0) {
    const P = this.loc.pose();
    return this.fly(new ScanAt({ x: P.x, y: P.y, headings: [yaw] }, { localizer: this.loc, map: this.map, labels: [], look, label: "Turning" }), { soft: true });
  }

  // Where to look at what only the live depth says blocks the way (why: blocker() with live), head-on: from about 1 m in
  // front of its doorway's middle on this side (a door leaf, live depth across a doorway), or of it along the bearing from
  // here (a way-ahead wedge, something placed in free space); a false one is then seen through and goes (nav/avoid.js).
  // -> { id, spot ([x, y] free, or null: from here), face: [x, y], text, seconds }
  closer(why, alt = this.o.alt) {
    const P = this.loc.pose(), d = why.door, face = d ? [(d.a[0] + d.b[0]) / 2, (d.a[1] + d.b[1]) / 2] : why.at;
    let n = d ? [-(d.b[1] - d.a[1]), d.b[0] - d.a[0]] : [P.x - face[0], P.y - face[1]];
    const L = Math.hypot(...n);
    n = L > 0.05 ? n.map((v) => (v / L) * Math.sign(n[0] * (P.x - face[0]) + n[1] * (P.y - face[1]) || 1)) : null;
    const spot = n && nearestFree(this.map, [face[0] + n[0], face[1] + n[1]], { alt, sigma: this.planSigma(), maxR: 0.4 });
    return { id: why.id, spot, face, text: why.text, seconds: this.o.relook };
  }
  async lookCloser(c, alt = this.o.alt) {
    const P = this.loc.pose(), p = c.spot && this.plan([P.x, P.y], c.spot, { alt });
    if (p?.ok && p.length > 0.2) await this.fly(new PathFollow(p.path, { localizer: this.loc, map: this.map, vmax: HOME_TRY.passVmax, doors: p.doors, need: this.map.lethal(this.planSigma()), label: "Flying in front of it for a closer look" }), { soft: true });
    return this.look(c.face, c.seconds);
  }

  // Face a spot [x, y] from here and watch it for `seconds`.
  async look(at, seconds) {
    const P = this.loc.pose();
    return this.fly(new ScanAt({ x: P.x, y: P.y, headings: [Math.atan2(at[1] - P.y, at[0] - P.x)] }, { localizer: this.loc, map: this.map, labels: [], look: seconds, snapshots: false, label: "Taking a closer look" }), { soft: true });
  }

  // A look around at a spot. Headings the detector didn't see (too few fresh frames with it working: ScanAt) don't count
  // as looked at: noted per room in this.unlooked (room -> why) for the summaries. -> the sightings, with .looked (the
  // share of headings looked at).
  async scan(spot, { labels = ["person", "cat", "dog"], label = "Looking around" } = {}) {
    this.status("scan", `${label}.`);
    const t0 = performance.now(), r = await this.fly(new ScanAt(spot, { localizer: this.loc, map: this.map, labels, label }), { soft: true });
    this.noteScan((performance.now() - t0) / 1000, spot.headings.length);
    const room = this.map.roomAt(spot.x, spot.y)?.id ?? null, frames = r.data?.frames ?? [];
    for (const f of frames) this.frames.push({ ...f, room, x: spot.x, y: spot.y });
    const looked = frames.length ? frames.filter((f) => f.looked !== false).length / frames.length : 0;
    if (looked < 1) this.unlooked.set(room, r.data?.why ?? (r.ok ? "the video stalled" : r.text.replace(/\.$/, "").toLowerCase()));
    this.detectNote ||= frames.find((f) => f.note)?.note ?? "";
    return Object.assign(r.data?.sightings ?? [], { looked });
  }

  // "; I couldn't check Room 2 (the person detector wasn't running)" for the rooms among ids a scan didn't see whole.
  notLooked(ids) {
    const miss = ids.filter((id) => this.unlooked.has(id));
    return miss.length ? `; I couldn't check all of ${miss.map((id) => `${theRoom(this.roomName(id))} (${this.unlooked.get(id)})`).join(", ")}` : "";
  }

  // Watch every new frame for the labels being searched (also while flying between scan points).
  // Every new frame: which tracks the detector shows (and which together), and, while searching, sightings of the labels.
  spot() {
    const sp = this.spotting, lat = this.perception?.latest;
    if (!lat?.t || lat.t === this.lastFrame || this.perception.frameAge > 400 || !this.ctl.isFlying()) return;
    this.lastFrame = lat.t;
    const ids = lat.detections.filter((d) => d.trackId != null && d.score >= 0.4).map((d) => d.trackId);
    for (const id of ids) {
      const t = this.tracks.get(id);
      this.tracks.set(id, { t0: t?.t0 ?? lat.t, t: lat.t });
      for (const other of ids) if (other !== id) this.together.add(`${id}|${other}`);
    }
    if (!sp) return;
    for (const d of lat.detections) {
      if (!sp.labels.includes(d.label) || d.score < 0.4) continue;
      const s = locate(d, { ctl: this.ctl, localizer: this.loc, map: this.map, t: lat.t });
      s.size = 2 * s.range * Math.tan((d.box.h * this.ctl.settings.get("hfov") * (Math.PI / 180) * ((lat.height || 3) / (lat.width || 4))) / 2);
      const c = sp.spotter.add(s);
      if (c) this.addFinding(c, sp);
    }
  }

  // The detector gave one it already found a new number: the old track(s) were last seen before the new one first
  // showed up (within renumber.within ms), never in the same frame, the spots within renumber.gate m and the sizes alike.
  renumbered(f, c) {
    const o = this.o.renumber, ids = [f.trackId, ...(f.aliases ?? [])].filter((id) => id != null), cur = this.tracks.get(c.trackId);
    if (c.trackId == null || !ids.length || !cur || Math.hypot(f.x - c.x, f.y - c.y) > o.gate) return false;
    if (f.size > 0 && c.size > 0 && Math.abs(Math.log(c.size / f.size)) > Math.log(1 + o.size)) return false;
    return ids.every((id) => {
      const old = this.tracks.get(id);
      return !this.together.has(`${id}|${c.trackId}`) && (!old || (old.t <= cur.t0 && cur.t0 - old.t < o.within));
    });
  }

  // A confirmed sighting -> a finding (or more of one already found: the same track, within MISSION.near m of where it was
  // first seen, renumbered by the detector, or, for people, anyone about the same size not seen in one frame with them and no faster
  // than MISSION.walk m/s away: one person walking through a patrol is one finding, followed: at, also). A spot off every
  // room isn't given a room (no "hallway"), nor a high-urgency alert; one only from boxes cut by the frame's edge is
  // "probably" there.
  addFinding(c, sp) {
    const t = performance.now(), same = this.findings.find((f) => f.label === c.label && ((c.trackId != null && (f.trackId === c.trackId || f.aliases?.includes(c.trackId))) ||
      Math.hypot(f.x - c.x, f.y - c.y) < this.o.near || this.renumbered(f, c) || (c.label === "person" && this.samePerson(f, c))));
    const room = this.map.roomAt(c.x, c.y)?.id ?? null;
    if (same) {
      if (c.trackId != null && same.trackId !== c.trackId && !same.aliases?.includes(c.trackId)) (same.aliases ??= []).push(c.trackId);
      same.at = [+c.x.toFixed(2), +c.y.toFixed(2)];
      same.lastT = c.t;
      const where = room ? `in ${theRoom(this.roomName(room))}` : null; // seen in another room since (said once each)
      if (where && room !== same.room && !(same.also ??= []).includes(where)) same.also.push(where);
      return;
    }
    const f = { label: c.label, trackId: c.trackId, room, roomName: this.roomName(room, [c.x, c.y]), x: +c.x.toFixed(2), y: +c.y.toFixed(2), score: +c.score.toFixed(2), approx: !!c.cut,
      size: +(c.size ?? 0).toFixed(2), near: this.nearby([c.x, c.y]), snapshot: this.perception.snapshot?.({ maxWidth: 480 }) ?? null, t: Math.round((t - this.t0) / 1000), lastT: c.t };
    Object.assign(f, { at: [f.x, f.y], where: this.placeOf(f) });
    this.findings.push(f);
    this.emit("finding", f);
    this.remember("addSighting", { label: f.label, trackId: f.trackId ?? null, room, x: f.x, y: f.y, z: this.map.floorAt(f.x, f.y) ?? 0, sigma: this.loc.pose()?.sigma ?? null,
      score: f.score, snapshot: f.snapshot, confirmed: true, near: f.near, mission: this.mission ?? null }); // memory words it ("look in the Office")
    this.alerts?.[sp.alert ? "notify" : "log"]?.({ text: `${f.label === "person" ? "A person" : `The ${f.label}`} ${f.where}${f.near ? `, near the ${f.near}` : ""}.`,
      image: f.snapshot, room, urgency: sp.alert && f.label === "person" && room ? "high" : "default", kind: "finding", pose: this.loc.pose(),
      label: f.label, score: f.score, trackId: f.trackId ?? null, x: f.x, y: f.y, box: c.box ?? null });
    if (sp.until?.(this.findings)) {
      this.found = new Stop("found", { found: true });
      this.ctl.abort("found it");
    }
  }
  // Could the person in finding f be the one now confirmed at c? Not if they were in one frame together or differ in size, nor if c was seen
  // before f was gone; else if f was last seen at most MISSION.walkWithin s before c first was and they could have walked
  // there since (MISSION.walk m/s, or the renumbering gate).
  samePerson(f, c) {
    const ids = [f.trackId, ...(f.aliases ?? [])].filter((id) => id != null);
    if (c.trackId != null && ids.some((id) => this.together.has(`${id}|${c.trackId}`))) return false;
    if (f.size > 0 && c.size > 0 && Math.abs(Math.log(c.size / f.size)) > Math.log(1 + this.o.renumber.size)) return false; // a child where an adult was
    const last = Math.max(f.lastT ?? -Infinity, ...ids.map((id) => this.tracks.get(id)?.t ?? -Infinity)), first = (c.trackId != null && this.tracks.get(c.trackId)?.t0) || c.t0, dt = (first - last) / 1000;
    return dt >= -0.5 && dt <= this.o.walkWithin && Math.hypot(f.at[0] - c.x, f.at[1] - c.y) <= Math.max(this.o.renumber.gate, this.o.walk * dt);
  }

  async searching(labels, { until, alert = false }, fn) {
    this.spotting = { labels, spotter: new Spotter(this.o), until, alert };
    this.found = null;
    try {
      return await fn();
    } catch (e) {
      if (!e?.found) throw e;
      return "found";
    } finally {
      this.spotting = null;
      this.found = null;
    }
  }

  // Where a multi-stop mission will be: the stop flown to now, then the rest (rooms by their middle while their spots
  // aren't known yet), then home or not, and about how long all of it takes (planned legs at cruise speed, a scan per
  // stop, the landing): the "plan" event, for where_am_i ("then Room 3 and Room 4, then home; about 70 s in all").
  ahead(stop, rest = [], home = false) {
    const P = this.loc.pose(), h = this.house.home, sigma = this.planSigma();
    const stops = [stop, ...rest.map((q) => (typeof q === "string" ? { room: q, at: roomCenter(this.map, q, { alt: this.o.alt, sigma }) } : q))]
      .map((q) => ({ room: q.room ?? null, x: q.x ?? q.at?.[0], y: q.y ?? q.at?.[1] }));
    let at = [P.x, P.y], s = 0;
    const v = this.cruise(), leg = (to) => {
      const p = this.plan(at, to);
      s += (p.ok ? p.length : 1.5 * Math.hypot(to[0] - at[0], to[1] - at[1])) / v;
      at = to;
    };
    for (const q of stops) if (Number.isFinite(q.x)) (leg([q.x, q.y]), (s += this.scanSeconds()));
    if (home && h) (leg([h.x, h.y]), (s += this.o.landTime));
    this.emit("plan", { stops: stops.map(({ room, x, y }) => ({ room, name: room && this.roomName(room), ...(Number.isFinite(x) && { x: +x.toFixed(2), y: +y.toFixed(2) }) })), home, seconds: Math.round(s) });
  }

  // ---- naming ----

  // A room's name; off every room (a doorway, or a spot past the map's outlines): "area past the Living room (off the map)"
  // by the room nearest `at`, else "area between rooms". Never a room the house doesn't have.
  roomName(id, at = null) {
    const r = this.house.rooms.find((q) => q.id === id), near = !r && at && this.nearestRoom(at);
    return r ? r.name : near ? `area past ${theRoom(near.name)} (off the map)` : "area between rooms";
  }
  nearestRoom([x, y]) {
    let best = null, bd = Infinity;
    for (const r of this.house.rooms) {
      const d = r.outline?.length > 2 ? polyDist(x, y, r.outline) : Infinity;
      if (d < bd) [best, bd] = [r, d];
    }
    return best;
  }
  // Where a finding is, in words: "in the Living room", "probably in Room 2" (only boxes cut by the frame's edge placed it),
  // or "somewhere past Room 3 (I couldn't place it on the map)".
  placeOf(f) {
    if (f.room) return `${f.approx ? "probably " : ""}in ${theRoom(f.roomName)}`;
    const near = this.nearestRoom([f.x, f.y]);
    return near ? `somewhere past ${theRoom(near.name)} (I couldn't place it on the map)` : "somewhere I couldn't place on the map";
  }
  // The planner's reasons in plain words, each once (no coordinates, no σ): "start (0.01, 0.16) has no free space within
  // 1 m (σ 0.17 m)" -> someone standing by the drone (or the pad), or the space around it not clear on the map.
  plain(reason, from = null) {
    const P = this.loc.pose(), at = from ?? [P.x, P.y], pad = !!from, who = pad ? "the pad" : "me";
    const person = [...(this.map.temps?.values() ?? [])].some((t) => t.kind === "person" && (t.polygon ? polyDist(at[0], at[1], t.polygon) : Math.hypot(at[0] - t.x, at[1] - t.y) - (t.r ?? 0)) < 0.3);
    const one = (r) => {
      let m;
      if (/^start\b.*no free space/.test(r)) return person ? `someone is standing by ${who}: please step back about 2 m` : `the space around ${who} isn't clear on the map`;
      if (/^goal\b.*no free space/.test(r)) return "there's no free space to fly to there on the map";
      if ((m = /^no doorway from (.+) to (.+) has room/.exec(r))) return `no doorway from ${theRoom(m[1])} to ${theRoom(m[2])} has room for me right now`;
      if (/^no path\b/.test(r)) return "there's no way there through space the map knows is free";
      if (r === "no free space") return "it has no free space to fly in on the map";
      return r.replace(/\s*\(σ [\d.]+ m\)/g, "").replace(/\s*\(-?[\d.]+, -?[\d.]+\)/g, "").replace(/ with σ [\d.]+ m/g, "");
    };
    return [...new Set(String(reason ?? "").split(/;\s*/).filter(Boolean).map(one))].join("; ");
  }

  nearby([x, y], within = 1.5) {
    let best = null, bd = within;
    for (const l of this.house.landmarks ?? []) {
      const d = Math.hypot(l.x - x, l.y - y);
      if (d < bd && l.name.length <= 30) [best, bd] = [l.name.toLowerCase(), d];
    }
    return best;
  }

  // A place -> { point, name, room, face? }.
  resolve(target) {
    const P = this.loc.pose(), sigma = this.planSigma();
    if (target && typeof target === "object" && Number.isFinite(target.x)) return { point: [target.x, target.y], name: where([target.x, target.y]) };
    if (/^(home|home pad|pad|base|takeoff spot)$/.test(norm(target)) && this.house.home) return { point: [this.house.home.x, this.house.home.y], name: "the home pad" };
    const room = findRoom(this.house, target);
    if (room) {
      // Its most open spot, else (furniture can cut that off at this height) its viewpoints, else just inside a doorway.
      const vps = viewpoints(this.map, room.id, "person", { alt: this.o.alt, sigma }).points.map((v) => [v.x, v.y]);
      const doors = this.map.doors.filter((d) => d.passable && d.rooms.includes(room.id) && d.rooms[1]).map((d) => {
        const n = [-(d.b[1] - d.a[1]), d.b[0] - d.a[0]], L = Math.hypot(...n), m = [(d.a[0] + d.b[0]) / 2, (d.a[1] + d.b[1]) / 2];
        const p = [1, -1].map((sg) => [m[0] + (sg * 0.6 * n[0]) / L, m[1] + (sg * 0.6 * n[1]) / L]).find((q) => this.map.roomAt(...q)?.id === room.id);
        return p && nearestFree(this.map, p, { alt: this.o.alt, sigma, maxR: 0.5 });
      });
      const goals = [roomCenter(this.map, room.id, { alt: this.o.alt, sigma }), ...vps, ...doors].filter(Boolean);
      if (!goals.length) throw new Stop(`The ${room.name} has no room for me to fly in.`);
      return { point: goals[0], alternatives: goals.slice(1), name: theRoom(room.name), room: room.id };
    }
    const l = findLandmark(this.house, target, P);
    if (l) {
      const c = nearestFree(this.map, [l.x, l.y], { alt: this.o.alt, sigma, maxR: 2 });
      if (!c) throw new Stop(`There's no free space next to the ${l.name}.`);
      return { point: c, name: `the ${l.name.toLowerCase()}`, room: l.room, face: [l.x, l.y] };
    }
    throw new Stop(`I don't know a place called "${target}" in this house. Rooms: ${this.house.rooms.map((r) => r.name).join(", ")}.`);
  }

  // ---- missions ----

  async goTo(target) {
    const t = this.resolve(target), start = this.loc.pose();
    const point = [t.point, ...(t.alternatives ?? [])].find((g) => this.plan([start.x, start.y], g).ok) ?? t.point;
    const p = await this.flyTo(point, { label: `Flying to ${t.name}` });
    if (t.face) {
      const P = this.loc.pose();
      await this.face(Math.atan2(t.face[1] - P.y, t.face[0] - P.x));
    }
    const P = this.loc.pose(), err = Math.hypot(P.x - p.to[0], P.y - p.to[1]), short = Math.hypot(p.to[0] - point[0], p.to[1] - point[1]);
    return `Arrived at ${t.name} (${p.length.toFixed(1)} m${err > 0.3 ? `, ${err.toFixed(1)} m off` : ""})${short > 0.3 ? `: as close as I can get, ${short.toFixed(1)} m short (past that is blocked or space I haven't seen)` : ""}.`;
  }

  // Doorway first: stop just inside the last doorway into the room and look; go further in only if that sees too little.
  async lookIn(roomQ) {
    const room = findRoom(this.house, roomQ);
    if (!room) throw new Stop(`There's no room called "${roomQ}". Rooms: ${this.house.rooms.map((r) => r.name).join(", ")}.`);
    const sigma = this.planSigma(), vps = viewpoints(this.map, room.id, "person", { alt: this.o.alt, sigma }).points;
    if (!vps.length) throw new Stop(`I can't find a spot to look into ${theRoom(room.name)} from.`);
    const P = this.loc.pose(), spots = [];
    if (P.room !== room.id) {
      const p = this.plan([P.x, P.y], [vps[0].x, vps[0].y]), door = p.ok ? p.doors.filter((d) => d.rooms[1] === room.id).at(-1) : null;
      const at = door && nearestFree(this.map, door.post, { alt: this.o.alt, sigma, maxR: 0.4 });
      if (at) spots.push({ x: at[0], y: at[1], cover: coverageFrom(this.map, room.id, at), door: true });
    }
    if (!spots.length || spots[0].cover < 0.9) spots.push(...vps.map((v) => ({ ...v, cover: v.coverage })));
    // A spot it can't reach (a doorway that looks closed) is skipped; the look counts only from inside the room or seeing
    // it whole through open doorways (roomView).
    const sightings = [];
    let view = null, blocked = "";
    for (const s of spots) {
      const c = this.roomCentroid(room), yaw0 = Math.atan2(c[1] - s.y, c[0] - s.x), n = Math.ceil(360 / (this.ctl.settings.get("hfov") * 0.8));
      try {
        await this.flyTo([s.x, s.y], { label: s.door ? `Flying to the doorway of ${theRoom(room.name)}` : `Flying into ${theRoom(room.name)}`, scans: 1 });
      } catch (e) {
        if (!(e instanceof Stop) || !e.blocked) throw e;
        blocked ||= e.message.replace(/\.$/, "").replace(/ \(Events.*\)$/, "").replace(/^(?!I\b)./, (q) => q.toLowerCase()); // "I couldn't…" keeps its I
        continue;
      }
      const since = Date.now();
      sightings.push(...(await this.scan({ x: s.x, y: s.y, headings: Array.from({ length: n }, (_, i) => yaw0 + (2 * Math.PI * i) / n) }, { label: `Looking into ${theRoom(room.name)}` })));
      const v = this.roomView(room.id, undefined, since);
      if (!view || v.inside || v.share > view.share) view = v;
      if (s.cover >= 0.9 && (v.inside || v.share >= this.o.covered)) break;
    }
    const sp = new Spotter(this.o);
    for (const s of sightings) {
      const c = sp.add(s);
      if (c) this.addFinding(c, {});
    }
    const seen = this.findings.filter((f) => f.room === room.id), name = theRoom(room.name);
    if (!view) throw new Stop(`I couldn't look into ${name}: ${blocked || "I couldn't get to a spot to look from"}.`, { blocked: true });
    const part = view.inside || view.share >= this.o.covered ? "" : `; I only saw ${view.shut && view.share < 0.05 ? "none of it (its doorway looks closed)" : `about ${Math.round(100 * view.share)}% of it, from outside`}`;
    return `Looked into ${name}: ${seen.length ? seen.map((f) => `${f.label === "person" ? "a person" : `the ${f.label}`}${f.near ? ` near the ${f.near}` : ""}`).join(", ") : this.unlooked.size || part ? "no people or pets seen" : "no people or pets"}${this.notLooked([...this.unlooked.keys()])}${part}.`;
  }

  // Share of a room's floor under furniture (pet height to 1 m): where someone could be out of sight.
  clutter(id) {
    const ri = this.map.rooms.findIndex((r) => r.id === id), occ = [this.map.occ[this.map.bandOf(0.6)], this.map.occ[this.map.bandOf(1.0)]];
    let n = 0, full = 0;
    for (let k = 0; k < this.map.N; k++) if (this.map.room[k] === ri) (n++, (full += occ[0][k] || occ[1][k] ? 1 : 0));
    return n ? full / n : 0;
  }

  roomCentroid(room) {
    const o = room.outline;
    return [o.reduce((a, p) => a + p[0], 0) / o.length, o.reduce((a, p) => a + p[1], 0) / o.length];
  }

  async patrol(rooms) {
    const ids = this.roomIds(rooms), h = this.house.home;
    if (!h) throw new Stop("There's no home pad yet: set one first.");
    const route = patrolRoute(this.map, ids, [h.x, h.y], { alt: this.o.alt, sigma: this.planSigma() });
    if (!route.stops.length) throw new Stop(`I can't plan a patrol: ${this.plain(route.reason ?? route.skipped.map((s) => s.reason).join("; "), [h.x, h.y])}.`);
    const visited = [], missed = [], unseen = [];
    await this.searching(["person", "cat", "dog"], { alert: this.settings?.get?.("patrolAlerts") !== false }, async () => {
      for (const [i, s] of route.stops.entries()) {
        this.ahead(s, route.stops.slice(i + 1), true);
        let at = s;
        try {
          await this.flyTo([s.x, s.y], { label: `Patrolling: ${this.roomName(s.room)}`, scans: 1 });
        } catch (e) { // a room it can't get to now: the others still get their look; already in it, it looks from here
          if (!(e instanceof Stop) || !e.blocked) throw e;
          const P = this.loc.pose();
          if (P.room !== s.room) {
            missed.push(` I couldn't get to ${theRoom(this.roomName(s.room))}: ${clause(e.message)}.`);
            this.status("fly", `I can't get to ${theRoom(this.roomName(s.room))}: ${e.message} Going on.`);
            continue;
          }
          this.status("scan", `I can't get to my spot in ${theRoom(this.roomName(s.room))} (${clause(e.message)}), so I'm looking around from here.`);
          at = { ...s, x: P.x, y: P.y };
        }
        const since = Date.now();
        await this.scan({ ...at, headings: s.headings.length ? s.headings : [0, Math.PI / 2, Math.PI, -Math.PI / 2] }, { labels: [], label: `Checking ${theRoom(this.roomName(s.room))}` });
        // patrolled only from inside, or seen whole through open doorways (a spot the planner moved out of the room, past a
        // doorway that looks closed, sees nothing of it), and no more of it than the camera saw
        const v = this.roomView(s.room, undefined, since);
        if (v.inside || v.share >= this.o.covered) visited.push(s.room);
        else unseen.push(` I couldn't check ${theRoom(this.roomName(s.room))}: ${v.shut && v.share < 0.05 ? "its doorway looks closed, so I saw none of it" : `I only saw about ${Math.round(100 * v.share)}% of it, from outside`}.`);
      }
    });
    this.emit("plan", null);
    // where it really ended up: home, or (said as such) a clear spot elsewhere
    const back = await this.goHome(), end = back.home ? "" : ` ${back.text}`;
    const people = this.findings.filter((f) => f.label === "person"), pets = this.findings.filter((f) => f.label !== "person"), seen = visited.filter((id) => !this.unlooked.has(id));
    // rooms the route couldn't reach: why (a doorway that looks closed, said as such), not just "skipped"
    const shut = this.closedDoors(), why = (s) => (shut.some((d) => d.rooms.includes(s.room)) ? "its doorway looks closed, so I saw none of it" : this.plain(s.reason, [h.x, h.y]));
    const skipped = route.skipped.map((s) => ` I couldn't check ${theRoom(this.roomName(s.room))}: ${why(s)}.`).join(""), gone = `${missed.join("")}${unseen.join("")}`;
    // "nobody" only for rooms the detector really looked at
    const who = people.length ? people.map((f) => `a person ${f.where}${f.also?.length ? ` (then ${f.also.join(", then ")})` : ""}`).join(", ") : seen.length ? `nobody${seen.length < visited.length ? ` in ${seen.map((id) => this.roomName(id)).join(", ")}` : ""}` : "no one seen";
    if (!visited.length) throw new Stop(`I couldn't get to any room to patrol.${gone}${skipped}${back.home ? ` I came back ${this.full ? "and landed on the home pad" : "over the home pad"}.` : end}`);
    return `Patrolled ${visited.map((id) => this.roomName(id)).join(", ")}${back.home ? (this.full ? " and landed home" : " and came back over the home pad") : ""}: ${who}${pets.length ? `, ${pets.map((f) => `the ${f.label} ${f.where}`).join(", ")}` : ""}${this.notLooked(visited)}.${gone}${skipped}${end}`;
  }

  roomIds(rooms) {
    if (!rooms?.length) return this.house.rooms.map((r) => r.id);
    const out = rooms.map((q) => findRoom(this.house, q)?.id);
    if (out.includes(undefined)) throw new Stop(`I don't know the room "${rooms[out.indexOf(undefined)]}". Rooms: ${this.house.rooms.map((r) => r.name).join(", ")}.`);
    return out;
  }

  // Rooms nearest first (by flight), the current one first.
  orderRooms(ids) {
    const P = this.loc.pose();
    const d = new Map(ids.map((id) => {
      if (id === P.room) return [id, -1];
      const c = roomCenter(this.map, id, { alt: this.o.alt, sigma: this.planSigma() }), p = c && this.plan([P.x, P.y], c);
      return [id, p?.ok ? p.length : Infinity];
    }));
    return [...ids].sort((a, b) => d.get(a) - d.get(b));
  }

  // People: one scan per room at person height sees a standing person over most furniture (and every scan so far counts
  // for every room it sees into, through doorways too: a room already seen whole that way gets none). A seated or lying
  // person can hide behind a sofa back, so rooms
  // seen past furniture less than MISSION.dense then get the denser furniture-aware viewpoints (the planner's "pet" set,
  // at person height, the most furnished rooms first), only those that add MISSION.gain, until MISSION.covered. One person
  // (or a pet) ends the search when found; a plural one ("people", "everyone") ends when every room is seen that well, or
  // early on the battery, saying which rooms it didn't get to. A scan spot a person or something new blocks is skipped.
  async searchFor(target, rooms, { all = false } = {}) {
    const n = norm(target), label = /^(people|persons?|someone|somebody|anyone|anybody|everyone|humans?)$/.test(n) ? "person" : toDetectorLabel(target);
    const ids = this.orderRooms(this.roomIds(rooms));
    if (!label) {
      for (const id of ids) await this.lookIn(id);
      return `I can't recognize "${target}" on my own, so I looked into ${ids.map((id) => theRoom(this.roomName(id))).join(", ")}; the pictures are attached.`;
    }
    const pet = label !== "person", alt = pet ? this.o.petAlt : this.o.alt, many = all || /^(people|persons|everyone|everybody)$/.test(n);
    if (pet && !this.full) this.ctl.askPilot("low", "Fly low, about knee height, so I can see the floor.");
    const until = (fs) => !many && fs.some((f) => f.label === label);
    const scanned = [], seen = (id, spots = scanned, hides = pet ? "tall" : "mid") => seenShare(this.map, id, spots, { hides, shut: this.closedDoors() }), skipped = new Set();
    const res = await this.searching([label], { until }, async () => {
      for (const kind of pet ? ["pet"] : ["person", "pet"]) {
        const order = kind === "person" || pet ? ids : [...ids].sort((a, b) => this.clutter(b) - this.clutter(a));
        for (const [k, id] of order.entries()) {
          const dense = kind === "pet" && !pet;
          if (dense && seen(id) >= this.o.dense) continue;
          if (kind === "person" && scanned.length && seen(id, scanned, "tall") >= this.o.covered) continue; // seen whole from elsewhere
          const todo = viewpoints(this.map, id, kind, { alt, sigma: this.planSigma() }).points.filter((v) => !scanned.some((q) => Math.hypot(q[0] - v.x, q[1] - v.y) < 1));
          while (todo.length && !(dense && seen(id) >= this.o.covered)) {
            const P = this.loc.pose(), v = todo.splice(todo.indexOf(todo.reduce((a, b) => (Math.hypot(a.x - P.x, a.y - P.y) <= Math.hypot(b.x - P.x, b.y - P.y) ? a : b))), 1)[0];
            if (dense && seen(id, [...scanned, [v.x, v.y]]) - seen(id) < this.o.gain) continue; // adds too little
            this.ahead({ room: id, x: v.x, y: v.y }, order.slice(k + 1));
            try {
              await this.flyTo([v.x, v.y], { alt, label: `Searching ${theRoom(this.roomName(id))}`, scans: 1, patience: 0 });
            } catch (e) {
              if (!(e instanceof Stop)) throw e;
              if (e.blocked) {
                skipped.add(id);
                continue;
              }
              if (!e.home || !many || e.eyes) throw e;
              const not = ids.filter((r) => seen(r) < this.o.covered).map((r) => theRoom(this.roomName(r)));
              throw Object.assign(e, { message: `The battery is getting low, so I stopped searching${not.length ? ` (not fully searched: ${not.join(", ")})` : ""}.` });
            }
            skipped.delete(id);
            const got = await this.scan(v, { labels: [], label: `Looking for ${pet ? `the ${label}` : "people"} in ${theRoom(this.roomName(id))}` });
            if (got.looked >= 0.75) scanned.push([v.x, v.y]); // a spot the detector didn't see from doesn't cover the room
          }
        }
      }
    });
    const hits = this.findings.filter((f) => f.label === label), part = [...skipped].filter((id) => seen(id) < this.o.covered);
    const gaps = part.length ? ` I couldn't get to every spot in ${part.map((id) => theRoom(this.roomName(id))).join(", ")} (the way was blocked).` : "";
    if (!hits.length) return `Searched ${ids.map((id) => theRoom(this.roomName(id))).join(", ")} and didn't find ${pet ? `the ${label}` : "anyone"}${this.notLooked(ids)}.${gaps}`;
    const f = hits[0], P = this.loc.pose();
    if (res === "found") await this.face(Math.atan2(f.y - P.y, f.x - P.x), 0.3);
    return `Found ${hits.map((h) => `${h.label === "person" ? "a person" : `the ${h.label}`} ${h.where}${h.near ? ` near the ${h.near}` : ""}${h.also?.length ? ` (then ${h.also.join(", then ")})` : ""}`).join(", ")}${many ? this.notLooked(ids) : ""}.${gaps}`;
  }

  // Pets: their likely spots first (sofas, beds, chairs; low), then the rooms' floor viewpoints, until both are found.
  async checkOnPets(rooms) {
    const ids = this.orderRooms(this.roomIds(rooms)), sigma = this.planSigma(), alt = this.o.petAlt;
    if (!this.full) this.ctl.askPilot("low", "Fly low, about knee height, so I can see the floor.");
    const spots = [];
    for (const l of this.house.landmarks ?? []) {
      if (!PET_SPOTS.test(l.name) || !ids.includes(l.room) || spots.some((s) => Math.hypot(s.at[0] - l.x, s.at[1] - l.y) < 1.5)) continue;
      const c = nearestFree(this.map, [l.x, l.y], { alt, sigma, maxR: 1.5 });
      if (c) spots.push({ x: c[0], y: c[1], at: [l.x, l.y], room: l.room });
    }
    const until = (fs) => PETS.every((p) => fs.some((f) => f.label === p)), skipped = new Set();
    // a spot whose way is blocked is skipped (the room noted), as in searchFor: the other rooms still get checked
    const reach = async (at, o, id) => {
      try {
        return (await this.flyTo(at, o), true);
      } catch (e) {
        if (!(e instanceof Stop) || !e.blocked) throw e;
        return (skipped.add(id), false);
      }
    };
    await this.searching(PETS, { until }, async () => {
      for (const [k, id] of ids.entries()) {
        for (const s of spots.filter((s) => s.room === id)) {
          this.ahead(s, ids.slice(k + 1));
          if (await reach([s.x, s.y], { alt, label: `Checking the ${this.nearby(s.at) ?? "pet spot"}`, scans: 1 }, id)) await this.face(Math.atan2(s.at[1] - s.y, s.at[0] - s.x), 0.8);
        }
        for (const v of viewpoints(this.map, id, "pet", { alt, sigma }).points) {
          this.ahead({ room: id, x: v.x, y: v.y }, ids.slice(k + 1));
          if (await reach([v.x, v.y], { alt, label: `Looking for the pets in ${theRoom(this.roomName(id))}`, scans: 1 }, id))
            await this.scan(v, { labels: [], label: `Looking for the pets in ${theRoom(this.roomName(id))}` });
        }
      }
    });
    const say = PETS.map((p) => {
      const f = this.findings.find((x) => x.label === p);
      return f ? `the ${p} is ${f.where}${f.near ? ` near the ${f.near}` : ""}` : `no sign of the ${p}`;
    });
    const gaps = skipped.size ? ` I couldn't get to every spot in ${[...skipped].map((id) => theRoom(this.roomName(id))).join(", ")} (the way was blocked).` : "";
    return `${say.join("; ")}${this.notLooked(ids)}.${gaps}`.replace(/^./, (c) => c.toUpperCase());
  }

  // Home and land (co-pilot: the pilot is asked to land). ReturnHome keeps trying a blocked way while the battery allows,
  // then lands on a clear spot nearby and says why. -> what happened, in words
  async returnHome() {
    return (await this.goHome()).text;
  }
  // -> { text, home (it ended over or on the home pad, not on a clear spot elsewhere) }
  async goHome() {
    const h = this.house.home;
    if (!h) throw new Stop("There's no home pad yet: set one first.");
    if (!this.ctl.isFlying()) return { text: "Already on the ground.", home: false };
    this.status("home", "Flying home.");
    const r = await this.fly(new ReturnHome({ localizer: this.loc, map: this.map, home: h, label: "Flying home", ...this.homeHooks(), battery: () => this.battery()?.secondsLeft, say: (t) => this.status("home", t) }), { soft: true });
    if (!r.ok && !r.safeLanded) throw new Stop(r.text);
    if (!this.full) {
      await this.fly(new WaitFor("Waiting for the pilot to land", (c) => !c.isFlying(), 25, "Landed.", "Still flying after 25 s."), { soft: true });
      return r.safeLanded ? { text: r.text, home: false } : { text: "Back over the home pad; asked the pilot to land.", home: true };
    }
    if (r.safeLanded) return { text: r.text, home: false };
    const P = this.loc.pose(), off = Math.hypot(P.x - h.x, P.y - h.y);
    return { text: `Landed on the home pad${off > 0.3 ? `, ${off.toFixed(1)} m off` : ""}.`, home: true };
  }

  // How the drone brakes, measured (full auto): from the most open spot of this room, along its longest free run, three
  // runs at 0.3 m/s for up to 2 m, then a stop (behaviors.js BrakeTest, from the localizer's position and velocity, which
  // are video-late: the lag found includes the video's delay), back and forth along that corridor where it has room (else
  // from the spot again). Each run's whole travel from the stop command to standstill gives its braking; the most careful
  // of them (the lowest braking, the longest reaction) is saved as settings "brake" { decel, react, v0, at, measured, n }
// (in the simulator "simBrake": its braking says nothing about the real drone's):
  // nav/avoid.js stops by it, and the real drone's 0.3 m/s limit until then becomes AVOID.realMeasured. A fit no whoop
  // makes (braking under MISSION.brakeMin m/s², a reaction over MISSION.brakeReact s) is refused and the settings kept;
  // until MISSION.brakeAgree measurements in a row agree (within 30%), the braking saved is no harder than the simulator's
  // (AVOID.decel, 0.7 m/s²). The runs' spread isn't checked: the simulator's own vary by about 80%.
  async calibrate() {
    if (!this.full) throw new Stop("Measuring my braking needs full autonomy (Settings → Autonomy).");
    const P0 = this.loc.pose(), room = P0.room ?? this.map.roomAt(P0.x, P0.y)?.id, c = room && roomCenter(this.map, room, { alt: this.o.alt, sigma: this.planSigma() });
    if (!c) throw new Stop("I need to be in a room on the map to measure my braking.");
    // along a run with room to spare either side (one grazing a sofa slows the drone as it drifts a few cm toward it), from
    // the spot near the room's middle (10 cm grid, 32 directions) with the longest, itself 10 cm clear of that margin; where
    // the drone stops a few cm off it, the same corridor with that much less margin (from the middle 5 cm made 4 m of run 0.4)
    // a run's free length at every height the drone may be at (the speed cap looks at them all: a corridor clear at 1 m only
    // stopped a run under a shelf halfway)
    const fl = (p) => this.map.floorAt(p.x, p.y) ?? 0, zs = (p) => [-0.15, 0, 0.15].map((dz) => clamp(p.z + dz, fl(p) + 0.3, fl(p) + this.map.o.bands.at(-1)));
    const runOf = (p, dir, need) => Math.min(...zs(p).map((z) => freeRun(this.map, { ...p, z }, dir, need)));
    const runsAt = (p, need) => Array.from({ length: 32 }, (_, i) => (i * Math.PI) / 16).map((yaw) => ({ yaw, d: runOf(p, [Math.cos(yaw), Math.sin(yaw)], need) })).reduce((a, b) => (b.d > a.d ? b : a));
    const margin = this.map.o.droneRadius + 2 * Math.max(P0.sigma, 0.03) + 0.25, at = (x, y) => ({ x, y, z: (this.map.floorAt(x, y) ?? 0) + this.o.alt });
    let spot = { ...at(...c), d: -1, yaw: null };
    for (let i = -6; i <= 6; i++)
      for (let j = -6; j <= 6; j++) {
        const q = at(c[0] + 0.1 * i, c[1] + 0.1 * j);
        if (this.map.roomAt(q.x, q.y)?.id !== room || !(this.map.clearance(q.x, q.y, q.z) >= margin + 0.1)) continue;
        const b = runsAt(q, margin);
        if (b.d > spot.d + 0.05) spot = { ...q, ...b };
      }
    const runs = [], delay = this.ctl.videoDelay / 1000, need = () => this.map.o.droneRadius + 2 * this.loc.pose().sigma + 0.25, dir = (yaw) => [Math.cos(yaw), Math.sin(yaw)];
    let yaw0 = null, failed = "";
    for (let k = 0; runs.length < this.o.brakeRuns && k < this.o.brakeRuns + 2; k++) { // a run that didn't show a stop is run again
      // the way back along the corridor on even runs, when it has room from where the drone stopped; else from the spot again
      let yaw = yaw0 == null ? null : k % 2 ? yaw0 + Math.PI : yaw0, d = yaw == null ? 0 : runOf(this.loc.pose(), dir(yaw), need());
      if (d < this.o.brakeRoom) {
        await this.flyTo([spot.x, spot.y], { label: "Flying to an open spot to measure my braking" });
        const P = this.loc.pose(), off = Math.hypot(P.x - spot.x, P.y - spot.y), along = (y) => runOf(P, dir(y), need() - (spot.yaw != null && y === spot.yaw && off < 0.2 ? off : 0));
        const best = runsAt(P, need());
        yaw0 ??= spot.yaw != null && off < 0.2 && along(spot.yaw) > best.d ? spot.yaw : best.yaw;
        [yaw, d] = along(yaw0) >= this.o.brakeRoom ? [yaw0, along(yaw0)] : [best.yaw, best.d]; // off the corridor a little: the best way from here
        if (d < this.o.brakeRoom) {
          if (runs.length >= 2) break;
          throw new Stop(`There isn't room here to measure my braking (${d.toFixed(1)} m clear at most; I need ${this.o.brakeRoom} m).`);
        }
      }
      await this.face(yaw);
      this.status("calibrate", `Measuring my braking: run ${runs.length + 1} of ${this.o.brakeRuns}, a short run, then a stop.`);
      const r = await this.fly(new BrakeTest({ localizer: this.loc, yaw, run: Math.min(2, d - 1.0) }), { soft: true });
      if (r.ok) runs.push({ ...r.data, react: Math.max(0.1, r.data.lag - delay) });
      else failed = r.text;
    }
    if (runs.length < 2) throw new Stop(`I couldn't measure my braking: ${failed || "too few runs"}`);
    // one reaction for all (the longest), each run's braking after it (a run that stopped within the reaction says nothing
    // about braking); the lowest braking: the model then stops no shorter than any run did
    const react = Math.max(...runs.map((q) => q.react)), T = delay + react, v0 = runs.reduce((a, q) => a + q.v0, 0) / runs.length;
    const fits = runs.filter((q) => q.travel - q.v0 * T > 0.02).map((q) => q.v0 ** 2 / (2 * (q.travel - q.v0 * T))), lo = fits.length ? Math.min(...fits) : AVOID.decel;
    const list = runs.map((q) => `${q.travel.toFixed(2)} m from ${q.v0.toFixed(2)} m/s`).join(", ");
    if (!(lo >= this.o.brakeMin && react <= this.o.brakeReact)) // nothing a whoop does: a measurement gone wrong
      throw new Stop(`My braking runs didn't make sense (${list}), so I kept the careful settings I had. Try again in a larger, open room, away from drafts.`);
    const key = brakeKey(this.settings), was = this.settings?.get?.(key), agree = was?.measured > 0 && Math.abs(was.measured - lo) <= 0.3 * Math.min(was.measured, lo), n = agree ? (was.n ?? 1) + 1 : 1;
    const decel = n >= this.o.brakeAgree ? lo : Math.min(lo, AVOID.decel);
    this.settings?.set?.(key, { decel: +decel.toFixed(2), react: +react.toFixed(2), v0: +v0.toFixed(2), measured: +lo.toFixed(2), n, at: Date.now() });
    const stop = stopDistance(v0, T, decel);
    return `Measured my braking over ${runs.length} runs (${list} after the stop command, as my position estimate saw it): from ${v0.toFixed(2)} m/s I now count on stopping within ${stop.toFixed(2)} m. Saved the most careful fit${decel < lo ? ` (braking no harder than ${AVOID.decel} m/s² until ${this.o.brakeAgree} measurements agree; this is ${n === 1 ? "the first" : `number ${n}`})` : ""}: I'll stop by that from now on${key === "brake" ? `, and the real drone may fly up to ${AVOID.realMeasured} m/s` : " in the simulator (the real drone measures its own)"}.`;
  }

  // ---- for the agent ----

  // One line about where we are: room, nearby landmark, the way home, how sure, battery.
  whereAmI() {
    const P = this.loc.pose(), h = this.house?.home, b = this.battery();
    if (P.status === "lost") return `I don't know where I am right now${this.loc.flown && !this.ctl.isFlying() ? `: put me on the home pad and press "The drone is on its home pad"` : ""}.`;
    const bits = [`In ${theRoom(this.roomName(P.room))}`];
    const near = this.nearby([P.x, P.y], 1.2);
    if (near) bits.push(`near the ${near}`);
    if (h) bits.push(`${Math.hypot(h.x - P.x, h.y - P.y).toFixed(1)} m from the home pad`);
    bits.push(`position good to about ${Math.max(0.05, P.sigma).toFixed(2)} m (${P.source})`);
    if (P.z != null && this.map.floorAt(P.x, P.y) != null) bits.push(`${(P.z - this.map.floorAt(P.x, P.y)).toFixed(1)} m up`);
    if (b) bits.push(`battery ${b.vbat.toFixed(1)} V, about ${Math.round(b.secondsLeft)} s of flying left`);
    return `${bits.join(", ")}.`;
  }
}

