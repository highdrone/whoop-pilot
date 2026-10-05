// Wave C integration check (docs/HOME-DRONE.md, "Wave C contracts"): the whole stack wired as main.js will wire it, on the
// user's real capture served by tools/whoop.mjs, in real time (timers, as the app's loop would run them with the window
// visible): the 3D voxel map (built, or &cache=1 loaded from OPFS) on the HomeMap; the simulator flying the house in the
// splat twin with the O4's blur, noise, compression, OSD and a lens error; the localizer "fused" with splat fixes
// (SplatLocalizer, its position database built on the ground); live depth (Depth Anything V2 Small) into avoidance, the
// change detector and the voxels; the flight memory (IndexedDB) with Claude's change and detection checks (a fake Claude:
// no key, no network; it answers from the scene's truth); and the mission runner flying patrols in full auto until
// &seconds of flight. Mid-flight a box goes onto the path ahead and a doorway on a later leg is closed; a person walks
// about. Measured: localization error against the truth, the control loop's tick gaps, video and simulator rates, each
// model's time, what the change detector reported and how it was handled (memory, temporary obstacle, a new path), the
// memory's trail and sightings, and the closest the drone came to the box, the door, the person and anything mapped.
// &scenario=sealed (wave C2, blocker 12): the drone flies to &from= (default the Living room), then every doorway into
// &room= (default Room 3) is closed (the scan has them open), it looks into the sealed room, then patrols: the closed
// doorways must be seen (door leaves or door-closed changes), nothing touched, and both summaries must say the room
// couldn't be checked. Also measured: whether σ covers the error (NEES as rates: within 2.5σ, over 3σ, over 0.3 m while "ok";
// on the ground, between fixes and with fixes coming), the longest gap between vision fixes, how long each safety reason
// held, and the temporary obstacles near every contact. &load= (the shell's load average, recorded with the result: busy
// machines make real-time numbers pessimistic); &claudeConf= (the fake Claude's confidence in "no change", default 0.7 as in
// C1: return home passes such a change only from 0.8); &trace=1 (every control tick in the air: the setpoint through each
// safety layer, the truth, the estimate, the controller's own height); &askfix=0 (vision relocalizes only when lost). A run
// whose simulator camera falls back to the simple one, or with no vision fix in its first 10 s of flight, stops and fails.
// Fix 3: the height in the air (estimate against the truth: p95 under 0.12 m, 97% within 2.5 σz; the vision fixes' own
// height error at their frame's time, fixes[].zerr), the trace's throttle, hover estimate, true climb rate and flow quality,
// and the door readings (nav/avoid.js leafLog: every doorway read closed or seen through, scene.doorReadings).
// window.result has it all; &save= gets c-int-*.png and c-int-result.json (&tag= prefixes the names).
import { importCapture } from "../app/js/house/import.js";
import { houseSource } from "../app/js/ui/housepanel.js";
import { openStore } from "../app/js/house/store.js";
import { houseFrame } from "../app/js/house/frames.js";
import { buildVoxels, voxelCentres, UNKNOWN, FREE, OCCUPIED, FLAG } from "../app/js/house/voxels.js";
import { plan } from "../app/js/house/planner.js";
import { Twin } from "../app/js/twin/twin.js";
import { Simulator } from "../app/js/sim/simulator.js";
import { makeHouseWorld } from "../app/js/sim/twin-world.js";
import { augmentOptions, LENS_ERROR } from "../app/js/sim/augment.js";
import { Perception, simSource } from "../app/js/perception.js";
import { FlightController } from "../app/js/controller.js";
import { Localizer } from "../app/js/nav/localizer.js";
import { SplatLocalizer } from "../app/js/nav/splatloc.js";
import { Safety } from "../app/js/safety.js";
import { staticClearance } from "../app/js/nav/avoid.js";
import { ChangeDetector } from "../app/js/nav/changes.js";
import { LiveDepth, mapDepth } from "../app/js/vision/depth.js";
import { HouseMemory } from "../app/js/memory/memory.js";
import { Claude } from "../app/js/ai/claude.js";
import { Inspector } from "../app/js/ai/inspect.js";
import { Alerts } from "../app/js/alerts.js";
import { MissionRunner } from "../app/js/missions.js";
import { ToolBox } from "../app/js/tools.js";
import { DEFAULTS } from "../app/js/settings.js";
import { wrapAngle } from "../app/js/util.js";

const $ = (id) => document.getElementById(id);
const q = new URLSearchParams(location.search), SAVE = q.get("save"), TAG = q.get("tag") ?? "c-int", SCENARIO = q.get("scenario") ?? "patrol";
const lines = [], result = (window.result = { done: false, checks: {}, progress: "setup" });
const log = (...a) => (lines.push(a.map((x) => (typeof x === "string" ? x : JSON.stringify(x))).join(" ")), ($("log").textContent = lines.slice(-400).join("\n")));
const r2 = (v) => (Number.isFinite(v) ? +v.toFixed(2) : null), r3 = (v) => (Number.isFinite(v) ? +v.toFixed(3) : null);
const quant = (v, f) => { const a = v.filter(Number.isFinite).sort((x, y) => x - y); return a.length ? r3(a[Math.min(a.length - 1, Math.floor(f * a.length))]) : null; };
const stats = (v) => ({ median: quant(v, 0.5), p95: quant(v, 0.95), max: v.length ? r3(Math.max(...v.filter(Number.isFinite))) : null, n: v.length });
function check(name, ok, detail) {
  result.checks[name] = { ok, detail };
  const tr = $("checks").insertRow();
  tr.insertCell().textContent = ok ? "PASS" : "FAIL";
  tr.cells[0].className = ok ? "pass" : "fail";
  tr.insertCell().textContent = name;
  tr.insertCell().textContent = detail;
}
const upload = (name, body) => SAVE && fetch(`${SAVE}/save?name=${name.replace(/^c-int/, TAG)}`, { method: "POST", body }).catch((e) => log("save failed", e.message));
const savePng = (canvas, name) => new Promise((r) => canvas.toBlob((b) => r(upload(name, b)), "image/png"));
let lastProgress = 0;
const progress = (text, force = false) => {
  result.progress = text;
  if (force || performance.now() - lastProgress > 5000) (lastProgress = performance.now()), upload("c-int-progress.json", JSON.stringify({ progress: text, scenario: SCENARIO, t: Date.now() }));
};
for (const ev of ["error", "unhandledrejection"]) addEventListener(ev, (e) => log("PAGE ERROR", String(e.reason?.stack ?? e.error?.stack ?? e.message)));
const longTasks = [];
try { new PerformanceObserver((l) => l.getEntries().forEach((e) => longTasks.push({ t: e.startTime, ms: e.duration }))).observe({ type: "longtask", buffered: false }); } catch {}

// ---------------------------------------------------------------- the house, its 3D map and the twin

const wallNow = performance.now.bind(performance); // the wall clock (performance.now becomes the simulation's in flight)
const T0 = performance.now();
const projects = (await (await fetch("/house-files/")).json()).projects ?? [];
const project = projects.find((p) => p.id === q.get("capture")) ?? projects.find((p) => p.ready);
if (!project) throw new Error("no capture in /house-files/");
const imp = await importCapture(houseSource(project));
const { house, map } = imp;
const splat = { name: imp.splat.name, bytes: new Uint8Array(imp.splat.bytes).slice().buffer };
log("house", house.name, house.rooms.map((r) => r.name), "home", house.home, `import ${Math.round(performance.now() - T0)} ms`);
const store = await openStore();
const twin = await Twin.create({ splat: new Blob([splat.bytes]), house });
let vox = q.get("cache") ? await store.loadVoxels(house) : null, voxBuild = null;
if (vox) await vox.refresh();
else {
  const t = performance.now(), centres = await voxelCentres(splat.bytes, splat.name, houseFrame({ f: house.frame.f, Yf: house.frame.Yf }));
  vox = await buildVoxels({ house, map, centres, twin, onProgress: (p) => p.text && progress(`3D map: ${p.text}`) });
  voxBuild = { ms: Math.round(performance.now() - t), stats: vox.stats() };
  await store.saveVoxels(house, vox).catch((e) => log("saveVoxels", e.message));
}
map.setVoxels(vox);
result.setup = { house: house.name, rooms: house.rooms.map((r) => r.name), doors: house.doors.length, voxels: vox.stats(), voxBuild, cached: !voxBuild, scenario: SCENARIO, load: q.get("load"), href: location.search };
log("voxels", result.setup.voxels.grid, `free ${result.setup.voxels.free_m3} m3, unknown ${result.setup.voxels.unknown_m3} m3`, voxBuild ? `built in ${voxBuild.ms} ms` : "loaded");

// ---------------------------------------------------------------- the app's parts, as main.js wires them

// &rate=0.5: the simulated world runs at most at half the wall clock's speed (the GPU's work, which takes wall time, then
// keeps up as on a machine twice as fast); 1: real time
const RATE = +(q.get("rate") ?? 1);
const DET = q.get("det") ?? "detector", SEED = +(q.get("seed") ?? 11), FLIGHT_S = +(q.get("seconds") ?? 90);
const values = { ...DEFAULTS, mode: "sim", autonomy: "full", videoDelay: 100, hfov: 127, uptilt: 20, locSource: "fused", simDetections: DET, simLoc: "vision", simAugment: true,
  apiKey: "fake-key-for-the-check", aiVision: "on", aiBudget: 0.5, avoid: true, patrolAlerts: true };
const listeners = new Set();
const settings = { get: (k) => values[k], all: () => values, set: (k, v) => { values[k] = v; listeners.forEach((f) => f({ key: k, value: v })); }, on: (e, f) => (listeners.add(f), () => listeners.delete(f)) };

const AUG = augmentOptions({ lensError: LENS_ERROR });
const sim = new Simulator({ house, map, twin, detections: DET, augment: AUG, seed: SEED, uptilt: values.uptilt, hfov: values.hfov });
const PERSON = q.get("person") !== "0";
const CLAUDE_CONF = +(q.get("claudeConf") ?? 0.7); // the fake Claude's confidence in "no change"
result.setup.claudeConf = CLAUDE_CONF;
result.setup.askfix = q.get("askfix") !== "0";
sim.useWorld(makeHouseWorld(house, map, { seed: SEED, cast: PERSON ? [{ id: "person1", kind: "person" }] : [] }), twin);
sim.videoDelay = values.videoDelay;
// a run that isn't what it says stops at once, failing (the verifier's seed 11 with a person: the simulator's splat camera
// fell back to the simple one, 0 vision fixes, and the numbers described a flight without vision localization)
let invalid = null;
const invalidate = (why) => {
  if (invalid) return;
  [invalid] = [why, log("RUN INVALID:", why)];
  try { missions.stop(`the check stopped: ${why}`); } catch {} // (before the runner is made: nothing flies yet)
};
sim.on("error", (m) => (log("sim error:", m), /splat camera failed/i.test(m) && invalidate(`the simulator's splat camera fell back to the simple one (${m})`)));
const watch = { contacts: 0, hardest: 0 };
const collide = sim.world.collide.bind(sim.world);
watch.log = [];
sim.world.collide = (body, rad, h) => {
  const v = collide(body, rad, h);
  if (v > 0 && body === sim.drone) {
    watch.contacts++;
    watch.hardest = Math.max(watch.hardest, v);
    const L = watch.log.at(-1), t = r2(S.lastT ?? 0);
    if (!L || t - L.t1 > 0.5) watch.log.push({ t0: t, t1: t, x: r2(body.x), y: r2(body.y), z: r2(body.z - body.floorZ), v: r2(v), steps: 1, behavior: ctl.behavior?.label ?? null, why: safety.status().reason || null,
      pose: { ...((p) => ({ x: r2(p.x), y: r2(p.y), z: r2(p.z - body.floorZ), sigma: r2(p.sigma), status: p.status }))(loc.pose()) }, fix: Math.round(loc.fixQuality().visionAge),
      temps: [...map.temps.values()].filter((m) => Math.hypot((m.x ?? m.polygon?.[0]?.[0]) - body.x, (m.y ?? m.polygon?.[0]?.[1]) - body.y) < 2).map((m) => ({ id: m.id, kind: m.kind, source: m.source ?? null, x: r2(m.x), y: r2(m.y), r: r2(m.r) })) });
    else Object.assign(L, { t1: t, steps: L.steps + 1, v: Math.max(L.v, r2(v)) });
  }
  return v;
};

const perception = new Perception();
perception.geometry = { hfov: values.hfov, uptilt: values.uptilt };
perception.on("status", (s) => s.text && !s.progress && log("perception:", s.text));
perception.on("error", (m) => log("perception error:", m));
perception.setSource(simSource(sim));
const ctl = new FlightController({ settings, perception });
ctl.attach(sim.radio, "sim");
ctl.on("pilot-request", ({ kind }) => sim.pilotRequest(kind));

const loc = new Localizer({ ctl, map, house, settings });
loc.setSource("fused");
loc.expectVision = true; // as ui/session.js sets it while vision localization runs: a flight whose tracking never locks on counts from its take-off
const memory = await HouseMemory.open(house.id, { world: "sim" });
await memory.clear();
memory.setHouse({ house, map, vox, save: null }); // the simulator's memory never edits or saves the house
memory.on("error", (m) => log("memory error:", m));

// A fake Claude: the change and detection checks' requests answered from the scene's truth (what was really added where).
const truth = { box: null, door: null }, personLog = []; // [Date.now(), x, y] of the person, 10 Hz
const asked = [], checking = [];
const fakeFetch = async (url, init) => {
  const body = JSON.parse(init.body), sys = body.system?.[0]?.text ?? body.system ?? "", content = body.messages.at(-1).content;
  let answer;
  if (/double-check a small indoor drone's object detector/.test(sys)) // a person is there only if the scene has one within 6 m
    answer = sim.world.actors.list.some((a) => a.kind === "person" && Math.hypot(a.x - sim.drone.x, a.y - sim.drone.y) < 6)
      ? { present: true, actual: "a person", confidence: 0.9, why: "a person stands there" } : { present: false, actual: "a picture on the wall", confidence: 0.85, why: "no one is there" };
  else {
    // the person where the evidence was taken: within 4 s (wall) of the change's report, the person's closest approach to it
    const c = checking.shift(), near = (x, y) => Math.min(Infinity, ...personLog.filter(([t]) => Math.abs(t - (c?.t ?? 0)) < 4000).map(([, px, py]) => Math.hypot(px - x, py - y)));
    const shut = truth.door?.sealed?.map((id) => map.doors.find((q) => q.id === id)) ?? [], nearShut = c && shut.some((q) => toSeg(c, q.a, q.b) < 0.8);
    const nearBox = c && truth.box && Math.hypot(c.x - truth.box.x, c.y - truth.box.y) < 1.2, door = (c?.door && (c.door === truth.door?.id || truth.door?.sealed?.includes(c.door))) || (c?.kind === "door-closed" && nearShut);
    answer = door ? { verdict: "real-change", kind: "door-closed", what: "a closed door", confidence: 0.9, why: "the doorway is shut in the live picture" }
      : nearBox && c.kind === "obstacle" ? { verdict: "real-change", kind: "obstacle", what: "a tall cardboard box", confidence: 0.9, why: "a box stands where the scan has open floor" }
      : nearShut ? { verdict: "real-change", kind: "obstacle", what: "a closed door", confidence: 0.9, why: "a door is shut across the opening" }
      : c && near(c.x, c.y) < 1.0 ? { verdict: "person-or-pet", kind: "obstacle", what: "a person", confidence: 0.92, why: "a person stands there" }
      : { verdict: "no-change", kind: "none", what: "nothing new", confidence: CLAUDE_CONF, why: "the difference is the scan's smear" }; // &claudeConf (C1: 0.7, under return home's 0.8 to pass)
    asked.push({ t: performance.now(), kind: c?.kind, x: r2(c?.x), y: r2(c?.y), verdict: answer.verdict, images: content.filter((b) => b.type === "image").length, kb: Math.round(init.body.length / 1024) });
  }
  const usage = { input_tokens: 1400, output_tokens: 120, cache_creation_input_tokens: 0, cache_read_input_tokens: 0 };
  return new Response(JSON.stringify({ id: `msg_${asked.length}`, type: "message", role: "assistant", model: body.model, stop_reason: "end_turn", content: [{ type: "text", text: JSON.stringify(answer) }], usage }),
    { status: 200, headers: { "content-type": "application/json" } });
};
const claude = new Claude({ settings, fetch: fakeFetch });
const inspector = new Inspector({ claude, watch: true });
const checkChange = inspector.checkChange.bind(inspector);
inspector.checkChange = (c, o) => (checking.push(c), checkChange(c, o));
inspector.setMemory(memory);
const alerts = new Alerts({ settings, speak: null, memory, inspector, fetch: () => Promise.reject(new Error("no network in the check")) });

const safety = new Safety({ map, localizer: loc, settings, house, alerts });
ctl.safety = safety;
// &askfix=1 (the default): until nav/splatloc.js reads localizer.needFix() (the safety layer's hold asks for a relocalization
// before "lost"), it is shown a lost pose while one is asked for, so it relocalizes as it will; &askfix=0: as it is
const ASKFIX = q.get("askfix") !== "0", splatLoc = ASKFIX ? new Proxy(loc, { get: (o, k) => (k === "pose" ? () => ((p) => (o.needFix?.() && !p.conflict && p.status !== "lost" ? { ...p, status: "lost" } : p))(o.pose())
  : typeof o[k] === "function" ? o[k].bind(o) : o[k]) }) : loc;
const splatloc = new SplatLocalizer({ localizer: splatLoc, ctl, settings, house, map });
splatloc.store = store;
splatloc.setTwin(twin);
splatloc.on("status", ({ level, text, progress: p }) => p == null && log(`splatloc ${level}:`, text));
splatloc.attach(perception);
const changes = new ChangeDetector({ map, house, memory, avoid: safety.avoid, render: (pose, view) => twin.pixels(pose, view) });
const depth = new LiveDepth({ perception, localizer: loc, ctl, settings, twin, calib: null, osd: splatloc.osd });
const timing = { avoid: [], changes: [], integrate: [], frames: 0, integrated: { rays: 0, free: 0, hits: 0, cut: 0, skipped: {} } };
depth.on("status", ({ text, why }) => (safety.avoid.setDepthStatus(why ?? ""), text && log("depth:", text)));
depth.on("error", (m) => log("depth error:", m));
depth.on("depth", (f) => {
  if (ctl.safety !== safety) return;
  timing.frames++;
  vox.markSeen?.(f.pose, f.lens, Date.now(), { width: f.width, height: f.height, quality: { age: performance.now() - f.t } }); // what the camera saw (as main.js does)
  let t = wallNow();
  safety.avoid.ingest(f);
  timing.avoid.push(wallNow() - t);
  t = wallNow();
  changes.ingest(f);
  timing.changes.push(wallNow() - t);
  t = wallNow();
  if (q.get("integrate") === "0") return; // &integrate=0: the 3D map as built (no flight evidence)
  const r = vox.integrate(f.pose, { ...f, depth: mapDepth(f) }, f.lens, { t: f.t });
  timing.integrate.push(wallNow() - t);
  const I = timing.integrated;
  if (r.skipped) I.skipped[r.skipped] = (I.skipped[r.skipped] ?? 0) + 1;
  else Object.assign(I, { rays: I.rays + r.rays, free: I.free + r.free, hits: I.hits + r.hits, cut: I.cut + (r.cut ? 1 : 0) });
});
// &trace=1: every control tick in the air, what each layer made of the behavior's setpoint (body frame vx, vy, vz in; the
// height keep's vz; the caps; the geofence's push in H; out), with the truth, the estimate and the controller's own height
const TRACE = q.get("trace") === "1", trace = (window.icTrace = []);
if (TRACE) {
  let cur = null;
  const wrap = (obj, fn, after) => { const f = obj[fn].bind(obj); obj[fn] = (...a) => { const out = f(...a); after(a, out); return out; }; };
  const sp3 = (v) => [r2(v?.vx), r2(v?.vy), r2(v?.vz)], toH = (v, yaw) => [r2(Math.cos(yaw) * (v?.vx ?? 0) + Math.sin(yaw) * (v?.vy ?? 0)), r2(Math.sin(yaw) * (v?.vx ?? 0) - Math.cos(yaw) * (v?.vy ?? 0))];
  wrap(safety, "keepHeight", (a, out) => cur && (cur.kh = r2(out.vz), (cur.zHold = r2(safety.zHold)), (cur.zI = r2(safety.zI))));
  wrap(safety.avoid, "limit", ([sp], out) => cur && Object.assign(cur, { lim: sp3(sp), cap: r2(safety.avoid.last.cap), capWhy: safety.avoid.last.why || null, vert: safety.avoid.last.vertical || null, capped: sp3(out) }));
  wrap(safety, "geofence", ([sp, p], out) => cur && Object.assign(cur, { push: ((a, b) => [r2(b[0] - a[0]), r2(b[1] - a[1])])(toH(sp, p.yaw), toH(out, p.yaw)), I: safety.pushed.map(r2) }));
  const filter = safety.filter.bind(safety);
  safety.filter = (sp, st) => {
    const d = sim.drone;
    cur = d.airborne ? {} : null;
    const out = filter(sp, st);
    if (cur) {
      const p = loc.pose(), f = d.floorZ ?? 0;
      trace.push({ t: r2(S.lastT), b: ctl.behavior?.label ?? null, x: r2(d.x), y: r2(d.y), z: r2(d.z - f), ex: r2(p.x), ey: r2(p.y), ez: r2(p.z - f), zs: r2(p.zSigma), s: r2(p.sigma), st: p.status[0],
        age: Math.round(loc.fixQuality().visionAge), cz: r2(ctl.est.z), cvz: r2(ctl.est.vz), tvz: r2(d.vz), lvz: r2(loc.height?.().vz), lead: ctl.zLoc ? 1 : 0, thr: r3(ctl.out.thr), hov: r3(ctl.hover.estimate(ctl.tel?.vbat)), fq: r2(ctl.est.flowQ), tilt: r3(Math.cos(ctl.est.pitchAngle) * Math.cos(ctl.est.rollAngle)), zRef: r2(ctl.zRef), yaw: r2(d.yaw), h: r2(ctl.est.heading), in: sp3(sp), out: sp3(out), why: safety.reason || null, ...cur });
    }
    cur = null;
    return out;
  };
}
const rejects = {}, fixLog = [];
splatloc.on("reject", ({ reason }) => { const k = String(reason).replace(/[\d.]+/g, "#"); rejects[k] = (rejects[k] ?? 0) + 1; });
// the true height when a frame was taken (the fix's z against it: performance.now ms of the simulation, 10 s kept)
const zHist = [], zAt = (t) => { let i = zHist.length - 1; while (i > 0 && zHist[i][0] > t) i--; return zHist[i]?.[1] ?? sim.drone.z; };
splatloc.on("fix", (f) => fixLog.push({ t: r2(S.lastT ?? 0), inliers: f.inliers, reloc: !!f.reloc, weak: !!f.weak, err: r3(Math.hypot(f.pose.x - sim.drone.x, f.pose.y - sim.drone.y)),
  ...(Number.isFinite(f.pose.z) && sim.drone.airborne && { zerr: r3(f.pose.z - zAt(f.t ?? performance.now())), ft: r3(((f.t ?? performance.now()) - S.t0) / 1000), z: r3(f.pose.z - (sim.drone.floorZ ?? 0)), used: loc.fixes.last?.used ?? null }) }));
const missions = new MissionRunner({ ctl, map, house, localizer: loc, perception, alerts, settings, memory, splat: splatloc, depth });
window.ic = { map, vox, loc, safety, sim, missions, depth, changes, memory, splatloc, ctl }; // for looking around from the console
const tools = new ToolBox({ ctl, perception, settings, speak: () => {}, alerts });
tools.setHouse({ house, map, missions, localizer: loc, memory, vox, splat: splatloc });
tools.setAI({ claude });

// ---------------------------------------------------------------- on the ground: models, the position database, the pad

progress("loading the models and building the position database", true);
const tm = performance.now();
const [depthInfo, db] = await Promise.all([
  depth.start((l, n) => progress(`depth model ${Math.round((100 * l) / n)}%`)),
  splatloc.ensureDb({ onProgress: ({ done, total }) => progress(`position database ${done}/${total}`) }),
  DET === "detector" ? new Promise((res) => { const t = setInterval(() => (perception.detector.ready || perception.detectorFailed) && (clearInterval(t), res()), 200); }) : null,
]);
result.setup.models = { depth: depthInfo, db: { keyframes: db?.n ?? null, ms: db?.meta?.ms ?? null }, detector: DET === "detector" ? perception.detector.backend ?? "failed" : "truth",
  features: splatloc.features?.info ?? null, ms: Math.round(performance.now() - tm) };
log("models", result.setup.models);

// the loop: as main.js's frame() (simulator step, its camera about every 30 ms, perception) and the controller's 33 ms tick,
// clocked by a worker's timer (a hidden page's own timers may be held to one a second; a worker's are not). One clock for
// the page: performance.now() is the simulator's time, which follows the wall clock but steps at most 50 ms at a time
// (sim.step's cap), so a page slowed down by the machine slows the simulated world with it instead of tearing the
// controller's time from the drone's. GPU work (twin renders, models) still takes wall time: on a slow machine it looks
// relatively faster. Wall-clock gaps of the loop and the ticks are measured as well (wall.*), and the time dilation.
const realNow = wallNow;
// waits on the wall clock by message ticks: a hidden page's timers are held to one a minute after 5 minutes (a 10 s wait
// between flights took 50 min in a hidden pane)
const pause = (ms) => new Promise((res) => { const end = realNow() + ms, c = new MessageChannel(); c.port1.onmessage = () => (realNow() >= end ? (c.port1.close(), res()) : c.port2.postMessage(0)); c.port2.postMessage(0); });
let vnow = realNow(), lastReal = vnow, lastRender = 0, lastTick = 0, lastTickReal = vnow, running = true;
// within a task the simulation's clock runs with the wall clock, at most 50 ms past the last step: monotonic, and short
// durations (budgets, model times) are measured right
performance.now = () => vnow + Math.min((realNow() - lastReal) * RATE, 50); // (with &rate, module timings are in simulated ms)
const wall = { loop: [], tick: [], t0: vnow, v0: vnow };
// where the page's own thread goes (wall ms per part)
const busy = {}, timeIt = (name, obj, fn) => {
  const f = obj[fn].bind(obj);
  obj[fn] = (...a) => { const t = realNow(); try { return f(...a); } finally { busy[name] = (busy[name] ?? 0) + realNow() - t; } };
};
timeIt("sim.step", sim, "step");
timeIt("sim.renderCamera", sim, "renderCamera");
timeIt("sim.queueFrame", sim, "queueFrame");
timeIt("perception.process (+ detector)", perception, "process");
timeIt("ctl.tick (behaviors, safety, avoid caps)", ctl, "tick");
timeIt("splatloc.onFrame", splatloc, "onFrame");
timeIt("depth.frame (sync part)", depth, "frame");
// the loop's clock: a MessageChannel ping on this thread, stepping every 16 ms of wall time (a hidden page holds its own
// timers, and even a worker's, to a few a second; message tasks run as they come)
const ch = new MessageChannel();
ch.port1.onmessage = () => {
  if (!running) return;
  if (realNow() - lastReal >= 16) step();
  ch.port2.postMessage(0);
};
function step() {
  const real = realNow(), dt = Math.min((real - lastReal) * RATE, 50);
  wall.loop.push(real - lastReal);
  vnow += dt;
  lastReal = real;
  sim.step(dt / 1000);
  if (vnow - lastRender >= 30) (sim.renderCamera(), (lastRender = vnow));
  perception.process();
  if (vnow - lastTick >= 30) {
    if (airborne()) wall.tick.push(real - lastTickReal);
    [lastTick, lastTickReal] = [vnow, real];
    ctl.tick();
  }
  const t = realNow();
  sample(vnow);
  busy.sample = (busy.sample ?? 0) + realNow() - t;
}
ch.port2.postMessage(0);
const tickGaps = [], loopGaps = wall.loop;
ctl.on("tick", ({ gap }) => airborne() && Number.isFinite(gap) && tickGaps.push(gap));
const airborne = () => sim.drone.airborne;
sim.prepareForMission({ copilot: false }); // the pilot's switches: armed, AI on (the radio's telemetry says so a moment later)
await pause(1500);
loc.reset({ x: sim.drone.x, y: sim.drone.y, yaw: sim.drone.yaw }); // the pad button

// ---------------------------------------------------------------- measuring

const S = (window.icS = { timeline: [], boxWhy: {}, t0: null, air: 0, lastT: null, err: [], err3: [], zErr: [], yaw: [], sigma: [], status: {}, truth: [], est: [], person: [], fps: [], perFps: [], det: [], splatMs: [], depth: [],
  near: { box: Infinity, door: Infinity, person: Infinity, mapped: Infinity }, paths: [], events: [], shots: [], nees: { n: 0, within: 0, over3: 0, overOk: 0, okN: 0, worstOk: 0, by: {} }, reasons: {}, fixGap: 0, lastFix: null });
loc.on("fix", (f) => { const t = S.lastT; if (t != null && S.lastFix != null && sim.drone.airborne) S.fixGap = Math.max(S.fixGap, t - S.lastFix); S.lastFix = t; if (sim.drone.airborne && f?.source === "splat") S.airFixes = (S.airFixes ?? 0) + 1; });
let nextSample = 0, lastPath = null, plan0 = null;
missions.on("path", ({ path }) => { lastPath = path; S.paths.push({ t: S.lastT, path }); });
missions.on("plan", (p) => (plan0 = p));
missions.on("status", ({ phase, text }) => {
  const d = sim.drone, near = (x, y, r) => [...map.temps.values()].filter((t) => Math.hypot((t.x ?? t.polygon?.[0]?.[0]) - x, (t.y ?? t.polygon?.[0]?.[1]) - y) < r)
    .map((t) => ({ id: t.id, kind: t.kind, source: t.source ?? null, x: r2(t.x), y: r2(t.y), r: r2(t.r), poly: t.polygon?.length ?? 0, zMin: r2(t.zMin), zMax: r2(t.zMax) }));
  const pad = /home pad/.test(text) ? near(house.home.x, house.home.y, 1.5) : undefined, temps = /in the way|blocked|blocks|looks closed/i.test(text) ? near(d.x, d.y, 3) : undefined;
  S.events.push({ t: r2(S.lastT), phase, text, ...(pad && { pad }), ...(temps && { at: [r2(d.x), r2(d.y)], temps }) });
  log(`[${(S.lastT ?? 0).toFixed(1)} s] ${phase}: ${text}`);
});
missions.on("finding", (f) => log("finding", f.label, f.roomName, f.x, f.y));
const reported = [], leaves = [];
safety.avoid.on("temp", (t) => t.added && t.source === "door leaf" && leaves.push({ t: r2(S.lastT), door: t.door }));
changes.on("change", (c) => {
  const err = c.kind === "door-closed" && truth.door ? Math.hypot(c.x - truth.door.mid[0], c.y - truth.door.mid[1]) : truth.box ? Math.hypot(c.x - truth.box.x, c.y - truth.box.y) : null;
  reported.push({ t: r2(S.lastT), kind: c.kind, x: c.x, y: c.y, z: c.z, size: c.size, door: c.door ?? null, memoryId: c.memoryId, err: r3(err) });
  log("CHANGE reported", reported.at(-1));
  shot(`change-${c.kind}`);
});
const memEvents = [];
memory.on("change", (e) => memEvents.push({ t: r2(S.lastT), phase: e.phase, id: e.id, kind: e.kind, status: e.status, onMap: map.temps.has(e.id), by: e.by ?? null }));
memory.on("annotate", (r) => r.claude && memEvents.push({ t: r2(S.lastT), phase: "claude", id: r.id, kind: r.kind, verdict: r.claude.verdict ?? r.claude.status, passing: !!r.passing }));

function shot(tag) {
  const v = sim.video?.canvas;
  if (!v) return;
  const c = $("shot");
  [c.width, c.height] = [v.width, v.height];
  c.getContext("2d").drawImage(v, 0, 0);
  $("shotCap").textContent = `camera at ${(S.lastT ?? 0).toFixed(1)} s: ${tag}`;
  savePng(c, `c-int-${tag}.png`);
  S.shots.push(tag);
}

const toRect = (p, b) => { const c = Math.cos(-b.yaw), s = Math.sin(-b.yaw), dx = p.x - b.x, dy = p.y - b.y, u = c * dx - s * dy, v = s * dx + c * dy;
  return Math.hypot(Math.max(0, Math.abs(u) - b.w / 2), Math.max(0, Math.abs(v) - b.d / 2)); };
const toSeg = (p, a, b) => { const dx = b[0] - a[0], dy = b[1] - a[1], L2 = dx * dx + dy * dy, t = Math.max(0, Math.min(1, ((p.x - a[0]) * dx + (p.y - a[1]) * dy) / L2));
  return Math.hypot(p.x - a[0] - t * dx, p.y - a[1] - t * dy); };

function sample(now) {
  const d = sim.drone;
  if (!d.airborne) return;
  if (S.t0 == null) S.t0 = now;
  const t = (now - S.t0) / 1000;
  if (S.lastT != null) S.air += Math.min(0.1, t - S.lastT);
  zHist.push([now, d.z]);
  while (zHist[0][0] < now - 10000) zHist.shift();
  S.lastT = t;
  if (S.air > 10 && !S.airFixes) invalidate("no vision fix in the first 10 s of flight");
  // closest approaches: every step
  if (truth.box && d.z < truth.box.z + truth.box.h + 0.1) S.near.box = Math.min(S.near.box, toRect(d, truth.box));
  if (truth.door?.a) S.near.door = Math.min(S.near.door, toSeg(d, truth.door.a, truth.door.b));
  for (const [a, b] of truth.door?.segs ?? []) S.near.door = Math.min(S.near.door, toSeg(d, a, b));
  for (const a of sim.world.actors.list) if (a.kind === "person") S.near.person = Math.min(S.near.person, Math.hypot(d.x - a.x, d.y - a.y));
  if (now < nextSample) return;
  nextSample = now + 100;
  const p = loc.pose();
  S.near.mapped = Math.min(S.near.mapped, staticClearance(map, d.x, d.y, d.z));
  S.status[p.status] = (S.status[p.status] ?? 0) + 1;
  if (loc.known && p.status !== "lost") {
    const e = Math.hypot(p.x - d.x, p.y - d.y), N = S.nees;
    S.err.push(e);
    S.err3.push(Math.hypot(p.x - d.x, p.y - d.y, p.z - d.z));
    if (d.airborne) S.zErr.push([Math.abs(p.z - d.z), p.zSigma]);
    S.yaw.push((Math.abs(wrapAngle(p.yaw - d.yaw)) * 180) / Math.PI);
    S.sigma.push(p.sigma);
    // rates, and per phase (on the ground, in the air between fixes (none for 1 s), in the air with fixes coming): the tails
    // come from the gaps, contacts and a drone moved on the ground, not the core σ
    const phase = !d.airborne ? "ground" : loc.fixQuality().visionAge > 1000 ? "gap" : "fixes", B = (N.by[phase] ??= { n: 0, within: 0, over3: 0, overOk: 0 });
    for (const M of [N, B]) [M.n, M.within, M.over3, M.overOk] = [M.n + 1, M.within + (e <= 2.5 * p.sigma), M.over3 + (e > 3 * p.sigma && e > 0.2), M.overOk + (p.status === "ok" && e > 0.3)];
    if (p.status === "ok") [N.okN, N.worstOk] = [N.okN + 1, e > 0.3 ? Math.max(N.worstOk, e) : N.worstOk];
  }
  const why = safety.status().reason;
  if (why) S.reasons[why.replace(/\d+(\.\d+)?/g, "#")] = r2((S.reasons[why.replace(/\d+(\.\d+)?/g, "#")] ?? 0) + 0.1);
  S.truth.push([d.x, d.y]);
  S.est.push([p.x, p.y]);
  const P = sim.world.actors.list.find((a) => a.kind === "person");
  if (P) S.person.push([P.x, P.y]), personLog.push([Date.now(), P.x, P.y]);
  if (S.truth.length % 5 === 0) S.timeline.push({ t: r2(t), x: r2(d.x), y: r2(d.y), z: r2(d.z - d.floorZ), ex: r2(p.x), ey: r2(p.y), ez: r2(p.z - d.floorZ), zs: r3(p.zSigma), err: r3(Math.hypot(p.x - d.x, p.y - d.y)), sigma: r3(p.sigma), status: p.status,
    fixes: loc.fixes.used, contacts: watch.contacts, b: ctl.behavior?.label ?? null, why: safety.status().reason || null, fps: sim.stats.fps, yawRate: r2(d.yawRate ?? 0) });
  if (S.truth.length % 10 === 0) {
    S.fps.push(sim.stats.fps);
    S.perFps.push(perception.fps);
    S.det.push(perception.detector.stats?.inferMs ?? NaN);
    S.splatMs.push(splatloc.stats.ms);
    S.depth.push({ ...depth.stats });
    drawFpv();
    drawMap();
    progress(`flying ${t.toFixed(0)} s (air ${S.air.toFixed(0)} s): error ${S.err.at(-1)?.toFixed(3)} m, σ ${p.sigma.toFixed(3)}, ${safety.status().reason || "ok"}; changes ${reported.length}`);
  }
  scenery(t);
}

// The scene changes: a box onto the path ahead (12 s into the flight), a doorway on a later leg closed (30 s in). The
// sealed scenario closes its doorways before take-off instead.
function scenery(t) {
  const d = sim.drone;
  if (SCENARIO !== "patrol") return;
  if (!truth.box && t > 12 && lastPath?.length > 1) {
    // along the path from the drone's nearest point on it, every 0.1 m from 1.8 to 3.5 m ahead
    let k = 0, best = Infinity, u0 = 0;
    for (let i = 0; i < lastPath.length - 1; i++) {
      const [a, b] = [lastPath[i], lastPath[i + 1]], L = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1e-9;
      const u = Math.max(0, Math.min(1, ((d.x - a[0]) * (b[0] - a[0]) + (d.y - a[1]) * (b[1] - a[1])) / L ** 2)), e = Math.hypot(a[0] + u * (b[0] - a[0]) - d.x, a[1] + u * (b[1] - a[1]) - d.y);
      if (e < best) [best, k, u0] = [e, i, u * L];
    }
    let along = -u0;
    outer: for (let i = k; i < lastPath.length - 1; i++) {
      const [a, b] = [lastPath[i], lastPath[i + 1]], L = Math.hypot(b[0] - a[0], b[1] - a[1]);
      for (let s = 0; s < L; s += 0.1) {
        const at = along + s;
        if (at < 1.8) continue;
        if (at > 3.5) break outer;
        const x = a[0] + ((b[0] - a[0]) * s) / L, y = a[1] + ((b[1] - a[1]) * s) / L, f = map.floorAt(x, y), room = map.roomAt(x, y);
        const why = !room ? "room" : map.doors.some((q) => toSeg({ x, y }, q.a, q.b) < 0.5) ? "door" : Math.hypot(x - d.x, y - d.y) < 1.5 ? "near" : f == null ? "floor"
          : staticClearance(map, x, y, f + 0.7) < 0.3 ? "clearance" : "";
        S.boxWhy[why || "ok"] = (S.boxWhy[why || "ok"] ?? 0) + 1;
        if (why) continue; // in a room, not in a doorway, known free all round it
        const prop = sim.addObstacle({ id: "int-box", x, y, w: 0.5, d: 0.5, h: 1.5, color: "#8a6b4a" });
        truth.box = { ...prop, t: r2(t), room: room.name, onPath: r2(at) };
        log("BOX placed", truth.box);
        setTimeout(() => shot("box-ahead"), 1500);
        break outer;
      }
      along += L;
    }
  }
  if (!truth.door && t > 30) {
    const here = [d.x, d.y], stops = [...(lastPath?.length ? [{ x: lastPath.at(-1)[0], y: lastPath.at(-1)[1] }] : []), ...(plan0?.stops ?? []).slice(1), house.home].filter(Boolean);
    const cands = [];
    for (const s of stops) {
      const goal = s.x != null ? [s.x, s.y] : null, r = goal && plan(map, here, goal, { alt: 1.0 });
      if (r?.ok) for (const dr of [...r.doors].reverse()) cands.push(dr.id);
    }
    const doorOk = (q) => q && q.rooms[1] && q.passable !== false && Math.hypot(d.x - (q.a[0] + q.b[0]) / 2, d.y - (q.a[1] + q.b[1]) / 2) > 2.2;
    const pick = cands.map((id) => map.doors.find((q) => q.id === id)).find(doorOk) ?? map.doors.filter(doorOk).sort((a, b) => toSeg(d, a.a, a.b) - toSeg(d, b.a, b.b))[0];
    if (pick && sim.setDoor(pick.id, false)) {
      truth.door = { id: pick.id, a: pick.a, b: pick.b, mid: [(pick.a[0] + pick.b[0]) / 2, (pick.a[1] + pick.b[1]) / 2], rooms: pick.rooms.map((id) => house.rooms.find((r) => r.id === id)?.name), t: r2(t), onPlan: cands.includes(pick.id) };
      log("DOOR closed", truth.door);
    } else truth.door = { none: true };
  }
}

// ---------------------------------------------------------------- pictures

const ortho = imp.orthophoto?.bytes ? await createImageBitmap(new Blob([imp.orthophoto.bytes], { type: "image/png" })).catch(() => null) : null;
const B = { x0: map.x0, y0: map.y0, x1: map.x0 + map.W * map.o.cell, y1: map.y0 + map.H * map.o.cell };
function view(c) {
  const k = Math.min(c.width / (B.x1 - B.x0), c.height / (B.y1 - B.y0));
  return [(x) => (x - B.x0) * k, (y) => c.height - (y - B.y0) * k, k];
}
function drawFpv() {
  const v = sim.video?.canvas;
  if (v) $("fpv").getContext("2d").drawImage(v, 0, 0, 640, 480);
  $("fpvCap").textContent = `drone camera · sim ${sim.stats.fps} fps · perception ${perception.fps} fps · ${safety.status().reason || "ok"}`;
}
function drawMap() {
  const c = $("map"), g = c.getContext("2d"), [X, Y, k] = view(c);
  g.fillStyle = "#000";
  g.fillRect(0, 0, c.width, c.height);
  const o = house.orthophoto;
  if (ortho && o) (g.globalAlpha = 0.55), g.drawImage(ortho, X(o.x0), Y(o.y0 + o.height * o.res), o.width * o.res * k, o.height * o.res * k), (g.globalAlpha = 1);
  const poly = (pts, stroke, fill, w = 1) => { g.beginPath(); pts.forEach(([x, y], i) => (i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)))); g.closePath(); if (fill) (g.fillStyle = fill), g.fill(); if (stroke) (g.strokeStyle = stroke), (g.lineWidth = w), g.stroke(); };
  const line = (pts, color, w = 1.5, dash = []) => { g.beginPath(); g.setLineDash(dash); pts.forEach(([x, y], i) => (i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)))); g.strokeStyle = color; g.lineWidth = w; g.stroke(); g.setLineDash([]); };
  for (const r of house.rooms) poly(r.outline, "#666", null);
  for (const d of house.doors) line([d.a, d.b], d.id === truth.door?.id ? "#ff3030" : "#5a5", d.id === truth.door?.id ? 5 : 2);
  for (const t of map.temps.values()) {
    const col = t.kind === "person" ? "rgba(200,80,255,0.35)" : t.kind === "change" ? "rgba(255,60,60,0.35)" : "rgba(255,190,40,0.4)";
    if (t.polygon) poly(t.polygon, null, col);
    else (g.beginPath(), g.arc(X(t.x), Y(t.y), t.r * k, 0, 7), (g.fillStyle = col), g.fill());
  }
  if (truth.box && !truth.box.none) {
    const b = truth.box, c2 = Math.cos(b.yaw), s2 = Math.sin(b.yaw), pts = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([u, v]) => [b.x + (u * b.w * c2 - v * b.d * s2) / 2, b.y + (u * b.w * s2 + v * b.d * c2) / 2]);
    poly(pts, "#fff", "#8a6b4a", 1);
  }
  if (lastPath) line(lastPath.map((p) => [p[0], p[1]]), "#4aa3ff", 1.5, [5, 4]);
  line(S.person, "rgba(0,220,255,0.6)", 1);
  line(S.truth, "#fff", 1.5);
  line(S.est, "#3f3", 1.2);
  g.font = "12px ui-monospace";
  for (const r of reported) {
    g.strokeStyle = "#ff0";
    g.lineWidth = 2;
    const x = X(r.x), y = Y(r.y);
    g.beginPath(); g.moveTo(x - 6, y - 6); g.lineTo(x + 6, y + 6); g.moveTo(x + 6, y - 6); g.lineTo(x - 6, y + 6); g.stroke();
    g.fillStyle = "#ff0";
    g.fillText(r.kind, x + 8, y - 6);
  }
  const d = sim.drone;
  g.fillStyle = "#f44";
  g.beginPath(); g.arc(X(d.x), Y(d.y), 5, 0, 7); g.fill();
}
function drawVox() {
  const c = $("vox"), g = c.getContext("2d"), s = vox.slice(1.0, { map }), img = g.createImageData(s.width, s.height);
  for (let r = 0; r < s.height; r++)
    for (let col = 0; col < s.width; col++) {
      const i = r * s.width + col, o = 4 * ((s.height - 1 - r) * s.width + col), st = s.data[i];
      const z = map.floorAt(s.x0 + (col + 0.5) * s.res, s.y0 + (r + 0.5) * s.res), vi = z == null ? -1 : vox.idx(s.x0 + (col + 0.5) * s.res, s.y0 + (r + 0.5) * s.res, z + 1.0);
      const flight = vi >= 0 && vox.flags[vi] & FLAG.FLIGHT;
      const rgb = st === FREE ? (flight ? [120, 230, 120] : [225, 225, 215]) : st === OCCUPIED ? [40, 40, 50] : [120, 120, 120];
      [img.data[o], img.data[o + 1], img.data[o + 2], img.data[o + 3]] = [...rgb, 255];
    }
  const tmp = new OffscreenCanvas(s.width, s.height);
  tmp.getContext("2d").putImageData(img, 0, 0);
  const k = Math.min(c.width / s.width, c.height / s.height);
  g.fillStyle = "#000";
  g.fillRect(0, 0, c.width, c.height);
  g.imageSmoothingEnabled = false;
  g.drawImage(tmp, 0, 0, s.width * k, s.height * k);
}

// ---------------------------------------------------------------- the flights

const flights = [], tf = performance.now(), tfWall = realNow(), whereAmI = [], fly = async (m) => {
  if (invalid) return { ok: false, summary: `not flown: ${invalid}`, findings: [], seconds: 0 };
  progress(`${m.kind} ${m.room ?? m.target ?? ""}`, true);
  const r = await missions.run(m);
  flights.push({ kind: m.kind, ok: r.ok, summary: r.summary, seconds: r.seconds, findings: r.findings.map((f) => ({ label: f.label, room: f.roomName, x: f.x, y: f.y })) });
  log(`${m.kind.toUpperCase()} ${flights.length}: ${r.ok ? "ok" : "not ok"} in ${r.seconds} s: ${r.summary}`);
  return r;
};
result.checklist = missions.preflightCheck().map(({ id, level, text }) => ({ id, level, text }));
setTimeout(async () => whereAmI.push((await tools.call("where_am_i", {})).text), 25000);
const ROOM = house.rooms.find((r) => r.name === (q.get("room") ?? "Room 3")) ?? house.rooms.at(-1), FROM = q.get("from") ?? "Living room", sealed = [];
if (SCENARIO === "sealed") {
  truth.box = { none: true };
  await fly({ kind: "goTo", target: FROM });
  for (const d of map.doors.filter((d) => d.rooms.includes(ROOM.id) && d.rooms[1] && d.passable !== false)) if (sim.setDoor(d.id, false)) sealed.push(d.id);
  truth.door = { sealed, room: ROOM.name, segs: sealed.map((id) => ((d) => [d.a, d.b])(map.doors.find((q) => q.id === id))) };
  log("SEALED", ROOM.name, sealed);
  await fly({ kind: "lookIn", room: ROOM.name });
  for (let w = 0; w < 50 && !sim.drone.airborne; w++) await pause(200);
  await fly({ kind: "patrol" });
  for (let w = 0; w < 50 && sim.drone.airborne; w++) await pause(200);
} else
  while (S.air < FLIGHT_S && flights.length < 3 && realNow() - tfWall < 1800000 && !invalid) {
    const r = await fly({ kind: "patrol" });
    for (let w = 0; w < 50 && sim.drone.airborne; w++) await pause(200);
    if (!r.ok && !r.seconds) break; // refused before flying
  }
await pause(2500);
running = false;
wall.end = [vnow, realNow()];
performance.now = realNow;
drawFpv();
drawMap();
drawVox();

// ---------------------------------------------------------------- results

const memFlights = memory.flights.map((f) => ({ id: f.id, kind: f.kind, mission: f.mission, seconds: f.t1 && r2((f.t1 - f.t0) / 1000), rooms: f.rooms.map((id) => house.rooms.find((r) => r.id === id)?.name), distance: f.distance,
  trail: memory.trail({ flightId: f.id }).length, lost: memory.trail({ flightId: f.id }).filter((s) => s.lost).length, summary: f.summary }));
const changeRecs = memory.changes.map((c) => ({ id: c.id, kind: c.kind, status: c.status, by: c.by ?? null, x: c.x, y: c.y, room: c.room, n: c.n, claude: c.claude && { verdict: c.claude.verdict, confidence: c.claude.confidence, status: c.claude.status }, passing: !!c.passing,
  onMap: map.temps.has(c.id), evidence: !!c.evidence?.live }));
for (const [i, c] of memory.changes.entries()) if (c.evidence?.live) await upload(`c-int-evidence-${i}-${c.kind}-live.jpg`, c.evidence.live), c.evidence.expected && (await upload(`c-int-evidence-${i}-${c.kind}-expected.jpg`, c.evidence.expected));
const boxTime = truth.box?.t, newPaths = boxTime != null ? S.paths.filter((p) => p.t > boxTime) : [];
const clearOf = (p, b) => Math.min(...p.path.map(([x, y]) => toRect({ x, y }, b)));
const replans = truth.box ? newPaths.map((p) => ({ t: r2(p.t), clear: r2(clearOf(p, truth.box)) })) : [];
const tickJitter = tickGaps.map((g) => Math.abs(g - 33.3));
const depthLast = S.depth.at(-1) ?? {};
result.flight = {
  wallSeconds: r2((realNow() - tfWall) / 1000), airSeconds: r2(S.air), flights, invalid,
  loc: { error: stats(S.err), error3d: stats(S.err3), yawDeg: stats(S.yaw), sigma: stats(S.sigma), status: S.status, fixes: loc.fixes.used, rejected: loc.fixes.rejected, conflicts: loc.fixes.conflicts ?? 0,
    fixQuality: loc.fixQuality(), splat: { ...splatloc.stats, recent: undefined, last: undefined },
    // in the air: |estimate - true height| and the share within 2.5 σz; the vision fixes' own height error (at their frame's time)
    height: { error: stats(S.zErr.map(([e]) => e)), within: r3(S.zErr.filter(([e, s]) => e <= 2.5 * s).length / Math.max(1, S.zErr.length)), zSigma: stats(S.zErr.map(([, s]) => s)),
      fixes: stats(fixLog.filter((f) => f.zerr != null).map((f) => Math.abs(f.zerr))), fixBias: quant(fixLog.filter((f) => f.zerr != null).map((f) => f.zerr), 0.5) } },
  control: { tickGapMs: stats(tickGaps), tickJitterMs: stats(tickJitter), wallTickGapMs: stats(wall.tick), loopGapMs: stats(loopGaps), dilation: r3((wall.end[0] - wall.v0) / (wall.end[1] - wall.t0)),
    busyPct: Object.fromEntries(Object.entries({ ...busy, "depth handler (avoid, changes, voxels)": timing.avoid.concat(timing.changes, timing.integrate).reduce((a, b) => a + b, 0) }).map(([k, v]) => [k, r2((100 * v) / (wall.end[1] - wall.t0))])), longTasks: { n: longTasks.length, max: r2(Math.max(0, ...longTasks.map((l) => l.ms))), over100: longTasks.filter((l) => l.ms > 100).length } },
  rates: { simFramesPerWallS: r2(sim.stats.frames / ((wall.end[1] - wall.t0) / 1000)), simFps: stats(S.fps), perceptionFps: stats(S.perFps), depthFrames: timing.frames, depthHz: r2(timing.frames / Math.max(1, S.air)), fixesPerS: r2(loc.fixes.used / Math.max(1, S.air)) },
  modelsMs: { rfdetr: stats(S.det), splatFix: stats(S.splatMs), depthModel: stats(S.depth.map((s) => s.modelMs)), depthTwin: stats(S.depth.map((s) => s.twinMs)), depthAlign: stats(S.depth.map((s) => s.alignMs)),
    depthGrid: stats(S.depth.map((s) => s.gridMs)), twinRender: r2(twin.stats.renderMs), avoidIngest: stats(timing.avoid), changesIngest: stats(timing.changes), voxIntegrate: stats(timing.integrate) },
  depth: { absRel: r3(depthLast.absRel), inliers: depthLast.inliers, skipped: depthLast.skipped, frames: depthLast.frames },
  voxels: { ...timing.integrated, after: vox.stats() },
  scene: { box: truth.box, door: truth.door, leaves, doorReadings: (safety.avoid.leafLog ?? []).filter((l) => l.closed || l.sure != null).slice(-120), leavesMade: safety.avoid.leafMade ?? [] },
  honesty: { ...S.nees, withinShare: r3(S.nees.within / Math.max(1, S.nees.n)), over3Share: r3(S.nees.over3 / Math.max(1, S.nees.n)), overOkShare: r3(S.nees.overOk / Math.max(1, S.nees.n)), worstOk: r3(S.nees.worstOk),
    by: Object.fromEntries(Object.entries(S.nees.by).map(([k, b]) => [k, { n: b.n, within: r3(b.within / Math.max(1, b.n)), over3: r3(b.over3 / Math.max(1, b.n)), overOk: r3(b.overOk / Math.max(1, b.n)) }])),
    longestFixGapS: r2(S.fixGap), safetySeconds: S.reasons, nis: loc.fixQuality().nis },
  changes: { reported, memory: changeRecs, events: memEvents, claude: asked, replans, newPathsAfterBox: newPaths.length },
  memory: { flights: memFlights, sightings: memory.sightings.map((s) => ({ label: s.label, room: s.room, x: s.x, y: s.y, n: s.n ?? null })), stats: memory.stats(),
    recallPerson: memory.recall("where did you last see a person?"), whatChanged: memory.whatChanged(), whereAmI },
  closest: { box: r3(S.near.box), door: r3(S.near.door), person: r3(S.near.person), mapped: r3(S.near.mapped), contacts: watch.contacts, hardest: r3(watch.hardest), crashed: sim.drone.crashed },
  events: S.events.slice(-120), safetyReason: safety.status().reason, splatRejects: rejects, fixes: fixLog, contactLog: watch.log, boxWhy: S.boxWhy, timeline: S.timeline,
  ...(TRACE && { trace }),
};
const F = result.flight;
log("RESULT", F);

// ---------------------------------------------------------------- checks (thresholds set before the run)

const H = F.honesty, patrol = SCENARIO === "patrol";
check("a valid run: the simulator's splat camera throughout, vision fixes within 10 s of the take-off", !invalid, invalid ?? `${S.airFixes ?? 0} vision fixes in the air`);
if (patrol) check("flew at least the asked time in the air", F.airSeconds >= FLIGHT_S * 0.95, `${F.airSeconds} s in the air over ${flights.length} patrol(s), ${F.wallSeconds} s wall`);
check("localization against the truth: median < 0.10 m, p95 < 0.30 m, never lost", F.loc.error.median < 0.1 && F.loc.error.p95 < 0.3 && !F.loc.status.lost, `median ${F.loc.error.median} m, p95 ${F.loc.error.p95} m, max ${F.loc.error.max}; ${JSON.stringify(F.loc.status)}; ${F.loc.fixes} fixes (${F.rates.fixesPerS}/s)`);
// a calibrated 2D Gaussian has 95.6% within 2.5σ and 1.1% beyond 3σ: rates, not "never" (which would push σ up everywhere)
const pct = (v) => `${(100 * v).toFixed(1)}%`;
const HZ = F.loc.height;
check("height in the air: error p95 < 0.12 m, >= 97% within 2.5σz", HZ.error.p95 < 0.12 && HZ.within >= 0.97,
  `median ${HZ.error.median} m, p95 ${HZ.error.p95} m, max ${HZ.error.max}; ${pct(HZ.within)} within 2.5σz (σz median ${HZ.zSigma.median}); the fixes' own height: |err| median ${HZ.fixes.median} p95 ${HZ.fixes.p95}, bias ${HZ.fixBias} m`);
check("σ is honest: >= 94% within 2.5σ, <= 2% over 3σ (and 0.2 m), <= 0.5% over 0.3 m off while \"ok\"", H.withinShare >= 0.94 && H.over3Share <= 0.02 && H.overOkShare <= 0.005,
  `${pct(H.withinShare)} within 2.5σ, ${pct(H.over3Share)} over 3σ (${H.over3} of ${H.n}), ${pct(H.overOkShare)} over 0.3 m while "ok" (worst ${H.worstOk} m); ${Object.entries(H.by).map(([k, b]) => `${k}: ${b.n} samples, ${pct(b.within)} within, ${pct(b.over3)} over 3σ, ${pct(b.overOk)} over 0.3 m ok`).join("; ")}; longest gap between fixes in the air ${H.longestFixGapS} s; NIS ${H.nis}`);
check("control loop (33 ms ticks, wall clock): gaps p95 < 50 ms, max < 250 ms", F.control.wallTickGapMs.p95 < 50 && F.control.wallTickGapMs.max < 250,
  `wall gaps median ${F.control.wallTickGapMs.median} p95 ${F.control.wallTickGapMs.p95} max ${F.control.wallTickGapMs.max} ms; sim time ran at ${F.control.dilation} of the wall clock; long tasks ${F.control.longTasks.n} (max ${F.control.longTasks.max} ms)`);
check("video: simulator >= 20 fps and perception >= 15 fps (median)", F.rates.simFps.median >= 20 && F.rates.perceptionFps.median >= 15, `sim ${F.rates.simFps.median} fps, perception ${F.rates.perceptionFps.median} fps`);
check("live depth runs at >= 2 Hz in flight", F.rates.depthHz >= 2, `${F.rates.depthFrames} frames, ${F.rates.depthHz} Hz; model ${F.modelsMs.depthModel.median} ms, twin ${F.modelsMs.depthTwin.median} ms, align ${F.modelsMs.depthAlign.median} ms`);
const boxRep = reported.find((r) => r.kind === "obstacle" && r.err != null && r.err < 0.6), doorRep = reported.find((r) => r.kind === "door-closed" && r.door === truth.door?.id);
if (patrol) {
  check("the box: reported as an obstacle within 0.6 m", !!boxRep, truth.box ? `${boxRep ? `${boxRep.err} m off at ${boxRep.t} s` : "not reported"} (placed at ${truth.box.t} s, ${truth.box.onPath} m along the path)` : "no box placed");
  const boxMem = boxRep && changeRecs.find((c) => c.id === boxRep.memoryId);
  check("the box handled: in memory, on the map as an obstacle, and a new path that keeps clear of it", !!boxMem && (boxMem.onMap || boxMem.status === "confirmed") && replans.some((p) => p.clear >= 0.3),
    `memory ${boxMem ? `${boxMem.status}${boxMem.by ? ` by ${boxMem.by}` : ""}, claude ${boxMem.claude?.verdict}, on map ${boxMem.onMap}` : "none"}; paths after the box: ${replans.map((p) => `${p.t} s ${p.clear} m`).join(", ") || "none"}`);
  check("the closed door: reported as door-closed", !!doorRep || truth.door?.none, truth.door?.none ? "no door to close" : doorRep ? `door ${doorRep.door} at ${doorRep.t} s` : `door ${truth.door?.id} not reported`);
} else {
  const seen = new Set([...leaves.map((l) => l.door), ...reported.filter((r) => r.kind === "door-closed").map((r) => r.door)].filter((id) => truth.door.sealed.includes(id)));
  const look = flights.find((f) => f.kind === "lookIn"), pat = flights.find((f) => f.kind === "patrol"), name = ROOM.name.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
  check(`the sealed ${ROOM.name}: a closed doorway seen (a door leaf or a door-closed change)`, seen.size > 0, `${seen.size} of ${truth.door.sealed.length} doorways seen closed (${[...seen].join(", ")}); leaves at ${leaves.map((l) => `${l.t} s ${l.door}`).join(", ") || "none"}`);
  const wrong = [...new Set([...leaves.map((l) => l.door), ...reported.filter((r) => r.kind === "door-closed").map((r) => r.door)].filter((id) => id && !truth.door.sealed.includes(id)))];
  check("no open doorway taken for a closed one (a door leaf or a door-closed change on a doorway that wasn't sealed)", !wrong.length, wrong.length ? `taken for closed: ${wrong.join(", ")}` : "none");
  check(`look in ${ROOM.name}: says it couldn't`, !!look && !look.ok && new RegExp(`couldn't look into ${name}`).test(look.summary) && /closed/.test(look.summary), `"${look?.summary}"`);
  check(`patrol: ${ROOM.name} not claimed (said it couldn't be checked)`, !!pat && !new RegExp(`Patrolled[^.]*${name}`).test(pat.summary) && new RegExp(`couldn't [^.]*${name}`).test(pat.summary), `"${pat?.summary}"`);
}
const near = (v, r) => Number.isFinite(v) && v <= r; // (null: never near one: no box in the sealed scenario, nobody about)
check("no contact; the box, the door and the person kept clear", watch.contacts === 0 && !sim.drone.crashed && !near(F.closest.box, 0.15) && !near(F.closest.door, 0.1) && !near(F.closest.person, 0.5),
  `contacts ${watch.contacts}; closest: box ${F.closest.box} m, door ${F.closest.door} m, person ${F.closest.person} m, anything mapped (or unknown) ${F.closest.mapped} m`);
const trailN = memFlights.reduce((a, f) => a + f.trail, 0); // flights are timed by the wall clock (Date.now), samples by the simulation's
check("memory: a trail at 2 Hz of flight, and the flights' rooms", memFlights.length >= 1 && trailN >= 1.6 * F.airSeconds, `${trailN} samples for ${F.airSeconds} s in the air; ${memFlights.map((f) => `rooms ${f.rooms.join(", ")}`).join("; ")}`);
check("Claude (fake) checked the changes with pictures", asked.length > 0 && asked.every((a) => a.images === 2), `${asked.length} checks: ${asked.map((a) => `${a.kind} -> ${a.verdict}`).join(", ")}`);
result.done = true;
await savePng($("map"), "c-int-map.png");
await savePng($("vox"), "c-int-voxels.png");
await savePng($("fpv"), "c-int-fpv-end.png");
await upload("c-int-result.json", JSON.stringify(result, null, 1));
progress("done", true);
