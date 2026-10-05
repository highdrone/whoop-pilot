// Browser check for the wave C panels (app/js/ui/view3d.js, history.js, changecard.js, coverage.js, survey.js, calib.js,
// recheck.js), each mounted on its own, without main.js, against a real capture served by tools/whoop.mjs: imported through
// /house-files/ (newest with a 3D scan, or ?project=ID), the splat twin, the 3D map (built, or loaded from this origin's
// storage), and a flight memory filled with a made-up history: yesterday's patrol through every room (σ growing, a lost
// stretch), today's look in a room, a person, the cat and the dog seen (pictures rendered by the twin with its people and
// pets), a new box and a closed door with live and expected pictures (the twin with and without a prop), changes already
// decided; and a flight going on now along a planned path. A fake Claude (a fetch stand-in, nothing leaves the page)
// answers the survey (boxing the capture's own objects) and the change checks; the calibration runs are made up; the
// localization check reads the recordings in the server's data folder (replayed with nav/replay.js when it is there).
// Results in window.result (done, checks, numbers); window.finish() then applies the survey and saves the calibration.
// Checks that need a visible page (timing) say SKIP when it is hidden (a background tab throttles its timers).
// ?mode=real&flying=1: the real drone in the air (the 3D view must not render); light or dark as the system is; ?auto=0: just
// mount everything for a person to try; ?rooms=r1,r2: survey only those.
import { importCapture } from "../app/js/house/import.js";
import { houseSource } from "../app/js/ui/housepanel.js";
import { openStore } from "../app/js/house/store.js";
import { houseFrame } from "../app/js/house/frames.js";
import { buildVoxels, voxelCentres } from "../app/js/house/voxels.js";
import { coverageReport } from "../app/js/house/coverage.js";
import { plan, roomCenter } from "../app/js/house/planner.js";
import { Twin, pinholeLens, intrinsics, project } from "../app/js/twin/twin.js";
import { droneCamera } from "../app/js/twin/pose.js";
import { HouseMemory } from "../app/js/memory/memory.js";
import { Claude } from "../app/js/ai/claude.js";
import { Inspector, CHANGE_PROMPT } from "../app/js/ai/inspect.js";
import { surveyHouse, estimateSurvey, SURVEY, SURVEY_PROMPT } from "../app/js/ai/survey.js";
import * as Lens from "../app/js/nav/lens.js";
import { Emitter, sleep } from "../app/js/util.js";
import { View3D } from "../app/js/ui/view3d.js";
import { HistoryPanel } from "../app/js/ui/history.js";
import { ChangeCard } from "../app/js/ui/changecard.js";
import { CoveragePanel } from "../app/js/ui/coverage.js";
import { SurveyPanel } from "../app/js/ui/survey.js";
import { CalibPanel } from "../app/js/ui/calib.js";
import { RecheckPanel } from "../app/js/ui/recheck.js";

const $ = (id) => document.getElementById(id);
const q = new URLSearchParams(location.search);
const lines = [];
const log = (...a) => { lines.push(a.map((x) => (typeof x === "string" ? x : JSON.stringify(x))).join(" ")); $("log").textContent = lines.join("\n"); };
const status = (t) => ($("status").textContent = t);
const result = (window.result = { done: false, checks: {}, numbers: {}, errors: [] });
const errors = result.errors;
addEventListener("error", (e) => errors.push(String(e.message)));
addEventListener("unhandledrejection", (e) => errors.push(String(e.reason?.message ?? e.reason)));
const consoleError = console.error.bind(console);
console.error = (...a) => (errors.push(a.map((x) => String(x?.message ?? x)).join(" ")), consoleError(...a));
function check(name, ok, detail = "") { // ok null: skipped (not measured), never a pass
  result.checks[name] = ok === null ? { ok: null, skipped: true, detail } : { ok: !!ok, detail };
  const tr = $("checks").insertRow();
  tr.insertCell().textContent = ok === null ? "SKIP" : ok ? "PASS" : "FAIL";
  tr.cells[0].className = ok === null ? "skip" : ok ? "pass" : "fail";
  tr.insertCell().textContent = name;
  tr.insertCell().textContent = detail;
}
const until = async (fn, ms = 10000, step = 100) => { const t = performance.now(); while (performance.now() - t < ms) { const v = await fn(); if (v) return v; await sleep(step); } return null; };
const state = (window.state = { mode: q.get("mode") ?? "sim", flying: q.get("flying") === "1", paused: false });

class FakeSettings extends Emitter {
  constructor(values) { super(); this.values = values; }
  get(k) { return this.values[k]; }
  set(k, v) { if (this.values[k] === v) return; this.values[k] = v; this.emit("change", { key: k, value: v }); }
}
const settings = (window.settings = new FakeSettings({ apiKey: "fake-key-for-the-check", model: "claude-opus-5", aiModel: "", aiVision: "ask", aiBudget: 0.5, uptilt: 20, videoDelay: 100 }));

// ---------------------------------------------------------------- the house, its twin and 3D map

const t0 = performance.now();
const projects = (await (await fetch("/house-files/")).json()).projects ?? [];
const capture = projects.find((p) => p.id === q.get("project")) ?? projects.find((p) => p.ready);
if (!capture) throw new Error("no capture with a 3D scan under /house-files/");
status(`importing ${capture.name}`);
const imp = await importCapture(houseSource(capture, ({ loaded, total }) => status(`downloading the 3D scan ${Math.round((100 * loaded) / total)}%`)));
const { house, map, splat } = imp;
house.splatFile = splat.name;
const store = await openStore();
await store.writeFile(house.id, "calib.json", "null"); // no lens from an earlier run of this check (its calibrations are made up)
if ((await store.readFile(house.id, splat.name))?.byteLength === splat.bytes.byteLength) await store.saveHouse(house, {});
else await store.saveImport(imp); // the splat too: nav/replay.js's worker reads the house from this origin's storage
status("starting the twin");
const twin = (window.twin = await Twin.create({ splat: new Blob([splat.bytes]), house }));
let vox = q.get("rebuild") === "1" ? null : await store.loadVoxels(house), built = false;
if (vox) await vox.refresh();
else {
  status("building the 3D map (about half a minute)");
  const centres = await voxelCentres(splat.bytes, splat.name, houseFrame({ f: house.frame.f, Yf: house.frame.Yf }));
  vox = await buildVoxels({ house, map, centres, twin, onProgress: (p) => status(`3D map: ${p.text ?? p.phase}`) });
  await store.saveVoxels(house, vox);
  built = true;
}
map.setVoxels(vox);
result.numbers.setup = { capture: capture.id, rooms: house.rooms.map((r) => r.name), seconds: +((performance.now() - t0) / 1000).toFixed(1), voxels: built ? "built" : "loaded", twin: twin.info.gpu };
log(`capture ${capture.name}: ${house.rooms.length} rooms (${house.rooms.map((r) => r.name).join(", ")}), twin ${twin.info.splats} splats on ${twin.info.gpu}, 3D map ${built ? "built" : "loaded"}; ${result.numbers.setup.seconds} s`);

// ---------------------------------------------------------------- pictures from the twin

const toCam = (cam, P) => { const r = [0, 1, 2].map((k) => P[k] - cam.p[k]); return [0, 1, 2].map((k) => cam.R[0][k] * r[0] + cam.R[1][k] * r[1] + cam.R[2][k] * r[2]); };
const floorAt = (x, y) => map.floorAt(x, y) ?? 0;
const centreOf = (id, alt = 1.0) => { const c = roomCenter(map, id, { alt }); return c && [c[0], c[1], floorAt(c[0], c[1]) + alt]; };
const lookPose = (at, look) => ({ x: at[0], y: at[1], z: at[2], yaw: Math.atan2(look[1] - at[1], look[0] - at[0]), pitch: Math.atan2(at[2] - look[2], Math.hypot(look[0] - at[0], look[1] - at[1])), roll: 0 });
async function jpeg({ width, height, data }) {
  const c = new OffscreenCanvas(width, height);
  c.getContext("2d").putImageData(new ImageData(new Uint8ClampedArray(data.buffer, data.byteOffset, data.length), width, height), 0, 0);
  return c.convertToBlob({ type: "image/jpeg", quality: 0.82 });
}
async function shot(at, look, { actors = [], props = [], withProps = true, w = 320, h = 240 } = {}) {
  await twin.setActors(actors);
  await twin.setProps(props);
  const px = await twin.pixels(lookPose(at, look), { width: w, height: h, lens: pinholeLens(75), actors: actors.length > 0, props: withProps });
  return jpeg(px);
}
// The share of pixels that differ clearly between the live and the expected picture (the prop shows).
async function differ({ live, expected }) {
  const px = async (b) => { const bm = await createImageBitmap(b), c = new OffscreenCanvas(bm.width, bm.height), g = c.getContext("2d"); g.drawImage(bm, 0, 0); return g.getImageData(0, 0, bm.width, bm.height).data; };
  const [a, b] = await Promise.all([px(live), px(expected)]);
  let n = 0;
  for (let i = 0; i < a.length; i += 4) n += Math.abs(a[i] - b[i]) + Math.abs(a[i + 1] - b[i + 1]) + Math.abs(a[i + 2] - b[i + 2]) > 60 ? 1 : 0;
  return +(n / (a.length / 4)).toFixed(3);
}
// A spot `d` m from `p` toward the room's middle, at camera height, inside the room.
function standOff(p, room, d = 2.0) {
  const c = centreOf(room) ?? [p[0] + 1, p[1], 1];
  for (let k = 0; k < 24; k++) {
    const a = Math.atan2(c[1] - p[1], c[0] - p[0]) + (k % 2 ? 1 : -1) * Math.ceil(k / 2) * 0.26, x = p[0] + d * Math.cos(a), y = p[1] + d * Math.sin(a);
    if (map.roomAt(x, y) && map.clearance(x, y, floorAt(x, y) + 1.1) > 0.25) return [x, y, floorAt(x, y) + 1.1];
  }
  return [c[0], c[1], c[2] + 0.1];
}
// A free floor spot in a room, about `off` m from its middle, in the direction `turn` (rad) or the nearest free one to it.
function spotIn(room, off = 0.8, turn = 0) {
  const c = centreOf(room);
  for (const d of [off, off * 0.7, off * 1.3, off * 0.4])
    for (let k = 0; k < 24; k++) {
      const a = turn + (k % 2 ? 1 : -1) * Math.ceil(k / 2) * 0.26, x = c[0] + d * Math.cos(a), y = c[1] + d * Math.sin(a);
      if (map.roomAt(x, y)?.id === room && [0.25, 0.6, 1.0].every((z) => map.clearance(x, y, floorAt(x, y) + z) > 0.35)) return [x, y, floorAt(x, y)];
    }
  return [c[0] + off, c[1], floorAt(c[0] + off, c[1])];
}

// Live and expected pictures of a change: from the spot around it (inside a room, nothing up close) where the prop shows
// best (clearly, with some of the room around it).
async function evidence(look, props) {
  let best = null;
  for (const d of [1.5, 2.0])
    for (let k = 0; k < 8; k++) {
      const a = (k * Math.PI) / 4, x = look[0] + d * Math.cos(a), y = look[1] + d * Math.sin(a), z = floorAt(x, y) + 1.1;
      if (!map.roomAt(x, y) || map.clearance(x, y, z) < 0.3 || vox.raycast([x, y, z], [look[0] - x, look[1] - y, look[2] - z], 3).d < Math.hypot(look[0] - x, look[1] - y, look[2] - z) - 0.4) continue;
      const pair = { live: await shot([x, y, z], look, { props }), expected: await shot([x, y, z], look, { props, withProps: false }) }, share = await differ(pair);
      const score = Math.min(share, 0.35) - Math.max(0, share - 0.6); // it shows, with the room around it
      if (!best || score > best.score) best = { ...pair, share, score };
    }
  return best;
}

// ---------------------------------------------------------------- a made-up flight history

const rooms = [...map.rooms].sort((a, b) => b.area - a.area).map((r) => r.id), [R1, R2, R3] = [rooms[0], rooms[1] ?? rooms[0], rooms[2] ?? rooms[1] ?? rooms[0]];
const home = house.home ?? { x: centreOf(R1)[0], y: centreOf(R1)[1], yaw: 0 };
const pathThrough = (pts, alt = 1.0) => {
  const out = [];
  for (let i = 1; i < pts.length; i++) {
    const p = plan(map, pts[i - 1], pts[i], { alt });
    if (p.ok) out.push(...(out.length ? p.path.slice(1) : p.path));
  }
  return out;
};
// Samples along a path at `speed` m/s every 0.5 s from t: [{ t, x, y, z, yaw }].
function walk(path, t, speed = 0.3) {
  const out = [];
  let d = 0;
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1], b = path[i], L = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]), yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
    for (; d <= L; d += speed * 0.5) out.push({ t: (t += 500), x: a[0] + ((b[0] - a[0]) * d) / L, y: a[1] + ((b[1] - a[1]) * d) / L, z: a[2] + ((b[2] - a[2]) * d) / L, yaw });
    d -= L;
  }
  return out;
}
const pad = [home.x, home.y, floorAt(home.x, home.y) + 0.05];
const liveRoute = pathThrough([pad, centreOf(R1), centreOf(R2), centreOf(R3)]); // planned before the made-up changes block anything
let fixed = Date.now() - 864e5;
const memory = (window.memory = await HouseMemory.open(house.id, { indexedDB: null, now: () => fixed ?? Date.now() }));
memory.setHouse({ house, map, vox, save: null });
const fly = (samples, sigma = () => 0.05, lost = () => false) => samples.forEach((s, i) => memory.samplePose(lost(i) ? { status: "lost" } : { ...s, sigma: sigma(i), status: "ok", room: map.roomAt(s.x, s.y)?.id ?? null }, (fixed = s.t)));
status("making a flight history");
{ // yesterday evening: a patrol of every room; σ grows in the second room, the position is lost for a few seconds in the third
  const d = new Date(Date.now() - 864e5);
  d.setHours(18, 10, 0, 0);
  const route = pathThrough([pad, ...rooms.map((r) => centreOf(r)), pad]), s = walk(route, d.getTime()), n = s.length;
  memory.startFlight({ kind: "patrol", mission: { kind: "patrol" } }, (fixed = d.getTime()));
  fly(s, (i) => (i > n * 0.3 && i < n * 0.55 ? 0.05 + 0.5 * ((i - n * 0.3) / (n * 0.25)) : 0.05), (i) => i > n * 0.62 && i < n * 0.62 + 10);
  const pp = spotIn(R1, 1.2, 0), pv = standOff([pp[0], pp[1], pp[2] + 1.2], R1, 2.2);
  memory.addSighting({ label: "person", room: R1, x: pp[0], y: pp[1], z: pp[2] + 1.0, score: 0.91, sigma: 0.05, claude: "confirmed", t: s[Math.floor(n * 0.2)].t,
    snapshot: await shot(pv, [pp[0], pp[1], pp[2] + 1.0], { actors: [{ id: "p1", kind: "person", x: pp[0], y: pp[1], z: pp[2], yaw: 2.4, pose: "stand" }] }) });
  const dp = spotIn(R2, 0.6, 1), dv = standOff([dp[0], dp[1], dp[2] + 0.4], R2, 1.8);
  memory.addSighting({ label: "dog", room: R2, x: dp[0], y: dp[1], z: dp[2] + 0.3, score: 0.74, sigma: 0.18, t: s[Math.floor(n * 0.45)].t,
    snapshot: await shot(dv, [dp[0], dp[1], dp[2] + 0.3], { actors: [{ id: "d1", kind: "dog", x: dp[0], y: dp[1], z: dp[2], yaw: 0.6, pose: "stand" }] }) });
  // a box that was confirmed then, and someone taken for a change
  const ob = spotIn(R2, 0.7, 3);
  const old = memory.addChange({ kind: "obstacle", x: ob[0], y: ob[1], z: ob[2] + 0.2, size: 0.35, what: "laundry basket" }, s[Math.floor(n * 0.5)].t);
  memory.resolveChange(old.id, "confirmed", "Confirmed by you.", { by: "user" }, s[Math.floor(n * 0.5)].t + 60000);
  const tp = spotIn(R1, 1.5, 2.5), passer = memory.addChange({ kind: "obstacle", x: tp[0], y: tp[1], z: tp[2] + 0.8, size: 0.5 }, s[Math.floor(n * 0.15)].t);
  memory.resolveChange(passer.id, "dismissed", "A person or pet, said by you.", { by: "user", transient: true }, s[Math.floor(n * 0.15)].t + 30000);
  memory.endFlight("Patrolled every room.", (fixed = s.at(-1).t + 2000));
}
let boxChange = null, doorChange = null;
{ // today, 25 minutes ago: look in the third room; the cat, a new box, a closed door
  const t = Date.now() - 25 * 60e3, route = pathThrough([pad, centreOf(R3), pad]), s = walk(route, t), n = s.length;
  memory.startFlight({ kind: "lookIn", mission: { kind: "lookIn", room: map.rooms.find((r) => r.id === R3).name } }, (fixed = t));
  fly(s.slice(0, Math.floor(n * 0.6)));
  const cp = spotIn(R3, 0.6, 2), cv = standOff([cp[0], cp[1], cp[2] + 0.3], R3, 1.6);
  memory.addSighting({ label: "cat", room: R3, x: cp[0], y: cp[1], z: cp[2] + 0.2, score: 0.83, sigma: 0.06, t: (fixed = s[Math.floor(n * 0.5)].t),
    snapshot: await shot(cv, [cp[0], cp[1], cp[2] + 0.2], { actors: [{ id: "c1", kind: "cat", x: cp[0], y: cp[1], z: cp[2], yaw: 1.2, pose: "walk" }] }) });
  const bp = spotIn(R1, 1.0, 4.2), boxProp = { id: "box", kind: "box", x: bp[0], y: bp[1], z: bp[2], w: 0.45, d: 0.35, h: 0.4, yaw: 0.4, color: 0x9a7b4f };
  const boxPics = await evidence([bp[0], bp[1], bp[2] + 0.2], [boxProp]);
  boxChange = memory.addChange({ kind: "obstacle", x: bp[0], y: bp[1], z: bp[2] + 0.2, size: 0.45, zMin: bp[2], zMax: bp[2] + 0.4, evidence: boxPics }, s[Math.floor(n * 0.3)].t);
  result.numbers.evidence = { box: boxPics?.share };
  memory.annotateChange(boxChange.id, { claude: { status: "answered", verdict: "real-change", kind: "obstacle", what: "cardboard box", confidence: 0.86, why: "A brown box on the floor that the 3D scan doesn't have.", suggest: "confirmed", small: false, t: s[Math.floor(n * 0.3)].t } });
  memory.seen(boxChange.id, s[Math.floor(n * 0.4)].t);
  boxChange.n = 3;
  const door = (house.doors ?? []).find((d) => d.rooms[1] && d.passable !== false);
  if (door) {
    const mx = (door.a[0] + door.b[0]) / 2, my = (door.a[1] + door.b[1]) / 2, fl = floorAt(mx, my), wdt = Math.hypot(door.b[0] - door.a[0], door.b[1] - door.a[1]);
    const panel = { id: "door", kind: "panel", x: mx, y: my, z: fl, w: wdt, h: Math.min(2.05, (door.headZ ?? fl + 2.05) - fl), yaw: Math.atan2(door.b[1] - door.a[1], door.b[0] - door.a[0]), color: 0xe8e2d6 };
    const doorPics = await evidence([mx, my, fl + 1], [panel]);
    doorChange = memory.addChange({ kind: "door-closed", door: door.id, x: mx, y: my, z: fl + 1, size: wdt, evidence: doorPics }, s[Math.floor(n * 0.55)].t);
    result.numbers.evidence.door = doorPics?.share;
    memory.annotateChange(doorChange.id, { claude: { status: "unconfirmed", reason: "waiting for your OK to Claude's picture checks", waiting: true, t: s[Math.floor(n * 0.55)].t } });
  }
  await twin.setActors([]);
  await twin.setProps([]);
  fly(s.slice(Math.floor(n * 0.6)));
  memory.endFlight("Looked in the room.", (fixed = s.at(-1).t + 2000));
}
// now: a flight that started 20 s ago, through the rooms to the third and back, at 0.25 m/s, again and again
const AGO = 20000, liveSamples = walk(liveRoute, Date.now() - AGO, 0.25), startAt = Date.now() - AGO, N = liveSamples.length;
memory.startFlight({ kind: "lookIn", mission: { kind: "lookIn", room: map.rooms.find((r) => r.id === R3).name } }, (fixed = startAt));
fly(liveSamples.filter((s) => s.t <= Date.now()));
fixed = null;
log(`history: ${memory.flights.length} flights, ${memory.sightings.length} sightings, ${memory.changes.length} changes; live route ${liveRoute.length} points, ${N} samples`);

// The drone now and its plan (the localizer and the mission runner's events): along the route in real time, then back.
const missions = (window.missions = new Emitter());
const at = () => {
  const i = Math.max(0, Math.floor((Date.now() - startAt) / 500)) % (2 * N - 2), back = i >= N, k = back ? 2 * N - 2 - i : i;
  return { ...liveSamples[k], yaw: liveSamples[k].yaw + (back ? Math.PI : 0), k, back };
};
const localizer = (window.localizer = {
  known: true,
  pose: () => {
    const s = at(), sigma = 0.04 + 0.05 * Math.sin(Date.now() / 5000) ** 2, stale = Math.floor(Date.now() / 5000) % 6 === 5; // as nav/localizer.js: no camera fix for a while reads "degraded"
    return { x: s.x, y: s.y, z: s.z, yaw: s.yaw, sigma, status: stale || sigma >= 0.25 ? "degraded" : "ok", ...(stale && { stale: 4000 }), source: "fused", room: map.roomAt(s.x, s.y)?.id ?? null };
  },
});
const stopName = (id) => map.rooms.find((r) => r.id === id)?.name;
setInterval(() => {
  if (state.paused) return;
  const s = at(), route = s.back ? [...liveRoute].reverse() : liveRoute, from = route.findIndex((p) => Math.hypot(p[0] - s.x, p[1] - s.y) < 0.35), left = route.slice(Math.max(0, from));
  missions.emit("path", { path: left.length > 1 ? left : route });
  const goal = s.back ? R1 : R3, c = centreOf(goal), secs = Math.max(0, (s.back ? s.k : N - 1 - s.k) * 0.5);
  missions.emit("plan", { stops: [{ room: goal, name: stopName(goal), x: c[0], y: c[1], seconds: secs }], home: false, seconds: secs });
  memory.samplePose(localizer.pose());
}, 500);

// ---------------------------------------------------------------- a fake Claude

const W = SURVEY.width, H = SURVEY.height, K = intrinsics(pinholeLens(SURVEY.hfov), W, H);
const NAMES = [[/sofa|couch/i, "sofa"], [/^table$/i, "table"], [/chair/i, "chair"], [/storage|cabinet|dresser/i, "cabinet"], [/fireplace/i, "fireplace"], [/bed$/i, "bed"], [/tv|television/i, "tv"], [/desk/i, "desk"]];
const HAZARD = [[/^ceiling fan$/i, "ceiling-fan"], [/curtain|shade|blind/i, "curtain"], [/^stairs?$/i, "stairs"], [/plant/i, "plant"], [/pendant|chandelier/i, "hanging-lamp"]];
const objects = (house.landmarks ?? []).map((l) => {
  const kind = HAZARD.find(([re]) => re.test(l.name))?.[1], name = kind ? null : NAMES.find(([re]) => re.test(l.name))?.[1];
  if (!kind && !name) return null;
  const foot = l.footprint ?? (Number.isFinite(l.x) ? [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([a, b]) => [l.x + 0.3 * a, l.y + 0.3 * b]) : null);
  const zMax = l.top ?? (l.z ?? 1) + 0.25, zMin = l.size ? zMax - l.size[2] : zMax - 0.5;
  return foot && { name, kind, foot, zMin, zMax };
}).filter(Boolean);
const shown = []; // the survey's views, as rendered (pixels after depth)
const surveyTwin = { pixels: (pose, view) => (shown.push(pose), twin.pixels(pose, view)), depth: (pose, view) => twin.depth(pose, view) };
function boxIn(pose, o) {
  const cam = droneCamera(pose, 0), cs = o.foot.flatMap(([x, y]) => [[x, y, o.zMin], [x, y, o.zMax]]).map((P) => toCam(cam, P));
  if (cs.some((c) => c[2] < 0.2)) return null;
  const px = cs.map((c) => project(K, c)), xs = px.map((p) => p[0]), ys = px.map((p) => p[1]);
  const b = [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)], cb = [Math.max(0, b[0]), Math.max(0, b[1]), Math.min(W, b[2]), Math.min(H, b[3])];
  const area = (b[2] - b[0]) * (b[3] - b[1]), carea = Math.max(0, cb[2] - cb[0]) * Math.max(0, cb[3] - cb[1]);
  return carea >= 0.5 * area && cb[2] - cb[0] >= 24 && cb[3] - cb[1] >= 24 ? { box: cb.map(Math.round), area: carea } : null;
}
const asked = (result.numbers.claude = { survey: 0, change: 0, other: 0 });
const fakeFetch = async (url, init) => {
  const body = JSON.parse(init.body), sys = body.system[0].text, content = body.messages[0].content, images = content.filter((b) => b.type === "image").length;
  await sleep(250);
  let answer;
  if (sys.startsWith(SURVEY_PROMPT.slice(0, 60))) {
    asked.survey++;
    const views = shown.slice(-images), room = map.roomAt(views[0].x, views[0].y)?.id;
    const found = objects.map((o) => {
      let best = null;
      views.forEach((v, i) => { const b = boxIn(v, o); if (b && (!best || b.area > best.area)) best = { ...b, view: i + 1 }; });
      return best && { o, ...best };
    }).filter(Boolean);
    const mine = (f) => map.roomAt(f.o.foot.reduce((a, p) => a + p[0], 0) / f.o.foot.length, f.o.foot.reduce((a, p) => a + p[1], 0) / f.o.foot.length)?.id === room;
    const has = (n) => found.some((f) => mine(f) && (f.o.name === n || f.o.kind === n));
    const [kind, name] = has("bed") ? ["bedroom", "Bedroom"] : has("sofa") ? ["living room", "Living room"] : has("stairs") ? ["stairwell", "Stair hall"] : has("desk") ? ["office", "Office"] : ["hallway", "Hallway"];
    answer = { room: { kind, name, confidence: room === R1 ? 0.84 : 0.66 },
      landmarks: found.filter((f) => f.o.name).map((f, i) => ({ name: f.o.name, view: f.view, box: f.box, confidence: i ? 0.62 : 0.88 })),
      hazards: found.filter((f) => f.o.kind).map((f) => ({ kind: f.o.kind, view: f.view, box: f.box, confidence: 0.8, why: { "ceiling-fan": "a ceiling fan over the room", curtain: "long curtains by the window", stairs: "stairs going down", plant: "a leafy plant" }[f.o.kind] ?? "a hazard" })) };
  } else if (sys.startsWith(CHANGE_PROMPT.slice(0, 60))) {
    asked.change++;
    answer = { verdict: "real-change", kind: "door-closed", what: "bedroom door", confidence: 0.82, why: "The doorway that was open in the scan is closed by a white door." };
  } else (asked.other++, (answer = { present: true, actual: "a cat", confidence: 0.9, why: "a tabby cat" }));
  const usage = { input_tokens: images * 590 + 400, cache_creation_input_tokens: 0, cache_read_input_tokens: Math.round(sys.length / 4), output_tokens: 900 };
  return new Response(JSON.stringify({ id: "msg_fake", type: "message", role: "assistant", model: body.model, stop_reason: "end_turn", content: [{ type: "text", text: JSON.stringify(answer) }], usage }),
    { status: 200, headers: { "content-type": "application/json" } });
};
const claude = (window.claude = new Claude({ settings, fetch: fakeFetch }));
const inspect = new Inspector({ claude });
inspect.setMemory(memory);

// ---------------------------------------------------------------- the panels

const v3d = (window.v3d = new View3D($("view3d"), { house, twin, vox, map, localizer, missions, memory, mode: () => state.mode, flying: () => state.flying }));
const card = (window.card = new ChangeCard($("changeCard"), { memory, inspect, settings }));
const hist = (window.hist = new HistoryPanel($("history"), { memory, map, inspect, settings }));
const flying = () => state.flying;
const cov = (window.cov = new CoveragePanel($("coverage"), { house, map, vox, coverageReport, flying }));
const events = (window.events = []);
v3d.on("select", (e) => (events.push(["v3d.select", e]), e.kind === "change" && card.show(e.id)));
hist.on("trail", (e) => (events.push(["hist.trail", e]), v3d.setFlight(e.flightId)));
hist.on("select", (e) => events.push(["hist.select", e]));
card.on("resolved", (e) => events.push(["card.resolved", e]));
card.on("show", (e) => events.push(["card.show", e]));
cov.on("show", (e) => events.push(["cov.show", e]));
const applied = [];
const survey = (window.survey = new SurveyPanel($("survey"), { house, map, vox, twin: surveyTwin, settings, claude, survey: surveyHouse, estimate: estimateSurvey, flying,
  onApply: async (changes) => {
    const done = changes.apply(house);
    map.finalize();
    const voxels = vox.applyObjects(house);
    await store.saveVoxels(house, vox);
    v3d.invalidate();
    cov.refresh();
    applied.push({ changes, done, voxels });
    return { ...done, voxels };
  } }));
// The replay: nav/replay.js's checkRecording (the panel's default) unless ?replay=canned, or when it isn't there: then the
// numbers tools/loc-check's replay gave for the same synthetic recording (out/c-loc-result.json), over a few seconds.
const RP = await import("../app/js/nav/replay.js").catch(() => null), replayHow = RP?.checkRecording && q.get("replay") !== "canned" ? "nav/replay.js" : "canned";
const replay = replayHow !== "canned" ? null : async (id, { onProgress, signal }) => {
  for (let i = 1; i <= 12; i++) (await sleep(150, signal), onProgress?.({ seconds: i * 5, of: 62, text: `Replaying: ${i * 5} s of 62 s` }));
  return { region: { sx: 0, sy: 0, sw: 640, sh: 480 }, pad: { x: 0.007, y: 0.161 }, groundFixVsPad: 0.007, endVsPad: 0.185, endStatus: "ok", gated: 0, conflicts: 0, anchored: 0, delay: null,
    seconds: 62.4, flyingSeconds: 59.4, units: 1870, decoded: 1870, start: "pad command", foundAfter: 1, truth: true, error: { median: 0.041, p95: 0.124, max: 0.172, n: 198 }, status: { ok: 198 }, fixRate: 5.03, accepted: 299 };
};
const madeCalib = (source) => ({ ...Lens.calibOf(Lens.droneLens(null, 20), 640, 480), fx: 257.9, fy: 257.1, cx: 324.6, cy: 236.8, k: [0.021, -0.008, 0, 0], uptiltDeg: 21.6, rms: 0.71, source,
  verified: { err: 0.021, yawSpread: 0.6, accepted: 6, at: new Date().toISOString() } });
const fakeRun = (phases) => async (_, o) => {
  const { onProgress, signal } = o ?? _;
  for (const [phase, n] of phases) for (let i = 1; i <= n; i++) (await sleep(120, signal), onProgress?.({ phase, samples: i, of: n, done: i, total: n }));
  return { calib: madeCalib(phases[0][0] === "turn" ? "pad" : "recording"), verify: { ok: true, accepted: 6, n: 6, medianErr: 0.021 } };
};
const listRecs = () => fetch("/rec/list").then((r) => r.json()).then((j) => j.recordings ?? []).catch(() => []);
const calib = (window.calib = new CalibPanel($("calib"), { twin, house, map, settings, lens: Lens, recordings: listRecs, calibrateLive: fakeRun([["turn", 10], ["fit", 1], ["check", 6]]),
  ...(replayHow === "canned" && { calibrateRecording: fakeRun([["turn", 10], ["fit", 1], ["check", 6]]) }), save: (c) => store.writeFile(house.id, "calib-check.json", JSON.stringify(c)), current: async () => null, flying })); // made up: never the lens replays use
const recheck = (window.recheck = new RecheckPanel($("recheck"), { recordings: listRecs, house, settings, flying, ...(replay && { replay }) }));
recheck.on("calibrate", (e) => events.push(["recheck.calibrate", e]));
result.numbers.replay = replayHow;
status("mounted");

// ---------------------------------------------------------------- the checks

async function checks() {
  // 3D view
  const r0 = v3d.renders;
  const rendered = await until(() => v3d.renders > r0 && v3d.bg, 15000);
  check("3D view: the twin renders the house from a camera inside the rooms (sim, ≤ 2 a second)", state.mode === "real" && state.flying ? true : !!rendered,
    `${v3d.renders} renders, ${v3d.stats().renderMs} ms each; badge "${v3d.ui.badgeLong.textContent}"; caption "${v3d.ui.where.textContent}" / "${v3d.ui.next.textContent}"`);
  const t = performance.now(), n0 = v3d.renders;
  await sleep(4000);
  const rate = ((v3d.renders - n0) * 1000) / (performance.now() - t);
  check("3D view: at most 2 renders a second", rate <= 2.05, `${rate.toFixed(2)}/s over 4 s`);
  const p0 = v3d.ui.where.textContent;
  Object.assign(state, { mode: "real", flying: true });
  const n1 = v3d.renders, f1 = v3d.skipped.flying;
  v3d.moved();
  await sleep(3000);
  check("3D view: never renders while the real drone flies (a still picture or the drawing, live marks on it)", v3d.renders === n1 && v3d.skipped.flying > f1,
    `renders ${n1} → ${v3d.renders}, held back ${v3d.skipped.flying - f1} times; badge "${v3d.ui.badgeLong.textContent}"; caption "${p0}" → "${v3d.ui.where.textContent}"`);
  Object.assign(state, { mode: q.get("mode") ?? "sim", flying: q.get("flying") === "1" });
  // The simulator's camera (the drone lens, 320x240, 15 a second) with the 3D view off, then on: the 3D view mustn't slow it.
  const simCam = async (ms) => {
    const t0 = performance.now(), lat = [];
    while (performance.now() - t0 < ms) {
      const t = performance.now(), p = localizer.pose();
      (await twin.render({ x: p.x, y: p.y, z: p.z, yaw: p.yaw, pitch: 0, roll: 0 }, { width: 320, height: 240 })).close();
      lat.push(performance.now() - t);
      await sleep(Math.max(0, 66 - (performance.now() - t)));
    }
    lat.sort((a, b) => a - b);
    return { fps: +((lat.length * 1000) / (performance.now() - t0)).toFixed(2), p95: Math.round(lat[Math.floor(0.95 * lat.length)]) };
  };
  v3d.setHouse(house, null, vox, map);
  const off = await simCam(6000);
  v3d.setHouse(house, twin, vox, map);
  v3d.setFollow(true);
  await sleep(1000);
  const r1 = v3d.renders, b1 = v3d.skipped.busy, on = await simCam(6000);
  result.numbers.load = { off, on, view3d: { renders: v3d.renders - r1, skippedBusy: v3d.skipped.busy - b1 }, hidden: document.hidden };
  check("3D view: the simulator's camera keeps its pace with the 3D view open (within 5%)", document.hidden ? null : on.fps >= 0.95 * off.fps,
    `${off.fps} → ${on.fps} frames a second, p95 ${off.p95} → ${on.p95} ms; the 3D view rendered ${v3d.renders - r1} times, held back ${v3d.skipped.busy - b1}${document.hidden ? " (page hidden: timers throttled, not measured)" : ""}`);
  // Real mode in the air: a stand-in for the vision localizer (twin renders back to back, as its fixes are) with the 3D view
  // closed, then open: the 3D view must cost it nothing (it never renders then; only its marks are drawn).
  const fixes = async (ms) => {
    const t0 = performance.now();
    let n = 0;
    while (performance.now() - t0 < ms) {
      const p = localizer.pose();
      (await twin.render({ x: p.x, y: p.y, z: p.z, yaw: p.yaw, pitch: 0, roll: 0 }, { width: 320, height: 240, actors: false })).close();
      n++;
    }
    return +((n * 1000) / (performance.now() - t0)).toFixed(2);
  };
  Object.assign(state, { mode: "real", flying: true });
  v3d.setHouse(house, null, vox, map);
  const locOff = await fixes(5000);
  v3d.setHouse(house, twin, vox, map);
  const rr = v3d.renders, locOn = await fixes(5000);
  result.numbers.realLoad = { off: locOff, on: locOn, renders: v3d.renders - rr, hidden: document.hidden };
  check("3D view: in real flight the localizer's renders keep their pace with the 3D view open (within 5%, no 3D view renders)", document.hidden ? null : locOn >= 0.95 * locOff && v3d.renders === rr,
    `${locOff} → ${locOn} localizer renders a second; the 3D view rendered ${v3d.renders - rr} times${document.hidden ? " (page hidden: timers throttled, not measured)" : ""}`);
  Object.assign(state, { mode: q.get("mode") ?? "sim", flying: q.get("flying") === "1" });
  // the simulator's true pose (main.js's truth has no z) drawn apart from the estimate; on the way home, no stale path ahead
  v3d.truth = () => { const p = localizer.pose(); return { x: p.x + 0.4, y: p.y + 0.2, yaw: p.yaw }; };
  await sleep(400);
  const truthDrawn = v3d.drawn?.truth;
  v3d.truth = null;
  state.paused = true;
  await sleep(600);
  missions.emit("status", { phase: "home", text: "Flying home." });
  await sleep(300);
  const homeCaption = v3d.ui.next.textContent, homePath = v3d.path;
  missions.emit("done", {});
  state.paused = false;
  check("3D view: the simulator's true pose shows apart from the estimate; on the way home the caption says so and no old path is drawn ahead", truthDrawn === true && homeCaption === "Going home to the pad" && homePath == null,
    `truth drawn ${truthDrawn}; caption "${homeCaption}", path ${homePath ? `${homePath.length} points` : "none"}`);
  // a tap on the drone follows it (no "select": the app would leave the 3D view); a tap doesn't keep the keyboard
  v3d.setFollow(false);
  await sleep(300);
  const dHit = v3d.hits.find((hh) => hh.kind === "drone"), selects = events.filter(([k]) => k === "v3d.select").length;
  if (dHit) v3d.pick([dHit.u, dHit.v]);
  const cv = $("view3d"), r0c = cv.getBoundingClientRect();
  cv.focus();
  for (const type of ["pointerdown", "pointerup"]) cv.dispatchEvent(new PointerEvent(type, { bubbles: true, pointerId: 1, pointerType: "mouse", isPrimary: true, clientX: r0c.left + 5, clientY: r0c.top + 5, button: 0 }));
  const az0 = v3d.cam?.az, key = new KeyboardEvent("keydown", { key: "ArrowLeft", bubbles: true, cancelable: true });
  cv.dispatchEvent(key);
  check("3D view: a tap on the drone follows it without leaving the 3D view; a tap doesn't keep the arrow keys from the simulator", !!dHit && v3d.follow && events.filter(([k]) => k === "v3d.select").length === selects
    && document.activeElement !== cv && !key.defaultPrevented && v3d.cam?.az === az0, `drone ${dHit ? "tapped" : "not drawn"}; follow ${v3d.follow}; focus on the canvas ${document.activeElement === cv}; arrow taken ${key.defaultPrevented}`);
  // the map card's default size (308 x 150): the words and the buttons don't cover each other
  const box = document.querySelector(".pc-3d"), keepStyle = box.getAttribute("style");
  box.setAttribute("style", "width: 308px; height: 196px");
  await sleep(500);
  const rc = (sel) => document.querySelector(sel).getBoundingClientRect(), cap = rc(".v3d-where"), bar = rc(".v3d-bar"), badge = rc(".v3d-badge"), canvasR = rc("#view3d");
  const overlap = (a, b) => a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom;
  check("3D view in a narrow card (308 px): caption on top, one row of buttons at the bottom, short badge, nothing overlapping", !overlap(cap, bar) && !overlap(badge, bar) && bar.height < 40 && bar.bottom <= canvasR.bottom + 1 && cap.width > 0.5 * canvasR.width - 20,
    `canvas ${Math.round(canvasR.width)}×${Math.round(canvasR.height)}; caption ${Math.round(cap.width)} px "${v3d.ui.where.textContent}"; buttons ${Math.round(bar.width)}×${Math.round(bar.height)} at the ${bar.top > canvasR.top + canvasR.height / 2 ? "bottom" : "top"}; badge "${document.querySelector(".v3d-badge").innerText}"`);
  keepStyle == null ? box.removeAttribute("style") : box.setAttribute("style", keepStyle);
  await sleep(300);
  // a change pin in the 3D view opens the card
  v3d.home();
  await sleep(600);
  const pin = await until(() => v3d.hits.find((hh) => hh.kind === "change" && hh.id === boxChange.id), 3000);
  if (pin) v3d.pick([pin.u, pin.v]);
  const opened = await until(() => card.id === boxChange.id && $("changeCard").querySelectorAll(".chg-pic img").length === 2, 3000);
  const imgs = [...$("changeCard").querySelectorAll(".chg-pic img")];
  await until(() => imgs.every((i) => i.complete && i.naturalWidth > 0), 3000);
  check("change card: a tap on the change pin opens it with the live and the expected picture and Claude's verdict", !!pin && !!opened && imgs.every((i) => i.naturalWidth > 0) && result.numbers.evidence.box > 0.01,
    `pin ${pin ? `at ${Math.round(pin.u)}, ${Math.round(pin.v)}` : "not drawn"}; pictures ${imgs.map((i) => `${i.naturalWidth}x${i.naturalHeight}`).join(", ")} (the box is ${Math.round(100 * result.numbers.evidence.box)}% of the live one); "${$("changeCard").querySelector(".chg-verdict")?.textContent}"`);
  // On a phone, Show on map folds the card to one line so the map it centred shows (and the line never widens the page)
  if (matchMedia("(max-width: 900px)").matches) {
    [...$("changeCard").querySelectorAll("button")].find((b) => b.textContent === "Show on map")?.click();
    await sleep(300);
    const tall = $("changeCard").getBoundingClientRect().height, folded = card.collapsed && tall < 90, wide = document.documentElement.scrollWidth - (visualViewport?.width ?? innerWidth);
    [...$("changeCard").querySelectorAll("button")].find((b) => b.textContent === "Decide")?.click();
    await sleep(200);
    check("change card on a phone: Show on map folds it to one line, without widening the page; Decide opens it again", folded && wide <= 1 && !card.collapsed && events.some(([k]) => k === "card.show"),
      `folded to ${Math.round(tall)} px; page ${wide > 0 ? `${Math.round(wide)} px too wide` : "fits"}; open again ${!card.collapsed}`);
  }
  // Ask Claude on the door (waiting for consent, after "Not now"): what is sent, to whom and the cost on the card; the first
  // tap asks "Send these 2 pictures?" in place; only Send sends them
  if (doorChange) {
    card.show(doorChange.id);
    claude.answerConsent(false); // the banner's "Not now"
    await sleep(300);
    const before = asked.change, hint = $("changeCard").querySelector(".chg-hint")?.textContent ?? "", ask = [...$("changeCard").querySelectorAll("button")].find((b) => /Ask Claude/.test(b.textContent));
    ask?.click();
    await sleep(300);
    const confirmText = $("changeCard").querySelector(".chg-ask p")?.textContent ?? "", sentEarly = asked.change - before;
    [...$("changeCard").querySelectorAll(".chg-ask button")].find((b) => b.textContent === "Send them")?.click();
    const said = await until(() => memory.changes.find((c) => c.id === doorChange.id)?.claude?.status === "answered", 5000);
    check("change card: Ask Claude shows what it sends, to whom and the cost; after Not now it asks in place first; Send sends the two pictures once; the change waits for the user", /2 pictures to Anthropic's Claude API with your API key \((about \$\d\.\d\d|less than \$0\.01)\)/.test(hint) && /^Send these 2 pictures to Claude\?/.test(confirmText) && sentEarly === 0 && !!said && asked.change === before + 1 && memory.changes.find((c) => c.id === doorChange.id).status === "suspected",
      `hint "${hint}"; asked "${confirmText.slice(0, 90)}…"; ${sentEarly} sent before Send, ${asked.change - before} after; "${$("changeCard").querySelector(".chg-verdict")?.textContent}"`);
    claude.declined = false;
    const dismiss = [...$("changeCard").querySelectorAll("button")].find((b) => /^Dismiss/.test(b.textContent));
    dismiss?.click();
    check("change card: Dismiss resolves it in the memory (by you) and says so", memory.changes.find((c) => c.id === doorChange.id).status === "dismissed" && events.some(([k, e]) => k === "card.resolved" && e.id === doorChange.id),
      `"${$("changeCard").querySelector(".chg-note")?.textContent}" · "${$("changeCard").querySelector(".chg-effect")?.textContent}"`);
    card.show(boxChange.id);
  }
  // History
  await sleep(400);
  const flights = $("history").querySelectorAll(".hist-flight"), tiles = [...$("history").querySelectorAll(".hist-seen img")];
  await until(() => tiles.every((i) => i.complete && i.naturalWidth > 0), 3000);
  flights[1]?.click();
  const trail = events.findLast(([k]) => k === "hist.trail");
  await sleep(400);
  check("history: flights by day with rooms and what it saw; a click shows that flight's trail in 3D", flights.length === memory.flights.length && !!trail?.[1].flightId && v3d.flightId === trail[1].flightId && v3d.drawn?.flight === trail[1].flightId && v3d.drawn.trail > 10,
    `${flights.length} flights (${[...$("history").querySelectorAll(".hist-day h5")].map((e) => e.textContent).join(", ")}); "${flights[1]?.querySelector("strong")?.textContent}"; 3D view: ${v3d.drawn?.trail} trail pieces, ${v3d.drawn?.lost} losses`);
  check("history: last seen people and pets with their pictures", tiles.length >= 3 && tiles.every((i) => i.naturalWidth > 0),
    [...$("history").querySelectorAll(".hist-seen small")].map((e) => e.textContent).join(" | "));
  hist.ask("where did you last see the cat?");
  check("history: the question box answers from the memory, with the picture", /cat/i.test($("history").querySelector(".hist-reply p")?.textContent ?? "") && !!$("history").querySelector(".hist-reply img"),
    $("history").querySelector(".hist-reply p")?.textContent);
  const row = $("history").querySelector(".hist-change .hist-row");
  row?.click();
  check("history: a change opens its card in place", !!$("history").querySelector(".hist-card .chg-card"), $("history").querySelector(".hist-card .chg-title")?.textContent ?? "");
  // Coverage
  await until(() => cov.report, 8000);
  const sug = $("coverage").querySelector(".cov-sugg .btn");
  sug?.click();
  const shownAt = events.findLast(([k]) => k === "cov.show")?.[1];
  check("coverage: the score, a row per room, rescans in words with Show on map", !!cov.report && $("coverage").querySelectorAll(".cov-tr:not(.cov-th)").length === map.rooms.length && (!cov.report.suggestions.length || Number.isFinite(shownAt?.z)),
    `${cov.report?.score}%: ${$("coverage").querySelector(".cov-sugg li span")?.textContent ?? "no rescans"}`);
  // Survey: estimate, start (the OK), review
  const est = survey.estimate();
  check("survey: the cost first, in dollars and pictures", /About \$\d+\.\d\d/.test($("survey").querySelector(".svy-cost")?.textContent ?? ""), $("survey").querySelector(".svy-cost")?.textContent);
  const rooms = q.get("rooms")?.split(",");
  if (rooms) survey.est = estimateSurvey({ house, map, settings, rooms, model: claude.model });
  $("survey").querySelector(".row .btn")?.click();
  const ts = performance.now(), reviewed = await until(() => survey.state === "review" || (survey.state === "idle" && survey.error), 300000, 250);
  await sleep(1500);
  const crops = [...$("survey").querySelectorAll(".svy-crop")];
  const drawn = crops.filter((c) => { const d = c.getContext("2d").getImageData(c.width / 2, c.height / 2, 1, 1).data; return d[3] > 0; }).length;
  check("survey: progress, then Claude's suggestions with a picture each, ticked by the rules", survey.state === "review" && crops.length === survey.res.landmarks.length + survey.res.hazards.length && drawn === crops.length,
    `${((performance.now() - ts) / 1000).toFixed(0)} s, ${asked.survey} requests; ${survey.res?.rooms.length} rooms, ${survey.res?.landmarks.length} landmarks, ${survey.res?.hazards.length} hazards; ${drawn}/${crops.length} pictures; ticked: ${survey.picks ? survey.picks.rooms.size + survey.picks.landmarks.size + survey.picks.hazards.size : 0}; ${survey.error ?? ""}`);
  result.numbers.survey = { estimate: est.total, cost: survey.res?.cost, rooms: survey.res?.rooms.map((r) => ({ name: r.name, suggested: r.suggestedName, seen: r.seen })) };
  // Recordings
  await until(() => recheck.recs, 4000);
  const recBtn = $("recheck").querySelector(".rck-h .btn");
  recBtn?.click();
  const checked = await until(() => [...recheck.runs.values()][0]?.state === "done", 240000, 250);
  const run = [...recheck.runs.values()][0];
  check(`recordings: Check localization gives plain pass/fail words (replay: ${replayHow})`, !recheck.recs.length || (!!checked && !!$("recheck").querySelector(".rck-verdict")),
    recheck.recs.length ? `${recheck.recs.length} recordings; "${$("recheck").querySelector(".rck-verdict")?.textContent ?? run?.error}"; ${run?.checks.length} lines` : "no recordings in the server's data folder");
  // Calibration from that recording (it has no turn on the pad: a plain refusal, or a result)
  if (recheck.recs.length) {
    await calib.load();
    [...$("calib").querySelectorAll("button")].find((b) => b.textContent === "Calibrate from it")?.click();
    await until(() => calib.state === "idle" && (calib.result || calib.error), 180000, 250);
    check("calibration from a recording: a result or a refusal in plain words, the lens unchanged on a refusal", !!(calib.result || /^Calibration didn't work: .+\. The lens is unchanged\.$/.test(calib.error ?? "")),
      calib.result ? $("calib").querySelector(".cal-result p")?.textContent : calib.error);
  }
  // Calibration, live (made up), to its result
  const live = [...$("calib").querySelectorAll("button")].find((b) => b.textContent === "Start");
  live?.click();
  await until(() => calib.result, 8000);
  await until(() => calib.result?.picture, 6000);
  await sleep(300);
  check("calibration: progress, then the result in words with the before/after picture", !!calib.result && !!$("calib").querySelector(".cal-pic"),
    `${$("calib").querySelector(".cal-result p")?.textContent ?? calib.error}; picture ${calib.result?.picture ? "from the twin" : "missing"}`);
  await moreChecks();
  // Layout and errors
  const vw = Math.round(visualViewport?.width ?? innerWidth), over = document.documentElement.scrollWidth - vw; // a phone's layout viewport grows with what overflows it: the visual one doesn't
  check("layout: nothing wider than the window", over <= 1, `${vw} px window, ${document.documentElement.scrollWidth} px content`);
}

// The fixes after review: nothing runs in the air, undone means undone, words that agree, older flights read back.
async function moreChecks() {
  const box = () => $("extra").appendChild(document.createElement("div"));
  // Calibration: a live result is in use until saved (Discard puts the saved lens back); a new map of the same house keeps
  // it; nothing starts in the air; the live way is off in the simulator
  {
    const restored = [];
    calib.restoreFn = () => restored.push(1);
    calib.setHouse(house, twin, map);
    const kept = !!calib.result, words = $("calib").querySelector(".cal-result")?.textContent ?? "";
    [...$("calib").querySelectorAll("button")].find((b) => /^Discard/.test(b.textContent))?.click();
    const back = restored.length === 1 && !calib.result;
    state.flying = true;
    await sleep(1300);
    const noteAir = /Land first/.test($("calib").textContent);
    await calib.run("recording");
    const refused = /^Land first/.test(calib.error ?? "") && calib.state === "idle";
    state.flying = false;
    settings.values.mode = "sim";
    calib.render();
    const start = [...$("calib").querySelectorAll("button")].find((b) => b.textContent === "Start"), simOff = start?.disabled && /needs the real drone's camera/.test($("calib").textContent);
    delete settings.values.mode;
    calib.error = null;
    calib.render();
    check("calibration: a live result is in use until saved and Discard puts the saved lens back; a new map of the same house keeps it; nothing starts in the air; no live calibration in the simulator",
      kept && /In use now until you save or discard it/.test(words) && back && noteAir && refused && simOff,
      `kept ${kept}; "${words.match(/In use now[^.]*\./)?.[0] ?? "no in-use words"}"; restore calls ${restored.length}; in the air: "${calib.error ?? "(cleared)"}"; simulator Start disabled ${!!start?.disabled}`);
    [...$("calib").querySelectorAll("button")].find((b) => b.textContent === "Start")?.click(); // a result again, for finish()
    await until(() => calib.result, 8000);
  }
  // The localization check: its verdict agrees with its lines; a check that didn't finish says "Not checked"; never in the air
  {
    const files = { meta: { house: house.id, app: { mode: "real" }, lens: {} }, index: [{ t: 0 }, { t: 60000 }], commands: [],
      telemetry: [{ t: 0, fm: "STAB", sticks: { thr: 0 }, est: { heading: 0 } }, { t: 60000, fm: "STAB", sticks: { thr: 0 }, est: { heading: 0 } }] };
    const pass = { ok: true, title: "The camera found the drone's position all through this flight.", lines: [{ id: "video", ok: true, text: "Video: 1800 of 1800 frames decoded." }, { id: "fixes", ok: true, text: "Position fixes from the camera: 5 a second." }] };
    let air = false;
    const host = box(), rk = new RecheckPanel(host, { recordings: [{ id: "rec-a", label: "flight" }], house, settings, fetchRec: async () => structuredClone(files), replay: async () => pass, flying: () => air });
    await rk.check("rec-a");
    const v1 = host.querySelector(".rck-verdict"), cal1 = [...host.querySelectorAll("button")].some((b) => /Calibrate/.test(b.textContent));
    files.commands = [{ tool: "pad", args: { x: home.x, y: home.y, yaw: 0 } }, { tool: "picture", args: { sx: 240, sy: 0, sw: 1440, sh: 1080 } }];
    await rk.check("rec-a");
    const v2 = host.querySelector(".rck-verdict")?.textContent, cal2 = [...host.querySelectorAll("button")].some((b) => /Calibrate/.test(b.textContent));
    rk.replay = async () => { throw new Error("twin worker failed"); };
    await rk.check("rec-a");
    const v3 = host.querySelector(".rck-verdict"), folded = !!host.querySelector("details.rck-files"), cal3 = [...host.querySelectorAll("button")].some((b) => /Calibrate/.test(b.textContent));
    rk.replay = (id, { signal }) => new Promise((_, no) => signal.addEventListener("abort", () => no(new DOMException("Aborted", "AbortError"))));
    const running = rk.check("rec-a");
    await sleep(300);
    air = true;
    await Promise.race([running, sleep(3000)]);
    const v4 = host.querySelector(".rck-verdict")?.textContent ?? "";
    await rk.check("rec-a");
    const v5 = host.querySelector(".rck-verdict")?.textContent ?? "", btnOff = host.querySelector(".rck-h .btn")?.disabled;
    rk.dispose();
    check("localization check: no pad press with a passing replay is not a pass; a check that didn't finish says Not checked in plain words; calibrate only after a good real one; never in the air",
      v1?.dataset.ok === "false" && v1.textContent !== pass.title && !cal1 && v2 === pass.title && cal2 && v3?.dataset.ok === "false" && /^Not checked: the replay stopped before the end/.test(v3.textContent) && !/worker/.test(v3.textContent) && folded && !cal3
        && /^Not checked: the drone took off/.test(v4) && /^Not checked: the drone is flying/.test(v5) && btnOff,
      `no pad: "${v1?.textContent}"; with pad: "${v2}"; failed: "${v3?.textContent}"; took off: "${v4}"; in the air: "${v5}"`);
  }
  // Coverage in the air (no report worked out): plain words, then the report after landing
  {
    let air = true;
    const host = box(), c2 = new CoveragePanel(host, { house, map, vox, coverageReport: (o) => (air ? null : coverageReport(o)), flying: () => air });
    await sleep(300);
    const inAir = host.textContent;
    air = false;
    const after = await until(() => c2.report, 4000);
    c2.dispose();
    check("coverage: in the air it says it's worked out on the ground (no program error), and shows after landing", /worked out on the ground/.test(inAir) && !/Cannot|null/.test(inAir) && !!after, `"${inAir}"; after landing: ${after ? `${after.score}%` : "nothing"}`);
  }
  // History and the 3D view on older flights whose trails are no longer in RAM (a memory in this page's storage, 30 samples in RAM)
  {
    const m2 = await HouseMemory.open(`${house.id}-panels-check`, { ramSamples: 30 });
    await m2.clear();
    m2.setHouse({ house, map, vox, save: null });
    let tt = Date.now() - 4 * 3600e3;
    for (let f = 0; f < 3; f++) {
      const s = walk(pathThrough([pad, centreOf(R1), pad]), tt).slice(0, 40);
      m2.startFlight({ kind: "patrol", mission: { kind: "patrol" } }, tt);
      s.forEach((p, i) => m2.samplePose(i >= 10 && i < 14 ? { status: "lost" } : { ...p, sigma: 0.05, status: "ok", room: map.roomAt(p.x, p.y)?.id ?? null }, p.t));
      m2.endFlight("Patrolled the living room.", (tt = s.at(-1).t + 1000));
      tt += 3600e3;
    }
    await m2.flush();
    const inRam = m2.flights.filter((f) => m2.trails.has(f.id)).length, host = box(), h2 = new HistoryPanel(host, { memory: m2, map, settings });
    const shown = await until(() => [...host.querySelectorAll(".hist-flight small")].filter((e) => /position lost 1 time/.test(e.textContent)).length === 3, 4000);
    v3d.setMemory(m2);
    await v3d.setFlight(m2.flights[0].id);
    await sleep(400);
    const drawn = { ...v3d.drawn }, lostRows = [...host.querySelectorAll(".hist-flight small")].filter((e) => /position lost/.test(e.textContent)).length;
    v3d.setMemory(memory);
    await v3d.setFlight(null);
    h2.dispose();
    await m2.clear();
    check("history and 3D view: an older flight's trail no longer in RAM is read back (its loss counted, its path drawn)", inRam < 3 && !!shown && drawn.trail > 5 && drawn.lost >= 1,
      `${3 - inRam} of 3 trails out of RAM; history shows "position lost" on ${lostRows} of 3 flights; 3D view drew ${drawn.trail} pieces, ${drawn.lost} losses`);
  }
  // The survey: a new map of the same house keeps the paid review; throwing it away asks first; Stop and take-off in words
  {
    const reviewBefore = survey.state === "review";
    survey.setHouse(house, map, vox, surveyTwin);
    const stays = reviewBefore && survey.state === "review" && !!$("survey").querySelector(".svy-list");
    [...$("survey").querySelectorAll(".svy-apply button")].find((b) => b.textContent === "Discard")?.click();
    const ask = $("survey").querySelector(".svy-confirm")?.textContent ?? "";
    [...$("survey").querySelectorAll(".svy-confirm button")].find((b) => b.textContent === "Keep")?.click();
    const keptReview = survey.state === "review";
    let air = false;
    const host = box(), s2 = new SurveyPanel(host, { house, map, vox, twin: surveyTwin, settings, claude, flying: () => air, onApply: async () => ({}) });
    const stopAt = async (how) => {
      s2.start();
      await until(() => s2.state === "running" && s2.progress?.index >= 1, 60000, 100);
      if (how === "you") [...host.querySelectorAll("button")].find((b) => b.textContent === "Stop")?.click();
      else air = true;
      await until(() => s2.state !== "running", 10000, 100);
      const t = host.textContent;
      s2.discard();
      air = false;
      return t;
    };
    const byYou = await stopAt("you"), byAir = await stopAt("flying");
    s2.dispose();
    const plain = (t) => !/abort|\.\./i.test(t);
    check("survey: a new map of the same house keeps the review; Discard asks first (with the cost); Stop and a take-off are said in plain words",
      stays && /^Throw away this \$\d+\.\d\d survey\?/.test(ask) && keptReview && /You stopped the survey|Stopped: nothing was changed/.test(byYou) && plain(byYou) && /took off/.test(byAir) && plain(byAir),
      `stays ${stays}; "${ask.slice(0, 40)}"; Stop: "${byYou.match(/(You stopped[^.]*|Stopped:[^.]*)\./)?.[0]}"; take-off: "${byAir.match(/[^.]*took off[^.]*\./)?.[0]}"`);
  }
}

window.finish = async () => {
  const before = (house.landmarks ?? []).filter((l) => l.source === "claude").length, kBefore = (house.keepouts ?? []).filter((k) => k.source === "claude").length;
  if (survey.state === "review") {
    survey.countEl.click();
    await until(() => survey.state === "done", 5000);
  }
  const a = applied.at(-1);
  check("survey: Apply puts the ticked items on the map (source claude), the glass in the 3D map, and says the real cost", survey.state === "done" && !!a
    && (house.landmarks ?? []).filter((l) => l.source === "claude").length - before === a.changes.landmarks.length && (house.keepouts ?? []).filter((k) => k.source === "claude").length - kBefore === a.changes.keepouts.length,
    `${a ? `${a.changes.rooms.length} names, ${a.changes.landmarks.length} landmarks, ${a.changes.keepouts.length} no-fly zones, ${a.voxels} voxels` : "not applied"}; "${$("survey").textContent.slice(0, 160)}"`);
  if (calib.result) [...$("calib").querySelectorAll("button")].find((b) => b.textContent === "Save it")?.click();
  const saved = await until(async () => (await store.readFile(house.id, "calib-check.json")) && !calib.result, 4000);
  check("calibration: Save hands calib.json to save() and the panel says it is calibrated", !!saved && /^Calibrated/.test($("calib").querySelector(".cal-now")?.textContent ?? ""), $("calib").querySelector(".cal-now")?.textContent);
  check("no console errors", !errors.length, errors.slice(0, 5).join(" | "));
  return result;
};

if (q.get("auto") !== "0") {
  try {
    await checks();
  } catch (e) {
    log("ERROR", String(e?.stack || e));
    check("runs", false, String(e?.message || e));
  }
}
result.numbers.view3d = v3d.stats();
result.numbers.events = events.map(([k, e]) => `${k} ${e?.kind ?? e?.id ?? e?.flightId ?? ""}`);
result.done = true;
status("done");
