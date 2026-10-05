// Avoiding things in the house fixture (docs/HOME-DRONE.md, "Wave C contracts"), headless: the simulator's house world with
// the real controller, localizer (the simulator's truth), safety layer with nav/avoid.js, mission runner, house memory and
// change detector (nav/changes.js). Live depth is a depth camera made from the world's own geometry (walls, furniture,
// scene changes, people as upright cylinders): what vision/depth.js delivers once aligned, a 100° pinhole view at 5 Hz,
// as late as the video; the twin's expected depth is the same camera in the house as captured (no changes, no people).
// Scene changes come from the simulator (sim.addObstacle, sim.setDoor). Then avoid-cases.mjs's avoidCases(): what the
// real drone needs beyond that (a box before an open doorway, height uncertainty). Wave C2: a hover pushed off the wall while
// return home waits (whatever the caps looking ahead say, with the height sinking), a false suspected change looked at and
// dropped, one Claude called no change passed slowly on the way home, and a room sealed behind closed doors. Fix 1: the
// battery's way home passes it too, every mission does after a closer look, a patrol in a room it can't cross looks from
// where it is, a doorway seen closed from one oblique spot isn't a door leaf, braking measured over three runs. Fix 2:
// return home looks at a doorway only the live depth sees closed from in front of it, as the other missions do; a scan by
// a doorway with the flow misreading the climb rate keeps its spot and height. Fix 3: a door leaf needs 3 frames from 2
// viewpoints, never from a doorway's sliver at a grazing angle, nor right after it was seen through; what the camera sees
// ahead counts only where the pose's error (a height 0.2 m off) doesn't explain it.
// Usage: cd tools && node test-avoid.mjs   (ONLY=<part of a name> runs those)
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { rig, now, runTests } from "./test-sim.mjs";

const I = await import("../app/js/house/import.js");
const P = await import("../app/js/house/planner.js");
const { Localizer } = await import("../app/js/nav/localizer.js");
const { Safety } = await import("../app/js/safety.js");
const { MissionRunner } = await import("../app/js/missions.js");
const { AVOID, Avoid, freeRun, stoppable, stopDistance, safeSpot, staticClearance } = await import("../app/js/nav/avoid.js");
const { ChangeDetector, CHANGES } = await import("../app/js/nav/changes.js");
const { HouseMemory } = await import("../app/js/memory/memory.js");
const { makeHouseWorld } = await import("../app/js/sim/twin-world.js");
const { intrinsics, unproject, project } = await import("../app/js/twin/lens.js");
const { droneCamera } = await import("../app/js/twin/pose.js");
const { Move, ScanAt, ReturnHome, BrakeTest } = await import("../app/js/behaviors.js");
const { DEG } = await import("../app/js/util.js");
const V = await import("../app/js/house/voxels.js").catch(() => null);
const { alignDepth } = await import("../app/js/vision/depth.js");

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
export const { house, map } = await I.importCapture(fsSource(FIXTURE));
const HOME = house.home;
const tick = () => new Promise((r) => setImmediate(r));
const fmt = (v) => (Number.isFinite(v) ? v.toFixed(2) : String(v));
const forever = () => new Promise(() => {});
export const DEPTH_VIEW = { width: 56, height: 42, hfov: 100, uptilt: 20 };

// Metres along each pixel's ray of a pinhole view from a drone pose { x, y, z (camera), yaw, pitch, roll }: the world's
// faces at the height the ray has there, the floor and ceiling (each room's own), and with `actors` the people and pets
// as upright cylinders. With `rgb`, a picture too: each surface's colour under a pattern fixed to the house (so a view of
// the same place looks the same from anywhere). -> { width, height, lens, depth, rgb? }
export function depthCamera(world, pose, { width = DEPTH_VIEW.width, height = DEPTH_VIEW.height, hfov = DEPTH_VIEW.hfov, uptilt = DEPTH_VIEW.uptilt, actors = true, rgb = false } = {}) {
  const lens = { model: "pinhole", hfovDeg: hfov, uptiltDeg: uptilt }, K = intrinsics(lens, width, height), { p, R } = droneCamera(pose, uptilt);
  const depth = new Float32Array(width * height), bodies = actors ? world.sprites().filter((s) => s.label) : [], px = rgb ? new Uint8ClampedArray(4 * width * height) : null, hit = {};
  for (let v = 0; v < height; v++)
    for (let u = 0; u < width; u++) {
      const r = unproject(K, u + 0.5, v + 0.5), d = [0, 1, 2].map((a) => R[a][0] * r[0] + R[a][1] * r[1] + R[a][2] * r[2]), i = v * width + u;
      depth[i] = cast(world, p, d, bodies, 12, hit);
      if (!px) continue;
      const [x, y, z] = [0, 1, 2].map((a) => p[a] + d[a] * depth[i]), k = 1 + 0.15 * Math.sin(9 * x + 3 * y) * Math.sin(11 * y - 4 * z) + 0.1 * Math.sin(23 * (x + z));
      for (let c = 0; c < 3; c++) px[4 * i + c] = hit.color[c] * k;
      px[4 * i + 3] = 255;
    }
  return { width, height, lens, depth, ...(px && { rgb: { width, height, data: px } }) };
}
function cast(world, o, d, bodies, max = 12, hit = {}) {
  let best = max;
  hit.color = [235, 232, 228];
  const flat = (f, at, color) => {
    const t = f == null ? -1 : (f - o[2]) / d[2];
    if (!(t > 0 && t < best)) return;
    const g = at(o[0] + d[0] * t, o[1] + d[1] * t), t2 = g == null ? t : (g - o[2]) / d[2];
    best = Math.min(best, t2 > 0 ? t2 : t);
    hit.color = color;
  };
  if (d[2] < -1e-6) flat(world.floorAt(o[0], o[1]), (x, y) => world.floorAt(x, y), [150, 118, 86]);
  if (d[2] > 1e-6) flat(world.ceilingAt(o[0], o[1]), (x, y) => world.ceilingAt(x, y), [235, 232, 228]);
  const hn = Math.hypot(d[0], d[1]);
  if (hn > 1e-6)
    for (const h of world.castAll(o[0], o[1], d[0] / hn, d[1] / hn, best * hn)) {
      const t = h.dist / hn, z = o[2] + d[2] * t;
      if (t >= best) break;
      if (z >= h.seg.zMin && z <= h.seg.zMax) {
        best = t;
        hit.color = h.seg.solid?.color ?? h.seg.color ?? [200, 196, 188];
        break;
      }
    }
  for (const s of bodies) {
    const ox = o[0] - s.x, oy = o[1] - s.y, a = d[0] * d[0] + d[1] * d[1], b = 2 * (ox * d[0] + oy * d[1]), c = ox * ox + oy * oy - s.w * s.w, disc = b * b - 4 * a * c;
    if (a < 1e-9 || disc < 0) continue;
    const t = (-b - Math.sqrt(disc)) / (2 * a), z = o[2] + d[2] * t;
    if (t > 0 && t < best && z >= s.z0 && z <= s.z1) (best = t), (hit.color = [90, 70, 60]);
  }
  return best;
}

// An Avoid fed live depth frames of `live` (the expected view: `scan`) from each pose in turn (frames each, 200 ms apart):
// "leaf" (a door leaf on doorway id), "maybe" (read closed, not sure yet) or "none". far: what is nearer than the scan reads
// this many times too far; sliver: every pixel whose ray crosses that doorway's opening reads at its plane (the depth's
// soft edges filling it), or with near, its depth pulled that share of the way toward the plane (a far side read too near);
// through: a pose the doorway is first seen through from (2 frames of the true view).
export function doorLeaf(live, scan, id, poses, { frames = 4, far = 1, sliver = null, near = 0, through = null, after = null } = {}) {
  const av = new Avoid({ map, localizer: { ctl: { videoDelay: 0, est: { ttc: Infinity } }, pose: () => poses[0], velocity: () => [0, 0] }, settings: { get: (k) => ({ mode: "sim", avoid: true })[k] } });
  const frame = (pose, plain = false) => {
    const f = depthCamera(live, pose), expected = depthCamera(scan, pose, { actors: false }).depth;
    if (far !== 1) f.depth = f.depth.map((d, i) => (d < expected[i] - 0.2 ? Math.min(expected[i], d * far) : d));
    if (sliver && !plain) {
      const dr = sliver, K = intrinsics(f.lens, f.width, f.height), { p, R } = droneCamera(pose, f.lens.uptiltDeg), ex = dr.b[0] - dr.a[0], ey = dr.b[1] - dr.a[1];
      for (let v = 0; v < f.height; v++)
        for (let u = 0; u < f.width; u++) {
          const r = unproject(K, u + 0.5, v + 0.5), d = [0, 1, 2].map((a) => R[a][0] * r[0] + R[a][1] * r[1] + R[a][2] * r[2]), den = d[0] * ey - d[1] * ex;
          const t = ((dr.a[0] - p[0]) * ey - (dr.a[1] - p[1]) * ex) / den, s = ((dr.a[0] - p[0]) * d[1] - (dr.a[1] - p[1]) * d[0]) / den, z = p[2] + t * d[2];
          if (Math.abs(den) > 1e-9 && t > 0 && s > 0 && s < 1 && z > dr.sillZ && z < dr.headZ && f.depth[v * f.width + u] > t + 0.05)
            f.depth[v * f.width + u] = near ? t + (1 - near) * (f.depth[v * f.width + u] - t) : t * (1 + 0.1 * Math.sin(u + v)); // (near: the far side pulled that share toward the plane)
        }
    }
    return { ...f, expected, pose };
  };
  let t = 1000;
  if (through) for (let i = 0; i < 2; i++) (av.ingest({ ...frame(through, true), t }, t), (t += 200));
  for (const pose of poses) for (let i = 0; i < frames; i++) (av.ingest({ ...frame(pose), t }, t), (t += 200));
  const state = () => ([...av.temps.values()].some((q) => q.source === "door leaf" && q.door === id) ? "leaf" : av.leafMaybe?.has(`leaf-${id}`) ? "maybe" : "none");
  let out = state();
  if (after) { // { wait (ms with no frames), world (seen from the last pose next, 2 frames) } -> "leaf, then <state after the wait>, then <after looking>"
    av.expire((t += after.wait));
    const kept = state();
    [live, sliver] = [after.world, null];
    for (let i = 0; i < 2; i++) (av.ingest({ ...frame(poses.at(-1)), t }, t), (t += 200));
    out = `${out}, then ${kept}, then ${state()}`;
  }
  for (const k of [...av.temps.keys()]) av.removeTemp(k);
  return out;
}

// An Avoid fed live depth frames with pictures (140 x 105: the browser's grid of cells) along `poses` (250 ms apart) in
// `live`, the expected view and the frames' pose dz m above each (σ 0.05, σz zSigma); misread: what is nearer than the
// scan reads as the scan's own depth behind it. -> { cue: per frame "" | "slow" | "stop", shares, wedges, temps }
function wayAheadRun(live, scan, poses, { dz = 0, zSigma = 0.1, misread = false } = {}) {
  let P = null;
  const av = new Avoid({ map, localizer: { ctl: { videoDelay: 0, est: { ttc: Infinity } }, pose: () => P, velocity: () => [0, 0] } }), out = { cue: [], shares: [], wedges: [] };
  av.on("temp", (t) => t.added && t.source === "way ahead" && out.wedges.push(t));
  poses.forEach((pose, i) => {
    const t = 1000 + 250 * i, at = { ...pose, z: pose.z + dz }, f = depthCamera(live, pose, { width: 140, height: 105, rgb: true }), e = depthCamera(scan, at, { width: 140, height: 105, rgb: true, actors: false });
    if (misread) f.depth = f.depth.map((d, j) => (d < e.depth[j] - 0.05 ? e.depth[j] : d));
    av.ingest({ ...f, t, pose: (P = { ...at, sigma: 0.05, zSigma, status: "ok" }), expected: e.depth, expectedRgb: e.rgb, depthErr: 0.5 }, t);
    const a = av.ahead?.t === t ? av.ahead : null;
    out.cue.push(a?.stop ? "stop" : a?.slow ? "slow" : "");
    out.shares.push(a ? a.share : 0);
  });
  out.temps = [...av.temps.values()];
  for (const k of [...av.temps.keys()]) av.removeTemp(k); // (off the shared map)
  return out;
}

// People and pets as boxes in the depth view (a perfect detector), for the change detector's mask.
function boxesIn(world, pose, f) {
  const K = intrinsics(f.lens, f.width, f.height), { p, R } = droneCamera(pose, f.lens.uptiltDeg), out = [];
  for (const s of world.sprites().filter((q) => q.label)) {
    const px = [];
    for (const [dx, dy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]])
      for (const z of [s.z0, s.z1]) {
        const v = [s.x + dx * s.w - p[0], s.y + dy * s.w - p[1], z - p[2]], c = [0, 1, 2].map((a) => R[0][a] * v[0] + R[1][a] * v[1] + R[2][a] * v[2]);
        if (c[2] > 0.05) px.push(project(K, c));
      }
    if (!px.length) continue;
    const xs = px.map((q) => q[0] / f.width), ys = px.map((q) => q[1] / f.height), x0 = Math.max(0, Math.min(...xs)), y0 = Math.max(0, Math.min(...ys));
    const x1 = Math.min(1, Math.max(...xs)), y1 = Math.min(1, Math.max(...ys));
    if (x1 > x0 && y1 > y0) out.push({ x: x0, y: y0, w: x1 - x0, h: y1 - y0, label: s.label });
  }
  return out;
}

// The simulator in the house with the autonomy stack, a house memory and live depth. People and pets are taken out unless
// `keep` names them. depth: false flies on the map (and flow) alone. pictures: frames carry the live and expected pictures;
// misread: what is nearer than the scan reads this many times too far (no farther than the scan), as monocular depth can.
export async function stack({ autonomy = "full", seed = 7, videoDelay = 0, keep = [], depth = true, changes = true, settings: extra = {}, pictures = false, misread = 1 } = {}) {
  for (const id of [...(map.temps?.keys() ?? [])]) map.removeTemp(id);
  const r = rig({ house, map, autonomy, seed, videoDelay });
  const { sim, ctl, values } = r;
  Object.assign(values, extra);
  sim.world.actors.list = sim.world.actors.list.filter((a) => keep.includes(a.id));
  const settings = ctl.settings;
  const localizer = new Localizer({ ctl, map, house, settings });
  localizer.setSource("truth");
  localizer.setTruth(() => ({ x: sim.drone.x, y: sim.drone.y, z: sim.drone.z, yaw: sim.drone.yaw }));
  const safety = new Safety({ map, localizer, settings, house });
  ctl.safety = safety;
  const memory = await HouseMemory.open(house.id, { indexedDB: null, now: () => 1.7e12 + now() });
  memory.setHouse({ house, map });
  const missions = new MissionRunner({ ctl, map, house, localizer, perception: r.perception, settings, memory });
  const detector = changes ? new ChangeDetector({ map, house, memory, avoid: safety.avoid }) : null;
  const pristine = makeHouseWorld(house, map, { seed, cast: [] });
  const watch = { contacts: 0, hardest: 0, steps: 0, onStep: null, frames: 0, depthMs: 0 };
  const collide = sim.world.collide.bind(sim.world);
  sim.world.collide = (body, rad, h) => {
    const v = collide(body, rad, h);
    if (v > 0 && body === sim.drone) {
      watch.contacts++;
      watch.hardest = Math.max(watch.hardest, v);
      if (process.env.DEBUG) console.log(`      contact ${fmt(v)} m/s at (${fmt(body.x)}, ${fmt(body.y)}) ${ctl.behavior?.label} ${safety.reason}`);
    }
    return v;
  };
  // Live depth: captured every 200 ms (when flying), delivered videoDelay later with the pose it was taken from.
  const queue = [];
  let next = 0;
  const feed = () => {
    const t = now(), d = sim.drone;
    if (depth && d.airborne && t >= next) {
      next = t + 200;
      const pose = { x: d.x, y: d.y, z: d.z + 0.025, yaw: d.yaw, pitch: d.pitch, roll: d.roll }, t0 = process.hrtime.bigint();
      // exact depth, but taken to be as unsure as LiveDepth's (AVOID.depthErr, CHANGES.depthErr); a perfect detector's people and pets
      const f = { ...depthCamera(sim.world, pose, { rgb: pictures }), t, boxes: boxesIn(sim.world, pose, DEPTH_VIEW_LENS) };
      const e = depthCamera(pristine, pose, { actors: false, rgb: pictures });
      [f.expected, f.expectedRgb] = [e.depth, e.rgb];
      if (misread !== 1) for (let i = 0; i < f.depth.length; i++) if (f.depth[i] < e.depth[i] - 0.05) f.depth[i] = Math.min(e.depth[i], f.depth[i] * misread);
      watch.depthMs += Number(process.hrtime.bigint() - t0) / 1e6;
      queue.push(f);
    }
    while (queue.length && now() - queue[0].t >= videoDelay) {
      const f = queue.shift(), L = localizer.poseAt(f.t);
      f.pose = { ...L, z: L.z + 0.025, pitch: sim.drone.pitch, roll: sim.drone.roll, sigma: localizer.pose().sigma, status: localizer.pose().status };
      safety.avoid.ingest(f);
      detector?.ingest(f);
      watch.frames++;
    }
  };
  const drive = async (promise, maxSeconds = 300) => {
    let out = null, err = null;
    promise.then((v) => (out = { v }), (e) => (err = e));
    for (let t = 0; t < maxSeconds && !out && !err; t += 1 / 60) {
      r.wait(1 / 60);
      watch.steps++;
      feed();
      watch.onStep?.(t);
      await tick();
    }
    if (err) throw err;
    return out ? out.v : { ok: false, summary: "test timeout", timeout: true, findings: [] };
  };
  return { ...r, settings, localizer, safety, missions, memory, detector, watch, drive };
}
const DEPTH_VIEW_LENS = { width: DEPTH_VIEW.width, height: DEPTH_VIEW.height, lens: { model: "pinhole", hfovDeg: DEPTH_VIEW.hfov, uptiltDeg: DEPTH_VIEW.uptilt } };

// Distance from the drone (centre) to a box { x, y, w, d } (axis-aligned).
const toBox = (d, b) => Math.hypot(Math.max(0, Math.abs(d.x - b.x) - b.w / 2), Math.max(0, Math.abs(d.y - b.y) - b.d / 2));

// Hover at (x, y) facing yaw (H).
async function placeAt(s, x, y, yaw) {
  const r = await s.drive(s.missions.run({ kind: "goTo", target: { x, y } }), 60);
  assert.ok(r.ok, r.summary);
  await s.drive(s.ctl.run(new ScanAt({ x, y, headings: [yaw] }, { localizer: s.localizer, map, labels: [], look: 0.3 })), 15);
}

const tests = {
  "stopping: the cap stops in time at every speed (stoppable and stopDistance agree; the map's free run along a ray)"() {
    for (const T of [0.25, 0.5])
      for (const d of [0.1, 0.5, 1, 2]) {
        const v = stoppable(d, T);
        assert.ok(Math.abs(stopDistance(v, T) - d) < 1e-9, `T ${T} d ${d}`);
      }
    const c = P.roomCenter(map, "r1"), run = freeRun(map, { x: c[0], y: c[1], z: map.floorAt(...c) + 1 }, [1, 0], 0.09);
    assert.ok(run > 0.3 && run < 4, `free run ${fmt(run)} m east of the living room's middle`);
    const spot = safeSpot(map, { x: HOME.x, y: HOME.y }, { sigma: 0.15 });
    assert.ok(spot && map.clearance(spot.x, spot.y, map.floorAt(spot.x, spot.y) + 0.6) >= 0.15 + 0.3, "a clear spot near the pad");
  },

  async "a box dropped on the route mid-flight: the drone stops at least 0.3 m short, plans around it and arrives (live depth)"() {
    for (const videoDelay of [0, 250]) {
      const s = await stack({ seed: 31, videoDelay }), goal = [3.9, 1.4], box = { id: "box", x: 1.7, y: 2.0, w: 0.6, d: 0.6, h: 1.4 };
      let dropped = false, minBox = Infinity, stop = null;
      const seen = [];
      s.safety.avoid.on("temp", (t) => t.added && t.source === "live depth" && seen.push(t));
      s.watch.onStep = () => {
        const d = s.sim.drone;
        if (!dropped && d.airborne && d.x > -0.2 && d.y > 1.0) {
          s.sim.addObstacle(box); // on the way through the doorway into the living room, 1.5 m ahead
          dropped = { at: [d.x, d.y], dist: toBox(d, box) };
        }
        if (dropped) minBox = Math.min(minBox, toBox(d, box));
        if (dropped && stop == null && Math.hypot(d.vx, d.vy) < 0.05) stop = toBox(d, box);
      };
      const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 150), d = s.sim.drone;
      console.log(`      ${videoDelay} ms: box dropped ${fmt(dropped.dist)} m ahead; stopped ${fmt(stop)} m from it, closest ${fmt(minBox)} m, "${r.summary}" ${fmt(Math.hypot(d.x - goal[0], d.y - goal[1]))} m from the goal; ` +
        `live-depth obstacle(s) at ${seen.map((t) => `(${fmt(t.x)}, ${fmt(t.y)}) r ${fmt(t.r)}`).join(", ")}; ${s.watch.frames} depth frames (${fmt(s.watch.depthMs / Math.max(1, s.watch.frames))} ms each in Node), ${s.watch.contacts} contacts`);
      assert.ok(dropped, "the box was dropped");
      assert.ok(stop >= 0.3 && minBox >= 0.3, `stopped ${fmt(stop)} m from the box, came within ${fmt(minBox)} m`);
      assert.ok(r.ok && Math.hypot(d.x - goal[0], d.y - goal[1]) < 0.3, r.summary);
      assert.ok(seen.some((t) => Math.hypot(t.x - box.x, t.y - box.y) < 0.6), "the box became a temporary obstacle on the map");
      assert.equal(s.watch.contacts, 0);
      s.sim.removeObstacle("box");
    }
  },

  // Monocular depth can read a plain new thing far too far (a box 0.8 m ahead as 2-3 m in tools/depth-check.html): here
  // as the scan's own depth behind it. The picture still shows something the scan lacks filling the way ahead.
  async "a plain box the live depth reads as the wall behind it: what the camera sees ahead stops the drone short, and it goes around"() {
    const out = [];
    for (const pictures of [false, true]) {
      const s = await stack({ seed: 31, videoDelay: 250, pictures, misread: Infinity }), goal = [3.9, 1.4], box = { id: "box", x: 1.7, y: 2.0, w: 0.6, d: 0.6, h: 1.4, color: "#7a5a3a" };
      let dropped = false, minBox = Infinity, stop = null, why = "";
      s.watch.onStep = () => {
        const d = s.sim.drone;
        if (!dropped && d.airborne && d.x > -0.2 && d.y > 1.0) (s.sim.addObstacle(box), (dropped = { dist: toBox(d, box) }));
        if (dropped) minBox = Math.min(minBox, toBox(d, box));
        if (dropped && stop == null && Math.hypot(d.vx, d.vy) < 0.05) [stop, why] = [toBox(d, box), s.safety.avoid.last.why];
      };
      const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 150), d = s.sim.drone;
      out.push({ pictures, r, minBox, contacts: s.watch.contacts, hardest: s.watch.hardest, at: Math.hypot(d.x - goal[0], d.y - goal[1]) });
      console.log(`      ${pictures ? "with" : "without"} the pictures: box dropped ${fmt(dropped.dist)} m ahead; ${stop != null ? `stopped ${fmt(stop)} m from it ("${why}"), ` : ""}closest ${fmt(minBox)} m, ` +
        `${s.watch.contacts ? `${s.watch.contacts} contact steps up to ${fmt(s.watch.hardest)} m/s, ` : ""}"${r.summary}" ${fmt(out.at(-1).at)} m from the goal`);
      s.sim.removeObstacle("box");
    }
    const q = out[1];
    assert.ok(q.minBox >= 0.3 && q.contacts === 0, `came within ${fmt(q.minBox)} m (${q.contacts} contacts)`);
    assert.ok(q.r.ok && q.at < 0.3, q.r.summary);
  },

  async "a closed door: reported as door-closed at that doorway, the drone goes through another; every door closed: it says what blocks it"() {
    const door = map.doors.find((q) => q.id === "r2:w4-o1"); // Room 2 -> Living room
    const s = await stack({ seed: 32 }), goal = [2.0, 2.2], changes = [];
    s.sim.setDoor(door.id, false);
    s.memory.on("change", (c) => changes.push(c));
    let doors = [];
    s.missions.on("path", (p) => doors.push(p.doors.map((q) => q.id).join(" > ")));
    const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 150), d = s.sim.drone;
    const ch = s.memory.changes.find((c) => c.kind === "door-closed");
    console.log(`      "${r.summary}" via ${doors.join(" | then ")}; memory: ${s.memory.changes.map((c) => `${c.kind} ${c.door ?? ""} at (${fmt(c.x)}, ${fmt(c.y)}) ${c.status}`).join("; ")}; ${s.watch.contacts} contacts`);
    assert.ok(ch && ch.door === door.id, "a closed door at r2:w4-o1 in memory");
    assert.ok(Math.hypot(ch.x - (door.a[0] + door.b[0]) / 2, ch.y - (door.a[1] + door.b[1]) / 2) < 0.6, `at (${fmt(ch.x)}, ${fmt(ch.y)})`);
    assert.ok(r.ok && Math.hypot(d.x - goal[0], d.y - goal[1]) < 0.3, r.summary);
    assert.ok(doors.at(-1).includes("r3"), `went another way: ${doors.at(-1)}`);
    assert.equal(s.watch.contacts, 0);
    // Every way out of Room 2 closed.
    const t = await stack({ seed: 33 });
    for (const id of ["r2:w4-o1", "r2:w1-o1+r3:w2-o1"]) t.sim.setDoor(id, false);
    const r2 = await t.drive(t.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 150);
    console.log(`      both doors closed: "${r2.summary}"; memory: ${t.memory.changes.map((c) => `${c.kind} ${c.door ?? ""}`).join("; ")}`);
    assert.ok(!r2.ok && /looks closed/.test(r2.summary), r2.summary);
    assert.equal(t.watch.contacts, 0);
    for (const id of ["r2:w4-o1", "r2:w1-o1+r3:w2-o1"]) t.sim.setDoor(id, true);
  },

  async "a person standing in the way: the drone waits or goes around, never comes within 1.5 m, and arrives once they leave"() {
    for (const videoDelay of [0, 250]) {
      const s = await stack({ seed: 34, videoDelay, keep: ["person1"] }), a = s.sim.world.actors.list[0], goal = [-0.7, 5.2];
      a.place(-0.6, 3.3, "stand", 1e6); // Room 3, just past the opening from Room 2: on the way
      // While they stand there the drone keeps 1.5 m; when they walk off (into the living room, through the doorway next
      // to the drone) it never flies toward them inside 1.5 m.
      let minD = Infinity, toward = Infinity, waited = 0, phase = "stand";
      s.watch.onStep = (t) => {
        const d = s.sim.drone, dist = Math.hypot(d.x - a.x, d.y - a.y), closing = (d.vx * (a.x - d.x) + d.vy * (a.y - d.y)) / dist;
        if (d.airborne && phase === "stand") minD = Math.min(minD, dist);
        if (d.airborne && closing > 0.15) toward = Math.min(toward, dist);
        if (s.ctl.behavior?.name === "hold") waited += 1 / 60;
        if (t > 12 && phase === "stand") (phase = "walk"), Object.assign(a, { path: [[-0.6, 3.3, 0], [0.2, 3.4, 0], [1.6, 4.6, 0], [2.4, 5.1, 0]], i: 0, goal: { then: "stand" }, state: "walk" });
        if (phase === "walk" && a.state !== "walk") (phase = "gone"), a.place(2.4, 5.1, "stand", 1e6);
      };
      const t0 = now(), r = await s.drive(s.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 150), d = s.sim.drone;
      console.log(`      ${videoDelay} ms: closest ${fmt(minD)} m while they stood there, ${fmt(toward)} m flying toward them, held ${waited.toFixed(0)} s, "${r.summary}" after ${((now() - t0) / 1000).toFixed(0)} s, ${fmt(Math.hypot(d.x - goal[0], d.y - goal[1]))} m from the goal`);
      assert.ok(minD >= 1.5 && toward >= 1.5, `came within ${fmt(minD)} m, ${fmt(toward)} m flying toward them`);
      assert.ok(r.ok && Math.hypot(d.x - goal[0], d.y - goal[1]) < 0.3, r.summary);
      assert.equal(s.watch.contacts, 0);
    }
  },

  async "return home past a blocker: home once the way clears; with a person on the pad, a clear spot nearby and why"() {
    // Someone stands in Room 2 by the doorways for 20 s (every way into Room 2 passes within 1.5 m of them), then goes
    // to the far end of Room 3.
    const s = await stack({ seed: 35, keep: ["person1"] }), a = s.sim.world.actors.list[0];
    a.place(-1.4, 5.4, "stand", 1e6);
    await placeAt(s, 2.6, 4.8, Math.PI);
    a.place(0.0, 2.3, "stand", 1e6);
    let minD = Infinity, phase = "stand";
    s.watch.onStep = (t) => {
      if (s.sim.drone.airborne && phase === "stand") minD = Math.min(minD, Math.hypot(s.sim.drone.x - a.x, s.sim.drone.y - a.y));
      if (t > 20 && phase === "stand") (phase = "walk"), Object.assign(a, { path: [[0.0, 2.3, 0], [-0.6, 3.4, 0], [-1.4, 5.4, 0]], i: 0, goal: { then: "stand" }, state: "walk" });
      if (phase === "walk" && a.state !== "walk") (phase = "gone"), a.place(-1.4, 5.4, "stand", 1e6);
    };
    const texts = [];
    s.missions.on("status", (st) => texts.push(st.text));
    const r = await s.drive(s.missions.run({ kind: "returnHome" }), 200), d = s.sim.drone;
    console.log(`      the way blocked for 20 s: "${r.summary}" (${[...new Set(texts)].join(" / ")}), ${fmt(Math.hypot(d.x - HOME.x, d.y - HOME.y))} m from the pad, closest to the person ${fmt(minD)} m`);
    assert.ok(r.ok && !d.airborne && Math.hypot(d.x - HOME.x, d.y - HOME.y) < 0.4, r.summary);
    assert.ok(minD >= 1.5, `came within ${fmt(minD)} m`);
    // Someone stays on the pad: it keeps trying while the battery allows, then lands on a clear spot and says why.
    const t = await stack({ seed: 36, keep: ["person1"] }), b = t.sim.world.actors.list[0];
    b.place(-0.3, 4.4, "stand", 1e6);
    await placeAt(t, 2.6, 4.8, Math.PI);
    b.place(HOME.x + 0.3, HOME.y + 0.5, "stand", 1e6);
    t.sim.drone.soc = 0.4;
    let minB = Infinity;
    t.watch.onStep = () => t.sim.drone.airborne && (minB = Math.min(minB, Math.hypot(t.sim.drone.x - b.x, t.sim.drone.y - b.y)));
    const t0 = now(), r2 = await t.drive(t.missions.run({ kind: "returnHome" }), 300), e = t.sim.drone;
    const clear = map.clearance(e.x, e.y, (map.floorAt(e.x, e.y) ?? 0) + 0.6);
    console.log(`      someone on the pad: "${r2.summary}" after ${((now() - t0) / 1000).toFixed(0)} s, landed ${!e.airborne} at (${fmt(e.x)}, ${fmt(e.y)}), ${fmt(Math.hypot(e.x - b.x, e.y - b.y))} m from the person (closest ${fmt(minB)}), ${fmt(clear)} m clear, battery ${Math.round(e.soc * 100)}%`);
    assert.ok(!e.airborne && !e.crashed, "landed");
    assert.ok(/couldn't get home/.test(r2.summary) && /person/.test(r2.summary), r2.summary);
    assert.ok(Math.hypot(e.x - b.x, e.y - b.y) >= 1.5 && minB >= 1.5, "away from the person");
    assert.ok(e.soc > 0.1, `${Math.round(e.soc * 100)}% left`);
    assert.equal(t.watch.contacts + s.watch.contacts, 0);
  },

  async "full speed at a wall with 250 ms video delay: the cap stops it short (and without it?)"() {
    const row = [];
    for (const capOn of [false, true])
      for (const speed of capOn ? [0.55, 1.0] : [1.0]) { // without the cap (the map's geofence alone): at full speed only
        const s = await stack({ seed: 37, videoDelay: 250, depth: capOn });
        if (!capOn) s.safety.avoid.limit = (sp) => sp;
        await placeAt(s, 2.0, 4.9, 0); // facing east along the living room's north side, 3 m from the wall
        let closest = Infinity;
        s.watch.onStep = () => (closest = Math.min(closest, 5.0 - s.sim.drone.x));
        await s.drive(s.ctl.run(new Move("forward", 6, speed)), 15);
        await s.drive(forever(), 2);
        row.push({ capOn, speed, contacts: s.watch.contacts, hardest: s.watch.hardest, closest });
        console.log(`      ${capOn ? "with" : "without"} the cap at ${speed} m/s: ${s.watch.contacts ? `contact at ${fmt(s.watch.hardest)} m/s` : `stopped ${fmt(closest)} m from the wall`}`);
      }
    for (const q of row.filter((q) => q.capOn)) assert.ok(q.contacts === 0 && q.closest > map.o.droneRadius, `${q.speed} m/s with the cap: ${q.contacts} contacts`);
  },

  async "unknown space: a place off the map is refused without flying; with voxels, unknown space is neither planned into nor flown into"() {
    const s = await stack({ seed: 38, depth: false });
    const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: 6.5, y: 1.0 } }), 10);
    console.log(`      off the map: "${r.summary}" (airborne ${s.sim.drone.airborne})`);
    assert.ok(!r.ok && /can't find a way/.test(r.summary) && !s.sim.drone.airborne, r.summary);
    if (!V?.VoxelMap?.forMap || !map.setVoxels) return console.log("      (no voxel map in this build: skipped the 3D part)");
    // Everything in the rooms known free except the living room east of x = 3.4, never seen.
    const vox = V.VoxelMap.forMap(map), X = 3.4;
    for (let i = 0; i < vox.n; i++) {
      const [x, y, z] = vox.center(i), k = map.idx(x, y);
      if (k < 0 || map.room[k] < 0 || map.wall[k] || x > X) continue;
      const h = z - map.floorZ[k];
      if (h < 0.1 || z > map.ceilZ[k] - 0.1) continue;
      vox.set(i, map.blocked[map.bandOf(h)][k] ? 64 : -40);
    }
    vox.computeClearance();
    map.setVoxels(vox);
    try {
      const t = await stack({ seed: 39, depth: false });
      let far = -Infinity;
      t.watch.onStep = () => (far = Math.max(far, t.sim.drone.x));
      const r2 = await t.drive(t.missions.run({ kind: "goTo", target: { x: 4.1, y: 1.2 } }), 60);
      console.log(`      to a spot in the unknown part: "${r2.summary}", furthest east x ${fmt(far)}`);
      assert.ok(far < X - map.o.droneRadius && /as close as I can get/.test(r2.summary), r2.summary);
      await placeAt(t, 2.4, 4.8, 0);
      far = -Infinity;
      await t.drive(t.ctl.run(new Move("forward", 2.5, 0.5)), 10);
      await t.drive(forever(), 2);
      console.log(`      flying east at 0.5 m/s toward it from x 2.4: stopped at x ${fmt(far)} (unknown from ${X}), ${t.watch.contacts} contacts`);
      assert.ok(far < X - map.o.droneRadius, `reached x ${fmt(far)}`);
    } finally {
      map.setVoxels(null);
    }
  },

  // C1's integration run: 728 contact steps while return home waited with an accurate position (σ 0.04 m), over a low cabinet
  // by Room 2's west wall, the height sinking 0.88 -> 0.61 m, "too close to something on the map" the whole time.
  async "hovering while return home waits, drifting toward the west wall over a low cabinet, the height sinking, something new in view ahead: the push holds it off the wall, the height is kept, no contact"() {
    const out = [];
    for (const [drift, sink] of [[0.15, 0], [0.15, 0.15]]) {
      const s = await stack({ seed: 51, pictures: true, changes: false }), box = { id: "box", x: -0.55, y: 1.15, w: 0.5, d: 0.5, h: 1.5 };
      await placeAt(s, -1.45, 1.15, 0); // over the cabinet's east edge (0.7 m tall, against the wall), facing east
      s.sim.addObstacle(box); // something new 0.65 m ahead: the camera's way-ahead stop says no closer that way
      map.addTemp({ id: "block", kind: "change", change: "obstacle", polygon: [[-2.5, 0.45], [1.5, 0.45], [1.5, 0.7], [-2.5, 0.7]], until: Infinity }); // the way home
      s.sim.drone.trimDrift = { x: -drift * 1.1, y: 0 };
      if (sink) {
        const per = s.perception, update = per.update.bind(per), vfov = s.values.hfov * DEG * 0.75;
        per.update = () => (update(), per.latest.flow && (per.latest.flow.dy += sink / (vfov * 2.2))); // the flow says it climbs: the controller's own height sinks
      }
      let minC = Infinity, low = Infinity, stopped = 0;
      s.watch.onStep = () => {
        const d = s.sim.drone;
        if (!d.airborne) return;
        minC = Math.min(minC, map.clearance(d.x, d.y, d.z));
        low = Math.min(low, d.z - d.floorZ);
        s.safety.avoid.ahead = { t: now(), share: 0.8, deep: 0.5, backed: true, slow: true, stop: true }; // the camera: no closer that way
        stopped++;
      };
      const rh = new ReturnHome({ localizer: s.localizer, map, home: house.home, battery: () => 200, patience: 60 });
      await s.drive(s.ctl.run(rh), 25);
      out.push({ drift, sink, contacts: s.watch.contacts, minC, low, stopped, phase: rh.phase });
      console.log(`      drift ${drift} m/s toward the wall${sink ? `, the height biased down ${sink} m/s` : ""}: ${s.watch.contacts} contact steps, closest ${fmt(minC)} m to the map, lowest ${fmt(low)} m up (the cabinet: 0.7 m), the way ahead "stop" for ${(stopped / 60).toFixed(0)} s, still ${rh.phase}ing`);
      map.removeTemp("block");
      s.sim.removeObstacle("box");
      await s.drive(s.missions.run({ kind: "returnHome" }), 60);
    }
    for (const q of out) assert.ok(q.contacts === 0 && q.low > 0.75, `drift ${q.drift}, sink ${q.sink}: ${q.contacts} contacts, lowest ${fmt(q.low)} m`);
  },

  // C1: a phantom "obstacle" right after take-off (Claude: no change, 70%) stayed a blocking obstacle; return home retried
  // 15 times over 138 s.
  async "a false suspected change on the way (nothing is there): the drone takes a closer look, the change detector sees the spot as the scan has it and drops it, and the drone arrives"() {
    const s = await stack({ seed: 53 }), goal = [-0.7, 0.4], at = [-0.7, 1.4], f = map.floorAt(...at), texts = [];
    s.missions.on("status", (q) => texts.push(q.text));
    await placeAt(s, -0.7, 2.6, -Math.PI / 2); // Room 2, facing south down the hallway
    // floor to ceiling across the hallway (2.2 m wide) on the way to the pad's end of it
    const rec = s.memory.addChange({ kind: "obstacle", x: at[0], y: at[1], z: f + 1.0, size: 1.6, zMin: f, zMax: f + 2.2 });
    s.detector.reported.push({ id: "phantom", memoryId: rec.id, x: at[0], y: at[1], z: f + 1.0, size: 1.6, near: true, agree: 0 }); // as if this detector had reported it
    const blocked = !P.plan(map, [-0.7, 2.6], goal, { alt: 1.0, sigma: 0.17 }).ok;
    const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 90), d = s.sim.drone;
    console.log(`      the way ${blocked ? "blocked" : "open"} by the phantom; "${texts.find((t) => /closer look/.test(t)) ?? "no closer look"}"; the memory: ${rec.status}${rec.by ? ` by ${rec.by}` : ""}; "${r.summary}" ${fmt(Math.hypot(d.x - goal[0], d.y - goal[1]))} m from the goal, ${s.watch.contacts} contacts`);
    assert.ok(blocked, "the phantom blocks the way");
    assert.ok(texts.some((t) => /closer look/.test(t)) && rec.status === "dismissed", `${rec.status}`);
    assert.ok(r.ok && Math.hypot(d.x - goal[0], d.y - goal[1]) < 0.3, r.summary);
    assert.equal(s.watch.contacts, 0);
  },

  async "return home past a suspected change Claude called no change (85% sure; the change detector off): it passes slowly, watching with live depth, and lands on the pad (also from inside the change's disc)"() {
    for (const [y0, size] of [[2.6, 0.6], [2.6, 1.6]]) {
      const s = await stack({ seed: 55, changes: false }), at = [-0.7, 1.4], f = map.floorAt(...at), texts = [];
      s.missions.on("status", (q) => texts.push(q.text));
      await placeAt(s, -0.7, y0, -Math.PI / 2);
      const rec = s.memory.addChange({ kind: "obstacle", x: at[0], y: at[1], z: f + 0.5, size, zMin: f, zMax: f + 2.2 });
      s.memory.annotateChange(rec.id, { claude: { status: "answered", verdict: "no-change", kind: "none", what: "nothing new", confidence: 0.85, why: "the scan's smear" } });
      s.safety.avoid.eyes = () => ({ ok: true, why: "" }); // live depth sees (the fixture has no 3D map, which eyes() also wants)
      const t = map.temps.get(rec.id), inside = !!t && Math.hypot(-0.7 - t.x, y0 - t.y) < t.r; // its disc (place error twice over) may hold the drone: the plan escapes it, safety wouldn't
      const blocked = inside || !P.plan(map, [-0.7, y0], [HOME.x, HOME.y], { alt: 1.0, sigma: 0.17 }).ok;
      let near = 0;
      const filter = s.safety.filter.bind(s.safety);
      s.safety.filter = (sp, st) => { // what the safety layer lets the drone fly within 0.5 m of it while passing it
        const o = filter(sp, st), d = s.sim.drone;
        if (d.airborne && s.safety.avoid.passing.size && Math.hypot(d.x - at[0], d.y - at[1]) < (t?.r ?? 0.9) + 0.5) near = Math.max(near, Math.hypot(o.vx ?? 0, o.vy ?? 0));
        return o;
      };
      const r = await s.drive(s.missions.run({ kind: "returnHome" }), 150), d = s.sim.drone;
      console.log(`      ${size} m, the drone ${fmt(Math.hypot(-0.7 - at[0], y0 - at[1]))} m from it (its disc ${fmt(t?.r)} m): the way home ${inside ? "blocked (it starts inside)" : blocked ? "blocked" : "open"}; "${texts.find((t) => /passing it/.test(t)) ?? "no pass"}"; asked at most ${fmt(near)} m/s within 0.5 m of it; "${r.summary}" ${fmt(Math.hypot(d.x - HOME.x, d.y - HOME.y))} m from the pad`);
      assert.ok(blocked && texts.some((t) => /passing it slowly/.test(t)), "passed it");
      assert.ok(near <= AVOID.creep + 0.05, `${fmt(near)} m/s near it`); // the creep (the push off a wall may add a little)
      assert.ok(r.ok && !d.airborne && Math.hypot(d.x - HOME.x, d.y - HOME.y) < 0.4, r.summary);
      assert.ok(rec.status === "suspected" && !s.safety.avoid.passing.size, "still for the user to look at; no longer passed");
      assert.equal(s.watch.contacts, 0);
      s.memory.resolveChange(rec.id, "dismissed", "a test");
    }
  },

  // Blocker 12 (C1): the closed-door path was never flown end to end, and patrols said "patrolled" for a room seen only
  // through a closed doorway.
  async "Room 3 sealed (its three doorways closed): looking into it from the living room, every doorway is seen closed, it says it couldn't look into Room 3, and touches nothing"() {
    const s = await stack({ seed: 54 }), ids = ["r3:w3-o1", "r3:w4-o1", "r2:w1-o1+r3:w2-o1"], leaves = new Set();
    s.safety.avoid.on("temp", (t) => t.added && t.source === "door leaf" && leaves.add(t.door));
    await placeAt(s, 2.0, 3.4, Math.PI);
    for (const id of ids) s.sim.setDoor(id, false);
    try {
      const r = await s.drive(s.missions.run({ kind: "lookIn", room: "Room 3" }), 200);
      const closed = s.memory.changes.filter((c) => c.kind === "door-closed").map((c) => c.door);
      console.log(`      "${r.summary}"; door leaves: ${[...leaves].join(", ") || "none"}; door-closed changes: ${closed.join(", ") || "none"}; ${s.watch.contacts} contacts`);
      assert.ok(!r.ok && /couldn't look into Room 3/.test(r.summary) && /closed/.test(r.summary), r.summary);
      assert.ok(leaves.size + closed.length >= 1, "a closed doorway was seen");
      assert.ok([...leaves, ...closed].every((id) => ids.includes(id)), `an open doorway taken for closed: ${[...leaves, ...closed].join(", ")}`); // Room 2's doorway is in line with one of them
      assert.equal(s.watch.contacts, 0);
    } finally {
      for (const id of ids) s.sim.setDoor(id, true);
    }
  },

  "depth alignment: relative depth (unknown scale and shift, a bend, 3% noise) made metric against the empty-house depth; a new box keeps its own depth"() {
    const world = makeHouseWorld(house, map, { seed: 41, cast: [] }), plain = makeHouseWorld(house, map, { seed: 41, cast: [] });
    world.addObstacle({ id: "crate", x: 2.4, y: 2.0, z: 0, w: 0.5, d: 0.5, h: 0.9 });
    const pose = { x: 1.1, y: 2.1, z: 1.0, yaw: 0, pitch: 0, roll: 0 }, size = { width: 112, height: 84 };
    const live = depthCamera(world, pose, { ...size, actors: false }), exp = depthCamera(plain, pose, { ...size, actors: false }), K = intrinsics(live.lens, size.width, size.height);
    const rand = mulberry(9), f = (u, v) => Math.hypot((u + 0.5 - K.cx) / K.fx, (v + 0.5 - K.cy) / K.fy, 1);
    const disp = new Float32Array(size.width * size.height); // what a monocular model says: 2.7 / z + 0.4, bent by ±4% across the frame, noisy
    for (let v = 0; v < size.height; v++)
      for (let u = 0; u < size.width; u++) {
        const i = v * size.width + u, z = live.depth[i] / f(u, v);
        disp[i] = (2.7 / z + 0.4) * (1 + 0.04 * (u / size.width - 0.5) + 0.03 * (rand() - 0.5) * 2);
      }
    const a = alignDepth(disp, exp.depth, K, { rand }), crate = [], rest = [];
    for (let i = 0; i < disp.length; i++) {
      const rel = Math.abs(a.depth[i] - live.depth[i]) / live.depth[i];
      if (live.depth[i] < exp.depth[i] - 0.2) crate.push(rel);
      else if (live.depth[i] < 6) rest.push(rel);
    }
    const med = (v) => v.sort((x, y) => x - y)[v.length >> 1];
    console.log(`      scale ${fmt(a.scale)} (2.7), shift ${fmt(a.shift)} (0.4), ${Math.round(a.inliers * 100)}% agree; error ${fmt(med(rest) * 100)}% elsewhere, ${fmt(med(crate) * 100)}% on the crate (${crate.length} px), absRel vs the twin ${fmt(a.absRel)}`);
    assert.ok(Math.abs(a.scale - 2.7) < 0.3 && med(rest) < 0.05 && med(crate) < 0.1 && crate.length > 50);
  },

  "change detector: people, a pose a few cm off and depth noise make no change; a new box does, at its place"() {
    const world = makeHouseWorld(house, map, { seed: 40, cast: [] }), plain = makeHouseWorld(house, map, { seed: 40, cast: [] }), people = makeHouseWorld(house, map, { seed: 40 });
    const person = people.actors.list.find((a) => a.id === "person1");
    const det = new ChangeDetector({ map, house }), rand = mulberry(3), noisy = (d) => d.map((v) => v * (1 + 0.04 * (rand() - 0.5) * 2));
    const pose = (i) => ({ x: 1.1 + 0.05 * i, y: 2.1, z: 1.0, yaw: 0.05 - 0.01 * i, pitch: 0, roll: 0, sigma: 0.05, status: "ok" }); // the living room, facing east
    let blobs = 0;
    for (let i = 0; i < 20; i++) {
      const p = pose(i), off = { ...p, x: p.x + 0.04, y: p.y - 0.03, yaw: p.yaw + 0.01 }; // the pose used is a few cm and 0.6° off
      person.place(2.9, 2.6 - 0.05 * i, "stand", 1e6);
      const live = depthCamera(people, p), f = { ...live, depth: noisy(live.depth), depthErr: 0.05, t: i * 200, pose: off, expected: depthCamera(plain, off, { actors: false }).depth, boxes: boxesIn(people, p, live) };
      blobs += det.ingest(f, i * 200).length;
    }
    console.log(`      20 frames with a person walking by, the pose 5 cm off, ±4% depth noise: ${blobs} blob(s), ${det.stats.changes} change(s)`);
    assert.equal(det.stats.changes, 0);
    world.addObstacle({ id: "crate", x: 2.6, y: 2.0, z: 0, w: 0.5, d: 0.5, h: 0.7 });
    const found = [];
    det.on("change", (c) => found.push(c));
    for (let i = 0; i < 12; i++) {
      const p = pose(i), live = depthCamera(world, p);
      det.ingest({ ...live, depth: noisy(live.depth), depthErr: 0.05, t: 5000 + i * 200, pose: p, expected: depthCamera(plain, p, { actors: false }).depth }, 5000 + i * 200);
    }
    return new Promise((done) => setImmediate(() => {
      const c = found[0];
      console.log(`      a 0.5 m crate at (2.60, 2.00): ${found.map((q) => `${q.kind} at (${fmt(q.x)}, ${fmt(q.y)}, ${fmt(q.z)}) ${fmt(q.size)} m, ${fmt(q.zMin)}-${fmt(q.zMax)} m up, in ${q.room}`).join("; ")}`);
      assert.ok(c && c.kind === "obstacle" && Math.hypot(c.x - 2.6, c.y - 2.0) < 0.3, "the crate, within 0.3 m");
      assert.ok(c.z >= c.zMin && c.z <= c.zMax && c.zMin <= (map.floorAt(c.x, c.y) ?? 0) + 0.05 && c.zMax >= 0.6, `its height ${fmt(c.zMin)}-${fmt(c.zMax)} m, at ${fmt(c.z)} (the crate: floor to 0.7 m)`);
      assert.ok(map.temps.has(c.id), "on the map until resolved");
      det.resolve(c.id);
      assert.ok(!map.temps.has(c.id));
      done();
    }));
  },

  async "a change Claude takes for a person or pet keeps blocking while the detector still sees it, and lapses a minute after it's out of view"() {
    const out = [];
    for (const renew of [false, true]) {
      let clock = 1.75e12;
      const memory = await HouseMemory.open(`${house.id}:renew-${renew}`, { indexedDB: null, now: () => clock });
      memory.setHouse({ house, map });
      const world = makeHouseWorld(house, map, { seed: 42, cast: [] }), plain = makeHouseWorld(house, map, { seed: 42, cast: [] });
      world.addObstacle({ id: "crate", x: 2.6, y: 2.0, z: 0, w: 0.5, d: 0.5, h: 0.7 });
      const det = new ChangeDetector({ map, house, memory }), found = [], seen = memory.seen.bind(memory);
      let signs = 0;
      memory.seen = (...a) => (signs++, seen(...a));
      if (!renew) det.renew = () => {}; // before: a reported spot seen again said nothing
      det.on("change", (c) => found.push(c));
      const views = Array.from({ length: 12 }, (_, i) => { // passing it, back and forth
        const y = 1.3 + 0.07 * i, pose = { x: 1.3, y, z: 1.0, yaw: Math.atan2(2.0 - y, 1.3), pitch: 0, roll: 0, sigma: 0.05, status: "ok" };
        return { ...depthCamera(world, pose), pose, expected: depthCamera(plain, pose, { actors: false }).depth };
      });
      const view = (i, t) => det.ingest({ ...views[i % 12], depthErr: 0.05, t }, t);
      for (let i = 0; i < 18; i++) view(i, i * 200);
      await tick();
      const r = memory.changes.find((q) => q.id === found[0]?.memoryId);
      assert.ok(r && r.status === "suspected", "reported to the memory");
      memory.passing(r.id); // Claude: a person or pet
      let lapsed = null;
      for (let k = 0; k < 140 && lapsed == null; k++) { // 70 s more in view, a frame every 0.5 s
        clock += 500;
        view(k, 3000 + 500 * k);
        memory.expirePassing(clock);
        if (r.status !== "suspected") lapsed = (500 * (k + 1)) / 1000;
      }
      const blocking = r.status === "suspected" && map.temps.has(r.id);
      clock += 61000; // out of view a minute
      memory.expirePassing(clock);
      out.push({ renew, lapsed, blocking, after: r.status, onMap: map.temps.has(r.id), signs });
      console.log(`      ${renew ? "after" : "before"}: ${lapsed == null ? "still blocking after 70 s in view" : `dismissed after ${lapsed} s while still in view`} (the detector told the memory ${signs} times); a minute out of view: ${r.status}${map.temps.has(r.id) ? ", still on the map" : ""}`);
      memory.setHouse({});
      for (const id of [...map.temps.keys()]) map.removeTemp(id);
    }
    const q = out[1];
    assert.ok(q.lapsed == null && q.blocking && q.signs > 10, "kept blocking while in view");
    assert.ok(q.after === "dismissed" && !q.onMap, "lapsed once out of view");
  },

  // Wave C2 fix 1 (the flight reviews): the safety layer's own way home passes what return home passes; every mission
  // passes a change Claude called no change once a closer look didn't clear it; a patrol already in a room it can't get to
  // the spot of looks around from where it is; a doorway seen closed from one oblique spot only is no door leaf; braking
  // measured over several runs never predicts a shorter stop than the drone makes.
  async "the safety layer's way home (battery low) passes a suspected change Claude called no change (85%), slowly, as return home does, and lands on the pad"() {
    const s = await stack({ seed: 55, changes: false }), at = [-0.7, 1.4], f = map.floorAt(...at), texts = [];
    s.safety.on("status", (q) => texts.push(q.text));
    await placeAt(s, -0.7, 2.6, -Math.PI / 2);
    const rec = s.memory.addChange({ kind: "obstacle", x: at[0], y: at[1], z: f + 0.5, size: 0.6, zMin: f, zMax: f + 2.2 });
    s.memory.annotateChange(rec.id, { claude: { status: "answered", verdict: "no-change", kind: "none", what: "nothing new", confidence: 0.85, why: "the scan's smear" } });
    s.safety.avoid.eyes = () => ({ ok: true, why: "" });
    s.safety.battery = () => ({ vbat: 3.65, vComp: 3.65, soc: 0.5, secondsLeft: 100 }); // low voltage: the safety layer's trip home
    let passed = false, landed = null, done;
    const t0 = now(), down = new Promise((res) => (done = res));
    // landed: on the floor with the throttle cut (a touchdown alone may still lift off), seen on the step it happens (a
    // wall-clock timer here let the simulator run on a busy machine)
    s.watch.onStep = () => ((passed ||= s.safety.avoid.passing.size > 0), landed == null && !s.sim.drone.airborne && !s.ctl.airborne && now() - t0 > 3000 && done((landed = (now() - t0) / 1000)));
    await s.drive(down, 150);
    const d = s.sim.drone;
    console.log(`      "${texts.find((t) => /passing it/.test(t)) ?? texts.at(-1)}"; ${d.airborne ? "still flying" : `landed after ${fmt(landed)} s`} ${fmt(Math.hypot(d.x - HOME.x, d.y - HOME.y))} m from the pad; ${s.watch.contacts} contacts`); // C2: waited a minute
    assert.ok(passed && texts.some((t) => /passing it slowly/.test(t)), "passed it");
    assert.ok(!d.airborne && Math.hypot(d.x - HOME.x, d.y - HOME.y) < 0.4 && landed < 60, `landed after ${landed} s`);
    assert.equal(s.watch.contacts, 0);
    s.memory.resolveChange(rec.id, "dismissed", "a test");
  },

  async "every mission passes a suspected change Claude called no change once a closer look didn't clear it (a go-to down the hallway); a patrol already in a room whose spot it can't reach looks around from where it is"() {
    const s = await stack({ seed: 53, changes: false }), goal = [-0.7, 0.4], at = [-0.7, 1.4], f = map.floorAt(...at), texts = [];
    s.missions.on("status", (q) => texts.push(q.text));
    await placeAt(s, -0.7, 2.6, -Math.PI / 2);
    const rec = s.memory.addChange({ kind: "obstacle", x: at[0], y: at[1], z: f + 1.0, size: 1.6, zMin: f, zMax: f + 2.2 });
    s.memory.annotateChange(rec.id, { claude: { status: "answered", verdict: "no-change", kind: "none", what: "nothing new", confidence: 0.85, why: "the scan's smear" } });
    s.safety.avoid.eyes = () => ({ ok: true, why: "" });
    const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 120), d = s.sim.drone;
    console.log(`      go-to: "${texts.find((t) => /closer look/.test(t)) ?? "no closer look"}" then "${texts.find((t) => /passing it/.test(t)) ?? "no pass"}"; "${r.summary}" ${fmt(Math.hypot(d.x - goal[0], d.y - goal[1]))} m from the goal`);
    assert.ok(texts.some((t) => /closer look/.test(t)) && texts.some((t) => /passing it slowly/.test(t)) && r.ok && Math.hypot(d.x - goal[0], d.y - goal[1]) < 0.3, r.summary); // C2: "Something new ... blocks the way"
    assert.ok(rec.status === "suspected" && !s.safety.avoid.passing.size, "still the user's to look at; no longer passed");
    s.memory.resolveChange(rec.id, "dismissed", "a test");
    // the patrol: something new walls the drone into Room 2's south-west corner (the route from the pad to Room 2's spot
    // goes round it; from the drone, nothing does); once it looks around from where it is, the way opens again
    const stop = P.patrolRoute(map, ["r2"], [HOME.x, HOME.y], { alt: 1.0, sigma: 0.17 }).stops[0], f2 = map.floorAt(-1.3, 0.6);
    await placeAt(s, -1.3, 0.6, 0);
    const walls = [[[-0.85, -0.2], [-0.6, -0.2], [-0.6, 1.45], [-0.85, 1.45]], [[-1.95, 1.2], [-0.6, 1.2], [-0.6, 1.45], [-1.95, 1.45]]];
    walls.forEach((polygon, i) => map.addTemp({ id: `wall-test-${i}`, kind: "obstacle", source: "live depth", polygon, zMin: f2 - 0.1, zMax: f2 + 2.6, until: Infinity }));
    const open = s.missions.on("status", (q) => /from here/.test(q.text) && walls.forEach((_, i) => map.removeTemp(`wall-test-${i}`)));
    texts.length = 0;
    const pr = await s.drive(s.missions.run({ kind: "patrol", rooms: ["Room 2"] }), 200);
    console.log(`      patrol: the spot (${fmt(stop.x)}, ${fmt(stop.y)}) blocked: "${texts.find((t) => /from here/.test(t)) ?? "no look from here"}"; "${pr.summary}"`);
    assert.ok(texts.some((t) => /looking around from here/.test(t)) && /^Patrolled Room 2/.test(pr.summary), pr.summary); // C2: "I couldn't get to any room to patrol"
    open();
    walls.forEach((_, i) => map.removeTemp(`wall-test-${i}`));
    assert.equal(s.watch.contacts, 0);
  },

  // Fix 3: a leaf needs the doorway read closed on 3 frames in a row from 2 viewpoints (Room 2's open doorway was taken for
  // closed in 3 of 6 sealed runs in the browser, each time from one spot beside it).
  "door leaves: a closed doorway seen from one spot 60° off its normal isn't a leaf, nor from one spot head-on (one viewpoint); head-on from two spots 0.3 m apart on the way in, or from two bearings 20° apart, it is; its depth read 1.4 x too far, still"() {
    const id = "r2:w4-o1", dr = map.doors.find((d) => d.id === id), world = makeHouseWorld(house, map, { seed: 1, cast: [] }), shut = makeHouseWorld(house, map, { seed: 1, cast: [] });
    shut.setDoor(id, false);
    const mid = [(dr.a[0] + dr.b[0]) / 2, (dr.a[1] + dr.b[1]) / 2], L = Math.hypot(dr.b[0] - dr.a[0], dr.b[1] - dr.a[1]), n0 = [-(dr.b[1] - dr.a[1]) / L, (dr.b[0] - dr.a[0]) / L];
    const n = map.roomAt(mid[0] + n0[0], mid[1] + n0[1])?.id === "r1" ? n0 : [-n0[0], -n0[1]]; // toward the living room
    const at = ([dist, deg]) => {
      const a = (deg * Math.PI) / 180, u = [n[0] * Math.cos(a) - n[1] * Math.sin(a), n[1] * Math.cos(a) + n[0] * Math.sin(a)], x = mid[0] + dist * u[0], y = mid[1] + dist * u[1];
      return { x, y, z: map.floorAt(x, y) + 1.0, yaw: Math.atan2(mid[1] - y, mid[0] - x), pitch: 0, roll: 0, sigma: 0.05 };
    };
    const leaf = (spots, far = 1) => doorLeaf(shut, world, id, spots.map(at), { far });
    const out = { "60° off": leaf([[2.0, -60]]), "head-on, one spot": leaf([[1.2, 0]]), "head-on, 1.5 then 1.2 m": leaf([[1.5, 0], [1.2, 0]]), "30° then 10° off": leaf([[1.8, -30], [1.8, -10]]),
      "head-on, two spots, read 1.4 x too far": leaf([[1.5, 0], [1.2, 0]], 1.4) };
    console.log(`      ${Object.entries(out).map(([k, v]) => `${k}: ${v}`).join("; ")}`); // C2 fix 2: a leaf from one spot head-on
    assert.deepEqual(out, { "60° off": "none", "head-on, one spot": "maybe", "head-on, 1.5 then 1.2 m": "leaf", "30° then 10° off": "leaf", "head-on, two spots, read 1.4 x too far": "leaf" });
  },

  // Fix 3 (the browser's sealed runs, 3 of 6: Room 2's open doorway became a door leaf from beside it, facing along it):
  // the opening there is a sliver of the picture at a grazing angle and the depth's soft edges fill it with the jambs'
  // and walls' depth, so it read as on the doorway's plane.
  "door leaves: an open doorway whose sliver at a grazing angle reads at its own plane (the depth's soft edges), from two spots beside it, isn't a leaf; one seen through a moment before (1.5 s) isn't either, nor one whose far side reads 40% too near; the same doorway closed and approached is, and stays (40 s unseen) until it is seen through"() {
    const id = "r3:w3-o1", dr = map.doors.find((d) => d.id === id), world = makeHouseWorld(house, map, { seed: 1, cast: [] }), shut = makeHouseWorld(house, map, { seed: 1, cast: [] });
    shut.setDoor(id, false);
    const pose = (x, y, yaw) => ({ x, y, z: map.floorAt(x, y) + 1.0, yaw, pitch: 0, roll: 0, sigma: 0.05 });
    const beside = [pose(0.8, 3.6, Math.PI / 2), pose(0.8, 3.3, Math.PI / 2)], before = [pose(1.6, 3.63, Math.PI), pose(1.3, 3.63, Math.PI)];
    const graze = doorLeaf(world, world, id, beside, { sliver: dr }), seen = doorLeaf(world, world, id, before, { sliver: dr, through: before[0], frames: 3 }), closed = doorLeaf(shut, world, id, before);
    const kept = doorLeaf(shut, world, id, before, { after: { wait: 40000, world } }); // 40 s unseen, then the door opened and seen through
    // Room 2's open doorway from the living room 1.3-1.5 m off, Room 3 sealed behind it, its far side read 40% too near (a11d)
    const sealed = makeHouseWorld(house, map, { seed: 1, cast: [] }), r2 = map.doors.find((d) => d.id === "r2:w4-o1");
    for (const q of ["r2:w1-o1+r3:w2-o1", "r3:w3-o1", "r3:w4-o1"]) sealed.setDoor(q, false);
    const squeezed = doorLeaf(sealed, world, r2.id, [pose(1.6, 1.38, 2.15), pose(1.75, 1.28, 2.15), pose(1.55, 1.45, 2.2)], { sliver: r2, near: 0.4 });
    console.log(`      open, read on its plane from beside it: ${graze}; read so after it was seen through: ${seen}; closed, approached: ${closed}; unseen for 40 s, then seen open: ${kept}; Room 2's open doorway, its far side read 40% too near: ${squeezed}`); // C2 fix 2: a leaf from beside it; a leaf forgotten after 20 s; a leaf on Room 2's doorway
    assert.equal(graze, "none");
    assert.equal(squeezed, "none");
    assert.notEqual(seen, "leaf");
    assert.equal(closed, "leaf");
    assert.equal(kept, "leaf, then leaf, then none");
  },

  // Fix 3 (tools/depth-check.html with the height 0.2 m off: the way-ahead cue slowed on 12-22 of 240 quiet frames and
  // placed 1-4 wedges): the expected view rendered 0.2 m off moves a furniture top's edge, or a pattern on the floor,
  // across the window, in depth and in looks.
  "the way ahead with the localized height 0.2 m off: approaching furniture, what that error explains neither slows the drone nor places a wedge; a box ahead seen with the same error (or 0.1 m off, its depth read as the wall behind) still slows it, with a live-depth obstacle at it or a wedge"() {
    const plain = makeHouseWorld(house, map, { seed: 40, cast: [] }), line = (x, y, deg, z, n, step = 0.075) => Array.from({ length: n }, (_, i) => {
      const a = (deg * Math.PI) / 180, px = x + step * i * Math.cos(a), py = y + step * i * Math.sin(a);
      return { x: px, y: py, z: (map.floorAt(x, y) ?? 0) + z, yaw: a, pitch: 0, roll: 0 };
    });
    // toward the living room's furniture at 0.9 m, the estimate 0.2 m high (σz 0.1: 2 σz)
    const quiet = [line(1.25, 3.1, 0, 0.9, 5), line(1.294, 3.206, -45, 0.9, 5)].map((p) => wayAheadRun(plain, plain, p, { dz: 0.2 }));
    const boxed = makeHouseWorld(house, map, { seed: 40, cast: [] }), BOX = { id: "box", x: 1.9, y: 1.9, z: map.floorAt(1.9, 1.9) ?? 0, w: 0.6, d: 0.5, h: 1.4, color: "#7a5a3a" };
    boxed.addObstacle(BOX);
    const toBox = line(0.5, 1.9, 0, 1.0, 9), seen = wayAheadRun(boxed, plain, toBox, { dz: -0.2 }), looks = [-0.1, 0].map((dz) => wayAheadRun(boxed, plain, toBox, { dz, zSigma: 0.05, misread: true }));
    const say = (r) => `${r.shares.map((v, i) => `${v.toFixed(2)}${r.cue[i] ? `(${r.cue[i]})` : ""}`).join(" ")}, ${r.wedges.length} wedge(s)`;
    const near = (r, src) => r.temps.filter((t) => t.source === src && Math.hypot(t.x - BOX.x, t.y - BOX.y) < 0.8).length;
    console.log(`      furniture, the height 0.2 m high: ${quiet.map(say).join("; ")}`);
    console.log(`      a box 1.2 m ahead at the start, 0.2 m low: ${say(seen)}, ${near(seen, "live depth")} live-depth obstacle(s) at it; read as the wall behind, 0.1 m low and right: ${looks.map(say).join("; ")}`);
    for (const r of quiet) assert.ok(r.cue.every((c) => !c) && !r.wedges.length, "nothing for the height error");
    assert.ok(seen.cue.filter(Boolean).length >= 2 && near(seen, "live depth") >= 1, "the box seen with its depth slows the drone, with a live-depth obstacle at it");
    for (const r of looks) assert.ok(r.cue.filter(Boolean).length >= 2 && r.wedges.length >= 1, "the box seen on looks alone slows the drone and places a wedge");
  },

  async "an open doorway the live depth took for closed (a door leaf on Room 2's doorway, as the browser made): the drone looks at it from in front, sees through it, and flies on through"() {
    const s = await stack({ seed: 57 }), texts = [], dr = map.doors.find((d) => d.id === "r2:w4-o1"), f = map.floorAt(1.0, 2.2);
    s.missions.on("status", (q) => texts.push(q.text));
    await placeAt(s, 2.6, 2.0, 0); // in the living room, facing away from the doorway
    s.safety.avoid.addTemp({ id: `leaf-${dr.id}`, kind: "obstacle", source: "door leaf", door: dr.id, x: 0.5, y: 2.25, r: 0.8, polygon: [[0.31, 1.48], [0.68, 1.48], [0.68, 3.02], [0.31, 3.02]], zMin: f - 0.1, zMax: f + 2.3, until: now() + 30000 }, now());
    map.addTemp({ id: "detour-test", kind: "obstacle", source: "test", polygon: [[0.2, 3.3], [0.8, 3.3], [0.8, 4.0], [0.2, 4.0]], zMin: f - 0.1, zMax: f + 2.6, until: Infinity }); // the way round through Room 3 shut
    const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: -0.7, y: 2.0 } }), 120), d = s.sim.drone;
    map.removeTemp("detour-test");
    console.log(`      "${texts.find((t) => /from in front of it/.test(t)) ?? "no closer look"}"; the leaf ${s.safety.avoid.temps.has(`leaf-${dr.id}`) ? "still there" : "gone"}; "${r.summary}" ${fmt(Math.hypot(d.x + 0.7, d.y - 2.0))} m from the goal, ${s.watch.contacts} contacts`); // C2: "The doorway between Room 2 and the Living room looks closed."
    assert.ok(texts.some((t) => /from in front of it/.test(t)) && r.ok && Math.hypot(d.x + 0.7, d.y - 2.0) < 0.3, r.summary);
    assert.equal(s.watch.contacts, 0);
  },

  // Fix 2 (the verifier): return home (the missions' and the safety layer's) only waited and faced a blocker only the live
  // depth sees, from where it was, until the battery sent it to a clear spot in the wrong room (seed 12 in the browser).
  async "return home blocked by an open doorway the live depth took for closed (a door leaf on Room 2's doorway, the way round through Room 3 shut): it looks at it from in front, sees through it, and lands on the pad"() {
    const s = await stack({ seed: 57 }), texts = [], dr = map.doors.find((d) => d.id === "r2:w4-o1"), f = map.floorAt(1.0, 2.2);
    s.missions.on("status", (q) => texts.push(q.text));
    await placeAt(s, 2.6, 2.0, 0); // in the living room, facing away from the doorway home
    s.safety.avoid.addTemp({ id: `leaf-${dr.id}`, kind: "obstacle", source: "door leaf", door: dr.id, x: 0.5, y: 2.25, r: 0.8, polygon: [[0.31, 1.48], [0.68, 1.48], [0.68, 3.02], [0.31, 3.02]], zMin: f - 0.1, zMax: f + 2.3, until: now() + 30000 }, now());
    map.addTemp({ id: "detour-test", kind: "obstacle", source: "test", polygon: [[0.2, 3.3], [0.8, 3.3], [0.8, 4.0], [0.2, 4.0]], zMin: f - 0.1, zMax: f + 2.6, until: Infinity });
    const blocked = !P.plan(map, [2.6, 2.0], [HOME.x, HOME.y], { alt: 1.0, sigma: 0.17 }).ok;
    const r = await s.drive(s.missions.run({ kind: "returnHome" }), 150), d = s.sim.drone;
    map.removeTemp("detour-test");
    console.log(`      the way home ${blocked ? "blocked" : "open"}; "${texts.find((t) => /closer look/.test(t)) ?? "no closer look"}"; the leaf ${s.safety.avoid.temps.has(`leaf-${dr.id}`) ? "still there" : "gone"}; "${r.summary}" ${fmt(Math.hypot(d.x - HOME.x, d.y - HOME.y))} m from the pad, ${s.watch.contacts} contacts`); // C2 fix 1: waits, then "landed in the Living room on a clear spot instead"
    assert.ok(blocked && texts.some((t) => /closer look from in front of it/.test(t)), "a closer look");
    assert.ok(r.ok && !d.airborne && Math.hypot(d.x - HOME.x, d.y - HOME.y) < 0.4 && !/clear spot/.test(r.summary), r.summary);
    assert.equal(s.watch.contacts, 0);
  },

  // Fix 2 (the verifier's browser run, seed 12): a patrol's scan by Room 2's doorway climbed from 1.0 to 2.1 m with the
  // position right (the controller's own height, read off the flow's climb rate, drifted down; nothing held the drone
  // down) and left its spot by 1.2 m, into the doorway's jamb.
  async "a scan by Room 2's doorway with the flow misreading the climb rate (the controller's own height drifts) and a drift toward the doorway: the drone stays within 0.3 m of its spot and its height (0.35 m sinking), and touches nothing"() {
    const out = [];
    for (const [bias, drift] of [[0.2, 0], [0.2, 0.1], [-0.2, 0.1]]) {
      const s = await stack({ seed: 61, pictures: true }), spot = [1.04, 2.25];
      await placeAt(s, spot[0], spot[1], Math.PI);
      const d = s.sim.drone, z0 = d.z, per = s.perception, update = per.update.bind(per), vfov = s.values.hfov * DEG * 0.75;
      per.update = () => (update(), per.latest.flow && (per.latest.flow.dy -= bias / (vfov * 2.2))); // the flow says it sinks (bias > 0): the controller climbs to hold its own height
      d.trimDrift = { x: -drift, y: drift / 2 };
      let far = 0, dz = 0;
      s.watch.onStep = () => d.airborne && ((far = Math.max(far, Math.hypot(d.x - spot[0], d.y - spot[1]))), (dz = Math.max(dz, Math.abs(d.z - z0))));
      const r = await s.drive(s.ctl.run(new ScanAt({ x: spot[0], y: spot[1], headings: [Math.PI, -Math.PI / 2, 0, Math.PI / 2] }, { localizer: s.localizer, map, labels: [], look: 0.8, label: "Checking the Living room" })), 40);
      out.push({ bias, drift, far, dz, contacts: s.watch.contacts, ok: r.ok });
      d.trimDrift = { x: 0, y: 0 };
      per.update = update;
      await s.drive(s.missions.run({ kind: "returnHome" }), 60);
    }
    console.log(`      ${out.map((q) => `climb rate misread ${q.bias > 0 ? "down" : "up"} ${Math.abs(q.bias)} m/s${q.drift ? `, drifting ${q.drift} m/s toward the doorway` : ""}: ${fmt(q.far)} m off the spot at most, height ${fmt(q.dz)} m off, ${q.contacts} contacts`).join("; ")}`); // C2 fix 1: climbs on (0.2 m/s: 1.4 m up, into the head)
    for (const q of out) assert.ok(q.ok && q.far < 0.3 && q.dz < (q.bias > 0 ? 0.3 : 0.35) && q.contacts === 0, JSON.stringify(q)); // a sink is caught from 8 cm under, a climb from 12 cm over
  },

  // Fix 2 (browser seeds 11 and 12, the Living room's low furniture): a leg over it on the 1.4 m band sank to 1.15 m; the
  // climb back was capped ("something above": the furniture beside it at that height) and the drone touched it.
  "climbing and sinking: only what comes closer on the way caps them: a drone sunk to beside a table's top climbs back off it; sinking onto the table and climbing past the top of what the map knows still stop"() {
    const av = new Avoid({ map, localizer: { ctl: { videoDelay: 0, est: { ttc: Infinity } }, pose: () => null, velocity: () => [0, 0] }, settings: { get: (k) => ({ mode: "sim", avoid: true })[k] } }), r = map.o.droneRadius;
    let spot = null; // over furniture: the 1.0 m band blocked there, the 1.4 m band clear
    for (let y = -1; y < 6 && !spot; y += 0.05)
      for (let x = -2; x < 6 && !spot; x += 0.05) {
        const f = map.floorAt(x, y), room = map.roomAt(x, y);
        if (f == null || !room || map.doors.some((d) => Math.hypot(x - (d.a[0] + d.b[0]) / 2, y - (d.a[1] + d.b[1]) / 2) < 1)) continue;
        if (staticClearance(map, x, y, f + 1.0) < r - 0.04 && staticClearance(map, x, y, f + 1.4) > 0.4 && staticClearance(map, x, y, f + 0.3) < r) spot = { x, y, f };
      }
    assert.ok(spot, "furniture in the fixture");
    const at = (h, zSigma = 0.02) => ({ x: spot.x, y: spot.y, z: spot.f + h, yaw: 0, sigma: 0.03, zSigma, status: "ok" });
    const climb = av.vertical({ vz: 0.25 }, at(1.15), 0.1), sink = av.vertical({ vz: -0.25 }, at(1.45), 0.1);
    console.log(`      furniture at (${fmt(spot.x)}, ${fmt(spot.y)}): sunk to 1.15 m (the 1.0 m band ${fmt(staticClearance(map, spot.x, spot.y, spot.f + 1.15))} m clear), climbing at 0.25: ${fmt(climb.vz)} m/s; at 1.45 m sinking at 0.25: ${fmt(sink.vz)} m/s ("${av.last.vertical ?? ""}")`);
    assert.ok(climb.vz === 0.25, "climbs back off it");
    assert.ok(sink.vz > -0.25 && sink.vz <= 0, "sinking onto it is still capped");
    // a ceiling: climbing toward it still stops
    const c = map.ceilZ[map.idx(spot.x, spot.y)], high = av.vertical({ vz: 0.3 }, { ...at(0), z: Math.min(c - map.o.ceilingMargin - 0.05, spot.f + map.o.bands.at(-1) + map.o.bandHalf - 0.05) }, 0.1);
    assert.ok(high.vz < 0.3, `under the top of what the map knows: ${high.vz}`);
  },

  async "braking measured over three runs: the most careful fit, no harder than the simulator's 0.7 m/s² on a first measurement; the model then stops no shorter than the drone really does at 0.3 and 0.45 m/s; measured in the simulator it stays the simulator's (the real drone keeps 0.3 m/s until it measures its own, then 0.4 m/s)"() {
    const s = await stack({ seed: 48, videoDelay: 150, depth: false, changes: false }), delay = 0.15;
    await s.drive(s.missions.run({ kind: "goTo", target: { x: 2.4, y: 2.4 } }), 60);
    const r = await s.drive(s.missions.run({ kind: "calibrate" }), 150), b = s.settings.get("simBrake");
    console.log(`      "${r.summary}" -> ${JSON.stringify(b)}`);
    assert.equal(s.settings.get("mode"), "sim");
    assert.equal(s.settings.get("brake"), null, "the simulator's braking isn't the real drone's");
    assert.match(r.summary, /in the simulator \(the real drone measures its own\)/);
    assert.ok(r.ok && b.decel <= AVOID.decel && b.n === 1 && b.measured >= b.decel, r.summary);
    const out = [];
    for (const V of [0.3, 0.45]) {
      await s.drive(s.missions.run({ kind: "goTo", target: { x: 2.4, y: 2.4 } }), 60);
      const Q = s.localizer.pose(), best = Array.from({ length: 32 }, (_, k) => (k * Math.PI) / 16).map((yaw) => ({ yaw, d: freeRun(map, Q, [Math.cos(yaw), Math.sin(yaw)], map.o.droneRadius + 0.3) })).reduce((a, c) => (c.d > a.d ? c : a));
      await s.drive(s.ctl.run(new ScanAt({ x: Q.x, y: Q.y, headings: [best.yaw] }, { localizer: s.localizer, map, labels: [], look: 0.3 })), 15);
      const bt = new BrakeTest({ localizer: s.localizer, yaw: best.yaw, speed: V, run: Math.min(1.5, best.d - 1.0) }), d = s.sim.drone, tr = [];
      s.watch.onStep = () => tr.push({ x: d.x, y: d.y, vx: d.vx, vy: d.vy, ph: bt.phase, t: now() });
      await s.drive(s.ctl.run(bt), 25);
      s.watch.onStep = null;
      const i = tr.findIndex((q) => q.ph === "stop"), u = [Math.cos(best.yaw), Math.sin(best.yaw)], v = tr[i].vx * u[0] + tr[i].vy * u[1];
      const D = Math.max(...tr.slice(i).filter((q) => q.t - tr[i].t < 2000).map((q) => (q.x - tr[i].x) * u[0] + (q.y - tr[i].y) * u[1])), model = stopDistance(v, delay + b.react, b.decel);
      out.push({ V, v, D, model });
    }
    console.log(`      ${out.map((o) => `${o.V} m/s asked (${fmt(o.v)} at the stop command): stopped within ${fmt(o.D)} m, the model says ${fmt(o.model)} m`).join("; ")}`);
    assert.ok(out.every((o) => o.model >= o.D - 0.02), "the model never stops shorter than the drone"); // C2's single run: 0.23 m predicted, 0.45 m travelled
    s.settings.set("mode", "real");
    assert.ok(!s.safety.avoid.brake().measured && s.safety.avoid.realTop().v === AVOID.realVmax, "the real drone: not measured by the simulator's runs");
    s.settings.set("brake", b); // as if the real drone had measured the same
    assert.equal(s.safety.avoid.realTop().v, AVOID.realMeasured, "the real drone's cap once measured");
    s.settings.set("brake", null);
    s.settings.set("mode", "sim");
  },
};

function mulberry(seed) {
  return () => {
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  Object.assign(tests, (await import("./avoid-cases.mjs")).avoidCases({ stack, depthCamera, house, map })); // the real drone's (wave C reviews)
  const only = process.env.ONLY;
  await runTests(only ? Object.fromEntries(Object.entries(tests).filter(([k]) => k.includes(only))) : tests);
}
