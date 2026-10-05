// Obstacle avoidance on the house map (docs/HOME-DRONE.md, "Wave C contracts"). The safety layer runs every setpoint
// through limit(): the speed along the commanded direction is capped so the drone can stop (video delay + reaction,
// then braking) before it comes within its radius + 2σ (+ a margin) of anything the map holds along that ray (walls,
// furniture, keep-outs, temporary obstacles and, once the 3D map is attached (map.setVoxels: map.clearance is then the
// 3D one), unknown space), at any height the drone may be at (its height ± 2σz); climbing and sinking are capped the same
// way on the column above and below it (temporary obstacles only when it would enter them from above or below: inside a
// person's zone, up and down are as close; a landing sets down on the floor, a forced one on whatever is below). Live depth (vision/depth.js, metric, from the
// localized pose) adds what the map lacks: the nearest live point in the drone's corridor caps the speed too, and
// clusters of live points in space the map calls free (much nearer than the twin expected, off the floor, and in a part
// of the picture nav/changes.js differences() finds nearer than the scan after taking off the bias monocular depth bends
// whole walls by, and looking different) become temporary obstacles the planner routes around: the outline of what was
// seen of each over the last AVOID.nearKeep ms (never more), as a polygon. Points on a doorway's plane (or behind it, as
// far as a new thing's depth reads too far) are a door leaf: a temporary obstacle on the doorway itself, never stretched
// toward the camera (that would close the doorways beside it too), until nav/changes.js reports the closed door; the
// corridor cap stops short of them as of every new point: margin m short of where it may be at the nearest, no creep.
// Monocular depth reads a new thing's range badly (often too far), so those points count as nearer by AVOID.depthErr.
// It can be far worse: a plain box 0.8 m ahead read as 2-3 m (tools/depth-check.html), so what the camera sees ahead
// counts too: when much of the way ahead (the corridor window around the flight direction in the picture) is nearer than
// the scan (depth, bias taken off) or doesn't look like it, something the map lacks may be there at a range nobody knows:
// a creep, and with depth support (or, on looks alone, once it looms faster than the scan behind it would as the drone
// closes in: a lamp or a shadow on a wall doesn't) no closer once it fills the way, with a wedge over its bearings (up to
// the scan's surface behind it) on the map for the planner to go around; what the pose's error explains (the expected
// view rendered a height σz off: nav/changes.js heldUp(), and for looks looksHeld() here) doesn't count. The flow's time
// to contact (zoom) slows a forward flight too. Temporary obstacles (people, unmapped things, suspected changes) go on
// the map through map.addTemp/removeTemp and expire here.
// Eyes: on the real drone (settings "mode" "real", or "Rehearse the real flight": simRehearse) the drone flies at most AVOID.blind while it can't see what the scan
// lacks: live depth off (settings "avoid" false, no model, no twin, frames stopped or older than depthAge), a pose too
// unsure to place what it sees, live depth that doesn't match the scan, or no 3D map yet (unknown space isn't known);
// eyes() says which, and blindMs counts the time flown so. Braking: settings "brake" { decel, react } once measured
// (missions.js "calibrate"); until then the real drone brakes as AVOID.realDecel and flies at most AVOID.realVmax, after
// it at most AVOID.realMeasured (realTop()). Braking measured in the simulator is the simulator's ("simBrake": brakeKey()),
// never the real drone's.
// Vision fixes too sparse (over AVOID.fixAge ms old or under AVOID.fixRate a second, with vision the position source) are
// blind too: what the camera sees can't be placed. cruise() is the one speed every time budget uses (missions' legs, the
// way home, safety.js's battery trigger): AVOID.cruise, or a share of whatever cap holds the drone back, or the speed its
// legs have been flown at this flight (noteLeg(): planned length over the time taken). pass(id): a mission may pass a
// suspected change Claude called no change, slowly and only while the drone can see (eyes()): the cap then reads the map
// without it. lift(id, shape): a suspected change the missions took off the map unseen (missions.js liftStale): while the
// drone can't see, it creeps near where it was.
import { Emitter, clamp, wrapAngle } from "../util.js";
import { droneCamera } from "../twin/pose.js";
import { intrinsics, unproject, project } from "../twin/lens.js";
import { nearestFree, plan } from "../house/planner.js";
import { differences, heldUp, CHANGES } from "./changes.js";

// The settings key braking is measured into and read from: the simulator's own (a rehearsal included) or the real drone's.
export const brakeKey = (settings) => (settings?.get?.("mode") === "sim" ? "simBrake" : "brake");

export const AVOID = {
  decel: 0.7, // m/s²: how hard the controller brakes (drag plus its gentle tilt), measured in the simulator
  react: 0.25, // s on top of the video delay: control ticks, the radio script's 50 ms and the stick slew
  realDecel: 0.4, realVmax: 0.3, // the real drone until its braking is measured (settings "brake")
  realMeasured: 0.4, // m/s at most on the real drone once it is (not the missions' vmax: one calibration is a few runs)
  vDecel: 0.25, vReact: 0.4, // m/s², s: climbing and sinking stop slower (height is held off the late video's climb rate)
  touch: 0.15, // m/s: a forced landing (the battery is empty) sets down on what is below this slowly
  blind: 0.22, // m/s at most on the real drone while it can't see what the scan lacks (eyes())
  minInliers: 0.3, // share of a depth frame's pixels agreeing with the scan, under which it can't be trusted
  margin: 0.15, // m kept beyond radius + 2σ at speed; inside it only a creep (creep m/s) is left
  creep: 0.12,
  range: 4, // m a ray is followed
  depthAge: 1200, // ms: older live depth is ignored
  points: 70, // live-depth points across a frame
  minConf: 0.5,
  nearRange: 2.5, // m: live points this close can be unmapped obstacles...
  nearClear: 0.2, // ...when the map has free space at least this far around them,
  nearAbove: 0.15, // this far off the floor and the ceiling,
  nearRatio: 0.85, // and (with the twin's expected depth) nearer than this share of it
  nearMin: 6, // points in a 0.2 m cell, seen on nearFrames frames
  nearFrames: 2,
  nearKeep: 20000, // ms on the map after the last sighting (and how long a sighting shapes its outline)
  nearPad: 0.15, // m around the outline (what it hides behind its face)
  depthErr: 0.5, // share a new thing's monocular depth can read too far (60-100% at worst in tools/depth-check.html, on the
  // twin's renders with the O4's blur, noise and compression added; frames may say: f.depthErr): such points count as
  // that much nearer for the cap, and their obstacle reaches back toward the camera by as much
  ttcStop: 0.6, ttcSlow: 1.5, // s: flow time to contact
  window: [0.2, 0.25], // the corridor window: half its width and height, shares of the picture, around the flight direction
  aheadSlow: 0.25, aheadStop: 0.55, // shares of it unlike the scan (2 frames in a row): creep; stop going closer
  aheadDeep: 0.1, // share of it nearer than the scan (depth) that backs a stop and a wedge
  aheadLoom: [0.15, 1.1], // on looks alone: after this many m toward it, it must have grown this much faster than the scan
  // behind it would have (a lamp or a shadow on the scan's own surface grows as that surface does); shares over 3 frames
  aheadSigma: 0.15, aheadOff: 0.5, // only from a pose this good and a frame no more unlike the scan than this as a whole
  aheadCheck: 0.125, // a window at least this share unlike the scan (half the slow share: below it the cue can neither
  // come on nor stay on) has its cells tested against the pose's error; where only their looks differ, a guess at the
  // error must explain this share of them (or have been taken so in the last second): looksHeld()
  aheadPosed: 0.5,
  aheadAt: [0.35, 1.5], // m: the temporary obstacle for what fills the way: a wedge over its bearings from this far ahead to
  // the scan's surface behind it (it is somewhere in front of that), at most this far
  fixAge: 1000, fixRate: 2, // vision fixes older than this (ms) or rarer (per s): the pose can't place what the camera sees
  cruise: 0.3, // m/s a planned path averages in the simulator (vmax 0.55, slower at corners, doors and the goal)
  cruiseShare: 0.85, // of a speed cap, what a path flown under it averages
  cruiseLegs: 6, // m: the legs flown (planned length over the time they took, waits included) are averaged over about this much
  passNear: 1.0, // m: within this of a suspected change it may pass, the drone creeps
  leafHeadOn: 50, leafSpread: 20, // degrees: a door leaf from a view more oblique than leafHeadOn off the doorway's normal needs
  // another view of it closed whose bearing from the doorway differs by leafSpread (an open doorway seen 55-60° off its
  // normal read closed in the browser: the opening is a sliver of the picture there, and the depth's soft edges fill it)
  // A doorway read closed (doorReading()): cells seen at most leafGraze degrees off its normal, leafJamb m clear of its jambs
  // (and of the wall's thickness, leafThick m when the house doesn't say, at that angle), the live depth nearer than leafFar
  // of the way to the far side, over leafWidth m of the opening (or half of it), on leafFrames frames in a row from viewpoints
  // leafApart m or leafTurn degrees apart, and not seen through in the last leafOpen ms (a misread flickering in; a door shut
  // in front of the drone still counts a second later) (Room 2's open doorway read closed in 3 of 6 sealed runs: grazing views
  // from beside it, Room 3's closed doorway just behind it, one viewpoint)
  // A leaf so made stays leafKeep ms after the last closed reading (a closed door doesn't open by itself: forgotten after 20 s,
  // a patrol flew back into one it had seen from afar) unless the doorway is seen through on nearFrames frames.
  // A door's live depth reads at most leafErr too far (it is flush with the wall the depth is aligned on, not a box read as
  // the wall behind it), and a far side seen through an opening can read 30-40% too near (Room 2's open doorway, 1.3 m off,
  // its far side 3 m off read as 2.1 m: a leaf with leafFar 0.5 and the box's depthErr)
  leafGraze: 65, leafJamb: 0.08, leafThick: 0.1, leafFar: 1 / 3, leafErr: 0.3, leafWidth: 0.5, leafFrames: 3, leafApart: 0.25, leafTurn: 10, leafOpen: 1500, leafKeep: 60000,
};

// Speed (m/s) from which a drone stops within d metres with reaction time T and braking a.
export const stoppable = (d, T, a = AVOID.decel) => (d > 0 ? a * (Math.sqrt(T * T + (2 * d) / a) - T) : 0);
// How far a drone at v m/s travels before it stands still.
export const stopDistance = (v, T, a = AVOID.decel) => v * T + (v * v) / (2 * a);
// The speed (m/s) to budget time at: the safety layer's avoid.cruise(), AVOID.cruise without one.
export const cruiseSpeed = (ctl, now) => ctl?.safety?.avoid?.cruise?.(now).v ?? AVOID.cruise;

// Free run (m) from p = { x, y, z } along the horizontal unit direction [dx, dy] before something on the map comes
// within `need` (sphere tracing on map.clearance). From inside that distance (or inside an obstacle: a person's zone the
// drone is backing out of) only getting closer than the best clearance so far along the ray counts.
export function freeRun(map, p, [dx, dy], need, max = AVOID.range) {
  const step = map.o.cell / 2;
  let best = -Infinity;
  for (let d = 0; d < max; ) {
    const c = map.clearance(p.x + dx * d, p.y + dy * d, p.z);
    if (c < need && c < best - 0.01) return Math.max(0, d - step);
    best = Math.max(best, c);
    d += Math.max(step, c - need);
  }
  return max;
}

// The map's clearance at (x, y, z) without temporary obstacles: HomeMap.base (static, per band) under the same height
// rules (0 more than 5 cm below the floor, near the ceiling or a door head, above the top band), and the voxels.
export function staticClearance(map, x, y, z) {
  const k = map.idx(x, y);
  if (!map.base || k < 0) return map.clearance(x, y, z);
  const o = map.o, h = z - map.floorZ[k];
  if (h < -0.05 || h > o.bands.at(-1) + o.bandHalf || z > map.ceilZ[k] - o.ceilingMargin || z > map.headZ[k] - o.headMargin) return 0;
  const c = map.base[map.bandOf(h)][k];
  return map.vox ? Math.min(c, map.vox.clearance3(x, y, z)) : c;
}

// The map without its temporary obstacles (people, things the live depth found, suspected changes), for the planner: what
// a way would be if they weren't there. Its own copies of the doors (their clearances), cached per map build and voxels.
const views = new WeakMap();
export function staticView(map) {
  map.sync?.();
  if (!map.base) return map;
  const v = views.get(map);
  if (v && v.base === map.base && v.vox === map.vox && v.ver === map.vox?.version) return v.view;
  const view = Object.create(map, { temps: { value: new Map() }, doors: { value: map.doors.map((d) => ({ ...d })), writable: true }, changed: { value: true, writable: true } });
  view.overlay();
  views.set(map, { base: map.base, vox: map.vox, ver: map.vox?.version, view });
  return view;
}

// The map without the temporary obstacles `ids` (a suspected change return home may pass), for planning and the cap; the
// others stay. Cached until the map's overlay changes.
const without = new WeakMap();
export function withoutTemps(map, ids) {
  map.sync?.();
  if (!map.base || !ids.length) return map;
  const key = [...ids].sort().join("|"), v = without.get(map);
  if (v && v.key === key && v.flat === map.flat && v.vox === map.vox && v.ver === map.vox?.version) return v.view;
  const temps = new Map([...map.temps].filter(([id]) => !ids.includes(id)));
  const view = Object.create(map, { temps: { value: temps }, doors: { value: map.doors.map((d) => ({ ...d })), writable: true }, changed: { value: true, writable: true } });
  view.overlay();
  without.set(map, { key, flat: map.flat, vox: map.vox, ver: map.vox?.version, view });
  return view;
}

// What temporary obstacles stand between from and to: the plan without them; null when even that fails (the place itself
// can't be reached: off the map, unknown space, no doorway wide enough), else { path (that plan), on: the temporary
// obstacles within its corridor (need + 0.1 m of the path at its heights), first along the way first }.
export function blockers(map, from, to, { alt = 1.0, sigma = map.o.sigma, climb = true } = {}) {
  const p = plan(staticView(map), from, to, { alt, sigma, climb });
  if (!p.ok) return null;
  const need = map.lethal(sigma) + 0.1, pts = p.path, s = [0];
  for (let i = 1; i < pts.length; i++) s.push(s[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
  map.sync?.();
  const on = [];
  for (const t of map.temps?.values() ?? []) {
    let at = Infinity;
    for (let i = 0; i < pts.length && at === Infinity; i++) {
      const a = pts[Math.max(0, i - 1)], b = pts[i], z = b[2];
      if (z != null && (z < (t.zMin ?? -Infinity) - 0.2 || z > (t.zMax ?? Infinity) + 0.2)) continue;
      for (let k = 0; k <= 4; k++) {
        const x = a[0] + ((b[0] - a[0]) * k) / 4, y = a[1] + ((b[1] - a[1]) * k) / 4;
        if ((t.polygon ? polyDist(x, y, t.polygon) : Math.hypot(x - t.x, y - t.y) - t.r) < need) { at = s[Math.max(0, i - 1)] + (Math.hypot(b[0] - a[0], b[1] - a[1]) * k) / 4; break; }
      }
    }
    if (at < Infinity) on.push({ ...t, along: at });
  }
  return { path: p, on: on.sort((a, b) => a.along - b.along) };
}

// The nearest spot to land on: open floor (clearance >= need + 0.3 at the lowest band, nothing at pet height within
// 0.2 m), on the map's free space (so seen and free with voxels), away from temporary obstacles, that a plan reaches.
// -> { x, y, path } | null
export function safeSpot(map, from, { sigma = 0.15, alt = map.o.bands[0], maxR = 3 } = {}) {
  const low = map.occ?.[map.levels.length - 1], cell = map.o.cell, wide = sigma + 0.3;
  const clearLow = (x, y) => {
    for (let dy = -0.2; dy <= 0.2; dy += cell) for (let dx = -0.2; dx <= 0.2; dx += cell) if (low?.[map.idx(x + dx, y + dy)]) return false;
    return true;
  };
  for (let r = 0.5; r <= maxR; r += 0.5) {
    const c = nearestFree(map, [from.x, from.y], { alt, sigma: wide, maxR: r });
    if (!c || !clearLow(...c)) continue;
    const p = plan(map, [from.x, from.y], c, { alt: 1.0, sigma, climb: false });
    if (p.ok) return { x: c[0], y: c[1], path: p };
  }
  return null;
}

export class Avoid extends Emitter {
  constructor({ map, localizer, settings = null }) {
    super();
    Object.assign(this, { map, loc: localizer, ctl: localizer.ctl, settings });
    this.temps = new Map(); // id -> { ...temp, pushed: { x, y, r, polygon, until } (what the map holds) }
    this.cloud = null; // the latest live depth as H points: { t, n, x, y, z, near, cx, cy, cz, sigma, inliers }
    this.seen = new Map(); // near-field cells: key -> { n frames, last t, ... }
    this.last = { run: Infinity, live: Infinity, cap: Infinity, ttc: Infinity, why: "" };
    this.nearId = 0;
    this.depthWhy = ""; // why live depth isn't running, from vision/depth.js (setDepthStatus)
    this.blindMs = 0; // time flown on the real drone without eyes (eyes())
    this.flown = null; // { v (m/s), m (m) }: the speed legs have been flown at this flight (noteLeg(), cruise())
    this.passing = new Map(); // id -> until (performance.now ms): temporary obstacles return home may pass (pass())
    this.lifted = new Map(); // id -> shape: suspected changes the missions took off the map unseen (lift()): a creep near them while blind
  }

  setMap(map) {
    for (const id of [...this.temps.keys()]) this.removeTemp(id);
    Object.assign(this, { map, leafMaybe: null, leafViews: null });
    this.lifted.clear();
  }

  get real() { // the real drone, or the simulator rehearsing it (its caps too)
    return this.settings?.get?.("mode") === "real" || this.settings?.get?.("simRehearse") === true;
  }

  // vision/depth.js LiveDepth's state: "" while it delivers frames, else why it doesn't (for eyes()).
  setDepthStatus(why = "") {
    this.depthWhy = why;
  }

  // Can the drone see what the scan lacks? -> { ok, why }
  eyes(now = performance.now()) {
    const c = this.cloud, q = this.loc.fixQuality?.(now);
    if (this.settings?.get?.("avoid") === false) return { ok: false, why: "live depth is turned off" };
    if (!this.map?.vox) return { ok: false, why: "no 3D map yet, so unknown space isn't known" };
    if (!c || now - c.t > AVOID.depthAge) return { ok: false, why: this.depthWhy || (c ? "live depth frames stopped" : "no live depth yet") };
    if (!(c.sigma <= AVOID.aheadSigma)) return { ok: false, why: "my position is too unsure to place what the camera sees" };
    if (c.inliers < AVOID.minInliers) return { ok: false, why: "live depth doesn't match the 3D scan" };
    if (q?.vision && (q.visionAge > AVOID.fixAge || q.rate < AVOID.fixRate)) return { ok: false, why: "the camera's position fixes are too sparse" };
    return { ok: true, why: "" };
  }

  // The caps that hold whatever the direction (the real drone unable to see, its braking not measured): { v (m/s), why }.
  // The geofence's push keeps to them too.
  speedCap(now = performance.now()) {
    const eyes = this.real ? this.eyes(now) : { ok: true };
    let v = Infinity, why = "";
    if (!eyes.ok) [v, why] = [AVOID.blind, `flying slowly: ${eyes.why}`];
    const top = this.realTop();
    if (top.v < v) [v, why] = [top.v, top.why];
    return { v, why };
  }
  // The real drone's top speed by its braking: { v, why } (Infinity in the simulator).
  realTop() {
    if (!this.real) return { v: Infinity, why: "" };
    return this.brake().measured ? { v: AVOID.realMeasured, why: `at most ${AVOID.realMeasured} m/s on the real drone` } : { v: AVOID.realVmax, why: `at most ${AVOID.realVmax} m/s until my braking is measured` };
  }

  // The speed (m/s) to budget time at, and why it is less than AVOID.cruise: { v, why }.
  cruise(now = performance.now()) {
    let v = AVOID.cruise, why = "";
    const cap = (c, w) => c < v && ([v, why] = [c, w]);
    const top = this.realTop();
    cap(top.v * AVOID.cruiseShare, top.why);
    if (this.real) {
      const e = this.eyes(now);
      if (!e.ok) cap(AVOID.blind * AVOID.cruiseShare, `flying slowly: ${e.why}`);
    }
    if (this.flown?.m >= 1.5) cap(Math.max(0.08, this.flown.v), "as fast as my legs have been flown");
    return { v, why };
  }

  // A leg flown: `m` m of planned path in `s` s (waits and detours included), for cruise(): a running average by length.
  noteLeg(m, s) {
    if (!(m > 0.5 && s > 0)) return;
    const f = (this.flown ??= { v: 0, m: 0 });
    f.m += m;
    f.v += (m / s - f.v) * Math.max(m / AVOID.cruiseLegs, m / f.m);
  }

  // Return home may pass the temporary obstacle `id` (a suspected change Claude called no change) until `until`.
  pass(id, until = Infinity) {
    this.passing.set(id, until);
  }
  unpass(id) {
    this.passing.delete(id);
  }
  // A suspected change taken off the map unseen (missions.js liftStale), its shape as the map had it: while the drone can't
  // see what the scan lacks, it creeps within AVOID.passNear of it.
  lift(id, shape) {
    this.lifted.set(id, shape);
  }
  unlift(id) {
    this.lifted.delete(id);
  }
  // The ids being passed now (none while the drone can't see: then everything counts).
  passable(now = performance.now()) {
    for (const [id, until] of this.passing) if (until <= now || !this.map?.temps?.has(id)) this.passing.delete(id);
    return this.passing.size && this.eyes(now).ok ? [...this.passing.keys()] : [];
  }

  // How the drone brakes: { decel (m/s²), react (s on top of the video delay), measured }.
  brake() {
    const b = this.settings?.get?.(brakeKey(this.settings));
    if (b?.decel > 0) return { decel: clamp(b.decel, 0.2, 2), react: clamp(b.react ?? AVOID.react, 0.1, 1), measured: true };
    return { decel: this.real ? AVOID.realDecel : AVOID.decel, react: AVOID.react, measured: !this.real };
  }

  // Is the column above (x, y) known free from the floor to `up` m above it (a take-off)? The map without temporary
  // obstacles, then those over the column itself, named (a person's zone is left to the take-off's wait for them). -> "" | why not
  columnClear(x, y, up = 1.1) {
    const map = this.map, floor = map.floorAt(x, y), r = map.o.droneRadius;
    if (floor == null) return "I'm not over the floor of a room on the map";
    for (let h = 0.15; h <= up; h += 0.05) if (staticClearance(map, x, y, floor + h) < r) return map.vox ? "something is above me, or the space above me isn't mapped" : "something is above me";
    const over = [...(map.temps?.values?.() ?? [])].find((t) => t.kind !== "person" && (t.polygon ? polyDist(x, y, t.polygon) : Math.hypot(x - t.x, y - t.y) - t.r) < r
      && (t.zMax ?? Infinity) > floor + 0.15 && (t.zMin ?? -Infinity) < floor + up);
    if (over) return over.source === "door leaf" ? "a doorway here looks closed" : over.kind === "change" ? `something the 3D scan doesn't have is ${(over.zMin ?? -Infinity) > floor + 0.15 ? "above me" : "right beside me"} (a change in History)` : "something the live depth found is above me";
    return "";
  }

  // ---- temporary obstacles ----

  // { id, kind, x, y, r | polygon, zMin, zMax, until (performance.now ms) }. Refreshing one with the same id moves it on
  // the map only when it moved 0.25 m, grew (a polygon: a corner 0.2 m outside what the map holds), shrank by 0.25 m, or
  // the map's copy is about to expire (each change rebuilds clearances).
  addTemp(t, now = performance.now()) {
    const old = this.temps.get(t.id), p = old?.pushed;
    const out = (a, b, d) => a.some((q) => polyDist(q[0], q[1], b) > d);
    const moved = !p || !t.polygon !== !p.polygon || (t.polygon ? out(t.polygon, p.polygon, 0.2) || out(p.polygon, t.polygon, 0.25) : Math.hypot(t.x - p.x, t.y - p.y) > 0.25 || (t.r ?? 0) > p.r + 0.1);
    const entry = { ...t, pushed: p };
    if (moved || p.until - now < 2000 || t.until < p.until) {
      if (p) this.map.removeTemp?.(t.id);
      entry.pushed = { x: t.x, y: t.y, r: t.r ?? 0, polygon: t.polygon, until: t.until === Infinity ? Infinity : Math.max(t.until, now + 5000) };
      const { pushed, hist, sightings, ...temp } = entry;
      this.map.addTemp?.({ ...temp, until: entry.pushed.until });
      if (!p) this.emit("temp", { ...temp, added: true });
    }
    this.temps.set(t.id, entry);
    return entry;
  }

  removeTemp(id) {
    const t = this.temps.get(id);
    if (!t) return;
    this.temps.delete(id);
    this.map.removeTemp?.(id);
    this.emit("temp", { ...t, removed: true });
  }

  expire(now = performance.now()) {
    for (const [id, t] of this.temps) if (t.until <= now) this.removeTemp(id);
  }

  // Temporary obstacles that cover (x, y) (within pad m), for messages: [{ id, kind, ... }].
  tempsAt(x, y, pad = 0) {
    return [...this.temps.values()].filter((t) => (t.polygon ? polyDist(x, y, t.polygon) <= pad : Math.hypot(x - t.x, y - t.y) <= t.r + pad));
  }

  // ---- live depth ----

  // A metric depth frame (vision/depth.js LiveDepth "depth" event, or the simulator's): { t (capture, ms), pose (H, with
  // pitch and roll if known), width, height, lens (pinhole: { model, hfovDeg, uptiltDeg }), depth (m along each ray,
  // rows top-down, NaN unknown), conf?, expected? (the twin's empty-house depth, same layout), boxes? (people and pets,
  // normalised in this view: their own no-fly zones cover them, so they make no unmapped obstacle), stats? ({ inliers }) }.
  ingest(f, now = performance.now()) {
    // where the picture is nearer than the scan (people masked), from a pose good enough to tell
    const g = f.expected ? (f.grid ??= differences(f)) : null, nearer = g && (f.pose.sigma ?? 0) <= AVOID.aheadSigma ? g.near : null;
    const { width: w, height: h } = f, stride = Math.max(1, Math.round(w / AVOID.points)), K = intrinsics(f.lens, w, h), cam = droneCamera(f.pose, f.lens.uptiltDeg ?? 0);
    const n = Math.ceil(w / stride) * Math.ceil(h / stride), x = new Float32Array(n), y = new Float32Array(n), z = new Float32Array(n), near = new Uint8Array(n), door = new Int16Array(n);
    const R = cam.R, map = this.map, err = f.depthErr ?? AVOID.depthErr, shrink = 1 / (1 + err), cx = new Float32Array(n), cy = new Float32Array(n), cz = new Float32Array(n);
    let m = 0;
    for (let v = stride >> 1; v < h; v += stride)
      for (let u = stride >> 1; u < w; u += stride) {
        const i = v * w + u, d = f.depth[i];
        if (!(d > 0.05) || (f.conf && f.conf[i] < AVOID.minConf) || f.mask?.[i]) continue;
        const r = unproject(K, u + 0.5, v + 0.5), ray = [0, 1, 2].map((a) => R[a][0] * r[0] + R[a][1] * r[1] + R[a][2] * r[2]);
        const X = cam.p[0] + d * ray[0], Y = cam.p[1] + d * ray[1], Z = cam.p[2] + d * ray[2];
        [x[m], y[m], z[m]] = [X, Y, Z];
        near[m] = 0;
        if (d < AVOID.nearRange && (!f.expected || (nearer?.[((v / g.C) | 0) * g.gw + ((u / g.C) | 0)] && !(f.expected[i] * AVOID.nearRatio - 0.1 < d))) && !f.boxes?.some((b) => inBox(b, (u + 0.5) / w, (v + 0.5) / h))) {
          const fl = map.floorAt(X, Y), ce = map.ceilingAt(X, Y);
          const on = fl != null && Z > fl + AVOID.nearAbove && Z < ce - AVOID.nearAbove && staticClearance(map, X, Y, Z) > AVOID.nearClear && onDoorway(map, cam.p, ray, d, err);
          if (on) [near[m], door[m]] = [2, on.k];
          else if (on === null) near[m] = 1;
        }
        // new to the scan (an obstacle, or a door leaf: 2): where it is if its depth read as far too far as a new thing's can
        [cx[m], cy[m], cz[m]] = near[m] ? [0, 1, 2].map((a) => cam.p[a] + ([X, Y, Z][a] - cam.p[a]) * shrink) : [X, Y, Z];
        m++;
      }
    this.cloud = { t: f.t, n: m, x, y, z, near, door, cx, cy, cz, sigma: f.pose.sigma ?? 0, inliers: f.stats?.inliers ?? 1 };
    this.nearField(f, now);
    this.leaves(f, K, cam, err, now);
    this.seenClear(f, K, cam, now);
    this.wayAhead(f, K, now);
  }

  // The share of the corridor window that is nearer than the scan (depth, bias taken off) or doesn't look like it (people
  // and pets masked): this.ahead { t, share, deep, stop, slow } once two frames in a row agree, kept while at least half
  // the slow share stays (closing in slowly mustn't let it go); a frame that can't tell (a poor pose, or most of the picture
  // unlike the scan) keeps the last answer until it is stale. slow: a creep. stop (no closer) and a temporary obstacle over
  // where it may be (a wedge across the bearings of the differing cells, from AVOID.aheadAt[0] m ahead to the scan's
  // surface behind them, at most aheadAt[1]; one seen again within 0.5 m is moved, not doubled) only with depth support
  // (aheadDeep of the window nearer than the scan, on two frames in a row), or on looks alone once it loomed: after aheadLoom[0] m flown toward it,
  // grown aheadLoom[1] times faster than the scan's surface behind it would have (lighting on a wall grows with the wall).
  // A wedge goes once the window sees its bearings as the scan has them again (nearFrames frames), and none is made where
  // something already on the map (a door leaf, a live-depth obstacle, a suspected change) explains what differs.
  // The pose's error has its say (a window at least AVOID.aheadCheck unlike the scan): the expected view is rendered where
  // the drone thinks it is, and a height 0.2 m off moves a sofa's back or a floor's pattern across the window, so a cell
  // counts only if it still differs seen from the camera moved as nav/changes.js heldUp() moves it (up and down by 2 σz:
  // the frame's, else CHANGES.zSigma, or σ for a pose as sure as the simulator's truth; sideways by 2 σ): its depth
  // (heldUp()), or, where only its looks differ, its looks (looksHeld()). (σz as the localizer says it, not floored as the
  // change detector floors it: a looks-only cell the sweep explains is lost evidence of a plain new thing. Only on a grid
  // with each cell's live depth, as differences() makes it.)
  wayAhead(f, K, now) {
    if (!f.expected || !f.rgb || !f.expectedRgb || !(f.pose.sigma <= AVOID.aheadSigma)) return;
    const g = (f.grid ??= differences(f)), v0 = K.cy + K.fy * Math.tan(((f.lens.uptiltDeg ?? 0) * Math.PI) / 180), [wx, wy] = AVOID.window;
    if (g.off > AVOID.aheadOff && !this.ahead?.slow) return;
    const win = [];
    for (let k = 0; k < g.gw * g.gh; k++) {
      const u = ((k % g.gw) + 0.5) * g.C, v = (((k / g.gw) | 0) + 0.5) * g.C;
      if (!(Math.abs(u - K.cx) > wx * f.width || Math.abs(v - v0) > wy * f.height || g.n[k] < (g.C * g.C) / 4)) win.push(k);
    }
    let held = null, still = null;
    if (g.lv && g.far && win.length && win.filter((k) => g.looks?.[k] || g.near[k]).length >= AVOID.aheadCheck * win.length) {
      held = heldUp(g, f, { z: f.pose.zSigma ?? (f.pose.sigma <= CHANGES.exact ? f.pose.sigma : CHANGES.zSigma), xy: f.pose.sigma });
      const only = win.filter((k) => g.looks?.[k] && !g.near[k] && !g.far[k]);
      const keep = this.aheadGuess && f.t - this.aheadGuess.t < 1000 ? this.aheadGuess.key : null, lh = only.length && f.rgb.data && f.expectedRgb.data ? looksHeld(f, g, K, droneCamera(f.pose, f.lens.uptiltDeg ?? 0).R, held.moves, only, keep) : null;
      still = lh ? lh.still : new Set(only);
      if (lh?.sure) this.aheadGuess = { key: lh.guess, t: f.t };
    }
    let n = 0, odd = 0, deep = 0, u0 = Infinity, u1 = -Infinity;
    const behind = [];
    for (const k of win) {
      const u = ((k % g.gw) + 0.5) * g.C, v = (((k / g.gw) | 0) + 0.5) * g.C, near = held ? held.near[k] : g.near[k];
      n++;
      if (!(held ? near || held.far[k] || still.has(k) : g.looks?.[k] || near)) continue;
      odd++;
      deep += near;
      [u0, u1] = [Math.min(u0, u - g.C / 2), Math.max(u1, u + g.C / 2)];
      behind.push(g.expected[(v | 0) * f.width + (u | 0)]);
    }
    const share = n ? odd / n : 0, last = this.aheadLast, P = f.pose, deepShare = n ? deep / n : 0;
    this.aheadLast = { share, deep: deepShare, t: f.t };
    // a wedge whose bearings the window now sees as the scan has them (on nearFrames frames) goes: it stands for something
    // seen there only while it is seen (a lighting or lens artefact never sticks)
    const half = Math.atan((wx * f.width) / K.fx);
    for (const t of [...this.temps.values()]) {
      if (t.source !== "way ahead") continue;
      const inView = n > 0 && Math.abs(wrapAngle(Math.atan2(t.y - P.y, t.x - P.x) - P.yaw)) < half && Math.hypot(t.x - P.x, t.y - P.y) < AVOID.aheadAt[1] + 0.3;
      t.clear = inView && share < AVOID.aheadSlow / 2 ? (t.clear ?? 0) + 1 : inView ? 0 : t.clear ?? 0;
      if (t.clear >= AVOID.nearFrames) this.removeTemp(t.id);
    }
    const both = Math.min(share, last && f.t - last.t < 1000 ? last.share : 0), far = behind.filter((e) => e > 0).sort((a, b) => a - b), B = far[far.length >> 1];
    const keep = share >= AVOID.aheadSlow / 2, was = this.ahead && f.t - this.ahead.t < 2 * AVOID.depthAge ? this.ahead : null;
    // looming: the share as it was when the drone was aheadLoom[0] m back along its heading, against how much the scan's
    // surface behind it would have grown since
    const hist = (this.aheadHist = [...(was ? this.aheadHist ?? [] : []).filter((q) => f.t - q.t < 8000), { t: f.t, x: P.x, y: P.y, share, B }]);
    const ahead = (q) => (P.x - q.x) * Math.cos(P.yaw) + (P.y - q.y) * Math.sin(P.yaw), b = hist.findLastIndex((q) => ahead(q) >= AVOID.aheadLoom[0]), back = hist[b], flown = back ? ahead(back) : 0;
    const mean = (i) => { const q = hist.slice(Math.max(0, i - 1), i + 2); return q.reduce((s, h) => s + h.share, 0) / q.length; }, then = back ? mean(b) : 0, nowShare = mean(hist.length - 2);
    const looms = !!back && then > 0.05 && back.B > flown + 0.1 && Math.sqrt(nowShare / then) > AVOID.aheadLoom[1] * (back.B / (back.B - flown));
    // depth support on two frames in a row (one frame's depth spike is noise: the O4's blur and compression made one every
    // few seconds in the twin)
    const deepBoth = Math.min(deepShare, last && f.t - last.t < 1000 ? last.deep ?? 0 : 0), backed = deepBoth >= AVOID.aheadDeep || looms || (!!was?.backed && keep);
    this.ahead = { t: f.t, share, deep: deepShare, backed, slow: both >= AVOID.aheadSlow || (!!was?.slow && keep), stop: backed && (both >= AVOID.aheadStop || (!!was?.stop && keep)) };
    if (!this.ahead.slow || !odd || !backed) return;
    const [r0, rMax] = AVOID.aheadAt, r1 = clamp((B ?? rMax) - 0.1, r0 + 0.3, rMax);
    const pad = 0.1, bs = [P.yaw - Math.atan((u1 - K.cx) / K.fx) - pad, P.yaw - Math.atan((u0 - K.cx) / K.fx) + pad], at = (r, b) => [P.x + r * Math.cos(b), P.y + r * Math.sin(b)];
    const polygon = [at(r0, bs[0]), at(r1, bs[0]), at(r1, (bs[0] + bs[1]) / 2), at(r1, bs[1]), at(r0, bs[1])];
    const [x, y] = centroid(polygon), old = [...this.temps.values()].find((t) => t.source === "way ahead" && Math.hypot(t.x - x, t.y - y) < 0.5);
    // already on the map along its middle bearing, before the scan's surface (a door leaf, what live depth placed, a
    // suspected change): that is what differs, at a known place; no wedge in front of it (one closed off a side doorway)
    const mid = (bs[0] + bs[1]) / 2, hit = (t) => { for (let r = r0; r <= (B ?? rMax) + 0.2; r += 0.1) { const [qx, qy] = at(r, mid); if (t.polygon ? polyDist(qx, qy, t.polygon) < 0.05 : Math.hypot(qx - t.x, qy - t.y) < (t.r ?? 0)) return true; } return false; };
    if ([...(this.map.temps?.values() ?? []), ...(this.leafMaybe?.values() ?? [])].some((t) => t.kind !== "person" && t.source !== "way ahead" && hit(t))) return old && this.removeTemp(old.id);
    // its height is as unknown as its range: every height (nothing to climb over)
    this.addTemp({ id: old?.id ?? `ahead-${++this.nearId}`, kind: "obstacle", source: "way ahead", x, y, r: reach([x, y], polygon), polygon, until: now + AVOID.nearKeep }, now);
  }

  // An unmapped obstacle (or door leaf) the camera now sees past (the depth toward its middle well beyond it) on 2 frames
  // goes.
  seenClear(f, K, cam, now) {
    const R = cam.R;
    for (const t of [...this.temps.values()]) {
      if (t.source !== "live depth" && t.source !== "door leaf") continue;
      const v = [t.x - cam.p[0], t.y - cam.p[1], (t.zMin + t.zMax) / 2 - cam.p[2]], c = [0, 1, 2].map((a) => R[0][a] * v[0] + R[1][a] * v[1] + R[2][a] * v[2]);
      const px = c[2] > 0.3 && c[2] < AVOID.nearRange + 1 && project(K, c);
      if (!px || px[0] < 0 || px[1] < 0 || px[0] >= f.width || px[1] >= f.height) continue;
      const d = f.depth[(px[1] | 0) * f.width + (px[0] | 0)], far = Math.hypot(...v) + Math.max(0.3, t.r);
      t.clear = d > far ? (t.clear ?? 0) + 1 : 0;
      if (t.clear >= AVOID.nearFrames) this.removeTemp(t.id);
    }
  }

  // Live points in free space, clustered on a 0.2 m grid: cells seen on nearFrames frames in a row make a sighting (the
  // outline of its points as read and as near as they may be, its depth read too far, and their heights). Each temporary
  // obstacle is the outline of its sightings in the last nearKeep ms (padded nearPad m, what it hides behind its face),
  // from the floor (new things stand on it) to their highest point (+ 0.1 m): it never grows past what was seen, and a
  // later misread doesn't move it off the face seen before. A sighting overlapping several joins them.
  nearField(f, now) {
    const { n, x, y, z, near, cx, cy } = this.cloud, cells = new Map(), g = 0.2;
    for (let i = 0; i < n; i++) {
      if (near[i] !== 1) continue;
      const key = `${Math.floor(x[i] / g)},${Math.floor(y[i] / g)}`;
      const c = cells.get(key) ?? cells.set(key, { n: 0, pts: [] }).get(key);
      c.n++;
      c.pts.push(i);
    }
    const next = new Map();
    for (const [key, c] of cells) if (c.n >= AVOID.nearMin) next.set(key, { frames: (this.seen.get(key)?.frames ?? 0) + 1, ...c });
    this.seen = next;
    const done = new Set();
    for (const [key, c] of next) {
      if (done.has(key) || c.frames < AVOID.nearFrames) continue;
      const group = [], stack = [key];
      done.add(key);
      while (stack.length) {
        const k = stack.pop(), [i, j] = k.split(",").map(Number);
        group.push(next.get(k));
        for (let a = -1; a <= 1; a++)
          for (let b = -1; b <= 1; b++) {
            const q = `${i + a},${j + b}`;
            if (!done.has(q) && next.get(q)?.frames >= AVOID.nearFrames) (done.add(q), stack.push(q));
          }
      }
      const idx = group.flatMap((q) => q.pts), hull = convexHull(idx.flatMap((i) => [[x[i], y[i]], [cx[i], cy[i]]]));
      const sight = { t: now, hull, z0: Math.min(...idx.map((i) => z[i])), z1: Math.max(...idx.map((i) => z[i])) };
      const same = [...this.temps.values()].filter((t) => t.source === "live depth" && t.sightings && hull.some((q) => polyDist(q[0], q[1], t.polygon) < 0.2));
      for (const t of same.slice(1)) (same[0].sightings.push(...t.sightings), this.removeTemp(t.id));
      const sightings = [...(same[0]?.sightings ?? []).filter((s) => now - s.t < AVOID.nearKeep), sight], outline = convexHull(sightings.flatMap((s) => s.hull));
      const polygon = convexHull(outline.flatMap(([px, py]) => Array.from({ length: 8 }, (_, a) => [px + AVOID.nearPad * Math.cos((a * Math.PI) / 4), py + AVOID.nearPad * Math.sin((a * Math.PI) / 4)])));
      const [ox, oy] = centroid(polygon);
      this.addTemp({ id: same[0]?.id ?? `near-${++this.nearId}`, kind: "obstacle", source: "live depth", x: ox, y: oy, r: reach([ox, oy], polygon), polygon, sightings,
        zMin: Math.min(this.map.floorAt(ox, oy) ?? Infinity, ...sightings.map((s) => s.z0)) - 0.1, zMax: Math.max(...sightings.map((s) => s.z1)) + 0.1, until: now + AVOID.nearKeep }, now);
    }
  }

  // Door leaves: live points on a doorway's plane (near 2: a closed door the scan doesn't have; nav/changes.js reports it
  // once it has seen it from enough places), nearMin or more, and the doorway read closed cell by cell (doorReading(): only
  // where the scan's far side is beyond what a closed door's depth can read, not at a grazing angle, clear of the jambs;
  // nearer than halfway to that far side on CHANGES.doorShare of them, over leafWidth m of the opening) on leafFrames frames
  // in a row, from viewpoints leafApart m (or leafTurn degrees of bearing) apart, not seen through in the last leafOpen ms,
  // from within leafHeadOn of the doorway's normal or seen so from two bearings leafSpread apart (in nearKeep ms), and not
  // through it at another doorway just behind it (Room 3's closed doorways seen through Room 2's open one read as that one
  // closed, 3 of 6 sealed runs in the browser; until then a maybe-leaf, which only keeps way-ahead wedges off it): a
  // temporary obstacle on the doorway itself, never in front of it (the depth error that way would close the doorways
  // beside it too), over its width (the part seen closed in the last nearKeep ms when some was seen through), sill to head,
  // padded nearPad m, for leafKeep ms after the last closed reading unless seen through on nearFrames frames: the planner,
  // the geofence and the cap keep off it as off anything on the map (a push toward the free space the scan has there would
  // lean on it). Every reading goes to leafLog (the last 200), for checks.
  leaves(f, K, cam, err, now) {
    const { n, near, door } = this.cloud, by = new Map(), seen = new Map(), views = (this.leafViews ??= new Map()), log = (this.leafLog ??= []);
    for (let i = 0; i < n; i++) if (near[i] === 2) by.set(door[i], (by.get(door[i]) ?? 0) + 1);
    for (const [k, dr] of this.map.doors.entries()) {
      const ex = dr.b[0] - dr.a[0], ey = dr.b[1] - dr.a[1], m0 = [dr.a[0] + ex / 2, dr.a[1] + ey / 2];
      if (Math.hypot(m0[0] - cam.p[0], m0[1] - cam.p[1]) > AVOID.range + 1) continue;
      const rd = doorReading(f, K, cam, dr, this.map, err), count = by.get(k) ?? 0;
      if (!rd.n) continue;
      const mine = (views.get(dr.id) ?? []).filter((v) => now - v.t < AVOID.nearKeep), closed = rd.closed && count >= AVOID.nearMin;
      const bearing = Math.atan2(cam.p[1] - m0[1], cam.p[0] - m0[0]), off = Math.acos(Math.min(1, Math.abs(Math.cos(bearing) * -ey + Math.sin(bearing) * ex) / Math.hypot(ex, ey))) * (180 / Math.PI);
      mine.push({ t: now, x: cam.p[0], y: cam.p[1], bearing, closed, open: rd.open });
      views.set(dr.id, mine);
      const id = `leaf-${dr.id}`, old = this.temps.get(id);
      if (old && rd.open && mine.slice(-AVOID.nearFrames).every((v) => v.open)) this.removeTemp(id); // seen through it: open after all
      if (rd.closed || rd.open || count >= AVOID.nearMin) log.push({ t: Math.round(now), door: dr.id, at: cam.p.map((v) => +v.toFixed(2)), off: Math.round(off), points: count, n: rd.n, shut: rd.shut, through: rd.through, L: +rd.L.toFixed(2), width: +rd.width.toFixed(2), closed, open: rd.open });
      while (log.length > 200) log.shift();
      if (!closed) continue;
      seen.set(k, (this.leafSeen?.get(k) ?? 0) + 1);
      const shut = mine.filter((v) => v.closed);
      let apart = 0, turn = 0;
      for (const a of shut) for (const b of shut) [apart, turn] = [Math.max(apart, Math.hypot(a.x - b.x, a.y - b.y)), Math.max(turn, Math.abs(wrapAngle(a.bearing - b.bearing)) * (180 / Math.PI))];
      const spread = Math.max(...shut.map((v) => Math.abs(wrapAngle(v.bearing - bearing)))) * (180 / Math.PI), through = mine.some((v) => v.open && now - v.t < AVOID.leafOpen);
      if (seen.get(k) < AVOID.leafFrames) continue;
      const sure = old || (!through && (apart >= AVOID.leafApart || turn >= AVOID.leafTurn) && (off <= AVOID.leafHeadOn || spread >= AVOID.leafSpread) && !behind(this.map, cam.p, dr, err, rd.s));
      log.at(-1).sure = !!sure;
      if (sure && !old) (this.leafMade ??= []).push(log.at(-1)) > 20 && this.leafMade.shift(); // the reading each leaf was made on
      // the whole opening unless some of it was seen through (a door closes its doorway: a part left open on the map let
      // the planner route through the rest of a 2.2 m doorway seen closed)
      const sightings = [...(old?.sightings ?? []).filter((v) => now - v.t < AVOID.nearKeep), rd.through ? { t: now, s0: rd.s[0], s1: rd.s[1] } : { t: now, s0: 0, s1: 1 }];
      const s0 = Math.min(...sightings.map((v) => v.s0)), s1 = Math.max(...sightings.map((v) => v.s1)), L = Math.hypot(ex, ey);
      const at = (u, o) => [dr.a[0] + u * ex - (o * ey) / L, dr.a[1] + u * ey + (o * ex) / L];
      const polygon = convexHull([at(s0, -0.03), at(s0, 0.03), at(s1, -0.03), at(s1, 0.03)].flatMap(([px, py]) => Array.from({ length: 8 }, (_, a) => [px + AVOID.nearPad * Math.cos((a * Math.PI) / 4), py + AVOID.nearPad * Math.sin((a * Math.PI) / 4)])));
      const [ox, oy] = centroid(polygon), floor = this.map.floorAt(ox, oy) ?? dr.sillZ ?? 0;
      const leaf = { id, kind: "obstacle", source: "door leaf", door: dr.id, x: ox, y: oy, r: reach([ox, oy], polygon), polygon, sightings,
        zMin: Math.min(floor, dr.sillZ ?? floor) - 0.1, zMax: (dr.headZ ?? floor + 2.1) + 0.1, until: now + (sure ? AVOID.leafKeep : AVOID.nearKeep) };
      // not sure yet (one viewpoint, one oblique view, seen through lately, or another doorway just behind): no leaf for the
      // planner, but it explains what the way ahead sees (no wedge in front of a doorway whose far side is closed: Room 3's
      // archway seen through Room 2's doorway)
      if (sure) (this.addTemp(leaf, now), this.leafMaybe?.delete(id));
      else (this.leafMaybe ??= new Map()).set(id, leaf);
    }
    this.leafSeen = seen;
    for (const [id, t] of this.leafMaybe ?? []) if (t.until <= now) this.leafMaybe.delete(id);
  }

  // Distance along the horizontal unit direction from p before the drone's sphere (radius need, stretched up and down by
  // the height's 2σz) meets a live point (a new thing's at the nearest it may be, and margin m nearer: no creeping up to
  // it); the third nearest, so a stray pixel doesn't stop it.
  corridor(p, [dx, dy], need, now) {
    const c = this.cloud, dz = 2 * (p.zSigma ?? 0);
    if (!c || now - c.t > AVOID.depthAge) return Infinity;
    const best = [Infinity, Infinity, Infinity];
    for (let i = 0; i < c.n; i++) {
      const rx = c.cx[i] - p.x, ry = c.cy[i] - p.y, rz = Math.max(0, Math.abs(c.cz[i] - p.z) - dz), along = rx * dx + ry * dy;
      if (along <= 0) continue;
      const lat2 = rx * rx + ry * ry - along * along + rz * rz;
      if (lat2 >= need * need) continue;
      const d = along - Math.sqrt(need * need - lat2) - (c.near[i] ? AVOID.margin : 0);
      if (d < best[2]) (best[2] = d), best.sort((a, b) => a - b);
    }
    return Math.max(0, best[2]);
  }

  // ---- the speed cap ----

  // Climbing and sinking: the column above (below) the drone's body, from its height ± 2σz, before something comes within
  // its radius: the map without its temporary obstacles (sinking: down to 0.12 m over the floor, which a landing sets down
  // on), and the temporary obstacles it would enter from above (below): over their outline (within its radius + 2σ), now
  // above (below) their heights. One it is already inside (a person's zone, at every height it may be at) is the horizontal cap's: up or down,
  // it is as close. The height loop brakes gently (vDecel, vReact) and there is no creep: it stops margin m short
  // ("something below": a landing knows it isn't down), except a forced landing (sp.touchdown: the battery is empty), which
  // sets down on what is below at AVOID.touch m/s. Only what comes closer on the way counts: the clearance is a ball's (and a
  // band's), so a drone sunk to just over a table read its top as "something above" and could never climb back off it.
  vertical(sp, p, T, map = this.map) {
    const vz = sp.vz, floor = map.floorAt(p.x, p.y);
    if (!(Math.abs(vz) > 0.02) || floor == null) return sp;
    const up = vz > 0, r = map.o.droneRadius, dz = 2 * (p.zSigma ?? 0), wide = r + 2 * (p.sigma ?? 0);
    const over = [...(map.temps?.values() ?? this.temps.values())].filter((t) => (up ? p.z < (t.zMin ?? -Infinity) : p.z > (t.zMax ?? Infinity)) &&
      (t.polygon ? polyDist(p.x, p.y, t.polygon) : Math.hypot(p.x - t.x, p.y - t.y) - (t.r ?? 0)) < wide);
    let d = 0;
    const z0 = Math.max(p.z, floor + 0.1), known = floor + map.o.bands.at(-1) + (map.o.bandHalf ?? 0); // a height estimate under the floor is wrong, not a reason to stay down
    let c = staticClearance(map, p.x, p.y, z0);
    for (; d < 2.5; d += 0.05) {
      const z = z0 + (up ? 1 : -1) * (d + 0.05), was = c;
      if (!up && z < floor + 0.12) return sp;
      if (!up && z > known) continue; // over every band the map knows nothing (all "0"): sinking from there is back to what it knows
      c = staticClearance(map, p.x, p.y, z);
      if ((c < r && c < was - 0.01) || over.some((t) => (up ? z + r > t.zMin : z - r < t.zMax))) break;
    }
    const cap = stoppable(d - dz - AVOID.margin, T + AVOID.vReact, AVOID.vDecel);
    if (Math.abs(vz) <= cap) return sp;
    if (sp.touchdown && !up) return (this.last.vertical = "setting down on something below"), { ...sp, vz: -Math.max(cap, Math.min(-vz, AVOID.touch)) };
    this.last.vertical = up ? "something above" : "something below";
    return { ...sp, vz: Math.sign(vz) * cap };
  }

  // sp: body-frame setpoint ({ vx forward, vy right, vz up }); p: the pose. Returns sp with the speeds capped.
  limit(sp, p, now = performance.now()) {
    this.expire(now);
    const br = this.brake(), T = (this.ctl.videoDelay ?? 0) / 1000 + br.react, eyes = this.real ? this.eyes(now) : { ok: true }, dms = this.lastT == null ? 0 : clamp(now - this.lastT, 0, 100);
    if (!eyes.ok) this.blindMs += dms;
    this.lastT = now;
    this.last = { ...this.last, vertical: "" };
    const pass = this.passable(now), map = withoutTemps(this.map, pass); // without what is being passed
    sp = this.vertical(sp, p, (this.ctl.videoDelay ?? 0) / 1000, map);
    if (sp.vx === undefined && sp.vy === undefined) return sp;
    const c = Math.cos(p.yaw), s = Math.sin(p.yaw), bx = sp.vx ?? 0, by = sp.vy ?? 0;
    const hx = c * bx + s * by, hy = s * bx - c * by, v = Math.hypot(hx, hy);
    if (v < 0.02) return sp;
    // the map's free run at every height the drone may be at (± 2σz, within the bands the geofence checks)
    const dir = [hx / v, hy / v], need = map.o.droneRadius + 2 * p.sigma, floor = map.floorAt(p.x, p.y), dz = 2 * (p.zSigma ?? 0);
    const zs = floor == null || dz < 0.05 ? [p.z] : [p.z - dz, p.z, p.z + dz].map((z) => clamp(z, floor + 0.3, floor + map.o.bands.at(-1)));
    const look = this.settings?.get?.("avoid") !== false, run = Math.min(...zs.map((z) => freeRun(map, { ...p, z }, dir, need))), live = look ? this.corridor(p, dir, need, now) : Infinity, D = Math.min(run, live);
    const vel = this.loc.velocity?.() ?? [0, 0], moving = Math.max(v, vel[0] * dir[0] + vel[1] * dir[1]);
    let cap = Math.max(stoppable(D - AVOID.margin, T, br.decel), Math.min(AVOID.creep, stoppable(D - 0.02, T, br.decel)));
    // the drone already moves faster than the setpoint toward it (drift, momentum): the cap leaves room for that
    if (moving > v) cap = Math.max(0, cap - (moving - v));
    let why = cap < v ? (live < run ? "something ahead (live depth)" : "an obstacle on the map ahead") : "";
    // Inside a temporary obstacle (something the live depth found, a suspected change; people have their own standoff):
    // the map says nothing there, so only away from its middle passes (close up, the camera may no longer see it at all).
    for (const t of map.temps?.values() ?? this.temps.values()) {
      if (t.kind === "person" || p.z < (t.zMin ?? -Infinity) - need || p.z > (t.zMax ?? Infinity) + need) continue;
      const [mx, my] = t.polygon ? centroid(t.polygon) : [t.x, t.y];
      const inside = t.polygon ? polyDist(p.x, p.y, t.polygon) < need : Math.hypot(p.x - mx, p.y - my) < t.r + need;
      if (inside && dir[0] * (mx - p.x) + dir[1] * (my - p.y) > 0) (cap = 0), (why = "inside something new the map lacks");
    }
    const ttc = look ? this.ctl.est?.ttc ?? Infinity : Infinity, forward = Math.abs(wrapAngle(Math.atan2(dir[1], dir[0]) - p.yaw)) < 0.7, a = this.ahead;
    if (ttc < AVOID.ttcSlow && forward) {
      const k = clamp((ttc - AVOID.ttcStop) / (AVOID.ttcSlow - AVOID.ttcStop), 0, 1);
      if (v * k < cap) (cap = v * k), (why = "something ahead (image zoom)");
    }
    if (look && forward && a?.slow && now - a.t < 2 * AVOID.depthAge) {
      const c2 = a.stop ? 0 : AVOID.creep;
      if (c2 < cap) (cap = c2), (why = "something the map lacks is in the way ahead");
    }
    const near = (t) => t && (t.polygon ? polyDist(p.x, p.y, t.polygon) : Math.hypot(p.x - t.x, p.y - t.y) - (t.r ?? 0)) < AVOID.passNear;
    for (const id of pass) if (near(this.map.temps.get(id)) && AVOID.creep < cap) (cap = AVOID.creep), (why = "slowly past something Claude saw no change in"); // passing a change Claude called no change
    if (this.lifted.size && AVOID.creep < cap && [...this.lifted.values()].some(near) && !(this.real ? eyes : this.eyes(now)).ok) (cap = AVOID.creep), (why = "slowly past something I took off the map: I can't see right now");
    if (!eyes.ok && AVOID.blind < cap) (cap = AVOID.blind), (why = `flying slowly: ${eyes.why}`);
    const top = this.realTop();
    if (top.v < cap) (cap = top.v), (why = top.why);
    this.last = { run, live, cap, ttc: D / Math.max(0.05, moving), why, t: now, held: cap < 0.03 && v >= 0.1, blind: !eyes.ok && eyes.why, vertical: this.last.vertical };
    if (cap >= v) return sp;
    const k = cap / v;
    return { ...sp, vx: bx * k, vy: by * k };
  }
}

// Is a live point at d m along the unit ray from o on a doorway's plane, as nav/changes.js takes a door leaf: the ray
// crosses the doorway (across its width, sill to head) at t, and d is no nearer than t - max(0.25, 0.15 t) and no farther
// than a new thing's depth can read too far (t (1 + err) + that)? Of the doorways that fit, the one it is nearest to (off
// its plane plus past its ends): doorways meeting at a corner (Room 3's doorway in line with Room 2's, the archway across
// their end) mustn't take each other's closed leaf. -> { k (index in map.doors), s (0-1 along it) } | null
export function onDoorway(map, o, ray, d, err) {
  let best = null;
  for (const [k, dr] of map.doors.entries()) {
    const ex = dr.b[0] - dr.a[0], ey = dr.b[1] - dr.a[1], den = ray[0] * ey - ray[1] * ex;
    if (Math.abs(den) < 1e-6) continue;
    const t = ((dr.a[0] - o[0]) * ey - (dr.a[1] - o[1]) * ex) / den, s = ((dr.a[0] - o[0]) * ray[1] - (dr.a[1] - o[1]) * ray[0]) / den, z = o[2] + t * ray[2], tol = Math.max(0.25, 0.15 * t);
    if (!(t > 0 && s > -0.05 && s < 1.05 && z > (dr.sillZ ?? -Infinity) - 0.1 && z < (dr.headZ ?? Infinity) + 0.1 && d > t - tol && d < t * (1 + err) + tol)) continue;
    const score = Math.max(0, -s, s - 1) * Math.hypot(ex, ey) + Math.abs(d - t); // m off its plane, plus m past its ends
    if (!best || score < best.score) best = { k, s, score };
  }
  return best && { k: best.k, s: best.s };
}

// How a doorway reads in a live depth frame, over its opening (12 across, 10 from sill to head), at the cells in the picture
// (no person or pet over them), seen within AVOID.leafGraze of the doorway's normal (further round, the opening is a sliver
// and the depth's soft edges fill it with the jambs' and walls' depth), clear of the jambs (the wall's thickness at that
// angle, leafJamb more), with a far side beyond what a closed door's depth can read (a new thing's read err too far, and
// the tolerance): the scan's depth there, or another doorway the ray crosses behind this one (it may be closed: the sealed
// Room 3 behind Room 2's open doorway). Of those (n), shut: the live depth on the plane (no nearer than the tolerance, no
// farther than a door's depth reads too far: leafErr) and nearer than leafFar of the way to the far side; through: beyond
// what a closed door (or a new thing) can read and past leafFar of the way.
// -> { n, shut, through, closed (shut on CHANGES.doorShare of n, at least 12, over leafWidth m of the opening or half of
// it), open (through on half of n, at least 12), s: [s0, s1] (the part read shut), L (median range), width (m read shut) }
export function doorReading(f, K, cam, dr, map, err = AVOID.depthErr) {
  const R = cam.R, c = cam.p, ex = dr.b[0] - dr.a[0], ey = dr.b[1] - dr.a[1], W = Math.hypot(ex, ey), [nx, ny] = [-ey / W, ex / W], [z0, z1] = [dr.sillZ ?? 0, dr.headZ ?? 2];
  const thick = Math.max(dr.depth ?? 0, AVOID.leafThick), graze = Math.cos((AVOID.leafGraze * Math.PI) / 180), Ls = [], ss = [];
  let n = 0, shut = 0, through = 0;
  for (let i = 0; i < 12; i++)
    for (let j = 0; j < 10; j++) {
      const sv = (i + 0.5) / 12, P = [dr.a[0] + sv * ex - c[0], dr.a[1] + sv * ey - c[1], z0 + ((j + 0.5) / 10) * (z1 - z0) - c[2]], L = Math.hypot(...P), across = P[0] * nx + P[1] * ny, along = (P[0] * ex + P[1] * ey) / W;
      if (Math.abs(across) < graze * L) continue;
      // the jamb the ray runs toward hides thick x tan(angle) of the opening's side; both edges blur leafJamb into it
      const hide = (thick * Math.abs(along)) / Math.abs(across) + AVOID.leafJamb;
      if ((along > 0 ? (1 - sv) * W < hide || sv * W < AVOID.leafJamb : sv * W < hide || (1 - sv) * W < AVOID.leafJamb)) continue;
      const cc = [0, 1, 2].map((a) => R[0][a] * P[0] + R[1][a] * P[1] + R[2][a] * P[2]), px = cc[2] > 0.1 && project(K, cc);
      if (!px || !(px[0] >= 0 && px[1] >= 0 && px[0] < f.width && px[1] < f.height) || f.boxes?.some((b) => inBox(b, px[0] / f.width, px[1] / f.height, 0))) continue;
      const k = (px[1] | 0) * f.width + (px[0] | 0), e = f.expected?.[k], a = f.depth[k], tol = Math.max(CHANGES.door, CHANGES.doorRel * L), most = L * (1 + err) + tol;
      if (!(a > 0) || !(e > 0)) continue;
      let far = e;
      for (const q of map.doors) {
        if (q === dr) continue;
        const qx = q.b[0] - q.a[0], qy = q.b[1] - q.a[1], den = P[0] * qy - P[1] * qx;
        if (Math.abs(den) < 1e-9) continue;
        const u = ((q.a[0] - c[0]) * qy - (q.a[1] - c[1]) * qx) / den, s2 = ((q.a[0] - c[0]) * P[1] - (q.a[1] - c[1]) * P[0]) / den, z = c[2] + u * P[2];
        if (u > 1.02 && s2 > -0.05 && s2 < 1.05 && z > (q.sillZ ?? -Infinity) && z < (q.headZ ?? Infinity)) far = Math.min(far, u * L);
      }
      if (!(far > most)) continue;
      n++;
      Ls.push(L);
      const mid = L + AVOID.leafFar * (far - L);
      if (a > L - tol && a < Math.min(mid, L * (1 + AVOID.leafErr) + tol)) (shut++, ss.push(sv));
      else if (a >= mid && a > most) through++;
    }
  const s = ss.length ? [Math.min(...ss), Math.max(...ss)] : [0, 0], width = ss.length ? (s[1] - s[0] + 1 / 12) * W : 0;
  return { n, shut, through, closed: n >= 12 && shut >= CHANGES.doorShare * n && width >= Math.min(AVOID.leafWidth, W / 2), open: n >= 12 && through >= n / 2, s, width,
    L: Ls.length ? Ls.sort((p, q) => p - q)[Ls.length >> 1] : NaN };
}

// Another doorway just behind doorway dr as seen from o (across most of the part [s0, s1] of its width seen closed, within
// the reach of its plane test: a new thing's depth read up to err too far, and the tolerance), whose closed leaf would read
// as dr's -> that doorway | null.
export function behind(map, o, dr, err, [s0, s1] = [0, 1]) {
  const ex = dr.b[0] - dr.a[0], ey = dr.b[1] - dr.a[1], us = [0.1, 0.3, 0.5, 0.7, 0.9].map((u) => s0 + u * (s1 - s0));
  for (const q of map.doors) {
    if (q === dr) continue;
    const qx = q.b[0] - q.a[0], qy = q.b[1] - q.a[1];
    let hits = 0;
    for (const u of us) {
      const px = dr.a[0] + u * ex - o[0], py = dr.a[1] + u * ey - o[1], t = Math.hypot(px, py), rx = px / t, ry = py / t, den = rx * qy - ry * qx;
      if (Math.abs(den) < 1e-6) continue;
      const t2 = ((q.a[0] - o[0]) * qy - (q.a[1] - o[1]) * qx) / den, s2 = ((q.a[0] - o[0]) * ry - (q.a[1] - o[1]) * rx) / den;
      if (s2 > 0 && s2 < 1 && t2 > t + 0.02 && t2 < t * (1 + err) + Math.max(0.25, 0.15 * t)) hits++;
    }
    if (hits > us.length / 2) return q;
  }
  return null;
}

// Typed arrays looksHeld() reuses frame to frame (no garbage on the page's thread): name -> array; the samples' rays per lens.
const scratch = new Map(), rayCache = { key: "", rays: null }, buf = (name, Type, n) => {
  let a = scratch.get(name);
  if (!(a instanceof Type) || a.length !== n) scratch.set(name, (a = new Type(n)));
  return a;
};

// Of `cells` (they look unlike the scan where its depth agrees: nav/changes.js differences()), those that still look
// unlike it from the camera moved by each of `moves` (world offsets: heldUp()'s). Each move's view of the expected surface
// is z-buffered as heldUp() does it (samples every C/2 px, and halfway to the next one on the same surface, into C/2 px
// bins, each keeping the nearest sample's offset from where the render had it), and a cell is compared with the expected
// picture at each offset its bins hold, as differences() compares (colour after the light's gain, within a cell; gradients
// over it and its 8 neighbours), on the picture as registered and as rendered; a cell nothing lands in, or one a nearer
// surface moves over (by CHANGES.jump: a sofa's back moved over the floor behind it shows faces the render never saw),
// shows what the render never saw from there: no telling. The pose's error is
// one move for the whole picture: each move, or a height error between two (up: 2 σz and σz; down the same), is a guess,
// and the one explaining the most cells takes them off if it explains at least AVOID.aheadPosed of them (a real thing's
// cells match some move here and there by chance: a fifth to a third of them in the twin), or if it is the guess taken
// on a frame in the last second by that share (`keep`: the error lasts). -> { still: Set of cells, guess: the key of the
// guess taken, sure: taken by that share }
function looksHeld(f, g, K, R, moves, cells, keep = null) {
  const { width: w, height: h } = f, { C, gw, gh } = g, N = gw * gh, D = f.expected, s = g.shift ?? { sx: 0, sy: 0 }, w2 = w >> 1, h2 = h >> 1, C2 = C >> 1, W2 = w2 + 1;
  // both pictures at half size (2 x 2 means, as differences() takes its gradients), the live one's cell means, the
  // expected one's summed-area table (its blocks anywhere)
  const half = (P, out) => {
    for (let v = 0; v < h2; v++)
      for (let u = 0; u < w2; u++) {
        const i = 4 * (2 * v * w + 2 * u), j = i + 4 * w, o = 3 * (v * w2 + u);
        for (let c = 0; c < 3; c++) out[o + c] = (P[i + c] + P[i + 4 + c] + P[j + c] + P[j + 4 + c]) / 4;
      }
    return out;
  };
  const HL = half(f.rgb.data, buf("hl", Float32Array, 3 * w2 * h2)), HE = half((g.expectedRgb ?? f.expectedRgb).data, buf("he", Float32Array, 3 * w2 * h2));
  const cl = buf("cl", Float32Array, 3 * N).fill(0), cn = buf("cn", Float32Array, N).fill(0), S = buf("sat", Float32Array, 3 * W2 * (h2 + 1)), em = [0, 0, 0];
  for (let v = 0; v < h2; v++) {
    let r0 = 0, r1 = 0, r2 = 0;
    const row = (((2 * v) / C) | 0) * gw;
    for (let u = 0; u < w2; u++) {
      const i = 3 * (v * w2 + u), k = row + (((2 * u) / C) | 0), o = 3 * ((v + 1) * W2 + u + 1), q = o - 3 * W2;
      (cl[3 * k] += HL[i]), (cl[3 * k + 1] += HL[i + 1]), (cl[3 * k + 2] += HL[i + 2]), (cn[k] += 4);
      (r0 += HE[i]), (r1 += HE[i + 1]), (r2 += HE[i + 2]);
      (S[o] = S[q] + r0), (S[o + 1] = S[q + 1] + r1), (S[o + 2] = S[q + 2] + r2);
    }
  }
  for (let k = 0; k < 3 * N; k++) cl[k] /= cn[(k / 3) | 0] / 4 || 1;
  const block = (x0, y0) => { // the expected picture's mean colour over the C x C px block from (x0, y0) px, clipped -> em
    const xa = Math.max(0, Math.min(w2, Math.round(x0 / 2))), xb = Math.max(0, Math.min(w2, Math.round(x0 / 2) + C2)), ya = Math.max(0, Math.min(h2, Math.round(y0 / 2))), yb = Math.max(0, Math.min(h2, Math.round(y0 / 2) + C2)), n = (xb - xa) * (yb - ya);
    if (n < (C2 * C2) / 4) return false;
    const A = 3 * (yb * W2 + xb), B = 3 * (ya * W2 + xb), P = 3 * (yb * W2 + xa), Q = 3 * (ya * W2 + xa);
    for (let c = 0; c < 3; c++) em[c] = (S[A + c] - S[B + c] - S[P + c] + S[Q + c]) / n;
    return true;
  };
  // the light's gain per channel around a cell (as differences(): per block of CHANGES.light cells, the median of live /
  // expected over it and the 8 around), for the blocks asked about
  const RL = CHANGES.light, bw = Math.ceil(gw / RL), bh = Math.ceil(gh / RL), ratio = new Map(), lit = new Map(), bOf = (k) => ((((k / gw) | 0) / RL) | 0) * bw + (((k % gw) / RL) | 0);
  const med = (a) => a.sort((x, y) => x - y)[a.length >> 1] ?? 1, ratios = (b) => {
    if (!ratio.has(b)) {
      const out = [[], [], []], c0 = (b % bw) * RL, r0 = ((b / bw) | 0) * RL;
      for (let r = r0; r < Math.min(gh, r0 + RL); r++)
        for (let c = c0; c < Math.min(gw, c0 + RL); c++) if (cn[r * gw + c] >= 4 && block(c * C, r * C)) for (let q = 0; q < 3; q++) if (em[q] > 20) out[q].push(cl[3 * (r * gw + c) + q] / em[q]);
      ratio.set(b, out);
    }
    return ratio.get(b);
  };
  let gain0 = null;
  const gainAt = (b) => {
    if (!lit.has(b))
      lit.set(b, [0, 1, 2].map((q) => {
        const rs = [];
        for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) { const x = (b % bw) + dc, y = ((b / bw) | 0) + dr; if (x >= 0 && y >= 0 && x < bw && y < bh) for (const v of ratios(y * bw + x)[q]) rs.push(v); }
        if (rs.length >= 8) return med(rs);
        gain0 ??= [0, 1, 2].map((c) => { const a = []; for (let i = 0; i < bw * bh; i++) for (const v of ratios(i)[c]) a.push(v); return med(a); });
        return gain0[q];
      }));
    return lit.get(b);
  };
  // gradients on the half-size greys, blurred, and each picture's noise floor (the 20th percentile of the cells' energy)
  const [lo, hi] = CHANGES.texture, grads = (HP, name) => {
    const y = buf("y", Float32Array, w2 * h2), gr = buf("gr", Float32Array, w2 * h2).fill(0), e = buf("e", Float32Array, N).fill(0), n = buf("n", Float32Array, N).fill(0), r = [];
    for (let i = 0; i < w2 * h2; i++) y[i] = 0.3 * HP[3 * i] + 0.59 * HP[3 * i + 1] + 0.11 * HP[3 * i + 2];
    for (let v = 1; v < h2 - 1; v++) for (let u = 1; u < w2 - 1; u++) { const i = v * w2 + u; gr[i] = Math.abs(y[i + 1] - y[i - 1]) + Math.abs(y[i + w2] - y[i - w2]); }
    const G = boxBlur(boxBlur(gr, w2, h2, 1, CHANGES.spread, buf("b1", Float32Array, w2 * h2)), w2, h2, w2, CHANGES.spread, buf(name, Float32Array, w2 * h2));
    for (let v = 1; v < h2 - 1; v++) for (let u = 1; u < w2 - 1; u++) { const k = (((2 * v) / C) | 0) * gw + (((2 * u) / C) | 0), a = G[v * w2 + u]; (e[k] += a * a), n[k]++; }
    for (let k = 0; k < N; k++) if (n[k] > 0 && cn[k] >= 4) r.push(e[k] / n[k]);
    return [G, r.sort((a, b) => a - b)[Math.floor(0.2 * (r.length - 1))] ?? 0];
  };
  const [GL, floorL] = grads(HL, "gl"), [GE, floorE] = grads(HE, "ge");
  // live cell k against the expected picture moved by (ox, oy) px: alike?
  const alike = (k, ox, oy) => {
    const c0 = k % gw, r0 = (k / gw) | 0, x0 = c0 * C, y0 = r0 * C, G = gainAt(bOf(k));
    let dc = Infinity;
    for (let dr = -1; dr <= 1; dr++)
      for (let dq = -1; dq <= 1; dq++) if (block(x0 + dq * C - ox, y0 + dr * C - oy)) dc = Math.min(dc, (Math.abs(cl[3 * k] - G[0] * em[0]) + Math.abs(cl[3 * k + 1] - G[1] * em[1]) + Math.abs(cl[3 * k + 2] - G[2] * em[2])) / 3);
    if (!(dc <= CHANGES.colour)) return false;
    const hx = Math.round(ox / 2), hy = Math.round(oy / 2);
    let n = 0, sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    for (let v = Math.max(1, (r0 - 1) * C2, hy + 1); v < Math.min(h2 - 1, (r0 + 2) * C2, h2 - 1 + hy); v++)
      for (let u = Math.max(1, (c0 - 1) * C2, hx + 1); u < Math.min(w2 - 1, (c0 + 2) * C2, w2 - 1 + hx); u++) {
        const a = GL[v * w2 + u], b = GE[(v - hy) * w2 + u - hx];
        n++, (sa += a), (sb += b), (saa += a * a), (sbb += b * b), (sab += a * b);
      }
    if (n < 4) return false;
    const ea = saa / n, eb = sbb / n, ra = ea / (floorL + 10), rb = eb / (floorE + 10), va = ea - (sa / n) ** 2, vb = eb - (sb / n) ** 2;
    const ncc = va > 1 && vb > 1 ? (sab / n - (sa / n) * (sb / n)) / Math.sqrt(va * vb) : 1;
    return !((ra > hi && rb > hi && ncc < CHANGES.ncc) || (Math.max(ra, rb) > hi && Math.min(ra, rb) < lo));
  };
  const left = g.shift ? cells.filter((k) => !alike(k, -s.sx, -s.sy)) : cells; // (as rendered, unmoved)
  if (!left.length) return { still: new Set(), guess: null, sure: false };
  const st = Math.max(1, C >> 1), ws = Math.ceil(w / st), hs = Math.ceil(h / st), M = ws * hs, jr = 1 + CHANGES.jump, pin = K.model === "pinhole";
  const zb = buf("zb", Float32Array, M), ox = buf("ox", Float32Array, M), oy = buf("oy", Float32Array, M), at = buf("at", Float32Array, 3 * M), Ej = buf("Ej", Float32Array, M);
  const key = `${K.model}|${w}x${h}|${K.fx}|${K.fy}|${K.cx}|${K.cy}|${st}`;
  if (rayCache.key !== key) {
    rayCache.rays = new Float32Array(3 * M).fill(NaN);
    for (let v = 0, j = 0; v < h; v += st) for (let u = 0; u < w; u += st, j++) { const r = unproject(K, u + 0.5, v + 0.5); if (r) rayCache.rays.set(r, 3 * j); }
    rayCache.key = key;
  }
  const rays = rayCache.rays;
  for (let v = 0, j = 0; v < h; v += st) for (let u = 0; u < w; u += st, j++) Ej[j] = rays[3 * j] === rays[3 * j] ? D[v * w + u] : NaN;
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity; // the cells left (px): nothing landing elsewhere matters,
  const m = 6 * C; // nor any sample from further off than this (px: a surface moved that far is within half a metre)
  const by = new Map(), tried = new Map(); // move's key -> the cells it explains; (cell, offset) -> alike
  for (const k of left) (x0 = Math.min(x0, (k % gw) * C)), (x1 = Math.max(x1, (k % gw) * C + C)), (y0 = Math.min(y0, ((k / gw) | 0) * C)), (y1 = Math.max(y1, ((k / gw) | 0) * C + C));
  for (const [i, t] of moves.entries()) {
    if (!i) continue;
    const tc = [0, 1, 2].map((a) => R[0][a] * t[0] + R[1][a] * t[1] + R[2][a] * t[2]), why = new Set();
    zb.fill(Infinity);
    for (let v = 0, j = 0; v < h; v += st)
      for (let u = 0; u < w; u += st, j++) {
        const e = Ej[j], x = rays[3 * j] * e - tc[0], y = rays[3 * j + 1] * e - tc[1], z = rays[3 * j + 2] * e - tc[2];
        at[3 * j] = NaN;
        if (!(e > 0) || !(z > 0.05) || u < x0 - m || u > x1 + m || v < y0 - m || v > y1 + m) continue;
        const px = pin ? null : project(K, [x, y, z]);
        if (pin) (at[3 * j] = K.cx + (K.fx * x) / z), (at[3 * j + 1] = K.cy + (K.fy * y) / z);
        else if (px) (at[3 * j] = px[0]), (at[3 * j + 1] = px[1]);
        at[3 * j + 2] = Math.sqrt(x * x + y * y + z * z);
      }
    for (let j = 0; j < M; j++) {
      const x = at[3 * j], y = at[3 * j + 1];
      if (!(x >= x0 - st && x < x1 + st && y >= y0 - st && y < y1 + st)) continue; // (NaN too)
      const d = at[3 * j + 2], dx = x - ((j % ws) * st + 0.5), dy = y - (((j / ws) | 0) * st + 0.5);
      for (let m = 0; m < 3; m++) { // the sample, and halfway to the next one along its row and column on the same surface
        const q = m === 0 ? j : m === 1 ? (j % ws < ws - 1 ? j + 1 : -1) : j + ws, r = Ej[q] / Ej[j];
        if (m && !(q >= 0 && q < M && at[3 * q] === at[3 * q] && r <= jr && r >= 1 / jr)) continue;
        const X = m ? (x + at[3 * q]) / 2 : x, Y = m ? (y + at[3 * q + 1]) / 2 : y, Dd = m ? (d + at[3 * q + 2]) / 2 : d;
        if (!(X >= 0 && Y >= 0 && X < w && Y < h)) continue;
        const b = ((Y / st) | 0) * ws + ((X / st) | 0);
        if (Dd < zb[b]) (zb[b] = Dd), (ox[b] = dx), (oy[b] = dy);
      }
    }
    for (const k of left) {
      const c0 = ((k % gw) * C) / st, r0 = (((k / gw) | 0) * C) / st;
      let landed = false, same = false;
      for (let r = r0; r < Math.min(hs, r0 + C / st) && !same; r++)
        for (let c = c0; c < Math.min(ws, c0 + C / st) && !same; c++) {
          const b = r * ws + c, key = k * 1e6 + (Math.round(ox[b] / 2) + 500) * 1000 + Math.round(oy[b] / 2) + 500;
          if (zb[b] === Infinity) continue;
          landed = true;
          if (zb[b] * jr < Ej[b]) same = true; // a nearer surface moved over it: its edge shows sides the render never saw
          else if (tried.has(key)) same = tried.get(key);
          else tried.set(key, (same = alike(k, ox[b], oy[b]) || (!!g.shift && alike(k, ox[b] - s.sx, oy[b] - s.sy))));
        }
      if (same || !landed) why.add(k);
    }
    by.set(String(i), why);
  }
  for (const [key, sign] of [["up", 1], ["down", -1]]) { // (a height error between two moves)
    const fam = [...moves.keys()].filter((i) => Math.sign(moves[i][2]) === sign);
    if (fam.length > 1) by.set(key, new Set(fam.flatMap((i) => [...by.get(String(i))])));
  }
  let best = null;
  for (const [key, why] of by) if (!best || why.size > by.get(best).size) best = key;
  const sure = !!best && by.get(best).size >= AVOID.aheadPosed * left.length, guess = sure ? best : by.has(keep) ? keep : null;
  return { still: new Set(guess ? left.filter((k) => !by.get(guess).has(k)) : left), guess, sure };
}

// A running box filter of radius r along one axis (step 1: rows, step w: columns) of a w x h image (as nav/changes.js).
function boxBlur(a, w, h, step, r, out = new Float32Array(a.length)) {
  const n = step === 1 ? w : h, lines = step === 1 ? h : w;
  for (let l = 0; l < lines; l++) {
    const base = step === 1 ? l * w : l;
    let t = 0;
    for (let i = 0; i < Math.min(n, r + 1); i++) t += a[base + i * step];
    for (let i = 0; i < n; i++) {
      out[base + i * step] = t / (Math.min(n - 1, i + r) - Math.max(0, i - r) + 1);
      if (i + r + 1 < n) t += a[base + (i + r + 1) * step];
      if (i - r >= 0) t -= a[base + (i - r) * step];
    }
  }
  return out;
}

// Convex hull of [[x, y]] (Andrew's monotone chain), counter-clockwise.
export function convexHull(pts) {
  const p = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (p.length < 3) return p;
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]), lo = [], hi = [];
  for (const q of p) {
    while (lo.length >= 2 && cross(lo.at(-2), lo.at(-1), q) <= 0) lo.pop();
    lo.push(q);
  }
  for (const q of p.reverse()) {
    while (hi.length >= 2 && cross(hi.at(-2), hi.at(-1), q) <= 0) hi.pop();
    hi.push(q);
  }
  return [...lo.slice(0, -1), ...hi.slice(0, -1)];
}

const centroid = (poly) => [0, 1].map((a) => poly.reduce((s, q) => s + q[a], 0) / poly.length);
const reach = ([x, y], poly) => Math.max(...poly.map((q) => Math.hypot(q[0] - x, q[1] - y)));

// Distance from (x, y) to a polygon (0 inside).
export function polyDist(x, y, poly) {
  let best = Infinity;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [ax, ay] = poly[j], [bx, by] = poly[i], dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy, t = L2 ? clamp(((x - ax) * dx + (y - ay) * dy) / L2, 0, 1) : 0;
    best = Math.min(best, Math.hypot(x - ax - t * dx, y - ay - t * dy));
  }
  return inside(x, y, poly) ? 0 : best;
}

const inBox = (b, u, v, pad = 0.15) => u > b.x - pad * b.w && u < b.x + b.w * (1 + pad) && v > b.y - pad * b.h && v < b.y + b.h * (1 + pad);

function inside(x, y, p) {
  let r = false;
  for (let i = 0, j = p.length - 1; i < p.length; j = i++)
    if (p[i][1] > y !== p[j][1] > y && x < ((p[j][0] - p[i][0]) * (y - p[i][1])) / (p[j][1] - p[i][1]) + p[i][0]) r = !r;
  return r;
}
