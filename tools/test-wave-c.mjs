// The seams between wave C's modules (docs/HOME-DRONE.md, "Wave C contracts") on the house fixture: a change the detector
// reports goes through the memory onto the map and the planner, and out again when resolved; confirmed changes in the 3D
// map are undone exactly (memory.apply with voxels.js's mark: an obstacle, a moved thing, a "gone"); the real drone's
// missions wait for vision localization to be set up (calibration, database, pad check); where_am_i matches the camera
// view to the scan when the position is unsure; the take-off column; video bars not looked for yet; calibration stops new
// tracking jobs before waiting for the worker; every setting the modules read has a default, every downloaded model is
// pinned and permissively licensed. Flight (wave C2): the "Ready to fly?" checklist and the real drone's refusals (on the
// ground and in the air), the localizer honest when vision fixes stop, the safety layer slowing and holding for them, the
// geofence's push never cancelled by a cap looking ahead, budgets at the speed the drone really flies, people remembered
// only after two frames (seated ones with a smaller zone), unseen suspected changes off the map in flight, rooms behind a
// doorway that looks closed not counted as seen, and which suspected changes return home may pass, rooms counted no more than the camera saw of them.
// Fix 1 (the flight reviews): return home in the air whatever the checklist says, refusals that change nothing, changes
// from an earlier flight kept after take-off, one person walking one finding (off the map: no room), the pad's room, a busy
// machine, the detector and live depth on the real drone, plain refusals, scan budgets, the localizer moved on the ground
// and with vision expected, the slow-video hold, the push fading with the fix's age, who counts as seated.
// Usage: cd tools && node test-wave-c.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { createHash } from "node:crypto";

const I = await import("../app/js/house/import.js");
const V = await import("../app/js/house/voxels.js");
const { plan } = await import("../app/js/house/planner.js");
const { HouseMemory } = await import("../app/js/memory/memory.js");
const { ChangeDetector } = await import("../app/js/nav/changes.js");
const { Avoid } = await import("../app/js/nav/avoid.js");
const { SplatLocalizer } = await import("../app/js/nav/splatloc.js");
const { MissionRunner, MISSION } = await import("../app/js/missions.js");
const { ReturnHome, HOME_TRY } = await import("../app/js/behaviors.js");
const { Localizer, LOC } = await import("../app/js/nav/localizer.js");
const { Safety, SAFETY } = await import("../app/js/safety.js");
const { AVOID, onDoorway, behind } = await import("../app/js/nav/avoid.js");
const { FlightController } = await import("../app/js/controller.js");
const { intrinsics, pinholeLens } = await import("../app/js/twin/lens.js");
const { roomCenter } = await import("../app/js/house/planner.js");
const { ToolBox } = await import("../app/js/tools.js");
const { pictureProblem, DEPTH_MODELS, LiveDepth } = await import("../app/js/vision/depth.js");
const { MODELS } = await import("../app/js/vision/models.js");
const { XFEAT } = await import("../app/js/nav/features.js");
const { DEFAULTS } = await import("../app/js/settings.js");
const { Emitter, DEG } = await import("../app/js/util.js");

const ROOT = path.join(import.meta.dirname, "..");
const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const fixture = await I.importCapture(fsSource(FIXTURE), { keepPoints: true });
const { house, map, points } = fixture;
const vox = await V.buildVoxels({ house, map, centres: points });
map.setVoxels(vox);
const hash = (...a) => createHash("sha256").update(Buffer.concat(a.map((x) => Buffer.from(x.buffer, x.byteOffset, x.byteLength)))).digest("hex").slice(0, 16);
const snapshot = () => hash(vox.lo, vox.flags);
let clock = 1.7e12;
const memoryFor = async (world = "real") => {
  const m = await HouseMemory.open(`${house.id}-${world}-${clock}`, { indexedDB: null, world, now: () => clock });
  const h = structuredClone(house);
  m.setHouse({ house: h, map, vox, save: null });
  return { m, h };
};
const settingsOf = (values) => ({ get: (k) => values[k], set: (k, v) => (values[k] = v), on: () => () => {} });
// A free spot at flying height in the living room's middle area, and one across the room for plans through it.
const spot = (x, y) => ({ x, y, z: map.floorAt(x, y) + 0.9 });

const tests = {
  "a change the detector reports goes through the memory onto the map (the planner goes round it) and off it again when resolved; the detector lets go"() {
    return (async () => {
      const { m } = await memoryFor("real"), cd = new ChangeDetector({ map, house, memory: m });
      const a = [2.0, 1.0], b = [4.0, 2.6], before = plan(map, a, b, { alt: 1.0 });
      assert.ok(before.ok, before.reason);
      const mid = before.path[Math.floor(before.path.length / 2)], at = { x: mid[0], y: mid[1] };
      const rec = m.addChange({ id: "change-test-1", kind: "obstacle", ...spot(at.x, at.y), size: 0.5, zMin: map.floorAt(at.x, at.y), zMax: map.floorAt(at.x, at.y) + 1.6 }, clock);
      cd.reported.push({ id: "change-test-1", memoryId: rec.id, x: at.x, y: at.y, z: 1, size: 0.5, near: true });
      assert.ok(map.temps.has(rec.id), "a suspected obstacle is a temporary obstacle on the map");
      const round = plan(map, a, b, { alt: 1.0 });
      const minD = Math.min(...round.path.map(([x, y]) => Math.hypot(x - at.x, y - at.y)));
      console.log(`      plan through the spot: ${before.length.toFixed(2)} m; with the change on it ${round.ok ? `${round.length.toFixed(2)} m, closest ${minD.toFixed(2)} m` : round.reason}`);
      assert.ok(!round.ok || minD >= 0.35, `the plan passes ${minD.toFixed(2)} m from the change`);
      m.resolveChange(rec.id, "dismissed", "a test");
      assert.ok(!map.temps.has(rec.id) && cd.reported.length === 0, "dismissed: off the map, and the detector forgot it");
      cd.dispose();
    })();
  },

  async "confirmed changes in the 3D map: an obstacle undone, a moved thing undone and a 'gone' undone leave the voxels as they were (or blocked: never freer)"() {
    const { m, h } = await memoryFor("real"), s0 = snapshot(), x = 2.6, y = 1.6, f = map.floorAt(x, y);
    const ob = m.addChange({ kind: "obstacle", x, y, z: f + 0.3, size: 0.4, zMin: f, zMax: f + 0.6 }, ++clock);
    m.resolveChange(ob.id, "confirmed");
    assert.ok(ob.applied.occupied > 0 && vox.state(x, y, f + 0.3) === V.OCCUPIED && h.keepouts.some((k) => k.change === ob.id), "confirmed: occupied and a keep-out");
    m.resolveChange(ob.id, "dismissed", "changed my mind");
    console.log(`      obstacle: ${ob.applied.freed} voxels put back; the map ${snapshot() === s0 ? "is as it was" : "differs"}`);
    assert.equal(snapshot(), s0, "an obstacle confirmed then undone leaves the voxels exactly as they were");
    // moved: the old place 0.25 m away overlaps the new one; confirmed, the overlap stays occupied; undone, as before
    const mv = m.addChange({ kind: "moved", x, y, z: f + 0.3, size: 0.4, zMin: f, zMax: f + 0.6, from: { x: x - 0.25, y, z: f + 0.3 } }, ++clock);
    m.resolveChange(mv.id, "confirmed");
    assert.equal(vox.state(x - 0.1, y, f + 0.3), V.OCCUPIED, "where the old and new places overlap the moved thing stays");
    m.resolveChange(mv.id, "dismissed", "changed my mind");
    const back = vox.state(x - 0.35, y, f + 0.3), wasThere = V.OCCUPIED;
    assert.equal(back, wasThere, "undone: the old place is filled again");
    // gone over a spot: undone, its box is blocked (nothing is freer than before)
    const st0 = vox.st.slice(), gone = m.addChange({ kind: "gone", x, y, z: f + 0.3, size: 0.4, zMin: f, zMax: f + 0.6 }, ++clock);
    m.resolveChange(gone.id, "confirmed");
    m.resolveChange(gone.id, "dismissed", "changed my mind");
    let freer = 0;
    for (let i = 0; i < vox.n; i++) freer += vox.st[i] === V.FREE && st0[i] !== V.FREE;
    console.log(`      moved: overlap kept; gone undone: ${gone.applied.occupied} voxels blocked, ${freer} freer than before`);
    assert.equal(freer, 0, "a 'gone' undone leaves no voxel freer than before it");
  },

  "the real drone: missions need the 3D map, the camera's picture and vision localization set up and running (the database read live), on the ground and after a manual take-off; the simulator (and its truth) as before; the checklist says what to do"() {
    const values = { mode: "real", startOnPad: true, aiVision: "ask", aiBudget: 0.5 }, settings = settingsOf(values);
    let flying = false, secondsLeft = 200;
    const ctl = Object.assign(new Emitter(), { blocker: () => "", isFlying: () => flying, safety: { battery: () => ({ vbat: 4.0, secondsLeft }), avoid: { brake: () => ({ measured: false }) } } });
    let q = { visionAge: Infinity, rate: 0 };
    const loc = { source: "fused", pose: () => ({ x: -0.64, y: 2.06, z: 0, yaw: 0, sigma: 0.05, status: "ok" }), flown: false, fixQuality: () => q }; // on the ground in the open (Room 2)
    const splat = { enabled: true, calibOk: false, db: null, pad: null, features: { info: { backend: "webgpu" } }, trust: () => (splat.pad?.ok ? "verified" : "advisory") };
    let frames = Infinity; // no video yet: the picture blocks; the machine's pace and the fixes wait for it (info)
    const perception = { get frameAge() { return frames; }, fps: 30, latest: {}, on: () => () => {} };
    const m = new MissionRunner({ ctl, map, house, localizer: loc, perception, settings, splat });
    const say = () => m.preflight(), at = (id) => m.preflightCheck().find((c) => c.id === id);
    splat.calibOk = true;
    const noVideo = { say: say(), picture: at("picture"), fix: at("fix"), pace: at("pace") };
    console.log(`      no video: "${noVideo.say}"; fix ${noVideo.fix.level} ("${noVideo.fix.text}"), pace ${noVideo.pace.level} ("${noVideo.pace.text}")`);
    assert.ok(/No video from the drone yet/.test(noVideo.say) && noVideo.picture.fix?.action === "goggles" && noVideo.fix.level === "info" && noVideo.pace.level === "info" && /video/.test(noVideo.fix.text + noVideo.pace.text), JSON.stringify(noVideo));
    splat.calibOk = false;
    frames = 30;
    const steps = [say()];
    splat.calibOk = true;
    steps.push(say());
    splat.db = {};
    steps.push(say());
    splat.pad = { ok: true, text: "matches" };
    steps.push(say());
    q = { visionAge: 200, rate: 4.2 };
    steps.push(say());
    console.log(`      ${steps.map((t) => t ?? "ok").map((t) => t.slice(0, 48)).join(" | ")}`);
    assert.ok(/Calibrate the camera/.test(steps[0]) && /database/.test(steps[1]) && /pad button/.test(steps[2]) && /No position fix from the camera/.test(steps[3]) && steps[4] === null, steps.join(" | "));
    splat.db = null;
    assert.ok(/database/.test(say()), "the database is read live, not from the pad check's snapshot");
    splat.db = {};
    flying = true; // after a manual take-off the checks still hold
    q = { visionAge: 3000, rate: 0.4 };
    const air = say();
    q = { visionAge: 150, rate: 4 };
    assert.ok(/No position fix/.test(air) && say() === null, `in the air: ${air}`);
    flying = false;
    const vox0 = map.vox;
    map.setVoxels(null);
    const noMap = say();
    map.setVoxels(vox0);
    splat.features.info.backend = "wasm";
    const wasm = say();
    splat.features.info.backend = "webgpu";
    splat.enabled = false;
    const off = say();
    splat.enabled = true;
    secondsLeft = 30;
    const low = say();
    secondsLeft = 200;
    console.log(`      no 3D map: "${noMap.slice(0, 60)}…"; WASM: "${wasm.slice(0, 60)}…"; vision off: "${off.slice(0, 60)}…"; battery: "${low.slice(0, 50)}…"`);
    assert.ok(/3D map/.test(noMap) && /WebGPU/.test(wasm) && /Vision localization is off/.test(off) && /battery is too low/.test(low));
    const list = m.preflightCheck(), levels = new Set(["block", "warn", "info"]), actions = new Set(["import-house", "build-3d", "radio", "pad", "crop-video", "vision-on", "calibrate-camera", "build-db", "models", "measure-braking", "charge", "ai-settings"]);
    assert.ok(list.every((c) => c.id && typeof c.ok === "boolean" && levels.has(c.level) && c.text && (c.ok ? c.level === "info" : c.level !== "info") && (!c.fix || actions.has(c.fix.action)) && ["before", "battery"].includes(c.when)), JSON.stringify(list.filter((c) => !levels.has(c.level))));
    assert.ok(list.some((c) => c.id === "braking" && c.level === "warn" && /0\.3 m\/s/.test(c.text)) && list.some((c) => c.id === "ai" && /\$0\.50/.test(c.text)), "braking and AI consent on the list");
    console.log(`      the checklist, ready: ${list.map((c) => `${c.id}:${c.level}`).join(" ")}`);
    values.mode = "sim";
    splat.enabled = false;
    map.setVoxels(null);
    assert.equal(say(), null, "the simulator needs none of it, not even the 3D map");
    map.setVoxels(vox0);
    values.mode = "real";
    loc.source = "truth";
    assert.equal(say(), null, "the simulator's truth rehearsing the real caps: no vision needed");
    m.dispose();
  },

  async "localizer: vision fixes stop for 4 s while the drone yaws at 1.2 rad/s and drifts 0.5 m/s unseen, the flow under-reading 40% at 5 fps: σ covers the error (within 2.5σ >= 95%), the status leaves ok before the error reaches 0.3 m; hovering still without a fix, lost after 5 s whatever σ says (the vision side relocalizes)"() {
    const real = globalThis.performance;
    let clock = 1000;
    Object.defineProperty(globalThis, "performance", { value: { now: () => clock }, configurable: true, writable: true });
    try {
      let seed = 1;
      const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647 - 0.5) * 2;
      const ctl = Object.assign(new Emitter(), { est: { heading: 0, vx: 0, vy: 0, vz: 0, flowQ: 0.8, rotating: false }, videoDelay: 100, perception: { fps: 5, latest: { flow: null } }, isFlying: () => true });
      const loc = new Localizer({ ctl, map: null, house: null });
      loc.reset({ x: 0, y: 0, yaw: 0, z: 1 });
      const T = { x: 0, y: 0, yaw: 0 }, hist = [], gap = [], dt = 1 / 30;
      let nextFix = 0, nextFrame = 0;
      for (let t = 0; t < 9; t += dt) {
        clock += 1000 * dt;
        const B = t >= 5, w = B ? 1.2 : 0, v = B ? [0, 0.5] : [0.2, 0];
        [T.x, T.y, T.yaw] = [T.x + v[0] * dt, T.y + v[1] * dt, T.yaw - w * dt];
        hist.push({ t: clock, ...T });
        ctl.est.heading += w * dt;
        ctl.est.rotating = w > 0.44;
        if (clock >= nextFrame) { // a 5 fps video: the flow reads 60% of the motion, nothing sideways while turning
          nextFrame = clock + 200;
          const c = Math.cos(T.yaw), sn = Math.sin(T.yaw);
          ctl.perception.latest.flow = { t: clock, quality: 0.8, span: 0.2 };
          [ctl.est.vx, ctl.est.vy] = [0.6 * (c * v[0] + sn * v[1]), ctl.est.rotating ? 0 : 0.6 * (sn * v[0] - c * v[1])];
        }
        ctl.emit("tick", { dt, now: clock });
        if (!B && clock >= nextFix) { // splat fixes at 4 Hz, 100 ms late
          nextFix = clock + 250;
          const h = hist.findLast((q) => q.t <= clock - 100) ?? hist[0];
          loc.fix({ x: h.x + 0.02 * rnd(), y: h.y + 0.02 * rnd(), z: 1, yaw: h.yaw + 0.01 * rnd(), sigma: 0.05, yawSigma: 0.03, t: clock - 100, source: "splat" });
        }
        if (B) {
          const p = loc.pose();
          gap.push({ t: t - 5, err: Math.hypot(p.x - T.x, p.y - T.y), sigma: p.sigma, status: p.status });
        }
      }
      const within = gap.filter((g) => g.err <= 2.5 * g.sigma).length / gap.length, left = gap.find((g) => g.status !== "ok"), at = (s) => gap.find((g) => g.t >= s);
      console.log(`      within 2.5σ ${Math.round(100 * within)}% of the gap; status left ok at ${left.t.toFixed(2)} s (error ${left.err.toFixed(2)} m, σ ${left.sigma.toFixed(2)}); ${[0.5, 1, 2].map((s) => `${s} s: error ${at(s).err.toFixed(2)} σ ${at(s).sigma.toFixed(2)} ${at(s).status}`).join("; ")}; fix quality ${JSON.stringify((({ vision, nis }) => ({ vision, nis }))(loc.fixQuality()))}`);
      assert.ok(within >= 0.95, `within 2.5σ only ${Math.round(100 * within)}%`);
      assert.ok(left && left.err <= 0.3 && gap.filter((g) => g.err > 0.3 && g.status === "ok").length === 0, `left ok at ${left?.err.toFixed(2)} m`); // C1: ok for 4.1 s, 1.2 m off
      assert.ok(gap.at(-1).status === "lost" && loc.fixQuality().vision, "lost by the end of 4 s without a fix; vision was the source");
      // hovering still with no vision fix: the flow says it's still, so σ grows slowly, but the silence itself counts
      const still = new Localizer({ ctl, map: null, house: null }), seen = {};
      still.reset({ x: 0, y: 0, yaw: 0, z: 1 });
      Object.assign(ctl.est, { vx: 0, vy: 0, rotating: false });
      ctl.perception.fps = 30;
      for (let t = 0; t < 8; t += dt) {
        clock += 1000 * dt;
        ctl.perception.latest.flow = { t: clock, quality: 0.8, span: 0.1 };
        ctl.emit("tick", { dt, now: clock });
        if (t < 2 && Math.round(t * 30) % 8 === 0) still.fix({ x: 0, y: 0, z: 1, yaw: 0, sigma: 0.05, yawSigma: 0.03, t: clock - 50, source: "splat" });
        const p = still.pose();
        for (const at of [1, 3.6, 6.9, 7.9]) if (t >= at && !seen[at]) seen[at] = `${p.status} σ ${p.sigma.toFixed(2)}`;
      }
      console.log(`      hovering still, the fixes stopping at 2 s: ${Object.entries(seen).map(([t, v]) => `${t} s ${v}`).join("; ")}`);
      assert.ok(/^ok/.test(seen[1]) && /^degraded/.test(seen[3.6]) && /^lost/.test(seen[7.9]) && parseFloat(seen[7.9].split("σ ")[1]) < LOC.lost, JSON.stringify(seen));
      still.dispose();
    } finally {
      Object.defineProperty(globalThis, "performance", { value: real, configurable: true, writable: true });
    }
  },

  "safety: with vision the position source, turns are slowed, sparse fixes cap the speed at 0.15 m/s, none for 2.5 s hold its position (turned back to the last fix's heading, or with none to turn back to, toward the open side with a relocalization asked for); the geofence's push is never cancelled by a cap looking ahead, and winds up only holding still"() {
    const settings = settingsOf({ mode: "sim", autonomy: "full", hfov: 127, uptilt: 20, maxYawRate: 120 }), open = roomCenter(map, "r1", { alt: 1.0 });
    let q = { visionAge: 200, rate: 4, vision: true }, P = { x: open[0], y: open[1], z: map.floorAt(...open) + 1, yaw: 0, sigma: 0.05, zSigma: 0.02, status: "ok" };
    const ctl = Object.assign(new Emitter(), { est: { heading: 0.3, ttc: Infinity }, videoDelay: 100, autonomy: "full", airborne: true, heightSlowdown: 1, settings, isFlying: () => true, perception: { frameAge: Infinity, latest: { t: 0, detections: [] }, find: () => [] } });
    let asked = 0;
    const loc = Object.assign(new Emitter(), { ctl, pose: () => P, poseAt: () => P, velocity: () => [0, 0], visionActive: () => q.vision, fixQuality: () => q, askFix: () => asked++ });
    const safety = new Safety({ map, localizer: loc, settings, house });
    const f = (sp) => safety.filter(sp, { ctl, now: 1e6, dt: 0.033 });
    const a = f({ vx: 0.5, vy: 0, yawRate: 1 });
    q = { visionAge: 1300, rate: 1.2, vision: true };
    const b = f({ vx: 0.5, vy: 0, heading: 1 });
    q = { visionAge: 3000, rate: 0.4, vision: true };
    const c = f({ vx: 0.5, vy: 0, heading: 1 }), why = safety.reason;
    q = { visionAge: Infinity, rate: 0, vision: false };
    const d = f({ vx: 0.5, vy: 0, heading: 1 });
    const turn = Math.abs(c.heading - 0.3) / DEG;
    console.log(`      fresh: ${a.vx.toFixed(2)} m/s, turns ≤ ${a.maxYawRate} rad/s; sparse: ${Math.hypot(b.vx, b.vy).toFixed(2)} m/s ≤ ${b.maxYawRate} rad/s; none for 3 s: ${c.vx === undefined ? "holding" : c.vx} ("${why}"), turning ${turn.toFixed(0)}° to the open side (no last fix to turn back to), a relocalization asked ${asked} time(s); no vision: ${d.vx.toFixed(2)} m/s`);
    assert.ok(a.vx === 0.5 && a.maxYawRate === SAFETY.yawVision, "fresh fixes: full speed, turns slowed");
    assert.ok(Math.abs(Math.hypot(b.vx, b.vy) - SAFETY.sparse) < 1e-9 && b.maxYawRate === SAFETY.yawStale, "sparse: 0.15 m/s, slower turns");
    assert.ok(c.vx === undefined && c.vy === undefined && turn >= 59 && turn <= 121 && asked === 1 && /no camera position fix/.test(why), "no fix for 3 s: hold position, look elsewhere, ask for a relocalization");
    assert.ok(d.vx === 0.5 && d.maxYawRate === undefined, "no vision: as before");
    // a turn back to make: asked once facing there 1 s with no fix; at once when σ says the estimate has drifted
    const hold = (sigma) => {
      const s2 = new Safety({ map, localizer: Object.assign(loc, { vision: { yaw: 1.2 }, headingFor: (yaw) => yaw }), settings, house }), n0 = asked;
      P = { ...P, sigma };
      q = { visionAge: 3000, rate: 0.4, vision: true };
      s2.filter({ vx: 0.3, heading: 0.3 }, { ctl, now: 2e6, dt: 0.033 });
      return asked - n0;
    };
    const sure = hold(0.05), unsure = hold(0.2);
    [P, q] = [{ ...P, sigma: 0.05 }, { visionAge: Infinity, rate: 0, vision: false }];
    delete loc.vision, delete loc.headingFor;
    console.log(`      a turn back to make: asked ${sure} time(s) at its start with σ 0.05 m, ${unsure} with σ 0.2 m`);
    assert.ok(sure === 0 && unsure === 1, `${sure} ${unsure}`);
    // a hover 5-12 cm from a wall (inside the buffer) while every cap looking ahead says stop: the push still comes
    const wall = (() => { for (let x = -1.8; x < -1.0; x += 0.01) { const z = map.floorAt(x, 2.2) + 1, c = map.clearance(x, 2.2, z); if (c > 0.05 && c < 0.12) return { x, z }; } })();
    P = { ...P, x: wall.x, y: 2.2, z: wall.z };
    safety.avoid.limit = (sp) => ({ ...sp, ...(sp.vx !== undefined && { vx: 0, vy: 0 }) }); // a way-ahead stop, say
    const e = f({ heading: 0.3 }), push = Math.hypot(e.vx ?? 0, e.vy ?? 0);
    q = { visionAge: 700, rate: 4, vision: true }; // a fix 0.7 s old (a turn the flow can't follow): a gentler push
    const e2 = f({ heading: 0.3 }), push2 = Math.hypot(e2.vx ?? 0, e2.vy ?? 0);
    q = { visionAge: Infinity, rate: 0, vision: false };
    console.log(`      ${map.clearance(wall.x, 2.2, wall.z).toFixed(2)} m from the wall at (${wall.x.toFixed(2)}, 2.20), every forward cap at 0: pushed off at ${push.toFixed(2)} m/s ("${safety.reason}"); with the fix 0.7 s old ${push2.toFixed(2)} m/s`);
    assert.ok(push > 0.1, `the push: ${push}`);
    assert.ok(push2 > 0.1 && push2 <= SAFETY.pushStale + 1e-9, `the push with an old fix: ${push2}`);
    // the push alone carrying the drone along the wall (still inside the buffer) 0.6 m from where it began: it stops there
    const ys = [0.6, 0.7, 0.8, -0.6, -0.7, -0.8].map((dy) => 2.2 + dy).find((y) => { const c = map.clearance(wall.x, y, wall.z); return c > 0.03 && c < 0.2; });
    f({ heading: 0.3 });
    P = { ...P, y: ys };
    const slid = f({ heading: 0.3 }), why2 = safety.reason, gone = Math.hypot(slid.vx ?? 0, slid.vy ?? 0);
    P = { ...P, y: 2.2 };
    const back = Math.hypot(f({ heading: 0.3 }).vx ?? 0, 0);
    console.log(`      pushed along the wall to y ${ys.toFixed(2)} (still ${map.clearance(wall.x, ys, wall.z).toFixed(2)} m from it): ${gone.toFixed(2)} m/s ("${why2}"); back where it began: ${back.toFixed(2)} m/s`);
    assert.ok(gone < 0.01 && /too close/.test(why2) && back > 0.1, `${gone} ${back}`);
    // lost (the mission stops, it holds), found again within holdLost: it flies home rather than hover where it stopped
    ctl.safety = safety;
    safety.watch({ ...P, status: "lost" });
    const held = safety.takeover?.kind;
    safety.watch({ ...P, status: "ok" });
    console.log(`      lost: ${held}; found again: ${safety.takeover?.kind} (${safety.takeover?.label})`);
    assert.ok(held === "lost" && safety.takeover?.kind === "found" && safety.takeover.name === "home", `${held} ${safety.takeover?.kind}`);
    safety.takeover = null;
    // flying along it inside the buffer (a path there): the push doesn't wind up off the way; holding still it does
    safety.avoid.limit = (sp) => sp;
    const run = (sp, n) => {
      safety.pushed = [0, 0];
      for (let i = 0; i < n; i++) safety.filter(sp, { ctl, now: 1e6 + 33 * i, dt: 0.033 });
      return Math.hypot(...safety.pushed);
    };
    const along = run({ vx: 0, vy: 0.3, heading: 0.3 }, 90), still = run({ heading: 0.3 }, 90);
    console.log(`      3 s inside the buffer: the push's integral ${along.toFixed(2)} m/s flying along the wall, ${still.toFixed(2)} m/s holding still`);
    assert.ok(along < 0.02 && still > 0.1 && still <= SAFETY.pushMax + 1e-9, `${along} ${still}`);
  },

  "budgets at the speed the drone really flies: the real drone without live depth and with its braking unmeasured budgets at most the blind cap (0.22 m/s, a share of it); safety's trip home, the runner's legs and return home's patience use it"() {
    const settings = settingsOf({ mode: "real", avoid: true });
    const P = { x: 2.6, y: 4.8, z: map.floorAt(2.6, 4.8) + 1, yaw: 0, sigma: 0.05, zSigma: 0.02, status: "ok" };
    const ctl = Object.assign(new Emitter(), { est: { heading: 0, ttc: Infinity }, videoDelay: 100, autonomy: "full", tel: { vbat: 3.9 }, settings, isFlying: () => true, blocker: () => "", perception: null });
    const loc = Object.assign(new Emitter(), { ctl, pose: () => P, poseAt: () => P, velocity: () => [0, 0], fixQuality: () => null });
    const safety = new Safety({ map, localizer: loc, settings, house });
    ctl.safety = safety;
    const v = safety.avoid.cruise(), trip = safety.timeHome(P, 1e6), r = plan(map, [P.x, P.y], [house.home.x, house.home.y], { sigma: 0.15, climb: false });
    console.log(`      cruise ${v.v.toFixed(3)} m/s ("${v.why}"); the trip home ${r.length.toFixed(1)} m: ${trip.toFixed(0)} s (at 0.35 m/s it was ${(r.length / 0.35 + 8).toFixed(0)} s)`);
    assert.ok(v.v <= AVOID.blind && /flying slowly/.test(v.why), `${v.v}`);
    assert.ok(Math.abs(trip - (r.length / v.v + SAFETY.landTime)) < 0.5, "timeHome at cruise()");
    const m = new MissionRunner({ ctl, map, house, localizer: loc, perception: null, settings });
    let left = 0;
    m.battery = () => ({ vbat: 3.9, secondsLeft: left });
    const leg = 3.0, home = r.length / v.v + MISSION.landTime, needSlow = leg / v.v + MISSION.budget * home + MISSION.reserve, needOld = leg / 0.3 + MISSION.budget * (r.length / 0.3 + MISSION.landTime) + MISSION.reserve;
    left = (needSlow + needOld) / 2; // enough at the old 0.3 m/s, not at the speed it really flies
    const refused = !m.afford([P.x, P.y], leg);
    left = needSlow + 1;
    assert.ok(refused && m.afford([P.x, P.y], leg), "a leg that fits only at 0.3 m/s is refused up front");
    const rh = new ReturnHome({ localizer: loc, map, home: house.home, battery: () => left });
    rh.tries = 1;
    left = HOME_TRY.budget * ((1.5 * Math.hypot(house.home.x - P.x, house.home.y - P.y)) / 0.3 + HOME_TRY.landTime) + HOME_TRY.reserve + 1;
    console.log(`      a ${leg} m leg needs ${needSlow.toFixed(0)} s now (${needOld.toFixed(0)} s at 0.3 m/s); return home with ${left.toFixed(0)} s left: ${rh.outOfTime(ctl, 1e6) ? "lands nearby" : "keeps trying"}`);
    assert.ok(rh.outOfTime(ctl, 1e6), "return home's patience runs out at the real speed");
    m.dispose();
  },

  "people: one frame of a person isn't remembered on the map (a false detection never closes a doorway); two in a row are; someone seated (or lying) still gets the smaller zone"() {
    const values = { mode: "sim", autonomy: "full", hfov: 127, uptilt: 20, videoDelay: 0 }, settings = { get: (k) => values[k], all: () => values, on: () => () => {} };
    const per = Object.assign(new Emitter(), { frameAge: 0, latest: { t: 1, detections: [], width: 320, height: 240 }, find: (l) => per.latest.detections.filter((d) => d.label === l) });
    const ctl = new FlightController({ settings, perception: per });
    ctl.isFlying = () => true;
    const P = { x: 2.0, y: 2.2, z: map.floorAt(2.0, 2.2) + 1, yaw: 0, sigma: 0.05, zSigma: 0.02, status: "ok" };
    const loc = Object.assign(new Emitter(), { ctl, pose: () => P, poseAt: () => P, velocity: () => [0, 0] });
    const safety = new Safety({ map, localizer: loc, settings, house });
    const frame = (t, dets) => (Object.assign(per.latest, { t, detections: dets }), safety.filter({ vx: 0.1, vy: 0 }, { ctl, now: t, dt: 0.033 }));
    const person = (box, trackId = 7) => ({ label: "person", score: 0.9, trackId, box });
    const zones = () => [...safety.avoid.temps.values()].filter((t) => t.kind === "person");
    frame(1000, [person({ x: 0.45, y: 0.2, w: 0.06, h: 0.45 })]);
    const one = zones().length;
    frame(1033, []);
    frame(1500, [person({ x: 0.45, y: 0.2, w: 0.06, h: 0.45 }, 8)]);
    frame(1533, [person({ x: 0.45, y: 0.2, w: 0.06, h: 0.45 }, 8)]);
    const two = zones(), standR = two[0]?.r;
    for (const z of zones()) safety.avoid.removeTemp(z.id);
    frame(3000, [person({ x: 0.42, y: 0.35, w: 0.16, h: 0.3 }, 9)]); // seated: about 1.4 times as tall as wide
    frame(3033, [person({ x: 0.42, y: 0.35, w: 0.16, h: 0.3 }, 9)]);
    const sat = zones()[0];
    console.log(`      one frame: ${one} zones; two: ${two.length} (r ${standR} m); seated: r ${sat?.r} m (seated ${sat?.seated})`);
    assert.ok(one === 0 && two.length === 1 && standR === SAFETY.standoff, "two frames in a row make a zone");
    assert.ok(sat && sat.r === SAFETY.seated && sat.seated, "a seated person's zone is the smaller one");
    for (const z of zones()) safety.avoid.removeTemp(z.id);
  },

  "the way ahead: a wedge needs depth support on two frames in a row (a one-frame spike makes none), goes once the window sees its bearings as the scan has them, and isn't made where a door leaf already explains the difference"() {
    const c = roomCenter(map, "r1", { alt: 1.0 }), P = { x: c[0], y: c[1], z: map.floorAt(...c) + 1, yaw: 0, sigma: 0.05, zSigma: 0.02, status: "ok" };
    const av = new Avoid({ map, localizer: { ctl: { videoDelay: 0, est: { ttc: Infinity } }, pose: () => P, velocity: () => [0, 0] }, settings: settingsOf({ mode: "sim", avoid: true }) });
    const w = 64, h = 48, C = 4, gw = 16, gh = 12, lens = { model: "pinhole", hfovDeg: 100, uptiltDeg: 0 }, K = intrinsics(lens, w, h);
    const frame = (t, looks, near) => { // a synthetic differences() grid: the corridor window unlike the scan (looks), nearer than it (near)
      const L = new Uint8Array(gw * gh), N = new Uint8Array(gw * gh);
      for (let k = 0; k < gw * gh; k++) if (Math.abs(((k % gw) + 0.5) * C - K.cx) <= 0.2 * w && Math.abs((((k / gw) | 0) + 0.5) * C - K.cy) <= 0.25 * h) [L[k], N[k]] = [looks, near];
      const f = { t, pose: P, lens, width: w, height: h, expected: new Float32Array(w * h).fill(3), rgb: {}, expectedRgb: {} };
      f.grid = { C, gw, gh, n: new Uint16Array(gw * gh).fill(C * C), looks: L, near: N, off: 0, expected: f.expected };
      av.wayAhead(f, K, t);
    };
    const wedges = () => [...av.temps.values()].filter((t) => t.source === "way ahead").length;
    frame(1000, 1, 1);
    frame(1200, 1, 0); // depth backed it on one frame only
    const spike = wedges();
    frame(1400, 1, 1);
    frame(1600, 1, 1);
    const real = wedges();
    frame(1800, 0, 0);
    frame(2000, 0, 0); // seen as the scan has it, twice
    const cleared = wedges();
    av.addTemp({ id: "leaf", kind: "obstacle", source: "door leaf", x: P.x + 1, y: P.y, r: 0.3, polygon: [[P.x + 0.95, P.y - 0.6], [P.x + 1.05, P.y - 0.6], [P.x + 1.05, P.y + 0.6], [P.x + 0.95, P.y + 0.6]], until: Infinity }, 2000);
    frame(2200, 1, 1);
    frame(2400, 1, 1);
    const explained = wedges();
    av.removeTemp("leaf");
    for (const id of [...av.temps.keys()]) av.removeTemp(id);
    console.log(`      a one-frame depth spike: ${spike} wedges; two frames: ${real}; then seen as the scan twice: ${cleared}; with a door leaf on its bearing: ${explained}`);
    assert.ok(spike === 0 && real === 1 && cleared === 0 && explained === 0, [spike, real, cleared, explained].join(" "));
  },

  "door leaves at a corner: a live point on a closed doorway's plane goes to that doorway, not to an open one in line with it or across its end (Room 2's doorway, in line with Room 3's and ending at the archway, was taken for closed)"() {
    const o = [1.4, 1.53, 1.0], id = (k) => (k == null ? "none" : map.doors[k].id), at = (q) => {
      const v = [q[0] - o[0], q[1] - o[1], q[2] - o[2]], d = Math.hypot(...v);
      return id(onDoorway(map, o, v.map((c) => c / d), d, 0.1)?.k);
    };
    const r3 = at([0.48, 2.95, 1.0]), arch = at([0.42, 2.875, 1.0]), open = at([0.495, 2.2, 1.0]);
    console.log(`      on Room 3's leaf just past Room 2's doorway: ${r3}; on the archway's leaf by its end: ${arch}; in Room 2's doorway: ${open}`);
    assert.ok(r3 === "r3:w3-o1" && arch === "r2:w1-o1+r3:w2-o1" && open === "r2:w4-o1", `${r3} ${arch} ${open}`);
    // from where the browser made a leaf of Room 2's open doorway (1.21, 1.52): the closed archway is just behind the part
    // of it seen closed, so that view can't tell; head-on from the living room nothing is behind it
    const r2 = map.doors.find((d) => d.id === "r2:w4-o1"), from = (x, y, span) => behind(map, [x, y, 1.2], r2, AVOID.depthErr, span)?.id ?? "nothing";
    const there = from(1.21, 1.52, [0.4, 1]), whole = from(1.21, 1.52, [0, 1]), head = from(2.0, 2.25, [0, 1]);
    console.log(`      behind Room 2's doorway from (1.21, 1.52): ${there} (its upper part), ${whole} (all of it); from the living room head-on: ${head}`);
    assert.ok(there === "r2:w1-o1+r3:w2-o1" && whole === "nothing" && head === "nothing", `${there} ${whole} ${head}`);
  },

  async "suspected changes: one not seen again for a minute comes off the map in flight (back on when seen again, and after landing); return home may pass only those Claude called no change, sure enough; a room behind a doorway that looks closed isn't seen from outside"() {
    const { m: mem } = await memoryFor("real");
    let flying = true, P = { x: -1.2, y: 2.6, z: 1.2, yaw: Math.PI / 2, sigma: 0.05, status: "ok", room: "r2" };
    const ctl = Object.assign(new Emitter(), { isFlying: () => flying, blocker: () => "", autonomy: "full" });
    const loc = { pose: () => P, flown: true };
    const m = new MissionRunner({ ctl, map, house, localizer: loc, perception: null, settings: settingsOf({ mode: "sim" }), memory: mem });
    const from = [3.5, 1.4], way = plan(map, from, [house.home.x, house.home.y], { alt: 1.0, sigma: 0.17 }).path, mid = way[Math.floor(way.length * 0.3)];
    const [x, y] = mid, f = map.floorAt(x, y), rec = mem.addChange({ kind: "obstacle", x, y, z: f + 0.4, size: 0.4, zMin: f, zMax: f + 0.8 }, clock);
    const t0 = 5e5;
    m.liftStale(t0);
    const before = map.temps.has(rec.id);
    clock += 61000;
    m.liftStale(t0 + 2000);
    const lifted = !map.temps.has(rec.id);
    clock += 1000;
    mem.seen(rec.id, clock);
    m.liftStale(t0 + 4000);
    const back = map.temps.has(rec.id);
    clock += 61000;
    m.liftStale(t0 + 6000);
    flying = false;
    m.liftStale(t0 + 8000);
    console.log(`      on the map: ${before}; a minute unseen: ${lifted ? "off" : "on"}; seen again: ${back ? "on" : "off"}; landed: ${map.temps.has(rec.id) ? "on" : "off"}`);
    assert.ok(before && lifted && back && map.temps.has(rec.id));
    // passing: the way home from the living room blocked by it; only a no-change at >= 0.8 is passable
    P = { x: from[0], y: from[1], z: map.floorAt(...from) + 1, yaw: Math.PI, sigma: 0.05, status: "ok", room: "r1" };
    const blocks = m.blocker([house.home.x, house.home.y]);
    const passable = (claude) => (mem.annotateChange(rec.id, { claude }), m.passableChanges([house.home.x, house.home.y]));
    const sure = passable({ status: "answered", verdict: "no-change", confidence: 0.85 }), unsure = passable({ status: "answered", verdict: "no-change", confidence: 0.7 }), real = passable({ status: "answered", verdict: "real-change", confidence: 0.9 });
    console.log(`      the way home: ${blocks ? `"${blocks.text}" at (${blocks.at.map((v) => v.toFixed(1))})${blocks.relook ? ", worth a closer look" : ""}` : "clear"}; passable: no change 85% [${sure}], 70% [${unsure}], real 90% [${real}]`);
    mem.resolveChange(rec.id, "dismissed", "a test");
    // a room behind a doorway that looks closed: nothing of it seen from outside
    const arch = map.doors.find((d) => d.id === "r2:w1-o1+r3:w2-o1"), spot = { x: -0.7, y: 2.5 };
    const open = m.roomView("r3", spot);
    map.addTemp({ id: "leaf-test", kind: "obstacle", source: "door leaf", door: arch.id, polygon: [[arch.a[0], arch.a[1] - 0.1], [arch.b[0], arch.b[1] - 0.1], [arch.b[0], arch.b[1] + 0.1], [arch.a[0], arch.a[1] + 0.1]], until: Infinity });
    const shut = m.roomView("r3", spot);
    map.removeTemp("leaf-test");
    console.log(`      Room 3 from Room 2 by the archway: open ${Math.round(100 * open.share)}% seen; closed ${Math.round(100 * shut.share)}% (shut ${shut.shut})`);
    assert.ok(blocks?.relook && blocks.at && sure.includes(rec.id) && !unsure.length && !real.length, "only a sure no-change is passable");
    assert.ok(!open.inside && open.share > 0.2 && shut.share === 0 && shut.shut, "a doorway that looks closed hides the room");
    // and no more of it than the camera saw since the look began (vox.markSeen): looking away from the archway, next to none
    const since = Date.now() - 1, ac = [(arch.a[0] + arch.b[0]) / 2, (arch.a[1] + arch.b[1]) / 2], at = Math.atan2(ac[1] - spot.y, ac[0] - spot.x), pct = (v) => (v == null ? "none" : `${Math.round(100 * v)}%`);
    const look = (yaw) => vox.markSeen({ x: spot.x, y: spot.y, z: map.floorAt(spot.x, spot.y) + 1.0, yaw, pitch: 0, roll: 0, sigma: 0.05, status: "ok" }, pinholeLens(90), Date.now(), { width: 160, height: 120, quality: { age: 50, luma: 120 } });
    const none = m.roomView("r3", spot, since);
    look(at + Math.PI);
    const away = m.roomView("r3", spot, since);
    look(at);
    const toward = m.roomView("r3", spot, since);
    vox.seen = null; // the fixture's 3D map as it was
    console.log(`      the camera's marks: none ${pct(none.cam)} (${pct(none.share)} counted); looking away ${pct(away.cam)} (${pct(away.share)}); at the archway ${pct(toward.cam)} (${pct(toward.share)})`);
    assert.ok(none.cam === null && none.share === open.share, "without marks: the plan's estimate");
    assert.ok(away.cam < 0.05 && away.share <= away.cam && toward.cam > away.cam + 0.1 && toward.share <= Math.min(open.share, toward.cam), "capped by what the camera saw");
    m.dispose();
  },

  async "where_am_i: with the position unsure it matches the camera view to the scan first, and says when the view disagrees; with a good position it only reports the fix rate"() {
    let pose = { x: 1, y: 1, z: 1, yaw: 0, sigma: 0.4, status: "degraded", source: "fused", room: house.rooms[0].id }, asked = 0, answer = { ok: true, checked: true };
    const loc = { pose: () => pose, fixQuality: () => ({ age: 400, rate: 4.2, used: 30 }), flown: true };
    const ctl = Object.assign(new Emitter(), { isFlying: () => true, blocker: () => "" });
    const missions = new MissionRunner({ ctl, map, house, localizer: loc, perception: null, settings: settingsOf({ mode: "real" }) });
    missions.battery = () => null;
    const splat = { enabled: true, locateNow: async () => (asked++, answer) };
    const tb = new ToolBox({ ctl, perception: null, settings: settingsOf({}), speak: () => {} });
    tb.setHouse({ house, map, missions, localizer: loc, splat });
    const a = (await tb.call("where_am_i", {})).text;
    answer = { ok: false, disagrees: true, reason: "gate" };
    const b = (await tb.call("where_am_i", {})).text;
    pose = { ...pose, sigma: 0.05, status: "ok" };
    const c = (await tb.call("where_am_i", {})).text;
    console.log(`      unsure: "${a.slice(0, 90)}…"\n      disagrees: "${b.slice(0, 60)}…"\n      sure: "${c.slice(0, 60)}…"`);
    assert.ok(/agrees with my position/.test(a) && /4\.2\/s/.test(a) && /disagrees/.test(b) && asked === 2 && /^Vision fixes: 4\.2\/s/.test(c), [a, b, c].join(" | "));
    missions.dispose();
  },

  async "return home plans with the position's σ now, not the σ when it set out (a bad moment's σ left no way through a doorway for good), and never widens its plan off the pad"() {
    const homeRoom = map.roomAt(house.home.x, house.home.y)?.id, other = house.rooms.find((r) => r.id !== homeRoom && roomCenter(map, r.id, { alt: 1.0 }));
    const c = roomCenter(map, other.id, { alt: 1.0 }), loc = { pose: () => ({ x: c[0], y: c[1], z: map.floorAt(...c) + 1, yaw: 0, sigma: 0.05, status: "ok" }) };
    let s = 0.6;
    const said = [], rh = new ReturnHome({ localizer: loc, map, home: house.home, sigma: () => s, say: (t) => said.push(t) });
    const ctl = { autonomy: "full", est: { heading: 0 }, run: () => {} };
    rh.start(ctl);
    const first = rh.phase;
    s = 0.17;
    rh.go(ctl, performance.now());
    console.log(`      σ 0.6 at the start: ${first} ("${said[0]}"); σ 0.17 on the next try: ${rh.phase}`);
    assert.ok(first === "wait" && /σ 0\.6 m|no doorway|no path/.test(said[0]) && rh.phase === "fly", `${first} ${rh.phase} ${said.join(" | ")}`);
    assert.ok(!/\d\.\d{3}/.test(said[0]), "the planner's σ is said rounded");
    // many tries in, the widest plan (0.45 m) ends off the pad by its wall: it plans with the σ instead, never "something is on the pad"
    const near = roomCenter(map, homeRoom, { alt: 1.0 }), wide = plan(map, near, [house.home.x, house.home.y], { alt: 1.0, sigma: HOME_TRY.sigmaMax, climb: true });
    Object.assign(c, near);
    rh.tries = 20;
    said.length = 0;
    rh.go(ctl, performance.now());
    const end = rh.child?.path?.at(-1) ?? rh.child?.pts?.at(-1);
    console.log(`      the widest plan ${wide.ok ? `ends ${Math.hypot(wide.to[0] - house.home.x, wide.to[1] - house.home.y).toFixed(2)} m off the pad` : "fails"}; after 20 tries: ${rh.phase}${rh.phase === "fly" && end ? `, to ${Math.hypot(end[0] - house.home.x, end[1] - house.home.y).toFixed(2)} m from the pad` : ""} ${said.join(" | ")}`);
    assert.ok(!wide.ok || Math.hypot(wide.to[0] - house.home.x, wide.to[1] - house.home.y) > 0.25, "the widest plan can't reach the pad (else this proves nothing)");
    assert.ok(rh.phase === "fly" && !said.some((t) => /home pad/.test(t)), said.join(" | "));
    // the mission runner's return home follows the localizer's σ as it changes
    let sg = 0.2;
    const ml = { pose: () => ({ ...loc.pose(), sigma: sg }) }, mctl = Object.assign(new Emitter(), { isFlying: () => true, blocker: () => "" });
    const m = new MissionRunner({ ctl: mctl, map, house, localizer: ml, perception: null, settings: settingsOf({ mode: "sim" }) });
    let flown = null;
    m.fly = async (b) => (b instanceof ReturnHome && (flown = b), { ok: true });
    await m.returnHome();
    const a = flown.sigma;
    sg = 0.03;
    console.log(`      the mission's return home: planning σ ${a.toFixed(2)} with the pose σ 0.20, ${flown.sigma.toFixed(2)} once it is 0.03`);
    assert.ok(a > flown.sigma + 0.1, "the runner's ReturnHome re-reads the planning σ");
    m.dispose();
  },

  "the take-off column: a person's zone next to the pad doesn't block it (the take-off waits for them); something new over the pad is named"() {
    const settings = settingsOf({ mode: "sim" }), loc = { pose: () => ({ x: house.home.x, y: house.home.y, z: 0, sigma: 0.05, status: "ok" }), ctl: { videoDelay: 0 } };
    const avoid = new Avoid({ map, localizer: loc, settings }), { x, y } = house.home, f = map.floorAt(x, y);
    const clear = avoid.columnClear(x, y, 1.0);
    map.addTemp({ id: "p", kind: "person", x: x + 1.2, y, r: 1.5, until: Infinity });
    const person = avoid.columnClear(x, y, 1.0);
    map.removeTemp("p");
    map.addTemp({ id: "box", kind: "change", change: "obstacle", x, y, r: 0.3, zMin: f + 0.6, zMax: f + 0.8, until: Infinity });
    const box = avoid.columnClear(x, y, 1.0);
    map.removeTemp("box");
    map.addTemp({ id: "beside", kind: "change", change: "obstacle", x: x + 0.5, y, r: 0.6, zMin: f, zMax: f + 0.8, until: Infinity }); // a confirmed box's keep-out reaching the pad
    const beside = avoid.columnClear(x, y, 1.0);
    map.removeTemp("beside");
    console.log(`      clear: "${clear}"; a person 1.2 m away: "${person}"; a box over the pad: "${box}"; a keep-out beside it: "${beside}"`);
    assert.ok(clear === "" && person === "" && /doesn't have is above me/.test(box), [clear, person, box].join(" | "));
    assert.match(beside, /doesn't have is right beside me \(a change in History\)/);
  },

  "video bars: a 16:9 stream whose bars haven't been looked for yet is not passed as a 16:9 crop of the picture; without a source to look at, as before"() {
    const src = { kind: "goggles", element: () => ({}), ready: () => true, region: () => ({ sx: 0, sy: 0, sw: 1920, sh: 1080 }) };
    const waiting = pictureProblem({ source: src, latest: { width: 1920, height: 1080 } }), none = pictureProblem({ latest: { width: 1920, height: 1080 } });
    const four = pictureProblem({ source: { ...src, region: () => ({ sx: 0, sy: 0, sw: 1440, sh: 1080 }) }, latest: {} });
    console.log(`      1920x1080 not looked at yet: "${waiting}"; no source: "${none || "ok"}"; 1440x1080: "${four || "ok"}"`);
    assert.ok(/checking the video for black bars/.test(waiting) && none === "" && four === "");
  },

  async "splat calibration stops new tracking jobs before it waits for the vision worker (a busy worker never starves it), and lets them go again"() {
    const loc = { pose: () => ({ x: 1, y: 1, z: 0.05, yaw: 0, sigma: 0.02, status: "ok" }), source: "fused", poseAt: () => loc.pose() };
    const sl = new SplatLocalizer({ localizer: loc, ctl: { isFlying: () => false, videoDelay: 0 }, house, map });
    sl.perception = { source: { kind: "goggles", ready: () => true, element: () => ({}), region: () => ({ sx: 0, sy: 0, sw: 640, sh: 480 }) } };
    sl.busy = true;
    const ac = new AbortController(), run = sl.calibrate({ signal: ac.signal });
    const during = sl.calibrating;
    setTimeout(() => ac.abort(), 100);
    const err = await run.then(() => null, (e) => e.message);
    console.log(`      while waiting for the worker: calibrating ${during}; then "${err}", calibrating ${sl.calibrating}`);
    assert.ok(during === true && /busy/.test(err) && sl.calibrating === false);
  },

  "settings: every key the app's modules read has a default (settings.js)"() {
    const dir = path.join(ROOT, "app", "js"), keys = new Set();
    const walk = (d) => fs.readdirSync(d, { withFileTypes: true }).forEach((e) => (e.isDirectory() ? walk(path.join(d, e.name)) : e.name.endsWith(".js") && keys.add(path.join(d, e.name))));
    walk(dir);
    const read = new Set();
    for (const f of keys) for (const m of fs.readFileSync(f, "utf8").matchAll(/settings\??\.(?:get|set)\??\.?\(\s*"([a-zA-Z]+)"/g)) read.add(m[1]);
    const missing = [...read].filter((k) => !(k in DEFAULTS));
    console.log(`      ${read.size} keys read; ${missing.length ? `no default: ${missing.join(", ")}` : "all have defaults"}; the wire's: locVision ${DEFAULTS.locVision}, simLoc ${DEFAULTS.simLoc}, memoryDays ${DEFAULTS.memoryDays}`);
    assert.deepEqual(missing, []);
    for (const k of ["locVision", "simLoc", "simAugment", "avoid", "depthModel", "aiVision", "aiBudget", "memoryDays", "memoryFlights", "brake"]) assert.ok(k in DEFAULTS, k);
  },

  "models: every model the app downloads is pinned (a revision, its size and SHA-256) and Apache/MIT/BSD; the vendored XFeat file matches its checksum"() {
    const specs = [...Object.values(MODELS), ...Object.values(DEPTH_MODELS)];
    for (const s of specs) {
      assert.ok(/^https:\/\/huggingface\.co\/onnx-community\/[^/]+\/resolve\/[0-9a-f]{40}\//.test(s.url), `${s.name}: ${s.url} isn't pinned to a revision`);
      assert.ok(s.size > 1e6 && /^[0-9a-f]{64}$/.test(s.sha256), `${s.name}: size and sha256`);
    }
    for (const s of Object.values(DEPTH_MODELS)) assert.ok(/^(Apache-2\.0|MIT|BSD)/.test(s.licence), `${s.name}: ${s.licence}`);
    const xf = fs.readFileSync(path.join(ROOT, "app", "vendor", "xfeat", "xfeat_640x480.onnx"));
    assert.equal(xf.length, XFEAT.size);
    assert.equal(createHash("sha256").update(xf).digest("hex"), XFEAT.sha256);
    const banned = /lighterglue|superpoint|yolo|dinov2-(base|large)|depth-anything-v2-(base|large)/i;
    for (const s of specs) assert.ok(!banned.test(s.url), `${s.name}: ${s.url}`);
    console.log(`      ${specs.length} downloads pinned (${specs.map((s) => s.name).join(", ")}); XFeat vendored, ${xf.length} bytes`);
  },

  // ---- wave C2, fix 1 (the flight reviews) ----

  async "in the air, return home needs only what flying home needs (a low battery, an old vision fix, a busy machine send it home, never refuse it); a refused mission neither stops the one flying nor flies home by itself; the battery refuses only on the ground"() {
    const make = (mode, secondsLeft, age, { hang = false } = {}) => {
      const fix = { visionAge: age, rate: 2.2 };
      const ran = [], aborted = [], settings = settingsOf({ mode, startOnPad: true, autonomy: "full" });
      const ctl = Object.assign(new Emitter(), { autonomy: "full", videoDelay: 100, blocker: () => "", isFlying: () => true, abort: (why) => aborted.push(why), settings: { get: () => 82 },
        run: (b) => (ran.push(b.name), hang && ran.length === 1 ? new Promise(() => {}) : Promise.resolve({ ok: true, text: "Done." })),
        safety: { battery: () => ({ vbat: 3.6, secondsLeft }), avoid: { brake: () => ({ measured: true }), cruise: () => ({ v: 0.3 }), eyes: () => ({ ok: true }) } } });
      const loc = { source: "fused", pose: () => ({ x: -0.64, y: 2.06, z: 1.2, yaw: 0, sigma: 0.05, status: "ok", room: "r2" }), flown: true, fixQuality: () => fix, sigmaFloor: () => 0.05 };
      const splat = { enabled: true, calibOk: true, db: {}, pad: { ok: true }, features: { info: { backend: "webgpu" } }, trust: () => "verified" };
      const perception = { source: { detections: true }, latest: {}, on: () => () => {}, fps: hang ? 30 : 9, frameAge: 30 }; // a busy machine too
      const m = new MissionRunner({ ctl, map, house, localizer: loc, perception, settings, splat, depth: hang ? { estimator: {} } : null });
      m.preflightCheck = ((orig) => (o) => orig.call(m, o).filter((c) => c.id !== "picture"))(m.preflightCheck); // the picture is the vision side's
      if (hang) Object.assign(ctl.safety.avoid, { depthWhy: "", cloud: { t: performance.now() + 1e9 } });
      return { m, ran, aborted, fix };
    };
    const out = [];
    for (const [mode, left, age] of [["sim", 45, 0], ["real", 200, 1300], ["real", 45, 200]]) {
      const { m, ran } = make(mode, left, age), r = await m.run({ kind: "returnHome" }), battery = m.preflightCheck().find((c) => c.id === "battery");
      out.push(`${mode} ${left} s left, fix ${age} ms: ${ran.includes("home") ? "home" : "REFUSED"} ("${r.summary}"), battery item ${battery.level}`);
      assert.ok(r.ok && ran.includes("home") && battery.level !== "block", out.at(-1));
      m.dispose();
    }
    const { m, ran, aborted, fix } = make("real", 200, 200, { hang: true });
    fix.rate = 4;
    const going = m.run({ kind: "goTo", target: { x: 2.0, y: 1.5 } });
    for (let i = 0; i < 5; i++) await new Promise((r) => setImmediate(r));
    fix.visionAge = 1300; // the fix aged while it flies
    const busy = m.busy, ac = m.ac, p = await m.run({ kind: "patrol" });
    out.push(`a patrol asked in the air with the fix 1.3 s old: "${p.summary.slice(0, 70)}…", refused ${p.refused}, behaviors run ${ran.join(",")}, the goTo still flying ${m.ac === ac && busy}`);
    console.log(`      ${out.join("\n      ")}`);
    assert.ok(busy && !p.ok && p.refused && /No position fix/.test(p.summary) && ran.length === 1 && m.ac === ac && !aborted.length, out.at(-1));
    going.catch(() => {}); // its flight never ends (the fake controller): left as it is
    m.dispose();
  },

  async "suspected changes from an earlier flight stay on the map after take-off (unseen counts from the take-off), come off after a minute unseen in flight only while the drone can see what the scan lacks, stay off through gaps in what it sees, and go back on after 3 s blind unless the drone is at it or flying through it"() {
    const { m: mem } = await memoryFor("real");
    let flying = false, eyes = { ok: true }, pose = { x: 2.9, y: 1.6, z: 1.2, yaw: 0, sigma: 0.05, status: "ok", room: "r1" };
    const lifted = new Set(), ctl = Object.assign(new Emitter(), { isFlying: () => flying, blocker: () => "", autonomy: "full", behavior: null,
      safety: { avoid: { eyes: () => eyes, lift: (id) => lifted.add(id), unlift: (id) => lifted.delete(id) } } });
    const m = new MissionRunner({ ctl, map, house, localizer: { pose: () => pose, flown: true }, perception: null, settings: settingsOf({ mode: "real" }), memory: mem });
    const said = [];
    m.on("status", (s) => /hasn't been seen again/.test(s.text) && said.push(s.text));
    const f = map.floorAt(2.6, 1.6), rec = mem.addChange({ kind: "obstacle", x: 2.6, y: 1.6, z: f + 0.4, size: 0.5, zMin: f, zMax: f + 1.6 }, clock);
    let t = 7e5;
    const at = (ms) => ((clock += ms), m.liftStale((t += ms)), map.temps.has(rec.id)), blindFor = (n) => ((eyes = { ok: false, why: "live depth frames stopped" }), Array.from({ length: n }, () => at(1100)).at(-1));
    clock += 864e5; // the next day
    at(1000);
    flying = true;
    const s = [at(0), at(2000), at(28000), at(31000)], creep = lifted.has(rec.id);
    const gap = blindFor(2); // a gap between fixes (2.2 s), the drone at the very spot
    eyes = { ok: true };
    at(1100);
    const atIt = blindFor(4); // 4.4 s blind at it: stays off (a creep near it instead)
    pose = { ...pose, x: 4.2, y: 3.0 }; // 2 m off
    const away = at(1100);
    eyes = { ok: true };
    const again = at(1100);
    ctl.behavior = { name: "path", path: [[4.2, 3.0, 1.2], [3.3, 2.2, 1.2], [2.6, 1.6, 1.2], [1.5, 1.4, 1.2]], i: 0 }; // flying through it
    const leg = blindFor(4);
    ctl.behavior = null;
    const done = at(1100);
    console.log(`      a day later, after take-off: ${s.slice(0, 3).map((v) => (v ? "on" : "off")).join(", ")} (0, 2 and 30 s); 61 s: ${s[3] ? "on" : "off"} (avoid told: ${creep}); blind 2.2 s at it: ${gap ? "on" : "off"}; 4.4 s at it: ${atIt ? "on" : "off"}; 2 m away: ${away ? "on" : "off"}; seeing again: ${again ? "on" : "off"}; 4.4 s blind on a leg through it: ${leg ? "on" : "off"}, after it ${done ? "on" : "off"}; said ${said.length} time(s)`);
    assert.ok(s[0] && s[1] && s[2] && !s[3] && creep, JSON.stringify(s));
    assert.ok(!gap && !atIt && away && !again && !leg && done && said.length === 1, JSON.stringify({ gap, atIt, away, again, leg, done, said: said.length }));
    flying = false;
    at(1000);
    assert.ok(map.temps.has(rec.id) && !lifted.size, "back on after landing");
    mem.resolveChange(rec.id, "dismissed", "a test");
    m.dispose();
  },

  "findings: one person walking through a patrol is one finding and one alert (seen in another room since: said); two seen in one frame are two; a spot off every room is no room (not a hallway the house hasn't) and no high-urgency alert"() {
    const alerts = [], mk = () => {
      const ctl = Object.assign(new Emitter(), { isFlying: () => true, blocker: () => "", abort() {} });
      const m = new MissionRunner({ ctl, map, house, localizer: { pose: () => ({ x: 0, y: 0, z: 1, yaw: 0, sigma: 0.05, status: "ok" }) }, perception: { snapshot: () => null }, settings: settingsOf({ mode: "sim" }),
        alerts: { notify: (a) => alerts.push({ ...a, how: "notify" }), log: (a) => alerts.push({ ...a, how: "log" }) } });
      Object.assign(m, { findings: [], t0: 0, mission: { kind: "patrol" } });
      return m;
    };
    const see = (m, id, x, y, t0, t) => (m.tracks.set(id, { t0, t }), m.addFinding({ label: "person", trackId: id, x, y, score: 0.9, t, t0, size: 0.5 }, { alert: true }));
    const walk = mk();
    see(walk, 1, -0.64, 2.06, 1000, 1500); // Room 2
    see(walk, 2, 2.0, 1.5, 6000, 6400); // the Living room 2.7 m on, 4.5 s later
    see(walk, 3, 0.11, 3.71, 10500, 11000); // Room 3
    const one = walk.findings, high = alerts.filter((a) => a.urgency === "high").length;
    const two = mk();
    two.together.add("4|5").add("5|4");
    see(two, 4, -0.64, 2.06, 20000, 20500);
    see(two, 5, 0.6, 2.06, 20000, 20500);
    let off = null; // just outside a room's outline, in no room
    for (const r of house.rooms) for (const [x, y] of r.outline) for (const [dx, dy] of [[0.4, 0], [-0.4, 0], [0, 0.4], [0, -0.4]]) if (!off && !map.roomAt(x + dx, y + dy) && map.idx(x + dx, y + dy) >= 0) off = [x + dx, y + dy];
    const lone = mk(), n0 = alerts.length;
    see(lone, 6, off[0], off[1], 30000, 30500);
    const f = lone.findings[0], a = alerts[n0];
    console.log(`      walking: ${one.length} finding (${one[0].where}, then ${one[0].also?.join(", then ")}), ${high} high alert; two in one frame: ${two.findings.length}; off the map at (${off.map((v) => v.toFixed(1))}): "${f.where}", room ${f.room}, alert ${a.urgency}`);
    assert.ok(one.length === 1 && high === 1 && one[0].also?.length === 2, "one person, one alert");
    assert.equal(two.findings.length, 2, "two people seen together");
    assert.ok(f.room === null && /somewhere past/.test(f.where) && !/hallway/.test(f.where + f.roomName) && a.urgency === "default", JSON.stringify(f));
    for (const m of [walk, two, lone]) m.dispose();
  },

  "the checklist: room around the take-off spot (the fixture's pad is 0.32 m from a wall: a warning in the simulator, a refusal on the real drone) and a machine too busy for the real drone; the planner's reasons in plain words, once each; a scan's time budget at the speed vision scans really turn"() {
    const values = { mode: "sim", startOnPad: true }, settings = settingsOf(values), h = house.home;
    const ctl = Object.assign(new Emitter(), { blocker: () => "", isFlying: () => false, autonomy: "full", videoDelay: 100, settings: { get: (k) => (k === "maxYawRate" ? 120 : 82) }, safety: { battery: () => ({ vbat: 4.0, secondsLeft: 200 }), avoid: { brake: () => ({ measured: true }), depthWhy: "" } } });
    let vision = false;
    const loc = { source: "fused", pose: () => ({ x: h.x, y: h.y, z: 0, yaw: 0, sigma: 0.03, status: "ok", room: "r1" }), flown: false, fixQuality: () => ({ visionAge: 100, rate: 2.4 }), visionActive: () => vision };
    const splat = { enabled: true, calibOk: true, db: {}, pad: { ok: true }, features: { info: { backend: "webgpu" } }, trust: () => "verified" };
    const perception = { source: { detections: true }, latest: {}, on: () => () => {}, fps: 8, frameAge: 30 };
    const m = new MissionRunner({ ctl, map, house, localizer: loc, perception, settings, splat, depth: { estimator: { info: {} }, twin: {}, loaded: true } });
    const item = (id) => m.preflightCheck().find((c) => c.id === id);
    const sim = item("pad");
    values.mode = "real";
    const real = item("pad"), pace = item("pace");
    console.log(`      the pad: ${m.padRoom(h.x, h.y).toFixed(2)} m clear: simulator ${sim.level}, real ${real.level} ("${real.text}", button "${real.fix?.label}" -> ${real.fix?.action}); pace: ${pace.level} ("${pace.text}")`);
    assert.ok(sim.level === "warn" && real.level === "block" && /Move the pad/.test(real.text) && pace.level === "block" && /too busy/.test(pace.text) && /8 frames/.test(pace.text));
    assert.ok(real.fix?.action === "home" && sim.fix?.action === "home" && /Home tool/.test(real.text), "the pad item's button opens the map's Home tool (main.js readyFix 'home')");
    // the real drone waits for the person detector to run, and full auto for live depth (co-pilot may go without, slowly)
    const depthOn = item("depth");
    m.depth = null;
    const depthFull = item("depth");
    ctl.autonomy = "copilot";
    const depthCo = item("depth");
    ctl.autonomy = "full";
    perception.detector = { ready: false, backend: "rfdetr" };
    delete perception.source.detections;
    const detReal = item("detector");
    values.mode = "sim";
    const detSim = item("detector");
    console.log(`      live depth loaded: ${depthOn.level}; not loaded: full auto ${depthFull.level}, co-pilot ${depthCo.level}; the detector loading: real ${detReal.level} ("${detReal.text}"), simulator ${detSim.level}`);
    assert.ok(depthOn.level === "info" && depthFull.level === "block" && depthCo.level === "warn" && detReal.level === "block" && detSim.level === "warn");
    map.addTemp({ id: "zone-test", kind: "person", x: h.x + 0.3, y: h.y + 0.3, r: 1.5, until: Infinity });
    const said = m.plain("start (0.01, 0.16) has no free space within 1 m (σ 0.17 m); start (0.01, 0.16) has no free space within 1 m (σ 0.17 m); no path with σ 0.17 m", [h.x, h.y]);
    map.removeTemp("zone-test");
    const bare = m.plain("start (0.01, 0.16) has no free space within 1 m (σ 0.17 m)", [h.x, h.y]);
    console.log(`      "${said}" | "${bare}"`);
    assert.ok(/someone is standing by the pad/.test(said) && said.split(";").length === 2 && !/σ|\(-?\d/.test(said + bare) && /isn't clear on the map/.test(bare), said);
    const plain4 = m.scanSeconds(4);
    vision = true;
    const vis4 = m.scanSeconds(4);
    m.noteScan(30, 4);
    const slow = m.scanSeconds(4);
    console.log(`      a 4-heading scan budgets ${plain4.toFixed(1)} s, with vision ${vis4.toFixed(1)} s (16.4 s measured in the simulator), after a 30 s one ${slow.toFixed(1)} s`);
    assert.ok(plain4 >= MISSION.scanTime && vis4 >= 15 && slow >= 30 - 1e-9);
    m.dispose();
  },

  // Fix 2 (the verifier): LiveDepth's own state on the ground ("not started", "stopped", the last flight's) refused every
  // full-auto mission on the real drone from the pad.
  async "live depth on the ground: the checklist blocks full auto only on what keeps it from starting (off, its model still loading or failed to load, no 3D twin: each said as it is), never on its state from before or from the last flight; in the air on that state and frames coming"() {
    const values = { mode: "real", startOnPad: true, avoid: true }, settings = settingsOf(values);
    let flying = false;
    const avoid = { depthWhy: "", setDepthStatus(why = "") { this.depthWhy = why; }, brake: () => ({ measured: true }), cloud: null };
    const ctl = Object.assign(new Emitter(), { blocker: () => "", isFlying: () => flying, autonomy: "full", videoDelay: 100, settings: { get: () => 82 }, safety: { battery: () => ({ vbat: 4.1, secondsLeft: 220 }), avoid } });
    const loc = { source: "fused", pose: () => ({ x: 1.5, y: 1.2, z: flying ? 1 : 0, yaw: 0, sigma: 0.03, status: "ok" }), flown: false, fixQuality: () => ({ visionAge: 100, rate: 5 }), visionActive: () => true, poseAt: () => loc.pose() };
    const perception = { source: { detections: true, ready: () => true }, latest: {}, on: () => () => {}, fps: 30, frameAge: 30 };
    const depth = new LiveDepth({ perception, localizer: loc, ctl, settings, twin: {} });
    const model = () => (depth.warmed = { info: {}, load: async () => ({ name: "Depth Anything V2 Small", cached: true, loadMs: 5 }), dispose() {} }); // the model loads (no download here)
    depth.on("status", ({ why }) => ctl.safety.avoid.setDepthStatus(why ?? "")); // as ui/session.js does
    const m = new MissionRunner({ ctl, map, house, localizer: loc, perception, settings, depth }), item = () => m.preflightCheck().find((c) => c.id === "depth"), out = [];
    const note = (what) => (out.push(`${what}: ${item().level} ("${avoid.depthWhy}")`), item().level);
    // the first download, still going: "still loading" (a wait, not a failure); then a dropped one: its error
    let done;
    const dl = (depth.warmed = { load: () => new Promise((r) => (done = () => ((dl.info = {}), r({ name: "Depth Anything V2 Small", cached: false, loadMs: 9000 })))), dispose() {} });
    const starting = depth.start(), loadingText = item().text, loading = note("its model downloading");
    done();
    await starting;
    const loaded = note("downloaded");
    depth.stop();
    depth.warmed = { load: async () => { throw new Error("the download was cut off"); }, dispose() {} };
    await depth.start().catch(() => {});
    const failedText = item().text, failed = note("the download cut off");
    model();
    await depth.start();
    const started = note("started on the ground");
    await depth.frame({ t: 1 }); // a frame on the ground changes nothing (it runs only in the air)
    const frame = note("after a frame on the ground");
    values.avoid = false;
    depth.stop();
    const off = note("turned off");
    values.avoid = true;
    model();
    await depth.start();
    const back = note("on again");
    flying = true;
    depth.state("I don't know where I am");
    const air = note("in the air, lost");
    depth.state("");
    avoid.cloud = { t: performance.now() };
    const running = note("in the air, frames coming");
    depth.state("the view has too little of the scan to line the depth up with");
    flying = false;
    const landed = note("landed, the last frame's state kept");
    depth.setTwin(null);
    const noTwin = note("no 3D twin");
    console.log(`      ${out.join("; ")}\n      "${loadingText}" | "${failedText}"`);
    assert.ok(loading === "block" && /still loading/.test(loadingText) && !/isn't running|didn't load/.test(loadingText) && loaded === "info", loadingText);
    assert.ok(failed === "block" && /didn't load \(the download was cut off\)/.test(failedText), failedText);
    assert.ok(started === "info" && frame === "info" && off === "block" && back === "info" && air === "block" && running === "info" && landed === "info" && noTwin === "block", out.join("; "));
    depth.setTwin({});
    const blocks = m.preflightCheck().filter((c) => c.level === "block").map((c) => c.id);
    assert.ok(!blocks.includes("depth"), `a patrol from the pad refused: ${blocks}`); // C2 fix 1: "Live depth isn't running (not started)…"
    m.dispose();
  },

  // Fix 2: the checklist no longer waits on the ground for live depth's first frame (it comes only in the air), so the
  // take-off does: a full-auto mission on the real drone never flies its legs blind.
  async "a full-auto mission on the real drone (or its rehearsal) hovers after its take-off until live depth sees (at most 10 s), then flies; without it, it comes home and lands instead of flying blind; the simulator, the simulator's truth and co-pilot don't wait"() {
    const h = house.home, goal = roomCenter(map, "r1", { alt: 1.0 }), out = [];
    const make = ({ mode = "real", source = "fused", autonomy = "full", eyesAt = Infinity, rehearse = false }) => {
      let t = 0, flying = false;
      const ran = [], values = { mode, startOnPad: true, autonomy, simRehearse: rehearse }, settings = settingsOf(values);
      const ctl = Object.assign(new Emitter(), { autonomy, videoDelay: 100, blocker: () => "", isFlying: () => flying, settings: { get: (k) => (k === "hfov" ? 127 : 82) }, abort() {}, askPilot() {},
        safety: { battery: () => ({ vbat: 4.1, secondsLeft: 220 }), personClose: () => false, avoid: { real: mode === "real" || rehearse, brake: () => ({ measured: true }), cruise: () => ({ v: 0.3 }), noteLeg() {}, eyes: () => (t >= eyesAt ? { ok: true, why: "" } : { ok: false, why: "no live depth yet" }) } },
        run: async (b) => { // a fake flight: the take-off and the landing at once, a wait stepped in 0.1 s, every leg flown
          ran.push(b.name === "wait" ? `${b.label} (${t.toFixed(1)} s)` : b.name);
          if (b.name === "takeoff") return (flying = true), { ok: true, text: "Up." };
          if (b.name === "home") return (flying = false), { ok: true, text: "Landed." };
          if (b.name !== "wait") return { ok: true, text: "Done." };
          if (b.label === "Waiting for the pilot to take off") return (flying = true), { ok: true, text: "Flying." };
          for (b.t0 = t * 1000; ; t += 0.1) { const r = b.update(ctl, 0.1, t * 1000); if (r.done) return (ran.push(`${r.done.ok ? "seeing" : "still blind"} at ${t.toFixed(1)} s`), r.done); }
        } });
      const loc = { source, pose: () => ({ x: flying ? goal[0] : h.x, y: flying ? goal[1] : h.y, z: flying ? 1 : 0, yaw: 0, sigma: 0.03, status: "ok", room: "r1" }), flown: false, fixQuality: () => ({ visionAge: 100, rate: 5 }), visionActive: () => true, sigmaFloor: () => 0.03 };
      loc.poseAt = loc.pose;
      const m = new MissionRunner({ ctl, map, house, localizer: loc, perception: null, settings });
      m.preflight = () => null; // the checklist is tested on its own
      return { m, ran };
    };
    const cases = [["real, live depth sees 3 s after the take-off", { eyesAt: 3 }], ["real, it never does", {}], ["rehearsing the real flight in the simulator, it sees at 2 s", { mode: "sim", rehearse: true, eyesAt: 2 }],
      ["the simulator", { mode: "sim" }], ["real with the simulator's truth", { source: "truth" }], ["co-pilot", { autonomy: "copilot" }]];
    const res = {};
    for (const [name, o] of cases) {
      const { m, ran } = make(o), r = await m.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } });
      res[name] = { r, ran };
      out.push(`${name}: ${ran.join(" > ")} -> "${r.summary}"`);
      m.dispose();
    }
    console.log(`      ${out.join("\n      ")}`);
    const a = res[cases[0][0]], b = res[cases[1][0]];
    assert.ok(a.r.ok && /Waiting for live depth/.test(a.ran[1]) && a.ran.includes("seeing at 3.0 s") && a.ran.includes("path"), out[0]);
    assert.ok(!b.r.ok && b.ran.includes("still blind at 10.1 s") && !b.ran.includes("path") && b.ran.at(-1) === "home" && /Live depth didn't start once I was up \(no live depth yet\)/.test(b.r.summary) && /Landed on the home pad/.test(b.r.summary), out[1]);
    assert.ok(res[cases[2][0]].r.ok && res[cases[2][0]].ran.includes("seeing at 2.0 s"), out[2]);
    for (const [name] of cases.slice(3)) assert.ok(res[name].r.ok && !res[name].ran.some((q) => /live depth/.test(q)), `${name}: ${res[name].ran}`);
  },

  // Fix 2, from the UI round: "Rehearse the real flight" flies by the real caps; the picture line per video source; the AI
  // switch alone only warns in the list (the page asks for it as a mission starts) while missions still refuse on it.
  "the rehearsal flies by the real drone's caps; the picture line says what to do per video source (the goggles' picture is found by itself); the AI switch off is a warning in the list, a refusal for a mission"() {
    const P = { x: 2.6, y: 4.8, z: map.floorAt(2.6, 4.8) + 1, yaw: 0, sigma: 0.05, zSigma: 0.02, status: "ok" }, caps = {};
    for (const [name, v] of [["the simulator", { mode: "sim" }], ["rehearsing", { mode: "sim", simRehearse: true }], ["the real drone", { mode: "real" }]]) {
      const av = new Avoid({ map, localizer: { ctl: { videoDelay: 0, est: { ttc: Infinity } }, pose: () => P, velocity: () => [0, 0], fixQuality: () => null }, settings: settingsOf({ avoid: true, ...v }) });
      caps[name] = { real: av.real, cap: av.speedCap(), top: av.realTop().v, measured: av.brake().measured };
    }
    console.log(`      ${Object.entries(caps).map(([k, c]) => `${k}: real ${c.real}, cap ${c.cap.v} ("${c.cap.why}"), top ${c.top}`).join("; ")}`);
    assert.ok(!caps["the simulator"].real && caps["the simulator"].cap.v === Infinity, "the plain simulator: no real caps");
    for (const k of ["rehearsing", "the real drone"]) assert.ok(caps[k].real && caps[k].cap.v === AVOID.blind && caps[k].top === AVOID.realVmax && !caps[k].measured, k);
    // the picture, per source: the goggles' picture is found by itself (no box to drag while it looks), a capture's crop is the user's
    const values = { mode: "real", startOnPad: true }, settings = settingsOf(values), tel = { engaged: true, failsafe: false };
    let blocker = "";
    const ctl = Object.assign(new Emitter(), { blocker: () => blocker, radio: {}, telemetryFresh: true, tel, autonomy: "full", isFlying: () => false, videoDelay: 100, settings: { get: () => 82 }, safety: { battery: () => ({ vbat: 4.0, secondsLeft: 200 }), avoid: { brake: () => ({ measured: true }), depthWhy: "" } } });
    const loc = { source: "fused", pose: () => ({ x: -0.64, y: 2.06, z: 0, yaw: 0, sigma: 0.03, status: "ok", room: "r2" }), flown: false, fixQuality: () => ({ visionAge: 100, rate: 4 }), visionActive: () => true };
    const per = { frameAge: 30, fps: 30, latest: {}, on: () => () => {}, state: "looking", base: { sx: 0, sy: 0, sw: 1920, sh: 1080 }, kind: "goggles" };
    Object.defineProperty(Object.assign(per, { pictureState: () => per.state, region: () => null }), "source", { get: () => ({ kind: per.kind, ready: () => true, region: () => per.base }) });
    const m = new MissionRunner({ ctl, map, house, localizer: loc, perception: per, settings }), pic = () => m.preflightCheck().find((c) => c.id === "picture"), said = {};
    for (const [kind, state, sw, sh] of [["goggles", "looking", 1920, 1080], ["goggles", "dark", 1920, 1080], ["goggles", "whole", 320, 320], ["camera", "looking", 1920, 1080], ["camera", "whole", 320, 320]])
      Object.assign(per, { kind, state, base: { sx: 0, sy: 0, sw, sh } }), (said[`${kind} ${state}`] = pic());
    console.log(`      ${Object.entries(said).map(([k, c]) => `${k}: "${c.text}"${c.fix ? ` [${c.fix.label}]` : ""}`).join("\n      ")}`);
    assert.ok(/Still finding the camera picture in the goggles' video/.test(said["goggles looking"].text) && !/drag a box/.test(said["goggles looking"].text + said["goggles dark"].text) && !said["goggles looking"].fix, said["goggles looking"].text);
    assert.ok(/point the camera at a lit room/.test(said["goggles dark"].text) && /found by itself; if this stays, drag a box/.test(said["goggles whole"].text) && said["goggles whole"].fix?.action === "crop-video", said["goggles whole"].text);
    assert.ok(/Still finding the camera picture in the video/.test(said["camera looking"].text) && / \(drag a box around the picture on the video\)\.$/.test(said["camera whole"].text), said["camera whole"].text);
    assert.ok(Object.values(said).every((c) => c.level === "block"), "each still refuses");
    // the AI switch off: a warning in the list, still a refusal to start
    Object.assign(per, { state: "whole", base: { sx: 0, sy: 0, sw: 1440, sh: 1080 } });
    Object.assign(tel, { engaged: false });
    blocker = "The AI switch on the radio is off.";
    const sw = m.preflightCheck().find((c) => c.id === "radio"), refused = m.preflight({ kind: "patrol" });
    Object.assign(ctl, { telemetryFresh: false });
    blocker = "The radio isn't connected (no telemetry).";
    const unlinked = m.preflightCheck().find((c) => c.id === "radio");
    console.log(`      the AI switch off: ${sw.level} ("${sw.text}"), a patrol: "${refused}"; no telemetry: ${unlinked.level}`);
    assert.ok(sw.level === "warn" && !sw.ok && /flip it on/.test(sw.text) && /AI switch on the radio is off/.test(refused ?? "") && unlinked.level === "block", sw.text);
    m.dispose();
  },

  // Fix 2 (the verifier's browser run): a scan climbed from 1.0 to 2.1 m on the controller's own flow height (the safety
  // layer only ever climbed back) and touched a doorway's head, which the map's top band (1.4 m) doesn't reach.
  "height: the safety layer brings a scan or a hover back down from 12 cm over the height held (and up from 8 cm under, as before), holds it no higher than the map's top band, and over every band holds still and comes down without a geofence push"() {
    const settings = settingsOf({ mode: "sim", autonomy: "full", hfov: 127, uptilt: 20 }), open = roomCenter(map, "r1", { alt: 1.0 }), f = map.floorAt(...open);
    let P = { x: open[0], y: open[1], z: f + 1.0, yaw: 0, sigma: 0.05, zSigma: 0.05, status: "ok" };
    const ctl = Object.assign(new Emitter(), { est: { heading: 0, ttc: Infinity }, videoDelay: 100, autonomy: "full", airborne: true, heightSlowdown: 1, settings, isFlying: () => true, perception: { frameAge: Infinity, latest: { t: 0, detections: [] }, find: () => [] } });
    const loc = Object.assign(new Emitter(), { ctl, pose: () => P, poseAt: () => P, velocity: () => [0, 0], visionActive: () => false, fixQuality: () => ({}) });
    const safety = new Safety({ map, localizer: loc, settings, house });
    const vz = (z, sp = { heading: 0 }) => ((P = { ...P, z: f + z }), safety.filter(sp, { ctl, now: 1e6, dt: 0.033 }));
    const held = vz(1.0).vz, over = vz(1.1).vz, high = vz(1.3), under = vz(0.8).vz;
    safety.zHold = null;
    const start = vz(1.6); // a hold begun up there is held at the top band
    const hold = safety.zHold - f, above = vz(2.1, { vx: 0.3, vy: 0.1, heading: 0 });
    console.log(`      held at 1.0 m: at 1.0 ${held ?? "no"} climb; 1.1 m: ${over ?? "nothing"}; 1.3 m: ${high.vz?.toFixed(2)} m/s; 0.8 m: +${under?.toFixed(2)} m/s; begun at 1.6 m: held at ${hold.toFixed(1)} m (${start.vz?.toFixed(2)} m/s); at 2.1 m flying on: vx ${above.vx}, vz ${above.vz?.toFixed(2)} ("${safety.reason}")`);
    assert.ok(held === undefined && over === undefined && high.vz < -0.1 && under > 0.05, "a band each way");
    assert.ok(Math.abs(hold - map.o.bands.at(-1)) < 1e-9 && start.vz < 0, "no higher than the top band");
    assert.ok(above.vx === undefined && above.vy === undefined && above.vz <= -SAFETY.zDown + 1e-9 && /too high/.test(safety.reason), "over every band: hold still, come down");
  },

  async "the localizer: moved on the ground (the vision fixes then disagree), σ grows until a fix is taken (within 8 s); a flight whose vision never locks on counts from its take-off (degraded at 1.5 s, lost at 5 s) when vision is expected"() {
    const real = globalThis.performance;
    let clock = 1000;
    Object.defineProperty(globalThis, "performance", { value: { now: () => clock }, configurable: true, writable: true });
    try {
      let flying = false;
      const ctl = Object.assign(new Emitter(), { est: { heading: 0, vx: 0, vy: 0, vz: 0, flowQ: 0.8, rotating: false }, videoDelay: 100, perception: { fps: 30, latest: { flow: null } }, isFlying: () => flying });
      const loc = new Localizer({ ctl, map: null, house: null }), dt = 1 / 30;
      loc.reset({ x: 0, y: 0, yaw: 0, z: 0 });
      let truth = 0, taken = null, next = 0;
      for (let t = 0; t < 14; t += dt) {
        clock += 1000 * dt;
        if (t >= 3) truth = 0.45; // a foot nudged it
        ctl.emit("tick", { dt, now: clock });
        if (clock >= next) (next = clock + 250, loc.fix({ x: truth, y: 0, z: 0, yaw: 0, sigma: 0.04, yawSigma: 0.03, t: clock - 50, source: "splat" }));
        if (t > 3 && taken == null && Math.abs(loc.pose().x - truth) < 0.1) taken = t - 3;
      }
      console.log(`      moved 0.45 m on the ground: the fixes taken again after ${taken?.toFixed(1)} s (σ ${loc.pose().sigma.toFixed(2)}, error ${Math.abs(loc.pose().x - truth).toFixed(2)} m)`);
      assert.ok(taken != null && taken < 8, `taken after ${taken}`);
      const air = new Localizer({ ctl, map: null, house: null }), seen = {};
      air.expectVision = true;
      air.reset({ x: 0, y: 0, yaw: 0, z: 0 });
      flying = true;
      for (let t = 0; t < 6; t += dt) {
        clock += 1000 * dt;
        ctl.perception.latest.flow = { t: clock, quality: 0.8, span: 0.1 }; // the flow says it hovers
        ctl.emit("tick", { dt, now: clock });
        for (const at of [1, 2, 5.5]) if (t >= at && !seen[at]) seen[at] = air.pose().status;
      }
      console.log(`      no vision fix since the take-off: ${Object.entries(seen).map(([t, v]) => `${t} s ${v}`).join(", ")}; vision age ${Math.round(air.fixQuality().visionAge)} ms`);
      assert.ok(seen[1] === "ok" && seen[2] === "degraded" && seen[5.5] === "lost" && air.visionActive(), JSON.stringify(seen));
      loc.dispose();
      air.dispose();
    } finally {
      Object.defineProperty(globalThis, "performance", { value: real, configurable: true, writable: true });
    }
  },

  "safety: the real drone's video too slow to fly by holds it still, then lands it; the geofence's push fades to nothing as the last vision fix ages to the hold; a standing adult, a seated sim actor (taller than 1.8 times its width) and a tall box aren't 'seated'"() {
    const real = globalThis.performance;
    let clock = 1e6;
    Object.defineProperty(globalThis, "performance", { value: { now: () => clock }, configurable: true, writable: true });
    try {
      const values = { mode: "real", autonomy: "full", hfov: 127, uptilt: 20 }, settings = { get: (k) => values[k], all: () => values, on: () => () => {} };
      const per = Object.assign(new Emitter(), { fps: 8, frameAge: 30, latest: { t: 0, detections: [], width: 320, height: 240 }, find: () => [] });
      const ctl = new FlightController({ settings, perception: per });
      ctl.isFlying = () => true;
      const open = roomCenter(map, "r1", { alt: 1.0 }), P = { x: open[0], y: open[1], z: map.floorAt(...open) + 1, yaw: 0, sigma: 0.05, zSigma: 0.02, status: "ok" };
      let q = { visionAge: 100, rate: 4, vision: true };
      const loc = Object.assign(new Emitter(), { ctl, pose: () => P, poseAt: () => P, velocity: () => [0, 0], visionActive: () => q.vision, fixQuality: () => q, headingFor: () => 0 });
      const safety = new Safety({ map, localizer: loc, settings, house });
      ctl.safety = safety;
      const states = [];
      for (const s of [0, 1, 2.5, 5.5]) {
        clock = 1e6 + 1000 * s;
        safety.watch(P);
        const out = safety.filter({ vx: 0.3, vy: 0, heading: 1 }, { ctl, now: clock, dt: 0.033 });
        states.push(`${s} s: ${out.vx === undefined ? "held" : `${out.vx.toFixed(2)} m/s`}${safety.takeover ? ` (${safety.takeover.label ?? safety.takeover.name})` : ""}`);
      }
      console.log(`      8 frames a second on the real drone: ${states.join("; ")}`);
      assert.ok(/held/.test(states[2]) && !/held/.test(states[0]) && /Landing/.test(states[3]), states.join("; "));
      // co-pilot: the pilot is asked to land it, and the alert doesn't say the drone is landing itself
      safety.takeover = null;
      per.fps = 30;
      safety.watch(P);
      const trips = [], asked = [];
      safety.on("trip", (t) => trips.push(t));
      ctl.askPilot = (kind, text) => asked.push(`${kind}: ${text}`);
      values.autonomy = "copilot";
      safety.tripped.clear(); // another flight
      per.fps = 8;
      for (const s of [0, 5.5]) (clock = 2e6 + 1000 * s), safety.watch(P);
      const pace = trips.find((t) => t.kind === "pace-land");
      console.log(`      co-pilot: asked "${asked[0]}"; the alert: "${pace?.reason}" (${pace?.action}); takeover ${safety.takeover?.name ?? "none"}`);
      assert.ok(asked.length === 1 && /^land/.test(asked[0]) && pace && pace.action === "warn" && /take over and land/.test(pace.reason) && !/I'm landing/.test(pace.reason) && !safety.takeover, pace?.reason);
      values.autonomy = "full";
      safety.takeover = null;
      per.fps = 30;
      safety.watch(P);
      const caps = [];
      for (const age of [300, 1000, 2000, 2400]) {
        q = { visionAge: age, rate: 4, vision: true };
        safety.vision({}, ctl, clock);
        caps.push(safety.pushCap);
      }
      console.log(`      the push's cap with the last fix 0.3, 1, 2, 2.4 s old: ${caps.map((c) => (Number.isFinite(c) ? c.toFixed(2) : "none")).join(", ")} m/s`);
      assert.ok(caps[0] === Infinity && caps[1] <= SAFETY.pushStale && caps[2] < caps[1] && caps[3] < 0.02);
      const box = (w, h, y = 0.3) => ({ label: "person", score: 0.9, box: { x: 0.4, y, w, h } });
      const stand = safety.seated(box(0.1, 0.42), per, 3), sit = safety.seated(box(0.15, 0.31), per, 3), wide = safety.seated(box(0.2, 0.25, 0.45), per, 2.5), far = safety.seated(box(0.16, 0.3, 0.1), per, 6);
      console.log(`      seated: standing adult ${stand}, the sim's sitting actor ${sit}, someone lying or wide ${wide}, a squat box 6 m off (2 m tall) ${far}`);
      assert.ok(!stand && !sit && wide && !far);
    } finally {
      Object.defineProperty(globalThis, "performance", { value: real, configurable: true, writable: true });
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
    console.log(`FAIL  ${name}\n      ${e.message.split("\n").slice(0, 6).join("\n      ")}\n      ${e.stack?.split("\n").find((l) => l.includes("test-wave-c.mjs:"))?.trim() ?? ""}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed (${((performance.now() - t0) / 1000).toFixed(1)} s)`);
process.exit(failed ? 1 : 0);
