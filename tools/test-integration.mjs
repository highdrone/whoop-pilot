// The seams between the Wave B tracks, as main.js wires them: the simulated co-pilot holds the drone still between
// commands (so a mission that ends in the air doesn't drift into a wall), the safety layer does nothing while it isn't
// the controller's filter and keeps no stale state for when it is attached again, the localizer forgets the
// simulator's position on switching to the real drone, and npm test runs every suite in this folder.
// Usage: cd tools && node test-integration.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";

const { Simulator } = await import("../app/js/sim/simulator.js");
const { Safety } = await import("../app/js/safety.js");
const { Localizer } = await import("../app/js/nav/localizer.js");
const { Emitter } = await import("../app/js/util.js");
const { FlightController } = await import("../app/js/controller.js");
const { MissionRunner } = await import("../app/js/missions.js");
const { locate } = await import("../app/js/behaviors.js");
const I = await import("../app/js/house/import.js");

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const { house, map } = await I.importCapture(fsSource(FIXTURE));

// Hover with the simulated pilot holding height and the AI not steering; `thumbs` false: the old pilot (throttle only).
function hoverDrift(seed, { thumbs = true, secs = 30 } = {}) {
  const sim = new Simulator({ headless: true, seed });
  if (!thumbs) {
    const keys = sim.readKeys.bind(sim);
    sim.readKeys = (dt) => (keys(dt), (sim.handsOn = true));
  }
  sim.pilotRequest("takeoff");
  for (let t = 0; t < 6; t += 0.02) sim.step(0.02);
  const d = sim.drone, x0 = d.x, y0 = d.y;
  let stick = 0, vmax = 0;
  for (let t = 0; t < secs; t += 0.02) {
    sim.step(0.02);
    stick = Math.max(stick, Math.abs(sim.pilot.pitch), Math.abs(sim.pilot.roll));
    vmax = Math.max(vmax, Math.hypot(d.vx, d.vy));
  }
  return { drift: Math.hypot(d.x - x0, d.y - y0), stick, vmax, airborne: d.airborne, crashed: d.crashed, override: Object.values(sim.bridge.ovr).some(Boolean) };
}

const fakeCtl = () => Object.assign(new Emitter(), {
  est: { heading: 0, vx: 0, vy: 0, vz: 0, flowQ: 0 }, autonomy: "full", tel: { vbat: 4.0, sticks: { thr: 0.5 } }, mask: 0, out: { thr: 0.5 },
  videoDelay: 0, safety: null, flying: true, requests: [], isFlying() { return this.flying; }, askPilot(kind) { this.requests.push(kind); },
});

// A camera at pose P (H) and a detection whose box puts its feet `seen` m away toward T (box straight from the
// controller's own camera geometry, so rangeOf() reads back `seen`).
function sighting(P, T, seen = Math.hypot(T[0] - P.x, T[1] - P.y)) {
  const settings = { get: (k) => ({ hfov: 127, uptilt: 20 })[k], all: () => ({ hfov: 127, uptilt: 20 }) };
  const ctl = { settings, videoDelay: 0, est: { pitchAngle: 0 }, perception: { latest: { t: 0, width: 4, height: 3 } } };
  ctl.bearingOf = FlightController.prototype.bearingOf;
  ctl.elevationOf = FlightController.prototype.elevationOf;
  const DEG = Math.PI / 180, vfov = (127 * DEG * 3) / 4, yaw = Math.atan2(T[1] - P.y, T[0] - P.x);
  const cx = 0.5 + (P.yaw - yaw) / (127 * DEG), hc = P.z + 0.025 - (map.floorAt(T[0], T[1]) ?? 0);
  const bottom = 0.5 - (-Math.atan2(hc, seen) - 20 * DEG) / vfov, h = (2 * Math.atan2(0.85, seen)) / vfov;
  const det = { label: "person", score: 0.9, box: { x: cx - 0.02, y: bottom - h, w: 0.04, h } };
  return locate(det, { ctl, localizer: { poseAt: () => P }, map, t: 0 });
}

// A detection of a standing person (1.7 m) at T seen from camera pose P, with the controller's own camera geometry; tall
// scales the person (a child: 1.1).
function personAt(P, T, trackId, tall = 1.7) {
  const DEG = Math.PI / 180, vfov = (127 * DEG * 3) / 4, yaw = Math.atan2(T[1] - P.y, T[0] - P.x), R = Math.hypot(T[0] - P.x, T[1] - P.y);
  const cx = 0.5 + (P.yaw - yaw) / (127 * DEG), hc = P.z + 0.025 - (map.floorAt(T[0], T[1]) ?? 0);
  const bottom = 0.5 - (-Math.atan2(hc, R) - 20 * DEG) / vfov, top = 0.5 - (Math.atan2(tall - hc, R) - 20 * DEG) / vfov, w = (2 * Math.atan2(0.25, R)) / (127 * DEG);
  return { label: "person", score: 0.9, trackId, box: { x: cx - w / 2, y: top, w, h: bottom - top } };
}

// The map's free run along a straight ray (what locate() used to cut every sighting at).
function straightFree(P, T) {
  const a = Math.atan2(T[1] - P.y, T[0] - P.x), R = Math.hypot(T[0] - P.x, T[1] - P.y);
  for (let d = 0.1; d < R; d += map.o.cell / 2) {
    const k = map.idx(P.x + d * Math.cos(a), P.y + d * Math.sin(a));
    if (k < 0 || map.room[k] < 0 || map.wall[k]) return d - 0.1;
  }
  return R;
}

const tests = {
  "simulated co-pilot: between commands its thumbs stop the drift, never enough to take over from the AI"() {
    for (const seed of [1, 2, 3, 4, 5, 6]) {
      const on = hoverDrift(seed), off = hoverDrift(seed, { thumbs: false });
      console.log(`      seed ${seed}: ${on.drift.toFixed(2)} m in 30 s (throttle only: ${off.drift.toFixed(2)} m), stick ≤ ${on.stick.toFixed(2)}`);
      assert.ok(on.airborne && !on.crashed, `seed ${seed}: still hovering`);
      assert.ok(on.drift < 1 && on.vmax < 0.15, `seed ${seed}: drifted ${on.drift.toFixed(2)} m, up to ${on.vmax.toFixed(2)} m/s`);
      assert.ok(on.stick < 0.29 && !on.override, `seed ${seed}: stick ${on.stick.toFixed(2)} would take over from the AI`);
      assert.ok(off.drift > 2 * on.drift, `seed ${seed}: the thumbs made the difference (${off.drift.toFixed(2)} vs ${on.drift.toFixed(2)} m)`);
    }
  },

  "safety: detached it neither trips nor asks the pilot, and keeps no stale state for when it is attached again"() {
    const ctl = fakeCtl(), loc = Object.assign(new Emitter(), { ctl, pose: () => lost, velocity: () => [0, 0] });
    const lost = { x: 1, y: 1, z: 1, yaw: 0, sigma: 0.8, status: "lost" }, trips = [];
    const safety = new Safety({ map, localizer: loc, settings: null, house });
    safety.on("trip", (t) => trips.push(t.kind));
    loc.emit("pose", lost);
    assert.deepEqual(trips, []);
    assert.equal(safety.lostSince, null);
    assert.equal(safety.takeover, null);
    ctl.safety = safety;
    loc.emit("pose", lost);
    assert.deepEqual(trips, ["lost"]);
    assert.ok(safety.takeover && safety.lostSince != null);
    ctl.safety = null; // e.g. the simulator switched to the demo apartment mid-flight
    loc.emit("pose", lost);
    assert.equal(safety.takeover, null);
    assert.equal(safety.lostSince, null);
    assert.equal(safety.tripped.size, 0);
    ctl.safety = safety;
    loc.emit("pose", lost);
    assert.deepEqual(trips, ["lost", "lost"], "attached again: a fresh trip, not one left from before");
    assert.ok(performance.now() - safety.lostSince < 100, "the lost timer starts again");
  },

  "safety: the sag-compensated battery keeps tracking while detached (the missions panel's budget)"() {
    const ctl = fakeCtl(), loc = Object.assign(new Emitter(), { ctl, pose: () => ({ status: "ok" }) });
    const safety = new Safety({ map, localizer: loc, settings: null, house });
    for (let i = 0; i < 300; i++) loc.emit("pose", { x: 1, y: 1, z: 1, yaw: 0, sigma: 0.05, status: "ok" });
    assert.ok(safety.vComp > 4.0, `vComp ${safety.vComp}`);
    ctl.tel.vbat = 3.6;
    for (let i = 0; i < 300; i++) loc.emit("pose", { x: 1, y: 1, z: 1, yaw: 0, sigma: 0.05, status: "ok" });
    assert.ok(safety.vComp < 4.0, `vComp follows the pack: ${safety.vComp}`);
  },

  "safety: a low battery doesn't turn a landing the pilot asked for into a flight home"() {
    const ctl = fakeCtl(), pose = { x: house.home.x + 2, y: house.home.y + 1.5, z: 1, yaw: 0, sigma: 0.05, status: "ok" };
    const loc = Object.assign(new Emitter(), { ctl, pose: () => pose, velocity: () => [0, 0] }), trips = [];
    const safety = new Safety({ map, localizer: loc, settings: null, house });
    safety.on("trip", (t) => trips.push(t.kind));
    Object.assign(ctl, { safety, behavior: { name: "land" }, tel: { vbat: 3.6, sticks: { thr: 0 } } });
    for (let i = 0; i < 5; i++) loc.emit("pose", pose);
    assert.deepEqual(trips, []);
    ctl.behavior = null;
    loc.emit("pose", pose);
    assert.deepEqual(trips, ["battery-home"], "hovering, it still flies home");
  },

  "localizer: forget() (switching to the real drone) makes the position unknown, and the pad brings it back"() {
    const ctl = fakeCtl();
    ctl.flying = false;
    const loc = new Localizer({ ctl, map, house }), poses = [];
    loc.on("pose", (p) => poses.push(p.status));
    loc.reset({ x: house.home.x, y: house.home.y, yaw: house.home.yaw });
    assert.equal(loc.pose().status, "ok");
    loc.flown = true;
    loc.forget();
    assert.equal(loc.known, false);
    assert.equal(loc.pose().status, "lost");
    assert.equal(loc.flown, true, "flown stays: missions still ask for the pad after a flight");
    assert.deepEqual(poses, ["ok", "lost"], "the page re-checks the safety layer on the pose event");
    loc.reset({ x: house.home.x, y: house.home.y, yaw: house.home.yaw });
    assert.equal(loc.pose().status, "ok");
  },

  "locate: someone seen past a wall the map has right in front of the drone (an arch) stays where they are"() {
    const pad = house.home, P = { x: pad.x, y: pad.y, z: (map.floorAt(pad.x, pad.y) ?? 0) + 1, yaw: Math.atan2(3.6 - pad.y, 4.3 - pad.x) }, T = [4.3, 3.6];
    const R = Math.hypot(T[0] - P.x, T[1] - P.y), wall = straightFree(P, T);
    assert.ok(wall < 0.35 * R, `precondition: the map has a wall ${wall.toFixed(2)} m along a ${R.toFixed(2)} m sight line`);
    const s = sighting(P, T);
    console.log(`      seen ${R.toFixed(2)} m away past a mapped wall ${wall.toFixed(2)} m out: placed ${s.range.toFixed(2)} m out in ${s.room}`);
    assert.ok(Math.abs(s.range - R) < 0.1 * R, `range ${s.range.toFixed(2)} (the wall cut would give ${wall.toFixed(2)})`);
    assert.equal(s.room, map.roomAt(...T).id);
  },

  "locate: a range overestimated past the room's far wall is still pulled back inside the room"() {
    const r2 = house.rooms.find((r) => r.name === "Room 2"), x0 = Math.min(...r2.outline.map((p) => p[0])), y = 1.5;
    const P = { x: x0 + 1.8, y, z: (map.floorAt(x0 + 1.8, y) ?? 0) + 1, yaw: Math.PI }, T = [x0 + 0.4, y]; // facing the west wall
    const s = sighting(P, T, 2.4); // really 1.4 m away, read as 2.4 (past the wall at 1.8)
    console.log(`      read 2.40 m toward a wall ${(P.x - x0).toFixed(2)} m away: placed ${s.range.toFixed(2)} m out in ${s.room}`);
    assert.ok(s.range < P.x - x0 && s.x > x0, `placed at x ${s.x.toFixed(2)}, wall at ${x0.toFixed(2)}`);
    assert.equal(s.room, r2.id);
  },

  "locate: seen through that wall but ranged past the house, it is pulled back to the last room on the line"() {
    const pad = house.home, P = { x: pad.x, y: pad.y, z: (map.floorAt(pad.x, pad.y) ?? 0) + 1, yaw: Math.atan2(3.6 - pad.y, 4.3 - pad.x) }, T = [4.3, 3.6];
    const s = sighting(P, T, 9); // a seated person's head read as a standing one's: far too far
    console.log(`      read 9.00 m through the wall: placed ${s.range.toFixed(2)} m out at (${s.x.toFixed(2)}, ${s.y.toFixed(2)}) in ${s.room}`);
    assert.equal(s.room, map.roomAt(...T).id);
    assert.ok(s.range > Math.hypot(T[0] - P.x, T[1] - P.y), "past where they really are, but inside the house");
  },

  "missions: one finding per tracked person or pet, however far apart its sightings land; what a stopped mission found"() {
    const ctl = Object.assign(fakeCtl(), { abort() {} });
    const runner = new MissionRunner({ ctl, map, house, localizer: { pose: () => ({}) }, perception: { snapshot: () => null }, alerts: null, settings: null });
    Object.assign(runner, { findings: [], t0: performance.now() });
    const found = [];
    runner.on("finding", (f) => found.push(f));
    const sp = { labels: ["person", "cat"] }, [x, y] = [house.home.x, house.home.y + 1];
    runner.addFinding({ label: "person", x, y, score: 0.9, trackId: "person-1" }, sp);
    runner.addFinding({ label: "person", x: x + 1.6, y, score: 0.9, trackId: "person-1" }, sp); // the same person, ranged worse
    runner.addFinding({ label: "person", x: x + 1.6, y: y + 0.2, score: 0.9, trackId: "person-2" }, sp);
    runner.addFinding({ label: "cat", x, y, score: 0.9 }, sp);
    runner.addFinding({ label: "cat", x: x + 0.5, y, score: 0.9 }, sp); // no track: within MISSION.near is the same cat
    assert.deepEqual(found.map((f) => [f.label, f.trackId]), [["person", "person-1"], ["person", "person-2"], ["cat", undefined]]);
    assert.match(runner.sofar(), /^ Before that I found a person in .+, the cat in .+\.$/);
    runner.findings = [];
    assert.equal(runner.sofar(), "");
  },

  // Wave B (e2e item 5): with the real detector a person's track id can change mid-flight, and a jump of more than 1 m
  // made a second finding of the same person. Wave B code, sequence "renumbered": 2 findings.
  async "missions: a person the detector renumbers (new id, 1.4 m off) is one finding; two seen together, or one much smaller, stay two"() {
    const settings = { get: (k) => ({ hfov: 127, uptilt: 20 })[k], all: () => ({ hfov: 127, uptilt: 20 }) };
    const perception = { latest: { t: 0, detections: [], width: 4, height: 3 }, frameAge: 0, snapshot: () => null };
    const ctl = Object.assign(fakeCtl(), { settings, perception, est: { pitchAngle: 0 }, abort() {} });
    [ctl.bearingOf, ctl.elevationOf] = [FlightController.prototype.bearingOf, FlightController.prototype.elevationOf];
    const P = { x: 1.2, y: 2.0, z: (map.floorAt(1.2, 2.0) ?? 0) + 1, yaw: 0 }, A = [3.6, 1.9], B = [3.7, 3.3]; // the living room, facing east
    const run = async (frames) => {
      const runner = new MissionRunner({ ctl, map, house, localizer: { poseAt: () => P, pose: () => ({ ...P, status: "ok" }) }, perception, settings: null });
      Object.assign(runner, { findings: [], t0: 0 });
      let done;
      const search = runner.searching(["person"], {}, () => new Promise((r) => (done = r)));
      frames.forEach((dets, i) => {
        perception.latest = { t: 1000 + 100 * i, detections: dets, width: 4, height: 3 };
        runner.spot();
      });
      done();
      await search;
      runner.unsub();
      return runner.findings;
    };
    const six = (fn) => Array.from({ length: 6 }, fn);
    const renumbered = await run([...six(() => [personAt(P, A, "p1")]), ...six(() => [personAt(P, B, "p7")])]);
    const together = await run(six(() => [personAt(P, A, "a"), personAt(P, B, "b")]));
    const child = await run([...six(() => [personAt(P, A, "p1")]), ...six(() => [personAt(P, B, "p9", 1.0)])]);
    console.log(`      renumbered: ${renumbered.length} finding(s) (${renumbered.map((f) => [f.trackId, ...(f.aliases ?? [])].join("=")).join(", ")}); two together: ${together.length}; then a child where the adult was: ${child.length}`);
    assert.equal(renumbered.length, 1);
    assert.deepEqual(renumbered[0].aliases, ["p7"]);
    assert.equal(together.length, 2);
    assert.equal(child.length, 2);
  },

  "simulator: ground-truth boxes carry the actor as their track"() {
    const sim = new Simulator({ headless: true, house, map, seed: 3 });
    const gt = sim.world.actorSpecs().map((a) => ({ id: a.id, kind: a.kind, visibleFraction: 1, visibleBox: { x: 0.4, y: 0.4, w: 0.1, h: 0.2 } }));
    const dets = sim.oracleDetections(gt, { x: 0, y: 0 });
    assert.ok(dets.length >= 2);
    for (const [i, d] of dets.entries()) assert.equal(d.trackId, gt[i].id);
  },

  "npm test runs every suite in tools/"() {
    const dir = import.meta.dirname, line = JSON.parse(fs.readFileSync(path.join(dir, "package.json"), "utf8")).scripts.test;
    const suites = fs.readdirSync(dir).filter((f) => /^test-.*\.mjs$/.test(f));
    for (const f of suites) assert.ok(line.includes(`node ${f}`), `${f} isn't in npm test`);
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
