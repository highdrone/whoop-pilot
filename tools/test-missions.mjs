// Missions on the house map, headless, in the capture fixture's house (fixtures/house) with the simulator's house world
// (sim/twin-world.js) and the real controller, behaviors, planner, localizer (nav/localizer.js), safety layer and
// MissionRunner (missions.js). Detections come from test-sim's FakePerception (ground-truth boxes, hidden behind walls and
// furniture). Localization is the simulator's truth unless a test says otherwise.
// Acceptance (docs/HOME-DRONE.md, research P2-P5): random goTo routes, patrol, person search, odometry bias toward a
// wall, battery return-home, a person walking into the path, localization loss; plus the localizer (odometry drift
// against σ, delayed fixes replayed), doorway transit, look-in, pets, co-pilot, alerts and the Claude tools. Wave C2: vision
// fixes stopping mid-scan by a wall (the localizer honest, the safety layer slowing, holding, landing before contact), the
// real drone's slower budgets with a 240 s pack, and seated people passed slowly at the smaller standoff. Fix 1: vision
// fixes only away from a blank wall (the heading skipped, a turn back, no landing); a patrol that couldn't land home says so.
// Fix 2: fixes that stop with the heading unchanged: a relocalization asked for at once, a turn to see more of the room;
// the landing text when it couldn't get home says where it really landed. Fix 3: the flow's climb rate misreading (half
// the climb, noise, +0.8 m/s bursts) with vision fixes: the height estimate follows the fixes and the drone keeps its height.
// Usage: cd tools && node test-missions.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { rig, now, runTests } from "./test-sim.mjs";

const I = await import("../app/js/house/import.js");
const P = await import("../app/js/house/planner.js");
const { Localizer } = await import("../app/js/nav/localizer.js");
const { Safety } = await import("../app/js/safety.js");
const { MissionRunner, findRoom, findLandmark, coverageFrom } = await import("../app/js/missions.js");
const { Alerts } = await import("../app/js/alerts.js");
const { ToolBox, HOUSE_TOOL_DEFS, TOOL_DEFS, houseSummary } = await import("../app/js/tools.js");
const { parseCommand, LocalPlanner } = await import("../app/js/localplanner.js");
const { inPolygon } = await import("../app/js/house/homemap.js");
const { mulberry32 } = await import("../app/js/sim/world.js");
const { HouseMemory } = await import("../app/js/memory/memory.js");
const { DEG } = await import("../app/js/util.js");
const { ScanAt, Hold, ReturnHome } = await import("../app/js/behaviors.js");
const { SAFETY } = await import("../app/js/safety.js");

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const { house, map } = await I.importCapture(fsSource(FIXTURE));
const HOME = house.home;
const tick = () => new Promise((r) => setImmediate(r));

// The simulator in the house with the whole autonomy stack. People and pets are taken out unless `keep` names them.
function stack({ autonomy = "full", seed = 7, videoDelay = 0, source = "truth", keep = [], options = {}, settings: extra = {}, home = house, memory = null } = {}) {
  for (const id of [...(map.temps?.keys() ?? [])]) map.removeTemp(id); // a fresh flight: no one is remembered on the shared map
  const r = rig({ house: home, map, autonomy, seed, videoDelay });
  const { sim, ctl, values } = r;
  Object.assign(values, extra);
  sim.world.actors.list = sim.world.actors.list.filter((a) => keep.includes(a.id));
  const settings = ctl.settings;
  const localizer = new Localizer({ ctl, map, house: home, settings });
  localizer.setSource(source);
  localizer.setTruth(() => ({ x: sim.drone.x, y: sim.drone.y, z: sim.drone.z, yaw: sim.drone.yaw }));
  const pushes = [];
  const alerts = new Alerts({ settings, speak: () => {}, fetch: async (url, init) => (pushes.push({ url, init }), new Response("{}")), Notification: undefined, indexedDB: undefined });
  const safety = new Safety({ map, localizer, settings, house: home, alerts });
  ctl.safety = safety;
  const missions = new MissionRunner({ ctl, map, house: home, localizer, perception: r.perception, alerts, settings, options, memory });
  const requests = [];
  ctl.on("pilot-request", (q) => requests.push(q.kind));
  const watch = { contacts: 0, hardest: 0, keepout: 0, minClear: Infinity, steps: 0, onStep: null };
  const collide = sim.world.collide.bind(sim.world);
  sim.world.collide = (body, rad, h) => {
    const v = collide(body, rad, h);
    if (v > 0 && body === sim.drone) {
      watch.contacts++;
      watch.hardest = Math.max(watch.hardest, v);
      if (process.env.DEBUG) console.log(`      contact ${fmt(v)} m/s at (${fmt(body.x)}, ${fmt(body.y)}, ${fmt(body.z - body.floorZ)} m up) ${ctl.behavior?.label}, est (${fmt(localizer.pose().x)}, ${fmt(localizer.pose().y)}) σ ${fmt(localizer.pose().sigma)} ${safety.reason}`);
    }
    return v;
  };
  // Steps the world until `promise` settles (or maxSeconds), checking keep-outs every step.
  const drive = async (promise, maxSeconds = 300) => {
    let out = null, err = null;
    promise.then((v) => (out = { v }), (e) => (err = e));
    for (let t = 0; t < maxSeconds && !out && !err; t += 1 / 60) {
      r.wait(1 / 60);
      watch.steps++;
      const d = sim.drone;
      if (d.airborne && inKeepout(d)) {
        watch.keepout++;
        if (process.env.DEBUG) console.log(`      keep-out at (${fmt(d.x)}, ${fmt(d.y)}, z ${fmt(d.z)}) ${ctl.behavior?.label}`);
      }
      watch.onStep?.(t);
      await tick();
    }
    if (err) throw err;
    return out ? out.v : { ok: false, summary: "test timeout", timeout: true, findings: [] };
  };
  return { ...r, settings, localizer, safety, missions, alerts, pushes, watch, drive, requests };
}

const inKeepout = (d) =>
  map.keepouts.some((k) => d.z >= (k.zMin ?? -Infinity) && d.z <= (k.zMax ?? Infinity) && (k.polygon ? inPolygon(d.x, d.y, k.polygon) : Math.hypot(d.x - k.x, d.y - k.y) <= k.r));

// A random free spot (comfortably inside the planner's free space) in a random room.
function randomSpot(rand, { margin = 0.1, room } = {}) {
  const cl = map.clear[map.bandOf(1.0)], need = map.lethal(0.17) + margin;
  for (;;) {
    const k = Math.floor(rand() * map.N);
    if (map.room[k] < 0 || cl[k] < need || (room && map.rooms[map.room[k]].id !== room)) continue;
    return map.center(k % map.W, (k / map.W) | 0);
  }
}

const fmt = (v) => v.toFixed(2);
const forever = () => new Promise(() => {});
const truthOf = (sim) => ({ x: sim.drone.x, y: sim.drone.y, z: sim.drone.z, yaw: sim.drone.yaw });
// Free for the drone at its height, or within its radius of a spot that is.
const nearFree = (d) => !!P.nearestFree(map, [d.x, d.y], { band: map.bandOf(Math.max(0.3, d.z - (map.floorAt(d.x, d.y) ?? 0))), sigma: 0, maxR: map.o.droneRadius });

const tests = {
  async "goTo: 20 random routes arrive within 0.2 m (>= 95%), no contacts, no keep-out entries"() {
    for (const videoDelay of [0, 250]) {
      const s = stack({ seed: 11 + videoDelay, videoDelay }), rand = mulberry32(5 + videoDelay), errs = [], times = [];
      for (let i = 0; i < 20; i++) {
        s.sim.drone.soc = 1;
        const goal = randomSpot(rand), t0 = now();
        const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 150);
        const d = s.sim.drone, err = Math.hypot(d.x - goal[0], d.y - goal[1]);
        errs.push(r.ok ? err : Infinity);
        times.push((now() - t0) / 1000);
        if (!r.ok || err > 0.2) console.log(`      route ${i} to (${goal.map(fmt)}): ${r.summary} err ${fmt(err)}`);
      }
      const ok = errs.filter((e) => e <= 0.2).length;
      console.log(`      ${videoDelay} ms video delay: ${ok}/20 within 0.2 m (worst ${fmt(Math.max(...errs))} m), ${s.watch.contacts} contacts, ${s.watch.keepout} keep-out steps, ${fmt(times.reduce((a, b) => a + b) / 20)} s per route`);
      assert.ok(ok >= 19, `${ok}/20 arrived within 0.2 m`);
      assert.equal(s.watch.contacts, 0, "wall contacts");
      assert.equal(s.watch.keepout, 0, "keep-out entries");
      assert.ok(!s.sim.drone.crashed);
    }
  },

  async "patrol: every room in <= 120 s, landing within 0.3 m of the home pad"() {
    const s = stack({ seed: 3 });
    const t0 = now(), r = await s.drive(s.missions.run({ kind: "patrol" }), 200), secs = (now() - t0) / 1000;
    const d = s.sim.drone, err = Math.hypot(d.x - HOME.x, d.y - HOME.y);
    console.log(`      "${r.summary}" in ${secs.toFixed(0)} s, ${fmt(err)} m from the pad, ${s.watch.contacts} contacts, battery ${Math.round(d.soc * 100)}%`);
    assert.ok(r.ok, r.summary);
    assert.ok(secs <= 120, `${secs.toFixed(0)} s`);
    assert.ok(err <= 0.3, `${fmt(err)} m from home`);
    assert.ok(!d.airborne && !d.crashed, "landed");
    assert.equal(s.watch.contacts, 0);
    assert.equal(s.watch.keepout, 0);
    for (const rm of house.rooms) assert.ok(r.summary.includes(rm.name), `visited ${rm.name}`);
  },

  async "searchFor: a randomly placed standing person is found (>= 95%) within 150 s, right room, <= 1 m"() {
    const rand = mulberry32(99), out = [];
    for (let i = 0; i < 20; i++) {
      const s = stack({ seed: 20 + i, keep: ["person1"] }), a = s.sim.world.actors.list[0];
      const spot = randomSpot(rand, { margin: 0.05 });
      a.place(spot[0], spot[1], "stand", 1e6);
      const t0 = now(), r = await s.drive(s.missions.run({ kind: "searchFor", target: "person" }), 200), secs = (now() - t0) / 1000;
      const f = r.findings.find((x) => x.label === "person"), room = map.roomAt(a.x, a.y).id;
      const ok = !!f && secs <= 150 && f.room === room && Math.hypot(f.x - a.x, f.y - a.y) <= 1;
      out.push({ ok, secs, err: f ? Math.hypot(f.x - a.x, f.y - a.y) : null });
      if (!ok) console.log(`      trial ${i}: person at (${spot.map(fmt)}) in ${room}: "${r.summary}" ${f ? `at (${fmt(f.x)}, ${fmt(f.y)}) in ${f.room}` : ""} after ${secs.toFixed(0)} s`);
      assert.equal(s.watch.contacts, 0);
    }
    const ok = out.filter((o) => o.ok);
    console.log(`      ${ok.length}/20 found in time in the right room within 1 m; ${fmt(Math.max(...ok.map((o) => o.secs)))} s worst, position error ${fmt(Math.max(...ok.map((o) => o.err)))} m worst`);
    assert.ok(ok.length >= 19, `${ok.length}/20`);
  },

  async "safety: a biased flow pushing the drone toward a wall stops it before contact"() {
    for (const bias of [0.15, 0.3]) {
      const s = stack({ seed: 4 });
      const goal = [-0.65, 2.4]; // Room 2, about 0.9 m from its west wall
      await s.drive(s.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 60);
      // From now on the flow says the drone drifts east (+x): the controller rolls west to cancel it, toward the wall.
      const per = s.perception, update = per.update.bind(per), hfov = s.values.hfov * DEG;
      per.update = () => {
        update();
        const f = per.latest.flow, d = s.sim.drone;
        if (!f) return;
        // a fake velocity of `bias` m/s toward +x in H, in the body frame (forward, right), as image motion at ~2.2 m depth
        const fwd = bias * Math.cos(d.yaw), right = bias * Math.sin(d.yaw);
        f.dx -= right / (hfov * 2.2);
        f.div += fwd / 2.2;
      };
      let minClear = Infinity;
      s.watch.onStep = () => (minClear = Math.min(minClear, map.clearance(s.sim.drone.x, s.sim.drone.y)));
      await s.drive(s.missions.run({ kind: "lookIn", room: "Room 2" }), 40);
      await s.drive(new Promise(() => {}), 15);
      console.log(`      flow bias ${bias} m/s: closest ${fmt(minClear)} m to the map's obstacles, ${s.watch.contacts} contacts; safety: ${s.safety.status().reason || "ok"}`);
      assert.equal(s.watch.contacts, 0, "contact");
      assert.ok(minClear > 0.03, `${fmt(minClear)} m`);
    }
  },

  async "safety: battery low flies home with >= 15% left"() {
    const s = stack({ seed: 8 });
    s.sim.drone.soc = 0.45;
    let trip = null;
    s.safety.on("trip", (t) => t.kind === "battery-home" && !trip && (trip = { ...t, soc: s.sim.drone.soc, at: now() }));
    const r = await s.drive(s.missions.run({ kind: "patrol" }), 300);
    await s.drive(new Promise(() => {}), 60);
    const d = s.sim.drone, err = Math.hypot(d.x - HOME.x, d.y - HOME.y);
    console.log(`      "${r.summary}"; ${trip ? `return-home at ${Math.round(trip.soc * 100)}%: ${trip.reason}` : "no trip"}; landed ${!d.airborne} ${fmt(err)} m from the pad with ${Math.round(d.soc * 100)}%`);
    assert.ok(trip, "battery return-home");
    assert.ok(trip.soc >= 0.15, `tripped at ${trip.soc}`);
    assert.ok(!d.airborne && !d.crashed && err < 0.5, "landed home");
    assert.ok(d.soc >= 0.1, `${d.soc} left on the pad`);
  },

  async "safety: a person walking into the path: the drone keeps at least 1 m from them (holds or goes around) and arrives"() {
    for (const videoDelay of [0, 250]) {
      const s = stack({ seed: 5, videoDelay, keep: ["person1"] }), a = s.sim.world.actors.list[0];
      // Waiting 2 m north of the route (Wave B had them at (3.4, 2.7), whose no-fly zone now sends the drone around the
      // south side, where furniture hides them until they step out 0.6 m away). The drone must come to rest at least 1 m
      // short and never fly at the person inside 1 m; the person may still walk past the hovering drone closer than that.
      a.place(3.4, 3.4, "stand", 1e6);
      let walking = false, minD = Infinity, stopped = 0, passed = Infinity, rest = null;
      s.watch.onStep = () => {
        const d = s.sim.drone, dist = Math.hypot(d.x - a.x, d.y - a.y), closing = (d.vx * (a.x - d.x) + d.vy * (a.y - d.y)) / dist;
        if (!walking && d.x > 1.4) {
          walking = true; // across open floor in front of the drone at 0.8 m/s, ending behind it
          Object.assign(a, { path: [[3.4, 3.4, 0], [3.4, 0.5, 0], [0.9, 0.4, 0]], i: 0, goal: { then: "stand" }, state: "walk" });
        }
        if (walking && closing > 0.2) minD = Math.min(minD, dist);
        if (walking) passed = Math.min(passed, dist);
        if (s.safety.reason === "a person is close") stopped++;
        if (stopped && rest == null && Math.hypot(d.vx, d.vy) < 0.1) rest = dist;
      };
      const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: 4.1, y: 1.2 } }), 120);
      console.log(`      ${videoDelay} ms: ${rest == null ? "never had to stop" : `stopped ${fmt(rest)} m from the person`}, ${fmt(minD)} m at the closest while flying toward them (${fmt(passed)} m as they walked past), held ${(stopped / 60).toFixed(1)} s, then "${r.summary}"`);
      assert.ok(walking, "the person crossed");
      assert.ok((rest ?? Infinity) >= 1.0 && minD >= 1.0, `stopped ${fmt(rest ?? NaN)} m, closest flying toward ${fmt(minD)} m`);
      assert.ok(r.ok, r.summary);
      assert.equal(s.watch.contacts, 0);
    }
  },

  // Wave B gave up here ("A person is too close to go on.", closest 1.90 / 1.72 m standing at 0 / 250 ms, 1.90 / 1.88 m
  // lying); now the person becomes a no-fly zone the planner goes around. Someone seated or lying still: 1.0 m, and within
  // 1.5 m of them the drone creeps (wave C2: the 1.5 m zone closed whole rooms).
  async "safety: someone standing, sitting or lying on the path: the drone keeps 1.5 m from someone standing (1.0 m, creeping, from someone seated or lying still) and goes around them (or says a person blocks the way)"() {
    for (const pose of ["stand", "sit", "lie"])
      for (const videoDelay of [0, 250]) {
        const s = stack({ seed: 5, videoDelay, keep: ["person1"] }), a = s.sim.world.actors.list[0], keep = pose === "stand" ? SAFETY.standoff : SAFETY.seated;
        a.place(2.16, 1.58, pose, 1e6); // on the way to (4.1, 1.2), on the open floor
        let minD = Infinity, fast = 0;
        s.watch.onStep = () => {
          const d = s.sim.drone, L = Math.hypot(d.x - a.x, d.y - a.y);
          if (!d.airborne) return;
          minD = Math.min(minD, L);
          if (L < SAFETY.standoff - 0.1) fast = Math.max(fast, Math.hypot(d.vx, d.vy));
        };
        const t0 = now(), r = await s.drive(s.missions.run({ kind: "goTo", target: { x: 4.1, y: 1.2 } }), 120), d = s.sim.drone;
        console.log(`      ${pose} ${videoDelay} ms: closest ${fmt(minD)} m${fast ? ` (at most ${fmt(fast)} m/s within 1.4 m)` : ""}, "${r.summary}" in ${((now() - t0) / 1000).toFixed(0)} s, ${fmt(Math.hypot(d.x - 4.1, d.y - 1.2))} m from the goal`);
        assert.ok(minD >= keep, `${pose}, ${videoDelay} ms: came within ${fmt(minD)} m`);
        assert.ok(fast <= SAFETY.creepNear + 0.08, `${pose}: ${fmt(fast)} m/s within the standoff`);
        assert.ok(r.ok ? Math.hypot(d.x - 4.1, d.y - 1.2) < 0.3 : /person/.test(r.summary), r.summary);
        assert.equal(s.watch.contacts, 0);
      }
  },

  async "safety: losing localization holds, then lands in place without leaving free space (full); co-pilot asks the pilot"() {
    for (const autonomy of ["full", "copilot"]) {
      const s = stack({ autonomy, seed: 6, source: "fused", videoDelay: 250 });
      s.localizer.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
      const rand = mulberry32(3), gauss = () => Math.sqrt(-2 * Math.log(Math.max(1e-9, rand()))) * Math.cos(2 * Math.PI * rand());
      // splat-style fixes: every 0.5 s, 5 cm noise, 250 ms late
      let fixes = true, nextFix = 0, pending = [], freeze = null, lostAt = null, outside = 0, trips = [];
      s.safety.on("trip", (t) => trips.push(t.kind));
      const update = s.perception.update.bind(s.perception);
      s.perception.update = () => (freeze && now() < freeze ? null : update());
      s.watch.onStep = () => {
        const d = s.sim.drone, t = now();
        if (fixes && t >= nextFix) {
          nextFix = t + 500;
          pending.push({ at: t + 250, fix: { x: d.x + 0.05 * gauss(), y: d.y + 0.05 * gauss(), yaw: d.yaw + 2 * DEG * gauss(), sigma: 0.05, t } });
        }
        while (pending.length && pending[0].at <= t) s.localizer.fix(pending.shift().fix);
        if (fixes && d.y > 1.3) [fixes, freeze, pending] = [false, t + 1200, []]; // on the way to the doorway: the fixes stop and the video freezes for 1.2 s
        if (!fixes && s.localizer.pose().status === "lost") lostAt ??= { x: d.x, y: d.y, t };
        if (lostAt && d.airborne && !nearFree(d)) outside++;
      };
      const r = await s.drive(s.missions.run({ kind: "goTo", target: "Living room" }), 120);
      await s.drive(forever(), 15);
      const d = s.sim.drone, moved = lostAt ? Math.hypot(d.x - lostAt.x, d.y - lostAt.y) : NaN;
      console.log(`      ${autonomy}: "${r.summary}"; lost ${lostAt ? "yes" : "no"}, trips ${trips.join(", ")}, pilot asked ${s.requests.join(", ") || "nothing"}; ${d.airborne ? "flying" : "on the ground"} ${fmt(moved)} m from where it got lost, ${outside} steps outside free space`);
      assert.ok(lostAt, "localization lost");
      assert.ok(!r.ok, "the mission stopped");
      assert.equal(outside, 0, "left free space");
      assert.ok(!d.crashed && s.watch.contacts === 0);
      if (autonomy === "full") {
        assert.ok(trips.includes("lost") && trips.includes("lost-land"), trips.join());
        assert.ok(!d.airborne, "landed");
        assert.ok(moved < 0.6, `drifted ${fmt(moved)} m while holding and landing`);
      } else assert.ok(s.requests.includes("land"), "asked the pilot");
    }
  },

  async "localizer: odometry drifts within its σ, fused fixes (250 ms late, replayed) stay within 0.3 m, outliers are rejected"() {
    const s = stack({ seed: 9, videoDelay: 250 });
    const odo = new Localizer({ ctl: s.ctl, map, house }), fused = new Localizer({ ctl: s.ctl, map, house });
    odo.setSource("odometry");
    for (const l of [odo, fused]) l.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
    const rand = mulberry32(4), gauss = () => Math.sqrt(-2 * Math.log(Math.max(1e-9, rand()))) * Math.cos(2 * Math.PI * rand());
    let nextFix = 0, pending = [], outlier = null;
    const stats = { odo: [], fused: [] };
    s.watch.onStep = () => {
      const d = s.sim.drone, t = now();
      if (t >= nextFix && d.airborne) {
        nextFix = t + 1000;
        pending.push({ at: t + 250, fix: { x: d.x + 0.1 * gauss(), y: d.y + 0.1 * gauss(), sigma: 0.1, t } });
      }
      while (pending.length && pending[0].at <= t) fused.fix(pending.shift().fix);
      if (!outlier && t > 20000) outlier = { used: fused.fix({ x: d.x + 3, y: d.y - 2, sigma: 0.1, t: t - 100 }) };
      for (const [k, l] of [["odo", odo], ["fused", fused]]) {
        const p = l.pose();
        stats[k].push({ err: Math.hypot(p.x - d.x, p.y - d.y), sigma: p.sigma, status: p.status });
      }
    };
    await s.drive(s.missions.run({ kind: "goTo", target: "Living room" }), 90);
    await s.drive(s.missions.run({ kind: "goTo", target: "Room 3" }), 90);
    const o = stats.odo, f = stats.fused.slice(120);
    const within = o.filter((q) => q.err <= 2 * q.sigma + 0.05).length / o.length;
    console.log(`      odometry: error up to ${fmt(Math.max(...o.map((q) => q.err)))} m, σ ${fmt(o[0].sigma)} -> ${fmt(o.at(-1).sigma)} m (${o.at(-1).status}), within 2σ ${Math.round(within * 100)}% of the time, flow scale ${fmt(odo.scale)}`);
    console.log(`      fused: error up to ${fmt(Math.max(...f.map((q) => q.err)))} m (95% under ${fmt(f.map((q) => q.err).sort((a, b) => a - b)[Math.floor(f.length * 0.95)])} m), σ up to ${fmt(Math.max(...f.map((q) => q.sigma)))} m, ${fused.fixes.used} fixes used, ${fused.fixes.rejected} rejected, flow scale ${fmt(fused.scale)}`);
    assert.ok(within >= 0.9, `odometry error within 2σ only ${within}`);
    assert.ok(o.at(-1).sigma > o[0].sigma * 3, "σ grows with odometry");
    const e95 = f.map((q) => q.err).sort((a, b) => a - b)[Math.floor(f.length * 0.95)];
    assert.ok(e95 < 0.3 && Math.max(...f.map((q) => q.err)) < 0.45, `fused error: 95% ${fmt(e95)} m`);
    assert.ok(f.every((q) => q.status === "ok"), "fused stays ok");
    assert.equal(outlier.used, false, "a 3.6 m jump is rejected");
    assert.ok(fused.fixes.rejected >= 1);
    assert.equal(new Localizer({ ctl: s.ctl, map, house }).pose().status, "lost", "no pose before a reset or fix");
  },

  async "localizer: still on the pad σ stays put (no drift for 60 s); a co-pilot mission 30 s later starts with a good pose"() {
    const s = stack({ autonomy: "copilot", seed: 3, source: "fused" });
    s.localizer.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
    s.wait(60);
    const p = s.localizer.pose();
    console.log(`      after 60 s on the pad: σ ${fmt(p.sigma)} m (${p.status}), ${fmt(Math.hypot(p.x - HOME.x, p.y - HOME.y))} m from where it was set`);
    assert.ok(p.sigma < 0.1 && p.status === "ok" && Math.hypot(p.x - HOME.x, p.y - HOME.y) < 0.01, `σ ${p.sigma}`);
    const c = stack({ autonomy: "copilot", seed: 3, source: "fused" }), trips = [];
    c.localizer.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
    c.safety.on("trip", (t) => trips.push(t.kind));
    c.wait(30);
    let up = null, later = null;
    c.watch.onStep = () => c.sim.drone.airborne && ((up ??= { t: now(), p: c.localizer.pose() }), now() - up.t > 3000 && (later ??= c.localizer.pose()));
    await c.drive(c.missions.run({ kind: "goTo", target: "Room 3" }), 5); // its first seconds (odometry alone gets lost after a couple of metres)
    console.log(`      co-pilot, 30 s after the reset: σ ${fmt(up.p.sigma)} m (${up.p.status}) at take-off, ${fmt(later.sigma)} m (${later.status}) 3 s later`);
    assert.ok(up.p.status === "ok" && up.p.sigma < 0.1, `σ ${up.p.sigma} at take-off`);
    assert.ok(later.status !== "lost" && !trips.includes("lost"), trips.join());
  },

  async "localizer: the flow goes on the tick it describes; a fix 50 ms late at 250 ms video delay is still in the pose 400 ms later"() {
    const s = stack({ seed: 9, videoDelay: 250 });
    const A = new Localizer({ ctl: s.ctl, map, house }), B = new Localizer({ ctl: s.ctl, map, house });
    for (const l of [A, B]) l.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
    let shift = null, later = null, placed = [];
    const t0 = now(); // the clock runs on from the tests before
    s.watch.onStep = () => {
      const t = now(), flying = s.ctl.isFlying() && t - t0 > 9000, k = flying ? A.ticks.findLastIndex((tk) => tk.flows.length) : -1;
      if (k >= 0) placed.push(t - A.ticks[k].t); // how far back the newest flow reading went
      if (!shift && k >= 0) {
        const a0 = A.pose();
        assert.ok(A.fix({ x: a0.x + 0.15, y: a0.y, sigma: 0.03, t: t - 50 }));
        shift = { t, dx: A.pose().x - B.pose().x };
      } else if (shift && !later && t >= shift.t + 400) later = A.pose().x - B.pose().x;
    };
    await s.drive(s.missions.run({ kind: "goTo", target: "Living room" }), 12);
    const lag = placed.sort((a, b) => a - b)[placed.length >> 1];
    console.log(`      flow readings go ${lag.toFixed(0)} ms back (video 250 + smoothing 100); a +0.15 m fix moved the pose ${fmt(shift.dx)} m, ${fmt(later)} m 400 ms later`);
    assert.ok(lag >= 330 && lag <= 400, `flow placed ${lag} ms back`);
    assert.ok(shift.dx > 0.08 && later > 0.8 * shift.dx, `fix ${fmt(shift.dx)} m -> ${fmt(later)} m`);
  },

  async "fused localization: 10 routes with splat-style fixes (1 s, ±0.1 m in x, y and z, 250 ms late) arrive within 0.3 m (>= 90%)"() {
    const s = stack({ seed: 21, videoDelay: 250, source: "fused" }), rand = mulberry32(8);
    const gauss = () => Math.sqrt(-2 * Math.log(Math.max(1e-9, rand()))) * Math.cos(2 * Math.PI * rand());
    s.localizer.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
    let nextFix = 0, pending = [];
    s.watch.onStep = () => {
      const d = s.sim.drone, t = now();
      if (t >= nextFix) {
        nextFix = t + 1000;
        pending.push({ at: t + 250, fix: { x: d.x + 0.1 * gauss(), y: d.y + 0.1 * gauss(), z: d.z + 0.1 * gauss(), yaw: d.yaw + 3 * DEG * gauss(), sigma: 0.1, t } });
      }
      while (pending.length && pending[0].at <= t) s.localizer.fix(pending.shift().fix);
    };
    const errs = [];
    for (let i = 0; i < 10; i++) {
      s.sim.drone.soc = 1;
      const goal = randomSpot(rand, { margin: 0.25 });
      const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: goal[0], y: goal[1] } }), 150);
      errs.push(r.ok ? Math.hypot(s.sim.drone.x - goal[0], s.sim.drone.y - goal[1]) : Infinity);
      if (!r.ok) console.log(`      route ${i}: ${r.summary}`);
    }
    const ok = errs.filter((e) => e <= 0.3).length;
    console.log(`      ${ok}/10 within 0.3 m (errors ${errs.map(fmt).join(", ")}), ${s.watch.contacts} contacts${s.watch.contacts ? ` (hardest ${fmt(s.watch.hardest)} m/s)` : ""}, ${s.localizer.fixes.used} fixes used, ${s.localizer.fixes.rejected} rejected`);
    assert.ok(ok >= 9, `${ok}/10`);
    assert.ok(s.watch.contacts <= 1 && s.watch.hardest < 0.5, "at most a gentle bump (the estimate is good to about ±0.2 m here)");
    assert.equal(s.watch.keepout, 0);
  },

  async "DoorTransit: lines up at the doorway, checks it, crosses at 0.25 m/s"() {
    const s = stack({ seed: 12, options: { narrowDoor: 2 } }), phases = [];
    s.missions.on("status", (st) => phases.push(st.phase));
    let transit = 0;
    s.watch.onStep = () => (transit += s.ctl.behavior?.name === "door" ? 1 : 0);
    const r = await s.drive(s.missions.run({ kind: "goTo", target: "Living room" }), 120);
    const d = s.sim.drone, goal = P.roomCenter(map, "r1", { sigma: 0.17 });
    console.log(`      "${r.summary}" with ${phases.filter((p) => p === "door").length} doorway transit(s), ${(transit / 60).toFixed(1)} s in them, ${fmt(Math.hypot(d.x - goal[0], d.y - goal[1]))} m from the goal`);
    assert.ok(r.ok, r.summary);
    assert.ok(phases.includes("door") && transit > 60, "used DoorTransit");
    assert.equal(s.watch.contacts, 0);
  },

  async "lookIn: from the doorway first, sees the person in the living room"() {
    const s = stack({ seed: 13, keep: ["person1"] }), a = s.sim.world.actors.list[0], texts = [];
    a.place(3.3, 2.2, "stand", 1e6);
    s.missions.on("status", (st) => texts.push(st.text));
    const r = await s.drive(s.missions.run({ kind: "lookIn", room: "living room" }), 120);
    const f = r.findings.find((x) => x.label === "person");
    console.log(`      "${r.summary}" (${r.frames?.length} frames; ${texts.filter((t) => /doorway|into/.test(t)).join(" / ")})${f ? `; person placed ${fmt(Math.hypot(f.x - a.x, f.y - a.y))} m off` : ""}`);
    assert.ok(r.ok, r.summary);
    assert.ok(texts.some((t) => /doorway of the Living room/.test(t)), "doorway first");
    assert.ok(f && f.room === "r1" && Math.hypot(f.x - a.x, f.y - a.y) <= 1, "found the person");
    assert.ok(r.frames.length >= 4 && r.frames.every((fr) => Number.isFinite(fr.yaw)));
    assert.ok(coverageFrom(map, "r1", [0.9, 2.3]) > 0.5);
  },

  async "checkOnPets: finds the cat and the dog in their rooms"() {
    const s = stack({ seed: 14, keep: ["cat", "dog"] });
    const [cat, dog] = ["cat", "dog"].map((id) => s.sim.world.actors.list.find((a) => a.id === id));
    cat.place(3.6, 1.0, "stand", 1e6);
    dog.place(-0.2, 4.3, "stand", 1e6);
    const t0 = now(), r = await s.drive(s.missions.run({ kind: "checkOnPets" }), 240);
    const at = (label) => r.findings.find((f) => f.label === label);
    console.log(`      "${r.summary}" in ${((now() - t0) / 1000).toFixed(0)} s; cat ${at("cat") ? fmt(Math.hypot(at("cat").x - cat.x, at("cat").y - cat.y)) : "-"} m off, dog ${at("dog") ? fmt(Math.hypot(at("dog").x - dog.x, at("dog").y - dog.y)) : "-"} m off`);
    assert.ok(at("cat")?.room === "r1" && at("dog")?.room === "r3", r.summary);
    assert.ok(Math.hypot(at("cat").x - cat.x, at("cat").y - cat.y) < 1.2 && Math.hypot(at("dog").x - dog.x, at("dog").y - dog.y) < 1.2);
    assert.equal(s.watch.contacts, 0);
  },

  async "co-pilot: the pilot takes off and holds height, the mission steers, and the pilot lands on request"() {
    const s = stack({ autonomy: "copilot", seed: 15 });
    const r = await s.drive(s.missions.run({ kind: "goTo", target: "Room 3" }), 120);
    const d = s.sim.drone, goal = P.roomCenter(map, "r3", { sigma: 0.17 }), agl = d.z - d.floorZ, off = Math.hypot(d.x - goal[0], d.y - goal[1]);
    const h = await s.drive(s.missions.run({ kind: "returnHome" }), 120);
    console.log(`      "${r.summary}" ${fmt(off)} m off at ${fmt(agl)} m; "${h.summary}"; asked: ${s.requests.join(", ")}`);
    assert.ok(off < 0.25, `${fmt(off)} m from the goal`);
    assert.ok(r.ok && h.ok, `${r.summary} / ${h.summary}`);
    assert.ok(s.requests[0] === "takeoff" && s.requests.includes("land"));
    assert.ok(agl > 0.7 && agl < 1.3, `height ${agl}`);
    assert.ok(!s.sim.drone.airborne, "landed by the pilot");
    assert.ok(Math.hypot(s.sim.drone.x - HOME.x, s.sim.drone.y - HOME.y) < 0.5);
  },

  async "missions refuse cleanly: unknown places and rooms, no home pad, and a stop() mid-flight"() {
    const s = stack({ seed: 16 });
    let r = await s.drive(s.missions.run({ kind: "goTo", target: "kitchen" }), 5);
    assert.ok(!r.ok && /don't know a place called "kitchen"/.test(r.summary) && /Living room/.test(r.summary), r.summary);
    r = await s.drive(s.missions.run({ kind: "lookIn", room: "garage" }), 5);
    assert.ok(!r.ok && /no room called/.test(r.summary), r.summary);
    const noHome = stack({ seed: 16, home: { ...structuredClone(house), home: null } });
    r = await noHome.drive(noHome.missions.run({ kind: "patrol" }), 5);
    assert.ok(!r.ok && /home pad/.test(r.summary), r.summary);
    const p = s.missions.run({ kind: "goTo", target: "Living room" });
    s.watch.onStep = (t) => t > 8 && s.missions.stop("test");
    r = await s.drive(p, 30);
    assert.ok(!r.ok && /Stopped/.test(r.summary), r.summary);
    assert.equal(findRoom(house, "the living room").id, "r1");
    assert.equal(findLandmark(house, "couch").name, "sofa");
    assert.equal(findLandmark(house, "fireplace").source, "ai");
  },

  async "alerts: speech, permission only from a click, a notification, ntfy push with the picture, the event log; nothing waits for an unanswered prompt"() {
    class Unanswered {
      static permission = "default";
      static requestPermission = () => new Promise(() => {}); // nobody at the Mac
    }
    const pushed = [], quiet = new Alerts({ settings: { get: (k) => ({ ntfyTopic: "whoop-x" })[k] }, fetch: async (u) => (pushed.push(u), new Response("{}")), Notification: Unanswered, indexedDB: undefined });
    const q = await Promise.race([quiet.notify({ text: "A person in the Living room.", urgency: "high" }), new Promise((r) => setTimeout(() => r("hung"), 500))]);
    assert.deepEqual(q, { spoke: false, notified: false, pushed: true, logged: true }, "push and log without waiting for the prompt");
    assert.equal(pushed.length, 1);
    const sent = [], said = [], asked = [], shown = [];
    class N {
      static permission = "default";
      static async requestPermission() {
        asked.push(1);
        return (N.permission = "granted");
      }
      constructor(title, o) {
        shown.push({ title, ...o });
      }
    }
    const settings = { values: { ntfyTopic: "whoop-test-topic", speak: true }, get: (k) => settings.values[k] };
    const alerts = new Alerts({ settings, speak: (t) => said.push(t), fetch: async (url, init) => (sent.push({ url, ...init }), new Response("{}")), Notification: N, indexedDB: undefined });
    const image = Buffer.from([0xff, 0xd8, 0xff, 0xd9]).toString("base64");
    assert.equal((await alerts.notify({ text: "Before asking.", urgency: "low" })).notified, false, "no prompt from notify()");
    assert.equal(asked.length, 0);
    assert.equal(await alerts.requestPermission(), "granted"); // the Settings button
    sent.length = said.length = 0;
    const a = await alerts.notify({ text: "A person in the Living room — near the sofa.", image, room: "r1", urgency: "high", pose: { x: 1, y: 2, yaw: 0.5, sigma: 0.1 } });
    const b = await alerts.notify({ text: "Patrol done.", urgency: "low" });
    assert.deepEqual(a, { spoke: true, notified: true, pushed: true, logged: true });
    assert.equal(asked.length, 1, "asked once, from the click");
    assert.equal(shown[0].body, "A person in the Living room — near the sofa.");
    assert.equal(sent[0].method, "PUT");
    assert.equal(sent[0].url, "https://ntfy.sh/whoop-test-topic");
    assert.equal(sent[0].headers.Filename, "whoop.jpg");
    assert.equal(sent[0].headers.Priority, "5");
    assert.match(sent[0].headers.Message, /^=\?UTF-8\?B\?/, "non-ASCII text is RFC 2047 encoded");
    assert.deepEqual([...sent[0].body], [0xff, 0xd8, 0xff, 0xd9]);
    assert.equal(sent[1].method, "POST");
    assert.equal(sent[1].body, "Patrol done.");
    assert.ok(b.pushed && said.length === 2);
    const ev = await alerts.events();
    assert.deepEqual(ev.map((e) => e.text), ["Patrol done.", "A person in the Living room — near the sofa.", "Before asking."]);
    assert.deepEqual(ev[1].pose, { x: 1, y: 2, yaw: 0.5, sigma: 0.1 });
    settings.values.ntfyTopic = "";
    assert.equal((await alerts.notify({ text: "x" })).pushed, false, "no topic, no push");
    const s = stack({ seed: 17, keep: ["person1"] });
    s.sim.world.actors.list[0].place(0.0, 4.6, "stand", 1e6);
    s.values.ntfyTopic = "whoop-test-topic";
    const r = await s.drive(s.missions.run({ kind: "patrol" }), 200);
    assert.ok(r.findings.some((f) => f.label === "person"), r.summary);
    assert.ok(s.pushes.some((p) => p.init.headers.Priority === "5"), "a person on patrol pushes an alert");
  },

  async "tools: with the map Claude gets missions and no hand flying; go_to, where_am_i, set_home, mark_landmark, notify"() {
    const home = structuredClone(house), saved = [];
    const s = stack({ seed: 18, home });
    const tools = new ToolBox({ ctl: s.ctl, perception: s.perception, settings: s.settings, speak: () => {}, alerts: s.alerts });
    assert.equal(tools.defs(), TOOL_DEFS);
    tools.setHouse({ house: home, map, missions: s.missions, localizer: s.localizer, save: async (h) => saved.push(h.home) });
    const names = tools.defs().map((t) => t.name);
    assert.equal(tools.defs(), HOUSE_TOOL_DEFS);
    for (const n of ["where_am_i", "go_to", "look_in", "search_for", "patrol", "return_home", "set_home", "mark_landmark", "notify", "find", "follow", "land"]) assert.ok(names.includes(n), n);
    for (const n of ["turn", "move", "fly_toward"]) {
      assert.ok(!names.includes(n), `${n} hidden with a map`);
      const r = await tools.call(n, { degrees: 90, direction: "forward", meters: 1, object: "door" }, undefined, { from: "claude" }); // asked for anyway
      assert.ok(r.isError && /isn't available while the house map is active/.test(r.text), r.text);
    }
    assert.ok(!s.sim.drone.airborne, "nothing flew");
    const go = await s.drive(tools.call("go_to", { place: "Living room" }), 120);
    assert.ok(!go.isError && /Arrived at the Living room/.test(go.text), go.text);
    const w = await tools.call("where_am_i", {});
    assert.match(w.text, /In the Living room.*m from the home pad/);
    const m = await tools.call("mark_landmark", { name: "Reading corner" });
    assert.match(m.text, /Saved "Reading corner"/);
    assert.ok(home.landmarks.some((l) => l.name === "Reading corner" && l.source === "user" && l.room === "r1"));
    const back = await s.drive(tools.call("go_to", { place: "reading corner" }), 60);
    assert.ok(!back.isError, back.text);
    const sh = await tools.call("set_home", {});
    assert.ok(!sh.isError && home.home.source === "user" && saved.at(-1) === home.home, sh.text);
    const n = await tools.call("notify", { text: "Test alert", urgency: "low" });
    assert.match(n.text, /Alert sent/);
    assert.match(houseSummary(home, map), /Reading corner \(saved\)/);
  },

  async "offline parser: rooms, landmarks and missions with the map, the old commands without"() {
    const cases = {
      "go to the living room": ["go_to", { place: "living room" }],
      "fly over to the sofa": ["go_to", { place: "sofa" }],
      "look in room 3": ["look_in", { room: "Room 3" }],
      "find people": ["search_for", { target: "people" }],
      "is anyone home? find someone": ["search_for", { target: "person" }],
      "find the kitty": ["search_for", { target: "cat" }],
      "check on the pets": ["search_for", { target: "pets" }],
      "patrol the house": ["patrol", {}],
      "come home": ["return_home", {}],
      "where am I": ["where_am_i", {}],
      "what changed?": ["what_changed", {}],
      "anything different since yesterday": ["what_changed", { since: "yesterday" }],
      "Where did you last see the cat?": ["recall", { question: "Where did you last see the cat?" }],
      "when were you last in room 2": ["recall", { question: "when were you last in room 2" }],
      "survey the house": ["survey_house", {}],
    };
    for (const [text, [tool, input]] of Object.entries(cases)) assert.deepEqual(parseCommand(text, { house }), { tool, input }, text);
    assert.deepEqual(parseCommand("go to the kitchen"), { explore: "kitchen" });
    // with the map, a place it doesn't have is the mission's refusal (no wandering by hand); directions and things in view stay as they were
    assert.deepEqual(parseCommand("go to the kitchen", { house }), { tool: "go_to", input: { place: "kitchen" } });
    assert.deepEqual(parseCommand("fly to the bedroom", { house }), { tool: "go_to", input: { place: "bedroom" } });
    assert.deepEqual(parseCommand("move to the left", { house }), { tool: "move", input: { direction: "left", meters: 1 } });
    assert.deepEqual(parseCommand("fly toward the cat", { house }), { tool: "approach", input: { object: "cat", distance: "medium" } });
    assert.deepEqual(parseCommand("find the cat"), { tool: "find", input: { object: "cat" } });
    const s = stack({ seed: 19 });
    const tools = new ToolBox({ ctl: s.ctl, perception: s.perception, settings: s.settings, speak: () => {} });
    tools.setHouse({ house, map, missions: s.missions, localizer: s.localizer });
    const lp = new LocalPlanner({ tools, ctl: s.ctl, settings: s.settings }), said = [];
    lp.on("say", (t) => said.push(t));
    await s.drive(lp.run("go to the kitchen"), 10);
    assert.match(said.join(" "), /don't know a place called "kitchen".*Rooms: Living room, Room 2, Room 3/);
    assert.ok(!s.sim.drone.airborne, "no exploring");
    await s.drive(lp.run("fly to room 3"), 120);
    assert.match(said.join(" "), /Arrived at Room 3/);
  },

  async "missions: after landing lost away from the pad, the next one refuses until the pad button; a fresh start uses the pad"() {
    const s = stack({ seed: 6, source: "fused" }), texts = [];
    s.missions.on("status", (st) => texts.push(st.text));
    const r1 = await s.drive(s.missions.run({ kind: "goTo", target: "Living room" }), 120); // odometry only: lost on the way
    await s.drive(forever(), 15);
    const d = s.sim.drone, away = Math.hypot(d.x - HOME.x, d.y - HOME.y);
    assert.ok(texts[0] === "Starting from the home pad.", texts[0]);
    assert.ok(!d.airborne && s.localizer.pose().status === "lost" && s.localizer.flown, `"${r1.summary}"`);
    const r2 = await s.drive(s.missions.run({ kind: "goTo", target: "home" }), 10);
    console.log(`      "${r1.summary}", landed ${fmt(away)} m from the pad; next: "${r2.summary}"; ${s.missions.whereAmI()}`);
    assert.ok(!r2.ok && /press "The drone is on its home pad"/.test(r2.summary), r2.summary);
    assert.ok(!d.airborne, "didn't take off");
    assert.match(s.missions.whereAmI(), /put me on the home pad/);
    s.localizer.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw }); // the pad button
    assert.ok(!s.localizer.flown && s.localizer.pose().status === "ok");
  },

  // Wave B (e2e report, item 1): the remembered spot was the nearest a person could be, 1-3 m nearer than they were, with
  // a 2 m keep-out around it, so missions that never came near anyone stopped. Before (Wave B code, these three cases):
  // 0/3 completed ("A person is too close to go on."), remembered spots 1.25 m from the people (median), 3.28 m (90%).
  async "people who aren't really close don't stop missions: they are remembered where they are (median < 0.7 m) and missions past them complete"() {
    const cases = [
      { people: [[3.6, 4.4, "stand"], [1.3, 5.2, "sit"]], m: { kind: "goTo", target: "Room 3" } },
      { people: [[3.0, 1.6, "lie"]], m: { kind: "lookIn", room: "Living room" } },
      { people: [[4.4, 2.0, "stand"]], m: { kind: "searchFor", target: "people" } },
    ];
    const errs = [];
    for (const [i, c] of cases.entries()) {
      const s = stack({ seed: 202 + i, keep: ["person1", "person2"].slice(0, c.people.length) }), as = s.sim.world.actors.list;
      c.people.forEach(([x, y, st], k) => as[k].place(x, y, st, 1e6));
      let minD = Infinity;
      s.watch.onStep = () => {
        if (s.sim.drone.airborne) for (const a of as) minD = Math.min(minD, Math.hypot(s.sim.drone.x - a.x, s.sim.drone.y - a.y));
        for (const z of s.safety.avoid.temps.values()) if (z.kind === "person") errs.push(Math.min(...as.map((a) => Math.hypot(a.x - z.x, a.y - z.y))));
      };
      const r = await s.drive(s.missions.run(c.m), 240);
      console.log(`      ${c.m.kind} past ${c.people.map(([x, y, st]) => `someone ${{ stand: "standing", sit: "sitting", lie: "lying" }[st]} at (${x}, ${y})`).join(" and ")}: "${r.summary}", closest ${fmt(minD)} m`);
      assert.ok(r.ok && minD >= 1.5, r.summary);
      assert.equal(s.watch.contacts, 0);
    }
    errs.sort((a, b) => a - b);
    const med = errs[errs.length >> 1], p90 = errs[Math.floor(errs.length * 0.9)];
    console.log(`      remembered spots: ${fmt(med)} m from the people (median), ${fmt(p90)} m (90%)`);
    assert.ok(med < 0.7, `median ${fmt(med)} m`);
  },

  // Wave B (e2e item 4): "find people" searched everywhere: 171 s and half the battery for 2 people seen at take-off. The
  // Wave B code in this scenario: stopped after 98 s ("A person is too close to go on."), 27% of the battery, 4 scans.
  async "find people (plural): one scan per room, denser scans only where furniture hid too much, blocked spots skipped; ends with everyone found"() {
    const s = stack({ seed: 5, keep: ["person1", "person2"] }), [p1, p2] = s.sim.world.actors.list;
    p1.place(-1.2, 5.2, "stand", 1e6);
    p2.place(2.6, 1.9, "stand", 1e6);
    const soc0 = s.sim.drone.soc, t0 = now();
    let scans = 0;
    s.missions.on("status", (st) => st.phase === "scan" && scans++);
    const r = await s.drive(s.missions.run({ kind: "searchFor", target: "people" }), 300), secs = (now() - t0) / 1000;
    console.log(`      "${r.summary}" in ${secs.toFixed(0)} s, ${scans} scans, ${Math.round((soc0 - s.sim.drone.soc) * 100)}% of the battery`);
    assert.ok(r.ok && r.findings.filter((f) => f.label === "person").length === 2, r.summary);
    assert.ok(scans <= 6 && secs <= 130, `${scans} scans, ${secs.toFixed(0)} s`);
    assert.equal(s.watch.contacts, 0);
  },

  // Wave B (e2e item 3): in co-pilot the AI let go of pitch and roll when a mission ended in the air, and the pilot had
  // to catch the drift at once. Before, with the pilot's thumbs off the sticks: 0.93-1.29 m of drift in 10 s and wall
  // contacts (seeds 1-3).
  async "co-pilot: a mission that ends in the air keeps holding position (safety attached) until the pilot takes a stick"() {
    for (const seed of [1, 2, 3]) {
      const s = stack({ autonomy: "copilot", seed });
      const keys = s.sim.readKeys.bind(s.sim);
      s.sim.readKeys = (dt) => (keys(dt), (s.sim.handsOn = true)); // the pilot's right thumb is off the stick
      const r = await s.drive(s.missions.run({ kind: "goTo", target: "Living room" }), 120);
      const d = s.sim.drone, x0 = d.x, y0 = d.y;
      let drift = 0;
      s.watch.onStep = () => (drift = Math.max(drift, Math.hypot(d.x - x0, d.y - y0)));
      await s.drive(forever(), 10);
      const held = s.ctl.status, mask = s.ctl.mask, still = drift, contacts = s.watch.contacts;
      s.watch.onStep = null;
      s.sim.keys.add("ArrowRight"); // the pilot rolls right: the radio script hands roll back, the AI lets go
      await s.drive(forever(), 1);
      s.sim.keys.delete("ArrowRight");
      console.log(`      seed ${seed}: "${r.summary}", ${fmt(still)} m of drift in the 10 s after it ("${held}"); after a stick: "${s.ctl.status}"`);
      assert.ok(r.ok && still < 0.4 && /Holding position/.test(held) && mask & 1 && mask & 2, `drift ${fmt(still)}, mask ${mask}`);
      assert.ok(s.ctl.mask === 0 && /pilot moved the sticks/.test(s.ctl.status), s.ctl.status);
      assert.equal(contacts, 0, "contacts before the pilot took over");
    }
  },

  async "memory: every flight is recorded (start, pose trail at 2 Hz, end with what the missions said) and findings become sightings; the patrol says where it will be"() {
    const memory = await HouseMemory.open(house.id, { indexedDB: null, now: () => 1.7e12 + now() });
    memory.setHouse({ house, map });
    const s = stack({ seed: 17, keep: ["person1"], memory }), plans = [];
    s.sim.world.actors.list[0].place(0.0, 4.6, "stand", 1e6);
    s.missions.on("plan", (p) => plans.push(p && { ...p, at: now() }));
    const t0 = now(), r = await s.drive(s.missions.run({ kind: "patrol" }), 200), secs = (now() - t0) / 1000;
    await s.drive(forever(), 1);
    const f = memory.flights.at(-1), trail = memory.trail({ flightId: f.id }), seen = memory.sightings.filter((q) => q.label === "person");
    const air = (f.t1 - f.t0) / 1000;
    console.log(`      "${r.summary}": flight ${f.kind} ${air.toFixed(0)} s in the air, ${trail.length} trail samples (${(trail.length / air).toFixed(2)} Hz), rooms ${f.rooms.join(" > ")}, ${seen.length} sighting(s) of a person; summary "${f.summary}"`);
    assert.equal(memory.flights.length, 1);
    assert.ok(f.t1 && f.summary.includes(r.summary), f.summary);
    assert.ok(Math.abs(trail.length / air - 2) < 0.25, `${trail.length} samples in ${air} s`);
    assert.ok(seen.length >= 1 && seen[0].confirmed === true && seen[0].room, "the person is a sighting");
    assert.ok(seen.every((q) => q.sigma >= 0 && q.sigma < 0.1), `sightings carry the pose's σ: ${seen.map((q) => q.sigma)}`);
    assert.ok(new Set(f.rooms).size === 3, f.rooms.join());
    const real = plans.filter(Boolean), last = plans.findLastIndex(Boolean);
    console.log(`      plans: ${real.map((p) => `[${p.stops.map((q) => q.name).join(" > ")}${p.home ? " > home" : ""}: ${p.seconds} s]`).join(" ")}, then ${plans.at(-1) === null ? "null on the way home" : "?"}; the first said ${real[0]?.seconds} s, the patrol took ${secs.toFixed(0)} s`);
    assert.ok(real.length === 3 && real.every((p, i) => p.home && p.stops.length === 3 - i && p.stops.every((q) => q.room && Number.isFinite(q.x))), "a plan per stop, the stops still ahead");
    assert.ok(real[0].seconds > 0.5 * secs && real[0].seconds < 2 * secs && real[1].seconds < real[0].seconds, `${real.map((p) => p.seconds)} s for ${secs.toFixed(0)} s`);
    assert.ok(last === plans.length - 2 && plans.at(-1) === null, "none on the way home");
  },

  "safety: a pose past a room's outline (in the wall) is pushed back in, not let through"() {
    const r2 = house.rooms.find((r) => r.name === "Room 2"), x0 = Math.min(...r2.outline.map((p) => p[0])), y = 2.4, z = map.floorAt(x0 + 0.3, y) + 1.0;
    for (const dx of [0.05, -0.05, -0.12]) {
      const pose = { x: x0 + dx, y, z, zSigma: 0.02, yaw: Math.PI, sigma: 0.05, status: "ok" }; // facing the west wall
      const ctl = { isFlying: () => true, perception: { frameAge: Infinity, find: () => [] }, videoDelay: 0 };
      const safety = new Safety({ map, localizer: { on() {}, ctl, pose: () => pose, velocity: () => [-0.2, 0] }, settings: null, house });
      safety.floor = map.floorAt(x0 + 0.3, y); // it came from Room 2
      const out = safety.filter({ vx: 0.3, vy: 0, heading: 0 }, { ctl, now: 1000, dt: 0.033, gap: 33 }); // forward = into the wall
      const hx = Math.cos(pose.yaw) * out.vx + Math.sin(pose.yaw) * out.vy; // H x (east = back into the room)
      assert.ok(hx > 0.2, `${dx} m from the outline: H vx ${fmt(hx)}`);
      assert.match(safety.reason, /too close/);
      assert.equal(out.heading, 0);
    }
  },

  "safety: a late controller tick (throttled tab) hovers for a second; observer and ground pass through"() {
    const [x, y] = P.roomCenter(map, "r1"), loc = { on() {}, ctl: null, pose: () => ({ x, y, z: 1.0, yaw: 0, sigma: 0.05, status: "ok" }) };
    const ctl = { isFlying: () => true, perception: { frameAge: Infinity } };
    loc.ctl = ctl;
    const safety = new Safety({ map, localizer: loc, settings: null, house });
    const sp = { vx: 0.1, vy: 0, heading: 1 };
    assert.deepEqual(safety.filter(sp, { ctl, now: 1000, gap: 350 }), { heading: 1 });
    assert.deepEqual(safety.filter(sp, { ctl, now: 1800, gap: 33 }), { heading: 1 });
    assert.equal(safety.filter(sp, { ctl, now: 2100, gap: 33 }), sp);
    assert.equal(safety.status().ok, true);
    ctl.isFlying = () => false;
    assert.equal(safety.filter(sp, { ctl, now: 3000, gap: 900 }), sp);
  },

  // C1's integration run: ScanAt yawed at 1.2 rad/s, the fixes stopped for 4 s, the error grew 0.16 -> 0.66 m while σ said
  // 0.07 -> 0.20 and "ok", and the drone touched a wall.
  async "vision fixes stop mid-scan by a wall (the flow under-reading 40% at 5 fps, a drift toward the wall): σ covers the error, the status degrades, and the safety layer slows the turn, holds and lands without touching anything"() {
    const s = stack({ seed: 17, source: "fused", videoDelay: 100 }), rand = mulberry32(17), gauss = () => Math.sqrt(-2 * Math.log(Math.max(1e-9, rand()))) * Math.cos(2 * Math.PI * rand());
    s.localizer.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
    let fixes = true, nextFix = 0, pending = [], slow = false, n = 0, gap = null;
    const per = s.perception, update = per.update.bind(per), spot = [-1.25, 1.6], reasons = new Map(), samples = [];
    per.update = () => { // after the gap starts: a 5 fps video whose flow reads 60% of the motion
      if (!slow) return update();
      if (n++ % 6) return;
      update();
      const f = per.latest.flow;
      if (f) Object.assign(f, { dx: f.dx * 0.6, dy: f.dy * 0.6, div: f.div * 0.6, span: 0.2 });
    };
    s.watch.onStep = () => {
      const d = s.sim.drone, t = now();
      if (fixes && t >= nextFix) { // splat fixes at 4 Hz, 100 ms late
        nextFix = t + 250;
        pending.push({ at: t + 100, fix: { x: d.x + 0.03 * gauss(), y: d.y + 0.03 * gauss(), z: d.z, yaw: d.yaw + DEG * gauss(), sigma: 0.05, yawSigma: 0.03, t, source: "splat" } });
      }
      while (pending.length && pending[0].at <= t) s.localizer.fix(pending.shift().fix);
      if (!gap || !d.airborne) return;
      const p = s.localizer.pose();
      if (p.status !== "lost") samples.push({ t: (t - gap) / 1000, err: Math.hypot(p.x - d.x, p.y - d.y), sigma: p.sigma, status: p.status, yawRate: Math.abs(d.yawRate ?? 0) });
      const why = s.safety.status().reason;
      if (why) reasons.set(why.replace(/\d+(\.\d+)?/g, "#"), (reasons.get(why.replace(/\d+(\.\d+)?/g, "#")) ?? 0) + 1);
    };
    const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: spot[0], y: spot[1] } }), 90);
    assert.ok(r.ok, r.summary);
    const before = s.localizer.fixQuality();
    // now the GPU can't keep up: no more fixes, a slow video under-reading the motion, a drift toward the west wall
    [fixes, slow, gap, per.fps] = [false, true, now(), 5];
    pending = [];
    s.sim.drone.trimDrift = { x: -0.12, y: 0 };
    await s.drive(s.ctl.run(new ScanAt({ x: spot[0], y: spot[1], headings: [Math.PI / 2, Math.PI, -Math.PI / 2, 0] }, { localizer: s.localizer, map, labels: [], look: 0.8 })), 20);
    await s.drive(forever(), 12);
    const d = s.sim.drone, within = samples.filter((q) => q.err <= 2.5 * q.sigma).length / samples.length, left = samples.find((q) => q.status !== "ok"), turn = Math.max(...samples.filter((q) => q.t > 0.8).map((q) => q.yawRate), 0);
    console.log(`      ${before.rate.toFixed(1)} fixes/s before; in the gap: within 2.5σ ${Math.round(100 * within)}% (${samples.length} samples), status left ok at ${left ? `${fmt(left.t)} s (error ${fmt(left.err)} m, σ ${fmt(left.sigma)})` : "never"}, turns at most ${fmt(turn)} rad/s after 0.8 s; ` +
      `${[...reasons].map(([k, v]) => `"${k}" ${(v / 60).toFixed(1)} s`).join(", ")}; ${d.airborne ? "still flying" : "landed"}, ${s.watch.contacts} contacts`);
    assert.ok(within >= 0.9, `within 2.5σ ${within}`);
    assert.ok(left && samples.every((q) => q.status !== "ok" || q.err <= 0.35), "the status leaves ok before the error passes 0.35 m");
    assert.ok([...reasons.keys()].some((k) => /no camera position fix|lost track/.test(k)), [...reasons.keys()].join(" | "));
    assert.ok(turn <= SAFETY.yawVision + 0.1, `turned at ${fmt(turn)} rad/s`);
    assert.equal(s.watch.contacts, 0, "contact");
    assert.ok(!d.airborne && !d.crashed, "landed in place");
  },

  // Wave C1 budgeted the trip home at 0.35 m/s, the legs at 0.3 m/s; the real drone without live depth flies at most 0.22.
  async "the real drone's budgets (no live depth, braking unmeasured: at most 0.22 m/s): a 240 s pack's patrol turns home in time and lands with at least 10 s left"() {
    const s = stack({ seed: 19, settings: { mode: "real", flightSeconds: 240 } }); // the simulator's truth stands in for the camera's position
    s.sim.drone.soc = 0.55;
    let trip = null, low = Infinity;
    s.safety.on("trip", (t) => t.kind === "battery-home" && !trip && (trip = { ...t, soc: s.sim.drone.soc }));
    s.watch.onStep = () => s.sim.drone.airborne && (low = Math.min(low, s.safety.battery()?.secondsLeft ?? Infinity));
    const r = await s.drive(s.missions.run({ kind: "patrol" }), 400);
    await s.drive(forever(), 90);
    const d = s.sim.drone, left = (d.soc - 0.1) * 240, err = Math.hypot(d.x - HOME.x, d.y - HOME.y); // the pack's truth: 240 s to 10%
    console.log(`      "${r.summary.slice(0, 160)}"; ${trip ? `home at ${Math.round(trip.soc * 100)}% (${trip.reason})` : "no battery trip"}; budget speed ${fmt(s.safety.avoid.cruise().v)} m/s; landed ${!d.airborne} ${fmt(err)} m from the pad with ${Math.round(left)} s left (${Math.round(d.soc * 100)}%; the voltage said ${Math.round(s.safety.battery()?.secondsLeft)} s, at least ${Math.round(low)} s in the air)`);
    assert.ok(!d.airborne && !d.crashed, "landed");
    assert.ok(err < 0.5, `${fmt(err)} m from the pad`);
    assert.ok(left >= 10, `${left} s left`);
    assert.equal(s.watch.contacts, 0);
  },

  // Wave C2 fix 1 (the flight reviews): a blank wall (no vision fix facing it) isn't stared into until the position is
  // lost; a patrol that couldn't land home says where it did.
  async "vision fixes only away from a blank wall: the scan skips the heading facing it, the hold turns back to where the camera last placed itself, and nothing lands or touches"() {
    const s = stack({ seed: 17, source: "fused", videoDelay: 100 }), rand = mulberry32(17), gauss = () => 0.03 * (rand() - 0.5) * 2, BLANK = -Math.PI / 2;
    s.localizer.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
    let next = 0, pending = [], lost = false, held = false;
    s.watch.onStep = () => {
      const d = s.sim.drone, t = now(), blank = Math.abs(Math.atan2(Math.sin(d.yaw - BLANK), Math.cos(d.yaw - BLANK))) < 45 * DEG;
      if (t >= next) (next = t + 250), !blank && pending.push({ at: t + 100, fix: { x: d.x + gauss(), y: d.y + gauss(), z: d.z, yaw: d.yaw, sigma: 0.05, yawSigma: 0.03, t, source: "splat" } });
      while (pending.length && pending[0].at <= t) s.localizer.fix(pending.shift().fix);
      lost ||= d.airborne && s.localizer.pose().status === "lost";
      held ||= /no camera position fix/.test(s.safety.status().reason);
    };
    const go = await s.drive(s.missions.run({ kind: "goTo", target: { x: -0.6, y: 1.4 } }), 90);
    assert.ok(go.ok, go.summary);
    const r = await s.drive(s.ctl.run(new ScanAt({ x: -0.6, y: 1.4, headings: [Math.PI / 2, Math.PI, -Math.PI / 2, 0] }, { localizer: s.localizer, map, labels: [], look: 0.8 })), 60);
    await s.drive(forever(), 10);
    const frames = r.data?.frames ?? [], d = s.sim.drone;
    console.log(`      "${r.text}" ${frames.filter((f) => f.looked).length} of ${frames.length} headings looked (${r.data?.why ?? "all"}); held for a fix ${held}; lost ${lost}; ${d.airborne ? "still flying" : "LANDED"}; ${s.watch.contacts} contacts`); // C2: held facing the wall, lost at 14.9 s, landed
    assert.ok(r.ok && frames.filter((f) => f.looked).length === 3 && frames.length === 4 && /couldn't place itself/.test(r.data.why), r.text);
    assert.ok(!lost && d.airborne && s.watch.contacts === 0, "kept its position and flew on");
  },

  async "a patrol whose way home stays blocked (someone stands on the pad) says it landed on a clear spot, not that it landed home"() {
    const s = stack({ seed: 36, keep: ["person1"] }), b = s.sim.world.actors.list[0];
    b.place(-0.3, 5.4, "stand", 1e6);
    s.sim.drone.soc = 0.45;
    s.missions.on("status", (st) => st.phase === "scan" && b.place(HOME.x + 0.3, HOME.y + 0.5, "stand", 1e6)); // onto the pad once the look begins
    const r = await s.drive(s.missions.run({ kind: "patrol", rooms: ["Living room"] }), 400), d = s.sim.drone;
    console.log(`      "${r.summary}"; landed ${!d.airborne} ${fmt(Math.hypot(d.x - HOME.x, d.y - HOME.y))} m from the pad`); // C2: "...and landed home"
    assert.ok(!d.airborne && Math.hypot(d.x - HOME.x, d.y - HOME.y) > 0.5, "landed elsewhere");
    assert.ok(!/landed home|came back/.test(r.summary) && /couldn't get home/.test(r.summary) && /clear spot/.test(r.summary), r.summary);
    assert.equal(s.watch.contacts, 0);
  },

  // Fix 2 (the verifier): fixes that stop with the heading unchanged (the scene in view changed, or the height estimate
  // drifted off what the render shows) left the hold nothing to turn back to: lost 5 s later, then a landing in place.
  async "vision fixes stop with the heading unchanged (tracking can't place the view from where the estimate has drifted to): the hold asks the vision side to relocalize at once and turns to see more of the room; nothing is lost and the drone flies on"() {
    const s = stack({ seed: 23, source: "fused", videoDelay: 100 }), rand = mulberry32(23), gauss = () => 0.03 * (rand() - 0.5) * 2;
    s.localizer.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
    let next = 0, pending = [], changed = null, reloc = null, lost = false, asked = null, t0 = null, tracked = 0, h0 = null, toward = null;
    const relocs = [], fixOf = (d, t, more = {}) => ({ x: d.x + gauss(), y: d.y + gauss(), z: d.z, yaw: d.yaw, sigma: 0.05, yawSigma: 0.03, t, source: "splat", ...more });
    s.localizer.on("need-fix", () => (asked ??= now() - t0));
    s.watch.onStep = () => {
      const d = s.sim.drone, t = now(), stuck = changed != null && !relocs.length;
      if (t >= next) (next = t + 250), !stuck && pending.push({ at: t + 100, fix: fixOf(d, t) }); // tracking: none until a relocalization puts the estimate back
      while (pending.length && pending[0].at <= t) (s.localizer.fix(pending.shift().fix), changed != null && tracked++);
      // the stored database's relocalization: 800 ms after it is wanted (asked for, or lost: splatloc's own rule), from any heading
      if (changed != null && !reloc && (s.localizer.needFix?.() || s.localizer.pose().status === "lost")) reloc = { at: t + 800, fix: fixOf(d, t, { reloc: true }) };
      if (reloc && reloc.at <= t) (s.localizer.fix(reloc.fix), relocs.push((t - t0) / 1000), (reloc = null));
      if (changed != null && s.safety.fixHold) toward ??= s.safety.fixHold.heading - h0; // where the hold turns (controller heading)
      lost ||= d.airborne && changed != null && s.localizer.pose().status === "lost";
      if (process.env.TRACE && changed != null && Math.round((t - t0) / 1000 * 30) % 6 === 0) { const p = s.localizer.pose(), q = s.localizer.fixQuality(); console.log(`        ${((t - t0) / 1000).toFixed(1)} true (${fmt(d.x)}, ${fmt(d.y)}) est (${fmt(p.x)}, ${fmt(p.y)}) σ ${fmt(p.sigma)} ${p.status} fix ${Math.round(q.visionAge)} yaw ${Math.round(d.yaw / DEG)} ${s.ctl.behavior?.label} | ${s.safety.status().reason}`); }
    };
    const go = await s.drive(s.missions.run({ kind: "goTo", target: { x: 2.0, y: 1.6 } }), 90);
    assert.ok(go.ok, go.summary);
    [changed, t0, h0] = [s.sim.drone.yaw, now(), s.ctl.est.heading]; // tracking stops with the heading as at the last fix
    await s.drive(s.ctl.run(new Hold(12)), 15);
    const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: 2.6, y: 2.0 } }), 60);
    console.log(`      relocalization asked ${asked != null ? `${(asked / 1000).toFixed(1)} s` : "never"} after the last fix; relocalized at ${relocs.map((v) => v.toFixed(1)).join(", ")} s; the hold turned ${toward != null ? `${Math.round(Math.abs(toward) / DEG)}°` : "nowhere"} toward the open side; ${tracked} tracking fixes since; lost ${lost}; then "${r.summary}"; ${s.watch.contacts} contacts`); // C2 fix 1: lost at 5 s, landed in place
    assert.ok(asked != null && asked < 3000 && relocs.length && relocs[0] < asked / 1000 + 2, `asked ${asked}, relocalized ${relocs}`);
    assert.ok(!lost && Math.abs(toward) > 50 * DEG, `lost ${lost}, turned toward ${toward}`);
    assert.ok(r.ok && s.watch.contacts === 0, r.summary);
  },

  // Fix 3 (the browser's sealed runs, seeds 11 and 12): the height was 0.23-0.27 m off at p95 (4-6% of the time beyond
  // 2.5 σz) while the vision fixes' own heights were 1-4 cm off: the flow's climb rate (half the climb, ±0.3 m/s, once
  // +0.8 m/s while the drone sank) moved the localizer's height between fixes and was all the controller held height by.
  async "the flow's climb rate misreading (half the climb, ±0.3 m/s, +0.8 m/s for a second every 6 s) with vision fixes 4 a second (2 cm in height): the height estimate stays within 0.12 m (p95) and 2.5 σz (97%), the drone keeps the height held through a scan (0.25 m) and, once up, stays 0.45 m over the floor, and touches nothing"() {
    const s = stack({ seed: 29, source: "fused", videoDelay: 100 }), rand = mulberry32(29), g = () => Math.sqrt(-2 * Math.log(Math.max(1e-9, rand()))) * Math.cos(2 * Math.PI * rand());
    s.localizer.reset({ x: HOME.x, y: HOME.y, yaw: HOME.yaw });
    const per = s.perception, update = per.update.bind(per), vfov = s.values.hfov * DEG * 0.75, errs = [], within = [], scan = { z0: null, dev: 0 };
    let next = 0, pending = [], noise = 0, seen = null, low = Infinity, home = false, risen = false;
    per.update = () => { // each new frame's climb rate: half the drone's, ±0.3 m/s hanging together over ~0.1 s, +0.8 m/s bursts
      update();
      const f = per.latest.flow, d = s.sim.drone;
      if (!f || f === seen || !d.airborne) return;
      seen = f;
      noise = 0.6 * noise + 0.8 * 0.3 * g();
      f.dy += (-0.5 * d.vz + noise + ((now() / 1000) % 6 < 1 ? 0.8 : 0)) / (vfov * 2.5);
    };
    s.watch.onStep = () => {
      const d = s.sim.drone, t = now();
      if (t >= next) (next = t + 250), pending.push({ at: t + 100, fix: { x: d.x + 0.03 * g(), y: d.y + 0.03 * g(), z: d.z + 0.02 * g(), yaw: d.yaw + DEG * g(), sigma: 0.05, yawSigma: 0.03, t, source: "splat" } });
      while (pending.length && pending[0].at <= t) s.localizer.fix(pending.shift().fix);
      if (!d.airborne || s.ctl.behavior?.name === "takeoff" || home) return;
      const p = s.localizer.pose(), e = Math.abs(p.z - d.z);
      errs.push(e);
      within.push(e <= 2.5 * p.zSigma);
      if ((risen ||= d.z - d.floorZ > 0.7)) low = Math.min(low, d.z - d.floorZ); // (once up after the take-off)
      if (scan.z0 != null && t - scan.z0 > 1500 && s.safety.zHold != null) scan.dev = Math.max(scan.dev, Math.abs(d.z - s.safety.zHold)); // from 1.5 s in: the height held
    };
    const go = await s.drive(s.missions.run({ kind: "goTo", target: { x: 2.0, y: 1.6 } }), 90);
    scan.z0 = now();
    const sc = await s.drive(s.ctl.run(new ScanAt({ x: 2.0, y: 1.6, headings: [0, Math.PI / 2, Math.PI, -Math.PI / 2] }, { localizer: s.localizer, map, labels: [], look: 1.5 })), 40);
    [scan.z0, home] = [null, true];
    const back = await s.drive(s.missions.run({ kind: "returnHome" }), 90);
    const q = (a, f) => [...a].sort((x, y) => x - y)[Math.floor(f * (a.length - 1))], share = within.filter(Boolean).length / within.length;
    console.log(`      height estimate error median ${fmt(q(errs, 0.5))} p95 ${fmt(q(errs, 0.95))} max ${fmt(Math.max(...errs))} m, ${(100 * share).toFixed(1)}% within 2.5 σz; the scan kept within ${fmt(scan.dev)} m of the height held; lowest ${fmt(low)} m over the floor; "${go.summary}" / scan ${sc.ok} / "${back.summary}"; ${s.watch.contacts} contacts`); // C2 fix 2: p95 0.47, 81%, the scan 0.74 m off, down to the floor
    assert.ok(go.ok && sc.ok && back.ok, `${go.summary} / ${sc.text} / ${back.summary}`);
    assert.ok(q(errs, 0.95) < 0.12 && share >= 0.97, `p95 ${q(errs, 0.95)}, within ${share}`);
    assert.ok(scan.dev < 0.25 && low > 0.45, `scan ${scan.dev}, lowest ${low}`);
    assert.equal(s.watch.contacts, 0);
  },

  // Fix 2 (the verifier): "…so I landed in the Living room on a clear spot instead: the clear spot couldn't be reached".
  "return home that can't get home says where it really landed: on a clear spot only when it got there; where it was (and why) when the spot couldn't be reached or there was none; co-pilot: where the pilot was asked to land"() {
    const room = house.rooms.find((r) => r.name === "Living room"), c = P.roomCenter(map, room.id, { alt: 1.0 }), at = { x: c[0], y: c[1], z: 1, yaw: 0, sigma: 0.05, status: "ok" };
    const out = [];
    for (const [autonomy, more] of [["full", ""], ["full", "the clear spot couldn't be reached"], ["full", "there's no clear spot nearby either"], ["copilot", "there's no clear spot nearby either"]]) {
      const rh = new ReturnHome({ localizer: { pose: () => at }, map, home: HOME }), ctl = { autonomy, askPilot() {}, est: { heading: 0 } };
      rh.why = "something new in the Living room blocks the way";
      let r = rh.landHere(ctl, 0, more); // as the spot path does: "" once it got to the spot (783, 835-836)
      if (!r) (rh.child = { update: () => ({ done: { ok: true, text: "Landed." } }) }), (r = rh.update(ctl, 0.03, 100));
      out.push(r.done.text);
    }
    console.log(`      ${out.map((t) => `"${t}"`).join("\n      ")}`);
    assert.ok(/landed in the Living room on a clear spot instead\.$/.test(out[0]), out[0]);
    for (const t of out.slice(1, 3)) assert.ok(/so I landed where I was in the Living room: (the clear spot couldn't be reached|there's no clear spot nearby either)\.$/.test(t) && !/on a clear spot/.test(t), t);
    assert.ok(/asked the pilot to land where I am; there's no clear spot nearby either/.test(out[3]), out[3]);
  },

  "house summary: deterministic, no coordinates, rooms, doorways, landmarks and keep-outs"() {
    const a = houseSummary(house, map), b = houseSummary(structuredClone(house), map);
    assert.equal(a, b);
    for (const t of ["Living room", "Room 2", "Room 3", "Doorways:", "Home pad: in Room 2", "2 sofas", "ceiling fan (Living room)", "stairs (Room 3)"]) assert.ok(a.includes(t), t);
    assert.ok(!/\d\.\d{3}/.test(a), "no raw coordinates");
    assert.ok(a.length < 1500, `${a.length} chars`);
  },
};

// avoid-cases.mjs's missionCases() (wave C reviews): a sealed room on patrol, a place off the map, a misread of an
// unmapped box, flying without live depth on the real drone, climbing under something, a failed person detector,
// measuring the braking, video that isn't the camera's picture; on test-avoid.mjs's stack.
const A = await import("./test-avoid.mjs");
Object.assign(tests, (await import("./avoid-cases.mjs")).missionCases({ stack: A.stack, depthCamera: A.depthCamera, house: A.house, map: A.map }));
const only = process.env.ONLY;
await runTests(only ? Object.fromEntries(Object.entries(tests).filter(([k]) => k.includes(only))) : tests);
