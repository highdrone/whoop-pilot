// What the real drone needs from avoidance beyond the simulator's clean world (the wave C reviews), headless, on the house
// fixture with test-avoid.mjs's stack: a new thing in front of an open doorway is not a closed door; a sealed room doesn't
// end a patrol or close the doorways beside it; a place off the map is refused on the ground whatever else is on the map;
// a misread live depth doesn't move or grow what it found; without its eyes the real drone flies slowly and says so;
// climbing is capped under something; the height's uncertainty counts; a scan the person detector didn't see doesn't
// count as looked; braking is measured; video that isn't the camera's picture is refused; a landing beside someone or
// over something is never cut up in the air; a closed door whose depth reads too far is still a closed door, and isn't
// touched; a doorway it can't get through ends the leg after a few tries.
// test-avoid.mjs runs avoidCases() and test-missions.mjs missionCases(), each with test-avoid.mjs's { stack, depthCamera,
// house, map }, so npm test has them (ONLY= picks by name).
import assert from "node:assert/strict";
import { now, rig } from "./test-sim.mjs";

const { ChangeDetector } = await import("../app/js/nav/changes.js");
const { Avoid, AVOID, freeRun, polyDist } = await import("../app/js/nav/avoid.js");
const { makeHouseWorld } = await import("../app/js/sim/twin-world.js");
const { Move, ScanAt, Land, LAND } = await import("../app/js/behaviors.js");
const { PictureFinder, pictureProblem } = await import("../app/js/vision/depth.js");
const { MissionRunner } = await import("../app/js/missions.js");
const { Localizer } = await import("../app/js/nav/localizer.js");
const { Safety } = await import("../app/js/safety.js");
const { roomCenter } = await import("../app/js/house/planner.js");

const fmt = (v) => (Number.isFinite(v) ? v.toFixed(2) : String(v));
const tick = () => new Promise((r) => setImmediate(r));
const forever = () => new Promise(() => {});
const toBox = (d, b) => Math.hypot(Math.max(0, Math.abs(d.x - b.x) - b.w / 2), Math.max(0, Math.abs(d.y - b.y) - b.d / 2));

const MISSION = /sealed|off the map|detector that failed|braking measured|camera's picture|eyes off|climbing|unmapped box found|can't get through/;
export const avoidCases = (ctx) => Object.fromEntries(Object.entries(cases(ctx)).filter(([k]) => !MISSION.test(k)));
export const missionCases = (ctx) => Object.fromEntries(Object.entries(cases(ctx)).filter(([k]) => MISSION.test(k)));

function cases({ stack, depthCamera, house, map }) {
  const HOME = house.home;
  async function placeAt(s, x, y, yaw) {
    const r = await s.drive(s.missions.run({ kind: "goTo", target: { x, y } }), 60);
    assert.ok(r.ok, r.summary);
    await s.drive(s.ctl.run(new ScanAt({ x, y, headings: [yaw] }, { localizer: s.localizer, map, labels: [], look: 0.3 })), 15);
  }

  const cases1 = {
    // Before: a 1.3-1.6 m box 0.75 m before the open Room 2 / Room 3 doorway was reported as "door-closed" there (zMin 0, below
    // the floor), closing the doorway on the map.
    async "a box 0.75 m in front of an open doorway is an obstacle at its place, never a closed door"() {
      for (const h of [0.8, 1.3, 1.6])
        for (const misread of [1, 1.4]) {
          const world = makeHouseWorld(house, map, { seed: 40, cast: [] }), plain = makeHouseWorld(house, map, { seed: 40, cast: [] });
          const box = { id: "box", x: -0.25, y: 2.12, z: 0.242, w: 0.5, d: 0.5, h };
          world.addObstacle(box);
          const det = new ChangeDetector({ map, house }), found = [];
          det.on("change", (c) => found.push(c));
          for (let i = 0; i < 24; i++) { // walking up to it from the south of Room 2, weaving
            const y = 0.3 + 0.04 * i, x = -0.25 + 0.5 * Math.sin(i / 4), pose = { x, y, z: 1.24, yaw: Math.atan2(2.12 - y, -0.25 - x), pitch: 0, roll: 0, sigma: 0.05, status: "ok" };
            const live = depthCamera(world, pose), exp = depthCamera(plain, pose, { actors: false }).depth; // depth error: CHANGES.depthErr, as LiveDepth's
            if (misread !== 1) for (let k = 0; k < live.depth.length; k++) if (live.depth[k] < exp[k] - 0.05) live.depth[k] = Math.min(exp[k], live.depth[k] * misread);
            det.ingest({ ...live, t: i * 200, pose, expected: exp }, i * 200);
          }
          await new Promise((r) => setTimeout(r, 0)); // reports finish asynchronously (evidence, memory)
          const c = found[0], floor = map.floorAt(box.x, box.y);
          console.log(`      ${h} m box, depth x${misread}: ${found.map((q) => `${q.kind}${q.door ? ` ${q.door}` : ""} at (${fmt(q.x)}, ${fmt(q.y)}) ${fmt(q.zMin)}-${fmt(q.zMax)} m`).join("; ") || "nothing"}`);
          assert.ok(found.every((q) => !q.kind.startsWith("door")), "no door change");
          assert.ok(c?.kind === "obstacle" && Math.hypot(c.x - box.x, c.y - box.y) < 0.45, "the box, at its place");
          assert.ok(Math.abs(c.zMin - floor) < 0.05, `from the floor (${fmt(floor)}), not below it`);
          for (const id of [...map.temps.keys()]) map.removeTemp(id);
        }
    },

    // Before: the closed Room 2 / Room 3 door became a live-depth disc of radius 0.9 m reaching -1.5..4.1 m up across the open
    // Room 2 -> Living room doorway; the patrol stopped at its first room after 20 s, hovering mid-house in full auto. Then
    // (fix round 1) with the door's depth read 1.4 x too far (within the error assumed for new things) it was no longer a
    // closed door and nothing was on the map for it: 225-446 contact steps against it (up to 0.20 m/s), 213 s.
    async "a patrol with Room 3 sealed (its doors' depth exact or read 1.4 x too far): it skips Room 3 (saying why), patrols the others and lands home without touching anything; the open doorway stays open"() {
      for (const misread of [1, 1.4]) await sealed(misread);
    },
  };

  // A patrol with Room 3's doors closed, their live depth read `misread` times too far.
  async function sealed(misread) {
    const s = await stack({ seed: 44, misread });
    for (const id of ["r2:w1-o1+r3:w2-o1", "r3:w3-o1", "r3:w4-o1"]) s.sim.setDoor(id, false);
    const open = map.doors.find((d) => d.id === "r2:w4-o1"), live = [];
    s.safety.avoid.on("temp", (t) => t.source === "live depth" && t.added && live.push(t));
    let worst = 0; // how much of the open doorway's width live-depth obstacles ever covered
    s.watch.onStep = () => {
      let n = 0;
      for (let u = 0.05; u < 1; u += 0.1) {
        const x = open.a[0] + u * (open.b[0] - open.a[0]), y = open.a[1] + u * (open.b[1] - open.a[1]);
        n += [...s.safety.avoid.temps.values()].some((t) => t.source === "live depth" && polyDist(x, y, t.polygon) < map.o.droneRadius + 0.17) ? 1 : 0;
      }
      worst = Math.max(worst, n / 10);
    };
    const t0 = now(), r = await s.drive(s.missions.run({ kind: "patrol" }), 300), d = s.sim.drone;
    const tall = Math.max(0, ...live.map((t) => t.zMax - t.zMin));
    console.log(`      depth x${misread}: "${r.summary}" after ${((now() - t0) / 1000).toFixed(0)} s, ${fmt(Math.hypot(d.x - HOME.x, d.y - HOME.y))} m from the pad, airborne ${d.airborne}; contact steps ${s.watch.contacts}; live-depth obstacles: ${live.length}, tallest ${fmt(tall)} m; at most ${Math.round(worst * 100)}% of the open doorway covered`);
    assert.ok(/Living room/.test(r.summary.split("landed home")[0]) && /couldn't get to Room 3/.test(r.summary), r.summary);
    assert.ok(!d.airborne && Math.hypot(d.x - HOME.x, d.y - HOME.y) < 0.4, "landed home");
    assert.ok(worst <= 0.5 && tall < 2.6, "the open doorway stays passable");
    assert.equal(s.watch.contacts, 0, "never touched the closed doors");
    for (const id of ["r2:w1-o1+r3:w2-o1", "r3:w3-o1", "r3:w4-o1"]) s.sim.setDoor(id, true);
  }

  const cases2 = {
    // Before (fix round 1): inside a person's zone (every height counts as inside it) the vertical cap held a landing at
    // hover height; Land took the quiet hover for a touchdown and cut the throttle 1.01 m up (the drone fell at 3.0 m/s;
    // 0.80 m up at 2.9 m/s with someone standing 1 m away).
    async "a landing beside someone touches down slowly; over something it stops, says so and never cuts up there; a forced landing sets down on it slowly"() {
      for (const where of ["a person's zone over the drone", "someone standing 1 m away"]) {
        const s = await stack({ seed: 51, depth: false, changes: false });
        await s.drive(s.missions.run({ kind: "goTo", target: { x: 2.4, y: 2.2 } }), 60);
        const d = s.sim.drone, floor = map.floorAt(d.x, d.y), at = [d.x + 1, d.y];
        const person = () => s.safety.avoid.addTemp({ id: "p", kind: "person", x: where.startsWith("a person") ? d.x + 1 : at[0], y: at[1], r: 1.5, zMin: floor - 0.1, zMax: floor + 2.2, until: now() + 5000 });
        const b = new Land({ at: [d.x, d.y], localizer: s.localizer });
        let cut = null, fastest = 0;
        s.watch.onStep = () => (person(), b.cutAt != null && cut == null && (cut = d.z - floor), (fastest = Math.max(fastest, -d.vz)));
        person();
        const r = await s.drive(s.ctl.run(b), 20);
        console.log(`      ${where}: "${r.text}" throttle cut ${fmt(cut)} m up, fastest descent ${fmt(fastest)} m/s`);
        assert.ok(r.ok && cut < 0.15 && fastest < 0.5 && !d.airborne, `cut ${fmt(cut)} m up, ${fmt(fastest)} m/s`);
        s.safety.avoid.removeTemp("p");
      }
      // over a crate (a suspected change below the drone): the cap stops the descent over it and says so; a forced landing
      // goes on at AVOID.touch; inside a person's zone sinking isn't capped
      const c = roomCenter(map, "r1"), floor = map.floorAt(...c), fake = { ctl: { videoDelay: 250, est: { ttc: Infinity } }, velocity: () => [0, 0] };
      map.addTemp({ id: "crate", kind: "change", change: "obstacle", x: c[0], y: c[1], r: 0.4, zMin: floor - 0.1, zMax: floor + 0.5, until: Infinity });
      const a = new Avoid({ map, localizer: fake }), pose = (z) => ({ x: c[0], y: c[1], z: floor + z, yaw: 0, sigma: 0.05, zSigma: 0.02 });
      const high = a.limit({ vz: -0.35 }, pose(1.2), 1e6).vz, low = a.limit({ vz: -0.35 }, pose(0.7), 1e6), why = a.last.vertical, forced = a.limit({ vz: -0.35, touchdown: true }, pose(0.7), 1e6).vz;
      map.removeTemp("crate");
      map.addTemp({ id: "zone", kind: "person", x: c[0] + 0.5, y: c[1], r: 1.5, zMin: floor - 0.1, zMax: floor + 2.2, until: Infinity });
      const zone = a.limit({ vz: -0.35 }, pose(0.7), 1e6).vz;
      map.removeTemp("zone");
      console.log(`      over a 0.5 m crate: sinking from 1.2 m ${fmt(high)} m/s, from 0.7 m ${fmt(low.vz)} m/s ("${why}"), forced ${fmt(forced)} m/s; in a person's zone ${fmt(zone)} m/s`);
      assert.ok(high < -0.3 && low.vz > -0.02 && why === "something below" && Math.abs(forced + AVOID.touch) < 0.01 && zone === -0.35);
      // Land held over something: not down (no cut), ends blocked after LAND.blocked s; a forced one asks to set down
      const land = (onto) => {
        const ctl = { est: { heading: 0, vz: 0 }, perception: { latest: { flow: { dx: 0, dy: 0, div: 0 } } }, out: { thr: 0.5 }, safety: { avoid: { last: { vertical: "something below" } } } };
        const b = new Land({ onto });
        b.start(ctl);
        b.t0 = 0;
        let r = null, touch = false, t = 0;
        for (; t <= 8000 && !r?.done; t += 33) (r = b.update(ctl, 0.033, t)), (touch ||= !!r.sp?.touchdown);
        return { r, cut: b.cutAt, touch, t };
      };
      const held = land(false), forcedLand = land(true);
      console.log(`      Land held over it: "${held.r.done?.text}" after ${fmt(held.t / 1000)} s, throttle cut: ${held.cut ?? "no"}; forced: sets down ${forcedLand.touch}, cut: ${forcedLand.cut ?? "no"}`);
      assert.ok(held.r.done && !held.r.done.ok && held.r.done.blocked && held.cut == null && held.t / 1000 > LAND.blocked && held.t / 1000 < LAND.blocked + 0.5);
      assert.ok(!forcedLand.r.done && forcedLand.touch && forcedLand.cut == null);
    },

    // Found in fix round 2: a doorway the drone couldn't line up with was planned again and again (up to 30 times, about 9 s
    // each: a patrol hovered 200 s in front of it until the battery ran low).
    async "a doorway it can't get through ends the leg after 3 tries, saying why (no endless replanning)"() {
      const s = await stack({ seed: 53, depth: false, changes: false }), run = s.ctl.run.bind(s.ctl);
      s.missions.o.narrowDoor = 1; // every doorway is flown with DoorTransit
      let tries = 0;
      s.ctl.run = (b, sig) => (b.name === "door" ? (tries++, Promise.resolve({ ok: false, text: "Couldn't line up with the doorway." })) : run(b, sig));
      const r = await s.drive(s.missions.run({ kind: "goTo", target: "Living room" }), 120);
      console.log(`      "${r.summary}" after ${tries} tries`);
      assert.ok(!r.ok && /couldn't get through the doorway to the Living room/.test(r.summary) && tries === 3, r.summary);
      s.ctl.run = run;
      await s.drive(s.missions.run({ kind: "returnHome" }), 60);
    },
  };

  return {
    ...cases1,
    ...cases2,
    // Before: with any suspected change or remembered person anywhere, a goal off the map made the drone take off (and say
    // that change blocked the way), or hover 20 s blaming the person.
    async "a place off the map is refused on the ground, whatever else is on the map; only what is on the way is blamed"() {
      for (const what of ["a change", "a person"]) {
        const s = await stack({ seed: 38, depth: false });
        const add = () => (what === "a change" ? map.addTemp({ id: "far", kind: "change", change: "obstacle", x: 2.6, y: 2.0, r: 0.3, until: Infinity })
          : s.safety.avoid.addTemp({ id: "far", kind: "person", x: 2.6, y: 2.0, r: 1.5, zMin: -0.1, zMax: 2.2, until: now() + 5000 }));
        add();
        s.watch.onStep = () => what === "a person" && add();
        let top = 0;
        const t0 = now(), r = await s.drive(s.missions.run({ kind: "goTo", target: { x: 6.5, y: 1.0 } }), 40);
        top = s.sim.drone.z - map.floorAt(HOME.x, HOME.y);
        console.log(`      with ${what} in the Living room: "${r.summary}" after ${((now() - t0) / 1000).toFixed(0)} s, airborne ${s.sim.drone.airborne}`);
        assert.ok(!r.ok && /can't find a way/.test(r.summary) && !s.sim.drone.airborne && top < 0.1, r.summary);
        // the same thing on the way is blamed: a person remembered in the doorway to Room 3
        s.safety.avoid.removeTemp("far");
        map.removeTemp("far");
        const b = s.missions.blocker([-0.7, 4.5]);
        s.safety.avoid.addTemp({ id: "door", kind: "person", x: -0.6, y: 2.9, r: 1.5, zMin: -0.1, zMax: 2.2, until: now() + 5000 });
        const b2 = s.missions.blocker([-0.7, 4.5]);
        console.log(`      blamed with nothing on the way: ${b?.text ?? "nothing"}; with someone in the doorway: ${b2?.text}`);
        assert.ok(!b && /person in Room 2|person in Room 3/.test(b2?.text ?? ""), "only what is on the way");
        s.safety.avoid.removeTemp("door");
      }
    },

    // Before: after a misread (the depth reading twice too far), the obstacle's disc moved behind the box's face; every merge
    // padded its heights by another 0.1 m and kept the largest radius.
    async "an unmapped box found on live depth: a later misread neither moves it off the box's face nor grows it"() {
      const s = await stack({ seed: 31 }), box = { id: "box", x: 2.3, y: 4.95, w: 0.5, d: 0.5, h: 1.4 };
      await placeAt(s, 0.9, 4.95, 0);
      s.sim.addObstacle(box);
      let mis = 1;
      const ingest = s.safety.avoid.ingest.bind(s.safety.avoid);
      s.safety.avoid.ingest = (f, t) => {
        if (mis !== 1) for (let i = 0; i < f.depth.length; i++) if (f.depth[i] < f.expected[i] - 0.05) f.depth[i] = Math.min(f.expected[i], f.depth[i] * mis);
        return ingest(f, t);
      };
      const live = () => [...s.safety.avoid.temps.values()].filter((t) => t.source === "live depth");
      await s.drive(forever(), 3);
      const before = live()[0];
      mis = 2;
      await s.drive(forever(), 4);
      const after = live()[0], face = [box.x - box.w / 2, box.y], area = (p) => Math.abs(p.reduce((a, q, i) => a + q[0] * p[(i + 1) % p.length][1] - p[(i + 1) % p.length][0] * q[1], 0)) / 2;
      console.log(`      before: (${fmt(before?.x)}, ${fmt(before?.y)}), ${fmt(before && area(before.polygon))} m², ${fmt(before?.zMin)}-${fmt(before?.zMax)} m; after 4 s read twice too far: (${fmt(after?.x)}, ${fmt(after?.y)}), ${fmt(after && area(after.polygon))} m², ${fmt(after?.zMin)}-${fmt(after?.zMax)} m; the face ${fmt(after && polyDist(...face, after.polygon))} m outside it`);
      assert.ok(before && after && polyDist(...face, after.polygon) < 0.05, "still over the box's face");
      assert.ok(after.zMax - after.zMin < 2.2 && after.zMin > map.floorAt(box.x, box.y) - 0.15, "its heights don't grow");
      assert.ok(area(after.polygon) < 3, "bounded by what was seen");
      s.sim.removeObstacle("box");
    },

    // Before: with live depth off the drone flew the dropped-box route at mission speed (0.4 m/s into the box, 276 contact
    // steps) and said "Arrived" as if nothing was missing.
    async "eyes off on the real drone (no live depth, no 3D map): it flies slowly, says why, and the summary counts it"() {
      const s = await stack({ seed: 31, videoDelay: 90, depth: false, changes: false, settings: { mode: "real" } }), box = { id: "box", x: 1.7, y: 2.0, w: 0.6, d: 0.6, h: 1.4 };
      let dropped = false, asked = 0, first = null, prev = 0, reasons = new Set();
      const filter = s.safety.filter.bind(s.safety);
      s.safety.filter = (sp, st) => { const o = filter(sp, st); if (s.sim.drone.airborne) asked = Math.max(asked, Math.hypot(o.vx ?? 0, o.vy ?? 0)); return o; };
      s.watch.onStep = () => {
        const d = s.sim.drone;
        if (!dropped && d.airborne && d.x > -0.2 && d.y > 1.0) (s.sim.addObstacle(box), (dropped = true));
        if (dropped && first == null && s.watch.contacts) first = prev; // the speed it came in at
        prev = Math.hypot(d.vx, d.vy);
        if (s.safety.reason) reasons.add(s.safety.reason);
      };
      const r = await s.drive(s.missions.run({ kind: "goTo", target: { x: 3.9, y: 1.4 } }), 150);
      console.log(`      "${r.summary}"; fastest asked ${fmt(asked)} m/s; ${first != null ? `ran into the box (which nothing could see) at ${fmt(first)} m/s` : "no contact"}; eyes: ${s.safety.avoid.eyes().why}; reasons: ${[...reasons].join(" / ")}`);
      assert.ok(asked <= AVOID.blind + 0.01, `asked for ${fmt(asked)} m/s`);
      assert.ok(/slowly without live depth/.test(r.summary) && [...reasons].some((q) => /flying slowly/.test(q)), "it says so");
      assert.ok(first == null || first <= 0.3, `contact at ${fmt(first)} m/s`);
      s.sim.removeObstacle("box");
    },

    "the height's uncertainty counts: at ±0.3 m the cap stops for a table it may be level with"() {
      // a table: occupied at the 0.6 m band, free at 1.0 m, with free space in front of it
      const lo = map.occ[map.bandOf(0.6)], hi = map.occ[map.bandOf(1.0)];
      let at = null;
      for (let k = 0; k < map.N && !at; k++) {
        if (!lo[k] || hi[k] || map.room[k] < 0) continue;
        const [x, y] = map.center(k % map.W, (k / map.W) | 0);
        for (const yaw of [0, Math.PI / 2, Math.PI, -Math.PI / 2]) {
          const p = [x - 1.0 * Math.cos(yaw), y - 1.0 * Math.sin(yaw)], z = (map.floorAt(...p) ?? 0) + 1.0;
          if (map.roomAt(...p)?.id === map.roomAt(x, y)?.id && map.clearance(p[0], p[1], z - 0.4) > 0.5 && freeRun(map, { x: p[0], y: p[1], z }, [Math.cos(yaw), Math.sin(yaw)], 0.15) > 1.5) at = { x: p[0], y: p[1], z, yaw, table: [x, y] };
          if (at) break;
        }
      }
      assert.ok(at, "a table in the fixture");
      const fake = { ctl: { videoDelay: 250, est: { ttc: Infinity } }, velocity: () => [0, 0] };
      const run = (zSigma) => {
        const a = new Avoid({ map, localizer: fake }), out = a.limit({ vx: 1.0, vy: 0 }, { ...at, sigma: 0.05, zSigma }, 1e6);
        return { cap: Math.hypot(out.vx, out.vy), run: a.last.run };
      };
      const sure = run(0.02), unsure = run(0.3);
      console.log(`      a table at (${fmt(at.table[0])}, ${fmt(at.table[1])}) 1 m ahead at 1.0 m up: height ±0.02 m: free run ${fmt(sure.run)} m, ${fmt(sure.cap)} m/s; ±0.3 m: ${fmt(unsure.run)} m, ${fmt(unsure.cap)} m/s`);
      assert.ok(unsure.run < sure.run - 0.3 && unsure.cap < sure.cap, "slower when the height is unsure");
    },

    // Before: vertical speeds were never capped: a climb went on into whatever the map had above.
    async "climbing is capped under something on the map, and a take-off under it is refused"() {
      const s = await stack({ seed: 45, depth: false, changes: false });
      await placeAt(s, 2.4, 2.2, 0);
      const floor = map.floorAt(2.4, 2.2);
      map.addTemp({ id: "shelf", kind: "change", change: "obstacle", x: 2.4, y: 2.2, r: 0.4, zMin: floor + 1.45, zMax: floor + 1.55, until: Infinity });
      let top = 0, why = "", away = 0; // the highest it got while under the shelf (its body over the shelf's outline)
      s.watch.onStep = () => {
        const d = s.sim.drone, off = Math.hypot(d.x - 2.4, d.y - 2.2);
        if (off < 0.4 + map.o.droneRadius) top = Math.max(top, d.z - floor);
        away = Math.max(away, off);
        why ||= s.safety.avoid.last.vertical;
      };
      await s.drive(s.ctl.run(new Move("up", 1.0, 0.35)), 8);
      await s.drive(forever(), 2);
      console.log(`      under a shelf 1.45 m up: climbed to ${fmt(top)} m under it (${why || "-"}; at most ${fmt(away)} m from its middle)`);
      assert.ok(top < 1.45 - map.o.droneRadius && why === "something above", `climbed to ${fmt(top)} m`);
      map.removeTemp("shelf");
      const t = await stack({ seed: 46, depth: false, changes: false }), pf = map.floorAt(HOME.x, HOME.y);
      map.addTemp({ id: "shelf", kind: "change", change: "obstacle", x: HOME.x, y: HOME.y, r: 0.3, zMin: pf + 0.7, zMax: pf + 0.8, until: Infinity });
      const r = await t.drive(t.missions.run({ kind: "goTo", target: { x: 0, y: 1.5 } }), 20);
      console.log(`      a shelf 0.7 m over the pad: "${r.summary}" (airborne ${t.sim.drone.airborne})`);
      assert.ok(!r.ok && /won't take off/.test(r.summary) && !t.sim.drone.airborne, r.summary);
      map.removeTemp("shelf");
    },

    // Before: a scan counted as "looked at" whatever the detector did; a search with it down said "didn't find anyone".
    async "a person detector that failed to load: scans don't count as looked at, and the summary says which rooms weren't checked"() {
      const s = await stack({ seed: 47, depth: false, changes: false });
      Object.assign(s.perception, { detector: { ready: false }, detectorFailed: true });
      const r = await s.drive(s.missions.run({ kind: "lookIn", room: "Living room" }), 120);
      console.log(`      "${r.summary}"`);
      assert.ok(/couldn't check/.test(r.summary) && /detector failed/.test(r.summary) && !/: no people or pets\./.test(r.summary), r.summary);
      delete s.perception.detector;
      s.perception.detectorFailed = false;
    },

    async "braking measured in the simulator (full auto): plausible numbers, saved for the simulator's cap (never as the real drone's)"() {
      const s = await stack({ seed: 48, videoDelay: 150, depth: false, changes: false });
      await placeAt(s, 2.4, 2.4, 0);
      const r = await s.drive(s.missions.run({ kind: "calibrate" }), 90), b = s.settings.get("simBrake");
      console.log(`      "${r.summary}" -> ${JSON.stringify(b)}`);
      assert.ok(r.ok && b && b.decel > 0.25 && b.decel < 3 && b.react > 0 && b.react < 1, r.summary);
      assert.ok(s.safety.avoid.brake().measured && Math.abs(s.safety.avoid.brake().decel - b.decel) < 0.01);
      assert.equal(s.settings.get("brake"), null, "the real drone's braking stays unmeasured");
    },

    async "video that isn't the camera's picture: black bars are found; a mission on the real drone refuses it"() {
      const W = 96, H = 54, pillar = (osd) => { // the O4's 4:3 picture in a 16:9 frame, OSD text in the bars
        const g = new Uint8Array(W * H);
        for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) g[y * W + x] = x >= 12 && x < 84 ? 60 + ((x * 7 + y * 13) % 120) : osd && y % 9 === 1 && x % 5 < 3 ? 230 : 8;
        return g;
      };
      const f = new PictureFinder();
      for (let i = 0; i < 25; i++) f.add(pillar(i % 3 === 0), W, H);
      const dark = new PictureFinder();
      for (let i = 0; i < 25; i++) dark.add(new Uint8Array(W * H).fill(10), W, H);
      const p = f.rect();
      console.log(`      pillarboxed: the picture is x ${fmt(p?.x)}-${fmt(p && p.x + p.w)}, y ${fmt(p?.y)}-${fmt(p && p.y + p.h)}; a dark room: ${JSON.stringify(dark.rect())}`);
      assert.ok(p && Math.abs(p.x - 0.125) < 0.02 && Math.abs(p.w - 0.75) < 0.03 && p.h > 0.95 && dark.rect() === null);
      const per = (w, h) => ({ latest: { width: w, height: h } });
      const probs = [[1440, 1080], [1920, 1080], [1920, 1200], [1080, 1080]].map(([w, h]) => [`${w}x${h}`, pictureProblem(per(w, h))]);
      console.log(`      ${probs.map(([k, v]) => `${k}: ${v || "ok"}`).join("; ")}`);
      assert.ok(!probs[0][1] && !probs[1][1] && probs[2][1] && probs[3][1]);
      // the real drone, a square crop: the mission refuses before flying
      const r = rig({ house, map, autonomy: "full", seed: 49, aspect: 1 }), settings = r.ctl.settings;
      Object.assign(r.values, { mode: "real" });
      const loc = new Localizer({ ctl: r.ctl, map, house, settings });
      loc.setSource("truth");
      loc.setTruth(() => ({ x: r.sim.drone.x, y: r.sim.drone.y, z: r.sim.drone.z, yaw: r.sim.drone.yaw }));
      r.ctl.safety = new Safety({ map, localizer: loc, settings, house });
      const m = new MissionRunner({ ctl: r.ctl, map, house, localizer: loc, perception: r.perception, settings });
      let out = null;
      m.run({ kind: "goTo", target: "Living room" }).then((v) => (out = v));
      for (let i = 0; i < 60 && !out; i++) (r.wait(1 / 30), await tick());
      console.log(`      a 320x320 frame on the real drone: "${out?.summary}"`);
      assert.ok(out && !out.ok && /crop/.test(out.summary) && !r.sim.drone.airborne, out?.summary);
    },
  };
}
