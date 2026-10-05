// The simulator inside the user's house (app/js/sim/twin-world.js, actors.js) on the capture fixture (fixtures/house):
// the world matches the map (rooms, floors, the home pad; rays at the flight bands stop where the map is blocked; the
// drone bounces off the map's walls and furniture), test-sim's flights with the real controller (take-off, hover,
// turns, moves, finding and approaching the cat, co-pilot, the radio failsafe) in the house's 4:3 camera geometry,
// people and pets that walk through doorways and never through walls, the 50 ms radio mixer, the battery, the layered
// raycast camera (headless, with a stand-in canvas; floors at negative coordinates too), a splat twin disposed
// mid-frame, and the test rig's own idea of "on the ground" on a raised floor.
// Usage: cd tools && node test-house-sim.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { rig, now, trueHeading, runTests, fakeCanvas, CAT_SIZE } from "./test-sim.mjs";

const I = await import("../app/js/house/import.js");
const { Simulator, BridgeLogic, MIXER_PERIOD } = await import("../app/js/sim/simulator.js");
const { makeHouseWorld } = await import("../app/js/sim/twin-world.js");
const B = await import("../app/js/behaviors.js");
const { DEG } = await import("../app/js/util.js");
const { MASK } = await import("../app/js/protocol.js");

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const { house, map } = await I.importCapture(fsSource(FIXTURE));
const room = (name) => house.rooms.find((r) => r.name === name);
const agl = (sim) => sim.drone.z - sim.drone.floorZ;
const houseRig = (o = {}) => {
  const r = rig({ house, map, ...o });
  quiet(r.sim.world);
  return r;
};
// People and the dog sit still in the living room's far corner, so flights see only what each test places.
function quiet(world) {
  const spots = [[4.3, 6.0], [3.9, 6.1], [4.5, 5.6]];
  world.actors.list.filter((a) => a.kind !== "cat").forEach((a, i) => a.place(...spots[i], "sit", 1e6));
}
// A free spot in the living room 0.8-1.6 m from the first thing (hits(face)) a ray meets 0.6 m up, and the yaw to it.
function aimAt(world, band, hits) {
  const r1 = map.rooms.find((r) => r.id === "r1").index, cl = map.clear[band];
  for (let r = 0; r < map.H; r += 3)
    for (let c = 0; c < map.W; c += 3) {
      const k = r * map.W + c, [x, y] = map.center(c, r);
      if (map.room[k] !== r1 || cl[k] < 0.4) continue;
      for (let a = 0; a < 360; a += 15) {
        const h = world.castRay(x, y, Math.cos(a * DEG), Math.sin(a * DEG), 1.6, map.floorZ[k] + 0.6);
        if (h && h.dist > 0.8 && hits(h.seg)) return { start: { x, y }, yaw: a * DEG };
      }
    }
  return null;
}

// A free floor spot in a room that the home pad sees at every height from the floor to 1 m.
function visibleSpot(world, roomId, from) {
  const rm = map.rooms.find((r) => r.id === roomId), cl = map.clear[map.bandOf(0.6)];
  for (let r = 0; r < map.H; r += 4)
    for (let c = 0; c < map.W; c += 4) {
      const k = r * map.W + c, [x, y] = map.center(c, r), floor = map.floorZ[k];
      if (map.room[k] !== rm.index || cl[k] < 0.35 || Math.hypot(x - from.x, y - from.y) < 3) continue;
      if ([0.1, 0.4, 0.7, 1.0].every((h) => !world.blocked(from.x, from.y, x, y, floor + h))) return [x, y];
    }
  return null;
}

const tests = {
  "world: rooms and names from the capture, per-room floors and ceilings, the drone on the home pad"() {
    const { sim } = houseRig();
    const w = sim.world;
    assert.deepEqual(w.rooms.map((r) => r.name), house.rooms.map((r) => r.name));
    assert.deepEqual([w.start.x, w.start.y], [house.home.x, house.home.y]);
    assert.equal(sim.truth().room, room("Room 2").name);
    assert.ok(Math.abs(sim.drone.z - room("Room 2").floorZ) < 1e-6, `on Room 2's floor: z ${sim.drone.z}`);
    for (const r of house.rooms) {
      const [x, y] = r.outline.reduce(([a, b], p) => [a + p[0] / r.outline.length, b + p[1] / r.outline.length], [0, 0]);
      if (map.roomAt(x, y)?.id !== r.id) continue; // the living room's outline holds the others
      assert.equal(w.roomAt(x, y).name, r.name);
      assert.equal(w.floorAt(x, y), map.floorAt(x, y));
      assert.ok(w.ceilingAt(x, y) > r.floorZ + 2, `${r.name} ceiling`);
    }
    const labels = w.items.filter((i) => i.label).map((i) => i.label);
    assert.ok(labels.includes("couch") && labels.includes("chair"), `RoomPlan furniture labelled: ${labels}`);
    console.log(`      ${w.solids.length} solids (${w.items.length} furniture blobs), ${w.faces.length} faces; start ${w.start.x}, ${w.start.y} in ${sim.truth().room}`);
  },

  "collisions where the map says: rays at the flight bands stop where the map is blocked, and only there"() {
    const w = makeHouseWorld(house, map, { seed: 1 }), { cell } = map.o;
    let rand = 12345;
    const rnd = () => (rand = (rand * 1103515245 + 12345) >>> 0) / 4294967296;
    // physically blocked map cells at height z: walls, outside the rooms, splat obstacles in the band at that height
    const blockedAt = (q, z) => {
      if (q < 0 || map.room[q] < 0 || map.wall[q]) return true;
      const b = map.bandOf(z - map.floorZ[q]);
      return Math.abs(z - map.floorZ[q] - map.o.bands[b]) < 0.01 && !!map.occ[b][q];
    };
    const nearBlocked = ([x, y], z) => {
      const c0 = Math.floor((x - map.x0) / cell), r0 = Math.floor((y - map.y0) / cell);
      for (let r = r0 - 2; r <= r0 + 2; r++)
        for (let c = c0 - 2; c <= c0 + 2; c++) {
          if (c < 0 || r < 0 || c >= map.W || r >= map.H) return true;
          const [cx, cy] = map.center(c, r);
          if (blockedAt(r * map.W + c, z) && Math.hypot(cx - x, cy - y) <= 0.06) return true;
        }
      return false;
    };
    const nearSolid = ([x, y], z) => w.solids.some((s) => {
      if (z < s.zMin || z > s.zMax) return false;
      const dx = x - s.cx, dy = y - s.cy, t = Math.abs(dx * s.ux + dy * s.uy) - s.hl, q = Math.abs(-dx * s.uy + dy * s.ux) - s.hw;
      return Math.hypot(Math.max(t, 0), Math.max(q, 0)) <= 0.06;
    });
    const bad = [];
    let n = 0;
    while (n < 900) {
      const b = Math.floor(rnd() * 3), k = Math.floor(rnd() * map.N);
      if (map.room[k] < 0 || map.clear[b][k] < 0.15) continue;
      const [x, y] = map.center(k % map.W, (k / map.W) | 0), floor = map.floorZ[k], z = floor + map.o.bands[b], a = rnd() * 2 * Math.PI;
      const dx = Math.cos(a), dy = Math.sin(a);
      let pm = null, other = false;
      for (let s = 0; s < 12 && !pm && !other; s += 0.01) {
        const q = map.idx(x + dx * s, y + dy * s);
        if (q >= 0 && map.room[q] >= 0 && Math.abs(map.floorZ[q] - floor) > 0.01) other = true; // another floor: the bands differ
        else if (blockedAt(q, z)) pm = map.center(q % map.W, (q / map.W) | 0);
      }
      if (other) continue;
      n++;
      const hit = w.castRay(x, y, dx, dy, 12, z), pw = hit && [x + dx * hit.dist, y + dy * hit.dist];
      if (!pw || !pm || !nearBlocked(pw, z) || !nearSolid(pm, z))
        bad.push({ from: [x, y, z].map((v) => +v.toFixed(2)), deg: +((a / DEG).toFixed(0)), world: pw?.map((v) => +v.toFixed(3)), map: pm?.map((v) => +v.toFixed(3)), kind: hit?.seg.kind });
    }
    console.log(`      ${n - bad.length} of ${n} rays: the world's first obstacle is where the map's is (within 6 cm)${bad.length ? `; not: ${JSON.stringify(bad[0])}` : ""}`);
    assert.ok(bad.length <= n * 0.001, `${bad.length} rays disagree: ${JSON.stringify(bad.slice(0, 6))}`);
  },

  "collisions: flown into the sofa and into a wall, the drone stops at the map's obstacle"() {
    const band = map.bandOf(0.6);
    for (const [what, hits, blocks] of [
      ["the sofa", (seg) => seg.item?.label === "couch", (k) => map.occ[band][k]],
      ["a wall", (seg) => seg.kind === "wall", (k) => map.wall[k] || map.room[k] < 0],
    ]) {
      const sim = new Simulator({ headless: true, seed: 2, house, map });
      quiet(sim.world);
      const aim = aimAt(sim.world, band, hits);
      assert.ok(aim, `${what}: a free spot 0.8-1.6 m in front of it`);
      sim.pilotRequest("takeoff");
      sim.assist.targetZ = 0.6;
      Object.assign(sim.drone, aim.start, { yaw: aim.yaw });
      for (let t = 0; t < 4; t += 1 / 60) sim.step(1 / 60);
      Object.assign(sim.drone, aim.start, { vx: 0, vy: 0, yaw: aim.yaw, yawRate: 0 });
      sim.keys.add("ArrowUp");
      let contact = null;
      for (let t = 0; t < 8 && !contact; t += 1 / 60) {
        sim.drone.lastImpact = 0;
        sim.step(1 / 60);
        if (sim.drone.lastImpact > 0) contact = { x: sim.drone.x, y: sim.drone.y, z: agl(sim), v: sim.drone.lastImpact };
      }
      assert.ok(contact, `${what}: no contact`);
      // within the drone's radius (+ a cell) of a cell the map blocks at that band, and not inside one
      let near = Infinity;
      for (let dy = -0.2; dy <= 0.2; dy += 0.01)
        for (let dx = -0.2; dx <= 0.2; dx += 0.01) {
          const k = map.idx(contact.x + dx, contact.y + dy);
          if (k >= 0 && blocks(k)) near = Math.min(near, Math.hypot(dx, dy));
        }
      console.log(`      ${what}: hit at ${contact.v.toFixed(1)} m/s at (${contact.x.toFixed(2)}, ${contact.y.toFixed(2)}), ${contact.z.toFixed(2)} m up, ${(near * 100).toFixed(0)} cm from the map's obstacle`);
      assert.ok(near <= 0.05 + 0.06, `${what}: contact ${near.toFixed(3)} m from the obstacle`);
      assert.ok(!blocks(map.idx(contact.x, contact.y)), `${what}: inside the obstacle`);
      assert.ok(!sim.drone.crashed, `${what}: a gentle bump is not a crash`);
    }
  },

  async "full auto: takes off from the pad (0.24 m up the entry step), hovers, lands"() {
    const { sim, ctl, run } = houseRig();
    const r = await run(new B.TakeOff(), 10);
    assert.ok(r.ok, r.text);
    const start = { x: sim.drone.x, y: sim.drone.y };
    await run(new B.Hold(8), 12);
    const h = agl(sim), drift = Math.hypot(sim.drone.x - start.x, sim.drone.y - start.y);
    console.log(`      hover ${h.toFixed(2)} m above Room 2's floor (z ${sim.drone.z.toFixed(2)}), drift ${drift.toFixed(2)} m in 8 s, hover model ${ctl.hover.estimate(sim.drone.vbat).toFixed(3)}`);
    assert.ok(h > 0.35 && h < 2.1, `height ${h.toFixed(2)}`);
    assert.ok(drift < 1.5, `drifted ${drift.toFixed(2)} m`);
    const l = await run(new B.Land(), 12);
    assert.ok(l.ok, l.text);
    assert.ok(!sim.drone.airborne && !sim.drone.crashed, "landed softly");
    assert.ok(Math.abs(sim.drone.z - room("Room 2").floorZ) < 0.01, `on the entry floor, z ${sim.drone.z.toFixed(3)}`);
  },

  async "full auto: turns 90 right and 135 left, then moves forward up the entry hall"() {
    const { sim, run } = houseRig({ seed: 4 });
    await run(new B.TakeOff(), 10);
    let h0 = trueHeading(sim);
    let r = await run(new B.Turn(90), 10);
    let turned = (trueHeading(sim) - h0) / DEG;
    assert.ok(r.ok && Math.abs(turned - 90) < 15, `turned ${turned.toFixed(1)}`);
    h0 = trueHeading(sim);
    await run(new B.Turn(-135), 10);
    const turned2 = (trueHeading(sim) - h0) / DEG;
    assert.ok(Math.abs(turned2 + 135) < 18, `turned ${turned2.toFixed(1)}`);
    await run(new B.Turn(45), 10); // north again, up Room 2 toward Room 3
    const p0 = { x: sim.drone.x, y: sim.drone.y, yaw: sim.drone.yaw };
    r = await run(new B.Move("forward", 1.5, 0.35), 12);
    await run(new B.Hold(1.5), 3);
    const along = (sim.drone.x - p0.x) * Math.cos(p0.yaw) + (sim.drone.y - p0.y) * Math.sin(p0.yaw);
    console.log(`      turned ${turned.toFixed(0)}° and ${turned2.toFixed(0)}°; moved ${along.toFixed(2)} m forward (asked 1.5), now in ${sim.truth().room}`);
    assert.ok(along > 0.7 && along < 2.8, `moved ${along.toFixed(2)} m`);
    assert.ok(!sim.drone.crashed);
  },

  async "full auto: finds the cat in the next room and approaches it"() {
    const { sim, run } = houseRig({ seed: 3 });
    const cat = sim.world.cat, spot = visibleSpot(sim.world, "r3", sim.world.start);
    assert.ok(spot, "a spot in Room 3 seen from the pad");
    cat.place(...spot, "sit", 600);
    await run(new B.TakeOff(), 10);
    const f = await run(new B.Search("cat"), 25);
    assert.ok(f.ok, f.text);
    const a = await run(new B.Approach("cat", { size: CAT_SIZE }), 45);
    const d = Math.hypot(cat.x - sim.drone.x, cat.y - sim.drone.y);
    console.log(`      cat at (${spot.map((v) => v.toFixed(2))}) in ${sim.world.roomAt(...spot).name}; approach: "${a.text}" ending ${d.toFixed(2)} m from it, drone in ${sim.truth().room}, ${agl(sim).toFixed(2)} m up`);
    assert.ok(a.ok, a.text);
    assert.ok(d > 0.45 && d < 2.2, `distance ${d.toFixed(2)}`);
    assert.ok(!sim.drone.crashed);
  },

  async "co-pilot: the (simulated) pilot takes off from the pad and holds height above the entry floor"() {
    const { sim, ctl, run } = houseRig({ autonomy: "copilot" });
    ctl.askPilot("takeoff", "take off please");
    const w = await run(new B.WaitFor("wait", (c) => c.isFlying(), 25, "flying", "not flying"), 26);
    assert.ok(w.ok, w.text);
    await run(new B.Hold(2), 4);
    const h0 = trueHeading(sim);
    const r = await run(new B.Turn(60), 10);
    const turned = (trueHeading(sim) - h0) / DEG;
    console.log(`      co-pilot turn ${turned.toFixed(0)}° (asked 60), ${agl(sim).toFixed(2)} m above the floor`);
    assert.ok(r.ok && Math.abs(turned - 60) < 15);
    assert.ok(agl(sim) > 0.5, "pilot kept height");
  },

  async "radio failsafe through the 50 ms mixer: the quad descends and idles instead of flying away"() {
    const { sim, run, wait } = houseRig();
    await run(new B.TakeOff(), 10);
    await run(new B.Hold(2), 4);
    assert.ok(sim.drone.airborne);
    wait(7, false);
    assert.ok(sim.bridge.failsafe, "bridge latched failsafe");
    assert.ok(!sim.drone.airborne, "on the ground");
    assert.ok(!sim.drone.crashed, `soft landing (last impact ${sim.drone.lastImpact.toFixed(2)} m/s)`);
  },

  "50 ms mixer: outputs change only when the script runs; failsafe latches 0.30-0.35 s after the last packet"() {
    const b = new BridgeLogic(), p = { roll: 0, pitch: 0, yaw: 0, thr: 0.4, aiSwitch: true }, beep = () => {};
    const cmd = (thr) => ({ seq: 1, mask: MASK.thr | MASK.pitch, roll: 0, pitch: 0.1, thr, yaw: 0, hover: 0.45 });
    let t = 0, changes = 0, last = null;
    const ticks = [];
    for (; t < 1; t += 1 / 240) {
      if (Math.round(t * 240) % 8 === 0) b.receive(cmd(0.4 + 0.1 * Math.sin(t * 10))); // the app at 30 Hz
      const o = b.outputs(p, t, beep);
      if (last && o.thr !== last.thr) changes++, ticks.push(t);
      last = o;
    }
    assert.equal(MIXER_PERIOD, 0.05);
    assert.ok(changes >= 17 && changes <= 20, `${changes} changes in 1 s`);
    assert.ok(ticks.every((v, i) => !i || v - ticks[i - 1] >= 0.05 - 1e-6), "never faster than 20 Hz");
    const lastRx = b.lastRx;
    let latched = null;
    for (; t < 2 && latched == null; t += 1 / 240) {
      b.outputs(p, t, beep);
      if (b.failsafe) latched = t - lastRx;
    }
    console.log(`      ${changes} output changes in 1 s; failsafe ${(latched * 1000).toFixed(0)} ms after the last packet was read`);
    assert.ok(latched >= 0.3 && latched <= 0.35 + 1e-6, `latched after ${latched}`);
    const off = b.outputs({ ...p, aiSwitch: false, thr: 0.2 }, t, beep);
    assert.equal(off.thr, 0.2, "AI switch off: the pilot's stick at once");
  },

  "people and pets: walk between rooms through the doorways, never through walls; same seed, same day"() {
    const day = (seed) => {
      const w = makeHouseWorld(house, map, { seed });
      const log = new Map(w.actors.list.map((a) => [a.id, { rooms: new Set(), doors: 0, through: 0, states: new Set() }]));
      const prev = new Map(w.actors.list.map((a) => [a.id, [a.x, a.y, w.roomAt(a.x, a.y)?.id]]));
      for (let s = 0; s < 10 * 60 * 60; s++) {
        w.step(1 / 60, null);
        for (const a of w.actors.list) {
          const L = log.get(a.id), [px, py, pr] = prev.get(a.id), r = w.roomAt(a.x, a.y)?.id;
          L.rooms.add(r);
          L.states.add(a.state);
          if (r && pr && r !== pr) L.doors++;
          if (w.blocked(px, py, a.x, a.y)) L.through++;
          const k = map.idx(a.x, a.y);
          assert.ok(k >= 0 && map.room[k] >= 0 && !map.wall[k], `${a.id} at (${a.x.toFixed(2)}, ${a.y.toFixed(2)}) is off the floor plan`);
          assert.ok(Math.abs(a.z - map.floorAt(a.x, a.y) - a.onTop) < 1e-6, `${a.id} stands on the floor`);
          prev.set(a.id, [a.x, a.y, r ?? pr]);
        }
      }
      return { log, end: w.actors.list.map((a) => [a.id, +a.x.toFixed(4), +a.y.toFixed(4), a.state]) };
    };
    const { log, end } = day(5);
    for (const [id, L] of log) {
      console.log(`      ${id}: ${L.rooms.size} rooms, ${L.doors} doorway crossings, ${[...L.states].join("/")}`);
      assert.equal(L.through, 0, `${id} went through a wall`);
      assert.ok(L.rooms.size >= 2 && L.doors >= 2, `${id} stayed put: ${[...L.rooms]}`);
    }
    assert.ok([...log.values()].some((L) => L.states.has("sit")) && [...log.values()].some((L) => L.states.has("lie") || L.states.has("nap")), "sitting and lying");
    assert.deepEqual(day(5).end, end, "deterministic");
  },

  "battery: a full 480 mAh pack hovers about 4 minutes (3.3 V under load)"() {
    const sim = new Simulator({ headless: true, seed: 1, house, map });
    quiet(sim.world);
    sim.pilotRequest("takeoff");
    let t = 0, low = null;
    for (; t < 600 && low == null; t += 1 / 60) {
      sim.step(1 / 60);
      if (sim.drone.airborne && sim.drone.vbat < 3.3) low = t;
    }
    console.log(`      3.3 V under load after ${(low / 60).toFixed(2)} min, ${Math.round(sim.drone.soc * 100)}% left`);
    assert.ok(low > 3.5 * 60 && low < 5 * 60, `${low} s`);
  },

  "layered raycast camera (stand-in canvas): headers, floors, the couch's ground-truth box"() {
    const restore = fakeCanvas();
    try {
      const sim = new Simulator({ seed: 1, house, map });
      quiet(sim.world);
      const sofa = sim.world.items.find((i) => i.label === "couch" && i.cells > 100);
      Object.assign(sim.drone, { x: sofa.x + 2.2, y: sofa.y - 0.2, z: 1.0, yaw: Math.atan2(0.2, -2.2) });
      const t0 = Date.now();
      for (let i = 0; i < 5; i++) sim.renderCamera();
      const ms = (Date.now() - t0) / 5;
      const dets = sim.video.detections, couch = dets.find((d) => d.label === "couch");
      console.log(`      ${sim.video.canvas.width}x${sim.video.canvas.height}, ${ms.toFixed(1)} ms a frame; boxes: ${dets.map((d) => `${d.label} ${d.dist.toFixed(1)} m`).join(", ")}`);
      assert.equal(sim.video.canvas.width / sim.video.canvas.height, 4 / 3);
      const demo = new Simulator({ seed: 1 });
      demo.renderCamera();
      assert.equal(demo.video.canvas.width / demo.video.canvas.height, 4 / 3, "the demo apartment is 4:3 too");
      assert.ok(couch && Math.abs(couch.box.x + couch.box.w / 2 - 0.5) < 0.15, `couch ahead: ${JSON.stringify(couch)}`);
      const z = sim.renderer.zpix, W = sim.renderer.W;
      assert.ok(z.every((v) => v > 0 && v <= 30), "every pixel shows something");
      assert.ok(z[(sim.renderer.H - 1) * W + W / 2] < 2, "floor right below");
    } finally {
      restore();
    }
  },

  "layered raycast camera: floors keep their texture west and south of the house's origin (negative coordinates)"() {
    const restore = fakeCanvas();
    try {
      const sim = new Simulator({ seed: 1, house, map });
      quiet(sim.world);
      sim.renderer.noise = 0;
      Object.assign(sim.drone, { x: -0.3, y: 1.4, z: room("Room 2").floorZ + 0.3, yaw: Math.PI }); // facing west: all x < -0.3
      sim.renderCamera();
      const { W, H, image: { data } } = sim.renderer, like = (r, g, b, [R, G, B]) => Math.abs(g / r - G / R) < 0.025 && Math.abs(b / r - B / R) < 0.025;
      let plank = 0, groove = 0;
      for (let i = W * Math.round(H * 0.75) * 4; i < data.length; i += 4) {
        const [r, g, b] = data.subarray(i, i + 3);
        if (r < 50) continue;
        if (like(r, g, b, [178, 142, 102])) plank++;
        else if (like(r, g, b, [95, 70, 45])) groove++;
      }
      console.log(`      Room 2's floor west of x = 0: ${plank} plank pixels, ${groove} groove pixels`);
      assert.ok(plank > 2000 && groove < plank * 0.3, `planks ${plank}, grooves ${groove}`);
    } finally {
      restore();
    }
  },

  async "splat camera: a twin disposed mid-frame by a world change is dropped quietly; a real failure is reported"() {
    const restore = fakeCanvas();
    try {
      const sim = new Simulator({ seed: 1, house, map }), errors = [];
      sim.on("error", (t) => errors.push(t));
      const a = fakeTwin();
      sim.setTwin(a);
      sim.renderCamera(); // a frame in flight
      await a.dispose();
      await settle();
      assert.deepEqual(errors, []);
      assert.equal(sim.twin, null, "back on the raycast camera");
      sim.renderCamera();
      assert.equal(sim.video.canvas.width, 320);
      const b = fakeTwin();
      sim.setTwin(b);
      sim.renderCamera();
      b.reply();
      await settle();
      assert.equal(sim.video.canvas.width, 640, "the twin's frame");
      sim.renderCamera();
      b.fail(new Error("WebGL context lost"));
      await settle();
      assert.equal(errors.length, 1);
      assert.match(errors[0], /splat camera failed.*context lost/);
    } finally {
      restore();
    }
  },

  async "splat camera under the GPU budget: while vision runs it is held to its cap (24 fps), and while it is under that rate its frames go ahead of live depth in the twin's queue (localization and depth work on them), behind it when over"() {
    const restore = fakeCanvas(), { GPU, BUDGET } = await import("../app/js/vision/budget.js"), { PRIO } = await import("../app/js/twin/queue.js");
    try {
      const sim = new Simulator({ seed: 1, house, map }), t = fakeTwin(), prios = [], render = t.render;
      t.render = (pose, view) => (prios.push(view.prio), render());
      sim.setTwin(t);
      for (let k = 0; k < 10; k++) GPU.mark("loc", 50, performance.now() - 150 * k); // vision localization runs
      sim.renderCamera();
      t.reply();
      await settle();
      for (let k = 0; k < 2 * BUDGET.cameraFps; k++) GPU.mark("camera", 20, performance.now() - 1999 + (k * 1000) / BUDGET.cameraFps); // at the cap over the window
      sim.camSlot = {};
      sim.renderCamera();
      t.reply();
      await settle();
      assert.deepEqual(prios, ["cameraBehind", "camera"]);
      assert.ok(PRIO.loc < PRIO.cameraBehind && PRIO.cameraBehind < PRIO.depth && PRIO.depth < PRIO.camera);
    } finally {
      for (const m of Object.values(GPU.marks)) m.length = 0;
      restore();
    }
  },

  "test rig: FakePerception sees the drone on the ground on a raised floor (Room 2, 0.24 m) as on the demo's"() {
    for (const [name, r] of [["demo", rig()], ["house", houseRig()]]) {
      const d = r.sim.drone;
      assert.ok(!d.airborne && d.z - d.floorZ < 1e-6, `${name}: on the floor`);
      Object.assign(d, { yawRate: 1.5, vx: 0.4 });
      r.perception.update();
      const f = r.perception.latest.flow;
      assert.ok(Math.abs(f.dx) < 0.011 && Math.abs(f.div) < 0.011, `${name}: on the ground, flow ${JSON.stringify(f)}`);
    }
  },
};

// Stands in for twin/twin.js's Twin: calls wait for reply() or fail(), and dispose() rejects them like Twin's.
function fakeTwin() {
  let dead = false;
  const pending = [];
  const call = (result) => () => (dead ? Promise.reject(new Error("twin disposed")) : new Promise((res, rej) => pending.push({ res: () => res(result()), rej })));
  return {
    stats: { renderMs: 4 },
    setLens: async () => {},
    setActors: async () => {},
    render: call(() => ({ width: 640, height: 480, close() {} })),
    oracle: call(() => []),
    reply: () => pending.splice(0).forEach((p) => p.res()),
    fail: (e) => pending.splice(0).forEach((p) => p.rej(e)),
    dispose: async () => ((dead = true), pending.splice(0).forEach((p) => p.rej(new Error("twin disposed")))),
  };
}
const settle = () => new Promise((r) => setTimeout(r, 0));

await runTests(tests);
