// The house model, map and planner (app/js/house/) on a real capture cut down to a fixture (fixtures/house, made by
// make-house-fixture.mjs): the W -> H frame, import and its sanity checks, the occupancy map (walls, doorways,
// the ceiling fan, stairs), plans, viewpoints, patrols, the store, and speed. Synthetic houses cover what this one
// lacks: a doorway through a real wall, low ceilings, big rooms.
// HOUSE_CAPTURE=<SiteSpec project folder> also runs the capture checks on the full capture (its 76 MB splat).
// Usage: cd tools && node test-house.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";

const F = await import("../app/js/house/frames.js");
const I = await import("../app/js/house/import.js");
const { HomeMap } = await import("../app/js/house/homemap.js");
const P = await import("../app/js/house/planner.js");
const S = await import("../app/js/house/store.js");

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir, only) => ({
  name: path.basename(dir),
  read: (p) => (only && !only.includes(p) ? null : fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (only ? [] : fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const json = (p) => JSON.parse(fs.readFileSync(path.join(FIXTURE, p), "utf8"));
const near = (a, b, tol, what) => assert.ok(Math.abs(a - b) <= tol, `${what}: ${a} vs ${b} (±${tol})`);
const SIGMA = 0.25, SAFE = 0.05 + SIGMA;
const segDist = ([x, y], a, b) => {
  const dx = b[0] - a[0], dy = b[1] - a[1], t = Math.max(0, Math.min(1, ((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy)));
  return Math.hypot(x - a[0] - t * dx, y - a[1] - t * dy);
};
// The chimney's brick face (r1 north wall) where the tape check ends, y = 5.7614 in the plan, against the splat.
const TAPE_WALL = json("site.json").checks.find((c) => c.id === "t2").b[1];
function northWall(points) {
  const hist = new Float32Array(81);
  for (let i = 0; i < points.opacity.length; i++) {
    const [x, y, z] = [points.xyz[3 * i], points.xyz[3 * i + 1], points.xyz[3 * i + 2]];
    if (points.opacity[i] >= 102 && points.scale[i] < 0.05 && x > 2.5 && x < 4.4 && y > 5.4 && y < 6.2 && z > 0.4 && z < 2.2)
      hist[Math.round((y - 5.4) * 100)] += 1;
  }
  return 5.4 + hist.indexOf(Math.max(...hist)) / 100;
}
// A SiteSpec project from plain objects (paths -> JSON), for synthetic houses.
const memorySource = (files) => ({
  name: "synthetic",
  read: (p) => (files[p] ? new TextEncoder().encode(JSON.stringify(files[p])) : null),
  list: (p) => Object.keys(files).filter((f) => f.startsWith(p + "/")).map((f) => f.slice(p.length + 1)),
});
// A 4 x 4 m room in plan coordinates (rooms.json: x right, y up, heights above its own floor) with doors on walls
// w1..w4 (south, east, north, west): [wall, start, width].
function planRoom(id, x0, { floorY = 0, ceiling = 2.5, doors = [] } = {}) {
  const o = [[x0, 0], [x0 + 4, 0], [x0 + 4, 4], [x0, 4]], walls = o.map((a, i) => ({ id: `w${i + 1}`, a, b: o[(i + 1) % 4] }));
  const openings = doors.map(([wall_id, start, width], i) => ({ id: `o${i + 1}`, wall_id, kind: "door", start, width, sill: 0, head: 2.0 }));
  return { id, name: id, model: { outline: o, floor_y: floorY, ceiling_height: ceiling, walls, openings } };
}
const synthetic = (rooms) => I.importCapture(memorySource({
  "outputs/scene.json": { units: "meters", alignment: { scale_source: "tags" }, tags: [], cameras: [] },
  "outputs/plans/test-rooms.json": { wall_thickness_nominal: 0.12, rooms },
}));
// A bare antimatter15 .splat made from the fixture's centres.
const centres = await I.readSplatPoints(fs.readFileSync(path.join(FIXTURE, "outputs/splat-centres.bin")));
function dotSplat(pts = centres) {
  const buf = new Uint8Array(32 * pts.n), dv = new DataView(buf.buffer);
  for (let i = 0; i < pts.n; i++) {
    for (let j = 0; j < 3; j++) dv.setFloat32(32 * i + 4 * j, pts.xyz[3 * i + j], true);
    for (let j = 3; j < 6; j++) dv.setFloat32(32 * i + 4 * j, pts.scale[i], true);
    buf.set([128, 128, 128, pts.opacity[i], 255, 128, 128, 128], 32 * i + 24);
  }
  return buf;
}

const t0 = performance.now();
const fixture = await I.importCapture(fsSource(FIXTURE), { keepPoints: true });
const importMs = performance.now() - t0;
const { house, map } = fixture;
const internal = house.doors.filter((d) => d.rooms[0] && d.rooms[1]);

// Everything the acceptance list asks of a capture of this house, for the fixture and the full capture alike.
function checkCapture({ house, map, points, warnings }, label) {
  const tag = (id) => house.tags.find((t) => t.id === id);
  near(tag(3).center[2], 0, 0.01, `${label}: tag 3 (living-room floor) z`);
  near(tag(0).center[2], 0.245, 0.01, `${label}: tag 0 (entry floor) z`);
  const room = (id) => house.rooms.find((r) => r.id === id);
  near(room("r2").floorZ - room("r1").floorZ, 0.245, 0.01, `${label}: step from the living room up to the entry`);
  for (const c of house.checks.floors) assert.ok(c.splatZ != null && Math.abs(c.dz) <= 0.03, `${label}: splat floor in ${c.room} at ${c.splatZ} vs plan ${c.floorZ}`);
  near(house.checks.planScale, 1, 1e-4, `${label}: plan drawn with the current tape correction`);
  assert.ok(Math.abs(house.checks.scale.scale - 1) <= 0.005, `${label}: splat walls vs plan walls scale ${house.checks.scale.scale}`);
  assert.ok(!warnings.some((w) => /scale|floor/.test(w.code)), `${label}: no scale or floor warnings: ${warnings.map((w) => w.text)}`);

  const wall = northWall(points);
  near(wall, TAPE_WALL, 0.03, `${label}: splat north wall of the living room vs the tape-checked wall`);

  const fans = map.keepouts.filter((k) => k.kind === "fan");
  assert.ok(fans.some((k) => Math.hypot(k.x - 2.95, k.y - 2.92) <= 0.25 && k.r >= 1.0), `${label}: fan keep-out near (2.95, 2.92): ${JSON.stringify(fans)}`);
  for (const k of fans) for (const d of house.doors) assert.ok(segDist([k.x, k.y], d.a, d.b) > 0.5, `${label}: a "fan" at door ${d.id}'s header`);

  const doors = house.doors.filter((d) => d.rooms[0] && d.rooms[1]);
  assert.equal(doors.length, 4, `${label}: internal openings ${doors.map((d) => d.id)}`);
  for (const d of doors) assert.ok(d.passable && Math.min(...d.clearance) >= SAFE, `${label}: ${d.id} passable at σ ${SIGMA}: clearance ${d.clearance}`);
  for (const r of house.rooms) {
    const goal = P.roomCenter(map, r.id), p = P.plan(map, [house.home.x, house.home.y], goal, { alt: 1.0, sigma: SIGMA });
    assert.ok(p.ok, `${label}: home -> ${r.name}: ${p.reason}`);
    assert.ok(p.minClearance >= SAFE, `${label}: home -> ${r.name} min clearance ${p.minClearance}`);
  }
  return { wall, fan: fans[0] };
}

const tests = {
  "frames: W -> H per docs/HOME-DRONE.md (scale, floor, round trip, three.js root, camera poses, yaw)"() {
    const [site, roomsAuto, rooms, scene] = [json("site.json"), json("work/room/rooms.auto.json"), json("outputs/plans/house-rooms.json"), json("outputs/scene.json")];
    const fr = F.houseFrame({ site, roomsAuto, rooms, scene });
    assert.equal(fr.f, 0.9879);
    assert.equal(fr.Yf, 0.2483);
    const corner = roomsAuto.rooms.find((r) => r.id === "r2").model.outline[0];
    const h = fr.toH([corner[0], 0, corner[1]]);
    near(h[0], 0.497, 0.001, "r2 corner x");
    near(h[1], 2.869, 0.001, "r2 corner y");
    // rooms.json's splat-measured rooms are rooms.auto in H, vertex for vertex
    for (const id of ["r2", "r3"]) {
      const a = roomsAuto.rooms.find((r) => r.id === id).model.outline, b = rooms.rooms.find((r) => r.id === id).model.outline;
      a.forEach((p, i) => {
        const q = fr.toH([p[0], 0, p[1]]);
        assert.ok(Math.hypot(q[0] - b[i][0], q[1] - b[i][1]) < 0.001, `${id} vertex ${i}`);
      });
    }
    const p = [1.234, -0.567, 4.321], back = fr.toW(fr.toH(p));
    p.forEach((v, i) => near(back[i], v, 1e-9, "round trip"));
    const m = fr.threeRoot, three = [0, 1, 2].map((r) => m[r] * p[0] + m[4 + r] * p[1] + m[8 + r] * p[2] + m[12 + r]);
    F.hToThree(fr.toH(p)).forEach((v, i) => near(three[i], v, 1e-9, "three.js root matrix"));
    F.threeToH(three).forEach((v, i) => near(v, fr.toH(p)[i], 1e-9, "three -> H"));
    near(fr.toH([0, 0.2483, 0])[2], 0, 1e-12, "the lowest floor is z = 0");
    for (const c of scene.cameras) {
      const pose = fr.camToH(c.rotation, c.position), R = pose.R;
      const det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
      near(det, 1, 0.01, "camera rotation stays proper");
      assert.ok(pose.p[2] > 1.4 && pose.p[2] < 2.6, `capture camera height ${pose.p[2]}`);
    }
    // Controller heading is clockwise-positive: turning right by 0.5 rad lowers H yaw by 0.5.
    const ref = { yaw0: Math.PI / 2, heading0: 3.0 };
    near(F.yawFromHeading(3.5, ref), Math.PI / 2 - 0.5, 1e-12, "yaw from heading");
    for (const yaw of [-3, -1, 0, 2, 3.1]) near(F.yawFromHeading(F.headingFromYaw(yaw, ref, 40), ref), yaw, 1e-9, "heading <-> yaw");
    assert.ok(Math.abs(F.headingFromYaw(-Math.PI / 2, ref, 40) - 40) <= Math.PI, "the heading setpoint stays within half a turn of the current heading");
  },

  "capture checks on the fixture (tags, floor step, plan and splat scale, the chimney wall, fan, doorways, plans)"() {
    const r = checkCapture(fixture, "fixture");
    console.log(`      north wall ${r.wall.toFixed(3)} m, fan keep-out (${r.fan.x}, ${r.fan.y}) r ${r.fan.r.toFixed(2)} m, splat scale ${house.checks.scale.scale} (${house.checks.scale.inliers} walls agree)`);
  },

  "import: rooms, ceilings, doors, windows, tags, landmarks, keep-outs, home pad, orthophoto, camera path"() {
    assert.equal(house.version, 1);
    assert.equal(house.source.kind, "sitespec");
    assert.deepEqual(house.rooms.map((r) => r.id), ["r1", "r2", "r3"]);
    assert.equal(house.rooms[0].name, "Living room", "the AI scene's room name");
    for (const id of ["r2", "r3"]) {
      const r = house.rooms.find((q) => q.id === id);
      assert.ok(r.ceiling.map && r.ceiling.z - r.floorZ < 2.6, `${id}: ceiling from the ceiling map (${r.ceiling.z}), not ceiling_height`);
    }
    assert.ok(fixture.warnings.some((w) => w.code === "ceiling"), "the wrong ceiling_height is reported");
    assert.equal(map.roomAt(-1, 1.5).id, "r2", "overlapping outlines: the smallest room owns a point");
    assert.equal(map.roomAt(3, 3).id, "r1");
    near(map.floorAt(-1, 1.5), 0.242, 0.001, "entry floor");
    near(map.ceilingAt(-1, 1.5), 2.69, 0.06, "entry ceiling from the map");
    assert.equal(house.doors.filter((d) => d.rooms.includes("r2") && d.rooms.includes("r3")).length, 1, "a doorway listed by both rooms is one door");
    assert.ok(house.doors.every((d) => d.kind !== "window") && house.windows.length >= 10, "windows are windows");
    assert.deepEqual(house.doors.filter((d) => !d.passable).map((d) => d.id), ["r2:w2-o1", "r3:w1-o1"], "the two high pass-throughs (sills 2.67 and 0.44 m) are not doorways");
    assert.equal(house.tags.length, 6);
    near(house.tags.find((t) => t.id === 2).normal[0], -1, 0.01, "tag 2 on the east wall faces into the room (−x)");
    assert.ok(house.landmarks.some((l) => l.name === "sofa" && l.source === "roomplan" && l.room === "r1"), "RoomPlan objects as landmarks");
    assert.ok(house.landmarks.some((l) => l.source === "ai" && /fireplace/i.test(l.name)), "AI scene fixtures as landmarks");
    assert.ok(house.landmarks.some((l) => l.source === "splat" && l.name === "ceiling fan"));
    const stairs = house.keepouts.filter((k) => k.kind === "stairs");
    assert.equal(stairs.length, 2, "stairs from RoomPlan and from the AI scene");
    assert.ok(!map.free(-1.8, 5.5, 1.242) && !map.free(-1.34, 4.2, 1.242), "no flying over the stairs");
    assert.ok(Math.hypot(house.home.x, house.home.y) < 0.3 && map.free(house.home.x, house.home.y, 1.242), `home pad by tag 0: ${JSON.stringify(house.home)}`);
    near(house.home.yaw, Math.PI / 2, 1e-4, "home faces tag 0's arrow (+y)");
    assert.deepEqual(Object.keys(house.orthophoto), ["x0", "y0", "res", "width", "height"]);
    near(house.orthophoto.res, 0.0148185, 1e-9, "orthophoto resolution (already ×f)");
    assert.equal(house.cameras.length, 30);
    assert.ok(house.cameras.every((c) => c.length === 4));
    assert.equal(house.frame.f, 0.9879);
  },

  "import: a wrong tape correction is reported"() {
    return (async () => {
      const wrong = await I.importCapture(fsSource(FIXTURE), { f: 1 });
      assert.ok(wrong.warnings.some((w) => w.code === "plan-scale"), "plan vs capture: 1.2% off");
      const worse = await I.importCapture(fsSource(FIXTURE), { f: 0.97 });
      assert.ok(worse.warnings.some((w) => w.code === "scale"), `splat walls vs plan walls 1.8% off: ${worse.house.checks.scale.scale}`);
    })();
  },

  "map: walls with doorway gaps and headers for the simulator, keep-outs, clearance, stats"() {
    const walls = map.walls();
    for (const d of internal) {
      const mid = [(d.a[0] + d.b[0]) / 2, (d.a[1] + d.b[1]) / 2];
      assert.ok(walls.filter((w) => w.kind === "wall").every((w) => segDist(mid, w.a, w.b) > 0.3), `${d.id}: no wall across the doorway`);
      const below = d.headZ < house.rooms.find((r) => r.id === d.rooms[0]).ceiling.z; // r3's north opening reaches the ceiling
      assert.equal(walls.some((w) => w.kind === "header" && w.door === d.id && w.zMin === d.headZ), below, `${d.id}: header above the doorway`);
    }
    assert.ok(!map.free(10, 10, 1) && map.clearance(-5, 0, 1) === 0, "outside the house is blocked");
    assert.ok(map.clearance(-0.6, 2.0, 1.242) > 0.5, "the middle of the entry is open");
    const st = map.stats();
    near(st.interior_m2, 42.8, 0.3, "interior area");
    assert.ok(st.flyable_m2[1] > 12 && st.flyable_m2[1.4] > st.flyable_m2[1], `flyable area ${JSON.stringify(st.flyable_m2)}`);
    console.log(`      ${st.grid}, ${st.splats} splats, flyable at σ ${SIGMA}: ${JSON.stringify(st.flyable_m2)} m², ${walls.length} wall pieces`);
  },

  "planner: the internal doorways are flyable at σ 0.25, plans report doors, impossible plans give a reason"() {
    const graph = P.roomGraph(map, { sigma: SIGMA });
    for (const e of graph.edges) {
      assert.ok(e.open, `${e.door} open`);
      const p = P.plan(map, e.wa, e.wb, { alt: 1.0, sigma: SIGMA, climb: false });
      assert.ok(p.ok && p.doors.some((d) => d.id === e.door), `through ${e.door}: ${p.reason ?? p.doors.map((d) => d.id)}`);
      assert.ok(p.minClearance >= SAFE);
      const d = p.doors.find((q) => q.id === e.door);
      assert.ok(Math.hypot(d.pre[0] - e.wa[0], d.pre[1] - e.wa[1]) < 1e-9, "door waypoint on the side the path comes from");
    }
    assert.equal(P.roomRoute(graph, "r2", "r1").length, 1);
    const start = house.cameras[0], p = P.plan(map, start, P.roomCenter(map, "r1"), { sigma: SIGMA });
    assert.ok(p.ok, `from the capture start: ${p.reason}`);
    for (const q of p.path) near(q[2] - map.floorAt(q[0], q[1]), [0.6, 1.0, 1.4].find((b) => Math.abs(q[2] - map.floorAt(q[0], q[1]) - b) < 1e-6) ?? -1, 1e-6, "path z = floor + a band");
    const out = P.plan(map, [house.home.x, house.home.y], [9, 9]);
    assert.ok(!out.ok && /goal/.test(out.reason), out.reason);
    const fat = P.plan(map, [house.home.x, house.home.y], P.roomCenter(map, "r3"), { sigma: 0.9 });
    assert.ok(!fat.ok && fat.reason, "σ 0.9 m fits nowhere");
    const low = P.plan(map, [house.home.x, house.home.y], P.roomCenter(map, "r1"), { alt: 1.0, climb: false });
    const climbing = P.plan(map, [house.home.x, house.home.y], P.roomCenter(map, "r1"), { alt: 1.0 });
    assert.ok(climbing.ok && (!low.ok || low.length >= climbing.length - 1e-9), "climbing over furniture never makes a plan worse");
    console.log(`      home -> living room ${climbing.length.toFixed(2)} m at ${climbing.bands.join("/")} m, climb ${climbing.climb.toFixed(1)} m, through ${climbing.doors.map((d) => d.id)}`);
  },

  "planner: viewpoints for people and pets, patrol from the home pad"() {
    for (const r of house.rooms) {
      const v = P.viewpoints(map, r.id, "person", { sigma: SIGMA });
      assert.ok(v.points.length <= 2 && v.coverage >= 0.9, `${r.id} person: ${v.points.length} points, ${v.coverage}`);
      assert.ok(v.points.every((q) => map.free(q.x, q.y, q.z, SAFE)) && v.points[0].headings.length >= 3);
    }
    for (const id of ["r2", "r3"]) assert.ok(P.viewpoints(map, id, "pet").coverage >= 0.9, `${id} pet coverage`);
    const pet = P.viewpoints(map, "r1", "pet");
    const home = [house.home.x, house.home.y], tour = P.patrolRoute(map, undefined, home, { sigma: SIGMA });
    assert.ok(tour.ok && tour.stops.length === 3 && !tour.skipped.length, JSON.stringify(tour.skipped));
    const last = tour.legs.at(-1).path.at(-1);
    assert.ok(Math.hypot(last[0] - home[0], last[1] - home[1]) < 0.05, "patrol ends on the pad");
    assert.ok(tour.legs.every((l) => l.minClearance >= SAFE));
    console.log(`      patrol ${tour.stops.map((s) => s.room).join(" -> ")}: ${tour.length.toFixed(1)} m; pets in the living room: ${pet.points.length} scans see ${(pet.coverage * 100).toFixed(0)}%`);
  },

  "speed: map build < 300 ms, planner < 20 ms per query"() {
    const { xyz, opacity, scale } = fixture.points;
    let t = performance.now();
    const m = new HomeMap(house).addSplats(xyz, opacity, scale).finalize();
    const build = performance.now() - t;
    assert.ok(build < 300, `map build ${build.toFixed(0)} ms`);
    const goals = house.rooms.map((r) => P.roomCenter(m, r.id)), starts = [[house.home.x, house.home.y], ...internal.map((d) => d.a), ...goals];
    const ms = [];
    for (let k = 0; k < 3; k++)
      for (const s of starts)
        for (const g of goals) {
          t = performance.now();
          P.plan(m, s, g, { sigma: SIGMA });
          ms.push(performance.now() - t);
        }
    ms.sort((a, b) => a - b);
    const p95 = ms[Math.floor(ms.length * 0.95)];
    assert.ok(p95 < 20, `planner p95 ${p95.toFixed(1)} ms`);
    console.log(`      import ${importMs.toFixed(0)} ms, map build ${build.toFixed(1)} ms, ${ms.length} plans: median ${ms[ms.length >> 1].toFixed(1)} ms, p95 ${p95.toFixed(1)} ms, max ${ms.at(-1).toFixed(1)} ms`);
  },

  async "store: save, list, load, the occupancy cache, files, delete (memory adapter)"() {
    const store = await S.openStore(S.memoryDir());
    const before = JSON.parse(JSON.stringify(house));
    await store.saveImport(fixture);
    assert.deepEqual((await store.listHouses()).map((h) => [h.id, h.name, h.rooms]), [[house.id, house.name, 3]]);
    const loaded = await store.loadHouse(house.id);
    assert.deepEqual({ ...loaded, saved: 0 }, { ...before, saved: 0, splatFile: null }, "the fixture has centres, no renderable splat");
    const m = await store.loadMap(loaded);
    assert.ok(!m.stale && !m.rebuilt, "from the cache");
    for (const [x, y] of [[-0.6, 2], [2.5, 4.7], [4, 0.8]]) assert.equal(m.clearance(x, y, 1), map.clearance(x, y, 1), "same clearance from the cache");
    assert.deepEqual(m.keepouts.filter((k) => k.kind === "fan").length, 1, "the saved fan is not found twice");
    const coarse = await store.loadMap(loaded, { cell: 0.1 }), { xyz, opacity, scale } = fixture.points;
    const want = new HomeMap(loaded, { cell: 0.1 }).addSplats(xyz, opacity, scale).finalize();
    assert.ok(!coarse.stale && coarse.rebuilt, "a cache for another grid is not used: the map is rebuilt from the stored centres");
    assert.deepEqual(coarse.stats(), want.stats());
    await store.writeFile(house.id, "calib.json", { uptilt: 22 });
    assert.equal(JSON.parse(new TextDecoder().decode(await store.readFile(house.id, "calib.json"))).uptilt, 22);
    assert.equal(await store.readFile(house.id, "orthophoto.png"), null, "the fixture has no orthophoto image");
    assert.equal(await store.deleteHouse(house.id), true);
    assert.deepEqual(await store.listHouses(), []);
    assert.equal(await store.loadHouse(house.id), null);
  },

  async "import: a Spacial project (outputs only) and a bare splat get one 'Whole capture' room"() {
    const spacial = await I.importCapture(fsSource(FIXTURE, ["outputs/scene.json", "outputs/splat-centres.bin"]));
    assert.equal(spacial.house.source.kind, "spacial");
    assert.deepEqual(spacial.house.rooms.map((r) => r.name), ["Whole capture"]);
    assert.equal(spacial.house.frame.floorSource, "splat p98");
    assert.equal(spacial.house.tags.length, 6);
    assert.ok(spacial.warnings.some((w) => w.code === "no-rooms"));
    const c = P.roomCenter(spacial.map, "all");
    assert.ok(c && P.plan(spacial.map, c, [c[0] + 0.5, c[1]]).ok);
    const bare = await I.importCapture({ name: "living.splat", bytes: dotSplat() });
    assert.equal(bare.house.source.kind, "splat");
    assert.equal(bare.splat.name, "splat.splat");
    assert.equal(bare.house.rooms.length, 1);
    assert.ok(bare.warnings.some((w) => w.code === "bare"));
    assert.ok(bare.map.stats().flyable_m2[1] > 5);
  },

  async "splat readers: SPZ v3 and PLY decode to the same centres, opacity and scale"() {
    const xyz = [[1.25, -0.5, 3.75], [-2, 0.125, 0.0625]], alpha = [200, 90], logs = [[-4, -5, -6], [-2.5, -3, -3.5]];
    const n = 2, fb = 12, spz = new Uint8Array(16 + n * (9 + 1 + 3 + 3 + 4)), dv = new DataView(spz.buffer);
    [0x5053474e, 3, n].forEach((v, i) => dv.setUint32(4 * i, v, true));
    spz[12] = 0;
    spz[13] = fb;
    xyz.forEach((p, i) => p.forEach((v, j) => {
      const q = Math.round(v * (1 << fb)) & 0xffffff;
      spz.set([q & 255, (q >> 8) & 255, (q >> 16) & 255], 16 + 9 * i + 3 * j);
    }));
    spz.set(alpha, 16 + 9 * n);
    logs.forEach((l, i) => spz.set(l.map((v) => Math.round((v + 10) * 16)), 16 + 13 * n + 3 * i));
    const gz = new Uint8Array(await new Response(new Blob([spz]).stream().pipeThrough(new CompressionStream("gzip"))).arrayBuffer());
    const head = `ply\nformat binary_little_endian 1.0\nelement vertex ${n}\nproperty float x\nproperty float y\nproperty float z\nproperty float opacity\nproperty float scale_0\nproperty float scale_1\nproperty float scale_2\nend_header\n`;
    const ply = new Uint8Array(head.length + 28 * n), pv = new DataView(ply.buffer, head.length);
    ply.set(new TextEncoder().encode(head));
    xyz.forEach((p, i) => [...p, Math.log(alpha[i] / 255 / (1 - alpha[i] / 255)), ...logs[i]].forEach((v, j) => pv.setFloat32(28 * i + 4 * j, v, true)));
    for (const [name, bytes] of [["splat.spz", gz], ["splat.ply", ply]]) {
      const p = await I.readSplatPoints(bytes, name);
      assert.equal(p.n, n, name);
      xyz.flat().forEach((v, i) => near(p.xyz[i], v, 1 / (1 << fb), `${name} position`));
      alpha.forEach((a, i) => near(p.opacity[i], a, 1, `${name} opacity`));
      logs.forEach((l, i) => near(Math.log(p.scale[i]), Math.max(...l), 0.04, `${name} scale`));
    }
    const filtered = await I.readSplatPoints(gz, "splat.spz", { minOpacity: 100 });
    assert.equal(filtered.n, 1, "filters while decoding");
  },

  async "map: a doorway through a 12 cm wall is one door, bridged across the wall and flyable"() {
    // Outlines are the walls' inner faces, so rooms with a real wall between them are 0.12 m apart (SiteSpec draws
    // wall_thickness_nominal outside the measured face).
    const both = await synthetic([planRoom("A", 0, { doors: [["w2", 1.5, 0.9]] }), planRoom("B", 4.12, { doors: [["w4", 1.6, 0.9]] })]);
    assert.equal(both.house.doors.length, 1, `listed by both rooms: ${both.house.doors.map((d) => d.id)}`);
    const [d] = both.house.doors;
    near(d.depth, 0.12, 1e-6, "door depth = the wall between the faces");
    assert.deepEqual([...d.rooms].sort(), ["A", "B"]);
    assert.ok(P.roomGraph(both.map, { sigma: SIGMA }).edges.every((e) => e.open), "room graph open");
    const p = P.plan(both.map, [2, 2], [6.12, 2], { sigma: SIGMA });
    assert.ok(p.ok && p.minClearance >= SAFE && p.doors[0]?.id === d.id, `A -> B: ${p.reason ?? p.minClearance}`);
    const jambs = both.map.walls().filter((w) => w.kind === "wall" && w.door === d.id);
    assert.equal(jambs.length, 2, "the simulator gets the doorway's two jambs");
    assert.equal(both.map.walls().filter((w) => w.kind === "header" && w.door === d.id).length, 2, "and a header on each face");
    // Listed by one room only, with B's floor a step (0.2 m) higher, and a thicker wall.
    for (const gap of [0.12, 0.25]) {
      const one = await synthetic([planRoom("A", 0, { doors: [["w2", 1.5, 0.9]] }), planRoom("B", 4 + gap, { floorY: -0.2 })]);
      const q = P.plan(one.map, [2, 2], [6 + gap, 2], { sigma: SIGMA });
      assert.ok(q.ok && q.minClearance >= SAFE, `${gap} m wall, one listing: ${q.reason ?? q.minClearance}`);
      near(one.map.floorAt(4 + gap / 2, 1.95), 0.2, 1e-6, "the passage takes the higher floor");
      assert.equal(one.map.clearance(4 + gap / 2, 1.95, 2.2), 0, "and the door head");
    }
    const far = await synthetic([planRoom("A", 0, { doors: [["w2", 1.5, 0.9]] }), planRoom("B", 4.5)]);
    assert.ok(!P.plan(far.map, [2, 2], [6.5, 2]).ok, "0.5 m apart is not a doorway between them");
  },

  "map: clearance(x, y, z) is 0 below the floor, near the ceiling, at a door head and above the top band"() {
    assert.ok(map.free(-0.6, 2.0, 1.242) && map.free(-0.6, 2.0, 0.242), "entry: free at 1 m and on the floor (take-off, landing)");
    for (const z of [0.1, 2.6, 10, -0.26]) assert.ok(!map.free(-0.6, 2.0, z), `entry (floor 0.242, ceiling 2.697): not free at z ${z}`);
    for (const z of [3.63, 10]) assert.ok(!map.free(3.5, 1.2, z), `living room (ceiling 3.13): not free at z ${z}`);
    assert.ok(map.clearance(-0.6, 2.0) > 0.5, "no z: the 1 m band");
  },

  async "map: a low ceiling is enforced at the queried height; a house without rooms is refused"() {
    const low = await synthetic([planRoom("A", 0, { ceiling: 1.9 })]);
    assert.ok(low.map.free(2, 2, 1.4) && !low.map.free(2, 2, 1.6), "1.9 m ceiling: free at 1.4 m, not at 1.6 m (0.4 m margin)");
    assert.throws(() => new HomeMap({ rooms: [], doors: [] }), /house has no rooms/);
  },

  async "import: the outputs folder picked directly still gets the tape correction; stale plans are reported"() {
    const out = await I.importCapture(fsSource(path.join(FIXTURE, "outputs")), { keepPoints: true });
    assert.equal(out.house.frame.f, 0.9879, "f from the correction recorded beside the plans");
    near(out.house.frame.Yf, house.frame.Yf, 1e-4, "same lowest floor");
    assert.ok(out.warnings.some((w) => w.code === "outputs-folder"), "asks for the project folder");
    near(northWall(out.points), TAPE_WALL, 0.03, "splat north wall vs the tape-checked wall");
    const t1 = (h) => h.tags.find((t) => t.id === 1).center;
    t1(out.house).forEach((v, i) => near(v, t1(house)[i], 0.001, "tag 1 where the project-folder import puts it"));
    // site.json corrected again after the plans were drawn (and no rooms.auto to compare against)
    const src = fsSource(FIXTURE), site = json("site.json");
    site.scale_correction.factor = 0.9885;
    const stale = await I.importCapture({
      ...src,
      read: (p) => (p === "site.json" ? Buffer.from(JSON.stringify(site)) : p === "work/room/rooms.auto.json" ? null : src.read(p)),
    });
    assert.ok(stale.warnings.some((w) => w.code === "plan-scale"), `0.06% stale plans: ${stale.warnings.map((w) => w.code)}`);
  },

  async "planner: no start, goal or home pad gives a reason, not a TypeError"() {
    assert.deepEqual(P.plan(map, [0, 0], null), { ok: false, reason: "no goal" });
    assert.deepEqual(P.plan(map, P.roomCenter(map, "nope"), [0, 0]), { ok: false, reason: "no start" });
    const tour = P.patrolRoute(map, undefined, null);
    assert.ok(!tour.ok && /home/.test(tour.reason) && tour.stops.length === 0);
    const bare = await I.importCapture({ name: "living.splat", bytes: dotSplat() });
    assert.equal(bare.house.home, null, "a bare splat has no pad");
    assert.equal(P.patrolRoute(bare.map, undefined, bare.house.home).ok, false);
  },

  "speed: viewpoints and patrols in a 120 m² room stay under 100 ms"() {
    const big = { rooms: [{ id: "big", name: "Big", outline: [[0, 0], [12, 0], [12, 10], [0, 10]], floorZ: 0, ceiling: { z: 2.6 } }], doors: [] };
    const furniture = [];
    for (let bx = 1; bx < 11.5; bx += 2.5)
      for (let by = 1; by < 9.5; by += 2.5)
        for (let x = bx; x < bx + 1; x += 0.03)
          for (let y = by; y < by + 0.6; y += 0.03) for (let z = 0.05; z < 0.4 + bx / 8; z += 0.1) furniture.push(x, y, z); // 0.4-1.8 m tall
    const n = furniture.length / 3;
    for (const clutter of [false, true]) {
      const m = new HomeMap(big);
      if (clutter) m.addSplats(Float32Array.from(furniture), new Uint8Array(n).fill(230), new Float32Array(n).fill(0.02));
      m.finalize();
      for (const target of ["person", "pet"]) {
        const t = performance.now(), v = P.viewpoints(m, "big", target), ms = performance.now() - t;
        const what = `${clutter ? "furnished" : "empty"} ${target}: ${ms.toFixed(0)} ms, ${v.points.length} points see ${v.coverage}`;
        assert.ok(ms < 100 && v.coverage > 0.75, what);
      }
      let t = performance.now();
      assert.ok(P.patrolRoute(m, undefined, [0.5, 0.5]).ok);
      const first = performance.now() - t;
      t = performance.now();
      P.patrolRoute(m, undefined, [0.5, 0.5]);
      const again = performance.now() - t;
      assert.ok(first < 100 && again < first, `patrol ${first.toFixed(0)} ms, again ${again.toFixed(0)} ms (viewpoints cached)`);
    }
  },

  async "store: a damaged cache is rebuilt from the stored splat; re-importing keeps the user's edits; odd house.json files are listed"() {
    const store = await S.openStore(S.memoryDir());
    const first = await I.importCapture(fsSource(FIXTURE)), spot = P.roomCenter(first.map, "r1");
    assert.ok(first.map.free(...spot, 1.0));
    first.house.home = { x: 1, y: 2, yaw: 0, source: "user" };
    first.house.keepouts.push({ kind: "zone", x: spot[0], y: spot[1], r: 0.4, source: "user" });
    first.house.landmarks.push({ name: "dog bed", x: 4, y: 1, z: 0, source: "user" });
    Object.assign(first.house.rooms[0], { name: "Lounge", nameSource: "user" });
    await store.saveImport(first);
    const again = await I.importCapture(fsSource(FIXTURE));
    await store.saveImport(again);
    const h = await store.loadHouse(again.house.id);
    assert.deepEqual(h.home, first.house.home, "the user's home pad");
    assert.ok(h.keepouts.some((k) => k.source === "user") && h.landmarks.some((l) => l.name === "dog bed"), "the user's keep-out and landmark");
    assert.equal(h.rooms[0].name, "Lounge", "the user's room name");
    assert.ok(!again.map.free(...spot, 1.0), "the user's keep-out is in the imported map");
    await store.writeFile(h.id, "occupancy.bin", new Uint8Array(8));
    const m = await store.loadMap(h);
    assert.ok(!m.stale && m.rebuilt && !m.free(...spot, 1.0), "rebuilt from the stored centres, with the user's keep-out");
    assert.deepEqual(m.stats(), again.map.stats());
    assert.ok(!(await store.loadMap(h)).rebuilt, "and the cache is written again");
    // A bare .splat import keeps its own file name; its map is rebuilt from it.
    const bare = await I.importCapture({ name: "living.splat", bytes: dotSplat() });
    await store.saveImport(bare);
    const hb = await store.loadHouse(bare.house.id);
    assert.equal(hb.splatFile, "splat.splat");
    await store.writeFile(hb.id, "occupancy.bin", new Uint8Array(8));
    const mb = await store.loadMap(hb);
    assert.ok(mb.rebuilt && !mb.stale);
    assert.deepEqual(mb.stats(), bare.map.stats());
    // Nothing stored to rebuild from: stale, walls and keep-outs only.
    const walls = await synthetic([planRoom("A", 0)]);
    await store.saveHouse(walls.house);
    assert.equal((await store.loadMap(walls.house)).stale, true);
    await store.writeFile("hand-edited", "house.json", "{}");
    const listed = await store.listHouses();
    assert.ok(listed.some((x) => x.id === "hand-edited" && x.rooms === 0) && listed.length === 4, JSON.stringify(listed));
  },

  async "splat readers: PLY with other elements before or after the vertices, lists refused"() {
    const ply = ({ before, after, list }) => {
      const head = ["ply", "format binary_little_endian 1.0", ...(before ? ["element camera 2", "property uchar k"] : []),
        "element vertex 3", "property float x", "property float y", "property float z", ...(list ? ["property list uchar int idx"] : []),
        ...(after ? ["element extra 1000", "property uchar v"] : []), "end_header", ""].join("\n");
      const pre = before ? 2 : 0, u8 = new Uint8Array(head.length + pre + 36 + (after ? 1000 : 0)), dv = new DataView(u8.buffer);
      u8.set(new TextEncoder().encode(head));
      u8.fill(9, head.length, head.length + pre);
      for (let i = 0; i < 9; i++) dv.setFloat32(head.length + pre + 4 * i, i + 0.5, true);
      return u8;
    };
    for (const opts of [{ after: true }, { before: true }, { before: true, after: true }]) {
      const p = await I.readSplatPoints(ply(opts), "a.ply");
      assert.equal(p.n, 3, JSON.stringify(opts));
      assert.deepEqual([...p.xyz], [0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5], JSON.stringify(opts));
    }
    await assert.rejects(I.readSplatPoints(ply({ list: true }), "a.ply"), /PLY/);
  },

  "style: the house modules keep to the project's line width"() {
    const dir = path.join(import.meta.dirname, "..", "app", "js", "house");
    for (const f of fs.readdirSync(dir).filter((n) => n.endsWith(".js")))
      fs.readFileSync(path.join(dir, f), "utf8").split("\n").forEach((l, i) => assert.ok(l.length <= 150, `${f}:${i + 1} is ${l.length} characters`));
  },

  async "full capture (HOUSE_CAPTURE)"() {
    const dir = process.env.HOUSE_CAPTURE;
    if (!dir || !fs.existsSync(path.join(dir, "outputs", "scene.json"))) return console.log("      skipped: set HOUSE_CAPTURE=<SiteSpec project folder> to run it");
    const t = performance.now();
    const full = await I.importCapture(fsSource(dir), { keepPoints: true });
    const ms = performance.now() - t;
    assert.equal(full.house.cameras.length, 300);
    assert.ok(full.splat?.name === "splat.spz", "the renderer gets the SPZ");
    const r = checkCapture(full, "full capture");
    assert.equal(full.map.splats, map.splats, "the fixture holds every splat the map uses");
    assert.deepEqual(full.map.stats(), map.stats(), "same map from the fixture and the full capture");
    console.log(`      ${full.splat.bytes.length / 1e6 | 0} MB SPZ imported in ${ms.toFixed(0)} ms; north wall ${r.wall.toFixed(3)} m; fan (${r.fan.x}, ${r.fan.y})`);
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
