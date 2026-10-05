// No false changes (app/js/nav/changes.js, memory/memory.js), headless on the house fixture: live depth is the simulator
// world's own depth camera (test-avoid.mjs's depthCamera), the twin's expected view the same camera in the house as
// captured, and the 3D scan a voxel map made from the house map (free where the map is free, occupied where it isn't,
// unknown off this corner of the Living room). A new thing is reported only where the scan saw free space, from viewpoints
// apart that agree in 3D; nothing in space the scan never saw, nothing turning on the spot, nothing "gone" where the twin
// renders a smear or a faint layer (glass, an aquarium), nothing from frames below the trust input; what a suspected
// change blocks covers the whole thing (its place's error twice over); two reports of one box are one record covering
// both, two boxes a gap apart two records, each covered; one the camera keeps not seeing (in plain view, trusted
// frames) stops blocking, off view or untrusted it keeps blocking. Fix 3: the localized height 0.2 m off by a sideboard
// (the expected view rendered there, the live one from the true pose) makes no change, a box seen with that error still
// is one, and frames whose height is unsure are skipped.
// Usage: cd tools && node test-changes.mjs
import assert from "node:assert/strict";
import { depthCamera, house, map } from "./test-avoid.mjs";
import { runTests } from "./test-sim.mjs";

const { ChangeDetector, CHANGES, changeTrust } = await import("../app/js/nav/changes.js");
const { HouseMemory, MEMORY } = await import("../app/js/memory/memory.js");
const { makeHouseWorld } = await import("../app/js/sim/twin-world.js");
const V = await import("../app/js/house/voxels.js");

const fmt = (v) => (Number.isFinite(v) ? v.toFixed(2) : String(v));
const tick = () => new Promise((r) => setImmediate(r));
const CRATE = { id: "crate", x: 2.6, y: 2.0, z: 0, w: 0.5, d: 0.5, h: 0.7 };
const AREA = { x: [0.4, 3.6], y: [0.6, 3.4] };

// The scan of the area as a capture would have it: depth views of `world` from spots 0.6 m apart at two heights, six
// headings each, carve free space up to each surface and mark the surface occupied (FAINT where `faint` says: glass);
// space no view reached, and wherever `unknown` says, stays unknown.
const { droneCamera } = await import("../app/js/twin/pose.js");
const { intrinsics, unproject } = await import("../app/js/twin/lens.js");
function scan(world, { unknown = () => false, faint = () => false } = {}) {
  const vox = V.VoxelMap.forMap(map), hits = new Set(), rays = [];
  for (let x = AREA.x[0] + 0.3; x < AREA.x[1]; x += 0.6)
    for (let y = AREA.y[0] + 0.3; y < AREA.y[1]; y += 0.6)
      for (const h of [0.5, 1.2]) {
        const floor = map.floorAt(x, y);
        if (floor == null || !(map.clearance(x, y, floor + h) > 0.15)) continue;
        for (let k = 0; k < 6; k++) {
          const pose = { x, y, z: floor + h, yaw: (k * Math.PI) / 3, pitch: 0, roll: 0 }, cam = depthCamera(world, pose, { actors: false });
          const K = intrinsics(cam.lens, cam.width, cam.height), { p, R } = droneCamera(pose, cam.lens.uptiltDeg);
          for (let v = 0; v < cam.height; v++)
            for (let u = 0; u < cam.width; u++) {
              const r = unproject(K, u + 0.5, v + 0.5), d = [0, 1, 2].map((a) => R[a][0] * r[0] + R[a][1] * r[1] + R[a][2] * r[2]), t = cam.depth[v * cam.width + u];
              if (!(t < 8)) continue;
              const end = vox.idx(p[0] + d[0] * (t + 0.01), p[1] + d[1] * (t + 0.01), p[2] + d[2] * (t + 0.01));
              if (end >= 0) hits.add(end);
              rays.push([p, d, t]);
            }
        }
      }
  for (const [p, d, t] of rays)
    for (let s = 0.05; s < t - 0.05; s += 0.025) {
      const i = vox.idx(p[0] + d[0] * s, p[1] + d[1] * s, p[2] + d[2] * s);
      if (i >= 0 && !hits.has(i) && vox.st[i] !== V.FREE) vox.set(i, -32, V.FLAG.CARVED);
    }
  for (const i of hits) vox.set(i, 64, V.FLAG.CAPTURE | (faint(...vox.center(i)) ? V.FLAG.FAINT : 0));
  for (let i = 0; i < vox.n; i++) if (vox.st[i] && unknown(...vox.center(i))) vox.set(i, 0), (vox.flags[i] = 0);
  return vox;
}
const inCrate = (x, y, z, pad = 0.05) => Math.abs(x - CRATE.x) < CRATE.w / 2 + pad && Math.abs(y - CRATE.y) < CRATE.d / 2 + pad && z < CRATE.h + pad;
const withScan = (vox) => Object.create(map, { vox: { value: vox } }); // the house map with this 3D scan

// Views of the crate's spot: passing it sideways (0.77 m), or turning on one spot (no baseline), at 1 m up.
const toward = (x, y) => Math.atan2(CRATE.y - y, CRATE.x - x);
const strafe = Array.from({ length: 12 }, (_, i) => ({ x: 1.3, y: 1.3 + 0.07 * i, z: 1.0, yaw: toward(1.3, 1.3 + 0.07 * i), pitch: 0, roll: 0, sigma: 0.05, status: "ok" }));
const short = Array.from({ length: 12 }, (_, i) => ({ x: 1.95, y: 1.55 + 0.018 * i, z: 1.0, yaw: toward(1.95, 1.55 + 0.018 * i), pitch: 0, roll: 0, sigma: 0.05, status: "ok" }));
const lowStrafe = strafe.map((p) => ({ ...p, z: 0.65 })); // low enough to see the crate down to 0.1 m over the floor
const turn = Array.from({ length: 12 }, (_, i) => ({ x: 1.3, y: 1.7, z: 1.0, yaw: toward(1.3, 1.7) + 0.25 * Math.sin(i / 2), pitch: 0, roll: 0, sigma: 0.05, status: "ok" }));

// A detector watching `poses` (cycled over `frames`, 5 Hz) where `live` is the world now and `expected` the house as the
// scan has it. -> { det, found }
// dz: the localized height that far off (the frames' pose and the expected view there, the live view from the true one);
// zSigma: what the frames say their height's σ is.
async function watch({ live, expected, poses, vox = null, frames = 24, trust = null, memory = null, dz = 0, zSigma = null }) {
  for (const id of [...map.temps.keys()]) map.removeTemp(id);
  const det = new ChangeDetector({ map: vox ? withScan(vox) : map, house, trust, memory }), found = [];
  det.on("change", (c) => found.push(c));
  const views = poses.map((pose) => {
    const at = { ...pose, z: pose.z + dz, ...(zSigma != null && { zSigma }) };
    return { ...depthCamera(live, pose), pose: at, expected: depthCamera(expected, at, { actors: false }).depth };
  });
  for (let i = 0; i < frames; i++) det.ingest({ ...views[i % views.length], depthErr: 0.05, t: i * 200 }, i * 200);
  await tick();
  return { det, found };
}
const worlds = () => {
  const plain = makeHouseWorld(house, map, { seed: 40, cast: [] }), boxed = makeHouseWorld(house, map, { seed: 40, cast: [] });
  boxed.addObstacle(CRATE);
  return { plain, boxed };
};
const near = (c) => Math.hypot(c.x - CRATE.x, c.y - CRATE.y);
// The share of a box's footprint (w x d at x, y) inside the temporary obstacles on the map.
const covered = (b, temps = [...map.temps.values()]) => {
  let n = 0, inside = 0;
  for (let i = 0; i < 40; i++) for (let j = 0; j < 40; j++) {
    const x = b.x - b.w / 2 + ((i + 0.5) * b.w) / 40, y = b.y - b.d / 2 + ((j + 0.5) * b.d) / 40;
    n++, (inside += temps.some((t) => Math.hypot(x - t.x, y - t.y) <= t.r));
  }
  return inside / n;
};

const tests = {
  async "a new box where the scan saw free space: reported once, at its place, when views from apart agree; turning on one spot reports nothing"() {
    const { plain, boxed } = worlds(), vox = scan(plain);
    const a = await watch({ live: boxed, expected: plain, poses: strafe, vox });
    console.log(`      passing it: ${a.found.map((c) => `${c.kind} at (${fmt(c.x)}, ${fmt(c.y)}) ${fmt(c.size)} m, σ ${fmt(c.sigma)}`).join("; ") || "nothing"} (${a.det.stats.blobs} blobs, ${a.det.stats.scan} the scan ruled out)`);
    assert.equal(a.found.length, 1, "one report");
    assert.ok(a.found[0].kind === "obstacle" && near(a.found[0]) < 0.3, `at the box: ${fmt(near(a.found[0]))} m`);
    assert.ok(a.found[0].sigma >= CHANGES.placeErr, "σ said no better than the placement error measured");
    console.log(`      its obstacle on the map: r ${fmt(map.temps.get(a.found[0].id)?.r)} m, ${Math.round(100 * covered({ ...CRATE, w: CRATE.w, d: CRATE.d }))}% of the box inside`);
    assert.ok(covered(CRATE) >= 0.95, "the box inside what the change blocks");
    const b = await watch({ live: boxed, expected: plain, poses: turn, vox });
    console.log(`      turning on the spot: ${b.found.length} change(s), ${b.det.stats.blobs} blobs (no viewpoint ${CHANGES.baseline} m from another)`);
    assert.equal(b.found.length, 0, "no baseline, no change");
    const c = await watch({ live: boxed, expected: plain, poses: short, vox });
    console.log(`      a 0.2 m sidestep 0.5 m from it: ${c.found.length} change(s), ${c.det.candidates.map((q) => `σ ${fmt(q.sigma)}, ${fmt(q.base)} m apart`).join("; ")}`);
    assert.equal(c.found.length, 0, "viewpoints 0.2 m apart place nothing");
  },

  async "the localized height 0.2 m off by a sideboard's top: no change either way; a crate and a cabinet seen with the same error are still reported, well-evidenced; 0.4 m off by a table, what is left is weak at most; frames whose height is unsure are skipped"() {
    // a sideboard (1 m tall) in the scan and the house alike, passed 1 m in front at 1.2 m with the estimate 0.2 m too high
    // (its top's far edge stood where the render saw past it: a phantom obstacle) or too low (the render saw the top where
    // the camera sees past it: a phantom gone)
    const BOARD = { id: "sideboard", x: 1.2, y: 2.25, z: 0, w: 0.8, d: 0.5, h: 1.0 }, at = (x, y) => Math.atan2(BOARD.y - y, BOARD.x - x);
    const plain = makeHouseWorld(house, map, { seed: 40, cast: [] });
    plain.addObstacle(BOARD);
    const vox = scan(plain), by = Array.from({ length: 12 }, (_, i) => ({ x: 0.75 + 0.08 * i, y: 1.2, z: 1.2, yaw: at(0.75 + 0.08 * i, 1.2), pitch: 0, roll: 0, sigma: 0.05, status: "ok" }));
    const runs = {};
    for (const dz of [0.2, -0.2]) runs[dz] = await watch({ live: plain, expected: plain, poses: by, vox, frames: 36, dz });
    console.log(`      by the sideboard, the height 0.2 m too high: ${runs[0.2].found.map((c) => c.kind).join(", ") || "nothing"}, too low: ${runs[-0.2].found.map((c) => c.kind).join(", ") || "nothing"} (cells the pose's error explains: ${runs[0.2].det.stats.poseErr}, ${runs[-0.2].det.stats.poseErr})`);
    assert.deepEqual([runs[0.2].found.length, runs[-0.2].found.length], [0, 0], "no change from the height error");
    assert.ok(runs[0.2].det.stats.poseErr > 0 && runs[-0.2].det.stats.poseErr > 0, "there were differences: the pose's error explained them");
    // the crate (its top 0.3 m under the camera, at the picture's bottom from 1.3 m) and a cabinet passed 1.3 m off
    const { plain: house0, boxed } = worlds(), scan0 = scan(house0), box = {}, cab = {}, CAB = { id: "cabinet", x: 1.2, y: 2.25, z: 0, w: 0.6, d: 0.5, h: 1.4 };
    const cabbed = makeHouseWorld(house, map, { seed: 40, cast: [] }), past = by.map((p) => ({ ...p, y: 1.0, z: 1.0, yaw: Math.atan2(CAB.y - 1.0, CAB.x - p.x) }));
    cabbed.addObstacle(CAB);
    for (const dz of [0.2, -0.2]) (box[dz] = await watch({ live: boxed, expected: house0, poses: strafe, vox: scan0, dz })), (cab[dz] = await watch({ live: cabbed, expected: house0, poses: past, vox: scan0, dz }));
    const said = (r, at) => r.found.map((c) => `${c.kind} ${fmt(Math.hypot(c.x - at.x, c.y - at.y))} m off${c.weak ? " (weak)" : ""}`).join(", ") || "nothing";
    console.log(`      with the same error, the crate: ${[0.2, -0.2].map((dz) => said(box[dz], CRATE)).join("; ")}; the cabinet: ${[0.2, -0.2].map((dz) => said(cab[dz], CAB)).join("; ")}`);
    for (const dz of [0.2, -0.2]) {
      assert.ok(box[dz].found.length === 1 && box[dz].found[0].kind === "obstacle" && near(box[dz].found[0]) < 0.3 && !box[dz].found[0].weak, `the crate (height ${dz} m off), well-evidenced`);
      const [c] = cab[dz].found;
      assert.ok(cab[dz].found.length === 1 && c.kind === "obstacle" && Math.hypot(c.x - CAB.x, c.y - CAB.y) < 0.4 && !c.weak, `the cabinet (height ${dz} m off), well-evidenced`);
    }
    // beyond the sweep (0.4 m: 4 σz): the table top under the render's view in the live one; whatever is left is weak
    const TABLE = { ...BOARD, id: "table", h: 0.75 }, tabled = makeHouseWorld(house, map, { seed: 40, cast: [] }), atT = (x, y) => Math.atan2(TABLE.y - y, TABLE.x - x);
    tabled.addObstacle(TABLE);
    const far = await watch({ live: tabled, expected: tabled, poses: by.map((p) => ({ ...p, z: 1.0, yaw: atT(p.x, p.y) })), vox: scan(tabled), frames: 36, dz: 0.4 });
    console.log(`      by a table, the height 0.4 m too high: ${far.found.map((c) => `${c.kind}${c.weak ? " (weak)" : ""}`).join(", ") || "nothing"}`);
    assert.ok(far.found.every((c) => c.weak), "never a well-evidenced change from the height's error");
    const unsure = await watch({ live: boxed, expected: house0, poses: strafe, vox: scan0, zSigma: 0.2 });
    const trusted = await watch({ live: boxed, expected: house0, poses: strafe, vox: scan0, trust: () => ({ ok: true, why: "", zSigma: 0.2 }) });
    console.log(`      σz 0.2 m (the frame's, or the trust input's): ${unsure.found.length} and ${trusted.found.length} change(s), ${unsure.det.stats.skipped.height} and ${trusted.det.stats.skipped.height} frames skipped`);
    assert.deepEqual([unsure.found.length, unsure.det.stats.skipped.height, trusted.found.length, trusted.det.stats.skipped.height], [0, 24, 0, 24]);
  },

  async "nothing where the scan never saw (unknown space), nothing gone where the twin renders a smear or a faint layer; a real gone thing is"() {
    const { plain, boxed } = worlds();
    const unknown = await watch({ live: boxed, expected: plain, poses: strafe, vox: scan(plain, { unknown: (x, y, z) => inCrate(x, y, z, 0.3) }) });
    console.log(`      a box in space the scan never saw: ${unknown.found.length} change(s), ${unknown.det.stats.scan} blob(s) the scan ruled out`);
    assert.equal(unknown.found.length, 0);
    assert.ok(unknown.det.stats.scan > 0);
    // The twin shows a cabinet the house doesn't have: a scan smear (the voxels there are free: the splat's floaters made
    // no surface), a faint layer (glass, an aquarium: occupied but FAINT) or a real thing since taken away (occupied).
    // (A cabinet passed 1.3 m off, its face in view: the crate is only its top at the picture's bottom from there, all of
    // it within what the height's error moves, fix 3.)
    const CAB = { id: "cabinet", x: 1.2, y: 2.25, z: 0, w: 0.6, d: 0.5, h: 1.4 }, cab = makeHouseWorld(house, map, { seed: 40, cast: [] });
    cab.addObstacle(CAB);
    const inCab = (x, y, z) => Math.abs(x - CAB.x) < CAB.w / 2 + 0.08 && Math.abs(y - CAB.y) < CAB.d / 2 + 0.08 && z < CAB.h + 0.08;
    const past = Array.from({ length: 12 }, (_, i) => ({ x: 0.75 + 0.08 * i, y: 1.0, z: 1.0, yaw: Math.atan2(CAB.y - 1.0, CAB.x - 0.75 - 0.08 * i), pitch: 0, roll: 0, sigma: 0.05, status: "ok" }));
    const runs = {};
    for (const [name, vox] of Object.entries({ smear: scan(plain), glass: scan(cab, { faint: inCab }), removed: scan(cab) }))
      runs[name] = await watch({ live: plain, expected: cab, poses: past, vox, frames: 30 });
    const off = (c) => Math.hypot(c.x - CAB.x, c.y - CAB.y);
    console.log(`      the twin's cabinet not there now: smear ${runs.smear.found.length}, glass ${runs.glass.found.length}, really removed ${runs.removed.found.map((c) => `${c.kind} ${fmt(off(c))} m off`).join(", ") || "0"}`);
    assert.equal(runs.smear.found.length, 0, "a smear is not a gone thing");
    assert.equal(runs.glass.found.length, 0, "a faint layer is not a gone thing");
    assert.ok(runs.removed.found.some((c) => c.kind === "gone" && off(c) < 0.5), "a surface the scan had, now gone");
  },

  async "below the trust input nothing is ingested; changeTrust: the simulator always, the real drone only calibrated, past the pad check, with a fresh fix"() {
    const { plain, boxed } = worlds(), vox = scan(plain);
    const no = await watch({ live: boxed, expected: plain, poses: strafe, vox, trust: () => ({ ok: false, why: "the pad check hasn't passed yet" }) });
    assert.deepEqual([no.found.length, no.det.stats.skipped.trust, no.det.untrusted], [0, 24, "the pad check hasn't passed yet"]);
    const settings = (mode) => ({ get: (k) => (k === "mode" ? mode : undefined) }), pose = { x: 1, y: 2, sigma: 0.05, status: "ok" };
    const loc = (age) => ({ fixQuality: () => ({ age }), pose: () => pose }), splat = (t) => ({ enabled: true, trust: () => t });
    const why = (o) => changeTrust(o)({ pose }).why;
    assert.equal(why({ settings: settings("sim") }), "");
    assert.match(why({ settings: settings("real"), splat: splat("verified"), localizer: loc(100), calib: null }), /calibrated/);
    assert.match(why({ settings: settings("real"), splat: splat("advisory"), localizer: loc(100), calib: { verified: {} } }), /pad check/);
    assert.match(why({ settings: settings("real"), splat: splat("verified"), localizer: loc(5000), calib: () => ({ verified: {} }) }), /fresh/);
    assert.equal(why({ settings: settings("real"), splat: splat("verified"), localizer: loc(200), calib: () => ({ verified: {} }) }), "");
    assert.match(changeTrust({ settings: settings("real"), splat: splat("verified"), localizer: loc(200), calib: { verified: {} } })({ pose: { ...pose, sigma: 0.3 } }).why, /sure/);
  },

  async "reports of one box (0.3 and 0.37 m off) are one record blocking the whole box; two boxes 1.1 m apart stay two, each blocked; one the camera keeps not seeing stops blocking, off view it keeps blocking"() {
    let t = 1.76e12;
    const memory = await HouseMemory.open(`${house.id}:merge`, { indexedDB: null, now: () => t }), h = structuredClone(house);
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
    memory.setHouse({ house: h, map });
    memory.startFlight({ kind: "patrol" });
    // σ as the detector says it (at least CHANGES.placeErr): the browser runs placed the 0.5 m box 0.19-0.37 m off
    const a = memory.addChange({ kind: "obstacle", x: CRATE.x + 0.3, y: CRATE.y, z: 0.4, size: 0.45, sigma: 0.25 }, t);
    const b = memory.addChange({ kind: "obstacle", x: CRATE.x - 0.26, y: CRATE.y + 0.26, z: 0.4, size: 0.5, sigma: 0.25 }, (t += 3000));
    const other = memory.addChange({ kind: "obstacle", x: CRATE.x - 2.0, y: CRATE.y, z: 0.4, size: 0.4 }, (t += 1000));
    const tp = map.temps.get(a.id);
    console.log(`      two reports 0.62 m apart: one record (n ${a.n}) at (${fmt(a.x)}, ${fmt(a.y)}) blocking r ${fmt(tp.r)} m: ${Math.round(100 * covered(CRATE, [tp]))}% of the box, both reports inside`);
    assert.ok(b === a && a.n === 2 && other !== a && memory.changes.length === 2);
    assert.ok(memory.reach(a) >= 0.55 && tp.r >= memory.reach(a) + 0.1 + 2 * MEMORY.placeErr - 0.01, "it reaches every report, and its place's error twice over beyond");
    assert.ok(covered(CRATE, [tp]) >= 0.99 && [[CRATE.x + 0.3, CRATE.y, 0.225], [CRATE.x - 0.26, CRATE.y + 0.26, 0.25]].every(([x, y, s]) => Math.hypot(x - tp.x, y - tp.y) + s <= tp.r), "every report and the whole box inside");
    // two different boxes with a 0.6 m gap between them (1.1 m apart), and a chair 0.95 m from a box: each blocked where it is
    const P = { x: 1.6, y: 2.0, w: 0.5, d: 0.5 }, Q = { x: 2.7, y: 2.0, w: 0.5, d: 0.5 };
    const m2 = await HouseMemory.open(`${house.id}:two`, { indexedDB: null, now: () => t });
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
    m2.setHouse({ house: h, map });
    const p = m2.addChange({ kind: "obstacle", x: P.x, y: P.y, z: 0.4, size: 0.5, sigma: 0.25 }, t), q = m2.addChange({ kind: "obstacle", x: Q.x, y: Q.y, z: 0.4, size: 0.5, sigma: 0.25 }, (t += 2000));
    console.log(`      two boxes 1.1 m apart: ${m2.changes.length} records; ${Math.round(100 * covered(P))}% and ${Math.round(100 * covered(Q))}% of them blocked; each centre ${fmt(Math.hypot(P.x - map.temps.get(p.id).x, 0))} m and ${fmt(Math.hypot(Q.x - map.temps.get(q.id).x, 0))} m from its obstacle's`);
    assert.ok(p !== q && covered(P) === 1 && covered(Q) === 1 && map.temps.get(p.id).x === P.x && map.temps.get(q.id).x === Q.x);
    const m3 = await HouseMemory.open(`${house.id}:chair`, { indexedDB: null, now: () => t });
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
    m3.setHouse({ house: h, map });
    m3.addChange({ kind: "obstacle", x: 1.6, y: 2.0, z: 0.3, size: 0.4, sigma: 0.25 }, t);
    m3.addChange({ kind: "obstacle", x: 2.55, y: 2.0, z: 0.4, size: 0.5, sigma: 0.25 }, (t += 2000));
    console.log(`      a box and a chair 0.95 m apart: ${m3.changes.length} record(s); ${Math.round(100 * covered({ x: 1.6, y: 2.0, w: 0.4, d: 0.4 }))}% and ${Math.round(100 * covered({ x: 2.55, y: 2.0, w: 0.5, d: 0.5 }))}% blocked`);
    assert.ok(covered({ x: 1.6, y: 2.0, w: 0.4, d: 0.4 }) === 1 && covered({ x: 2.55, y: 2.0, w: 0.5, d: 0.5 }) === 1, "neither left uncovered by a merge");
    const m4 = await HouseMemory.open(`${house.id}:small`, { indexedDB: null, now: () => t });
    m4.addChange({ kind: "obstacle", x: 1.6, y: 2.0, z: 0.3, size: 0.3, sigma: 0.25 }, t);
    m4.addChange({ kind: "obstacle", x: 2.55, y: 2.0, z: 0.3, size: 0.3, sigma: 0.25 }, (t += 2000));
    assert.equal(m4.changes.length, 2, "two small things 0.95 m apart (0.65 m between them): two records");
    for (const m of [m2, m3, memory]) m.setHouse({});
    memory.setHouse({ house: h, map });
    // a minute and a bit of flying without looking at it: still blocking; in plain view on trusted frames without seeing
    // it for 10 s: off the map (still waiting for an answer); seen again: back
    for (let s = 0; s < 62; s++) (t += 1000), memory.samplePose({ x: 0.5, y: 1.0, z: 1, yaw: Math.PI, sigma: 0.05, status: "ok" }, t);
    assert.ok(map.temps.has(a.id) && !a.stale, "off view: blocking");
    for (let s = 0; s < 21; s++) memory.unseen(a.id, 500, (t += 500));
    assert.ok(!map.temps.has(a.id) && a.status === "suspected" && a.stale, "looked at and not seen: no longer blocking");
    assert.deepEqual(memory.openChanges(), { open: 2, blocking: 1, stale: 1, people: 0 });
    memory.seen(a.id, t);
    assert.ok(map.temps.has(a.id) && !a.stale, "seen again: blocking again");
    for (let s = 0; s < 21; s++) memory.unseen(a.id, 500, (t += 500));
    memory.endFlight("Landed.", (t += 1000));
    assert.ok(map.temps.has(a.id) && map.temps.has(other.id), "on the ground every open change blocks");
    memory.startFlight({ kind: "patrol" }, (t += 1000));
    assert.ok(map.temps.has(a.id) && !a.unseen, "and again at the next take-off");
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
  },

  async "the detector: the same thing again grows what it blocks (one change); two things 0.95 m apart are two changes, both on the map; a report where one already is only renews it; none where the camera itself was"() {
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
    const det = new ChangeDetector({ map, house }), found = [];
    det.on("change", (c) => found.push(c));
    const cand = (x, y, size = 0.5) => ({ near: true, x, y, z: 0.4, size, sigma: 0.12, zMin: 0, zMax: 0.8, room: "r1", n: 6, cuts: 0 });
    await det.report(cand(CRATE.x + 0.2, CRATE.y), null, 0);
    await det.report(cand(CRATE.x - 0.2, CRATE.y + 0.15), null, 1000);
    const tp = map.temps.get(found[0].id);
    console.log(`      one box twice: ${found.length} change, its obstacle r ${fmt(tp.r)} m around both reports, ${Math.round(100 * covered(CRATE))}% of the box`);
    assert.ok(found.length === 1 && det.reported[0].spots?.length === 1 && covered(CRATE) === 1);
    assert.ok([[CRATE.x + 0.2, CRATE.y], [CRATE.x - 0.2, CRATE.y + 0.15]].every(([x, y]) => Math.hypot(x - tp.x, y - tp.y) + 0.25 <= tp.r));
    await det.report(cand(CRATE.x + 0.25, CRATE.y + 0.05), null, 2000);
    assert.ok(found.length === 1 && det.reported[0].spots.length === 1, "where it already is: seen again, nothing added");
    await det.report({ ...cand(0.9, 1.0), views: [{ c: [0.95, 1.05, 0.4] }, { c: [1.3, 1.6, 1.0] }] }, null, 2500);
    assert.ok(found.length === 1 && det.stats.flown === 1, "a thing the camera was inside of while seeing it: not one");
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
    const d2 = new ChangeDetector({ map, house }), two = [];
    d2.on("change", (c) => two.push(c));
    await d2.report(cand(1.6, 2.0, 0.4), null, 0);
    await d2.report(cand(2.55, 2.0, 0.5), null, 1000);
    await d2.report(cand(3.4, 2.0, 0.5), null, 2000);
    console.log(`      a box, a chair 0.95 m from it and a third thing: ${two.length} changes; ${Math.round(100 * covered({ x: 2.55, y: 2.0, w: 0.5, d: 0.5 }))}% of the chair blocked`);
    assert.equal(two.length, 3, "the chair is a change of its own");
    assert.ok(two.every((c) => map.temps.get(c.id).r >= c.size / 2 + 0.1 + 2 * CHANGES.placeErr - 0.01), "each blocks its place's error twice over");
    assert.ok(covered({ x: 1.6, y: 2.0, w: 0.4, d: 0.4 }) === 1 && covered({ x: 2.55, y: 2.0, w: 0.5, d: 0.5 }) === 1);
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
  },

  async "an open change from an earlier flight: looked at on trusted frames and as the scan expects down to its foot, it is dismissed; below the trust input nothing happens to it; the simulator rehearsing on vision has the real trust rules but the calibration"() {
    const { plain } = worlds(), vox = scan(plain);
    let t = 1.76e12;
    const memory = await HouseMemory.open(`${house.id}:old`, { indexedDB: null, now: () => t });
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
    memory.setHouse({ house: structuredClone(house), map });
    const old = memory.addChange({ kind: "obstacle", x: CRATE.x, y: CRATE.y, z: 0.6, size: 0.5, zMin: 0, zMax: 0.7, sigma: 0.25 }, t);
    memory.startFlight({ kind: "patrol" }, (t += 1000));
    const top = await watch({ live: plain, expected: plain, poses: strafe, vox, memory }); // from 1 m up: its top only
    assert.ok(old.status === "suspected" && !old.unseen, `its top alone in view says nothing about its foot (${old.status})`);
    void top;
    const no = await watch({ live: plain, expected: plain, poses: lowStrafe, vox, memory, trust: () => ({ ok: false, why: "no fresh camera position fix" }) });
    assert.ok(old.status === "suspected" && !old.unseen && memory.blocking(old), "untrusted frames: nothing happens to it");
    const yes = await watch({ live: plain, expected: plain, poses: lowStrafe, vox, memory });
    console.log(`      the box gone, trusted frames: ${old.status} (${old.note}), after ${fmt(old.unseen / 1000)} s in plain view; untrusted: ${no.det.stats.skipped.trust} frames skipped`);
    assert.deepEqual([old.status, old.by], ["dismissed", "detector"]);
    assert.ok(old.unseen > 0, "the memory heard how long it was in plain view, not there");
    const settings = (o) => ({ get: (k) => ({ mode: "sim", ...o })[k] }), pose = { x: 1, y: 2, sigma: 0.05, status: "ok" };
    const loc = (age) => ({ fixQuality: () => ({ age }), pose: () => pose }), splat = (t, enabled = true) => ({ enabled, trust: () => t });
    const why = (o) => changeTrust(o)({ pose }).why;
    assert.equal(why({ settings: settings({ simLoc: "truth" }) }), "", "on its true pose");
    assert.match(why({ settings: settings({ simLoc: "vision" }), splat: splat("trusted"), localizer: loc(5000) }), /fresh/, "rehearsing: a stale fix pauses it");
    assert.match(why({ settings: settings({ simLoc: "vision" }), splat: splat("trusted", false), localizer: loc(100) }), /off/);
    assert.equal(why({ settings: settings({ simLoc: "vision" }), splat: splat("trusted"), localizer: loc(100) }), "", "no calib.json or pad check in the simulator");
    assert.match(changeTrust({ settings: settings({ simLoc: "vision" }), splat: splat("trusted"), localizer: loc(100) })({ pose: { ...pose, sigma: 0.3 } }).why, /sure/);
    void yes;
    memory.endFlight("Landed.", (t += 1000));
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
  },

  async "a crate still there whose report was fitted over its top (1.3 m, its depth read far) isn't 'no longer there' from views over the crate, nor does it stop blocking; gone, it is dismissed"() {
    const { plain, boxed } = worlds(), vox = scan(plain);
    let t = 1.76e12;
    const memory = await HouseMemory.open(`${house.id}:high`, { indexedDB: null, now: () => t });
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
    memory.setHouse({ house: structuredClone(house), map });
    const c = memory.addChange({ kind: "obstacle", x: CRATE.x, y: CRATE.y, z: 1.3, size: 0.5, zMin: 0, zMax: 1.6, sigma: 0.25 }, t);
    memory.startFlight({ kind: "patrol" }, (t += 1000));
    for (const poses of [strafe, lowStrafe]) await watch({ live: boxed, expected: plain, poses, vox, memory });
    console.log(`      the crate there, report fitted at 1.3 m: ${c.status}, unseen ${fmt((c.unseen ?? 0) / 1000)} s, blocking ${memory.blocking(c)}`);
    assert.ok(c.status === "suspected" && memory.blocking(c), `still there: ${c.status} (${c.note ?? ""})`);
    await watch({ live: plain, expected: plain, poses: lowStrafe, vox, memory });
    assert.deepEqual([c.status, c.by], ["dismissed", "detector"], "gone, seen down to its foot");
    memory.endFlight("Landed.", (t += 1000));
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
  },
};

await runTests(tests);
