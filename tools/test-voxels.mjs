// The 3D map (app/js/house/voxels.js, coverage.js) on the house fixture (fixtures/house): a deterministic build from the
// splat centres, the plan and the capture cameras' rays; walls, floors and ceilings occupied, rooms free at flying height,
// space no ray reached unknown; clearance3 against brute force, raycasts against the plan's walls, the planner and the map's
// overlays (voxels, temporary obstacles), the bytes in OPFS, live depth (gated, one update per voxel per frame, flights with
// pose error, a person) and what the drone saw, what the splat misses (RoomPlan tops, glass, glossy tops), maps that can
// tell nothing free, doors, the coverage report, and a stand-in twin (depth by raycasting surfaces, alpha-blended through a
// sheer layer) for the browser's depth carving and frontier sweep.
// Usage: cd tools && node test-voxels.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { createHash } from "node:crypto";

const I = await import("../app/js/house/import.js");
const V = await import("../app/js/house/voxels.js");
const C = await import("../app/js/house/coverage.js");
const P = await import("../app/js/house/planner.js");
const S = await import("../app/js/house/store.js");
const { HomeMap } = await import("../app/js/house/homemap.js");
const { intrinsics, unproject, pinholeLens, DRONE_LENS } = await import("../app/js/twin/lens.js");
const { droneCamera } = await import("../app/js/twin/pose.js");
const { UNKNOWN, FREE, OCCUPIED, FLAG } = V;

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const near = (a, b, tol, what) => assert.ok(Math.abs(a - b) <= tol, `${what}: ${a} vs ${b} (±${tol})`);
const SIGMA = 0.25, SAFE = 0.05 + SIGMA;
const ALL = { budget: Infinity }; // integrate() without its time budget: what it does, not how fast this machine is now
let seed = 11;
const rand = () => ((seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296);
const hash = (...arrays) => {
  const h = createHash("sha256");
  for (const a of arrays) h.update(new Uint8Array(a.buffer, a.byteOffset, a.byteLength));
  return h.digest("hex").slice(0, 16);
};

const fixture = await I.importCapture(fsSource(FIXTURE), { keepPoints: true });
const { house, map, points } = fixture;
const room = (id) => map.rooms.find((r) => r.id === id);

// A sealed 0.5 m box of dense opaque splats on the open floor of the living room, 0.1 to 0.6 m up: its inside is out of
// every ray's reach.
const BOX = { x: 3.0, y: 1.9, z0: 0.1, s: 0.5 };
function withBox(pts) {
  const add = [];
  for (let a = 0; a <= BOX.s + 1e-9; a += 0.02)
    for (let b = 0; b <= BOX.s + 1e-9; b += 0.02)
      for (const [x, y, z] of [[0, a, b], [BOX.s, a, b], [a, 0, b], [a, BOX.s, b], [a, b, 0], [a, b, BOX.s]])
        add.push(BOX.x - BOX.s / 2 + x, BOX.y - BOX.s / 2 + y, BOX.z0 + z);
  const n = add.length / 3, m = pts.opacity.length;
  const xyz = new Float32Array(3 * (m + n)), opacity = new Uint8Array(m + n).fill(255), scale = new Float32Array(m + n).fill(0.01);
  xyz.set(pts.xyz);
  xyz.set(add, 3 * m);
  opacity.set(pts.opacity);
  scale.set(pts.scale);
  return { xyz, opacity, scale };
}
const centres = withBox(points);

let t = performance.now();
const vox = await V.buildVoxels({ house, map, centres });
const buildMs = performance.now() - t;
const flat = new HomeMap(house).addSplats(points.xyz, points.opacity, points.scale).finalize(); // the 2.5D map alone, for comparison

// Exact depth (metres along each pixel's ray, NaN where nothing occupied within 6 m) through a map whose st is the truth.
function depthOf(truth, pose, K, W, H) {
  const R = droneCamera(pose, 0).R, d = new Float32Array(W * H).fill(NaN);
  for (let y = 0; y < H; y++)
    for (let x = 0; x < W; x++) {
      const c = unproject(K, x + 0.5, y + 0.5), hit = truth.raycast([pose.x, pose.y, pose.z], [0, 1, 2].map((k) => R[k][0] * c[0] + R[k][1] * c[1] + R[k][2] * c[2]), 6);
      if (hit.state === OCCUPIED) d[y * W + x] = hit.d;
    }
  return d;
}
const diff = (a, b) => {
  let n = 0;
  for (let i = 0; i < a.n; i++) n += a.lo[i] !== b.lo[i] || a.st[i] !== b.st[i] || a.flags[i] !== b.flags[i] ? 1 : 0;
  return n;
};
// A doorway's footprint as door() fills it: the line a-b grown s either side.
function doorwayOf(d, s) {
  const L = Math.hypot(d.b[0] - d.a[0], d.b[1] - d.a[1]), n = [-(d.b[1] - d.a[1]) / L, (d.b[0] - d.a[0]) / L];
  return [[d.a, -s], [d.b, -s], [d.b, s], [d.a, s]].map(([p, t]) => [p[0] + n[0] * t, p[1] + n[1] * t]);
}
const snap = (v) => ({ n: v.n, lo: v.lo.slice(), st: v.st.slice(), flags: v.flags.slice() });

// Brute-force clearance: the nearest blocked voxel (as clearance3 counts them) by scanning a cube around the point.
function brute(v, i, mask) {
  const { nx, ny, nz, res } = v, layer = nx * ny, c = i % nx, r = ((i / nx) | 0) % ny, l = (i / layer) | 0, R = Math.ceil(V.VOX_DEFAULTS.clearCap / res) + 1;
  let best = Infinity;
  for (let dl = -R; dl <= R; dl++)
    for (let dr = -R; dr <= R; dr++)
      for (let dc = -R; dc <= R; dc++) {
        const C2 = c + dc, R2 = r + dr, L2 = l + dl;
        if (C2 < 0 || R2 < 0 || L2 < 0 || C2 >= nx || R2 >= ny || L2 >= nz || !mask[(L2 * ny + R2) * nx + C2]) continue;
        best = Math.min(best, Math.hypot(dc, dr, dl));
      }
  return Math.min(V.VOX_DEFAULTS.clearCap, best === Infinity ? Infinity : Math.max(0, best * res - res / 2));
}

// A stand-in for the twin: depth along each pixel's ray to the first voxel of `truth` that is not free; through a voxel
// of `thin` (a 30%-opaque layer) the alpha-weighted mean with what is behind it, as Spark's expected depth is.
function fakeTwin(truth, thin = null) {
  const calls = { batches: 0, poses: 0 };
  return {
    calls,
    async depthBatch(poses, view) {
      calls.batches++;
      calls.poses += poses.length;
      const w = view.width, h = view.height, K = intrinsics(view.lens, w, h);
      return poses.map((pose) => {
        const R = droneCamera(pose, 0).R, d = new Float32Array(w * h);
        for (let v = 0; v < h; v++)
          for (let u = 0; u < w; u++) {
            const c = unproject(K, u + 0.5, v + 0.5), dir = [0, 1, 2].map((k) => R[k][0] * c[0] + R[k][1] * c[1] + R[k][2] * c[2]);
            const o = [pose.x, pose.y, pose.z], hit = truth.raycast(o, dir, 8);
            let t = hit.d;
            if (thin && hit.state === OCCUPIED && thin[truth.idx(...o.map((q, k) => q + dir[k] * (t + 1e-3)))]) {
              while (t < 8 && thin[truth.idx(...o.map((q, k) => q + dir[k] * (t + 1e-3)))]) t += truth.res / 4;
              const behind = truth.raycast(o.map((q, k) => q + dir[k] * t), dir, 8 - t);
              d[v * w + u] = behind.state === OCCUPIED ? 0.3 * hit.d + 0.7 * (t + behind.d) : NaN;
              continue;
            }
            d[v * w + u] = hit.state === OCCUPIED ? t + truth.res / 4 : NaN;
          }
        return d;
      });
    },
  };
}

const tests = {
  "build: deterministic, timed, every state present"() {
    console.log(`      ${vox.stats().grid}: ${buildMs.toFixed(0)} ms (${JSON.stringify(vox.built.phases)}), ${JSON.stringify(vox.counts())}`);
    assert.ok(buildMs < 5000, `build took ${buildMs.toFixed(0)} ms`);
    return V.buildVoxels({ house, map, centres }).then((again) => {
      assert.equal(hash(again.lo, again.flags), hash(vox.lo, vox.flags), "two builds, the same voxels");
      const c = vox.counts();
      assert.ok(c.unknown > 0 && c.free > 0 && c.occupied > 0);
      assert.equal(vox.nx, map.W);
      assert.equal(vox.x0, map.x0);
      near(vox.z0, -0.1, 1e-9, "z0: the lowest floor - 0.1");
    });
  },

  "walls, floors and ceilings are occupied; the splat alone marks most wall faces"() {
    let walls = 0, wallOcc = 0, faces = 0, faceSplat = 0;
    for (let k = 0; k < map.N; k++) {
      const [x, y] = map.center(k % map.W, (k / map.W) | 0);
      if (map.wall[k]) {
        for (const z of [0.5, 1.2, 2.0]) (walls++, (wallOcc += vox.state(x, y, z) === OCCUPIED ? 1 : 0));
        continue;
      }
      if (map.room[k] < 0) continue;
      const fl = map.floorZ[k], ce = map.ceilZ[k];
      assert.equal(vox.state(x, y, fl - 0.03), OCCUPIED, `floor under (${x.toFixed(2)}, ${y.toFixed(2)})`);
      if (ce < vox.z0 + vox.nz * vox.res - 0.05) assert.equal(vox.state(x, y, ce + 0.03), OCCUPIED, `ceiling over (${x.toFixed(2)}, ${y.toFixed(2)})`);
      // Room cells beside a wall: within 10 cm of its face, at 0.5-2 m, does the splat put a surface there?
      const by = [1, -1, map.W, -map.W].some((d) => map.wall[k + d]);
      if (!by) continue;
      for (let z = fl + 0.5; z < Math.min(ce, fl + 2.0); z += 0.25) {
        faces++;
        let hit = false;
        for (let dz = -0.05; dz <= 0.05 && !hit; dz += 0.05)
          for (const d of [0, 1, -1, map.W, -map.W]) {
            const [qx, qy] = map.center((k + d) % map.W, ((k + d) / map.W) | 0), i = vox.idx(qx, qy, z + dz);
            if (i >= 0 && vox.flags[i] & FLAG.CAPTURE) hit = true;
          }
        faceSplat += hit ? 1 : 0;
      }
    }
    console.log(`      plan walls occupied ${wallOcc}/${walls}; wall faces with splat surfaces within 5 cm: ${((100 * faceSplat) / faces).toFixed(0)}%`);
    assert.equal(wallOcc, walls);
    assert.ok(faceSplat / faces > 0.4, `wall faces the splat marks: ${faceSplat}/${faces}`);
  },

  "room interiors at 1 m are mostly known free; every obstacle the 2.5D map has is occupied in its band"() {
    const out = [];
    for (const r of map.rooms) {
      let n = 0, free = 0;
      for (let k = 0; k < map.N; k++) {
        if (map.room[k] !== r.index || map.wall[k] || flat.clear[flat.bandOf(1.0)][k] < 0.3) continue;
        const [x, y] = map.center(k % map.W, (k / map.W) | 0);
        n++;
        free += vox.state(x, y, map.floorZ[k] + 1.0) === FREE ? 1 : 0;
      }
      out.push(`${r.name} ${((100 * free) / n).toFixed(1)}%`);
      assert.ok(free / n > 0.97, `${r.name}: ${free}/${n} open cells free at 1 m`);
    }
    console.log(`      open floor, 1 m up, known free: ${out.join(", ")}`);
    // Each cell the 2.5D map blocks in a band (splats summed over ±bandHalf) has an occupied voxel in that height range.
    const half = flat.o.bandHalf, missed = [];
    flat.o.bands.forEach((alt, b) => {
      for (let k = 0; k < map.N; k++) {
        if (map.room[k] < 0 || map.wall[k] || !flat.occ[b][k] || map.floorZ[k] + alt + half > map.ceilZ[k]) continue;
        const [x, y] = map.center(k % map.W, (k / map.W) | 0);
        let occ = false;
        for (let z = map.floorZ[k] + alt - half + 0.025; z < map.floorZ[k] + alt + half && !occ; z += 0.05) occ = vox.state(x, y, z) === OCCUPIED;
        if (!occ) missed.push(`${alt} m at (${x.toFixed(2)}, ${y.toFixed(2)})`);
      }
    });
    assert.deepEqual(missed.slice(0, 5), [], `${missed.length} 2.5D obstacle cells with nothing occupied in their band`);
  },

  "a region no capture ray could reach stays unknown (inside a sealed box), and so does space behind furniture"() {
    let inside = 0, unknown = 0;
    for (let z = BOX.z0 + 0.1; z < BOX.z0 + BOX.s - 0.08; z += 0.05)
      for (let y = BOX.y - BOX.s / 2 + 0.1; y < BOX.y + BOX.s / 2 - 0.08; y += 0.05)
        for (let x = BOX.x - BOX.s / 2 + 0.1; x < BOX.x + BOX.s / 2 - 0.08; x += 0.05) (inside++, (unknown += vox.state(x, y, z) === UNKNOWN ? 1 : 0));
    assert.ok(inside > 20 && unknown === inside, `box inside: ${unknown}/${inside} unknown`);
    near(vox.raycast([BOX.x - 1, BOX.y, 0.35], [1, 0, 0], 3).d, 1 - BOX.s / 2, 0.08, "a ray meets the box's face");
    const band = V.bandMask(vox, map), c = [0, 0, 0];
    for (let i = 0; i < vox.n; i++) if (band[i]) c[vox.st[i]]++;
    console.log(`      at drone heights (0.3-1.6 m): ${(c[1] * 0.000125).toFixed(1)} m³ free, ${(c[0] * 0.000125).toFixed(1)} m³ unknown, ${(c[2] * 0.000125).toFixed(1)} m³ occupied`);
    assert.ok(c[0] > 0.02 * (c[0] + c[1]), "behind and under furniture stays unknown");
  },

  "clearance3 matches brute force (occupied and unknown count, floor and ceiling zones do not)"() {
    const mask = vox.blockedMask();
    let worst = 0, checked = 0, low = 0;
    for (let s = 0; s < 400; s++) {
      const i = Math.floor(rand() * vox.n), want = brute(vox, i, mask), got = vox.dist[i];
      worst = Math.max(worst, Math.abs(want - got));
      checked++;
      low += got < 0.5 ? 1 : 0;
    }
    // and points near surfaces, where it matters
    for (let s = 0, k = 0; s < 4000 && k < 300; s++) {
      const i = Math.floor(rand() * vox.n);
      if (vox.st[i] !== FREE || vox.dist[i] > 0.4) continue;
      worst = Math.max(worst, Math.abs(brute(vox, i, mask) - vox.dist[i]));
      k++;
      checked++;
    }
    assert.ok(worst < 1e-4, `worst difference ${worst} over ${checked} voxels (${low} near something)`);
    const [x, y] = P.roomCenter(map, "r3"), fl = map.floorAt(x, y);
    assert.ok(vox.clearance3(x, y, fl + 0.6) > 0.3, `0.6 m over the open floor of Room 3: ${vox.clearance3(x, y, fl + 0.6)} (the floor itself does not count)`);
    assert.equal(vox.clearance3(-50, 0, 1), 0, "off the grid");
  },

  "raycasts: exact against a fine march through the voxels, and they meet the plan's walls where they are"() {
    // The traversal itself: from random points in free space, everything before d is free and the voxel just past it is not.
    let bad = 0, rays = 0;
    for (let s = 0; s < 5000 && rays < 300; s++) {
      const i = Math.floor(rand() * vox.n);
      if (vox.st[i] !== FREE) continue;
      const o = vox.center(i).map((v) => v + (rand() - 0.5) * vox.res * 0.98), a = rand() * 2 * Math.PI, e = (rand() - 0.5) * Math.PI;
      const d = [Math.cos(e) * Math.cos(a), Math.cos(e) * Math.sin(a), Math.sin(e)], hit = vox.raycast(o, d, 4), at = (t) => vox.state(...o.map((v, k) => v + d[k] * t));
      let ok = hit.d === 4 ? at(3.999) === FREE : at(hit.d + 1e-5) !== FREE;
      for (let t = 0; t < hit.d - 1e-5 && ok; t += 0.001) ok = at(t) === FREE;
      bad += ok ? 0 : 1;
      rays++;
    }
    const worst = bad;
    assert.equal(worst, 0, `${bad} of ${rays} rays stopped in the wrong place`);
    // The plan's walls, from 1.2 m inside each long edge at 1.3 m up, where the 2.5D map has nothing in the way and no
    // doorway is near: the first surface is the wall (the splat's, a few cm in front of the plan's line, or the plan's).
    const errs = [];
    for (const r of map.rooms)
      r.outline.forEach((a, e) => {
        const b = r.outline[(e + 1) % r.outline.length], L = Math.hypot(b[0] - a[0], b[1] - a[1]), u = [(b[0] - a[0]) / L, (b[1] - a[1]) / L];
        if (L < 1.2) return;
        const area = r.outline.reduce((acc, p, i) => acc + p[0] * r.outline[(i + 1) % r.outline.length][1] - r.outline[(i + 1) % r.outline.length][0] * p[1], 0);
        const inward = area > 0 ? [-u[1], u[0]] : [u[1], -u[0]];
        for (const f of [0.3, 0.5, 0.7]) {
          const p = [a[0] + u[0] * L * f + inward[0] * 1.2, a[1] + u[1] * L * f + inward[1] * 1.2];
          if (map.roomAt(...p)?.id !== r.id) continue;
          let clear = true;
          for (let d = 0; d < 1.45; d += 0.05) {
            const k = map.idx(p[0] - inward[0] * d, p[1] - inward[1] * d);
            if (k < 0 || flat.occ[flat.bandOf(1.4)][k] || flat.occ[flat.bandOf(1.0)][k] || map.headZ[k] < Infinity) clear = false;
          }
          const hit = vox.raycast([...p, (map.floorAt(...p) ?? 0) + 1.3], [-inward[0], -inward[1], 0], 3);
          if (clear) errs.push(hit.state === OCCUPIED ? hit.d - 1.2 : Infinity);
        }
      });
    errs.sort((p, q) => Math.abs(p) - Math.abs(q));
    const within = errs.filter((e) => Math.abs(e) <= vox.res).length;
    console.log(`      ${errs.length} rays at 1.3 m: ${within} within a voxel of the plan's wall line, all within ${(Math.abs(errs.at(-1)) * 100).toFixed(1)} cm`);
    // (One wall of Room 2: the splat's surface stands 7 cm in front of the plan's line.)
    assert.ok(errs.length >= 6 && within / errs.length >= 0.6 && Math.abs(errs.at(-1)) <= 2 * vox.res, `${errs.map((e) => e.toFixed(3))}`);
    const r = vox.raycast([-0.6, 2.0, 1.242], [0, 0, -1], 3);
    near(r.d, 1.0, vox.res, "straight down onto the entry floor (0.242)");
    assert.equal(r.state, OCCUPIED);
    assert.deepEqual(vox.raycast([-0.6, 2.0, 1.242], [0, 0, -1], 0.5), { d: 0.5, state: FREE }, "nothing within max");
  },

  "thin things: with the faint splats (5% opacity and up) the railing's balusters, a table's base and chair legs mark at 5 cm"() {
    const meta = JSON.parse(fs.readFileSync(path.join(FIXTURE, "outputs/thin-centres.json"), "utf8"));
    const fr = { f: house.frame.f, Yf: house.frame.Yf };
    const tc = I.readSplatPoints(fs.readFileSync(path.join(FIXTURE, "outputs/thin-centres.bin")));
    return tc.then(async (t) => {
      const frame = (await import("../app/js/house/frames.js")).houseFrame(fr);
      const inBox = (x, y, z) => meta.boxes.some((b) => [x, y, z].every((v, k) => v >= b.min[k] && v < b.max[k]));
      const keep = [];
      for (let i = 0; i < points.opacity.length; i++) if (!inBox(points.xyz[3 * i], points.xyz[3 * i + 1], points.xyz[3 * i + 2])) keep.push(i);
      const m = keep.length + t.n, all = { xyz: new Float32Array(3 * m), opacity: new Uint8Array(m), scale: new Float32Array(m) };
      keep.forEach((i, j) => {
        all.xyz.set(points.xyz.subarray(3 * i, 3 * i + 3), 3 * j);
        all.opacity[j] = points.opacity[i];
        all.scale[j] = points.scale[i];
      });
      all.xyz.set(frame.pointsToH(t.xyz), 3 * keep.length);
      all.opacity.set(t.opacity, keep.length);
      all.scale.set(t.scale.map((v) => v * frame.f), keep.length);
      const faint = await V.buildVoxels({ house, map, centres: all }), opaque = vox;
      const rail = (v) => {
        const fl = map.floorAt(-0.87, 5.0), rows = [];
        for (let h = 0.15; h <= 0.86; h += 0.1) {
          const ys = [];
          for (let y = 3.7; y < 6.35; y += 0.05) {
            let occ = false;
            for (let x = -1.0; x < -0.72; x += 0.05) occ ||= v.state(x, y, fl + h) === OCCUPIED;
            if (occ) ys.push(y);
          }
          rows.push(ys.slice(1).reduce((g, y, k) => Math.max(g, y - ys[k]), ys.length ? Math.max(ys[0] - 3.85, 6.3 - ys.at(-1)) : 9));
        }
        return Math.max(...rows);
      };
      const count = (v, [x0, x1, y0, y1], hs) => {
        let n = 0;
        for (let y = y0; y < y1; y += 0.05) for (let x = x0; x < x1; x += 0.05) for (const h of hs) n += v.state(x, y, map.floorAt(x, y) + h) === OCCUPIED ? 1 : 0;
        return n;
      };
      const table = [2.63, 3.83, 2.65, 3.85], chair = [1.77, 2.77, 4.78, 5.78];
      const got = { rail: [rail(faint), rail(opaque)], table: [count(faint, table, [0.1, 0.15]), count(opaque, table, [0.1, 0.15])],
        chair: [count(faint, chair, [0.2, 0.25, 0.3]), count(opaque, chair, [0.2, 0.25, 0.3])] };
      console.log(`      railing: widest gap along it ${got.rail[0].toFixed(2)} m with faint splats, ${got.rail[1].toFixed(2)} m with opaque ones only; `
        + `table base ${got.table.join(" vs ")} voxels, chair legs ${got.chair.join(" vs ")}`);
      assert.ok(got.rail[0] <= 0.3, `railing gap ${got.rail[0]}`);
      assert.ok(got.rail[1] > got.rail[0] && got.table[0] > 2 * got.table[1] && got.chair[0] > 1.5 * got.chair[1], JSON.stringify(got));
      for (let i = 0; i < faint.n; i++) if (faint.flags[i] & FLAG.CAPTURE) assert.notEqual(faint.st[i], FREE);
    });
  },

  "the planner with voxels: home to every room, at least the drone's radius + σ from anything occupied or unknown"() {
    const m = new HomeMap(house).addSplats(points.xyz, points.opacity, points.scale).finalize().setVoxels(vox);
    const mask = vox.blockedMask();
    for (const r of house.rooms) {
      const goal = P.roomCenter(m, r.id), p = P.plan(m, [house.home.x, house.home.y], goal, { alt: 1.0, sigma: SIGMA });
      assert.ok(p.ok, `home -> ${r.name}: ${p.reason}`);
      let worst = Infinity;
      for (const q of p.path) {
        const i = vox.idx(...q);
        assert.equal(vox.st[i], FREE, `(${q.map((v) => v.toFixed(2))}) on the way to ${r.name} is known free`);
        worst = Math.min(worst, brute(vox, i, mask));
      }
      assert.ok(worst >= SAFE - vox.res * 0.87, `home -> ${r.name}: ${worst.toFixed(3)} m from occupied or unknown`);
      assert.ok(p.minClearance >= SAFE);
    }
    const tour = P.patrolRoute(m, undefined, [house.home.x, house.home.y], { sigma: SIGMA });
    assert.ok(tour.ok && !tour.skipped.length, `patrol: ${JSON.stringify(tour.skipped)}`);
    // Where the 2.5D map alone says free but the space was never seen, the map with voxels does not.
    let differ = 0;
    for (let k = 0; k < m.N; k++) {
      if (!(flat.clear[1][k] >= SAFE)) continue;
      const [x, y] = m.center(k % m.W, (k / m.W) | 0), z = m.floorZ[k] + 1.0;
      if (!m.free(x, y, z)) differ++;
      else assert.ok(vox.free3(x, y, z, SAFE), "free on the map means known free in 3D");
    }
    console.log(`      at 1 m, ${((differ * 0.0025)).toFixed(1)} m² the 2.5D map calls free are not (near unknown or 3D surfaces)`);
    assert.ok(differ > 0);
    near(m.clearance(-0.6, 2.0, 1.242), Math.min(flat.clearance(-0.6, 2.0, 1.242), vox.clearance3(-0.6, 2.0, 1.242)), 1e-6, "the smaller of the two");
  },

  "temporary obstacles block, replan around and expire; the 2.5D map without voxels honours them too"() {
    for (const m of [new HomeMap(house).addSplats(points.xyz, points.opacity, points.scale).finalize(), flat.setVoxels(vox)]) {
      let clock = 1000;
      m.clock = () => clock;
      const from = [house.home.x, house.home.y], to = P.roomCenter(m, "r1"), p0 = P.plan(m, from, to, { sigma: SIGMA });
      assert.ok(p0.ok, p0.reason);
      const mid = p0.path[Math.floor(p0.path.length * 0.6)];
      m.addTemp({ id: "person-1", kind: "person", x: mid[0], y: mid[1], r: 0.5, zMin: 0, zMax: 2.2, until: 5000 });
      assert.ok(!m.free(mid[0], mid[1], mid[2]), "inside it is not free");
      const p1 = P.plan(m, from, to, { sigma: SIGMA });
      assert.ok(p1.ok, `replanned: ${p1.reason}`);
      const gap = Math.min(...p1.path.map((q) => Math.hypot(q[0] - mid[0], q[1] - mid[1])));
      assert.ok(gap >= 0.5 + SAFE - 0.05, `the new path keeps ${gap.toFixed(2)} m from its centre`);
      assert.ok(p1.length > p0.length, "a detour");
      m.addTemp({ id: "high", kind: "change", x: mid[0], y: mid[1] + 3, r: 0.3, zMin: 2.4, zMax: 2.6 });
      assert.ok(m.free(mid[0], mid[1] + 3, m.floorAt(mid[0], mid[1] + 3) + 1.0) || !flat.free(mid[0], mid[1] + 3, 1), "above the bands: no effect");
      m.removeTemp("high");
      clock = 6000; // past until
      assert.ok(m.free(mid[0], mid[1], mid[2], SAFE), "expired");
      assert.equal(m.temps.size, 0);
      m.addTemp({ id: "x", x: mid[0], y: mid[1], r: 0.4 });
      assert.ok(!m.free(mid[0], mid[1], mid[2]));
      assert.ok(m.removeTemp("x") && m.free(mid[0], mid[1], mid[2], SAFE) && !m.removeTemp("x"));
    }
    flat.setVoxels(null);
  },

  async "serialize / load round trip, and the store keeps it per house (stale when the house changes)"() {
    const v = await V.buildVoxels({ house, map, centres });
    v.markSeen({ x: -0.6, y: 2.0, z: 1.242, yaw: 0, pitch: 0, roll: 0 }, DRONE_LENS, 1.79e12);
    const bytes = v.serialize(), back = V.VoxelMap.load(bytes);
    console.log(`      ${(bytes.byteLength / 1024).toFixed(0)} KB for ${v.n} voxels (${(v.n * 3 / 1024).toFixed(0)} KB raw)`);
    assert.ok(bytes.byteLength < 2e6);
    assert.equal(hash(back.lo, back.flags, back.st, back.floorL, back.ceilL, back.seen), hash(v.lo, v.flags, v.st, v.floorL, v.ceilL, v.seen));
    assert.deepEqual([back.x0, back.y0, back.z0, back.nx, back.ny, back.nz, back.res, back.key, back.epoch], [v.x0, v.y0, v.z0, v.nx, v.ny, v.nz, v.res, v.key, v.epoch]);
    assert.equal(back.seenAt(-0.6, 2.0, 1.242), 1.79e12);
    for (let s = 0; s < 50; s++) {
      const p = [map.x0 + rand() * map.W * 0.05, map.y0 + rand() * map.H * 0.05, rand() * 3];
      assert.equal(back.clearance3(...p), v.clearance3(...p));
    }
    assert.throws(() => V.VoxelMap.load(new Uint8Array(16)), /not a voxel map/);
    const store = await S.openStore(S.memoryDir());
    assert.equal(await store.loadVoxels(house), null, "none yet");
    await store.saveVoxels(house, v);
    const got = await store.loadVoxels(house);
    assert.ok(got && hash(got.lo) === hash(v.lo), "saved and loaded");
    const moved = { ...house, rooms: house.rooms.map((r, i) => (i ? r : { ...r, floorZ: r.floorZ + 0.1 })) };
    assert.equal(await store.loadVoxels(moved), null, "a changed house: build again");
    assert.notEqual(V.voxelKey(moved), V.voxelKey(house));
    assert.equal(V.voxelKey({ ...house, name: "renamed", landmarks: [] }), V.voxelKey(house), "names and marks don't matter");
  },

  async "live depth: one update per voxel per frame, gated on the pose and the picture; capture surfaces stay"() {
    const v = await V.buildVoxels({ house, map, centres });
    v.computeClearance();
    const lens = pinholeLens(60), w = 64, h = 48, K = intrinsics(lens, w, h), frameOf = (data, extra = {}) => ({ width: w, height: h, data, ...extra });
    const pose = { x: 2.6, y: 4.2, z: 1.2, yaw: Math.PI / 2, pitch: 0, roll: 0, sigma: 0.02, status: "ok" };
    const hit = v.raycast([pose.x, pose.y, pose.z], [0, 1, 0], 4);
    assert.equal(hit.state, OCCUPIED, "something ahead");
    const wallI = v.idx(pose.x, pose.y + hit.d + 0.01, pose.z), was = v.flags[wallI];
    // Live depth that sees 0.5 m beyond the wall, ten frames: the capture surface stays and is reported.
    const far = new Float32Array(w * h).fill(hit.d + 0.5);
    let r;
    for (let k = 0; k < 10; k++) r = v.integrate(pose, frameOf(far), lens, { t: 1000 + k, ...ALL });
    assert.equal(v.st[wallI], OCCUPIED, "still occupied");
    assert.equal(v.flags[wallI] & FLAG.CAPTURE, was & FLAG.CAPTURE);
    assert.ok(r.contradicted.length > 0, "reported for nav/changes.js");
    // Not when the twin's expected depth sees as far (its depth is alpha-blended and runs deeper than the first surface).
    const near = new Float32Array(w * h).fill(hit.d);
    assert.equal(v.integrate(pose, frameOf(far, { expected: far }), lens, ALL).contradicted.length, 0, "the twin expects it");
    assert.ok(v.integrate(pose, frameOf(far, { expected: near }), lens, ALL).contradicted.length > 0, "the twin expects the surface");
    // Something new in free space 0.8 m ahead.
    const box = new Float32Array(w * h).fill(NaN), ahead = 0.8;
    for (let y = 20; y < 28; y++) for (let x = 28; x < 36; x++) box[y * w + x] = ahead / unproject(K, x + 0.5, y + 0.5)[2];
    const spot = [pose.x, pose.y + ahead + 0.02, pose.z], before = [spot[0], spot[1] - 0.2, spot[2]], ver = v.version;
    assert.equal(v.state(...spot), FREE, "free before");
    // No frame counts at a doubtful pose, nor pixels the depth model isn't sure of, under the OSD or on a person.
    for (const bad of [{ ...pose, sigma: 0.2 }, { ...pose, status: "lost" }, { ...pose, sigma: undefined }])
      assert.equal(v.integrate(bad, frameOf(box), lens, ALL).skipped, "pose");
    v.integrate(pose, frameOf(box, { conf: new Float32Array(w * h).fill(0.2) }), lens, ALL);
    v.integrate(pose, frameOf(box, { mask: new Uint8Array(w * h).fill(1) }), lens, ALL);
    v.integrate(pose, frameOf(box, { boxes: [{ x: 0.4, y: 0.4, w: 0.2, h: 0.2, label: "person" }] }), lens, ALL);
    assert.equal(v.version, ver, "nothing changed");
    // One frame: doubtful (not free any more, and clearance3 shrinks around it at once); the second: occupied.
    v.integrate(pose, frameOf(box), lens, ALL);
    assert.equal(v.state(...spot), UNKNOWN, "after one look: doubtful");
    assert.ok(v.clearance3(...before) <= 0.2, "clearance3 updated around it without a full rebuild");
    v.integrate(pose, frameOf(box), lens, ALL);
    assert.equal(v.state(...spot), OCCUPIED, "an obstacle after two looks");
    // Out of time (budget, ms): it stops at its first look at the clock (every 32 rays), and those rays cover the whole
    // picture coarsely (coarse to fine).
    const cut = V.VoxelMap.load(v.serialize()), us = [], vs = [];
    const rc = cut.integrate(pose, frameOf(new Float32Array(w * h).fill(1.0)), lens, { budget: -1 });
    cut.eachRay(pose, lens, w, h, 4800, (x, y) => (us.push(x), vs.push(y), us.length >= 32));
    const span = (a) => Math.max(...a) - Math.min(...a);
    assert.ok(rc.cut && rc.rays === 32 && span(us) > 0.8 * w && span(vs) > 0.8 * h, `${rc.rays} rays over ${span(us)} x ${span(vs)} px`);
    // It leaves: misses wear it away (flight evidence is not capture evidence).
    const open = new Float32Array(w * h).fill(ahead + 1.0);
    for (let k = 0; k < 12; k++) v.integrate(pose, frameOf(open), lens, ALL);
    assert.equal(v.state(...spot), FREE, "free again");
    // Space never seen turns free only after three frames agree.
    const unseen = [];
    v.region({ min: spot.map((q) => q - 0.1), max: spot.map((q) => q + 0.1) }, (i) => {
      [v.lo[i], v.st[i], v.flags[i]] = [0, UNKNOWN, 0];
      unseen.push(i);
    });
    const states = [1, 2, 3].map(() => (v.integrate(pose, frameOf(open), lens, ALL), unseen.filter((i) => v.st[i] === FREE).length));
    assert.deepEqual(states, [0, 0, unseen.length], "free after the third frame");
    await v.refresh();
    // A confirmed change: an obstacle flight evidence can't clear (and that is saved), and a removal that frees a splat
    // surface (never the plan's walls).
    v.mark({ x: spot[0], y: spot[1], z: spot[2], r: 0.06 }, OCCUPIED);
    for (let k = 0; k < 12; k++) v.integrate(pose, frameOf(open), lens, ALL);
    assert.equal(v.state(...spot), OCCUPIED, "confirmed obstacle stays");
    assert.equal(V.VoxelMap.load(v.serialize()).state(...spot), OCCUPIED, "and is saved");
    const wallXYZ = v.center(wallI);
    v.mark({ x: wallXYZ[0], y: wallXYZ[1], z: wallXYZ[2], r: 0.01 }, "free");
    assert.equal(v.st[wallI], was & FLAG.PLAN ? OCCUPIED : FREE, "a confirmed removal frees a splat surface");
  },

  "live depth keeps to its time budget: rays stop in time to apply what they found, new obstacles first"() {
    const v = V.VoxelMap.load(vox.serialize()), base = V.VoxelMap.load(vox.serialize());
    v.computeClearance();
    base.computeClearance();
    const lens = pinholeLens(90), W = 160, H = 120, K = intrinsics(lens, W, H);
    const pose = { x: 2.6, y: 4.2, z: 1.2, yaw: Math.PI / 2, pitch: 0, roll: 0, sigma: 0.05, status: "ok" };
    // Something new across the whole view 0.8 m ahead: every ray ends on free space (a stamp() each), the worst case.
    const wall = new Float32Array(W * H), frame = { width: W, height: H, depth: wall };
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) wall[y * W + x] = 0.8 / unproject(K, x + 0.5, y + 0.5)[2];
    const reset = () => (v.lo.set(base.lo), v.st.set(base.st), v.flags.set(base.flags), v.dist.set(base.dist), (v.pending.n = 0));
    v.integrate(pose, { ...frame, depth: wall.map(() => NaN) }, lens); // its buffers, once per map
    const time = (o) => {
      const ms = [];
      let r, fed = 0;
      for (let k = 0; k < 7; k++) {
        reset();
        const t0 = performance.now();
        r = v.integrate(pose, frame, lens, o);
        ms.push(performance.now() - t0);
        fed += r.hits > 0 && r.free > 255 ? 1 : 0;
      }
      return { ms: ms.sort((a, b) => a - b)[3], r, fed };
    };
    const all = time(ALL), runs = [2, 4, 8].map((budget) => ({ budget, ...time({ budget }) }));
    reset();
    const none = v.integrate(pose, frame, lens, { budget: -1 });
    console.log(`      a new wall across the view: everything ${all.ms.toFixed(1)} ms (${all.r.rays} rays, ${all.r.hits} hits, ${all.r.free} misses); `
      + runs.map((q) => `budget ${q.budget}: ${q.ms.toFixed(1)} ms (${q.r.rays} rays, ${q.r.hits} hits, ${q.r.free} misses)`).join("; "));
    assert.ok(none.cut && none.rays === 32 && none.hits > 20 && none.flipped >= none.hits && none.free <= 255, `out of time at once: ${JSON.stringify({ ...none, contradicted: 0 })}`);
    for (const q of runs) assert.ok(q.ms < 1.5 * q.budget + 1, `budget ${q.budget} ms: median ${q.ms.toFixed(1)} ms`);
    assert.ok(runs.every((q) => q.fed >= 4), `frames get their hits and misses past the first clock look in: ${runs.map((q) => q.fed)} of 7`);
  },

  "the map's overlay follows what flights block, near them only, the same as redoing it all"() {
    const v = V.VoxelMap.load(vox.serialize());
    v.computeClearance();
    v.built = vox.built;
    const m = new HomeMap(house).addSplats(points.xyz, points.opacity, points.scale).finalize().setVoxels(v), full = m.overlay.bind(m);
    let fulls = 0;
    m.overlay = () => (fulls++, full());
    const lens = pinholeLens(90), W = 120, H = 90, K = intrinsics(lens, W, H);
    const pose = { x: 2.6, y: 4.2, z: 1.2, yaw: Math.PI / 2, pitch: 0, roll: 0, sigma: 0.05, status: "ok" };
    const thing = (x0, x1, y0, y1, d) => {
      const out = new Float32Array(W * H).fill(NaN);
      for (let y = y0; y < y1; y++) for (let x = x0; x < x1; x++) out[y * W + x] = d / unproject(K, x + 0.5, y + 0.5)[2];
      return { width: W, height: H, depth: out };
    };
    const check = (what) => {
      const got = m.clear.map((c) => c.slice()), doors = m.doors.map((d) => d.clearance.join());
      full();
      m.clear.forEach((c, b) => assert.ok(c.every((x, k) => x === got[b][k]), `${what}: band ${b} differs from a full overlay`));
      assert.deepEqual(m.doors.map((d) => d.clearance.join()), doors, `${what}: doorways`);
    };
    const was = m.clear, ms = [], flips = [];
    for (const [k, f] of [thing(56, 64, 40, 48, 0.9), thing(56, 64, 40, 48, 0.9), thing(24, 30, 50, 56, 0.6)].entries()) {
      const n = fulls, r = v.integrate(pose, f, lens, ALL);
      flips.push(r.flipped);
      const t0 = performance.now();
      m.sync();
      ms.push(performance.now() - t0);
      assert.equal(fulls, n, `frame ${k}: cell by cell`);
      check(`frame ${k}`);
    }
    assert.ok(flips[0] > 0 && flips[2] > 0 && m.clear !== was, "new arrays (the planner caches by them)");
    assert.ok(m.clearance(pose.x, pose.y + 0.9, pose.z) < 0.1, "the new thing is in the map");
    const before = fulls;
    v.computeClearance();
    m.sync();
    assert.equal(fulls, before + 1, "a refresh redoes it all");
    v.integrate(pose, thing(20, 100, 15, 75, 0.7), lens, ALL); // more than pending lists one by one
    m.sync();
    assert.equal(fulls, before + 2, "so does an obstacle too big to follow voxel by voxel");
    check("a big obstacle");
    console.log(`      overlay after a frame: ${ms.map((x, k) => `${x.toFixed(2)} ms (${flips[k]} voxels flipped)`).join(", ")} cell by cell`);
  },

  async "flights: pose error and noise leave no phantoms, a person isn't remembered, clearance3 stays a lower bound"() {
    const v0 = await V.buildVoxels({ house, map, centres });
    v0.computeClearance();
    const truth = V.VoxelMap.load(v0.serialize());
    for (let i = 0; i < truth.n; i++) truth.st[i] = truth.st[i] === OCCUPIED ? OCCUPIED : FREE;
    const lens = pinholeLens(100), W = 140, H = 105, K = intrinsics(lens, W, H);
    const gauss = () => Math.sqrt(-2 * Math.log(1 - rand())) * Math.cos(2 * Math.PI * rand());
    const render = (t, pose, noise = 0) => {
      const R = droneCamera(pose, 0).R, d = new Float32Array(W * H).fill(NaN);
      for (let y = 0; y < H; y++)
        for (let x = 0; x < W; x++) {
          const c = unproject(K, x + 0.5, y + 0.5), dir = [0, 1, 2].map((k) => R[k][0] * c[0] + R[k][1] * c[1] + R[k][2] * c[2]);
          const hit = t.raycast([pose.x, pose.y, pose.z], dir, 6);
          if (hit.state === OCCUPIED) d[y * W + x] = hit.d * (1 + noise * gauss());
        }
      return d;
    };
    // Home to every room at 1 m, a frame every 0.25 m, the localizer 10 cm off (σ 0.05, "ok") and 3% depth noise.
    const route = [];
    for (const r of house.rooms) {
      const p = P.plan(flat, [house.home.x, house.home.y], P.roomCenter(flat, r.id), { alt: 1.0, sigma: SIGMA });
      for (let k = 1; k < p.path.length; k++) {
        const [a, b] = [p.path[k - 1], p.path[k]], L = Math.hypot(b[0] - a[0], b[1] - a[1]), yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
        for (let s = 0; s < L; s += 0.25) route.push({ x: a[0] + ((b[0] - a[0]) * s) / L, y: a[1] + ((b[1] - a[1]) * s) / L, z: a[2], yaw, pitch: 0, roll: 0 });
      }
    }
    const v = V.VoxelMap.load(v0.serialize()), band = V.bandMask(v0, map), ms = [];
    v.computeClearance();
    const phantoms = (q) => {
      let n = 0;
      for (let i = 0; i < q.n; i++) n += band[i] && v0.st[i] === FREE && q.st[i] !== FREE ? 1 : 0;
      return n * v0.res ** 3;
    };
    const doors = house.doors.filter((d) => d.rooms[1]).map((d) => {
      const x = (d.a[0] + d.b[0]) / 2, y = (d.a[1] + d.b[1]) / 2;
      return [x, y, (map.floorAt(x + 0.05, y) ?? 0) + 1.0];
    });
    const exact = v.integrate({ ...route[5], sigma: 0.02, status: "ok" }, { width: W, height: H, depth: render(truth, route[5]) }, lens, { budget: Infinity });
    assert.equal(exact.contradicted.length, 0, "an exact frame contradicts nothing");
    let worst = 0;
    const narrowed = doors.map(() => 0), least = doors.map((p) => v0.clearance3(...p));
    for (const [k, tp] of route.entries()) {
      const a = rand() * 2 * Math.PI, d = render(truth, tp, 0.03), t0 = performance.now();
      v.integrate({ ...tp, x: tp.x + 0.1 * Math.cos(a), y: tp.y + 0.1 * Math.sin(a), sigma: 0.05, status: "ok" }, { width: W, height: H, depth: d }, lens, { budget: Infinity });
      v.clearance3(tp.x, tp.y, tp.z);
      ms.push(performance.now() - t0);
      if (k % 4) continue;
      await v.refresh(); // the browser refreshes at most every second (4 frames here) in its worker
      worst = Math.max(worst, phantoms(v));
      doors.forEach((p, j) => (least[j] = Math.min(least[j], v.clearance3(...p))));
    }
    ms.sort((p, q) => p - q);
    await v.refresh();
    doors.forEach((p, j) => (narrowed[j] = v0.clearance3(...p) - v.clearance3(...p)));
    const back = V.VoxelMap.load(v.serialize()), need = flat.lethal(SIGMA);
    console.log(`      ${route.length} frames, 10 cm pose error + 3% depth noise: integrate + query median ${ms[ms.length >> 1].toFixed(1)} ms, `
      + `max ${ms.at(-1).toFixed(1)} ms; phantoms at drone heights at most ${worst.toFixed(4)} m³ in flight, ${phantoms(back)} m³ saved; `
      + `doorways ${doors.map((p, j) => `${v0.clearance3(...p).toFixed(2)} m -> at least ${least[j].toFixed(2)}`).join(", ")} in flight`);
    assert.ok(worst < 0.05 && phantoms(v) < 0.05, `phantoms ${worst} m³`);
    assert.equal(phantoms(back), 0, "nothing flights alone found is saved");
    doors.forEach((p, j) => assert.ok(least[j] >= Math.min(need, v0.clearance3(...p)), "no doorway closes for the planner"));
    assert.ok(Math.max(...narrowed) <= 0.03, `doorways at the end: ${narrowed.map((x) => x.toFixed(3))}`);
    assert.ok(ms[ms.length >> 1] < 15, "integrate is cheap enough for the main thread");
    // A person standing 1.2 m ahead for 2 s at 4 Hz: an obstacle while there, forgotten at the end of the flight.
    const pose = { x: 2.6, y: 4.2, z: 1.2, yaw: Math.PI / 2, pitch: 0, roll: 0, sigma: 0.03, status: "ok" }, scene = V.VoxelMap.load(truth.serialize());
    scene.st.set(truth.st);
    const body = [];
    scene.region({ min: [pose.x - 0.25, pose.y + 1.2, 0.0], max: [pose.x + 0.25, pose.y + 1.5, 1.7] }, (i) => ((scene.st[i] = OCCUPIED), v0.st[i] === FREE && body.push(i)));
    const person = render(scene, pose), p = V.VoxelMap.load(v0.serialize());
    p.computeClearance();
    for (let k = 0; k < 8; k++) p.integrate(pose, { width: W, height: H, depth: person }, lens, { budget: Infinity });
    const occ = (q) => body.filter((i) => q.st[i] === OCCUPIED).length;
    assert.ok(occ(p) > 20, `the person is an obstacle during the flight: ${occ(p)} voxels`);
    // clearance3 never reads more than the truth while the refresh is pending (sphere tracing relies on it).
    const mask = p.blockedMask();
    let over = 0, checked = 0;
    for (let s = 0; s < 40000 && checked < 300; s++) {
      const i = Math.floor(rand() * p.n), c = p.center(i);
      if (p.st[i] !== FREE || Math.hypot(c[0] - pose.x, c[1] - pose.y - 1.3) > 2.5) continue;
      checked++;
      over += p.clearance3(...c) > brute(p, i, mask) + 1e-5 ? 1 : 0;
    }
    assert.ok(checked === 300 && over === 0, `clearance3 over the truth at ${over} of ${checked} voxels near the person`);
    assert.equal(occ(V.VoxelMap.load(p.serialize())), 0, "not saved");
    p.forgetFlight();
    assert.equal(occ(p), 0, "forgotten when the flight ends");
  },

  "markSeen / seenAt: what the drone camera saw, until the first surface, from frames worth trusting"() {
    const v = V.VoxelMap.load(vox.serialize()), pose = { x: 2.6, y: 4.2, z: 1.2, yaw: Math.PI / 2, pitch: 0, roll: 0, sigma: 0.05, status: "ok" };
    const r = v.markSeen(pose, DRONE_LENS, 5000);
    assert.ok(r.marked > 1000 && r.skipped === null, `${r.marked} voxels`);
    const ahead = v.raycast([pose.x, pose.y, pose.z], [0, 1, 0], 4);
    assert.equal(v.seenAt(pose.x, pose.y + ahead.d / 2, pose.z), 5000, "on the way");
    assert.equal(v.seenAt(pose.x, pose.y + ahead.d + 0.01, pose.z), 5000, "the surface");
    assert.equal(v.seenAt(pose.x, pose.y - 0.5, pose.z), null, "behind the camera");
    for (let i = 0; i < v.n; i++) if (v.seen[i] && v.st[i] === UNKNOWN) assert.fail("unknown space is never stamped seen");
    for (const [q, why] of [[{ age: 500 }, "stale"], [{ luma: 10 }, "dark"], [{ blurred: true }, "blurred"]])
      assert.deepEqual(v.markSeen(pose, DRONE_LENS, 9000, { quality: q }), { marked: 0, skipped: why });
    assert.equal(v.markSeen({ ...pose, status: "lost" }, DRONE_LENS, 9000).skipped, "pose");
    assert.equal(v.seenAt(pose.x, pose.y + ahead.d / 2, pose.z), 5000, "skipped frames mark nothing");
    v.markSeen(pose, DRONE_LENS, 9000, { width: 640, height: 360, quality: { age: 120, luma: 90 } });
    assert.equal(v.seenAt(pose.x, pose.y + ahead.d / 2, pose.z), 9000, "the last time");
  },

  async "what the splat misses: RoomPlan tops, a glass table, a glossy black top and accepted glass are never free"() {
    // RoomPlan's tables and storage in the fixture: the slab under each top is occupied.
    const tops = house.landmarks.filter((l) => l.source === "roomplan" && l.footprint && /^(table|storage)/.test(l.name));
    const mid = (l) => [0, 1].map((k) => l.footprint.reduce((s, p) => s + p[k], 0) / l.footprint.length);
    assert.ok(tops.length >= 6);
    for (const l of tops) assert.equal(vox.state(...mid(l), l.top - 0.03), OCCUPIED, `${l.name} top at ${l.top}`);
    // A glass table in Room 2: an opaque rim and legs, nothing inside (the floor shows through), and RoomPlan's box.
    const g = { x0: -1.09, x1: -0.19, y0: 1.76, y1: 2.36 }, fl = map.floorAt(-0.64, 2.06), top = fl + 0.5, add = [];
    for (let x = g.x0; x <= g.x1 + 1e-9; x += 0.02)
      for (let y = g.y0; y <= g.y1 + 1e-9; y += 0.02) {
        const rim = Math.min(x - g.x0, g.x1 - x, y - g.y0, g.y1 - y) < 0.04;
        if (rim) add.push([x, y, top - 0.01, 255]);
        if (rim && Math.min(x - g.x0, g.x1 - x) < 0.04 && Math.min(y - g.y0, g.y1 - y) < 0.04)
          for (let z = fl + 0.02; z < top; z += 0.02) add.push([x, y, z, 255]);
      }
    // A glossy black top in Room 3: faint splats only (about 0.1 opacity per 5 cm voxel), and no RoomPlan box.
    const b = { x: 0.11, y: 3.71, s: 0.8 }, fb = map.floorAt(b.x, b.y), btop = fb + 0.45;
    for (let x = b.x - b.s / 2; x < b.x + b.s / 2; x += 0.05) for (let y = b.y - b.s / 2; y < b.y + b.s / 2; y += 0.05) add.push([x + 0.025, y + 0.025, btop - 0.025, 26]);
    const m = centres.opacity.length, cc = { xyz: new Float32Array(3 * (m + add.length)), opacity: new Uint8Array(m + add.length), scale: new Float32Array(m + add.length) };
    cc.xyz.set(centres.xyz);
    cc.opacity.set(centres.opacity);
    cc.scale.set(centres.scale);
    add.forEach(([x, y, z, a], k) => (cc.xyz.set([x, y, z], 3 * (m + k)), (cc.opacity[m + k] = a), (cc.scale[m + k] = 0.02)));
    const glass = { name: "table", source: "roomplan", x: -0.64, y: 2.06, top, footprint: [[g.x0, g.y0], [g.x1, g.y0], [g.x1, g.y1], [g.x0, g.y1]] };
    const pane = { kind: "glass", x: 4.16, y: 0.76, r: 0.3, source: "claude" }, flp = map.floorAt(pane.x, pane.y);
    const withAll = await V.buildVoxels({ house: { ...house, landmarks: [...house.landmarks, glass], keepouts: [...house.keepouts, pane] }, map, centres: cc });
    const without = await V.buildVoxels({ house, map, centres: cc, options: { faint: V.VOX_DEFAULTS.occWeight } });
    const freeShare = (v, [x0, x1, y0, y1], z) => {
      let n = 0, f = 0;
      for (let x = x0 + 0.06; x < x1 - 0.06; x += 0.05) for (let y = y0 + 0.06; y < y1 - 0.06; y += 0.05) (n++, (f += v.state(x, y, z) === FREE ? 1 : 0));
      return f / n;
    };
    const G = [g.x0, g.x1, g.y0, g.y1], B = [b.x - b.s / 2, b.x + b.s / 2, b.y - b.s / 2, b.y + b.s / 2];
    const got = { glass: [freeShare(without, G, top - 0.03), freeShare(withAll, G, top - 0.03)], black: [freeShare(without, B, btop - 0.025), freeShare(withAll, B, btop - 0.025)] };
    console.log(`      free in the top's layer, without -> with: glass ${got.glass.map((v) => `${(100 * v).toFixed(0)}%`).join(" -> ")}, `
      + `glossy black ${got.black.map((v) => `${(100 * v).toFixed(0)}%`).join(" -> ")}`);
    assert.ok(got.glass[0] > 0.5 && got.black[0] > 0.5, "the capture's rays see through both");
    assert.deepEqual([got.glass[1], got.black[1]], [0, 0], "neither is free with RoomPlan's box and the faint layer");
    near(withAll.raycast([-0.64, 2.06, top + 0.5], [0, 0, -1], 2).d, 0.5, 0.06, "straight down onto the glass, not the floor");
    near(withAll.raycast([b.x, b.y, btop + 0.5], [0, 0, -1], 2).d, 0.5, 0.06, "straight down onto the glossy top");
    assert.equal(withAll.state(pane.x, pane.y, flp + 1.0), OCCUPIED, "an accepted glass hazard");
    assert.ok(withAll.flags[withAll.idx(pane.x, pane.y, flp + 1.0)] & FLAG.OBJECT);
    // Live depth sees the floor through the glossy top as the capture did: from three poses 1 m up, pitched down at it,
    // six frames each. Its layer stays unknown in flight and when saved, and seeing through it is not a change.
    const layer = [];
    for (let x = B[0] + 0.06; x < B[1] - 0.06; x += 0.05) for (let y = B[2] + 0.06; y < B[3] - 0.06; y += 0.05) layer.push(withAll.idx(x, y, btop - 0.025));
    assert.ok(layer.every((i) => withAll.flags[i] & FLAG.FAINT), "the faint layer is marked");
    withAll.computeClearance();
    const truth = V.VoxelMap.load(withAll.serialize()), lens = pinholeLens(90), W = 160, H = 120, K = intrinsics(lens, W, H);
    for (let i = 0; i < truth.n; i++) truth.st[i] = truth.st[i] === OCCUPIED ? OCCUPIED : FREE;
    const views = [[0, -1.0, Math.PI / 2], [0.8, 0, Math.PI], [-0.8, 0, 0]].map(([dx, dy, yaw]) => ({ x: b.x + dx, y: b.y + dy, z: fb + 1.0, yaw, pitch: 0.6, roll: 0, sigma: 0.03, status: "ok" }));
    let past = 0, carved = 0;
    for (let f = 0; f < 6; f++)
      for (const p of views) {
        const r = withAll.integrate(p, { width: W, height: H, depth: depthOf(truth, p, K, W, H) }, lens, ALL);
        [past, carved] = [past + r.contradicted.length, carved + r.free];
      }
    const back = V.VoxelMap.load(withAll.serialize()), freeIn = (q) => layer.filter((i) => q.st[i] === FREE).length;
    console.log(`      18 frames seeing the floor through the glossy top (${carved} misses elsewhere): ${freeIn(withAll)} of its ${layer.length} voxels free in flight, ${freeIn(back)} saved`);
    assert.deepEqual([freeIn(withAll), freeIn(back), past], [0, 0, 0], "never free, and not reported as gone");
    assert.ok(carved > 1000 && layer.every((i) => back.flags[i] & FLAG.FAINT), "the flights did carve elsewhere; the mark is saved");
    near(back.raycast([b.x, b.y, btop + 0.5], [0, 0, -1], 2).d, 0.5, 0.06, "straight down still stops at the glossy top");
  },

  async "maps that can tell nothing free say so, and the 2.5D map flies alone"() {
    const noScan = await V.buildVoxels({ house, map, centres: null }), noPath = await V.buildVoxels({ house: { ...house, cameras: [] }, map, centres });
    for (const [v, why] of [[noScan, /splat/], [noPath, /camera path/]]) {
      assert.ok(v.built.usable === false && why.test(v.built.why), v.built.why);
      assert.equal(v.counts().free, 0, "nothing free");
      const rep = C.coverageReport({ house, map, vox: v });
      assert.ok(rep.score === 0 && rep.note === v.built.why && !rep.suggestions.length);
      const m = new HomeMap(house).addSplats(points.xyz, points.opacity, points.scale).finalize().setVoxels(v);
      assert.equal(m.vox, null, "setVoxels ignores it");
      assert.ok(P.plan(m, [house.home.x, house.home.y], P.roomCenter(m, "r3"), { sigma: SIGMA }).ok);
    }
    assert.ok(vox.built.usable && vox.built.why === null);
  },

  "doors open and close in place, and the saved map stays current"() {
    const v = V.VoxelMap.load(vox.serialize()), d = house.doors.find((q) => q.id === "r2:w1-o1+r3:w2-o1");
    const p = [(d.a[0] + d.b[0]) / 2, (d.a[1] + d.b[1]) / 2, d.sillZ + 1.0];
    assert.equal(v.state(...p), FREE, "open at capture");
    assert.ok(v.door(d, false) > 100);
    assert.equal(v.state(...p), OCCUPIED, "closed");
    assert.ok(v.flags[v.idx(...p)] & FLAG.CHANGE);
    v.computeClearance();
    assert.ok(!P.plan(new HomeMap(house).addSplats(points.xyz, points.opacity, points.scale).finalize().setVoxels(v), [-0.64, 2.06], [0.11, 3.71], { sigma: SIGMA }).path
      ?.some((q) => Math.hypot(q[0] - p[0], q[1] - p[1]) < 0.2), "no path through the closed door");
    v.door(d, true);
    assert.equal(v.state(...p), FREE, "open again");
    const flipped = { ...house, doors: house.doors.map((q) => (q === d ? { ...q, passable: false, closedBy: "c1" } : q)) };
    assert.equal(V.voxelKey(flipped), V.voxelKey(house), "a door's state is not in the key");
  },

  "confirmed changes undo exactly: a door closed then opened, an obstacle then gone; never-seen space never turns free"() {
    const v = V.VoxelMap.load(vox.serialize());
    let closed = 0, unknownShut = 0, scanKept = 0;
    for (const d of house.doors) {
      if (!d.passable) v.door(d, true); // closed at capture: open it first, then the cycle
      const before = snap(v), doorway = [];
      v.region({ polygon: doorwayOf(d, Math.max(d.depth ?? 0, 0.12) + v.res), zMin: d.sillZ, zMax: d.headZ }, (i) => doorway.push(i));
      closed += v.door(d, false);
      unknownShut += doorway.filter((i) => before.st[i] === UNKNOWN && v.st[i] === OCCUPIED).length;
      assert.ok(doorway.every((i) => v.st[i] === OCCUPIED), `${d.id}: shut`);
      const back = V.VoxelMap.load(v.serialize()); // the priors are saved with it
      back.door(d, true);
      v.door(d, true);
      assert.equal(diff(v, before), 0, `${d.id}: closed and opened again, ${diff(v, before)} voxels differ`);
      assert.equal(diff(back, before), 0, `${d.id}: the same after a save in between`);
      assert.equal(v.prior.size + back.prior.size, 0, `${d.id}: nothing left to undo`);
      scanKept += doorway.filter((i) => before.flags[i] & FLAG.CAPTURE && v.st[i] === OCCUPIED).length;
    }
    console.log(`      ${house.doors.length} doors closed and opened again: ${closed} voxels shut (${unknownShut} never seen), all back as they were, ${scanKept} jamb and wall-face voxels kept`);
    // A confirmed obstacle across the sealed box's side: inside it (never seen), its wall and the open floor beside it.
    const region = { min: [BOX.x + 0.1, BOX.y - 0.1, BOX.z0 + 0.2], max: [BOX.x + 0.6, BOX.y + 0.1, BOX.z0 + 0.4] }, before = snap(v), inside = [];
    v.region(region, (i) => inside.push(i));
    const kinds = [UNKNOWN, FREE, OCCUPIED].map((s) => inside.filter((i) => before.st[i] === s).length);
    assert.ok(kinds.every((k) => k > 10), `a mixed region: ${kinds}`);
    v.mark(region, OCCUPIED);
    assert.ok(inside.every((i) => v.st[i] === OCCUPIED));
    v.mark(region, FREE, { scan: false });
    assert.equal(diff(v, before), 0, "an obstacle undone leaves the map as it was");
    v.mark(region, "occupied");
    V.VoxelMap.load(v.serialize()).mark(region, "free"); // and through a save
    v.mark(region, "free");
    const freed = inside.filter((i) => before.st[i] !== FREE && v.st[i] === FREE);
    assert.ok(inside.every((i) => before.st[i] !== UNKNOWN || v.st[i] === UNKNOWN), "never-seen space is unknown again");
    assert.ok(freed.length > 0 && freed.every((i) => before.flags[i] & FLAG.CAPTURE), "a removal frees the scan's own surfaces in it, only those");
    // What a flight alone found occupied goes back to its state before the flight, not to free.
    const u = inside.find((i) => before.st[i] === UNKNOWN);
    v.set(u, 40, FLAG.FLIGHT);
    v.mark({ min: v.center(u).map((q) => q - 0.01), max: v.center(u).map((q) => q + 0.01) }, FREE);
    assert.equal(v.st[u], UNKNOWN);
  },

  "slices for the UI: rows from y0 up, heights absolute or above each floor"() {
    const s = vox.slice(1.0, { map }), a = vox.slice(map.floorAt(-0.6, 2.0) + 1.0);
    assert.deepEqual([s.width, s.height, s.x0, s.y0, s.res], [vox.nx, vox.ny, vox.x0, vox.y0, vox.res]);
    const k = map.idx(-0.6, 2.0);
    assert.equal(s.data[k], vox.state(-0.6, 2.0, map.floorZ[k] + 1.0));
    assert.equal(a.data[k], s.data[k]);
    assert.ok(s.data.some((x) => x === FREE) && s.data.some((x) => x === OCCUPIED) && s.data.some((x) => x === UNKNOWN));
  },

  "coverage report: per room volumes, gaps in plain words with where to stand and how high"() {
    t = performance.now();
    const rep = C.coverageReport({ house, map, vox });
    const ms = performance.now() - t;
    console.log(`      score ${rep.score} (${ms.toFixed(0)} ms): ${rep.rooms.map((r) => `${r.name} ${r.knownFreePct}% free, ${r.unknownPct}% unknown, ${r.gaps.length} gaps`).join("; ")}`);
    for (const g of rep.rooms[0].gaps.slice(0, 3)) console.log(`      "${g.text}" (${g.size} m², ${g.heightBand})`);
    for (const s of rep.suggestions.slice(0, 2)) console.log(`      -> "${s.text}" at (${s.x}, ${s.y})`);
    assert.ok(rep.score > 50 && rep.score < 100 && rep.note === null, `score ${rep.score}`);
    assert.equal(rep.flyableClearance, map.lethal(), "flyable: the planner's clearance (drone radius + σ)");
    assert.equal(rep.rooms.length, house.rooms.length);
    for (const r of rep.rooms) {
      near(r.knownFreePct + r.unknownPct + r.occupiedPct, 100, 0.3, `${r.name} adds up`);
      assert.ok(r.flyableM3 > 1, `${r.name} flyable ${r.flyableM3} m³`);
      for (const g of r.gaps) {
        assert.ok(g.text.includes(r.name) && /never seen/.test(g.text) && !/undefined|NaN/.test(g.text), g.text);
        assert.equal(map.roomAt(g.x, g.y)?.id ?? r.id, r.id, `${g.text} at (${g.x}, ${g.y})`);
        assert.match(g.heightBand, /^\d\.\d+-\d\.\d+ m$/);
        assert.ok(g.size > 0);
      }
    }
    assert.ok(rep.rooms[0].gaps.length > 0 && rep.suggestions.length > 0);
    for (const s of rep.suggestions) {
      assert.ok([0.5, 0.7, 1.2].includes(s.height) && /scanner/.test(s.text) && !/undefined|NaN/.test(s.text), s.text);
      assert.ok(map.roomAt(s.x, s.y), `stand at (${s.x}, ${s.y}) inside the house`);
    }
    assert.ok(rep.rooms.flatMap((r) => r.gaps).some((g) => /sofa|table|storage|chair/.test(g.text)), "gaps named by the furniture beside them");
  },

  async "with a twin: depth cubes and a frontier sweep know more, only where a capture camera looked, never through a surface"() {
    // The twin sees surfaces the map lacks: boxes in space no capture ray reached (made of splats too big or too faint for
    // the map) and a sheer 30%-opaque curtain the map has as a faint layer, whose expected depth lands behind it.
    const curtain = { x: -0.64, y0: 1.6, y1: 2.5, h0: 0.3, h1: 1.3 }, fl = map.floorAt(curtain.x, 2.0), add = [];
    for (let y = curtain.y0; y < curtain.y1; y += 0.05) for (let z = fl + curtain.h0; z < fl + curtain.h1; z += 0.05) add.push([curtain.x, y, z]);
    const m = centres.opacity.length, cc = { xyz: new Float32Array(3 * (m + add.length)), opacity: new Uint8Array(m + add.length), scale: new Float32Array(m + add.length) };
    cc.xyz.set(centres.xyz);
    cc.opacity.set(centres.opacity);
    cc.scale.set(centres.scale);
    add.forEach((q, k) => (cc.xyz.set(q, 3 * (m + k)), (cc.opacity[m + k] = 26), (cc.scale[m + k] = 0.03)));
    const base = await V.buildVoxels({ house, map, centres: cc }), truth = V.VoxelMap.load(base.serialize()), thin = new Uint8Array(truth.n);
    for (let i = 0; i < truth.n; i++) truth.st[i] = truth.st[i] === OCCUPIED ? OCCUPIED : FREE;
    for (const q of add) {
      const i = truth.idx(...q);
      [truth.st[i], thin[i]] = [OCCUPIED, 1];
    }
    const band = V.bandMask(base, map), hidden = [], layer = base.nx * base.ny;
    for (let i = 0; i < base.n && hidden.length < 6; i++) {
      if (!band[i] || base.st[i] !== UNKNOWN || base.st[i + layer] !== FREE || thin[i]) continue;
      const c = base.center(i);
      if (hidden.some((h) => Math.hypot(h[0] - c[0], h[1] - c[1]) < 1.0) || Math.hypot(c[0] - BOX.x, c[1] - BOX.y) < 0.6) continue;
      hidden.push(c);
      truth.region({ x: c[0], y: c[1], z: c[2], r: 0.08 }, (j) => base.st[j] === UNKNOWN && (truth.st[j] = OCCUPIED));
    }
    const twin = fakeTwin(truth, thin), progress = [];
    t = performance.now();
    const v = await V.buildVoxels({ house, map, centres: cc, twin, onProgress: (p) => progress.push(p.phase),
      options: { twinSize: 64, capturePositions: 0.6, frontierViews: 240, frontierRound: 10 } });
    const ms = performance.now() - t, a = [0, 0, 0], b = [0, 0, 0];
    for (let i = 0; i < v.n; i++) if (band[i]) (a[base.st[i]]++, b[v.st[i]]++);
    console.log(`      ${twin.calls.poses} depth views in ${twin.calls.batches} batches, ${ms.toFixed(0)} ms: unknown at drone heights ${(a[0] * 0.000125).toFixed(2)} -> ${(b[0] * 0.000125).toFixed(2)} m³`);
    assert.ok(b[0] < a[0] * 0.75, "the twin resolves unknown space");
    assert.ok(v.built.views === twin.calls.poses && v.built.views > 0 && v.built.usable);
    assert.deepEqual([...new Set(progress)], ["splats", "capture", "twin", "frontier", "clearance", "done"]);
    let bad = 0, curtainFree = 0;
    for (let i = 0; i < v.n; i++) if (v.st[i] === FREE && truth.st[i] === OCCUPIED) (bad++, (curtainFree += thin[i]));
    assert.equal(bad, 0, `${bad} surfaces carved (${curtainFree} of the curtain)`);
    for (let z = BOX.z0 + 0.1; z < BOX.z0 + BOX.s - 0.08; z += 0.05) assert.equal(v.state(BOX.x, BOX.y, z), UNKNOWN, "the sealed box");
    // The twin carves only what some capture camera had a line of sight to (up to the first occupied voxel).
    const cams = house.cameras.filter((c) => v.idx(c[0], c[1], c[2]) >= 0), visible = V.carveFromCameras(V.VoxelMap.load(base.serialize()), cams, V.VOX_DEFAULTS, null);
    let blind = 0, twinFree = 0;
    for (let i = 0; i < v.n; i++) if (v.st[i] === FREE && v.flags[i] & FLAG.TWIN && !(v.flags[i] & FLAG.CARVED)) (twinFree++, (blind += visible[i] ? 0 : 1));
    assert.ok(twinFree > 1000 && blind === 0, `${blind} of ${twinFree} twin-carved voxels no capture camera saw`);
  },

  "edt3 on its own: a single blocked voxel, an empty grid"() {
    const b = new Uint8Array(5 * 5 * 5);
    b[62] = 1; // the centre
    const d = V.edt3(b, 5, 5, 5, 1);
    assert.equal(d[62], 0);
    near(d[0], Math.sqrt(12) - 0.5, 1e-6, "corner");
    assert.equal(V.edt3(new Uint8Array(8), 2, 2, 2, 1, 3)[0], 3, "nothing blocked: the cap");
  },
};

let failed = 0;
const T0 = performance.now();
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name}\n      ${e.stack?.split("\n").slice(0, 3).join("\n      ")}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed (${((performance.now() - T0) / 1000).toFixed(1)} s)`);
process.exit(failed ? 1 : 0);
