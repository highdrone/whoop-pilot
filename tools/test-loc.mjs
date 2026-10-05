// Vision localization against the splat twin (docs/HOME-DRONE.md, "Wave C contracts"), headless: the O4 lens model and
// the fisheye -> pinhole rectification (nav/lens.js on twin/lens.js's math), camera <-> body poses, lens calibration from
// matched points, OpenCV.js PnP (the vendored build, run in Node), XFeat post-processing and the mutual matcher, the
// overlay mask, the relocalization database, SplatLocalizer's checks and scheduling (fake twin and worker), the EKF with
// splat-style fixes (noise, latency, outliers, height bias, lost and relocalized), and the simulator's scene changes.
// The browser side (real XFeat, DINOv2, twin renders, a flight through the house) is tools/loc-check.html.
// Usage: cd tools && node test-loc.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { createRequire } from "node:module";

let clock = 0;
Object.defineProperty(globalThis, "performance", { value: { now: () => clock }, configurable: true, writable: true });

const TL = await import("../app/js/twin/lens.js");
const L = await import("../app/js/nav/lens.js");
const F = await import("../app/js/nav/features.js");
const { OsdMask } = await import("../app/js/nav/osdmask.js");
const { SplatLocalizer, RelocDb, relocPoses, SPLAT } = await import("../app/js/nav/splatloc.js");
const { Localizer, LOC } = await import("../app/js/nav/localizer.js");
const { augmentOptions, simLens, LENS_ERROR, jpegLoss } = await import("../app/js/sim/augment.js");
const { Simulator } = await import("../app/js/sim/simulator.js");
const { Emitter, wrapAngle, DEG } = await import("../app/js/util.js");
const I = await import("../app/js/house/import.js");

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const { house, map } = await I.importCapture(fsSource(FIXTURE));

const T0 = Date.now();
const rng = (seed) => () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
const gaussOf = (r) => () => Math.sqrt(-2 * Math.log(Math.max(1e-12, r()))) * Math.cos(2 * Math.PI * r());
const q = (a, f) => { const b = a.filter(Number.isFinite).sort((x, y) => x - y); return b.length ? b[Math.min(b.length - 1, Math.floor(f * b.length))] : NaN; };
const fmt = (v, d = 3) => (Number.isFinite(v) ? v.toFixed(d) : String(v));
const close = (a, b, tol, what = "") => assert.ok(Math.abs(a - b) <= tol, `${what} ${a} vs ${b} (tol ${tol})`);
const sub = (a, b) => a.map((v, i) => v - b[i]);

// A controller as the localizer sees it: heading (clockwise, drifting), flow velocity (forward, right) / scale, climb rate.
class FakeCtl extends Emitter {
  constructor(videoDelay = 100) {
    super();
    this.est = { heading: 0, vx: 0, vy: 0, vz: 0, flowQ: 0.8, rotating: false, pitchAngle: 0, rollAngle: 0 };
    this.perception = { latest: { flow: { t: 0 } } };
    Object.assign(this, { videoDelay, flying: true });
  }
  isFlying() { return this.flying; }
}

const tests = {
  "lens: nav/lens.js is twin/lens.js's model (same functions), calib.json round trip, project/unproject inverse over the fisheye"() {
    assert.equal(L.project, TL.project);
    assert.equal(L.unproject, TL.unproject);
    assert.equal(L.intrinsics, TL.intrinsics);
    const def = L.droneLens(null, 25);
    assert.equal(def.model, "equidistant");
    assert.equal(def.diagFovDeg, 159);
    assert.equal(def.uptiltDeg, 25);
    for (const lens of [def, L.droneLens(L.calibOf(L.droneLens(null, 20), 640, 480)), TL.resolveLens(simLens(127, 20, LENS_ERROR))]) {
      const K = L.intrinsics(lens, 640, 480), back = L.droneLens(L.calibOf(lens, 640, 480)), K2 = L.intrinsics(back, 640, 480);
      for (const k of ["fx", "fy", "cx", "cy", "k1"]) close(K2[k], K[k], 1e-3, `calib round trip ${k}`);
      close(back.uptiltDeg, lens.uptiltDeg, 0.01, "uptilt");
      const r = rng(5);
      let worst = 0;
      for (let i = 0; i < 2000; i++) {
        const u = r() * 640, v = r() * 480, d = L.unproject(K, u, v);
        if (!d) continue;
        const p = L.project(K, d);
        worst = Math.max(worst, Math.hypot(p[0] - u, p[1] - v));
      }
      assert.ok(worst < 1e-6, `project(unproject) off by ${worst} px`);
    }
    // 159 deg across the diagonal, about 127 x 95 through the centre
    const fov = TL.fieldOfView(def, 640, 480);
    close(fov.diag, 159, 0.5, "diagonal fov");
    close(fov.h, 127.2, 1, "horizontal fov");
  },

  "rectify: the fisheye -> 100 deg pinhole lookup samples the right rays (8-bit error), maps back exactly, 640x480 and 320x240"() {
    const src = L.droneLens(null, 20), W = 640, H = 480, Kf = L.intrinsics(src, W, H);
    // A smooth scene on the sphere of directions, drawn into the fisheye frame
    const scene = ([x, y, z]) => 127 + 90 * Math.sin(3 * Math.atan2(x, z)) * Math.cos(4 * Math.atan2(y, Math.hypot(x, z)));
    const img = new Uint8ClampedArray(W * H * 4);
    for (let v = 0, i = 0; v < H; v++) for (let u = 0; u < W; u++, i += 4) {
      const d = L.unproject(Kf, u + 0.5, v + 0.5), g = d ? scene(d) : 0;
      img[i] = img[i + 1] = img[i + 2] = g;
      img[i + 3] = 255;
    }
    for (const [ow, oh] of [L.RECT_SIZES.full, L.RECT_SIZES.half]) {
      let t = Date.now();
      const m = L.rectifyMap(src, W, H, ow, oh), r = new L.Rectifier(m), lutMs = Date.now() - t;
      t = Date.now();
      const out = r.rgba(img), ms = Date.now() - t, errs = [];
      for (let v = 0, i = 0; v < oh; v++) for (let u = 0; u < ow; u++, i++) {
        if (!r.valid[i]) continue;
        errs.push(Math.abs(out[4 * i] - scene(L.unproject(m.K, u + 0.5, v + 0.5))));
      }
      const rnd = rng(9);
      let back = 0;
      for (let k = 0; k < 500; k++) {
        const u = 1 + rnd() * (ow - 2), v = 1 + rnd() * (oh - 2), s = r.toSource(u, v);
        if (!s) continue;
        const p = L.project(m.K, L.unproject(Kf, s[0], s[1]));
        back = Math.max(back, Math.hypot(p[0] - u, p[1] - v));
      }
      console.log(`      ${ow}x${oh}: ${(m.valid * 100).toFixed(1)}% inside the fisheye, error median ${fmt(q(errs, 0.5), 2)} / p99 ${fmt(q(errs, 0.99), 2)} levels; LUT ${lutMs} ms, resample ${ms} ms; toSource round trip ${fmt(back, 4)} px`);
      assert.ok(m.valid > 0.97, "the 100 deg view lies inside the 159 deg fisheye");
      assert.ok(q(errs, 0.5) < 1 && q(errs, 0.99) < 4, "resampling error");
      assert.ok(back < 0.05, "toSource inverts the map");
    }
  },

  "poses: camera <-> body (any uptilt), OpenCV rvec/tvec <-> camera, a point straight ahead lands on the principal point"() {
    const r = rng(3);
    for (let i = 0; i < 200; i++) {
      const pose = { x: r() * 6 - 1, y: r() * 6, z: 0.3 + r() * 1.5, yaw: wrapAngle(r() * 7), pitch: (r() - 0.5) * 0.6, roll: (r() - 0.5) * 0.6 }, up = 15 + r() * 20;
      const { R, C } = L.cameraOf(pose, up), b = L.bodyOf(R, C, up);
      for (const k of ["x", "y", "z"]) close(b[k], pose[k], 1e-9, k);
      for (const k of ["yaw", "pitch", "roll"]) close(wrapAngle(b[k] - pose[k]), 0, 1e-9, k);
      const { rvec, tvec } = L.pnpFromCamera(R, C), c2 = L.cameraFromPnP(rvec, tvec);
      assert.ok(L.rotAngle(c2.R, R) < 1e-6 && Math.hypot(...sub(c2.C, C)) < 1e-9, "rvec/tvec round trip");
      const K = L.intrinsics(L.rectLens(up), 640, 480), ahead = [C[0] + 2 * R[0][2], C[1] + 2 * R[1][2], C[2] + 2 * R[2][2]], p = L.projectH(K, R, C, ahead);
      assert.ok(Math.hypot(p[0] - 320, p[1] - 240) < 1e-6);
    }
    // level body, 20 deg uptilt: the optical axis rises 20 deg
    const { R } = L.cameraOf({ x: 0, y: 0, z: 1, yaw: 0, pitch: 0, roll: 0 }, 20);
    close(Math.asin(R[2][2]) / DEG, 20, 1e-9, "uptilt");
  },

  "calibrateLens: a wider, off-centre lens with 2 deg more uptilt is recovered from a pad turn (fx 1%, centre 2 px, uptilt 0.5 deg)"() {
    const truth = TL.resolveLens(simLens(127, 20, LENS_ERROR)), Kt = L.intrinsics(truth, 640, 480), r = rng(21), g = gaussOf(rng(22));
    // a room's walls and furniture as points, the drone on the pad turning slowly
    const pts = [];
    for (let i = 0; i < 3000; i++) {
      const a = r() * 2 * Math.PI, d = 1.2 + r() * 3.5;
      pts.push([d * Math.cos(a), d * Math.sin(a), r() * 2.4]);
    }
    const samples = [];
    for (let k = 0; k < 6; k++) {
      const pose = { x: 0, y: 0, z: 0.05, yaw: (k * 60 + 7) * DEG, pitch: 0, roll: 0 }, cam = L.cameraOf(pose, truth.uptiltDeg), uv = [], xyz = [];
      for (const X of pts) {
        const p = L.projectH(Kt, cam.R, cam.C, X);
        if (!p || p[0] < 2 || p[1] < 2 || p[0] > 638 || p[1] > 478 || uv.length >= 400) continue;
        uv.push(p[0] + 0.3 * g(), p[1] + 0.3 * g());
        xyz.push(...X);
      }
      const guess = L.cameraOf({ ...pose, yaw: pose.yaw + 0.8 * DEG, x: 0.02 }, 20); // PnP with the assumed lens: a little off
      samples.push({ uv: Float64Array.from(uv), xyz: Float64Array.from(xyz), R: guess.R, C: guess.C, grounded: true });
    }
    const t = Date.now(), { calib } = L.calibrateLens(samples, { lens: L.droneLens(null, 20) }), K0 = L.intrinsics(L.droneLens(null, 20), 640, 480);
    console.log(`      true fx ${fmt(Kt.fx, 1)} cx ${fmt(Kt.cx, 1)} cy ${fmt(Kt.cy, 1)} uptilt ${truth.uptiltDeg}; assumed fx ${fmt(K0.fx, 1)}; calibrated fx ${calib.fx} fy ${calib.fy} cx ${calib.cx} cy ${calib.cy} k ${calib.k.slice(0, 2)} uptilt ${calib.uptiltDeg}, rms ${calib.rms} px (${calib.points} points, ${Date.now() - t} ms)`);
    close(calib.fx / Kt.fx, 1, 0.01, "fx");
    close(calib.fy / Kt.fy, 1, 0.01, "fy");
    close(calib.cx, Kt.cx, 2, "cx");
    close(calib.cy, Kt.cy, 2, "cy");
    close(calib.uptiltDeg, truth.uptiltDeg, 0.5, "uptilt");
    assert.ok(calib.rms < 0.6, `rms ${calib.rms}`);
    assert.equal(calib.model, "equidistant");
    // and it becomes the drone lens
    const back = L.intrinsics(L.droneLens(calib), 640, 480);
    close(back.fx, calib.fx, 1e-6, "droneLens(calib)");
  },

  async "PnP: the vendored OpenCV.js (gzipped, checked) loads in Node; RANSAC EPnP + LM finds the camera through 30% outliers (1 cm, 0.1 deg) with a sane covariance"() {
    let t = Date.now();
    const dir = path.join(import.meta.dirname, "..", "app", "vendor", "opencv");
    const { cv } = await F.loadOpenCV(await F.openCvText(fs.readFileSync(path.join(dir, "opencv.js.gz"))), { require: createRequire(import.meta.url), dirname: dir });
    assert.ok(fs.statSync(path.join(dir, "opencv.js.gz")).size < 4e6 && !fs.existsSync(path.join(dir, "opencv.js")), "vendored gzipped (3.5 MB), checked unpacked");
    const loadMs = Date.now() - t, K = L.intrinsics(L.rectLens(20), 640, 480), r = rng(13), g = gaussOf(rng(14));
    const errs = [];
    for (let trial = 0; trial < 5; trial++) {
      const pose = { x: 1 + r() * 3, y: 1 + r() * 3, z: 0.6 + r(), yaw: r() * 6, pitch: (r() - 0.5) * 0.3, roll: (r() - 0.5) * 0.3 }, cam = L.cameraOf(pose, 20);
      const obj = [], img = [];
      for (let i = 0; i < 500; i++) {
        const u = r() * 640, v = r() * 480, d = L.unproject(K, u, v), s = 0.8 + r() * 5, c = d.map((x) => (x * s) / d[2]);
        obj.push(...[0, 1, 2].map((k) => cam.C[k] + cam.R[k][0] * c[0] + cam.R[k][1] * c[1] + cam.R[k][2] * c[2]));
        img.push(...(r() < 0.3 ? [r() * 640, r() * 480] : [u + 0.5 * g(), v + 0.5 * g()]));
      }
      t = Date.now();
      const sol = F.solvePnP(cv, Float64Array.from(obj), Float64Array.from(img), K, { thr: 4 });
      const ms = Date.now() - t, body = L.bodyOf(sol.R, sol.C, 20);
      errs.push({ pos: Math.hypot(...sub(sol.C, cam.C)), rot: L.rotAngle(sol.R, cam.R) / DEG, yaw: Math.abs(wrapAngle(body.yaw - pose.yaw)) / DEG, inl: sol.inliers.length, rms: sol.rms, sx: Math.sqrt(sol.cov[0][0]), ms });
    }
    console.log(`      OpenCV.js ${loadMs} ms to load; ${errs.map((e) => `${fmt(e.pos * 100, 2)} cm / ${fmt(e.rot, 3)} deg, ${e.inl} inliers, rms ${fmt(e.rms, 2)} px, sigma x ${fmt(e.sx * 100, 2)} cm, ${e.ms} ms`).join("; ")}`);
    for (const e of errs) {
      assert.ok(e.pos < 0.01 && e.rot < 0.1 && e.yaw < 0.1, `pose error ${e.pos} m ${e.rot} deg`);
      assert.ok(e.inl > 320 && e.inl < 380, `${e.inl} inliers of ~350`);
      assert.ok(e.sx > 1e-4 && e.sx < 0.01, `covariance ${e.sx}`);
    }
    assert.equal(F.solvePnP(cv, new Float64Array(9), new Float64Array(6), K), null, "too few points");
  },

  "XFeat post-processing: softmax cells -> a keypoint at the peak's pixel, unit descriptors, topK, masked pixels skipped; mutual NN"() {
    const W = 64, H = 48, w8 = 8, h8 = 6, n8 = w8 * h8, logits = new Float32Array(65 * n8), feats = new Float32Array(64 * n8), rel = new Float32Array(n8).fill(1);
    for (let p = 0; p < n8; p++) logits[64 * n8 + p] = 4; // dustbin: no keypoint anywhere
    const r = rng(2);
    for (let i = 0; i < feats.length; i++) feats[i] = r() - 0.5;
    const peak = (x8, y8, c, v) => (logits[c * n8 + y8 * w8 + x8] = v);
    peak(3, 2, 19, 12); // pixel (3*8 + 3, 2*8 + 2)
    peak(5, 4, 0, 10); // pixel (40, 32)
    const f = F.xfeatPost(feats, logits, rel, W, H, { topK: 8 });
    assert.equal(f.n, 2);
    assert.deepEqual([f.kpts[0], f.kpts[1]], [27, 18]);
    assert.deepEqual([f.kpts[2], f.kpts[3]], [40, 32]);
    for (let i = 0; i < f.n; i++) close(Math.hypot(...f.desc.subarray(64 * i, 64 * i + 64)), 1, 1e-5, "unit descriptor");
    assert.equal(F.xfeatPost(feats, logits, rel, W, H, { topK: 1 }).n, 1);
    const g = F.xfeatPost(feats, logits, rel, W, H, { allow: (x, y) => !(x === 27 && y === 18) });
    assert.deepEqual([g.n, g.kpts[0], g.kpts[1]], [1, 40, 32]);
    // mutual nearest neighbours: b is a shuffled, noisy a plus distractors
    const n = 60, dA = new Float32Array(64 * n), perm = Array.from({ length: n }, (_, i) => i).sort(() => r() - 0.5), m = n + 30, dB = new Float32Array(64 * m);
    const unit = (a, o) => { let s = 0; for (let c = 0; c < 64; c++) s += a[o + c] ** 2; for (let c = 0; c < 64; c++) a[o + c] /= Math.sqrt(s); };
    for (let i = 0; i < n; i++) { for (let c = 0; c < 64; c++) dA[64 * i + c] = r() - 0.5; unit(dA, 64 * i); }
    for (let j = 0; j < m; j++) {
      for (let c = 0; c < 64; c++) dB[64 * j + c] = j < n ? dA[64 * perm[j] + c] + 0.03 * (r() - 0.5) : r() - 0.5;
      unit(dB, 64 * j);
    }
    const { pairs } = F.matchCpu({ n, desc: dA }, { n: m, desc: dB });
    let right = 0;
    for (let k = 0; k < pairs.length; k += 2) right += perm[pairs[k + 1]] === pairs[k];
    assert.ok(right >= 0.95 * n && right === pairs.length / 2, `${right}/${pairs.length / 2} right of ${n}`);
    const model = F.mnnModel(16, 16);
    assert.deepEqual([model[0], model[1]], [0x08, 8], "ONNX ir_version 8");
    assert.ok(Buffer.from(model).includes("ArgMax") && Buffer.from(model).includes("ReduceMax"));
  },

  "lift: render keypoints to H through the twin's depth; depth edges and holes give no point"() {
    const W = 64, H = 48, K = L.intrinsics(L.rectLens(20), W, H), cam = L.cameraOf({ x: 1, y: 2, z: 1, yaw: 0.3, pitch: 0, roll: 0 }, 20);
    const depth = new Float32Array(W * H);
    for (let v = 0; v < H; v++) for (let u = 0; u < W; u++) {
      const d = L.unproject(K, u + 0.5, v + 0.5), z = u < 40 ? 2 : 4; // a wall 2 m ahead, then a step to 4 m
      depth[v * W + u] = v === 5 && u === 5 ? NaN : z / d[2];
    }
    const kpts = Float32Array.from([10, 20, 39, 20, 50, 30, 5, 5]), P = F.lift(kpts, 4, depth, W, H, K, cam);
    for (const i of [0, 2]) {
      const X = [P[3 * i], P[3 * i + 1], P[3 * i + 2]], d = sub(X, cam.C), zc = d[0] * cam.R[0][2] + d[1] * cam.R[1][2] + d[2] * cam.R[2][2];
      close(zc, i ? 4 : 2, 1e-4, "camera depth");
      const p = L.projectH(K, cam.R, cam.C, X);
      close(p[0], kpts[2 * i] + 0.5, 1e-3, "u");
      close(p[1], kpts[2 * i + 1] + 0.5, 1e-3, "v");
    }
    assert.ok(Number.isNaN(P[3]) && Number.isNaN(P[9]), "the edge and the hole");
  },

  "OSD mask: burned-in text, a ticking timer, crosshair and the fisheye's black border are masked from moving, noisy frames; still frames teach nothing"() {
    const W = 160, H = 120, o = new OsdMask({ width: W, height: H, minFrames: 20 }), r = rng(4), g = gaussOf(rng(41));
    const osd = (x, y) => (y >= 6 && y < 14 && ((x >= 6 && x < 40) || (x >= 120 && x < 154))) || (y >= 106 && y < 114 && x >= 6 && x < 50) || (Math.abs(x - 80) <= 6 && y === 60) || (Math.abs(y - 60) <= 6 && x === 80);
    const timer = (x, y) => y >= 6 && y < 14 && x >= 140 && x < 154; // its seconds digits: a new value every 10 frames
    const border = (x, y) => Math.hypot((x - 80) / 1.05, y - 60) > 95;
    let k = 0;
    const frame = (ox, oy, noise = 6) => {
      const f = new Float32Array(W * H);
      for (let y = 0; y < H; y++) for (let x = 0; x < W; x++)
        f[y * W + x] = Math.max(0, Math.min(255, (border(x, y) ? 3 : timer(x, y) ? 140 + 100 * Math.sin(x * 1.3 + Math.floor(k / 10)) : osd(x, y) ? 250
          : 128 + 60 * Math.sin((x + ox) * 0.31) * Math.cos((y + oy) * 0.23) + 30 * Math.sin((x + ox) * 0.07 + (y + oy) * 0.11)) + noise * g()));
      return f;
    };
    assert.equal(o.add(frame(0, 0)), false, "the first frame only primes it");
    assert.equal(o.add(frame(0, 0)), false, "a still (noisy) frame is ignored");
    for (let i = 0; i < 10; i++) o.add(frame(0, 0, 10));
    assert.equal(o.frames, 0, "noise is not motion");
    for (; k < 40; k++) o.add(frame(r() * 40, r() * 40));
    assert.ok(o.ready);
    const m = o.mask();
    let osdHit = 0, osdN = 0, bHit = 0, bN = 0, keep = 0, keepN = 0;
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      const i = y * W + x;
      if (osd(x, y)) (osdN++, (osdHit += !m[i]));
      else if (border(x, y)) (bN++, (bHit += !m[i]));
      else if (Math.hypot(x - 80, y - 60) < 80 && ![-1, 0, 1].some((d) => osd(x + d, y) || osd(x, y + d))) (keepN++, (keep += m[i]));
    }
    console.log(`      overlay ${osdHit}/${osdN} masked (timer included), border ${bHit}/${bN}, scene kept ${keep}/${keepN}; coverage ${fmt(o.coverage(), 3)}`);
    assert.ok(osdHit === osdN && bHit >= 0.98 * bN && keep >= 0.97 * keepN);
    const back = OsdMask.from(JSON.parse(JSON.stringify(o.toJSON())));
    assert.deepEqual(Array.from(back.mask()), Array.from(m), "calib.json osdMask round trip");
  },

  async "OSD mask on the pad: a still picture with auto exposure hunting or mains-flicker bands never counts as motion, a moving one with the same light still learns the overlay; vision learns only while flying or turning, never uses a mask hiding over 30% (learned, calib.json, saved by a calibration), drops one after which the matches collapse, and live depth holds the same mask object"() {
    const W = 160, H = 120, r = rng(9), g = gaussOf(rng(10)), box = (x, y) => y >= 8 && y < 16 && x >= 10 && x < 60;
    const light = () => ({ gain: 0.8 + 0.4 * r(), c: 1 + 0.1 * (r() - 0.5), ph: r() * 2 * Math.PI });
    const shot = (ox, oy, { gain, c, ph }) => {
      const f = new Float32Array(W * H);
      for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
        const v = 128 + 60 * Math.sin((x + ox) * 0.31) * Math.cos((y + oy) * 0.23) + 30 * Math.sin((x + ox) * 0.07 + (y + oy) * 0.11);
        f[y * W + x] = box(x, y) ? 250 : Math.max(0, Math.min(255, ((v - 128) * c + 128) * gain + 6 * Math.sin((2 * Math.PI * y) / 40 + ph) + 3 * g()));
      }
      return f;
    };
    const still = new OsdMask({ width: W, height: H, minFrames: 20 });
    for (let k = 0; k < 80; k++) still.add(shot(0, 0, light()));
    assert.equal(still.frames, 0, "exposure hunting and flicker on a still picture are not motion");
    const moving = new OsdMask({ width: W, height: H, minFrames: 20 });
    for (let k = 0; k < 60 && !moving.ready; k++) moving.add(shot(r() * 40, r() * 40, light()));
    const m = moving.mask();
    let hit = 0, n = 0;
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) if (box(x, y)) (n++, (hit += !m[y * W + x]));
    console.log(`      still: 0 of 80 frames counted; moving: ready after ${moving.frames}, overlay ${hit}/${n} masked, coverage ${fmt(moving.coverage(), 3)}`);
    assert.ok(moving.ready && hit === n && moving.usable && moving.coverage() < 0.15);
    const bad = OsdMask.from({ width: W, height: H, rle: [10, W * H - 20, 10] }); // hides nearly everything
    assert.ok(bad.ready && !bad.usable);
    // one mask object: vision's and live depth's; calib.json's mask copied into it; a bad one never used
    const { LiveDepth } = await import("../app/js/vision/depth.js");
    const ctl = new FakeCtl(100), loc = new Localizer({ ctl, map: { floorAt: () => 0, ceilingAt: () => 2.6, roomAt: () => ({ id: "r1" }) } });
    const calib = { ...L.calibOf(L.droneLens(null, 20), 640, 480), region: { sx: 240, sy: 0, sw: 1440, sh: 1080 }, verified: { err: 0.01 } };
    const sp = new SplatLocalizer({ localizer: loc, ctl, lens: L.droneLens(null, 20) }), depth = new LiveDepth({ perception: new Emitter(), localizer: loc, ctl, osd: sp.osd });
    sp.setCalib({ ...calib, osdMask: moving.toJSON() });
    await new Promise((res) => setImmediate(res));
    assert.ok(depth.osd === sp.osd && depth.osd.ready && depth.osd.usable && depth.osd.fixed, "live depth sees calib.json's mask");
    sp.setCalib({ ...calib, osdMask: bad.toJSON() });
    await new Promise((res) => setImmediate(res));
    assert.ok(sp.osd.usable && sp.osd.coverage() < 0.15, "a mask hiding nearly everything isn't taken");
    assert.ok(!new SplatLocalizer({ localizer: loc, ctl, calib: { ...calib, osdMask: bad.toJSON() } }).osd.ready);
    assert.equal(await sp.setOsdMask(bad), false);
    // learned only while the drone flies or turns
    const learner = new SplatLocalizer({ localizer: loc, ctl, lens: L.droneLens(null, 20) });
    let learned = 0;
    learner.learnOsd = () => learned++;
    const el = { width: 640, height: 480 }, feed = (k) => { for (let i = 0; i < k; i++) learner.onFrame({ t: clock, element: el, region: { sx: 0, sy: 0, sw: 640, sh: 480 }, picture: true }); };
    ctl.flying = false;
    feed(30);
    assert.equal(learned, 0, "on the pad, still: nothing learned");
    ctl.est.rotating = true;
    feed(30);
    ctl.est.rotating = false;
    ctl.flying = true;
    feed(30);
    assert.equal(learned, 20, "turning by hand or flying: every third frame");
    // a new mask after which the matches collapse is dropped; the worker keeps only the fisheye border
    const { sl, fake, setTruth, loc: rl } = splatRig(), statuses = [], masks = [];
    sl.on("status", (st) => statuses.push(st.text));
    setTruth({ x: 2, y: 3, z: 1, yaw: 0.4 });
    rl.reset({ x: 2, y: 3, z: 1, yaw: 0.4 });
    fake.setMask = async (mm) => (masks.push(mm), { masked: 0.05 });
    await sl.track(frame(), clock);
    sl.rect = { key: "k" };
    for (let k = 0; k < 3; k++) await sl.track(frame(), (clock += 200));
    assert.ok(sl.installOsd(moving) && sl.osd.usable);
    fake.inliers = 12;
    for (let k = 0; k < SPLAT.osdCollapse.tries; k++) await sl.track(frame(), (clock += 200));
    assert.ok(!sl.osd.ready && sl.osdDrops === 1 && masks.at(-1).width === 1 && masks.at(-1).data[0] === 1, "dropped");
    assert.ok(statuses.some((t) => /stopped using its on-screen display mask: the camera matched the 3D scan far worse with it \(12 points, 500 before\)/.test(t)), statuses.join(" | "));
    fake.inliers = 500;
    sl.installOsd(moving);
    for (let k = 0; k < SPLAT.osdCollapse.tries; k++) await sl.track(frame(), (clock += 200));
    assert.ok(sl.osd.usable && sl.osdDrops === 1, "a mask that matches as well stays");
    // a live calibration saves only a usable learned mask (calibrate()'s calib.json)
    assert.ok(/this\.osd\.usable \? \{ \.\.\.out, osdMask/.test(SplatLocalizer.prototype.calibrate.toString()));
  },

  "relocalization database: known-free poses only (map.free), 12 headings; reloc.bin round trip; search finds the frame's keyframe, no near duplicates"() {
    const poses = relocPoses(map, { grid: 0.5, heights: [0.7, 1.3], headings: 12 });
    assert.ok(poses.length > 200);
    for (const p of poses) assert.ok(map.free(p.x, p.y, p.z) && map.roomAt(p.x, p.y), `pose ${JSON.stringify(p)}`);
    const spots = new Map();
    for (const p of poses) spots.set(`${p.x},${p.y},${p.z}`, (spots.get(`${p.x},${p.y},${p.z}`) ?? 0) + 1);
    assert.ok([...spots.values()].every((n) => n === 12));
    const r = rng(8), dim = 768, n = 400, desc = [], qd = new Int8Array(n * dim), scale = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const d = new Float32Array(dim);
      let s = 0, m = 0;
      for (let k = 0; k < dim; k++) (d[k] = r() - 0.5), (s += d[k] ** 2);
      for (let k = 0; k < dim; k++) (d[k] /= Math.sqrt(s)), (m = Math.max(m, Math.abs(d[k])));
      for (let k = 0; k < dim; k++) qd[i * dim + k] = Math.round((d[k] * 127) / m);
      scale[i] = m / 127;
      desc.push(d);
    }
    const P = Float32Array.from(poses.slice(0, n).flatMap((p) => [p.x, p.y, p.z, p.yaw]));
    const db = RelocDb.load(new RelocDb({ key: "k", n, dim, grid: 0.5 }, P, qd, scale).serialize());
    assert.equal(db.meta.key, "k");
    assert.deepEqual(Array.from(db.poses), Array.from(P));
    const hits = db.search(desc[123], { k: 5 });
    assert.equal(hits[0].i, 123);
    assert.ok(hits[0].score > 0.98 && hits[1].score < 0.5);
    for (let a = 0; a < hits.length; a++)
      for (let b = a + 1; b < hits.length; b++)
        assert.ok(!(Math.hypot(hits[a].x - hits[b].x, hits[a].y - hits[b].y) < 0.3 && Math.abs(hits[a].z - hits[b].z) < 0.2 && Math.abs(wrapAngle(hits[a].yaw - hits[b].yaw)) < 0.4));
    console.log(`      ${poses.length} keyframes in the fixture (${spots.size} positions), ${(db.bytes / 1e6).toFixed(2)} MB per ${n}`);
  },

  "relocalization database with pitched keyframes (opt-in): pitch kept per keyframe through reloc.bin, given back by search; the plain one unchanged"() {
    const poses = relocPoses(map, { grid: 1, heights: [1], headings: 4, pitches: [0, 40 * DEG, -40 * DEG] });
    assert.ok(poses.length % 12 === 0 && poses.filter((p) => p.pitch > 0).length === poses.length / 3);
    const n = poses.length, dim = 8, q = new Int8Array(n * dim).map((_, i) => ((i * 37) % 255) - 127), scale = new Float32Array(n).fill(1 / 127);
    const db = new RelocDb({ key: "k", pitches: [0, 40 * DEG, -40 * DEG], stride: 5, dim, n }, Float32Array.from(poses.flatMap((p) => [p.x, p.y, p.z, p.yaw, p.pitch])), q, scale);
    const back = RelocDb.load(db.serialize()), desc = Float32Array.from(q.subarray(7 * dim, 8 * dim), (v) => v / 127);
    assert.deepEqual([back.stride, back.n, Array.from(back.poses)], [5, n, Array.from(db.poses)]);
    assert.ok(Math.abs(back.search(desc, { k: 1 })[0].pitch - poses[7].pitch) < 1e-6 && db.bytes === 8 + 24 * n + n * dim);
    const plain = new RelocDb({ key: "k", dim, n: 1 }, Float32Array.from([1, 2, 3, 0.5]), new Int8Array(dim).fill(1), new Float32Array([1]));
    assert.ok(RelocDb.load(plain.serialize()).search(new Float32Array(dim).fill(1))[0].pitch === undefined && plain.bytes === 8 + 20 + dim);
  },

  // Fix 3 (the browser's sealed runs: the height 0.23-0.27 m off at p95, 4% of the time beyond 2.5 σz, with fixes whose
  // own heights were 1-4 cm off): the flow's climb rate, read at about half the climb with ±0.3 m/s of noise and once +0.8
  // m/s while the drone sank, moved the height between fixes as if it were right.
  "EKF height: the flow's climb rate at half the climb, ±0.3 m/s and a +0.8 m/s burst every 7 s, fixes 4 a second with heights 2 cm off: with the throttle known the height stays within 0.1 m (p95) and 2.5 σz (97%); without it (the flow alone between fixes) within 0.25 m"() {
    const out = {};
    for (const known of [true, false]) {
      const ctl = new FakeCtl(100), loc = new Localizer({ ctl }), r = rng(7), g = gaussOf(rng(77)), dt = 1 / 30;
      Object.assign(ctl, { mask: known ? 4 : 0, out: { thr: 0.45 }, tel: { vbat: 3.9, sticks: { thr: 0 } }, hover: known ? { estimate: () => 0.46 } : undefined }); // the hover throttle 2% off
      loc.setSource("fused");
      ctl.flying = false;
      loc.reset({ x: 1, y: 1, yaw: 0, z: 1 });
      ctl.flying = true;
      let z = 1, vz = 0, gust = 0, noise = 0, tt = 0, next = clock;
      const hist = [], pending = [], errs = [], inside = [];
      for (let i = 0; i < 60 * 30; i++) {
        clock += 1000 * dt;
        tt += dt;
        // the throttle: a hover with slow swells and a climb and a sink a few seconds long; the drone's thrust grows as
        // throttle^1.6 over a hover throttle of 0.45, drag 1.4/s, gusts (0.2 m/s², 1.2 s)
        const thr = 0.45 * (1 + 0.03 * Math.sin(tt / 1.3) + (tt % 20 > 8 && tt % 20 < 10 ? 0.03 : tt % 20 > 14 && tt % 20 < 16 ? -0.03 : 0));
        gust += (-gust / 1.2) * dt + 0.2 * Math.sqrt((2 * dt) / 1.2) * g();
        vz += (9.81 * ((thr / 0.45) ** 1.6 - 1) - 1.4 * vz + gust) * dt;
        z += vz * dt;
        hist.push({ t: clock, z });
        noise = 0.6 * noise + 0.8 * 0.3 * g();
        ctl.out.thr = thr;
        Object.assign(ctl.est, { vx: 0, vy: 0, vz: 0.5 * vz + noise + (tt % 7 < 1 ? 0.8 : 0), heading: 0 });
        ctl.perception.latest.flow = { t: clock };
        ctl.emit("tick", { dt, now: clock });
        if (clock >= next) (next += 250), pending.push({ at: clock + 250, fix: { x: 1, y: 1, z: z + 0.02 * g(), yaw: 0, sigma: 0.05, yawSigma: 0.03, t: clock, source: "splat", inliers: 500 } });
        while (pending.length && pending[0].at <= clock) loc.fix(pending.shift().fix);
        if (tt < 3) continue;
        const p = loc.pose(), e = Math.abs(p.z - z);
        errs.push(e);
        inside.push(e <= 2.5 * p.zSigma);
      }
      out[known ? "throttle" : "flow"] = { p95: q(errs, 0.95), med: q(errs, 0.5), within: inside.filter(Boolean).length / inside.length };
    }
    console.log(`      ${Object.entries(out).map(([k, v]) => `${k === "throttle" ? "the throttle known" : "the flow alone"}: height error ${fmt(v.med)} / ${fmt(v.p95)} m, ${(100 * v.within).toFixed(1)}% within 2.5 σz`).join("; ")}`); // C2 fix 2: 0.11 / 0.39 m, 87% (the throttle not used)
    assert.ok(out.throttle.p95 < 0.1 && out.throttle.within >= 0.97, JSON.stringify(out.throttle));
    assert.ok(out.flow.p95 < 0.25 && out.flow.within >= 0.97, JSON.stringify(out.flow));
  },

  async "EKF with splat fixes at 4 Hz (4 cm noise, 250 ms late, 5% outliers, a 0.1 m/s climb-rate bias): sigma < 0.1 m, outliers gated, height follows"() {
    const { loc, run } = flightRig();
    const out = await run({ seconds: 80, rate: 4 });
    console.log(`      error ${fmt(out.err.med)} / ${fmt(out.err.p95)} / ${fmt(out.err.max)} m, height ${fmt(out.zerr.med)} / ${fmt(out.zerr.p95)} m, yaw ${fmt(out.yaw.med, 2)} deg; sigma ${fmt(out.sigma.med)} / ${fmt(out.sigma.p95)} m; ${JSON.stringify(out.status)}; outliers gated ${out.outGated}/${out.outliers}, good fixes used ${out.goodUsed}/${out.good}; HUD ${JSON.stringify(out.hud)}`);
    assert.ok(out.err.med < 0.06 && out.err.p95 < 0.15, "horizontal error");
    assert.ok(out.zerr.med < 0.06 && out.zerr.p95 < 0.15, "height error");
    assert.ok(out.yaw.med < 1.5, "yaw error");
    assert.ok(out.sigma.med < 0.1 && out.sigma.p95 < 0.1, "sigma");
    assert.deepEqual(Object.keys(out.status), ["ok"]);
    assert.ok(out.outGated >= 0.9 * out.outliers && out.goodUsed >= 0.95 * out.good, "gate");
    assert.ok(out.hud.rate >= 3 && out.hud.rate <= 5 && out.hud.age < 600 && out.hud.source === "splat" && out.hud.inliers === 500);
    assert.equal(loc.pose().status, "ok");
  },

  async "EKF: fixes at 3 Hz and at 5 Hz keep sigma < 0.1 m; without fixes it gets lost, a relocalization brings it back; the pad still resets"() {
    for (const rate of [3, 5]) {
      const { run } = flightRig({ seed: rate });
      const out = await run({ seconds: 60, rate });
      console.log(`      ${rate} Hz: error ${fmt(out.err.med)} / ${fmt(out.err.p95)} m, sigma ${fmt(out.sigma.med)} / ${fmt(out.sigma.p95)} m, ${JSON.stringify(out.status)}`);
      assert.ok(out.sigma.p95 < 0.1 && out.err.p95 < 0.15 && out.err.p95 < out.sigma.p95 && !out.status.lost, `${rate} Hz`);
    }
    const { loc, run, truth, hist } = flightRig({ seed: 9 });
    await run({ seconds: 20, rate: 4 });
    const lost = await run({ seconds: 30, rate: 0, flowBias: 0.15 });
    console.log(`      no fixes for 30 s: sigma ${fmt(lost.sigma.max)} m, status ${loc.pose().status}, error ${fmt(lost.err.max)} m`);
    assert.equal(loc.pose().status, "lost");
    const p = truth(), before = loc.pose();
    assert.ok(loc.fix({ x: p.x, y: p.y, z: p.z, yaw: p.yaw, sigma: 0.05, yawSigma: 1 * DEG, t: clock, source: "splat", reloc: true }), "a relocalization is taken");
    const after = loc.pose();
    console.log(`      relocalized: ${fmt(Math.hypot(before.x - p.x, before.y - p.y))} m -> ${fmt(Math.hypot(after.x - p.x, after.y - p.y))} m, sigma ${fmt(after.sigma)}, ${after.status}`);
    assert.ok(Math.hypot(after.x - p.x, after.y - p.y) < 0.1 && after.status === "ok");
    const again = await run({ seconds: 15, rate: 4 });
    assert.ok(again.err.p95 < 0.15 && !again.status.lost, "tracks again");
    // forgotten, then a relocalization 350 ms late while turning: the pose is where the drone is now, not where it was
    await run({ seconds: 2, rate: 0 });
    const was = hist.find((h) => h.t >= clock - 350), turned = Math.abs(wrapAngle(truth().yaw - was.yaw)) / DEG;
    loc.forget(); // just after that frame was taken
    assert.ok(loc.fix({ x: was.x, y: was.y, z: was.z, yaw: was.yaw, sigma: 0.03, yawSigma: 1 * DEG, t: was.t, source: "splat", reloc: true }));
    const now = loc.pose(), tr = truth();
    console.log(`      forgotten, relocalized from a frame 350 ms old (turned ${fmt(turned, 1)} deg since): ${fmt(Math.hypot(now.x - tr.x, now.y - tr.y))} m, ${fmt(Math.abs(wrapAngle(now.yaw - tr.yaw)) / DEG, 2)} deg from the drone now`);
    assert.ok(Math.hypot(now.x - tr.x, now.y - tr.y) < 0.08 && Math.abs(wrapAngle(now.yaw - tr.yaw)) < 1.5 * DEG && now.status === "ok");
    // a fix far off while tracking is gated, and so are three more like it (see the conflict test)
    const t = truth();
    assert.ok(loc.fix({ ...t, sigma: 0.04, t: clock, source: "splat" }));
    const off = { x: t.x + 1.2, y: t.y, z: t.z, yaw: t.yaw, sigma: 0.04, source: "splat" };
    for (let k = 0; k < 4; k++) assert.equal(loc.fix({ ...off, t: clock }), false, `off fix ${k + 1}`);
    // the home pad
    loc.ctl.flying = false;
    loc.reset({ x: 0.5, y: 0.5, yaw: 1 });
    assert.deepEqual([loc.pose().status, +loc.pose().sigma.toFixed(2)], ["ok", 0.05]);
  },

  "EKF conflict: repeated wrong solves never win (hovering, or one spot while the drone moves); fixes that agree with each other and with odometry over 0.3 m re-anchor only on a confirming solve"() {
    const fly = conflictRig();
    // hovering, tracking: then the same view solved 0.8 m off, 12 times at 5 Hz
    fly.go({ s: 3, v: 0, fixes: 5 });
    const t0 = fly.truth(), off = (p) => ({ x: p.x + 0.8, y: p.y, z: 1, yaw: p.yaw, sigma: 0.04, yawSigma: 0.01, source: "splat" });
    const used = fly.go({ s: 2.4, v: 0, fixes: 5, fix: off });
    let p = fly.loc.pose();
    console.log(`      hovering, 12 solves 0.8 m off: used ${used.filter(Boolean).length}; pose ${fmt(Math.hypot(p.x - t0.x, p.y - t0.y))} m from the truth, sigma ${fmt(p.sigma)} (${p.status}), conflict ${JSON.stringify({ n: p.conflict?.n, apart: fmt(p.conflict?.apart), ready: p.conflict?.ready })}`);
    assert.ok(!used.some(Boolean) && Math.hypot(p.x - t0.x, p.y - t0.y) < 0.05, "never adopted");
    assert.ok(p.conflict && p.conflict.n >= 3 && !p.conflict.ready && Math.abs(p.conflict.apart - 0.8) < 0.05, "a second hypothesis, not ready: no motion");
    assert.ok(p.sigma > 0.4 && p.status === "degraded", "sigma covers both");
    assert.equal(fly.loc.fix({ ...off(fly.truth()), t: clock, confirm: true }), false, "a confirmation can't settle it without motion");
    // good fixes again: the conflict goes
    fly.go({ s: 0.6, v: 0, fixes: 5 });
    assert.ok(!fly.loc.pose().conflict && fly.loc.pose().status === "ok");
    // flying: the solves stay at one spot 0.8 m to the side (the view's wrong answer) while odometry says the drone moved 1 m
    const t1 = fly.truth(), spot = { ...off(t1), x: t1.x, y: t1.y + 0.8 }, u2 = fly.go({ s: 2.4, v: 0.45, fixes: 5, fix: () => spot });
    p = fly.loc.pose();
    assert.ok(!u2.some(Boolean) && !p.conflict?.ready, `one spot while moving: ${JSON.stringify(p.conflict)}`);
    assert.equal(fly.loc.fix({ ...spot, t: clock, confirm: true }), false);
    fly.go({ s: 0.6, v: 0.45, fixes: 5 });
    // the filter has really drifted 0.6 m (its odometry was wrong): fixes agree with each other and with the motion
    fly.shift(0.6);
    const u3 = fly.go({ s: 2, v: 0.45, yawRate: 0.3, fixes: 5, fix: (q) => ({ x: q.x, y: q.y, z: 1, yaw: q.yaw, sigma: 0.04, yawSigma: 0.01, source: "splat" }) });
    p = fly.loc.pose();
    const tr = fly.truth();
    console.log(`      the filter 0.6 m off, consistent solves for 2 s: used ${u3.filter(Boolean).length}, conflict ${JSON.stringify({ n: p.conflict?.n, apart: fmt(p.conflict?.apart), ready: p.conflict?.ready })}, status ${p.status}`);
    assert.ok(!u3.some(Boolean) && p.conflict?.ready && Math.hypot(p.conflict.x - tr.x, p.conflict.y - tr.y) < 0.1, "ready: agreeing fixes over 1 s and 0.3 m");
    assert.equal(fly.loc.fix({ x: tr.x, y: tr.y, z: 1, yaw: tr.yaw, sigma: 0.04, yawSigma: 0.01, t: clock, source: "splat" }), false, "an ordinary fix still can't");
    assert.equal(fly.loc.fix({ x: tr.x + 0.5, y: tr.y - 0.4, z: 1, yaw: tr.yaw, sigma: 0.04, yawSigma: 0.01, t: clock, source: "splat", confirm: true }), false, "a confirmation elsewhere doesn't");
    const was = fly.hist.find((h) => h.t >= clock - 150); // a solve of a frame 150 ms old, from a render 0.5 m away
    assert.equal(fly.loc.fix({ x: was.x + 0.01, y: was.y, z: 1, yaw: was.yaw, sigma: 0.04, yawSigma: 0.01, t: was.t, source: "splat", confirm: true }), true, "a confirming solve re-anchors");
    p = fly.loc.pose();
    console.log(`      confirmed: ${fmt(Math.hypot(p.x - tr.x, p.y - tr.y))} m from the truth, sigma ${fmt(p.sigma)} (${p.status})`);
    assert.ok(Math.hypot(p.x - tr.x, p.y - tr.y) < 0.08 && p.status === "ok" && !p.conflict && fly.loc.fixes.anchored === 1, `re-anchored: ${JSON.stringify(p)}`);
  },

  "EKF history: a relocalization 4 s late after forget() is carried to now (sigma grown); one older than the 8 s history is refused"() {
    const fly = conflictRig();
    fly.go({ s: 2, v: 0.4, fixes: 5 });
    fly.loc.forget();
    fly.go({ s: 4.2, v: 0.4, yawRate: 0.25 });
    const was = fly.hist.find((h) => h.t >= clock - 4000);
    assert.ok(fly.loc.fix({ x: was.x, y: was.y, z: 1, yaw: was.yaw, sigma: 0.05, yawSigma: 0.02, t: was.t, source: "splat", reloc: true }));
    let p = fly.loc.pose(), tr = fly.truth();
    console.log(`      4 s late: ${fmt(Math.hypot(p.x - tr.x, p.y - tr.y))} m from the drone now, sigma ${fmt(p.sigma)} (${p.status})`);
    assert.ok(Math.hypot(p.x - tr.x, p.y - tr.y) < 0.25 && p.sigma > 0.1, "carried, not taken as now");
    fly.loc.forget();
    fly.go({ s: 9, v: 0.4, yawRate: 0.25 });
    const old = fly.hist.find((h) => h.t >= clock - 8500);
    assert.equal(fly.loc.fix({ x: old.x, y: old.y, z: 1, yaw: old.yaw, sigma: 0.05, yawSigma: 0.02, t: old.t, source: "splat", reloc: true }), false);
    assert.deepEqual([fly.loc.fixes.last.why, fly.loc.pose().status], ["stale", "lost"]);
  },

  async "SplatLocalizer: a good solve becomes a splat fix (capture time, sigma from the covariance); weak, implausible and far ones are refined or rejected; 5 Hz, one at a time, relocalize when lost"() {
    const { sl, loc, fake, truth, setTruth, ctl, twin } = splatRig();
    setTruth({ x: 2, y: 3, z: 1, yaw: 0.4 });
    loc.reset({ x: 2.03, y: 2.98, yaw: 0.41, z: 1 });
    const t = clock - 120;
    let r = await sl.track(frame(), t);
    assert.ok(r.ok, r.reason);
    const f = loc.fixes.last;
    assert.deepEqual([f.source, f.t, f.inliers, f.used], ["splat", t, 500, true]);
    assert.ok(f.sigma >= SPLAT.sigmaFloor && f.sigma < 0.1 && r.yawSigma < 2 * DEG, `sigma ${f.sigma} ${r.yawSigma}`);
    assert.equal(fake.solves, 1);
    // few inliers: next to the prior a weak fix (sigma doubled); fewer than weakInliers, or away from the prior, rejected
    fake.inliers = 80;
    r = await sl.track(frame(), clock);
    assert.ok(r.ok && r.weak && r.sigma >= 2 * SPLAT.sigmaFloor && loc.fixes.last.used, `weak: ${r.reason}`);
    fake.inliers = 50;
    r = await sl.track(frame(), clock);
    assert.deepEqual([r.ok, r.reason], [false, "50 inliers"]);
    fake.inliers = 80;
    setTruth({ x: 2.3, y: 3, z: 1, yaw: 0.4 });
    r = await sl.track(frame(), clock);
    assert.deepEqual([r.ok, r.reason], [false, "80 inliers"]);
    assert.equal(sl.stats.rejects, 2);
    // the answer is 0.3 m from where it rendered (the localizer was unsure): rendered again there and solved again
    fake.inliers = 500;
    fake.solves = 0;
    loc.reset({ x: 2, y: 3, yaw: 0.4, z: 1, sigma: 0.3 });
    setTruth({ x: 2.3, y: 3, z: 1, yaw: 0.4 });
    r = await sl.track(frame(), clock);
    assert.ok(r.ok && r.refined && fake.solves === 2, `refined ${r.refined}, ${fake.solves} solves`);
    // under the floor
    setTruth({ x: 2.3, y: 3, z: -0.5, yaw: 0.4 });
    r = await sl.track(frame(), clock);
    assert.match(r.reason, /floor/);
    setTruth({ x: 2.3, y: 3, z: 1, yaw: 0.4 });
    // scheduling from perception frames: at most `rate` a second, never two at once, at the frame's capture time
    const perception = new Emitter();
    sl.attach(perception);
    const started = [];
    const track = sl.track.bind(sl);
    sl.track = (img, tt) => (started.push({ tt, at: clock }), track(img, tt));
    const el = { width: 640, height: 480 }, frames = async (n, extra = {}) => {
      for (let k = 0; k < n; k++) {
        clock += 25;
        perception.emit("frame", { t: clock, element: el, region: { sx: 0, sy: 0, sw: 640, sh: 480 }, captured: clock - 90, detections: [{ label: "person", box: { x: 0.1, y: 0.2, w: 0.1, h: 0.3 } }, { label: "chair", box: { x: 0, y: 0, w: 1, h: 1 } }], ...extra });
        await new Promise((res) => setImmediate(res));
      }
    };
    sl.o.slowSigma = 0;
    await frames(40);
    let gaps = started.slice(1).map((s, i) => s.at - started[i].at);
    assert.ok(started.length >= 4 && started.length <= 6 && Math.min(...gaps) >= 200, `${started.length} tracks in 1 s, gaps ${gaps}`);
    assert.ok(started.every((s) => s.at - s.tt === 90), "capture time");
    assert.deepEqual(fake.lastBoxes, [{ x: 0.1, y: 0.2, w: 0.1, h: 0.3 }], "people and pets masked, not chairs");
    sl.o.slowSigma = SPLAT.slowSigma; // sigma under 5 cm: 3 Hz
    started.length = 0;
    await frames(40);
    gaps = started.slice(1).map((s, i) => s.at - started[i].at);
    assert.ok(loc.pose().sigma < SPLAT.slowSigma && started.length >= 2 && started.length <= 4 && Math.min(...gaps) >= 1000 / SPLAT.slowRate - 1, `slow: ${started.length} tracks, gaps ${gaps}`);
    // a frame decoded more than staleMs before it was processed is skipped (its time would be a guess)
    started.length = 0;
    await frames(20, { captured: undefined, decoded: undefined });
    const n0 = started.length;
    await frames(20, { captured: undefined, decoded: clock - 400 });
    assert.ok(n0 >= 1 && started.length === n0, `stale frames: ${n0} then ${started.length}`);
    // lost: relocalization instead of tracking, at most every relocEvery
    sl.track = track;
    loc.forget();
    sl.db = { search: () => [{ x: 2.2, y: 3.1, z: 1, yaw: 0.3, score: 0.9 }, { x: 0, y: 0, z: 1, yaw: 0, score: 0.8 }], meta: {} };
    let relocs = 0;
    const reloc = sl.relocalize.bind(sl);
    sl.relocalize = (...a) => (relocs++, reloc(...a));
    for (let k = 0; k < 20; k++) {
      clock += 100;
      perception.emit("frame", { t: clock, element: el, region: { sx: 0, sy: 0, sw: 640, sh: 480 }, captured: clock - 90 });
      await new Promise((res) => setImmediate(res));
    }
    assert.ok(relocs >= 1 && relocs <= 2, `${relocs} relocalizations in 2 s`);
    const p = loc.pose();
    assert.ok(p.status === "ok" && Math.hypot(p.x - 2.3, p.y - 3) < 0.05, `relocalized to ${fmt(p.x)}, ${fmt(p.y)} (${p.status})`);
    assert.equal(sl.stats.relocs, 1);
    // a keyframe 0.3 m off gives a weak solve: solved again where it puts the drone; keyframes render at the drone's attitude
    sl.detach();
    loc.forget();
    ctl.est.pitchAngle = 0.12;
    Object.assign(fake, { queue: [60, 500], solves: 0 });
    twin.poses.length = 0;
    r = await sl.relocalize(frame(), clock - 90);
    assert.ok(r.ok && r.rank === 1 && fake.solves === 2 && loc.pose().status === "ok", `rank ${r.rank}, ${fake.solves} solves, ${r.reason ?? ""}`);
    assert.ok(Math.abs(twin.poses[0].pitch - 0.12) < 1e-9 && Math.hypot(twin.poses[0].x - 2.2, twin.poses[0].y - 3.1) < 1e-9, "the keyframe rendered at the drone's attitude");
    Object.assign(fake, { queue: [20], solves: 0 }); // too weak to try again: the next keyframe (2.3 m off, so accept() renders again at the answer)
    loc.forget();
    r = await sl.relocalize(frame(), clock - 90);
    assert.ok(r.ok && r.rank === 2 && r.refined && fake.solves === 3, `rank ${r.rank}, ${fake.solves} solves`);
    ctl.est.pitchAngle = 0;
    // calib.json: its lens and uptilt, and its overlay mask
    const osd = new OsdMask({ width: 16, height: 12 });
    osd.fixed = Uint8Array.from({ length: 192 }, (_, i) => +(i > 20));
    sl.setCalib({ ...L.calibOf(L.resolveLens({ ...L.DRONE_LENS, uptiltDeg: 27 }), 640, 480), osdMask: osd.toJSON() });
    assert.deepEqual([sl.uptilt, sl.view.lens.uptiltDeg, Array.from(sl.osd.mask()).join("")], [27, 27, Array.from(osd.fixed).join("")]);
  },

  async "SplatLocalizer on real video: advisory until the pad check passes, wider sigma until a verified calibration for this framing; the 4:3 picture only; a calib.json for another framing is set aside"() {
    const { sl, loc, fake, ctl, setTruth } = splatRig(), statuses = [];
    sl.on("status", (s) => statuses.push(s.text));
    let pic = { sx: 240, sy: 0, sw: 1440, sh: 1080 };
    const src = { kind: "goggles", ready: () => true, element: () => ({ width: 1920, height: 1080 }), region: () => ({ sx: 0, sy: 0, sw: 1920, sh: 1080 }) };
    const perception = Object.assign(new Emitter(), { source: src, region: () => pic });
    sl.attach(perception);
    sl.useRegion({ element: src.element(), region: pic, picture: true });
    setTruth({ x: 2, y: 3, z: 0, yaw: 0.4 });
    ctl.flying = false;
    loc.reset({ x: 2, y: 3, yaw: 0.4 }); // on the pad
    // not checked yet: shown, not used
    const used0 = loc.fixes.used;
    let r = await sl.track(frame(), clock);
    assert.ok(r.ok && r.advisory && loc.fixes.used === used0 && sl.trust() === "advisory" && sl.stats.advisory === 1, `advisory ${JSON.stringify(r.reason)}`);
    // the framing: the whole 16:9 stream (the picture not found) is refused; the picture is taken
    let tracked = 0;
    const track = sl.track.bind(sl);
    sl.track = (...a) => (tracked++, track(...a));
    perception.emit("frame", { t: clock, element: src.element(), region: { sx: 0, sy: 0, sw: 1920, sh: 1080 }, picture: true, captured: clock - 90 });
    assert.ok(/isn't the camera's 4:3 picture \(it is 1920x1080/.test(sl.framing) && tracked === 0 && statuses.some((t) => /paused: the video isn't the camera's 4:3 picture/.test(t)), sl.framing);
    perception.emit("frame", { t: clock, element: src.element(), region: pic, picture: false, captured: clock - 90 });
    assert.ok(/hasn't been found/.test(sl.framing) && tracked === 0);
    clock += 500;
    perception.emit("frame", { t: clock, element: src.element(), region: pic, picture: true, captured: clock - 90 });
    await new Promise((res) => setImmediate(res));
    assert.ok(sl.framing === "" && tracked === 1, `${sl.framing}, ${tracked}`);
    sl.track = track;
    // the pad check: frames solved at the pad pose land on it -> verified; fixes used, with the uncalibrated floor
    const v = await sl.verifyOnPad({ n: 6 });
    assert.ok(v.ok && v.accepted === 6 && v.medianErr < 0.05 && !v.calibrated && !v.ready && sl.trust() === "verified", v.text);
    assert.match(v.text, /matches the 3D scan on the pad.*wider error bars/);
    r = await sl.track(frame(), clock);
    assert.ok(r.ok && !r.advisory && loc.fixes.last.used && r.sigma >= SPLAT.rawSigmaFloor && r.yawSigma >= SPLAT.rawYawFloor, `raw floors: ${r.sigma}`);
    // the drone 0.3 m off its pad: the check fails, back to advisory
    setTruth({ x: 2.3, y: 3, z: 0, yaw: 0.4 });
    const bad = await sl.verifyOnPad({ n: 6 });
    assert.ok(!bad.ok && bad.medianErr > 0.25 && sl.trust() === "advisory", bad.text);
    assert.match(bad.text, /puts the drone 3[01] cm and [\d.]+° \w+ of its pad mark\. Check that it sits on its mark.*if it does, the camera needs calibrating/);
    setTruth({ x: 2, y: 3, z: 0, yaw: 0.4 });
    // a verified calibration for this framing: normal floors, after a new pad check (the lens changed)
    const calib = { ...L.calibOf(L.droneLens(null, 20), 640, 480), source: "twin", rms: 0.6, region: { ...pic, frame: [1920, 1080], kind: "goggles" }, verified: { err: 0.01, yawErr: 0.3 } };
    sl.setCalib(calib);
    assert.ok(sl.calibOk && sl.trust() === "advisory", "a new lens needs the pad check again");
    assert.ok((await sl.verifyOnPad()).ready === false && sl.trust() === "verified");
    r = await sl.track(frame(), clock);
    assert.ok(r.ok && r.sigma < 0.1 && r.yawSigma < 2 * DEG, `calibrated: ${r.sigma}`);
    // the same calib.json with another framing (the user dragged a crop): default lens, "calibrate again"
    pic = { sx: 260, sy: 10, sw: 1400, sh: 1060 };
    sl.useRegion({ element: src.element(), region: pic, picture: true });
    assert.ok(!sl.calibOk && sl.lens.diagFovDeg === 159 && statuses.some((t) => /Calibrate the camera again: the video framing changed/.test(t)) && sl.trust() === "advisory");
    assert.ok(/4:3 picture \(it is 1400x1060/.test(sl.framing), "1400x1060 is 1% off 4:3");
  },

  async "pad check: with a calibration verified for this framing a drone set down 8 cm and 6 deg off its mark passes and is re-seated on the camera (never tighter than the pad button's 5 cm, 3 deg); without one a lens error can't be told from a misplaced drone, so 4 cm off passes without moving the pose and 10 cm fails naming both; a tilted view fails"() {
    const { sl, loc, fake, ctl, setTruth } = splatRig(), pic = { sx: 240, sy: 0, sw: 1440, sh: 1080 };
    const src = { kind: "goggles", ready: () => true, element: () => ({ width: 1920, height: 1080 }), region: () => pic };
    sl.attach(Object.assign(new Emitter(), { source: src, region: () => pic }));
    sl.useRegion({ element: src.element(), region: pic, picture: true });
    ctl.flying = false;
    const resets = [], reset0 = loc.reset.bind(loc);
    loc.reset = (a) => (resets.push(a), reset0(a));
    const at = (dx, dyaw, bias = null) => { loc.reset({ x: 2, y: 3, yaw: 0.4 }); resets.length = 0; setTruth({ x: 2 + dx, y: 3, z: 0, yaw: 0.4 + dyaw * DEG }); fake.bias = bias; return sl.verifyOnPad({ n: 6 }); };
    fake.noise = 0.003;
    // no calibration: a lens whose field of view is off solves the still drone 10 cm and 0.6 deg away, every frame the same
    assert.ok(!sl.calibOk);
    const lens = await at(0, 0, { x: 0.1, yaw: -0.6 * DEG });
    console.log(`      uncalibrated, solves 10 cm off: "${lens.text}"`);
    assert.ok(!lens.ok && !lens.reseated && resets.length === 0 && sl.trust() === "advisory" && lens.spread < 0.01, JSON.stringify(lens));
    assert.match(lens.text, /puts the drone 10 cm and 0\.6° right of its pad mark\. Check that it sits on its mark facing the home heading; if it does, the camera needs calibrating/);
    const small = await at(0, 0, { x: 0.04, yaw: 2 * DEG });
    assert.ok(small.ok && !small.reseated && resets.length === 0 && Math.abs(loc.pose().x - 2) < 1e-9 && sl.trust() === "verified", `within 5 cm, 3 deg: the hand placement stands (${small.text})`);
    assert.match(small.text, /\(within 4 cm and 2\.\d° of its mark\)\. Vision fixes are on, with wider error bars until the camera is calibrated \(a still drone can't show a lens error\)/);
    assert.ok(!(await at(0.08, 6)).ok, "8 cm and 6 deg off fails without a calibration");
    // a wrong camera angle: the drone looks tilted on its pad
    const tilted = await at(0, 0, { pitch: 4 * DEG });
    assert.ok(!tilted.ok && tilted.tilt >= 3.9 && /sees the drone tilted 4\.\d° on its pad\. Check that it sits flat; if it does, the camera's angle is off: calibrate the camera/.test(tilted.text), tilted.text);
    // a calibration verified for this framing: the camera re-seats the pose
    sl.setCalib({ ...L.calibOf(L.droneLens(null, 20), 640, 480), source: "twin", rms: 0.6, region: { ...pic, frame: [1920, 1080], kind: "goggles" }, verified: { err: 0.01 } });
    assert.ok(sl.calibOk);
    const off = await at(0.08, 6), p = loc.pose();
    console.log(`      calibrated, 8 cm, 6 deg off: "${off.text}" -> pose ${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.yaw / DEG, 1)} deg, sigma ${fmt(resets[0]?.sigma)} m, ${fmt(resets[0]?.yawSigma / DEG, 1)} deg`);
    assert.ok(off.ok && off.reseated && sl.trust() === "verified" && off.spread <= SPLAT.pad.spread, JSON.stringify(off));
    assert.ok(Math.abs(p.x - 2.08) < 0.01 && Math.abs(p.y - 3) < 0.01 && Math.abs(wrapAngle(p.yaw - 0.4 - 6 * DEG)) < 0.5 * DEG && p.status === "ok", "re-seated on the solves");
    assert.ok(resets.length === 1 && resets[0].sigma >= SPLAT.pad.sigma && resets[0].yawSigma >= SPLAT.pad.yawSigma - 1e-9 && p.sigma >= 0.049, `never tighter than the pad button: ${JSON.stringify(resets[0])}`);
    assert.match(off.text, /sits 8 cm and 6\.\d° left of its pad mark, so its position now comes from the camera\. Vision fixes are on\.$/);
    const big = await at(0.14, 0);
    assert.ok(big.ok && resets[0].sigma >= 0.07 - 1e-9, "half the offset at least");
    const far = await at(0.25, 2);
    assert.ok(!far.ok && !far.reseated && sl.trust() === "advisory" && Math.abs(loc.pose().x - 2) < 1e-9, far.text);
    assert.match(far.text, /puts the drone 25 cm and 2\.\d° left of its pad mark, more than 15 cm/);
    const turned = await at(0, -12);
    assert.ok(!turned.ok && /\d+ mm and 12\.\d° right/.test(turned.text), turned.text);
    fake.noise = 0.04; // a picture that doesn't hold still or match: each view solves somewhere else
    const shaky = await at(0, 0);
    assert.ok(!shaky.ok && shaky.spread > SPLAT.pad.spread && /views from the pad disagree with each other \(\d+ cm/.test(shaky.text), shaky.text);
    fake.noise = 0.003;
    const tiny = await at(0.004, 0.1);
    assert.ok(tiny.ok && tiny.reseated && !/pad mark/.test(tiny.text), `a few mm: re-seated without a word (${tiny.text})`);
    fake.bias = null;
  },

  async "no WebGPU means no vision for the real drone (fixes shown only, the pad check refuses and says why); Where am I? answers within its deadline while DINOv2 downloads, its progress one line every 5%"() {
    const { sl, loc, fake, ctl, setTruth } = splatRig(), pic = { sx: 0, sy: 0, sw: 640, sh: 480 }, statuses = [];
    sl.on("status", (s) => statuses.push(s));
    const src = { kind: "goggles", ready: () => true, element: () => ({ width: 640, height: 480 }), region: () => pic };
    sl.attach(Object.assign(new Emitter(), { source: src, region: () => pic }));
    sl.useRegion({ element: src.element(), region: pic, picture: true });
    ctl.flying = false;
    setTruth({ x: 2, y: 3, z: 0, yaw: 0.4 });
    loc.reset({ x: 2, y: 3, yaw: 0.4 });
    fake.info = { backend: "wasm", fallbacks: [], dino: "fake" };
    const v = await sl.verifyOnPad();
    assert.ok(!v.ok && /runs on the CPU here \(this browser has no WebGPU\): too slow for the real drone/.test(v.text) && sl.trust() === "advisory", v.text);
    fake.info = { backend: "webgpu", fallbacks: ["XFeat on WASM (WebGPU: op not supported)"], dino: "fake" };
    assert.match(sl.cpuOnly(), /XFeat on WASM/);
    fake.info = { backend: "webgpu", fallbacks: [], dino: "fake" };
    assert.ok((await sl.verifyOnPad()).ok && sl.trust() === "verified" && sl.cpuOnly() === "");
    fake.info.backend = "wasm"; // a later restart fell back: advisory again
    assert.equal(sl.trust(), "advisory");
    fake.info.backend = "webgpu";
    // DINOv2 not downloaded yet: Where am I? says so within the deadline, the download goes on
    let release;
    Object.assign(fake, { loadDino: () => new Promise((r) => (release = r)) });
    Object.assign(sl, { db: { search: () => [], meta: {} } });
    const w0 = Date.now(), r = await sl.locateNow({ deadline: 1600 });
    assert.ok(!r.ok && r.downloading && /still downloading/.test(r.reason) && Date.now() - w0 < 1200, `${r.reason} after ${Date.now() - w0} ms`);
    release();
    // its download progress: one updating line (key, progress), at most every 5%
    statuses.length = 0;
    for (let b = 0; b <= 100; b++) sl.dinoProgress({ what: "DINOv2-small", loaded: b * 444e3, total: 44.4e6 });
    const lines = statuses.filter((s) => s.key === "vision-model");
    assert.ok(lines.length <= 21 && lines.length >= 20 && lines.every((s) => s.progress >= 0 && s.progress <= 1) && /100%/.test(lines.at(-1).text), `${lines.length} lines`);
    // complete: 100% went out once, however often the worker says so again; a download started over shows from its start
    for (let k = 0; k < 3; k++) sl.dinoProgress({ what: "DINOv2-small", loaded: 44.4e6, total: 44.4e6 });
    assert.equal(statuses.filter((s) => s.key === "vision-model").length, lines.length, "100% once");
    for (const b of [0, 30, 100]) sl.dinoProgress({ what: "DINOv2-small", loaded: b * 444e3, total: 44.4e6 });
    assert.deepEqual(statuses.filter((s) => s.key === "vision-model").slice(lines.length).map((s) => s.progress), [0, 0.3, 1]);
  },

  async "ready to fly: before the battery the models, the position database (followed even when the app started its build) and the 3D map are prepared in one progress line; with the battery in only the picture and the pad check remain, against the O4's 2.5 min on the ground timed from the first sign of power (the radio's battery reading or the video), started again only on a sign the battery went out (a late radio, a radio pause or the goggles asleep isn't one); a stale or failed pad check and a 16:9 picture say what to do"() {
    const { Readiness, O4 } = await import("../app/js/nav/readiness.js");
    const gpu = Object.getOwnPropertyDescriptor(globalThis.navigator, "gpu");
    Object.defineProperty(globalThis.navigator, "gpu", { value: {}, configurable: true });
    try {
      const values = { mode: "real", locVision: true, avoid: true, depthModel: "dav2s" }, settings = { get: (k) => values[k] }, order = [], ctl = new Emitter();
      const splat = Object.assign(new Emitter(), { enabled: true, features: null, db: null, calibOk: false, calib: null, region: null, pad: null, twin: {}, ctl, dbKey: () => "k", cpuOnly: () => "", lensKey: () => "lens A",
        framingOf: (r, known) => SplatLocalizer.prototype.framingOf.call({ calib: splat.calib, o: SPLAT }, r, known),
        ready: async () => (order.push("vision"), (splat.features = { alive: true, info: { backend: "webgpu", fallbacks: [] } })),
        dino: async () => { order.push("dino"); for (const p of [0.25, 0.5, 1]) splat.emit("status", { key: "vision-model", progress: p }); splat.features.info.dinoLoaded = true; },
        // as SplatLocalizer's: one build, shared; its progress goes to the first caller and out as "db" events
        ensureDb: ({ onProgress } = {}) => (splat.dbBuild ??= (async () => {
          order.push("posdb");
          if (splat.hold) await new Promise((r) => (splat.release = r)); // the app's build still runs when prepare() gets there
          for (const done of [2, 5, 10]) (await new Promise((r) => setTimeout(r, 5)), splat.emit("db", { done, total: 10 }), onProgress?.({ done, total: 10 }));
          splat.db = { n: 10, meta: { key: "k" } };
          splat.dbBuild = null;
        })()),
        trust: () => (splat.pad?.ok && splat.pad.key === splat.lensKey() ? "verified" : "advisory") });
      const detector = Object.assign(new Emitter(), { ready: false, load: async () => { order.push("detector"); detector.emit("status", { progress: 0.5 }); detector.ready = true; } });
      const depth = { loaded: false, warm: async (cb) => { order.push("depth"); cb(50, 100); depth.loaded = true; } };
      let has3D = false, pic = { sx: 240, sy: 0, sw: 1440, sh: 1080 }, state = "found";
      const src = { kind: "goggles", ready: () => true, element: () => ({}), baseRegion: () => ({ sx: 0, sy: 0, sw: 1920, sh: 1080 }), region: () => pic };
      const perception = Object.assign(new Emitter(), { detector, source: null, pictureState: () => state, region: () => pic });
      const ready = new Readiness({ splat, depth, perception, settings, has3D: () => has3D, prepare3D: async ({ onProgress }) => (order.push("map3d"), onProgress({ done: 1, total: 2 }), (has3D = true)) });
      ready.attach(perception);
      const ids = (items) => Object.fromEntries(items.map((i) => [i.id, i.ok]));
      let items = ready.items();
      assert.deepEqual(ids(items), { models: false, webgpu: true, posdb: false, calib: false, picture: false, padcheck: false });
      assert.ok(items.filter((i) => !i.ok).every((i) => i.level === "block") && items.find((i) => i.id === "models").fix.action === "prepare" && items.find((i) => i.id === "picture").phase === "battery");
      const lines = [];
      splat.hold = true;
      splat.ensureDb({ onProgress: () => {} }); // the app's own build, started first (session.sync): its progress goes to the app
      const offGate = ready.on("change", () => ready.running?.step === "posdb" && splat.release?.());
      const r = await ready.run("prepare", { onProgress: (p) => lines.push(p) });
      offGate();
      assert.ok(r.ok, JSON.stringify(r.failed));
      assert.deepEqual(order, ["posdb", "vision", "dino", "depth", "detector", "map3d"]);
      assert.ok(lines.every((l, i) => !i || l.progress >= lines[i - 1].progress) && lines.at(-1).progress === 1 && /Ready for the battery/.test(lines.at(-1).text), lines.map((l) => l.progress).join(" "));
      assert.ok(lines.some((l) => /depth model.* 50%/.test(l.text)), "the depth download's percent");
      assert.ok(["2", "5", "10"].every((d) => lines.some((l) => l.text.includes(`position database (${d} of 10 views)`))), `the app's build followed: ${lines.map((l) => l.text).join(" | ")}`);
      // the database built by prepare() itself: its lines too
      Object.assign(splat, { db: null, hold: false });
      lines.length = 0;
      await ready.prepare({ onProgress: (p) => lines.push(p) });
      assert.ok(lines.some((l) => /position database \(5 of 10 views\)/.test(l.text)) && lines.some((l) => /position database \(10 of 10 views\)/.test(l.text)), lines.map((l) => l.text).join(" | "));
      items = ready.items();
      assert.deepEqual(ids(items), { models: true, webgpu: true, posdb: true, calib: false, picture: false, padcheck: false });
      // a failing part: the rest still runs, the failure is said
      const ensure = splat.ensureDb;
      splat.ensureDb = async () => { throw new Error("the 3D scan isn't loaded"); };
      splat.db = null;
      const r2 = await ready.prepare();
      assert.ok(!r2.ok && /isn't loaded/.test(r2.failed.posdb));
      splat.ensureDb = ensure;
      splat.db = { n: 10, meta: { key: "k" } };
      // battery in: video appears; the picture is found, the pad check passes; the O4's clock runs (from the video: about)
      Object.assign(splat, { calibOk: true, calib: { fx: 300, rms: 0.6, verified: { from: "recording x" } } });
      perception.source = src;
      for (let k = 0; k < 10; k++) (clock += 33), perception.emit("frame", {});
      items = ready.items();
      assert.deepEqual(ids(items), { models: true, webgpu: true, posdb: true, calib: true, picture: true, padcheck: false, standby: true });
      assert.match(items.find((i) => i.id === "picture").text, /1440x1080, black bars taken off/);
      splat.pad = { ok: true, n: 6, key: "lens A", text: "The camera view matches the 3D scan on the pad." };
      for (let k = 0; k < 80; k++) (clock += 1000), perception.emit("frame", {});
      clock += 20000; // goggles off for 20 s (they sleep), then on again: the same battery
      for (let k = 0; k < 2; k++) perception.emit("frame", {});
      items = ready.items();
      const sb = items.find((i) => i.id === "standby");
      assert.ok(items.filter((i) => i.id !== "standby").every((i) => i.ok) && !sb.ok && sb.level === "warn" && /Battery in for about 1:40 \(timed from the video\): about 0:49 before the O4 overheats/.test(sb.text), sb.text);
      clock += O4.gap + 10;
      perception.emit("frame", {}); // a minute with no sign of power (no video, no radio): a new battery, timed from the video
      assert.ok(ready.standby().seconds < 0.1 && ready.standby().approx && ready.standby().by === "video");
      const tel = (vbat, lq = 100, source) => ctl.emit("telemetry", { vbat, lq, source }), v0 = clock;
      const since = (t0, why) => assert.ok(Math.abs(ready.standby().seconds - (clock - t0) / 1000) < 0.01, `${why}: ${ready.standby()?.seconds} s, not ${(clock - t0) / 1000}`);
      const video = (s, also) => { for (let k = 0; k < s * 10; k++) (clock += 100), perception.emit("frame", {}), also?.(k); };
      // the radio connected 100 s after the video started: it keeps the video's start
      video(100);
      tel(4.18);
      since(v0, "a late radio");
      assert.match(ready.items().find((i) => i.id === "standby").text, /^Battery in for about 1:40 \(timed from the video\): about 0:50 before/);
      // the radio pauses 4 s (its USB re-plugged, the bridge restarted) while the video runs: the same battery
      video(4);
      tel(4.18);
      since(v0, "a radio pause with the video running");
      // the radio says "no drone" for 5 s while the video runs (a link glitch): the same battery
      video(5, () => tel(0, 0));
      tel(4.18);
      since(v0, "the radio's \"no drone\" with the video running");
      // the goggles asleep 20 s and the radio silent 10 s of it: no sign either way, the same battery
      clock += 20000;
      tel(4.18);
      since(v0, "the goggles asleep and the radio re-plugged");
      // the radio says "no drone" for 4 s with no video: the battery is out, no clock; then the radio's reading times it
      for (let k = 0; k < 40; k++) (clock += 100), tel(0, 0);
      assert.equal(ready.standby(), null, "battery out");
      assert.ok(!ready.items().some((i) => i.id === "standby"));
      for (let k = 0; k < 20; k++) (clock += 100), tel(4.2, 100, "sim");
      assert.equal(ready.standby(), null, "the simulator's telemetry is no sign of the real battery");
      tel(4.31);
      const t0 = clock;
      for (let k = 0; k < 60; k++) (clock += 1000), tel(4.3 - k * 0.001);
      assert.ok(Math.abs(ready.standby().seconds - 60) < 0.01 && !ready.standby().approx && ready.standby().by === "radio");
      assert.match(ready.items().find((i) => i.id === "standby").text, /^Battery in for 1:00: about 1:30 before/);
      clock += 3000; // a reading every second meanwhile, no video
      tel(4.25);
      assert.ok(Math.abs(ready.standby(clock).seconds - (clock - t0) / 1000) < 0.01, "goggles off: still the same battery");
      tel(0, 0); // the drone unpowered: link quality 0
      clock += O4.unplugged + 500;
      tel(4.35);
      assert.ok(ready.standby().seconds < 0.1, "a battery swap restarts it");
      ctl.isFlying = () => true;
      for (let k = 0; k < 30; k++) (clock += 1000), tel(3.55); // flying: the pack sags
      ctl.isFlying = () => false;
      clock += 500;
      tel(3.85); // landed: it recovers at once, the same battery
      assert.ok(ready.standby().seconds > 30, "a landing isn't a battery swap");
      clock += 15000;
      const t1 = clock;
      video(5, (k) => k === 10 && tel(3.86)); // the goggles on again: a reading with the video running, then a jump
      tel(4.33);
      since(t1 - 45500, "a voltage jump with the video running throughout isn't a swap");
      clock += 3000;
      tel(4.5); // readings that never stopped (the radio holds the last), no video: then a fresh pack's voltage
      assert.ok(ready.standby().seconds < 0.1, "a fresh pack restarts it");
      // the pad check: passed for another lens or picture; failed for this one; on the CPU
      splat.lensKey = () => "lens B";
      let pc = ready.items().find((i) => i.id === "padcheck");
      assert.ok(!pc.ok && /was for another camera setup.*press the pad button again/.test(pc.text) && pc.fix.action === "pad", pc.text);
      splat.pad = { ok: false, n: 6, key: "lens B", text: "The camera puts the drone 10 cm and 0.6° right of its pad mark. Check that it sits on its mark." };
      pc = ready.items().find((i) => i.id === "padcheck");
      assert.equal(pc.text, splat.pad.text);
      splat.cpuOnly = () => "vision localization runs on the CPU here (this browser has no WebGPU)";
      assert.match(ready.items().find((i) => i.id === "padcheck").text, /^The pad check can't pass here: vision localization runs on the CPU/);
      splat.cpuOnly = () => "";
      // the O4 at 16:9 (no bars: "whole"): the picture is the video, but vision needs the 4:3 picture
      [pic, state] = [{ sx: 0, sy: 0, sw: 1920, sh: 1080 }, "whole"];
      const pi = ready.items().find((i) => i.id === "picture");
      assert.ok(!pi.ok && pi.level === "block" && /^Vision is paused: the video isn't the camera's 4:3 picture \(it is 1920x1080\): set the O4's camera to 4:3 in the goggles/.test(pi.text), pi.text);
      values.mode = "sim";
      assert.deepEqual(ready.items(), [], "the simulator on its truth: nothing to check");
    } finally {
      if (gpu) Object.defineProperty(globalThis.navigator, "gpu", gpu); else delete globalThis.navigator.gpu;
    }
  },

  async "recording checks (Settings -> Recordings): what stops a recording being checked, and a replay's numbers in plain words (the pad is a real flight's truth)"() {
    const { recordingProblems, judgeReplay, REPLAY, regionOf } = await import("../app/js/nav/replay.js");
    // the picture's place: a recording started while the goggles' picture was still looked for stores the whole stream
    const P43 = { sx: 240, sy: 0, sw: 1440, sh: 1080 }, WHOLE = { sx: 0, sy: 0, sw: 1920, sh: 1080 }, crop = { sx: 100, sy: 40, sw: 1500, sh: 1000 };
    assert.deepEqual(regionOf({ meta: { lens: { region: WHOLE } }, commands: [{ tool: "picture", args: WHOLE }, { tool: "picture", args: P43 }, { tool: "pad", args: {} }] }), P43);
    assert.deepEqual(regionOf({ meta: { lens: { region: P43 } }, commands: [] }), P43);
    assert.equal(regionOf({ meta: { lens: { region: WHOLE } }, commands: [{ tool: "picture", args: WHOLE }] }), null, "16:9 with no picture: can't tell");
    assert.deepEqual(regionOf({ meta: { lens: { region: crop, calib: { region: crop } } }, commands: [] }), crop, "the framing its calibration was made for");
    const tel = [{ t: 1, est: { heading: 0 }, fm: "ANGLE*", sticks: { thr: 0 } }, { t: 2, est: { heading: 0.1 }, fm: "ANGLE", sticks: { thr: 0.5 } }];
    const good = { meta: { house: "h", lens: { region: { sx: 240, sy: 0, sw: 1440, sh: 1080 } }, app: { mode: "real" } }, index: [{ t: 1, off: 0, bytes: 10 }], telemetry: tel, commands: [{ tool: "pad", args: { x: 0, y: 0, yaw: 0 } }] };
    assert.deepEqual(recordingProblems(good, 10), []);
    const bad = recordingProblems({ meta: { app: { mode: "real" } }, index: [{ t: 1, off: 0, bytes: 10 }], telemetry: tel.map(({ est, ...s }) => s), commands: [] }, 5);
    assert.ok(bad.length === 6 && bad.every((p) => !/undefined/.test(p)), bad.join(" | "));
    assert.match(bad.join(" | "), /shorter than its index.*no heading.*wasn't flying a house.*press the pad button.*picture's place in the video/);
    const out = { truth: false, units: 1800, decoded: 1800, seconds: 63, flyingSeconds: 60, region: { sx: 240, sy: 0, sw: 1440, sh: 1080 }, start: "pad command", foundAfter: 0.4, fixRate: 4.2, inliers: { median: 600 }, msPerFix: { median: 70.4 },
      status: { ok: 200 }, endVsPad: 0.04, groundFixVsPad: 0.01, conflicts: 1, anchored: 1, delay: { offset: -60, videoDelay: 160 }, videoDelay: 100, error: {}, vsRecorded: {} };
    const r = judgeReplay(out);
    console.log(`      "${r.title}" | ${r.lines.map((l) => `${l.ok === false ? "x" : l.ok ? "ok" : "-"} ${l.text}`).join(" | ")}`);
    assert.ok(!r.ok && r.verdict === "fail" && /Video delay to 160 ms/.test(r.title), r.title);
    assert.ok(r.lines.find((l) => l.id === "closure").ok && /4 cm from the pad/.test(r.lines.find((l) => l.id === "closure").text) && r.lines.find((l) => l.id === "conflicts").ok === null);
    const fine = judgeReplay({ ...out, delay: { offset: 10, videoDelay: 90 } });
    assert.ok(fine.ok && fine.verdict === "pass" && /all through this flight/.test(fine.title) && /70 ms/.test(fine.lines.find((l) => l.id === "fixes").text));
    const off = judgeReplay({ ...out, delay: null, endVsPad: 0.3, fixRate: REPLAY.fixRate - 1, status: { ok: 150, lost: 20 } });
    assert.deepEqual(off.lines.filter((l) => l.ok === false).map((l) => l.id), ["fixes", "lost", "closure"]);
  },

  async "SplatLocalizer failures: a relocalization slower than the position history isn't 'found' (a fresh frame next); obstacles; a conflict is confirmed from a render 0.5 m away; Where am I? never waits for a database build; worker failures back off, timeouts restart it"() {
    const { sl, loc, fake, ctl, twin, setTruth } = splatRig(), statuses = [];
    sl.on("status", (s) => statuses.push(s));
    setTruth({ x: 2.3, y: 3, z: 1, yaw: 0.4 });
    // the localizer refuses a relocalization older than its history (8 s of controller ticks)
    for (let k = 0; k < 300; k++) (clock += 33), ctl.emit("tick", { dt: 0.033, now: clock });
    loc.forget();
    sl.db = { search: () => [{ x: 2.2, y: 3.1, z: 1, yaw: 0.3, score: 0.9 }], meta: {} };
    sl.o.relocBudget = Infinity;
    let r = await sl.relocalize(frame(), clock - 9000);
    assert.ok(!r.ok && r.stale && sl.stats.relocs === 0 && /longer than the position history/.test(r.reason) && loc.pose().status === "lost", JSON.stringify(r.reason));
    assert.ok(!statuses.some((s) => /Found where/.test(s.text)), "not announced as found");
    // ...and gives up by itself once the frame is relocBudget old (8 candidates at 0.9 s each)
    sl.o.relocBudget = SPLAT.relocBudget;
    sl.db = { search: () => Array.from({ length: 8 }, (_, k) => ({ x: k * 0.4, y: 0.5, z: 1, yaw: 0, score: 0.9 - k / 10 })), meta: {} };
    const solve = fake.solve;
    fake.solve = async (a) => ((clock += 900), { ...(await solve(a)), inliers: 30 });
    r = await sl.relocalize(frame(), clock);
    fake.solve = solve;
    assert.ok(!r.ok && r.stale && r.tried < 8 && sl.stats.relocs === 0, `${r.tried} tried: ${r.reason}`);
    // a perception-driven relocalization that went stale tries again on the next frame, not 2 s later
    const perception = new Emitter(), el = { width: 640, height: 480 };
    sl.attach(perception);
    let relocs = 0;
    const reloc = sl.relocalize.bind(sl);
    sl.relocalize = async (...a) => (relocs++, relocs === 1 ? { ok: false, stale: true } : reloc(...a));
    for (let k = 0; k < 3; k++) {
      clock += 40;
      perception.emit("frame", { t: clock, element: el, region: { sx: 0, sy: 0, sw: 640, sh: 480 }, captured: clock - 90 });
      await new Promise((res) => setImmediate(res));
    }
    assert.ok(relocs >= 2 && loc.pose().status !== "lost", `${relocs} relocalizations`);
    sl.detach();
    sl.relocalize = reloc;
    // no free voxel within 2 sigma (inside something, or space the scan never saw) is no place for the drone; the 2.5D
    // map alone says nothing (its bands are too coarse); outside the carved heights nothing either
    const box = (x, y) => Math.abs(x - 2) < 0.4 && Math.abs(y - 3) < 0.4, seen = (x) => x < 3;
    sl.map = { floorAt: () => 0, ceilingAt: () => 2.6, roomAt: () => ({ id: "r1" }), clearance: () => 0, vox: { state: (x, y) => (box(x, y) ? 2 : seen(x) ? 1 : 0) } };
    const at = (x, z = 1) => ({ x, y: 3, z, pitch: 0, roll: 0 });
    assert.deepEqual([sl.plausible(at(2), 0.03), sl.plausible(at(2.38), 0.03), sl.plausible(at(3.5), 0.03), sl.plausible(at(2.97), 0.03), sl.plausible(at(2, 1.9), 0.03)],
      ["inside an obstacle or space the scan never saw", null, "inside an obstacle or space the scan never saw", null, null]);
    delete sl.map.vox;
    assert.equal(sl.plausible(at(2), 0.03), null, "2.5D only: no answer");
    sl.map = loc.map;
    // a ready conflict: the frame solved against a render 0.5 m behind the second hypothesis, sent as a confirmation
    const sent = [], pose0 = loc.pose.bind(loc), fix0 = loc.fix.bind(loc);
    loc.pose = () => ({ ...pose0(), conflict: { x: 2.3, y: 3, yaw: 0.4, n: 4, ready: true } });
    loc.fix = (f) => (sent.push(f), true);
    twin.poses.length = 0;
    r = await sl.confirm(frame(), clock - 100);
    const rp = twin.poses[0];
    assert.ok(r.ok && sent[0]?.confirm && Math.abs(Math.hypot(rp.x - 2.3, rp.y - 3) - 0.5) < 1e-6 && Math.abs(wrapAngle(Math.atan2(3 - rp.y, 2.3 - rp.x) - 0.4)) < 1e-6, `confirm from ${JSON.stringify(rp)}`);
    [loc.pose, loc.fix] = [pose0, fix0];
    // Where am I? without a stored database: says so at once (never builds one); busy: waits at most `wait`
    const src = { kind: "goggles", ready: () => true, element: () => el, region: () => ({ sx: 0, sy: 0, sw: 640, sh: 480 }) };
    sl.attach(Object.assign(new Emitter(), { source: src }));
    let built = 0;
    sl.buildDb = async () => built++;
    Object.assign(sl, { db: null, store: { readFile: async () => null } });
    const w0 = Date.now();
    r = await sl.locateNow();
    assert.ok(!r.ok && /no relocalization database/.test(r.reason) && built === 0 && Date.now() - w0 < 200, r.reason);
    sl.busy = true;
    r = await sl.locateNow({ wait: 60 });
    sl.busy = false;
    assert.match(r.reason, /busy/);
    // lost with no database: the stored one is looked for once, none is built in flight
    let reads = 0;
    Object.assign(sl, { house: { id: "h" }, store: { readFile: async () => (reads++, null) } });
    const per = new Emitter();
    sl.attach(Object.assign(per, { source: { ...src, kind: "sim" } }));
    loc.forget();
    for (let k = 0; k < 10; k++) (clock += 100), per.emit("frame", { t: clock, element: el, region: { sx: 0, sy: 0, sw: 640, sh: 480 }, captured: clock - 90 });
    await new Promise((res) => setImmediate(res));
    assert.deepEqual([reads, built], [1, 0]);
    sl.detach();
    // DINOv2 can't load: relocalization is off for a while (one status line), tracking goes on
    loc.reset({ x: 2.3, y: 3, yaw: 0.4, z: 1 });
    Object.assign(fake, { info: { dino: "fake" }, loadDino: async () => { fake.dinoTries = (fake.dinoTries ?? 0) + 1; throw new Error("HTTP 503"); } });
    sl.db = { search: () => [], meta: {} };
    statuses.length = 0;
    for (let k = 0; k < 3; k++) await sl.relocalize(frame(), clock).catch(() => null);
    assert.ok(fake.dinoTries === 1 && statuses.filter((s) => /Relocalization is unavailable \(HTTP 503\)/.test(s.text)).length === 1, `${fake.dinoTries} tries`);
    assert.ok((await sl.track(frame(), clock)).ok, "tracking without DINOv2");
    // the worker can't start: one status line, then nothing until retry[0] has passed
    const create = F.Features.create;
    let creates = 0;
    F.Features.create = async () => (creates++, Promise.reject(new Error("no WebGPU adapter")));
    const s2 = new SplatLocalizer({ twin, localizer: loc, lens: L.droneLens(null, 20), map: loc.map, ctl }), st2 = [];
    s2.on("status", (s) => st2.push(s.text));
    for (let k = 0; k < 4; k++) await s2.track(frame(), clock).catch((e) => s2.fail(e));
    clock += SPLAT.retry[0] + 10;
    await s2.track(frame(), clock).catch((e) => s2.fail(e));
    F.Features.create = create;
    assert.ok(creates === 2 && st2.length === 2 && /trying again in 30 s/.test(st2[0]), `${creates} starts, ${JSON.stringify(st2)}`);
    // a worker that stops answering: calls time out; two in a row and the localizer starts a new one
    const W0 = globalThis.Worker, ms0 = F.CALL_MS.default;
    globalThis.Worker = class { postMessage({ id, op }) { if (op === "init") setTimeout(() => this.onmessage({ data: { id, result: { backend: "fake", dino: "x" } } })); } terminate() { this.dead = true; } };
    F.CALL_MS.default = 30;
    const hung = await F.Features.create(), s3 = new SplatLocalizer({ twin, localizer: loc, lens: L.droneLens(null, 20), map: loc.map, ctl, features: hung }), st3 = [];
    s3.on("status", (s) => st3.push(s.text));
    const e1 = await s3.track(frame(), clock).catch((e) => e), alive1 = !!s3.features, e2 = await s3.track(frame(), clock).catch((e) => e);
    [globalThis.Worker, F.CALL_MS.default] = [W0, ms0];
    assert.ok(e1.timeout && /didn't answer \(rectifier/.test(e1.message) && alive1 && e2.timeout && s3.features === null && !hung.alive && s3.stats.timeouts === 2 && !s3.busy, `${e1.message}; ${st3}`);
    assert.ok(st3.some((t) => /restarting it/.test(t)));
  },

  "video delay check: fixes timed 50 ms late during turns show up as a 50 ms offset (videoDelay should be 50 ms more)"() {
    const fly = conflictRig(), sl = new SplatLocalizer({ localizer: fly.loc, lens: L.droneLens(null, 20), ctl: fly.ctl, features: {} });
    fly.go({ s: 1, v: 0.3, yawRate: 1 });
    for (let k = 0; k < 40; k++) {
      fly.go({ s: 0.2, v: 0.3, yawRate: k % 10 < 5 ? 1 : -0.8 });
      const t = clock - 300, i = fly.hist.findIndex((h) => h.t > t - 50), a = fly.hist[i - 1], b = fly.hist[i], u = (t - 50 - a.t) / (b.t - a.t); // what the frame shows: 50 ms older than its time says
      sl.lagSample({ t, yaw: a.yaw + wrapAngle(b.yaw - a.yaw) * u + 0.004 * Math.sin(k) });
    }
    sl.lagSample({ t: clock - 20, yaw: fly.truth().yaw }); // too recent: waits until the history covers it + 150 ms
    assert.equal(sl.lagQueue.length, 1);
    fly.go({ s: 0.3, v: 0.3, yawRate: 1 });
    const d = sl.delayCheck();
    assert.equal(sl.lagQueue.length, 0);
    console.log(`      ${JSON.stringify(d)}`);
    assert.ok(d && d.n >= 20 && Math.abs(d.offset + 50) <= 10 && Math.abs(d.videoDelay - 150) <= 10 && d.rmsDeg < 0.5, JSON.stringify(d));
  },

  async "picture in the goggles' stream: the O4's 4:3 picture pillarboxed in 1920x1080 (OSD over bars and picture) is found to the pixel and snapped to 4:3; a stream source reports it as its region"() {
    const { PictureFrame } = await import("../app/js/perception.js");
    // the OSD as the Goggles 3 draw it: text over both bars and the picture, and on the right bar a column of icons and
    // numbers and a banner reaching into the picture (40% of those columns bright)
    const W = 1920, H = 1080, r = rng(31), osd = (x, y) => (((y >= 30 && y < 66) || (y >= 1010 && y < 1046)) && ((x >= 40 && x < 210) || (x >= 900 && x < 1020) || (x >= 1710 && x < 1880)))
      || (x >= 1690 && x < 1790 && y % 100 < 40) || (y >= 840 && y < 900 && x >= 1310 && x < 1790);
    const X0 = 242, frameAt = (k) => { // the picture: x 242..1681 (off the 4 px grid of the coarse copy), textured, dark patches that move; bars 0-2; white OSD text
      const g = new Uint8Array(W * H), ph = r() * 6;
      for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
        const inPic = x >= X0 && x < X0 + 1440, v = inPic ? 70 + 60 * Math.sin(x / 37 + ph) * Math.cos(y / 53 + k) + 40 * r() : 2 * r();
        g[y * W + x] = osd(x, y) && (x + y) % 3 ? 235 : (inPic && ((x / 120 + k) % 7) < 1 ? 4 : v);
      }
      return g;
    };
    const down = (g, w, h, sx = 0, sw = W) => { // box-filtered grey copy of columns sx..sx+sw at w x h (what a canvas draw gives)
      const out = new Uint8Array(w * h), fx = sw / w, fy = H / h;
      for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
        let a = 0, n = 0;
        for (let yy = Math.floor(y * fy); yy < Math.floor((y + 1) * fy); yy += 2) for (let xx = Math.floor(x * fx); xx < Math.max(Math.floor(x * fx) + 1, Math.floor((x + 1) * fx)); xx++) (a += g[yy * W + sx + xx]), n++;
        out[y * w + x] = a / n;
      }
      return out;
    };
    const P = new PictureFrame(), base = { sx: 0, sy: 0, sw: W, sh: H };
    assert.equal(P.snap(base), null, "not before `need` samples");
    for (let k = 0; k < P.need + 16; k++) { // the strips start once the coarse picture is known
      const g = frameAt(k);
      P.add(down(g, 480, 270), 480, 270);
      const e = P.edges(base);
      if (e) for (const [side, at] of [["left", e[0]], ["right", e[1]]]) { const x = Math.round(at) - 12; P.refine({ side, x, gray: down(g, 24, 64, x, 24), w: 24, h: 64 }); }
    }
    const pic = P.snap(base);
    console.log(`      found ${JSON.stringify(pic)} (coarse ${JSON.stringify(P.coarse())})`);
    assert.deepEqual(pic, { sx: X0, sy: 0, sw: 1440, sh: 1080, bars: true });
    // no bars: the region as it is
    const Q = new PictureFrame();
    for (let k = 0; k < Q.need; k++) Q.add(Uint8Array.from({ length: 64 * 48 }, () => 40 + 100 * r()), 64, 48);
    assert.deepEqual(Q.snap({ sx: 0, sy: 0, sw: 640, sh: 480 }), { sx: 0, sy: 0, sw: 640, sh: 480, bars: false });
    // a stream source (USB capture, window capture) takes the picture inside the user's crop as its region
    const { CameraSource } = await import("../app/js/perception.js");
    globalThis.document ??= { createElement: () => ({}) };
    const cam = Object.assign(new CameraSource(), { video: { videoWidth: 1920, videoHeight: 1080 } });
    assert.deepEqual(cam.region(), base);
    cam.setPicture({ sx: 240, sy: 0, sw: 1440, sh: 1080 });
    assert.deepEqual(cam.region(), { sx: 240, sy: 0, sw: 1440, sh: 1080 });
    cam.crop = { x: 0.5, y: 0, w: 0.5, h: 1 }; // a crop the picture doesn't fit in: the crop
    assert.deepEqual(cam.region(), { sx: 960, sy: 0, sw: 960, sh: 1080 });
  },

  "augment: JPEG's loss in JS (no encoder: a hidden page's waits a second a frame): flat stays flat, detail loses more at lower quality, 8x8 blocks, 4:2:0 colour"() {
    const w = 96, h = 64, r = rng(5), img = new Uint8ClampedArray(w * h * 4);
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      const i = 4 * (y * w + x), v = 120 + 50 * Math.sin(x / 3) * Math.cos(y / 5) + 30 * (r() - 0.5);
      img.set([v, v, v, 255], i); // grey: the loss is the luma's quantization
    }
    const psnr = (q) => {
      const p = jpegLoss(img.slice(), w, h, q);
      let se = 0;
      for (let i = 0; i < p.length; i += 4) for (let c = 0; c < 3; c++) se += (p[i + c] - img[i + c]) ** 2;
      return { p, db: 10 * Math.log10((255 * 255 * w * h * 3) / se) };
    };
    const hi = psnr(0.9), mid = psnr(0.6), lo = psnr(0.1);
    assert.ok(hi.db > mid.db + 2 && mid.db > lo.db + 2 && lo.db > 18 && hi.db < 45, `PSNR ${hi.db.toFixed(1)} / ${mid.db.toFixed(1)} / ${lo.db.toFixed(1)} dB`);
    assert.ok(mid.p.every((v, i) => i % 4 !== 3 || v === 255), "alpha untouched");
    const flat = jpegLoss(new Uint8ClampedArray(16 * 16 * 4).fill(90), 16, 16, 0.3);
    assert.ok(flat.every((v, i) => (i % 4 === 3 ? v === 90 : Math.abs(v - 90) <= 1)), "a flat patch stays flat");
    const step = new Uint8ClampedArray(16 * 8 * 4);
    for (let x = 0; x < 16; x++) for (let y = 0; y < 8; y++) step.set(x < 8 ? [40, 40, 40, 255] : [200, 200, 200, 255], 4 * (y * 16 + x));
    assert.deepEqual(Array.from(jpegLoss(step.slice(), 16, 8, 0.6).filter((_, i) => i % 4 === 0)), Array.from(step.filter((_, i) => i % 4 === 0)), "an edge on the 8x8 grid is kept exactly");
    const red = jpegLoss(new Uint8ClampedArray([255, 0, 0, 255, 0, 0, 255, 255, 255, 0, 0, 255, 0, 0, 255, 255]), 2, 2, 1);
    assert.ok(red[0] < 200 && red[4] > 60 && Math.abs(red[0] - red[4] - (0.299 - 0.114) * 255) < 3, `red and blue pixels share their 2x2 chroma: only luma tells them apart (R ${red[0]}, ${red[4]})`);
  },

  "simulator: augmentation options and the lens error; addObstacle and setDoor change the collision world and go to the twin as props (guarded without setProps)"() {
    const a = augmentOptions(true);
    assert.deepEqual(a.lensError, LENS_ERROR);
    assert.ok(a.blur && a.noise && a.exposure && a.osd && a.jpeg > 0);
    assert.equal(augmentOptions(null), null);
    assert.equal(augmentOptions({ jpeg: 0 }).lensError, null);
    const lens = simLens(127, 20, LENS_ERROR);
    assert.deepEqual([+lens.diagFovDeg.toFixed(3), lens.cx, lens.uptiltDeg], [+(127 * 1.25 * 1.03).toFixed(3), 0.51, 22]);
    const sim = new Simulator({ headless: true, house, map, seed: 3, augment: true });
    assert.equal(sim.twinLens().uptiltDeg, 22);
    const props = [], errors = [];
    sim.on("error", (e) => errors.push(e));
    sim.setTwin({ setLens: async () => {}, setProps: async (list) => props.push(list) });
    const w = sim.world, crate = { id: "crate", x: 2.4, y: 2.0, w: 0.5, d: 0.5, h: 0.9 };
    assert.equal(w.blocked(1.8, 2.0, 3.0, 2.0, 0.5), false);
    sim.addObstacle(crate);
    assert.equal(w.blocked(1.8, 2.0, 3.0, 2.0, 0.5), true, "the crate blocks at 0.5 m");
    assert.equal(w.blocked(1.8, 2.0, 3.0, 2.0, 1.2), false, "not above it");
    const body = { x: 2.45, y: 2.0, z: 0.5, vx: 0, vy: 0 };
    w.collide(body, 0.05);
    assert.ok(Math.abs(body.x - 2.4) >= 0.29 || Math.abs(body.y - 2.0) >= 0.29, "pushed out of the crate");
    assert.deepEqual(props.at(-1).map((p) => [p.id, p.kind, p.z]), [["crate", "box", map.floorAt(2.4, 2.0)]]);
    const door = map.doors.find((d) => d.id === "r2:w4-o1"), mid = [(door.a[0] + door.b[0]) / 2, (door.a[1] + door.b[1]) / 2];
    const across = (z) => w.blocked(mid[0] - 0.4, mid[1], mid[0] + 0.4, mid[1], z);
    assert.equal(across(1.0), false, "the doorway is open");
    assert.equal(sim.setDoor(door.id, false), true);
    assert.equal(across(1.0), true, "closed");
    assert.deepEqual(props.at(-1).map((p) => p.id), ["crate", "door:r2:w4-o1"]);
    const leaf = props.at(-1)[1];
    assert.ok(leaf.kind === "panel" && Math.hypot(leaf.x - mid[0], leaf.y - mid[1]) < 0.1 && leaf.h > 1.8);
    sim.setDoor(door.id, true);
    sim.removeObstacle("crate");
    assert.equal(across(1.0), false, "open again");
    assert.equal(w.blocked(1.8, 2.0, 3.0, 2.0, 0.5), false, "crate gone");
    assert.deepEqual(props.at(-1), []);
    sim.setTwin({ setLens: async () => {} }); // an older twin: no setProps
    sim.addObstacle(crate);
    sim.addObstacle({ ...crate, id: "crate2", x: 3 });
    assert.equal(errors.length, 1);
    assert.match(errors[0], /setProps/);
  },
};

// ---------------------------------------------------------------- rigs

// The localizer on a fake controller flying a wandering route at 0.45 m/s with turns, flow at the wrong scale (0.84) with
// slow bias, a heading that drifts, a climb rate 0.1 m/s off; splat-style fixes of the truth.
function flightRig({ seed = 1, videoDelay = 100 } = {}) {
  const ctl = new FakeCtl(videoDelay), loc = new Localizer({ ctl }), r = rng(seed), g = gaussOf(rng(seed + 100));
  loc.setSource("fused");
  let x = 1, y = 1, z = 1, yaw = 0, fb = [0, 0], tt = 0;
  const hist = [], k = 0.84, dt = 1 / 30;
  ctl.flying = false;
  loc.reset({ x, y, yaw, z });
  ctl.flying = true;
  const truth = () => ({ x, y, z, yaw });
  async function run({ seconds, rate = 4, noise = 0.04, outl = 0.05, latency = 250, flowBias = 0.05 }) {
    const pending = [], errs = [], zerr = [], yawE = [], sig = [], status = {};
    let nextFix = clock, outliers = 0, outGated = 0, good = 0, goodUsed = 0;
    for (let i = 0; i < seconds * 30; i++) {
      clock += 1000 * dt;
      tt += dt;
      const v = 0.45 * (Math.sin(tt * 0.3) > -0.3 ? 1 : 0.2), yr = 0.5 * Math.sin(tt * 0.21) + (Math.floor(tt / 8) % 2 ? 0.6 : -0.3) * (tt % 8 < 1.5);
      yaw = wrapAngle(yaw + yr * dt);
      x += v * Math.cos(yaw) * dt;
      y += v * Math.sin(yaw) * dt;
      const vz = 0.3 * Math.cos(tt / 5) / 5;
      z += vz * dt;
      hist.push({ t: clock, x, y, z, yaw });
      fb = fb.map((b) => b * 0.95 + 0.05 * g() * flowBias * 4);
      Object.assign(ctl.est, { vx: v / k + fb[0], vy: fb[1], vz: vz + 0.1, heading: -yaw + 0.0005 * tt });
      ctl.perception.latest.flow = { t: clock };
      ctl.emit("tick", { dt, now: clock });
      if (rate && clock >= nextFix) {
        nextFix += 1000 / rate;
        const o = r() < outl, h = hist.at(-1);
        pending.push({ at: clock + latency, o, fix: { x: h.x + (o ? (r() < 0.5 ? -1 : 1) * (0.8 + r()) : noise * g()), y: h.y + (o ? (r() - 0.5) * 2 : noise * g()), z: h.z + noise * g(),
          yaw: h.yaw + (o ? 0.5 : 0.015 * g()), sigma: 0.05, yawSigma: 0.025, t: clock, source: "splat", inliers: 500 } });
      }
      while (pending.length && pending[0].at <= clock) {
        const { o, fix } = pending.shift(), used = loc.fix(fix);
        if (o) (outliers++, (outGated += !used));
        else (good++, (goodUsed += used));
      }
      const p = loc.pose();
      if (i < 60) continue;
      errs.push(Math.hypot(p.x - x, p.y - y));
      zerr.push(Math.abs(p.z - z));
      yawE.push(Math.abs(wrapAngle(p.yaw - yaw)) / DEG);
      sig.push(p.sigma);
      status[p.status] = (status[p.status] ?? 0) + 1;
    }
    const s = (a) => ({ med: q(a, 0.5), p95: q(a, 0.95), max: Math.max(...a) });
    const hud = loc.fixQuality(clock);
    return { err: s(errs), zerr: s(zerr), yaw: s(yawE), sigma: s(sig), status, outliers, outGated, good, goodUsed, hud: { ...hud, age: Math.round(hud.age), rate: +hud.rate.toFixed(1) } };
  }
  return { ctl, loc, run, truth, hist };
}

// A drone flying straight at v (m/s) turning at yawRate with exact odometry, fixes at `fixes` Hz (fix(truth) -> fix, or
// the truth with 3 cm noise). shift(d): the filter's estimate jumps d m sideways (as if the odometry had slipped).
function conflictRig() {
  const ctl = new FakeCtl(100), loc = new Localizer({ ctl }), g = gaussOf(rng(5)), hist = [];
  loc.setSource("fused");
  let x = 1, y = 1, yaw = 0;
  ctl.flying = false;
  loc.reset({ x, y, yaw, z: 1 });
  ctl.flying = true;
  const truth = () => ({ x, y, yaw });
  const go = ({ s, v = 0.4, yawRate = 0, fixes = 0, fix = (p) => ({ x: p.x + 0.03 * g(), y: p.y + 0.03 * g(), z: 1, yaw: p.yaw + 0.005 * g(), sigma: 0.04, yawSigma: 0.01, source: "splat" }) }) => {
    const used = [];
    for (let i = 0; i < s * 30; i++) {
      clock += 1000 / 30;
      yaw = wrapAngle(yaw + yawRate / 30);
      x += (v * Math.cos(yaw)) / 30;
      y += (v * Math.sin(yaw)) / 30;
      hist.push({ t: clock, x, y, yaw });
      Object.assign(ctl.est, { vx: v, vy: 0, vz: 0, heading: -yaw, flowQ: 0.8 });
      ctl.perception.latest.flow = { t: clock };
      ctl.emit("tick", { dt: 1 / 30, now: clock });
      if (fixes && i % Math.round(30 / fixes) === 0) used.push(loc.fix({ ...fix(truth()), t: clock }));
    }
    return used;
  };
  const shift = (d) => { // the estimate and its history (late measurements replay from there), without the fixes it had
    for (const k of loc.ticks) k.fixes = [];
    for (const st of [loc.kf, ...loc.ticks.map((k) => k.pre)]) (st.s[0] -= d * Math.sin(yaw)), (st.s[1] += d * Math.cos(yaw));
  };
  return { ctl, loc, go, truth, shift, hist };
}

const frame = () => new ImageBitmap();
globalThis.ImageBitmap ??= class { width = 640; height = 480; close() {} };
globalThis.createImageBitmap ??= async () => new ImageBitmap();

// SplatLocalizer with a fake twin and worker: the worker's PnP answers the truth's camera (± 1 cm, 0.2 deg).
function splatRig() {
  const ctl = new FakeCtl(100), loc = new Localizer({ ctl, map: { floorAt: () => 0, ceilingAt: () => 2.6, roomAt: () => ({ id: "r1" }) } }), g = gaussOf(rng(77));
  loc.setSource("fused");
  let truth = null;
  const fake = {
    inliers: 500, solves: 0, lastBoxes: null, info: { dino: "fake" }, loadDino: async () => (fake.info.dinoLoaded = true),
    rectifier: async ({ key }) => ({ key, valid: 1 }), setMask: async () => ({ masked: 0 }), release: () => {},
    frame: async ({ boxes, dino }) => ((fake.lastBoxes = boxes), { id: 1, n: 2048, ms: {}, desc: dino ? new Float32Array(768) : null }),
    render: async () => ({ id: 2, n: 2048, lifted: 1500, ms: {} }),
    solve: async () => {
      fake.solves++;
      const n = fake.noise ?? 0.01, b = fake.bias ?? {}, cam = L.cameraOf({ ...truth, pitch: b.pitch ?? 0, roll: 0, x: truth.x + (b.x ?? 0) + n * g(), y: truth.y + n * g(), yaw: truth.yaw + (b.yaw ?? 0) + 0.2 * n * g() }, 20);
      const c = 1e-5;
      return { ok: true, matches: 900, lifted: 800, inliers: fake.queue?.length ? fake.queue.shift() : fake.inliers, R: cam.R, C: cam.C, rms: 0.9, cov: [0, 1, 2, 3, 4, 5].map((i) => [0, 1, 2, 3, 4, 5].map((j) => (i === j ? (i < 3 ? c : 1e-7) : 0))), ms: {} };
    },
  };
  const twin = { poses: [], pixels: async (pose) => (twin.poses.push(pose), { width: 640, height: 480, data: new Uint8ClampedArray(4) }), depth: async () => new Float32Array(1) };
  const sl = new SplatLocalizer({ twin, localizer: loc, lens: L.droneLens(null, 20), map: loc.map, ctl, features: fake });
  sl.osd = OsdMask.from({ width: 4, height: 3, rle: [12] });
  return { sl, loc, fake, ctl, twin, truth: () => truth, setTruth: (p) => (truth = p) };
}

let failed = 0;
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name}\n      ${e.stack?.split("\n").slice(0, 12).join("\n      ") ?? e}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed (${((Date.now() - T0) / 1000).toFixed(1)} s)`);
process.exit(failed ? 1 : 0);
