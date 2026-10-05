// The O4 camera for localization (docs/HOME-DRONE.md, Wave C): its fisheye (twin/lens.js's models and math, not a
// copy), calib.json, the fisheye -> pinhole rectification the matcher works in, camera <-> body poses, and lens
// calibration from frames matched against twin renders.
// calib.json: { model: "equidistant", fx, fy, cx, cy, k: [k1..k4], uptiltDeg, width, height, osdMask?, rms, source }
// (pixels at width x height). A lens object is twin/lens.js's (fx, fy / width, cx, cy / size).
// Rectified view: a pinhole camera with the drone camera's own axes, RECT_HFOV wide, 4:3 (640x480 or 320x240). The twin
// renders the same view (pinhole lens, the camera uptilt), so live and expected images line up pixel for pixel.
import { DRONE_LENS, DEG, intrinsics, project, unproject, resolveLens } from "../twin/lens.js";
import { droneCamera, mul3 } from "../twin/pose.js";

export { DRONE_LENS, DEG, intrinsics, project, unproject, resolveLens };
export const RECT_HFOV = 100;
export const RECT_SIZES = { full: [640, 480], half: [320, 240] };
export const CAM_DZ = 0.025; // the camera sits this far above the body centre (sim/simulator.js cameraPose)

// The drone lens: calib.json when there is one, else the O4 default at the settings' uptilt.
export function droneLens(calib, uptiltDeg = DRONE_LENS.uptiltDeg) {
  if (!calib?.fx) return resolveLens({ ...DRONE_LENS, uptiltDeg });
  const { width: w, height: h, k = [] } = calib;
  return resolveLens({ model: "equidistant", fx: calib.fx / w, fy: (calib.fy ?? calib.fx) / w, cx: calib.cx / w, cy: calib.cy / h, aspect: w / h,
    k1: k[0] ?? 0, k2: k[1] ?? 0, k3: k[2] ?? 0, k4: k[3] ?? 0, uptiltDeg: calib.uptiltDeg ?? uptiltDeg });
}

export function calibOf(lens, width = 640, height = 480, extra = {}) {
  const K = intrinsics(lens, width, height), r = (v) => +v.toFixed(4);
  return { model: "equidistant", fx: r(K.fx), fy: r(K.fy), cx: r(K.cx), cy: r(K.cy), k: [K.k1, K.k2, K.k3, K.k4].map(r), uptiltDeg: +(resolveLens(lens).uptiltDeg ?? 20).toFixed(2),
    width, height, rms: null, source: "default", ...extra };
}

export const rectLens = (uptiltDeg = DRONE_LENS.uptiltDeg, hfovDeg = RECT_HFOV) => ({ model: "pinhole", hfovDeg, cx: 0.5, cy: 0.5, uptiltDeg });

// Source pixel (x right, y down, continuous: pixel i covers [i, i + 1)) of every rectified pixel, NaN where the
// fisheye does not reach (or the source frame is cropped, e.g. 16:9).
export function rectifyMap(srcLens, srcW, srcH, outW = 640, outH = 480, hfovDeg = RECT_HFOV) {
  const Kf = intrinsics(srcLens, srcW, srcH), Kp = intrinsics(rectLens(0, hfovDeg), outW, outH), map = new Float32Array(outW * outH * 2);
  let valid = 0;
  for (let v = 0, i = 0; v < outH; v++) for (let u = 0; u < outW; u++, i += 2) {
    const p = project(Kf, unproject(Kp, u + 0.5, v + 0.5));
    const ok = p && p[0] >= 0.5 && p[1] >= 0.5 && p[0] <= srcW - 0.5 && p[1] <= srcH - 0.5;
    map[i] = ok ? p[0] : NaN;
    map[i + 1] = ok ? p[1] : NaN;
    valid += ok;
  }
  return { map, K: Kp, Kf, outW, outH, srcW, srcH, valid: valid / (outW * outH) };
}

// Bilinear resampling through a rectifyMap: precomputed top-left index and 8-bit weights, so a 640x480 frame is a few
// ms of integer work (worker side). valid: Uint8Array per output pixel.
export class Rectifier {
  constructor({ map, outW, outH, srcW, srcH, K, Kf }) {
    Object.assign(this, { map, outW, outH, srcW, srcH, K, Kf });
    const n = outW * outH;
    this.idx = new Int32Array(n).fill(-1);
    this.w = new Uint8Array(n * 2);
    this.valid = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const x = map[2 * i] - 0.5, y = map[2 * i + 1] - 0.5;
      if (!(x >= 0 && y >= 0)) continue;
      const x0 = Math.min(srcW - 2, x | 0), y0 = Math.min(srcH - 2, y | 0);
      this.idx[i] = y0 * srcW + x0;
      this.w[2 * i] = Math.round(Math.min(1, x - x0) * 255);
      this.w[2 * i + 1] = Math.round(Math.min(1, y - y0) * 255);
      this.valid[i] = 1;
    }
  }
  // src: RGBA bytes srcW x srcH -> RGBA bytes outW x outH (invalid pixels black).
  rgba(src, out = new Uint8ClampedArray(this.outW * this.outH * 4)) {
    const { idx, w, srcW } = this, n = idx.length, row = srcW * 4;
    for (let i = 0; i < n; i++) {
      const o = i * 4, k = idx[i];
      if (k < 0) { out[o] = out[o + 1] = out[o + 2] = 0; out[o + 3] = 255; continue; }
      const a = k * 4, fx = w[2 * i], fy = w[2 * i + 1], gx = 255 - fx, gy = 255 - fy;
      for (let c = 0; c < 3; c++)
        out[o + c] = (gy * (gx * src[a + c] + fx * src[a + 4 + c]) + fy * (gx * src[a + row + c] + fx * src[a + row + 4 + c]) + 32512) / 65025;
      out[o + 3] = 255;
    }
    return out;
  }
  // Fisheye pixel of a rectified pixel position (bilinear in the map), or null.
  toSource(u, v) {
    const { map, outW, outH } = this, x = Math.min(outW - 1, Math.max(0, u - 0.5)), y = Math.min(outH - 1, Math.max(0, v - 0.5));
    const x0 = Math.min(outW - 2, x | 0), y0 = Math.min(outH - 2, y | 0), fx = x - x0, fy = y - y0, at = (c, r, k) => map[2 * (r * outW + c) + k];
    const out = [0, 1].map((k) => (1 - fy) * ((1 - fx) * at(x0, y0, k) + fx * at(x0 + 1, y0, k)) + fy * ((1 - fx) * at(x0, y0 + 1, k) + fx * at(x0 + 1, y0 + 1, k)));
    return Number.isFinite(out[0]) && Number.isFinite(out[1]) ? out : null;
  }
}

// ---------------------------------------------------------------- camera <-> body poses

const T3 = (A) => [0, 1, 2].map((i) => [A[0][i], A[1][i], A[2][i]]);
const rotY = (t) => [[Math.cos(t), 0, Math.sin(t)], [0, 1, 0], [-Math.sin(t), 0, Math.cos(t)]];
export const BODY_FROM_CV = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]];

// Camera of a body pose: R (columns = OpenCV camera axes in H), centre C (CAM_DZ above the body).
export function cameraOf(pose, uptiltDeg) {
  const c = droneCamera({ ...pose, z: pose.z + CAM_DZ }, uptiltDeg);
  return { R: c.R, C: c.p };
}

// Body pose of a camera (R, C as above) under an uptilt: yaw, pitch (+ nose down), roll as sim/drone.js.
export function bodyOf(R, C, uptiltDeg) {
  const B = mul3(mul3(R, T3(BODY_FROM_CV)), rotY(uptiltDeg * DEG));
  return { x: C[0], y: C[1], z: C[2] - CAM_DZ, yaw: Math.atan2(B[1][0], B[0][0]), pitch: Math.asin(Math.max(-1, Math.min(1, -B[2][0]))), roll: Math.atan2(B[2][1], B[2][2]) };
}

// OpenCV solvePnP output (world -> camera rvec, tvec) <-> camera R (camera -> world), C.
export function rodrigues([x, y, z]) {
  const t = Math.hypot(x, y, z);
  if (t < 1e-12) return [[1, -z, y], [z, 1, -x], [-y, x, 1]];
  const [a, b, c] = [x / t, y / t, z / t], s = Math.sin(t), k = 1 - Math.cos(t);
  return [[1 + k * (a * a - 1), k * a * b - s * c, k * a * c + s * b], [k * a * b + s * c, 1 + k * (b * b - 1), k * b * c - s * a], [k * a * c - s * b, k * b * c + s * a, 1 + k * (c * c - 1)]];
}
export function rvecOf(R) {
  const c = Math.max(-1, Math.min(1, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)), t = Math.acos(c);
  if (t < 1e-9) return [0, 0, 0];
  if (Math.PI - t < 1e-6) {
    const i = [0, 1, 2].reduce((a, b) => (R[b][b] > R[a][a] ? b : a), 0), v = [0, 1, 2].map((j) => (R[i][j] + (i === j ? 1 : 0)) / 2);
    const n = Math.hypot(...v);
    return v.map((x) => (x / n) * t);
  }
  const k = t / (2 * Math.sin(t));
  return [(R[2][1] - R[1][2]) * k, (R[0][2] - R[2][0]) * k, (R[1][0] - R[0][1]) * k];
}
export const cameraFromPnP = (rvec, tvec) => { const Rw = rodrigues(rvec), R = T3(Rw); return { R, C: R.map((row) => -(row[0] * tvec[0] + row[1] * tvec[1] + row[2] * tvec[2])) }; };
export const pnpFromCamera = (R, C) => { const Rw = T3(R); return { rvec: rvecOf(Rw), tvec: Rw.map((row) => -(row[0] * C[0] + row[1] * C[1] + row[2] * C[2])) }; };
export const rotAngle = (A, B) => { const M = mul3(A, T3(B)); return Math.acos(Math.max(-1, Math.min(1, (M[0][0] + M[1][1] + M[2][2] - 1) / 2))); };

// ---------------------------------------------------------------- calibration

// Pixel of H point X seen by camera (R, C) through intrinsics K (twin/lens.js), or null.
export function projectH(K, R, C, X) {
  const d = [X[0] - C[0], X[1] - C[1], X[2] - C[2]], p = [0, 1, 2].map((j) => R[0][j] * d[0] + R[1][j] * d[1] + R[2][j] * d[2]);
  return p[2] > 1e-6 || K.model !== "pinhole" ? project(K, p) : null;
}

const smallRot = ([a, b, c]) => rodrigues([a, b, c]);
function solve(A, b) {
  const n = b.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c];
    if (Math.abs(d) < 1e-15) return null;
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c] / d; if (f) for (let j = c; j <= n; j++) M[r][j] -= f * M[c][j]; }
  }
  return M.map((r, i) => r[n] / r[i]);
}

// Lens (intrinsics + uptilt) from frames matched against twin renders. samples: [{ uv: [u0, v0, u1, ...] fisheye pixels,
// xyz: [x0, y0, z0, ...] the same points in H (lifted with twin depth), R, C: the camera's pose (a first guess: PnP with
// the old lens), grounded: the drone sat on the pad, turned by hand }], all at width x height. Joint Levenberg-Marquardt,
// Huber-weighted (2 px), over fx, fy, cx, cy, k1, k2, and the poses: a free sample has its own 6; the grounded ones
// share one body on the pad (its camera centre, the camera's offset forward of the yaw axis, the pad's tilt and the
// camera's uptilt) and have only their yaw, so focal length can't be traded for distance view by view.
// pad: { x, y, z } the camera centre on the pad, a first guess (else the grounded guesses' mean).
// Returns calib.json (rms: reprojection RMS in px of the inliers; pad: the fitted pad body) and the refined poses.
export function calibrateLens(samples, { lens = DRONE_LENS, width = 640, height = 480, iterations = 40, huber = 2, fit = ["fx", "fy", "cx", "cy", "k1", "k2"], pad = null } = {}) {
  const K0 = intrinsics(lens, width, height), names = ["fx", "fy", "cx", "cy", "k1", "k2"], up0 = resolveLens(lens).uptiltDeg ?? DRONE_LENS.uptiltDeg;
  let th = [K0.fx, K0.fy, K0.cx, K0.cy, K0.k1, K0.k2];
  const Kof = (t) => ({ ...K0, fx: t[0], fy: t[1], cx: t[2], cy: t[3], k1: t[4], k2: t[5] });
  const ground = samples.map((s, j) => (s.grounded ? j : -1)).filter((j) => j >= 0), G = ground.length;
  const mean = (k) => ground.reduce((a, j) => a + samples[j].C[k], 0) / G;
  // shared: [Cx, Cy, Cz, forward offset, pad tilt about x, about y, uptilt (deg)]
  let g = G ? [pad?.x ?? mean(0), pad?.y ?? mean(1), pad?.z ?? mean(2), 0, 0, 0, up0] : [];
  let yaws = ground.map((j) => bodyOf(samples[j].R, samples[j].C, up0).yaw);
  let poses = samples.map((s) => ({ R: s.R.map((r) => r.slice()), C: s.C.slice() }));
  const padPose = (gg, yaw) => {
    const T = rodrigues([gg[4], gg[5], 0]), R = mul3(T, cameraOf({ x: 0, y: 0, z: -CAM_DZ, yaw, pitch: 0, roll: 0 }, gg[6]).R), f = [Math.cos(yaw) * gg[3], Math.sin(yaw) * gg[3], 0];
    return { R, C: [0, 1, 2].map((k) => gg[k] + T[k][0] * f[0] + T[k][1] * f[1] + T[k][2] * f[2]) };
  };
  const poseOf = (j, gg = g, ys = yaws, ps = poses) => (samples[j].grounded ? padPose(gg, ys[ground.indexOf(j)]) : ps[j]);
  const free = names.map((n) => fit.includes(n)), steps = [0.5, 0.5, 0.5, 0.5, 1e-3, 1e-3], gSteps = [1e-3, 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 0.01];
  const residuals = (K, pose, s) => {
    const n = s.uv.length / 2, r = new Float64Array(2 * n);
    for (let i = 0; i < n; i++) {
      const p = projectH(K, pose.R, pose.C, [s.xyz[3 * i], s.xyz[3 * i + 1], s.xyz[3 * i + 2]]);
      r[2 * i] = p ? p[0] - s.uv[2 * i] : 50;
      r[2 * i + 1] = p ? p[1] - s.uv[2 * i + 1] : 50;
    }
    return r;
  };
  const moved = (pose, d) => ({ R: mul3(pose.R, smallRot(d.slice(3))), C: [pose.C[0] + d[0], pose.C[1] + d[1], pose.C[2] + d[2]] });
  const weight = (r) => { const a = Math.abs(r); return a <= huber ? 1 : huber / a; };
  const rho = (v) => (Math.abs(v) <= huber ? v * v : 2 * huber * Math.abs(v) - huber * huber);
  const cost = (t, gg, ys, ps) => samples.reduce((a, s, j) => a + residuals(Kof(t), poseOf(j, gg, ys, ps), s).reduce((b, v) => b + rho(v), 0), 0);
  // unknowns: intrinsics, shared pad body, grounded yaws, then 6 per free sample
  const P = 6, iG = P, iY = iG + g.length, iF = iY + G, fr = samples.map((s, j) => (s.grounded ? -1 : j)).filter((j) => j >= 0), N = iF + 6 * fr.length;
  let lambda = 1e-3, c0 = cost(th, g, yaws, poses);
  for (let it = 0; it < iterations; it++) {
    const A = Array.from({ length: N }, () => new Float64Array(N)), b = new Float64Array(N), K = Kof(th);
    samples.forEach((s, j) => {
      const pose = poseOf(j), r0 = residuals(K, pose, s), cols = [], col = (k, r1, h) => cols.push({ k, d: r1.map((v, i) => (v - r0[i]) / h) });
      for (let p = 0; p < P; p++) if (free[p]) { const t2 = th.slice(); t2[p] += steps[p]; col(p, residuals(Kof(t2), pose, s), steps[p]); }
      if (s.grounded) {
        const gi = ground.indexOf(j);
        for (let q = 0; q < g.length; q++) { const g2 = g.slice(); g2[q] += gSteps[q]; col(iG + q, residuals(K, padPose(g2, yaws[gi]), s), gSteps[q]); }
        col(iY + gi, residuals(K, padPose(g, yaws[gi] + 1e-4), s), 1e-4);
      } else {
        const f = fr.indexOf(j);
        for (let q = 0; q < 6; q++) { const d = [0, 0, 0, 0, 0, 0]; d[q] = q < 3 ? 1e-3 : 1e-4; col(iF + 6 * f + q, residuals(K, moved(pose, d), s), d[q]); }
      }
      for (let i = 0; i < r0.length; i++) {
        const w = weight(r0[i]);
        for (const a of cols) {
          const ja = a.d[i] * w;
          if (!ja) continue;
          b[a.k] -= ja * r0[i];
          for (const c of cols) A[a.k][c.k] += ja * c.d[i];
        }
      }
    });
    let improved = false;
    for (let tries = 0; tries < 6 && !improved; tries++) {
      const M = A.map((row, i) => Array.from(row, (v, k) => (i === k ? v * (1 + lambda) + (v ? 0 : 1e-9) : v)));
      const dx = solve(M, Array.from(b));
      if (!dx) { lambda *= 10; continue; }
      const t2 = th.map((v, p) => v + (free[p] ? dx[p] : 0)), g2 = g.map((v, q) => v + dx[iG + q]), y2 = yaws.map((v, i) => v + dx[iY + i]);
      const p2 = poses.map((ps, j) => (samples[j].grounded ? ps : moved(ps, dx.slice(iF + 6 * fr.indexOf(j), iF + 6 * fr.indexOf(j) + 6))));
      const c1 = cost(t2, g2, y2, p2);
      if (c1 < c0) [th, g, yaws, poses, c0, lambda, improved] = [t2, g2, y2, p2, c1, Math.max(1e-7, lambda / 10), true];
      else lambda *= 10;
    }
    if (!improved) break;
  }
  poses = samples.map((_, j) => poseOf(j));
  const K = Kof(th);
  let se = 0, n = 0;
  samples.forEach((s, j) => residuals(K, poses[j], s).forEach((v, i, r) => {
    if (i % 2 || Math.hypot(v, r[i + 1]) > 3 * huber) return;
    se += v * v + r[i + 1] * r[i + 1];
    n++;
  }));
  const calib = { model: "equidistant", fx: +th[0].toFixed(3), fy: +th[1].toFixed(3), cx: +th[2].toFixed(3), cy: +th[3].toFixed(3), k: [+th[4].toFixed(5), +th[5].toFixed(5), 0, 0],
    uptiltDeg: +(G ? g[6] : up0).toFixed(2), width, height, rms: n ? +Math.sqrt(se / n).toFixed(3) : null, source: "twin", points: n, frames: samples.length,
    ...(G && { pad: { C: g.slice(0, 3).map((v) => +v.toFixed(4)), forward: +g[3].toFixed(4), tiltDeg: [+(g[4] / DEG).toFixed(2), +(g[5] / DEG).toFixed(2)] } }) };
  return { calib, poses };
}
