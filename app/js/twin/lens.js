// Drone camera lens models and the view layout the twin renders them from. Pure math (no three.js), so Node
// tests and the page share it with the worker.
// Camera frame: OpenCV (x right, y down, z forward). Pixels: u right, v down, pixel centres at +0.5.
// A lens is { model: "equidistant" | "pinhole", diagFovDeg | hfovDeg, fx?, fy? (focal / image width),
//   cx, cy (principal point / image size), k1..k4 (Kannala-Brandt: theta_d = theta (1 + k1 theta^2 + ...)), uptiltDeg,
//   aspect (equidistant: the frame diagFovDeg is measured on; other output shapes are crops of it at the same width) }.
export const DEG = Math.PI / 180;

// Meteor65 Pro II O4 Wide until calibrated: 4:3, 159 deg diagonal, equidistant (about 127 x 95 deg), 20 deg uptilt.
// 16:9 output is its centre crop (127 x 72 deg), as the O4 records it. The Goggles 3's live stream is not a crop: it
// pillarboxes the 4:3 picture in 1920x1080 (1440x1080 in the middle), and perception.js finds the picture in it.
export const DRONE_LENS = Object.freeze({ model: "equidistant", diagFovDeg: 159, aspect: 4 / 3, cx: 0.5, cy: 0.5, k1: 0, k2: 0, k3: 0, k4: 0, uptiltDeg: 20 });
export const pinholeLens = (hfovDeg = 90, extra = {}) => ({ model: "pinhole", hfovDeg, cx: 0.5, cy: 0.5, uptiltDeg: 0, ...extra });

// A lens that names its model is complete: missing fields come from that model's defaults (DRONE_LENS, pinholeLens()),
// never from the lens it replaces, so a pinhole view does not inherit the drone's uptilt. Without a model it adjusts base.
export const resolveLens = (l, base = DRONE_LENS) => (l?.model ? { ...(l.model === "pinhole" ? pinholeLens() : DRONE_LENS), ...l } : { ...base, ...l });

const thetaD = (K, t) => { const t2 = t * t; return t * (1 + t2 * (K.k1 + t2 * (K.k2 + t2 * (K.k3 + t2 * K.k4)))); };
const thetaDdt = (K, t) => { const t2 = t * t; return 1 + t2 * (3 * K.k1 + t2 * (5 * K.k2 + t2 * (7 * K.k3 + t2 * 9 * K.k4))); };

// Pixel intrinsics of a lens at an image size.
export function intrinsics(lens, width, height) {
  const L = resolveLens(lens);
  const K = { model: L.model, width, height, cx: L.cx * width, cy: L.cy * height, k1: +L.k1 || 0, k2: +L.k2 || 0, k3: +L.k3 || 0, k4: +L.k4 || 0 };
  if (L.model === "pinhole") {
    K.fx = L.fx ? L.fx * width : width / 2 / Math.tan((L.hfovDeg * DEG) / 2);
    K.k1 = K.k2 = K.k3 = K.k4 = 0;
  } else if (L.model === "equidistant") {
    K.fx = L.fx ? L.fx * width : Math.hypot(width, L.aspect ? width / L.aspect : height) / 2 / thetaD(K, (L.diagFovDeg * DEG) / 2);
  } else throw new Error(`unknown lens model ${L.model}`);
  K.fy = L.fy ? L.fy * width : K.fx;
  return K;
}

// Unit ray (camera frame) through pixel position (u, v), or null outside the lens.
export function unproject(K, u, v) {
  const mx = (u - K.cx) / K.fx, my = (v - K.cy) / K.fy;
  if (K.model === "pinhole") { const n = Math.hypot(mx, my, 1); return [mx / n, my / n, 1 / n]; }
  const td = Math.hypot(mx, my);
  let t = td;
  if (K.k1 || K.k2 || K.k3 || K.k4) for (let i = 0; i < 10; i++) t -= (thetaD(K, t) - td) / thetaDdt(K, t);
  if (!(t >= 0 && t < Math.PI)) return null;
  const s = td > 1e-12 ? Math.sin(t) / td : 1;
  return [mx * s, my * s, Math.cos(t)];
}

// Pixel position [u, v] of a camera-frame direction, or null if the lens cannot see it.
export function project(K, [x, y, z]) {
  if (K.model === "pinhole") return z > 1e-9 ? [K.cx + (K.fx * x) / z, K.cy + (K.fy * y) / z] : null;
  const rho = Math.hypot(x, y), t = Math.atan2(rho, z);
  if (rho < 1e-12) return [K.cx, K.cy];
  const r = thetaD(K, t) / rho;
  return [K.cx + K.fx * r * x, K.cy + K.fy * r * y];
}

// Camera-aligned cube faces: forward, right, down axes in the camera frame.
export const FACES = [
  { name: "front", f: [0, 0, 1], r: [1, 0, 0], d: [0, 1, 0] },
  { name: "right", f: [1, 0, 0], r: [0, 0, -1], d: [0, 1, 0] },
  { name: "left", f: [-1, 0, 0], r: [0, 0, 1], d: [0, 1, 0] },
  { name: "down", f: [0, 1, 0], r: [1, 0, 0], d: [0, 0, -1] },
  { name: "up", f: [0, -1, 0], r: [1, 0, 0], d: [0, 0, 1] },
  { name: "back", f: [0, 0, -1], r: [-1, 0, 0], d: [0, 1, 0] },
];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

// The pinhole views that cover a lens at an output size, packed left to right into one atlas, and a lookup
// table that maps every output pixel to its atlas texel.
// Wide lenses use the camera-aligned cube faces they actually see (never the back one at 159 deg), each with a
// frustum cropped to the rays that land on it, at quality x the lens's centre resolution. A pinhole lens is one
// exact view at ss x the output size (ss 2 averages 2x2 through bilinear sampling).
// lut: Float32 RGBA per pixel in GL row order (bottom row first): atlas u, v, range factor (distance along the
// ray per unit of view depth), valid.
export function layout(lens, width, height, { quality = 1, ss = 1, pad = 2 } = {}) {
  const K = intrinsics(lens, width, height);
  const n = width * height, faceOf = new Int8Array(n), A = new Float32Array(n), B = new Float32Array(n), Rf = new Float32Array(n);
  const views = [];
  if (K.model === "pinhole") {
    const v = { ...FACES[0], face: 0, aMin: -K.cx / K.fx, aMax: (width - K.cx) / K.fx, bMin: -K.cy / K.fy, bMax: (height - K.cy) / K.fy, w: width * ss, h: height * ss };
    views.push(v);
    for (let j = 0, i = 0; j < height; j++) for (let x = 0; x < width; x++, i++) {
      const a = (x + 0.5 - K.cx) / K.fx, b = (j + 0.5 - K.cy) / K.fy;
      A[i] = a; B[i] = b; Rf[i] = Math.hypot(a, b, 1);
    }
  } else {
    const box = FACES.map(() => [Infinity, -Infinity, Infinity, -Infinity, 0]);
    for (let j = 0, i = 0; j < height; j++) for (let x = 0; x < width; x++, i++) {
      const d = unproject(K, x + 0.5, j + 0.5);
      if (!d) { faceOf[i] = -1; continue; }
      let k = 0, best = -2;
      for (let q = 0; q < 6; q++) { const c = dot(d, FACES[q].f); if (c > best) { best = c; k = q; } }
      const F = FACES[k], a = dot(d, F.r) / best, b = dot(d, F.d) / best, bx = box[k];
      faceOf[i] = k; A[i] = a; B[i] = b; Rf[i] = 1 / best;
      if (a < bx[0]) bx[0] = a; if (a > bx[1]) bx[1] = a; if (b < bx[2]) bx[2] = b; if (b > bx[3]) bx[3] = b; bx[4]++;
    }
    const fFace = quality * K.fx, m = pad / fFace;
    for (let k = 0; k < 6; k++) {
      const [a0, a1, b0, b1, count] = box[k];
      if (!count) continue;
      const w = Math.ceil((a1 - a0 + 2 * m) * fFace), h = Math.ceil((b1 - b0 + 2 * m) * fFace);
      views.push({ ...FACES[k], face: k, aMin: a0 - m, aMax: a0 - m + w / fFace, bMin: b0 - m, bMax: b0 - m + h / fFace, w, h });
    }
  }
  let x0 = 0;
  for (const v of views) { v.x0 = x0; x0 += v.w + 1; }
  const atlasW = x0 - 1, atlasH = Math.max(...views.map((v) => v.h));
  const byFace = new Map(views.map((v) => [v.face, v]));
  const lut = new Float32Array(n * 4);
  for (let j = 0, i = 0; j < height; j++) for (let x = 0; x < width; x++, i++) {
    const o = ((height - 1 - j) * width + x) * 4;
    if (faceOf[i] < 0) continue;
    const v = byFace.get(faceOf[i]);
    lut[o] = (v.x0 + ((A[i] - v.aMin) / (v.aMax - v.aMin)) * v.w) / atlasW;
    lut[o + 1] = (((v.bMax - B[i]) / (v.bMax - v.bMin)) * v.h) / atlasH;
    lut[o + 2] = Rf[i];
    lut[o + 3] = 1;
  }
  return { K, views, atlasW, atlasH, lut, width, height };
}

// Horizontal and vertical field of view (deg) through the principal point, for UI and docs.
export function fieldOfView(lens, width = 640, height = 480) {
  const K = intrinsics(lens, width, height);
  const ang = (u, v) => { const d = unproject(K, u, v); return d ? Math.acos(d[2]) / DEG : NaN; };
  return { h: ang(0, K.cy) + ang(width, K.cy), v: ang(K.cx, 0) + ang(K.cx, height), diag: ang(0, 0) + ang(width, height) };
}
