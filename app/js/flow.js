// Global image motion from the FPV feed: sparse pyramidal Lucas-Kanade on a grid of points, then a
// robust fit of shift + zoom + rotation. The flight controller turns this into drift, climb/sink and
// "about to hit something" (time-to-contact) estimates.
//
// Drift at hover moves the image by a fraction of a pixel per frame, so each frame is compared with
// the one from ~100 ms earlier (not the previous frame) to get a usable signal.

const GRID_X = 16;
const GRID_Y = 9;
const WIN = 4; // half window (9x9)
const ITER = 6;
const LEVELS = 3;
const BASELINE = 0.1; // seconds between compared frames

export class FlowEstimator {
  constructor(width, height) {
    this.w = width;
    this.h = height;
    this.history = []; // { t, pyr }
  }

  // gray: Float32Array (w*h), t: ms. focusY: where the horizon straight ahead is (0..1 of height):
  // forward motion expands the image around that point. Returns image motion rates:
  // dx, dy = fraction of image width/height per second, div = zoom rate (1/s), rot = rad/s.
  update(gray, t, focusY = 0.5) {
    const pyr = [{ g: gray, w: this.w, h: this.h }];
    for (let l = 1; l < LEVELS; l++) pyr.push(downsample(pyr[l - 1]));
    const hist = this.history;
    hist.push({ t, pyr });
    while (hist.length > 1 && t - hist[1].t >= BASELINE) hist.shift();
    if (hist.length > 12) hist.shift();
    const ref = hist[0];
    const span = (t - ref.t) / 1000;
    if (ref === hist[hist.length - 1] || span <= 0.02) return null;

    const { w, h } = this;
    const a = ref.pyr[0].g;
    const cx = w / 2;
    const cy = focusY * h;
    const pts = [];
    const x0 = w * 0.06;
    const y0 = h * 0.08;
    const sx = (w * 0.88) / (GRID_X - 1);
    const sy = (h * 0.84) / (GRID_Y - 1);
    for (let j = 0; j < GRID_Y; j++) {
      for (let i = 0; i < GRID_X; i++) {
        const px = x0 + i * sx;
        const py = y0 + j * sy;
        if (!textured(a, w, h, px, py)) continue;
        let u = 0;
        let v = 0;
        let ok = true;
        for (let l = LEVELS - 1; l >= 0 && ok; l--) {
          const s = 1 << l;
          const r = track(ref.pyr[l], pyr[l], px / s, py / s, u / s, v / s);
          if (!r) ok = false;
          else {
            u = r[0] * s;
            v = r[1] * s;
          }
        }
        if (ok) pts.push({ x: px - cx, y: py - cy, u, v });
      }
    }
    const fit = robustFit(pts);
    if (!fit) return { dx: 0, dy: 0, div: 0, rot: 0, quality: 0, points: pts.length, span };
    return {
      dx: fit.tx / w / span,
      dy: fit.ty / h / span,
      div: fit.s / span,
      rot: fit.r / span,
      quality: Math.min(1, fit.inliers / 40) * fit.inlierRatio,
      points: fit.inliers,
      span,
    };
  }
}

function downsample({ g, w, h }) {
  const w2 = w >> 1;
  const h2 = h >> 1;
  const out = new Float32Array(w2 * h2);
  for (let y = 0; y < h2; y++) {
    for (let x = 0; x < w2; x++) {
      const i = y * 2 * w + x * 2;
      out[y * w2 + x] = (g[i] + g[i + 1] + g[i + w] + g[i + w + 1]) * 0.25;
    }
  }
  return { g: out, w: w2, h: h2 };
}

// Enough gradient in both directions to track (min eigenvalue of the structure tensor).
function textured(g, w, h, cx, cy) {
  let sxx = 0;
  let syy = 0;
  let sxy = 0;
  const x0 = Math.round(cx);
  const y0 = Math.round(cy);
  for (let y = y0 - WIN; y <= y0 + WIN; y++) {
    for (let x = x0 - WIN; x <= x0 + WIN; x++) {
      if (x < 1 || y < 1 || x >= w - 1 || y >= h - 1) return false;
      const gx = (g[y * w + x + 1] - g[y * w + x - 1]) * 0.5;
      const gy = (g[(y + 1) * w + x] - g[(y - 1) * w + x]) * 0.5;
      sxx += gx * gx;
      syy += gy * gy;
      sxy += gx * gy;
    }
  }
  const tr = sxx + syy;
  const det = sxx * syy - sxy * sxy;
  const minEig = tr / 2 - Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
  return minEig > 250;
}

function sample(g, w, h, x, y) {
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x > w - 1.001) x = w - 1.001;
  if (y > h - 1.001) y = h - 1.001;
  const xi = x | 0;
  const yi = y | 0;
  const fx = x - xi;
  const fy = y - yi;
  const i = yi * w + xi;
  return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
}

const N = (2 * WIN + 1) * (2 * WIN + 1);
const GX = new Float32Array(N);
const GY = new Float32Array(N);
const IA = new Float32Array(N);

// Lucas-Kanade for one point on one pyramid level. Returns [u, v] displacement or null.
function track(A, B, px, py, u, v) {
  const { g: a, w, h } = A;
  const b = B.g;
  let sxx = 0;
  let syy = 0;
  let sxy = 0;
  let k = 0;
  for (let dy = -WIN; dy <= WIN; dy++) {
    for (let dx = -WIN; dx <= WIN; dx++) {
      const x = px + dx;
      const y = py + dy;
      const ex = (sample(a, w, h, x + 1, y) - sample(a, w, h, x - 1, y)) * 0.5;
      const ey = (sample(a, w, h, x, y + 1) - sample(a, w, h, x, y - 1)) * 0.5;
      GX[k] = ex;
      GY[k] = ey;
      IA[k] = sample(a, w, h, x, y);
      sxx += ex * ex;
      syy += ey * ey;
      sxy += ex * ey;
      k++;
    }
  }
  const det = sxx * syy - sxy * sxy;
  if (det < 1e-3) return null;
  for (let it = 0; it < ITER; it++) {
    let bx = 0;
    let by = 0;
    k = 0;
    for (let dy = -WIN; dy <= WIN; dy++) {
      for (let dx = -WIN; dx <= WIN; dx++) {
        const e = sample(b, w, h, px + dx + u, py + dy + v) - IA[k];
        bx += e * GX[k];
        by += e * GY[k];
        k++;
      }
    }
    const du = -(syy * bx - sxy * by) / det;
    const dv = -(sxx * by - sxy * bx) / det;
    u += du;
    v += dv;
    if (Math.abs(u) > w * 0.3 || Math.abs(v) > h * 0.3) return null;
    if (du * du + dv * dv < 0.0002) break;
  }
  return [u, v];
}

// Fit displacement = t + s*p + r*perp(p) (zoom s and rotation r about the focus point) with outlier
// rejection.
function robustFit(pts) {
  if (pts.length < 8) return null;
  let model = lsq(pts);
  let inl = pts;
  for (let pass = 0; pass < 3 && model; pass++) {
    const res = pts.map((p) => residual(model, p));
    const sorted = [...res].sort((x, y) => x - y);
    const thr = Math.max(0.25, sorted[Math.floor(sorted.length * 0.6)] * 2.5);
    inl = pts.filter((_, i) => res[i] <= thr);
    if (inl.length < 8) return null;
    model = lsq(inl);
  }
  if (!model) return null;
  return { ...model, inliers: inl.length, inlierRatio: inl.length / pts.length };
}

function residual(m, p) {
  const eu = p.u - (m.tx + m.s * p.x - m.r * p.y);
  const ev = p.v - (m.ty + m.s * p.y + m.r * p.x);
  return Math.hypot(eu, ev);
}

// Linear least squares for [tx, ty, s, r] (normal equations, 4x4).
function lsq(pts) {
  const A = Array.from({ length: 4 }, () => new Float64Array(4));
  const b = new Float64Array(4);
  const add = (r0, r1, r2, r3, val) => {
    const row = [r0, r1, r2, r3];
    for (let i = 0; i < 4; i++) {
      b[i] += row[i] * val;
      for (let j = 0; j < 4; j++) A[i][j] += row[i] * row[j];
    }
  };
  for (const p of pts) {
    add(1, 0, p.x, -p.y, p.u);
    add(0, 1, p.y, p.x, p.v);
  }
  const x = solve4(A, b);
  return x ? { tx: x[0], ty: x[1], s: x[2], r: x[3] } : null;
}

function solve4(A, b) {
  const n = 4;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-9) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}
