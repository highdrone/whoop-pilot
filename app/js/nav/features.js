// Local features and place descriptors for localization against the twin (docs/HOME-DRONE.md, Wave C):
// XFeat (verlab, Apache-2.0; app/vendor/xfeat) keypoints and descriptors, a mutual-nearest-neighbour matcher, keypoints
// lifted to H with the twin's depth, OpenCV.js PnP (RANSAC EPnP + LM) with a covariance, and DINOv2-small whole-image
// descriptors for relocalization. The networks run in features-worker.js (onnxruntime-web, WebGPU, WASM fallback); the
// pure parts below are shared with it and with tools/test-loc.mjs.
import { rodrigues, cameraFromPnP } from "./lens.js";

export const XFEAT = { width: 640, height: 480, url: new URL("../../vendor/xfeat/xfeat_640x480.onnx", import.meta.url).href, size: 4569993,
  sha256: "f99d7dd5fc454067fa59b53e7f184b2f6da3ddf636030ce7430e1276e7558b08" };
// OpenCV.js 4.12.0 (app/vendor/opencv: dist/opencv.js gzipped, 3.5 MB instead of 10.9), checked as unpacked.
export const OPENCV = { url: new URL("../../vendor/opencv/opencv.js.gz", import.meta.url).href, size: 10872779, sha256: "bd0c3e6448043de04f6a64a12cb7b759f78c3ab8f7c35c9f2e0f71c88bb17103" };
export const DINO = { width: 294, height: 224, dim: 768 }; // 21 x 16 patches of 14 px; [CLS, mean patch] of 384 each
export const MIN_COS = 0.82; // XFeat's mutual-NN threshold

// XFeat's 1/8-resolution outputs -> keypoints (pixel indices x, y), scores and unit 64-d descriptors: softmax over the 65
// cell channels (the 65th is "no keypoint"), 5x5 non-maximum suppression on the full-resolution heat, score = heat x
// reliability, the topK best of those allow(x, y) accepts.
export function xfeatPost(feats, logits, rel, W, H, { topK = 2048, thr = 0.05, allow = null } = {}) {
  const w8 = W >> 3, h8 = H >> 3, n8 = w8 * h8, heat = new Float32Array(W * H), e = new Float32Array(65), live = new Uint8Array(n8);
  for (let p = 0; p < n8; p++) {
    let m = -Infinity, s = 0, best = 0;
    for (let c = 0; c < 65; c++) m = Math.max(m, logits[c * n8 + p]);
    for (let c = 0; c < 65; c++) s += e[c] = Math.exp(logits[c * n8 + p] - m);
    const y8 = (p / w8) | 0, x8 = p - y8 * w8;
    for (let c = 0; c < 64; c++) {
      const v = (heat[(y8 * 8 + (c >> 3)) * W + x8 * 8 + (c & 7)] = e[c] / s);
      if (v > best) best = v;
    }
    live[p] = best > thr;
  }
  let score = new Float32Array(4096), xy = new Int32Array(4096), nc = 0;
  for (let p = 0; p < n8; p++) {
    if (!live[p]) continue;
    const y8 = (p / w8) | 0, x8 = p - y8 * w8;
    for (let y = Math.max(2, y8 * 8); y < Math.min(H - 2, y8 * 8 + 8); y++)
      for (let x = Math.max(2, x8 * 8); x < Math.min(W - 2, x8 * 8 + 8); x++) {
        const v = heat[y * W + x];
        if (v <= thr) continue;
        let max = true;
        for (let dy = -2; dy <= 2 && max; dy++) for (let dx = -2; dx <= 2; dx++) if (heat[(y + dy) * W + x + dx] > v) { max = false; break; }
        if (!max || (allow && !allow(x, y))) continue;
        if (nc === score.length) {
          const s2 = new Float32Array(2 * nc), x2 = new Int32Array(2 * nc);
          s2.set(score); x2.set(xy);
          [score, xy] = [s2, x2];
        }
        score[nc] = v * bilinear(rel, w8, h8, x / 8 - 0.5, y / 8 - 0.5, 0);
        xy[nc++] = y * W + x;
      }
  }
  // the topK best: a threshold from a sorted copy, then those in score order
  const n = Math.min(topK, nc), order = Array.from({ length: nc }, (_, i) => i);
  if (nc > n) {
    const cut = Float32Array.from(score.subarray(0, nc)).sort()[nc - n];
    let k = 0;
    for (let i = 0; i < nc; i++) if (score[i] > cut) order[k++] = i;
    for (let i = 0; i < nc && k < n; i++) if (score[i] === cut) order[k++] = i;
    order.length = k;
  }
  order.sort((a, b) => score[b] - score[a]);
  const kpts = new Float32Array(2 * n), scores = new Float32Array(n), desc = new Float32Array(64 * n);
  for (let i = 0; i < n; i++) {
    const o = order[i], x = xy[o] % W, y = (xy[o] / W) | 0;
    kpts[2 * i] = x;
    kpts[2 * i + 1] = y;
    scores[i] = score[o];
    const fx = Math.min(Math.max(x / 8 - 0.5, 0), w8 - 1.001), fy = Math.min(Math.max(y / 8 - 0.5, 0), h8 - 1.001), x0 = fx | 0, y0 = fy | 0;
    const ax = fx - x0, ay = fy - y0, w00 = (1 - ax) * (1 - ay), w01 = ax * (1 - ay), w10 = (1 - ax) * ay, w11 = ax * ay, b = y0 * w8 + x0;
    let nrm = 0;
    for (let c = 0, off = b; c < 64; c++, off += n8) {
      const v = feats[off] * w00 + feats[off + 1] * w01 + feats[off + w8] * w10 + feats[off + w8 + 1] * w11;
      desc[64 * i + c] = v;
      nrm += v * v;
    }
    nrm = 1 / Math.sqrt(nrm + 1e-12);
    for (let c = 0; c < 64; c++) desc[64 * i + c] *= nrm;
  }
  return { n, kpts, scores, desc };
}

function bilinear(a, w, h, x, y, off) {
  x = Math.min(Math.max(x, 0), w - 1.001);
  y = Math.min(Math.max(y, 0), h - 1.001);
  const x0 = x | 0, y0 = y | 0, fx = x - x0, fy = y - y0, i = off + y0 * w + x0;
  return a[i] * (1 - fx) * (1 - fy) + a[i + 1] * fx * (1 - fy) + a[i + w] * (1 - fx) * fy + a[i + w + 1] * fx * fy;
}

// Matcher output (best b for each a with its cosine, best a for each b) -> mutual pairs above minCos: { pairs [ia, ib, ...], cos }.
export function mutual(iab, sab, iba, na, nb, minCos = MIN_COS) {
  const pairs = [], cos = [];
  for (let i = 0; i < na; i++) {
    const j = iab[i];
    if (j < nb && iba[j] === i && sab[i] > minCos) pairs.push(i, j), cos.push(sab[i]);
  }
  return { pairs: Int32Array.from(pairs), cos: Float32Array.from(cos) };
}

// The same on the CPU (tests, and when the matcher cannot run).
export function matchCpu(a, b, minCos = MIN_COS) {
  const iab = new Int32Array(a.n), sab = new Float32Array(a.n).fill(-2), iba = new Int32Array(b.n), sba = new Float32Array(b.n).fill(-2);
  for (let i = 0; i < a.n; i++)
    for (let j = 0; j < b.n; j++) {
      let s = 0;
      for (let c = 0; c < 64; c++) s += a.desc[64 * i + c] * b.desc[64 * j + c];
      if (s > sab[i]) (sab[i] = s), (iab[i] = j);
      if (s > sba[j]) (sba[j] = s), (iba[j] = i);
    }
  return mutual(iab, sab, iba, a.n, b.n, minCos);
}

// ONNX model of the matcher, written here (no file): a [N, D], b [M, D] -> iab int32 [N] (argmax over b), sab [N] (its
// cosine), iba int32 [M] (argmax over a). Fixed shapes, so WebGPU compiles its shaders once; unused rows are zeros.
export function mnnModel(N, M, D = 64) {
  const bytes = [], enc = new TextEncoder();
  const varint = (out, v) => { v = BigInt(v); do { let b = Number(v & 0x7fn); v >>= 7n; if (v) b |= 0x80; out.push(b); } while (v); };
  const field = (out, no, wire, payload) => { varint(out, (no << 3) | wire); if (wire === 0) varint(out, payload); else { varint(out, payload.length); out.push(...payload); } };
  const msg = (f) => { const o = []; f(o); return o; };
  const str = (s) => [...enc.encode(s)];
  const attrInt = (name, v) => msg((o) => { field(o, 1, 2, str(name)); field(o, 3, 0, v); field(o, 20, 0, 2); });
  const attrInts = (name, vs) => msg((o) => { field(o, 1, 2, str(name)); for (const v of vs) field(o, 8, 0, v); field(o, 20, 0, 7); });
  const node = (op, ins, outs, attrs = []) => msg((o) => { ins.forEach((s) => field(o, 1, 2, str(s))); outs.forEach((s) => field(o, 2, 2, str(s))); field(o, 4, 2, str(op)); attrs.forEach((a) => field(o, 5, 2, a)); });
  const value = (name, type, dims) => msg((o) => {
    field(o, 1, 2, str(name));
    field(o, 2, 2, msg((t) => field(t, 1, 2, msg((tt) => { field(tt, 1, 0, type); field(tt, 2, 2, msg((s) => dims.forEach((d) => field(s, 1, 2, msg((dd) => field(dd, 1, 0, d)))))); }))));
  });
  const graph = msg((g) => {
    [node("Transpose", ["b"], ["bt"], [attrInts("perm", [1, 0])]), node("MatMul", ["a", "bt"], ["s"]),
      node("ArgMax", ["s"], ["iab64"], [attrInt("axis", 1), attrInt("keepdims", 0)]), node("ReduceMax", ["s"], ["sab"], [attrInts("axes", [1]), attrInt("keepdims", 0)]),
      node("ArgMax", ["s"], ["iba64"], [attrInt("axis", 0), attrInt("keepdims", 0)]),
      node("Cast", ["iab64"], ["iab"], [attrInt("to", 6)]), node("Cast", ["iba64"], ["iba"], [attrInt("to", 6)])].forEach((n) => field(g, 1, 2, n));
    field(g, 2, 2, str("mnn"));
    field(g, 11, 2, value("a", 1, [N, D]));
    field(g, 11, 2, value("b", 1, [M, D]));
    field(g, 12, 2, value("iab", 6, [N]));
    field(g, 12, 2, value("sab", 1, [N]));
    field(g, 12, 2, value("iba", 6, [M]));
  });
  field(bytes, 1, 0, 8);
  field(bytes, 7, 2, graph);
  field(bytes, 8, 2, msg((o) => { field(o, 1, 2, []); field(o, 2, 0, 17); }));
  return Uint8Array.from(bytes);
}

// Keypoints of a pinhole render (pixel indices) -> H points through its depth (metres along each ray, NaN where the
// splats are thin; Float32Array rows top-down, W x H) and camera { R (columns: camera axes in H), C }; NaN where the depth
// is missing or jumps (an edge: the keypoint could sit on either side). s: the depth's size over the keypoints' (0.5: a
// half-size depth render; the keypoint's own ray takes its pixel's depth).
export function lift(kpts, n, depth, W, H, K, { R, C }, s = 1) {
  const P = new Float32Array(3 * n).fill(NaN);
  for (let i = 0; i < n; i++) {
    const x = kpts[2 * i], y = kpts[2 * i + 1], dx0 = Math.min(W - 1, ((x + 0.5) * s) | 0), dy0 = Math.min(H - 1, ((y + 0.5) * s) | 0), d = depth[dy0 * W + dx0];
    if (!(d > 0.05 && d < 20)) continue;
    let lo = d, hi = d;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      const v = depth[Math.min(H - 1, Math.max(0, dy0 + dy)) * W + Math.min(W - 1, Math.max(0, dx0 + dx))];
      lo = Math.min(lo, v); hi = Math.max(hi, v);
    }
    if (!(hi - lo < 0.03 + 0.06 * d)) continue;
    const a = (x + 0.5 - K.cx) / K.fx, b = (y + 0.5 - K.cy) / K.fy, r = d / Math.hypot(a, b, 1), c = [a * r, b * r, r];
    for (let k = 0; k < 3; k++) P[3 * i + k] = C[k] + R[k][0] * c[0] + R[k][1] * c[1] + R[k][2] * c[2];
  }
  return P;
}

// Reprojection residuals of H points obj (3n) against pixels img (2n) for a pinhole camera (R, C).
function reproject(K, R, C, obj, img, idx) {
  const r = new Float64Array(2 * idx.length);
  idx.forEach((i, k) => {
    const d = [obj[3 * i] - C[0], obj[3 * i + 1] - C[1], obj[3 * i + 2] - C[2]];
    const x = R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2], y = R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2], z = R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2];
    r[2 * k] = z > 1e-6 ? K.cx + (K.fx * x) / z - img[2 * i] : 1e3;
    r[2 * k + 1] = z > 1e-6 ? K.cy + (K.fy * y) / z - img[2 * i + 1] : 1e3;
  });
  return r;
}

const mul3 = (A, B) => A.map((row) => [0, 1, 2].map((j) => row[0] * B[0][j] + row[1] * B[1][j] + row[2] * B[2][j]));

// Covariance of the camera centre (H, m^2) and its yaw (about H z, rad^2) from the reprojection Jacobian of the inliers:
// s^2 (J^T J)^-1 over [C, rotations about H x, y, z], s the residual RMS (at least 0.5 px).
export function pnpCovariance(K, R, C, obj, img, idx) {
  const r0 = reproject(K, R, C, obj, img, idx), m = r0.length, J = [];
  for (let p = 0; p < 6; p++) {
    const h = p < 3 ? 1e-4 : 1e-5, d = [0, 0, 0, 0, 0, 0];
    d[p] = h;
    const R1 = mul3(rodrigues(d.slice(3)), R), C1 = [C[0] + d[0], C[1] + d[1], C[2] + d[2]];
    J.push(reproject(K, R1, C1, obj, img, idx).map((v, i) => (v - r0[i]) / h));
  }
  const A = J.map((a) => J.map((b) => a.reduce((s, v, i) => s + v * b[i], 0)));
  const rms = Math.sqrt(r0.reduce((s, v) => s + v * v, 0) / Math.max(1, m)), s2 = Math.max(0.5, rms) ** 2, inv = invert(A);
  return { rms, cov: inv ? inv.map((row) => row.map((v) => v * s2)) : null };
}

function invert(A) {
  const n = A.length, M = A.map((r, i) => [...r, ...r.map((_, j) => (i === j ? 1 : 0))]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c];
    if (Math.abs(d) < 1e-18) return null;
    for (let j = 0; j < 2 * n; j++) M[c][j] /= d;
    for (let r = 0; r < n; r++) if (r !== c && M[r][c]) for (let j = 0, f = M[r][c]; j < 2 * n; j++) M[r][j] -= f * M[c][j];
  }
  return M.map((r) => r.slice(n));
}

// OpenCV.js -> PnP on H points obj (Float64 3n) and pixels img (Float64 2n, continuous: pixel centres at +0.5) for a
// pinhole K. RANSAC EPnP, LM refinement on its inliers, then the inliers again at thr. Returns null or { R, C, inliers (indices), rms, cov, ransacInliers }.
export function solvePnP(cv, obj, img, K, { thr = 4, iters = 400, conf = 0.999 } = {}) {
  const n = obj.length / 3;
  if (n < 6) return null;
  const mats = [], keep = (m) => (mats.push(m), m);
  try {
    const o = keep(cv.matFromArray(n, 1, cv.CV_64FC3, Array.from(obj))), p = keep(cv.matFromArray(n, 1, cv.CV_64FC2, Array.from(img)));
    const Km = keep(cv.matFromArray(3, 3, cv.CV_64F, [K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0, 1])), dist = keep(cv.Mat.zeros(4, 1, cv.CV_64F));
    const rvec = keep(new cv.Mat()), tvec = keep(new cv.Mat()), inl = keep(new cv.Mat());
    let ok = false;
    try {
      ok = cv.solvePnPRansac(o, p, Km, dist, rvec, tvec, false, iters, thr, conf, inl, cv.SOLVEPNP_EPNP);
    } catch {
      ok = false;
    }
    if (!ok || inl.rows < 6) return null;
    const first = Array.from(inl.data32S.slice(0, inl.rows));
    const sub = (idx, src, k) => idx.flatMap((i) => Array.from(src.subarray(k * i, k * i + k)));
    const o2 = keep(cv.matFromArray(first.length, 1, cv.CV_64FC3, sub(first, obj, 3))), p2 = keep(cv.matFromArray(first.length, 1, cv.CV_64FC2, sub(first, img, 2)));
    try {
      cv.solvePnPRefineLM(o2, p2, Km, dist, rvec, tvec);
    } catch {}
    let { R, C } = cameraFromPnP(Array.from(rvec.data64F), Array.from(tvec.data64F));
    const all = Array.from({ length: n }, (_, i) => i), res = reproject(K, R, C, obj, img, all);
    const inliers = all.filter((i) => Math.hypot(res[2 * i], res[2 * i + 1]) < thr);
    if (inliers.length < 6) return null;
    const { rms, cov } = pnpCovariance(K, R, C, obj, img, inliers);
    return { R, C, inliers, rms, cov, ransacInliers: first.length };
  } finally {
    for (const m of mats) m.delete();
  }
}

// The vendored opencv.js.gz's bytes -> the script's text, unpacked (DecompressionStream: pages, workers and Node alike)
// and checked against OPENCV's size and SHA-256.
export async function openCvText(gz) {
  const buf = await new Response(new Blob([gz]).stream().pipeThrough(new DecompressionStream("gzip"))).arrayBuffer();
  const sum = Array.from(new Uint8Array(await crypto.subtle.digest("SHA-256", buf)), (b) => b.toString(16).padStart(2, "0")).join("");
  if (buf.byteLength !== OPENCV.size || sum !== OPENCV.sha256) throw new Error("OpenCV.js: checksum mismatch");
  return new TextDecoder().decode(buf);
}

// Loads OpenCV.js from its text: in a module worker (no importScripts) and in Node alike, as CommonJS (Node: pass its
// require and the file's directory, which Emscripten's Node path reads). -> { cv }: the Emscripten module is a thenable
// that resolves to itself, so it must not be returned from an async function bare.
export async function loadOpenCV(text, { require, dirname = "" } = {}) {
  const module = { exports: {} };
  new Function("module", "exports", "require", "__dirname", text)(module, module.exports, require, dirname); // eslint-disable-line no-new-func
  const cv = module.exports;
  if (!cv.Mat) await new Promise((resolve) => (cv.onRuntimeInitialized = resolve));
  return { cv };
}

// ---------------------------------------------------------------- DINOv2 place descriptors

const MEAN = [0.485, 0.456, 0.406], STD = [0.229, 0.224, 0.225];
// RGBA w x h -> pixel_values [1, 3, 224, 294]: bilinear resize, ImageNet normalisation.
export function dinoInput(rgba, w, h, out = new Float32Array(3 * DINO.width * DINO.height)) {
  const W = DINO.width, H = DINO.height, n = W * H;
  for (let y = 0; y < H; y++) {
    const sy = Math.min(h - 1.001, Math.max(0, ((y + 0.5) * h) / H - 0.5)), y0 = sy | 0, fy = sy - y0;
    for (let x = 0; x < W; x++) {
      const sx = Math.min(w - 1.001, Math.max(0, ((x + 0.5) * w) / W - 0.5)), x0 = sx | 0, fx = sx - x0, i = (y0 * w + x0) * 4, j = y * W + x;
      for (let c = 0; c < 3; c++) {
        const v = (1 - fy) * ((1 - fx) * rgba[i + c] + fx * rgba[i + 4 + c]) + fy * ((1 - fx) * rgba[i + 4 * w + c] + fx * rgba[i + 4 * w + 4 + c]);
        out[c * n + j] = (v / 255 - MEAN[c]) / STD[c];
      }
    }
  }
  return out;
}

// last_hidden_state [1, T, D] -> unit [CLS / |CLS|, mean patch / |mean patch|] / sqrt 2 (so a dot product is in -1..1).
export function dinoDescriptor(hidden, T, D) {
  const out = new Float32Array(2 * D);
  for (let k = 0; k < D; k++) out[k] = hidden[k];
  for (let t = 1; t < T; t++) for (let k = 0; k < D; k++) out[D + k] += hidden[t * D + k];
  for (const [a, b] of [[0, D], [D, 2 * D]]) {
    let s = 0;
    for (let k = a; k < b; k++) s += out[k] * out[k];
    s = 1 / Math.sqrt(2 * s + 1e-12);
    for (let k = a; k < b; k++) out[k] *= s;
  }
  return out;
}

// ---------------------------------------------------------------- client

// Per-call limits (ms): a worker that hangs (GPU stall, device loss) must not hold the localizer forever.
export const CALL_MS = { init: 120000, loadDino: 600000, global: 15000, default: 6000 };

// const f = await Features.create({ backend: "auto" | "webgpu" | "wasm" });
// Images go to the worker (transferred) and stay there under an id; matching and PnP refer to ids. DINOv2 (relocalization
// only) is loaded on demand: loadDino(), with onProgress({ what, loaded, total }) on its first download (44 MB, then
// cached); tracking never waits for it.
export class Features {
  #worker;
  #seq = 0;
  #pending = new Map();
  info = {};
  onProgress = null;
  timeouts = 0; // calls that timed out in a row

  static async create({ backend = "auto", dino = false, onProgress, workerUrl = new URL("./features-worker.js", import.meta.url) } = {}) {
    const f = new Features(), w = (f.#worker = new Worker(workerUrl, { type: "module" }));
    f.onProgress = onProgress ?? null;
    w.onmessage = ({ data: { id, result, error, progress } }) => {
      if (progress) return f.onProgress?.(progress);
      const p = f.#pending.get(id);
      f.#pending.delete(id);
      clearTimeout(p?.timer);
      if (error) p?.reject(new Error(error)); else p?.resolve(result);
    };
    w.onerror = (e) => f.#failAll(new Error(e.message || "features worker failed"));
    try {
      f.info = await f.#call("init", { backend });
      if (dino) await f.loadDino();
    } catch (e) {
      f.dispose();
      throw e;
    }
    return f;
  }

  // DINOv2 for relocalization (once). -> { dino, ms }.
  async loadDino() {
    if (this.info.dinoLoaded) return this.info;
    const r = await (this.dinoLoading ??= this.#call("loadDino", {}).finally(() => (this.dinoLoading = null)));
    return Object.assign(this.info, r, { dinoLoaded: true });
  }
  // Rectifier for a source frame size: { key, lens, srcW, srcH, outW, outH, hfov }.
  rectifier(spec) { return this.#call("rectifier", spec); }
  // Mask (OsdMask.mask()) for a rectifier: { key, width, height, data }.
  setMask(spec) { return this.#call("setMask", spec); }
  // A live frame: { image: ImageBitmap | { width, height, data }, rect: key, boxes: [{x, y, w, h}] (source-normalised),
  // topK, dino, keep } -> { id, n, kpts, ms, desc? }.
  frame(args) { return this.#call("frame", args, args.image instanceof ImageBitmap ? [args.image] : args.image?.data ? [args.image.data.buffer] : []); }
  // A twin render (pinhole): { rgba: { width, height, data }, depth, depthW?, depthH? (a smaller depth render), K, cam: { R, C }, topK } -> { id, n, lifted, ms }.
  render(args) { return this.#call("render", args, [args.rgba.data.buffer, ...(args.depth ? [args.depth.buffer] : [])]); }
  // Match a live frame with a render and solve the camera: { live, ref, K, thr, iters, minCos, viz } -> PnP result.
  solve(args) { return this.#call("solve", args); }
  // DINOv2 descriptor of a stored frame (id) or an image ({ width, height, data }).
  global(args) { return this.#call("global", args, args.rgba ? [args.rgba.data.buffer] : []); }
  // Fisheye correspondences of a solve, for lens calibration: { live, ref } -> { uv, xyz }.
  calibSample(args) { return this.#call("calibSample", args); }
  release(ids) { return this.#call("release", ids); }
  rectified(id) { return this.#call("rectified", id); }

  get alive() {
    return !!this.#worker;
  }

  dispose() {
    this.#worker?.terminate();
    this.#worker = null;
    this.#failAll(new Error("features disposed"));
  }

  #call(op, args, transfer = []) {
    if (!this.#worker) return Promise.reject(new Error("features disposed"));
    return new Promise((resolve, reject) => {
      const id = ++this.#seq, ms = CALL_MS[op] ?? CALL_MS.default;
      const timer = setTimeout(() => {
        this.#pending.delete(id);
        this.timeouts++;
        reject(Object.assign(new Error(`the vision worker didn't answer (${op}, ${ms / 1000} s)`), { timeout: true }));
      }, ms);
      this.#pending.set(id, { resolve: (v) => ((this.timeouts = 0), resolve(v)), reject, timer });
      this.#worker.postMessage({ id, op, args }, transfer);
    });
  }

  #failAll(err) {
    for (const p of this.#pending.values()) (clearTimeout(p.timer), p.reject(err));
    this.#pending.clear();
  }
}
