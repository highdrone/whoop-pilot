// Live metric depth from the drone camera (docs/HOME-DRONE.md, "Wave C contracts"): Depth Anything V2 Small (Apache-2.0,
// code and weights; its Base and Large models are CC BY-NC 4.0, never those) on onnxruntime-web WebGPU in a module worker
// (depth-worker.js), on the camera rectified to nav/lens.js's pinhole view (100° wide, the camera's own axes), 280x210,
// about 4 Hz, one frame in flight. The model's output is relative (affine-invariant inverse depth), so every frame is
// aligned (in the worker too, off the control loop's thread) against the twin's empty-house depth at the localized pose
// ({ actors: false, props: false }, the same view):
// RANSAC on pixel pairs, then weighted least squares on the pixels that agree within DEPTH.agree; the pixels that don't
// agree are where the house differs (or someone stands), and keep a metric depth from the fit. Confidence per pixel from
// the fit's spread at that depth. The camera frame goes to the worker as it is (its aspect kept): the camera's picture
// only, found inside the video region when the goggles add black bars around it (PictureFinder: the O4's 4:3 picture in
// the Goggles 3's 1920x1080 stream); a picture that is neither the lens's aspect nor a 16:9 crop of it isn't used (status
// "Live depth is off: ..."). One frame is in flight at a time, from the camera grab to the aligned frame.
// LiveDepth emits "depth" frames for nav/avoid.js and nav/changes.js:
// { t, pose (camera: the body pose with z at the camera, pitch, roll, sigma, status), width, height, lens, depth (m along
// each ray), conf, expected, expectedRgb (the twin's empty-house picture), rgb ({ width, height, data }: the rectified
// live picture), hires (the same at DEPTH.hires times the size: evidence pictures), boxes (people and pets in this view),
// mask? (1 where the camera frame is the OSD, nav/osdmask.js), grid (nav/changes.js differences() of this frame, made in
// the worker: avoid and the change detector read it), stats: { modelMs, twinMs, alignMs, gridMs, absRel, inliers } }.
import { Emitter } from "../util.js";
import { intrinsics, unproject, project } from "../twin/lens.js";
import { rectLens, droneLens, rectifyMap, RECT_HFOV, CAM_DZ } from "../nav/lens.js";
import { GPU } from "./budget.js";
import { OSD_MAX } from "../nav/osdmask.js";

const HF = "https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/4472b7362082ad9968fee890ca0f1e5aca36b93d/onnx";
export const DEPTH_MODELS = { // keys: settings "depthModel"
  dav2s: { name: "Depth Anything V2 Small", licence: "Apache-2.0", url: `${HF}/model.onnx`, size: 99060839, sha256: "afb6a5c28f3b6bf1618c6e43f02073ef9dfdc70e937502d51603e57b0a1df10c" },
  "dav2s-fp16": { name: "Depth Anything V2 Small (fp16)", licence: "Apache-2.0", url: `${HF}/model_fp16.onnx`, size: 49642442, sha256: "2df6223f206b5164e21f664ace61dabeb9bb6a49b8b5a3e00510b4807d0f5b04" },
};

export const DEPTH = {
  model: "dav2s",
  width: 280, height: 210, // multiples of 14 (the ViT patch), 4:3
  src: [512, 384], // the camera's picture goes to the worker this wide (its own aspect: 384 high at 4:3, 288 at 16:9)
  hfov: RECT_HFOV,
  hz: 4,
  hires: 2, // the rectified picture also at this scale (560x420 from the 512x384 frame: the evidence of a change)
  agree: 0.1, // relative depth difference within which a pixel agrees with the twin
  iters: 96, sample: 1500, // RANSAC
  near: 0.25, far: 8, // m: expected depths used for the fit
  osdMax: OSD_MAX, // share of the picture an OSD mask may hide
  staleMs: 100, // a frame decoded longer ago than this is too old to time well (nav/splatloc.js SPLAT.staleMs)
};

// ImageNet mean/std, as one lookup per channel (the DPT preprocessor's), RGBA -> planar float32.
const LUT = [[0.485, 0.229], [0.456, 0.224], [0.406, 0.225]].map(([m, s]) => Float32Array.from({ length: 256 }, (_, v) => (v / 255 - m) / s));
export function toInput(rgba, n, out = new Float32Array(3 * n)) {
  const [r, g, b] = LUT;
  for (let i = 0, j = 0; i < n; i++, j += 4) [out[i], out[n + i], out[2 * n + i]] = [r[rgba[j]], g[rgba[j + 1]], b[rgba[j + 2]]];
  return out;
}

// Length of each pixel's ray per unit of depth along the optical axis (range = z x factor).
const factorCache = new Map();
function rayFactors(K) {
  const key = `${K.width}x${K.height}|${K.fx}|${K.cx}|${K.cy}`;
  if (!factorCache.has(key)) {
    const f = new Float32Array(K.width * K.height);
    for (let v = 0, i = 0; v < K.height; v++) for (let u = 0; u < K.width; u++, i++) f[i] = Math.hypot((u + 0.5 - K.cx) / K.fx, (v + 0.5 - K.cy) / K.fy, 1);
    factorCache.set(key, f);
  }
  return factorCache.get(key);
}

// disp: the model's output (bigger is nearer), K.width x K.height; expected: the twin's range along each ray (m, NaN where
// it has none); K: the pinhole view's intrinsics. disp ≈ s / z + t, z the depth along the optical axis. -> { depth (range,
// m, NaN beyond the fit), conf (0-1), scale, shift, inliers (share of the usable pixels), absRel (median |z - z_twin| / z_twin
// over the usable pixels), n } | null (too little of the view to align with).
export function alignDepth(disp, expected, K, { agree = DEPTH.agree, iters = DEPTH.iters, sample = DEPTH.sample, rand = Math.random } = {}) {
  const n = K.width * K.height, f = rayFactors(K), idx = [];
  for (let i = 0; i < n; i++) {
    const z = expected[i] / f[i];
    if (z >= DEPTH.near && z <= DEPTH.far && Number.isFinite(disp[i])) idx.push(i);
  }
  if (idx.length < 100) return null;
  const zOf = (i) => expected[i] / f[i], S = Array.from({ length: Math.min(sample, idx.length) }, () => idx[(rand() * idx.length) | 0]);
  const agrees = (i, s, t) => {
    const q = disp[i] - t;
    return q > 0 && Math.abs(s / q - zOf(i)) < agree * zOf(i);
  };
  let best = null, most = -1;
  for (let k = 0; k < iters; k++) {
    const a = S[(rand() * S.length) | 0], b = S[(rand() * S.length) | 0], ua = 1 / zOf(a), ub = 1 / zOf(b);
    if (Math.abs(ua - ub) < 0.05 * Math.max(ua, ub)) continue;
    const s = (disp[a] - disp[b]) / (ua - ub), t = disp[a] - s * ua;
    if (!(s > 0)) continue;
    let c = 0;
    for (const i of S) c += agrees(i, s, t) ? 1 : 0;
    if (c > most) [most, best] = [c, { s, t }];
  }
  if (!best) return null;
  // Weighted least squares (weights z²: about the relative depth error) on every pixel that agrees, twice.
  let { s, t } = best, inl = 0;
  for (let round = 0; round < 2; round++) {
    let W = 0, Su = 0, Sd = 0, Suu = 0, Sud = 0;
    inl = 0;
    for (const i of idx) {
      if (!agrees(i, s, t)) continue;
      const z = zOf(i), w = z * z, u = 1 / z;
      [W, Su, Sd, Suu, Sud, inl] = [W + w, Su + w * u, Sd + w * disp[i], Suu + w * u * u, Sud + w * u * disp[i], inl + 1];
    }
    const det = W * Suu - Su * Su;
    if (inl < 50 || !(Math.abs(det) > 1e-12)) break;
    const s2 = (W * Sud - Su * Sd) / det;
    if (!(s2 > 0)) break;
    [s, t] = [s2, (Sd - s2 * Su) / W];
  }
  const res = [], rel = [];
  for (const i of idx) {
    const z = zOf(i), q = disp[i] - t;
    if (agrees(i, s, t)) res.push(Math.abs(disp[i] - (s / z + t)));
    rel.push(q > 0 ? Math.abs(s / q - z) / z : 1);
  }
  const med = (a) => a.sort((x, y) => x - y)[a.length >> 1] ?? NaN, spread = 1.4826 * med(res);
  const depth = new Float32Array(n), conf = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    const q = disp[i] - t, z = q > 0 ? s / q : NaN;
    depth[i] = z > 0.05 && z < 30 ? z * f[i] : NaN;
    conf[i] = depth[i] > 0 ? 1 / (1 + ((spread * z) / s / agree) ** 2) : 0;
  }
  return { depth, conf, scale: s, shift: t, inliers: inl / idx.length, absRel: med(rel), n: idx.length };
}

// A depth frame's depth for the 3D map (VoxelMap.integrate): monocular depth can read a new thing 30-100% too far, which
// would carve free space through it and past what the scan saw, so a pixel only counts where it isn't farther than the
// twin expects (by DEPTH.agree): agreeing ones confirm the scan, nearer ones mark something new, none carves past a surface.
export function mapDepth(f) {
  const out = new Float32Array(f.depth.length);
  for (let i = 0; i < out.length; i++) out[i] = f.depth[i] <= f.expected[i] * (1 + DEPTH.agree) ? f.depth[i] : NaN;
  return out;
}

// Normalised boxes { x, y, w, h } in the source (fisheye) frame -> the same in the rectified view (clipped), or null.
export function boxToView(b, Kf, Kp) {
  const pts = [];
  for (let k = 0; k <= 4; k++)
    for (const [u, v] of [[b.x + (b.w * k) / 4, b.y], [b.x + (b.w * k) / 4, b.y + b.h], [b.x, b.y + (b.h * k) / 4], [b.x + b.w, b.y + (b.h * k) / 4]]) {
      const r = unproject(Kf, u * Kf.width, v * Kf.height), p = r && r[2] > 0.05 && project(Kp, r);
      if (p) pts.push([p[0] / Kp.width, p[1] / Kp.height]);
    }
  if (!pts.length) return null;
  const x0 = Math.max(0, Math.min(...pts.map((p) => p[0]))), y0 = Math.max(0, Math.min(...pts.map((p) => p[1])));
  const x1 = Math.min(1, Math.max(...pts.map((p) => p[0]))), y1 = Math.min(1, Math.max(...pts.map((p) => p[1])));
  return x1 > x0 && y1 > y0 ? { x: x0, y: y0, w: x1 - x0, h: y1 - y0 } : null;
}

// Pixels of the rectified view that come from unusable camera pixels: usable (1 = scene) at mw x mh over the camera frame
// (an OsdMask's mask()) -> Uint8Array W x H, 1 = ignore.
export function viewMask(usable, mw, mh, lens, W = DEPTH.width, H = DEPTH.height, hfov = DEPTH.hfov) {
  const { map } = rectifyMap(lens, mw, mh, W, H, hfov), out = new Uint8Array(W * H);
  for (let i = 0; i < W * H; i++) {
    const x = map[2 * i], y = map[2 * i + 1];
    out[i] = Number.isFinite(x) && usable[Math.min(mh - 1, y | 0) * mw + Math.min(mw - 1, x | 0)] ? 0 : 1;
  }
  return out;
}

// The camera's picture within the video: dark bars at the sides (the O4's 4:3 picture pillarboxed in the goggles'
// 16:9) or at the top and bottom, learnt over `frames` sampled frames (OSD text in the bars is fine: a bar column is one
// whose pixels are mostly dark in nearly every frame). add(gray, w, h); rect() -> the picture { x, y, w, h } (normalised
// in the region) once ready, or null when the whole region looks dark (a dark room: can't tell).
export class PictureFinder {
  constructor({ frames = 20, dark = 28, share = 0.85, steady = 0.9 } = {}) {
    Object.assign(this, { need: frames, dark, share, steady, n: 0, cols: null, rows: null, size: null });
  }
  get ready() {
    return this.n >= this.need;
  }
  add(gray, w, h) {
    if (this.size?.[0] !== w || this.size?.[1] !== h) Object.assign(this, { size: [w, h], n: 0, cols: new Uint16Array(w), rows: new Uint16Array(h) });
    const cd = new Uint16Array(w), rd = new Uint16Array(h);
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) if (gray[y * w + x] < this.dark) (cd[x]++, rd[y]++);
    for (let x = 0; x < w; x++) this.cols[x] += cd[x] >= this.share * h;
    for (let y = 0; y < h; y++) this.rows[y] += rd[y] >= this.share * w;
    this.n++;
  }
  rect() {
    if (!this.ready) return null;
    const bar = (v) => v >= this.steady * this.n, [w, h] = this.size, edge = (a, L) => {
      let lo = 0, hi = L;
      while (lo < L && bar(a[lo])) lo++;
      while (hi > lo && bar(a[hi - 1])) hi--;
      return [lo, hi];
    };
    const [x0, x1] = edge(this.cols, w), [y0, y1] = edge(this.rows, h);
    return x1 - x0 < 0.3 * w || y1 - y0 < 0.3 * h ? null : { x: x0 / w, y: y0 / h, w: (x1 - x0) / w, h: (y1 - y0) / h };
  }
}

// Grey levels of a video element's region at w x h (a canvas draw), or null without a canvas (Node).
export function sampleGray(element, r, w = 96, h = 54) {
  if (typeof OffscreenCanvas === "undefined" || !element) return null;
  const g = (sampleGray.ctx ??= new OffscreenCanvas(w, h).getContext("2d", { willReadFrequently: true }));
  if (g.canvas.width !== w || g.canvas.height !== h) [g.canvas.width, g.canvas.height] = [w, h];
  g.drawImage(element, r.sx, r.sy, r.sw, r.sh, 0, 0, w, h);
  const px = g.getImageData(0, 0, w, h).data, out = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) out[i] = 0.299 * px[4 * i] + 0.587 * px[4 * i + 1] + 0.114 * px[4 * i + 2];
  return out;
}

// The camera picture's finder for a video source: the Perception feeding it (perception.js PictureFrame, the one finder the
// app uses: it also crops the source) as { ready, rect() } (rect normalised in the source's region; null while not known),
// else a PictureFinder of its own, fed (at most every 300 ms, from a perception "frame" or a check) until it is ready.
const finders = new WeakMap();
export function pictureOf(source, now = performance.now()) {
  if (!source?.element || !source.ready?.()) return null;
  const per = source[Symbol.for("whoop.perception")];
  if (per?.pictureState) {
    const st = per.pictureState();
    return { ready: st !== "looking", perception: per, rect: () => (st === "found" || st === "whole" || st === "sim" ? { x: 0, y: 0, w: 1, h: 1 } : null) };
  }
  let f = finders.get(source);
  if (!f) finders.set(source, (f = { finder: new PictureFinder(), t: -Infinity, crop: null }));
  const r = source.region();
  if (JSON.stringify(f.crop) !== JSON.stringify(r)) Object.assign(f, { finder: new PictureFinder(), crop: r }); // a new crop: learn again
  if (!f.finder.ready && now - f.t >= 300) {
    f.t = now;
    const g = sampleGray(source.element(), r);
    if (g) f.finder.add(g, 96, 54);
  }
  return f.finder;
}

// Aspects (width / height) of a camera picture: the lens's (the O4's 4:3) or the same width cropped to 16:9.
const ASPECTS = [4 / 3, 16 / 9];
// Why the video frame isn't the camera's picture ("" when it is, or when that can't be told yet): black bars inside the
// region (the goggles' pillarbox), or an aspect that is neither the lens's nor a 16:9 crop of it. Bearings, ranges, depth
// and localization all assume the picture.
export function pictureProblem(perception, aspect = perception?.geometry?.aspect ?? 4 / 3) {
  if (perception?.pictureState) return problemOf(perception, aspect);
  const src = perception?.source, r = src?.region?.(), w = r?.sw ?? perception?.latest?.width, h = r?.sh ?? perception?.latest?.height;
  if (!(w > 0 && h > 0)) return "";
  const finder = pictureOf(src), pic = finder?.rect();
  if (pic && (pic.w < 0.96 || pic.h < 0.96)) return `the video has black bars around the camera picture (the picture is ${Math.round(pic.w * w)}x${Math.round(pic.h * h)} of ${w}x${h}): crop the video to it`;
  // not the lens's aspect and the bars not looked for yet (or too dark to see): a pillarboxed stream would pass as 16:9
  if (finder && !pic && Math.abs(w / h / aspect - 1) > 0.03)
    return finder.ready ? "the video is too dark to tell where the camera picture is in it" : "checking the video for black bars around the camera picture";
  if (![aspect, ...ASPECTS.slice(1)].some((a) => Math.abs(w / h / a - 1) <= 0.03)) return `the video is ${w}x${h}, not the camera's picture (4:3, or a 16:9 crop of it): crop the video to it`;
  return "";
}

// The same for an app Perception (perception.js): its source's region is already the picture once it is found.
function problemOf(perception, aspect) {
  const st = perception.pictureState(), src = perception.source;
  if (!st || st === "sim" || !src?.ready?.()) return "";
  const base = src.baseRegion?.() ?? src.region(), r = perception.region() ?? base, near = (a, b) => Math.abs(a / b - 1) <= 0.03;
  if (st === "looking" && !near(base.sw / base.sh, aspect)) return "checking the video for black bars around the camera picture";
  if (st === "dark" && !near(base.sw / base.sh, aspect)) return "the video is too dark to tell where the camera picture is in it";
  if (![aspect, ...ASPECTS.slice(1)].some((a) => near(r.sw / r.sh, a))) return `the video is ${r.sw}x${r.sh}, not the camera's picture (4:3, or a 16:9 crop of it): crop the video to it`;
  return "";
}

const HANG_MS = 4000;

// The worker, page side: load() -> { name, loadMs, cached, input, output }; run(bitmap, t) -> Promise { t, disp, rgba, hi, ms };
// align(t, expected, lens, { expectedRgb, boxes, mask }?) -> Promise { a: alignDepth() of frame t's disparity in the worker,
// grid: nav/changes.js differences() of the frame (with expectedRgb), ms, gridMs }.
export class DepthEstimator {
  constructor({ model = DEPTH.model, width = DEPTH.width, height = DEPTH.height, hfov = DEPTH.hfov, src = DEPTH.src, hires = DEPTH.hires } = {}) {
    Object.assign(this, { key: model, width, height, hfov, src, hires, worker: null, info: null, waiting: new Map(), seq: 0 });
  }
  // lens: the camera's (twin/lens.js style; nav/lens.js droneLens(calib)).
  load(lens, onProgress) {
    if (this.loading) return this.loading;
    this.worker = new Worker(new URL("./depth-worker.js", import.meta.url), { type: "module" });
    this.loading = new Promise((resolve, reject) => {
      this.worker.onmessage = ({ data: m }) => {
        if (m.type === "progress") onProgress?.(m.loaded, m.total);
        else if (m.type === "ready") resolve((this.info = m));
        else if (m.type === "error") reject(new Error(m.error));
        else {
          const p = this.waiting.get(m.id);
          this.waiting.delete(m.id);
          if (m.type === "fail") p?.reject(new Error(m.error));
          else p?.resolve(m);
        }
      };
      this.worker.onerror = (e) => {
        e.preventDefault?.();
        const err = new Error(e.message || "the depth worker crashed");
        reject(err);
        for (const p of this.waiting.values()) p.reject(err);
        this.waiting.clear();
      };
    });
    this.worker.postMessage({ type: "init", key: this.key, lens, src: this.src, width: this.width, height: this.height, hfov: this.hfov, hires: this.hires });
    return this.loading;
  }
  setLens(lens) {
    this.worker?.postMessage({ type: "lens", lens });
  }
  get busy() {
    return this.waiting.size > 0;
  }
  // bitmap: the camera frame at src size (transferred).
  run(bitmap, t) {
    return this.call({ type: "run", bitmap, t }, [bitmap]);
  }
  align(t, expected, lens, more = {}) {
    return this.call({ type: "align", t, expected, lens, ...more });
  }
  call(m, transfer = []) {
    const id = ++this.seq;
    return new Promise((resolve, reject) => {
      this.waiting.set(id, { resolve, reject });
      this.worker.postMessage({ ...m, id }, transfer);
      setTimeout(() => this.waiting.get(id) && (this.waiting.delete(id), reject(new Error(`the depth worker didn't answer in ${HANG_MS / 1000} s`))), HANG_MS);
    });
  }
  dispose() {
    this.worker?.terminate();
    this.worker = null;
  }
}

// From the camera frames (perception "frame" events) to aligned depth frames, at DEPTH.hz while flying with a twin and a
// position. Nothing runs (and nothing downloads) until start().
export class LiveDepth extends Emitter {
  // osd: an OsdMask (nav/osdmask.js) learning where the camera frame is the goggles' OSD, if there is one.
  constructor({ perception, localizer, ctl, settings, twin = null, calib = null, osd = null, options = {} }) {
    super();
    Object.assign(this, { perception, loc: localizer, ctl, settings, twin, calib, osd, osdView: null });
    this.o = { ...DEPTH, ...(DEPTH_MODELS[settings?.get?.("depthModel")] && { model: settings.get("depthModel") }), ...options };
    this.estimator = null;
    this.next = 0;
    this.inFlight = false;
    this.why = "not started";
    this.stats = { frames: 0, modelMs: 0, twinMs: 0, alignMs: 0, gridMs: 0, absRel: NaN, inliers: NaN, skipped: 0 };
  }
  get lens() {
    return droneLens(this.calib, this.settings?.get?.("uptilt"));
  }
  setTwin(twin) {
    this.twin = twin;
  }
  setCalib(calib) {
    this.calib = calib;
    this.estimator?.setLens(this.lens);
  }
  // "" while frames come, else why not: a "status" event on every change (and the error log only then).
  state(why, error = false) {
    if (why === this.why) return;
    this.why = why;
    this.emit("status", { text: why ? `Live depth is off: ${why}.` : "Live depth is running.", why, ok: !why });
    if (why && error) this.emit("error", `Live depth: ${why}`);
  }
  // The model downloaded (once) and loaded, frames not looked at yet (nav/readiness.js: before the battery goes in);
  // start() takes it over. A failed load (a dropped download) is let go, so the next warm() or start() tries again.
  warm(onProgress) {
    const e = (this.warmed ??= this.estimator ?? new DepthEstimator(this.o));
    return e.load(this.lens, onProgress).catch((err) => { this.drop(e); throw err; });
  }
  drop(e) {
    e.dispose();
    if (this.warmed === e) this.warmed = null;
    if (this.estimator === e) this.estimator = null;
  }
  get loaded() {
    return !!(this.estimator ?? this.warmed)?.info;
  }
  async start(onProgress) {
    if (!this.estimator) this.estimator = this.warmed ?? new DepthEstimator(this.o);
    const e = this.estimator;
    try {
      const info = await e.load(this.lens, onProgress);
      this.unsub ??= this.perception.on("frame", (f) => this.frame(f));
      this.emit("status", { text: `${info.name} ready (${info.cached ? "cached" : "downloaded"}, ${Math.round(info.loadMs)} ms)`, why: this.why, ok: !this.why, info });
      return info;
    } catch (err) {
      this.drop(e);
      this.state(`the depth model didn't load (${err.message})`, true);
      throw err;
    }
  }
  stop() {
    this.unsub?.();
    this.unsub = null;
    this.estimator?.dispose();
    this.estimator = this.warmed = null;
    this.state("stopped");
  }

  // The camera's picture -> { sx, sy, sw, sh, why }: the app's Perception finds it (perception.region(), black bars taken
  // off) and says what is wrong with it (pictureProblem); other perceptions (checks) through pictureOf's own finder. A
  // region of another aspect than the lens's waits until the finder has seen enough frames to tell a 16:9 crop of the
  // picture from the picture with bars around it.
  picture(src, now) {
    const per = this.perception;
    if (per.pictureState) return { ...(per.region() ?? src.region()), why: problemOf(per, this.lens.aspect ?? 4 / 3) };
    const r = src.region(), f = pictureOf(src, now), p = f?.rect(), q = p && (p.w < 0.96 || p.h < 0.96) ? { sx: r.sx + p.x * r.sw, sy: r.sy + p.y * r.sh, sw: p.w * r.sw, sh: p.h * r.sh } : r;
    const a = q.sw / q.sh, lens = this.lens.aspect ?? 4 / 3, ok = [lens, 16 / 9].some((x) => Math.abs(a / x - 1) <= 0.03);
    if (q === r && f && !f.ready && Math.abs(a / lens - 1) > 0.03) return { ...q, why: "checking the video for black bars around the camera picture" };
    return { ...q, why: ok ? "" : `the video (${Math.round(q.sw)}x${Math.round(q.sh)}) isn't the camera's picture: crop it to the picture` };
  }

  // One camera frame, if it is time, nothing is in flight (from the grab to the aligned frame), and there is a twin and a
  // position to align with.
  async frame(latest) {
    const now = performance.now(), src = this.perception.source, p = this.loc.pose();
    if (now < this.next || this.inFlight || !this.estimator || !src?.ready?.() || !this.ctl.isFlying()) return;
    if (latest.decoded != null && latest.t - latest.decoded > this.o.staleMs) return; // a late frame: its time is a guess
    if (!this.twin) return this.state("no 3D twin of the house is loaded");
    if (p.status === "lost") return this.state("I don't know where I am");
    const r = this.picture(src, now);
    if (r.why) return this.state(r.why);
    this.next = now + 1000 / this.o.hz;
    this.inFlight = true;
    const t = latest.t, at = latest.captured ?? (latest.decoded ?? t) - this.ctl.videoDelay, P = this.loc.poseAt(at), att = this.ctl.historyAt?.(at); // when the frame was taken, as splatloc times it
    const pose = { ...P, z: P.z + CAM_DZ, pitch: att?.p ?? 0, roll: att?.q ?? 0, sigma: p.sigma, zSigma: p.zSigma, status: p.status };
    const sw = this.o.src[0], sh = Math.round((sw * r.sh) / r.sw); // the picture's own aspect, never squeezed
    const lens = rectLens(this.lens.uptiltDeg, this.o.hfov), view = { width: this.o.width, height: this.o.height, lens, actors: false, props: false, prio: "depth" }, tw = this.twin;
    try {
      const bitmap = await createImageBitmap(src.element(), r.sx, r.sy, r.sw, r.sh, { resizeWidth: sw, resizeHeight: sh, resizeQuality: "medium" });
      const t0 = performance.now(), twinAt = (tw.pixelsDepth ? tw.pixelsDepth(pose, view).then((d) => [d.depth, d.pixels]) : Promise.all([tw.depth(pose, view), tw.pixels(pose, view)]))
        .then((d) => ((this.stats.twinMs = performance.now() - t0), d));
      const [out, [expected, expectedRgb]] = await Promise.all([this.estimator.run(bitmap, t), twinAt]);
      const K = intrinsics(lens, this.o.width, this.o.height), Kf = intrinsics(this.lens, sw, sh), o = this.osd;
      // an OSD is a few strokes of text: a mask hiding more than DEPTH.osdMax of the picture is a learning failure, not an
      // OSD; one vision dropped (its matches collapsed: OsdMask.clear) goes at once, it would hide obstacles
      if (!o?.ready) this.osdView = null;
      else if (!this.osdView || now - this.osdView.t > 5000) this.osdView = { t: now, mask: o.usable ? viewMask(o.mask(), o.width, o.height, this.lens, this.o.width, this.o.height, this.o.hfov) : null };
      // boxes are normalised in the frame's region (the source's, before the picture was known); the picture may be part of it
      const R = latest.sourceRegion ?? src.region(), boxes = latest.detections.filter((d) => /^(person|cat|dog)$/.test(d.label) && d.score >= 0.3)
        .map((d) => ({ ...d.box, x: (R.sx + d.box.x * R.sw - r.sx) / r.sw, y: (R.sy + d.box.y * R.sh - r.sy) / r.sh, w: (d.box.w * R.sw) / r.sw, h: (d.box.h * R.sh) / r.sh }))
        .map((b) => boxToView(b, Kf, K)).filter(Boolean), mask = this.osdView?.mask;
      const { a, grid, ms, gridMs } = await this.estimator.align(t, expected, lens, { expectedRgb, boxes, mask });
      Object.assign(this.stats, { frames: this.stats.frames + 1, modelMs: out.ms.run, alignMs: ms, gridMs });
      GPU.mark("depth", performance.now() - now);
      if (!a) return void (this.stats.skipped++, this.state("the view has too little of the scan to line the depth up with"));
      Object.assign(this.stats, { absRel: a.absRel, inliers: a.inliers });
      this.state("");
      this.emit("depth", { t, pose, width: this.o.width, height: this.o.height, lens, depth: a.depth, conf: a.conf, expected, expectedRgb, boxes, mask, ...(grid && { grid }),
        rgb: { width: this.o.width, height: this.o.height, data: out.rgba }, hires: out.hi && { width: this.o.width * this.o.hires, height: this.o.height * this.o.hires, data: out.hi },
        stats: { ...this.stats, scale: a.scale, shift: a.shift } });
    } catch (e) {
      this.stats.skipped++;
      this.state(e.message, true);
    } finally {
      this.inFlight = false;
    }
  }
}
