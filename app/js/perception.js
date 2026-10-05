// Frame pipeline: takes frames from the active video source (simulator, DJI Goggles 3, USB video device,
// or a captured window such as iPhone Mirroring), estimates image motion, runs the object
// detector, and makes JPEG snapshots for Claude.
//
// Every source exposes: ready(), element(), region() -> {sx, sy, sw, sh} (the part of the element that is
// the drone's video, after cropping), frameId(), and optionally detections() (ground truth: then no detector runs),
// captured() (when the current frame was taken, performance.now ms: the simulator knows) and decodedAt() (when it was
// decoded or painted). A source may also take the camera picture perception finds in it: baseRegion() (the user's crop,
// or the whole element) and setPicture(rect | null); its region() is then that picture (see PictureFrame).
// "frame" events carry, besides `latest`, the picture for consumers at their own rate (nav/splatloc.js): element,
// region (the camera picture: black bars taken off), sourceRegion (the source's region(), which detection boxes are
// normalised in), picture (false while the picture is still being looked for), captured (or undefined: then it was
// taken about videoDelay before t) and decoded. Copy them synchronously (createImageBitmap(element, ...region)): the
// element shows the next frame soon.
// Geometry: equidistant lens, so image position is linear in angle: hfov across the width, hfov * h / w down it
// (the O4 at 4:3: 127 x 95 deg; a 16:9 crop of it 127 x 71), the horizon uptilt below the centre. The Goggles 3 don't
// crop the O4's 4:3 picture into their 1920x1080 stream: they pillarbox it (1440x1080 in the middle, black bars beside,
// the OSD over both), so the picture must be found in the stream. It is held steady (PICTURE.hold): a found picture
// moves only when the finder disagrees by more than a few pixels for a while, since calibration, the pad check and the
// overlay mask are tied to it; but each look also checks the held picture against that one sample (bars still dark, or no
// new bars at the edges), and two looks in a row that contradict it start the search again (the stream's layout changed:
// its long-learned history would outvote the new layout for 20 s). pictureState() says where the search is;
// vision/depth.js pictureProblem() reads it.
import { FlowEstimator } from "./flow.js";
import { ObjectDetector } from "./detector.js";
import { LENS } from "./settings.js";
import { Emitter } from "./util.js";

// The camera's picture inside a video region, from grey copies (`width` px wide) of every `every`-th frame: per column
// and row the share of its pixels darker than `dark`, averaged over the recent samples (an exponential window, `decay`).
// Bars are where that share is high; the goggles draw their OSD over the bars too (icons, timers, banners: a third of
// a bar column can be bright), so the edges aren't thresholds: with bars beside the picture (and none above or below
// it), its width is the lens's aspect times the height, and its place the one where the share drops most at both edges
// (a matched filter). Strips at full resolution across the two edges (refine()) then place it to the pixel. Ready after
// `need` samples; it keeps learning, so a changed stream moves it.
export class PictureFrame {
  constructor({ need = 24, every = 5, width = 480, dark = 10, bar = 0.5, decay = 0.97, aspect = 4 / 3, centred = 0.7 } = {}) {
    Object.assign(this, { need, every, width, dark, bar, decay, aspect, centred });
    this.reset();
  }
  reset(key = null) {
    Object.assign(this, { key, count: 0, n: 0, cols: null, rows: null, fine: null });
  }
  get ready() {
    return this.count >= this.need;
  }
  // dark shares of each column and row of one grey sample
  shares(gray, w, h) {
    const cd = new Float32Array(w), rd = new Float32Array(h);
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) if (gray[y * w + x] < this.dark) (cd[x]++, rd[y]++);
    return { cols: cd.map((v) => v / h), rows: rd.map((v) => v / w) };
  }
  add(gray, w, h) {
    if (this.cols?.length !== w || this.rows.length !== h) Object.assign(this, { cols: new Float32Array(w), rows: new Float32Array(h), n: 0, count: 0, fine: null });
    const d = this.shares(gray, w, h), k = this.decay;
    d.cols.forEach((v, x) => (this.cols[x] = this.cols[x] * k + v));
    d.rows.forEach((v, y) => (this.rows[y] = this.rows[y] * k + v));
    this.n = this.n * k + 1;
    this.count++;
  }
  // The coarse picture: { side, tb (bars beside / above and below), x0, x1, y0, y1 (sample px) } or null (not ready).
  coarse() {
    if (!this.ready) return null;
    const C = this.cols.map((v) => v / this.n), R = this.rows.map((v) => v / this.n), W = C.length, H = R.length;
    const edge = (a) => { let lo = 0, hi = a.length; while (lo < hi && a[lo] >= this.bar) lo++; while (hi > lo && a[hi - 1] >= this.bar) hi--; return [lo, hi]; };
    let [x0, x1] = edge(C), [y0, y1] = edge(R);
    if (x1 - x0 < 0.3 * W || y1 - y0 < 0.3 * H) return null;
    const side = x0 > 0 || x1 < W, tb = y0 > 0 || y1 < H, step = (a, i, n) => (a[i - 1] ?? 1) - (a[i] ?? 1) + (a[i + n] ?? 1) - (a[i + n - 1] ?? 1);
    // the best place, or the centred one (where the Goggles 3 put it) when it fits nearly as well: a picture edge that stays
    // dark scores about as low as a bar's, which could otherwise slide the picture by that dark strip's width
    const fit = (a, n) => {
      let best = 0, bv = -Infinity;
      for (let i = 0; i + n <= a.length; i++) { const v = step(a, i, n); if (v > bv) [best, bv] = [i, v]; }
      const mid = Math.round((a.length - n) / 2);
      return bv > 0 && step(a, mid, n) >= this.centred * bv ? mid : best;
    };
    if (side && !tb) { const n = Math.min(W, Math.round(H * this.aspect)); x0 = fit(C, n); x1 = x0 + n; } // square sample pixels
    else if (tb && !side) { const n = Math.min(H, Math.round(W / this.aspect)); y0 = fit(R, n); y1 = y0 + n; }
    return { side, tb, x0, x1, y0, y1, W, H };
  }
  // A full-resolution strip of grey pixels across a side edge: { side: "left" | "right", x (its first column, element
  // px), gray, w, h }; strips at another x start again.
  refine({ side, x, gray, w, h }) {
    const f = (this.fine ??= {}), k = this.decay, d = this.shares(gray, w, h).cols;
    if (f[side]?.x !== x || f[side].a.length !== w) f[side] = { x, a: new Float32Array(w), n: 0 };
    d.forEach((v, i) => (f[side].a[i] = f[side].a[i] * k + v));
    f[side].n = f[side].n * k + 1;
  }
  // The picture in element pixels inside region r: { sx, sy, sw, sh, bars } or null (not known yet).
  snap(r) {
    const c = this.coarse();
    if (!c) return null;
    if (!c.side && !c.tb) return { ...r, bars: false };
    const sxs = r.sw / c.W, sys = r.sh / c.H;
    let sx = r.sx + c.x0 * sxs, sy = r.sy + c.y0 * sys, sw = (c.x1 - c.x0) * sxs, sh = (c.y1 - c.y0) * sys;
    if (c.side && !c.tb) {
      [sh, sy] = [r.sh, r.sy];
      sw = Math.min(r.sw, sh * this.aspect);
      // the strips: where the dark share drops most, into the picture (left) and out of it (right)
      const at = (e, left) => {
        if (!e || e.n < 0.5 * this.need) return null;
        const a = e.a.map((v) => v / e.n);
        let best = null, bv = 0.2;
        for (let i = 2; i < a.length - 1; i++) { const v = left ? (a[i - 2] + a[i - 1]) / 2 - (a[i] + a[i + 1]) / 2 : (a[i] + a[i + 1]) / 2 - (a[i - 2] + a[i - 1]) / 2; if (v > bv) [best, bv] = [e.x + i, v]; }
        return best;
      };
      const fl = at(this.fine?.left, true), fr = at(this.fine?.right, false), near = (v, u) => v != null && Math.abs(v - u) <= 3 * sxs;
      const cands = [near(fl, sx) && fl, near(fr, sx + sw) && fr - sw].filter((v) => v !== false);
      if (cands.length) sx = cands.reduce((a, b) => a + b, 0) / cands.length;
    } else if (c.tb && !c.side) {
      [sw, sx] = [r.sw, r.sx];
      sh = Math.min(r.sh, sw / this.aspect);
    }
    return { sx: Math.round(sx), sy: Math.round(sy), sw: Math.round(sw), sh: Math.round(sh), bars: true };
  }
  // The coarse edges in element px for the strips (null: no side bars).
  edges(r) {
    const c = this.coarse();
    if (!c?.side || c.tb) return null;
    const sxs = r.sw / c.W, sx = r.sx + c.x0 * sxs, sw = Math.min(r.sw, r.sh * this.aspect);
    return [sx, sx + sw];
  }
}

const WORK_W = 320;
// The found picture is kept unless the finder puts it more than px away (any edge) `n` checks in a row (a check every
// PictureFrame.every frames: about a second at 30 fps). A new stream or crop starts the search again, and so do `doubt`
// looks in a row whose own sample contradicts the held picture. Once the picture has held for `settled` checks it is
// looked at only every `settledEvery` frames (each look reads pixels back from the GPU), until a look doubts it.
export const PICTURE = { hold: { px: 4, n: 6 }, settled: 10, settledEvery: 30, doubt: 2 };
export const PERCEPTION = Symbol.for("whoop.perception"); // source[PERCEPTION]: the Perception it feeds (vision/depth.js)

export class Perception extends Emitter {
  constructor() {
    super();
    this.work = document.createElement("canvas");
    this.wctx = this.work.getContext("2d", { willReadFrequently: true });
    this.detector = new ObjectDetector();
    // "status": the detector's { text, level, backend, progress } (progress 0-1 on every percent of a model download, so
    // show it as one line that updates); "error": text, for warnings and failures.
    this.detector.on("status", (st) => (st.level === "warn" ? this.emit("error", st.text) : this.emit("status", st)));
    this.source = null;
    this.flow = null;
    this.lastFrameId = -1;
    this.frameCount = 0;
    this.fpsWindow = []; // when the last second's frames arrived (fps)
    this.latest = { t: 0, detections: [], flow: null, width: 0, height: 0 };
    this.geometry = { hfov: LENS.hfov, uptilt: LENS.uptilt }; // degrees; kept in sync with settings by main.js
    this.picture = new PictureFrame();
    this.applied = null; // the picture in use: { sx, sy, sw, sh, bars } in element px (held steady), or null
    this.moving = this.held = this.doubts = 0;
  }

  setSource(source) {
    this.source = source;
    this.picture.reset();
    Object.assign(this, { applied: null, moving: 0, held: 0, doubts: 0 });
    source?.setPicture?.(null);
    if (source && typeof source === "object") source[PERCEPTION] = this;
    this.flow = null;
    this.lastFrameId = -1;
    this.fpsWindow = [];
    this.latest = { t: 0, detections: [], flow: null, width: 0, height: 0 };
    this.detectorFailed = false; // a new source tries a failed load again; frames of the same one don't
    if (source && !source.detections) this.loadDetector();
  }

  loadDetector() {
    if (this.detector.ready || this.detector.loading || this.detectorFailed) return;
    this.detector.load().catch((e) => {
      this.detectorFailed = true;
      this.emit("error", `Object detector failed to load: ${e.message}`);
    });
  }

  get frameAge() {
    return this.latest.t ? performance.now() - this.latest.t : Infinity;
  }
  // Frames in the last second, as of now: it falls to 0 within a second of the frames stopping (the "Ready to fly?" list,
  // the HUD and Safety's pace rule read it between frames).
  get fps() {
    const w = this.fpsWindow, t = performance.now() - 1000;
    while (w.length && w[0] < t) w.shift();
    return w.length;
  }

  // The camera picture in the source's element: bars taken off (see PictureFrame), or the source's region while it isn't
  // known or has none.
  region() {
    const src = this.source;
    if (!src?.ready?.()) return null;
    const r = src.region(), a = this.applied;
    if (src.kind === "sim" || src.setPicture || !a?.bars || this.picture.key !== key(r)) return r;
    return { sx: a.sx, sy: a.sy, sw: a.sw, sh: a.sh };
  }

  // Where the search for the camera picture is: "sim" (the simulator's frame is the picture), "looking" (not enough
  // frames yet), "dark" (looked, but the video is too dark to tell), "found" (black bars beside or above it, taken off) or
  // "whole" (no bars: the region is the picture). No source: "".
  pictureState() {
    const src = this.source;
    if (!src) return "";
    if (src.kind === "sim") return "sim";
    if (this.applied) return this.applied.bars ? "found" : "whole";
    return this.picture.ready ? "dark" : "looking";
  }

  // Every picture.every-th frame: a grey copy of the source's own region (and full-resolution strips across the side
  // edges once they are roughly known) -> the picture; a source that takes it gets it.
  findPicture(src) {
    const P = this.picture, base = src.baseRegion?.() ?? src.region(), k = key(base);
    if (P.key !== k) {
      P.reset(k);
      this.held = 0;
      if (this.applied) (this.applied = null), src.setPicture?.(null);
    }
    if (typeof OffscreenCanvas === "undefined" || !base.sw || !base.sh) return;
    const w = P.width, h = Math.max(8, Math.round((w * base.sh) / base.sw)), g = (this.pctx ??= new OffscreenCanvas(w, h).getContext("2d", { willReadFrequently: true }));
    if (g.canvas.width !== w || g.canvas.height !== h) [g.canvas.width, g.canvas.height] = [w, h];
    const grey = (sx, sy, sw, sh, cw, ch) => {
      if (g.canvas.width !== cw || g.canvas.height !== ch) [g.canvas.width, g.canvas.height] = [cw, ch];
      g.drawImage(src.element(), sx, sy, sw, sh, 0, 0, cw, ch);
      const px = g.getImageData(0, 0, cw, ch).data, out = new Uint8Array(cw * ch);
      for (let i = 0; i < out.length; i++) out[i] = 0.299 * px[4 * i] + 0.587 * px[4 * i + 1] + 0.114 * px[4 * i + 2];
      return out;
    };
    const sample = grey(base.sx, base.sy, base.sw, base.sh, w, h);
    if (this.applied && this.contradicts(P.shares(sample, w, h), base)) {
      this.held = 0; // look every few frames again
      if (++this.doubts >= PICTURE.doubt) {
        P.reset(k);
        Object.assign(this, { applied: null, moving: 0, doubts: 0 });
        src.setPicture?.(null);
      }
    } else this.doubts = 0;
    P.add(sample, w, h);
    const e = P.edges(base);
    if (e) for (const [side, at] of [["left", e[0]], ["right", e[1]]]) {
      const x = Math.round(at) - 12;
      if (x >= base.sx && x + 24 <= base.sx + base.sw) P.refine({ side, x, gray: grey(x, base.sy, 24, base.sh, 24, 64), w: 24, h: 64 });
    }
    const pic = this.steady(P.snap(base));
    if (pic !== this.applied) {
      this.applied = pic;
      src.setPicture?.(pic?.bars ? { sx: pic.sx, sy: pic.sy, sw: pic.sw, sh: pic.sh } : null);
    }
  }

  // Does one sample's dark shares ({ cols, rows }) contradict the held picture in region r? Held with bars: its bars
  // aren't dark any more. Held whole in a region wider (or taller) than the lens's aspect: mostly dark bars where a
  // pillarbox (letterbox) would put them, beside a lit middle.
  contradicts({ cols, rows }, r) {
    const a = this.applied, P = this.picture, w = cols.length, h = rows.length, mean = (v, i0, i1) => { let s = 0; for (let i = i0; i < i1; i++) s += v[i]; return i1 > i0 ? s / (i1 - i0) : null; };
    const x0 = Math.round(((a.sx - r.sx) * w) / r.sw), x1 = Math.round(((a.sx + a.sw - r.sx) * w) / r.sw), y0 = Math.round(((a.sy - r.sy) * h) / r.sh), y1 = Math.round(((a.sy + a.sh - r.sy) * h) / r.sh);
    if (a.bars) {
      const bars = [[cols, 0, x0 - 2], [cols, x1 + 2, w], [rows, 0, y0 - 2], [rows, y1 + 2, h]].filter(([, i0, i1]) => i1 - i0 >= 4).map(([v, i0, i1]) => mean(v, i0, i1));
      return bars.length > 0 && mean(bars, 0, bars.length) < P.bar;
    }
    const side = Math.round((w - h * P.aspect) / 2), top = Math.round((h - w / P.aspect) / 2);
    const [v, n, b] = side >= 4 ? [cols, w, side] : top >= 4 ? [rows, h, top] : [];
    return !!v && Math.min(mean(v, 1, b - 1), mean(v, n - b + 1, n - 1)) >= 0.6 && mean(v, b + 2, n - b - 2) <= 0.25; // OSD over bars: a quarter of one can be bright
  }

  // The picture to use, given the finder's latest (null: not known, or too dark to tell): the first one at once; then the
  // one in use until the finder disagrees by more than PICTURE.hold.px for PICTURE.hold.n checks in a row (a dark spell
  // keeps it too).
  steady(pic) {
    const a = this.applied, { px, n } = PICTURE.hold;
    if (!a || !pic) return a ?? pic;
    if (pic.bars === a.bars && ["sx", "sy", "sw", "sh"].every((k) => Math.abs(pic[k] - a[k]) <= px)) return (this.moving = 0), this.held++, a;
    this.held = 0;
    return ++this.moving >= n ? ((this.moving = 0), pic) : a;
  }

  process() {
    const src = this.source;
    if (!src || !src.ready()) return;
    const id = src.frameId();
    if (id === this.lastFrameId) return;
    this.lastFrameId = id;
    if (src.kind !== "sim" && this.frameCount % (this.held >= PICTURE.settled ? PICTURE.settledEvery : this.picture.every) === 0) this.findPicture(src);
    const r = this.region();
    if (!r?.sw || !r.sh) return;
    const now = performance.now();
    const ww = WORK_W;
    const wh = Math.max(2, Math.round((WORK_W * r.sh) / r.sw / 2) * 2);
    if (this.work.width !== ww || this.work.height !== wh) {
      this.work.width = ww;
      this.work.height = wh;
      this.flow = null;
    }
    this.wctx.drawImage(src.element(), r.sx, r.sy, r.sw, r.sh, 0, 0, ww, wh);

    // Grayscale for motion estimation.
    if (!this.flow) this.flow = new FlowEstimator(ww, wh);
    const px = this.wctx.getImageData(0, 0, ww, wh).data;
    const gray = new Float32Array(ww * wh);
    let sum = 0;
    for (let i = 0, j = 0; i < gray.length; i++, j += 4) sum += gray[i] = px[j] * 0.3 + px[j + 1] * 0.59 + px[j + 2] * 0.11;
    // Forward motion makes the image expand around the horizon point, which sits below center
    // because the camera is tilted up.
    const g = this.geometry;
    const vfov = (g.hfov * wh) / ww;
    const focusY = Math.max(0.2, Math.min(0.95, 0.5 + g.uptilt / vfov));
    const f = this.flow.update(gray, now, focusY);
    const flow = f ? { ...f, t: now } : null;

    let detections = this.latest.detections;
    if (src.detections) detections = src.detections();
    else if (!this.detector.ready) this.loadDetector();
    else if (this.frameCount % 2 === 0) {
      try {
        detections = this.detector.detect(this.work, ww, wh, { t: now, flow, focusY, hfov: g.hfov, element: src.element(), region: r });
      } catch (e) {
        console.warn("detector error", e);
      }
    }
    this.frameCount++;
    this.fpsWindow.push(now);
    this.latest = { t: now, detections, flow, width: r.sw, height: r.sh, luma: sum / gray.length }; // luma: mean grey 0-255 (a dark picture shows little)
    const known = src.kind === "sim" || !!this.applied;
    this.emit("frame", { ...this.latest, element: src.element(), region: this.region() ?? r, sourceRegion: r, picture: known, captured: src.captured?.(), decoded: src.decodedAt?.() });
  }

  // Current frame as base64 JPEG (no data: prefix), for Claude.
  snapshot({ maxWidth = 640, quality = 0.72 } = {}) {
    const src = this.source;
    if (!src || !src.ready()) return null;
    const r = this.region();
    if (!r?.sw) return null;
    const s = Math.min(1, maxWidth / r.sw);
    const c = document.createElement("canvas");
    c.width = Math.round(r.sw * s);
    c.height = Math.round(r.sh * s);
    const g = c.getContext("2d");
    g.imageSmoothingQuality = "high";
    g.drawImage(src.element(), r.sx, r.sy, r.sw, r.sh, 0, 0, c.width, c.height);
    return c.toDataURL("image/jpeg", quality).split(",")[1];
  }

  // Detections of one label, best first.
  find(label, minScore = 0.4) {
    return this.latest.detections.filter((d) => d.label === label && d.score >= minScore).sort((a, b) => b.score - a.score);
  }
}

// The simulator's camera. Its frames carry ground-truth boxes unless sim.detections is "detector": then the frame has
// none and the real detector runs on it.
export function simSource(sim) {
  const truth = () => sim.video.detections;
  return {
    kind: "sim",
    get label() {
      return sim.twin ? "Simulator camera (splat twin)" : "Simulator camera";
    },
    ready: () => !!sim.video,
    element: () => sim.video.canvas,
    region: () => ({ sx: 0, sy: 0, sw: sim.video.canvas.width, sh: sim.video.canvas.height }),
    frameId: () => sim.video.id,
    captured: () => sim.video.t,
    get detections() {
      return sim.detections === "detector" || (sim.video && !sim.video.detections) ? undefined : truth;
    },
  };
}

// Shared by the USB device and window-capture sources: a <video> fed by a MediaStream, plus a crop.
class StreamSource {
  constructor(kind) {
    this.kind = kind;
    this.video = document.createElement("video");
    this.video.muted = true;
    this.video.playsInline = true;
    this.stream = null;
    this.frames = 0;
    this.label = "";
    this.crop = null; // normalized {x, y, w, h}
    this.picture = null; // element px, from Perception
    this.onEnded = null;
  }

  async attach(stream) {
    this.stop();
    this.stream = stream;
    const track = stream.getVideoTracks()[0];
    this.label = track?.label || this.kind;
    track?.addEventListener("ended", () => {
      if (this.stream === stream) {
        this.stop();
        this.onEnded?.();
      }
    });
    this.video.srcObject = stream;
    await this.video.play();
    if (this.video.requestVideoFrameCallback) {
      const onFrame = () => {
        this.frames++;
        if (this.stream === stream) this.video.requestVideoFrameCallback(onFrame);
      };
      this.video.requestVideoFrameCallback(onFrame);
    }
  }

  stop() {
    this.stream?.getTracks().forEach((t) => t.stop());
    this.stream = null;
    this.video.srcObject = null;
  }

  ready() {
    return !!this.stream && this.video.readyState >= 2 && this.video.videoWidth > 0;
  }
  element() {
    return this.video;
  }
  // The user's crop, or the whole video.
  baseRegion() {
    const vw = this.video.videoWidth;
    const vh = this.video.videoHeight;
    const c = this.crop;
    if (!c || c.w < 0.02 || c.h < 0.02) return { sx: 0, sy: 0, sw: vw, sh: vh };
    return { sx: Math.round(c.x * vw), sy: Math.round(c.y * vh), sw: Math.round(c.w * vw), sh: Math.round(c.h * vh) };
  }
  // The camera picture Perception found inside baseRegion() (black bars beside or above it), or null.
  setPicture(rect) {
    this.picture = rect;
  }
  region() {
    const b = this.baseRegion(), p = this.picture;
    return p && p.sx >= b.sx && p.sy >= b.sy && p.sx + p.sw <= b.sx + b.sw && p.sy + p.sh <= b.sy + b.sh ? { sx: p.sx, sy: p.sy, sw: p.sw, sh: p.sh } : b;
  }
  // A crop drawn on the shown video (normalised in region()) -> the crop to store (normalised in the whole video).
  cropFromView(c) {
    return cropFromView(c, this.region(), this.video.videoWidth, this.video.videoHeight);
  }
  frameId() {
    return this.video.requestVideoFrameCallback ? this.frames : this.video.currentTime;
  }
}

// A USB video device: 5.8 GHz receiver, HDMI/AV capture stick, or any webcam.
export class CameraSource extends StreamSource {
  constructor() {
    super("camera");
  }
  static async list() {
    const devices = await navigator.mediaDevices.enumerateDevices();
    return devices.filter((d) => d.kind === "videoinput");
  }
  async start(deviceId) {
    const stream = await navigator.mediaDevices.getUserMedia({
      audio: false,
      video: deviceId ? { deviceId: { exact: deviceId }, width: { ideal: 1280 }, height: { ideal: 720 } } : { width: { ideal: 1280 } },
    });
    await this.attach(stream);
  }
}

// A window on this Mac showing the drone's video: e.g. iPhone Mirroring running DJI Fly's live view,
// or QuickTime. Chrome asks which window to share.
export class ScreenSource extends StreamSource {
  constructor() {
    super("screen");
  }
  async start() {
    const stream = await navigator.mediaDevices.getDisplayMedia({
      video: { frameRate: { ideal: 30, max: 60 }, displaySurface: "window" },
      audio: false,
      preferCurrentTab: false,
      selfBrowserSurface: "exclude",
      surfaceSwitching: "include",
    });
    await this.attach(stream);
  }
}

const key = (r) => `${r.sx},${r.sy},${r.sw},${r.sh}`;

// A rectangle normalised in region r of a w x h element -> normalised in the whole element (a source's crop), or null.
export function cropFromView(c, r, w, h) {
  if (!c || !w || !h) return null;
  return { x: (r.sx + c.x * r.sw) / w, y: (r.sy + c.y * r.sh) / h, w: (c.w * r.sw) / w, h: (c.h * r.sh) / h };
}
