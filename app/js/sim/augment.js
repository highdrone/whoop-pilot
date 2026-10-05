// What the real O4 video does to the picture, applied to the splat twin's frames so vision is tested against something
// closer to it (docs/HOME-DRONE.md, Wave C): motion blur from the drone's turn and climb rates, auto-exposure hunting
// and white balance, sensor noise, compression (JPEG's loss standing in for H.264's), a burned-in OSD like the goggles'
// and a lens that is not quite the one the app assumes (lensError: the simulator gives the twin that lens; the localizer
// keeps the default). Canvas 2D compositing, then the compression in JS (jpegLoss: the canvas encoder waits about a
// second per frame in a hidden page): a few ms a frame at 320x240, about 20 at 640x480, so in a page it runs in a worker
// (AugmentWorker, augmenter()): the control loop and the UI never wait for it.
// augment: true for all of it, or { blur, exposure, noise (0-1 strengths), jpeg (quality, 0 = off), osd (bool),
// lensError: true | { fov (fraction), cx, cy (fraction of the frame), k1, uptiltDeg } }.
export const AUGMENT = { blur: 1, exposure: 1, noise: 1, jpeg: 0.6, osd: true, lensError: null };
export const LENS_ERROR = { fov: 0.03, cx: 0.01, cy: -0.008, k1: -0.02, uptiltDeg: 2 };
const EXPOSURE_S = 1 / 90; // the O4 indoors

export function augmentOptions(a) {
  if (!a) return null;
  const o = a === true ? { ...AUGMENT, lensError: LENS_ERROR } : { ...AUGMENT, ...a };
  if (o.lensError === true) o.lensError = LENS_ERROR;
  return o;
}

// The twin lens (twin/lens.js fields) of the simulated camera: the default from hfov and uptilt, plus the lens error.
export function simLens(hfovDeg, uptiltDeg, e = null) {
  return { diagFovDeg: hfovDeg * 1.25 * (1 + (e?.fov ?? 0)), cx: 0.5 + (e?.cx ?? 0), cy: 0.5 + (e?.cy ?? 0), k1: e?.k1 ?? 0, k2: 0, k3: 0, k4: 0,
    uptiltDeg: uptiltDeg + (e?.uptiltDeg ?? 0) };
}

export class Augmenter {
  constructor(opts, rand = Math.random) {
    Object.assign(this, { o: opts, rand, gain: 1, wb: [1, 1, 1], canvas: null, ctx: null, noiseTile: null });
  }

  #setup(w, h) {
    if (this.canvas?.width === w && this.canvas.height === h) return;
    this.canvas = new OffscreenCanvas(w, h);
    this.ctx = this.canvas.getContext("2d", { willReadFrequently: true });
    const n = new OffscreenCanvas(128, 128), g = n.getContext("2d"), img = g.createImageData(128, 128);
    for (let i = 0; i < img.data.length; i += 4) {
      const v = 128 + (this.rand() + this.rand() + this.rand() - 1.5) * 90;
      img.data[i] = img.data[i + 1] = img.data[i + 2] = v;
      img.data[i + 3] = 255;
    }
    g.putImageData(img, 0, 0);
    this.noiseTile = n;
  }

  // image: ImageBitmap or canvas; m: { yawRate, pitchRate (rad/s), fx (px per rad at the centre), vbat, seconds }.
  // -> ImageBitmap (the caller closes it).
  async apply(image, m = {}) {
    const { width: w, height: h } = image, o = this.o, r = this.rand;
    this.#setup(w, h);
    const g = this.ctx;
    // auto exposure hunts slowly; white balance drifts
    this.gain = Math.min(1.35, Math.max(0.7, this.gain + (r() - 0.5) * 0.06 * o.exposure + (1 - this.gain) * 0.02));
    this.wb = this.wb.map((v, i) => (i === 1 ? 1 : Math.min(1, Math.max(0.85, v + (r() - 0.5) * 0.02 * o.exposure))));
    const soft = 0.35 + 0.3 * o.blur;
    g.filter = `blur(${soft.toFixed(2)}px) brightness(${(1 + (this.gain - 1) * o.exposure).toFixed(3)}) contrast(${(1 + 0.1 * o.exposure * (r() - 0.5)).toFixed(3)})`;
    g.globalCompositeOperation = "source-over";
    g.globalAlpha = 1;
    g.drawImage(image, 0, 0);
    // motion blur: the picture smeared along the image motion during the exposure
    const fx = m.fx ?? w / 2.2, bx = (m.yawRate ?? 0) * fx * EXPOSURE_S * o.blur, by = (m.pitchRate ?? 0) * fx * EXPOSURE_S * o.blur, len = Math.hypot(bx, by);
    if (len > 0.75) {
      const taps = Math.min(6, Math.ceil(len));
      for (let i = 1; i <= taps; i++) {
        g.globalAlpha = 1 / (i + 1);
        g.drawImage(image, (bx * i) / taps - bx / 2, (by * i) / taps - by / 2);
      }
      g.globalAlpha = 1;
    }
    g.filter = "none";
    if (o.exposure && this.wb.some((v) => v < 1)) {
      g.globalCompositeOperation = "multiply";
      g.fillStyle = `rgb(${Math.round(255 * this.wb[0])},255,${Math.round(255 * this.wb[2])})`;
      g.fillRect(0, 0, w, h);
    }
    if (o.noise) {
      g.globalCompositeOperation = "overlay";
      g.globalAlpha = 0.22 * o.noise;
      const ox = -Math.floor(r() * 128), oy = -Math.floor(r() * 128);
      for (let y = oy; y < h; y += 128) for (let x = ox; x < w; x += 128) g.drawImage(this.noiseTile, x, y);
      g.globalAlpha = 1;
    }
    g.globalCompositeOperation = "source-over";
    if (o.osd) this.#osd(g, w, h, m);
    if (o.jpeg > 0) {
      const img = g.getImageData(0, 0, w, h);
      jpegLoss(img.data, w, h, o.jpeg);
      g.putImageData(img, 0, 0);
    }
    return this.canvas.transferToImageBitmap();
  }

  // The overlay alone on a transparent w x h canvas (where a learned OSD mask should be).
  overlay(w, h, m = {}) {
    const c = new OffscreenCanvas(w, h);
    this.#osd(c.getContext("2d"), w, h, m);
    return c;
  }

  // The goggles' overlay as the O4 burns it into a recording: battery, timer, mode, link, crosshair.
  #osd(g, w, h, m) {
    const s = w / 640, pad = 14 * s, sec = Math.floor(m.seconds ?? 0);
    g.font = `bold ${Math.round(17 * s)}px ui-monospace, Menlo, monospace`;
    g.lineWidth = 3 * s;
    g.strokeStyle = "rgba(0,0,0,0.7)";
    g.fillStyle = "rgba(255,255,255,0.95)";
    const text = (t, x, y, align = "left") => { g.textAlign = align; g.strokeText(t, x, y); g.fillText(t, x, y); };
    text(`${(m.vbat ?? 4.05).toFixed(2)}V`, pad, 28 * s);
    text(`${String(Math.floor(sec / 60)).padStart(2, "0")}:${String(sec % 60).padStart(2, "0")}`, w - pad, 28 * s, "right");
    text("ANGL", pad, h - 14 * s);
    text("27ms 50Mbps", w - pad, h - 14 * s, "right");
    text("RSSI 99", w / 2, 28 * s, "center");
    g.beginPath();
    g.moveTo(w / 2 - 12 * s, h / 2);
    g.lineTo(w / 2 + 12 * s, h / 2);
    g.moveTo(w / 2, h / 2 - 12 * s);
    g.lineTo(w / 2, h / 2 + 12 * s);
    g.stroke();
    g.strokeStyle = "rgba(255,255,255,0.95)";
    g.lineWidth = 1.5 * s;
    g.stroke();
  }
}

// The Augmenter in a worker (augment-worker.js): apply() takes the image (transferred) and resolves to the augmented
// ImageBitmap, in order, the same pictures as in place with mulberry32(seed).
export class AugmentWorker {
  constructor(opts, seed) {
    Object.assign(this, { o: opts, seq: 0, pending: new Map() });
    this.worker = new Worker(new URL("./augment-worker.js", import.meta.url), { type: "module" });
    this.worker.onmessage = ({ data: { id, bitmap, error } }) => {
      const p = this.pending.get(id);
      this.pending.delete(id);
      error ? p?.reject(new Error(error)) : p?.resolve(bitmap);
    };
    this.worker.onerror = (e) => this.#fail(new Error(e.message || "augment worker failed"));
    this.worker.postMessage({ init: { opts, seed } });
  }
  apply(image, m = {}) {
    return new Promise((resolve, reject) => {
      const id = ++this.seq;
      this.pending.set(id, { resolve, reject });
      this.worker.postMessage({ id, image, m }, image instanceof ImageBitmap ? [image] : []);
    });
  }
  overlay(w, h, m) {
    return (this.local ??= new Augmenter(this.o)).overlay(w, h, m);
  }
  dispose() {
    this.worker.terminate();
    this.#fail(new Error("augmenter disposed"));
  }
  #fail(e) {
    for (const p of this.pending.values()) p.reject(e);
    this.pending.clear();
  }
}

// An augmenter for these options: in a worker where pages have them, else in place (Node, tests).
export function augmenter(opts, seed, rand) {
  if (!opts) return null;
  return typeof Worker !== "undefined" && typeof OffscreenCanvas !== "undefined" && typeof ImageBitmap !== "undefined" ? new AugmentWorker(opts, seed) : new Augmenter(opts, rand);
}

// JPEG's loss without the encoder, in place on RGBA bytes w x h: YCbCr, chroma averaged over 2x2, 8x8 DCT blocks
// quantized with the standard tables at `quality` (0-1, as canvas.convertToBlob takes it; IJG scaling), and back.
const QY = [16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55, 14, 13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29, 51, 87, 80, 62,
  18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113, 92, 49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99];
const QC = [17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99, 24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99, ...new Array(32).fill(99)];
const COS = Float32Array.from({ length: 64 }, (_, i) => ((i & 7 ? 1 : Math.SQRT1_2) / 2) * Math.cos(((2 * (i >> 3) + 1) * (i & 7) * Math.PI) / 16)); // [x][u]

export function jpegLoss(px, w, h, quality = 0.6) {
  const q = Math.max(1, Math.min(100, Math.round(quality * 100))), sc = q < 50 ? 5000 / q : 200 - 2 * q;
  const table = (base) => Float32Array.from(base, (v) => Math.max(1, Math.min(255, Math.floor((v * sc + 50) / 100))));
  const n = w * h, cw = (w + 1) >> 1, ch = (h + 1) >> 1, Y = new Float32Array(n), Cb = new Float32Array(cw * ch), Cr = new Float32Array(cw * ch), cnt = new Uint8Array(cw * ch);
  for (let y = 0, i = 0; y < h; y++) for (let x = 0; x < w; x++, i++) {
    const r = px[4 * i], g = px[4 * i + 1], b = px[4 * i + 2], k = (y >> 1) * cw + (x >> 1);
    Y[i] = 0.299 * r + 0.587 * g + 0.114 * b;
    Cb[k] += 128 - 0.168736 * r - 0.331264 * g + 0.5 * b;
    Cr[k] += 128 + 0.5 * r - 0.418688 * g - 0.081312 * b;
    cnt[k]++;
  }
  for (let k = 0; k < Cb.length; k++) (Cb[k] /= cnt[k]), (Cr[k] /= cnt[k]);
  dctQuantize(Y, w, h, table(QY));
  const tc = table(QC);
  dctQuantize(Cb, cw, ch, tc);
  dctQuantize(Cr, cw, ch, tc);
  for (let y = 0, i = 0; y < h; y++) for (let x = 0; x < w; x++, i++) {
    const k = (y >> 1) * cw + (x >> 1), l = Y[i], cb = Cb[k] - 128, cr = Cr[k] - 128;
    px[4 * i] = l + 1.402 * cr;
    px[4 * i + 1] = l - 0.344136 * cb - 0.714136 * cr;
    px[4 * i + 2] = l + 1.772 * cb;
  }
  return px;
}

// Every 8x8 block of a plane (edges repeated) through DCT, quantization by Q and the inverse, in place.
function dctQuantize(p, w, h, Q) {
  const f = new Float32Array(64), t = new Float32Array(64);
  for (let by = 0; by < h; by += 8) for (let bx = 0; bx < w; bx += 8) {
    for (let y = 0; y < 8; y++) for (let x = 0, r = Math.min(h - 1, by + y) * w; x < 8; x++) f[y * 8 + x] = p[r + Math.min(w - 1, bx + x)] - 128;
    for (let y = 0; y < 8; y++) for (let u = 0; u < 8; u++) { let s = 0; for (let x = 0; x < 8; x++) s += f[y * 8 + x] * COS[x * 8 + u]; t[y * 8 + u] = s; }
    for (let v = 0; v < 8; v++) for (let u = 0; u < 8; u++) {
      let s = 0;
      for (let y = 0; y < 8; y++) s += t[y * 8 + u] * COS[y * 8 + v];
      const qv = Q[v * 8 + u];
      f[v * 8 + u] = Math.round(s / qv) * qv;
    }
    for (let v = 0; v < 8; v++) for (let x = 0; x < 8; x++) { let s = 0; for (let u = 0; u < 8; u++) s += f[v * 8 + u] * COS[x * 8 + u]; t[v * 8 + x] = s; }
    for (let y = 0; y < 8 && by + y < h; y++) for (let x = 0; x < 8 && bx + x < w; x++) {
      let s = 0;
      for (let v = 0; v < 8; v++) s += t[v * 8 + x] * COS[y * 8 + v];
      p[(by + y) * w + bx + x] = s + 128;
    }
  }
}
