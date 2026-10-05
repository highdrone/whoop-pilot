// Where the drone video is not the scene: an on-screen display burned into the picture (text, crosshair, bars) and
// the black border around the fisheye circle. Both stay put while the picture moves, so the mask is learned from
// frames in which the picture moved: a pixel is overlay (or a dark border, or a blank surface: no keypoints there either
// way) when it changed in under a quarter of them (a change must beat half the frame's median change: slow motion
// leaves most of a real scene unchanged from one frame to the next too, so this alone finds only the steadiest
// overlay) or when its grey level hardly varied over all of them (std under `quiet`: the scene seen along a flight
// varies by tens of levels, an overlay's fill only by the video's noise; a ticking timer is caught by the first test).
// "Moved" is judged on 8x8 block means, so sensor noise and compression are not taken for motion, and only on what is
// left after each row of blocks is fitted as gain x the last frame + offset: auto exposure hunting and mains flicker's
// rolling bands (LED lights) change a still picture everywhere, and a mask learned from a still picture hides all of it
// (a drone on its pad: 23 of 24 frames counted, coverage 0.998, no fixes for 10 s after take-off). A mask hiding more
// than OSD_MAX of the picture is a learning failure (usable false): nobody uses it. Grey levels at 640x480
// by default (OSD strokes are 2-3 pixels wide there: smaller, every pixel of them is half scene); the mask grows by
// `grow` cells around what it finds (anti-aliased edges). mask() is 1 where keypoints may be used. Serialized into
// calib.json as { width, height, rle }. Holders share one object: assign() copies a loaded or learned mask into it.
export const OSD_MAX = 0.3; // share of the picture an overlay mask may hide (an OSD is a few strokes of text)

export class OsdMask {
  constructor({ width = 640, height = 480, minFrames = 24, still = 0.25, quiet = 10, grow = Math.max(1, Math.round(width / 320)) } = {}) {
    Object.assign(this, { width, height, minFrames, still, quiet, grow });
    const n = width * height;
    this.prev = null;
    this.changed = new Uint16Array(n);
    this.sum = new Float64Array(n);
    this.sum2 = new Float64Array(n);
    this.dark = new Uint16Array(n);
    this.frames = 0;
    this.fixed = null; // a loaded mask
  }

  get ready() {
    return !!this.fixed || this.frames >= this.minFrames;
  }
  // Ready, and hiding no more than OSD_MAX of the picture.
  get usable() {
    return this.ready && this.coverage() <= OSD_MAX;
  }
  // This mask becomes o (a loaded or freshly learned one): whoever holds this object sees it.
  assign(o) {
    if (o !== this) Object.assign(this, { ...o, prev: o.prev, cache: null });
    return this;
  }
  // Nothing learned: start again.
  clear() {
    return this.assign(new OsdMask({ width: this.width, height: this.height, minFrames: this.minFrames, still: this.still, quiet: this.quiet, grow: this.grow }));
  }

  // gray: Uint8Array or Float32Array (0-255) at width x height (perception draws the frame that small). -> whether it
  // counted (the picture moved since the last one).
  add(gray) {
    const W = this.width, H = this.height, n = W * H, g = Float32Array.from(gray), prev = this.prev;
    this.prev = g;
    if (!prev) return false;
    const bw = W >> 3, bh = H >> 3, bp = new Float32Array(bw * bh), bg = new Float32Array(bw * bh), d = (this.d ??= new Uint8Array(n)), hist = new Uint32Array(256);
    for (let y = 0; y < bh * 8; y++) for (let x = 0, r = y * W, b = (y >> 3) * bw; x < bw * 8; x++) (bp[b + (x >> 3)] += prev[r + x] / 64), (bg[b + (x >> 3)] += g[r + x] / 64);
    const res = new Float32Array(bw * bh);
    for (let by = 0; by < bh; by++) { // the row's new block means as gain x old + offset: what is left over moved
      let sx = 0, sy = 0, sxx = 0, sxy = 0;
      for (let i = by * bw; i < (by + 1) * bw; i++) (sx += bp[i]), (sy += bg[i]), (sxx += bp[i] * bp[i]), (sxy += bp[i] * bg[i]);
      const mx = sx / bw, my = sy / bw, v = sxx / bw - mx * mx, a = v > 4 ? Math.min(2, Math.max(0.5, (sxy / bw - mx * my) / v)) : 1;
      for (let i = by * bw; i < (by + 1) * bw; i++) res[i] = Math.abs(bg[i] - a * bp[i] - (my - a * mx));
    }
    if (res.sort()[res.length >> 1] < 2.5) return false; // the picture hardly moved: says nothing
    for (let i = 0; i < n; i++) hist[(d[i] = Math.min(255, Math.abs(g[i] - prev[i]) | 0))]++;
    let med = 0;
    for (let c = 0; med < 256 && c + hist[med] <= n >> 1; med++) c += hist[med];
    const thr = Math.max(6, 0.5 * med), { changed, dark, sum, sum2 } = this;
    for (let i = 0; i < n; i++) {
      const v = g[i];
      if (d[i] > thr) changed[i]++;
      if (v < 12) dark[i]++;
      sum[i] += v;
      sum2[i] += v * v;
    }
    this.frames++;
    this.cache = null;
    return true;
  }

  // Uint8Array width x height: 1 = scene, 0 = overlay or border (grown by `grow` cells).
  mask() {
    if (this.fixed) return this.fixed;
    if (this.cache) return this.cache;
    const { width: W, height: H, frames, grow: k, sum, sum2 } = this, n = W * H, bad = new Uint8Array(n), out = new Uint8Array(n).fill(1), q2 = this.quiet ** 2 * frames * frames;
    if (frames < this.minFrames) return out;
    for (let i = 0; i < n; i++) bad[i] = this.changed[i] < this.still * frames || frames * sum2[i] - sum[i] * sum[i] < q2 || this.dark[i] > 0.9 * frames ? 1 : 0;
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      if (!bad[y * W + x]) continue;
      for (let dy = -k; dy <= k; dy++) for (let dx = -k; dx <= k; dx++) {
        const xx = x + dx, yy = y + dy;
        if (xx >= 0 && yy >= 0 && xx < W && yy < H) out[yy * W + xx] = 0;
      }
    }
    return (this.cache = out);
  }

  // Share of the frame masked out.
  coverage() {
    const m = this.mask();
    return 1 - m.reduce((a, v) => a + v, 0) / m.length;
  }

  toJSON() {
    const m = this.mask(), rle = [];
    for (let i = 0, v = 1, run = 0; i <= m.length; i++) {
      if (i < m.length && m[i] === v) { run++; continue; }
      rle.push(run);
      [v, run] = [1 - v, 1];
    }
    return { width: this.width, height: this.height, rle };
  }

  static from({ width, height, rle }) {
    const o = new OsdMask({ width, height }), m = new Uint8Array(width * height);
    let i = 0, v = 1;
    for (const run of rle) { m.fill(v, i, i + run); i += run; v = 1 - v; }
    o.fixed = m;
    return o;
  }
}

// Grey levels of an RGBA image downsampled (box) to w x h.
export function grayDown(rgba, W, H, w = 320, h = 240) {
  const out = new Float32Array(w * h), cnt = new Uint16Array(w * h);
  for (let y = 0; y < H; y++) {
    const yy = Math.min(h - 1, ((y * h) / H) | 0);
    for (let x = 0; x < W; x++) {
      const i = (y * W + x) * 4, k = yy * w + Math.min(w - 1, ((x * w) / W) | 0);
      out[k] += 0.299 * rgba[i] + 0.587 * rgba[i + 1] + 0.114 * rgba[i + 2];
      cnt[k]++;
    }
  }
  for (let k = 0; k < out.length; k++) out[k] /= cnt[k] || 1;
  return out;
}
