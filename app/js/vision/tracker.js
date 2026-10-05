// ByteTrack (Zhang et al., ECCV 2022; reference code MIT), written for this app. Two-stage association: high-score
// detections first, then low-score ones only to keep already-tracked objects alive (occlusion and motion blur lower the
// score, not the object's existence). A constant-velocity Kalman filter per box. Camera-motion compensation (CMC) moves
// every predicted box by how the whole picture moved since the last update, so tracks survive fast yaw (synthetic 60 px/frame
// pans, tools/test-vision.mjs: 15-37 ID switches per 6 s without it, 0 with it). Boxes are normalised {x, y, w, h}; times
// are ms (performance.now()).
import { DEG } from "../util.js";

const STD_POS = 1 / 20; // ByteTrack's noise weights relative to box height, per 1/30 s step
const STD_VEL = 1 / 160;
const STEP = 1000 / 30;
const LOST_DAMP = 0.7; // per update, with CMC: a lost track's own velocity is a stale guess
const FADE = 0.9; // reported score fades per update while only weak detections (or none, when coasting) back a track

// One box coordinate and its rate: a 2-state constant-velocity Kalman filter. ByteTrack's 8-state filter is four of
// these (its matrices are block diagonal per coordinate).
class Axis {
  constructor(p, sp, sv) {
    this.p = p;
    this.v = 0;
    this.pp = sp * sp;
    this.pv = 0;
    this.vv = sv * sv;
  }
  predict(k, qp, qv) {
    this.p += this.v * k;
    this.pp += k * (2 * this.pv + k * this.vv) + qp * qp * k;
    this.pv += k * this.vv;
    this.vv += qv * qv * k;
  }
  correct(z, r) {
    const s = this.pp + r * r;
    const kp = this.pp / s;
    const kv = this.pv / s;
    const e = z - this.p;
    this.p += kp * e;
    this.v += kv * e;
    this.vv -= kv * this.pv;
    this.pv *= 1 - kp;
    this.pp *= 1 - kp;
  }
  scale(k) {
    this.p *= k;
    this.v *= k;
    this.pp *= k * k;
    this.pv *= k * k;
    this.vv *= k * k;
  }
}

// State in image-height units (x scaled by the aspect ratio) so that rotation and noise are isotropic:
// centre X, Y, aspect a = w/h, height H.
class Track {
  constructor(id, d, t, A) {
    const { x, y, w } = d.box;
    const h = Math.max(1e-3, d.box.h);
    Object.assign(this, { id, label: d.label, born: t, hits: 1, score: d.score, best: d.score, lowStreak: 0, missed: 0, state: "new", seen: 0 });
    this.X = new Axis((x + w / 2) * A, 2 * STD_POS * h, 10 * STD_VEL * h);
    this.Y = new Axis(y + h / 2, 2 * STD_POS * h, 10 * STD_VEL * h);
    this.a = new Axis((w * A) / h, 1e-2, 1e-5);
    this.H = new Axis(h, 2 * STD_POS * h, 10 * STD_VEL * h);
  }
  predict(k) {
    const h = this.H.p;
    this.X.predict(k, STD_POS * h, STD_VEL * h);
    this.Y.predict(k, STD_POS * h, STD_VEL * h);
    this.a.predict(k, 1e-2, 1e-5);
    this.H.predict(k, STD_POS * h, STD_VEL * h);
  }
  correct({ x, y, w, h }, A) {
    h = Math.max(1e-3, h);
    const r = STD_POS * this.H.p;
    this.X.correct((x + w / 2) * A, r);
    this.Y.correct(y + h / 2, r);
    this.a.correct((w * A) / h, 1e-1);
    this.H.correct(h, r);
  }
  // Image motion m = {dx, dy, scale, rot, cx, cy}: the picture moved by (dx, dy) (fractions of width and height), zoomed
  // by scale and rolled by rot (rad, as flow.js measures it) about (cx, cy).
  warp(m, A) {
    const c = Math.cos(m.rot) * m.scale;
    const s = Math.sin(m.rot) * m.scale;
    const X = this.X.p - m.cx * A;
    const Y = this.Y.p - m.cy;
    this.X.p = m.cx * A + c * X - s * Y + m.dx * A;
    this.Y.p = m.cy + s * X + c * Y + m.dy;
    [this.X.v, this.Y.v] = [c * this.X.v - s * this.Y.v, s * this.X.v + c * this.Y.v];
    const k2 = m.scale * m.scale;
    for (const ax of [this.X, this.Y]) {
      ax.pp *= k2;
      ax.pv *= k2;
      ax.vv *= k2;
    }
    this.H.scale(m.scale);
  }
  box(A, k = 0) {
    const h = Math.max(1e-3, this.H.p + this.H.v * k);
    const w = this.a.p * h;
    return { x: (this.X.p + this.X.v * k - w / 2) / A, y: this.Y.p + this.Y.v * k - h / 2, w: w / A, h };
  }
  clone() {
    const c = Object.assign(Object.create(Track.prototype), this);
    for (const ax of ["X", "Y", "a", "H"]) c[ax] = Object.assign(Object.create(Axis.prototype), this[ax]);
    return c;
  }
}

export function iou(a, b) {
  const w = Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x);
  const h = Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y);
  if (w <= 0 || h <= 0) return 0;
  const i = w * h;
  return i / (a.w * a.h + b.w * b.h - i);
}

// Greedy highest-IoU-first matching within each label (as good as Hungarian for the few objects in a room).
function associate(tracks, dets, minIou, A) {
  const pairs = [];
  tracks.forEach((tr, i) => {
    const b = tr.box(A);
    dets.forEach((d, j) => {
      if (d.label !== tr.label) return;
      const v = iou(b, d.box);
      if (v >= minIou) pairs.push([v, i, j]);
    });
  });
  pairs.sort((p, q) => q[0] - p[0]);
  const ti = new Set();
  const dj = new Set();
  const matched = [];
  for (const [, i, j] of pairs) {
    if (ti.has(i) || dj.has(j)) continue;
    ti.add(i);
    dj.add(j);
    matched.push([tracks[i], dets[j]]);
  }
  return { matched, tracks: tracks.filter((_, i) => !ti.has(i)), dets: dets.filter((_, j) => !dj.has(j)) };
}

const clamp01 = (v) => (v < 0 ? 0 : v > 1 ? 1 : v);
const clampBox = ({ x, y, w, h }) => {
  const x0 = clamp01(x);
  const y0 = clamp01(y);
  return { x: x0, y: y0, w: clamp01(x + w) - x0, h: clamp01(y + h) - y0 };
};

export class Tracker {
  // high/low: ByteTrack's two detection tiers; birth: a high detection that starts a track (confirmed on its minHits-th
  // hit); highFor: {label: score} replacing both high and birth for some labels; keepLost: ms a lost track may still be
  // re-found (with its id) by a high-score detection; coast: ms a lost track is still reported at its predicted place
  // (bridges single missed detections).
  constructor({ high = 0.5, low = 0.1, birth = 0.5, highFor = {}, matchIou = 0.2, lowIou = 0.5, keepLost = 1000, coast = 150, minHits = 2, aspect = 4 / 3 } = {}) {
    Object.assign(this, { high, low, birth, highFor, matchIou, lowIou, keepLost, coast, minHits, aspect });
    this.nextId = 1; // never reset: callers hold on to ids, which must not come to name another object
    this.reset();
  }

  reset() {
    this.tracks = [];
    this.t = null;
    this.frame = 0;
  }

  // dets: [{label, score, box}] seen in the frame captured at t. motion: image motion since the previous update (see
  // MotionLog.between), or null for none. Returns the confirmed tracks matched in this frame or coasting.
  update(dets, t, motion = null) {
    const A = this.aspect;
    const k = this.t === null ? 1 : Math.min(30, Math.max(0.1, (t - this.t) / STEP));
    this.t = t;
    const f = ++this.frame;
    for (const tr of this.tracks) {
      if (motion) tr.warp(motion, A);
      if (tr.state !== "tracked") {
        tr.H.v = 0;
        // without CMC the velocity is mostly the camera's own turn, which does carry on
        if (motion) {
          tr.X.v *= LOST_DAMP;
          tr.Y.v *= LOST_DAMP;
        }
      }
      tr.predict(k);
    }
    const high = (d) => this.highFor[d.label] ?? this.high;
    const hi = dets.filter((d) => d.score >= high(d));
    const lo = dets.filter((d) => d.score >= this.low && d.score < high(d));
    const first = associate(this.tracks, hi, this.matchIou, A);
    const second = associate(first.tracks.filter((tr) => tr.state === "tracked"), lo, this.lowIou, A);
    for (const [tr, d] of first.matched) this.#hit(tr, d, f, false);
    for (const [tr, d] of second.matched) this.#hit(tr, d, f, true);
    for (const tr of this.tracks) {
      if (tr.seen === f) continue;
      tr.missed++;
      if (tr.state === "tracked") tr.lostAt = t;
      tr.state = tr.state === "new" ? "dead" : "lost";
    }
    this.tracks = this.tracks.filter((tr) => tr.state !== "dead" && !(tr.state === "lost" && t - tr.lostAt > this.keepLost));
    for (const d of first.dets) {
      if (d.score < (this.highFor[d.label] ?? this.birth)) continue;
      const tr = new Track(this.nextId++, d, t, A);
      tr.seen = f;
      if (this.minHits <= 1) tr.state = "tracked";
      this.tracks.push(tr);
    }
    return this.#live().map((tr) => this.#out(tr, tr.box(A)));
  }

  // The current tracks moved on to time t (the frame on screen now, a little after the detector's frame): the image
  // motion since then plus each track's own velocity. Leaves the tracker unchanged.
  predict(t, motion = null) {
    const A = this.aspect;
    const k = this.t === null ? 0 : Math.max(0, (t - this.t) / STEP);
    return this.#live().map((tr) => {
      if (!motion) return this.#out(tr, tr.box(A, k));
      const c = tr.clone();
      c.warp(motion, A);
      return this.#out(tr, c.box(A, k));
    });
  }

  #live() {
    return this.tracks.filter((tr) => (tr.state === "tracked" && tr.seen === this.frame) || (tr.state === "lost" && this.t - tr.lostAt <= this.coast));
  }

  #hit(tr, d, f, low) {
    tr.correct(d.box, this.aspect);
    tr.hits++;
    tr.seen = f;
    tr.score = d.score;
    tr.best = Math.max(tr.best, d.score);
    tr.lowStreak = low ? tr.lowStreak + 1 : 0;
    tr.missed = 0;
    if (tr.hits >= this.minHits) tr.state = "tracked";
  }

  #out(tr, box) {
    return {
      trackId: tr.id,
      label: tr.label,
      score: Math.max(tr.score, tr.best * FADE ** tr.lowStreak) * FADE ** tr.missed,
      best: tr.best,
      age: this.t - tr.born,
      hits: tr.hits,
      box: clampBox(box),
    };
  }
}

// Image motion between two times for CMC, from flow.js estimates (rates per second; each one covers the time since the
// previous) or, where the flow is unreliable (blank walls, darkness), from the yaw rate. With neither: no compensation.
// hfov: degrees across the picture, for the yaw rate. The O4's equidistant fisheye is linear in angle, so one number
// converts it everywhere; 127 is the O4 lens until calibrated.
export class MotionLog {
  constructor({ minQuality = 0.3, hfov = 127, keep = 3000 } = {}) {
    Object.assign(this, { minQuality, hfov, keep });
    this.samples = [];
  }

  // flow: a flow.js result or null; yawRate: rad/s, + = turning right; focusY: where flow.js centres zoom and roll.
  push(t, flow, { yawRate = null, focusY = 0.5, hfov = this.hfov } = {}) {
    let r = null;
    if (flow && flow.quality >= this.minQuality) {
      r = [flow.dx, flow.dy, flow.div, flow.rot || 0];
      t -= (flow.span || 0) * 500; // flow.js rates average the last `span` s, so they describe span/2 ago
    } else if (Number.isFinite(yawRate)) r = [-yawRate / (hfov * DEG), 0, 0, 0]; // turning right slides the picture left
    const s = this.samples;
    if (s.length && t <= s[s.length - 1].t) return;
    s.push({ t, r, focusY });
    while (s[0].t < t - this.keep) s.shift();
  }

  // -> {dx, dy, scale, rot, cx, cy} over (t0, t1], or null. The newest rates carry on for up to 0.2 s past their sample.
  between(t0, t1) {
    if (t0 === null || t0 === undefined || !(t1 > t0)) return null;
    const s = this.samples;
    let dx = 0;
    let dy = 0;
    let zoom = 0;
    let rot = 0;
    let cy = 0.5;
    let any = false;
    for (let i = 0; i < s.length; i++) {
      const { t, r } = s[i];
      const a = Math.max(i ? s[i - 1].t : -Infinity, t - 250);
      const b = i === s.length - 1 ? Math.max(t, Math.min(t1, t + 200)) : t;
      const dt = (Math.min(b, t1) - Math.max(a, t0)) / 1000;
      if (dt <= 0 || !r) continue;
      dx += r[0] * dt;
      dy += r[1] * dt;
      zoom += r[2] * dt;
      rot += r[3] * dt;
      cy = s[i].focusY;
      any = true;
    }
    return any ? { dx, dy, scale: Math.exp(zoom), rot, cx: 0.5, cy } : null;
  }
}
