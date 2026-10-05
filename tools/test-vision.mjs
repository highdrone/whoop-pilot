// The vision stack (app/js/vision/, app/js/detector.js) without a GPU: RF-DETR pre/post-processing (ImageNet
// normalisation, box decode, COCO label map, NMS-free top-k), the ByteTrack tracker with camera-motion compensation on
// synthetic pans, the motion log, the model store (download progress, size/SHA-256 checks, offline reuse) against a fake
// network and Cache Storage, and the detector's worker path against a fake worker. The model itself runs in Chrome:
// tools/vision-check.html.
import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { COCO_LABELS, COCO_IDS, ID2LABEL, toPlanar, decode } from "../app/js/vision/rfdetr.js";
import { Tracker, MotionLog, iou } from "../app/js/vision/tracker.js";
import { MODELS, fetchModel } from "../app/js/vision/models.js";
import * as detector from "../app/js/detector.js";
import { DEG, Emitter } from "../app/js/util.js";

const close = (a, b, eps = 1e-5) => assert.ok(Math.abs(a - b) < eps, `${a} != ${b}`);
const sigmoid = (v) => 1 / (1 + Math.exp(-v));
const logit = (p) => Math.log(p / (1 - p));
function rng(seed) {
  return () => {
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// RF-DETR outputs for Q queries: logits [Q, 91] all very negative except the given slots, boxes [Q, 4] cx, cy, w, h.
function outputs(queries, Q = queries.length) {
  const L = new Float32Array(Q * 91).fill(-9);
  const B = new Float32Array(Q * 4);
  queries.forEach(({ slots = {}, box = [0.5, 0.5, 0.1, 0.1] }, q) => {
    for (const [id, p] of Object.entries(slots)) L[q * 91 + +id] = logit(p);
    B.set(box, q * 4);
  });
  return [L, B, Q, 91];
}

// A camera yawing back and forth (sinusoid, given peak speed in image widths per detector frame at 15 Hz) over a row
// of objects; a share of detections dropped, boxes jittered, some scored low as if half-occluded. Flow samples at 30 Hz.
function pan({ peak = 60 / 512, frames = 90, drop = 0.25, seed = 1, lowShare = 0.1 } = {}) {
  const rand = rng(seed);
  const objs = [
    { gid: 1, label: "person", wx: -0.35, y: 0.25, w: 0.11, h: 0.5 },
    { gid: 2, label: "person", wx: -0.1, y: 0.3, w: 0.1, h: 0.45 },
    { gid: 3, label: "cat", wx: 0.12, y: 0.72, w: 0.13, h: 0.11 },
    { gid: 4, label: "person", wx: 0.3, y: 0.28, w: 0.12, h: 0.5 },
    { gid: 5, label: "couch", wx: 0.55, y: 0.5, w: 0.3, h: 0.25 },
    { gid: 6, label: "person", wx: 0.82, y: 0.22, w: 0.1, h: 0.55 },
  ];
  const amp = 0.45;
  const w = peak / amp; // rad per detector frame
  const camX = (f) => amp * Math.sin(w * f);
  const dt = 1000 / 15;
  const seq = [];
  for (let f = 0; f < frames; f++) {
    const gts = [];
    const dets = [];
    for (const o of objs) {
      const x = o.wx - camX(f) + 0.5 - o.w / 2;
      if (x < -o.w * 0.3 || x + o.w > 1 + o.w * 0.3) continue;
      const box = { x, y: o.y, w: o.w, h: o.h };
      gts.push({ gid: o.gid, box });
      if (rand() < drop) continue;
      const j = () => (rand() - 0.5) * 0.008;
      const score = rand() < lowShare ? 0.15 + rand() * 0.3 : 0.55 + rand() * 0.4;
      dets.push({ label: o.label, score, box: { x: x + j(), y: o.y + j(), w: o.w * (1 + j()), h: o.h * (1 + j()) } });
    }
    // content velocity in widths per second at the two video frames since the previous detector frame, each the mean
    // over its frame interval (what flow.js reports, once MotionLog has re-centred it)
    const flows = [0.5, 1].map((k) => ({ t: (f - 1 + k) * dt, dx: -amp * w * Math.cos(w * (f - 1.25 + k)) * 15, dy: 0, div: 0, rot: 0, quality: 0.8 }));
    seq.push({ t: f * dt, gts, dets, flows });
  }
  return seq;
}

// The RF-DETR score tiers the app's detector gives its tracker.
const RF_TIERS = (({ high, low, birth, highFor }) => ({ high, low, birth, highFor }))(new detector.ObjectDetector().tracker);

// ObjectDetector against a fake worker (frames queue up in w.pending until answered with w.reply), fake WebGPU and
// createImageBitmap, and a clock that jump(ms) moves on. Status texts collect in said.
async function withFakeWorker(fn) {
  const workers = [];
  const bitmaps = [];
  class FakeWorker {
    constructor(url, opts) {
      Object.assign(this, { url: String(url), opts, pending: [], terminated: false });
      workers.push(this);
    }
    postMessage(m) {
      if (m.type === "init") setTimeout(() => this.onmessage({ data: { type: "ready", name: m.key === "rfdetr-s" ? "RF-DETR-S" : "RF-DETR-N", res: 384, cached: true, init: m } }));
      else this.pending.push(m);
    }
    reply(make) {
      const m = this.pending.shift();
      this.onmessage({ data: make(m) });
    }
    terminate() {
      this.terminated = true;
    }
  }
  const gpu = Object.getOwnPropertyDescriptor(globalThis.navigator, "gpu");
  Object.defineProperty(globalThis.navigator, "gpu", { value: {}, configurable: true });
  Object.assign(globalThis, { Worker: FakeWorker, createImageBitmap: async (...a) => (bitmaps.push(a), { close() {} }) });
  const now = performance.now;
  let skew = 0;
  performance.now = () => now.call(performance) + skew;
  const { info, warn } = console;
  console.info = console.warn = () => {};
  try {
    const d = new detector.ObjectDetector();
    const said = [];
    d.on("status", (s) => said.push(s.text));
    await d.load();
    assert.equal(d.backend, "rfdetr");
    const tick = () => new Promise((r) => setTimeout(r, 0));
    const answer = (list) => (m) => ({ type: "dets", t: m.t, dets: list, ms: { pre: 1, run: 12, post: 0.2, total: 13.2 } });
    await fn({ d, said, workers, bitmaps, tick, answer, jump: (ms) => (skew += ms), src: { width: 320, height: 240 } });
  } finally {
    Object.assign(console, { info, warn });
    performance.now = now;
    if (gpu) Object.defineProperty(globalThis.navigator, "gpu", gpu);
    else delete globalThis.navigator.gpu;
    delete globalThis.Worker;
    delete globalThis.createImageBitmap;
  }
}

// Feeds the same detections through the detector for n frames (one result per frame) and returns what detect() reports.
async function feed({ d, workers, tick, answer, src }, dets, n, t0 = 1000) {
  let out = [];
  for (let k = 0; k < n; k++) {
    d.detect(src, 320, 240, { t: t0 + k * 67 });
    await tick();
    workers.at(-1).reply(answer(typeof dets === "function" ? dets(k) : dets));
    out = d.detect(src, 320, 240, { t: t0 + k * 67 + 10 });
  }
  return out;
}

// ID switches: a ground-truth object matched (IoU >= 0.5) to an output whose id differs from the one it had within the
// last second (an object out of view for longer may fairly come back with a new id).
function run(seq, { cmc = "flow", quality = 0.8, tracker = new Tracker() } = {}) {
  const log = new MotionLog({ hfov: 140 });
  let idsw = 0;
  let matched = 0;
  let total = 0;
  const last = {};
  const lastT = {};
  for (const fr of seq) {
    for (const fl of fr.flows) {
      if (cmc === "flow") log.push(fl.t, { ...fl, quality });
      if (cmc === "yaw") log.push(fl.t, { ...fl, quality: 0.05 }, { yawRate: -fl.dx * 140 * (Math.PI / 180) });
    }
    const out = tracker.update(fr.dets, fr.t, cmc ? log.between(tracker.t, fr.t) : null);
    const used = new Set();
    for (const gt of fr.gts) {
      total++;
      let best = null;
      let bv = 0.5;
      for (const o of out) {
        const v = iou(gt.box, o.box);
        if (!used.has(o.trackId) && v >= bv) [best, bv] = [o, v];
      }
      if (!best) continue;
      matched++;
      used.add(best.trackId);
      if (last[gt.gid] !== undefined && last[gt.gid] !== best.trackId && fr.t - lastT[gt.gid] <= 1000) idsw++;
      last[gt.gid] = best.trackId;
      lastT[gt.gid] = fr.t;
    }
  }
  return { idsw, recall: matched / total };
}

const tests = {
  "preprocessing: /255 then ImageNet mean/std, planar RGB"() {
    const rgba = new Uint8Array([255, 0, 128, 255, 10, 200, 30, 0]);
    const x = toPlanar(rgba, 2);
    const want = [
      [(1 - 0.485) / 0.229, (10 / 255 - 0.485) / 0.229],
      [(0 - 0.456) / 0.224, (200 / 255 - 0.456) / 0.224],
      [(128 / 255 - 0.406) / 0.225, (30 / 255 - 0.406) / 0.225],
    ];
    want.flat().forEach((v, i) => close(x[i], v));
    const out = new Float32Array(6);
    assert.equal(toPlanar(rgba, 2, out), out, "fills a reusable buffer");
  },

  "label map: logit slot = COCO category id, the app's names, unused ids skipped"() {
    assert.equal(COCO_LABELS.length, 80);
    assert.equal(COCO_IDS.length, 80);
    const want = { 1: "person", 17: "cat", 18: "dog", 62: "chair", 63: "couch", 64: "potted plant", 65: "bed", 67: "dining table", 72: "tv", 79: "oven", 81: "sink", 82: "refrigerator" };
    for (const [id, name] of Object.entries(want)) assert.equal(ID2LABEL[id], name);
    for (const id of [0, 12, 26, 29, 30, 45, 66, 68, 69, 71, 83]) assert.equal(ID2LABEL[id], null, `id ${id} unused`);
    assert.deepEqual(ID2LABEL.filter(Boolean), COCO_LABELS);
    assert.equal(detector.COCO_LABELS, COCO_LABELS, "detector.js re-exports the same list");
    assert.equal(detector.toDetectorLabel("my kitty"), "cat");
    assert.equal(detector.toDetectorLabel("the fridge"), "refrigerator");
    assert.equal(detector.toDetectorLabel("keys"), null);
  },

  "box decode: sigmoid scores, cx/cy/w/h -> clamped x/y/w/h, best named class per query"() {
    const dets = decode(
      ...outputs([
        { slots: { 17: 0.95, 18: 0.3 }, box: [0.4, 0.6, 0.2, 0.1] },
        { slots: { 12: 0.99, 1: 0.7 }, box: [0.05, 0.5, 0.2, 0.4] }, // slot 12 is unused: falls to person
        { slots: { 63: 0.08 } }, // below the threshold
      ]),
      { thr: 0.1 },
    );
    assert.equal(dets.length, 2);
    assert.equal(dets[0].label, "cat");
    close(dets[0].score, 0.95);
    for (const [k, v] of Object.entries({ x: 0.3, y: 0.55, w: 0.2, h: 0.1 })) close(dets[0].box[k], v);
    assert.equal(dets[1].label, "person");
    close(dets[1].score, 0.7);
    for (const [k, v] of Object.entries({ x: 0, y: 0.3, w: 0.15, h: 0.4 })) close(dets[1].box[k], v); // clipped at the left edge
  },

  "NMS-free top-k: one detection per query, best first, all above the threshold"() {
    const rand = rng(7);
    const Q = 300;
    const L = Float32Array.from({ length: Q * 91 }, () => (rand() - 0.8) * 12);
    const B = Float32Array.from({ length: Q * 4 }, () => 0.1 + rand() * 0.3);
    const all = decode(L, B, Q, 91, { thr: 0.1, topk: 1000 });
    const top = decode(L, B, Q, 91, { thr: 0.1, topk: 20 });
    assert.ok(all.length > 20 && all.length <= Q, `${all.length} detections`);
    assert.equal(top.length, 20);
    assert.deepEqual(top, all.slice(0, 20));
    for (let i = 1; i < all.length; i++) assert.ok(all[i - 1].score >= all[i].score);
    assert.ok(all.every((d) => d.score >= 0.1 && d.label));
    // the reported score is each query's best named class
    for (let q = 0; q < 5; q++) {
      const best = Math.max(...COCO_IDS.map((id) => L[q * 91 + id]));
      if (sigmoid(best) >= 0.1) assert.ok(all.some((d) => Math.abs(d.score - sigmoid(best)) < 1e-6));
    }
  },

  "tracker: ids hold on a still scene with dropouts, jitter and weak frames"() {
    const r = run(pan({ peak: 0, frames: 60, drop: 0.3, lowShare: 0.2 }), { cmc: null });
    assert.equal(r.idsw, 0);
    assert.ok(r.recall > 0.85, `recall ${r.recall}`);
  },

  "tracker: fast pans keep ids with camera-motion compensation, lose them without (default and RF-DETR tiers)"() {
    for (const tiers of [{}, RF_TIERS]) {
      for (const [peak, maxWith] of [[30 / 512, 1], [60 / 512, 2]]) {
        let without = 0;
        let withCmc = 0;
        for (const seed of [1, 2, 3, 4]) {
          for (const lowShare of [0.1, 0.3]) {
            const seq = pan({ peak, seed, lowShare });
            without += run(seq, { cmc: null, tracker: new Tracker(tiers) }).idsw;
            withCmc += run(seq, { cmc: "flow", tracker: new Tracker(tiers) }).idsw;
          }
        }
        assert.ok(withCmc <= maxWith, `${(peak * 512).toFixed(0)} px/frame: ${withCmc} ID switches with CMC`);
        assert.ok(without >= 4 * Math.max(1, withCmc), `${(peak * 512).toFixed(0)} px/frame: only ${without} without CMC`);
      }
    }
  },

  "tracker: CMC is off when flow quality is low, and the yaw rate can stand in"() {
    const seq = pan({ peak: 60 / 512, seed: 5 });
    const none = run(seq, { cmc: null });
    assert.deepEqual(run(seq, { cmc: "flow", quality: 0.1 }), none, "poor flow = no compensation");
    const yaw = run(seq, { cmc: "yaw" });
    assert.ok(yaw.idsw <= 2 && none.idsw > 4 * Math.max(1, yaw.idsw), `yaw ${yaw.idsw} vs none ${none.idsw}`);
  },

  "tracker: low-score detections keep a track alive (second stage) but never start one"() {
    const box = { x: 0.4, y: 0.3, w: 0.1, h: 0.4 };
    const tr = new Tracker();
    tr.update([{ label: "person", score: 0.9, box }], 0);
    const [a] = tr.update([{ label: "person", score: 0.9, box }], 67);
    assert.ok(a, "confirmed on the second hit");
    for (let f = 2; f < 8; f++) {
      const out = tr.update([{ label: "person", score: 0.2, box }], f * 67);
      assert.equal(out.length, 1);
      assert.equal(out[0].trackId, a.trackId);
    }
    const fresh = new Tracker();
    for (let f = 0; f < 5; f++) assert.deepEqual(fresh.update([{ label: "cat", score: 0.45, box }], f * 67), []);
    assert.equal(fresh.tracks.length, 0);
  },

  "tracker (RF-DETR tiers): a steady dim person or cat is tracked from 0.4, flicker and dim furniture are not"() {
    const tr = new Tracker(RF_TIERS);
    const rand = rng(11);
    const box = { x: 0.3, y: 0.2, w: 0.12, h: 0.45 };
    let id = null;
    for (let f = 0; f < 20; f++) {
      const dim = 0.41 + rand() * 0.06; // the dark-window silhouette: 0.41-0.47
      const flicker = { label: "person", score: 0.4 + rand() * 0.09, box: { x: f % 2 ? 0.05 : 0.7, y: 0.1 + rand() * 0.4, w: 0.08, h: 0.3 } }; // never twice in one place
      const out = tr.update([{ label: "person", score: dim, box }, flicker, { label: "chair", score: 0.48, box: { x: 0.6, y: 0.5, w: 0.2, h: 0.3 } }], f * 67);
      if (f === 0) continue;
      assert.equal(out.length, 1, `frame ${f}: ${JSON.stringify(out.map((o) => [o.label, o.trackId]))}`);
      assert.ok(out[0].score >= 0.4, "find() (0.4) sees it");
      id ??= out[0].trackId;
      assert.equal(out[0].trackId, id);
    }
    const cat = new Tracker(RF_TIERS);
    cat.update([{ label: "cat", score: 0.42, box }], 0);
    assert.equal(cat.update([{ label: "cat", score: 0.43, box }], 67)[0]?.label, "cat");
    const low = new Tracker(RF_TIERS);
    for (let f = 0; f < 5; f++) assert.deepEqual(low.update([{ label: "person", score: 0.38, box }], f * 67), [], "below 0.4 never starts one");
  },

  "tracker: ids are never reused, also after reset()"() {
    const tr = new Tracker();
    const box = { x: 0.3, y: 0.3, w: 0.2, h: 0.2 };
    tr.update([{ label: "cat", score: 0.9, box }], 0);
    const [a] = tr.update([{ label: "cat", score: 0.9, box }], 67);
    tr.reset();
    tr.update([{ label: "dog", score: 0.9, box }], 200);
    const [b] = tr.update([{ label: "dog", score: 0.9, box }], 267);
    assert.ok(b.trackId > a.trackId, `${b.trackId} after ${a.trackId}`);
  },

  "tracker: class-consistent: a dog box on top of a tracked cat gets its own id"() {
    const box = { x: 0.3, y: 0.6, w: 0.15, h: 0.12 };
    const tr = new Tracker();
    tr.update([{ label: "cat", score: 0.9, box }], 0);
    const [cat] = tr.update([{ label: "cat", score: 0.9, box }], 67);
    tr.update([{ label: "dog", score: 0.8, box }], 133);
    const out = tr.update([{ label: "dog", score: 0.8, box }, { label: "cat", score: 0.85, box }], 200);
    const ids = Object.fromEntries(out.map((o) => [o.label, o.trackId]));
    assert.equal(ids.cat, cat.trackId);
    assert.ok(ids.dog && ids.dog !== cat.trackId);
    for (const t of tr.tracks) assert.ok(["cat", "dog"].includes(t.label));
  },

  "tracker: age, best score, and a reported score that fades on weak detections"() {
    const box = { x: 0.2, y: 0.2, w: 0.2, h: 0.5 };
    const tr = new Tracker();
    tr.update([{ label: "person", score: 0.7, box }], 1000);
    tr.update([{ label: "person", score: 0.92, box }], 1067);
    let [o] = tr.update([{ label: "person", score: 0.6, box }], 1133);
    assert.equal(o.age, 133);
    assert.equal(o.hits, 3);
    close(o.best, 0.92);
    close(o.score, 0.92);
    const seen = [];
    for (let f = 0; f < 10; f++) [o] = tr.update([{ label: "person", score: 0.15, box }], 1200 + f * 67), seen.push(o.score);
    for (let i = 1; i < seen.length; i++) assert.ok(seen[i] < seen[i - 1]);
    assert.ok(seen[2] > 0.4 && seen[9] < 0.4, `${seen.map((s) => s.toFixed(2))}`);
    // re-found by a lost track after a short gap keeps the id
    const id = o.trackId;
    tr.update([], 2000);
    tr.update([], 2067);
    [o] = tr.update([{ label: "person", score: 0.8, box }], 2133);
    assert.equal(o.trackId, id);
    tr.update([], 2200);
    tr.update([], 3300); // lost > 1 s: forgotten
    assert.equal(tr.tracks.length, 0);
  },

  "tracker: zoom compensation predicts an object we fly toward"() {
    const fly = (cmc) => {
      const tr = new Tracker();
      const log = new MotionLog();
      const seen = new Set();
      let worst = 1;
      const div = 4.5; // 1/s: the picture grows 35% per detector frame, about the focus point (0.5, 0.6)
      for (let f = 0, g = 1; g * 0.06 < 0.4; f++, g *= Math.exp(div / 15)) {
        const t = (f * 1000) / 15;
        log.push(t, { dx: 0, dy: 0, div, rot: 0, quality: 0.9 }, { focusY: 0.6 });
        const s = 0.06 * g;
        const box = { x: 0.5 + 0.04 * g - s / 2, y: 0.6 - 0.03 * g - s / 2, w: s, h: s };
        const m = cmc ? log.between(tr.t, t) : null;
        if (f > 2) worst = Math.min(worst, iou(tr.predict(t, m)[0].box, box));
        tr.update([{ label: "dog", score: 0.8, box }], t, m).forEach((o) => seen.add(o.trackId));
      }
      return { ids: seen.size, worst };
    };
    const [on, off] = [fly(true), fly(false)];
    assert.equal(on.ids, 1);
    assert.ok(on.worst > 0.9 && off.worst < 0.6, `predicted-box IoU with ${on.worst.toFixed(2)}, without ${off.worst.toFixed(2)}`);
  },

  "tracker: predict() carries boxes on to the display time"() {
    const tr = new Tracker();
    const log = new MotionLog();
    for (let f = 0; f < 6; f++) {
      const x = 0.2 + 0.03 * f;
      tr.update([{ label: "cat", score: 0.9, box: { x, y: 0.5, w: 0.1, h: 0.1 } }], f * 67);
    }
    const [now] = tr.update([{ label: "cat", score: 0.9, box: { x: 0.38, y: 0.5, w: 0.1, h: 0.1 } }], 6 * 67);
    const state = JSON.stringify(tr.tracks);
    const [later] = tr.predict(6 * 67 + 40);
    assert.ok(later.box.x > now.box.x + 0.01, "own velocity");
    log.push(6 * 67 + 20, { dx: -1, dy: 0, div: 0, rot: 0, quality: 0.9 });
    const [panned] = tr.predict(6 * 67 + 40, log.between(6 * 67, 6 * 67 + 40));
    close(panned.box.x, later.box.x - 0.04, 0.002);
    assert.equal(JSON.stringify(tr.tracks), state, "predict left the state alone");
  },

  "motion log: integrates flow rates between two times, yaw stands in, nothing -> null"() {
    const log = new MotionLog({ hfov: 120 });
    log.push(1000, { dx: 0.5, dy: -0.2, div: 0.1, rot: 0.05, quality: 0.9 }, { focusY: 0.7 });
    log.push(1100, { dx: 0.5, dy: -0.2, div: 0.1, rot: 0.05, quality: 0.9 }, { focusY: 0.7 });
    log.push(1200, null, {});
    log.push(1300, { dx: 9, dy: 9, div: 9, rot: 9, quality: 0.1 }, { yawRate: 1 }); // poor flow: yaw instead
    const m = log.between(1000, 1100);
    close(m.dx, 0.05);
    close(m.dy, -0.02);
    close(m.scale, Math.exp(0.01));
    close(m.rot, 0.005);
    assert.equal(m.cy, 0.7);
    close(log.between(1200, 1300).dx, -0.1 / ((120 * Math.PI) / 180));
    assert.equal(log.between(1100, 1200), null, "a gap in the flow is no motion, not a guess");
    close(log.between(1300, 1400).dx, (-0.1 / 120) * (180 / Math.PI), 1e-6); // newest rate carries on
    assert.equal(log.between(null, 1300), null);
    log.push(1500, { dx: 1, dy: 0, div: 0, rot: 0, quality: 0.9, span: 0.1 }); // a 0.1 s flow.js average describes 50 ms ago
    assert.equal(log.samples.at(-1).t, 1450);
    const o4 = new MotionLog(); // the O4 lens, 127 deg across, until calibrated
    o4.push(0, null, { yawRate: 1 });
    close(o4.between(0, 100).dx, -0.1 / (127 * DEG));
    o4.push(200, null, { yawRate: 1, hfov: 150 });
    close(o4.between(200, 300).dx, -0.1 / (150 * DEG));
  },

  async "model store: progress, size and SHA-256 checks, cached copy reused offline"() {
    const body = new Uint8Array(300000).map((_, i) => (i * 7) & 255);
    const spec = { name: "Test", url: "https://example.test/m.onnx", size: body.length, sha256: createHash("sha256").update(body).digest("hex") };
    const store = new Map();
    globalThis.caches = {
      open: async () => ({
        match: async (u) => (store.has(u) ? new Response(store.get(u).slice()) : undefined),
        put: async (u, r) => void store.set(u, new Uint8Array(await r.arrayBuffer())),
        delete: async (u) => store.delete(u),
      }),
    };
    let fetches = 0;
    let serve = body;
    globalThis.fetch = async () => {
      fetches++;
      if (!serve) throw new TypeError("offline");
      const chunks = [serve.slice(0, 100000), serve.slice(100000, 250000), serve.slice(250000)];
      return new Response(new ReadableStream({ pull: (c) => (chunks.length ? c.enqueue(chunks.shift()) : c.close()) }));
    };
    const progress = [];
    const a = await fetchModel(spec, (loaded, total) => progress.push([loaded, total]));
    assert.equal(a.cached, false);
    assert.deepEqual(new Uint8Array(a.bytes), body);
    assert.deepEqual(progress.at(-1), [body.length, body.length]);
    assert.ok(progress.length >= 3);
    assert.ok(store.has(spec.url), "cached");
    serve = null; // offline
    const b = await fetchModel(spec);
    assert.equal(b.cached, true);
    assert.equal(fetches, 1);
    assert.deepEqual(new Uint8Array(b.bytes), body);
    store.set(spec.url, body.slice(0, 1000)); // damaged cache entry, offline: fails, entry dropped
    await assert.rejects(fetchModel(spec), /offline/);
    assert.ok(!store.has(spec.url));
    serve = body;
    await assert.rejects(fetchModel({ ...spec, sha256: "0".repeat(64) }), /checksum/);
    serve = body.slice(0, 200000);
    await assert.rejects(fetchModel(spec), /expected 300000/);
    serve = new Uint8Array(400000);
    await assert.rejects(fetchModel(spec), /larger than expected/);
    assert.equal(store.size, 0, "nothing unverified was cached");
  },

  async "detector: worker path keeps the return shape, adds track ids, one frame in flight, falls back on failures"() {
    await withFakeWorker(async ({ d, said, workers, bitmaps, tick, answer, src }) => {
      const w = workers[0];
      assert.match(w.url, /\/app\/js\/vision\/rfdetr-worker\.js$/);
      assert.equal(w.opts.type, "module");
      const cat = (x) => ({ label: "cat", score: 0.9, box: { x, y: 0.5, w: 0.2, h: 0.2 } });
      assert.deepEqual(d.detect(src, 320, 240, { t: 1000 }), []);
      await tick();
      assert.deepEqual(d.detect(src, 320, 240, { t: 1033 }), [], "nothing until the first result");
      assert.equal(w.pending.length, 1, "one frame in flight");
      w.reply(answer([cat(0.4)]));
      d.detect(src, 320, 240, { t: 1066, element: { videoWidth: 1280, videoHeight: 720 }, region: { sx: 160, sy: 0, sw: 960, sh: 720 } });
      await tick();
      assert.deepEqual(bitmaps.at(-1).slice(1), [160, 0, 960, 720, { resizeWidth: 512, resizeHeight: 384, resizeQuality: "medium" }]);
      w.reply(answer([cat(0.4)]));
      const out = d.detect(src, 320, 240, { t: 1100 });
      assert.equal(out.length, 1);
      assert.deepEqual(Object.keys(out[0]).sort(), ["box", "label", "score", "trackId"]);
      assert.equal(out[0].label, "cat");
      close(out[0].score, 0.9);
      close(out[0].box.x, 0.4, 0.01);
      const id = out[0].trackId;
      // the picture slides left by 0.1 between the detector's frame (t 1066) and now: the box goes with it
      const [moved] = d.detect(src, 320, 240, { t: 1166, flow: { dx: -1, dy: 0, div: 0, rot: 0, quality: 0.9 } });
      assert.equal(moved.trackId, id);
      assert.ok(moved.box.x < 0.35, `box at ${moved.box.x}`);
      assert.deepEqual(d.detect(src, 320, 240, { t: 3000 }), [], "stale results are dropped");
      for (let k = 0; k < 3; k++) {
        await tick();
        if (!w.pending.length) d.detect(src, 320, 240, { t: 3100 + k });
        await tick();
        w.reply((m) => ({ type: "fail", t: m.t, error: "device lost" }));
      }
      await tick();
      assert.ok(said.some((t) => /stopped working \(device lost\); switching to MediaPipe/.test(t)), said.join(" | "));
      assert.notEqual(d.backend, "rfdetr");
    });
  },

  async "GPU budget: while vision localization or live depth runs the detector is held to 8 Hz and the simulator's camera to 24 fps, on average whatever beat the frames come on; caps off when disabled"() {
    const { GpuBudget, BUDGET } = await import("../app/js/vision/budget.js");
    const G = new GpuBudget();
    let t = 10000;
    assert.deepEqual([G.cap("detect", t), G.cap("camera", t), G.heavy(t)], [Infinity, Infinity, false]);
    for (let k = 0; k < 10; k++) G.mark("loc", 40, (t += 200));
    assert.ok(G.heavy(t) && G.cap("detect", t) === BUDGET.detectHz && G.cap("camera", t) === BUDGET.cameraFps);
    const r = G.report(t);
    assert.ok(r.heavy && r.loc.hz === 5 && r.loc.ms === 40 && r.caps.detectHz === BUDGET.detectHz, JSON.stringify(r));
    // offers on a beat (every frame at 30 and 60 fps, every second frame at 30 fps: Perception's), with jitter: the cap on
    // average (a start "after 1/cap s since the last" gets the next offer after it: 5 Hz of 6 at 15 offers a second)
    const rate = (every, kind, jitter = 3) => {
      const slot = {}, r = rng(5);
      let n = 0, t0 = t;
      for (let at = t0; at < t0 + 10000; at += every) {
        for (let k = 0; k < 10; k++) G.mark("loc", 40, at - 200 * k); // localization keeps running
        if (G.take(kind, slot, at + r() * jitter)) n++;
      }
      return n / 10;
    };
    const got = { d15: rate(1000 / 15, "detect"), d30: rate(1000 / 30, "detect"), d60: rate(1000 / 60, "detect"), c60: rate(1000 / 60, "camera") };
    console.log(`      detector at 15, 30, 60 offers/s: ${got.d15}, ${got.d30}, ${got.d60} Hz; camera at 60: ${got.c60} fps`);
    assert.ok([got.d15, got.d30, got.d60].every((v) => Math.abs(v - BUDGET.detectHz) <= 0.25) && Math.abs(got.c60 - BUDGET.cameraFps) <= 0.5, JSON.stringify(got));
    const slot = {};
    assert.ok(G.take("detect", slot, t) && !G.take("detect", slot, t + 60), "not twice in 1/8 s");
    assert.ok(G.take("detect", slot, t + 5000) && !G.take("detect", slot, t + 5001), "after a pause: one, not a burst");
    G.enabled = false;
    assert.ok(G.cap("detect", t) === Infinity && G.take("detect", {}, t) && G.take("detect", slot, t + 5001));
    G.enabled = true;
    assert.ok(!G.heavy(t + 12500), "the marks age out after the window");
  },

  async "detector at its GPU budget: with vision localization running and Perception's offers (every second frame), RF-DETR gets 8 frames a second; the tracker still answers every frame"() {
    const { GPU, BUDGET } = await import("../app/js/vision/budget.js");
    await withFakeWorker(async ({ d, workers, tick, answer, jump, src }) => {
      const w = workers[0], cat = { label: "cat", score: 0.9, box: { x: 0.4, y: 0.5, w: 0.2, h: 0.2 } };
      let sent = 0;
      try {
        for (let k = 0; k < 120; k++) { // 4 s of 30 fps, localization marking 5 Hz meanwhile
          if (k % 6 === 0) GPU.mark("loc", 50);
          if (k % 2 === 0) d.detect(src, 320, 240, { t: performance.now() });
          await tick();
          if (w.pending.length) (sent++, w.reply(answer([cat])));
          jump(1000 / 30);
        }
        console.log(`      ${sent / 4} detector frames a second with localization running (cap ${GPU.cap("detect")} Hz)`);
        assert.ok(Math.abs(sent / 4 - BUDGET.detectHz) <= 0.5, `${sent / 4} a second`);
        assert.equal(d.detect(src, 320, 240, { t: performance.now() }).length, 1, "tracked boxes every frame");
        await tick();
        if (w.pending.length) w.reply(answer([cat]));
        jump(3000);
        sent = 0;
        for (let k = 0; k < 30; k++) {
          d.detect(src, 320, 240, { t: performance.now() });
          await tick();
          if (w.pending.length) (sent++, w.reply(answer([cat])));
          jump(1000 / 30);
        }
        assert.ok(sent >= 25, `without localization or depth: ${sent} frames in 1 s`);
      } finally {
        for (const m of Object.values(GPU.marks)) m.length = 0;
      }
    });
  },

  async "detector: a worker that stops answering is noticed within 3 s and falls back at once"() {
    await withFakeWorker(async ({ d, said, workers, tick, answer, jump, src }) => {
      const w = workers[0];
      const cat = { label: "cat", score: 0.9, box: { x: 0.4, y: 0.5, w: 0.2, h: 0.2 } };
      assert.equal((await feed({ d, workers, tick, answer, src }, [cat], 3)).length, 1);
      d.detect(src, 320, 240, { t: 2000 });
      await tick();
      assert.equal(w.pending.length, 1, "a frame in flight that will never be answered");
      jump(1500);
      d.detect(src, 320, 240, { t: 3500 });
      assert.equal(d.backend, "rfdetr", "not yet: a slow frame is not a hang");
      jump(1600);
      assert.deepEqual(d.detect(src, 320, 240, { t: 5100 }), []);
      assert.equal(d.rf, null);
      assert.ok(w.terminated);
      assert.ok(said.some((t) => /stopped working \(the detector worker stopped answering.*\); switching to MediaPipe/.test(t)), said.join(" | "));
      await tick();
      assert.notEqual(d.backend, "rfdetr");
    });
  },

  async "detector: track ids stay unique across a reconfigure and a fallback"() {
    await withFakeWorker(async (h) => {
      const { d, workers, tick } = h;
      const cat = { label: "cat", score: 0.9, box: { x: 0.4, y: 0.5, w: 0.2, h: 0.2 } };
      const [a] = await feed(h, [cat], 3);
      d.configure({ quality: "accurate" });
      await tick();
      await tick();
      assert.equal(d.model, "RF-DETR-S");
      assert.equal(workers.length, 2);
      const [b] = await feed(h, [cat], 3, 5000);
      assert.ok(b.trackId > a.trackId, `cat#${b.trackId} after the switch, cat#${a.trackId} before`);
      for (let k = 0; k < 3; k++) {
        d.detect(h.src, 320, 240, { t: 6000 + k * 67 });
        await tick();
        workers.at(-1).reply((m) => ({ type: "fail", t: m.t, error: "device lost" }));
      }
      assert.ok(d.tracker.nextId > b.trackId, "the fallback's tracker carries on the numbering");
    });
  },

  async "detector: disposing while a frame is being cut out neither throws nor leaks the bitmap"() {
    await withFakeWorker(async ({ d, workers, tick, src }) => {
      let release;
      let closed = 0;
      globalThis.createImageBitmap = () => new Promise((r) => (release = () => r({ close: () => closed++ })));
      const unhandled = [];
      const onUnhandled = (e) => unhandled.push(e);
      process.on("unhandledRejection", onUnhandled);
      try {
        d.detect(src, 320, 240, { t: 1000 });
        d.configure({ quality: "accurate" });
        release();
        for (let k = 0; k < 4; k++) await tick();
        assert.deepEqual(unhandled, []);
        assert.equal(closed, 1);
        assert.equal(workers[0].pending.length, 0);
      } finally {
        process.off("unhandledRejection", onUnhandled);
      }
    });
  },

  async "detector: stays ready while it reloads, reporting nothing rather than the old boxes"() {
    await withFakeWorker(async (h) => {
      const { d, tick } = h;
      const person = { label: "person", score: 0.9, box: { x: 0.2, y: 0.2, w: 0.2, h: 0.5 } };
      assert.equal((await feed(h, [person], 3)).length, 1);
      d.configure({ quality: "accurate" });
      assert.equal(d.ready, true, "perception keeps calling detect()");
      assert.equal(d.backend, null);
      assert.deepEqual(d.detect(h.src, 320, 240, { t: 1300 }), []);
      await tick();
      await tick();
      assert.equal(d.backend, "rfdetr");
      assert.deepEqual(d.detect(h.src, 320, 240, { t: 1400 }), [], "the new model's first result isn't in yet");
    });
  },

  async "detector: forwards hfov for the yaw-rate stand-in; dim people reach find(), dim chairs don't"() {
    await withFakeWorker(async (h) => {
      const { d } = h;
      d.detect(h.src, 320, 240, { t: 500, flow: { dx: 0, dy: 0, div: 0, rot: 0, quality: 0.05 }, yawRate: 1, hfov: 127 });
      close(d.motion.samples.at(-1).r[0], -1 / (127 * DEG));
      d.detect(h.src, 320, 240, { t: 520, flow: null, yawRate: 1, hfov: 150 });
      close(d.motion.samples.at(-1).r[0], -1 / (150 * DEG));
      const box = { x: 0.3, y: 0.2, w: 0.12, h: 0.45 };
      const out = await feed(h, (k) => [{ label: "person", score: 0.41 + (k % 3) * 0.02, box }, { label: "chair", score: 0.45, box: { x: 0.6, y: 0.5, w: 0.2, h: 0.3 } }], 4);
      assert.deepEqual(out.map((o) => o.label), ["person"]);
      assert.ok(out[0].score >= 0.4);
    });
  },

  async "live depth: a model load that failed once (a dropped download) is tried again by the next warm() and by start(); its frames are timed when they were taken (captured, else decoded minus the video delay), and a frame decoded over 100 ms ago is skipped; an OSD mask vision dropped is let go at once"() {
    const { LiveDepth, DEPTH } = await import("../app/js/vision/depth.js");
    const { OsdMask } = await import("../app/js/nav/osdmask.js");
    const made = [], masks = [], saved = { Worker: globalThis.Worker, createImageBitmap: globalThis.createImageBitmap };
    let fails = 1;
    globalThis.Worker = class {
      constructor() { made.push(this); }
      postMessage(m) {
        if (m.type === "init") setTimeout(() => this.onmessage({ data: fails-- > 0 ? { type: "error", error: "network error while downloading the model" } : { type: "ready", name: "Depth Anything V2 Small", cached: false, loadMs: 5 } }));
        else if (m.type === "run") setTimeout(() => this.onmessage({ data: { id: m.id, type: "ran", t: m.t, ms: { run: 1 }, rgba: new Uint8Array(4) } }));
        else if (m.type === "align") (masks.push(m.mask ?? null), setTimeout(() => this.onmessage({ data: { id: m.id, type: "aligned", a: null, ms: 1 } })));
      }
      terminate() { this.dead = true; }
    };
    globalThis.createImageBitmap = async () => ({ close() {} });
    try {
      const at = [], ctl = { isFlying: () => true, videoDelay: 100, historyAt: () => null };
      const loc = { pose: () => ({ x: 1, y: 2, z: 1, yaw: 0, sigma: 0.05, status: "ok" }), poseAt: (t) => (at.push(t), { x: 1, y: 2, z: 1, yaw: 0 }) };
      const src = { kind: "goggles", ready: () => true, element: () => ({}), region: () => ({ sx: 0, sy: 0, sw: 640, sh: 480 }) };
      const perception = Object.assign(new Emitter(), { source: src, pictureState: () => "whole", region: () => src.region() });
      const twin = { pixelsDepth: async () => ({ depth: new Float32Array(DEPTH.width * DEPTH.height), pixels: new Uint8Array(4) }) };
      const d = new LiveDepth({ perception, localizer: loc, ctl, twin });
      await assert.rejects(d.warm(), /network error/);
      assert.ok(made[0].dead && !d.warmed && !d.estimator && !d.loaded, "the failed load is let go");
      await d.warm();
      assert.ok(d.loaded && made.length === 2);
      await d.start();
      assert.ok(d.estimator === d.warmed && made.length === 2, "start() takes the warmed model");
      const d2 = new LiveDepth({ perception, localizer: loc, ctl, twin });
      fails = 1;
      await assert.rejects(d2.start(), /network error/);
      assert.ok(!d2.estimator);
      await d2.start();
      assert.ok(d2.estimator?.info && made.length === 4, "start() after a failed start loads again");
      const now = performance.now();
      await d.frame({ t: now, decoded: now - 30, detections: [] });
      d.next = 0;
      await d.frame({ t: now, decoded: now - 150, detections: [] });
      d.next = 0;
      await d.frame({ t: now, captured: now - 80, decoded: now - 20, detections: [] });
      assert.deepEqual(at.map((t) => +(now - t).toFixed(3)), [130, 80], "decoded - videoDelay; the late frame skipped; captured when the source knows it");
      // vision's OSD mask (one object, shared) hides the top quarter; vision drops it (its matches collapsed): depth too, at once
      const osd = OsdMask.from({ width: 640, height: 480, rle: [0, 640 * 120, 640 * 360] }), d3 = new LiveDepth({ perception, localizer: loc, ctl, twin, osd });
      await d3.start();
      const share = () => (masks.at(-1) ? +(masks.at(-1).reduce((a, v) => a + v, 0) / masks.at(-1).length).toFixed(2) : 0);
      await d3.frame({ t: performance.now(), decoded: performance.now(), detections: [] });
      const before = share();
      osd.clear();
      d3.next = 0;
      await d3.frame({ t: performance.now(), decoded: performance.now(), detections: [] });
      assert.ok(before > 0.15 && share() === 0, `the depth mask hid ${before}, then ${share()} after the drop`);
    } finally {
      Object.assign(globalThis, saved);
    }
  },

  "model specs are pinned: revision, size, SHA-256, input size"() {
    for (const [key, m] of Object.entries({ "rfdetr-n": [384, 108074865], "rfdetr-s": [512, 114680416] })) {
      const s = MODELS[key];
      assert.match(s.url, /^https:\/\/huggingface\.co\/onnx-community\/rfdetr_(nano|small)-ONNX\/resolve\/[0-9a-f]{40}\/onnx\/model\.onnx$/);
      assert.match(s.sha256, /^[0-9a-f]{64}$/);
      assert.deepEqual([s.res, s.size], m);
    }
  },
};

let failed = 0;
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name}\n      ${e.stack?.split("\n").slice(0, 3).join("\n      ")}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed`);
process.exit(failed ? 1 : 0);
