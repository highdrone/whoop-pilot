// The GPU's time, shared by everything that sees: vision localization (twin renders + XFeat + PnP, nav/splatloc.js),
// live depth (twin renders + Depth Anything, vision/depth.js), the detector (RF-DETR, vision/rfdetr.js), the simulator's
// camera (twin renders, sim/simulator.js), and pictures for people (the 3D view, change evidence, the survey). One GPU
// on a Mac: when they all ask at once the twin worker's queue and the GPU stall, and localization starved of frames is
// what put the drone into walls (wave C1's real-time run: 0.4 fixes/s). The rules:
//   - the twin worker serves its queue by priority (twin/twin.js PRIO): localization, then live depth, then the
//     simulator's camera, then evidence, then the 3D view, then map and database building (the simulator's camera goes
//     between localization and depth while it is under its frame rate: localization and depth work on its frames); a job
//     waiting long moves up one level a second, so nothing waits for ever;
//   - while localization or live depth runs (heavy()), the detector is held to BUDGET.detectHz and the simulator's
//     camera to BUDGET.cameraFps. take() keeps starts on a grid of 1000/cap ms, so a caller offered frames on a beat (the
//     detector every second video frame) gets the cap on average, not the first frame after each 1/cap s (5 of 6 Hz);
//   - everyone marks what they finished (mark(kind, ms)), so report() says what each got: rates, times, caps.
// Localization isn't slowed for live depth: measured on the real drone's load, depth gained nothing from it (3.6 Hz
// either way) and localization lost its headroom for failed solves on real video.
// GPU.enabled = false turns the caps and priorities off (tools/loc-check.html?only=bench&budget=0 compares).
import { Emitter } from "../util.js";

export const BUDGET = { window: 2000, detectHz: 8, cameraFps: 24 }; // the sim camera: 24 so it delivers 20 or more when it can
const KINDS = ["loc", "depth", "detect", "camera", "evidence", "view", "build"];

export class GpuBudget extends Emitter {
  constructor(options = {}) {
    super();
    this.o = { ...BUDGET, ...options };
    this.enabled = true;
    this.marks = Object.fromEntries(KINDS.map((k) => [k, []]));
  }
  // kind finished a job that took ms (wall, from the request to the answer).
  mark(kind, ms = 0, t = performance.now()) {
    const m = (this.marks[kind] ??= []);
    m.push({ t, ms });
    while (m.length && m[0].t < t - this.o.window) m.shift();
  }
  recent(kind, now = performance.now()) {
    const m = this.marks[kind] ?? [];
    while (m.length && m[0].t < now - this.o.window) m.shift();
    return m;
  }
  // Jobs per second over the window.
  rate(kind, now) {
    return this.recent(kind, now).length / (this.o.window / 1000);
  }
  active(kind, now) {
    return this.recent(kind, now).length > 0;
  }
  // Localization or live depth is running: the GPU is theirs first.
  heavy(now) {
    return this.enabled && (this.active("loc", now) || this.active("depth", now));
  }
  // Most per second for kind now (Infinity: no cap).
  cap(kind, now) {
    if (!this.heavy(now)) return Infinity;
    return kind === "detect" ? this.o.detectHz : kind === "camera" ? this.o.cameraFps : Infinity;
  }
  // May kind start a job now? slot: the caller's own { next } (when its rate allows the next start), moved on when it may:
  // starts sit on a grid of 1000/cap ms (a start late on the grid leaves the next one sooner), a grid that fell a whole
  // step behind (idle, or busy) starts again from now. No cap: always.
  take(kind, slot, now = performance.now()) {
    const gap = 1000 / this.cap(kind, now);
    if (!gap) return (slot.next = now), true;
    if (now < (slot.next ?? -Infinity)) return false;
    slot.next = (now - slot.next < gap ? slot.next : now) + gap;
    return true;
  }
  // What each got: { [kind]: { hz, ms (median) } } plus the caps in force and whether the GPU is the heavy users'.
  report(now = performance.now()) {
    const out = { heavy: this.heavy(now), enabled: this.enabled, caps: { detectHz: this.cap("detect", now), cameraFps: this.cap("camera", now) } };
    for (const k of Object.keys(this.marks)) {
      const m = this.recent(k, now);
      if (!m.length) continue;
      const ms = m.map((x) => x.ms).sort((a, b) => a - b);
      out[k] = { hz: +(m.length / (this.o.window / 1000)).toFixed(1), ms: +ms[ms.length >> 1].toFixed(1) };
    }
    return out;
  }
}

export const GPU = new GpuBudget();
