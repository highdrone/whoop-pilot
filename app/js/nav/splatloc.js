// Where the drone is, from its own video against the house's splat twin (docs/HOME-DRONE.md, Wave C).
// track(frame, t): render the twin (no actors, no props: the house as captured) at the localizer's pose for the moment
// the frame was taken, with the rectified pinhole view; XFeat on both; mutual matches; the render's keypoints lifted to
// H with its depth (a half-size depth render); OpenCV PnP (RANSAC EPnP + LM); enough inliers and a small reprojection
// error (or fewer, SPLAT.weakInliers, landing next to the prior: sigma doubled), with free space around it, else rejected; a
// large correction is checked by rendering again at the answer; then localizer.fix({ ..., source: "splat" }), whose
// Mahalanobis gate and conflict rules (localizer.js) have the last word. sigma comes from the PnP covariance, inflated
// for the twin's and the lens's own errors, with a higher floor while the real camera has no verified calibration.
// relocalize(frame, t): DINOv2 descriptor of the frame (DINOv2 loads on first use; tracking never waits for it), the
// nearest keyframes of a database rendered once per house and camera tilt (OPFS /houses/<id>/reloc.bin, built on the
// ground: ensureDb()), PnP against renders at the best few, refined, then a fix that replaces the estimate (or, while
// the localizer has a conflict, one more view that may confirm it). It gives up within SPLAT.relocBudget of the frame
// (the localizer can't carry an older one to now) and tries a fresh frame. confirm(frame, t): a ready conflict is
// solved again against a render 0.5 m away from the second hypothesis.
// Trust: the simulator's video is used as it comes. Real video is advisory (fixes shown, never applied) until
// verifyOnPad() passes with the current lens and framing: the drone on the home pad, its pose reset there, a few frames
// solved there land near the pad, level (SPLAT.pad). A still drone can't tell a lens error from a drone set down off its
// mark (a 4% wrong field of view moves every solve 10 cm the same way), so the pose is re-seated on the camera only with
// a calibration verified for this framing (and never tighter than the pad button's own 5 cm, 3 deg); without one the
// camera must land within a few cm of the mark and the hand placement stands. The framing must be the O4's 4:3 picture
// (perception finds it in the goggles' pillarboxed 16:9 stream) or the one a calib.json was made for; a calib.json for
// another framing is set aside ("calibrate again").
// attach(perception) schedules all of it from its "frame" events: tracking at `rate` Hz (slowRate while sigma is under
// slowSigma) while the pose is known, relocalization (every 2 s at most) while it is lost (the stored database only:
// it is never built in flight), a confirmation when a conflict is ready; one at a time, all heavy work in workers with
// time limits (two timeouts in a row restart the features worker). It also learns the overlay and fisheye border mask
// (osdmask.js) from every third frame while the drone flies or turns, up to OSD_FRAMES of them (or takes a stored one:
// calib.json osdMask), and keeps keypoints off it and off people and pets; a mask hiding more than OSD_MAX of the
// picture is never used, and one after which the matches collapse (SPLAT.osdCollapse) is dropped.
import { Emitter, clamp, wrapAngle } from "../util.js";
import { droneLens, rectLens, intrinsics, cameraOf, bodyOf, calibrateLens, CAM_DZ, RECT_HFOV, DEG } from "./lens.js";
import { Features, DINO } from "./features.js";
import { OsdMask } from "./osdmask.js";
import { LOC } from "./localizer.js";
import { GPU } from "../vision/budget.js";
import { Readiness } from "./readiness.js";

export const SPLAT = {
  rate: 5, // Hz (at most: one at a time)
  slowRate: 3, // Hz while the localizer's sigma is under slowSigma
  slowSigma: 0.05,
  minInliers: 120, // accepted fix (640x480 rectified)
  weakInliers: 60, // fewer, down to this: accepted only next to the prior (weakNear m, weakTurn), sigma x weakInflate
  weakNear: 0.1,
  weakTurn: 3 * DEG,
  weakInflate: 2,
  relocInliers: 150,
  relocRefine: 40, // a keyframe's solve with this many inliers (not enough to accept) is solved again where it puts the drone
  relocBudget: LOC.history - 2000, // ms from the frame: then a fresh frame
  confirmAway: 0.5, // m between a conflict's second hypothesis and the render that confirms it
  confirmInliers: 60,
  maxRms: 2.5, // px
  thr: 4, // px, RANSAC
  refineMove: 0.12, // m (or refineTurn): render again at the answer and solve again
  refineTurn: 5 * DEG,
  inflate: 2.5, // PnP covariance -> fix sigma (model errors: lens, twin)
  sigmaFloor: 0.025, // m
  yawFloor: 0.6 * DEG,
  rawSigmaFloor: 0.15, // m, and rawYawFloor: the real camera without a verified calibration for this framing
  rawYawFloor: 3 * DEG,
  relocEvery: 2000, // ms between relocalization attempts while lost
  advisoryEvery: 30000, // ms: while real video is advisory, a find is shown and looked for again only this often
  candidates: 8,
  backend: "auto", // features worker: "webgpu" | "wasm" | "auto"
  // verifyOnPad: frames, accepted at least; the solves must agree with each other (spread, spreadYaw: a steady picture),
  // sit level (tilt: a wrong camera angle shows as a tilted drone) and lie within near, nearYaw of the pad with a calibration
  // verified for this framing, else within rawNear, rawNearYaw; a re-seat's sigma at least sigma, yawSigma (the pad
  // button's); calibrate()'s check: median error pos, headings yaw apart at most
  pad: { n: 6, accept: 4, spread: 0.02, spreadYaw: 1 * DEG, tilt: 3 * DEG, near: 0.15, nearYaw: 10 * DEG, rawNear: 0.05, rawNearYaw: 3 * DEG, sigma: 0.05, yawSigma: 3 * DEG, pos: 0.05, yaw: 2 * DEG },
  osdCollapse: { tries: 4, inliers: 30 }, // a new overlay mask is dropped when the next tries' median inliers fall under this
  aspect: 4 / 3, // the O4's picture
  aspectTol: 0.005,
  regionTol: 2, // px: a calib.json's framing
  staleMs: 100, // a frame decoded longer ago than this is too old to time well
  retry: [30000, 120000], // ms before trying the features worker again after a failure (then the last, repeated)
  renderMs: 5000, // a twin render's time limit
  depthScale: 0.5, // the depth render's size relative to the colour one
};
const VIEW = { width: 640, height: 480 }, DB_VIEW = { width: 320, height: 240 };
const SRC_W = 640; // frames are scaled to this width before they go to the worker
const OSD_FRAMES = 600; // moving frames the overlay mask learns from; then it stays as it is
const LAG = { step: 10, n: 31, minRate: 0.4 }; // video delay check: offsets -150..150 ms, turns faster than 0.4 rad/s
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const median = (a) => { const b = a.filter(Number.isFinite).sort((x, y) => x - y); return b.length ? b[b.length >> 1] : null; };
const sameRegion = (a, b, tol) => ["sx", "sy", "sw", "sh"].every((k) => Math.abs((a[k] ?? 0) - (b[k] ?? 0)) <= tol);

export class SplatLocalizer extends Emitter {
  constructor({ twin = null, localizer, lens = null, calib = null, settings = null, house = null, map = null, ctl = localizer?.ctl, features = null, store = null, options = {} }) {
    super();
    Object.assign(this, { twin, localizer, settings, house, map, ctl, features, store, db: null, busy: false, enabled: true });
    this.o = { ...SPLAT, ...options };
    this.stats = { fixes: 0, rejects: 0, relocs: 0, relocFails: 0, advisory: 0, timeouts: 0, ms: 0, inliers: 0, rms: 0, lastError: null, last: null, recent: [] };
    this.calib = calib;
    this.region = null;
    this.calibOk = false;
    this.pad = null;
    this.framing = "";
    if (lens) this.setLens(lens);
    else this.applyCalib();
    Object.assign(this, { lastStart: -Infinity, lastReloc: -Infinity, retryAt: -Infinity, dinoRetryAt: -Infinity, failures: 0, boxes: [] });
    this.osd = new OsdMask(); // one object for good: live depth holds it too (setOsdMask copies into it)
    const m = calib?.osdMask && usableMask(calib.osdMask);
    if (m) this.osd.assign(m);
    Object.assign(this, { osdFrames: 0, osdSent: -1, osdDrops: 0, osdWatch: null, inlierLog: [] });
  }

  // The drone lens (twin/lens.js object; calib.json goes through droneLens()). The rectified view keeps its uptilt.
  setLens(lens) {
    this.lens = lens;
    this.uptilt = lens.uptiltDeg ?? 20;
    this.view = { ...VIEW, lens: rectLens(this.uptilt), actors: false, props: false, prio: "loc" };
    this.depthView = { ...this.view, width: Math.round(VIEW.width * this.o.depthScale), height: Math.round(VIEW.height * this.o.depthScale) };
    this.K = intrinsics(this.view.lens, VIEW.width, VIEW.height);
    this.rect = null;
  }

  // calib.json (lens.js; null: the O4 default at the settings' uptilt), with its overlay mask when it has one. It applies
  // to the framing it was made for (calib.region) only.
  setCalib(calib) {
    this.calib = calib;
    this.warnedCalib = false;
    this.applyCalib();
    const m = calib?.osdMask && usableMask(calib.osdMask);
    if (m) this.setOsdMask(m).catch(() => {});
  }
  applyCalib() {
    const c = this.calib?.fx ? this.calib : null, r = this.region, match = !!c && (!c.region || !r || sameRegion(c.region, r, this.o.regionTol));
    this.calibOk = match && !!c.region && !!c.verified;
    this.setLens(droneLens(match ? c : null, this.settings?.get?.("uptilt") ?? 20));
    if (c && !match && !this.warnedCalib) {
      this.warnedCalib = true;
      this.emit("status", { level: "warn", text: `Calibrate the camera again: the video framing changed since it was calibrated (${c.region.sw}x${c.region.sh} then, ${r.sw}x${r.sh} now).` });
    }
  }

  setTwin(twin) {
    this.twin = twin;
  }

  setHouse(house, map) {
    if (house?.id !== this.house?.id) Object.assign(this, { db: null, dbTried: false, dbBuild: null, pad: null });
    Object.assign(this, { house, map });
  }

  // The features worker (XFeat, matcher, OpenCV; DINOv2 later), made once; after a failure not again for a while.
  async ready() {
    if (this.features && this.features.alive === false) this.features = null;
    if (this.features) return this.features;
    if (performance.now() < this.retryAt) throw Object.assign(new Error(`vision localization is unavailable (${this.unavailable})`), { reported: true });
    try {
      this.features = await (this.featuresLoading ??= Features.create({ backend: this.o.backend }).finally(() => (this.featuresLoading = null)));
      Object.assign(this, { failures: 0, unavailable: null, rect: null, osdSent: -1 });
      this.features.onProgress = (p) => this.dinoProgress(p);
      return this.features;
    } catch (e) {
      const wait = this.o.retry[Math.min(this.failures++, this.o.retry.length - 1)];
      Object.assign(this, { retryAt: performance.now() + wait, unavailable: e.message.split("\n")[0] });
      this.emit("status", { level: "warn", text: `Vision localization is unavailable (${this.unavailable}); trying again in ${Math.round(wait / 1000)} s.` });
      throw Object.assign(e, { reported: true });
    }
  }

  // DINOv2's first download ({ what, loaded, total }): a status that updates one line (key, progress), every 5% at most,
  // 100% once; a download started again (after a failure) shows from its start.
  dinoProgress(p) {
    const pct = Math.floor((100 * p.loaded) / p.total), last = this.dinoPct ?? -5;
    if (pct === last || (pct > last && pct < 100 && pct - last < 5)) return;
    this.dinoPct = pct;
    this.emit("status", { level: "info", key: "vision-model", text: `Downloading ${p.what} for relocalization (once): ${pct}% of ${Math.round(p.total / 1e6)} MB`, progress: p.loaded / p.total });
  }

  // DINOv2 (relocalization only), loaded on first use; a failure disables relocalization for a while, not tracking.
  async dino() {
    const f = await this.ready();
    if (f.info.dinoLoaded) return f;
    if (performance.now() < this.dinoRetryAt) throw Object.assign(new Error(`relocalization is unavailable (${this.dinoError})`), { reported: true });
    try {
      await f.loadDino();
      return f;
    } catch (e) {
      Object.assign(this, { dinoRetryAt: performance.now() + this.o.retry.at(-1), dinoError: e.message.split("\n")[0] });
      this.emit("status", { level: "warn", text: `Relocalization is unavailable (${this.dinoError}); tracking goes on.` });
      throw Object.assign(e, { reported: true });
    }
  }

  // A worker call failed: two timeouts in a row restart the worker (a GPU stall or a lost device).
  trouble(e) {
    if (!e?.timeout && !/timed out|didn't (answer|render)/.test(e?.message ?? "")) return;
    this.stats.timeouts++;
    if (this.features && (this.features.timeouts ?? 0) >= 2) {
      this.features.dispose?.();
      this.features = null;
      this.emit("status", { level: "warn", text: "The vision worker stopped answering; restarting it." });
    }
  }
  release(ids) {
    Promise.resolve(this.features?.release(ids)).catch(() => {});
  }
  // The next video frame (or ms, whichever comes first: a page's timers are slow while it is hidden, frames aren't).
  nextFrame(ms = 150) {
    let off = null;
    return Promise.race([new Promise((r) => (off = this.perception?.on?.("frame", r))), sleep(ms)]).finally(() => off?.());
  }
  async idle(ms = 2000, signal = null) { // wall time (the page's clock may be a simulation's)
    for (const t0 = Date.now(); this.busy && Date.now() - t0 < ms && !signal?.aborted;) await sleep(20);
    return !this.busy;
  }

  // A rectifier for this source size (the worker keeps it).
  async rectFor(w, h) {
    const key = `${w}x${h}:${JSON.stringify(this.lens)}`;
    if (this.rect?.key !== key) {
      this.rect = { key, ...(await this.features.rectifier({ key, lens: this.lens, srcW: w, srcH: h, outW: VIEW.width, outH: VIEW.height, hfov: RECT_HFOV })) };
      if (this.osd.usable) await this.features.setMask({ key, ...this.osdMaskData() });
    }
    return this.rect.key;
  }

  // An OsdMask (osdmask.js) to keep keypoints off the overlay and the border, copied into this.osd (live depth holds the
  // same object); one hiding more than OSD_MAX of the picture is refused. -> whether it is in use.
  async setOsdMask(osd) {
    if (!osd.usable) return false;
    this.osd.assign(osd);
    if (this.rect && this.features) this.osdCoverage = (await this.features.setMask({ key: this.rect.key, ...this.osdMaskData() })).masked;
    return true;
  }
  // The new mask's first tries (accept()): their matches collapsed -> the mask goes (learned again, once more at most),
  // the worker keeps only the fisheye border.
  watchOsd(inliers) {
    const L = this.inlierLog, w = this.osdWatch, C = this.o.osdCollapse;
    L.push(inliers ?? 0);
    if (L.length > 8) L.shift();
    if (!w || (w.after.push(inliers ?? 0), w.after.length < C.tries)) return;
    this.osdWatch = null;
    const after = median(w.after);
    if (after >= C.inliers || (w.before != null && after >= 0.25 * w.before)) return;
    this.osd.clear();
    this.osdDrops++;
    if (this.rect && this.features) Promise.resolve(this.features.setMask({ key: this.rect.key, width: 1, height: 1, data: new Uint8Array([1]) })).catch(() => {});
    this.emit("status", { level: "info", text: `Vision localization stopped using its on-screen display mask: the camera matched the 3D scan far worse with it (${Math.round(after)} points, ${w.before == null ? "none before" : `${Math.round(w.before)} before`}).` });
  }
  osdMaskData() {
    const m = this.osd.mask();
    return { width: this.osd.width, height: this.osd.height, data: m };
  }

  // ---------------------------------------------------------------- trust and framing

  // "trusted" (the simulator's video, or calls without a perception: checks, replays), "verified" (real video, the pad
  // check passed with this lens and framing) or "advisory" (real video not checked, or matched on the CPU: fixes are
  // shown, never applied).
  trust() {
    const src = this.perception?.source;
    if (!src || src.kind === "sim") return "trusted";
    return this.pad?.ok && this.pad.key === this.lensKey() && !this.cpuOnly() ? "verified" : "advisory";
  }
  // Why the vision worker is too slow for the real drone ("" when it isn't, or before it started): without WebGPU XFeat
  // and the matcher run on the CPU (WASM), about 400 ms a fix, 2 a second at best, which the real drone can't fly on.
  cpuOnly() {
    const i = this.features?.info;
    if (!i?.backend) return "";
    const slow = i.backend !== "webgpu" ? "this browser has no WebGPU" : i.fallbacks?.find((f) => /^(XFeat|the matcher)/.test(f));
    return slow ? `vision localization runs on the CPU here (${slow}): too slow for the real drone (about 2 fixes a second); use Chrome with WebGPU on this Mac` : "";
  }
  lensKey() {
    const r = this.region;
    return `${JSON.stringify(this.lens)}|${r ? `${r.sx},${r.sy},${r.sw},${r.sh}` : ""}`;
  }
  // The real camera without a verified calibration for this framing: higher sigma floors.
  get raw() {
    const src = this.perception?.source;
    return !!src && src.kind !== "sim" && !this.calibOk;
  }

  // The picture region of a frame: a new one re-checks the calib.json and the framing. -> "" or why vision can't use it.
  useRegion(f) {
    const r = f.region ?? { sx: 0, sy: 0, sw: f.element.width || f.element.videoWidth, sh: f.element.height || f.element.videoHeight };
    const key = `${r.sx},${r.sy},${r.sw},${r.sh}`, sim = this.perception?.source?.kind === "sim";
    if (key !== this.regionKey) {
      this.regionKey = key;
      this.region = { sx: r.sx, sy: r.sy, sw: r.sw, sh: r.sh, frame: [f.element.width || f.element.videoWidth, f.element.height || f.element.videoHeight] };
      this.applyCalib();
    }
    const why = sim ? "" : this.framingOf(r, f.picture);
    if (why !== this.framing && why) this.emit("status", { level: "warn", text: `Vision localization is paused: ${why}.` });
    return (this.framing = why);
  }
  // Why a picture region (picture: false while perception still looks for it) can't be used by real video: "" when it can
  // (the O4's 4:3 picture, or the framing a calib.json was made for). No side effects (the checklist asks too).
  framingOf(r, picture = true) {
    if (this.calib?.fx && this.calib.region && sameRegion(this.calib.region, r, this.o.regionTol)) return "";
    return picture === false ? "the camera picture hasn't been found in the video yet"
      : Math.abs(r.sw / r.sh / this.o.aspect - 1) > this.o.aspectTol ? `the video isn't the camera's 4:3 picture (it is ${Math.round(r.sw)}x${Math.round(r.sh)}): set the O4's camera to 4:3 in the goggles, and remove any crop of the video` : "";
  }

  // ---------------------------------------------------------------- scheduling

  attach(perception) {
    this.detach();
    this.perception = perception;
    this.unsub = perception.on("frame", (f) => this.onFrame(f));
    this.checklist({ perception }).attach(perception);
  }

  // "Ready to fly?" (nav/readiness.js): the vision items of the app's checklist for { mode }, and prepare({ onProgress,
  // signal }) for everything before the battery goes in. checklist({ depth, perception, prepare3D, has3D }) gives it live
  // depth, the detector (perception.detector) and main.js's 3D map build.
  checklist(parts) {
    const r = (this.readyCheck ??= new Readiness({ splat: this, perception: this.perception, settings: this.settings }));
    if (parts) r.set(parts);
    return r;
  }
  readiness(opts) {
    return this.checklist().items(opts);
  }
  prepare(opts) {
    return this.checklist().prepare(opts);
  }
  detach() {
    this.unsub?.();
    this.unsub = null;
  }

  onFrame(f) {
    const now = performance.now(), loc = this.localizer;
    if (!this.enabled || !f?.element) return;
    if (this.useRegion(f)) return;
    if (!this.osd.fixed && this.osd.frames < OSD_FRAMES && this.osdDrops < 2 && (this.ctl?.isFlying?.() || this.ctl?.est?.rotating) && this.osdFrames++ % 3 === 0) this.learnOsd(f);
    this.lagFlush();
    if (this.busy || this.calibrating || !this.twin || loc.source !== "fused" || now < this.retryAt) return;
    if (f.decoded != null && f.t - f.decoded > this.o.staleMs) return; // a late frame: its time is a guess
    const p = loc.pose(), asked = !p.conflict && !!loc.needFix?.(), lost = p.status === "lost" || asked, rate = p.sigma < this.o.slowSigma ? this.o.slowRate : this.o.rate;
    if (asked && this.askSeen !== loc.fixAsked) (this.askSeen = loc.fixAsked), (this.lastReloc = -Infinity); // safety.js asked: try at once
    if (lost && !this.db && !this.dbTried) (this.dbTried = true), this.loadDb().catch(() => null); // the stored one only
    const relocDue = lost && this.db && now - this.lastReloc >= this.o.relocEvery, due = now - this.lastStart >= 1000 / rate;
    // truly lost: only relocalization helps; in a conflict: confirm when it is ready, else keep looking (and relocalize)
    const job = lost && !p.conflict ? (relocDue ? "reloc" : null) : !due ? null : p.conflict?.ready ? "confirm" : relocDue ? "reloc" : "track";
    if (!job) return;
    const t = f.captured ?? (f.decoded ?? f.t) - (this.ctl?.videoDelay ?? 0), r = this.region, sr = f.sourceRegion ?? r;
    this.boxes = (f.detections ?? []).filter((d) => ["person", "cat", "dog"].includes(d.label)).map(({ box: b }) => ({ x: (sr.sx + b.x * sr.sw - r.sx) / r.sw, y: (sr.sy + b.y * sr.sh - r.sy) / r.sh, w: (b.w * sr.sw) / r.sw, h: (b.h * sr.sh) / r.sh }));
    const image = grab(f);
    if (!image) return;
    (job === "reloc" ? this.relocalize(image, t).then((o) => o.stale && (this.lastReloc = -Infinity)) : job === "confirm" ? this.confirm(image, t) : this.track(image, t))
      .catch((e) => this.fail(e)).finally(() => GPU.mark("loc", performance.now() - now));
  }

  // The overlay mask learns from grey copies of the frames (the drone flying or turning: a still picture says nothing);
  // the worker gets it when ready and every 100 more, if it hides no more than OSD_MAX, and the next tries are watched.
  learnOsd(f) {
    const o = this.osd, r = f.region ?? { sx: 0, sy: 0, sw: f.element.width, sh: f.element.height };
    const g = (this.osdCtx ??= new OffscreenCanvas(o.width, o.height).getContext("2d", { willReadFrequently: true }));
    g.drawImage(f.element, r.sx, r.sy, r.sw, r.sh, 0, 0, o.width, o.height);
    const px = g.getImageData(0, 0, o.width, o.height).data, gray = new Float32Array(o.width * o.height);
    for (let i = 0; i < gray.length; i++) gray[i] = 0.299 * px[4 * i] + 0.587 * px[4 * i + 1] + 0.114 * px[4 * i + 2];
    if (o.add(gray) && o.ready && (this.osdSent < 0 || o.frames - this.osdSent >= 100) && this.features && this.rect) {
      this.osdSent = o.frames;
      this.installOsd(o);
    }
  }
  // A learned mask into use (if it hides no more than OSD_MAX), the next tries watched (watchOsd).
  installOsd(o) {
    if (!o.usable) return false;
    this.osdWatch = { before: this.inlierLog.length >= 3 ? median(this.inlierLog.slice(-5)) : null, after: [] };
    this.setOsdMask(o).catch(() => {});
    return true;
  }

  // One status line per distinct problem.
  fail(e) {
    this.trouble(e);
    const msg = e.message.split("\n")[0];
    this.stats.lastError = msg;
    if (msg === this.lastFail || e.reported) return;
    this.lastFail = msg;
    this.emit("status", { level: "warn", text: `Splat localization: ${msg}` });
  }

  // ---------------------------------------------------------------- tracking

  // frame: ImageBitmap (or canvas, video, ImageData-like) of the drone camera, its fisheye as `lens`; t: when it was
  // taken (performance.now ms). -> { ok, advisory?, reason?, pose?, inliers, ms }.
  async track(frame, t) {
    if (this.busy) return { ok: false, reason: "busy" };
    this.busy = true;
    const t0 = (this.lastStart = performance.now());
    try {
      await this.ready();
      const prior = this.localizer.poseAt(t), body = this.bodyGuess(prior, t), renders = this.renderAt(body);
      renders.catch(() => {});
      const live = await this.liveFrame(frame); // the twin renders meanwhile
      try {
        const r = await this.solveAt(live.id, body, { renders });
        const out = await this.accept(r, prior, t, t0, { live });
        out.parts = { live: live.ms, ...r.ms };
        return out;
      } finally {
        this.release(live.id);
      }
    } catch (e) {
      this.trouble(e);
      throw e;
    } finally {
      this.busy = false;
    }
  }

  // The body pose to render from: the localizer's position and yaw, the controller's attitude estimate.
  bodyGuess(p, t) {
    const h = this.ctl?.historyAt?.(t), e = this.ctl?.est;
    return { x: p.x, y: p.y, z: p.z, yaw: p.yaw, pitch: h?.p ?? e?.pitchAngle ?? 0, roll: h?.q ?? e?.rollAngle ?? 0 };
  }

  async liveFrame(frame, { dino = false } = {}) {
    const image = await toBitmap(frame), key = await this.rectFor(image.width, image.height);
    return this.features.frame({ image, rect: key, boxes: this.boxes, dino });
  }

  // The twin's expected picture and depth (half size) at a body pose (the rectified view), within renderMs: one job for
  // both, first in the twin's queue (vision/budget.js).
  renderAt(body) {
    const pose = { ...body, z: body.z + CAM_DZ }, t = this.twin;
    return Promise.race([t.pixelsDepth ? t.pixelsDepth(pose, this.view, this.depthView).then((r) => [r.pixels, r.depth]) : Promise.all([t.pixels(pose, this.view), t.depth(pose, this.depthView)]),
      sleep(this.o.renderMs).then(() => { throw Object.assign(new Error("the 3D twin didn't render in time"), { timeout: true }); })]);
  }

  // Render at a body pose (or take renders already started), match the live frame (id) against it, PnP.
  // -> { body, sol, ref, ms }.
  async solveAt(liveId, body, { viz = false, renders = null } = {}) {
    const t0 = performance.now(), [rgba, depth] = await (renders ?? this.renderAt(body));
    const t1 = performance.now(), cam = cameraOf(body, this.uptilt), dv = this.depthView;
    const ref = await this.features.render({ rgba, depth, depthW: dv.width, depthH: dv.height, K: this.K, cam, keepPx: viz });
    const sol = await this.features.solve({ live: liveId, ref: ref.id, K: this.K, thr: this.o.thr, viz });
    if (!viz) this.release(ref.id);
    return { body, sol, ref, ms: { twin: t1 - t0, ref: ref.ms, ...sol.ms } };
  }

  quality(sol, min = this.o.minInliers) {
    if (!sol.ok) return `no pose (${sol.matches} matches, ${sol.lifted} with depth)`;
    if (sol.inliers < min) return `${sol.inliers} inliers`;
    if (sol.rms > this.o.maxRms) return `reprojection ${sol.rms.toFixed(1)} px`;
    return null;
  }

  // A solve -> checks, refinement, the fix. extra: { live } (its id, for a refining solve), reloc (replace the
  // estimate), confirm (a solve from another viewpoint for a conflict: no refinement, fewer inliers will do).
  async accept(r, prior, t, t0, { live, reloc = false, confirm = false } = {}) {
    let { sol } = r, why = this.quality(sol, confirm ? this.o.confirmInliers : this.o.minInliers), est = sol.ok ? bodyOf(sol.R, sol.C, this.uptilt) : null, refined = false, weak = false;
    this.watchOsd(sol.ok ? sol.inliers : 0);
    if (why && !reloc && !confirm && est && sol.inliers >= this.o.weakInliers && sol.rms <= this.o.maxRms && prior &&
        Math.hypot(est.x - prior.x, est.y - prior.y) <= Math.min(0.2, Math.max(this.o.weakNear, 1.5 * (prior.sigma ?? 0))) && Math.abs(wrapAngle(est.yaw - prior.yaw)) <= this.o.weakTurn)
      [why, weak] = [null, true]; // few inliers, but where the drone should be
    if (!why && !weak && live && (Math.hypot(est.x - r.body.x, est.y - r.body.y, est.z - r.body.z) > this.o.refineMove || Math.abs(wrapAngle(est.yaw - r.body.yaw)) > this.o.refineTurn)) {
      const r2 = await this.solveAt(live.id, { ...est, pitch: clamp(est.pitch, -0.6, 0.6), roll: clamp(est.roll, -0.6, 0.6) });
      const why2 = this.quality(r2.sol), est2 = r2.sol.ok ? bodyOf(r2.sol.R, r2.sol.C, this.uptilt) : null;
      if (why2) why = `refine: ${why2}`;
      else if (Math.hypot(est2.x - est.x, est2.y - est.y) > 0.25) why = "refine disagrees";
      else [sol, est, refined] = [r2.sol, est2, true];
    }
    const { sigma, yawSigma } = sol.ok ? this.sigmaOf(sol, weak) : {};
    if (!why) why = this.plausible(est, sigma);
    const ms = performance.now() - t0, s = this.stats;
    if (why) {
      s.rejects++;
      s.lastError = why;
      this.emit("reject", { reason: why, inliers: sol.inliers, t, ms, pose: est });
      return { ok: false, reason: why, inliers: sol.inliers, ms, sol, pose: null, rejected: est };
    }
    const fix = { x: est.x, y: est.y, z: est.z, yaw: est.yaw, sigma, yawSigma, t, source: "splat", inliers: sol.inliers, rms: +sol.rms.toFixed(2), ms: Math.round(ms), reloc, ...(weak && { weak }) };
    const advisory = this.trust() === "advisory", used = !advisory && this.localizer.fix({ ...fix, ...(confirm && { confirm }) });
    Object.assign(s, { ms: s.ms ? s.ms * 0.8 + ms * 0.2 : ms, inliers: sol.inliers, rms: sol.rms, last: { ...fix, used, advisory, refined, at: performance.now() } });
    if (advisory) {
      s.advisory++;
      this.emit("fix", { pose: est, sigma, yawSigma, inliers: sol.inliers, rms: sol.rms, ms, t, refined, reloc, advisory: true });
      return { ok: true, advisory: true, pose: est, sigma, yawSigma, inliers: sol.inliers, rms: sol.rms, ms, refined, weak, sol };
    }
    s.recent.push(performance.now());
    while (s.recent.length && s.recent[0] < performance.now() - 5000) s.recent.shift();
    if (!used) {
      const refused = this.localizer.fixes?.last?.why ?? "gate";
      s.rejects++;
      s.lastError = refused === "stale" ? `took ${Math.round(performance.now() - t)} ms from the frame, longer than the position history` : refused === "source" ? "the localizer isn't taking fixes" : "disagrees with the position estimate";
      this.emit("reject", { reason: s.lastError, inliers: sol.inliers, t, ms, pose: est });
      return { ok: false, reason: s.lastError, refused, pose: est, inliers: sol.inliers, ms, sol };
    }
    s.fixes++;
    s.lastError = null;
    this.lagSample(fix);
    this.emit("fix", { pose: est, sigma, yawSigma, inliers: sol.inliers, rms: sol.rms, ms, t, refined, reloc, confirm, weak });
    return { ok: true, pose: est, sigma, yawSigma, inliers: sol.inliers, rms: sol.rms, ms, refined, weak, sol };
  }

  // Not a place the drone can be: tilted beyond flying, off the floor plan, under the floor or over the ceiling, or (with
  // the 3D map: house/voxels.js) no free voxel within 2 sigma (at least 5 cm): inside something solid or in space the
  // scan never saw, between 0.25 and 1.65 m above the floor where the map is carved. The 2.5D map alone can't say (its
  // bands mark a table top 0.3 m under the drone as blocking: 18 of 356 good fixes in a flight were rejected that way).
  plausible(p, sigma = 0.05) {
    if (Math.abs(p.pitch) > 0.8 || Math.abs(p.roll) > 0.8) return "tilted beyond what the drone can fly";
    const m = this.map;
    if (!m) return null;
    const floor = m.floorAt(p.x, p.y), ceil = m.ceilingAt(p.x, p.y);
    if (floor == null) return "outside the house";
    if (p.z < floor - 0.15 || p.z > (Number.isFinite(ceil) ? ceil : floor + 3) + 0.1) return "below the floor or above the ceiling";
    const v = m.vox, h = p.z - floor;
    if (!v?.state || h < 0.25 || h > 1.65) return null;
    const d = Math.max(0.05, 2 * sigma), r = d / Math.SQRT2;
    const near = [[0, 0, 0], [d, 0, 0], [-d, 0, 0], [0, d, 0], [0, -d, 0], [0, 0, d], [0, 0, -d], [r, r, 0], [r, -r, 0], [-r, r, 0], [-r, -r, 0]];
    return near.some(([dx, dy, dz]) => v.state(p.x + dx, p.y + dy, p.z + dz) === 1) ? null : "inside an obstacle or space the scan never saw";
  }

  sigmaOf(sol, weak = false) {
    const c = sol.cov, a = c?.[0][0] ?? 1, b = c?.[0][1] ?? 0, d = c?.[1][1] ?? 1, raw = this.raw, k = weak ? this.o.weakInflate : 1;
    const sxy = Math.sqrt(Math.max(0, (a + d) / 2 + Math.sqrt(((a - d) / 2) ** 2 + b * b)));
    return { sigma: clamp(k * (this.o.inflate * sxy + (raw ? this.o.rawSigmaFloor : this.o.sigmaFloor)), 0.02, 0.5),
      yawSigma: clamp(k * (this.o.inflate * Math.sqrt(Math.max(0, c?.[5][5] ?? 1)) + (raw ? this.o.rawYawFloor : this.o.yawFloor)), 0.3 * DEG, 20 * DEG) };
  }

  // Fixes in the last 5 s, per second.
  get rate() {
    return this.stats.recent.length / 5;
  }

  // Video delay check: during turns, each used fix's yaw against the filter's yaw at its time + d (d -150..150 ms): the d
  // that fits best says how far the frame times are off (negative: the frames are older than videoDelay says). A fix
  // waits in lagQueue until the filter's history covers its time + 150 ms.
  lagSample(f) {
    (this.lagQueue ??= []).push({ t: f.t, yaw: f.yaw });
    this.lagFlush();
  }
  lagFlush() {
    const loc = this.localizer, end = loc.trail?.at(-1)?.t, Q = this.lagQueue, half = LAG.n >> 1;
    if (!loc.poseAt || end == null || !Q?.length) return;
    while (Q.length && Q[0].t + LAG.step * half <= end) {
      const f = Q.shift(), a = loc.poseAt(f.t - 60), b = loc.poseAt(f.t + 60);
      if (Math.abs(wrapAngle(b.yaw - a.yaw)) / 0.12 < LAG.minRate || f.t < loc.trail[0].t + LAG.step * half) continue;
      const L = (this.lag ??= { s1: new Float64Array(LAG.n), s2: new Float64Array(LAG.n), n: 0 });
      for (let k = 0; k < LAG.n; k++) {
        const r = wrapAngle(f.yaw - loc.poseAt(f.t + (k - half) * LAG.step).yaw);
        L.s1[k] += r;
        L.s2[k] += r * r;
      }
      L.n++;
    }
  }
  // -> { n (fixes in turns), offset (ms), videoDelay (what it should be), rmsDeg } or null before 20 such fixes.
  delayCheck() {
    this.lagFlush();
    const L = this.lag;
    if (!L || L.n < 20) return null;
    let best = 0, bv = Infinity;
    for (let k = 0; k < LAG.n; k++) { const v = L.s2[k] / L.n - (L.s1[k] / L.n) ** 2; if (v < bv) [best, bv] = [k, v]; }
    const offset = (best - (LAG.n >> 1)) * LAG.step;
    return { n: L.n, offset, videoDelay: Math.round((this.ctl?.videoDelay ?? 0) - offset), rmsDeg: +(Math.sqrt(Math.max(0, bv)) / DEG).toFixed(2) };
  }

  // ---------------------------------------------------------------- relocalization

  // -> { ok, advisory?, rank (of the keyframe that worked, 1 = best), pose, inliers, ms, tried, stale? (too late: try a
  // new frame), disagrees? (force false: it found a pose the estimate doesn't allow) }. force: replace the estimate (a
  // lost or unknown pose); false: only a check of it. While the localizer has a conflict it is one more view (confirm).
  async relocalize(frame, t, { force = true } = {}) {
    if (this.busy) return { ok: false, reason: "busy" };
    this.busy = true;
    const t0 = (this.lastReloc = this.lastStart = performance.now());
    let live = null;
    try {
      await this.ready();
      if (!this.db) await this.loadDb();
      if (!this.db) return { ok: false, reason: "no relocalization database yet" };
      await this.dino();
      live = await this.liveFrame(frame, { dino: true });
      const loc = this.localizer, cands = this.db.search(live.desc, { k: this.o.candidates }), conflict = loc.known && loc.pose().conflict, mode = conflict ? { confirm: true } : { reloc: force };
      let best = null, rank = 0, last = null;
      const late = () => ({ ok: false, stale: true, reason: `took ${Math.round(performance.now() - t)} ms from the frame, longer than the position history: trying a new frame`, tried: rank, candidates: cands, ms: performance.now() - t0 });
      for (const c of cands) {
        if (performance.now() - t > this.o.relocBudget) return late();
        rank++;
        let r = await this.solveAt(live.id, { ...this.bodyGuess(c, t), ...(c.pitch && { pitch: c.pitch }) }); // the keyframe's place and heading, the drone's attitude (a pitched keyframe's pitch)
        if (r.sol.ok && r.sol.inliers < this.o.relocInliers && r.sol.inliers >= this.o.relocRefine) { // the keyframe is 0.3 m or 15 deg off
          const est = bodyOf(r.sol.R, r.sol.C, this.uptilt);
          if (!this.plausible(est)) r = await this.solveAt(live.id, { ...est, pitch: clamp(est.pitch, -0.6, 0.6), roll: clamp(est.roll, -0.6, 0.6) });
        }
        if (!r.sol.ok || r.sol.inliers < this.o.relocInliers) {
          if (!best || r.sol.inliers > best.r.sol.inliers) best = { r, rank };
          continue;
        }
        const out = (last = await this.accept(r, loc.pose(), t, t0, { live, ...mode }));
        if (out.ok) {
          // an advisory find (real video before the pad check) can't be used: looked for again only every advisoryEvery
          // ms, and said once in that time
          const now = performance.now(), tell = !out.advisory || !(now - this.advisoryTold < this.o.advisoryEvery);
          if (out.advisory) Object.assign(this, { lastReloc: now + this.o.advisoryEvery - this.o.relocEvery }, tell && { advisoryTold: now });
          else this.stats.relocs++;
          if (tell) this.emit("status", { level: "info", text: `Found where the drone is (keyframe ${rank} of ${cands.length}, ${out.inliers} matches)${out.advisory ? ", shown only: the real camera isn't checked on the pad yet" : conflict ? ", which settles the disagreement" : ""}.` });
          return { ...out, rank, tried: rank, candidates: cands };
        }
        if (out.refused === "stale") return late();
        if (out.refused === "gate") return { ...out, disagrees: true, rank, tried: rank, candidates: cands }; // the estimate (or a conflict not ready) says otherwise
      }
      this.stats.relocFails++;
      return { ok: false, reason: last?.reason ?? `no keyframe matched (best ${best?.r.sol.inliers ?? 0} inliers)`, tried: rank, candidates: cands, ms: performance.now() - t0 };
    } catch (e) {
      this.trouble(e);
      throw e;
    } finally {
      if (live) this.release(live.id);
      this.busy = false;
    }
  }

  // A ready conflict (localizer.js): the frame solved against a render confirmAway from the second hypothesis (behind
  // it, else beside it, where the map says a drone fits), without refining at the answer; the localizer re-anchors if
  // it lands on that hypothesis again.
  async confirm(frame, t) {
    const c = this.localizer.pose().conflict;
    if (!c || this.busy) return { ok: false, reason: this.busy ? "busy" : "no conflict" };
    this.busy = true;
    const t0 = (this.lastStart = performance.now());
    try {
      await this.ready();
      const att = this.bodyGuess({ ...c, z: this.localizer.pose().z }, t), d = this.o.confirmAway;
      const spot = [Math.PI, Math.PI / 2, -Math.PI / 2, 0].map((a) => ({ ...att, x: c.x + d * Math.cos(c.yaw + a), y: c.y + d * Math.sin(c.yaw + a) }))
        .find((b) => !this.map?.free || this.map.free(b.x, b.y, b.z, 0.1)) ?? att;
      const live = await this.liveFrame(frame);
      try {
        const r = await this.solveAt(live.id, spot);
        return { ...(await this.accept(r, null, t, t0, { confirm: true })), from: spot };
      } finally {
        this.release(live.id);
      }
    } catch (e) {
      this.trouble(e);
      throw e;
    } finally {
      this.busy = false;
    }
  }

  // "Where am I?": relocalize on the picture the drone sees now (waiting at most `wait` ms for a running fix). The stored
  // database only (none: says so at once). A known pose is only checked (force false). It answers within `deadline` ms
  // whatever happens: a relocalization model still downloading is said so (and goes on downloading), a slow match is
  // left to finish by itself. -> relocalize()'s answer, plus advisory (the real camera isn't checked on the pad yet: the
  // answer is shown, not used).
  async locateNow({ signal = null, wait = 2000, deadline = 4000 } = {}) {
    const src = this.perception?.source, t0 = Date.now(), aborted = () => signal?.aborted && { ok: false, reason: "stopped" };
    if (!src?.ready?.()) return { ok: false, reason: "no video" };
    if (!(await this.idle(wait, signal))) return aborted() || { ok: false, reason: "the vision worker is busy" };
    if (aborted()) return aborted();
    if (!this.db) await this.loadDb().catch(() => null);
    if (!this.db) return { ok: false, reason: this.dbBuild ? `the relocalization database is still being built (${this.dbProgress?.done ?? 0} of ${this.dbProgress?.total ?? "?"} views)` : "there is no relocalization database for this house yet (it is built on the ground, in about 30 s)" };
    if (this.framing) return { ok: false, reason: this.framing };
    if (!this.features?.info.dinoLoaded) {
      const loading = this.dino().catch(() => null), left = deadline - (Date.now() - t0);
      if (!(await Promise.race([loading.then(() => true), sleep(Math.max(0, left - 1500)).then(() => false)])))
        return { ok: false, reason: `the relocalization model is still downloading${this.dinoPct != null ? ` (${this.dinoPct}%)` : ""}: ask again in a moment`, downloading: true };
    }
    if (aborted()) return aborted();
    const image = grab({ element: src.element(), region: this.perception.region?.() ?? src.region() });
    if (!image) return { ok: false, reason: "no video" };
    const p = this.localizer.pose(), force = !this.localizer.known || p.status === "lost";
    const t = src.captured?.() ?? (src.decodedAt?.() ?? performance.now()) - (this.ctl?.videoDelay ?? 0), left = Math.max(500, deadline - (Date.now() - t0));
    const r = await Promise.race([this.relocalize(image, t, { force }), sleep(left).then(() => ({ ok: false, reason: `matching the view took longer than ${Math.round(deadline / 1000)} s`, late: true }))]);
    return { ...r, checked: !force, advisory: this.trust() === "advisory" };
  }

  // ---------------------------------------------------------------- the database

  // The relocalization database: the stored one for this house and camera, else built now (once; later callers share
  // the build; paused while the drone flies). onProgress({ done, total, ms, paused? }). -> RelocDb.
  async ensureDb({ onProgress, signal, ...opts } = {}) {
    await this.ready();
    if (this.db?.meta.key === this.dbKey()) return this.db;
    if ((await this.loadDb()) && this.db.meta.key === this.dbKey()) return this.db;
    this.dbBuild ??= this.buildDb({ onProgress: (p) => ((this.dbProgress = p), this.emit("db", p), onProgress?.(p)), signal, ...opts }).finally(() => (this.dbBuild = null));
    return this.dbBuild;
  }

  dbKey() {
    const h = this.house;
    return h && `${h.id}|${h.splatFile}|${h.frame?.f}|${h.frame?.Yf}|${RECT_HFOV}|${Math.round(this.uptilt)}|${this.features?.info.dino ?? "dino"}`;
  }

  async loadDb() {
    if (!this.store || !this.house) return null;
    const buf = await this.store.readFile(this.house.id, "reloc.bin").catch(() => null);
    const db = buf && RelocDb.load(buf);
    if (db && db.meta.key === this.dbKey()) this.db = db;
    return this.db?.meta.key === this.dbKey() ? this.db : null;
  }

  // The keyframe database: renders of the twin at known-free poses (map.free) on a grid x heights above the floor x
  // headings (x pitches, rad nose down, when given), their DINOv2 descriptors; it waits while the drone flies (the twin and
  // the GPU are the flight's then). onProgress({ done, total, ms, paused? }); signal aborts. Saved to OPFS when there is a
  // store. -> RelocDb.
  async buildDb({ grid = 0.5, heights = [0.7, 1.3], headings = 12, pitches = [0], onProgress, signal, save = true } = {}) {
    await this.dino();
    const poses = relocPoses(this.map, { grid, heights, headings, pitches }), t0 = performance.now(), dim = DINO.dim, pitched = pitches.some((p) => p);
    const q = new Int8Array(poses.length * dim), scale = new Float32Array(poses.length), view = { ...DB_VIEW, lens: rectLens(this.uptilt), actors: false, props: false, prio: "build" };
    for (let i = 0; i < poses.length; i++) {
      if (signal?.aborted) throw new DOMException("Aborted", "AbortError");
      while (this.ctl?.isFlying?.() && !signal?.aborted) (onProgress?.({ done: i, total: poses.length, ms: performance.now() - t0, paused: true }), await sleep(1000));
      const p = poses[i], rgba = await this.twin.pixels({ x: p.x, y: p.y, z: p.z + CAM_DZ, yaw: p.yaw, pitch: p.pitch, roll: 0 }, view);
      const { desc } = await this.features.global({ rgba });
      scale[i] = quantize(desc, q, i * dim);
      if (i % 24 === 0 || i === poses.length - 1) onProgress?.({ done: i + 1, total: poses.length, ms: performance.now() - t0 });
    }
    const db = new RelocDb({ key: this.dbKey(), grid, heights, headings, ...(pitched && { pitches, stride: 5 }), dim, n: poses.length, built: Date.now(), ms: Math.round(performance.now() - t0) },
      Float32Array.from(poses.flatMap((p) => (pitched ? [p.x, p.y, p.z, p.yaw, p.pitch] : [p.x, p.y, p.z, p.yaw]))), q, scale);
    this.db = db;
    if (save && this.store && this.house) await this.store.writeFile(this.house.id, "reloc.bin", db.serialize());
    return db;
  }

  // ---------------------------------------------------------------- the pad check and lens calibration

  // Before take-off, the drone on its home pad and its pose reset there: n frames solved where the pad is (nothing applied
  // while solving). They must agree with each other (pad.spread, pad.spreadYaw: a steady picture that matches; on a still
  // drone that is repeatability, not the lens), show the drone level (pad.tilt: a wrong camera angle shows as a tilt) and
  // land near the pad. A still drone can't tell a lens error from a drone set down off its mark, so: with a calibration
  // verified for this framing, within pad.near, pad.nearYaw (a drone set down a few cm or degrees off) and the pose is
  // re-seated on their median, never tighter than the pad button's own pad.sigma, pad.yawSigma (nor than half the offset);
  // without one, within pad.rawNear, pad.rawNearYaw and the hand placement stands. Then real video's fixes are used with this
  // lens and framing. -> { ok, ready (ok, a verified calibration for this framing, and a relocalization database),
  // accepted, n, medianErr, maxErr, yawErr (deg; the median solve's offset from the pad), spread (m), spreadYaw (deg),
  // tilt (deg), offset: { x, y, yaw (deg), dist } | null, reseated, calibrated, db, key (lensKey()), text }.
  async verifyOnPad({ n = this.o.pad.n, signal = null } = {}) {
    const src = this.perception?.source, p = this.localizer.pose();
    const no = (why) => this.judgePad([], null, why);
    if (!src?.ready?.()) return no("there is no video");
    if (!this.twin) return no("the 3D scan isn't loaded yet");
    if (this.ctl?.isFlying?.()) return no("the drone is flying");
    if (p.status === "lost") return no("the drone's position isn't set: put it on the home pad and press the pad button");
    if (this.framing) return no(this.framing);
    await this.ready();
    if (this.cpuOnly()) return no(this.cpuOnly());
    const pad = { x: p.x, y: p.y, z: p.z, yaw: p.yaw, pitch: 0, roll: 0 }, rows = [], was = this.calibrating;
    this.calibrating = true; // tracking waits meanwhile, so the worker goes idle between the pad's frames
    try {
      for (let k = 0; k < n; k++) {
        if (signal?.aborted) throw new DOMException("Aborted", "AbortError");
        if (!(await this.idle(2000, signal))) return no("the vision worker is busy");
        const image = grab({ element: src.element(), region: this.perception.region?.() ?? src.region() });
        if (image) rows.push(await this.padSolve(image, pad));
        await this.nextFrame(150);
      }
    } finally {
      this.calibrating = was;
    }
    return this.judgePad(rows, pad);
  }

  // One frame solved at a pad pose, nothing applied. -> { ok, reason, inliers, err (m), yawErr (deg), x, y, z, yaw }.
  async padSolve(image, pad) {
    if (this.busy && !(await this.idle(2000))) return { ok: false, reason: "the vision worker is busy" };
    this.busy = true;
    try {
      await this.ready();
      const live = await this.liveFrame(image);
      try {
        const r = await this.solveAt(live.id, pad), why = this.quality(r.sol), est = r.sol.ok ? bodyOf(r.sol.R, r.sol.C, this.uptilt) : null;
        return { ok: !why, reason: why, inliers: r.sol.inliers, err: est && Math.hypot(est.x - pad.x, est.y - pad.y), yawErr: est && Math.abs(wrapAngle(est.yaw - pad.yaw)) / DEG, x: est?.x, y: est?.y, z: est?.z, yaw: est?.yaw, pitch: est?.pitch, roll: est?.roll };
      } finally {
        this.release(live.id);
      }
    } catch (e) {
      this.trouble(e);
      return { ok: false, reason: e.message.split("\n")[0] };
    } finally {
      this.busy = false;
    }
  }

  judgePad(rows, pad, why = "") {
    const P = this.o.pad, acc = rows.filter((r) => r.ok), r2 = (v) => (v == null ? null : +v.toFixed(3)), deg = (v) => `${Math.abs(v).toFixed(1)}°`;
    const cm = (v) => (v < 0.01 ? `${Math.round(v * 1000)} mm` : `${Math.round(v * 100)} cm`);
    // the median solve, and how far the others lie from it (the worst one set aside when there are five or more)
    const mid = acc.length ? { x: median(acc.map((r) => r.x)), y: median(acc.map((r) => r.y)), yaw: acc[0].yaw + median(acc.map((r) => wrapAngle(r.yaw - acc[0].yaw))) } : null;
    const dev = (f) => { const d = acc.map(f).sort((a, b) => a - b); return d.length ? d[Math.max(0, d.length - (d.length >= 5 ? 2 : 1))] : null; };
    const spread = mid && dev((r) => Math.hypot(r.x - mid.x, r.y - mid.y)), spreadYaw = mid && dev((r) => Math.abs(wrapAngle(r.yaw - mid.yaw))) / DEG;
    const tilt = mid && Math.max(Math.abs(median(acc.map((r) => r.pitch ?? 0))), Math.abs(median(acc.map((r) => r.roll ?? 0)))) / DEG;
    const offset = mid && pad ? { x: r2(mid.x - pad.x), y: r2(mid.y - pad.y), yaw: +(wrapAngle(mid.yaw - pad.yaw) / DEG).toFixed(1), dist: r2(Math.hypot(mid.x - pad.x, mid.y - pad.y)) } : null;
    const calibrated = this.calibOk, [near, nearYaw] = calibrated ? [P.near, P.nearYaw] : [P.rawNear, P.rawNearYaw];
    const agree = acc.length >= P.accept && spread <= P.spread && spreadYaw <= P.spreadYaw / DEG, level = tilt != null && tilt <= P.tilt / DEG;
    const close = !!offset && offset.dist <= near && Math.abs(offset.yaw) <= nearYaw / DEG, ok = !why && agree && level && close, max = acc.length ? Math.max(...acc.map((r) => r.err)) : null;
    const res = { ok, accepted: acc.length, n: rows.length, medianErr: offset?.dist ?? null, maxErr: r2(max), yawErr: offset ? Math.abs(offset.yaw) : null, spread: r2(spread), spreadYaw: spreadYaw == null ? null : +spreadYaw.toFixed(2),
      tilt: tilt == null ? null : +tilt.toFixed(1), offset, reseated: false, calibrated, db: !!this.db, key: this.lensKey(), at: Date.now() };
    res.ready = ok && res.calibrated && res.db;
    if (ok && calibrated) { // a calibrated camera knows better than the hand that set the drone down, within its own error
      this.localizer.reset({ x: mid.x, y: mid.y, z: pad.z, yaw: mid.yaw, sigma: Math.max(P.sigma, spread, offset.dist / 2), yawSigma: Math.max(P.yawSigma / DEG, spreadYaw, Math.abs(offset.yaw) / 2) * DEG });
      res.reseated = true;
    }
    const off = offset && `${cm(offset.dist)} and ${deg(offset.yaw)} ${offset.yaw >= 0 ? "left" : "right"}`, moved = res.reseated && (offset.dist > 0.02 || Math.abs(offset.yaw) > 1);
    res.text = why ? `The pad check can't run: ${why}.`
      : ok ? `The camera view matches the 3D scan on the pad${moved ? `: the drone sits ${off} of its pad mark, so its position now comes from the camera` : ` (within ${cm(offset.dist)} and ${deg(offset.yaw)} of its mark)`}. Vision fixes are on${calibrated ? "" : ", with wider error bars until the camera is calibrated (a still drone can't show a lens error)"}.`
      : acc.length < P.accept ? `The camera view doesn't match the 3D scan from the pad (${acc.length} of ${rows.length} views matched): check the lighting, that the drone sits on its pad and that the video shows the camera's picture, and calibrate the camera.`
      : !agree ? `The camera's views from the pad disagree with each other (${cm(spread)} and ${deg(spreadYaw)} apart): calibrate the camera (Settings → House), then press the pad button again.`
      : !level ? `The camera sees the drone tilted ${deg(tilt)} on its pad. Check that it sits flat; if it does, the camera's angle is off: calibrate the camera (Settings → House), then press the pad button again.`
      : calibrated ? `The camera puts the drone ${off} of its pad mark, more than ${cm(near)} or ${deg(nearYaw / DEG)}: put it on the pad facing the home heading and press the pad button again.`
      : `The camera puts the drone ${off} of its pad mark. Check that it sits on its mark facing the home heading; if it does, the camera needs calibrating (Settings → House). Then press the pad button again.`;
    if (pad) this.pad = res;
    this.emit("status", { level: ok ? "info" : "warn", text: res.text });
    return res;
  }

  // Lens calibration sample: the frame's inliers against the twin as fisheye pixels and H points (lens.calibrateLens).
  async calibSample(frame, body, { grounded = false } = {}) {
    await this.ready();
    const live = await this.liveFrame(frame), r = await this.solveAt(live.id, body);
    const s = r.sol.ok ? await this.features.calibSample({ live: live.id }) : null;
    this.release(live.id);
    return s && { ...s, R: r.sol.R, C: r.sol.C, grounded, inliers: r.sol.inliers };
  }

  // Frames [{ image (ImageBitmap, kept), body, grounded }] -> calib.json: matched and fitted `rounds` times, each round
  // matching again through the lens the last one found (a wrong lens loses the picture's edges to the matcher; each
  // round gets more of them back), each frame rendered at the heading its last solve found (on the ground the gyro
  // heading lags a hand turn by tens of degrees). The lens is put back as it was. -> { calib, rounds: [{ fx, rms,
  // frames, points }], samples }.
  async calibrateFrames(frames, { rounds = 3, pad = null, minPoints = 80 } = {}) {
    const lens0 = this.lens, hist = [], body = frames.map((f) => ({ ...f.body }));
    let calib = null, samples = [];
    try {
      for (let k = 0; k < rounds; k++) {
        samples = [];
        for (const [i, f] of frames.entries()) {
          const s = await this.calibSample(await createImageBitmap(f.image), body[i], { grounded: f.grounded }).catch(() => null);
          if (!s || s.uv.length < 2 * minPoints) continue;
          samples.push(s);
          if (f.grounded) body[i].yaw = bodyOf(s.R, s.C, this.uptilt).yaw;
        }
        if (samples.length < 4) break;
        const prev = calib;
        ({ calib } = calibrateLens(samples, { lens: this.lens, width: samples[0].width, height: samples[0].height, pad }));
        hist.push({ fx: calib.fx, cx: calib.cx, cy: calib.cy, k1: calib.k[0], uptiltDeg: calib.uptiltDeg, rms: calib.rms, frames: samples.length, points: calib.points });
        this.setLens(droneLens(calib));
        if (prev && Math.abs(prev.fx - calib.fx) < 0.3 && Math.abs(prev.cx - calib.cx) < 0.3 && Math.abs(prev.cy - calib.cy) < 0.3) break;
      }
    } finally {
      this.setLens(lens0);
    }
    if (!calib) throw new Error(`only ${samples.length} views matched the 3D scan: turn the drone slowly, all the way round`);
    return { calib, rounds: hist, samples: samples.length };
  }

  // "Calibrate the camera" on the home pad: the localizer reset there, the drone on the ground turned slowly through a
  // full circle (by hand; the heading follows the gyro, and lags), then held still. A frame every `step` of turn, then
  // calibrateFrames() (one pad body, rounds of matching), then a check with the new lens on fresh frames: solved where
  // the drone sits, they must land on the pad (pad.pos) with one heading (pad.yaw apart at most). Tracking waits
  // meanwhile (the lens changes between rounds). onProgress({ samples, of, yaw, phase: "turn" | "fit" | "check" }).
  // -> { calib (calib.json: with the framing it was made for (region), the check as `verified`, the overlay mask once
  // learned; in use from now), samples, rounds, verify } or throws (the lens stays as it was). The caller stores
  // calib.json. The heading isn't checked against the home heading here (the drone was turned): the pad button's check
  // (verifyOnPad) does that, and real video stays advisory until it passes with the new lens.
  async calibrate({ of = 10, step = 30 * DEG, timeout = 90000, rounds = 3, onProgress, signal } = {}) {
    const loc = this.localizer, src = this.perception?.source, t0 = performance.now(), frames = [];
    if (!src?.ready?.()) throw new Error("no video");
    if (loc.pose().status === "lost" || this.ctl?.isFlying()) throw new Error("set the drone on the home pad and press the pad button first");
    if (this.framing) throw new Error(this.framing);
    const shot = () => grab({ element: src.element(), region: this.perception.region?.() ?? src.region() });
    let last = null;
    this.calibrating = true; // tracking waits, so the worker goes idle (onFrame starts nothing new meanwhile)
    try {
      if (!(await this.idle(2000, signal))) throw new Error("the vision worker is busy");
      while (frames.length < of) {
        if (signal?.aborted) throw new DOMException("Aborted", "AbortError");
        if (performance.now() - t0 > timeout) break;
        await this.nextFrame(150);
        const p = loc.poseAt(src.captured?.() ?? (src.decodedAt?.() ?? performance.now()) - (this.ctl?.videoDelay ?? 0)); // the pose when the frame was taken
        if (last != null && Math.abs(wrapAngle(p.yaw - last)) < step) continue;
        const image = await shot();
        if (!image) continue;
        frames.push({ image, body: { x: p.x, y: p.y, z: (this.map?.floorAt(p.x, p.y) ?? p.z) + 0.01, yaw: p.yaw, pitch: 0, roll: 0 }, grounded: true });
        last = p.yaw;
        onProgress?.({ samples: frames.length, of, yaw: p.yaw, phase: "turn" });
      }
      if (frames.length < 4) throw new Error(`only ${frames.length} views: turn the drone slowly, all the way round`);
      onProgress?.({ samples: frames.length, of, phase: "fit" });
      const b = frames[0].body, { calib, rounds: hist, samples } = await this.calibrateFrames(frames, { rounds, pad: { x: b.x, y: b.y, z: b.z + CAM_DZ } });
      const lens0 = this.lens;
      this.setLens(droneLens(calib));
      onProgress?.({ samples: frames.length, of, phase: "check" });
      const p = loc.pose(), first = await this.padSolve(await shot(), { x: p.x, y: p.y, z: p.z, yaw: p.yaw, pitch: 0, roll: 0 });
      const pad = { x: p.x, y: p.y, z: p.z, yaw: first.yaw ?? p.yaw, pitch: 0, roll: 0 }, rows = []; // rendered at the heading the camera shows
      for (let k = 0; k < this.o.pad.n; k++) {
        await this.nextFrame(150);
        const image = await shot();
        if (image) rows.push(await this.padSolve(image, pad));
      }
      const acc = rows.filter((r) => r.ok), med = median(acc.map((r) => r.err)), yaws = acc.map((r) => wrapAngle(r.yaw - acc[0].yaw) / DEG);
      const spread = acc.length ? Math.max(...yaws) - Math.min(...yaws) : Infinity, P = this.o.pad;
      const verify = { ok: acc.length >= P.accept && med <= P.pos && spread <= P.yaw / DEG, accepted: acc.length, n: rows.length, medianErr: med == null ? null : +med.toFixed(3), yawSpread: +spread.toFixed(2),
        headingVsGyro: first.yaw == null ? null : +(wrapAngle(first.yaw - p.yaw) / DEG).toFixed(1) };
      if (!verify.ok) {
        this.setLens(lens0);
        throw new Error(`the new calibration didn't check out on the pad (${acc.length} of ${rows.length} views matched, ${med == null ? "-" : (med * 100).toFixed(0)} cm from the pad, headings ${spread.toFixed(1)}° apart)`);
      }
      const out = { ...calib, region: this.region ? { ...this.region, kind: src.kind } : null, verified: { err: verify.medianErr, yawSpread: verify.yawSpread, accepted: verify.accepted, at: new Date().toISOString() } };
      const full = this.osd.usable ? { ...out, osdMask: this.osd.toJSON() } : out;
      this.setCalib(full);
      this.pad = null; // the pad button's check, with the drone facing the home heading, comes next
      this.emit("status", { level: "info", text: `The camera is calibrated (${calib.rms} px). Put the drone back on its pad facing the home heading and press the pad button to check it.` });
      return { calib: full, samples, rounds: hist, verify };
    } finally {
      this.calibrating = false;
      for (const f of frames) f.image.close?.();
    }
  }

  dispose() {
    this.detach();
    this.features?.dispose();
    this.features = null;
  }
}

// Poses for the database: grid cell centres in a room where map.free says a drone fits at each height, every heading
// (and pitch).
export function relocPoses(map, { grid = 0.5, heights = [0.7, 1.3], headings = 12, pitches = [0] } = {}) {
  const out = [], { cell } = map.o, x1 = map.x0 + map.W * cell, y1 = map.y0 + map.H * cell;
  for (let y = map.y0 + grid / 2; y < y1; y += grid)
    for (let x = map.x0 + grid / 2; x < x1; x += grid) {
      if (!map.roomAt(x, y)) continue;
      const floor = map.floorAt(x, y);
      for (const h of heights) {
        const z = floor + h;
        if (!map.free(x, y, z)) continue;
        for (const pitch of pitches) for (let k = 0; k < headings; k++) out.push({ x: +x.toFixed(3), y: +y.toFixed(3), z: +z.toFixed(3), yaw: wrapAngle((k * 2 * Math.PI) / headings), pitch });
      }
    }
  return out;
}

function quantize(desc, q, off) {
  let m = 1e-9;
  for (const v of desc) m = Math.max(m, Math.abs(v));
  for (let k = 0; k < desc.length; k++) q[off + k] = Math.round((desc[k] * 127) / m);
  return m / 127;
}

// Keyframes: poses (x, y, z, yaw) and int8 descriptors with a scale each. reloc.bin: "WPRL", u32 header length, JSON
// header, then poses (f32 x 4n), scales (f32 x n), descriptors (i8 x n·dim), each 4-byte aligned.
export class RelocDb {
  constructor(meta, poses, q, scale) {
    Object.assign(this, { meta, poses, q, scale, n: meta.n, dim: meta.dim, stride: meta.stride ?? 4 });
  }

  // The k best keyframes by descriptor similarity, at most one per position and heading sector (no near duplicates).
  search(desc, { k = 8 } = {}) {
    const { n, dim, q, scale, poses } = this, s = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      let a = 0;
      for (let j = 0, o = i * dim; j < dim; j++) a += q[o + j] * desc[j];
      s[i] = a * scale[i];
    }
    const order = Array.from({ length: n }, (_, i) => i).sort((a, b) => s[b] - s[a]), out = [], st = this.stride;
    for (const i of order) {
      const c = { i, score: s[i], x: poses[st * i], y: poses[st * i + 1], z: poses[st * i + 2], yaw: poses[st * i + 3], ...(st > 4 && { pitch: poses[st * i + 4] }) };
      if (out.some((o) => Math.hypot(o.x - c.x, o.y - c.y) < 0.3 && Math.abs(o.z - c.z) < 0.2 && Math.abs(wrapAngle(o.yaw - c.yaw)) < 0.4)) continue;
      out.push(c);
      if (out.length >= k) break;
    }
    return out;
  }

  serialize() {
    const head = new TextEncoder().encode(JSON.stringify(this.meta)), hl = (head.length + 3) & ~3, n = this.n;
    const P = 4 * this.stride * n, buf = new ArrayBuffer(8 + hl + P + 4 * n + ((n * this.dim + 3) & ~3)), u8 = new Uint8Array(buf), dv = new DataView(buf);
    u8.set(new TextEncoder().encode("WPRL"), 0);
    dv.setUint32(4, hl, true);
    u8.set(head, 8);
    head.length < hl && u8.fill(32, 8 + head.length, 8 + hl);
    let o = 8 + hl;
    new Float32Array(buf, o, this.stride * n).set(this.poses);
    o += P;
    new Float32Array(buf, o, n).set(this.scale);
    o += 4 * n;
    new Int8Array(buf, o, n * this.dim).set(this.q);
    return buf;
  }

  static load(buf) {
    const u8 = new Uint8Array(buf), dv = new DataView(buf);
    if (new TextDecoder().decode(u8.subarray(0, 4)) !== "WPRL") return null;
    const hl = dv.getUint32(4, true), meta = JSON.parse(new TextDecoder().decode(u8.subarray(8, 8 + hl))), n = meta.n, P = 4 * (meta.stride ?? 4) * n;
    let o = 8 + hl;
    const poses = new Float32Array(buf.slice(o, o + P));
    o += P;
    const scale = new Float32Array(buf.slice(o, o + 4 * n));
    o += 4 * n;
    return new RelocDb(meta, poses, new Int8Array(buf.slice(o, o + n * meta.dim)), scale);
  }

  get bytes() {
    return 8 + (4 * this.stride + 4) * this.n + this.n * this.dim;
  }
}

// calib.json's osdMask -> an OsdMask, or null when it hides more than OSD_MAX of the picture (a learning failure).
function usableMask(json) {
  const m = OsdMask.from(json);
  return m.usable ? m : null;
}

// A perception "frame" event -> an ImageBitmap of its picture now (the source canvas is reused for the next frame),
// scaled to SRC_W wide. Returns a promise, or null when there is nothing to grab.
export function grab(f) {
  const el = f.element, r = f.region ?? { sx: 0, sy: 0, sw: el.width || el.videoWidth, sh: el.height || el.videoHeight };
  if (!r.sw || !r.sh) return null;
  const w = Math.min(SRC_W, r.sw), h = Math.round((w * r.sh) / r.sw);
  return createImageBitmap(el, r.sx, r.sy, r.sw, r.sh, w === r.sw ? {} : { resizeWidth: w, resizeHeight: h, resizeQuality: "medium" });
}

async function toBitmap(frame) {
  frame = await frame;
  if (frame instanceof ImageBitmap) return frame;
  if (frame?.data && frame.width) return createImageBitmap(new ImageData(new Uint8ClampedArray(frame.data), frame.width, frame.height));
  const w = frame.videoWidth ?? frame.width, h = frame.videoHeight ?? frame.height;
  return w > SRC_W ? createImageBitmap(frame, { resizeWidth: SRC_W, resizeHeight: Math.round((SRC_W * h) / w), resizeQuality: "medium" }) : createImageBitmap(frame);
}
