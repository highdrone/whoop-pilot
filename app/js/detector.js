// On-device object detection and tracking (80 everyday COCO classes: cat, dog, person, refrigerator, couch...).
// Default: RF-DETR on WebGPU in a worker (vision/), about twice as accurate as MediaPipe EfficientDet-Lite0 in good light
// and 2.7x in the dark, and faster. Without WebGPU, or if the model can't load, it switches to MediaPipe EfficientDet-Lite
// on its own and says so (status event). Either way detections go through ByteTrack, so each one carries a stable trackId.
import { Emitter } from "./util.js";
import { Settings } from "./settings.js";
import { COCO_LABELS, RfDetr, unavailable } from "./vision/rfdetr.js";
import { Tracker, MotionLog } from "./vision/tracker.js";

export { COCO_LABELS };

const MP_VERSION = "1.0.1";
const MP_URL = `https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@${MP_VERSION}/vision_bundle.mjs`;
const WASM_URL = `https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@${MP_VERSION}/wasm`;
const MP_MODELS = {
  fast: "https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/float16/1/efficientdet_lite0.tflite",
  accurate: "https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite2/float16/1/efficientdet_lite2.tflite",
};

// Score tiers per backend (their scores are calibrated differently). RF-DETR: ByteTrack's 0.5/0.1, measured on it; people
// and pets count from 0.4 (find()'s threshold), since a dim one scores 0.41-0.47. On 13 people-free frames of the house
// (capture photos and twin renders) no person, cat or dog scored above 0.11.
const TIERS = {
  rfdetr: { high: 0.5, low: 0.1, birth: 0.5, highFor: { person: 0.4, cat: 0.4, dog: 0.4 } },
  mediapipe: { high: 0.35, low: 0.15, birth: 0.35 },
};
const REPORT_MIN = 0.25; // tracks fading below this (only weak detections lately) aren't reported
const STALE_MS = 1000; // RF-DETR results older than this are dropped

const SYNONYMS = {
  kitty: "cat", kitten: "cat", cats: "cat", puppy: "dog", doggy: "dog", doggo: "dog", dogs: "dog",
  me: "person", human: "person", people: "person", man: "person", woman: "person", kid: "person", child: "person",
  fridge: "refrigerator", sofa: "couch", television: "tv", "tv screen": "tv", monitor: "tv", plant: "potted plant",
  table: "dining table", "kitchen table": "dining table", stove: "oven", ball: "sports ball", phone: "cell phone",
  mug: "cup", teddy: "teddy bear", bag: "backpack", toaster: "toaster", "trash can": null,
};

// Map what the user said ("the kitty", "my fridge") to a detector label, or null if it can't detect it.
export function toDetectorLabel(text) {
  if (!text) return null;
  const t = String(text).toLowerCase().replace(/^(the|a|an|my|our|that|this)\s+/g, "").replace(/[^a-z ]/g, "").trim();
  if (COCO_LABELS.includes(t)) return t;
  if (t in SYNONYMS) return SYNONYMS[t];
  for (const word of t.split(" ").reverse()) {
    if (COCO_LABELS.includes(word)) return word;
    if (SYNONYMS[word]) return SYNONYMS[word];
  }
  return null;
}

// Requested backend and quality: ?detector=mediapipe|rfdetr and ?quality=fast|accurate, else the settings "detector"
// (default "rfdetr") and "detectorQuality". ?nowebgpu makes RF-DETR unavailable, so MediaPipe runs.
function preferred() {
  const q = typeof location !== "undefined" ? new URLSearchParams(location.search) : new URLSearchParams();
  const saved = typeof document !== "undefined" ? new Settings() : null;
  return {
    backend: q.get("detector") || saved?.get("detector") || "rfdetr",
    quality: q.get("quality") || saved?.get("detectorQuality") || "fast",
  };
}

export class ObjectDetector extends Emitter {
  constructor(opts = {}) {
    super();
    const p = preferred();
    this.want = opts.backend || p.backend;
    this.quality = opts.quality || p.quality;
    this.backend = null; // the one running: "rfdetr" | "mediapipe"
    this.model = "";
    this.detector = null; // MediaPipe
    this.rf = null;
    this.loading = null;
    this.loadedOnce = false;
    this.error = null;
    this.status = "";
    this.lastTs = 0;
    this.motion = new MotionLog();
    this.tracker = new Tracker(TIERS.rfdetr);
    this.raw = { t: 0, dets: [] }; // the backend's latest untracked detections
    this.resultT = 0;
    this.failures = 0;
    this.gen = 0; // bumped on unload, so a load that finishes after a reconfigure is discarded
    this.stats = { inferMs: 0, latencyMs: 0, hz: 0 };
  }

  // Change backend ("rfdetr" | "mediapipe") or quality ("fast" | "accurate"); reloads if something was loaded.
  configure({ backend = this.want, quality = this.quality } = {}) {
    if (backend === this.want && quality === this.quality) return;
    this.want = backend;
    this.quality = quality;
    if (this.loading || this.ready) {
      this.#unload();
      this.load().catch(() => {});
    }
  }

  async load(quality = this.quality) {
    if (quality !== this.quality) this.configure({ quality });
    if (!this.loading) {
      const p = (this.loading = this.#load().catch((e) => {
        if (this.loading === p) Object.assign(this, { error: e, loading: null });
        throw e;
      }));
    }
    return this.loading;
  }

  async #load() {
    const gen = this.gen;
    let why = null;
    if (this.want !== "mediapipe") {
      why = unavailable();
      if (!why) {
        const rf = new RfDetr(this.quality);
        try {
          this.#say("Loading the RF-DETR detector...");
          const info = await rf.load((loaded, total) => this.#say(`Downloading the detector model (once): ${Math.round((loaded / total) * 100)}% of ${Math.round(total / 1e6)} MB`, "info", loaded / total));
          if (gen !== this.gen) return rf.dispose();
          rf.onResult = (m) => this.#onResult(m);
          Object.assign(this, { rf, backend: "rfdetr", model: info.name, failures: 0, loadedOnce: true });
          this.#retier("rfdetr");
          this.#say(`Detector: ${info.name} on WebGPU${info.cached ? "" : " (model downloaded and saved for offline use)"}.`);
          return;
        } catch (e) {
          rf.dispose();
          why = e.message;
        }
      }
      this.#say(`RF-DETR detector unavailable (${why}); loading MediaPipe instead.`, "warn");
    }
    await this.#loadMediaPipe(gen);
    if (gen !== this.gen) return;
    this.#retier("mediapipe");
    Object.assign(this, { backend: "mediapipe", loadedOnce: true });
    if (why) this.#say(`Detector: ${this.model}, the fallback (RF-DETR unavailable: ${why}). It misses more people and pets, more so in the dark.`, "warn");
    else this.#say(`Detector: ${this.model}.`);
  }

  async #loadMediaPipe(gen) {
    const { FilesetResolver, ObjectDetector: MPObjectDetector } = await import(MP_URL);
    const files = await FilesetResolver.forVisionTasks(WASM_URL);
    const opts = (delegate) => ({
      baseOptions: { modelAssetPath: MP_MODELS[this.quality] || MP_MODELS.fast, delegate },
      runningMode: "VIDEO",
      scoreThreshold: TIERS.mediapipe.low,
      maxResults: 25,
    });
    let det;
    try {
      det = await MPObjectDetector.createFromOptions(files, opts("GPU"));
    } catch {
      det = await MPObjectDetector.createFromOptions(files, opts("CPU"));
    }
    if (gen !== this.gen) return det.close();
    this.detector = det;
    this.model = `MediaPipe EfficientDet-Lite${this.quality === "accurate" ? 2 : 0}`;
  }

  #unload() {
    this.rf?.dispose();
    this.detector?.close?.();
    Object.assign(this, { rf: null, detector: null, backend: null, loading: null, error: null, resultT: 0, gen: this.gen + 1 });
    this.tracker.reset();
  }

  #retier(backend) {
    this.tracker = Object.assign(new Tracker({ ...TIERS[backend], aspect: this.tracker.aspect }), { nextId: this.tracker.nextId });
  }

  #say(text, level = "info", progress) {
    this.status = text;
    if (progress === undefined) (level === "warn" ? console.warn : console.info)(text);
    this.emit("status", { text, level, backend: this.backend, progress });
  }

  // Stays true from the first load on: while a reconfigure or fallback reloads, detect() returns [], so callers see
  // nothing rather than keep the last boxes from before the switch.
  get ready() {
    return this.loadedOnce;
  }

  // source: canvas/video/ImageBitmap showing the drone view (width x height px). Returns the tracked objects in it:
  // [{ label, score, box: {x,y,w,h} normalized, trackId }]; a trackId is never reused, even across backend switches.
  // Optional opts, all improving tracking:
  //   t: capture time (performance.now()); flow: this frame's flow.js result, focusY, and yawRate (rad/s, + = right) with
  //   hfov (degrees across the picture, default 127, the O4 lens) for camera-motion compensation; element + region
  //   {sx,sy,sw,sh}: the full-resolution video and the drone-view crop, used by RF-DETR instead of the (smaller) source.
  // RF-DETR runs in a worker, one frame in flight (~15 Hz): this returns the latest results, moved on to time t.
  detect(source, width, height, opts = {}) {
    const t = opts.t ?? performance.now();
    if (width && height) this.tracker.aspect = width / height;
    if ("flow" in opts || "yawRate" in opts) this.motion.push(t, opts.flow, opts);
    if (this.backend === "mediapipe") return this.#report(this.tracker.update(this.#mediapipe(source, width, height, t), t, this.#motionSince(t)));
    if (this.backend !== "rfdetr") return [];
    this.rf.submit(opts.element || source, t, opts.element ? opts.region : null);
    if (!this.resultT || t - this.resultT > STALE_MS) return [];
    return this.#report(this.tracker.predict(t, this.#motionSince(t)));
  }

  #motionSince(t) {
    return this.motion.between(this.tracker.t, t);
  }

  #mediapipe(source, width, height, t) {
    const ts = Math.max(t, this.lastTs + 1);
    this.lastTs = ts;
    const res = this.detector.detectForVideo(source, ts);
    const dets = (res.detections || []).map((d) => {
      const c = d.categories[0];
      const b = d.boundingBox;
      return { label: c.categoryName, score: c.score, box: { x: b.originX / width, y: b.originY / height, w: b.width / width, h: b.height / height } };
    });
    this.raw = { t, dets };
    return dets;
  }

  #onResult(m) {
    if (m.type === "fail") {
      console.warn("RF-DETR frame failed:", m.error);
      if (m.fatal || ++this.failures >= 3) this.#fallBack(m.error);
      return;
    }
    this.failures = 0;
    this.raw = { t: m.t, dets: m.dets };
    this.tracker.update(m.dets, m.t, this.#motionSince(m.t));
    const now = performance.now();
    const s = this.stats;
    const k = s.hz ? 0.2 : 1;
    s.inferMs += (m.ms.total - s.inferMs) * k;
    s.latencyMs += (now - m.t - s.latencyMs) * k;
    if (this.resultT) s.hz += (1000 / Math.max(1, m.t - this.resultT) - s.hz) * k;
    this.resultT = m.t;
  }

  #fallBack(why) {
    this.#unload();
    this.want = "mediapipe";
    this.#say(`RF-DETR detector stopped working (${why}); switching to MediaPipe.`, "warn");
    this.load().then(
      () => this.#say(`Detector: ${this.model}, the fallback (RF-DETR stopped working: ${why}).`, "warn"),
      (e) => this.#say(`MediaPipe detector failed to load too: ${e.message}`, "warn"),
    );
  }

  #report(tracks) {
    const out = [];
    for (const tr of tracks) if (tr.score >= REPORT_MIN) out.push({ label: tr.label, score: tr.score, box: tr.box, trackId: tr.trackId });
    return out;
  }
}
