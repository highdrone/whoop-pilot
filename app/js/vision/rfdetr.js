// RF-DETR (Roboflow, Apache-2.0) on onnxruntime-web WebGPU in a module worker: the pre/post-processing the worker runs
// (pure, also tested under Node) and the page-side client that feeds it one frame at a time, at most GPU.cap("detect")
// a second (vision/budget.js: 8 while vision localization or live depth runs; else as fast as it answers).
import { GPU } from "./budget.js";

export const COCO_LABELS = [
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
  "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
  "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
  "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
  "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
  "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
  "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
  "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
  "scissors", "teddy bear", "hair drier", "toothbrush",
];

// COCO category id of each label above. RF-DETR's 91 logit slots are indexed by category id; the 11 unused ids are skipped.
export const COCO_IDS = [
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 31, 32, 33, 34, 35, 36,
  37, 38, 39, 40, 41, 42, 43, 44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 67, 70,
  72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 89, 90,
];
export const ID2LABEL = Array.from({ length: 91 }, (_, id) => COCO_LABELS[COCO_IDS.indexOf(id)] ?? null);

export const MODEL_FOR = { fast: "rfdetr-n", accurate: "rfdetr-s" };

// /255 then ImageNet mean/std, as one lookup per channel. The onnx-community preprocessor config says not to normalise,
// but the graph starts at the patch convolution and needs it (mAP 45.0 -> 48.2, in the dark 26.8 -> 38.0).
const MEAN = [0.485, 0.456, 0.406];
const STD = [0.229, 0.224, 0.225];
const LUT = MEAN.map((m, c) => Float32Array.from({ length: 256 }, (_, v) => (v / 255 - m) / STD[c]));

// RGBA bytes of an n-pixel image (already stretched to the model's square input) -> planar RGB float32.
export function toPlanar(rgba, n, out = new Float32Array(3 * n)) {
  const [r, g, b] = LUT;
  for (let i = 0, j = 0; i < n; i++, j += 4) {
    out[i] = r[rgba[j]];
    out[n + i] = g[rgba[j + 1]];
    out[2 * n + i] = b[rgba[j + 2]];
  }
  return out;
}

const clamp01 = (v) => (v < 0 ? 0 : v > 1 ? 1 : v);

// logits [Q, C] (sigmoid gives the score) and boxes [Q, 4] as normalised cx, cy, w, h. A DETR query is one object, so
// there is no NMS: each query keeps its best named class if it clears thr, and the best topk queries are returned.
export function decode(logits, boxes, Q, C, { thr = 0.1, topk = 50 } = {}) {
  const lthr = Math.log(thr / (1 - thr));
  const dets = [];
  for (let q = 0; q < Q; q++) {
    const o = q * C;
    let best = lthr;
    let id = -1;
    for (const c of COCO_IDS) {
      if (logits[o + c] > best) {
        best = logits[o + c];
        id = c;
      }
    }
    if (id < 0) continue;
    const [cx, cy, w, h] = boxes.subarray(q * 4, q * 4 + 4);
    const x = clamp01(cx - w / 2);
    const y = clamp01(cy - h / 2);
    dets.push({ label: ID2LABEL[id], score: 1 / (1 + Math.exp(-best)), box: { x, y, w: clamp01(cx + w / 2) - x, h: clamp01(cy + h / 2) - y } });
  }
  dets.sort((a, b) => b.score - a.score);
  return dets.length > topk ? dets.slice(0, topk) : dets;
}

// Why RF-DETR can't run here, or null.
export function unavailable() {
  if (typeof location !== "undefined" && new URLSearchParams(location.search).has("nowebgpu")) return "WebGPU is turned off (?nowebgpu)";
  if (!globalThis.navigator?.gpu) return "this browser has no WebGPU";
  if (typeof Worker === "undefined") return "no Web Workers";
  return null;
}

const MAX_W = 512; // frames go to the worker at up to 512x384 (the 4:3 drone view)
// A frame takes ~15 ms (35 ms with the GPU shared). No reply for this long: the session hung (GPU device lost) or the
// worker died. Generous, because the fallback it triggers is for the rest of the flight.
const HANG_MS = 3000;

// Page side. load() -> {name, res, loadMs, cached}; then submit() frames, one in flight; results arrive at onResult as
// {type: "dets", t, dets, ms} or {type: "fail", t, error, fatal?}. t is the capture time given to submit(). fatal: the
// worker stopped answering, so no more results will come.
export class RfDetr {
  constructor(quality = "fast", { thr = 0.1, topk = 50 } = {}) {
    this.key = MODEL_FOR[quality] || MODEL_FOR.fast;
    this.opts = { thr, topk };
    this.worker = null;
    this.info = null;
    this.busy = false;
    this.sentAt = 0;
    this.slot = {}; // GPU.take("detect")'s
    this.onResult = null;
  }

  load(onProgress) {
    if (this.loading) return this.loading;
    globalThis.navigator?.storage?.persist?.().catch(() => {});
    this.worker = new Worker(new URL("./rfdetr-worker.js", import.meta.url), { type: "module" });
    this.loading = new Promise((resolve, reject) => {
      this.worker.onmessage = ({ data: m }) => {
        if (m.type === "progress") onProgress?.(m.loaded, m.total);
        else if (m.type === "ready") resolve((this.info = m));
        else if (m.type === "error") reject(new Error(m.error));
        else {
          this.busy = false;
          if (m.type === "dets") GPU.mark("detect", performance.now() - this.sentAt);
          this.onResult?.(m);
        }
      };
      this.worker.onerror = (e) => {
        e.preventDefault?.();
        const error = e.message || "the detector worker crashed";
        reject(new Error(error));
        this.busy = false;
        this.onResult?.({ type: "fail", t: 0, error });
      };
    });
    this.worker.postMessage({ type: "init", key: this.key, ...this.opts });
    return this.loading;
  }

  // source: canvas, video, ImageBitmap or VideoFrame; region {sx, sy, sw, sh} in source pixels (default: all of it).
  // Returns false, dropping the frame, while the previous one is still being processed.
  submit(source, t = performance.now(), region = null) {
    if (this.busy && performance.now() - this.sentAt > HANG_MS) {
      this.busy = false;
      this.onResult?.({ type: "fail", t, error: `the detector worker stopped answering (no reply in ${HANG_MS / 1000} s)`, fatal: true });
      return false;
    }
    if (this.busy || !this.info || !GPU.take("detect", this.slot)) return false;
    this.busy = true;
    const sw = region?.sw ?? source.videoWidth ?? source.displayWidth ?? source.width;
    const sh = region?.sh ?? source.videoHeight ?? source.displayHeight ?? source.height;
    const size = sw > MAX_W ? { resizeWidth: MAX_W, resizeHeight: Math.round((MAX_W * sh) / sw), resizeQuality: "medium" } : {};
    this.sentAt = performance.now();
    createImageBitmap(source, region?.sx ?? 0, region?.sy ?? 0, sw, sh, size)
      .then((bitmap) => (this.worker ? this.worker.postMessage({ type: "detect", bitmap, t }, [bitmap]) : bitmap.close())) // disposed meanwhile
      .catch((e) => {
        this.busy = false;
        this.onResult?.({ type: "fail", t, error: e.message });
      });
    return true;
  }

  dispose() {
    this.worker?.terminate();
    this.worker = null;
    this.onResult = null;
  }
}
