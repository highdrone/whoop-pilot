// Module worker running RF-DETR on onnxruntime-web WebGPU (see rfdetr.js for the protocol).
import * as ort from "../../vendor/ort/ort.webgpu.min.mjs";
import { MODELS, fetchModel } from "./models.js";
import { toPlanar, decode } from "./rfdetr.js";

ort.env.wasm.wasmPaths = new URL("../../vendor/ort/", import.meta.url).href;
ort.env.wasm.numThreads = 1; // with more, InferenceSession.create hangs inside a module worker (ORT-web 1.30.0)
ort.env.logLevel = "error";

let session, R, ctx, input, opts;

async function init({ key, thr, topk }) {
  if (!navigator.gpu || !(await navigator.gpu.requestAdapter())) throw new Error("no WebGPU adapter");
  const spec = MODELS[key];
  const t0 = performance.now();
  const { bytes, cached } = await fetchModel(spec, (loaded, total) => postMessage({ type: "progress", loaded, total }));
  const t1 = performance.now();
  // logSeverityLevel 3: errors only (ORT otherwise warns on every load that shape ops run on the CPU, which is intended)
  session = await ort.InferenceSession.create(new Uint8Array(bytes), { executionProviders: ["webgpu"], graphOptimizationLevel: "all", logSeverityLevel: 3 });
  R = spec.res;
  opts = { thr, topk };
  ctx = new OffscreenCanvas(R, R).getContext("2d", { willReadFrequently: true });
  input = new Float32Array(3 * R * R);
  // Warm-up compiles the shaders. A broken export can load fine and still return garbage, so check the outputs once.
  const { logits: L, pred_boxes: B } = await infer();
  const ok = L?.dims.length === 3 && L.dims[2] >= 91 && B?.dims[1] === L.dims[1] && B.dims[2] === 4 && L.data.every(Number.isFinite);
  if (!ok) throw new Error(`${spec.name} returned unexpected outputs`);
  postMessage({ type: "ready", name: spec.name, res: R, cached, fetchMs: t1 - t0, loadMs: performance.now() - t0 });
}

const infer = () => session.run({ [session.inputNames[0]]: new ort.Tensor("float32", input, [1, 3, R, R]) });

async function detect({ bitmap, t }) {
  const t0 = performance.now();
  ctx.drawImage(bitmap, 0, 0, R, R); // stretched, not letterboxed: that is how RF-DETR was trained and measured
  bitmap.close();
  toPlanar(ctx.getImageData(0, 0, R, R).data, R * R, input);
  const t1 = performance.now();
  const { logits: L, pred_boxes: B } = await infer();
  const t2 = performance.now();
  const dets = decode(L.data, B.data, L.dims[1], L.dims[2], opts);
  const t3 = performance.now();
  postMessage({ type: "dets", t, dets, ms: { pre: t1 - t0, run: t2 - t1, post: t3 - t2, total: t3 - t0 } });
}

self.onmessage = async ({ data }) => {
  if (data.type === "init") {
    await init(data).catch((e) => postMessage({ type: "error", error: e.message || String(e) }));
  } else if (data.type === "detect") {
    await detect(data).catch((e) => postMessage({ type: "fail", t: data.t, error: e.message || String(e) }));
  }
};
