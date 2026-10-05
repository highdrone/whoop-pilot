// Module worker running Depth Anything V2 Small on onnxruntime-web WebGPU (see depth.js for the protocol): the camera frame
// rectified to the pinhole view (nav/lens.js Rectifier), ImageNet-normalised, through the model; disparity and the
// rectified picture go back, with the same view at cfg.hires times the size (the evidence of a change). A frame of
// another size (the camera's picture keeps its aspect) gets its own rectifier. "align" then makes frame t's disparity
// (the last few are kept by t) metric against the twin's expected depth (alignDepth) and, given the twin's expected picture,
// compares the two views on nav/changes.js's grid (differences(): what nav/avoid.js and the change detector read), off
// the page's thread (the control loop runs there).
import * as ort from "../../vendor/ort/ort.webgpu.min.mjs";
import { fetchModel } from "./models.js";
import { DEPTH_MODELS, toInput, alignDepth } from "./depth.js";
import { differences } from "../nav/changes.js";
import { intrinsics } from "../twin/lens.js";
import { rectifyMap, Rectifier } from "../nav/lens.js";

ort.env.wasm.wasmPaths = new URL("../../vendor/ort/", import.meta.url).href;
ort.env.wasm.numThreads = 1; // with more, InferenceSession.create hangs inside a module worker (ORT-web 1.30.0)
ort.env.logLevel = "error";

let session, cfg, rect, rectHi, ctx, input, rgba;
const frames = new Map(); // t -> { disp, rgba }: the last few, for their "align"

function setLens(lens = cfg.lens) {
  cfg.lens = lens;
  rect = new Rectifier(rectifyMap(lens, cfg.src[0], cfg.src[1], cfg.width, cfg.height, cfg.hfov));
  rectHi = cfg.hires > 1 ? new Rectifier(rectifyMap(lens, cfg.src[0], cfg.src[1], cfg.width * cfg.hires, cfg.height * cfg.hires, cfg.hfov)) : null;
}

async function init(c) {
  if (!navigator.gpu || !(await navigator.gpu.requestAdapter())) throw new Error("no WebGPU adapter");
  cfg = c;
  const spec = DEPTH_MODELS[c.key], t0 = performance.now();
  const { bytes, cached } = await fetchModel(spec, (loaded, total) => postMessage({ type: "progress", loaded, total }));
  session = await ort.InferenceSession.create(new Uint8Array(bytes), { executionProviders: ["webgpu"], graphOptimizationLevel: "all", logSeverityLevel: 3 });
  setLens(c.lens);
  ctx = new OffscreenCanvas(c.src[0], c.src[1]).getContext("2d", { willReadFrequently: true });
  input = new Float32Array(3 * c.width * c.height);
  rgba = new Uint8ClampedArray(c.width * c.height * 4);
  const out = await infer(); // warm-up: compiles the shaders; a broken export loads fine and answers garbage
  if (out.data.length !== c.width * c.height || !out.data.every(Number.isFinite)) throw new Error(`${spec.name} returned unexpected outputs (${out.dims})`);
  postMessage({ type: "ready", name: spec.name, cached, loadMs: performance.now() - t0, input: session.inputNames[0], output: session.outputNames[0] });
}

async function infer() {
  const r = await session.run({ [session.inputNames[0]]: new ort.Tensor("float32", input, [1, 3, cfg.height, cfg.width]) });
  return r[session.outputNames[0]];
}

async function run({ id, bitmap, t }) {
  const t0 = performance.now();
  if (bitmap.width !== cfg.src[0] || bitmap.height !== cfg.src[1]) {
    cfg.src = [bitmap.width, bitmap.height];
    ctx = new OffscreenCanvas(...cfg.src).getContext("2d", { willReadFrequently: true });
    setLens();
  }
  ctx.drawImage(bitmap, 0, 0, cfg.src[0], cfg.src[1]);
  bitmap.close();
  const px = ctx.getImageData(0, 0, cfg.src[0], cfg.src[1]).data, out = rect.rgba(px, new Uint8ClampedArray(rgba.length)), hi = rectHi?.rgba(px) ?? null;
  toInput(out, cfg.width * cfg.height, input);
  const t1 = performance.now(), d = await infer(), t2 = performance.now();
  const disp = Float32Array.from(d.data);
  for (let i = 0; i < disp.length; i++) if (!rect.valid[i]) disp[i] = NaN; // outside the fisheye
  frames.set(t, { disp: disp.slice(), rgba: out.slice() });
  for (const k of frames.keys()) if (frames.size > 4) frames.delete(k);
  postMessage({ type: "depth", id, t, disp, rgba: out, hi, ms: { pre: t1 - t0, run: t2 - t1, total: performance.now() - t0 } }, [disp.buffer, out.buffer, ...(hi ? [hi.buffer] : [])]);
}

// Frame t's disparity against expected (range along each ray, the same view) -> a: { depth, conf, scale, shift, inliers,
// absRel, n } | null; with expectedRgb (the twin's picture; boxes, mask as the frame will carry them) also grid:
// differences() of the frame.
function align({ id, t, expected, lens, expectedRgb, boxes, mask }) {
  const fr = frames.get(t);
  frames.delete(t);
  if (!fr) throw new Error("its frame is gone from the depth worker");
  const t0 = performance.now(), a = alignDepth(fr.disp, expected, intrinsics(lens, cfg.width, cfg.height)), t1 = performance.now();
  const grid = a && expectedRgb ? differences({ width: cfg.width, height: cfg.height, lens, depth: a.depth, conf: a.conf, expected, expectedRgb, rgb: { width: cfg.width, height: cfg.height, data: fr.rgba }, boxes, mask }) : null;
  postMessage({ type: "aligned", id, a, grid, ms: t1 - t0, gridMs: performance.now() - t1 }, a ? [a.depth.buffer, a.conf.buffer] : []);
}

self.onmessage = async ({ data }) => {
  if (data.type === "init") await init(data).catch((e) => postMessage({ type: "error", error: e.message || String(e) }));
  else if (data.type === "lens") setLens(data.lens);
  else if (data.type === "run") await run(data).catch((e) => postMessage({ type: "fail", id: data.id, error: e.message || String(e) }));
  else if (data.type === "align") {
    try {
      align(data);
    } catch (e) {
      postMessage({ type: "fail", id: data.id, error: e.message || String(e) });
    }
  }
};
