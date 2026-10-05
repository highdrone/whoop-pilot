// Localization worker (features.js protocol { id, op, args } -> { id, result } | { id, error }): fisheye rectification,
// XFeat and the matcher on onnxruntime-web (WebGPU, else WASM), DINOv2-small, OpenCV.js PnP. Frames and renders stay
// here under ids (a few at a time), so the page only sends pixels in and gets poses out.
import * as ort from "../../vendor/ort/ort.webgpu.min.mjs";
import { MODELS, fetchModel, sha256 } from "../vision/models.js";
import { Rectifier, rectifyMap } from "./lens.js";
import { XFEAT, OPENCV, DINO, MIN_COS, xfeatPost, mutual, mnnModel, lift, solvePnP, loadOpenCV, openCvText, dinoInput, dinoDescriptor } from "./features.js";

ort.env.wasm.wasmPaths = new URL("../../vendor/ort/", import.meta.url).href;
ort.env.wasm.numThreads = 1; // more hangs InferenceSession.create in a module worker (ORT-web 1.30.0, see vision/rfdetr-worker.js)
ort.env.logLevel = "error";

const TOPK = 2048, KEEP = 12;
let xs, ms, ds, cv, cvLoading, backend, dinoSpec;
const fallbacks = [];
const rects = new Map(), store = new Map(), now = () => performance.now();
let seq = 0;
const W = XFEAT.width, H = XFEAT.height, input = new Float32Array(3 * W * H);

// WebGPU first, WASM if that session can't be made (a driver that rejects an op): the backend that took it is kept.
async function session(bytes, name) {
  const b = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes), make = (eps) => ort.InferenceSession.create(b, { executionProviders: eps, graphOptimizationLevel: "all", logSeverityLevel: 3 });
  if (backend !== "webgpu") return make(["wasm"]);
  try {
    return await make(["webgpu"]);
  } catch (e) {
    fallbacks.push(`${name} on WASM (WebGPU: ${String(e?.message || e).split("\n")[0]})`);
    return make(["wasm"]);
  }
}

async function init({ backend: want = "auto" }) {
  const t0 = now(), gpu = want !== "wasm" && !!navigator.gpu && !!(await navigator.gpu.requestAdapter().catch(() => null));
  if (want === "webgpu" && !gpu) throw new Error("no WebGPU adapter");
  backend = gpu ? "webgpu" : "wasm";
  const r = await fetch(XFEAT.url);
  if (!r.ok) throw new Error(`XFeat model: HTTP ${r.status}`);
  const xb = await r.arrayBuffer();
  if (xb.byteLength !== XFEAT.size || (await sha256(xb)) !== XFEAT.sha256) throw new Error("XFeat model: checksum mismatch");
  xs = await session(xb, "XFeat");
  ms = await session(mnnModel(TOPK, TOPK), "the matcher");
  const t1 = now();
  openCv().catch(() => {}); // unpacked and compiled meanwhile; the first solve waits for it
  dinoSpec = MODELS[gpu ? "dinov2-s" : "dinov2-s-q8"];
  // warm-up: compile the shaders once
  await xfeatRun(new Float32Array(3 * W * H));
  await ms.run({ a: new ort.Tensor("float32", new Float32Array(TOPK * 64), [TOPK, 64]), b: new ort.Tensor("float32", new Float32Array(TOPK * 64), [TOPK, 64]) });
  const t2 = now();
  await openCv();
  return { backend, fallbacks, xfeat: `${W}x${H}`, dino: dinoSpec.name, dinoLoaded: false, threads: ort.env.wasm.numThreads, loadMs: Math.round(now() - t0),
    modelsMs: Math.round(t2 - t0), opencvMs: Math.round(cvLoading.ms), opencvWaitMs: Math.round(now() - t2), coi: self.crossOriginIsolated };
}

// OpenCV.js, once: the vendored gzip unpacked, checked and compiled (about 0.5 s; it runs beside the XFeat warm-up).
function openCv() {
  return (cvLoading ??= (async () => {
    const t = now(), r = await fetch(OPENCV.url);
    if (!r.ok) throw new Error(`OpenCV.js: HTTP ${r.status}`);
    ({ cv } = await loadOpenCV(await openCvText(await r.arrayBuffer())));
    cvLoading.ms = now() - t;
  })());
}

// DINOv2 on demand (relocalization only): downloaded once (progress messages; outside the op queue, so tracking goes on
// meanwhile), then from Cache Storage; the session is made in the queue.
async function loadDino({ bytes, t0 }) {
  ds ??= await session(bytes, dinoSpec.name);
  await dinoRun(new Float32Array(3 * DINO.width * DINO.height));
  return { dino: dinoSpec.name, dinoMs: Math.round(now() - t0), fallbacks };
}

async function xfeatRun(planar) {
  const out = await xs.run({ input: new ort.Tensor("float32", planar, [1, 3, H, W]) }), by = {};
  for (const k of xs.outputNames) by[out[k].dims[1]] = out[k].data;
  return by;
}

async function dinoRun(planar) {
  const type = ds.inputMetadata?.[0]?.type, t = type === "float16" ? new ort.Tensor("float16", Float16Array.from(planar), [1, 3, DINO.height, DINO.width])
    : new ort.Tensor("float32", planar, [1, 3, DINO.height, DINO.width]);
  const out = await ds.run({ pixel_values: t }), lh = out.last_hidden_state ?? out[ds.outputNames[0]];
  return dinoDescriptor(lh.data instanceof Float32Array ? lh.data : Float32Array.from(lh.data), lh.dims[1], lh.dims[2]);
}

const planarOf = (rgba, out = input) => {
  const n = W * H;
  for (let i = 0, j = 0; i < n; i++, j += 4) {
    out[i] = rgba[j] / 255;
    out[n + i] = rgba[j + 1] / 255;
    out[2 * n + i] = rgba[j + 2] / 255;
  }
  return out;
};

async function detect(rgba, allow, topK = TOPK) {
  const t0 = now(), by = await xfeatRun(planarOf(rgba)), t1 = now();
  const f = xfeatPost(by[64], by[65], by[1], W, H, { topK, allow });
  return { f, ms: { net: t1 - t0, post: now() - t1 } };
}

function keep(entry) {
  const id = ++seq;
  store.set(id, entry);
  while (store.size > KEEP) store.delete(store.keys().next().value);
  return id;
}

// The rectifier on the GPU (WebGL2): the frame as a texture, the map as an RG32F lookup, bilinear sampling; rows come
// back top-down. Falls back to the CPU Rectifier where WebGL2 is missing.
class GlRectifier {
  constructor(r) {
    const c = (this.canvas = new OffscreenCanvas(r.outW, r.outH)), gl = (this.gl = c.getContext("webgl2", { antialias: false, depth: false, premultipliedAlpha: false }));
    if (!gl) throw new Error("no WebGL2");
    Object.assign(this, { w: r.outW, h: r.outH, srcW: r.srcW, srcH: r.srcH });
    const sh = (type, src) => { const o = gl.createShader(type); gl.shaderSource(o, src); gl.compileShader(o); if (!gl.getShaderParameter(o, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(o)); return o; };
    const prog = (this.prog = gl.createProgram());
    gl.attachShader(prog, sh(gl.VERTEX_SHADER, "#version 300 es\nvoid main(){vec2 p=vec2(float((gl_VertexID<<1)&2),float(gl_VertexID&2));gl_Position=vec4(p*2.0-1.0,0.0,1.0);}"));
    gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, `#version 300 es
precision highp float; uniform sampler2D src; uniform highp sampler2D lut; uniform vec2 size; out vec4 o;
void main(){ vec2 m = texelFetch(lut, ivec2(gl_FragCoord.xy), 0).xy; o = m.x < 0.0 ? vec4(0.0, 0.0, 0.0, 1.0) : vec4(texture(src, m / size).rgb, 1.0); }`));
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
    const lut = new Float32Array(r.map.length);
    for (let i = 0; i < lut.length; i++) lut[i] = Number.isNaN(r.map[i]) ? -1 : r.map[i];
    this.lut = this.texture(gl.RG32F, gl.RG, gl.FLOAT, r.outW, r.outH, lut, gl.NEAREST);
    this.src = this.texture(gl.RGBA8, gl.RGBA, gl.UNSIGNED_BYTE, r.srcW, r.srcH, null, gl.LINEAR);
    this.out = new Uint8Array(r.outW * r.outH * 4);
  }
  texture(internal, format, type, w, h, data, filter) {
    const gl = this.gl, t = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, t);
    gl.texImage2D(gl.TEXTURE_2D, 0, internal, w, h, 0, format, type, data);
    for (const [k, v] of [[gl.TEXTURE_MIN_FILTER, filter], [gl.TEXTURE_MAG_FILTER, filter], [gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE], [gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE]]) gl.texParameteri(gl.TEXTURE_2D, k, v);
    return t;
  }
  rgba(bitmap) {
    const gl = this.gl;
    gl.useProgram(this.prog);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.src);
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
    gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, gl.RGBA, gl.UNSIGNED_BYTE, bitmap);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, this.lut);
    gl.uniform1i(gl.getUniformLocation(this.prog, "src"), 0);
    gl.uniform1i(gl.getUniformLocation(this.prog, "lut"), 1);
    gl.uniform2f(gl.getUniformLocation(this.prog, "size"), this.srcW, this.srcH);
    gl.viewport(0, 0, this.w, this.h);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
    const out = new Uint8ClampedArray(this.w * this.h * 4);
    gl.readPixels(0, 0, this.w, this.h, gl.RGBA, gl.UNSIGNED_BYTE, out);
    return out;
  }
}

let canvas, ctx;
function pixelsOf(image) {
  if (image.data) return { width: image.width, height: image.height, data: image.data };
  if (!canvas || canvas.width !== image.width || canvas.height !== image.height) {
    canvas = new OffscreenCanvas(image.width, image.height);
    ctx = canvas.getContext("2d", { willReadFrequently: true });
  }
  ctx.drawImage(image, 0, 0);
  image.close?.();
  return ctx.getImageData(0, 0, canvas.width, canvas.height);
}

const ops = {
  init,
  loadDino,

  rectifier({ key, lens, srcW, srcH, outW = W, outH = H, hfov }) {
    const r = new Rectifier(rectifyMap(lens, srcW, srcH, outW, outH, hfov));
    let gl = null;
    try {
      gl = new GlRectifier(r);
    } catch (e) {
      console.warn("rectifying on the CPU:", e.message);
    }
    rects.set(key, { r, gl, mask: null });
    return { key, valid: +(r.valid.reduce((a, v) => a + v, 0) / r.valid.length).toFixed(3), K: r.K };
  },

  // A source-frame mask (OsdMask, 1 = usable) for a rectifier, as an output-pixel mask.
  setMask({ key, width, height, data }) {
    const e = rects.get(key), { r } = e, n = r.outW * r.outH, m = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      if (!r.valid[i]) continue;
      const x = Math.min(width - 1, ((r.map[2 * i] * width) / r.srcW) | 0), y = Math.min(height - 1, ((r.map[2 * i + 1] * height) / r.srcH) | 0);
      m[i] = data[y * width + x];
    }
    e.mask = m;
    return { masked: +(1 - m.reduce((a, v) => a + v, 0) / r.valid.reduce((a, v) => a + v, 0)).toFixed(3) };
  },

  async frame({ image, rect, boxes = [], topK = TOPK, dino = false, keep: keepPx = false }) {
    const t0 = now(), e = rect != null ? rects.get(rect) : null;
    if (rect != null && !e) throw new Error(`no rectifier ${rect}`);
    if (e && (image.width !== e.r.srcW || image.height !== e.r.srcH)) throw new Error(`frame ${image.width}x${image.height}, rectifier ${e.r.srcW}x${e.r.srcH}`);
    if (!e && (image.width !== W || image.height !== H)) throw new Error(`frame must be ${W}x${H} without a rectifier`);
    let rgba;
    if (e?.gl && image instanceof ImageBitmap) {
      rgba = e.gl.rgba(image);
      image.close();
    } else rgba = e ? e.r.rgba(pixelsOf(image).data) : pixelsOf(image).data;
    const t1 = now();
    const valid = e?.r.valid, mask = e?.mask, map = e?.r.map;
    const bx = boxes.map((b) => [b.x - 0.04 * b.w, b.y - 0.04 * b.h, b.x + 1.04 * b.w, b.y + 1.04 * b.h]);
    const allow = (x, y) => {
      const i = y * W + x;
      if (valid && !valid[i]) return false;
      if (mask && !mask[i]) return false;
      if (!bx.length) return true;
      const u = (map ? map[2 * i] : x + 0.5) / (e ? e.r.srcW : W), v = (map ? map[2 * i + 1] : y + 0.5) / (e ? e.r.srcH : H);
      return !bx.some((b) => u >= b[0] && u <= b[2] && v >= b[1] && v <= b[3]);
    };
    const { f, ms: m } = await detect(rgba, allow, topK);
    const g = dino && ds ? await dinoRun(dinoInput(rgba, W, H)) : null;
    const id = keep({ f, rgba: keepPx || dino ? rgba : null, rect, g });
    return { id, n: f.n, kpts: f.kpts, desc: g, ms: { rectify: t1 - t0, ...m, total: now() - t0 } };
  },

  async render({ rgba, depth, depthW = W, depthH = H, K, cam, topK = TOPK, keepPx = false }) {
    if (rgba.width !== W || rgba.height !== H) throw new Error(`render must be ${W}x${H}`);
    const t0 = now(), { f, ms: m } = await detect(rgba.data, null, topK);
    const P = depth ? lift(f.kpts, f.n, depth, depthW, depthH, K, cam, depthW / W) : null;
    let lifted = 0;
    if (P) for (let i = 0; i < f.n; i++) lifted += !Number.isNaN(P[3 * i]);
    const id = keep({ f, P, K, cam, rgba: keepPx ? rgba.data : null });
    return { id, n: f.n, lifted, ms: { ...m, total: now() - t0 } };
  },

  async solve({ live, ref, K, thr = 4, iters = 400, minCos = MIN_COS, viz = false }) {
    const a = store.get(live), b = store.get(ref);
    if (!a || !b) throw new Error("frame or render no longer kept");
    const t0 = now(), { pairs, cos } = await match(a.f, b.f, minCos), t1 = now();
    const obj = [], img = [], which = [];
    for (let k = 0; k < pairs.length; k += 2) {
      const i = pairs[k], j = pairs[k + 1];
      if (Number.isNaN(b.P[3 * j])) continue;
      obj.push(b.P[3 * j], b.P[3 * j + 1], b.P[3 * j + 2]);
      img.push(a.f.kpts[2 * i] + 0.5, a.f.kpts[2 * i + 1] + 0.5);
      which.push(i, j);
    }
    const o = Float64Array.from(obj), p = Float64Array.from(img), sol = solvePnP(cv, o, p, K, { thr, iters });
    const out = { ok: !!sol, matches: pairs.length / 2, lifted: obj.length / 3, inliers: sol?.inliers.length ?? 0, ms: { match: t1 - t0, pnp: now() - t1 } };
    if (sol) Object.assign(out, { R: sol.R, C: sol.C, rms: sol.rms, cov: sol.cov, ransacInliers: sol.ransacInliers });
    if (sol) a.lastSolve = { ref, which, inliers: sol.inliers, obj: o, img: p };
    if (viz) {
      const inl = new Set(sol?.inliers ?? []);
      out.viz = { live: Float32Array.from(which.filter((_, k) => k % 2 === 0).flatMap((i) => [a.f.kpts[2 * i], a.f.kpts[2 * i + 1]])),
        ref: Float32Array.from(which.filter((_, k) => k % 2).flatMap((j) => [b.f.kpts[2 * j], b.f.kpts[2 * j + 1]])),
        inlier: Uint8Array.from({ length: which.length / 2 }, (_, k) => inl.has(k)) };
    }
    return out;
  },

  // The last solve's inliers as fisheye pixels and H points (lens calibration).
  calibSample({ live }) {
    const a = store.get(live), s = a?.lastSolve, e = a && rects.get(a.rect);
    if (!s || !e) return null;
    const uv = [], xyz = [];
    for (const k of s.inliers) {
      const q = e.r.toSource(s.img[2 * k], s.img[2 * k + 1]);
      if (!q) continue;
      uv.push(q[0], q[1]);
      xyz.push(s.obj[3 * k], s.obj[3 * k + 1], s.obj[3 * k + 2]);
    }
    return { uv: Float64Array.from(uv), xyz: Float64Array.from(xyz), width: e.r.srcW, height: e.r.srcH };
  },

  async global({ id, rgba }) {
    if (!ds) throw new Error("DINOv2 is not loaded");
    const t0 = now();
    if (id != null) {
      const e = store.get(id);
      if (!e) throw new Error("frame no longer kept");
      if (!e.g) e.g = await dinoRun(dinoInput(e.rgba, W, H));
      return { desc: e.g, ms: now() - t0 };
    }
    return { desc: await dinoRun(dinoInput(rgba.data, rgba.width, rgba.height)), ms: now() - t0 };
  },

  rectified(id) {
    const e = store.get(id);
    return e?.rgba ? { width: W, height: H, data: new Uint8ClampedArray(e.rgba) } : null;
  },

  release(ids) {
    for (const id of [].concat(ids)) store.delete(id);
    return store.size;
  },
};

const pad = (f) => { const d = new Float32Array(TOPK * 64); d.set(f.desc.subarray(0, Math.min(f.n, TOPK) * 64)); return d; };
async function match(a, b, minCos) {
  const o = await ms.run({ a: new ort.Tensor("float32", pad(a), [TOPK, 64]), b: new ort.Tensor("float32", pad(b), [TOPK, 64]) });
  return mutual(o.iab.data, o.sab.data, o.iba.data, a.n, b.n, minCos);
}

const transferOf = (r) => (r?.data?.buffer ? [r.data.buffer] : []);
let queue = Promise.resolve();
const fail = (id, e) => self.postMessage({ id, error: String(e?.stack || e) });
const run = (id, fn) => (queue = queue.then(async () => {
  try {
    const result = await fn();
    self.postMessage({ id, result }, transferOf(result));
  } catch (e) {
    fail(id, e);
  }
}));
self.onmessage = ({ data: { id, op, args } }) => {
  if (op !== "loadDino" || ds) return run(id, () => ops[op](args ?? {}));
  const t0 = now();
  fetchModel(dinoSpec, (loaded, total) => postMessage({ progress: { what: dinoSpec.name, loaded, total } })).then(({ bytes }) => run(id, () => loadDino({ bytes, t0 })), (e) => fail(id, e));
};
