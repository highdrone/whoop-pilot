// Twin worker: Spark 2.3.1 + three 0.186.1 on an OffscreenCanvas. Renders the drone camera, expected depth and
// actor ground truth from the house splat, with actors and props (boxes, door panels) composited by depth.
// Protocol: { id, op, args, prio } -> { id, result, stats } | { id, error }; jobs by priority (nextJob).
// A wide lens is drawn as the camera-aligned pinhole views it needs (lens.js layout) into one atlas, then warped
// to the lens through a per-pixel lookup table. One SparkRenderer serves every pass (one sort per position):
// colour, and expected depth with actor transmittance (z*alpha, alpha, T in a float target) swap its material.
import * as THREE from "../../vendor/three/build/three.module.js";
import { resolveLens, intrinsics, layout } from "./lens.js";
import { droneCamera } from "./pose.js";
import { rootMatrix } from "../house/frames.js";
import { Actors, depthMaterial } from "./actors.js";
import { Props } from "./props.js";
import { nextJob, transferOf } from "./queue.js";

globalThis.window ??= globalThis; // Spark resolves URLs through window.location
const sparkModule = import("../../vendor/spark/spark.module.js");

const DEPTH_FRAG = `precision highp float;
precision highp int;
#include <splatDefines>
uniform float near; uniform float far; uniform bool encodeLinear; uniform float time; uniform bool debugFlag;
uniform float maxStdDev; uniform float minAlpha; uniform bool disableFalloff; uniform float falloff;
out vec4 fragColor;
in vec4 vRgba; in vec2 vSplatUv; in vec3 vNdc; flat in uint vSplatIndex; flat in float adjustedStdDev;
void main() {
  float a = vRgba.a, z2 = dot(vSplatUv, vSplatUv);
  if (z2 > adjustedStdDev * adjustedStdDev) discard;
  if (a <= 1.0) a = mix(a, a * exp(-0.5 * z2), falloff);
  else a = mix(1.0, 1.0 - pow(1.0 - exp(-0.5 * z2), exp((a * a - 1.0) / 2.718281828459045)), falloff);
  if (a < minAlpha) discard;
  float zview = 2.0 * near * far / ((far + near) - vNdc.z * (far - near));
  fragColor = vec4(zview * a, a, 0.0, a);
}`;
const WARP_FRAG = `precision highp float;
uniform sampler2D atlas; uniform sampler2D lut; uniform int mode;
out vec4 outColor;
void main() {
  vec4 L = texelFetch(lut, ivec2(gl_FragCoord.xy), 0);
  if (L.w == 0.0) { outColor = vec4(0.0, 0.0, 0.0, mode == 0 ? 1.0 : 0.0); return; }
  vec4 c = texture(atlas, L.xy);
  if (mode == 0) outColor = vec4(c.rgb, 1.0);
  else if (mode == 1) outColor = vec4(c.g > 1e-4 ? c.r / c.g * L.z : 0.0, c.g, c.b, 1.0);
  else outColor = c;
}`;
const MODE = { color: 0, depth: 1, id: 2 };
const NEAR = 0.02, FAR = 60;

let renderer, canvas, scene, spark, colorMat, depthMat, actorDepthMat, actors, props, warp, floatType, floatFilter, lens = resolveLens();
const opts = { resortDist: 0.1, quality: 1 };
let sortedAt = null;
const layouts = new Map(), targets = {}, faceCam = new THREE.PerspectiveCamera(90, 1, NEAR, FAR), sortCam = new THREE.Camera();
faceCam.matrixAutoUpdate = false;
const now = () => performance.now();
const stats = { loadMs: 0, splats: 0, renders: 0, sorts: 0, renderMs: 0, renderMsAvg: 0, sortMs: 0, fps: 0, views: 0, atlas: "" };
const recent = [];

async function init({ splat, frame, lens: l, resortDist, quality, minOpacity = 0 }) {
  const t0 = now();
  const { SparkRenderer, SplatMesh, PackedSplats } = await sparkModule;
  lens = checkedLens(l, lens);
  if (resortDist != null) opts.resortDist = resortDist;
  if (quality != null) opts.quality = quality;
  canvas = new OffscreenCanvas(640, 480);
  renderer = new THREE.WebGLRenderer({ canvas, antialias: false, alpha: false, powerPreference: "high-performance" });
  renderer.setPixelRatio(1);
  renderer.setClearColor(0x000000, 0);
  renderer.readRenderTargetPixelsAsync = readback; // Spark's sort keys come back through it too
  const gl = renderer.getContext();
  floatType = gl.getExtension("EXT_float_blend") ? THREE.FloatType : THREE.HalfFloatType;
  floatFilter = floatType === THREE.HalfFloatType || renderer.extensions.has("OES_texture_float_linear") ? THREE.LinearFilter : THREE.NearestFilter;

  scene = new THREE.Scene();
  spark = new SparkRenderer({ renderer, autoUpdate: false, enableLod: false });
  spark.readPause = 0; // its 1 ms setTimeout before each sort also stalls when hidden
  const vp = new THREE.Vector4(), before = spark.onBeforeRender;
  spark.onBeforeRender = function (r, ...rest) {
    before.call(this, r, ...rest);
    // Blend in sRGB like the trainer (Spark switches to linear for render targets: about 4 dB worse vs photos),
    // and size splats for the current view, not the whole atlas.
    this.uniforms.encodeLinear.value = false;
    r.getCurrentViewport(vp);
    this.uniforms.renderSize.value.set(vp.z, vp.w);
  };
  scene.add(spark);
  colorMat = spark.material;
  depthMat = new THREE.ShaderMaterial({ glslVersion: THREE.GLSL3, vertexShader: colorMat.vertexShader, fragmentShader: DEPTH_FRAG, uniforms: colorMat.uniforms,
    premultipliedAlpha: true, transparent: true, depthTest: true, depthWrite: false, side: THREE.DoubleSide });
  depthMat.allowOverride = false;
  actorDepthMat = depthMaterial();

  const root = new THREE.Group();
  root.matrixAutoUpdate = false;
  root.matrix.fromArray(rootMatrix(frame.f, frame.Yf));
  scene.add(root);
  const bytes = splat instanceof Blob ? await splat.arrayBuffer() : typeof splat === "string" ? await (await fetchOk(splat)).arrayBuffer() : splat;
  let mesh = new SplatMesh({ fileBytes: new Uint8Array(bytes), fileName: "splat.spz" });
  await mesh.initialized;
  if (minOpacity > 0) mesh = await pruned(mesh, minOpacity, SplatMesh, PackedSplats);
  root.add(mesh);
  scene.updateMatrixWorld(true); // Spark generates from matrixWorld, which a render would otherwise only set later
  stats.splats = mesh.packedSplats.numSplats;

  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a7a6a, 2.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4);
  sun.position.set(1, 3, 2);
  scene.add(sun);
  actors = new Actors();
  scene.add(actors.root);
  props = new Props();
  scene.add(props.root);

  const warpMat = new THREE.ShaderMaterial({ glslVersion: THREE.GLSL3, uniforms: { atlas: { value: null }, lut: { value: null }, mode: { value: 0 } },
    vertexShader: "void main() { gl_Position = vec4(position.xy, 0.0, 1.0); }", fragmentShader: WARP_FRAG, depthTest: false, depthWrite: false });
  const quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), warpMat);
  quad.frustumCulled = false;
  warp = { scene: new THREE.Scene().add(quad), cam: new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1), mat: warpMat };
  stats.loadMs = Math.round(now() - t0);
  return { splats: stats.splats, loadMs: stats.loadMs, floatType: floatType === THREE.FloatType ? "float32" : "half", gpu: gpuName(gl) };
}

async function fetchOk(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return r;
}
// GPU -> CPU through a pixel-pack buffer, polling the fence with message ticks. three's version polls with timers
// (throttled to ~100 ms when the page is hidden); a plain readPixels copies through IPC chunks (60 ms for Spark's
// 12 MB of sort keys on an M5 Max, against 6 ms this way).
const ticker = new MessageChannel(), waiters = [];
ticker.port2.onmessage = () => waiters.shift()?.();
const tick = () => new Promise((r) => { waiters.push(r); ticker.port1.postMessage(0); });
async function gpuDone() {
  const gl = renderer.getContext(), sync = gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0);
  gl.flush();
  while (gl.clientWaitSync(sync, 0, 0) === gl.TIMEOUT_EXPIRED) await tick();
  gl.deleteSync(sync);
}
// readStart queues the copy and returns the function that collects it once the GPU is done, so passes can share one wait.
async function readback(...args) {
  const collect = readStart(...args);
  await gpuDone();
  return collect();
}
function readStart(rt, x, y, w, h, buf, face, idx = 0) {
  const gl = renderer.getContext(), fbOf = (t) => (t ? renderer.properties.get(t).__webglFramebuffer : null);
  let fb = fbOf(rt);
  if (rt?.isWebGLCubeRenderTarget && face !== undefined) fb = fb[face];
  renderer.state.bindFramebuffer(gl.FRAMEBUFFER, fb);
  if (rt?.textures.length > 1) gl.readBuffer(gl.COLOR_ATTACHMENT0 + idx);
  const tex = rt?.textures[idx], pbo = gl.createBuffer();
  gl.bindBuffer(gl.PIXEL_PACK_BUFFER, pbo);
  gl.bufferData(gl.PIXEL_PACK_BUFFER, buf.byteLength, gl.STREAM_READ);
  gl.readPixels(x, y, w, h, tex?.format === THREE.RGBAIntegerFormat ? gl.RGBA_INTEGER : gl.RGBA,
    { [THREE.FloatType]: gl.FLOAT, [THREE.HalfFloatType]: gl.HALF_FLOAT, [THREE.UnsignedIntType]: gl.UNSIGNED_INT }[tex?.type] ?? gl.UNSIGNED_BYTE, 0);
  gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
  renderer.state.bindFramebuffer(gl.FRAMEBUFFER, fbOf(renderer.getRenderTarget()));
  return () => {
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, pbo);
    gl.getBufferSubData(gl.PIXEL_PACK_BUFFER, 0, buf);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    gl.deleteBuffer(pbo);
    return buf;
  };
}
const gpuName = (gl) => { const d = gl.getExtension("WEBGL_debug_renderer_info"); return d ? gl.getParameter(d.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER); };

// Copy of the splats with opacity >= minOpacity (75% of splats at 0.05, about 40 dB against the full set).
async function pruned(mesh, minOpacity, SplatMesh, PackedSplats) {
  const ps = mesh.packedSplats, arr = ps.packedArray, n = ps.numSplats, tb = Math.round(minOpacity * 255);
  let m = 0;
  for (let i = 0; i < n; i++) if (arr[i * 4] >>> 24 >= tb) m++;
  const cap = Math.ceil(m / 2048) * 2048, sub = new Uint32Array(cap * 4), extra = {}, src = {}, width = { sh1: 2, sh2: 4, sh3: 4 };
  const keys = Object.keys(width).filter((k) => ps.extra[k]);
  for (const k of keys) { src[k] = new Uint32Array(ps.extra[k].buffer, ps.extra[k].byteOffset, ps.extra[k].byteLength >> 2); extra[k] = new Uint32Array(cap * width[k]); }
  for (let i = 0, j = 0; i < n; i++) {
    if (arr[i * 4] >>> 24 < tb) continue;
    sub.set(arr.subarray(i * 4, i * 4 + 4), j * 4);
    for (const k of keys) extra[k].set(src[k].subarray(i * width[k], (i + 1) * width[k]), j * width[k]);
    j++;
  }
  const out = new SplatMesh({ packedSplats: new PackedSplats({ packedArray: sub, numSplats: m, extra, splatEncoding: ps.splatEncoding }) });
  await out.initialized;
  mesh.dispose();
  return out;
}

function target(key, w, h, o) {
  let t = targets[key];
  if (!t) t = targets[key] = new THREE.WebGLRenderTarget(w, h, { colorSpace: THREE.NoColorSpace, generateMipmaps: false, ...o });
  else if (t.width !== w || t.height !== h) t.setSize(w, h);
  return t;
}

const checkedLens = (l, base) => { const L = resolveLens(l, base); intrinsics(L, 4, 3); return L; };

function getLayout(view) {
  const width = view.width ?? 640, height = view.height ?? 480, L = view.lens ? resolveLens(view.lens, lens) : lens;
  const key = JSON.stringify([L, width, height, view.ss ?? 1, opts.quality]);
  let e = layouts.get(key);
  if (!e) {
    if (layouts.size > 8) { for (const v of layouts.values()) v.lut.dispose(); layouts.clear(); }
    const lay = layout(L, width, height, { ss: view.ss ?? 1, quality: opts.quality });
    const lut = new THREE.DataTexture(lay.lut, width, height, THREE.RGBAFormat, THREE.FloatType);
    lut.needsUpdate = true;
    e = { ...lay, lens: L, lut };
    layouts.set(key, e);
  }
  return e;
}

async function place(pose, lay) {
  const cam = droneCamera(pose, lay.lens.uptiltDeg ?? 0);
  const p = new THREE.Vector3(...cam.pThree);
  if (!sortedAt || p.distanceTo(sortedAt) > opts.resortDist) {
    const t = now();
    sortCam.position.copy(p);
    sortCam.updateMatrixWorld(true);
    await spark.update({ scene, camera: sortCam });
    stats.sortMs = +(now() - t).toFixed(1);
    stats.sorts++;
    sortedAt = p;
  }
  return cam;
}

const mv = (R, v) => R.map((row) => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
// Draw every view of the layout into an atlas target with the scene as currently set up.
function drawViews(lay, cam, atlas) {
  const [px, py, pz] = cam.pThree, neg = (v) => v.map((c) => -c);
  for (const v of lay.views) {
    const X = mv(cam.RThree, v.r), Y = neg(mv(cam.RThree, v.d)), Z = neg(mv(cam.RThree, v.f)); // three.js camera looks down -z, y up
    faceCam.matrix.set(X[0], Y[0], Z[0], px, X[1], Y[1], Z[1], py, X[2], Y[2], Z[2], pz, 0, 0, 0, 1);
    faceCam.updateMatrixWorld(true);
    faceCam.projectionMatrix.makePerspective(v.aMin * NEAR, v.aMax * NEAR, -v.bMin * NEAR, -v.bMax * NEAR, NEAR, FAR);
    faceCam.projectionMatrixInverse.copy(faceCam.projectionMatrix).invert();
    atlas.viewport.set(v.x0, 0, v.w, v.h);
    atlas.scissor.set(v.x0, 0, v.w, v.h);
    atlas.scissorTest = true;
    renderer.setRenderTarget(atlas);
    renderer.render(scene, faceCam);
  }
}

function warpTo(lay, atlas, mode, out) {
  warp.mat.uniforms.atlas.value = atlas.texture;
  warp.mat.uniforms.lut.value = lay.lut;
  warp.mat.uniforms.mode.value = mode;
  if (!out && (canvas.width !== lay.width || canvas.height !== lay.height)) renderer.setSize(lay.width, lay.height, false);
  renderer.setRenderTarget(out ?? null);
  renderer.render(warp.scene, warp.cam);
}

const atlasOf = (lay, kind) => kind === "float"
  ? target("floatAtlas", lay.atlasW, lay.atlasH, { type: floatType, minFilter: floatFilter, magFilter: floatFilter })
  : kind === "id" ? target("idAtlas", lay.atlasW, lay.atlasH, { minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter })
  : target("colorAtlas", lay.atlasW, lay.atlasH, { minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter });

function passColor(lay, cam, withActors, withProps) {
  spark.material = colorMat;
  spark.visible = true;
  actors.root.visible = withActors;
  props.root.visible = withProps;
  actors.use("shade");
  const atlas = atlasOf(lay, "color");
  try { drawViews(lay, cam, atlas); } finally { actors.root.visible = props.root.visible = true; }
  warpTo(lay, atlas, MODE.color);
}

// Expected depth (r, m along the ray), coverage (g) and, on actor pixels, the transmittance of the splats in front of
// the actor (b): Float32Array RGBA, GL row order. Actors are opaque, so with them the depth is the nearest of actor
// and splats, alpha-weighted. Returns the readStart collector.
function passFloat(lay, cam, withActors, withProps) {
  spark.material = depthMat;
  spark.visible = true;
  actors.root.visible = withActors;
  props.root.visible = withProps;
  scene.overrideMaterial = actorDepthMat; // props are opaque like actors (b = 1, then attenuated by splats in front)
  const atlas = atlasOf(lay, "float");
  try { drawViews(lay, cam, atlas); } finally { scene.overrideMaterial = null; spark.material = colorMat; actors.root.visible = props.root.visible = true; }
  const out = target("floatOut", lay.width, lay.height, { type: THREE.FloatType, depthBuffer: false, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
  warpTo(lay, atlas, MODE.depth, out);
  return readStart(out, 0, 0, lay.width, lay.height, new Float32Array(lay.width * lay.height * 4));
}

// Actor passes without splats into the id atlas, read back as RGBA bytes (GL row order): kind "id" draws every actor
// depth-tested (r = index of the nearest; props draw as 0 and hide what is behind them), "sil" draws only the actors of
// group, whole, one per channel. Reusing the targets before collecting is fine: the copies run in command order.
function passActors(lay, cam, kind, group, withProps = true) {
  spark.visible = false;
  actors.use(kind, group);
  props.use(withProps ? kind : "sil");
  const atlas = atlasOf(lay, "id");
  try { drawViews(lay, cam, atlas); } finally { spark.visible = true; actors.use("shade"); props.use("shade"); }
  const out = target("idOut", lay.width, lay.height, { depthBuffer: false, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
  warpTo(lay, atlas, MODE.id, out);
  return readStart(out, 0, 0, lay.width, lay.height, new Uint8Array(lay.width * lay.height * 4));
}

async function readCanvas(w, h) {
  const buf = await readback(null, 0, 0, w, h, new Uint8Array(w * h * 4)), data = new Uint8ClampedArray(w * h * 4);
  for (let y = 0; y < h; y++) data.set(buf.subarray((h - 1 - y) * w * 4, (h - y) * w * 4), y * w * 4);
  return data;
}

async function collected(...collectors) {
  await gpuDone();
  const out = collectors.map((c) => c());
  return out.length > 1 ? out : out[0];
}

function timed(t0) {
  const ms = now() - t0, t = now();
  stats.renders++;
  stats.renderMs = +ms.toFixed(1);
  stats.renderMsAvg = +(stats.renderMsAvg ? stats.renderMsAvg * 0.9 + ms * 0.1 : ms).toFixed(1);
  recent.push(t);
  while (recent.length && recent[0] < t - 1000) recent.shift();
  stats.fps = recent.length > 1 ? +(((recent.length - 1) * 1000) / (t - recent[0])).toFixed(1) : 0;
}

const ops = {
  init,
  setLens: (l) => ({ ...(lens = checkedLens(l, lens)) }),
  setActors: (list) => (actors.set(list), list.length),
  setProps: (list) => props.set(list),
  setOptions: (o) => (Object.assign(opts, o), { ...opts }),

  async render({ pose, view = {} }) {
    const t0 = now(), lay = getLayout(view), cam = await place(pose, lay);
    passColor(lay, cam, view.actors !== false, view.props !== false);
    stats.views = lay.views.length;
    stats.atlas = `${lay.atlasW}x${lay.atlasH}`;
    if (view.sync !== false) await gpuDone(); // honest timing and backpressure; sync: false pipelines
    const bitmap = canvas.transferToImageBitmap();
    timed(t0);
    return bitmap;
  },

  async pixels({ pose, view = {} }) {
    const t0 = now(), lay = getLayout(view), cam = await place(pose, lay);
    passColor(lay, cam, view.actors !== false, view.props !== false);
    const data = await readCanvas(lay.width, lay.height);
    timed(t0);
    return { width: lay.width, height: lay.height, data };
  },

  // pixels() and depth() at one pose from one placement (one sort check) and one GPU wait: { pixels, depth }.
  async pair({ pose, view = {}, depthView = view }) {
    const t0 = now(), lay = getLayout(view), cam = await place(pose, lay), { width: w, height: h } = lay;
    passColor(lay, cam, view.actors !== false, view.props !== false);
    const color = readStart(null, 0, 0, w, h, new Uint8Array(w * h * 4)), dl = depthView === view ? lay : getLayout(depthView);
    const [rgba, buf] = await collected(color, passFloat(dl, dl === lay ? cam : await place(pose, dl), depthView.actors !== false, depthView.props !== false));
    const data = new Uint8ClampedArray(w * h * 4);
    for (let y = 0; y < h; y++) data.set(rgba.subarray((h - 1 - y) * w * 4, (h - y) * w * 4), y * w * 4);
    timed(t0);
    return { pixels: { width: w, height: h, data }, depth: depthOf(buf, dl, depthView) };
  },

  // Expected distance along each pixel's ray (m, H), rows top-down, actors and props included unless view.actors or
  // view.props is false; NaN where the splats cover less than view.minCoverage (0.05) of the pixel.
  async depth({ pose, view = {} }) {
    const lay = getLayout(view), cam = await place(pose, lay);
    return depthOf(await collected(passFloat(lay, cam, view.actors !== false, view.props !== false)), lay, view);
  },

  // depth() for many poses in one call (map building): one sort per position, every readback collected at the end.
  // -> [Float32Array per pose]
  async depthBatch({ poses, view = {} }) {
    const lay = getLayout(view), out = [];
    for (let k = 0; k < poses.length; k += 12) {
      const pending = [];
      for (const pose of poses.slice(k, k + 12)) pending.push(passFloat(lay, await place(pose, lay), view.actors !== false, view.props !== false));
      await gpuDone();
      for (const c of pending) out.push(depthOf(c(), lay, view));
    }
    stats.renders += poses.length;
    return out;
  },

  // Ground truth per actor in view: box of its whole silhouette (pixels, ignoring anything in front; normalised
  // x, y, w, h from top left), and box and count of its visible pixels: in front of every other actor, and the splats
  // in front let through more than view.visibleT = 10% of its light (what still shows in the image).
  // visibleFraction = visiblePixels / pixels; an actor in view but hidden reports 0.
  async oracle({ pose, view = {} }) {
    const lay = getLayout(view), cam = await place(pose, lay), { width: w, height: h } = lay, list = actors.list(), minT = view.visibleT ?? 0.1;
    if (!list.length) return [];
    const groups = [], withProps = view.props !== false;
    for (let g = 0; g < list.length; g += 4) groups.push(passActors(lay, cam, "sil", list.slice(g, g + 4)));
    const [ids, T, ...sil] = await collected(passActors(lay, cam, "id", undefined, withProps), passFloat(lay, cam, true, withProps), ...groups);
    const acc = list.map(() => ({ n: 0, v: 0, b: [w, h, -1, -1], vb: [w, h, -1, -1] }));
    const grow = (b, x, y) => { if (x < b[0]) b[0] = x; if (y < b[1]) b[1] = y; if (x > b[2]) b[2] = x; if (y > b[3]) b[3] = y; };
    for (let r = 0; r < h; r++) for (let x = 0; x < w; x++) {
      const i = (r * w + x) * 4, k = ids[i], y = h - 1 - r;
      for (let g = 0; g < sil.length; g++) for (let c = 0; c < 4; c++) if (sil[g][i + c]) { const a = acc[g * 4 + c]; a.n++; grow(a.b, x, y); }
      if (k && k <= list.length && T[i + 2] > minT) { const a = acc[k - 1]; a.v++; grow(a.vb, x, y); }
    }
    const box = (b) => (b[2] < 0 ? null : { x: b[0] / w, y: b[1] / h, w: (b[2] + 1 - b[0]) / w, h: (b[3] + 1 - b[1]) / h });
    return list.flatMap((it, i) => (acc[i].n ? [{ id: it.spec.id, kind: it.spec.kind, box: box(acc[i].b), visibleBox: box(acc[i].vb),
      visibleFraction: +(acc[i].v / acc[i].n).toFixed(4), pixels: acc[i].n, visiblePixels: acc[i].v }] : []));
  },

  dispose() {
    for (const t of Object.values(targets)) t.dispose();
    for (const l of layouts.values()) l.lut.dispose();
    spark?.dispose();
    renderer?.dispose();
    renderer?.forceContextLoss();
    return true;
  },
};

function depthOf(buf, { width: w, height: h }, view) {
  const d = new Float32Array(w * h), min = view.minCoverage ?? 0.05;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const i = ((h - 1 - y) * w + x) * 4;
    d[y * w + x] = buf[i + 1] > min ? buf[i] : NaN;
  }
  return d;
}

// One job at a time, the most urgent first (queue.js).
const waiting = [];
let running = false;
async function pump() {
  if (running) return;
  running = true;
  for (let job; (job = nextJob(waiting, now())); ) {
    const { id, op, args } = job;
    try {
      const result = await ops[op](args);
      stats.queue = waiting.length;
      self.postMessage({ id, result, stats }, transferOf(result));
    } catch (e) {
      self.postMessage({ id, error: String(e?.stack || e) });
    }
  }
  running = false;
}
self.onmessage = ({ data }) => {
  waiting.push({ ...data, at: now() });
  pump();
};
