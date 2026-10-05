// Browser check for live depth (app/js/vision/depth.js) and change detection (app/js/nav/changes.js) on a real capture's
// splat twin: the drone camera is the twin's render (with an obstacle prop that isn't in the scan), the app's depth worker
// turns it into relative depth, alignDepth() (in the worker) makes it metric against the twin's empty-house depth at the pose, and the
// change detector watches a patrol: without the obstacle (people about: one standing whom the detector misses on a third of
// the frames and boxes 15% too small on the rest, one sitting whom it never sees) it must report nothing but the sitting
// person (something new to the scan), with it the obstacle at its place. &light=shadow|lamp|night: the house lit unlike
// the capture. The goggles part runs the app's LiveDepth itself on the Goggles 3's 1920x1080 stream with the O4's 4:3
// picture pillarboxed in it (OSD text in the bars). Results land in window.result; with &save= the pictures and results
// are POSTed there. The house's 3D map is built first (as the app does: splat centres, capture rays and the twin's depth;
// &vox=0 skips it, &cache=1 keeps it in OPFS), so the detector's scan checks run as in flight. Measured: false changes per
// minute on the quiet patrol (target 0; with &augment=1 the O4's video), the placed box's report (within 0.6 m), what that
// report blocks (at least 95% of the box's footprint: on the detector's own map obstacle, as the flight memory would
// put it on the map, and as the house's keep-out once the user confirms it on the real drone), its evidence (the
// outline off the picture's edge, or said to be cut), and a closed door on the patrol's way (door-closed reported, or a
// door leaf on the map from avoid). &zErr=m: the localized height that far off the whole way (the expected view rendered
// there, as in flight with a sure but wrong height: nothing may be reported for it, the box and the door still are);
// &zSigma=m: the height σ the frames say (default none: the detector's own floor).
import { Twin } from "../app/js/twin/twin.js";
import { importCapture } from "../app/js/house/import.js";
import { houseSource } from "../app/js/ui/housepanel.js";
import { patrolRoute } from "../app/js/house/planner.js";
import { DepthEstimator, LiveDepth, viewMask, pictureProblem, DEPTH } from "../app/js/vision/depth.js";
import { OsdMask, grayDown } from "../app/js/nav/osdmask.js";
import { rectLens, droneLens, CAM_DZ } from "../app/js/nav/lens.js";
import * as CHANGE_MODULE from "../app/js/nav/changes.js";
import { Avoid, AVOID } from "../app/js/nav/avoid.js";
import { Augmenter, augmentOptions, simLens, LENS_ERROR } from "../app/js/sim/augment.js";
import { buildVoxels, voxelCentres } from "../app/js/house/voxels.js";
import { houseFrame } from "../app/js/house/frames.js";
import { openStore } from "../app/js/house/store.js";
import { HouseMemory } from "../app/js/memory/memory.js";

const $ = (id) => document.getElementById(id);
const q = new URLSearchParams(location.search), only = q.get("only")?.split(","), save = q.get("save"), dump = q.get("dump");
// &changes=url: the change detector from another module (an earlier version, for a before/after on the same frames)
const { ChangeDetector } = q.get("changes") ? await import(q.get("changes")) : CHANGE_MODULE;
// &poseErr=m: the localized pose wanders this far (smoothly, and 30x that in degrees of yaw) from the camera's; the twin's
// expected view is rendered where the app thinks it is, as in flight.
const poseErr = +(q.get("poseErr") ?? 0), zErr = +(q.get("zErr") ?? 0), zSigma = q.get("zSigma") == null ? null : +q.get("zSigma");
// &augment=1: the O4's blur, exposure, noise, compression, OSD and a lens 3% wider with its uptilt 2° off what the app
// assumes (sim/augment.js); &augment=nolens: all of that but the lens error.
const aug = q.get("augment"), augmenter = aug && new Augmenter(augmentOptions(aug === "nolens" ? { lensError: null } : true)), osd = (window.osd = new OsdMask());
// &light=: the house lit unlike the capture: shadow (the left 45% of the view at half brightness), lamp (a warm pool of
// light, darker around it), night (35% as bright, warm)
const light = q.get("light");
async function relight(bmp) {
  if (!light) return bmp;
  const c = new OffscreenCanvas(bmp.width, bmp.height), g = c.getContext("2d"), [w, h] = [bmp.width, bmp.height];
  g.drawImage(bmp, 0, 0);
  bmp.close();
  g.globalCompositeOperation = "multiply";
  if (light === "shadow") (g.fillStyle = "rgb(128,128,128)"), g.fillRect(0, 0, 0.45 * w, h);
  else if (light === "lamp") {
    const r = g.createRadialGradient(0.7 * w, 0.35 * h, 0, 0.7 * w, 0.35 * h, 0.8 * w);
    r.addColorStop(0, "rgb(255,232,190)");
    r.addColorStop(1, "rgb(105,92,80)");
    g.fillStyle = r;
    g.fillRect(0, 0, w, h);
  } else if (light === "night") (g.fillStyle = "rgb(96,84,70)"), g.fillRect(0, 0, w, h);
  return createImageBitmap(c);
}
const lines = [], result = (window.result = { done: false, checks: {} });
const log = (...a) => (lines.push(a.map((x) => (typeof x === "string" ? x : JSON.stringify(x))).join(" ")), ($("log").textContent = lines.join("\n")));
const med = (v) => [...v].sort((a, b) => a - b)[v.length >> 1], p95 = (v) => [...v].sort((a, b) => a - b)[Math.floor(v.length * 0.95)];
const r2 = (v) => +(+v).toFixed(2), r3 = (v) => +(+v).toFixed(3);
function check(name, ok, detail) {
  result.checks[name] = { ok, detail };
  const tr = $("checks").insertRow();
  tr.insertCell().textContent = ok ? "PASS" : "FAIL";
  tr.cells[0].className = ok ? "pass" : "fail";
  tr.insertCell().textContent = name;
  tr.insertCell().textContent = detail;
}
async function part(name, fn) {
  if (only && !only.includes(name)) return;
  try { await fn(); } catch (e) { log(`ERROR in ${name}:`, String(e?.stack || e)); check(`${name}: runs`, false, String(e?.message || e)); }
}
const upload = (name, body) => save && fetch(`${save}/save?name=${name}`, { method: "POST", body }).catch((e) => log("save failed", e.message));
const shotPng = (id) => new Promise((r) => $(id).toBlob((b) => r(upload(`${id}.png`, b)), "image/png"));

// Depth (m) as a picture: near warm, far cool, NaN black.
function drawDepth(id, d, w, h, max = 6) {
  const c = $(id), g = c.getContext("2d"), img = g.createImageData(w, h);
  [c.width, c.height] = [w, h];
  for (let i = 0; i < w * h; i++) {
    const v = d[i], k = Number.isFinite(v) ? Math.max(0, Math.min(1, v / max)) : -1, o = 4 * i;
    [img.data[o], img.data[o + 1], img.data[o + 2], img.data[o + 3]] = k < 0 ? [0, 0, 0, 255] : [255 * (1 - k), 255 * (1 - Math.abs(k - 0.5) * 2) * 0.8, 255 * k, 255];
  }
  g.putImageData(img, 0, 0);
}
function drawRgba(id, px) {
  const c = $(id);
  [c.width, c.height] = [px.width, px.height];
  c.getContext("2d").putImageData(new ImageData(new Uint8ClampedArray(px.data), px.width, px.height), 0, 0);
}

let house, map, twin, est, route = [];
const W = DEPTH.width, H = DEPTH.height, LENS = rectLens(20), view = { width: W, height: H, lens: LENS };

// One drone camera frame at a pose: the twin's picture through the app's depth worker, aligned to the empty-house depth
// at `at` (where the app thinks the camera is). truth: the twin's depth of what the camera sees (actors and props).
async function frame(pose, t, at = pose, { miss = null } = {}) {
  let bmp = await relight(await twin.render(pose, { width: 640, height: 480 })), live = $("live");
  if (augmenter) {
    const raw = bmp;
    bmp = await augmenter.apply(raw, { yawRate: pose.yawRate ?? 0, pitchRate: 0, seconds: t / 1000 });
    raw.close();
    const g = new OffscreenCanvas(640, 480).getContext("2d", { willReadFrequently: true });
    g.drawImage(bmp, 0, 0);
    osd.add(grayDown(g.getImageData(0, 0, 640, 480).data, 640, 480)); // the app learns the goggles' OSD the same way
  }
  live.getContext("2d").drawImage(bmp, 0, 0);
  const small = await createImageBitmap(bmp, { resizeWidth: DEPTH.src[0], resizeHeight: DEPTH.src[1], resizeQuality: "medium" });
  bmp.close();
  const [out, expected, expectedRgb, truth, gt] = await Promise.all([est.run(small, t), twin.depth(at, { ...view, actors: false, props: false }),
    twin.pixels(at, { ...view, actors: false, props: false }), twin.depth(pose, view), twin.oracle(pose, view)]);
  // the detector: what the twin's oracle sees, but none on a third of the frames and 15% too small on the rest, and never
  // the ones in `miss` (ids)
  const shrink = (b) => ({ x: b.x + 0.075 * b.w, y: b.y + 0.075 * b.h, w: 0.85 * b.w, h: 0.85 * b.h });
  const boxes = miss && Math.round(t / 250) % 3 === 0 ? [] : gt.filter((o) => o.visibleBox && !miss?.includes(o.id)).map((o) => (miss ? shrink(o.visibleBox) : o.visibleBox));
  const mask = osd.ready && osd.coverage() <= DEPTH.osdMax ? viewMask(osd.mask(), osd.width, osd.height, droneLens(null, 20)) : undefined;
  result.osd = osd.ready ? r2(osd.coverage()) : null;
  const { a, grid, ms: alignMs, gridMs } = await est.align(t, expected, LENS, { expectedRgb, boxes, mask }); // in the depth worker, as in the app
  const rel = [];
  if (a) for (let i = 0; i < W * H; i++) if (truth[i] > 0.2 && truth[i] < 8 && a.depth[i] > 0) rel.push(Math.abs(a.depth[i] - truth[i]) / truth[i]);
  return { out, expected, expectedRgb, truth, a, grid, alignMs, gridMs, absRel: rel.length ? med(rel) : NaN, boxes, mask };
}

// The patrol: home -> one scan point per room -> home, sampled every 0.075 m (0.3 m/s at 4 Hz) facing along the way, with
// a 360° turn (4 s) at each room's point.
function patrolPoses(seconds) {
  const p = patrolRoute(map, house.rooms.map((r) => r.id), [house.home.x, house.home.y], { alt: 1.0, sigma: 0.17 }), out = [];
  for (const leg of p.legs.filter((l) => l.ok)) {
    const pts = leg.path;
    let carry = 0;
    for (let k = 1; k < pts.length; k++) {
      const [a, b] = [pts[k - 1], pts[k]], L = Math.hypot(b[0] - a[0], b[1] - a[1]), yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
      for (let s = carry; s < L; s += 0.075) out.push({ x: a[0] + ((b[0] - a[0]) * s) / L, y: a[1] + ((b[1] - a[1]) * s) / L, z: a[2] + CAM_DZ, yaw, pitch: 0, roll: 0 });
      carry = ((carry - L) % 0.075 + 0.075) % 0.075;
    }
    const end = out.at(-1);
    for (let i = 1; i <= 16; i++) out.push({ ...end, yaw: end.yaw + (2 * Math.PI * i) / 16 });
  }
  // shorter than asked: back the way it came, and round again
  const lap = out.slice();
  for (let k = 1; out.length < seconds * DEPTH.hz; k++) out.push(...(k % 2 ? [...lap].reverse().map((p) => ({ ...p, yaw: p.yaw + Math.PI })) : lap));
  // smooth the heading over 0.5 m so corners turn gradually
  const yaws = out.map((p, i) => { const b = out[Math.min(out.length - 1, i + 6)]; return Math.hypot(b.x - p.x, b.y - p.y) > 0.2 ? Math.atan2(b.y - p.y, b.x - p.x) : p.yaw; });
  out.forEach((p, i) => (p.yaw = yaws[i]));
  out.forEach((p, i) => (p.yawRate = i ? (p.yaw - out[i - 1].yaw) * DEPTH.hz : 0));
  return out.slice(0, seconds * DEPTH.hz);
}

// Where the app thinks pose i is: a smooth wander of about poseErr m (and 30x that in degrees of yaw), the same every run,
// and zErr m too high.
function localized(p, i) {
  if (zErr) p = { ...p, z: p.z + zErr };
  if (!poseErr) return p;
  const w = (k, f) => Math.sin(i * f + k) * 0.6 + Math.sin(i * f * 2.7 + 2 * k) * 0.4;
  return { ...p, x: p.x + poseErr * w(1, 0.031), y: p.y + poseErr * w(2, 0.023), z: p.z + 0.5 * poseErr * w(3, 0.041), yaw: p.yaw + poseErr * 0.52 * w(4, 0.027) };
}

// &dump=address: every patrol frame (the model's output, the live and expected pictures and depths) is POSTed there as
// one binary record (a JSON header line, then the arrays as float16 / uint8) for replaying the detector offline.
function dumpFrame(run, i, pose, at, f) {
  if (!dump) return;
  const h16 = (a) => new Uint8Array(Float16Array.from(a).buffer), arrays = { disp: h16(f.out.disp), expected: h16(f.expected), truth: h16(f.truth), rgba: f.out.rgba, expectedRgb: f.expectedRgb.data, ...(f.mask && { mask: f.mask }) };
  const head = JSON.stringify({ i, pose, at, boxes: f.boxes, width: W, height: H, lens: LENS, sizes: Object.fromEntries(Object.entries(arrays).map(([k, a]) => [k, a.byteLength])) }) + "\n";
  return fetch(`${dump}/dump?run=${run}&i=${i}`, { method: "POST", body: new Blob([head, ...Object.values(arrays)]) }).catch((e) => log("dump failed", e.message));
}

// The plan view: rooms, the patrol, the obstacle and what the detector reported.
function drawPlan(poses, box, changes) {
  const c = $("plan"), g = c.getContext("2d"), xs = map.rooms.flatMap((r) => r.outline.map((p) => p[0])), ys = map.rooms.flatMap((r) => r.outline.map((p) => p[1]));
  const x0 = Math.min(...xs) - 0.3, y1 = Math.max(...ys) + 0.3, s = Math.min(c.width / (Math.max(...xs) - x0 + 0.3), c.height / (y1 - Math.min(...ys) + 0.3));
  const P = (x, y) => [(x - x0) * s, (y1 - y) * s];
  g.fillStyle = "#151515";
  g.fillRect(0, 0, c.width, c.height);
  g.strokeStyle = "#667";
  for (const r of map.rooms) {
    g.beginPath();
    r.outline.forEach((p, i) => g[i ? "lineTo" : "moveTo"](...P(...p)));
    g.closePath();
    g.stroke();
  }
  g.strokeStyle = "#3a8";
  g.beginPath();
  poses.forEach((p, i) => g[i ? "lineTo" : "moveTo"](...P(p.x, p.y)));
  g.stroke();
  if (box) {
    g.fillStyle = "#eee";
    const [a, b] = [P(box.x - box.w / 2, box.y + box.d / 2), P(box.x + box.w / 2, box.y - box.d / 2)];
    g.fillRect(a[0], a[1], b[0] - a[0], b[1] - a[1]);
  }
  for (const ch of changes) {
    g.strokeStyle = ch.kind === "obstacle" ? "#f44" : "#fa0";
    g.beginPath();
    g.arc(...P(ch.x, ch.y), Math.max(4, (ch.size / 2) * s), 0, 2 * Math.PI);
    g.stroke();
    g.fillStyle = g.strokeStyle;
    g.fillText(ch.kind, P(ch.x, ch.y)[0] + 6, P(ch.x, ch.y)[1] - 6);
  }
}

// A patrol through the change detector, and nav/avoid.js on the same frames (what it would put on the map as unmapped
// obstacles, and how often what the camera sees ahead would slow the drone). -> { changes, frames, ms, absRel, best
// frame, avoid: { temps, ahead (frames with the way-ahead cue on) } }
async function patrol(poses, label, { miss = null } = {}) {
  const det = new ChangeDetector({ map, house, render: (pose, v) => twin.pixels(pose, v) }), changes = [], abs = [], model = [], align = [];
  let P = null, ahead = 0;
  const main = [], detMs = [], grid = []; // ms per frame on the page's thread (the change detector and avoid: the control loop shares it; the detector's part), in the worker (their grid)
  const av = new Avoid({ map, localizer: { ctl: { videoDelay: 0, est: { ttc: Infinity } }, pose: () => P, velocity: () => [0, 0] } }), temps = [];
  av.on("temp", (t) => t.added && temps.push({ source: t.source, x: r2(t.x), y: r2(t.y), r: r2(t.r) }));
  let wedges = 0;
  av.on("temp", (t) => t.added && t.source === "way ahead" && wedges++);
  // every report's pictures go to &save= too (what a false one looked like): <patrol>-<n>-<kind>-live.jpg / -expected.jpg
  det.on("change", (c) => (changes.push(c), save && ["live", "expected"].forEach((k) => c.evidence?.[k] && upload(`${label.replace(/\W+/g, "-")}-${changes.length}-${c.kind}-${k}.jpg`, c.evidence[k]))));
  window.dets = [...(window.dets ?? []), det]; // for a look from the console
  let best = null;
  for (const [i, pose] of poses.entries()) {
    const t = 1e6 + i * (1000 / DEPTH.hz), at = localized(pose, i), f = await frame(pose, t, at, { miss });
    await dumpFrame(label.replace(/\W+/g, "-"), i, pose, at, f);
    if (!f.a) continue;
    abs.push(f.absRel);
    model.push(f.out.ms.run);
    align.push(f.alignMs);
    grid.push(f.gridMs);
    const fr = { t, pose: (P = { ...at, sigma: Math.max(0.03, poseErr), status: "ok", ...(zSigma != null && { zSigma }) }), width: W, height: H, lens: LENS, depth: f.a.depth, conf: f.a.conf, expected: f.expected, grid: f.grid,
      expectedRgb: f.expectedRgb, boxes: f.boxes, mask: f.mask, rgb: { width: W, height: H, data: f.out.rgba }, hires: f.out.hi && { width: W * DEPTH.hires, height: H * DEPTH.hires, data: f.out.hi } };
    const t0 = performance.now(), blobs = det.ingest(fr, t);
    detMs.push(performance.now() - t0);
    av.ingest(fr);
    main.push(performance.now() - t0);
    ahead += av.ahead?.slow && av.ahead.t === t ? 1 : 0;
    const near = blobs.filter((b) => b.near).reduce((s, b) => s + b.cells, 0);
    if (!best || near > best.near) best = { near, i, f };
    if (i % 20 === 0) log(`${label}: frame ${i}/${poses.length}, model ${r2(f.out.ms.run)} ms, absRel ${r3(f.absRel)}, ${changes.length} change(s)`);
  }
  // reports finish their evidence asynchronously (the pictures, the twin's render): wait until each is out (no memory here:
  // one change event per report of its own), at most 10 s (a busy machine took longer than a fixed 50 ms)
  for (const t0 = performance.now(); det.reported.length > changes.length && performance.now() - t0 < 10000; ) await new Promise((r) => setTimeout(r, 50));
  av.setMap(map); // its temporary obstacles off the map
  return { det, changes, frames: abs.length, absRel: med(abs), absRel90: [...abs].sort((a, b) => a - b)[Math.floor(abs.length * 0.9)], modelMs: med(model), alignMs: med(align), best,
    avoid: { temps, ahead, wedges }, mainMs: { median: r2(med(main)), p95: r2(p95(main)), max: r2(Math.max(...main)), detectorP95: r2(p95(detMs)) }, gridMs: r2(med(grid)) };
}

function show(f) {
  drawRgba("rect", { width: W, height: H, data: f.out.rgba });
  const d = f.out.disp, lo = Math.min(...d.filter(Number.isFinite)), hi = Math.max(...d.filter(Number.isFinite));
  drawDepth("disp", d.map((v) => (Number.isFinite(v) ? 6 * (1 - (v - lo) / (hi - lo)) : NaN)), W, H);
  drawDepth("aligned", f.a.depth, W, H);
  drawDepth("expected", f.expected, W, H);
  const c = $("diff"), g = c.getContext("2d"), img = g.createImageData(W, H);
  for (let i = 0; i < W * H; i++) {
    const a = f.a.depth[i], e = f.expected[i], o = 4 * i, rel = a > 0 && e > 0 ? (a - e) / e : 0;
    [img.data[o], img.data[o + 1], img.data[o + 2], img.data[o + 3]] = [rel < -0.2 ? 230 : 40, 40, rel > 0.2 ? 230 : 40, 255];
  }
  g.putImageData(img, 0, 0);
}

try {
  await part("load", async () => {
    let project = q.get("project");
    if (!project) {
      const list = await (await fetch("/house-files/")).json();
      project = (list.projects ?? list).find((p) => p.ready ?? true)?.id;
    }
    if (!project) throw new Error("no capture under /house-files/");
    const t0 = performance.now();
    const imp = await importCapture(houseSource({ id: project }));
    ({ house, map } = imp);
    const t1 = performance.now();
    twin = window.twin = await Twin.create({ splat: `/house-files/${encodeURIComponent(project)}/outputs/${house.source?.files?.splat?.split("/").pop() ?? "splat.spz"}`, house });
    if (q.get("vox") !== "0") { // the 3D map, as the app builds it: the change detector's scan checks need it
      const t3 = performance.now(), store = q.get("cache") ? await openStore().catch(() => null) : null;
      let vox = store && (await store.loadVoxels(house).catch(() => null));
      if (vox) await vox.refresh();
      else {
        const bytes = imp.splat?.bytes ?? (await (await fetch(`/house-files/${encodeURIComponent(project)}/outputs/${house.source?.files?.splat?.split("/").pop() ?? "splat.spz"}`)).arrayBuffer());
        const centres = await voxelCentres(new Uint8Array(bytes).slice().buffer, imp.splat?.name ?? house.splatFile ?? "splat.spz", houseFrame({ f: house.frame.f, Yf: house.frame.Yf }));
        vox = await buildVoxels({ house, map, centres, twin, onProgress: (p) => p.text && ($("usage").dataset.progress = p.text) });
        await store?.saveVoxels(house, vox).catch(() => {});
      }
      map.setVoxels(vox);
      result.vox = { ms: Math.round(performance.now() - t3), ...vox.stats(), built: undefined };
      log("3D map", result.vox);
    }
    if (aug && aug !== "nolens") await twin.setLens(simLens(127, 20, LENS_ERROR)); // the camera's real lens; the app keeps assuming the default
    Object.assign(result, { augment: aug ?? "off", poseErr, zErr, zSigma });
    result.load = { rooms: house.rooms.length, importMs: Math.round(t1 - t0), twinMs: Math.round(performance.now() - t1), splats: twin.info.splats, gpu: twin.info.gpu };
    log("house", result.load);
    est = new DepthEstimator({ model: q.get("model") ?? DEPTH.model });
    const t2 = performance.now(), info = await est.load(droneLens(null, 20), (l, n) => l === n && log(`model downloaded (${(n / 1e6).toFixed(0)} MB)`));
    result.model = { ...info, ms: Math.round(performance.now() - t2) };
    log("model", result.model);
    check("depth model loads on WebGPU", true, `${info.name}, ${info.cached ? "cached" : "downloaded"}, ${Math.round(info.loadMs)} ms; input ${info.input}, output ${info.output}`);
  });

  const seconds = +(q.get("seconds") ?? 60);
  const poses = patrolPoses(seconds);
  route = poses;
  await part("align", async () => {
    const out = [];
    for (const p of poses.filter((_, i) => i % 12 === 0)) out.push(await frame(p, performance.now()));
    const abs = out.filter((f) => f.a).map((f) => f.absRel), ms = out.map((f) => f.out.ms), inl = out.filter((f) => f.a).map((f) => f.a.inliers);
    result.align = { frames: out.length, absRel: r3(med(abs)), absRel90: r3([...abs].sort((a, b) => a - b)[Math.floor(abs.length * 0.9)]), inliers: r2(med(inl)),
      modelMs: r2(med(ms.map((m) => m.run))), workerMs: r2(med(ms.map((m) => m.total))), alignMs: r2(med(out.map((f) => f.alignMs))), gridMs: r2(med(out.map((f) => f.gridMs))) };
    log("align", result.align);
    show(out[Math.floor(out.length / 2)]);
    check("depth model under 100 ms a frame (3-5 Hz with the rest)", result.align.workerMs < 100, `model ${result.align.modelMs} ms, worker total ${result.align.workerMs} ms, then alignment ${result.align.alignMs} ms and the change grid ${result.align.gridMs} ms in the worker too (median over ${out.length} frames)`);
    check("aligned depth within 15% of the twin's (median abs rel)", result.align.absRel < 0.15, `median abs rel ${result.align.absRel} (90%: ${result.align.absRel90}), ${Math.round(result.align.inliers * 100)}% of pixels agree`);
  });

  await part("change", async () => {
    // People about (masked by their boxes), no scene change: nothing may be reported.
    const room = map.roomAt(house.home.x, house.home.y)?.id, spot = poses.find((p, i) => i > 40 && map.roomAt(p.x, p.y)?.id !== room) ?? poses[60];
    // a seated person the detector never sees: by a wall of a room later on the patrol, 1.4 m beside the route
    const later = poses.find((p, i) => i > poses.length / 3 && map.roomAt(p.x, p.y)?.id !== map.roomAt(spot.x, spot.y)?.id && map.clearance(p.x + 1.4 * Math.cos(p.yaw - Math.PI / 2), p.y + 1.4 * Math.sin(p.yaw - Math.PI / 2), (map.floorAt(p.x, p.y) ?? 0) + 0.6) > 0.3) ?? poses[Math.floor(poses.length / 2)];
    const sit = { id: "p2", kind: "person", x: later.x + 1.4 * Math.cos(later.yaw - Math.PI / 2), y: later.y + 1.4 * Math.sin(later.yaw - Math.PI / 2), z: map.floorAt(later.x, later.y) ?? 0, yaw: later.yaw + Math.PI / 2, pose: "sit" };
    const stand = { id: "p1", kind: "person", x: spot.x + 1.2 * Math.cos(spot.yaw + 0.4), y: spot.y + 1.2 * Math.sin(spot.yaw + 0.4), z: map.floorAt(spot.x, spot.y) ?? 0, yaw: 0, pose: "stand" };
    // 1. someone standing about, the detector boxing them on every frame: nothing may be reported, nothing slow the way
    // (&nobody=1: nobody about, the house alone)
    await twin.setActors(q.get("nobody") === "1" ? [] : [stand]);
    await twin.setProps?.([]);
    const quiet = q.get("people") === "only" ? null : await patrol(poses, "no change");
    if (quiet) {
    const minutes = quiet.frames / DEPTH.hz / 60, perMin = r2(quiet.changes.length / minutes);
    result.quiet = { frames: quiet.frames, perMinute: perMin, changes: quiet.changes.map((c) => ({ kind: c.kind, x: r2(c.x), y: r2(c.y), z: r2(c.z), size: c.size, weak: !!c.weak })), stats: quiet.det.stats, absRel: r3(quiet.absRel), modelMs: r2(quiet.modelMs), avoid: quiet.avoid, mainMs: quiet.mainMs, gridMs: quiet.gridMs, light: light ?? "as captured", augment: aug ?? "off", vox: !!map.vox };
    log("no change", result.quiet);
    check(`no false changes over ${seconds} s of patrol (${q.get("nobody") === "1" ? "nobody about" : "someone standing about"}; ${light ? `${light} light` : "the capture's light"}; ${aug ? "the O4's video" : "clean video"}; ${map.vox ? "with" : "without"} the 3D map${zErr ? `; the height ${zErr} m off` : ""})`, quiet.changes.length === 0,
      `${quiet.changes.length} change(s) in ${quiet.frames} frames = ${perMin} a minute${quiet.changes.length ? ` (${quiet.changes.map((c) => `${c.kind}${c.weak ? " weak" : ""} at (${r2(c.x)}, ${r2(c.y)}, ${r2(c.z)})`).join(", ")})` : ""} (${quiet.det.stats.blobs} passing blobs, ${quiet.det.stats.scan} the scan ruled out, ${quiet.det.stats.unagreed} that didn't agree in 3D, ${quiet.det.stats.poseErr ?? "-"} cells the pose's error explains)`);
    check("change detector and avoid together under 10 ms of the page's thread a frame (p95)", quiet.mainMs.p95 < 10, `median ${quiet.mainMs.median} ms, p95 ${quiet.mainMs.p95} ms (the detector's ${quiet.mainMs.detectorP95} ms), max ${quiet.mainMs.max} ms per frame (4 Hz); their grid ${quiet.gridMs} ms in the depth worker`);
    const lv = quiet.avoid.temps.filter((t) => t.source === "live depth").length;
    check("nothing changed: live depth puts at most 2 unmapped obstacles on the map, the way-ahead cue slows on under 5% of frames and places no wedge", lv <= 2 && quiet.avoid.ahead < 0.05 * quiet.frames && !quiet.avoid.wedges,
      `${lv} unmapped obstacle(s) from live depth${lv ? ` (${quiet.avoid.temps.filter((t) => t.source === "live depth").map((t) => `(${t.x}, ${t.y}) r ${t.r}`).join(", ")})` : ""}, way ahead on ${quiet.avoid.ahead} of ${quiet.frames} frames, ${quiet.avoid.wedges} wedge(s)`);
    }
    // 2. a real detector: it misses the one standing on a third of the frames (boxes 15% small on the rest) and never
    // sees the one sitting. They are new to the scan: anything reported must be at one of them, nothing elsewhere.
    if (q.get("people") !== "0") {
      await twin.setActors([stand, sit].filter((a) => !q.get("cast") || q.get("cast") === a.pose)); // &cast=stand|sit: one of them
      const busy = await patrol(poses, "people, detector missing them", { miss: ["p2"] }), by = (c) => [stand, sit].find((a) => Math.hypot(c.x - a.x, c.y - a.y) < 0.8)?.pose;
      const away = busy.changes.filter((c) => !by(c)), wedgesAway = busy.avoid.temps.filter((t) => t.source !== "person" && Math.min(...[stand, sit].map((a) => Math.hypot(t.x - a.x, t.y - a.y))) > 1.2);
      result.people = { stand: { x: r2(stand.x), y: r2(stand.y) }, sit: { x: r2(sit.x), y: r2(sit.y) }, changes: busy.changes.map((c) => ({ kind: c.kind, x: r2(c.x), y: r2(c.y), by: by(c) ?? null })), avoid: busy.avoid };
      log("people", result.people);
      check("people the detector misses: whatever is reported is at them (a person standing or sitting still is new to the scan), nothing elsewhere", !away.length && !wedgesAway.length,
        `${busy.changes.length} change(s): ${busy.changes.map((c) => `${c.kind} ${by(c) ? `at the one ${by(c) === "sit" ? "sitting" : "standing"}` : `at (${r2(c.x)}, ${r2(c.y)}), away from both`}`).join("; ") || "none"}; way ahead on ${busy.avoid.ahead} frames, ${busy.avoid.temps.length} temporary obstacle(s), ${wedgesAway.length} away from them`);
      if (q.get("people") === "only") return;
    }
    // A box 0.9 m beside the route in a room after the first, on the drone's left.
    await twin.setActors([]);
    const side = (p, s) => [p.x + 0.9 * Math.cos(p.yaw + s * Math.PI / 2), p.y + 0.9 * Math.sin(p.yaw + s * Math.PI / 2)];
    const offDoors = (q) => house.doors.every((d) => { const dx = d.b[0] - d.a[0], dy = d.b[1] - d.a[1], t = Math.max(0, Math.min(1, ((q[0] - d.a[0]) * dx + (q[1] - d.a[1]) * dy) / (dx * dx + dy * dy))); return Math.hypot(q[0] - d.a[0] - t * dx, q[1] - d.a[1] - t * dy) > 0.7; });
    const free = (p, s) => map.clearance(...side(p, s), (map.floorAt(p.x, p.y) ?? 0) + 0.6) > 0.3 && map.roomAt(...side(p, s))?.id === map.roomAt(p.x, p.y)?.id && offDoors(side(p, s));
    let k = poses.findIndex((p, i) => i > 30 && Math.abs(p.yaw - poses[Math.min(poses.length - 1, i + 8)].yaw) < 0.2 && free(p, 1)), s = 1;
    if (k < 0) [k, s] = [poses.findIndex((p, i) => i > 30 && free(p, -1)), -1];
    if (k < 0) throw new Error("no free spot beside the route for the box");
    const at = { ...poses[k], yaw: poses[k].yaw + (s < 0 ? Math.PI : 0) }, box = { id: "box", kind: "box", x: at.x + 0.9 * Math.cos(at.yaw + Math.PI / 2), y: at.y + 0.9 * Math.sin(at.yaw + Math.PI / 2), z: map.floorAt(at.x, at.y) ?? 0, w: 0.5, d: 0.5, h: 0.8, yaw: 0, color: "#8a6b4a" };
    if (!twin.setProps) throw new Error("this twin has no setProps (MAP3D): can't place an obstacle");
    await twin.setProps([box]);
    const run = await patrol(poses, "with a box");
    const hit = run.changes.find((c) => c.kind === "obstacle"), err = hit ? Math.hypot(hit.x - box.x, hit.y - box.y) : Infinity;
    result.change = { box: { x: r2(box.x), y: r2(box.y) }, changes: run.changes.map((c) => ({ kind: c.kind, x: r2(c.x), y: r2(c.y), z: r2(c.z), zMin: c.zMin, zMax: c.zMax, size: c.size, room: c.room, weak: !!c.weak, evidence: !!c.evidence?.live })),
      avoid: run.avoid,
      error: r2(err), stats: run.det.stats, absRel: r3(run.absRel), modelMs: r2(run.modelMs) };
    log("with a box", result.change);
    check("the box is reported as an obstacle within 0.6 m, once", err < 0.6 && run.changes.filter((c) => Math.hypot(c.x - box.x, c.y - box.y) < 1.5).length === 1,
      hit ? `${hit.kind}${hit.weak ? " (weak)" : ""} at (${r2(hit.x)}, ${r2(hit.y)}) vs the box at (${r2(box.x)}, ${r2(box.y)}): ${r2(err)} m; ${run.changes.length} change(s) in all, ${run.changes.length - 1} elsewhere` : "not reported");
    // what the report blocks: the detector's own obstacle on the map, and the one the flight memory puts there (tempOf)
    const inside = (tp) => {
      let n = 0, k = 0;
      for (let i = 0; i < 40; i++) for (let j = 0; j < 40; j++) n++, (k += !!tp && Math.hypot(box.x - box.w / 2 + ((i + 0.5) * box.w) / 40 - tp.x, box.y - box.d / 2 + ((j + 0.5) * box.d) / 40 - tp.y) <= tp.r);
      return r2(k / n);
    };
    // and once the user confirms it on the real drone: the house's keep-out (a real-world memory on a copy of the house)
    const own = hit && map.temps.get(hit.id), kept = hit && new HouseMemory("depth-check").tempOf({ ...hit, n: 1 });
    let ko = null;
    if (hit) {
      const real = await HouseMemory.open("depth-check-confirmed", { indexedDB: null }), h = { rooms: house.rooms, doors: house.doors, keepouts: [] };
      real.setHouse({ house: h });
      const c = real.addChange({ ...hit, id: null, evidence: null });
      real.resolveChange(c.id, "confirmed", "depth-check");
      ko = h.keepouts.find((k) => k.change === c.id) ?? null;
      real.setHouse({});
    }
    result.change.covers = { detector: inside(own), memory: inside(kept), confirmed: inside(ko), r: [own?.r, kept?.r, ko?.r].map((v) => v && r2(v)), sigma: hit?.sigma ?? null, size: hit?.size ?? null };
    const pc = (v) => `${Math.round(100 * v)}%`, cv = result.change.covers;
    check("what the box's report blocks covers at least 95% of the box (its place's error twice over), also once confirmed", hit && cv.detector >= 0.95 && cv.memory >= 0.95 && cv.confirmed >= 0.95,
      hit ? `the detector's obstacle r ${cv.r[0]} m: ${pc(cv.detector)}; the memory's r ${cv.r[1]} m: ${pc(cv.memory)}; confirmed, the keep-out r ${cv.r[2]} m: ${pc(cv.confirmed)} (σ said ${hit.sigma}, ${hit.size} m across)` : "not reported");
    const dims = hit?.evidence?.live ? await Promise.all([hit.evidence.live, hit.evidence.expected].map((b) => b && createImageBitmap(b).then((i) => [i.width, i.height]))) : null;
    result.change.evidence = dims && { live: dims[0], expected: dims[1], box: hit.evidence.box, cut: hit.evidence.cut };
    check("with evidence pictures (live and expected, at least 320 px, the change outlined)", !!(dims?.[1] && Math.max(...dims[0]) >= 320 && Math.max(...dims[1]) >= 320),
      dims ? `${dims[0].join("x")} + ${dims[1].join("x")} px, ${hit.evidence.live.size} + ${hit.evidence.expected.size} bytes of JPEG` : "none");
    const eb = hit?.evidence?.box;
    check("the outline off the evidence picture's edge, or the change said to be cut there", !!eb && (eb.every((v) => v > 0.01 && v < 0.99) || hit.evidence.cut === true),
      eb ? `outline [${eb.join(", ")}]${hit.evidence.cut ? ", cut: only part of it in view" : ""}` : "no evidence");
    if (run.best) {
      const f = run.best.f, d = $("live").getContext("2d");
      d.drawImage(await twin.render(poses[run.best.i], { width: 640, height: 480 }), 0, 0);
      show(f);
      $("liveCap").textContent = `frame ${run.best.i}: the drone camera (twin render with the box)`;
      if (hit?.evidence?.live) for (const n of ["live", "expected"]) hit.evidence[n] && upload(`evidence-${n}.jpg`, hit.evidence[n]);
    }
    drawPlan(poses, box, run.changes);
    $("planCap").textContent = `${seconds} s patrol (green), the box (white), reported changes (red: obstacle): ${err < Infinity ? `${r2(err)} m off` : "none"}`;
  });

  await part("door", async () => {
    // A doorway on the patrol's way closed (a leaf across it, sill to head, as the simulator closes one): the patrol flies
    // to it, finds it shut, holds and goes back the way it came. Reported as door-closed for that doorway, or at least a
    // door leaf on the map (nav/avoid.js), and nothing else.
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
    await twin.setActors([]);
    const seg = (p, d) => { const dx = d.b[0] - d.a[0], dy = d.b[1] - d.a[1], t = Math.max(0, Math.min(1, ((p.x - d.a[0]) * dx + (p.y - d.a[1]) * dy) / (dx * dx + dy * dy))); return Math.hypot(p.x - d.a[0] - t * dx, p.y - d.a[1] - t * dy); };
    const k0 = poses.findIndex((p, i) => i > 20 && house.doors.some((d) => d.rooms[1] && seg(p, d) < 0.3));
    if (k0 < 0) throw new Error("the patrol passes no doorway between rooms");
    const door = house.doors.filter((d) => d.rooms[1]).sort((a, b) => seg(poses[k0], a) - seg(poses[k0], b))[0];
    const mid = [(door.a[0] + door.b[0]) / 2, (door.a[1] + door.b[1]) / 2], L = Math.hypot(door.b[0] - door.a[0], door.b[1] - door.a[1]);
    let n = [-(door.b[1] - door.a[1]) / L, (door.b[0] - door.a[0]) / L];
    if (map.roomAt(mid[0] + 0.3 * n[0], mid[1] + 0.3 * n[1])?.id === door.rooms[0]) n = [-n[0], -n[1]];
    const off = (door.depth ?? 0) / 2, floor = map.floorAt(mid[0] - 0.3 * n[0], mid[1] - 0.3 * n[1]) ?? 0, head = door.headZ ?? floor + 2.0;
    const leaf = { id: "door", kind: "panel", x: mid[0] + n[0] * off, y: mid[1] + n[1] * off, z: floor, w: L, d: 0.04, h: head - floor, yaw: Math.atan2(door.b[1] - door.a[1], door.b[0] - door.a[0]), color: "#c9b79c" };
    await twin.setProps([leaf]);
    const stop = poses.findIndex((p, i) => i > 20 && seg(p, door) < 0.6), there = poses.slice(Math.max(0, stop - 120), stop);
    const route = [...there, ...Array(8).fill(there.at(-1)), ...[...there].reverse().map((p) => ({ ...p, yaw: p.yaw + Math.PI }))];
    const run = await patrol(route, "a closed door");
    const shut = run.changes.find((c) => c.kind === "door-closed" && c.door === door.id), leaves = run.avoid.temps.filter((t) => t.source === "door leaf");
    const away = run.changes.filter((c) => c !== shut && Math.hypot(c.x - mid[0], c.y - mid[1]) > 1.0);
    result.door = { door: door.id, rooms: door.rooms, frames: run.frames, changes: run.changes.map((c) => ({ kind: c.kind, x: r2(c.x), y: r2(c.y), door: c.door ?? null })), leaves: leaves.length, stats: run.det.stats };
    log("closed door", result.door);
    check("a closed door on the way: door-closed for that doorway (or a door leaf on the map), nothing elsewhere", (!!shut || leaves.length > 0) && !away.length,
      `${shut ? `door-closed at (${r2(shut.x)}, ${r2(shut.y)})` : "no door-closed"}, ${leaves.length} door leaf obstacle(s); ${run.changes.length} change(s) in ${run.frames} frames, ${away.length} away from the door`);
    await twin.setProps([]);
  });

  await part("approach", async () => {
    // Straight at a box the map doesn't have, from the home pad, asking for 0.55 m/s the whole way: nav/avoid.js caps the
    // speed from the live depth alone (4 Hz, 250 ms late, the localized pose off by poseErr); the drone brakes 30% weaker
    // than the cap assumes (&brake= the share; 0.7 of AVOID.decel) and follows the cap 0.1 s late. It must stop short of the
    // box (1.2 m tall: not one to fly over). The clock is the simulated one, for the map's temporary obstacles too.
    for (const id of [...map.temps.keys()]) map.removeTemp(id); // the change part's reports
    const h = house.home, u = [Math.cos(h.yaw), Math.sin(h.yaw)], floor = map.floorAt(h.x, h.y) ?? 0, T0 = performance.now();
    const ahead = [2.0, 1.8, 1.6].find((d) => map.clearance(h.x + d * u[0], h.y + d * u[1], floor + 0.6) > 0.4) ?? 1.6;
    const box = { id: "box", kind: "box", x: h.x + ahead * u[0], y: h.y + ahead * u[1], z: floor, w: 0.5, d: 0.5, h: 1.2, yaw: h.yaw, color: "#8a6b4a" };
    await twin.setActors([]);
    await twin.setProps([box]);
    const delay = 250, ctl = { videoDelay: delay, est: { ttc: Infinity } }, err = q.get("approachErr"), brake = +(q.get("brake") ?? 0.7) * AVOID.decel;
    let P = { x: h.x, y: h.y, z: floor + 1.0, yaw: h.yaw, sigma: Math.max(0.03, poseErr), status: "ok" }, v = 0, cmd = 0;
    const avoid = new Avoid({ map, localizer: { ctl, pose: () => ({ ...P }), velocity: () => [v * u[0], v * u[1]] } }), queue = [], lag = [];
    const gap = () => Math.abs((box.x - P.x) * u[0] + (box.y - P.y) * u[1]) - box.d / 2 - 0.05; // the drone's 5 cm radius to the box's face
    let closest = Infinity, fastest = 0, frames = 0, t = 0, trace = (window.trace = []);
    const clock = map.clock;
    map.clock = () => T0 + t;
    for (; t < 15000 && !(t > 3000 && v < 0.01); t += 50) {
      if (t % 250 === 0) {
        const cam = { ...P, z: P.z + CAM_DZ, pitch: 0, roll: 0 }, at = localized(cam, t / 250), f = await frame(cam, 2e6 + t, at);
        await dumpFrame("approach", t / 250, cam, at, f);
        if (f.a) queue.push({ t, f: { t: T0 + t, pose: { ...at, sigma: P.sigma, status: "ok" }, width: W, height: H, lens: LENS, depth: f.a.depth, conf: f.a.conf, expected: f.expected, boxes: [],
          rgb: { width: W, height: H, data: f.out.rgba }, expectedRgb: f.expectedRgb, mask: f.mask, grid: f.grid, ...(err != null && { depthErr: +err }) } }); // as LiveDepth's frames
      }
      while (queue.length && t - queue[0].t >= delay) (avoid.ingest(queue.shift().f, T0 + t), frames++);
      lag.push(avoid.limit({ vx: 0.55, vy: 0 }, P, T0 + t).vx ?? 0);
      cmd = lag.length > 2 ? lag.shift() : 0; // 0.1 s from the cap to the motors
      v += Math.max(-brake * 0.05, Math.min(AVOID.decel * 0.05, cmd - v));
      [P.x, P.y] = [P.x + v * u[0] * 0.05, P.y + v * u[1] * 0.05];
      closest = Math.min(closest, gap());
      fastest = Math.max(fastest, v);
      if (t % 250 === 0) trace.push([t, r2(gap()), r2(v), r2(avoid.last.cap), r2(avoid.last.run), r2(avoid.last.live), r2(avoid.ahead?.share ?? 0), avoid.last.why,
        [...avoid.temps.values()].map((q) => `${q.source} ${r2(q.x)},${r2(q.y)} r${r2(q.r)}`).join(" ")]);
    }
    map.clock = clock;
    result.approach = { box: { x: r2(box.x), y: r2(box.y), ahead }, closest: r2(closest), fastest: r2(fastest), seconds: r2(t / 1000), frames, depthErr: err != null ? +err : AVOID.depthErr, brake: r2(brake), last: avoid.last.why };
    log("approach", result.approach);
    check("flying straight at an unmapped box on live depth alone: stops at least 0.15 m short", closest >= 0.15,
      `stopped ${r2(closest)} m from it (${ahead} m ahead at the start, up to ${r2(fastest)} m/s, braking at ${r2(brake)} m/s² where the cap assumes ${AVOID.decel}, ${frames} depth frames, ${delay} ms late)`);
  });

  await part("goggles", async () => {
    // The app's LiveDepth on what the Goggles 3 stream really looks like: 1920x1080, the O4's 4:3 picture pillarboxed at
    // x 240-1680, OSD text in the black bars, nothing cropped. It must find the picture itself (PictureFinder), keep its
    // aspect, and then read depth as well as on the picture alone; nothing changed, so avoid must stay quiet. Then the
    // same with the bar finder off (the 16:9 frame taken for a 16:9 crop of the picture: what not finding the bars does).
    // The real drone's missions refuse such video until it is cropped (pictureProblem).
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
    await twin.setActors([]);
    await twin.setProps([]);
    const cv = new OffscreenCanvas(1920, 1080), g = cv.getContext("2d");
    let n = 0, P = null;
    const source = { ready: () => true, element: () => cv, region: () => ({ sx: 0, sy: 0, sw: 1920, sh: 1080 }), frameId: () => n };
    const perception = { source, latest: { t: 0, detections: [], width: 1920, height: 1080 }, on: () => () => {} };
    const loc = { pose: () => ({ ...P, sigma: 0.03, status: "ok" }), poseAt: () => ({ ...P }) };
    const run = async (finder) => {
      const ld = new LiveDepth({ perception, localizer: loc, ctl: { videoDelay: 0, isFlying: () => true, historyAt: () => ({ p: 0, q: 0 }) }, settings: { get: (k) => (k === "uptilt" ? 20 : undefined) }, twin });
      if (!finder) ld.picture = (src) => ({ ...src.region(), why: "" });
      const out = { frames: 0, abs: [], ahead: 0, wedges: 0, live: 0, states: [], ms: [] }, av = new Avoid({ map, localizer: { ctl: { videoDelay: 0, est: { ttc: Infinity } }, pose: () => P, velocity: () => [0, 0] } });
      av.on("temp", (t) => t.added && (t.source === "way ahead" ? out.wedges++ : t.source === "live depth" && out.live++));
      ld.on("status", (st) => out.states.push(st.text));
      ld.on("depth", (f) => out.last = f);
      await ld.start();
      const steps = poses.filter((_, i) => i % 3 === 0).slice(0, 75);
      for (const [i, pose] of steps.entries()) {
        P = { ...pose, z: pose.z - CAM_DZ };
        const bmp = await relight(await twin.render(pose, { width: 640, height: 480 }));
        g.fillStyle = "#000";
        g.fillRect(0, 0, 1920, 1080);
        g.drawImage(bmp, 240, 0, 1440, 1080);
        bmp.close();
        g.fillStyle = "#fff";
        g.font = "28px monospace";
        for (const [x, y, t] of [[30, 60, "3.86V"], [30, 1040, "00:41"], [1700, 60, "REC"], [1700, 1040, `${120 + i} m`]]) g.fillText(t, x, y);
        n++;
        out.last = null;
        const t0 = performance.now();
        ld.next = 0;
        await ld.frame({ t: t0, detections: [] });
        if (!out.last) { await new Promise((r) => setTimeout(r, 120)); continue; } // waiting: the finder samples every 300 ms
        out.ms.push(performance.now() - t0);
        out.frames++;
        const f = out.last, truth = await twin.depth(f.pose, { width: f.width, height: f.height, lens: f.lens });
        const rel = [];
        for (let k = 0; k < truth.length; k++) if (truth[k] > 0.2 && truth[k] < 8 && f.depth[k] > 0) rel.push(Math.abs(f.depth[k] - truth[k]) / truth[k]);
        out.abs.push(med(rel));
        av.ingest({ ...f, pose: { ...f.pose, sigma: 0.03 } });
        out.ahead += av.ahead?.slow && av.ahead.t === f.t ? 1 : 0;
      }
      ld.stop();
      return { ...out, absRel: r3(med(out.abs)), ms: r2(med(out.ms)) };
    };
    const found = await run(true), problem = pictureProblem(perception), blind = await run(false);
    result.goggles = { found: { frames: found.frames, absRel: found.absRel, ahead: found.ahead, wedges: found.wedges, live: found.live, ms: found.ms, states: [...new Set(found.states)] },
      noFinder: { frames: blind.frames, absRel: blind.absRel, ahead: blind.ahead, wedges: blind.wedges, live: blind.live }, problem };
    log("goggles", result.goggles);
    check("goggles 1920x1080 with the picture pillarboxed: LiveDepth finds the picture, reads depth as on the picture alone, and avoid stays quiet",
      found.frames >= 20 && found.absRel < 0.15 && found.ahead <= 0.05 * found.frames && !found.wedges && found.live <= 1,
      `${found.frames} frames, median abs rel ${found.absRel}, way ahead on ${found.ahead}, ${found.wedges} wedge(s), ${found.live} unmapped obstacle(s), ${found.ms} ms a frame; with the bar finder off: abs rel ${blind.absRel}, way ahead on ${blind.ahead} of ${blind.frames}, ${blind.wedges} wedge(s), ${blind.live} unmapped obstacle(s)`);
    check("a mission on the real drone refuses that video until it is cropped", /black bars/.test(problem), problem || "no problem found");
  });
} catch (e) {
  log("ERROR", String(e?.stack || e));
}
result.done = true;
if (save) {
  for (const id of ["live", "rect", "disp", "aligned", "expected", "diff", "plan"]) await shotPng(id);
  await upload("results.json", JSON.stringify(result, null, 1));
}
log("done", JSON.stringify(result.checks));
