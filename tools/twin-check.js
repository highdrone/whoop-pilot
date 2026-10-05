// Browser check for the splat twin (app/js/twin): drone fisheye view, PSNR against a held-out capture photo,
// actor compositing and oracle ground truth (walls, actors hiding actors), lens rules, expected depth, what RF-DETR-N
// makes of the actors, and timing (visible or hidden page).
// Results land in window.result; window.bench() reruns the timing (e.g. with the tab hidden).
import { Twin, DRONE_LENS, pinholeLens, fieldOfView, poseFromCapture } from "../app/js/twin/twin.js";
import { houseFrame } from "../app/js/house/frames.js";
import { toPlanar, decode } from "../app/js/vision/rfdetr.js";
import { MODELS, fetchModel } from "../app/js/vision/models.js";

const $ = (id) => document.getElementById(id);
const q = new URLSearchParams(location.search);
const lines = [];
const log = (...a) => { lines.push(a.map((x) => (typeof x === "string" ? x : JSON.stringify(x))).join(" ")); $("log").textContent = lines.join("\n"); };
const result = (window.result = { done: false, checks: {} });
const DEG = Math.PI / 180, now = () => performance.now();
const med = (v) => +[...v].sort((a, b) => a - b)[v.length >> 1].toFixed(1);
const only = q.get("only")?.split(",");

const base = q.get("capture") && new URL(q.get("capture").replace(/\/?$/, "/"), location.href);
const url = (key, rel) => q.get(key) ?? (base && rel ? new URL(rel, base).href : null);
const getJson = async (u) => { const r = await fetch(u); if (!r.ok) throw new Error(`${u}: HTTP ${r.status}`); return r.json(); };

function check(name, ok, detail) {
  result.checks[name] = { ok, detail };
  const tr = $("checks").insertRow();
  tr.insertCell().textContent = ok ? "PASS" : "FAIL";
  tr.cells[0].className = ok ? "pass" : "fail";
  tr.insertCell().textContent = name;
  tr.insertCell().textContent = detail;
}
// One part of the page; an exception fails it without stopping the rest. ?only=a,b runs just those parts.
async function part(name, fn) {
  if (only && !only.includes(name)) return;
  try { await fn(); } catch (e) { log(`ERROR in ${name}:`, String(e?.stack || e)); check(`${name}: runs`, false, String(e?.message || e)); }
}

const draw = (id, bmp) => { const c = $(id); c.width = bmp.width; c.height = bmp.height; c.getContext("2d").drawImage(bmp, 0, 0); bmp.close?.(); return c; };
const putPixels = (id, px) => { const c = $(id); c.width = px.width; c.height = px.height; c.getContext("2d").putImageData(new ImageData(px.data, px.width, px.height), 0, 0); };
function drawBoxes(id, boxes, color) {
  const c = $(id), g = c.getContext("2d");
  g.lineWidth = 2; g.font = "12px ui-monospace,monospace";
  for (const o of boxes) {
    const b = o.visibleBox ?? o.box, x = b.x * c.width, y = b.y * c.height;
    g.strokeStyle = g.fillStyle = color ?? (o.visibleFraction > 0 ? "#3f6" : "#f44");
    g.setLineDash(o.visibleFraction > 0 || color ? [] : [5, 4]);
    g.strokeRect(x, y, b.w * c.width, b.h * c.height);
    g.fillText(o.text ?? `${o.id} ${(o.visibleFraction * 100).toFixed(0)}%`, x + 2, Math.max(12, y - 3));
  }
  g.setLineDash([]);
}
function psnr(a, b) {
  let se = 0;
  for (let i = 0; i < a.length; i += 4) for (let k = 0; k < 3; k++) { const d = a[i + k] - b[i + k]; se += d * d; }
  return se ? +(10 * Math.log10((255 * 255) / (se / ((a.length / 4) * 3)))).toFixed(2) : Infinity;
}
async function imagePixels(id, src, w, h) {
  const r = await fetch(src);
  if (!r.ok) throw new Error(`${src}: HTTP ${r.status}`);
  const img = await createImageBitmap(await r.blob()); // Image.decode() never settles in a hidden tab
  const c = $(id), g = c.getContext("2d");
  c.width = w; c.height = h; g.imageSmoothingQuality = "high";
  g.drawImage(img, 0, 0, w, h);
  return g.getImageData(0, 0, w, h).data;
}
// Pixel box of what changed between two renders of the same pose (the actor as actually rendered).
function diffBox(a, b, w, h) {
  let x0 = w, y0 = h, x1 = -1, y1 = -1, n = 0;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const i = (y * w + x) * 4;
    if (Math.abs(a[i] - b[i]) + Math.abs(a[i + 1] - b[i + 1]) + Math.abs(a[i + 2] - b[i + 2]) > 12) { n++; if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; }
  }
  return { box: x1 < 0 ? null : [x0, y0, x1 + 1, y1 + 1], n };
}
const pxBox = (b, w, h) => [b.x * w, b.y * h, (b.x + b.w) * w, (b.y + b.h) * h];
const boxErr = (a, b) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));
const iou = (a, b) => {
  const w = Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x), h = Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y);
  return w > 0 && h > 0 ? (w * h) / (a.w * a.h + b.w * b.h - w * h) : 0;
};

// Each actor as rendered (render with it minus render without it) against its oracle box, in pixels.
async function alignment(twin, pose, actors, view) {
  await twin.setActors(actors);
  const all = await twin.pixels(pose, view), gt = await twin.oracle(pose, view), out = [];
  for (const a of actors) {
    await twin.setActors(actors.filter((b) => b !== a));
    const without = await twin.pixels(pose, view), { box: d, n } = diffBox(all.data, without.data, all.width, all.height);
    const o = gt.find((g) => g.id === a.id);
    out.push({ id: a.id, visibleFraction: o?.visibleFraction ?? null, renderedBox: d, renderedPixels: n, errPx: d && o?.visibleBox ? +boxErr(d, pxBox(o.visibleBox, all.width, all.height)).toFixed(1) : null });
  }
  await twin.setActors(actors);
  return { gt, out };
}

// RF-DETR-N as the app runs it (rfdetr-worker.js: stretched to 384, ImageNet normalisation), on the page's thread.
// src: URL of its model.onnx, or "app" for the app's own copy (Cache Storage, downloaded on a miss).
async function loadDetector(src) {
  const ort = await import("../app/vendor/ort/ort.webgpu.min.mjs");
  ort.env.wasm.wasmPaths = new URL("../app/vendor/ort/", import.meta.url).href;
  ort.env.wasm.numThreads = 1;
  ort.env.logLevel = "error";
  const spec = MODELS["rfdetr-n"], R = spec.res;
  const bytes = src === "app" ? (await fetchModel(spec)).bytes : await (await fetch(src)).arrayBuffer();
  const session = await ort.InferenceSession.create(new Uint8Array(bytes), { executionProviders: ["webgpu"], graphOptimizationLevel: "all", logSeverityLevel: 3 });
  const g = new OffscreenCanvas(R, R).getContext("2d", { willReadFrequently: true }), input = new Float32Array(3 * R * R);
  return async (px) => {
    g.drawImage(await createImageBitmap(new ImageData(px.data, px.width, px.height)), 0, 0, R, R);
    toPlanar(g.getImageData(0, 0, R, R).data, R * R, input);
    const { logits: L, pred_boxes: B } = await session.run({ [session.inputNames[0]]: new ort.Tensor("float32", input, [1, 3, R, R]) });
    return decode(L.data, B.data, L.dims[1], L.dims[2], { thr: 0.05 });
  };
}

// Renders are GPU-synced (the default), so frame rates are real. Flying moves 5 cm and turns 2 deg per frame
// (0.75 m/s at 15 fps): a re-sort every other frame.
async function bench(twin, pose, frames = 90) {
  const view = { width: 640, height: 480 }, r = { visibility: document.visibilityState };
  let t = now(), sorts = twin.stats.sorts;
  for (let i = 0; i < frames; i++) (await twin.render({ ...pose, x: pose.x + 0.05 * (i % 40), yaw: pose.yaw + 2 * i * DEG }, view)).close();
  r.flyFps = +((frames * 1000) / (now() - t)).toFixed(1);
  r.flySorts = twin.stats.sorts - sorts;
  const same = [], moved = [];
  for (let i = 0; i < 20; i++) { t = now(); (await twin.render({ ...pose, yaw: pose.yaw + i * 3 * DEG }, view)).close(); same.push(now() - t); }
  for (let i = 0; i < 10; i++) { t = now(); (await twin.render({ ...pose, x: pose.x + 0.15 * (i + 1) }, view)).close(); moved.push(now() - t); }
  t = now();
  for (let i = 0; i < frames; i++) (await twin.render({ ...pose, yaw: pose.yaw + i * DEG }, view)).close();
  r.hoverFps = +((frames * 1000) / (now() - t)).toFixed(1);
  r.samePositionMs = med(same);
  r.newPositionMs = med(moved);
  const dep = [], ora = [];
  for (let i = 0; i < 5; i++) { t = now(); await twin.depth(pose, { width: 320, height: 240 }); dep.push(now() - t); t = now(); await twin.oracle(pose, view); ora.push(now() - t); }
  r.depth320Ms = med(dep);
  r.oracleMs = med(ora);
  r.stats = { ...twin.stats };
  return r;
}

try {
  if (!url("splat", "outputs/splat.spz")) throw new Error("no capture given (see the query parameters above)");
  $("usage").hidden = true;
  const manifest = !q.get("rooms") && base ? await getJson(new URL("outputs/plans/manifest.json", base)).catch(() => null) : null;
  const roomsUrl = url("rooms") ?? (manifest?.files?.json?.name ? new URL(`outputs/plans/${manifest.files.json.name}`, base).href : null);
  const [scene, site, roomsAuto, rooms] = await Promise.all([url("scene", "outputs/scene.json"), url("site", "site.json"), url("roomsAuto", "work/room/rooms.auto.json"), roomsUrl]
    .map((u) => (u ? getJson(u).catch(() => null) : null)));
  const frame = houseFrame({ site, roomsAuto, rooms, scene });
  log("frame", { f: frame.f, Yf: frame.Yf, floor: frame.floorSource }, "| lens", DRONE_LENS, "fov", fieldOfView(DRONE_LENS));
  const twin = (window.twin = await Twin.create({ splat: url("splat", "outputs/splat.spz"), frame }));
  log("loaded", twin.info);
  result.load = twin.info;

  // The drone camera 1 m above the living-room floor (room floors: z = f*Yf - floor_y). The default x, y and the
  // wall and hiding poses below are for the reference capture; give another capture &x=&y= and &wall=0.
  const floorZ = (r) => frame.roomZ(r.model.floor_y);
  const living = rooms?.rooms?.length ? rooms.rooms.reduce((a, r) => (floorZ(r) < floorZ(a) ? r : a)) : null;
  const fz = living ? floorZ(living) : 0;
  const home = { x: +(q.get("x") ?? 1.58), y: +(q.get("y") ?? 2.17), z: fz + 1.0, yaw: +(q.get("yaw") ?? 0) * DEG, pitch: 0, roll: 0 };
  const at = (dx, dy, o) => ({ x: home.x + dx, y: home.y + dy, z: fz, ...o });
  const cast = [
    { id: "person", kind: "person", ...at(2.0, 0.3), yaw: Math.PI, pose: "stand" },
    { id: "cat", kind: "cat", ...at(2.7, -0.9), yaw: 2.4, pose: "walk", phase: 1 },
    { id: "dog", kind: "dog", ...at(2.4, 1.2), yaw: -2.2, pose: "sit" },
  ];
  const view = { width: 640, height: 480 };

  await part("live", async () => {
    const live = (result.live = await alignment(twin, home, cast, view));
    draw("fish", await twin.render(home, view));
    drawBoxes("fish", live.gt);
    $("fishCap").textContent = `drone camera 640x480, ${DRONE_LENS.diagFovDeg} deg diagonal equidistant, ${DRONE_LENS.uptiltDeg} deg uptilt, at (${home.x}, ${home.y}, ${home.z.toFixed(2)}) m in ${living?.name ?? "the house"}; boxes = oracle`;
    log("living room oracle", live.gt, "alignment", live.out);
  });

  await part("psnr", async () => {
    const key = q.get("key") ?? "c00_f005069", cam = scene?.cameras?.find((c) => c.key === key);
    if (!cam) return log(`no camera ${key} in scene.json: skipping the PSNR check`);
    const pose = poseFromCapture(cam, frame), N = 320;
    // SiteSpec dataset camera L0c: PINHOLE 1476x1476, f 738.2188, c 738
    const lens = { model: "pinhole", fx: 738.21875 / 1476, cx: 0.5, cy: 0.5 };
    const t = now(), px = await twin.pixels(pose, { width: N, height: N, lens, ss: 2, actors: false }), ms = now() - t;
    const px1 = await twin.pixels(pose, { width: N, height: N, lens, actors: false });
    putPixels("pin", px);
    const photoUrl = url("photo", `work/dataset/images/L0c/${key}.jpg`), trainerUrl = url("trainer", `work/train/eval/L0c/${key}.jpg`);
    const photo = await imagePixels("photo", photoUrl, N, N);
    const trainer = await imagePixels("trainer", trainerUrl, N, N).catch(() => null);
    const cap = (result.capture = { key, psnr: psnr(px.data, photo), psnrNoSupersample: psnr(px1.data, photo), trainerVsPhoto: trainer && psnr(trainer, photo), twinVsTrainer: trainer && psnr(px.data, trainer), ms: +ms.toFixed(1) });
    $("pinCap").textContent = `twin 90 deg pinhole at ${key}: PSNR ${cap.psnr} dB vs photo (trainer ${cap.trainerVsPhoto ?? "?"} dB)`;
    log("capture pose", cap);
    check("PSNR at the capture pose >= 28 dB", cap.psnr >= 28, `${cap.psnr} dB (1x: ${cap.psnrNoSupersample}; trainer's own render: ${cap.trainerVsPhoto} dB; twin vs trainer ${cap.twinVsTrainer} dB)`);
  });

  if (q.get("wall") !== "0") await part("wall", async () => {
    // A person in front of the north wall, one behind it.
    const wallPose = { x: 3.46, y: 2.96, z: fz + 1.0, yaw: 90 * DEG, pitch: 0, roll: 0 };
    const pair = [
      { id: "front", kind: "person", x: 3.0, y: 4.3, z: fz, yaw: -Math.PI / 2, pose: "stand" },
      // behind the brick right of the fireplace (the dark firebox's splats are too thin to hide a person)
      { id: "behind", kind: "person", x: 3.95, y: 6.55, z: fz, yaw: -Math.PI / 2, pose: "stand" },
    ];
    const wall = (result.wall = await alignment(twin, wallPose, pair, view));
    draw("wall", await twin.render(wallPose, view));
    drawBoxes("wall", wall.gt);
    $("wallCap").textContent = "person 1.3 m in front of the north wall (green) and 0.8 m behind it (red, dashed: oracle says hidden)";
    log("wall oracle", wall.gt, "alignment", wall.out);
    const errs = [...(result.live?.out ?? []), ...wall.out].filter((a) => a.errPx != null);
    const front = wall.out.find((a) => a.id === "front"), behind = wall.gt.find((g) => g.id === "behind"), behindR = wall.out.find((a) => a.id === "behind");
    check("oracle boxes match rendered actors within 3 px", errs.length >= 3 && errs.every((a) => a.errPx <= 3), errs.map((a) => `${a.id} ${a.errPx}px`).join(", "));
    check("actor behind a wall: visibleFraction 0", behind?.visibleFraction === 0 && behindR?.renderedPixels < 0.01 * behind.pixels && front?.visibleFraction > 0.9,
      `behind ${behind?.visibleFraction} (${behind?.pixels} px silhouette, ${behindR?.renderedPixels} px changed in the render), front ${front?.visibleFraction}`);
  });

  await part("occlusion", async () => {
    // Actors hiding actors: whole silhouettes stay whole, the hidden share shows in visibleFraction.
    const P = { ...home, z: fz + 1.2 };
    const group = [
      { id: "near", kind: "person", ...at(0.6, 0), yaw: Math.PI, pose: "stand" },
      { id: "hidden", kind: "cat", ...at(2.5, 0, { z: fz + 0.9 }), yaw: Math.PI, pose: "sit" }, // on a shelf, right behind the chest
      { id: "part", kind: "dog", ...at(1.9, -0.9), yaw: Math.PI / 2, pose: "stand" }, // half behind the person's side
    ];
    await twin.setActors(group);
    const together = await twin.oracle(P, view), alone = {};
    for (const a of group) { await twin.setActors([a]); alone[a.id] = (await twin.oracle(P, view))[0]; }
    const T = Object.fromEntries(together.map((o) => [o.id, o])), same = (a, b) => JSON.stringify(a) === JSON.stringify(b);
    result.occlusion = { together, alone };
    log("occlusion", together, "alone", alone);
    const h = T.hidden, p = T.part, n = T.near;
    check("an actor hidden by another reports visibleFraction 0", !!h && h.visibleFraction === 0 && alone.hidden?.visibleFraction > 0.5 && same(h.box, alone.hidden.box) && h.pixels === alone.hidden.pixels,
      `hidden cat: ${h?.visibleFraction ?? "missing"} (${h?.pixels} px; alone ${alone.hidden?.visibleFraction}, ${alone.hidden?.pixels} px)`);
    check("a partly hidden actor keeps its whole box and loses visibleFraction", !!p && same(p.box, alone.part.box) && p.pixels === alone.part.pixels && p.visibleFraction < alone.part.visibleFraction - 0.1 && p.visibleFraction > 0
      && p.visiblePixels === Math.round(p.visibleFraction * p.pixels) && same(n?.visibleBox, alone.near.visibleBox),
      `dog ${p?.visibleFraction} (${p?.visiblePixels}/${p?.pixels} px; alone ${alone.part?.visibleFraction}, box ${same(p?.box, alone.part?.box) ? "unchanged" : "changed"}), front person ${n?.visibleFraction}`);
  });

  await part("lens", async () => {
    // A per-call lens with a model takes that model's defaults (a pinhole lens has no uptilt), and 16:9 drone frames
    // are the centre crop of the 4:3 lens.
    await twin.setActors([]);
    const v = { width: 320, height: 240 };
    const short = await twin.pixels(home, { ...v, lens: { model: "pinhole", hfovDeg: 90 } }), full = await twin.pixels(home, { ...v, lens: pinholeLens(90) });
    const tilted = await twin.pixels({ ...home, pitch: -20 * DEG }, { ...v, lens: pinholeLens(90) });
    const four = await twin.pixels(home, { width: 320, height: 240 }), wide = await twin.pixels(home, { width: 320, height: 180 });
    const crop = { data: four.data.slice(30 * 320 * 4, 210 * 320 * 4) };
    const r = (result.lens = { pinholeVsFull: psnr(short.data, full.data), pinholeVsTilted: psnr(short.data, tilted.data), crop169: psnr(crop.data, wide.data), fov169: fieldOfView(DRONE_LENS, 320, 180) });
    log("lens", r);
    check("per-call { model: pinhole, hfovDeg } is pinholeLens (no inherited uptilt)", r.pinholeVsFull === Infinity && r.pinholeVsTilted < 20, `vs pinholeLens(90): ${r.pinholeVsFull} dB, vs tilted 20 deg up: ${r.pinholeVsTilted} dB`);
    check("16:9 drone frame is the centre crop of the 4:3 lens", r.crop169 >= 30 && Math.abs(r.fov169.h - 127.2) < 0.2, `${r.crop169} dB against the 4:3 frame's middle rows; ${r.fov169.h.toFixed(1)} x ${r.fov169.v.toFixed(1)} deg`);
  });

  await part("depth", async () => {
    await twin.setActors(cast);
    const dv = { width: 320, height: 240 }, d = await twin.depth(home, dv);
    const rgba = new Uint8ClampedArray(dv.width * dv.height * 4);
    let covered = 0;
    for (let i = 0; i < d.length; i++) {
      const v = d[i], t = Number.isFinite(v) ? Math.min(1, v / 6) : 1;
      if (Number.isFinite(v)) covered++;
      rgba.set(Number.isFinite(v) ? [255 * (1 - t), 255 * (1 - Math.abs(t - 0.5) * 2), 255 * t, 255] : [0, 0, 0, 255], i * 4);
    }
    putPixels("depth", { width: dv.width, height: dv.height, data: rgba });
    const centre = d[(dv.height >> 1) * dv.width + (dv.width >> 1)];
    $("depthCap").textContent = `expected depth along each ray, 0-6 m (red near), drone lens, actors included; centre ${centre.toFixed(2)} m, ${((100 * covered) / d.length).toFixed(1)}% covered`;
    // A person 1.2 m ahead: depth at their chest is the person, and with actors: false the room behind.
    await twin.setActors([{ id: "p", kind: "person", ...at(1.2, 0), yaw: Math.PI, pose: "stand" }]);
    const [o] = await twin.oracle(home, dv), px = Math.round((o.box.x + o.box.w / 2) * dv.width), py = Math.round((o.box.y + o.box.h * 0.3) * dv.height);
    const withA = (await twin.depth(home, dv))[py * dv.width + px], without = (await twin.depth(home, { ...dv, actors: false }))[py * dv.width + px];
    result.depth = { centre, coveredPct: +((100 * covered) / d.length).toFixed(1), chest: withA, behindChest: without };
    log("depth", result.depth);
    check("expected depth sees actors (actors: false leaves them out)", withA > 0.95 && withA < 1.3 && without > withA + 0.5, `chest of a person 1.2 m ahead: ${withA.toFixed(2)} m; without actors ${without.toFixed(2)} m`);
  });

  await part("create", async () => {
    // A failed create must not leave its worker (and WebGL context) behind.
    const terminate = Worker.prototype.terminate;
    let ended = 0;
    Worker.prototype.terminate = function () { ended++; return terminate.call(this); };
    try {
      const err = await Twin.create({ splat: new URL("missing-splat.spz", base ?? location.href), frame }).then(() => null, (e) => e);
      check("a failed Twin.create rejects and ends its worker", !!err && ended === 1, `${err ? err.message.split("\n")[0] : "resolved"}; workers ended ${ended}`);
    } finally { Worker.prototype.terminate = terminate; }
  });

  if (q.get("det")) await part("det", async () => {
    // RF-DETR-N on the actors as the drone sees them; people and pets count from 0.4 (perception.js find()). The scenes
    // are poses it recognises; sitting dogs (teddy bear) and lying or rear views of pets are hit and miss: &detGrid=1
    // measures every kind, pose, heading and distance.
    const detect = await loadDetector(q.get("det")), minScore = +(q.get("detMin") ?? 0.4), DOWN = 20 * DEG;
    const low = { ...home, z: fz + 0.6, pitch: DOWN }, pin = { pose: { ...home, pitch: 35 * DEG }, view: { ...view, lens: pinholeLens(70) } };
    const found = async (pose, v, actors) => {
      await twin.setActors(actors);
      const gt = await twin.oracle(pose, v), px = await twin.pixels(pose, v), dets = await detect(px);
      return { px, gt, dets, rows: gt.map((o) => {
        const hits = dets.filter((d) => iou(d.box, o.visibleBox ?? o.box) >= 0.3).sort((a, b) => b.score - a.score), own = hits.find((d) => d.label === o.kind);
        return { id: o.id, kind: o.kind, score: +(own?.score ?? 0).toFixed(3), best: hits[0] ? `${hits[0].label} ${hits[0].score.toFixed(2)}` : "none", heightPx: Math.round(o.visibleBox.h * px.height) };
      }) };
    };
    const scenes = [
      { name: "living room", pose: home, view, actors: [cast[0]] },
      { name: "drone low, pets crossing", pose: low, view, actors: [
        { id: "cat", kind: "cat", ...at(1.2, -0.45), yaw: 90 * DEG, pose: "walk", phase: 1 },
        { id: "dog", kind: "dog", ...at(2.2, 0.1), yaw: 90 * DEG, pose: "walk", phase: 1 }] },
      { name: "drone low, dog and person", pose: low, view, actors: [
        { id: "dog", kind: "dog", ...at(1.6, -0.6), yaw: 120 * DEG, pose: "stand" },
        { id: "person", kind: "person", ...at(2.6, -0.1), yaw: 135 * DEG, pose: "walk", phase: 0.6 }] },
      { name: "pinhole 70 deg looking down", ...pin, actors: [
        { id: "cat", kind: "cat", ...at(1.5, -0.55), yaw: 90 * DEG, pose: "stand" },
        { id: "dog", kind: "dog", ...at(1.2, 0.15), yaw: Math.PI, pose: "lie" }] },
    ];
    const rows = [];
    for (const s of scenes) {
      const f = await found(s.pose, s.view, s.actors);
      rows.push(...f.rows.map((r) => ({ scene: s.name, ...r })));
      if (s === scenes[1]) {
        putPixels("det", f.px);
        drawBoxes("det", f.gt);
        drawBoxes("det", f.dets.filter((d) => d.score >= 0.2).map((d) => ({ ...d, text: `${d.label} ${d.score.toFixed(2)}` })), "#fc3");
      }
    }
    result.det = rows;
    log("detector", rows);
    $("detCap").textContent = `RF-DETR-N on the twin (green: oracle, yellow: detections >= 0.2), ${scenes[1].name}`;
    check(`RF-DETR-N finds every actor as its kind at >= ${minScore}`, rows.length === scenes.reduce((n, s) => n + s.actors.length, 0) && rows.every((r) => r.score >= minScore), rows.map((r) => `${r.id} ${r.score} (${r.scene}; best ${r.best}; ${r.heightPx} px)`).join(", "));
    if (q.get("detGrid") !== "1") return;
    // One actor at a time: every pose, five headings, two distances, from the low drone and the pinhole looking down.
    const grid = { person: [["stand", "walk", "sit"], [1.8, 2.6], [home, low]], cat: [["stand", "walk", "sit", "lie"], [1.2, 1.9], [low, pin]], dog: [["stand", "walk", "sit", "lie"], [1.5, 2.2], [low, pin]] };
    const tally = {};
    for (const [kind, [poses, dists, cams]] of Object.entries(grid)) for (const cam of cams) for (const pose of poses) for (const yaw of [180, 135, 90, 45, 0]) for (const d of dists) {
      const isPin = cam === pin, { rows: [r] } = await found(isPin ? pin.pose : cam, isPin ? pin.view : view,
        [{ id: "a", kind, ...at(isPin ? d * 0.8 : d, isPin ? -0.35 : -0.2), yaw: yaw * DEG, pose, phase: 1 }]);
      const t = (tally[`${kind} ${pose}`] ??= { n: 0, found: 0, mean: 0 });
      t.n++; t.found += r.score >= minScore; t.mean += (r.score - t.mean) / t.n;
    }
    for (const t of Object.values(tally)) t.mean = +t.mean.toFixed(2);
    const totals = Object.fromEntries(Object.keys(grid).map((k) => [k, Object.entries(tally).filter(([n]) => n.startsWith(k)).reduce((a, [, t]) => [a[0] + t.found, a[1] + t.n], [0, 0]).join("/")]));
    result.detGrid = { totals, tally };
    log("detector grid", result.detGrid);
    check("RF-DETR-N grid (information)", true, `${Object.entries(totals).map(([k, v]) => `${k} ${v}`).join(", ")}; ${Object.entries(tally).map(([k, t]) => `${k} ${t.found}/${t.n}`).join(", ")}`);
  });

  await part("bench", async () => {
    window.bench = (frames) => bench(twin, home, frames);
    if (q.get("bench") === "0") return;
    await twin.setActors(cast);
    const b = (result.bench = await bench(twin, home));
    log("bench", b);
    check("drone fisheye 640x480 >= 15 fps", b.flyFps >= 15, `flying ${b.flyFps} fps (${b.flySorts} re-sorts in 90 frames), hovering ${b.hoverFps} fps, new position ${b.newPositionMs} ms (page ${b.visibility})`);
    check("same-position render < 30 ms", b.samePositionMs < 30, `${b.samePositionMs} ms GPU-synced (page ${b.visibility}); load ${twin.info.loadMs} ms, depth 320x240 ${b.depth320Ms} ms, oracle ${b.oracleMs} ms`);
  });
  await twin.setActors(cast);
  draw("fish", await twin.render(home, view));
  drawBoxes("fish", await twin.oracle(home, view));
  result.done = true;
} catch (e) {
  log("ERROR", String(e?.stack || e));
  result.error = String(e?.stack || e);
  result.done = true;
}
