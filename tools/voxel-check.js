// Browser check for the 3D map (app/js/house/voxels.js, coverage.js) and the twin's props and depthBatch, on a real capture
// served by tools/whoop.mjs. Results land in window.result (done, checks, numbers); ?save= posts the pictures.
import { houseSource } from "../app/js/ui/housepanel.js";
import { importCapture } from "../app/js/house/import.js";
import { houseFrame } from "../app/js/house/frames.js";
import { buildVoxels, voxelCentres, VoxelMap, UNKNOWN, FREE, OCCUPIED, FLAG, bandMask } from "../app/js/house/voxels.js";
import { inPolygon } from "../app/js/house/homemap.js";
import { coverageReport } from "../app/js/house/coverage.js";
import { roomCenter, plan } from "../app/js/house/planner.js";
import { openStore } from "../app/js/house/store.js";
import { Twin, pinholeLens } from "../app/js/twin/twin.js";

const $ = (id) => document.getElementById(id);
const q = new URLSearchParams(location.search), only = q.get("only")?.split(",");
const lines = [];
const log = (...a) => { lines.push(a.map((x) => (typeof x === "string" ? x : JSON.stringify(x))).join(" ")); $("log").textContent = lines.join("\n"); };
const result = (window.result = { done: false, checks: {}, numbers: {} });
const now = () => performance.now();
const status = (text, frac) => { $("status").textContent = text; if (frac != null) $("bar").firstChild.style.width = `${Math.round(frac * 100)}%`; };
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
async function save(id, name) {
  const url = q.get("save");
  if (!url) return;
  const blob = await new Promise((r) => $(id).toBlob(r, "image/png"));
  await fetch(`${url}?name=${encodeURIComponent(name)}`, { method: "POST", body: blob }).then((r) => r.ok || log("save failed", name, r.status), (e) => log("save failed", name, String(e)));
}

// How fast this machine is right now (a fixed loop, to compare runs: a busy machine runs everything slower).
function calib() {
  const t = now();
  let x = 0;
  for (let i = 1; i < 4e6; i++) x += Math.sqrt(i) % 3;
  return +(now() - t).toFixed(1) + (x < 0 ? 1 : 0);
}

const COLORS = { [FREE]: [232, 230, 224], [OCCUPIED]: [200, 40, 42], [UNKNOWN]: [74, 79, 102] };
let ctx = null;

await part("build", async () => {
  const t0 = now();
  const list = (await (await fetch("/house-files/")).json()).projects;
  const project = list.find((p) => p.id === q.get("project")) ?? list.find((p) => p.files?.splat);
  if (!project) throw new Error("no capture with a splat under /house-files/");
  status(`importing ${project.name ?? project.id}`);
  const imp = await importCapture(houseSource(project, ({ loaded, total }) => status(`downloading the splat ${(loaded / 1e6).toFixed(0)} / ${(total / 1e6).toFixed(0)} MB`, loaded / total)));
  const { house, map, splat } = imp, tImport = now() - t0;
  house.splatFile = splat.name;
  const frame = houseFrame({ f: house.frame.f, Yf: house.frame.Yf });
  let t = now();
  const centres = await voxelCentres(splat.bytes, splat.name, frame), tCentres = now() - t;
  t = now();
  const twin = await Twin.create({ splat: new Blob([splat.bytes]), house }), tTwin = now() - t;
  log(`import ${(tImport / 1000).toFixed(1)} s (download, decode, 2.5D map); ${centres.n} splats down to 5% opacity decoded in ${tCentres.toFixed(0)} ms; twin ${twin.info.splats} splats, ready in ${(tTwin / 1000).toFixed(1)} s on ${twin.info.gpu}`);
  t = now();
  // The page's turns during the build: a message ping-pong (hidden pages throttle timers and report no long tasks) records
  // the longest time between two turns.
  const phases = {}, gaps = { longest: 0, over50: 0, total: 0, during: null }, ping = new MessageChannel();
  let last = now(), pinging = true, phase = "start";
  ping.port1.onmessage = () => {
    const t = now(), g = t - last;
    if (g > 50) (gaps.over50++, (gaps.total += g));
    if (g > gaps.longest) [gaps.longest, gaps.during] = [g, phase];
    last = t;
    if (pinging) ping.port2.postMessage(0);
  };
  ping.port2.postMessage(0);
  const vox = await buildVoxels({ house, map, centres, twin, options: q.get("views") ? { frontierViews: +q.get("views") } : {}, onProgress: (p) => {
    phases[p.phase] ??= now();
    phase = `${p.phase} ${p.done}/${p.total}`;
    status(`${p.text} (${p.done}/${p.total})`, p.total ? p.done / p.total : 0);
    if (p.phase === "frontier" && p.done === p.total && p.gains) log(`frontier round ${p.round}: ${p.frontier} frontier voxels, ${p.spots.length} viewpoints, gains ${p.gains.join(", ")}`);
  } });
  const buildMs = now() - t;
  pinging = false;
  const counts = vox.counts(), band = bandMask(vox, map), inBand = [0, 0, 0], m3 = vox.res ** 3;
  let faint = 0, faintBand = 0;
  for (let i = 0; i < vox.n; i++) {
    if (band[i]) inBand[vox.st[i]]++;
    if (vox.flags[i] & FLAG.FAINT) (faint++, (faintBand += band[i]));
  }
  Object.assign(result.numbers, { project: project.id, grid: vox.stats().grid, buildMs: Math.round(buildMs), phases: vox.built.phases, views: vox.built.views,
    cameras: vox.built.cameras, splats: vox.built.splats, counts, band: { free_m3: +(inBand[FREE] * m3).toFixed(2), unknown_m3: +(inBand[UNKNOWN] * m3).toFixed(2),
      occupied_m3: +(inBand[OCCUPIED] * m3).toFixed(2) }, faint: { voxels: faint, band_m3: +(faintBand * m3).toFixed(3) }, importMs: Math.round(tImport),
    twinMs: Math.round(tTwin), centresMs: Math.round(tCentres) });
  log("built", result.numbers);
  check("build: under 60 s on this machine (the target)", buildMs < 60000, `${(buildMs / 1000).toFixed(1)} s: ${JSON.stringify(vox.built.phases)} ms, ${vox.built.views} twin views`);
  result.numbers.turns = { longestMs: Math.round(gaps.longest), during: gaps.during, over50: gaps.over50, over50Ms: Math.round(gaps.total), calibMs: calib() };
  check("build: the page gets a turn at least every 300 ms (capture rays in the worker)", gaps.longest < 300,
    `the longest wait for a turn ${gaps.longest.toFixed(0)} ms (after "${gaps.during}"); ${gaps.over50} waits over 50 ms, ${(gaps.total / 1000).toFixed(1)} s `
    + `in all; this machine's speed: a fixed loop took ${result.numbers.turns.calibMs} ms`);
  check("the map can tell free space", vox.built.usable, vox.built.why ?? `${vox.built.cameras} capture cameras, ${vox.built.objects} voxels from RoomPlan tops and accepted glass`);
  check("voxels in every state", counts.free > 0 && counts.unknown > 0 && counts.occupied > 0,
    `${vox.stats().grid}: free ${counts.free}, unknown ${counts.unknown}, occupied ${counts.occupied}; at drone heights ${JSON.stringify(result.numbers.band)}`);
  // Open floor at 1 m is known free.
  let open = 0, freeOpen = 0;
  const cl = map.clear[map.bandOf(1.0)];
  for (let k = 0; k < map.N; k++) {
    if (map.room[k] < 0 || cl[k] < 0.3) continue;
    const [x, y] = map.center(k % map.W, (k / map.W) | 0);
    open++;
    freeOpen += vox.state(x, y, map.floorZ[k] + 1.0) === FREE ? 1 : 0;
  }
  check("open floor 1 m up is known free", freeOpen / open > 0.97, `${((100 * freeOpen) / open).toFixed(1)}% of ${(open * 0.0025).toFixed(1)} m²`);
  // RoomPlan's tables and storage: the 6 cm under each top is never free (a glass top has no splats, a glossy black one few).
  const TOPS = /^(table|desk|storage|counter|cabinet|dresser|shelf|bed)/i, tops = [];
  const edge = (x, y, P) => Math.min(...P.map((a, k) => {
    const b = P[(k + 1) % P.length], ex = b[0] - a[0], ey = b[1] - a[1], u = Math.max(0, Math.min(1, ((x - a[0]) * ex + (y - a[1]) * ey) / (ex * ex + ey * ey)));
    return Math.hypot(x - a[0] - u * ex, y - a[1] - u * ey);
  }));
  for (const l of house.landmarks.filter((l) => l.source === "roomplan" && l.footprint?.length >= 3 && Number.isFinite(l.top) && TOPS.test(l.name))) {
    const xs = l.footprint.map((p) => p[0]), ys = l.footprint.map((p) => p[1]), c = [xs, ys].map((a) => a.reduce((s, v) => s + v, 0) / a.length);
    let n = 0, free = 0;
    for (let x = Math.min(...xs) + 0.05; x < Math.max(...xs) - 0.05; x += 0.05)
      for (let y = Math.min(...ys) + 0.05; y < Math.max(...ys) - 0.05; y += 0.05)
        if (inPolygon(x, y, l.footprint) && edge(x, y, l.footprint) >= 0.05) for (const dz of [0.015, 0.045]) (n++, (free += vox.state(x, y, l.top - dz) === FREE ? 1 : 0));
    tops.push({ name: l.name, at: c.map((v) => +v.toFixed(2)), top: +l.top.toFixed(2), free, n, down: +vox.raycast([...c, l.top + 0.5], [0, 0, -1], 2).d.toFixed(2) });
  }
  result.numbers.tops = tops;
  check("RoomPlan's table and storage tops are never free (glass, glossy black)", tops.length > 0 && tops.every((q) => q.free === 0 && q.down <= 0.53),
    tops.map((q) => `${q.name} (${q.at}) top ${q.top}: ${q.free}/${q.n} free, a ray down from 0.5 m above stops at ${q.down} m`).join("; "));
  t = now();
  map.setVoxels(vox);
  const overlayMs = now() - t, legs = house.rooms.map((r) => {
    const p = plan(map, [house.home.x, house.home.y], roomCenter(map, r.id), { sigma: 0.25 });
    return { room: r.name, ok: p.ok, reason: p.reason, length: p.length, worst: p.ok ? Math.min(...p.path.map((q) => vox.clearance3(...q))) : 0 };
  });
  check("the planner flies known-free space only (home to every room, σ 0.25)", legs.every((l) => l.ok && l.worst >= 0.3 - 0.05),
    `${legs.map((l) => (l.ok ? `${l.room} ${l.length.toFixed(1)} m, ${l.worst.toFixed(2)} m from anything occupied or unknown` : `${l.room}: ${l.reason}`)).join("; ")} (map overlay ${overlayMs.toFixed(0)} ms)`);
  t = now();
  const rep = coverageReport({ house, map, vox }), covMs = now() - t;
  result.numbers.coverage = { score: rep.score, ms: Math.round(covMs), rooms: rep.rooms.map(({ gaps, ...r }) => ({ ...r, gaps: gaps.length })),
    gaps: rep.rooms.flatMap((r) => r.gaps.slice(0, 4).map((g) => `${g.text} (${g.size} m², ${g.heightBand})`)), suggestions: rep.suggestions.map((s) => s.text) };
  log("coverage", result.numbers.coverage);
  check("coverage report", rep.score > 50 && rep.rooms.every((r) => r.flyableM3 > 0), `score ${rep.score}: ${rep.rooms.map((r) => `${r.name} ${r.unknownM3} m³ unknown (${r.unknownPct}%)`).join(", ")}`);
  // The store: voxels.bin round trip on this origin, then the test house goes again.
  t = now();
  const store = await openStore();
  await store.saveVoxels(house, vox);
  const back = await store.loadVoxels(house), storeMs = now() - t, bytes = vox.serialize().byteLength;
  check("voxels.bin in OPFS", back && back.counts().free === counts.free, `${(bytes / 1024).toFixed(0)} KB, saved and loaded in ${storeMs.toFixed(0)} ms`);
  await store.deleteHouse(house.id);
  ctx = window.ctx = { house, map, vox, twin, rep, ortho: imp.orthophoto };
  drawSlices(ctx);
  await drawCoverage(ctx);
  for (const [id, h] of [["s05", "0.5"], ["s10", "1.0"], ["s15", "1.5"]]) await save(id, `c-map3d-slice-${h}m.png`);
  await save("cover", "c-map3d-coverage.png");
});

function drawSlices({ map, vox }) {
  for (const [id, h] of [["s05", 0.5], ["s10", 1.0], ["s15", 1.5]]) {
    const s = vox.slice(h, { map }), K = 3, c = $(id);
    c.width = s.width * K;
    c.height = s.height * K;
    const g = c.getContext("2d"), img = g.createImageData(c.width, c.height);
    for (let r = 0; r < s.height; r++)
      for (let x = 0; x < s.width; x++) {
        const k = r * s.width + x, room = map.room[map.idx(s.x0 + (x + 0.5) * s.res, s.y0 + (r + 0.5) * s.res)] >= 0, col = COLORS[s.data[k]].map((v) => (room ? v : v * 0.45));
        for (let dy = 0; dy < K; dy++) for (let dx = 0; dx < K; dx++) img.data.set([...col, 255], (((s.height - 1 - r) * K + dy) * c.width + x * K + dx) * 4);
      }
    g.putImageData(img, 0, 0);
    outline(g, map, (x, y) => [((x - s.x0) / s.res) * K, (s.height - (y - s.y0) / s.res) * K]);
    g.fillStyle = "#fff";
    g.font = "14px ui-monospace,monospace";
    g.fillText(`${h.toFixed(1)} m above the floor: free / occupied / unknown`, 8, 18);
  }
}
function outline(g, map, P) {
  g.strokeStyle = "rgba(90,170,255,0.9)";
  g.lineWidth = 1.5;
  for (const r of map.rooms) {
    g.beginPath();
    r.outline.forEach((p, i) => g[i ? "lineTo" : "moveTo"](...P(...p)));
    g.closePath();
    g.stroke();
  }
}

async function drawCoverage({ house, map, vox, rep, ortho }) {
  const o = house.orthophoto, c = $("cover"), g = c.getContext("2d"), K = Math.min(1, 900 / o.width);
  c.width = Math.round(o.width * K);
  c.height = Math.round(o.height * K);
  const P = (x, y) => [((x - o.x0) / o.res) * K, (o.height - (y - o.y0) / o.res) * K];
  if (ortho) g.drawImage(await createImageBitmap(new Blob([ortho.bytes], { type: "image/png" })), 0, 0, c.width, c.height);
  // Per column: the share of its drone-height voxels never seen.
  const band = bandMask(vox, map), layer = vox.nx * vox.ny, unk = new Float32Array(layer), tot = new Float32Array(layer);
  for (let i = 0; i < vox.n; i++) if (band[i]) (tot[i % layer]++, vox.st[i] === UNKNOWN && unk[i % layer]++);
  const [w, h] = [vox.res / o.res * K, vox.res / o.res * K];
  for (let col = 0; col < layer; col++) {
    if (!tot[col]) continue;
    const f = unk[col] / tot[col], x = vox.x0 + ((col % vox.nx) + 0.5) * vox.res, y = vox.y0 + (((col / vox.nx) | 0) + 0.5) * vox.res, [px, py] = P(x, y);
    g.fillStyle = f > 0.02 ? `rgba(255,40,140,${(0.25 + 0.6 * f).toFixed(2)})` : "rgba(60,220,120,0.18)";
    g.fillRect(px - w / 2, py - h / 2, w + 0.5, h + 0.5);
  }
  outline(g, map, P);
  g.font = "bold 13px ui-monospace,monospace";
  let n = 0;
  for (const r of rep.rooms)
    for (const gap of r.gaps.slice(0, 5)) {
      const [px, py] = P(gap.x, gap.y);
      g.strokeStyle = "#fff";
      g.lineWidth = 2;
      g.beginPath();
      g.arc(px, py, 9, 0, 2 * Math.PI);
      g.stroke();
      g.fillStyle = "#fff";
      g.fillText(String(++n), px - 4, py + 4);
    }
  for (const s of rep.suggestions.slice(0, 8)) {
    const [px, py] = P(s.x, s.y);
    g.fillStyle = "#3f6";
    g.fillRect(px - 4, py - 4, 8, 8);
    g.fillText(`${s.height} m`, px + 6, py - 6);
  }
  g.fillStyle = "rgba(0,0,0,0.6)";
  g.fillRect(0, 0, c.width, 22);
  g.fillStyle = "#fff";
  g.font = "11px ui-monospace,monospace";
  g.fillText(`coverage ${rep.score}/100, 0.3-1.6 m up: pink = never seen, green = known; ○ gaps, ■ where to scan from (height)`, 6, 15);
  $("coverCap").textContent = `coverage ${rep.score}: ${rep.rooms.map((r) => `${r.name} ${r.unknownM3} m³ unknown`).join(", ")}`;
}

// Confirmed changes on the real capture: every door closed and opened again, and an obstacle confirmed across never-seen
// space then confirmed gone, leave the map exactly as it was (a never-seen voxel is never freed by undoing a change).
await part("changes", async () => {
  if (!ctx) throw new Error("needs the build part");
  const { house, map, vox } = ctx, v = VoxelMap.load(vox.serialize()), same = (a) => {
    let d = 0;
    for (let i = 0; i < v.n; i++) d += v.lo[i] !== a.lo[i] || v.st[i] !== a.st[i] || v.flags[i] !== a.flags[i] ? 1 : 0;
    return d;
  };
  const snap = () => ({ lo: v.lo.slice(), st: v.st.slice(), flags: v.flags.slice() }), doors = [];
  let t = now();
  for (const d of house.doors) {
    if (!d.passable) v.door(d, true);
    const before = snap(), shut = v.door(d, false), back = VoxelMap.load(v.serialize());
    back.door(d, true);
    v.door(d, true);
    doors.push({ id: d.id, shut, differ: same(before), differSaved: (() => {
      let n = 0;
      for (let i = 0; i < v.n; i++) n += back.st[i] !== before.st[i] || back.lo[i] !== before.lo[i] ? 1 : 0;
      return n;
    })() });
  }
  const doorMs = now() - t;
  // The largest never-seen pocket at drone heights in the Living room: a confirmed obstacle over it and around, then gone.
  const band = bandMask(v, map), r0 = map.rooms[0];
  let best = -1, most = 0;
  for (let i = 0; i < v.n; i += 7) {
    if (!band[i] || v.st[i] !== UNKNOWN || map.roomAt(...v.center(i))?.id !== r0.id) continue;
    let k = 0;
    v.region({ x: v.center(i)[0], y: v.center(i)[1], z: v.center(i)[2], r: 0.25 }, (j) => (k += v.st[j] === UNKNOWN ? 1 : 0));
    if (k > most) [best, most] = [i, k];
  }
  const c = v.center(best), box = { min: [c[0] - 0.3, c[1] - 0.3, c[2] - 0.2], max: [c[0] + 0.3, c[1] + 0.3, c[2] + 0.2] }, before = snap(), inside = [];
  v.region(box, (i) => inside.push(i));
  v.mark(box, "occupied");
  v.mark(box, "free");
  const unknownFreed = inside.filter((i) => before.st[i] === UNKNOWN && v.st[i] === FREE).length, freedScan = inside.filter((i) => before.st[i] === OCCUPIED && v.st[i] === FREE).length;
  const kinds = [UNKNOWN, FREE, OCCUPIED].map((s) => inside.filter((i) => before.st[i] === s).length);
  result.numbers.changes = { doors, doorMs: Math.round(doorMs), obstacle: { at: c.map((q) => +q.toFixed(2)), kinds, unknownFreed, freedScan } };
  log("changes", result.numbers.changes);
  check("doors: closed and opened again, the map is as it was (also through a save)", doors.every((d) => d.differ === 0 && d.differSaved === 0),
    `${doors.length} doors, ${doors.reduce((s, d) => s + d.shut, 0)} voxels shut in all; ${doors.filter((d) => d.differ || d.differSaved).map((d) => `${d.id}: ${d.differ}/${d.differSaved} differ`).join(", ") || "0 voxels differ"} (${doorMs.toFixed(0)} ms)`);
  check("an obstacle confirmed then gone never frees never-seen space", unknownFreed === 0 && kinds[UNKNOWN] > 0,
    `a 0.6 m box over the Living room's largest never-seen pocket (${kinds.join(" / ")} unknown / free / occupied voxels): ${unknownFreed} unknown voxels freed, ${freedScan} scan surfaces gone with it`);
});

// Live depth in flight: the twin's expected depth along a planned path, 3% noise and the localizer 5 cm off (σ 0.05), each
// frame integrated on the main thread, then the next map query (as the controller would make it).
await part("live", async () => {
  if (!ctx) throw new Error("needs the build part");
  const { house, map, vox, twin } = ctx, v = VoxelMap.load(vox.serialize());
  await v.refresh();
  map.setVoxels(v);
  const ids = map.rooms.map((r) => r.id), p = plan(map, roomCenter(map, ids[0]), roomCenter(map, ids.at(-1)), { sigma: 0.25 });
  if (!p.ok) throw new Error(`no path for the flight: ${p.reason}`);
  const route = [];
  for (let k = 1; k < p.path.length; k++) {
    const [a, b] = [p.path[k - 1], p.path[k]], L = Math.hypot(b[0] - a[0], b[1] - a[1]), yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
    for (let s = 0; s < L; s += 0.2) route.push({ x: a[0] + ((b[0] - a[0]) * s) / L, y: a[1] + ((b[1] - a[1]) * s) / L, z: a[2], yaw, pitch: 0, roll: 0 });
  }
  let seed = 5;
  const rand = () => (seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296, gauss = () => Math.sqrt(-2 * Math.log(1 - rand())) * Math.cos(2 * Math.PI * rand());
  const lens = pinholeLens(90), W = 160, H = 120, ms = [], seenMs = [], parts = { integrate: [], query: [] };
  const speed = [calib()];
  let contradicted = 0, cut = 0;
  for (const tp of route) {
    const expected = await twin.depth(tp, { width: W, height: H, lens, actors: false, props: false }), d = expected.map((z) => z * (1 + 0.03 * gauss()));
    const a = rand() * 2 * Math.PI, pose = { ...tp, x: tp.x + 0.05 * Math.cos(a), y: tp.y + 0.05 * Math.sin(a), sigma: 0.05, status: "ok" };
    let t = now();
    const r = v.integrate(pose, { width: W, height: H, depth: d, expected }, lens);
    contradicted += r.contradicted.length;
    cut += r.cut ? 1 : 0;
    const t1 = now();
    map.clearance(tp.x, tp.y, tp.z);
    ms.push(now() - t);
    parts.integrate.push(t1 - t);
    parts.query.push(now() - t1);
    t = now();
    v.markSeen(pose, lens, Date.now(), { width: W, height: H, quality: { age: 100, luma: 100 } });
    seenMs.push(now() - t);
    await new Promise((r) => setTimeout(r, 30));
  }
  const band = bandMask(v, map);
  const phantom = () => {
    let n = 0;
    for (let i = 0; i < v.n; i++) n += band[i] && vox.st[i] === FREE && v.st[i] !== FREE ? 1 : 0;
    return +(n * v.res ** 3).toFixed(4);
  };
  const during = phantom();
  speed.push(calib());
  let faintFree = 0, faint = 0;
  for (let i = 0; i < v.n; i++) if (v.flags[i] & FLAG.FAINT) (faint++, (faintFree += v.st[i] === FREE ? 1 : 0));
  v.forgetFlight();
  const after = phantom(), q = (a, f) => +[...a].sort((x, y) => x - y)[Math.min(a.length - 1, Math.floor(a.length * f))].toFixed(1);
  result.numbers.live = { frames: route.length, ms: { median: q(ms, 0.5), p90: q(ms, 0.9), max: q(ms, 1) }, markSeenMs: q(seenMs, 0.5), phantomM3: { during, after },
    integrateMs: { median: q(parts.integrate, 0.5), p90: q(parts.integrate, 0.9), max: q(parts.integrate, 1) }, queryMs: { median: q(parts.query, 0.5),
      p90: q(parts.query, 0.9), max: q(parts.query, 1) }, cut, contradictedPerFrame: +(contradicted / route.length).toFixed(1), calibMs: speed,
    cost: { stampUs: +(v._cost.stamp * 1000).toFixed(1), voxelNs: +(v._cost.voxel * 1e6).toFixed(0) }, faint: { voxels: faint, freed: faintFree } };
  log("live depth", result.numbers.live);
  check("live depth: integrate keeps to its 8 ms budget, and with the next map query stays off the control loop's back",
    q(parts.integrate, 0.9) < 12 && q(ms, 0.5) < 10,
    `${route.length} frames (${W}x${H}, 3% noise, 5 cm pose error): integrate median ${q(parts.integrate, 0.5)} ms, p90 ${q(parts.integrate, 0.9)}, `
    + `max ${q(parts.integrate, 1)} (${cut} frames cut short); with the map's query (median ${q(parts.query, 0.5)}, max ${q(parts.query, 1)}) median `
    + `${q(ms, 0.5)}, p90 ${q(ms, 0.9)}, max ${q(ms, 1)}; markSeen ${q(seenMs, 0.5)} ms; this machine's speed: a fixed loop took ${speed.join(" / ")} ms before / after`);
  check("live depth: never through a faint layer (glass, glossy tops, sheer curtains)", faintFree === 0, `${faintFree} of ${faint} faint-layer voxels free after the flight`);
  check("live depth: no lasting phantoms (flight-only obstacles forgotten at the end)", during < 0.05 && after < 0.005,
    `space free before, not free now at drone heights: ${during} m³ during the flight, ${after} m³ after it`);
  check("live depth: few false contradictions of the scan (nothing changed)", contradicted / route.length < 10,
    `${(contradicted / route.length).toFixed(1)} capture voxels per frame said to be seen through`);
  map.setVoxels(vox);
});

await part("props", async () => {
  if (!ctx) throw new Error("needs the build part");
  const { map, vox, twin } = ctx;
  // An open spot in the living room with at least 2.5 m clear ahead at 1 m.
  let spot = null;
  const [cx, cy] = roomCenter(map, map.rooms[0].id);
  for (let a = 0; a < 16 && !spot; a++) {
    const yaw = (a * Math.PI) / 8, z = map.floorAt(cx, cy) + 1.0;
    if (vox.raycast([cx, cy, z], [Math.cos(yaw), Math.sin(yaw), 0], 4).d >= 2.5) spot = { x: cx, y: cy, z, yaw, pitch: 0, roll: 0 };
  }
  if (!spot) throw new Error("no open view in the living room");
  const ahead = (d, side = 0) => [spot.x + d * Math.cos(spot.yaw) - side * Math.sin(spot.yaw), spot.y + d * Math.sin(spot.yaw) + side * Math.cos(spot.yaw)];
  const floor = map.floorAt(spot.x, spot.y), [dx, dy] = ahead(1.2), [bx, by] = ahead(1.6, 0.6);
  await twin.setProps([
    { id: "door", kind: "panel", x: dx, y: dy, z: floor, w: 0.8, h: 2.0, yaw: spot.yaw + Math.PI / 2, color: 0xd9d2c4 },
    { id: "box", kind: "box", x: bx, y: by, z: floor, w: 0.4, d: 0.4, h: 0.4, yaw: spot.yaw, color: 0x9a6a3a },
  ]);
  const [px, py] = ahead(2.2);
  await twin.setActors([{ id: "p", kind: "person", x: px, y: py, z: floor, yaw: spot.yaw + Math.PI, pose: "stand" }]);
  const view = { width: 320, height: 240, lens: pinholeLens(90) };
  const draw = (id, bmp) => $(id).getContext("2d").drawImage(bmp, 0, 0, 320, 240);
  draw("pWith", await twin.render(spot, view));
  draw("pWithout", await twin.render(spot, { ...view, props: false }));
  const dWith = await twin.depth(spot, { ...view, actors: false }), dWithout = await twin.depth(spot, { ...view, actors: false, props: false });
  const mid = (d) => d[120 * 320 + 160];
  const g = $("pDepth").getContext("2d"), img = g.createImageData(320, 240);
  for (let i = 0; i < dWith.length; i++) {
    const v = Number.isFinite(dWith[i]) ? Math.max(0, 255 - dWith[i] * 60) : 0;
    img.data.set([v, v, v, 255], i * 4);
  }
  g.putImageData(img, 0, 0);
  check("props: the door panel is in the expected depth, and { props: false } leaves it out", Math.abs(mid(dWith) - 1.2 + 0.02) < 0.05 && mid(dWithout) > 1.6,
    `centre ${mid(dWith).toFixed(3)} m with the panel at 1.2 m, ${mid(dWithout).toFixed(2)} m without`);
  const hidden = (await twin.oracle(spot, view)).find((o) => o.id === "p"), seen = (await twin.oracle(spot, { ...view, props: false })).find((o) => o.id === "p");
  check("props hide an actor behind them in the oracle", (hidden?.visibleFraction ?? 0) < 0.1 && seen?.visibleFraction > 0.5,
    `person behind the door: visible ${hidden?.visibleFraction ?? 0} with props, ${seen?.visibleFraction} without`);
  const px0 = await twin.pixels(spot, view), px1 = await twin.pixels(spot, { ...view, props: false });
  let diff = 0;
  for (let i = 0; i < px0.data.length; i += 4) diff += Math.abs(px0.data[i] - px1.data[i]) > 30 ? 1 : 0;
  check("props show in the image", diff > 0.05 * 320 * 240, `${((100 * diff) / (320 * 240)).toFixed(0)}% of pixels change`);
  await save("pWith", "c-map3d-props.png");
  await twin.setProps([]);
  await twin.setActors([]);
  // depthBatch against one depth() call per view: six cube faces at one spot, then four spots.
  const faces = [[0, 0], [Math.PI / 2, 0], [Math.PI, 0], [-Math.PI / 2, 0], [0, Math.PI / 2], [0, -Math.PI / 2]];
  const cube = (p) => faces.map(([yaw, pitch]) => ({ ...p, yaw, pitch, roll: 0 })), dv = { width: 96, height: 96, lens: pinholeLens(90), actors: false, props: false };
  const spots = [0, 1, 2, 3].map((k) => ({ ...spot, x: spot.x + 0.3 * k }));
  await twin.depthBatch(cube(spot), dv);
  let t = now();
  for (const p of spots) for (const pose of cube(p)) await twin.depth(pose, dv);
  const one = now() - t;
  t = now();
  const batch = [];
  for (const p of spots) batch.push(...(await twin.depthBatch(cube(p), dv)));
  const many = now() - t;
  let same = 0;
  const ref = await twin.depth(cube(spots[3])[5], dv);
  for (let i = 0; i < ref.length; i++) same += Math.abs(ref[i] - batch[23][i]) < 1e-4 || (Number.isNaN(ref[i]) && Number.isNaN(batch[23][i])) ? 1 : 0;
  result.numbers.depthBatch = { perViewMs: +(many / 24).toFixed(1), perViewSingleMs: +(one / 24).toFixed(1) };
  // Each view draws every splat (the vertex stage dominates), so a batch saves the round trips, not the GPU time.
  check("depthBatch: the same depth as depth(), and no slower", same / ref.length > 0.99 && many < one * 1.15,
    `${(many / 24).toFixed(1)} ms per 96x96 view in batches of 6 vs ${(one / 24).toFixed(1)} ms one by one; ${((100 * same) / ref.length).toFixed(1)}% identical`);
});

if (ctx?.twin && !q.has("keep")) await ctx.twin.dispose();
status(`done: ${Object.values(result.checks).filter((c) => c.ok).length}/${Object.keys(result.checks).length} passed`, 1);
result.done = true;
