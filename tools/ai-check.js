// Browser check for the AI survey (app/js/ai/survey.js) on a real capture: import through the server, the splat twin
// renders each room's views (level, and down at the floor and up at the ceiling where a level camera misses them, plus
// views from other spots where the measured floor or ceiling fell short), a
// fake Claude (no network: a fetch stand-in) answers every request by boxing the capture's own objects (RoomPlan furniture
// with footprints, the fan the map found, curtains, stairs) the way a model would without knowing the scan's depth: the
// object's 3D box projected into the view that shows it largest (its middle unhidden, at least half of it in the
// picture), each edge jittered by up to ?jitter= (default 0.1) of the box's size; and the survey lifts the boxes to 3D.
// Then: where they landed against the objects' footprints (by kind), on the orthophoto with the floor no view saw, the
// views with the boxes, costs (plumbing only: the fake's usage, not a real model's thinking). Results in window.result.
import { importCapture } from "../app/js/house/import.js";
import { houseSource } from "../app/js/ui/housepanel.js";
import { Twin } from "../app/js/twin/twin.js";
import { Claude } from "../app/js/ai/claude.js";
import * as S from "../app/js/ai/survey.js";
import { pinholeLens, intrinsics, project, unproject } from "../app/js/twin/lens.js";
import { droneCamera } from "../app/js/twin/pose.js";
import { inPolygon } from "../app/js/house/homemap.js";

const $ = (id) => document.getElementById(id);
const q = new URLSearchParams(location.search);
const lines = [];
const log = (...a) => { lines.push(a.map((x) => (typeof x === "string" ? x : JSON.stringify(x))).join(" ")); $("log").textContent = lines.join("\n"); };
const result = (window.result = { done: false, checks: {} });
function check(name, ok, detail) {
  result.checks[name] = { ok, detail };
  const tr = $("checks").insertRow();
  tr.insertCell().textContent = ok ? "PASS" : "FAIL";
  tr.cells[0].className = ok ? "pass" : "fail";
  tr.insertCell().textContent = name;
  tr.insertCell().textContent = detail;
}
const W = S.SURVEY.width, H = S.SURVEY.height, lens = pinholeLens(S.SURVEY.hfov), K = intrinsics(lens, W, H), JITTER = +(q.get("jitter") ?? 0.1);
// A seeded generator per (object, view): the same boxes on every run.
const rand = (seed) => () => ((seed = (seed * 1664525 + 1013904223) >>> 0) / 2 ** 32);
const toCam = (cam, P) => { const r = [0, 1, 2].map((k) => P[k] - cam.p[k]); return [0, 1, 2].map((k) => cam.R[0][k] * r[0] + cam.R[1][k] * r[1] + cam.R[2][k] * r[2]); };

// The capture's own objects, as a careful model would name them.
const NAMES = [[/sofa|couch/i, "sofa"], [/^table$/i, "table"], [/chair/i, "chair"], [/storage|cabinet|dresser/i, "cabinet"], [/fireplace/i, "fireplace"], [/bed$/i, "bed"], [/tv|television/i, "tv"], [/desk/i, "desk"]];
const HAZARD = [[/^ceiling fan$/i, "ceiling-fan"], [/curtain|shade|blind/i, "curtain"], [/^stairs?$/i, "stairs"], [/plant/i, "plant"], [/pendant|chandelier/i, "hanging-lamp"]];
function groundTruth(house) {
  const out = [];
  for (const l of house.landmarks ?? []) {
    const kind = HAZARD.find(([re]) => re.test(l.name))?.[1], name = kind ? null : NAMES.find(([re]) => re.test(l.name))?.[1];
    if (!kind && !name) continue;
    const box = l.footprint && l.size && l.top != null ? { foot: l.footprint, zMin: l.top - l.size[2], zMax: l.top }
      : Number.isFinite(l.z) ? { foot: [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([a, b]) => [l.x + 0.35 * a, l.y + 0.35 * b]), zMin: l.z - 0.25, zMax: l.z + 0.25 } : null;
    if (box) out.push({ id: out.length, name, kind, label: l.name, source: l.source, x: l.x, y: l.y, ...box });
  }
  return out;
}

// The object's box in a view, as a model would draw it without the scan's depth: its 3D box (footprint, heights)
// projected, when its middle shows (the twin's depth along that ray reaches the object) and at least half the box is in
// the picture, then each edge jittered by up to JITTER of the box's size. The depth only says what a model sees in the
// picture: how much of the box something nearer hides (area: the part in the picture not hidden; the prompt asks for
// the view where a thing is clearest, largest and least hidden).
function boxIn(view, o, vi) {
  const cam = droneCamera(view.pose, 0), corners = o.foot.flatMap(([x, y]) => [[x, y, o.zMin], [x, y, o.zMax]]).map((P) => toCam(cam, P));
  if (corners.some((c) => c[2] < 0.15)) return null;
  const px = corners.map((c) => project(K, c)), xs = px.map((p) => p[0]), ys = px.map((p) => p[1]);
  const b = [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)], cb = [Math.max(0, b[0]), Math.max(0, b[1]), Math.min(W, b[2]), Math.min(H, b[3])];
  const area = (b[2] - b[0]) * (b[3] - b[1]), carea = Math.max(0, cb[2] - cb[0]) * Math.max(0, cb[3] - cb[1]);
  if (carea < 0.5 * area || cb[2] - cb[0] < 24 || cb[3] - cb[1] < 24) return null;
  const c3 = [o.foot.reduce((a, p) => a + p[0], 0) / o.foot.length, o.foot.reduce((a, p) => a + p[1], 0) / o.foot.length, (o.zMin + o.zMax) / 2];
  const cc = toCam(cam, c3), pc = project(K, cc), u = Math.floor((pc[0] / W) * view.dw), v = Math.floor((pc[1] / H) * view.dh);
  if (!(u >= 0 && v >= 0 && u < view.dw && v < view.dh)) return null;
  const half = Math.hypot(Math.max(...o.foot.map(([x, y]) => Math.hypot(x - c3[0], y - c3[1]))), (o.zMax - o.zMin) / 2), reach = Math.hypot(...cc) - half - 0.15;
  if (!(view.depth[v * view.dw + u] >= reach)) return null; // something else in front of its middle
  const near = Math.min(...corners.map((c) => Math.hypot(...c))) - 0.15;
  let seen = 0, all = 0;
  for (let y = cb[1] + 0.5; y < cb[3]; y += (cb[3] - cb[1]) / 12)
    for (let x = cb[0] + 0.5; x < cb[2]; x += (cb[2] - cb[0]) / 12) (all++, (seen += view.depth[Math.floor((y / H) * view.dh) * view.dw + Math.floor((x / W) * view.dw)] >= near ? 1 : 0));
  if (seen < 0.5 * all) return null; // mostly hidden
  const r = rand(o.id * 7919 + vi * 104729 + 17), j = (d) => (2 * r() - 1) * JITTER * d, bw = cb[2] - cb[0], bh = cb[3] - cb[1];
  const box = [cb[0] + j(bw), cb[1] + j(bh), cb[2] + j(bw), cb[3] + j(bh)].map((x, k) => Math.round(Math.max(0, Math.min(k % 2 ? H : W, x))));
  return { box, area: (carea * seen) / all, hidden: +(1 - seen / all).toFixed(2) };
}

async function main() {
  const projects = (await (await fetch("/house-files/")).json()).projects ?? [];
  const project = projects.find((p) => p.id === q.get("project")) ?? projects.find((p) => p.ready);
  if (!project) throw new Error("no capture with a 3D scan under /house-files/");
  log("capture:", project.name, `(${project.id})`);
  let t0 = performance.now();
  const imp = await importCapture(houseSource(project, ({ path, loaded, total }) => ($("orthoCap").textContent = `loading ${path} ${Math.round((100 * loaded) / total)}%`)));
  const { house, map } = imp;
  log(`imported in ${((performance.now() - t0) / 1000).toFixed(1)} s: ${house.rooms.length} rooms (${house.rooms.map((r) => r.name).join(", ")}), ${house.landmarks.length} landmarks`);
  t0 = performance.now();
  const twin = await Twin.create({ splat: imp.splat.bytes, house });
  log(`twin ready in ${((performance.now() - t0) / 1000).toFixed(1)} s`);
  const truth = groundTruth(house);
  log(`ground truth: ${truth.map((o) => o.name ?? o.kind).join(", ")}`);

  // The twin, recording the views it renders in colour (those are sent) with their depth (the fake Claude uses it to see
  // what is hidden).
  const views = [], depths = new Map();
  const rec = {
    pixels: (pose, view) => (views.push({ pose, ...depths.get(pose) }), twin.pixels(pose, view)),
    depth: async (pose, view) => {
      const depth = await twin.depth(pose, view);
      depths.set(pose, { depth, dw: view.width, dh: view.height });
      return depth;
    },
  };
  // The fake Claude: the request's pictures are the last views rendered; each object goes in the view that shows it largest.
  const asked = [], reported = [];
  const fakeFetch = async (url, init) => {
    const body = JSON.parse(init.body), content = body.messages[0].content, images = content.filter((b) => b.type === "image");
    const room = house.rooms.find((r) => content[0].text.startsWith(`This is ${r.name} (`)), mine = views.slice(-images.length);
    const found = truth.map((o) => {
      let best = null;
      mine.forEach((v, i) => {
        const b = boxIn(v, o, i);
        if (b && (!best || b.area > best.area)) best = { ...b, view: i + 1 };
      });
      return best && { o, ...best };
    }).filter(Boolean);
    for (const f of found) reported.push({ room: room.id, view: f.view, box: f.box, o: f.o });
    const has = (n) => found.some((f) => (f.o.name ?? f.o.kind) === n && map.roomAt(f.o.x, f.o.y)?.id === room.id);
    const [kind, name] = has("sofa") ? ["living room", "Living room"] : has("stairs") ? ["stairwell", "Stair hall"] : ["entrance", "Entry"];
    const answer = {
      room: { kind, name, confidence: 0.8 },
      landmarks: found.filter((f) => f.o.name).map((f) => ({ name: f.o.name, view: f.view, box: f.box, confidence: 0.85 })),
      hazards: found.filter((f) => f.o.kind).map((f) => ({ kind: f.o.kind, view: f.view, box: f.box, confidence: 0.8, why: `the capture lists "${f.o.label}"` })),
    };
    asked.push({ room: room.id, images: images.length, level: mine.filter((v) => v.pose.look === "level").length, more: mine.filter((v) => v.pose.more).length, kb: Math.round(init.body.length / 1024), system: body.system[0].text.length, format: body.output_config?.format?.type });
    const usage = { input_tokens: images.length * 590 + Math.round(content.filter((b) => b.type === "text").reduce((a, b) => a + b.text.length, 0) / 4),
      cache_creation_input_tokens: asked.length === 1 ? Math.round(body.system[0].text.length / 4) : 0, cache_read_input_tokens: asked.length === 1 ? 0 : Math.round(body.system[0].text.length / 4), output_tokens: 1100 };
    return new Response(JSON.stringify({ id: `msg_${asked.length}`, type: "message", role: "assistant", model: body.model, stop_reason: "end_turn",
      content: [{ type: "text", text: JSON.stringify(answer) }], usage }), { status: 200, headers: { "content-type": "application/json" } });
  };
  const values = { apiKey: "fake-key-for-the-check", model: "claude-opus-5", aiVision: "on", aiBudget: 0.5 };
  const settings = { get: (k) => values[k], set: (k, v) => (values[k] = v) };
  const claude = new Claude({ settings, fetch: fakeFetch });
  const rooms = q.get("rooms")?.split(",") ?? null;
  const est = S.estimateSurvey({ house, map, settings, rooms });
  log("estimate:", est.text);
  t0 = performance.now();
  const res = await S.surveyHouse({ house, map, twin: rec, settings, claude, rooms, estimate: est, onProgress: (p) => p.text && log(p.text) });
  const secs = (performance.now() - t0) / 1000;
  log("survey:", S.surveySummary(res, house));

  // Where the boxes landed against the objects.
  const footDist = (p, o) => {
    if (inPolygon(p.x, p.y, o.foot)) return 0;
    let best = Infinity;
    o.foot.forEach((a, i) => {
      const b = o.foot[(i + 1) % o.foot.length], ax = b[0] - a[0], ay = b[1] - a[1], k = Math.max(0, Math.min(1, ((p.x - a[0]) * ax + (p.y - a[1]) * ay) / (ax * ax + ay * ay || 1)));
      best = Math.min(best, Math.hypot(a[0] + k * ax - p.x, a[1] + k * ay - p.y));
    });
    return best;
  };
  const items = [...res.landmarks.map((l) => ({ ...l, what: l.name })), ...res.hazards.map((h) => ({ ...h, what: h.kind }))].map((it) => {
    const src = it.views.map((v) => reported.find((r) => r.room === v.room && r.view === v.view && r.box.join() === v.box.join())?.o).filter(Boolean);
    const errs = src.map((o) => footDist(it, o)), o = src[errs.indexOf(Math.min(...errs))];
    return { ...it, truth: o, err: o ? Math.min(...errs) : null, zOk: o ? it.z >= o.zMin - 0.25 && it.z <= o.zMax + 0.25 : null };
  });
  const placed = items.filter((i) => i.truth), errs = placed.map((i) => i.err).sort((a, b) => a - b);
  const median = errs.length ? errs[errs.length >> 1] : null, within = placed.filter((i) => i.err <= 0.3).length;
  Object.assign(result, {
    capture: project.id, rooms: house.rooms.length, estimate: est.total, cost: res.cost, seconds: +secs.toFixed(1), asked, unplaced: res.unplaced, reported: reported.length,
    items: items.map((i) => ({ what: i.what, room: i.room, x: i.x, y: i.y, z: i.z, r: i.r, views: i.views, err: i.err && +i.err.toFixed(3), zOk: i.zOk, known: i.known ?? null, truth: i.truth?.label,
      foot: i.truth && [i.truth.x, i.truth.y, +i.truth.zMin.toFixed(2), +i.truth.zMax.toFixed(2)] })),
    errors: { median: median && +median.toFixed(3), max: errs.length ? +errs.at(-1).toFixed(3) : null, within30cm: `${within}/${placed.length}` },
  });
  const surveyed = rooms ? rooms.length : house.rooms.length;
  check(`one request per room: 4-8 level pictures, at most ${S.SURVEY.maxExtra} planned at the floor or ceiling and ${S.SURVEY.maxMore} more from other spots, JSON-schema answer`,
    asked.length === surveyed && asked.every((a) => a.level >= 4 && a.level <= 8 && a.images - a.level - a.more <= S.SURVEY.maxExtra && a.more <= S.SURVEY.maxMore && a.format === "json_schema"),
    asked.map((a) => `${a.room}: ${a.level} level + ${a.images - a.level - a.more} pitched + ${a.more} from other spots, ${a.kb} KB`).join("; "));
  check("every room answered", res.rooms.every((r) => !r.error), res.rooms.map((r) => `${r.name} → ${r.suggestedName ?? r.error}`).join("; "));
  check("objects boxed by the fake Claude are placed (not dropped)", res.unplaced <= Math.max(1, reported.length * 0.15), `${reported.length} boxes, ${res.unplaced} not placed, ${items.length} after merging`);
  const kinds = {};
  for (const i of placed) {
    const k = (kinds[i.what] ??= { n: 0, within: 0, errs: [] });
    k.n++;
    k.within += i.err <= 0.3 ? 1 : 0;
    k.errs.push(+i.err.toFixed(2));
  }
  result.byKind = kinds;
  check("lifted points land on their object's footprint (≤ 0.3 m) for 80%", placed.length > 0 && within >= 0.8 * placed.length, `median ${median?.toFixed(2)} m, max ${errs.at(-1)?.toFixed(2)} m, ${within}/${placed.length} within 0.3 m; by kind: ${Object.entries(kinds).map(([k, v]) => `${k} ${v.within}/${v.n}`).join(", ")}`);
  check("heights inside the object's (±0.25 m)", placed.every((i) => i.zOk), placed.filter((i) => !i.zOk).map((i) => `${i.what} z ${i.z} vs ${i.truth.zMin.toFixed(2)}..${i.truth.zMax.toFixed(2)}`).join("; ") || "all");
  const off = Math.abs(est.total.cost - res.cost.cost) / res.cost.cost;
  check("cost plumbing: estimate vs the fake's reported usage (not a real model's thinking)", off < 0.3, `estimate $${est.total.cost.toFixed(3)}, reported $${res.cost.cost.toFixed(3)} (${Math.round(off * 100)}% off); ${est.perImage} tokens a picture, ${est.total.images} pictures`);
  const unseen = res.rooms.map((r) => ({ room: r.id, floor: r.unseen?.floorShare, ceiling: r.unseen?.ceilingShare, seen: r.seen, more: r.more, text: r.unseen?.text }));
  result.unseen = unseen;
  check("the floor and ceiling the views saw, measured from their depth (with views from other spots where the first ones fell short)", res.rooms.every((r) => r.unseen && r.unseen.floorShare <= 0.35),
    unseen.map((u) => `${u.room}: floor ${Math.round(100 * u.seen.floor)}% seen, ceiling ${Math.round(100 * u.seen.ceiling)}%${u.more ? ` (${u.more} view${u.more > 1 ? "s" : ""} from other spots)` : ""}`).join("; "));
  check("returned poses carry no planned shares (rooms[].seen is what the depth showed)", res.views.every((v) => !("planned" in v.pose) && !("coverage" in v.pose)), `${res.views.length} views`);
  check("survey time", true, `${secs.toFixed(1)} s for ${res.views.length} views (twin renders + depth + JPEG)`);

  // Pictures: the orthophoto with what landed where, and the views with the boxes.
  await drawOrtho(imp, house, map, res, items, truth);
  await drawViews(res, reported);
  const table = $("items");
  table.insertRow().innerHTML = "<th>thing</th><th>room</th><th>x, y, z</th><th>r</th><th>views</th><th>off its footprint</th><th>from the capture's</th>";
  for (const i of items) {
    const tr = table.insertRow();
    for (const t of [i.what + (i.known ? ` (known: ${i.known})` : ""), house.rooms.find((r) => r.id === i.room)?.name, `${i.x}, ${i.y}, ${i.z}`, i.r, i.views.length, i.err == null ? "-" : `${i.err.toFixed(2)} m`, i.truth?.label ?? "-"]) tr.insertCell().textContent = t;
  }
  const sink = q.get("sink");
  if (sink) for (const id of ["ortho", "views"]) {
    const blob = await new Promise((r) => $(id).toBlob(r, "image/png"));
    await fetch(new URL(`${id}.png`, sink), { method: "POST", body: blob }).catch((e) => log(`sink: ${e.message}`));
  }
  if (sink) await fetch(new URL("result.json", sink), { method: "POST", body: JSON.stringify(result, null, 1) }).catch(() => {});
  if (q.has("keep")) window.dbg = { imp, house, map, twin, res, reported, truth, views, S }; // ?keep: left for inspection
  else await twin.dispose();
}

async function drawOrtho(imp, house, map, res, items, truth) {
  const c = $("ortho"), g = c.getContext("2d"), o = house.orthophoto;
  const img = imp.orthophoto && (await createImageBitmap(new Blob([imp.orthophoto.bytes], { type: "image/png" })));
  const s = Math.min(1100 / o.width, 900 / o.height);
  c.width = Math.round(o.width * s);
  c.height = Math.round(o.height * s);
  if (img) g.drawImage(img, 0, 0, c.width, c.height);
  g.fillStyle = "rgba(0,0,0,0.35)";
  g.fillRect(0, 0, c.width, c.height);
  const P = (x, y) => [((x - o.x0) / o.res) * s, (o.height - (y - o.y0) / o.res) * s], px = 1 / o.res * s;
  const poly = (pts, stroke, w = 1, dash = []) => {
    g.beginPath();
    pts.forEach(([x, y], i) => g[i ? "lineTo" : "moveTo"](...P(x, y)));
    g.closePath();
    g.setLineDash(dash);
    g.strokeStyle = stroke;
    g.lineWidth = w;
    g.stroke();
    g.setLineDash([]);
  };
  for (const r of house.rooms) poly(r.outline, "rgba(160,180,255,0.6)", 1, [6, 4]);
  for (const t of truth) poly(t.foot, "rgba(255,255,255,0.85)", 1.5);
  g.font = "12px ui-monospace,monospace";
  for (const r of res.rooms) for (const p of r.unseen?.patches ?? []) {
    const [x, y] = P(p.x, p.y);
    g.fillStyle = p.what === "floor" ? "rgba(255,60,60,0.35)" : "rgba(160,90,255,0.3)";
    g.beginPath();
    g.arc(x, y, Math.sqrt(p.m2 / Math.PI) * px, 0, 2 * Math.PI);
    g.fill();
  }
  for (const v of res.views) {
    const [x, y] = P(v.pose.x, v.pose.y);
    g.strokeStyle = v.pose.look === "down" ? "#f6a" : v.pose.look === "up" ? "#a8f" : "#fd4";
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(x, y);
    g.lineTo(x + 0.6 * px * Math.cos(v.pose.yaw), y - 0.6 * px * Math.sin(v.pose.yaw));
    g.stroke();
    g.fillStyle = v.pose.more ? "#4ff" : "#fd4";
    g.fillRect(x - 3, y - 3, 6, 6);
  }
  for (const it of items) {
    const [x, y] = P(it.x, it.y), hz = !it.name;
    if (it.truth) {
      const t = it.truth, cx = t.foot.reduce((a, p) => a + p[0], 0) / t.foot.length, cy = t.foot.reduce((a, p) => a + p[1], 0) / t.foot.length;
      g.strokeStyle = "rgba(255,255,255,0.5)";
      g.beginPath();
      g.moveTo(x, y);
      g.lineTo(...P(cx, cy));
      g.stroke();
    }
    g.strokeStyle = g.fillStyle = hz ? "#f93" : "#3df";
    g.lineWidth = 2;
    g.beginPath();
    if (hz) g.arc(x, y, Math.max(4, it.r * px), 0, 2 * Math.PI);
    else g.arc(x, y, 5, 0, 2 * Math.PI);
    hz ? g.stroke() : g.fill();
    const label = `${it.what}${it.views.length > 1 ? ` ×${it.views.length}` : ""}${it.known ? " (known)" : ""}`;
    g.fillStyle = "rgba(0,0,0,0.7)";
    g.fillRect(x + 7, y - 13, g.measureText(label).width + 6, 16);
    g.fillStyle = hz ? "#fb6" : "#9ef";
    g.fillText(label, x + 10, y);
  }
}

async function drawViews(res, reported) {
  const c = $("views"), g = c.getContext("2d"), tw = 384, th = 288, cols = 4, rows = Math.ceil(res.views.length / cols);
  c.width = tw * cols;
  c.height = th * rows;
  g.font = "bold 13px ui-monospace,monospace";
  for (const [i, v] of res.views.entries()) {
    const bmp = await createImageBitmap(new Blob([Uint8Array.from(atob(v.image), (ch) => ch.charCodeAt(0))], { type: "image/jpeg" }));
    const x0 = (i % cols) * tw, y0 = Math.floor(i / cols) * th, k = tw / W;
    g.drawImage(bmp, x0, y0, tw, th);
    for (const r of reported.filter((r) => r.room === v.room && r.view === v.index)) {
      g.strokeStyle = g.fillStyle = r.o.kind ? "#f93" : "#3f6";
      g.lineWidth = 2;
      g.strokeRect(x0 + r.box[0] * k, y0 + r.box[1] * k, (r.box[2] - r.box[0]) * k, (r.box[3] - r.box[1]) * k);
      g.fillText(r.o.name ?? r.o.kind, x0 + r.box[0] * k + 3, y0 + Math.max(14, r.box[1] * k - 3));
    }
    const label = `${v.room} · view ${v.index}${v.pose.look !== "level" ? ` · ${v.pose.look}` : ""}`;
    g.fillStyle = "rgba(0,0,0,0.7)";
    g.fillRect(x0 + 4, y0 + 4, g.measureText(label).width + 10, 20);
    g.fillStyle = "#fd4";
    g.fillText(label, x0 + 9, y0 + 19);
  }
}

main().catch((e) => {
  log("ERROR", String(e?.stack || e));
  check("runs", false, String(e?.message || e));
}).finally(() => (result.done = true));
