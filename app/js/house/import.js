// Capture import: a SiteSpec project (outputs/, outputs/plans/, site.json, work/room/), a Spacial project (outputs/
// only) or a bare splat file -> house.json in the house frame H (docs/HOME-DRONE.md), plus the files to keep (the
// splat for the renderer, the orthophoto) and a HomeMap built from the splat. Sanity checks come back as warnings.
//
// Sources: a FileSystemDirectoryHandle (showDirectoryPicker), a File/Blob or { name, bytes } for a bare splat, or
// any { read(path) -> ArrayBuffer | Uint8Array | null (sync or async), list?(dir) -> names } (Node, a server route).

import { houseFrame } from "./frames.js";
import { HomeMap, MAP_DEFAULTS, polygonArea, roomAt } from "./homemap.js";

export const DOOR_RULE = { maxSill: 0.3, minHead: 1.6, minWidth: 0.55 }; // above the room floor; windows never pass
const GEOMETRY = { minOpacity: 77, maxScale: 0.3 }; // splats worth decoding for checks and the map (α ≥ 0.3)
const STAIRS = { radius: 0.75, margin: 0.25, depth: 3 }; // keep-out around a stair fixture; stairs may go down
export const CENTRES = "splat-centres.bin";

const r4 = (v) => Math.round(v * 1e4) / 1e4;
const xy4 = (p) => [r4(p[0]), r4(p[1])];
const median = (a) => (a.length ? [...a].sort((x, y) => x - y)[a.length >> 1] : null);

export function dirSource(dir) {
  const walk = async (parts) => {
    let d = dir;
    for (const p of parts) d = await d.getDirectoryHandle(p);
    return d;
  };
  return {
    name: dir.name,
    async read(path) {
      try {
        const parts = path.split("/");
        const file = await (await walk(parts.slice(0, -1))).getFileHandle(parts.at(-1));
        return await (await file.getFile()).arrayBuffer();
      } catch {
        return null;
      }
    },
    async list(path) {
      try {
        const names = [];
        for await (const name of (await walk(path.split("/").filter(Boolean))).keys()) names.push(name);
        return names;
      } catch {
        return [];
      }
    },
  };
}

const bytesOf = (b) => (b == null ? null : b instanceof ArrayBuffer ? new Uint8Array(b) : new Uint8Array(b.buffer, b.byteOffset, b.byteLength));
async function readJson(src, path) {
  const b = bytesOf(await src.read(path));
  return b ? JSON.parse(new TextDecoder().decode(b)) : null;
}

// ---------------------------------------------------------------------------------------------------------------
// Splat geometry: { n, xyz (W, packed Float32 triples), opacity (Uint8 0..255), scale (Float32, largest axis, m) }.
// Formats: SPZ v2/v3 (gzip), antimatter15 .splat (32 B), 3DGS binary PLY, and WPC1 "splat centres":
//   0  "WPC1"  4  u32 n  8  u32 flags (0)  12  u32 reserved (0)
//   16 Float32 x, y, z  [3n]  (capture frame W)
//   16+12n  Uint8 opacity [n]  (0..255, activated alpha)
//   16+13n  Uint8 scale [n]    (largest axis, SPZ coding: metres = exp(b / 16 − 10))
// ---------------------------------------------------------------------------------------------------------------

const scaleByte = (s) => Math.max(0, Math.min(255, Math.round((Math.log(s) + 10) * 16)));
const byteScale = (b) => Math.exp(b / 16 - 10);

export async function gunzip(bytes) {
  return new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream("gzip"))).arrayBuffer();
}

// The splat's geometry for the map and the checks, in H (scale × f).
const ptsToH = (pts, frame) => ({ xyz: frame.pointsToH(pts.xyz), opacity: pts.opacity, scale: pts.scale.map((s) => s * frame.f) });
export const geometryH = async (bytes, name, frame) => ptsToH(await readSplatPoints(bytes, name, GEOMETRY), frame);

export async function readSplatPoints(bytes, name = "", { minOpacity = 0, maxScale = Infinity } = {}) {
  let u8 = bytesOf(bytes);
  const tag = String.fromCharCode(...u8.subarray(0, 4));
  if (u8[0] === 0x1f && u8[1] === 0x8b) {
    u8 = new Uint8Array(await gunzip(u8));
    return readSpz(u8, minOpacity, maxScale);
  }
  if (tag === "WPC1") return readCentres(u8, minOpacity, maxScale);
  if (tag === "ply\n") return readPly(u8, minOpacity, maxScale);
  if (/\.splat$/i.test(name) || u8.length % 32 === 0) return readDotSplat(u8, minOpacity, maxScale);
  throw new Error(`${name || "splat"}: not an SPZ, .splat, PLY or splat-centres file`);
}

// keep(i) -> bool over n splats; get(i) -> [x, y, z, opacityByte, scale]
function collect(n, keep, get) {
  let m = 0;
  for (let i = 0; i < n; i++) if (keep(i)) m++;
  const xyz = new Float32Array(3 * m), opacity = new Uint8Array(m), scale = new Float32Array(m);
  for (let i = 0, j = 0; i < n; i++) {
    if (!keep(i)) continue;
    const [x, y, z, a, s] = get(i);
    xyz[3 * j] = x;
    xyz[3 * j + 1] = y;
    xyz[3 * j + 2] = z;
    opacity[j] = a;
    scale[j++] = s;
  }
  return { n: m, total: n, xyz, opacity, scale };
}

function readSpz(u8, minOpacity, maxScale) {
  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  const [magic, version, n, fb] = [dv.getUint32(0, true), dv.getUint32(4, true), dv.getUint32(8, true), u8[13]];
  if (magic !== 0x5053474e || (version !== 2 && version !== 3)) throw new Error(`SPZ version ${version} is not supported (2 or 3)`);
  const pos = 16, alpha = pos + 9 * n, scl = alpha + 4 * n; // positions, alphas, colours (3n), scales (3n)
  const k = 1 / (1 << fb);
  const i24 = (o) => ((u8[o] | (u8[o + 1] << 8) | (u8[o + 2] << 16)) << 8) >> 8;
  const sMax = (i) => Math.max(u8[scl + 3 * i], u8[scl + 3 * i + 1], u8[scl + 3 * i + 2]);
  const maxByte = maxScale === Infinity ? 255 : scaleByte(maxScale);
  return collect(
    n,
    (i) => u8[alpha + i] >= minOpacity && sMax(i) <= maxByte,
    (i) => [i24(pos + 9 * i) * k, i24(pos + 9 * i + 3) * k, i24(pos + 9 * i + 6) * k, u8[alpha + i], byteScale(sMax(i))],
  );
}

function readDotSplat(u8, minOpacity, maxScale) {
  const n = Math.floor(u8.length / 32), dv = new DataView(u8.buffer, u8.byteOffset, n * 32);
  const f = (i, j) => dv.getFloat32(32 * i + 4 * j, true);
  const s = (i) => Math.max(f(i, 3), f(i, 4), f(i, 5));
  return collect(n, (i) => u8[32 * i + 27] >= minOpacity && s(i) <= maxScale, (i) => [f(i, 0), f(i, 1), f(i, 2), u8[32 * i + 27], s(i)]);
}

function readCentres(u8, minOpacity, maxScale) {
  const n = new DataView(u8.buffer, u8.byteOffset).getUint32(4, true);
  const xyz = new Float32Array(u8.slice(16, 16 + 12 * n).buffer), op = u8.subarray(16 + 12 * n), sc = u8.subarray(16 + 13 * n);
  const keep = (i) => op[i] >= minOpacity && byteScale(sc[i]) <= maxScale;
  return collect(n, keep, (i) => [xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2], op[i], byteScale(sc[i])]);
}

export function writeCentres({ n, xyz, opacity, scale }) {
  const out = new Uint8Array(16 + 14 * n);
  out.set([0x57, 0x50, 0x43, 0x31]);
  new DataView(out.buffer).setUint32(4, n, true);
  out.set(new Uint8Array(xyz.buffer, xyz.byteOffset, 12 * n), 16);
  out.set(opacity, 16 + 12 * n);
  for (let i = 0; i < n; i++) out[16 + 13 * n + i] = scaleByte(scale[i]);
  return out.buffer;
}

const PLY_SIZE = { char: 1, int8: 1, uchar: 1, uint8: 1, short: 2, int16: 2, ushort: 2, uint16: 2, int: 4, int32: 4, uint: 4, uint32: 4 };
Object.assign(PLY_SIZE, { float: 4, float32: 4, double: 8, float64: 8 });

function readPly(u8, minOpacity, maxScale) {
  const end = new TextDecoder().decode(u8.subarray(0, Math.min(u8.length, 65536))).indexOf("end_header\n");
  if (end < 0) throw new Error("PLY: no end_header");
  const head = new TextDecoder().decode(u8.subarray(0, end)).split("\n");
  if (!head.some((l) => l.startsWith("format binary_little_endian"))) throw new Error("PLY: only binary_little_endian is supported");
  const elements = [];
  for (const l of head) {
    const w = l.trim().split(/\s+/), el = elements.at(-1);
    if (w[0] === "element") elements.push({ name: w[1], n: +w[2], stride: 0, props: {}, fixed: true });
    else if (w[0] === "property" && el) {
      if (!PLY_SIZE[w[1]]) el.fixed = false; // a list, or a type we cannot step over
      else [el.props[w[2]], el.stride] = [{ off: el.stride, type: w[1] }, el.stride + PLY_SIZE[w[1]]];
    }
  }
  const vi = elements.findIndex((e) => e.name === "vertex"), { n, stride, props } = elements[vi] ?? {};
  if (vi < 0 || !elements.slice(0, vi + 1).every((e) => e.fixed) || !props.x || !props.y || !props.z)
    throw new Error("PLY: needs a vertex element with x, y, z and fixed-size properties, after fixed-size elements only");
  const skip = elements.slice(0, vi).reduce((s, e) => s + e.n * e.stride, 0);
  const base = end + "end_header\n".length + skip, dv = new DataView(u8.buffer, u8.byteOffset + base, n * stride);
  const get = (i, p) => (PLY_SIZE[props[p].type] === 8 ? dv.getFloat64 : dv.getFloat32).call(dv, i * stride + props[p].off, true);
  const a = (i) => (props.opacity ? Math.round(255 / (1 + Math.exp(-get(i, "opacity")))) : 255);
  const s = (i) => (props.scale_0 ? Math.exp(Math.max(get(i, "scale_0"), get(i, "scale_1"), get(i, "scale_2"))) : 0.01);
  return collect(n, (i) => a(i) >= minOpacity && s(i) <= maxScale, (i) => [get(i, "x"), get(i, "y"), get(i, "z"), a(i), s(i)]);
}

// ---------------------------------------------------------------------------------------------------------------
// Plan -> house
// ---------------------------------------------------------------------------------------------------------------

function ceilingOf(m, zOff) {
  const cm = m.ceiling_map;
  if (!cm?.heights?.length) return { z: r4(m.ceiling_height + zOff) };
  // The map can cover far more than the room: keep its bounding box plus a cell.
  const xs = m.outline.map((p) => p[0]), ys = m.outline.map((p) => p[1]);
  const at = (v, v0) => Math.floor((v - v0) / cm.cell);
  const i0 = Math.max(0, at(Math.min(...xs), cm.x0) - 1), i1 = at(Math.max(...xs), cm.x0) + 1;
  const j0 = Math.max(0, at(Math.min(...ys), cm.y0) - 1), j1 = Math.min(cm.heights.length - 1, at(Math.max(...ys), cm.y0) + 1);
  const z = cm.heights.slice(j0, j1 + 1).map((row) => row.slice(i0, i1 + 1).map((h) => (h == null ? null : r4(h + zOff))));
  const all = z.flat().filter((v) => v != null);
  if (!all.length) return { z: r4(m.ceiling_height + zOff) };
  return { z: median(all), map: { x0: r4(cm.x0 + i0 * cm.cell), y0: r4(cm.y0 + j0 * cm.cell), cell: cm.cell, z } };
}

function roomsToH(roomsDoc, frame, ai, warn) {
  return roomsDoc.rooms.map((r) => {
    const m = r.model, zOff = frame.roomZ(m.floor_y), ceiling = ceilingOf(m, zOff);
    const name = ai?.rooms?.[r.id]?.name ?? r.name;
    if (ceiling.map && Math.abs(m.ceiling_height + zOff - ceiling.z) > 0.3)
      warn("ceiling", `${name}: the plan's ceiling height (${m.ceiling_height.toFixed(2)} m) disagrees with its ceiling map `
        + `(median ${(ceiling.z - zOff).toFixed(2)} m above the floor); using the map.`);
    const room = { id: r.id, name, outline: m.outline.map(xy4), floorZ: r4(zOff), ceiling, outlineSource: m.source?.outline ?? null };
    return name === r.name ? room : { ...room, planName: r.name };
  });
}

// Openings -> doors (with the room on each side) and windows. A doorway between two rooms is usually listed by
// both, on the two faces of the wall between them: those become one door on the first room's face, spanning what
// both agree on, with depth = the wall's thickness there (the second face is that far toward the other room).
function openingsToH(roomsDoc, frame, rooms) {
  const doors = [], windows = [], probe = MAP_DEFAULTS.doorReach, wall = (roomsDoc.wall_thickness_nominal ?? 0.12) + 0.1;
  for (const r of roomsDoc.rooms) {
    const m = r.model, zOff = frame.roomZ(m.floor_y), walls = new Map(m.walls.map((w) => [w.id, w]));
    const orient = Math.sign(polygonArea(m.outline)) || 1;
    for (const o of m.openings ?? []) {
      const w = walls.get(o.wall_id);
      if (!w) continue;
      const L = Math.hypot(w.b[0] - w.a[0], w.b[1] - w.a[1]), u = [(w.b[0] - w.a[0]) / L, (w.b[1] - w.a[1]) / L];
      const s0 = Math.max(0, o.start), s1 = Math.min(L, o.start + o.width);
      if (s1 - s0 < 0.05) continue;
      const a = xy4([w.a[0] + u[0] * s0, w.a[1] + u[1] * s0]), b = xy4([w.a[0] + u[0] * s1, w.a[1] + u[1] * s1]);
      const base = { id: `${r.id}:${o.id}`, a, b, sillZ: r4(o.sill + zOff), headZ: r4(o.head + zOff) };
      if (o.kind === "window") {
        windows.push({ ...base, room: r.id });
        continue;
      }
      const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2], out = [orient * u[1], -orient * u[0]];
      const inside = roomAt(rooms, mid[0] - probe * out[0], mid[1] - probe * out[1])?.id ?? r.id;
      const outside = roomAt(rooms, mid[0] + probe * out[0], mid[1] + probe * out[1])?.id ?? null;
      const rs = [inside, outside === inside ? null : outside];
      doors.push({ ...base, rooms: rs, width: r4(s1 - s0), kind: o.kind, floorZ: zOff, confidence: o.confidence });
    }
  }
  const merged = [];
  for (const d of doors) {
    const twin = merged.find((e) => samePair(e, d) && overlap(e, d, wall) > 0.5 * Math.max(e.width, d.width));
    if (!twin) merged.push(d);
    else Object.assign(twin, intersect(twin, d));
  }
  for (const d of merged) {
    d.passable = d.sillZ - d.floorZ <= DOOR_RULE.maxSill && d.headZ - d.floorZ >= DOOR_RULE.minHead && d.width >= DOOR_RULE.minWidth;
    delete d.floorZ;
  }
  return { doors: merged, windows };
}

const samePair = (e, d) =>
  (e.rooms[0] === d.rooms[0] && e.rooms[1] === d.rooms[1]) || (e.rooms[0] === d.rooms[1] && e.rooms[1] === d.rooms[0]);
// p relative to the line e.a -> e.b: t along it, s across it (signed).
function along(e, p) {
  const L = Math.hypot(e.b[0] - e.a[0], e.b[1] - e.a[1]), u = [(e.b[0] - e.a[0]) / L, (e.b[1] - e.a[1]) / L];
  const dx = p[0] - e.a[0], dy = p[1] - e.a[1];
  return { t: dx * u[0] + dy * u[1], s: dx * u[1] - dy * u[0], L, u };
}
// How much of e's width d covers, when d is parallel to e and at most `apart` from it (the other face of the wall).
function overlap(e, d, apart) {
  const p = along(e, d.a), q = along(e, d.b);
  if (Math.abs(p.s - q.s) > 0.05 || Math.max(Math.abs(p.s), Math.abs(q.s)) > apart) return 0;
  return Math.min(p.L, Math.max(p.t, q.t)) - Math.max(0, Math.min(p.t, q.t));
}
function intersect(e, d) {
  const p = along(e, d.a), q = along(e, d.b), at = (t) => xy4([e.a[0] + p.u[0] * t, e.a[1] + p.u[1] * t]);
  const t0 = Math.max(0, Math.min(p.t, q.t)), t1 = Math.min(p.L, Math.max(p.t, q.t));
  return {
    id: `${e.id}+${d.id}`,
    a: at(t0),
    b: at(t1),
    width: r4(t1 - t0),
    depth: r4(Math.max(e.depth ?? 0, Math.abs(p.s + q.s) / 2)),
    sillZ: Math.max(e.sillZ, d.sillZ),
    headZ: Math.min(e.headZ, d.headZ),
    kind: e.kind === "door" || d.kind === "door" ? "door" : e.kind,
    floorZ: Math.max(e.floorZ, d.floorZ),
  };
}

// An oriented rectangle grown by d on every side (RoomPlan footprints).
function growRect(p, d) {
  const c = [p.reduce((s, q) => s + q[0], 0) / p.length, p.reduce((s, q) => s + q[1], 0) / p.length];
  const axes = [0, 1].map((i) => {
    const v = [p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]], L = Math.hypot(...v);
    return [v[0] / L, v[1] / L];
  });
  return p.map((q) => {
    let [x, y] = q;
    for (const u of axes) {
      const s = Math.sign((q[0] - c[0]) * u[0] + (q[1] - c[1]) * u[1]);
      x += s * d * u[0];
      y += s * d * u[1];
    }
    return xy4([x, y]);
  });
}

// RoomPlan objects (work/room/iphone.json, capture-frame plan coordinates, heights above RoomPlan's own floor) and
// the AI scene fixtures (work/room/ai_scene.json, capture-frame plan coordinates) -> landmarks; stairs -> keep-outs.
function fixturesToH({ ai, iphone, frame, rooms }) {
  const { f } = frame, landmarks = [], keepouts = [];
  const place = (x, y) => roomAt(rooms, x, y);
  const stairsZ = (r) => ({ zMin: r4((r?.floorZ ?? 0) - STAIRS.depth), zMax: r4((r?.floorZ ?? 0) + STAIRS.depth) });
  for (const cap of iphone?.captures ?? [])
    for (const room of cap.rooms ?? []) {
      const zOff = room.model?.floor_y == null ? 0 : f * (frame.Yf - room.model.floor_y);
      for (const o of room.fixtures ?? []) {
        const x = f * o.x, y = f * o.y, r = place(x, y), footprint = o.footprint?.map(([a, b]) => xy4([f * a, f * b]));
        const [z, top, size] = [r4(zOff + f * o.height), r4(zOff + f * o.top), o.size?.map((v) => r4(f * v))];
        landmarks.push({ name: o.category ?? o.label, room: r?.id ?? null, x: r4(x), y: r4(y), z, top, size, footprint, source: "roomplan" });
        if (o.category === "stairs" && footprint?.length === 4)
          keepouts.push({ kind: "stairs", polygon: growRect(footprint, STAIRS.margin), ...stairsZ(r), source: "roomplan" });
      }
    }
  for (const [id, room] of Object.entries(ai?.rooms ?? {}))
    for (const o of room.fixtures ?? []) {
      const x = f * o.x, y = f * o.y, r = place(x, y) ?? rooms.find((q) => q.id === id);
      landmarks.push({ name: o.label, kind: o.kind, room: r?.id ?? null, x: r4(x), y: r4(y), z: r4((r?.floorZ ?? 0) + f * o.height), source: "ai" });
      if (o.kind === "stairs") keepouts.push({ kind: "stairs", x: r4(x), y: r4(y), r: STAIRS.radius, ...stairsZ(r), source: "ai" });
    }
  return { landmarks, keepouts };
}

// ---------------------------------------------------------------------------------------------------------------
// Checks against the splat (points already in H)
// ---------------------------------------------------------------------------------------------------------------

function roomEdges(room) {
  const p = room.outline, s = Math.sign(polygonArea(p)) || 1;
  return p.map((a, i) => {
    const b = p[(i + 1) % p.length], len = Math.hypot(b[0] - a[0], b[1] - a[1]), u = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
    return { a, b, len, u, n: [s * u[1], -s * u[0]], index: i };
  });
}

// Where the splat puts each long plan wall: the peak of opacity-weighted surface splats along its normal (±0.25 m,
// 0.3-2.0 m above the floor; open doorways skipped, windows kept since shades and glass sit in the wall plane).
// A scale error shows as offsets growing with the distance d from tag 0, the origin: offset ≈ (s − 1)·d. Curtains
// and furniture in front of some walls are outliers, so s comes from the largest group of walls that agree.
export function scaleFit(house, xyz, opacity, scale) {
  const walls = [];
  for (const rm of house.rooms)
    for (const e of roomEdges(rm)) {
      if (e.len < 1.2) continue;
      const tOf = (p) => (p[0] - e.a[0]) * e.u[0] + (p[1] - e.a[1]) * e.u[1], sOf = (p) => (p[0] - e.a[0]) * e.n[0] + (p[1] - e.a[1]) * e.n[1];
      // doorways on this wall, also across its thickness (a door listed on the other room's face, depth away)
      const onWall = (o) => [sOf(o.a), sOf(o.b)].every((s) => s > -0.1 && s < (o.depth ?? 0) + 0.1);
      const holes = house.doors.filter(onWall).map((o) => [Math.min(tOf(o.a), tOf(o.b)) - 0.1, Math.max(tOf(o.a), tOf(o.b)) + 0.1]);
      const hist = new Float32Array(51);
      let total = 0;
      for (let i = 0, n = opacity.length; i < n; i++) {
        if (opacity[i] < 102 || scale[i] > 0.05) continue;
        const dx = xyz[3 * i] - e.a[0], dy = xyz[3 * i + 1] - e.a[1], s = dx * e.n[0] + dy * e.n[1];
        if (s < -0.25 || s > 0.25) continue;
        const t = dx * e.u[0] + dy * e.u[1], h = xyz[3 * i + 2] - rm.floorZ;
        if (t < 0.1 * e.len || t > 0.9 * e.len || h < 0.3 || h > 2.0 || holes.some(([a, b]) => t > a && t < b)) continue;
        hist[Math.round((s + 0.25) * 100)] += opacity[i] / 255;
        total += opacity[i] / 255;
      }
      let k = 0;
      for (let j = 1; j < 51; j++) if (hist[j] > hist[k]) k = j;
      if (total < 20 || hist[k] < (3 * total) / 51) continue;
      let sw = 0, sx = 0;
      for (let j = Math.max(0, k - 1); j <= Math.min(50, k + 1); j++) [sw, sx] = [sw + hist[j], sx + hist[j] * (j / 100 - 0.25)];
      walls.push({ room: rm.id, edge: e.index, d: e.a[0] * e.n[0] + e.a[1] * e.n[1], offset: r4(sx / sw), weight: Math.round(total) });
    }
  const far = walls.filter((w) => Math.abs(w.d) >= 1.5);
  const agree = (k) => far.filter((w) => Math.abs(w.offset - k * w.d) <= 0.015);
  let inl = [], best = 0;
  for (const c of [0, ...far.map((w) => w.offset / w.d)]) {
    const a = agree(c);
    if (a.length > inl.length || (a.length === inl.length && Math.abs(c) < Math.abs(best))) [inl, best] = [a, c];
  }
  if (inl.length < 2) return { scale: null, walls };
  const k = inl.reduce((s, w) => s + w.d * w.offset, 0) / inl.reduce((s, w) => s + w.d * w.d, 0);
  return { scale: 1 + k, inliers: inl.length, rms: Math.sqrt(inl.reduce((s, w) => s + (w.offset - k * w.d) ** 2, 0) / inl.length), walls };
}

// rooms.json against rooms.auto.json × f for the rooms in both: 1 when the plan was drawn with the tape correction
// site.json holds now (it is redrawn when the correction changes; a stale one is off by the difference).
export function planScale(roomsDoc, roomsAuto, f) {
  let sxy = 0, sxx = 0;
  for (const r of roomsDoc?.rooms ?? []) {
    const q = roomsAuto?.rooms?.find((a) => a.id === r.id)?.model.outline;
    if (q?.length !== r.model.outline.length) continue;
    r.model.outline.forEach((p, i) => ([sxy, sxx] = [sxy + p[0] * q[i][0] + p[1] * q[i][1], sxx + q[i][0] ** 2 + q[i][1] ** 2]));
  }
  return sxx ? sxy / sxx / f : null;
}

// The splat floor in each room: the peak of surface splats within ±0.15 m of the plan's floor (further out it
// would be table tops and seats).
export function floorCheck(map, xyz, opacity, scale) {
  const hist = map.rooms.map(() => new Float32Array(31));
  for (let i = 0, n = opacity.length; i < n; i++) {
    if (scale[i] > 0.1) continue;
    const k = map.idx(xyz[3 * i], xyz[3 * i + 1]);
    if (k < 0 || map.room[k] < 0 || map.wall[k]) continue;
    const j = Math.round((xyz[3 * i + 2] - map.floorZ[k] + 0.15) * 100);
    if (j >= 0 && j <= 30) hist[map.room[k]][j] += opacity[i] / 255;
  }
  return map.rooms.map((rm, r) => {
    const h = hist[r], sm = (j) => (h[j - 1] ?? 0) + 2 * h[j] + (h[j + 1] ?? 0);
    let k = 0;
    for (let j = 1; j <= 30; j++) if (sm(j) > sm(k)) k = j;
    const total = h.reduce((a, b) => a + b, 0), dz = r4(k / 100 - 0.15);
    // a floor shows as a sharp peak; a flat histogram means no floor was captured here
    const flat = total < 20 || h[k] < (2.5 * total) / 31;
    return { room: rm.id, floorZ: rm.floorZ, ...(flat ? { splatZ: null } : { splatZ: r4(rm.floorZ + dz), dz }) };
  });
}

// ---------------------------------------------------------------------------------------------------------------

async function sha(bytes) {
  const d = new Uint8Array(await crypto.subtle.digest("SHA-256", bytes));
  return [...d.subarray(0, 8)].map((b) => b.toString(16).padStart(2, "0")).join("");
}

export async function importCapture(input, { name, f, mapOptions, keepPoints = false } = {}) {
  const warnings = [];
  const warn = (code, text) => warnings.push({ code, text });
  if (input instanceof Blob || input?.bytes) return importBare(input, { name, mapOptions, keepPoints, warnings, warn });
  const src = input?.kind === "directory" ? dirSource(input) : input;
  // A project folder, or its outputs/ folder picked directly.
  let base = "outputs/", scene = await readJson(src, "outputs/scene.json");
  if (!scene && (scene = await readJson(src, "scene.json"))) base = "";
  if (!scene) {
    const file = (await src.list?.(""))?.find((n) => /\.(spz|splat|ply)$/i.test(n));
    if (file) return importBare({ name: file, bytes: await src.read(file) }, { name, mapOptions, keepPoints, warnings, warn });
    throw new Error("No SiteSpec or Spacial capture here: outputs/scene.json is missing.");
  }
  const root = async (p) => (base ? readJson(src, p) : null); // site.json and work/ exist only above outputs/
  const plans = base + "plans/";
  const manifest = await readJson(src, plans + "manifest.json");
  const roomsName = (await src.list?.(plans.slice(0, -1)))?.find((n) => n.endsWith("-rooms.json")) ?? manifest?.files?.json?.name;
  const roomsDoc = roomsName ? await readJson(src, plans + roomsName) : null;
  const stem = roomsName?.replace(/-rooms\.json$/, "");
  const orthoGeo = (stem && (await readJson(src, `${plans}${stem}-orthophoto.json`))) ?? manifest?.orthophoto ?? null;
  const orthoName = manifest?.files?.orthophoto?.name ?? (stem && `${stem}-orthophoto.png`);
  const orthoPng = orthoGeo && orthoName ? bytesOf(await src.read(plans + orthoName)) : null;
  const [site, roomsAuto, ai, iphone, project] = await Promise.all(
    ["site.json", "work/room/rooms.auto.json", "work/room/ai_scene.json", "work/room/iphone.json", "project.json"].map(root),
  );
  const kind = roomsDoc || site || roomsAuto ? "sitespec" : "spacial";
  // The tape correction the plans were drawn with, recorded beside them: f when site.json is out of reach.
  const planF = [orthoGeo?.scale_correction, manifest?.orthophoto?.scale_correction].find((v) => v >= 0.9 && v <= 1.1);
  if (!base && kind === "sitespec")
    warn("outputs-folder", "You picked the capture's outputs folder: pick the project folder above it instead, so the tape-measure "
      + "correction, floor model, room names and stairs can be read.");

  // The renderer's splat, and the geometry for the map (centres when a helper prepared them: no 76 MB decode).
  let splat = null;
  for (const n of ["splat.spz", "splat.splat"]) {
    const bytes = splat ? null : bytesOf(await src.read(base + n));
    if (bytes) splat = { name: n, bytes };
  }
  const centres = bytesOf(await src.read(base + CENTRES));
  if (!splat && !centres) {
    const bytes = bytesOf(await src.read(base + "splat.ply"));
    if (bytes) splat = { name: "splat.ply", bytes };
  }
  const frame = houseFrame({ site, roomsAuto, rooms: roomsDoc, scene, f: f ?? (site ? undefined : planF) });
  const geo = centres ?? splat?.bytes;
  const pts = geo ? await readSplatPoints(geo, centres ? CENTRES : splat.name, GEOMETRY) : null;

  const files = {
    scene: base + "scene.json",
    ...(roomsName && { rooms: plans + roomsName }),
    ...(orthoPng && { orthophoto: plans + orthoName }),
    ...(splat && { splat: base + splat.name }),
    ...(centres && { centres: base + CENTRES }),
    ...(site && { site: "site.json" }),
    ...(roomsAuto && { roomsAuto: "work/room/rooms.auto.json" }),
    ...(ai && { aiScene: "work/room/ai_scene.json" }),
    ...(iphone && { iphone: "work/room/iphone.json" }),
  };

  const rooms = roomsDoc?.rooms?.length ? roomsToH(roomsDoc, frame, ai, warn) : [];
  const { doors, windows } = roomsDoc?.rooms?.length ? openingsToH(roomsDoc, frame, rooms) : { doors: [], windows: [] };
  if (scene.units !== "meters" || ["none", undefined].includes(scene.alignment?.scale_source))
    warn("units", "This capture has no metric scale (no AprilTags or IMU scale): distances in the map are not metres. "
      + "Re-run it with tags before flying.");
  if (!splat)
    warn("no-splat", centres
      ? "Only splat centres were found: the map has obstacles but there is nothing to render."
      : "No splat file was found: the map has walls only, no furniture.");

  const house = {
    id: await sha(splat?.bytes ?? centres ?? new TextEncoder().encode(JSON.stringify(roomsDoc ?? scene))),
    name: name ?? project?.name ?? roomsDoc?.project ?? src.name ?? "House",
    version: 1,
    source: { kind, name: project?.name ?? src.name ?? null, files },
    frame: { f: frame.f, Yf: frame.Yf, floorSource: frame.floorSource, note: "p_H = (f·x_W, f·z_W, f·(Yf − y_W)); W = the splat's frame" },
    rooms,
    doors,
    windows,
    tags: (scene.tags ?? []).map((t) => ({
      id: t.id,
      center: frame.toH(t.center).map(r4),
      normal: [-t.normal[0], -t.normal[2], t.normal[1]].map(r4), // out of the printed face (scene.json's points into the tag)
      size: t.size_m,
      corners: t.corners?.map((c) => frame.toH(c).map(r4)),
    })),
    landmarks: [],
    keepouts: [],
    home: null,
    orthophoto: orthoGeo ? { x0: orthoGeo.x0, y0: orthoGeo.y0, res: orthoGeo.res, width: orthoGeo.width, height: orthoGeo.height } : null,
    cameras: (scene.cameras ?? []).map((c) => {
      const p = frame.camToH(c.rotation, c.position);
      return [...p.p.map((v) => Math.round(v * 1e3) / 1e3), Math.round(p.yaw * 1e3) / 1e3];
    }),
  };
  const fx = fixturesToH({ ai, iphone, frame, rooms });
  house.landmarks.push(...fx.landmarks);
  house.keepouts.push(...fx.keepouts);
  if (!rooms.length) wholeCapture(house, pts && frame.pointsToH(pts.xyz), scene, frame, warn);
  const planRatio = planScale(roomsDoc, roomsAuto, frame.f);
  return finish(house, { pts, frame, splat, centres, orthoPng, mapOptions, keepPoints, warnings, warn, planned: rooms.length > 0, planRatio, planF });
}

// Without a room plan: one room over the splat's bounds (p2..p98) so the map and the simulator still work.
function wholeCapture(house, xyzH, scene, frame, warn) {
  let lo, hi;
  if (xyzH?.length) {
    const q = (axis) => {
      const v = new Float32Array(xyzH.length / 3);
      for (let i = 0; i < v.length; i++) v[i] = xyzH[3 * i + axis];
      v.sort();
      return [v[Math.floor(v.length * 0.02)], v[Math.floor(v.length * 0.98)]];
    };
    [lo, hi] = [0, 1, 2].map(q).reduce(([l, h], [a, b]) => [[...l, a], [...h, b]], [[], []]);
  } else if (scene?.splat?.bounds_p2) {
    const a = frame.toH(scene.splat.bounds_p2), b = frame.toH(scene.splat.bounds_p98);
    [lo, hi] = [a.map((v, i) => Math.min(v, b[i])), a.map((v, i) => Math.max(v, b[i]))];
  } else throw new Error("No room plan and no splat: nothing to build a house from.");
  const outline = [[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]].map(xy4);
  house.rooms = [{ id: "all", name: "Whole capture", outline, floorZ: 0, ceiling: { z: r4(Math.max(1.5, hi[2])) }, synthetic: true }];
  warn("no-rooms", "No room plan: the whole capture is one room from the splat's bounds. Add rooms and keep-outs before flying.");
}

async function importBare(file, { name, mapOptions, keepPoints, warnings, warn }) {
  const bytes = bytesOf(file.bytes ?? (await file.arrayBuffer())), fileName = file.name ?? "splat";
  const pts = await readSplatPoints(bytes, fileName, GEOMETRY);
  const ys = Float32Array.from({ length: pts.n }, (_, i) => pts.xyz[3 * i + 1]).sort();
  const frame = houseFrame({ f: 1, Yf: ys[Math.floor(ys.length * 0.98)] ?? 0 }); // y points down: the floor is the high end
  warn("bare", "A bare splat: assuming OpenCV axes (y down) in metres with the floor at its lowest 2%. Check the floor and scale before flying.");
  const house = {
    id: await sha(bytes),
    name: name ?? fileName.replace(/\.[^.]+$/, ""),
    version: 1,
    source: { kind: "splat", name: fileName, files: { splat: fileName } },
    frame: { f: 1, Yf: frame.Yf, floorSource: "splat p98", note: "p_H = (f·x_W, f·z_W, f·(Yf − y_W)); W = the splat's frame" },
    rooms: [],
    doors: [],
    windows: [],
    tags: [],
    landmarks: [],
    keepouts: [],
    home: null,
    orthophoto: null,
    cameras: [],
  };
  wholeCapture(house, frame.pointsToH(pts.xyz), null, frame, warn);
  const ext = fileName.toLowerCase().match(/\.(spz|splat|ply)$/)?.[1], splat = ext ? { name: `splat.${ext}`, bytes } : null;
  return finish(house, { pts, frame, splat, mapOptions, keepPoints, warnings, warn });
}

async function finish(house, o) {
  const { pts, frame, splat, centres, orthoPng, mapOptions, keepPoints, warnings, warn, planned, planRatio, planF } = o;
  const g = pts && ptsToH(pts, frame), map = new HomeMap(house, mapOptions);
  if (g) map.addSplats(g.xyz, g.opacity, g.scale);
  map.finalize();
  const checks = {};
  // Plans drawn before the tape correction last changed: rooms.json against rooms.auto.json × f (a fit, so 0.2%),
  // and the correction recorded beside the plans against the capture's (exact).
  if (planRatio != null) checks.planScale = +planRatio.toFixed(5);
  if (planF != null) checks.planF = planF;
  const recorded = planF != null && planF / frame.f;
  const stale = [Math.abs(planRatio - 1) > 0.002 && planRatio, Math.abs(recorded - 1) > 1e-4 && recorded].find(Boolean);
  if (stale)
    warn("plan-scale", `The floor plan was drawn with a tape-measure correction ${((stale - 1) * 100).toFixed(2)}% different from the one `
      + "the capture uses now. Open the project in SiteSpec so it redraws the plans, then import again.");
  if (g && planned) {
    const fit = scaleFit(house, g.xyz, g.opacity, g.scale), far = Math.max(...house.rooms.flatMap((r) => r.outline.map((p) => Math.hypot(...p))));
    const { inliers, walls } = fit;
    checks.scale = { scale: fit.scale && +fit.scale.toFixed(4), inliers, walls, rms: fit.rms && +fit.rms.toFixed(4), site: frame.f };
    const pc = ((fit.scale - 1) * 100).toFixed(1), cm = Math.abs((fit.scale - 1) * far * 100).toFixed(0);
    if (fit.scale == null) warn("scale-unchecked", "Could not compare the splat's walls with the plan: too few clear walls.");
    else if (Math.abs(fit.scale - 1) > 0.005)
      warn("scale", `The splat and the floor plan disagree in scale by ${pc}% (${fit.inliers} walls agree): about ${cm} cm at ${far.toFixed(1)} m `
        + "from tag 0. Check the tape-measure correction in SiteSpec and import again.");
    checks.floors = floorCheck(map, g.xyz, g.opacity, g.scale);
    for (const c of checks.floors) {
      const rm = house.rooms.find((r) => r.id === c.room), where = c.dz > 0 ? "above" : "below";
      if (c.splatZ == null) warn("floor", `${rm.name}: the splat shows no clear floor near the plan's floor, so its height is unchecked.`);
      else if (Math.abs(c.dz) > 0.05)
        warn("floor", `${rm.name}: the splat floor is ${Math.abs(c.dz * 100).toFixed(0)} cm ${where} the plan's floor.`);
    }
  }
  house.keepouts.push(...map.keepouts.filter((k) => !house.keepouts.includes(k)));
  for (const { x, y, z } of map.fans) house.landmarks.push({ name: "ceiling fan", room: map.roomAt(x, y)?.id ?? null, x, y, z, source: "splat" });
  house.home = homePad(house, map);
  house.checks = checks;
  house.warnings = warnings.map((w) => w.text);
  return {
    house,
    map,
    splat,
    centres: centres ? { name: CENTRES, bytes: centres } : null,
    orthophoto: orthoPng ? { name: "orthophoto.png", bytes: orthoPng } : null,
    warnings,
    ...(keepPoints && g && { points: g }),
  };
}

// The pad: tag 0 if it lies in a room (its arrow, +z_W, is +y in H), else the first capture camera inside one,
// moved to the nearest spot with a safe margin at 1 m.
function homePad(house, map) {
  const t0 = house.tags.find((t) => t.id === 0);
  const tag = t0 && map.roomAt(t0.center[0], t0.center[1]) && { x: t0.center[0], y: t0.center[1], yaw: Math.PI / 2, source: "tag 0" };
  const cam = house.cameras.map(([x, y, , yaw]) => ({ x, y, yaw, source: "capture start" })).find((p) => map.roomAt(p.x, p.y));
  const start = tag || cam;
  if (!start) return null;
  const b = map.bandOf(1.0), need = map.lethal(), { cell } = map.o;
  let best = null;
  for (let dy = -1; dy <= 1; dy += cell)
    for (let dx = -1; dx <= 1; dx += cell) {
      const k = map.idx(start.x + dx, start.y + dy);
      if (k >= 0 && map.clear[b][k] >= need && (!best || Math.hypot(dx, dy) < best.d)) best = { d: Math.hypot(dx, dy), k };
    }
  if (!best) return { ...start, x: r4(start.x), y: r4(start.y) };
  const [x, y] = map.center(best.k % map.W, (best.k / map.W) | 0);
  return { x: r4(x), y: r4(y), yaw: r4(start.yaw), source: start.source };
}
