// How much of the house the 3D map knows (docs/HOME-DRONE.md, "Wave C contracts"): per room, at the drone's heights
// (0.3-1.6 m above the floor), the known-free, unknown and occupied volumes, and the gaps worth a second pass with the
// scanner, in plain words. A gap is unknown space that touches known-free space (somewhere a drone could look into), cut
// into pieces about a person's reach across; unknown space sealed off by surfaces (inside a cabinet, under a sofa's skirt)
// is counted as hidden and never suggested. The capture camera rode at 1.3-2.1 m, so low gaps get a knee-height pass.
// twinOnlyPct: the share of a room's known-free space that only the twin's renders (the splat model, not a camera ray or a
// flight) vouch for. flyableM3: known-free space with the planner's clearance (drone radius + σ, options.sigma or the map's).
// A map that can't tell free space (no splat or no capture path: vox.built.why) scores 0 and says why in note.
import { UNKNOWN, FREE, OCCUPIED, FLAG, bandMask } from "./voxels.js";
import { polygonArea } from "./homemap.js";

export const COVERAGE = {
  band: [0.3, 1.6],
  piece: 1.2, // m: gaps are cut into tiles this wide
  minGap: 0.004, // m³ (32 voxels): smaller gaps are noise
  near: 1.3, // m: a landmark or doorway this close names a gap
  stand: 2.0, // m: how far to look for a free spot to scan from
};

const fmt = (v) => +v.toFixed(2);
const pct = (a, b) => (b ? +((100 * a) / b).toFixed(1) : 0);

export function coverageReport({ house, map, vox, options = {} }) {
  const o = { ...COVERAGE, ...options }, { nx, ny, nz, res, st } = vox, layer = nx * ny, v3 = res ** 3;
  const band = bandMask(vox, map, o.band), fly = map.lethal(o.sigma ?? map.o.sigma);
  const rooms = map.rooms.map((r) => ({ room: r, free: 0, twin: 0, unknown: 0, occupied: 0, flyable: 0, hidden: 0, gaps: [] }));
  const roomOf = new Int16Array(layer).fill(-1);
  for (let r = 0; r < ny; r++)
    for (let c = 0; c < nx; c++) {
      const k = map.idx(vox.x0 + (c + 0.5) * res, vox.y0 + (r + 0.5) * res);
      if (k >= 0) roomOf[r * nx + c] = map.room[k];
    }
  if (!vox.dist) vox.computeClearance();
  for (let i = 0; i < vox.n; i++) {
    if (!band[i]) continue;
    const R = rooms[roomOf[i % layer]];
    if (!R) continue;
    if (st[i] === FREE) {
      R.free++;
      if ((vox.flags[i] & (FLAG.TWIN | FLAG.CARVED | FLAG.FLIGHT)) === FLAG.TWIN) R.twin++;
      if (vox.dist[i] >= fly) R.flyable++;
    } else if (st[i] === OCCUPIED) R.occupied++;
    else R.unknown++;
  }
  // Unknown pieces at drone heights, 6-connected within one room.
  const seen = new Uint8Array(vox.n), stack = new Int32Array(vox.n);
  for (let i0 = 0; i0 < vox.n; i0++) {
    if (seen[i0] || !band[i0] || st[i0] !== UNKNOWN) continue;
    const ri = roomOf[i0 % layer], R = rooms[ri];
    let top = 0, frontier = 0;
    const cells = [];
    stack[top++] = i0;
    seen[i0] = 1;
    while (top) {
      const i = stack[--top], c = i % nx, r = ((i / nx) | 0) % ny, l = (i / layer) | 0;
      cells.push(i);
      const nb = [c > 0 && i - 1, c < nx - 1 && i + 1, r > 0 && i - nx, r < ny - 1 && i + nx, l > 0 && i - layer, l < nz - 1 && i + layer];
      for (const j of nb) {
        if (j === false) continue;
        if (st[j] === FREE) frontier++;
        else if (!seen[j] && band[j] && st[j] === UNKNOWN && roomOf[j % layer] === ri) (seen[j] = 1), (stack[top++] = j);
      }
    }
    if (!R) continue;
    if (!frontier) R.hidden += cells.length;
    else R.gaps.push(...pieces(vox, map, cells, o));
  }
  const all = rooms.reduce((a, R) => ({ free: a.free + R.free, unknown: a.unknown + R.unknown }), { free: 0, unknown: 0 });
  const out = rooms.map((R) => {
    const total = R.free + R.unknown + R.occupied;
    const gaps = R.gaps.filter((g) => g.voxels * v3 >= o.minGap).sort((a, b) => b.voxels - a.voxels)
      .map((g) => describe(house, map, vox, R.room, g, o));
    return { id: R.room.id, name: R.room.name, flyableM3: fmt(R.flyable * v3), knownFreePct: pct(R.free, total), unknownPct: pct(R.unknown, total),
      occupiedPct: pct(R.occupied, total), unknownM3: fmt(R.unknown * v3), hiddenM3: fmt(R.hidden * v3), twinOnlyPct: pct(R.twin, R.free), gaps };
  });
  const suggestions = [];
  for (const g of out.flatMap((r) => r.gaps.map((g) => ({ ...g, room: r }))).sort((a, b) => b.volume - a.volume)) {
    const s = g.suggestion, same = suggestions.find((q) => q.height === s.height && Math.hypot(q.x - s.x, q.y - s.y) < 1.0);
    if (same) same.gaps++;
    else suggestions.push({ ...s, gaps: 1 });
  }
  const note = vox.built?.usable === false ? vox.built.why : null;
  return { score: note ? 0 : Math.round(pct(all.free, all.free + all.unknown)), band: o.band, flyableClearance: fly, note, rooms: out,
    suggestions: note ? [] : suggestions.slice(0, 12) };
}

// Tiles of a gap about o.piece across (on the plan), each with its footprint, heights and voxel count.
function pieces(vox, map, cells, o) {
  const { nx, ny, res } = vox, layer = nx * ny, tile = Math.max(1, Math.round(o.piece / res)), by = new Map();
  for (const i of cells) {
    const c = i % nx, r = ((i / nx) | 0) % ny, key = `${Math.floor(c / tile)},${Math.floor(r / tile)}`;
    if (!by.has(key)) by.set(key, []);
    by.get(key).push(i);
  }
  return [...by.values()].map((list) => {
    const cols = new Set();
    let sx = 0, sy = 0, sz = 0, h0 = Infinity, h1 = -Infinity;
    for (const i of list) {
      const [x, y, z] = vox.center(i), h = z - (map.floorAt(x, y) ?? 0);
      cols.add(i % layer);
      [sx, sy, sz, h0, h1] = [sx + x, sy + y, sz + z, Math.min(h0, h), Math.max(h1, h)];
    }
    const n = list.length;
    return { x: sx / n, y: sy / n, z: sz / n, voxels: n, area: cols.size * res * res, h0: Math.max(o.band[0], h0 - res / 2),
      h1: Math.min(o.band[1], h1 + res / 2) };
  });
}

function describe(house, map, vox, room, g, o) {
  const where = place(house, map, room, g, o), heights = heightWords(g), name = room.name ?? room.id;
  const { height, how } = g.h1 <= 0.9 ? { height: 0.5, how: "at knee height (about 0.5 m)" }
    : g.h0 < 1.0 ? { height: 0.7, how: "low, at about 0.7 m (the scanner rode higher than the drone flies)" }
    : { height: 1.2, how: "at chest height (about 1.2 m)" };
  const spot = standAt(map, vox, g, height, o) ?? [g.x, g.y];
  const text = `${cap(where.text)} in ${name} was never seen ${heights}.`;
  const verb = where.object ? `Walk the scanner slowly around ${where.object}` : `Walk the scanner to ${where.text}`;
  return {
    x: fmt(g.x), y: fmt(g.y), z: fmt(g.z), size: fmt(g.area), volume: fmt(g.voxels * vox.res ** 3), heightBand: `${fmt(g.h0)}-${fmt(g.h1)} m`,
    text, voxels: g.voxels, suggestion: { text: `${verb} in ${name} ${how}, pointing it ${where.object ? "behind and under it" : "into the gap"}.`,
      x: fmt(spot[0]), y: fmt(spot[1]), height, room: room.id },
  };
}

const cap = (s) => s[0].toUpperCase() + s.slice(1);
function heightWords({ h0, h1 }) {
  if (h1 <= 0.9) return `below ${fmt(Math.ceil(h1 * 10) / 10)} m`;
  if (h0 >= 1.0) return `above ${fmt(Math.floor(h0 * 10) / 10)} m`;
  return `between ${fmt(Math.floor(h0 * 10) / 10)} and ${fmt(Math.ceil(h1 * 10) / 10)} m`;
}

// Names a gap: by a landmark of the room ("behind the sofa", "under the table"), a doorway, or where it lies in the
// room as the map shows it ("the top-left corner", "along the right wall", "the middle").
function place(house, map, room, g, o) {
  const marks = (house.landmarks ?? []).filter((l) => (l.room ?? map.roomAt(l.x, l.y)?.id) === room.id && l.name?.length <= 24 && (l.z ?? 0) < 1.8)
    .map((l) => ({ l, d: Math.hypot(l.x - g.x, l.y - g.y) })).filter((m) => m.d <= o.near).sort((a, b) => a.d - b.d);
  const mid = roomMiddle(room);
  if (marks.length) {
    const { l, d } = marks[0], name = `the ${l.name.toLowerCase()}`;
    const behind = (g.x - l.x) * (mid[0] - l.x) + (g.y - l.y) * (mid[1] - l.y) < 0;
    const under = d < 0.6 && l.z != null && g.h1 <= l.z + 0.15;
    return { text: `the space ${under ? "under" : behind ? "behind" : "beside"} ${name}`, object: name };
  }
  const door = (house.doors ?? []).map((d) => ({ d, m: [(d.a[0] + d.b[0]) / 2, (d.a[1] + d.b[1]) / 2] }))
    .filter(({ d, m }) => d.rooms.includes(room.id) && Math.hypot(m[0] - g.x, m[1] - g.y) < 0.9)[0];
  if (door) {
    const other = door.d.rooms.find((r) => r && r !== room.id), to = other && map.rooms.find((r) => r.id === other);
    return { text: `the space by the doorway${to ? ` to ${to.name}` : ""}` };
  }
  const xs = room.outline.map((p) => p[0]), ys = room.outline.map((p) => p[1]);
  const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)], m = 0.9;
  const lr = g.x - x0 < m ? "left" : x1 - g.x < m ? "right" : "", tb = y1 - g.y < m ? "top" : g.y - y0 < m ? "bottom" : "";
  if (lr && tb) return { text: `the ${tb}-${lr} corner (on the map)` };
  if (lr || tb) return { text: `the space along the ${lr || tb} wall (on the map)` };
  return { text: "a spot in the middle of the room" };
}
const roomMiddle = (room) => {
  const p = room.outline, A = polygonArea(p) || 1;
  let cx = 0, cy = 0;
  p.forEach(([x, y], i) => {
    const [u, v] = p[(i + 1) % p.length], k = x * v - u * y;
    cx += (x + u) * k;
    cy += (y + v) * k;
  });
  return [cx / (6 * A), cy / (6 * A)];
};

// The nearest known-free spot at that height, with room to stand a person there, to scan the gap from.
function standAt(map, vox, g, height, o) {
  let best = null, bd = Infinity;
  for (let dy = -o.stand; dy <= o.stand; dy += 0.1)
    for (let dx = -o.stand; dx <= o.stand; dx += 0.1) {
      const x = g.x + dx, y = g.y + dy, d = Math.hypot(dx, dy), fl = map.floorAt(x, y);
      if (d >= bd || d < 0.3 || fl == null || map.clearance(x, y, fl + 1.0) < 0.25) continue;
      if (vox.state(x, y, fl + height) !== FREE || vox.clearance3(x, y, fl + height) < 0.2) continue;
      [best, bd] = [[x, y], d];
    }
  return best;
}
