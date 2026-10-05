// The simulator's world inside a real house (docs/HOME-DRONE.md), built from its HomeMap so the drone hits what the
// planner avoids: the plan's walls as the map draws them (12 cm slabs outward from the room outlines, carried a wall's
// thickness past each corner, cut at passable doorways, a header over each doorway, and map.walls()' jambs where a
// doorway crosses to the other room), closed doors where a doorway leads off the map, the step where two floors meet in a
// doorway, furniture as boxes from the map's splat occupancy (per 5 cm cell, the heights its bands leave possible, so
// a box stands at a band's altitude exactly where the map blocks that band), the ceiling fans, per-room floors and
// ceilings, and people and pets (actors.js). Same interface as world.js; castAll() and regionAt() feed the layered
// raycast camera (render.js) when there is no splat twin. Everything is in the house frame H: x, y on the plan, z up.
import { mulberry32, pushOut } from "./world.js";
import { HouseActors } from "./actors.js";
import { roomCenter } from "../house/planner.js";
import { polygonArea } from "../house/homemap.js";

const DEG = Math.PI / 180;
const GRID = 0.5; // m, ray-walk cells

// RoomPlan object names -> the detector's labels (null: an obstacle nobody asks for).
const COCO = { sofa: "couch", couch: "couch", chair: "chair", table: "dining table", bed: "bed", television: "tv", tv: "tv",
  toilet: "toilet", sink: "sink", refrigerator: "refrigerator", oven: "oven", stove: "oven", storage: null, bathtub: null };
const FLOORS = [[/kitchen|bath|laundry|utility/i, "tile"], [/bed/i, "carpet"], [/hall|entry|corridor|room \d/i, "planks"]];
const WALLS = [[214, 204, 186], [200, 206, 214], [212, 200, 196], [204, 212, 200], [220, 214, 200]];
const COLORS = { couch: [70, 96, 140], chair: [122, 84, 52], "dining table": [138, 96, 60], bed: [226, 226, 232], tv: [30, 30, 34] };
const PLAIN = [[150, 140, 126], [128, 120, 112], [160, 150, 136], [118, 124, 112]];

const bbox = (pts) => {
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  return [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
};

export const makeHouseWorld = (house, map, opts) => new HouseWorld(house, map, opts);

export class HouseWorld {
  kind = "house";

  constructor(house, map, { seed = 1, cast } = {}) {
    if (!map.clear) throw new Error("the house map is not finalized");
    Object.assign(this, { house, map, rand: mulberry32(seed), solids: [], faces: [], items: [], stamp: 0 });
    const { cell } = map.o;
    this.bounds = { x0: map.x0, y0: map.y0, x1: map.x0 + map.W * cell, y1: map.y0 + map.H * cell };
    this.rooms = map.rooms.map((r, i) => {
      const [x0, y0, x1, y1] = bbox(r.outline);
      return { id: r.id, name: r.name, outline: r.outline, floorZ: r.floorZ, ceilZ: r.ceiling?.z ?? r.floorZ + 2.4, x0, y0, x1, y1,
        floor: FLOORS.find(([re]) => re.test(r.name))?.[1] ?? "wood", wall: WALLS[i % WALLS.length], style: "plain" };
    });
    this.cellSolid = new Int32Array(map.N).fill(-1);
    this.walls();
    for (const d of map.doors) this.door(d);
    this.furnish();
    for (const f of map.fans ?? []) {
      const r = Math.max(0.2, f.radius * 0.8);
      this.stampCells(this.slab({ a: [f.x - r, f.y], b: [f.x + r, f.y], s0: -r, s1: r, kind: "fan", zMin: f.z - 0.06, zMax: f.z + 0.06 }));
    }
    this.decorate();
    this.index();
    this.furniture = this.items;
    this.segments = this.faces.filter((f) => f.inner && (f.full || f.kind === "door")); // the plan's walls, for maps
    this.start = this.startPose();
    this.actors = new HouseActors(this, { seed, cast });
  }

  get cat() {
    return this.actors.cat;
  }
  roomAt(x, y) {
    const k = this.map.idx(x, y);
    return k < 0 || this.map.room[k] < 0 ? null : this.rooms[this.map.room[k]];
  }
  floorAt(x, y) {
    return this.map.floorAt(x, y);
  }
  ceilingAt(x, y) {
    return this.map.ceilingAt(x, y);
  }
  // What a ray crosses between two faces: the room, its floor and (rendered) ceiling, and the box over or under it.
  regionAt(x, y) {
    const k = this.map.idx(x, y), room = k < 0 || this.map.room[k] < 0 ? null : this.rooms[this.map.room[k]];
    const box = k < 0 || this.cellSolid[k] < 0 ? null : this.solids[this.cellSolid[k]];
    return { room, floor: room ? this.map.floorZ[k] : -1, ceil: room ? room.ceilZ : 99, box };
  }
  sprites() {
    return this.actors.sprites();
  }
  actorSpecs() {
    return this.actors.twinSpecs();
  }
  step(dt, drone) {
    this.actors.step(dt, drone);
  }

  // A box from the line a-b, s0..s1 to its side n (default: the left normal), z from zMin to zMax.
  slab({ a, b, n, s0, s1, zMin = -Infinity, zMax = Infinity, ...rest }) {
    const L = Math.hypot(b[0] - a[0], b[1] - a[1]), ux = (b[0] - a[0]) / L, uy = (b[1] - a[1]) / L;
    const [nx, ny] = n ?? [-uy, ux], m = (s0 + s1) / 2;
    const cx = (a[0] + b[0]) / 2 + nx * m, cy = (a[1] + b[1]) / 2 + ny * m, hl = L / 2, hw = (s1 - s0) / 2;
    const ex = Math.abs(ux) * hl + Math.abs(uy) * hw, ey = Math.abs(uy) * hl + Math.abs(ux) * hw;
    const s = { id: this.solids.length, cx, cy, ux, uy, hl, hw, zMin, zMax, x0: cx - ex, y0: cy - ey, x1: cx + ex, y1: cy + ey,
      color: [200, 196, 188], ...rest };
    s.full = zMin === -Infinity && zMax === Infinity;
    this.solids.push(s);
    const at = (t, q) => [cx + ux * t - uy * q, cy + uy * t + ux * q];
    const P = [at(-hl, -hw), at(hl, -hw), at(hl, hw), at(-hl, hw)];
    for (let i = 0; i < 4; i++) {
      const [p, q] = [P[i], P[(i + 1) % 4]], dx = q[0] - p[0], dy = q[1] - p[1], len = Math.hypot(dx, dy);
      if (len < 1e-4) continue;
      const fn = [dy / len, -dx / len]; // outward (corners run counter-clockwise)
      // the face on the line a-b of a wall or door slab is the one the room sees
      const inner = s0 === 0 && i === (n && (nx * -uy + ny * ux) < 0 ? 2 : 0);
      this.faces.push({ ax: p[0], ay: p[1], bx: q[0], by: q[1], dx, dy, len, n: fn, zMin, zMax, full: s.full, kind: s.kind, item: s.item ?? null,
        solid: s, inner, decor: [], shade: 0.78 + 0.22 * Math.max(0, (fn[0] + 2 * fn[1]) / Math.sqrt(5)) });
    }
    return s;
  }

  // Cells under a solid that is not a full wall, for regionAt (the first one wins).
  stampCells(s) {
    const { map } = this, { cell } = map.o;
    for (let y = s.y0; y <= s.y1 + cell; y += cell)
      for (let x = s.x0; x <= s.x1 + cell; x += cell) {
        const k = map.idx(x, y);
        if (k < 0 || this.cellSolid[k] >= 0) continue;
        const [px, py] = map.center(k % map.W, (k / map.W) | 0), dx = px - s.cx, dy = py - s.cy;
        if (Math.abs(dx * s.ux + dy * s.uy) <= s.hl && Math.abs(-dx * s.uy + dy * s.ux) <= s.hw) this.cellSolid[k] = s.id;
      }
  }

  // The direction off the line a-b that leaves the house (or the room `id`) at its middle.
  away(a, b, id) {
    const L = Math.hypot(b[0] - a[0], b[1] - a[1]), n = [(b[1] - a[1]) / L, -(b[0] - a[0]) / L];
    const r = this.map.roomAt((a[0] + b[0]) / 2 + n[0] * 0.06, (a[1] + b[1]) / 2 + n[1] * 0.06);
    return r && (id === undefined || r.id === id) ? [-n[0], -n[1]] : n;
  }

  // homemap.js rasterizeWalls(), as solids.
  walls() {
    const { map } = this, T = map.o.wallThickness;
    for (const rm of map.rooms) {
      const o = rm.outline, sg = Math.sign(polygonArea(o)) || 1, { wall: color, ceilZ } = this.rooms[rm.index];
      o.forEach((a, i) => {
        const b = o[(i + 1) % o.length], len = Math.hypot(b[0] - a[0], b[1] - a[1]);
        if (len < 1e-3) return;
        const u = [(b[0] - a[0]) / len, (b[1] - a[1]) / len], n = [sg * u[1], -sg * u[0]], at = (t) => [a[0] + u[0] * t, a[1] + u[1] * t];
        const gaps = map.gaps({ a, b, len, u, n, room: rm.id }).filter((g) => g.door.passable).sort((p, q) => p.t0 - q.t0);
        let t = -T;
        for (const g of [...gaps, { t0: len + T, t1: Infinity }]) {
          if (g.t0 - t > 0.005) this.slab({ a: at(t), b: at(g.t0), n, s0: 0, s1: T, kind: "wall", room: rm.id, color });
          if (g.door && g.door.headZ < ceilZ && g.t1 - g.t0 > 0.005)
            this.stampCells(this.slab({ a: at(g.t0), b: at(g.t1), n, s0: 0, s1: T, kind: "header", room: rm.id, door: g.door.id, color,
              zMin: g.door.headZ }));
          t = Math.max(t, g.t1);
        }
      });
    }
    for (const w of map.walls()) if (w.kind === "wall" && w.door && Math.hypot(w.b[0] - w.a[0], w.b[1] - w.a[1]) > 1e-3)
      this.slab({ a: w.a, b: w.b, s0: -0.01, s1: 0.01, kind: "jamb", room: w.room, color: this.rooms.find((r) => r.id === w.room)?.wall });
  }

  // Doorways off the map are closed (a door leaf, or a plain wall for an opening); between two floors, the step.
  door(d) {
    if (!d.passable) return;
    const [ra, rb] = d.rooms.map((id) => this.rooms.find((r) => r.id === id));
    if (!ra) return;
    if (!rb) {
      const n = this.away(d.a, d.b);
      this.slab({ a: d.a, b: d.b, n, s0: 0, s1: this.map.o.wallThickness, kind: "door", room: ra.id, door: d.id, zMax: d.headZ, color: ra.wall,
        leaf: d.kind === "door", headZ: d.headZ });
    } else if (Math.abs(ra.floorZ - rb.floorZ) > 0.02) {
      const n = this.away(d.a, d.b, ra.id), off = ra.floorZ < rb.floorZ ? 0 : (d.depth ?? 0);
      this.slab({ a: d.a, b: d.b, n, s0: off - 0.015, s1: off + 0.015, kind: "riser", door: d.id, zMax: Math.max(ra.floorZ, rb.floorZ) });
    }
  }

  // Furniture from the map's occupancy: per cell the occupied levels (pet height and the three flight bands) as height
  // runs, cells with the same runs merged into rectangles, 4-connected blobs as items, labelled from RoomPlan's objects.
  furnish() {
    const { map } = this, { W, H, N, occ, floorZ } = map, { cell } = map.o;
    const order = map.levels.map((_, i) => i).sort((a, b) => map.levels[a][0] - map.levels[b][0]), L = order.map((i) => map.levels[i]);
    const mask = new Uint8Array(N);
    for (let k = 0; k < N; k++) if (map.room[k] >= 0 && !map.wall[k]) order.forEach((li, j) => occ[li][k] && (mask[k] |= 1 << j));
    // A run of occupied levels j..e spans from the top of the free level below to the bottom of the free one above.
    const spans = (m) => {
      const out = [];
      for (let j = 0; j < L.length; j++) {
        if (!((m >> j) & 1)) continue;
        let e = j;
        while (e + 1 < L.length && (m >> (e + 1)) & 1) e++;
        out.push([j ? Math.max(L[j][0], L[j - 1][1]) : 0, e + 1 < L.length ? Math.min(L[e][1], L[e + 1][0]) : L[e][1]]);
        j = e;
      }
      return out;
    };
    const comp = new Int32Array(N).fill(-1), blobs = [];
    for (let k0 = 0; k0 < N; k0++) {
      if (!mask[k0] || comp[k0] >= 0) continue;
      const cells = [k0];
      comp[k0] = blobs.length;
      for (let i = 0; i < cells.length; i++)
        for (const d of [1, -1, W, -W]) {
          const k = cells[i] + d;
          if (k >= 0 && k < N && mask[k] && comp[k] < 0 && Math.abs((k % W) - (cells[i] % W)) <= 1) (comp[k] = blobs.length), cells.push(k);
        }
      blobs.push(cells);
    }
    const marks = (this.house.landmarks ?? []).filter((l) => l.source === "roomplan" && l.name?.toLowerCase() in COCO);
    const named = new Map();
    for (const l of marks) {
      let best = -1, bd = 0.45;
      for (let dy = -0.45; dy <= 0.45; dy += cell)
        for (let dx = -0.45; dx <= 0.45; dx += cell) {
          const k = map.idx(l.x + dx, l.y + dy);
          if (k >= 0 && comp[k] >= 0 && Math.hypot(dx, dy) < bd) [best, bd] = [comp[k], Math.hypot(dx, dy)];
        }
      if (best >= 0 && !named.has(best)) named.set(best, l.name.toLowerCase());
    }
    this.items = blobs.map((cells, i) => {
      let sx = 0, sy = 0, z0 = Infinity, z1 = -Infinity;
      const pts = cells.map((k) => map.center(k % W, (k / W) | 0));
      for (let j = 0; j < cells.length; j++) {
        [sx, sy] = [sx + pts[j][0], sy + pts[j][1]];
        for (const [lo, hi] of spans(mask[cells[j]])) [z0, z1] = [Math.min(z0, floorZ[cells[j]] + lo), Math.max(z1, floorZ[cells[j]] + hi)];
      }
      const [bx0, by0, bx1, by1] = bbox(pts), name = named.get(i) ?? null, label = name ? COCO[name] : null;
      const x = sx / cells.length, y = sy / cells.length;
      return { index: i, id: `f${i}`, name, label, x, y, z0, z1, cells: cells.length, r: Math.sqrt((cells.length * cell * cell) / Math.PI),
        w: Math.max(bx1 - bx0, by1 - by0) / 2 + cell / 2, box: [bx0 - cell / 2, by0 - cell / 2, bx1 + cell / 2, by1 + cell / 2],
        axis: bx1 - bx0 >= by1 - by0 ? 0 : Math.PI / 2, room: map.roomAt(x, y)?.id ?? null, seat: label === "couch" || label === "chair",
        color: COLORS[label] ?? PLAIN[i % PLAIN.length] };
    });
    // Rectangles: runs of equal cells along each row, continued down while the next row has the same run.
    const same = (a, b) => mask[a] === mask[b] && comp[a] === comp[b] && floorZ[a] === floorZ[b];
    let open = new Map();
    const close = (rc) => {
      const [x0, y0] = map.center(rc.c0, rc.r0), [x1, y1] = map.center(rc.c1, rc.r1), k = rc.r0 * W + rc.c0, item = this.items[comp[k]];
      const a = [x0 - cell / 2, (y0 + y1) / 2], b = [x1 + cell / 2, (y0 + y1) / 2], hw = (y1 - y0 + cell) / 2;
      const boxes = spans(mask[k]).map(([lo, hi]) =>
        this.slab({ a, b, s0: -hw, s1: hw, kind: "box", item, color: item.color, zMin: floorZ[k] + lo, zMax: floorZ[k] + hi }));
      this.stampCells(boxes[0]);
    };
    for (let r = 0; r <= H; r++) {
      const next = new Map();
      for (let c = 0; r < H && c < W; ) {
        const k = r * W + c;
        if (!mask[k]) {
          c++;
          continue;
        }
        let e = c;
        while (e + 1 < W && same(k, r * W + e + 1)) e++;
        const key = `${c},${e}`, prev = open.get(key);
        if (prev && same(prev.r0 * W + prev.c0, k)) (prev.r1 = r), next.set(key, prev), open.delete(key);
        else next.set(key, { c0: c, c1: e, r0: r, r1: r });
        c = e + 1;
      }
      for (const rc of open.values()) close(rc);
      open = next;
    }
  }

  // Windows on the wall faces that look into a room (heights above that room's floor), and closed doors' leaves.
  decorate() {
    for (const f of this.faces) {
      if (!f.inner || !["wall", "header", "door"].includes(f.kind)) continue;
      const room = this.roomAt((f.ax + f.bx) / 2 + f.n[0] * 0.08, (f.ay + f.by) / 2 + f.n[1] * 0.08);
      if (!room) continue;
      const ux = f.dx / f.len, uy = f.dy / f.len;
      const along = (p) => (p[0] - f.ax) * ux + (p[1] - f.ay) * uy, off = (p) => Math.abs((p[0] - f.ax) * uy - (p[1] - f.ay) * ux);
      if (f.kind === "door" && f.solid.leaf)
        f.decor.push({ kind: "door", room: room.id, u0: 0.03, u1: f.len - 0.03, v0: 0, v1: f.solid.headZ - room.floorZ });
      for (const w of this.house.windows ?? []) {
        if (off(w.a) > 0.1 || off(w.b) > 0.1) continue;
        const u0 = Math.max(0, Math.min(along(w.a), along(w.b))), u1 = Math.min(f.len, Math.max(along(w.a), along(w.b)));
        if (u1 - u0 > 0.05) f.decor.push({ kind: "window", room: room.id, u0, u1, v0: w.sillZ - room.floorZ, v1: w.headZ - room.floorZ });
      }
    }
  }

  index() {
    const { x0, y0, x1, y1 } = this.bounds, nx = Math.ceil((x1 - x0) / GRID), ny = Math.ceil((y1 - y0) / GRID), none = [];
    const cells = Array.from({ length: nx * ny }, () => none);
    const cellOf = (x, y) => [Math.max(0, Math.min(nx - 1, Math.floor((x - x0) / GRID))), Math.max(0, Math.min(ny - 1, Math.floor((y - y0) / GRID)))];
    this.faces.forEach((f, i) => {
      const [c0, r0] = cellOf(Math.min(f.ax, f.bx), Math.min(f.ay, f.by)), [c1, r1] = cellOf(Math.max(f.ax, f.bx), Math.max(f.ay, f.by));
      for (let r = r0; r <= r1; r++)
        for (let c = c0; c <= c1; c++) (cells[r * nx + c] === none ? (cells[r * nx + c] = []) : cells[r * nx + c]).push(i);
    });
    this.grid = { x0, y0, nx, ny, cells };
    this.allFaces = this.faces.map((_, i) => i);
  }

  // Grid cells along a ray (Amanatides-Woo) while visit(faces, entryDistance) returns true.
  walk(ox, oy, dx, dy, maxDist, visit) {
    const { x0, y0, nx, ny, cells } = this.grid;
    let cx = Math.floor((ox - x0) / GRID), cy = Math.floor((oy - y0) / GRID);
    if (cx < 0 || cy < 0 || cx >= nx || cy >= ny) return visit(this.allFaces, 0);
    const sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1, tdx = dx ? Math.abs(GRID / dx) : Infinity, tdy = dy ? Math.abs(GRID / dy) : Infinity;
    let tx = dx ? (x0 + (cx + (dx > 0)) * GRID - ox) / dx : Infinity, ty = dy ? (y0 + (cy + (dy > 0)) * GRID - oy) / dy : Infinity, t = 0;
    while (t <= maxDist && visit(cells[cy * nx + cx], t)) {
      if (tx < ty) [t, tx, cx] = [tx, tx + tdx, cx + sx];
      else [t, ty, cy] = [ty, ty + tdy, cy + sy];
      if (cx < 0 || cy < 0 || cx >= nx || cy >= ny) return;
    }
  }

  // Every face the ray crosses, nearest first, up to and including the first full-height wall: [{ dist, seg, u }].
  castAll(ox, oy, dx, dy, maxDist = 30) {
    const out = [], stamp = ++this.stamp;
    let full = maxDist;
    this.walk(ox, oy, dx, dy, maxDist, (list, t) => {
      if (t > full) return false;
      for (const i of list) {
        const f = this.faces[i];
        if (f.mark === stamp || f.solid.removed) continue;
        f.mark = stamp;
        const h = cross(f, ox, oy, dx, dy);
        if (!h || h.dist > full) continue;
        if (f.full) full = h.dist;
        out.push(h);
      }
      return true;
    });
    return out.filter((h) => h.dist <= full).sort((a, b) => a.dist - b.dist);
  }

  // The nearest face that blocks the ray at height z (without z: the nearest full-height wall). { dist, seg, u } or null.
  castRay(ox, oy, dx, dy, maxDist = 50, z = null) {
    const stamp = ++this.stamp;
    let best = null;
    this.walk(ox, oy, dx, dy, maxDist, (list, t) => {
      if (best && t > best.dist) return false;
      for (const i of list) {
        const f = this.faces[i];
        if (f.mark === stamp || f.solid.removed || (z == null ? !f.full : z < f.zMin || z > f.zMax)) continue;
        f.mark = stamp;
        const h = cross(f, ox, oy, dx, dy);
        if (h && h.dist <= maxDist && (!best || h.dist < best.dist)) best = h;
      }
      return true;
    });
    return best;
  }

  // Push a circle (at height body.z, ± height) out of every solid. Returns the impact speed (m/s, 0 if none).
  collide(body, radius, height = 0.05) {
    let impact = 0;
    for (const s of this.solids) {
      if (s.removed || body.z > s.zMax + height || body.z + height < s.zMin) continue;
      if (body.x < s.x0 - radius || body.x > s.x1 + radius || body.y < s.y0 - radius || body.y > s.y1 + radius) continue;
      const dx = body.x - s.cx, dy = body.y - s.cy, t = dx * s.ux + dy * s.uy, q = -dx * s.uy + dy * s.ux;
      const et = Math.abs(t) - s.hl, eq = Math.abs(q) - s.hw;
      if (et > 0 || eq > 0) {
        const ct = Math.max(-s.hl, Math.min(s.hl, t)), cq = Math.max(-s.hw, Math.min(s.hw, q));
        impact = Math.max(impact, pushOut(body, s.cx + ct * s.ux - cq * s.uy, s.cy + ct * s.uy + cq * s.ux, radius));
        continue;
      }
      // inside: out through the nearest side
      const alongT = et > eq, sg = (alongT ? Math.sign(t) : Math.sign(q)) || 1;
      const nx = alongT ? sg * s.ux : -sg * s.uy, ny = alongT ? sg * s.uy : sg * s.ux, pen = -(alongT ? et : eq) + radius;
      body.x += nx * pen;
      body.y += ny * pen;
      if (body.vx === undefined) continue;
      const vn = body.vx * nx + body.vy * ny;
      if (vn < 0) {
        body.vx -= 1.35 * vn * nx;
        body.vy -= 1.35 * vn * ny;
        impact = Math.max(impact, -vn);
      }
    }
    return impact;
  }

  // True if a full-height wall (or, with z, anything at that height) is between two points.
  blocked(ax, ay, bx, by, z) {
    const L = Math.hypot(bx - ax, by - ay);
    return L > 1e-6 && !!this.castRay(ax, ay, (bx - ax) / L, (by - ay) / L, L, z);
  }

  // ---------------------------------------------------------------- scene changes (Simulator.addObstacle, setDoor)

  // A box { id, x, y, z (bottom), w (along yaw), d, h, yaw } that blocks flight and rays like furniture.
  addObstacle({ id, x, y, z, w, d, h, yaw = 0, color }) {
    this.removeSolid(id);
    const ux = Math.cos(yaw), uy = Math.sin(yaw), a = [x - (ux * w) / 2, y - (uy * w) / 2], b = [x + (ux * w) / 2, y + (uy * w) / 2];
    const rgb = typeof color === "string" ? [1, 3, 5].map((i) => parseInt(color.slice(i, i + 2), 16)) : color ?? PLAIN[0];
    this.dynamic(id, [this.slab({ a, b, s0: -d / 2, s1: d / 2, kind: "box", item: { id, label: null, color: rgb }, color: rgb, zMin: z, zMax: z + h })]);
  }

  // Close (open false) or open doorway `id`: a leaf across it up to the door head. Returns the leaf as a twin panel
  // { x, y, z, w, d, h, yaw } when it is a new one, else null. A doorway off the map was closed in the capture: opening
  // it changes only the collision world.
  setDoor(id, open) {
    const d = this.map.doors.find((q) => q.id === id);
    if (!d) throw new Error(`no doorway ${id}`);
    const key = `door:${id}`, captured = this.solids.find((s) => s.kind === "door" && s.door === id && !s.change);
    this.removeSolid(key);
    if (captured) return (captured.removed = !!open), null;
    if (open) return null;
    const room = this.rooms.find((r) => r.id === d.rooms[0]), floor = room?.floorZ ?? this.map.floorAt(d.a[0], d.a[1]) ?? 0;
    const headZ = d.headZ ?? floor + 2.0, n = this.away(d.a, d.b, d.rooms[0]), off = d.rooms[1] ? (d.depth ?? 0) / 2 : 0.06;
    const a = [d.a[0] + n[0] * off, d.a[1] + n[1] * off], b = [d.b[0] + n[0] * off, d.b[1] + n[1] * off];
    this.dynamic(key, [this.slab({ a, b, s0: -0.02, s1: 0.02, kind: "door", door: id, leaf: true, headZ, color: room?.wall ?? [200, 196, 188], zMax: headZ })]);
    return { x: (a[0] + b[0]) / 2, y: (a[1] + b[1]) / 2, z: floor, w: Math.hypot(b[0] - a[0], b[1] - a[1]), d: 0.04, h: headZ - floor, yaw: Math.atan2(b[1] - a[1], b[0] - a[0]) };
  }

  // Solids added after the build (scene changes) under an id; their faces join the ray index.
  dynamic(id, solids) {
    for (const s of solids) {
      s.change = id;
      if (s.kind === "box") this.stampCells(s);
    }
    this.index();
  }

  // Removed solids stay in the lists, skipped by rays and collisions.
  removeSolid(id) {
    let hit = false;
    for (const s of this.solids) if (s.change === id && !s.removed) hit = s.removed = true;
    if (hit) for (let k = 0; k < this.cellSolid.length; k++) if (this.cellSolid[k] >= 0 && this.solids[this.cellSolid[k]].removed) this.cellSolid[k] = -1;
  }

  // The home pad (house.home: tag 0 moved to a safe spot), else tag 0, else the middle of the first room.
  startPose() {
    const { home, tags } = this.house, t0 = tags?.find((t) => t.id === 0);
    const p = home ?? (t0 && this.roomAt(t0.center[0], t0.center[1]) ? { x: t0.center[0], y: t0.center[1], yaw: Math.PI / 2 } : null);
    if (p) return { x: p.x, y: p.y, heading: (p.yaw ?? Math.PI / 2) / DEG };
    const c = roomCenter(this.map, this.rooms[0].id) ?? [this.rooms[0].outline[0][0], this.rooms[0].outline[0][1]];
    return { x: c[0], y: c[1], heading: 90 };
  }
}

function cross(f, ox, oy, dx, dy) {
  const denom = dx * f.dy - dy * f.dx;
  if (Math.abs(denom) < 1e-12) return null;
  const wx = f.ax - ox, wy = f.ay - oy, t = (wx * dy - wy * dx) / denom;
  if (t < 0 || t > 1) return null;
  const u = (wx * f.dy - wy * f.dx) / denom;
  return u > 1e-6 ? { dist: u, seg: f, u: t * f.len } : null;
}
