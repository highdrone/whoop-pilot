// 2.5D flight map of a house in the house frame H (docs/HOME-DRONE.md): a 5 cm grid with the plan's walls (12 cm,
// built outward from the room outlines, passable doorways cut out and bridged across the wall), opacity-weighted
// splat obstacles per altitude band, keep-outs (ceiling fans found here from hanging splat clusters, stairs, user
// zones), per-room floors and ceilings, and per band the clearance in metres to the nearest blocked cell, which the
// planner inflates. Overlays: temporary obstacles (addTemp: people, suspected changes) and the 3D voxel map (setVoxels:
// then clearances are the smaller of the 2.5D one and clearance3, which counts unknown space as blocked).

export const MAP_DEFAULTS = {
  cell: 0.05,
  margin: 0.3, // grid border around the rooms
  wallThickness: 0.12,
  doorReach: 0.3, // a doorway joins rooms whose outlines are up to this far apart (the wall between them)
  bands: [0.6, 1.0, 1.4], // flight altitudes above the local floor
  bandHalf: 0.3, // splats within ± this of a band block it (altitude is estimated, not measured)
  low: [0.1, 0.5], // pet height, for search visibility
  minOpacity: 0.4,
  maxScale: 0.2, // m; larger splats are fog, not surfaces
  cellWeight: 1.2, // summed opacity that makes a cell solid...
  support: 2, // ...when at least this many neighbours are solid too (kills lone floaters)
  hangMin: 1.6, // m above the floor: hanging things (fans, pendants) between these heights;
  hangMax: 2.8, // higher is out of reach (and on sloped or interpolated ceilings, often the ceiling itself)
  hangSeed: 2.5,
  hangGrow: 0.4,
  hangAway: 0.4, // m from walls, door and window lines (door headers and curtain rods are not fans)
  fanMinArea: 0.05, // m²
  fanJoin: 1.0, // clusters closer than this are one fan (sparse blades split it)
  fanMaxRadius: 0.9, // a fan is compact; wall tops, beams and a lower ceiling next door are not
  fanMaxArea: 1.5, // m²
  fanMargin: 0.4, // beyond the blade tips (the cluster spans the blades), for downwash on a 36 g quad
  fanMinRadius: 1.0,
  ceilingMargin: 0.4, // fly at least this far below the ceiling
  headMargin: 0.3, // and below a door head
  droneRadius: 0.05,
  sigma: 0.25, // default 1σ position uncertainty
};

export const polygonArea = (p) => p.reduce((a, [x, y], i) => a + x * p[(i + 1) % p.length][1] - p[(i + 1) % p.length][0] * y, 0) / 2;
const unit = (a, b) => {
  const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
  return { L, u: [(b[0] - a[0]) / L, (b[1] - a[1]) / L] };
};

export function inPolygon(x, y, p) {
  let inside = false;
  for (let i = 0, j = p.length - 1; i < p.length; j = i++) {
    const [xi, yi] = p[i], [xj, yj] = p[j];
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

// Rooms may overlap (an iPhone RoomPlan outline can contain the splat's smaller rooms): the smallest one wins.
export function roomAt(rooms, x, y) {
  let best = null, area = Infinity;
  for (const r of rooms) {
    if (!inPolygon(x, y, r.outline)) continue;
    const a = r.area ?? Math.abs(polygonArea(r.outline));
    if (a < area) [best, area] = [r, a];
  }
  return best;
}

// Ceiling z over a plan point from a house room ({ z } and/or { map: { x0, y0, cell, z: rows from y0 } }).
export function roomCeiling(room, x, y) {
  const m = room.ceiling?.map;
  const v = m && m.z[Math.floor((y - m.y0) / m.cell)]?.[Math.floor((x - m.x0) / m.cell)];
  return v ?? room.ceiling?.z ?? room.floorZ + 2.4;
}

// Outline edges as segments with the room interior on the left, plus the outward normal.
function edges(room) {
  const p = room.outline, s = Math.sign(polygonArea(p)) || 1;
  return p.map((a, i) => {
    const b = p[(i + 1) % p.length], { L: len, u } = unit(a, b);
    return { a, b, len, u, n: [s * u[1], -s * u[0]], id: `${room.id}#${i}`, room: room.id };
  });
}

const OCC_MAGIC = 0x31434f57; // "WOC1"

export class HomeMap {
  constructor(house, opts = {}) {
    this.house = house;
    this.o = { ...MAP_DEFAULTS, ...opts };
    const { cell, margin } = this.o;
    if (!house.rooms?.length) throw new Error("house has no rooms");
    this.rooms = house.rooms.map((r, index) => ({ ...r, index, area: Math.abs(polygonArea(r.outline)) }));
    this.doors = house.doors ?? [];
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const r of this.rooms)
      for (const [x, y] of r.outline) [x0, y0, x1, y1] = [Math.min(x0, x), Math.min(y0, y), Math.max(x1, x), Math.max(y1, y)];
    this.x0 = x0 - margin;
    this.y0 = y0 - margin;
    this.W = Math.ceil((x1 - x0 + 2 * margin) / cell);
    this.H = Math.ceil((y1 - y0 + 2 * margin) / cell);
    const N = (this.N = this.W * this.H);
    this.levels = [...this.o.bands.map((a) => [a - this.o.bandHalf, a + this.o.bandHalf]), this.o.low];
    this.room = new Int16Array(N).fill(-1);
    this.floorZ = new Float32Array(N);
    this.ceilZ = new Float32Array(N).fill(Infinity);
    this.headZ = new Float32Array(N).fill(Infinity); // door heads over the cut-out doorway cells
    this.wall = new Uint8Array(N);
    this.structure = new Uint8Array(N); // walls plus every door and window line
    this.acc = new Float32Array(N * this.levels.length);
    this.hang = new Float32Array(N);
    this.hangZ = new Float32Array(N);
    this.splats = 0;
    this.temps = new Map();
    this.vox = null;
    this.clock = () => performance.now(); // addTemp until (ms), like nav/avoid.js
    this.paintRooms();
    this.bridgeDoors();
    this.rasterizeWalls();
  }

  center(c, r) {
    return [this.x0 + (c + 0.5) * this.o.cell, this.y0 + (r + 0.5) * this.o.cell];
  }
  idx(x, y) {
    const c = Math.floor((x - this.x0) / this.o.cell), r = Math.floor((y - this.y0) / this.o.cell);
    return c < 0 || r < 0 || c >= this.W || r >= this.H ? -1 : r * this.W + c;
  }
  roomAt(x, y) {
    const k = this.idx(x, y);
    return k < 0 || this.room[k] < 0 ? null : this.rooms[this.room[k]];
  }
  floorAt(x, y) {
    const k = this.idx(x, y);
    return k < 0 || this.room[k] < 0 ? null : this.floorZ[k];
  }
  ceilingAt(x, y) {
    const k = this.idx(x, y);
    return k < 0 || this.room[k] < 0 ? null : this.ceilZ[k];
  }
  bandOf(alt) {
    let best = 0;
    this.o.bands.forEach((a, b) => Math.abs(a - alt) < Math.abs(this.o.bands[best] - alt) && (best = b));
    return best;
  }
  // z is absolute (H): below the floor (by more than its few cm of error), near the ceiling or a door head, or above
  // the top band (unmapped) is 0; else the nearest band's clearance, so take-off and landing use the lowest one, and
  // with voxels at most clearance3 there. Without z: the 1.0 m band. Temporary obstacles count either way.
  clearance(x, y, z) {
    this.sync();
    const k = this.idx(x, y), { bands, bandHalf, ceilingMargin, headMargin } = this.o;
    if (k < 0) return 0;
    if (z == null) return this.clear[this.bandOf(1.0)][k];
    const h = z - this.floorZ[k];
    if (h < -0.05 || h > bands.at(-1) + bandHalf || z > this.ceilZ[k] - ceilingMargin || z > this.headZ[k] - headMargin) return 0;
    const c = this.flat[this.bandOf(h)][k];
    return this.vox ? Math.min(c, this.vox.clearance3(x, y, z)) : c;
  }
  lethal(sigma = this.o.sigma) {
    return this.o.droneRadius + sigma;
  }
  free(x, y, z, r = this.lethal()) {
    return this.clearance(x, y, z) >= r;
  }

  // Scanline fill, larger rooms first so the smallest containing room ends up owning each cell.
  paintRooms() {
    const { W, H } = this;
    for (const rm of [...this.rooms].sort((a, b) => b.area - a.area)) {
      const p = rm.outline;
      for (let r = 0; r < H; r++) {
        const y = this.center(0, r)[1], xs = [];
        for (let i = 0, j = p.length - 1; i < p.length; j = i++)
          if (p[i][1] > y !== p[j][1] > y) xs.push(p[i][0] + ((y - p[i][1]) * (p[j][0] - p[i][0])) / (p[j][1] - p[i][1]));
        xs.sort((a, b) => a - b);
        for (let s = 0; s + 1 < xs.length; s += 2) {
          const c0 = Math.max(0, Math.ceil((xs[s] - this.x0) / this.o.cell - 0.5));
          const c1 = Math.min(W - 1, Math.floor((xs[s + 1] - this.x0) / this.o.cell - 0.5));
          for (let c = c0; c <= c1; c++) {
            const k = r * W + c, [x] = this.center(c, r);
            this.room[k] = rm.index;
            this.floorZ[k] = rm.floorZ;
            this.ceilZ[k] = roomCeiling(rm, x, y);
          }
        }
      }
    }
  }

  // Outlines are the walls' inner faces, so the wall between two rooms lies outside both and a doorway through it
  // would stay closed: paint it as interior (the higher floor, the lower ceiling) wherever a straight walk from the
  // doorway line crosses outside cells into one of the door's rooms within its depth (or doorReach).
  bridgeDoors() {
    const { cell, doorReach } = this.o, byId = new Map(this.rooms.map((r) => [r.id, r])), fill = new Map();
    for (const d of this.doors) {
      const pair = d.rooms.map((id) => byId.get(id));
      if (!d.passable || !pair[0] || !pair[1] || pair[0] === pair[1]) continue;
      const { L, u } = unit(d.a, d.b), reach = Math.max(d.depth ?? 0, doorReach) + cell, ids = pair.map((r) => r.index);
      const within = (k) => {
        const [x, y] = this.center(k % this.W, (k / this.W) | 0), t = (x - d.a[0]) * u[0] + (y - d.a[1]) * u[1];
        return t >= 0 && t <= L;
      };
      for (let t = 0; t <= L; t += cell / 2)
        for (const sg of [1, -1]) {
          const gap = [];
          for (let s = 0; s <= reach; s += cell / 2) {
            const k = this.idx(d.a[0] + u[0] * t - sg * u[1] * s, d.a[1] + u[1] * t + sg * u[0] * s);
            if (k < 0 || !within(k)) break; // past a jamb
            if (this.room[k] < 0) gap.push(k);
            else if (gap.length) {
              if (ids.includes(this.room[k])) for (const g of gap) fill.set(g, pair);
              break;
            }
          }
        }
    }
    for (const [k, [a, b]] of fill) {
      const [x, y] = this.center(k % this.W, (k / this.W) | 0), hi = a.floorZ >= b.floorZ ? a : b;
      this.room[k] = hi.index;
      this.floorZ[k] = hi.floorZ;
      this.ceilZ[k] = Math.min(roomCeiling(a, x, y), roomCeiling(b, x, y));
    }
  }

  // Passable doorways on an edge: doors on its line (a doorway shared by two rooms is on both outlines), and doors
  // of this room across the wall from it, on the other room's face (off: that face's distance outward).
  gaps(e) {
    const out = [];
    for (const d of this.doors) {
      const off = (p) => (p[0] - e.a[0]) * e.n[0] + (p[1] - e.a[1]) * e.n[1], along = (p) => (p[0] - e.a[0]) * e.u[0] + (p[1] - e.a[1]) * e.u[1];
      const [p, q] = [off(d.a), off(d.b)], on = Math.abs(p) <= 0.08 && Math.abs(q) <= 0.08;
      const deep = Math.max(d.depth ?? 0, this.o.doorReach) + 0.08;
      const across = d.rooms.includes(e.room) && Math.abs(p - q) <= 0.05 && Math.min(p, q) > 0 && Math.max(p, q) <= deep;
      if (!on && !across) continue;
      const t0 = Math.min(along(d.a), along(d.b)), t1 = Math.max(along(d.a), along(d.b));
      if (t1 > 0 && t0 < e.len) out.push({ t0, t1, door: d, off: on ? 0 : (p + q) / 2 });
    }
    return out;
  }

  // Each edge becomes a slab from the outline outward (the outline is the wall's inner face), closed at the corners.
  rasterizeWalls() {
    const { cell, wallThickness: T } = this.o;
    const slab = (e, t0, t1, s0, s1, fn) => {
      const pts = [[t0, s0], [t1, s0], [t0, s1], [t1, s1]].map(([t, s]) => [
        e.a[0] + e.u[0] * t + e.n[0] * s,
        e.a[1] + e.u[1] * t + e.n[1] * s,
      ]);
      const cx = pts.map((p) => Math.floor((p[0] - this.x0) / cell)), cy = pts.map((p) => Math.floor((p[1] - this.y0) / cell));
      for (let r = Math.max(0, Math.min(...cy)); r <= Math.min(this.H - 1, Math.max(...cy)); r++)
        for (let c = Math.max(0, Math.min(...cx)); c <= Math.min(this.W - 1, Math.max(...cx)); c++) {
          const [x, y] = this.center(c, r), dx = x - e.a[0], dy = y - e.a[1];
          const t = dx * e.u[0] + dy * e.u[1], s = dx * e.n[0] + dy * e.n[1];
          if (t >= t0 && t <= t1 && s > s0 && s <= s1) fn(r * this.W + c);
        }
    };
    for (const rm of this.rooms)
      for (const e of edges(rm)) {
        const gaps = this.gaps(e).filter((g) => g.door.passable).sort((a, b) => a.t0 - b.t0);
        let t = -T;
        for (const g of [...gaps, { t0: e.len + T, t1: Infinity }]) {
          if (g.t0 > t)
            slab(e, t, g.t0, -cell / 2, T, (k) => {
              this.wall[k] = 1;
              this.structure[k] = 1;
            });
          if (g.door) slab(e, g.t0, g.t1, -cell, T + cell, (k) => (this.headZ[k] = Math.min(this.headZ[k], g.door.headZ)));
          t = Math.max(t, g.t1);
        }
      }
    for (const o of [...this.doors, ...(this.house.windows ?? [])]) this.line(o.a, o.b, (k) => (this.structure[k] = 1));
  }

  line(a, b, fn) {
    const L = Math.hypot(b[0] - a[0], b[1] - a[1]), n = Math.max(1, Math.ceil(L / (this.o.cell / 2)));
    for (let i = 0; i <= n; i++) {
      const k = this.idx(a[0] + ((b[0] - a[0]) * i) / n, a[1] + ((b[1] - a[1]) * i) / n);
      if (k >= 0) fn(k);
    }
  }

  // Splat centres in H. opacity: 0..1 floats or 0..255 bytes; scale: the largest axis in metres.
  addSplats(xyz, opacity, scale) {
    const { minOpacity, maxScale, hangMin, hangMax } = this.o;
    const { levels, N, acc, hang, hangZ, room, floorZ, ceilZ } = this;
    const k255 = opacity instanceof Uint8Array ? 1 / 255 : 1;
    for (let i = 0, n = xyz.length / 3; i < n; i++) {
      const a = opacity[i] * k255;
      if (a < minOpacity || scale[i] > maxScale) continue;
      const z = xyz[3 * i + 2], k = this.idx(xyz[3 * i], xyz[3 * i + 1]);
      if (k < 0 || room[k] < 0) continue;
      const h = z - floorZ[k];
      for (let b = 0; b < levels.length; b++) if (h >= levels[b][0] && h <= levels[b][1]) acc[b * N + k] += a;
      if (h >= hangMin && h <= hangMax && z <= ceilZ[k] - 0.25) {
        hang[k] += a;
        hangZ[k] += a * z;
      }
      this.splats++;
    }
    return this;
  }

  // Occupancy cache: the raw per-cell sums, so thresholds can change without decoding the splat again.
  occupancyBytes() {
    const { x0, y0, W, H, levels, splats } = this;
    const head = new TextEncoder().encode(JSON.stringify({ x0, y0, W, H, cell: this.o.cell, levels, splats }));
    const pad = (4 - ((8 + head.length) % 4)) % 4, off = 8 + head.length + pad;
    const out = new Uint8Array(off + 4 * (this.acc.length + 2 * this.N));
    new DataView(out.buffer).setUint32(0, OCC_MAGIC, true);
    new DataView(out.buffer).setUint32(4, head.length, true);
    out.set(head, 8);
    const f = new Float32Array(out.buffer, off);
    f.set(this.acc);
    f.set(this.hang, this.acc.length);
    f.set(this.hangZ, this.acc.length + this.N);
    return out.buffer;
  }
  // false when the cache was made for another grid (rooms or options changed): add the splats again.
  loadOccupancy(buf) {
    const dv = new DataView(buf instanceof ArrayBuffer ? buf : buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength));
    if (dv.getUint32(0, true) !== OCC_MAGIC) return false;
    const len = dv.getUint32(4, true), h = JSON.parse(new TextDecoder().decode(new Uint8Array(dv.buffer, 8, len)));
    const same = ["x0", "y0", "W", "H"].every((k) => Math.abs(h[k] - this[k]) < 1e-6) && h.cell === this.o.cell;
    if (!same || JSON.stringify(h.levels) !== JSON.stringify(this.levels)) return false;
    const off = 8 + len + ((4 - ((8 + len) % 4)) % 4), f = new Float32Array(dv.buffer, off, this.acc.length + 2 * this.N);
    this.acc.set(f.subarray(0, this.acc.length));
    this.hang.set(f.subarray(this.acc.length, this.acc.length + this.N));
    this.hangZ.set(f.subarray(this.acc.length + this.N));
    this.splats = h.splats;
    return true;
  }

  finalize() {
    const { W, N, o } = this;
    this.occ = this.levels.map((_, b) => solid(this.acc.subarray(b * N, (b + 1) * N), W, this.H, o.cellWeight, o.support));
    this.fans = this.findFans();
    const known = (this.house.keepouts ?? []).filter((k) => k.kind === "fan");
    this.keepouts = [
      ...(this.house.keepouts ?? []),
      ...this.fans
        .filter((f) => !known.some((k) => Math.hypot(k.x - f.x, k.y - f.y) < 0.5))
        .map(({ x, y, z, radius }) => {
          const zMin = (this.floorAt(x, y) ?? 0) - 0.1;
          return { kind: "fan", x, y, r: Math.max(o.fanMinRadius, radius + o.fanMargin), zMin, zMax: z + 0.3, source: "splat" };
        }),
    ];
    const keep = this.o.bands.map(() => new Uint8Array(N));
    for (const ko of this.keepouts) this.rasterizeKeepout(ko, keep);
    this.blocked = [];
    this.base = [];
    this.o.bands.forEach((alt, b) => {
      const bl = new Uint8Array(N), occ = this.occ[b];
      for (let k = 0; k < N; k++) {
        const z = this.floorZ[k] + alt;
        const above = z > this.ceilZ[k] - o.ceilingMargin || z > this.headZ[k] - o.headMargin;
        bl[k] = this.room[k] < 0 || this.wall[k] || occ[k] || keep[b][k] || above ? 1 : 0;
      }
      this.blocked.push(bl);
      this.base.push(clearanceField(bl, W, this.H, o.cell));
    });
    return this.overlay();
  }

  // The 3D map (house/voxels.js, built over this grid): free and clearance with z, and the planner, then keep to known-free
  // space. null removes it, and so does a map that could tell nothing free (vox.built.usable false: no splat or capture
  // path), which would refuse every plan: the 2.5D map flies alone then.
  setVoxels(vox) {
    this.vox = vox?.built?.usable === false ? null : vox;
    return this.overlay();
  }
  // A temporary obstacle { id, x, y, r | polygon, zMin?, zMax? (absolute; none: every height), until? (ms on this.clock),
  // kind } until removeTemp(id) or until: free, clearance and plan keep away from it like from a keep-out.
  addTemp(t) {
    this.temps.set(t.id, { ...t });
    this.changed = true; // the clearances follow on the next query (sync)
    return t.id;
  }
  removeTemp(id) {
    const had = this.temps.delete(id);
    this.changed ||= had;
    return had;
  }
  // Drops expired temporary obstacles and follows new ones and changes in the voxels (cheap when nothing changed). Every
  // query here and in planner.js calls it; code that reads map.clear itself calls it first.
  sync() {
    if (!this.base) return this;
    const now = this.temps.size ? this.clock() : 0;
    for (const [id, t] of this.temps) if (t.until != null && t.until <= now) this.changed = this.temps.delete(id) || this.changed;
    if (this.changed) this.overlay();
    else if (this.vox && this.vox.version !== this.voxVersion) this.overlayVoxels();
    return this;
  }
  // flat: the 2.5D clearance with temporary obstacles; clear: that and, with voxels, clearance3 at each band's altitude.
  overlay() {
    if (!this.base) return this;
    this.changed = false;
    const { N, W, H, o, vox } = this;
    let flat = this.base;
    if (this.temps.size) {
      const keep = o.bands.map(() => new Uint8Array(N));
      for (const t of this.temps.values()) this.rasterizeKeepout(t, keep);
      flat = this.blocked.map((bl, b) => clearanceField(bl.map((v, k) => v | keep[b][k]), W, H, o.cell));
    }
    this.flat = flat;
    this.clear = vox ? flat.map((cl, b) => {
      const out = new Float32Array(N), alt = o.bands[b];
      for (let k = 0; k < N; k++) {
        if (!(cl[k] > 0)) continue;
        const [x, y] = this.center(k % W, (k / W) | 0);
        out[k] = Math.min(cl[k], vox.clearance3(x, y, this.floorZ[k] + alt));
      }
      return out;
    }) : flat;
    [this.voxVersion, this.voxOf] = [vox?.version, vox];
    return this.doorClearances();
  }
  // Only the voxels changed: clearance3 moved only within its cap of the voxels blocked since the last overlay (vox.edits),
  // so only those cells, into new arrays (planner.js caches by them). Anything else (a refresh, other voxels) redoes it all.
  overlayVoxels() {
    const { vox, W, H, o } = this, L = vox.edits.list, since = this.voxVersion;
    if (this.voxOf !== vox || !vox.dist || !(vox.edits.from <= since)) return this.overlay();
    let c0 = Infinity, c1 = -1, r0 = Infinity, r1 = -1;
    for (let k = L.length - 2; k >= 0 && L[k] > since; k -= 2) {
      const [x, y] = vox.center(L[k + 1]), c = Math.floor((x - this.x0) / o.cell), r = Math.floor((y - this.y0) / o.cell);
      [c0, c1, r0, r1] = [Math.min(c0, c), Math.max(c1, c), Math.min(r0, r), Math.max(r1, r)];
    }
    this.voxVersion = vox.version;
    if (c1 < 0) return this;
    const R = Math.ceil((vox.clearCap + vox.res) / o.cell) + 1;
    [c0, r0, c1, r1] = [Math.max(0, c0 - R), Math.max(0, r0 - R), Math.min(W - 1, c1 + R), Math.min(H - 1, r1 + R)];
    this.clear = this.clear.map((cl, b) => {
      const out = cl.slice(), flat = this.flat[b], alt = o.bands[b];
      for (let r = r0; r <= r1; r++)
        for (let c = c0, k = r * W + c; c <= c1; c++, k++) {
          if (!(flat[k] > 0)) continue;
          const [x, y] = this.center(c, r);
          out[k] = Math.min(flat[k], vox.clearance3(x, y, this.floorZ[k] + alt));
        }
      return out;
    });
    return this.doorClearances();
  }
  // Best clearance along each doorway, per band.
  doorClearances() {
    for (const d of this.doors) {
      const c = this.o.bands.map(() => 0);
      this.line(d.a, d.b, (k) => this.clear.forEach((cl, b) => (c[b] = Math.max(c[b], cl[k]))));
      d.clearance = c.map((v) => +v.toFixed(3));
    }
    return this;
  }

  rasterizeKeepout(ko, keep) {
    const { o } = this;
    const poly = ko.polygon;
    const xs = poly?.map((p) => p[0]), ys = poly?.map((p) => p[1]);
    const [bx0, by0, bx1, by1] = poly
      ? [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)]
      : [ko.x - ko.r, ko.y - ko.r, ko.x + ko.r, ko.y + ko.r];
    for (let y = by0; y <= by1 + o.cell; y += o.cell)
      for (let x = bx0; x <= bx1 + o.cell; x += o.cell) {
        const k = this.idx(x, y);
        if (k < 0) continue;
        const [cx, cy] = this.center(k % this.W, (k / this.W) | 0);
        if (poly ? !inPolygon(cx, cy, poly) : Math.hypot(cx - ko.x, cy - ko.y) > ko.r) continue;
        o.bands.forEach((alt, b) => {
          const z = this.floorZ[k] + alt;
          if (z + o.bandHalf >= (ko.zMin ?? -Infinity) && z - o.bandHalf <= (ko.zMax ?? Infinity)) keep[b][k] = 1;
        });
      }
  }

  // Hanging splat clusters away from walls, door and window lines: ceiling fans and pendant lamps.
  findFans() {
    const { W, H, N, o, hang } = this;
    const away = clearanceField(this.structure, W, H, o.cell);
    const seen = new Uint8Array(N), parts = [];
    for (let i = 0; i < N; i++) {
      if (seen[i] || hang[i] < o.hangSeed || away[i] < o.hangAway) continue;
      const stack = [i], cells = [];
      seen[i] = 1;
      while (stack.length) {
        const j = stack.pop();
        cells.push(j);
        for (const d of [1, -1, W, -W]) {
          const k = j + d;
          if (k >= 0 && k < N && !seen[k] && hang[k] >= o.hangGrow && away[k] >= o.hangAway - 0.1) {
            seen[k] = 1;
            stack.push(k);
          }
        }
      }
      parts.push(cells);
    }
    const centroid = (cells) => {
      let sx = 0, sy = 0, w = 0;
      for (const j of cells) {
        const [x, y] = this.center(j % W, (j / W) | 0);
        [sx, sy, w] = [sx + x * hang[j], sy + y * hang[j], w + hang[j]];
      }
      return [sx / w, sy / w];
    };
    for (let a = 0; a < parts.length; a++)
      for (let b = a + 1; b < parts.length && parts[a].length; b++) {
        const [p, q] = [centroid(parts[a]), centroid(parts[b])];
        if (parts[b].length && Math.hypot(p[0] - q[0], p[1] - q[1]) < o.fanJoin) [parts[a], parts[b]] = [[...parts[a], ...parts[b]], []];
      }
    return parts
      .filter((cells) => cells.length * o.cell ** 2 >= o.fanMinArea && cells.length * o.cell ** 2 <= o.fanMaxArea)
      .map((cells) => {
        const [x, y] = centroid(cells);
        let w = 0, sz = 0, radius = 0;
        for (const j of cells) {
          const [cx, cy] = this.center(j % W, (j / W) | 0);
          [w, sz, radius] = [w + hang[j], sz + this.hangZ[j], Math.max(radius, Math.hypot(cx - x, cy - y) + o.cell / 2)];
        }
        const r3 = (v) => +v.toFixed(3);
        return { x: r3(x), y: r3(y), z: r3(sz / w), radius: r3(radius), weight: +w.toFixed(1), cells: cells.length };
      })
      .filter((f) => f.radius <= o.fanMaxRadius);
  }

  // Solid wall pieces for the simulator's collision and raycast world: room edges minus doorways, the header above
  // each doorway, and its jambs where the doorway crosses a wall's thickness to the other room. Windows stay solid.
  walls() {
    const out = [];
    const at = (e, t, s = 0) => [+(e.a[0] + e.u[0] * t + e.n[0] * s).toFixed(4), +(e.a[1] + e.u[1] * t + e.n[1] * s).toFixed(4)];
    for (const rm of this.rooms) {
      const top = rm.ceiling?.z ?? rm.floorZ + 2.4;
      const piece = (a, b, more) => out.push({ a, b, zMin: rm.floorZ, zMax: top, room: rm.id, kind: "wall", ...more });
      for (const e of edges(rm)) {
        const gaps = this.gaps(e).filter((g) => g.door.passable).sort((a, b) => a.t0 - b.t0);
        let t = 0;
        for (const g of [...gaps, { t0: e.len, t1: e.len }]) {
          if (g.t0 - t > 0.01) piece(at(e, t), at(e, Math.min(g.t0, e.len)));
          if (!g.door) continue;
          const [t0, t1] = [Math.max(0, g.t0), Math.min(e.len, g.t1)];
          if (g.door.headZ < top) piece(at(e, t0), at(e, t1), { zMin: g.door.headZ, kind: "header", door: g.door.id });
          if (g.off > 0.02) for (const tj of [t0, t1]) piece(at(e, tj), at(e, tj, g.off), { door: g.door.id });
          t = Math.max(t, g.t1);
        }
      }
    }
    return out;
  }

  stats(sigma = this.o.sigma) {
    const a = this.o.cell ** 2, need = this.lethal(sigma);
    const count = (fn) => {
      let n = 0;
      for (let k = 0; k < this.N; k++) n += fn(k) ? 1 : 0;
      return +(n * a).toFixed(1);
    };
    return {
      grid: `${this.W}x${this.H} @ ${this.o.cell} m`,
      splats: this.splats,
      interior_m2: count((k) => this.room[k] >= 0),
      wall_m2: count((k) => this.wall[k]),
      flyable_m2: Object.fromEntries(this.o.bands.map((alt, b) => [alt, count((k) => this.clear[b][k] >= need)])),
      obstacles_m2: Object.fromEntries(this.o.bands.map((alt, b) => [alt, count((k) => this.occ[b][k] && this.room[k] >= 0)])),
      fans: this.fans.length,
      keepouts: this.keepouts.length,
      lethal_m: +need.toFixed(3),
    };
  }
}

// Cells whose summed opacity reaches thr and that have at least `support` such neighbours.
function solid(acc, W, H, thr, support) {
  const out = new Uint8Array(W * H);
  for (let r = 1; r < H - 1; r++)
    for (let c = 1; c < W - 1; c++) {
      const i = r * W + c;
      if (acc[i] < thr) continue;
      let nb = 0;
      for (const d of [1, -1, W, -W, W + 1, W - 1, 1 - W, -1 - W]) if (acc[i + d] >= thr) nb++;
      if (nb >= support) out[i] = 1;
    }
  return out;
}

// Metres from each cell centre to the nearest blocked cell's edge (exact Euclidean distance transform,
// Felzenszwalb-Huttenlocher, between centres, minus half a cell).
export function clearanceField(blocked, W, H, cell) {
  const INF = 1e12, d = new Float64Array(W * H), n = Math.max(W, H);
  const f = new Float64Array(n), v = new Int32Array(n), z = new Float64Array(n + 1), g = new Float64Array(n);
  for (let i = 0; i < W * H; i++) d[i] = blocked[i] ? 0 : INF;
  const pass = (len, get, set) => {
    for (let q = 0; q < len; q++) f[q] = get(q);
    let k = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = INF;
    for (let q = 1; q < len; q++) {
      let s;
      for (;;) {
        s = (f[q] + q * q - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        if (s > z[k]) break;
        k--; // z[0] = -INF stops this at 0
      }
      k++;
      v[k] = q;
      z[k] = s;
      z[k + 1] = INF;
    }
    k = 0;
    for (let q = 0; q < len; q++) {
      while (z[k + 1] < q) k++;
      g[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
    }
    for (let q = 0; q < len; q++) set(q, g[q]);
  };
  for (let c = 0; c < W; c++) pass(H, (r) => d[r * W + c], (r, x) => (d[r * W + c] = x));
  for (let r = 0; r < H; r++) pass(W, (c) => d[r * W + c], (c, x) => (d[r * W + c] = x));
  const out = new Float32Array(W * H);
  for (let i = 0; i < W * H; i++) out[i] = d[i] >= INF / 2 ? Infinity : Math.max(0, Math.sqrt(d[i]) * cell - cell / 2);
  return out;
}
