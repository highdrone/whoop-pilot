// The house in 3D (docs/HOME-DRONE.md, "Wave C contracts"): a 5 cm voxel map in the house frame H where every voxel is
// UNKNOWN until evidence says otherwise, so space no capture ray, twin view or flight ever reached is never treated as free.
// - Occupied: opacity-weighted splat centres (a voxel's summed opacity, with a neighbour that has some too, so lone
//   floaters drop out but a 2 cm chair leg, lamp pole or railing marks), the plan's walls, door heads, floors and ceilings,
//   and what the splat misses but the capture's object list knows (RoomPlan table and cabinet tops: a glass or glossy black
//   top has few or no splats) or the user accepted from a survey (glass, mirrors): FLAG.OBJECT.
// - Free: rays from the capture cameras (a 360° rig, so every direction) to the first surface they meet, carved only when
//   they end on one; a faint layer (some splats, too few to call a surface: glass, a glossy top, a sheer curtain) stops
//   them too and stays unknown for good (FLAG.FAINT), with what is behind it. With a twin, its depth renders at the capture
//   positions and at a frontier sweep of virtual drone viewpoints 0.3-1.6 m above the floors (placed only in space already
//   known free) carve further, but only voxels some capture camera had a line of sight to: the splat vouches only for
//   space a photo looked through.
// - Flight: integrate() adds live depth as log-odds, one update per voxel per frame, only at a good pose, never through
//   capture, plan, object, faint or confirmed surfaces (live depth sees through glass as the capture did); what flights
//   alone found occupied lasts for the flight (forgetFlight(), and serialize() saves without it), free space four frames
//   agreed on is kept and saved. markSeen() keeps when the drone camera last saw each voxel.
// - Confirmed changes (mark(), door()): an obstacle remembers what each voxel was before it (prior), and its removal or
//   an opened door puts that back, so a never-seen voxel is unknown again, never free.
// clearance3 is a 3D distance transform over occupied and unknown voxels, except each room's floor and ceiling layers (the
// 10 cm above the floor and below, the ceiling and above), which the 2.5D map's height rules already keep the drone off, so
// flying 0.6 m up is not 0.6 m from an obstacle. It never reads more than the truth: a voxel that becomes blocked lowers it
// at once within localUpdate, and farther the distance to what was blocked since (pending) caps it; where space was freed
// it stays smaller until refresh(), which runs in a worker in the browser and never on a query. raycast() and state() see
// floors and ceilings like anything else.
import { intrinsics, unproject, resolveLens, pinholeLens } from "../twin/lens.js";
import { droneCamera } from "../twin/pose.js";
import { readSplatPoints } from "./import.js";
import { inPolygon } from "./homemap.js";

export const UNKNOWN = 0, FREE = 1, OCCUPIED = 2;
export const FLAG = { CAPTURE: 1, PLAN: 2, CARVED: 4, TWIN: 8, FLIGHT: 16, CHANGE: 32, OBJECT: 64, FAINT: 128 };
// What flight evidence alone never erases, nor sees through: surfaces, and faint layers (live depth sees through glass too).
const HARD = FLAG.CAPTURE | FLAG.PLAN | FLAG.CHANGE | FLAG.OBJECT | FLAG.FAINT;
// Log-odds (Int8), states at ±T. A flight frame adds one hit or one miss per voxel (a hit wins): unknown is free after
// three frames that agree and occupied after one; free space turns doubtful (unknown, so blocked) after one hit and
// occupied after two. Flight evidence stays within [fmin, fmax], so a flight obstacle is doubtful again after five frames
// of misses; keep: flight free space this sure (four frames) is saved with the map.
const LO = { occ: 64, free: -32, T: 16, hit: 24, miss: 6, fmin: -32, fmax: 40, keep: -24 };
// What integrate() expects an update to cost (ms) before it has timed its own: a voxel turning blocked (stamp()'s ball),
// any other. Each map then follows what its frames measure (a moving average: the machine and its load).
const COST = { stamp: 0.006, voxel: 0.00003 };
const stateOf = (v) => (v >= LO.T ? OCCUPIED : v <= -LO.T ? FREE : UNKNOWN);
const MAGIC = 0x31585657; // "WVX1"
export const VOXEL_VERSION = 3;

export const VOX_DEFAULTS = {
  res: 0.05,
  minOpacity: 0.05, // splats used for occupancy (α), and their largest axis at most maxScale (m): larger ones are fog
  maxScale: 0.2,
  occWeight: 0.3, // summed opacity that makes a voxel occupied...
  support: 1, // ...with at least this many 26-neighbours that reach it too
  faint: 0.03, // summed opacity below occWeight that still stops a carving ray (a dark or glossy surface the splat barely has)
  camRays: 24000, // rays per capture camera (about 1.3° apart)
  camRange: 10, // m
  camClear: 0.12, // m around a capture camera: the operator stood there, so floaters inside are not surfaces
  twinSize: 96, // px per cube face of a twin depth view (90° pinhole, about 0.9° per pixel)
  twinRange: 7, // m: farther depth is too coarse to carve 5 cm voxels
  twinCoverage: 0.6, // splat opacity a pixel needs before its depth counts
  capturePositions: 0.3, // m between the capture positions the twin renders from
  frontierViews: 300, // most virtual views in the sweep (six per viewpoint)
  frontierRound: 16,
  frontierMinGain: 40, // voxels of unseen frontier a viewpoint must face to be worth rendering
  band: [0.3, 1.6], // drone heights above the floor
  clearCap: 2.0, // m: clearance3 never reports more
  localUpdate: 0.5, // m: radius within which clearance3 drops at once when a voxel becomes blocked
  refreshEvery: 1000, // ms: background refreshes after live changes, at most this often
};

// RoomPlan objects with a flat top worth a slab (sofas and chairs: the top is the backrest).
const TOPS = /^(table|desk|storage|counter|cabinet|dresser|shelf|bed|sink|stove|oven|dishwasher|refrigerator|washer|dryer)/i;

// Rays through the grid: Amanatides-Woo over voxel faces, into reusable buffers.
const MAXSTEP = 8192;
const pathI = new Int32Array(MAXSTEP), pathT = new Float32Array(MAXSTEP + 1);
const browser = typeof Worker !== "undefined" && typeof document !== "undefined";

export class VoxelMap {
  constructor({ x0, y0, z0, nx, ny, nz, res = 0.05, key = null, built = null }) {
    Object.assign(this, { x0, y0, z0, nx, ny, nz, res, key, built });
    const n = (this.n = nx * ny * nz);
    this.floorL = new Int16Array(nx * ny); // per column: the first layer above the floor zone, and the last below the ceiling's
    this.ceilL = new Int16Array(nx * ny).fill(nz - 1);
    this.lo = new Int8Array(n);
    this.st = new Uint8Array(n); // UNKNOWN / FREE / OCCUPIED, kept with lo
    this.flags = new Uint8Array(n);
    this.seen = null; // Float32 ms since this.epoch + 1 (0 = never), on the first markSeen
    this.epoch = 0;
    this.dist = null; // clearance3 per voxel, once computed
    this.stale = false; // some of dist may be off until refresh(): too small where space was freed, too large past localUpdate
    this.reach = Math.ceil(VOX_DEFAULTS.localUpdate / res) * res - res / 2; // what stamp() keeps exact
    this.pending = { n: 0, list: new Int32Array(64), min: [0, 0, 0], max: [0, 0, 0] }; // blocked since dist was computed
    this.prior = new Map(); // voxel -> (lo & 255) | flags << 8: what it was before a confirmed change made it occupied
    this.clearCap = VOX_DEFAULTS.clearCap;
    this.version = 0;
    this.edits = { from: 0, list: [] }; // [version, voxel] for each voxel blocked since version `from`: what moved clearance3
  }
  get dirty() {
    return !this.dist;
  }

  // Grid over a HomeMap's plan grid (same x0, y0, cell), z from the lowest floor - 0.1 to the highest ceiling + 0.1.
  static forMap(map, o = {}) {
    const res = o.res ?? map.o.cell;
    let zLo = Infinity, zHi = -Infinity;
    for (let k = 0; k < map.N; k++) {
      if (map.room[k] < 0) continue;
      zLo = Math.min(zLo, map.floorZ[k]);
      if (Number.isFinite(map.ceilZ[k])) zHi = Math.max(zHi, map.ceilZ[k]);
    }
    const scale = map.o.cell / res, z0 = +(zLo - 0.1).toFixed(4);
    const v = new VoxelMap({ x0: map.x0, y0: map.y0, z0, nx: Math.ceil(map.W * scale), ny: Math.ceil(map.H * scale),
      nz: Math.ceil((zHi + 0.1 - z0) / res), res, key: o.key ?? null });
    for (let r = 0; r < v.ny; r++)
      for (let c = 0; c < v.nx; c++) {
        const k = map.idx(v.x0 + (c + 0.5) * res, v.y0 + (r + 0.5) * res);
        if (k < 0 || map.room[k] < 0 || map.wall[k]) continue;
        v.floorL[r * v.nx + c] = Math.max(0, Math.ceil((map.floorZ[k] + 0.1 - z0) / res - 0.5));
        if (Number.isFinite(map.ceilZ[k])) v.ceilL[r * v.nx + c] = Math.min(v.nz - 1, Math.floor((map.ceilZ[k] - 0.05 - z0) / res - 0.5));
      }
    return v;
  }

  idx(x, y, z) {
    const c = Math.floor((x - this.x0) / this.res), r = Math.floor((y - this.y0) / this.res), l = Math.floor((z - this.z0) / this.res);
    return c < 0 || r < 0 || l < 0 || c >= this.nx || r >= this.ny || l >= this.nz ? -1 : (l * this.ny + r) * this.nx + c;
  }
  center(i) {
    const { nx, ny, res } = this, c = i % nx, r = ((i / nx) | 0) % ny, l = (i / (nx * ny)) | 0;
    return [this.x0 + (c + 0.5) * res, this.y0 + (r + 0.5) * res, this.z0 + (l + 0.5) * res];
  }
  state(x, y, z) {
    const i = this.idx(x, y, z);
    return i < 0 ? UNKNOWN : this.st[i];
  }
  // Metres from (x, y, z) to the nearest occupied or unknown voxel (centre distance minus half a voxel, at most clearCap).
  // Never more than the truth, so sphere tracing on it is safe: past localUpdate of a voxel blocked since the last
  // refresh, the distance to it (or, with many such voxels, to their bounding box) caps the stored value.
  clearance3(x, y, z) {
    const i = this.idx(x, y, z);
    if (i < 0) return 0;
    if (!this.dist) this.computeClearance(); // only before the first refresh()
    const d = this.dist[i], p = this.pending;
    return p.n && d > this.reach ? Math.min(d, this.pendingDist(i, d)) : d;
  }
  // Lower bound on the distance from voxel i to what was blocked since the last refresh, beyond localUpdate (stamp()).
  pendingDist(i, cap) {
    const { nx, ny, res } = this, p = this.pending, c = i % nx, r = ((i / nx) | 0) % ny, l = (i / (nx * ny)) | 0;
    const gap = (v, a, b) => (v < a ? a - v : v > b ? v - b : 0);
    const box = Math.hypot(gap(c, p.min[0], p.max[0]), gap(r, p.min[1], p.max[1]), gap(l, p.min[2], p.max[2])) * res - res / 2;
    if (box >= cap) return cap;
    if (p.n > p.list.length) return Math.max(this.reach, box);
    let best = cap;
    for (let k = 0; k < p.n; k++) {
      const j = p.list[k], e = Math.hypot((j % nx) - c, (((j / nx) | 0) % ny) - r, ((j / (nx * ny)) | 0) - l) * res - res / 2;
      if (e < best) best = e;
    }
    return Math.max(this.reach, best);
  }
  free3(x, y, z, r = 0) {
    return this.state(x, y, z) === FREE && this.clearance3(x, y, z) >= r;
  }

  set(i, v, flags = 0) {
    const was = this.st[i], s = stateOf(v);
    this.lo[i] = v;
    this.flags[i] |= flags;
    if (s === FREE) this.flags[i] &= ~HARD; // hard flags live on surfaces only
    if (s === was) return false;
    this.st[i] = s;
    this.version++;
    if (!this.dist) return true;
    const layer = this.nx * this.ny, col = i % layer, l = (i / layer) | 0;
    if (l < this.floorL[col] || l > this.ceilL[col]) return true;
    this.stale = true;
    if (s === FREE) this._since && (this._since.freed = true);
    else if (was === FREE) {
      this.stamp(i);
      this._since?.blocked.push(i);
      if (this.edits.from < Infinity) this.edits.list.push(this.version, i);
    }
    return true;
  }
  // A voxel became blocked: clearance3 drops at once within localUpdate of it (a precomputed ball of offsets), and farther
  // through pending until the next refresh.
  stamp(i) {
    const { nx, ny, nz, dist, pending: p } = this, b = (this._ball ??= ball(this, VOX_DEFAULTS.localUpdate)), R = b.R;
    const c = i % nx, r = ((i / nx) | 0) % ny, l = (i / (nx * ny)) | 0;
    dist[i] = 0;
    if (p.n < p.list.length) p.list[p.n] = i;
    else this.edits = { from: Infinity, list: [] }; // pending is a box now: clearance3 may drop anywhere near it
    if (!p.n++) [p.min, p.max] = [[c, r, l], [c, r, l]];
    else [c, r, l].forEach((v, a) => ((p.min[a] = Math.min(p.min[a], v)), (p.max[a] = Math.max(p.max[a], v))));
    if (c >= R && r >= R && l >= R && c < nx - R && r < ny - R && l < nz - R) {
      for (let k = 0; k < b.n; k++) {
        const j = i + b.lin[k];
        if (b.d[k] < dist[j]) dist[j] = b.d[k];
      }
      return;
    }
    for (let k = 0; k < b.n; k++) {
      const C = c + b.dc[k], Rr = r + b.dr[k], L = l + b.dl[k];
      if (C < 0 || Rr < 0 || L < 0 || C >= nx || Rr >= ny || L >= nz) continue;
      const j = i + b.lin[k];
      if (b.d[k] < dist[j]) dist[j] = b.d[k];
    }
  }
  // What clearance3 keeps away from: occupied and unknown voxels between each column's floor and ceiling zones.
  blockedMask() {
    const { nx, ny, nz, st, floorL, ceilL } = this, layer = nx * ny, b = new Uint8Array(this.n);
    for (let l = 0, i = 0; l < nz; l++)
      for (let col = 0; col < layer; col++, i++) b[i] = st[i] !== FREE && l >= floorL[col] && l <= ceilL[col] ? 1 : 0;
    return b;
  }
  computeClearance() {
    this.dist = edt3(this.blockedMask(), this.nx, this.ny, this.nz, this.res, VOX_DEFAULTS.clearCap);
    [this.stale, this.pending.n] = [false, 0];
    return this.renewed();
  }
  // clearance3 changed everywhere (HomeMap.sync() redoes its overlay); from here on edits lists what lowers it, while
  // pending lists them one by one.
  renewed() {
    this.edits = { from: this.pending.n > this.pending.list.length ? Infinity : ++this.version, list: [] };
    if (this.edits.from === Infinity) this.version++;
    return this;
  }
  // Recomputes clearance3 off the main thread when it can (a module worker, shared), else here. What became blocked while
  // the worker ran is stamped on top, so the result is never less safe than the map.
  refresh() {
    if (this._refreshing) return this._refreshing;
    if (this.dist && !this.stale) return Promise.resolve(this);
    if (!browser) return Promise.resolve(this.computeClearance());
    const since = (this._since = { blocked: [], freed: false }), mask = this.blockedMask();
    this._lastRefresh = performance.now();
    return (this._refreshing = inWorker("edt", { mask, nx: this.nx, ny: this.ny, nz: this.nz, res: this.res, cap: VOX_DEFAULTS.clearCap },
      [mask.buffer]).then(({ dist }) => {
      [this.dist, this.stale, this.pending.n] = [dist, since.freed || since.blocked.length > 0, 0];
      for (const i of since.blocked) if (this.st[i] !== FREE) this.stamp(i);
      return this.renewed();
    }).finally(() => ([this._since, this._refreshing] = [null, null])));
  }
  // After live changes: a refresh at most every refreshEvery ms, in the background (browser only).
  refreshSoon() {
    if (!this.stale || !browser || this._refreshing || this._soon) return;
    const wait = VOX_DEFAULTS.refreshEvery - (performance.now() - (this._lastRefresh ?? -1e9));
    if (wait <= 0) return void this.refresh().catch(() => {});
    this._soon = setTimeout(() => ((this._soon = null), this.refreshSoon()), wait);
  }
  // Whether a durable surface lies within m of voxel i: occupied or unknown, not made so by flights alone (what a live
  // reading near it most likely saw, give or take the pose's and the depth's error).
  nearSurface(i, m) {
    const { nx, ny, nz, st, flags } = this, b = (this._ball ??= ball(this, VOX_DEFAULTS.localUpdate));
    const c = i % nx, r = ((i / nx) | 0) % ny, l = (i / (nx * ny)) | 0;
    for (let k = 0; k < b.n && b.d[k] <= m; k++) {
      const C = c + b.dc[k], Rr = r + b.dr[k], L = l + b.dl[k];
      if (C < 0 || Rr < 0 || L < 0 || C >= nx || Rr >= ny || L >= nz) continue;
      const j = i + b.lin[k], f = flags[j];
      if (st[j] !== FREE && (!(f & FLAG.FLIGHT) || f & HARD)) return true;
    }
    return false;
  }

  // Voxels along the ray o + t·d (d unit) from t = 0 to maxT into pathI/pathT (entry distances; pathT[n] = the last exit).
  // Stops on the first voxel where ((arr[i] & mask) !== 0) !== invert, which is included: { n, hit, out } (out: left the grid).
  trace(ox, oy, oz, dx, dy, dz, maxT, arr, mask, invert = false) {
    const { x0, y0, z0, nx, ny, nz, res } = this;
    const gx = (ox - x0) / res, gy = (oy - y0) / res, gz = (oz - z0) / res;
    let c = Math.floor(gx), r = Math.floor(gy), l = Math.floor(gz);
    if (c < 0 || r < 0 || l < 0 || c >= nx || r >= ny || l >= nz) return { n: 0, hit: false, out: true };
    const sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1, sz = dz > 0 ? 1 : -1;
    const ax = Math.abs(dx), ay = Math.abs(dy), az = Math.abs(dz);
    const tdx = ax > 1e-12 ? res / ax : Infinity, tdy = ay > 1e-12 ? res / ay : Infinity, tdz = az > 1e-12 ? res / az : Infinity;
    let tx = ax > 1e-12 ? (dx > 0 ? c + 1 - gx : gx - c) * tdx : Infinity;
    let ty = ay > 1e-12 ? (dy > 0 ? r + 1 - gy : gy - r) * tdy : Infinity;
    let tz = az > 1e-12 ? (dz > 0 ? l + 1 - gz : gz - l) * tdz : Infinity;
    let t = 0, n = 0, i = (l * ny + r) * nx + c;
    const sr = sy * nx, sl = sz * nx * ny;
    for (;;) {
      pathI[n] = i;
      pathT[n++] = t;
      const tn = tx < ty ? (tx < tz ? tx : tz) : ty < tz ? ty : tz;
      if (((arr[i] & mask) !== 0) !== invert) return (pathT[n] = Math.min(tn, maxT)), { n, hit: true, out: false };
      if (tn >= maxT || n >= MAXSTEP) return (pathT[n] = Math.min(tn, maxT)), { n, hit: false, out: false };
      t = tn;
      if (tn === tx) {
        c += sx;
        i += sx;
        tx += tdx;
        if (c < 0 || c >= nx) return (pathT[n] = t), { n, hit: false, out: true };
      } else if (tn === ty) {
        r += sy;
        i += sr;
        ty += tdy;
        if (r < 0 || r >= ny) return (pathT[n] = t), { n, hit: false, out: true };
      } else {
        l += sz;
        i += sl;
        tz += tdz;
        if (l < 0 || l >= nz) return (pathT[n] = t), { n, hit: false, out: true };
      }
    }
  }

  // The first non-free voxel along a ray: { d (m to where the ray enters it), state }, or { d: max, state: FREE }.
  raycast([x, y, z], [dx, dy, dz], max = 10) {
    const L = Math.hypot(dx, dy, dz);
    const tr = this.trace(x, y, z, dx / L, dy / L, dz / L, max, this.st, FREE, true);
    if (tr.hit) return { d: pathT[tr.n - 1], state: this.st[pathI[tr.n - 1]] };
    return tr.out ? { d: pathT[tr.n], state: UNKNOWN } : { d: max, state: FREE };
  }

  // One horizontal layer of states for the UI, rows from y0 up like HomeMap. z absolute, or with { map } the height above
  // each cell's floor (cells off the rooms: above the lowest floor).
  slice(z, { map } = {}) {
    const { nx, ny, res } = this, data = new Uint8Array(nx * ny);
    for (let r = 0; r < ny; r++)
      for (let c = 0; c < nx; c++) {
        const x = this.x0 + (c + 0.5) * res, y = this.y0 + (r + 0.5) * res, fl = map ? (map.floorAt(x, y) ?? this.z0 + 0.1) : 0;
        const l = Math.floor((fl + z - this.z0) / res);
        data[r * nx + c] = l >= 0 && l < this.nz ? this.st[(l * ny + r) * nx + c] : UNKNOWN;
      }
    return { width: nx, height: ny, x0: this.x0, y0: this.y0, res, z, data };
  }

  // Live depth at a drone pose (H). depth: a nav/LiveDepth frame { width, height, depth, conf?, mask?, boxes? }, or
  // { width, height, data }, or a Float32Array (metres along each pixel's ray, rows top-down, NaN = none) like Twin.depth;
  // lens as in twin/lens.js (its uptiltDeg places the camera). Only at a good pose (status ok, pose.sigma <= maxSigma: else
  // { skipped: "pose" }); pixels with conf < minConf, under the OSD mask or in a person's or pet's box (normalised, grown
  // 20%) don't count. Each voxel gets one update per frame: a miss where the ray passed at least m = 2σ + 5% of the range
  // in front of what it saw, a hit where it ended (up to hitRange: monocular depth is coarse farther) unless a durable
  // surface within m and a voxel explains the reading (nearSurface: pose error moves readings off known surfaces). Rays stop
  // at capture, plan, object, faint and confirmed surfaces; one that sees well past such a surface (crossing at least half
  // a voxel of it, and farther than the twin's expected depth for that pixel says, when the frame has it: the splat's depth
  // is alpha-blended and runs deeper than its first surface) counts against it, and voxels two rays say so of are
  // reported in contradicted (for nav/changes.js); seeing past a faint layer (glass, a glossy top) is neither evidence of
  // free space nor a change. At most `budget` ms in all (the main thread's control loop comes first): rays stop when the time left would not
  // cover applying what they found (a new obstacle's stamp() costs most), then hits go in first and misses while time is
  // left (out.cut: rays or misses were left to later frames; rays go in an order whose every prefix covers the picture).
  integrate(pose, depth, lens, { source = "flight", t = Date.now(), weight = 1, width, height, maxRange = 5, hitRange = 3, rays = 4800,
    budget = 8, conf = depth.conf, mask = depth.mask, boxes = depth.boxes, expected = depth.expected, minConf = 0.5, maxSigma = 0.1 } = {}) {
    const out = { rays: 0, free: 0, hits: 0, flipped: 0, contradicted: [], source, t, skipped: null, cut: false }, sigma = pose.sigma;
    const until = performance.now() + budget;
    if ((pose.status && pose.status !== "ok") || !(sigma <= maxSigma)) return { ...out, skipped: "pose" };
    let blocking = 0, last = performance.now(); // blocking: hits on free voxels, each a stamp() when applied
    const cost = (this._cost ??= { ...COST });
    const data = depth.depth ?? depth.data ?? depth, w = depth.width ?? width ?? Math.round(Math.sqrt((data.length * 4) / 3));
    const h = depth.height ?? height ?? data.length / w;
    const { res, flags, st } = this, frame = (this._frame ??= new Uint8Array(this.n)), touched = [], past = new Map();
    const hit = Math.max(1, Math.round(LO.hit * weight)), miss = Math.max(1, Math.round(LO.miss * weight));
    const grown = (boxes ?? []).map((b) => [b.x - 0.2 * b.w, b.y - 0.2 * b.h, b.x + 1.2 * b.w, b.y + 1.2 * b.h]);
    const touch = (i, m) => (frame[i] || touched.push(i), (frame[i] |= m));
    this.eachRay(pose, lens, w, h, rays, (u, v, o, d) => {
      const p = v * w + u, dep = data[p];
      if (!(dep > 0.05) || dep > maxRange || (conf && !(conf[p] >= minConf)) || mask?.[p]) return;
      const nu = (u + 0.5) / w, nv = (v + 0.5) / h;
      if (grown.length && grown.some((g) => nu >= g[0] && nu <= g[2] && nv >= g[1] && nv <= g[3])) return;
      if (++out.rays % 32 === 0) {
        const now = performance.now(), chunk = now - last; // the next 32 rays, then applying it all, with a quarter to spare
        last = now;
        if (now + chunk + 1.25 * (blocking * cost.stamp + touched.length * cost.voxel) > until) return (out.cut = true);
      }
      const m = 2 * sigma + 0.05 * dep, tr = this.trace(o[0], o[1], o[2], d[0], d[1], d[2], dep + m, flags, HARD);
      let n = tr.n, end = -1, known = false;
      if (tr.hit) {
        n--;
        if (pathT[n] >= dep - m) known = true; // it ends on a lasting surface or a faint layer: that is what it saw
        else if (st[pathI[n]] === OCCUPIED && pathT[n + 1] - pathT[n] >= res / 2 && !(dep <= expected?.[p] + m))
          past.set(pathI[n], (past.get(pathI[n]) ?? 0) + 1);
      }
      for (let k = 0; k < n; k++) {
        const i = pathI[k];
        if (pathT[k + 1] <= dep - m) touch(i, 1);
        else {
          if (st[i] === OCCUPIED) known = true; // a known surface within the margin explains the reading
          if (pathT[k] <= dep && pathT[k + 1] > dep) end = i;
        }
      }
      if (end < 0 || known || dep > hitRange || frame[end] & 6) return;
      const near = this.nearSurface(end, m + res); // nor a durable surface within the margin (and a voxel) around it
      if (!near && st[end] === FREE) blocking++;
      touch(end, near ? 4 : 2);
    });
    const clamp = (v) => Math.min(LO.fmax, Math.max(LO.fmin, v)), flight = (i, d) => this.set(i, clamp(clamp(this.lo[i]) + d), FLAG.FLIGHT);
    const t0 = performance.now();
    let misses = 0;
    for (const i of touched) {
      const mk = frame[i];
      frame[i] = 0;
      if (mk & 2) (out.hits++, flight(i, hit) && out.flipped++);
      else if (mk & 1) touched[misses++] = i;
    }
    const t1 = performance.now();
    for (let k = 0; k < misses; k++) {
      if ((k & 255) === 255 && performance.now() > until) {
        out.cut = true;
        break;
      }
      out.free++;
      if (flight(touched[k], -miss)) out.flipped++;
    }
    const t2 = performance.now(), ema = (a, b) => 0.7 * a + 0.3 * b;
    if (blocking >= 20) cost.stamp = ema(cost.stamp, Math.max(0.0005, (t1 - t0 - touched.length * cost.voxel) / blocking));
    if (out.free >= 2000) cost.voxel = ema(cost.voxel, Math.max(0.000002, (t2 - t1) / out.free));
    for (const [i, c] of past) if (c >= 2 && out.contradicted.length < 500) out.contradicted.push(this.center(i));
    this.refreshSoon();
    return out;
  }

  // What the drone camera saw at a pose: through known-free space up to the first surface (occupied voxels count as seen,
  // unknown ones don't). A frame of the real size (width, height), its capture time t and the pose at that time;
  // quality { age (ms since capture), luma (mean, 0-255), blurred }: a stale (> 300 ms), dark (< 25) or blurred frame, or
  // a pose that isn't ok, marks nothing. -> { marked, skipped }.
  markSeen(pose, lens, t = Date.now(), { width = 320, height = 240, quality = {}, rays = 3072, maxRange = 8, maxSigma = 0.25 } = {}) {
    const skipped = (pose.status && pose.status !== "ok") || pose.sigma > maxSigma ? "pose" : quality.age > 300 ? "stale"
      : quality.luma < 25 ? "dark" : quality.blurred ? "blurred" : null;
    if (skipped) return { marked: 0, skipped };
    if (!this.seen) [this.seen, this.epoch] = [new Float32Array(this.n), t];
    const s = t - this.epoch + 1, { st, seen } = this;
    let marked = 0;
    this.eachRay(pose, lens, width, height, rays, (u, v, o, d) => {
      const tr = this.trace(o[0], o[1], o[2], d[0], d[1], d[2], maxRange, st, FREE, true);
      const n = tr.hit && st[pathI[tr.n - 1]] !== OCCUPIED ? tr.n - 1 : tr.n;
      for (let k = 0; k < n; k++) seen[pathI[k]] = s;
      marked += n;
    });
    return { marked, skipped: null };
  }
  seenAt(x, y, z) {
    const i = this.idx(x, y, z), s = i < 0 || !this.seen ? 0 : this.seen[i];
    return s ? this.epoch + s - 1 : null;
  }

  // About `rays` rays spread over a w×h image of the lens at a pose: fn(u, v, origin, unit direction in H), true to stop.
  // Coarse to fine (spread()), so any number of rays cut short still covers the picture.
  eachRay(pose, lens, w, h, rays, fn) {
    const L = resolveLens(lens), K = intrinsics(L, w, h), cam = droneCamera(pose, L.uptiltDeg ?? 0), R = cam.R;
    const step = Math.max(1, Math.round(Math.sqrt((w * h) / rays))), s0 = step >> 1;
    for (const q of spread(Math.ceil((w - s0) / step), Math.ceil((h - s0) / step))) {
      const u = s0 + (q & 0xffff) * step, v = s0 + (q >>> 16) * step, c = unproject(K, u + 0.5, v + 0.5);
      if (!c) continue;
      if (fn(u, v, cam.p, [R[0][0] * c[0] + R[0][1] * c[1] + R[0][2] * c[2], R[1][0] * c[0] + R[1][1] * c[1] + R[1][2] * c[2],
        R[2][0] * c[0] + R[2][1] * c[1] + R[2][2] * c[2]]) === true) return;
    }
  }

  // fn(i) for each voxel of a region: { min: [x, y, z], max: [x, y, z] }, a ball { x, y, z, r }, an upright cylinder
  // { x, y, r, zMin?, zMax? } or prism { polygon, zMin?, zMax?, inset? (m inside its edges) }; without zMin/zMax every
  // height. Voxels the box or the height range overlaps; for the ball, cylinder and polygon, those whose centre is inside
  // (the ball and cylinder grown half a voxel).
  region(b, fn) {
    const { res, x0, y0, z0, nx, ny, nz } = this, P = b.polygon, isBall = !b.min && !P && b.z != null && b.zMin == null && b.zMax == null;
    const xs = P?.map((p) => p[0]), ys = P?.map((p) => p[1]);
    const [lo, hi] = b.min ? [b.min, b.max]
      : P ? [[Math.min(...xs), Math.min(...ys), b.zMin ?? -Infinity], [Math.max(...xs), Math.max(...ys), b.zMax ?? Infinity]]
      : isBall ? [[b.x - b.r, b.y - b.r, b.z - b.r], [b.x + b.r, b.y + b.r, b.z + b.r]]
      : [[b.x - b.r, b.y - b.r, b.zMin ?? -Infinity], [b.x + b.r, b.y + b.r, b.zMax ?? Infinity]];
    const span = (a, z, n) => [Math.max(0, Math.floor((lo[a] - z) / res)), Math.min(n - 1, Math.ceil((hi[a] - z) / res) - 1)];
    const [c0, c1] = span(0, x0, nx), [r0, r1] = span(1, y0, ny), [l0, l1] = span(2, z0, nz);
    for (let r = r0; r <= r1; r++)
      for (let c = c0; c <= c1; c++) {
        const x = x0 + (c + 0.5) * res, y = y0 + (r + 0.5) * res;
        if (P && !(inPolygon(x, y, P) && (!b.inset || edgeDistance(x, y, P) >= b.inset))) continue;
        if (!b.min && !P && !isBall && Math.hypot(x - b.x, y - b.y) > b.r + res / 2) continue;
        for (let l = l0; l <= l1; l++) {
          if (isBall && Math.hypot(x - b.x, y - b.y, z0 + (l + 0.5) * res - b.z) > b.r + res / 2) continue;
          fn((l * ny + r) * nx + c);
        }
      }
  }

  // A confirmed change (memory/memory.js) over a region (see region()). OCCUPIED: each voxel that isn't a lasting surface
  // already becomes one (FLAG.CHANGE: flights can't clear it) and its state before is kept (prior). FREE: a voxel a change
  // made occupied goes back to that state (never-seen space is unknown again, never free); else a removal (scan: true,
  // "gone") frees what the scan or the user's objects put there, but not the plan's walls nor what lies behind a surface
  // (unknown), and an opened door (scan: false) only what its closed state at capture filled (FLAG.OBJECT), not the
  // jambs and wall faces the scan has around it. What flights alone made occupied goes back to its state before them.
  // state: OCCUPIED | FREE | "occupied" | "free". Returns the voxels changed.
  mark(box, state, { source = "change", scan = true } = {}) {
    const occ = state === OCCUPIED || state === "occupied", { st, flags, prior } = this, SURFACE = FLAG.CAPTURE | FLAG.OBJECT | FLAG.CHANGE;
    const gone = scan ? SURFACE : FLAG.OBJECT;
    let n = 0;
    const back = (i, lo, f) => ((flags[i] = f), this.set(i, lo), n++);
    this.region(box, (i) => {
      const p = prior.get(i);
      if (occ) {
        if (st[i] === OCCUPIED && flags[i] & HARD) return;
        if (p === undefined) prior.set(i, ((lo, f) => (lo & 255) | (f << 8))(...this.durableAt(i)));
        this.set(i, LO.occ, FLAG.CHANGE);
        n++;
      } else if (p !== undefined) (prior.delete(i), back(i, (p << 24) >> 24, p >> 8));
      else if (st[i] === OCCUPIED && !(flags[i] & HARD)) back(i, ...this.durableAt(i));
      else if (st[i] === OCCUPIED && !(flags[i] & FLAG.PLAN) && flags[i] & gone) back(i, LO.free, flags[i] & ~SURFACE);
    });
    this.changes = [...(this.changes ?? []), { source, state: occ ? OCCUPIED : FREE, box, voxels: n }].slice(-100);
    this.refreshSoon();
    return n;
  }
  // A confirmed door change (house.doors[] entry): closed fills its doorway (the wall's depth either side of its line, sill
  // to head) as a confirmed obstacle; open puts back what was there before it closed, and clears a door closed at capture
  // (built as FLAG.OBJECT, not as the plan's wall). Closing and opening again leaves the map as it was.
  door(d, open) {
    return this.mark({ polygon: doorway(d, Math.max(d.depth ?? 0, 0.12) + this.res), zMin: d.sillZ ?? -Infinity, zMax: d.headZ ?? Infinity },
      open ? FREE : OCCUPIED, { source: "door", scan: false });
  }
  // Things the splat can miss but the capture or the user knows, as OCCUPIED with FLAG.OBJECT (a confirmed change clears
  // them): RoomPlan tops of tables, cabinets and the like (a 6 cm slab under the top, 3 cm inside the footprint), and glass
  // and mirror keep-outs (accepted survey hazards) over their heights. Built with the map; again after a survey is
  // accepted. Returns the voxels set.
  applyObjects(house) {
    let n = 0;
    const occupy = (b) => this.region(b, (i) => (n += this.st[i] !== OCCUPIED && this.set(i, LO.occ, FLAG.OBJECT) ? 1 : 0));
    for (const l of house.landmarks ?? [])
      if (l.source === "roomplan" && l.footprint?.length >= 3 && Number.isFinite(l.top) && TOPS.test(l.name ?? ""))
        occupy({ polygon: l.footprint, inset: 0.03, zMin: l.top - 0.06, zMax: l.top });
    for (const k of house.keepouts ?? [])
      if (/^(glass|mirror)$/.test(k.kind) && (k.polygon || k.r > 0)) occupy({ ...k, zMin: k.zMin ?? -Infinity, zMax: k.zMax ?? Infinity });
    this.refreshSoon();
    return n;
  }

  // The map without what flights alone found: occupied or doubtful voxels go back to what they were before the flight
  // (free if the capture or the twin carved them, else unknown); free space at least four frames agreed on stays (flights
  // never see through a surface or a faint layer, so that is space a drone camera looked through).
  durable() {
    const lo = this.lo.slice(), flags = this.flags.slice();
    for (let i = 0; i < this.n; i++) if (flags[i] & FLAG.FLIGHT && !(flags[i] & HARD) && lo[i] > LO.keep) [lo[i], flags[i]] = this.durableAt(i);
    return { lo, flags };
  }
  // Voxel i's [lo, flags] without what flights alone found (durable()).
  durableAt(i) {
    const f = this.flags[i], lo = this.lo[i];
    return f & FLAG.FLIGHT && !(f & HARD) && lo > LO.keep ? [f & (FLAG.CARVED | FLAG.TWIN) ? LO.free : 0, f & ~FLAG.FLIGHT] : [lo, f];
  }
  // At the end of a flight: forget what it alone found (durable()). Returns the voxels that changed state.
  forgetFlight() {
    const { lo, flags } = this.durable();
    let n = 0;
    for (let i = 0; i < this.n; i++) if (lo[i] !== this.lo[i]) n += this.set(i, lo[i]) ? 1 : 0;
    this.flags.set(flags);
    this.refreshSoon();
    return n;
  }

  counts() {
    const c = [0, 0, 0];
    for (let i = 0; i < this.n; i++) c[this.st[i]]++;
    return { unknown: c[0], free: c[1], occupied: c[2] };
  }
  stats() {
    const c = this.counts(), m3 = this.res ** 3;
    return { grid: `${this.nx}x${this.ny}x${this.nz} @ ${this.res} m`, ...c, free_m3: +(c.free * m3).toFixed(2),
      unknown_m3: +(c.unknown * m3).toFixed(2), occupied_m3: +(c.occupied * m3).toFixed(2), built: this.built };
  }

  // Compact bytes for OPFS /houses/<id>/voxels.bin: a JSON header, then lo, flags, seen and the changes' priors run-length
  // coded. What flights alone found occupied is not saved (durable()).
  serialize() {
    const { x0, y0, z0, nx, ny, nz, res, key, built, epoch } = this, { lo, flags } = this.durable();
    const head = new TextEncoder().encode(JSON.stringify({ version: VOXEL_VERSION, x0, y0, z0, nx, ny, nz, res, key, built, epoch,
      seen: !!this.seen, prior: this.prior.size }));
    const bytes = (a) => new Uint8Array(a.buffer);
    const parts = [rle(bytes(lo)), rle(flags), rle(bytes(this.floorL), 2), rle(bytes(this.ceilL), 2)];
    if (this.seen) parts.push(rle(bytes(this.seen), 4));
    if (this.prior.size) parts.push(rle(bytes(Int32Array.from([...this.prior].flat())), 4));
    const size = 8 + head.length + parts.reduce((s, p) => s + 4 + p.length, 0), out = new Uint8Array(size), dv = new DataView(out.buffer);
    dv.setUint32(0, MAGIC, true);
    dv.setUint32(4, head.length, true);
    out.set(head, 8);
    let o = 8 + head.length;
    for (const p of parts) {
      dv.setUint32(o, p.length, true);
      out.set(p, o + 4);
      o += 4 + p.length;
    }
    return out.buffer;
  }
  static load(buf) {
    const u8 = buf instanceof Uint8Array ? buf : new Uint8Array(buf), dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
    if (u8.byteLength < 8 || dv.getUint32(0, true) !== MAGIC) throw new Error("not a voxel map");
    const len = dv.getUint32(4, true), h = JSON.parse(new TextDecoder().decode(u8.subarray(8, 8 + len)));
    if (h.version !== VOXEL_VERSION) throw new Error(`voxel map version ${h.version} (this app reads ${VOXEL_VERSION})`);
    const v = new VoxelMap(h);
    let o = 8 + len;
    const next = (into, unit) => {
      const n = dv.getUint32(o, true);
      unrle(u8.subarray(o + 4, o + 4 + n), into, unit);
      o += 4 + n;
    };
    next(new Uint8Array(v.lo.buffer), 1);
    next(v.flags, 1);
    next(new Uint8Array(v.floorL.buffer), 2);
    next(new Uint8Array(v.ceilL.buffer), 2);
    if (h.seen) next(new Uint8Array((v.seen = new Float32Array(v.n)).buffer), 4);
    if (h.prior) {
      const a = new Int32Array(2 * h.prior);
      next(new Uint8Array(a.buffer), 4);
      for (let k = 0; k < a.length; k += 2) v.prior.set(a[k], a[k + 1]);
    }
    v.epoch = h.epoch ?? 0;
    for (let i = 0; i < v.n; i++) v.st[i] = stateOf(v.lo[i]);
    return v;
  }
}

// Offsets of a ball of radius R (m) in a map's grid, nearest first, with clearance3's distance to each (centre distance -
// half a voxel).
function ball({ nx, ny, res }, radius) {
  const R = Math.ceil(radius / res), all = [];
  for (let l = -R; l <= R; l++)
    for (let r = -R; r <= R; r++)
      for (let c = -R; c <= R; c++) {
        const e = Math.sqrt(c * c + r * r + l * l);
        if (e <= R) all.push([c, r, l, Math.max(0, e * res - res / 2)]);
      }
  all.sort((a, b) => a[3] - b[3]);
  const col = (k, T) => T.from(all, (q) => q[k]);
  return { R, n: all.length, dc: col(0, Int8Array), dr: col(1, Int8Array), dl: col(2, Int8Array), d: col(3, Float32Array),
    lin: Int32Array.from(all, ([c, r, l]) => (l * ny + r) * nx + c) };
}

// Every point (col | row << 16) of a cols×rows grid once, coarse to fine: the centre, then the centres of 2×2, 4×4, 8×8...
// blocks (each level in bit-reversed order), so every prefix is spread over the whole grid, edges included. Cached.
const spreads = new Map();
function spread(cols, rows) {
  const key = cols * 65536 + rows;
  if (spreads.has(key)) return spreads.get(key);
  const seen = new Uint8Array(cols * rows), out = new Uint32Array(cols * rows);
  let n = 0;
  for (let g = 1, bits = 0; g < 2 * Math.max(cols, rows); g *= 2, bits++) {
    const rev = (i) => [...Array(bits).keys()].reduce((r, b) => r | (((i >> b) & 1) << (bits - 1 - b)), 0);
    for (let t = 0; t < g * g; t++) {
      const c = Math.min(cols - 1, Math.floor(((rev(t % g) + 0.5) * cols) / g));
      const r = Math.min(rows - 1, Math.floor(((rev((t / g) | 0) + 0.5) * rows) / g));
      if (!seen[r * cols + c]) (seen[r * cols + c] = 1), (out[n++] = c | (r << 16));
    }
  }
  spreads.set(key, out);
  return out;
}

// A doorway's footprint: its line a-b grown s either side across it.
function doorway(d, s) {
  const L = Math.hypot(d.b[0] - d.a[0], d.b[1] - d.a[1]) || 1, n = [-(d.b[1] - d.a[1]) / L, (d.b[0] - d.a[0]) / L];
  return [[d.a, -s], [d.b, -s], [d.b, s], [d.a, s]].map(([p, t]) => [p[0] + n[0] * t, p[1] + n[1] * t]);
}
function edgeDistance(x, y, P) {
  let best = Infinity;
  P.forEach((a, k) => {
    const b = P[(k + 1) % P.length], ex = b[0] - a[0], ey = b[1] - a[1];
    const t = Math.max(0, Math.min(1, ((x - a[0]) * ex + (y - a[1]) * ey) / (ex * ex + ey * ey || 1)));
    best = Math.min(best, Math.hypot(x - a[0] - t * ex, y - a[1] - t * ey));
  });
  return best;
}

// The shared voxel worker (voxel-worker.js): { op, ...args } -> result, with { progress } messages on the way.
let worker = null, seq = 0;
const waiting = new Map();
function inWorker(op, args, transfer, onProgress) {
  if (!worker) {
    worker = new Worker(new URL("./voxel-worker.js", import.meta.url), { type: "module" });
    worker.onmessage = ({ data }) => {
      const p = waiting.get(data.id);
      if (!p) return;
      if ("progress" in data) return p.onProgress?.(data.progress);
      waiting.delete(data.id);
      data.error ? p.reject(new Error(data.error)) : p.resolve(data);
    };
    worker.onerror = (e) => {
      for (const p of waiting.values()) p.reject(new Error(e.message || "voxel worker failed"));
      waiting.clear();
      worker = null;
    };
  }
  const id = ++seq;
  return new Promise((resolve, reject) => {
    waiting.set(id, { resolve, reject, onProgress });
    worker.postMessage({ id, op, ...args }, transfer);
  });
}
// Lets the page paint between build steps (a message, which hidden pages don't throttle the way they do timers).
const breathe = () => (browser ? new Promise((r) => {
  const ch = new MessageChannel();
  ch.port1.onmessage = () => (ch.port1.close(), r());
  ch.port2.postMessage(0);
}) : Promise.resolve());

// Run-length code of `unit`-byte words: [count varint][word], repeated.
function rle(u8, unit = 1) {
  const out = [], n = u8.length / unit, same = (a, b) => {
    for (let k = 0; k < unit; k++) if (u8[a * unit + k] !== u8[b * unit + k]) return false;
    return true;
  };
  for (let i = 0; i < n; ) {
    let j = i + 1;
    while (j < n && same(i, j)) j++;
    for (let c = j - i; ; c >>>= 7) {
      if (c < 128) {
        out.push(c);
        break;
      }
      out.push((c & 127) | 128);
    }
    for (let k = 0; k < unit; k++) out.push(u8[i * unit + k]);
    i = j;
  }
  return Uint8Array.from(out);
}
function unrle(src, dst, unit = 1) {
  for (let s = 0, d = 0; s < src.length; ) {
    let c = 0;
    for (let sh = 0; ; sh += 7) {
      const b = src[s++];
      c |= (b & 127) << sh;
      if (b < 128) break;
    }
    for (let k = 0; k < c; k++, d += unit) for (let q = 0; q < unit; q++) dst[d + q] = src[s + q];
    s += unit;
  }
}

// Exact 3D Euclidean distance transform (Felzenszwalb-Huttenlocher, separable): metres from each voxel centre to the
// nearest blocked voxel's centre, minus half a voxel, at most cap.
export function edt3(blocked, nx, ny, nz, res, cap = Infinity) {
  const INF = 1e10, n = nx * ny * nz, d = new Float32Array(n), m = Math.max(nx, ny, nz);
  const f = new Float64Array(m), g = new Float64Array(m), v = new Int32Array(m), zz = new Float64Array(m + 1);
  for (let i = 0; i < n; i++) d[i] = blocked[i] ? 0 : INF;
  const line = (len, base, stride) => {
    let any = false;
    for (let q = 0; q < len; q++) if ((f[q] = d[base + q * stride]) < INF) any = true;
    if (!any) return;
    let k = 0;
    v[0] = 0;
    zz[0] = -INF;
    zz[1] = INF;
    for (let q = 1; q < len; q++) {
      let s;
      for (;;) {
        s = (f[q] + q * q - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        if (s > zz[k]) break;
        k--;
      }
      v[++k] = q;
      zz[k] = s;
      zz[k + 1] = INF;
    }
    k = 0;
    for (let q = 0; q < len; q++) {
      while (zz[k + 1] < q) k++;
      g[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
    }
    for (let q = 0; q < len; q++) d[base + q * stride] = g[q];
  };
  const layer = nx * ny;
  for (let l = 0; l < nz; l++) for (let r = 0; r < ny; r++) line(nx, l * layer + r * nx, 1);
  for (let l = 0; l < nz; l++) for (let c = 0; c < nx; c++) line(ny, l * layer + c, nx);
  for (let i = 0; i < layer; i++) line(nz, i, layer);
  for (let i = 0; i < n; i++) d[i] = d[i] >= INF / 2 ? cap : Math.min(cap, Math.max(0, Math.sqrt(d[i]) * res - res / 2));
  return d;
}

// What the voxels were built from: a different value means the house's rooms, frame, doorways or capture changed (rebuild).
// A door opening or closing is not in it: that is applied in place (door()), so what flights learnt stays.
export function voxelKey(house) {
  const pick = { v: VOXEL_VERSION, id: house.id, frame: [house.frame?.f, house.frame?.Yf],
    rooms: (house.rooms ?? []).map((r) => [r.id, r.outline, r.floorZ, r.ceiling]),
    doors: (house.doors ?? []).map((d) => [d.a, d.b, d.sillZ, d.headZ]), cameras: house.cameras?.length ?? 0 };
  let h = 0x811c9dc5;
  for (const ch of JSON.stringify(pick)) h = Math.imul(h ^ ch.charCodeAt(0), 0x01000193) >>> 0;
  return h.toString(16).padStart(8, "0");
}

// ------------------------------------------------------------------------------------------------------------------
// Building the map

// The splat's centres for buildVoxels from its file (SPZ, .splat, PLY): every splat down to 5% opacity, in H. The faint ones
// carry thin things: on the reference capture the stair railing's balusters mark at every height with them, and with only
// the opaque ones (splat-centres.bin) it has gaps of up to 1.9 m.
export const VOXEL_SPLATS = { minOpacity: 13, maxScale: 0.3 };
export async function voxelCentres(bytes, name, frame) {
  const p = await readSplatPoints(bytes, name, VOXEL_SPLATS);
  return { n: p.n, xyz: frame.pointsToH(p.xyz), opacity: p.opacity, scale: p.scale.map((s) => s * frame.f) };
}

// centres: the splat's centres in H { xyz, opacity (0..255 bytes or 0..1), scale (largest axis, m) }, as many as you have
// (voxelCentres). twin: a Twin (twin/twin.js) to carve from its depth renders. onProgress({ phase: "splats" | "capture" |
// "twin" | "frontier" | "clearance" | "done", done, total, text }). The capture cameras' rays run in the voxel worker in
// the browser, and the page gets a turn between steps. Returns the VoxelMap; .built has the timings and whether it is
// usable: without a splat (no centres) or a capture path (house.cameras in the house) nothing can be told free, every
// room stays unknown, and built.why says so in plain words (fly on the 2.5D map then: HomeMap.setVoxels would refuse
// every plan).
export async function buildVoxels({ house, map, centres, twin = null, onProgress = () => {}, options = {} }) {
  const o = { ...VOX_DEFAULTS, ...options }, t0 = performance.now(), ms = {};
  const vox = VoxelMap.forMap(map, { res: o.res, key: voxelKey(house) });
  const lap = (k, t) => (ms[k] = Math.round(performance.now() - t));
  const scan = centres?.opacity?.length > 0;
  let t = performance.now();
  onProgress({ phase: "splats", done: 0, total: 1, text: "Marking surfaces from the scan" });
  const { used: splats, faint: faint0 } = scan ? occupyFromSplats(vox, centres, o) : { used: 0, faint: null };
  await breathe();
  occupyPlan(vox, map, house);
  const objects = vox.applyObjects(house);
  lap("surfaces", t);
  await breathe();
  t = performance.now();
  const cams = scan ? (house.cameras ?? []).filter((c) => vox.idx(c[0], c[1], c[2]) >= 0) : [];
  const progress = (done) => onProgress({ phase: "capture", done, total: cams.length, text: "Tracing the capture camera's rays" });
  const { visible, faint } = cams.length ? await carveCameras(vox, cams, o, faint0, progress) : {};
  const faintMask = faint ?? faint0; // faint layers stay unknown for good: flights don't see through them either
  if (faintMask) for (let i = 0; i < vox.n; i++) if (faintMask[i] && vox.st[i] === UNKNOWN) vox.flags[i] |= FLAG.FAINT;
  lap("capture", t);
  let views = 0;
  if (twin && visible) {
    t = performance.now();
    const solid = new Uint8Array(vox.n);
    for (let i = 0; i < vox.n; i++) solid[i] = vox.st[i] === OCCUPIED || faint[i] ? 1 : 0;
    const spots = [];
    for (const c of cams) if (!spots.some((s) => Math.hypot(s[0] - c[0], s[1] - c[1], s[2] - c[2]) < o.capturePositions)) spots.push(c);
    const aids = { solid, visible };
    views += await carveFromTwin(vox, twin, spots, o, aids, (done) => onProgress({ phase: "twin", done, total: spots.length,
      text: "Looking around from where the scanner was" }));
    lap("twinCapture", t);
    t = performance.now();
    views += await frontierSweep(vox, map, twin, o, aids, (done, total, round) => onProgress({ phase: "frontier", done, total, ...round,
      text: "Flying virtual viewpoints into the gaps" }));
    lap("frontier", t);
  }
  t = performance.now();
  onProgress({ phase: "clearance", done: 0, total: 1, text: "Measuring clearances" });
  await vox.refresh();
  lap("clearance", t);
  const band = bandMask(vox, map, o.band);
  let free = 0;
  for (let i = 0; i < vox.n; i++) free += band[i] && vox.st[i] === FREE ? 1 : 0;
  const why = !scan ? "No 3D scan: the capture's splat wasn't imported, so the 3D map has walls only. Import it again with its splat."
    : !cams.length ? "The capture has no camera path, so the 3D map can't tell space the scanner saw from space it never saw."
    : !free ? "The capture's rays found no free space at flying height." : null;
  vox.built = { at: Date.now(), ms: Math.round(performance.now() - t0), phases: ms, splats, objects, cameras: cams.length, views,
    twin: !!twin, usable: !why, why };
  onProgress({ phase: "done", done: 1, total: 1, text: why ?? "3D map ready" });
  return vox;
}

// Opacity summed per voxel; occupied where it reaches occWeight with `support` neighbours that reach it too; faint where it
// reaches o.faint but isn't occupied (returned as a mask for the carving rays).
function occupyFromSplats(vox, { xyz, opacity, scale }, o) {
  const { n, nx, ny, nz } = vox, acc = new Float32Array(n), k = opacity instanceof Uint8Array ? 1 / 255 : 1;
  let used = 0;
  for (let i = 0, m = opacity.length; i < m; i++) {
    const a = opacity[i] * k;
    if (a < o.minOpacity || scale[i] > o.maxScale) continue;
    const j = vox.idx(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    if (j < 0) continue;
    acc[j] += a;
    used++;
  }
  const thr = o.occWeight, faint = new Uint8Array(n);
  for (let l = 0; l < nz; l++)
    for (let r = 0; r < ny; r++)
      for (let c = 0; c < nx; c++) {
        const i = (l * ny + r) * nx + c;
        if (acc[i] < o.faint) continue;
        faint[i] = 1;
        if (acc[i] < thr) continue;
        let nb = 0;
        for (let dl = -1; dl <= 1 && nb < o.support; dl++)
          for (let dr = -1; dr <= 1 && nb < o.support; dr++)
            for (let dc = -1; dc <= 1; dc++) {
              const L = l + dl, R = r + dr, C = c + dc;
              if ((dl || dr || dc) && L >= 0 && R >= 0 && C >= 0 && L < nz && R < ny && C < nx && acc[(L * ny + R) * nx + C] >= thr) nb++;
            }
        if (nb >= o.support) (vox.set(i, LO.occ, FLAG.CAPTURE), (faint[i] = 0));
      }
  return { used, faint };
}

// The plan: walls floor to top, door heads, and below each room's floor and above its ceiling. A door that was closed
// (not passable) when the map was built fills its doorway between sill and head as FLAG.OBJECT, not the plan's wall, so
// door(d, true) can open it.
function occupyPlan(vox, map, house) {
  const { nx, ny, nz, res, z0 } = vox, shut = new Map();
  for (const d of house.doors ?? []) {
    if (d.passable) continue;
    vox.region({ polygon: doorway(d, Math.max(d.depth ?? 0, 0.12) + res), zMin: z0, zMax: z0 + res / 2 }, (i) => shut.set(i % (nx * ny), d));
  }
  for (let r = 0; r < ny; r++)
    for (let c = 0; c < nx; c++) {
      const x = vox.x0 + (c + 0.5) * res, y = vox.y0 + (r + 0.5) * res, k = map.idx(x, y);
      if (k < 0) continue;
      const wall = map.wall[k], room = map.room[k] >= 0, floor = map.floorZ[k], ceil = map.ceilZ[k], head = map.headZ[k];
      if (!wall && !room && !Number.isFinite(head)) continue;
      const door = wall && shut.get(r * nx + c), sill = door?.sillZ ?? -Infinity, top = door?.headZ ?? Infinity;
      for (let l = 0; l < nz; l++) {
        const z = z0 + (l + 0.5) * res;
        if (!(wall || (room && (z < floor || z > ceil)) || z > head)) continue;
        vox.set((l * ny + r) * nx + c, LO.occ, door && z > sill && z < top ? FLAG.OBJECT : FLAG.PLAN);
      }
    }
}

// Fibonacci sphere directions, the same on every run.
function sphere(n) {
  const out = new Float32Array(3 * n), ga = Math.PI * (3 - Math.sqrt(5));
  for (let i = 0; i < n; i++) {
    const z = 1 - (2 * (i + 0.5)) / n, s = Math.sqrt(1 - z * z);
    out.set([s * Math.cos(ga * i), s * Math.sin(ga * i), z], 3 * i);
  }
  return out;
}

// Free from a ray; flag CARVED for the capture camera's own rays, TWIN for the twin's renders (inferred from the model).
const carve = (vox, i, flag = FLAG.CARVED) => vox.st[i] === UNKNOWN && vox.set(i, LO.free, flag);

// carveFromCameras in the voxel worker in the browser (the arrays go there and come back), else here. -> { visible, faint }
async function carveCameras(vox, cams, o, faint, progress) {
  if (!browser) return { visible: carveFromCameras(vox, cams, o, faint, progress), faint };
  const { x0, y0, z0, nx, ny, nz, res } = vox;
  const r = await inWorker("carve", { grid: { x0, y0, z0, nx, ny, nz, res }, lo: vox.lo, st: vox.st, flags: vox.flags, faint, cams, o },
    [vox.lo.buffer, vox.st.buffer, vox.flags.buffer, faint.buffer], progress);
  [vox.lo, vox.st, vox.flags] = [r.lo, r.st, r.flags];
  vox.version++;
  return { visible: r.visible, faint: r.faint };
}

// From each capture camera, rays in every direction (the rig is a 360° camera) carve up to the first surface they meet,
// and only when they meet one in range: a ray that leaves the house or slips through the scan carves nothing. Rays stop
// one voxel before surfaces dilated by a voxel, so they cannot slip through the gaps of a sparse surface, and at faint
// voxels, which stay unknown with what is behind them. Returns the voxels some camera had a line of sight to (up to the
// first surface: the twin may carve only those).
export function carveFromCameras(vox, cams, o, faint, progress = () => {}) {
  const { n, nx, ny, nz, st } = vox, layer = nx * ny, stop = new Uint8Array(n), visible = new Uint8Array(n), dirs = sphere(o.camRays);
  for (const [x, y, z] of cams) {
    const R = Math.ceil(o.camClear / vox.res), i0 = vox.idx(x, y, z), c0 = i0 % nx, r0 = ((i0 / nx) | 0) % ny, l0 = (i0 / layer) | 0;
    for (let dl = -R; dl <= R; dl++)
      for (let dr = -R; dr <= R; dr++)
        for (let dc = -R; dc <= R; dc++) {
          const C = c0 + dc, Rr = r0 + dr, L = l0 + dl, j = (L * ny + Rr) * nx + C;
          if (C < 0 || Rr < 0 || L < 0 || C >= nx || Rr >= ny || L >= nz || (dc * dc + dr * dr + dl * dl) * vox.res ** 2 > o.camClear ** 2) continue;
          if (vox.flags[j] === FLAG.CAPTURE) vox.flags[j] = 0, vox.set(j, 0);
          if (faint) faint[j] = 0;
        }
  }
  for (let i = 0; i < n; i++) {
    if (faint?.[i]) stop[i] |= 2;
    if (st[i] !== OCCUPIED) continue;
    stop[i] |= 5;
    const c = i % nx, r = ((i / nx) | 0) % ny;
    if (c > 0) stop[i - 1] |= 1;
    if (c < nx - 1) stop[i + 1] |= 1;
    if (r > 0) stop[i - nx] |= 1;
    if (r < ny - 1) stop[i + nx] |= 1;
    if (i >= layer) stop[i - layer] |= 1;
    if (i + layer < n) stop[i + layer] |= 1;
  }
  cams.forEach(([x, y, z], ci) => {
    const i0 = vox.idx(x, y, z);
    if (stop[i0] & 1) return; // inside a surface the plan put there (a camera over the stairwell)
    for (let k = 0; k < o.camRays; k++) {
      const tr = vox.trace(x, y, z, dirs[3 * k], dirs[3 * k + 1], dirs[3 * k + 2], o.camRange, stop, 4); // to the first surface
      for (let q = 0; q < tr.n - (tr.hit ? 1 : 0); q++) visible[pathI[q]] = 1;
      let q = 0;
      while (q < tr.n && !(stop[pathI[q]] & 3)) q++; // where carving stops: next to a surface, or a faint layer
      if (q === tr.n) continue;
      for (let p = 0; p < q; p++) carve(vox, pathI[p]);
      const last = pathI[q];
      // The voxel in front of a surface the ray meets head on (not one beside the ray) is free too; a faint one and what is
      // behind it stay unknown.
      if (!(stop[last] & 6) && tr.hit && pathT[tr.n - 1] < pathT[q] + 1.5 * vox.res) carve(vox, last);
    }
    if (ci % 10 === 9) progress(ci + 1);
  });
  progress(cams.length);
  return visible;
}

// The six 90° faces of a cube at a position: yaw 0, 90, 180, 270°, then down and up (pitch is + nose down).
const FACES = [[0, 0], [Math.PI / 2, 0], [Math.PI, 0], [-Math.PI / 2, 0], [0, Math.PI / 2], [0, -Math.PI / 2]];

// Twin depth cubes at positions [[x, y, z], ...]: rays carve to just short of the rendered surface (the nearest depth around
// each pixel, so an object's edge is not carved through), stop at occupied and faint voxels (aids.solid) and carve only
// voxels a capture camera had a line of sight to (aids.visible); a smooth surface pixel marks its voxel occupied when
// nothing else knew it. Only pixels the splats cover (twinCoverage) count. The next cube renders while this one is carved.
async function carveFromTwin(vox, twin, spots, o, aids, progress) {
  const S = o.twinSize, lens = pinholeLens(90), K = intrinsics(lens, S, S), view = { width: S, height: S, lens, actors: false, props: false,
    minCoverage: o.twinCoverage };
  const cam = new Float32Array(3 * S * S);
  for (let v = 0; v < S; v++) for (let u = 0; u < S; u++) cam.set(unproject(K, u + 0.5, v + 0.5), 3 * (v * S + u));
  const poses = (p) => FACES.map(([yaw, pitch]) => ({ x: p[0], y: p[1], z: p[2], yaw, pitch, roll: 0 }));
  let next = spots.length ? twin.depthBatch(poses(spots[0]), view) : null;
  for (let s = 0; s < spots.length; s++) {
    const depths = await next;
    next = s + 1 < spots.length ? twin.depthBatch(poses(spots[s + 1]), view) : null;
    for (const [f, pose] of poses(spots[s]).entries()) {
      carveDepth(vox, pose, droneCamera(pose, 0).R, cam, depths[f], S, o, aids);
      if (f % 2) await breathe();
    }
    progress(s + 1);
  }
  return spots.length * FACES.length;
}

function carveDepth(vox, pose, R, cam, depth, S, o, { solid, visible }) {
  const res = vox.res, o3 = [pose.x, pose.y, pose.z];
  for (let v = 0; v < S; v++)
    for (let u = 0; u < S; u++) {
      const d = depth[v * S + u];
      if (!(d > 0.05) || d > o.twinRange) continue;
      let lo = d, hi = d;
      for (let dv = -1; dv <= 1; dv++)
        for (let du = -1; du <= 1; du++) {
          const q = depth[Math.min(S - 1, Math.max(0, v + dv)) * S + Math.min(S - 1, Math.max(0, u + du))];
          if (q < lo) lo = q;
          if (q > hi) hi = q;
        }
      const j = 3 * (v * S + u), cx = cam[j], cy = cam[j + 1], cz = cam[j + 2];
      const dx = R[0][0] * cx + R[0][1] * cy + R[0][2] * cz, dy = R[1][0] * cx + R[1][1] * cy + R[1][2] * cz;
      const dz = R[2][0] * cx + R[2][1] * cy + R[2][2] * cz;
      const tr = vox.trace(o3[0], o3[1], o3[2], dx, dy, dz, d + res / 2, solid, 1);
      for (let k = 0; k < tr.n && !(tr.hit && k === tr.n - 1) && pathT[k + 1] <= lo - res / 4; k++)
        if (visible[pathI[k]]) carve(vox, pathI[k], FLAG.TWIN);
      if (tr.hit || hi - lo > 0.03 * d + 0.02) continue;
      const end = vox.idx(o3[0] + dx * d, o3[1] + dy * d, o3[2] + dz * d);
      if (end >= 0 && vox.st[end] === UNKNOWN) (vox.set(end, LO.occ, FLAG.CAPTURE | FLAG.TWIN), (solid[end] = 1));
    }
}

// Next-best views: unknown voxels at drone heights that touch known-free space (the frontier) and that a capture camera
// had a line of sight to pull viewpoints, placed in free space with 10 cm all round at 0.5, 1.0 or 1.5 m above the floor;
// a frontier voxel that stays unknown after two views that faced it stops pulling (the inside of a cabinet is never
// going to show).
async function frontierSweep(vox, map, twin, o, aids, progress) {
  const { nx, ny, nz, n, res, st } = vox, layer = nx * ny, tried = new Uint8Array(n), band = bandMask(vox, map, o.band), { visible } = aids;
  const B = 30, H = 12, used = []; // gain box half-sizes: ±1.5 m across, ±0.6 m up and down
  let views = 0;
  for (let round = 0; views < o.frontierViews; round++) {
    await breathe();
    const fr = new Uint8Array(n);
    let any = 0;
    for (let i = 0; i < n; i++) {
      if (!band[i] || st[i] !== UNKNOWN || tried[i] >= 2 || !visible[i]) continue;
      const c = i % nx, r = ((i / nx) | 0) % ny;
      if ((c > 0 && st[i - 1] === FREE) || (c < nx - 1 && st[i + 1] === FREE) || (r > 0 && st[i - nx] === FREE) || (r < ny - 1 && st[i + nx] === FREE)
        || (i >= layer && st[i - layer] === FREE) || (i + layer < n && st[i + layer] === FREE)) (fr[i] = 1), any++;
    }
    if (!any) break;
    await breathe();
    const sum = prefix3(fr, nx, ny, nz), box = (c, r, l) => boxSum(sum, nx, ny, nz, c - B, r - B, l - H, c + B, r + B, l + H);
    const cands = [];
    for (let r = 2; r < ny - 2; r += 4)
      for (let c = 2; c < nx - 2; c += 4) {
        const k = map.idx(vox.x0 + (c + 0.5) * res, vox.y0 + (r + 0.5) * res);
        if (k < 0 || map.room[k] < 0 || map.wall[k]) continue;
        for (const h of [0.5, 1.0, 1.5]) {
          const l = Math.floor((map.floorZ[k] + h - vox.z0) / res);
          if (l < 2 || l >= nz - 2 || !roomy(vox, c, r, l)) continue;
          const g = box(c, r, l);
          if (g >= o.frontierMinGain) cands.push({ c, r, l, g });
        }
      }
    await breathe();
    cands.sort((a, b) => b.g - a.g || a.l - b.l || a.r - b.r || a.c - b.c);
    const pick = [];
    for (const q of cands) {
      if (pick.length >= Math.min(o.frontierRound, o.frontierViews - views)) break;
      const far = (p) => (p.c - q.c) ** 2 + (p.r - q.r) ** 2 + (p.l - q.l) ** 2 >= 16 ** 2;
      if (pick.every(far) && used.every(far)) pick.push(q);
    }
    if (!pick.length) break;
    const spots = pick.map((q) => vox.center((q.l * ny + q.r) * nx + q.c));
    const info = { round, frontier: any, gains: pick.map((q) => q.g), spots };
    await carveFromTwin(vox, twin, spots, o, aids, (done) => progress(views + done * FACES.length, views + pick.length * FACES.length, info));
    views += pick.length * FACES.length;
    used.push(...pick);
    for (const q of pick)
      for (let l = Math.max(0, q.l - H); l <= Math.min(nz - 1, q.l + H); l++)
        for (let r = Math.max(0, q.r - B); r <= Math.min(ny - 1, q.r + B); r++)
          for (let c = Math.max(0, q.c - B); c <= Math.min(nx - 1, q.c + B); c++) {
            const i = (l * ny + r) * nx + c;
            if (fr[i] && st[i] === UNKNOWN && tried[i] < 255) tried[i]++;
          }
  }
  return views;
}

// Free with every voxel within two (10 cm) free.
function roomy(vox, c, r, l) {
  const { nx, ny, nz, st } = vox;
  for (let dl = -2; dl <= 2; dl++)
    for (let dr = -2; dr <= 2; dr++)
      for (let dc = -2; dc <= 2; dc++) {
        const C = c + dc, R = r + dr, L = l + dl;
        if (C < 0 || R < 0 || L < 0 || C >= nx || R >= ny || L >= nz || st[(L * ny + R) * nx + C] !== FREE) return false;
      }
  return true;
}

// Voxels over a room cell (not a wall) between band[0] and band[1] above its floor.
export function bandMask(vox, map, [h0, h1] = VOX_DEFAULTS.band) {
  const { nx, ny, nz, res } = vox, out = new Uint8Array(vox.n);
  for (let r = 0; r < ny; r++)
    for (let c = 0; c < nx; c++) {
      const k = map.idx(vox.x0 + (c + 0.5) * res, vox.y0 + (r + 0.5) * res);
      if (k < 0 || map.room[k] < 0 || map.wall[k]) continue;
      for (let l = 0; l < nz; l++) {
        const h = vox.z0 + (l + 0.5) * res - map.floorZ[k];
        if (h >= h0 && h <= h1) out[(l * ny + r) * nx + c] = 1;
      }
    }
  return out;
}

// Summed-volume table (one larger on each axis) and box sums over it, clipped to the grid.
function prefix3(a, nx, ny, nz) {
  const X = nx + 1, Y = ny + 1, s = new Int32Array(X * Y * (nz + 1));
  for (let l = 1; l <= nz; l++)
    for (let r = 1; r <= ny; r++)
      for (let c = 1; c <= nx; c++) {
        const i = (l * Y + r) * X + c;
        s[i] = a[((l - 1) * ny + r - 1) * nx + c - 1] + s[i - 1] + s[i - X] + s[i - X * Y] - s[i - X - 1] - s[i - X * Y - 1] - s[i - X * Y - X]
          + s[i - X * Y - X - 1];
      }
  return s;
}
function boxSum(s, nx, ny, nz, c0, r0, l0, c1, r1, l1) {
  [c0, r0, l0] = [Math.max(0, c0), Math.max(0, r0), Math.max(0, l0)];
  [c1, r1, l1] = [Math.min(nx - 1, c1) + 1, Math.min(ny - 1, r1) + 1, Math.min(nz - 1, l1) + 1];
  if (c1 <= c0 || r1 <= r0 || l1 <= l0) return 0;
  const X = nx + 1, Y = ny + 1, at = (c, r, l) => s[(l * Y + r) * X + c];
  return at(c1, r1, l1) - at(c0, r1, l1) - at(c1, r0, l1) - at(c1, r1, l0) + at(c0, r0, l1) + at(c0, r1, l0) + at(c1, r0, l0) - at(c0, r0, l0);
}
