// Paths through the house on a HomeMap (homemap.js), all in the house frame H: a room graph through passable
// doorways, A* on the grid inflated by the drone radius plus the position uncertainty, string-pulling and corner
// smoothing, search viewpoints by greedy set cover, and patrol routes. The map's clearances carry its overlays
// (temporary obstacles; with voxels, only known-free space at each band's height), so every plan keeps to them.

const DOOR_STANDOFF = 0.6; // door waypoints this far either side of the doorway
const SOFT = 0.6; // m: closer than this to an obstacle costs extra, so paths keep to the middle
const pt = (p) => (Array.isArray(p) ? p : [p.x, p.y, p.z]);

// n points from rooms[0] to rooms[1]; mid is the middle of the passage (a deep doorway's line is on rooms[0]'s face).
function doorFrame(map, d) {
  const L = Math.hypot(d.b[0] - d.a[0], d.b[1] - d.a[1]);
  let mid = [(d.a[0] + d.b[0]) / 2, (d.a[1] + d.b[1]) / 2], n = [-(d.b[1] - d.a[1]) / L, (d.b[0] - d.a[0]) / L];
  const side = (s) => map.roomAt(mid[0] + s * DOOR_STANDOFF * n[0], mid[1] + s * DOOR_STANDOFF * n[1])?.id;
  if (side(1) === d.rooms[0] || side(-1) === d.rooms[1]) n = [-n[0], -n[1]];
  if (d.depth) mid = [mid[0] + (n[0] * d.depth) / 2, mid[1] + (n[1] * d.depth) / 2];
  const at = (s) => [mid[0] + s * DOOR_STANDOFF * n[0], mid[1] + s * DOOR_STANDOFF * n[1]];
  return { mid, n, wa: at(-1), wb: at(1) };
}

// Rooms joined by passable doorways; `open` says whether the doorway has room for the drone at this σ and altitude.
// With climb, a doorway is open if any band fits; clearance is the best band's.
export function roomGraph(map, { alt = 1.0, sigma = map.o.sigma, climb = true } = {}) {
  map.sync?.();
  const b0 = map.bandOf(alt), need = map.lethal(sigma);
  const edges = map.doors
    .filter((d) => d.passable && d.rooms[1] && d.rooms[0] !== d.rooms[1])
    .map((d) => {
      const { mid, wa, wb } = doorFrame(map, d), clearance = climb ? Math.max(0, ...(d.clearance ?? [])) : (d.clearance?.[b0] ?? 0);
      return { door: d.id, a: d.rooms[0], b: d.rooms[1], mid, wa, wb, width: d.width, clearance, open: clearance >= need };
    });
  return { rooms: map.rooms.map((r) => r.id), edges };
}

// Fewest doorways from one room to another over open edges, or null.
export function roomRoute(graph, from, to) {
  const prev = new Map([[from, null]]), queue = [from];
  while (queue.length) {
    const r = queue.shift();
    if (r === to) break;
    for (const e of graph.edges) {
      if (!e.open || (e.a !== r && e.b !== r)) continue;
      const next = e.a === r ? e.b : e.a;
      if (!prev.has(next)) prev.set(next, { r, e }), queue.push(next);
    }
  }
  if (!prev.has(to)) return null;
  const out = [];
  for (let r = to; prev.get(r); r = prev.get(r).r) out.unshift(prev.get(r).e);
  return out;
}

// The nearest cell with clearance >= need within maxR of p, as [x, y], or null.
export function nearestFree(map, p, { alt = 1.0, band = map.bandOf(alt), sigma = map.o.sigma, maxR = 1.0 } = {}) {
  map.sync?.();
  const need = map.lethal(sigma), cl = map.clear[band], { cell } = map.o;
  const k0 = map.idx(p[0], p[1]);
  if (k0 >= 0 && cl[k0] >= need) return [p[0], p[1]];
  const c0 = Math.floor((p[0] - map.x0) / cell), r0 = Math.floor((p[1] - map.y0) / cell), R = Math.ceil(maxR / cell);
  let best = null, bd = Infinity;
  for (let dr = -R; dr <= R; dr++)
    for (let dc = -R; dc <= R; dc++) {
      const c = c0 + dc, r = r0 + dr, d = dc * dc + dr * dr;
      if (d >= bd || d > R * R || c < 0 || r < 0 || c >= map.W || r >= map.H || cl[r * map.W + c] < need) continue;
      [best, bd] = [map.center(c, r), d];
    }
  return best;
}

// The free cell farthest from obstacles in a room: the "go to <room>" goal and a good place to look around. With voxels
// (whose clearance flattens out wherever the 3D space is roomy) the farthest on the plan among cells near the best in 3D.
export function roomCenter(map, roomId, { alt = 1.0, sigma = map.o.sigma } = {}) {
  map.sync?.();
  const b = map.bandOf(alt), ri = map.rooms.findIndex((r) => r.id === roomId), cl = map.clear[b], flat = map.flat?.[b] ?? cl;
  const need = map.lethal(sigma);
  let top = -Infinity, best = -1;
  for (let k = 0; k < map.N; k++) if (map.room[k] === ri && cl[k] > top) top = cl[k];
  const enough = Math.min(top, need + 0.25);
  for (let k = 0; k < map.N; k++) if (map.room[k] === ri && cl[k] >= need && cl[k] >= enough && (best < 0 || flat[k] > flat[best])) best = k;
  return best < 0 ? null : map.center(best % map.W, (best / map.W) | 0);
}

function los(map, cl, a, b, need) {
  const L = Math.hypot(b[0] - a[0], b[1] - a[1]), n = Math.ceil(L / (map.o.cell / 2));
  for (let i = 0; i <= n; i++) {
    const k = map.idx(a[0] + ((b[0] - a[0]) * i) / n, a[1] + ((b[1] - a[1]) * i) / n);
    if (k < 0 || cl[k] < need) return false;
  }
  return true;
}

class Heap {
  constructor() {
    this.k = [];
    this.p = [];
  }
  get size() {
    return this.k.length;
  }
  push(key, pri) {
    const { k, p } = this;
    let i = k.length;
    k.push(key);
    p.push(pri);
    while (i > 0) {
      const j = (i - 1) >> 1;
      if (p[j] <= pri) break;
      k[i] = k[j];
      p[i] = p[j];
      i = j;
    }
    k[i] = key;
    p[i] = pri;
  }
  pop() {
    const { k, p } = this, top = k[0], lk = k.pop(), lp = p.pop();
    if (k.length) {
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = -1;
        if (l < k.length && p[l] < lp) m = l;
        if (r < k.length && p[r] < (m < 0 ? lp : p[l])) m = r;
        if (m < 0) break;
        k[i] = k[m];
        p[i] = p[m];
        i = m;
      }
      k[i] = lk;
      p[i] = lp;
    }
    return top;
  }
}

// from, to: [x, y(, z)] or {x, y}. alt: preferred metres above the local floor (the map's nearest band); with
// climb the path may change band, over furniture or under nothing, where the preferred one is blocked: a climb
// costs twice its height and every metre away from the preferred band a quarter more. sigma: 1σ horizontal
// position uncertainty, inflated with the drone radius.
// -> { ok, path: [[x, y, z]], length, climb, doors: [{ id, rooms, pre, post }], minClearance, ... } | { ok: false, reason }
export function plan(map, from, to, { alt = 1.0, sigma = map.o.sigma, step = 0.1, snap = 1.0, climb = true } = {}) {
  if (!from || !to) return { ok: false, reason: from ? "no goal" : "no start" };
  map.sync?.();
  const need = map.lethal(sigma), { W, H, N } = map, B = map.o.bands, b0 = map.bandOf(alt), { cell } = map.o;
  const order = (climb ? B.map((_, b) => b) : [b0]).sort((x, y) => Math.abs(x - b0) - Math.abs(y - b0));
  const allowed = new Uint8Array(B.length);
  order.forEach((b) => (allowed[b] = 1));
  const [fp, tp] = [pt(from), pt(to)];
  const end = (p) => {
    for (const b of order) {
      const q = nearestFree(map, p, { band: b, sigma, maxR: snap });
      if (q) return { p: q, b };
    }
    return null;
  };
  const s = end(fp), g = end(tp), at = (p) => `(${p[0].toFixed(2)}, ${p[1].toFixed(2)})`;
  const sg = +sigma.toFixed(2);
  if (!s || !g) return { ok: false, reason: `${s ? "goal" : "start"} ${at(s ? tp : fp)} has no free space within ${snap} m (σ ${sg} m)` };
  const S = B.length * N, si = s.b * N + map.idx(...s.p), gi = g.b * N + map.idx(...g.p);
  if (map._g?.length !== S) [map._g, map._came, map._closed] = [new Float32Array(S), new Int32Array(S), new Uint8Array(S)];
  const gs = map._g.fill(Infinity), came = map._came.fill(-1), closed = map._closed.fill(0);
  const gk = gi % N, gc = gk % W, gr = (gk / W) | 0;
  const h = (k) => {
    const dc = Math.abs((k % W) - gc), dr = Math.abs(((k / W) | 0) - gr);
    return Math.max(dc, dr) + 0.41421356 * Math.min(dc, dr);
  };
  const cost = (v) => (v >= SOFT ? 1 : 1 + 4 * ((SOFT - v) / Math.max(0.05, SOFT - need)) ** 2);
  const heap = new Heap();
  const relax = (from, to, ng) => {
    if (ng < gs[to]) {
      gs[to] = ng;
      came[to] = from;
      heap.push(to, ng + h(to % N));
    }
  };
  gs[si] = 0;
  heap.push(si, h(si % N));
  let expanded = 0;
  while (heap.size) {
    const st = heap.pop();
    if (closed[st]) continue;
    closed[st] = 1;
    expanded++;
    if (st === gi) break;
    const b = (st / N) | 0, i = st - b * N, c = i % W, r = (i / W) | 0, cl = map.clear[b], pen = 1 + (0.25 * Math.abs(B[b] - B[b0])) / 0.4;
    for (let dr = -1; dr <= 1; dr++)
      for (let dc = -1; dc <= 1; dc++) {
        const cc = c + dc, rr = r + dr, j = rr * W + cc;
        if ((!dc && !dr) || cc < 0 || rr < 0 || cc >= W || rr >= H || closed[b * N + j] || cl[j] < need) continue;
        if (dc && dr && (cl[r * W + cc] < need || cl[rr * W + c] < need)) continue; // no corner cutting
        relax(st, b * N + j, gs[st] + (dc && dr ? Math.SQRT2 : 1) * cost(cl[j]) * pen);
      }
    for (const nb of [b - 1, b + 1])
      if (allowed[nb] && map.clear[nb][i] >= need) relax(st, nb * N + i, gs[st] + (2 * Math.abs(B[nb] - B[b])) / cell);
  }
  if (si !== gi && came[gi] < 0) {
    const ra = map.roomAt(...s.p), rb = map.roomAt(...g.p), route = ra && rb && roomRoute(roomGraph(map, { alt, sigma }), ra.id, rb.id);
    const why = ra && rb && !route
      ? `no doorway from ${ra.name} to ${rb.name} has room for the drone with σ ${sg} m`
      : `no path with σ ${sg} m`;
    return { ok: false, reason: why, expanded };
  }
  const states = [];
  for (let st = gi; st !== -1; st = came[st]) states.push(st);
  states.reverse();
  // Runs at one band, each pulled tight and smoothed in its own band; a band change is a climb in place.
  const runs = [];
  for (const st of states) {
    const b = (st / N) | 0, k = st - b * N, p = map.center(k % W, (k / W) | 0);
    if (runs.at(-1)?.b !== b) runs.push({ b, cells: [] });
    runs.at(-1).cells.push(p);
  }
  runs[0].cells[0] = s.p;
  runs.at(-1).cells[runs.at(-1).cells.length - 1] = g.p;
  for (let k = 1; k < runs.length; k++) runs[k].cells[0] = runs[k - 1].cells.at(-1);
  const path = [], knotsXY = [];
  let length = 0, climbed = 0, minClearance = Infinity;
  for (const run of runs) {
    const cl = map.clear[run.b], line = smooth(map, cl, run.cells, need);
    if (path.length) climbed += Math.abs(B[run.b] - (path.at(-1)[2] - map.floorAt(path.at(-1)[0], path.at(-1)[1])));
    knotsXY.push(...line);
    for (let k = 0; k < line.length; k++) {
      const [p, q] = [line[k], line[k + 1]];
      path.push([p[0], p[1], map.floorAt(p[0], p[1]) + B[run.b]]);
      minClearance = Math.min(minClearance, cl[map.idx(p[0], p[1])]);
      if (!q) break;
      const L = Math.hypot(q[0] - p[0], q[1] - p[1]), n = Math.max(1, Math.round(L / step));
      length += L;
      for (let m = 1; m < n; m++) {
        const x = p[0] + ((q[0] - p[0]) * m) / n, y = p[1] + ((q[1] - p[1]) * m) / n;
        path.push([x, y, map.floorAt(x, y) + B[run.b]]);
        minClearance = Math.min(minClearance, cl[map.idx(x, y)]);
      }
    }
  }
  const bands = runs.map((r) => B[r.b]), doors = doorsCrossed(map, knotsXY);
  const { length: waypoints } = knotsXY;
  return { ok: true, path, length, climb: climbed, doors, minClearance, alt: B[b0], bands, sigma, from: s.p, to: g.p, waypoints, expanded };
}

// String pulling (jump to the farthest cell in sight with a comfortable margin, else a near one), then Chaikin
// corner cutting kept only while every segment stays clear.
function smooth(map, cl, cells, need) {
  if (cells.length < 2) return cells.slice();
  const comfy = Math.min(need + 0.15, SOFT), knots = [cells[0]];
  for (let a = 0; a < cells.length - 1; ) {
    let z = cells.length - 1;
    while (z > a + 1 && !(los(map, cl, cells[a], cells[z], comfy) || (z - a < 4 && los(map, cl, cells[a], cells[z], need)))) z--;
    knots.push(cells[z]);
    a = z;
  }
  let line = knots;
  for (let round = 0; round < 2 && line.length > 2; round++) {
    const next = [line[0]];
    for (let k = 0; k < line.length - 1; k++) {
      const [p, q] = [line[k], line[k + 1]];
      if (k > 0) next.push([0.75 * p[0] + 0.25 * q[0], 0.75 * p[1] + 0.25 * q[1]]);
      if (k < line.length - 2) next.push([0.25 * p[0] + 0.75 * q[0], 0.25 * p[1] + 0.75 * q[1]]);
    }
    next.push(line.at(-1));
    if (next.every((p, k) => k === 0 || los(map, cl, next[k - 1], p, need))) line = next;
    else break;
  }
  return line;
}

function crosses(p, q, a, b) {
  const d = (u, v, w) => (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0]);
  return d(p, q, a) * d(p, q, b) < 0 && d(a, b, p) * d(a, b, q) < 0;
}

function doorsCrossed(map, line) {
  const out = [];
  for (let k = 0; k < line.length - 1; k++)
    for (const d of map.doors) {
      if (!crosses(line[k], line[k + 1], d.a, d.b) || out.at(-1)?.id === d.id) continue;
      const { wa, wb, n, mid } = doorFrame(map, d);
      const forward = (line[k + 1][0] - line[k][0]) * n[0] + (line[k + 1][1] - line[k][1]) * n[1] > 0;
      out.push({ id: d.id, rooms: forward ? [...d.rooms] : [d.rooms[1], d.rooms[0]], mid, pre: forward ? wa : wb, post: forward ? wb : wa });
    }
  return out;
}

// Scan points (a 360° look-around each) that together see most of a room. target "person": standing people,
// hidden only by walls and tall furniture; "pet": the floor, hidden by any furniture, and the camera, tilted up,
// only sees the floor beyond alt / tan(vfov/2 − uptilt). Lens: 4:3 equidistant, so vfov = 0.75·hfov.
// Cached per map build (treat the result as read-only).
const views = new WeakMap(); // map.clear, a new array on every finalize() -> Map(arguments -> result)
export function viewpoints(map, roomId, target = "person", opts = {}) {
  map.sync?.();
  let memo = views.get(map.clear);
  if (!memo) views.set(map.clear, (memo = new Map()));
  const key = JSON.stringify([roomId, target, opts]);
  if (!memo.has(key)) memo.set(key, scanPoints(map, roomId, target, opts));
  return memo.get(key);
}

function scanPoints(map, roomId, target, opts) {
  const pet = target === "pet";
  const { hfov = 127, uptilt = 20, alt = 1.0, sigma = map.o.sigma, range = pet ? 4 : 6, coverage = 0.95, max = 6 } = opts;
  const ri = map.rooms.findIndex((r) => r.id === roomId);
  if (ri < 0) return { points: [], coverage: 0, targets: 0 };
  const b = map.bandOf(alt), need = map.lethal(sigma), cl = map.clear[b], { W, H, N } = map, { cell } = map.o;
  const tall = map.occ[map.bandOf(1.4)], low = map.occ[map.levels.length - 1], mid = map.occ[map.bandOf(0.6)];
  const hides = (k) => (pet ? low[k] || mid[k] : tall[k]);
  const half = ((hfov * 0.75) / 2 - uptilt) * (Math.PI / 180), minFloor = pet && half > 0 ? alt / Math.tan(half) : 0;
  // Sample grids thin out with the room's area, so a big room costs about what a 30 m² one does.
  let cells = 0;
  for (let k = 0; k < N; k++) cells += map.room[k] === ri ? 1 : 0;
  const area = cells * cell * cell, grid = (n, min) => Math.max(min, Math.round(Math.sqrt(area / n) / cell));
  const targets = [], cands = [], ts = grid(600, 4), cs = grid(300, 6);
  for (let r = 0; r < H; r += ts)
    for (let c = 0; c < W; c += ts) {
      const k = r * W + c;
      if (map.room[k] === ri && !map.wall[k] && !hides(k)) targets.push(map.center(c, r));
    }
  for (let r = 0; r < H; r += cs)
    for (let c = 0; c < W; c += cs) if (map.room[r * W + c] === ri && cl[r * W + c] >= need) cands.push(map.center(c, r));
  for (const e of roomGraph(map, { alt, sigma }).edges)
    if (e.a === roomId || e.b === roomId) for (const w of [e.wa, e.wb]) if (cl[map.idx(...w)] >= need) cands.push(w);
  if (!targets.length || !cands.length) return { points: [], coverage: 0, targets: targets.length };
  const sees = (p, q) => {
    const L = Math.hypot(q[0] - p[0], q[1] - p[1]), n = Math.ceil(L / cell);
    for (let i = 1; i < n; i++) {
      const k = map.idx(p[0] + ((q[0] - p[0]) * i) / n, p[1] + ((q[1] - p[1]) * i) / n);
      if (k < 0 || map.room[k] < 0 || map.wall[k] || hides(k)) return false;
    }
    return true;
  };
  const inRange = cands.map((p) => {
    const out = [];
    targets.forEach((t, j) => {
      const L = Math.hypot(t[0] - p[0], t[1] - p[1]);
      if (L <= range && L >= minFloor * 0.8) out.push(j);
    });
    return out;
  });
  // Greedy set cover, lazily (CELF): a candidate's gain only shrinks as targets get covered, so its last score
  // bounds it and only candidates that still look best are scored again.
  const covered = new Uint8Array(targets.length), scored = new Int32Array(cands.length).fill(-1), vis = [], heap = new Heap();
  cands.forEach((_, i) => heap.push(i, -inRange[i].length));
  const points = [];
  let seen = 0;
  for (let round = 0; seen / targets.length < coverage && points.length < max && heap.size; round++) {
    let best = -1;
    while (heap.size) {
      const i = heap.pop();
      if (scored[i] === round) {
        best = i;
        break;
      }
      vis[i] = inRange[i].filter((j) => !covered[j] && sees(cands[i], targets[j]));
      scored[i] = round;
      heap.push(i, -vis[i].length);
    }
    if (best < 0 || !vis[best].length) break;
    let sx = 0, sy = 0;
    for (const j of vis[best]) [covered[j], sx, sy] = [1, sx + targets[j][0], sy + targets[j][1]];
    const gain = vis[best].length, [x, y] = cands[best], yaw0 = Math.atan2(sy / gain - y, sx / gain - x), n = Math.ceil(360 / (hfov * 0.8));
    seen += gain;
    const headings = Array.from({ length: n }, (_, i) => yaw0 + (2 * Math.PI * i) / n);
    points.push({ x, y, z: map.floorAt(x, y) + alt, room: map.roomAt(x, y)?.id ?? roomId, headings, coverage: seen / targets.length });
  }
  return { points, coverage: seen / targets.length, targets: targets.length };
}

// Home -> one scan point per room -> home, in the order with the shortest flight (exact for up to 7 rooms).
export function patrolRoute(map, rooms = map.rooms.map((r) => r.id), home, { alt = 1.0, sigma = map.o.sigma, target = "person" } = {}) {
  if (!home) return { ok: false, stops: [], legs: [], length: 0, skipped: [], reason: "no home pad: set one first" };
  const h = pt(home), skipped = [];
  const stops = [];
  for (const id of rooms) {
    const v = viewpoints(map, id, target, { alt, sigma }).points[0];
    const c = v ? [v.x, v.y] : roomCenter(map, id, { alt, sigma });
    if (c) stops.push({ room: id, x: c[0], y: c[1], z: map.floorAt(c[0], c[1]) + alt, headings: v?.headings ?? [] });
    else skipped.push({ room: id, reason: "no free space" });
  }
  const pts = [h, ...stops.map((s) => [s.x, s.y])], legs = new Map();
  const leg = (i, j) => {
    const key = i < j ? `${i},${j}` : `${j},${i}`;
    if (!legs.has(key)) legs.set(key, plan(map, pts[Math.min(i, j)], pts[Math.max(i, j)], { alt, sigma }));
    return legs.get(key);
  };
  const dist = (i, j) => (leg(i, j).ok ? leg(i, j).length : Infinity);
  const reach = stops.map((_, i) => i + 1).filter((i) => leg(0, i).ok);
  stops.forEach((s, i) => !reach.includes(i + 1) && skipped.push({ room: s.room, reason: leg(0, i + 1).reason }));
  const len = (order) => [0, ...order, 0].reduce((sum, i, k, a) => (k ? sum + dist(a[k - 1], i) : 0), 0);
  let best = reach;
  if (reach.length <= 7) {
    const perm = (a) => (a.length <= 1 ? [a] : a.flatMap((x, i) => perm(a.toSpliced(i, 1)).map((p) => [x, ...p])));
    for (const p of perm(reach)) if (len(p) < len(best)) best = p;
  } else {
    best = [];
    for (let cur = 0, left = [...reach]; left.length; ) {
      left.sort((a, b) => dist(cur, a) - dist(cur, b));
      best.push((cur = left.shift()));
    }
  }
  const order = [0, ...best, 0];
  const path = order.slice(1).map((i, k) => plan(map, pts[order[k]], pts[i], { alt, sigma })); // legs in flight direction
  const length = path.reduce((s, p) => s + (p.length ?? 0), 0);
  return { ok: path.every((p) => p.ok), stops: best.map((i) => stops[i - 1]), legs: path, length, skipped };
}
