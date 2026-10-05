// Top-down map of the active house (docs/HOME-DRONE.md) in the house frame H, x right and y up the page: the capture's
// orthophoto underneath (georeferenced: x0, y0 its lower-left corner, res metres per pixel, row 0 = max y), rooms with
// their names, the furniture the flight map found at 1 m, doors (green: passable, red: closed), windows, keep-outs
// (fans, stairs, yours), landmarks, AprilTags, the home pad, the running mission's path, the simulator's people and
// pets, mission findings and the drone: the localizer's estimate with its 2σ ring, coloured by status, and in the
// simulator the true pose (a white outline when it differs from the estimate).
// Tools: "pan" (drag; wheel or +/- zooms; arrows pan; 0 fits; a click on a finding selects it), "home" (click the pad,
// drag for its heading), "landmark" (click), "keepout" (click the corners; Enter, a double-click or the first corner
// closes it; Backspace undoes one; Esc cancels), "goto" (click where to fly). From the keyboard, Enter or Space uses
// the cross in the middle. Edits come out as events ("home" {x, y, yaw}, "landmark" {x, y, room}, "keepout"
// {polygon}, "goto" {x, y, room}, "select" finding, "change" id (a suspected change's pin), "gap" (a coverage gap),
// "tool" name); the owner applies and saves them.
// Layers (setLayer, kept in this browser): the 3D map at the drone's height (unknown space pink: never drawn like free;
// occupied dark), what the camera has seen (this flight, or ever between flights), coverage gaps to rescan (cyan "?"
// markers, off until the coverage report's "show on map"), the flight's trail from the memory (coloured by how sure the
// position was, older fading, lost marked), the route ahead (the path and the remaining stops), and temporary obstacles
// (people, what live depth found, the way ahead, suspected changes as pins, doorways that look closed). No-fly zones are
// red hatching. Findings are this flight's (newFlight() at take-off), the latest per label and room, hidden while a past
// flight's trail shows. With a conflict the camera's other position shows as a hollow marker.
import { Emitter } from "../util.js";

export const LAYERS = { unknown: true, seen: false, gaps: false, trail: true, route: true, obstacles: true };
const LAYER_KEY = "whoopPilot.mapLayers";
const [UNKNOWN, OCCUPIED] = [0, 2]; // house/voxels.js states
const SIGMA_COLOR = (s) => (s < 0.1 ? [74, 222, 128] : s < 0.25 ? [250, 204, 21] : [248, 113, 113]);

function loadLayers() {
  try {
    return { ...LAYERS, ...JSON.parse(localStorage.getItem(LAYER_KEY) || "{}") };
  } catch {
    return { ...LAYERS };
  }
}

// The drone's height above the floor the 3D map layer shows: its own in the air, 1 m (a usual flying height) on the ground.
export const layerHeight = (pose, floor) => {
  const h = pose && floor != null ? pose.z - floor : null;
  return h == null || !(h > 0.2) ? 1.0 : Math.round(Math.max(0.3, Math.min(1.6, h)) * 10) / 10;
};

// Trail samples ({ x, y, sigma } | { lost }) as segments to draw: colour by σ, alpha from 0.25 (oldest) to 0.95 (newest),
// and where the position was lost, the last point before it.
export function trailSegments(samples) {
  const out = { segs: [], lost: [] }, n = samples.length;
  let prev = null;
  samples.forEach((p, i) => {
    if (p.lost || !Number.isFinite(p.x)) {
      if (prev) out.lost.push([prev.x, prev.y]);
      prev = null;
      return;
    }
    if (prev) out.segs.push({ a: [prev.x, prev.y], b: [p.x, p.y], rgb: SIGMA_COLOR(p.sigma ?? 0), alpha: 0.25 + (0.7 * i) / Math.max(1, n - 1) });
    prev = p;
  });
  return out;
}

const TINT = ["#5b8def", "#ef8a5b", "#5bd1a4", "#c77dde", "#e6c35c", "#5bc2e6", "#e66d8f"];
const ACTOR = { person: ["#f472b6", "P"], cat: ["#f59e0b", "C"], dog: ["#a3e635", "D"] };
const STATUS = { ok: "#ffd84d", degraded: "#f97316", lost: "#ef4444" }; // as the 3D view's (unsure is not the cat's amber)
const KEEP_LABEL = { fan: "fan", stairs: "stairs", user: "no-fly" };
const HINT = {
  home: "Click the home pad; drag to point it the way the drone faces. Esc: done.",
  landmark: "Click a place to name it. Esc: done.",
  keepout: "Click the corners of the no-fly zone. Enter or the first corner closes it; Backspace undoes; Esc cancels.",
  goto: "Click where the drone should fly. Esc: cancel.",
};
const FONT = "ui-sans-serif, system-ui, -apple-system, sans-serif";

// The orthophoto's extent in H.
export const orthoRect = (o) => ({ x0: o.x0, y0: o.y0, x1: o.x0 + o.width * o.res, y1: o.y0 + o.height * o.res });

// Plan bounds of the rooms (and the home pad), with a margin.
export function houseBounds(house, margin = 0.4) {
  const pts = house.rooms.flatMap((r) => r.outline);
  if (house.home) pts.push([house.home.x, house.home.y]);
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  return { x0: Math.min(...xs) - margin, y0: Math.min(...ys) - margin, x1: Math.max(...xs) + margin, y1: Math.max(...ys) + margin };
}

// Canvas pixels <-> H for a view of `bounds` fitted into W x H pixels, zoomed by `zoom` about `center` ([x, y] in H).
export function viewTransform({ W, H, bounds, zoom = 1, center = null, pad = 10 }) {
  const fit = Math.min((W - 2 * pad) / (bounds.x1 - bounds.x0), (H - 2 * pad) / (bounds.y1 - bounds.y0));
  const s = Math.max(1e-6, fit * zoom);
  const [cx, cy] = center ?? [(bounds.x0 + bounds.x1) / 2, (bounds.y0 + bounds.y1) / 2];
  return { s, cx, cy, X: (x) => W / 2 + (x - cx) * s, Y: (y) => H / 2 - (y - cy) * s, toH: (px, py) => [cx + (px - W / 2) / s, cy - (py - H / 2) / s] };
}

// Where a room's name goes: the middle of the cells the map gives that room (rooms may overlap; the smallest owns a
// cell), moved onto one of them if the middle is someone else's.
export function roomLabels(map) {
  const n = map.rooms.length, sx = new Float64Array(n), sy = new Float64Array(n), c = new Float64Array(n);
  for (let k = 0; k < map.N; k++) {
    const r = map.room[k];
    if (r < 0) continue;
    const [x, y] = map.center(k % map.W, (k / map.W) | 0);
    sx[r] += x;
    sy[r] += y;
    c[r]++;
  }
  return map.rooms.map((rm, i) => {
    if (!c[i]) return null;
    let [x, y] = [sx[i] / c[i], sy[i] / c[i]];
    if (map.roomAt(x, y)?.index !== i) {
      let best = Infinity;
      for (let k = 0; k < map.N; k++) {
        if (map.room[k] !== i) continue;
        const p = map.center(k % map.W, (k / map.W) | 0), d = Math.hypot(p[0] - x, p[1] - y);
        if (d < best) [best, x, y] = [d, ...p];
      }
    }
    return { id: rm.id, x, y };
  });
}

export class MapView extends Emitter {
  constructor(canvas, opts = {}) {
    super();
    Object.assign(this, { canvas, house: null, map: null, localizer: null, missions: null, actors: null, truth: null, orthophoto: null, fov: 127 });
    Object.assign(this, { tool: "pan", zoom: 1, center: null, path: null, findings: [], draft: [], hover: null, drag: null, focused: false });
    Object.assign(this, { vox: null, memory: null, coverage: null, plan: null, trailFlight: null, findingsFlight: null, layers: loadLayers(), voxKey: "", voxAt: 0, pins: [] });
    this.base = document.createElement("canvas");
    this.baseKey = "";
    this.version = 0;
    this.unsub = [];
    canvas.tabIndex = 0;
    canvas.setAttribute("role", "application");
    canvas.setAttribute("aria-label", "House map. Arrow keys pan, plus and minus zoom, 0 fits.");
    this.bind();
    this.set(opts);
  }

  set({ house, map, orthophoto, missions, ...rest } = {}) {
    Object.assign(this, rest);
    if (house !== undefined || map !== undefined) this.setHouse(house ?? this.house, map ?? this.map, orthophoto);
    else if (orthophoto !== undefined) this.setOrthophoto(orthophoto);
    if (missions !== undefined) this.setMissions(missions);
    return this;
  }

  setHouse(house, map, orthophoto = null) {
    const same = house && house.id === this.house?.id;
    Object.assign(this, { house, map, orthophoto });
    this.labels = map ? roomLabels(map) : [];
    this.occ = null;
    if (!same) Object.assign(this, { zoom: 1, center: null, path: null, plan: null, findings: [], findingsFlight: null, draft: [], trailFlight: null });
    this.invalidate();
  }

  setLayer(name, on) {
    this.layers[name] = !!on;
    this.voxKey = "";
    try {
      localStorage.setItem(LAYER_KEY, JSON.stringify(this.layers));
    } catch {}
  }

  // Centre on a spot (a coverage gap, a sighting) and mark it for a few seconds.
  focusOn(x, y, zoom = 2.5) {
    this.center = [x, y];
    this.zoom = Math.max(this.zoom, zoom);
    this.flash = { x, y, until: performance.now() + 4000 };
  }

  // A past flight's trail instead of this flight's (null: this one, or the last); an older one, no longer in the memory's
  // RAM, is read back from its store.
  async showTrail(flightId) {
    this.trailFlight = flightId ?? null;
    this.pastTrail = null;
    if (flightId) this.layers.trail = true;
    const m = this.memory;
    if (!flightId || m?.trails?.has?.(flightId) || !m?.trailOf) return;
    const tr = await m.trailOf(flightId).catch(() => null);
    if (this.trailFlight === flightId && this.memory === m) this.pastTrail = { id: flightId, samples: tr ?? [] };
  }

  setOrthophoto(img) {
    this.orthophoto = img;
    this.invalidate();
  }

  setMissions(missions) {
    this.unsub.forEach((f) => f());
    this.missions = missions;
    this.unsub = missions?.on
      ? [
        missions.on("path", (e) => (this.path = e?.path ?? e)),
        missions.on("plan", (p) => (this.plan = p?.stops ? p : null)),
        missions.on("finding", (f) => this.addFinding(f)),
        missions.on("done", () => (this.path = this.plan = null)), // ended, failed or stopped: no plan is being flown
      ]
      : [];
  }

  setPath(path) {
    this.path = path;
  }

  // The latest finding per label and room, for the flight under way (or just flown).
  addFinding(f) {
    if (!f || !Number.isFinite(f.x) || !Number.isFinite(f.y)) return;
    this.findingsFlight = this.memory?.flight?.id ?? this.findingsFlight ?? null;
    this.findings = [...this.findings.filter((q) => q.label !== f.label || q.room !== f.room), f].slice(-30);
  }

  // A take-off: this flight's findings and trail from now on.
  newFlight() {
    Object.assign(this, { findings: [], findingsFlight: null, trailFlight: null });
  }

  // The house changed (an edit): redraw the fixed layer, keep the view.
  invalidate() {
    this.version++;
    this.occ = null;
    if (this.map) this.labels = roomLabels(this.map);
  }

  setTool(tool) {
    if (tool === this.tool) tool = "pan";
    this.tool = tool;
    this.draft = [];
    this.drag = null;
    this.canvas.dataset.tool = tool;
    this.emit("tool", tool);
  }

  fit() {
    this.zoom = 1;
    this.center = null;
  }

  zoomBy(k, at = null) {
    const t = this.view();
    if (!t) return;
    const z = Math.max(0.5, Math.min(12, this.zoom * k)), [cx, cy] = [t.cx, t.cy];
    if (at) {
      const [hx, hy] = t.toH(...at), f = this.zoom / z;
      this.center = [hx + (cx - hx) * f, hy + (cy - hy) * f];
    } else this.center = [cx, cy];
    this.zoom = z;
  }

  panBy(dxPx, dyPx) {
    const t = this.view();
    if (t) this.center = [t.cx - dxPx / t.s, t.cy + dyPx / t.s];
  }

  dispose() {
    this.setMissions(null);
    this.ac.abort();
  }

  // ------------------------------------------------------------------------------------------------- drawing

  size() {
    const c = this.canvas, dpr = window.devicePixelRatio || 1;
    const w = Math.round(c.clientWidth * dpr), h = Math.round(c.clientHeight * dpr);
    if (c.width !== w || c.height !== h) Object.assign(c, { width: w, height: h });
    return { W: w, H: h, k: dpr };
  }

  view() {
    if (!this.house) return null;
    const { width: W, height: H } = this.canvas;
    return viewTransform({ W, H, bounds: houseBounds(this.house), zoom: this.zoom, center: this.center, pad: 10 * (window.devicePixelRatio || 1) });
  }

  draw() {
    const { W, H, k } = this.size(), g = this.canvas.getContext("2d");
    if (!W || !H) return;
    g.setTransform(1, 0, 0, 1, 0, 0);
    if (!this.house || !this.map) {
      g.clearRect(0, 0, W, H);
      return;
    }
    const t = this.view();
    const key = `${W}x${H}:${t.s.toFixed(4)}:${t.cx.toFixed(4)}:${t.cy.toFixed(4)}:${this.version}:${!!this.orthophoto}`;
    if (key !== this.baseKey) this.drawBase(t, W, H, k, key);
    g.drawImage(this.base, 0, 0);
    this.drawLive(g, t, W, H, k);
  }

  drawBase(t, W, H, k, key) {
    const b = this.base;
    Object.assign(b, { width: W, height: H });
    const g = b.getContext("2d"), { X, Y, s } = t, house = this.house, map = this.map;
    this.baseKey = key;
    g.fillStyle = "#0a0e13";
    g.fillRect(0, 0, W, H);
    const ortho = this.orthophoto && house.orthophoto;
    if (ortho) {
      const r = orthoRect(house.orthophoto);
      g.globalAlpha = 0.9;
      g.imageSmoothingQuality = "high";
      g.drawImage(this.orthophoto, X(r.x0), Y(r.y1), (r.x1 - r.x0) * s, (r.y1 - r.y0) * s);
      g.globalAlpha = 1;
      g.fillStyle = "rgba(6,9,13,0.38)";
      g.fillRect(0, 0, W, H);
    }
    const poly = (pts) => {
      g.beginPath();
      pts.forEach(([x, y], i) => (i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y))));
      g.closePath();
    };
    const rooms = [...map.rooms].sort((a, b) => b.area - a.area);
    for (const r of rooms) {
      poly(r.outline);
      g.fillStyle = TINT[r.index % TINT.length] + (ortho ? "26" : "40");
      g.fill();
    }
    // Furniture and other obstacles the map found at the 1 m band, one image of map cells.
    this.occ ??= this.obstacleImage();
    if (this.occ) {
      g.imageSmoothingEnabled = false;
      g.drawImage(this.occ, X(map.x0), Y(map.y0 + map.H * map.o.cell), map.W * map.o.cell * s, map.H * map.o.cell * s);
      g.imageSmoothingEnabled = true;
    }
    for (const lm of house.landmarks ?? []) {
      if (!lm.footprint) continue;
      poly(lm.footprint);
      g.strokeStyle = "rgba(203,213,225,0.35)";
      g.lineWidth = k;
      g.stroke();
    }
    const hatch = hatching(g, k);
    for (const ko of map.keepouts ?? []) {
      if (ko.polygon) poly(ko.polygon);
      else {
        g.beginPath();
        g.arc(X(ko.x), Y(ko.y), ko.r * s, 0, 7);
      }
      g.fillStyle = "rgba(239,68,68,0.12)";
      g.fill();
      g.fillStyle = hatch;
      g.fill();
      g.strokeStyle = "rgba(248,113,113,0.95)";
      g.lineWidth = 1.5 * k;
      g.stroke();
      const [lx, ly] = ko.polygon ? ko.polygon.reduce(([a, b], p) => [a + p[0] / ko.polygon.length, b + p[1] / ko.polygon.length], [0, 0]) : [ko.x, ko.y];
      label(g, KEEP_LABEL[ko.kind] ?? ko.kind, X(lx), Y(ly), k, "#fecaca", 10);
    }
    g.lineJoin = "round";
    g.strokeStyle = "#d6dde6";
    g.lineWidth = Math.max(2 * k, 0.1 * s);
    for (const r of rooms) {
      poly(r.outline);
      g.stroke();
    }
    g.lineCap = "round";
    for (const w of house.windows ?? []) seg(g, X, Y, w.a, w.b, "#7dd3fc", Math.max(2 * k, 0.05 * s));
    for (const d of house.doors ?? []) {
      seg(g, X, Y, d.a, d.b, "#0a0e13", Math.max(3 * k, 0.14 * s));
      seg(g, X, Y, d.a, d.b, d.passable ? "rgba(74,222,128,0.9)" : "rgba(248,113,113,0.95)", Math.max(2 * k, 0.04 * s));
    }
    for (const tag of house.tags ?? []) {
      const [x, y] = tag.center, r = Math.max(3 * k, (tag.size ?? 0.15) * s * 0.5);
      g.fillStyle = "#f8fafc";
      g.fillRect(X(x) - r, Y(y) - r, 2 * r, 2 * r);
      g.fillStyle = "#111";
      g.fillRect(X(x) - r / 2, Y(y) - r / 2, r, r);
    }
    for (const lm of house.landmarks ?? []) {
      if (lm.footprint || !Number.isFinite(lm.x)) continue;
      const mine = lm.source === "user";
      g.fillStyle = mine ? "#ffd84d" : "rgba(203,213,225,0.7)";
      g.beginPath();
      g.arc(X(lm.x), Y(lm.y), (mine ? 4 : 2.5) * k, 0, 7);
      g.fill();
      if (mine || this.zoom >= 2.5) label(g, lm.name, X(lm.x), Y(lm.y) - 11 * k, k, mine ? "#ffd84d" : "#cbd5e1", mine ? 11 : 9.5);
    }
    for (const [i, l] of this.labels.entries()) {
      if (!l) continue;
      const r = map.rooms[i];
      label(g, r.name, X(l.x), Y(l.y), k, "#f1f5f9", 12.5, true);
    }
    if (house.home) this.pad(g, X(house.home.x), Y(house.home.y), house.home.yaw ?? Math.PI / 2, k);
  }

  obstacleImage() {
    const m = this.map, b = m.bandOf(1.0), occ = m.occ?.[b];
    if (!occ) return null;
    const c = Object.assign(document.createElement("canvas"), { width: m.W, height: m.H }), g = c.getContext("2d");
    const img = g.createImageData(m.W, m.H);
    for (let k = 0; k < m.N; k++) {
      if (!occ[k] || m.room[k] < 0) continue;
      const r = (k / m.W) | 0, o = 4 * ((m.H - 1 - r) * m.W + (k % m.W));
      img.data.set([148, 163, 184, 120], o);
    }
    g.putImageData(img, 0, 0);
    return c;
  }

  pad(g, x, y, yaw, k, alpha = 1) {
    g.globalAlpha = alpha;
    g.strokeStyle = "#38bdf8";
    g.fillStyle = "rgba(56,189,248,0.18)";
    g.lineWidth = 2 * k;
    g.beginPath();
    g.arc(x, y, 9 * k, 0, 7);
    g.fill();
    g.stroke();
    g.beginPath();
    g.moveTo(x, y);
    g.lineTo(x + Math.cos(-yaw) * 16 * k, y + Math.sin(-yaw) * 16 * k);
    g.stroke();
    label(g, "H", x, y, k, "#e0f2fe", 10, true, false);
    g.globalAlpha = 1;
  }

  // The 3D map's layer image at height h above each cell's floor, cached: made again when the map, the height or the layers
  // change, at most twice a second (and every 2 s while the seen layer is on).
  voxelImage(h) {
    const vox = this.vox, L = this.layers, m = this.map, now = performance.now();
    if (!vox?.slice || !(L.unknown || L.seen)) return null;
    const since = this.memory?.flight?.t0 ?? 0, key = `${vox.version}|${h}|${L.unknown}|${L.seen}|${since}|${this.version}|${L.seen ? Math.floor(now / 2000) : 0}`;
    if (key === this.voxKey || (this.voxImg && now - this.voxAt < 500)) return this.voxImg;
    const sl = vox.slice(h, { map: m }), c = (this.voxCanvas ??= document.createElement("canvas"));
    Object.assign(c, { width: sl.width, height: sl.height });
    const g = c.getContext("2d"), img = g.createImageData(sl.width, sl.height), d = img.data;
    for (let r = 0; r < sl.height; r++)
      for (let col = 0; col < sl.width; col++) {
        const x = sl.x0 + (col + 0.5) * sl.res, y = sl.y0 + (r + 0.5) * sl.res, cell = m.idx(x, y);
        if (cell < 0 || m.room[cell] < 0) continue;
        const st = sl.data[r * sl.width + col], o = 4 * ((sl.height - 1 - r) * sl.width + col);
        if (st === OCCUPIED) d.set([15, 23, 42, 165], o);
        else if (st === UNKNOWN) L.unknown && d.set([236, 72, 153, 92], o);
        else if (L.seen) {
          const at = vox.seenAt(x, y, (m.floorAt(x, y) ?? 0) + h);
          if (at != null && at >= since) d.set([56, 189, 248, 70], o);
        }
      }
    g.putImageData(img, 0, 0);
    Object.assign(this, { voxKey: key, voxAt: now, voxImg: { canvas: c, x0: sl.x0, y0: sl.y0, w: sl.width * sl.res, h: sl.height * sl.res } });
    return this.voxImg;
  }

  drawLayers(g, { X, Y, s }, k, est) {
    const L = this.layers, m = this.map;
    const img = this.voxelImage(layerHeight(est, est && m.floorAt(est.x, est.y)));
    if (img) {
      g.imageSmoothingEnabled = false;
      g.drawImage(img.canvas, X(img.x0), Y(img.y0 + img.h), img.w * s, img.h * s);
      g.imageSmoothingEnabled = true;
    }
    if (L.gaps)
      for (const gap of this.coverage?.()?.rooms?.flatMap((r) => r.gaps ?? []) ?? []) {
        const x = X(gap.x), y = Y(gap.y);
        g.fillStyle = "#22d3ee";
        g.strokeStyle = "rgba(5,8,12,0.85)";
        g.lineWidth = 1.5 * k;
        g.beginPath();
        g.arc(x, y, 6 * k, 0, 7);
        g.fill();
        g.stroke();
        label(g, "?", x, y, k, "#042f36", 9.5, true, false);
        this.pins.push({ kind: "gap", x, y, r: 11 * k, item: gap });
      }
    if (L.obstacles)
      for (const t of m.temps?.values?.() ?? []) {
        const door = t.source === "door leaf" && this.house.doors?.find((d) => d.id === t.door);
        if (door) {
          seg(g, X, Y, door.a, door.b, "rgba(239,68,68,0.95)", Math.max(4 * k, 0.08 * s));
          label(g, "looks closed", X((door.a[0] + door.b[0]) / 2), Y((door.a[1] + door.b[1]) / 2) - 10 * k, k, "#fecaca", 10);
          continue;
        }
        const [fill, stroke] = t.kind === "person" ? ["rgba(239,68,68,0.16)", "rgba(248,113,113,0.9)"] : t.kind === "change" ? ["rgba(217,70,239,0.18)", "rgba(232,121,249,0.95)"]
          : ["rgba(245,158,11,0.22)", "rgba(251,191,36,0.9)"];
        g.beginPath();
        if (t.polygon) t.polygon.forEach(([x, y], i) => (i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y))));
        else g.arc(X(t.x), Y(t.y), Math.max(2 * k, (t.r ?? 0.2) * s), 0, 7);
        g.closePath();
        g.fillStyle = fill;
        g.fill();
        g.strokeStyle = stroke;
        g.lineWidth = 1.2 * k;
        g.stroke();
      }
    if (L.trail) this.drawTrail(g, X, Y, k);
    if (L.route && this.plan?.stops?.length) {
      this.plan.stops.slice(1).forEach((st, i) => {
        const at = Number.isFinite(st.x) ? [st.x, st.y] : (() => {
          const l = this.labels.find((q) => q?.id === st.room);
          return l && [l.x, l.y - 0.35];
        })();
        if (!at) return;
        g.fillStyle = "rgba(255,216,77,0.92)";
        g.beginPath();
        g.arc(X(at[0]), Y(at[1]), 8 * k, 0, 7);
        g.fill();
        label(g, String(i + 1), X(at[0]), Y(at[1]), k, "#1a1400", 10, true, false);
      });
    }
  }

  drawTrail(g, X, Y, k) {
    const m = this.memory, id = this.trailFlight ?? m?.flight?.id ?? m?.flights?.at(-1)?.id;
    if (!id || !m?.trail) return;
    const { segs, lost } = trailSegments(this.pastTrail?.id === id ? this.pastTrail.samples : m.trail({ flightId: id }));
    g.lineWidth = 2 * k;
    g.lineCap = "round";
    for (const sg of segs) {
      g.strokeStyle = `rgba(${sg.rgb.join(",")},${sg.alpha.toFixed(2)})`;
      g.beginPath();
      g.moveTo(X(sg.a[0]), Y(sg.a[1]));
      g.lineTo(X(sg.b[0]), Y(sg.b[1]));
      g.stroke();
    }
    for (const [x, y] of lost) label(g, "✕", X(x), Y(y), k, "#f87171", 13, true);
  }

  drawLive(g, t, W, H, k) {
    const { X, Y, s } = t;
    const est = this.localizer?.pose?.();
    this.pins = [];
    this.drawLayers(g, t, k, est);
    const truth = this.truth?.();
    if (truth?.trail?.length > 1) {
      g.strokeStyle = "rgba(255,216,77,0.3)";
      g.lineWidth = 1.5 * k;
      g.beginPath();
      truth.trail.forEach((p, i) => (i ? g.lineTo(X(p.x), Y(p.y)) : g.moveTo(X(p.x), Y(p.y))));
      g.stroke();
    }
    const path = this.layers.route ? this.path : null;
    if (path?.length > 1) {
      g.strokeStyle = "rgba(255,216,77,0.95)";
      g.lineWidth = 2.5 * k;
      g.setLineDash([7 * k, 5 * k]);
      g.beginPath();
      path.forEach((p, i) => (i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))));
      g.stroke();
      g.setLineDash([]);
      const end = path.at(-1);
      g.fillStyle = "#ffd84d";
      g.beginPath();
      g.arc(X(end[0]), Y(end[1]), 5 * k, 0, 7);
      g.fill();
    }
    for (const a of this.actors?.() ?? []) {
      const [color, ch] = ACTOR[a.kind] ?? ["#e5e7eb", "?"];
      g.fillStyle = color;
      g.beginPath();
      g.arc(X(a.x), Y(a.y), 6.5 * k, 0, 7);
      g.fill();
      label(g, ch, X(a.x), Y(a.y), k, "#111", 9, true, false);
    }
    for (const f of !this.trailFlight || this.trailFlight === this.findingsFlight ? this.findings : []) {
      const x = X(f.x), y = Y(f.y);
      g.fillStyle = f.label === "person" ? "#f472b6" : "#f59e0b";
      g.beginPath();
      g.moveTo(x, y);
      g.arc(x, y - 14 * k, 7 * k, Math.PI * 0.75, Math.PI * 2.25);
      g.closePath();
      g.fill();
      label(g, f.label ?? "?", x, y - 27 * k, k, "#fde68a", 10);
    }
    if (this.layers.obstacles)
      for (const c of this.memory?.changes ?? []) {
        if (c.status !== "suspected" || !Number.isFinite(c.x)) continue;
        const x = X(c.x), y = Y(c.y);
        g.fillStyle = "#d946ef";
        g.strokeStyle = "rgba(5,8,12,0.8)";
        g.lineWidth = 1.5 * k;
        g.beginPath();
        g.moveTo(x, y);
        g.arc(x, y - 14 * k, 8 * k, Math.PI * 0.75, Math.PI * 2.25);
        g.closePath();
        g.fill();
        g.stroke();
        label(g, "?", x, y - 14 * k, k, "#fff", 11, true, false);
        this.pins.push({ kind: "change", x, y: y - 14 * k, r: 13 * k, item: c });
      }
    const fov = (this.fov * Math.PI) / 180;
    const other = est?.conflict;
    if (other && Number.isFinite(other.x)) {
      g.strokeStyle = "#f87171";
      g.lineWidth = 2 * k;
      g.setLineDash([4 * k, 3 * k]);
      g.beginPath();
      g.moveTo(X(est.x), Y(est.y));
      g.lineTo(X(other.x), Y(other.y));
      g.stroke();
      g.setLineDash([]);
      g.beginPath();
      g.arc(X(other.x), Y(other.y), 9 * k, 0, 7);
      g.stroke();
      label(g, "camera says", X(other.x), Y(other.y) - 16 * k, k, "#fecaca", 10);
    }
    if (truth && Number.isFinite(truth.x) && (!est || est.source !== "truth")) drone(g, X(truth.x), Y(truth.y), truth.yaw, k, est ? null : "#ffd84d", est ? 0 : fov, s);
    if (est && Number.isFinite(est.x)) {
      const color = STATUS[est.status] ?? STATUS.ok;
      if (est.sigma > 0 && est.source !== "truth") {
        g.strokeStyle = color;
        g.globalAlpha = 0.6;
        g.lineWidth = 1.5 * k;
        g.setLineDash([3 * k, 3 * k]);
        g.beginPath();
        g.arc(X(est.x), Y(est.y), 2 * est.sigma * s, 0, 7);
        g.stroke();
        g.setLineDash([]);
        g.globalAlpha = 1;
      }
      drone(g, X(est.x), Y(est.y), est.yaw, k, color, fov, s);
    }
    if (this.flash && performance.now() < this.flash.until) {
      const r = (14 + 6 * Math.sin(performance.now() / 120)) * k;
      g.strokeStyle = "#f472b6";
      g.lineWidth = 3 * k;
      g.beginPath();
      g.arc(X(this.flash.x), Y(this.flash.y), r, 0, 7);
      g.stroke();
    }
    if (this.tool === "home" && this.drag?.at) this.pad(g, X(this.drag.at[0]), Y(this.drag.at[1]), this.drag.yaw, k, 0.85);
    if (this.tool === "keepout" && this.draft.length) {
      const pts = [...this.draft, ...(this.hover ? [this.hover.h] : [])];
      g.strokeStyle = "#f87171";
      g.fillStyle = "rgba(239,68,68,0.18)";
      g.lineWidth = 2 * k;
      g.beginPath();
      pts.forEach(([x, y], i) => (i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y))));
      g.fill();
      g.stroke();
      for (const [x, y] of this.draft) g.fillRect(X(x) - 3 * k, Y(y) - 3 * k, 6 * k, 6 * k);
    }
    if (this.focused && this.keyboard && this.tool !== "pan") {
      g.strokeStyle = "#ffd84d";
      g.lineWidth = 1.5 * k;
      g.beginPath();
      g.moveTo(W / 2 - 12 * k, H / 2);
      g.lineTo(W / 2 + 12 * k, H / 2);
      g.moveTo(W / 2, H / 2 - 12 * k);
      g.lineTo(W / 2, H / 2 + 12 * k);
      g.stroke();
    }
    if (HINT[this.tool]) banner(g, HINT[this.tool], W, k);
    if (this.hover) {
      const [x, y] = this.hover.h, room = this.map.roomAt(x, y), pin = this.pinAt(this.hover.p);
      const text = pin?.kind === "gap" ? `Not seen by the scan: ${pin.item.text} (click for what to do)` : pin?.kind === "change" ? "A possible change: click to check it"
        : `${room ? room.name + " · " : ""}${x.toFixed(2)}, ${y.toFixed(2)} m`;
      g.font = `${10.5 * k}px ${FONT}`;
      g.textAlign = "left";
      g.textBaseline = "alphabetic";
      g.fillStyle = "rgba(203,213,225,0.9)";
      g.fillText(text, 8 * k, H - 8 * k, W - 16 * k);
    }
  }

  // ------------------------------------------------------------------------------------------------- input

  bind() {
    const c = this.canvas, ac = (this.ac = new AbortController()), o = { signal: ac.signal };
    const px = (e) => {
      const r = c.getBoundingClientRect(), dpr = c.width / Math.max(1, r.width);
      return [(e.clientX - r.left) * dpr, (e.clientY - r.top) * dpr];
    };
    c.addEventListener("pointerdown", (e) => {
      const t = this.view();
      if (!t || e.button > 0) return;
      this.keyboard = false;
      const p = px(e), h = t.toH(...p);
      c.setPointerCapture(e.pointerId);
      this.drag = { p0: p, last: p, at: this.tool === "home" ? h : null, yaw: this.house.home?.yaw ?? Math.PI / 2, moved: false };
    }, o);
    c.addEventListener("pointermove", (e) => {
      const t = this.view();
      if (!t) return;
      const p = px(e), h = t.toH(...p), d = this.drag;
      this.hover = { p, h };
      if (!d) return;
      if (Math.hypot(p[0] - d.p0[0], p[1] - d.p0[1]) > 5 * (window.devicePixelRatio || 1)) d.moved = true;
      if (this.tool === "home" && d.moved) d.yaw = Math.atan2(h[1] - d.at[1], h[0] - d.at[0]);
      else if (this.tool === "pan" && d.moved) this.panBy(p[0] - d.last[0], p[1] - d.last[1]);
      d.last = p;
    }, o);
    c.addEventListener("pointerup", (e) => {
      const t = this.view(), d = this.drag;
      this.drag = null;
      if (!t || !d) return;
      const p = px(e);
      if (this.tool === "home") return this.emit("home", { x: d.at[0], y: d.at[1], yaw: d.yaw });
      if (!d.moved) this.click(t.toH(...p), p, t);
    }, o);
    c.addEventListener("pointercancel", () => (this.drag = null), o);
    c.addEventListener("pointerleave", () => (this.hover = null), o);
    c.addEventListener("dblclick", (e) => {
      if (this.tool !== "keepout") return;
      e.preventDefault();
      this.closeKeepout();
    }, o);
    c.addEventListener("wheel", (e) => {
      const page = document.scrollingElement;
      if (!this.house || (page.scrollHeight > page.clientHeight + 1 && !e.ctrlKey && !e.metaKey)) return; // the page scrolls (narrow layout)
      e.preventDefault();
      this.zoomBy(Math.exp(-e.deltaY * 0.0015), px(e));
    }, { ...o, passive: false });
    c.addEventListener("focus", () => (this.focused = true), o);
    c.addEventListener("blur", () => (this.focused = false), o);
    c.addEventListener("keydown", (e) => this.key(e), o);
  }

  key(e) {
    const t = this.view();
    if (!t) return;
    const step = 40 * (window.devicePixelRatio || 1), mid = [this.canvas.width / 2, this.canvas.height / 2];
    const pan = { ArrowLeft: [step, 0], ArrowRight: [-step, 0], ArrowUp: [0, step], ArrowDown: [0, -step] }[e.key];
    this.keyboard = true;
    if (pan) this.panBy(...pan);
    else if (e.key === "+" || e.key === "=") this.zoomBy(1.25);
    else if (e.key === "-" || e.key === "_") this.zoomBy(0.8);
    else if (e.key === "0") this.fit();
    else if (e.key === "Enter" && this.tool === "keepout" && this.draft.length >= 3) this.closeKeepout();
    else if ((e.key === "Enter" || e.key === " ") && this.tool !== "pan") {
      if (this.tool === "home") this.emit("home", { x: t.cx, y: t.cy, yaw: this.house.home?.yaw ?? Math.PI / 2 });
      else this.click([t.cx, t.cy], mid, t);
    } else if (e.key === "Backspace" && this.tool === "keepout") this.draft.pop();
    else if (e.key === "Escape" && this.tool !== "pan") {
      if (this.tool === "keepout" && this.draft.length) this.draft = [];
      else this.setTool("pan");
    } else return;
    e.preventDefault();
    e.stopPropagation();
  }

  click(h, p, t) {
    const room = this.map.roomAt(...h)?.id ?? null;
    if (this.tool === "landmark") return this.emit("landmark", { x: h[0], y: h[1], room });
    if (this.tool === "goto") return this.emit("goto", { x: h[0], y: h[1], room });
    if (this.tool === "keepout") {
      const first = this.draft[0];
      if (first && this.draft.length >= 3 && Math.hypot(t.X(first[0]) - p[0], t.Y(first[1]) - p[1]) < 10 * (window.devicePixelRatio || 1)) return this.closeKeepout();
      this.draft.push(h);
      return;
    }
    const k = window.devicePixelRatio || 1, pin = this.pinAt(p);
    if (pin) return this.emit(pin.kind, pin.kind === "change" ? pin.item.id : pin.item);
    const hit = (!this.trailFlight || this.trailFlight === this.findingsFlight ? this.findings : []).find((f) => Math.hypot(t.X(f.x) - p[0], t.Y(f.y) - 14 * k - p[1]) < 12 * k);
    if (hit) this.emit("select", hit);
  }

  // The change pin or coverage gap under a canvas pixel (changes first: they are what a pilot acts on).
  pinAt(p) {
    const near = (q) => Math.hypot(q.x - p[0], q.y - p[1]) < q.r;
    return this.pins.find((q) => q.kind === "change" && near(q)) ?? this.pins.find(near) ?? null;
  }

  closeKeepout() {
    if (this.draft.length < 3) return;
    const polygon = this.draft.map(([x, y]) => [+x.toFixed(3), +y.toFixed(3)]);
    this.draft = [];
    this.emit("keepout", { polygon });
  }
}

// No-fly zones' red diagonal hatching (a pattern for the canvas g, k: device pixels per CSS pixel).
function hatching(g, k) {
  const n = Math.max(4, Math.round(7 * k)), c = Object.assign(document.createElement("canvas"), { width: n, height: n }), h = c.getContext("2d");
  h.strokeStyle = "rgba(248,113,113,0.6)";
  h.lineWidth = 1.2 * k;
  h.beginPath();
  for (const o of [-n, 0, n]) (h.moveTo(o, n), h.lineTo(o + n, 0));
  h.stroke();
  return g.createPattern(c, "repeat");
}

function seg(g, X, Y, a, b, color, width) {
  g.strokeStyle = color;
  g.lineWidth = width;
  g.beginPath();
  g.moveTo(X(a[0]), Y(a[1]));
  g.lineTo(X(b[0]), Y(b[1]));
  g.stroke();
}

function label(g, text, x, y, k, color, size, bold = false, shadow = true) {
  g.font = `${bold ? "600 " : ""}${size * k}px ${FONT}`;
  g.textAlign = "center";
  g.textBaseline = "middle";
  if (shadow) {
    g.lineWidth = 3 * k;
    g.strokeStyle = "rgba(5,8,12,0.85)";
    g.strokeText(text, x, y);
  }
  g.fillStyle = color;
  g.fillText(text, x, y);
}

function banner(g, text, W, k) {
  g.font = `${11.5 * k}px ${FONT}`;
  g.textAlign = "center";
  g.textBaseline = "middle";
  const w = Math.min(W - 12 * k, g.measureText(text).width + 20 * k);
  g.fillStyle = "rgba(5,8,12,0.78)";
  g.beginPath();
  g.roundRect((W - w) / 2, 6 * k, w, 22 * k, 8 * k);
  g.fill();
  g.fillStyle = "#fde68a";
  g.fillText(text, W / 2, 17 * k, w - 12 * k);
}

// The drone: an arrow, and with fov > 0 its camera's view wedge.
function drone(g, x, y, yaw, k, color, fov, s) {
  if (fov > 0) {
    g.fillStyle = "rgba(255,216,77,0.13)";
    g.beginPath();
    g.moveTo(x, y);
    g.arc(x, y, Math.max(30 * k, 1.6 * s), -yaw - fov / 2, -yaw + fov / 2);
    g.closePath();
    g.fill();
  }
  g.save();
  g.translate(x, y);
  g.rotate(-yaw);
  if (color) {
    g.fillStyle = "rgba(5,8,12,0.55)";
    g.beginPath();
    g.arc(0, 0, 11 * k, 0, 7);
    g.fill();
  }
  g.beginPath();
  g.moveTo(11 * k, 0);
  g.lineTo(-7.5 * k, 6.5 * k);
  g.lineTo(-3.5 * k, 0);
  g.lineTo(-7.5 * k, -6.5 * k);
  g.closePath();
  if (color) {
    g.fillStyle = color;
    g.fill();
    g.strokeStyle = "rgba(0,0,0,0.6)";
    g.lineWidth = k;
    g.stroke();
  } else {
    g.strokeStyle = "rgba(255,255,255,0.9)";
    g.lineWidth = 1.5 * k;
    g.stroke();
  }
  g.restore();
}
