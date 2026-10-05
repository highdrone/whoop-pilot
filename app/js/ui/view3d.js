// The house in 3D: where the drone is, has been and will go (docs/HOME-DRONE.md, Wave C; the PANELS contract).
// Background: the splat twin rendered from an orbit or chase pinhole camera inside the rooms (twin.render, actors on), at
// most 2 a second and only when the twin has time (never while it's busy, never more than VIEW3D.duty of its time; never
// while the real drone flies: the twin is finding the drone's position then, so the last picture stays with live marks on
// it); otherwise, and while the view moves, a drawing: floors, low walls, doorways (green open, red closed), no-fly zones,
// the home pad. Drawn on top with the same camera: the flight's trail (fading with age, coloured by how sure the position
// was, ✕ where it was lost), the drone now (its 2σ ring and a line down to the floor, coloured by the localizer's status),
// the path it will fly with its stops ("Going home" on the way back), what it saw (pins), changes from the scan waiting
// for a decision (magenta pins), temporary obstacles (people, live-depth blockers) and, as a layer, the space nobody has
// seen (a pink haze, a dot per 20 cm block with any unseen space in it: never flown) with the solid things near the path.
// The camera stays inside a room for a render: pulled in, else turned round the drone or raised (next to a wall, on the pad).
// Mouse and touch: drag turns, wheel or pinch zooms, shift-drag or two fingers move, a tap on the drone follows it; keys
// (when focused from the keyboard, so a tap doesn't take the simulator's arrows): arrows, + and −, 0 (whole house), F
// (follow); a tap on a pin emits "select" { kind: "change" | "sighting" | "finding", id, x, y, z }.
//   const v = new View3D(canvas, { house, twin, vox, localizer, missions, memory, map, truth, mode: () => "sim", flying: () => false, busy });
//   v.setHouse(house, twin, vox, map); v.setFlight(flightId | null); v.newFlight() (a take-off); v.invalidate(); v.dispose();
import { Emitter, clamp, wrapAngle } from "../util.js";
import { h } from "./dom.js";
import { pinholeLens, intrinsics, DEG } from "../twin/lens.js";
import { droneCamera } from "../twin/pose.js";
import { UNKNOWN, OCCUPIED } from "../house/voxels.js";
import { houseBounds } from "./mapview.js";

export const VIEW3D = {
  hfov: 70, near: 0.05, maxPx: 640, // render width at most (px)
  minInterval: 500, duty: 0.2, busyUse: 0.7, // ms between renders at least; share of the twin's time at most; its use above which it is busy
  tick: 100, settle: 300, // ms: overlay redraws; how long after the view stops moving it renders
  wallH: 0.6, // m: the drawing's cut-away walls
  chase: { el: 28 * DEG, dist: 1.9, min: 0.6 }, snap: 0.4, // m the drone moves before a following view re-centres
  cloud: { step: 4, max: 7000, band: [0.3, 1.6] }, near3: 0.45, // unseen space drawn per 4x4x4-voxel block; solid voxels within near3 m of the path
};
// The same colours as the map (ui/mapview.js): position sure, fairly sure, unsure (by σ; the words give ±2σ, as the
// caption and the HUD do); changes magenta; unseen space pink. The drone: yellow, orange when unsure, red when lost.
export const SIGMA_STEPS = [[0.1, "#4ade80", "sure (±20 cm or better)", ""], [0.25, "#facc15", "fairly sure (±20 to 50 cm)", "fairly sure"], [Infinity, "#f87171", "unsure (more than ±50 cm)", "unsure"]];
const sigmaStep = (s) => SIGMA_STEPS.find(([m]) => (s ?? 0) < m);
export const sigmaColor = (s) => sigmaStep(s)[1];
export const sigmaWord = (s) => sigmaStep(s)[3];
export const STATUS = { ok: "#ffd84d", degraded: "#f97316", lost: "#ef4444" };
const TINT = ["#5b8def", "#ef8a5b", "#5bd1a4", "#c77dde", "#e6c35c", "#5bc2e6", "#e66d8f"];
const PIN = { person: "#f472b6", cat: "#f59e0b", dog: "#a3e635" };
const FONT = "ui-sans-serif, system-ui, -apple-system, sans-serif";
const PINK = [236, 72, 153], CHANGE = "#d946ef";

// ---- camera math (pure) ----

// An orbit camera { target [x, y, z], az (rad, CCW from +x: the way it looks), el (rad down), dist } as a twin pose.
export function orbitPose({ target, az, el, dist }) {
  const f = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), -Math.sin(el)];
  return { x: target[0] - dist * f[0], y: target[1] - dist * f[1], z: target[2] - dist * f[2], yaw: az, pitch: el, roll: 0 };
}

// H -> camera (OpenCV: x right, y down, z forward) and pixels for a W x H picture of a pinhole hfov wide.
export function projector(cam, W, H, hfov = VIEW3D.hfov) {
  const pose = orbitPose(cam), c = droneCamera(pose, 0), R = c.R, p = c.p, K = intrinsics(pinholeLens(hfov), W, H);
  const toCam = (P) => {
    const d0 = P[0] - p[0], d1 = P[1] - p[1], d2 = P[2] - p[2];
    return [R[0][0] * d0 + R[1][0] * d1 + R[2][0] * d2, R[0][1] * d0 + R[1][1] * d1 + R[2][1] * d2, R[0][2] * d0 + R[1][2] * d1 + R[2][2] * d2];
  };
  const pix = (q) => [K.cx + (K.fx * q[0]) / q[2], K.cy + (K.fy * q[1]) / q[2]];
  return { pose, R, p, K, W, H, toCam, pix, px: (P) => { const q = toCam(P); return q[2] > VIEW3D.near ? [...pix(q), q[2]] : null; } };
}

// A polygon (camera frame) cut to the part in front of the near plane.
export function clipPoly(cs, near = VIEW3D.near) {
  const out = [];
  for (let i = 0; i < cs.length; i++) {
    const a = cs[i], b = cs[(i + 1) % cs.length], ina = a[2] >= near, inb = b[2] >= near;
    if (ina) out.push(a);
    if (ina !== inb) { const t = (near - a[2]) / (b[2] - a[2]); out.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), near]); }
  }
  return out;
}
export function clipSeg(a, b, near = VIEW3D.near) {
  if (a[2] < near && b[2] < near) return null;
  const cut = (p, q) => { const t = (near - p[2]) / (q[2] - p[2]); return [p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]), near]; };
  return [a[2] < near ? cut(a, b) : a, b[2] < near ? cut(b, a) : b];
}

// The trail as coloured, fading pieces: { segments: [{ a, b, color, alpha, sigma }], lost: [the last sure sample before each
// loss] }. A live flight fades over `fade` ms of age; a finished one from its first sample to its last.
export function trailSegments(samples, { now = Date.now(), live = false, fade = 120000, gap = 5000 } = {}) {
  const segments = [], lost = [], t1 = live ? now : samples.at(-1)?.t ?? now, t0 = live ? t1 - fade : samples[0]?.t ?? t1, span = Math.max(1, t1 - t0);
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1], b = samples[i];
    if (b.lost) { if (!a.lost) lost.push(a); continue; }
    if (a.lost || b.t - a.t > gap) continue;
    const sigma = Math.max(a.sigma ?? 0, b.sigma ?? 0);
    segments.push({ a, b, sigma, color: sigmaColor(sigma), alpha: +(0.2 + 0.8 * clamp((b.t - t0) / span, 0, 1)).toFixed(3) });
  }
  return { segments, lost };
}

// Inside a room, off the floor and below its ceiling (and not inside something, by the voxels): a twin render from there
// shows the room, not the outside of the scan.
export function insideRooms(map, vox, [x, y, z]) {
  const room = map?.roomAt?.(x, y);
  if (!room) return false;
  const fl = map.floorAt(x, y), ce = map.ceilingAt(x, y);
  if (fl == null || z < fl + 0.08 || (Number.isFinite(ce) && z > ce - 0.12)) return false;
  return !vox || vox.state(x, y, z) !== OCCUPIED;
}

// Unknown space at the drone's heights inside the rooms, one point per step x step x step block of voxels that has any
// (the block's unknown voxel nearest its middle) and the share of the block's space there that is unknown:
// Float32Array x, y, z, share. More than max blocks: every k-th.
export function unknownCloud(vox, map, { step = VIEW3D.cloud.step, max = VIEW3D.cloud.max, band = VIEW3D.cloud.band } = {}) {
  const { nx, ny, nz, res } = vox, floor = new Float32Array(nx * ny).fill(NaN); // the floor under each column inside a room
  for (let r = 0; r < ny; r++)
    for (let c = 0; c < nx; c++) {
      const k = map.idx(vox.x0 + (c + 0.5) * res, vox.y0 + (r + 0.5) * res);
      if (k >= 0 && map.room[k] >= 0 && !map.wall[k]) floor[r * nx + c] = map.floorZ[k];
    }
  const pts = [], m = (step - 1) / 2;
  for (let L = 0; L < nz; L += step)
    for (let R = 0; R < ny; R += step)
      for (let C = 0; C < nx; C += step) {
        let all = 0, n = 0, best = -1, bd = Infinity;
        for (let l = L; l < Math.min(nz, L + step); l++)
          for (let r = R; r < Math.min(ny, R + step); r++)
            for (let c = C; c < Math.min(nx, C + step); c++) {
              const f = floor[r * nx + c], up = vox.z0 + (l + 0.5) * res - f;
              if (!(up >= band[0] && up <= band[1])) continue; // NaN (no room) too
              all++;
              const i = (l * ny + r) * nx + c;
              if (vox.st[i] !== UNKNOWN) continue;
              n++;
              const d = (l - L - m) ** 2 + (r - R - m) ** 2 + (c - C - m) ** 2;
              if (d < bd) [best, bd] = [i, d];
            }
        if (n) pts.push(...vox.center(best), n / all);
      }
  const blocks = pts.length / 4, keep = Math.max(1, Math.ceil(blocks / max)), out = new Float32Array(4 * Math.ceil(blocks / keep));
  for (let i = 0, j = 0; i < blocks; i += keep, j++) out.set(pts.slice(4 * i, 4 * i + 4), 4 * j);
  return out;
}

// Solid voxels within r of a path (every other voxel each way), as x, y, z triples.
export function solidNear(vox, path, r = VIEW3D.near3, max = 3000) {
  const seen = new Set(), out = [], s = 2 * vox.res;
  for (let k = 0; k + 1 < (path?.length ?? 0); k++) {
    const a = path[k], b = path[k + 1], L = Math.hypot(b[0] - a[0], b[1] - a[1], (b[2] ?? 0) - (a[2] ?? 0)), n = Math.max(1, Math.ceil(L / 0.15));
    for (let t = 0; t <= n; t++) {
      const q = [a[0] + ((b[0] - a[0]) * t) / n, a[1] + ((b[1] - a[1]) * t) / n, (a[2] ?? 1) + (((b[2] ?? 1) - (a[2] ?? 1)) * t) / n];
      for (let dz = -r; dz <= r; dz += s) for (let dy = -r; dy <= r; dy += s) for (let dx = -r; dx <= r; dx += s) {
        if (dx * dx + dy * dy + dz * dz > r * r) continue;
        const i = vox.idx(q[0] + dx, q[1] + dy, q[2] + dz);
        if (i < 0 || seen.has(i) || vox.st[i] !== OCCUPIED) continue;
        seen.add(i);
        out.push(...vox.center(i));
        if (out.length >= 3 * max) return out;
      }
    }
  }
  return out;
}

const camKey = (c) => `${c.target.map((v) => v.toFixed(3)).join()},${c.az.toFixed(4)},${c.el.toFixed(4)},${c.dist.toFixed(3)}`;
const camApart = (a, b) => {
  const pa = orbitPose(a), pb = orbitPose(b);
  return { m: Math.hypot(pa.x - pb.x, pa.y - pb.y, pa.z - pb.z), rad: Math.max(Math.abs(wrapAngle(a.az - b.az)), Math.abs(a.el - b.el)) };
};
const ring = (x, y, z, r, n = 24) => Array.from({ length: n }, (_, i) => [x + r * Math.cos((2 * Math.PI * i) / n), y + r * Math.sin((2 * Math.PI * i) / n), z]);

export class View3D extends Emitter {
  constructor(canvas, { house = null, twin = null, vox = null, map = null, localizer = null, missions = null, memory = null, truth = null,
    mode = () => "sim", flying = () => false, busy = null, now = Date.now } = {}) {
    super();
    Object.assign(this, { canvas, localizer, memory, truth, mode, flying, busyFn: busy, now });
    Object.assign(this, { cam: null, follow: true, cloudOn: false, keyOn: false, flightId: null, flightTrail: null, flightSeq: 0, path: null, plan: null, goingHome: false, homePath: false, findings: [], hits: [] });
    Object.assign(this, { bg: null, rendering: false, lastRender: 0, renderMs: 60, renders: 0, skipped: { busy: 0, flying: 0, outside: 0 }, moving: 0, version: 0 });
    this.base = document.createElement("canvas");
    this.baseKey = "";
    this.unsub = [];
    canvas.tabIndex = 0;
    canvas.classList.add("v3d-canvas");
    canvas.setAttribute("role", "application");
    canvas.setAttribute("aria-label", "3D view of the house. Arrow keys turn the view, plus and minus zoom, 0 shows the whole house, F follows the drone.");
    this.buildUi();
    this.bind();
    this.setMissions(missions);
    this.setHouse(house, twin, vox, map);
    this.loop();
  }

  // ---------------------------------------------------------------------------------------------- state

  setHouse(house, twin = null, vox = null, map = this.map) {
    const same = house && house.id === this.house?.id;
    Object.assign(this, { house, twin, vox, map });
    this.bg?.bitmap.close?.();
    this.bg = null;
    this.cloud = null;
    this.solid = null;
    if (!same) (this.cam = null), (this.findings = []), (this.path = null), (this.plan = null), (this.goingHome = false);
    this.invalidate();
  }

  setMemory(memory) {
    this.memory = memory;
    this.invalidate();
  }

  setMissions(missions) {
    this.unsub.forEach((f) => f());
    this.missions = missions;
    this.unsub = missions?.on ? [
      missions.on("path", (e) => ((this.path = e?.path ?? e), (this.solid = null), (this.homePath = this.goingHome))),
      missions.on("plan", (p) => (this.plan = p)),
      // flying home (returnHome says so, without a path of its own): the last leg's path and stops are no longer ahead
      missions.on("status", (e) => {
        const home = e?.phase === "home";
        if (home && !this.goingHome && !this.homePath) Object.assign(this, { path: null, plan: null, solid: null });
        Object.assign(this, { goingHome: home }, !home && { homePath: false });
      }),
      missions.on("finding", (f) => Number.isFinite(f?.x) && this.findings.push(f) && this.findings.length > 50 && this.findings.shift()),
      missions.on("done", () => ((this.path = null), (this.plan = null), (this.solid = null), (this.goingHome = false), (this.homePath = false))),
    ] : [];
  }

  // A past flight's trail (History; read back from the memory's store when no longer in RAM), or null: this flight's (or
  // the last one's, on the ground).
  async setFlight(flightId) {
    const seq = ++this.flightSeq;
    Object.assign(this, { flightId, flightTrail: null });
    const all = flightId ? (this.memory?.trails?.get(flightId) ?? (await this.memory?.trailOf?.(flightId).catch(() => null)) ?? this.memory?.trail?.({ flightId }) ?? []) : null;
    if (seq !== this.flightSeq) return;
    this.flightTrail = all;
    const tr = all?.filter((s) => !s.lost);
    if (tr?.length) {
      const xs = tr.map((s) => s.x), ys = tr.map((s) => s.y), zs = tr.map((s) => s.z);
      const c = [(Math.min(...xs) + Math.max(...xs)) / 2, (Math.min(...ys) + Math.max(...ys)) / 2, Math.min(...zs)];
      this.setFollow(false);
      this.cam = { target: c, az: 75 * DEG, el: 55 * DEG, dist: Math.max(3, 1.3 * Math.hypot(Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys))) };
      this.moved(false);
    }
    this.invalidate();
  }

  // A take-off: this flight's findings from now on, and its own trail (not a past flight's from History).
  newFlight() {
    this.findings = [];
    if (this.flightId) this.setFlight(null);
    else this.invalidate();
  }

  // The house or the voxels changed: the drawing and the haze again.
  invalidate() {
    this.version++;
    this.cloud = null;
    this.solid = null;
  }

  setFollow(on) {
    this.follow = on;
    this.ui.follow.setAttribute("aria-pressed", String(on));
    if (on) this.cam = null;
  }

  overview() {
    if (!this.house) return null;
    const b = houseBounds(this.house, 0.2), z = Math.min(...this.house.rooms.map((r) => r.floorZ ?? 0));
    const span = Math.max(b.x1 - b.x0, b.y1 - b.y0), aspect = this.canvas.clientHeight / Math.max(1, this.canvas.clientWidth);
    const half = Math.min(VIEW3D.hfov / 2, Math.atan(Math.tan((VIEW3D.hfov * DEG) / 2) * aspect) / DEG); // the narrower of the two
    return { target: [(b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, z], az: 70 * DEG, el: 58 * DEG, dist: (0.6 * span) / Math.tan(half * DEG) + 1.5 };
  }

  // Behind and above the drone, looking where it looks, moved until the camera is inside a room (fit()).
  chase(p) {
    const C = VIEW3D.chase, cam = { target: [p.x, p.y, p.z], az: p.yaw ?? 0, el: C.el, dist: C.dist };
    return this.fit(cam) ?? { ...cam, dist: C.min };
  }

  // The camera cam, or the nearest one round the same target inside a room: pulled in first (to chase.min), then turned
  // (±45°, ±90°, ±135°, 180°), then from higher or lower; null when there's none (the drawing then).
  fit(cam) {
    const ok = (c) => insideRooms(this.map, this.vox, this.posOf(c)), min = Math.min(cam.dist, VIEW3D.chase.min);
    for (const el of [cam.el, 45 * DEG, 15 * DEG, 65 * DEG])
      for (const turn of [0, 45, -45, 90, -90, 135, -135, 180])
        for (let dist = cam.dist; dist >= min - 1e-9; dist -= 0.15) {
          const c = { ...cam, az: wrapAngle(cam.az + turn * DEG), el, dist };
          if (ok(c)) return c;
        }
    return null;
  }

  posOf(cam) {
    const q = orbitPose(cam);
    return [q.x, q.y, q.z];
  }

  pose() {
    const p = this.localizer?.pose?.();
    return p && Number.isFinite(p.x) && this.localizer.known !== false ? p : null;
  }

  // Following: re-centre once the drone has moved VIEW3D.snap from the middle, or out of the picture (keeping the turn
  // and zoom the user chose).
  track(p) {
    if (!this.follow || !p) return;
    if (!this.cam) return void (this.cam = this.chase(p));
    const t = this.cam.target, off = Math.hypot(p.x - t[0], p.y - t[1], p.z - t[2]), L = this.lastProj, pr = L?.px([p.x, p.y, p.z]);
    const out = L && (!pr || pr[0] < 0.15 * L.W || pr[0] > 0.85 * L.W || pr[1] < 0.15 * L.H || pr[1] > 0.85 * L.H);
    if (off > VIEW3D.snap || out) {
      const keep = { ...this.cam, target: [p.x, p.y, p.z] };
      this.cam = this.cam.dist > 4 ? keep : this.inRoom(keep);
    }
  }

  inRoom(cam) {
    return this.fit(cam) ?? cam;
  }

  moved(interactive = true) {
    if (interactive) this.moving = performance.now();
  }

  // ---------------------------------------------------------------------------------------------- the twin

  // Why a twin render may not happen now ("" when it may).
  holdReason() {
    if (!this.twin) return "the 3D scan isn't loaded";
    if (this.mode() === "real" && this.flying()) return "flying";
    if (!insideRooms(this.map, this.vox, this.posOf(this.cam))) return "outside";
    return "";
  }

  // The owner's word (busy()), the twin's own flag if it has one, else its stats: the share of the last second it spent
  // rendering, while it is rendering at all (the stats come with each answer, so they go stale when it is idle).
  twinBusy() {
    if (this.busyFn?.() || this.twin?.busy === true) return true;
    const s = this.twin?.stats, now = performance.now();
    if (s?.renders !== this.twinRenders) [this.twinRenders, this.twinSeen] = [s?.renders, now];
    if (now - (this.twinSeen ?? 0) > 1000) return false;
    return (s?.fps && s.renderMsAvg ? (s.fps * s.renderMsAvg) / 1000 : 0) > VIEW3D.busyUse;
  }

  async maybeRender(W, H) {
    if (this.rendering || !this.cam) return;
    const t = performance.now(), why = this.holdReason();
    if (why === "flying") return void this.skipped.flying++;
    if (why === "outside") return void this.skipped.outside++;
    if (why) return;
    if (t - this.moving < VIEW3D.settle) return;
    if (this.bg && camKey(this.bg.cam) === camKey(this.cam) && this.bg.aspect === W / H) return;
    if (t - this.lastRender < Math.max(VIEW3D.minInterval, (this.renderMs * (1 - VIEW3D.duty)) / VIEW3D.duty)) return;
    if (this.twinBusy()) return void this.skipped.busy++;
    const k = Math.min(1, VIEW3D.maxPx / W), w = Math.max(64, Math.round(W * k)), hh = Math.max(48, Math.round(H * k)), cam = { ...this.cam, target: [...this.cam.target] }, twin = this.twin;
    this.rendering = true;
    try {
      const bitmap = await twin.render(orbitPose(cam), { width: w, height: hh, lens: pinholeLens(VIEW3D.hfov), actors: true });
      if (twin !== this.twin) return bitmap.close?.();
      this.bg?.bitmap.close?.();
      this.bg = { bitmap, cam, aspect: W / H, at: Date.now() };
      this.renders++;
    } catch (e) {
      if (!/disposed/.test(e?.message)) this.emit("error", e);
    } finally {
      this.renderMs = 0.7 * this.renderMs + 0.3 * (performance.now() - t);
      this.lastRender = performance.now();
      this.rendering = false;
    }
  }

  // ---------------------------------------------------------------------------------------------- drawing

  loop() {
    if (this.disposed) return;
    try {
      if (this.canvas.isConnected && this.canvas.clientWidth > 0) this.draw();
    } catch (e) {
      console.error("View3D", e);
    }
    this.timer = setTimeout(() => this.loop(), VIEW3D.tick);
  }

  size() {
    const c = this.canvas, dpr = window.devicePixelRatio || 1, w = Math.round(c.clientWidth * dpr), hh = Math.round(c.clientHeight * dpr);
    if (c.width !== w || c.height !== hh) Object.assign(c, { width: w, height: hh });
    return { W: w, H: hh, k: dpr };
  }

  draw() {
    const { W, H, k } = this.size(), g = this.canvas.getContext("2d");
    if (!W || !H) return;
    g.setTransform(1, 0, 0, 1, 0, 0);
    if (!this.house || !this.map) {
      g.fillStyle = "#0a0e13";
      g.fillRect(0, 0, W, H);
      this.caption("", "");
      return;
    }
    const pose = this.pose();
    this.track(pose);
    this.cam ??= pose && this.follow ? this.chase(pose) : this.overview();
    // The last render, with its own camera, while it is (about) the view wanted: else the drawing at the camera wanted.
    const steady = performance.now() - this.moving > VIEW3D.settle, bg = this.bg, near = bg && bg.aspect === W / H && camApart(bg.cam, this.cam);
    const useBg = bg && near && (camKey(bg.cam) === camKey(this.cam) || (steady && near.m < 0.6 && near.rad < 25 * DEG) || (this.mode() === "real" && this.flying() && steady && near.m < 1.5));
    const cam = useBg ? bg.cam : this.cam, P = projector(cam, W, H);
    this.lastProj = P;
    if (useBg) g.drawImage(bg.bitmap, 0, 0, W, H);
    else {
      const key = `${W}x${H}:${camKey(cam)}:${this.version}:${this.cloudOn}`;
      if (key !== this.baseKey) this.drawBase(P, k, key);
      g.drawImage(this.base, 0, 0);
    }
    if (useBg && this.cloudOn) this.drawCloud(g, P, k);
    this.hits = [];
    this.drawLive(g, P, k, pose);
    this.badge(useBg, bg);
    this.maybeRender(W, H);
  }

  drawBase(P, k, key) {
    const b = this.base, { W, H } = P;
    Object.assign(b, { width: W, height: H });
    const g = b.getContext("2d"), house = this.house, map = this.map;
    this.baseKey = key;
    const sky = g.createLinearGradient(0, 0, 0, H);
    sky.addColorStop(0, "#0d131b");
    sky.addColorStop(1, "#070a0e");
    g.fillStyle = sky;
    g.fillRect(0, 0, W, H);
    const faces = [], poly = (pts) => clipPoly(pts.map(P.toCam));
    const floorOf = (r) => r.floorZ ?? map.floorAt(r.outline[0][0], r.outline[0][1]) ?? 0;
    house.rooms.forEach((r, i) => {
      const z = floorOf(r), c = poly(r.outline.map(([x, y]) => [x, y, z]));
      if (c.length >= 3) faces.push({ c, depth: Math.max(...c.map((q) => q[2])) + 50, fill: `${TINT[i % TINT.length]}33`, stroke: "rgba(214,221,230,0.35)" });
      r.outline.forEach((a, j) => {
        const bb = r.outline[(j + 1) % r.outline.length], q = poly([[a[0], a[1], z], [bb[0], bb[1], z], [bb[0], bb[1], z + VIEW3D.wallH], [a[0], a[1], z + VIEW3D.wallH]]);
        if (q.length >= 3) faces.push({ c: q, depth: q.reduce((s, v) => s + v[2], 0) / q.length, fill: "rgba(148,163,184,0.16)", stroke: "rgba(214,221,230,0.55)" });
      });
    });
    for (const d of house.doors ?? []) {
      const z = map.floorAt((d.a[0] + d.b[0]) / 2, (d.a[1] + d.b[1]) / 2) ?? 0, top = z + Math.min(VIEW3D.wallH + 0.25, (d.headZ ?? z + 2) - z);
      const q = poly([[d.a[0], d.a[1], z], [d.b[0], d.b[1], z], [d.b[0], d.b[1], top], [d.a[0], d.a[1], top]]);
      if (q.length >= 3) faces.push({ c: q, depth: q.reduce((s, v) => s + v[2], 0) / q.length - 0.05, fill: d.passable === false ? "rgba(239,68,68,0.35)" : "rgba(74,222,128,0.12)", stroke: d.passable === false ? "#f87171" : "rgba(74,222,128,0.8)", width: 1.3 });
    }
    for (const ko of map.keepouts ?? []) {
      const cx = ko.x ?? ko.polygon?.[0][0], cy = ko.y ?? ko.polygon?.[0][1], fl = map.floorAt(cx, cy) ?? 0, ce = map.ceilingAt(cx, cy);
      const z0 = Math.max(fl, ko.zMin ?? fl), z1 = Math.min(ko.zMax ?? fl + 2.2, Number.isFinite(ce) ? ce : fl + 2.4);
      const foot = ko.polygon ?? ring(ko.x, ko.y, 0, ko.r, 16).map(([x, y]) => [x, y]);
      for (const z of [z0, z1]) {
        const q = poly(foot.map(([x, y]) => [x, y, z]));
        if (q.length >= 3) faces.push({ c: q, depth: Math.max(...q.map((v) => v[2])) + 0.01, fill: z === z0 ? "rgba(239,68,68,0.22)" : "rgba(239,68,68,0.08)", stroke: "rgba(248,113,113,0.8)", dash: true });
      }
    }
    faces.sort((a, b) => b.depth - a.depth);
    g.lineJoin = "round";
    for (const f of faces) {
      g.beginPath();
      f.c.forEach((q, i) => { const [u, v] = P.pix(q); i ? g.lineTo(u, v) : g.moveTo(u, v); });
      g.closePath();
      g.fillStyle = f.fill;
      g.fill();
      g.setLineDash(f.dash ? [5 * k, 4 * k] : []);
      g.strokeStyle = f.stroke;
      g.lineWidth = (f.width ?? 1.2) * k;
      g.stroke();
    }
    g.setLineDash([]);
    house.rooms.forEach((r) => {
      const c = r.outline.reduce(([a, b], p) => [a + p[0] / r.outline.length, b + p[1] / r.outline.length], [0, 0]), q = P.px([c[0], c[1], floorOf(r)]);
      if (q) label(g, r.name, q[0], q[1], k, "#f1f5f9", Math.max(9, Math.min(13, 60 / q[2])), true);
    });
    for (const lm of house.landmarks ?? []) {
      if (!Number.isFinite(lm.x) || lm.footprint || !["user", "claude"].includes(lm.source)) continue;
      const q = P.px([lm.x, lm.y, lm.z ?? (map.floorAt(lm.x, lm.y) ?? 0) + 0.5]);
      if (!q) continue;
      g.fillStyle = lm.source === "user" ? "#ffd84d" : "#cbd5e1";
      g.beginPath();
      g.arc(q[0], q[1], 3 * k, 0, 7);
      g.fill();
      if (q[2] < 6) label(g, lm.name, q[0], q[1] - 10 * k, k, "#e2e8f0", 10);
    }
    if (this.cloudOn) this.drawCloud(g, P, k);
  }

  drawCloud(g, P, k) {
    if (!this.vox || !this.map) return;
    this.cloud ??= unknownCloud(this.vox, this.map);
    const c = this.cloud, pts = [], block = VIEW3D.cloud.step * this.vox.res;
    for (let i = 0; i < c.length; i += 4) {
      const q = P.px([c[i], c[i + 1], c[i + 2]]);
      if (q && q[0] > -20 && q[0] < P.W + 20 && q[1] > -20 && q[1] < P.H + 20) pts.push([...q, c[i + 3]]);
    }
    pts.sort((a, b) => b[2] - a[2]);
    for (const [u, v, z, share] of pts) { // a square the block's size, more opaque the more of it is unseen
      const s = clamp((block * P.K.fx) / z, 1.2 * k, 14 * k);
      g.fillStyle = `rgba(${PINK.join()},${(clamp(0.32 / Math.sqrt(z), 0.06, 0.3) * (0.45 + 0.55 * share)).toFixed(3)})`;
      g.fillRect(u - s / 2, v - s / 2, s, s);
    }
    if (this.path?.length > 1) {
      this.solid ??= solidNear(this.vox, this.path);
      for (let i = 0; i < this.solid.length; i += 3) {
        const q = P.px([this.solid[i], this.solid[i + 1], this.solid[i + 2]]);
        if (!q) continue;
        const s = clamp((0.1 * P.K.fx) / q[2], 1.5 * k, 10 * k);
        g.fillStyle = "rgba(30,41,59,0.75)";
        g.fillRect(q[0] - s / 2, q[1] - s / 2, s, s);
      }
    }
  }

  line3(g, P, a, b) {
    const s = clipSeg(P.toCam(a), P.toCam(b));
    if (!s) return false;
    const [u0, v0] = P.pix(s[0]), [u1, v1] = P.pix(s[1]);
    g.moveTo(u0, v0);
    g.lineTo(u1, v1);
    return true;
  }

  shape3(g, P, pts) {
    const c = clipPoly(pts.map(P.toCam));
    if (c.length < 2) return false;
    c.forEach((q, i) => { const [u, v] = P.pix(q); i ? g.lineTo(u, v) : g.moveTo(u, v); });
    g.closePath();
    return true;
  }

  floor(x, y) {
    return this.map.floorAt(x, y) ?? 0;
  }

  drawLive(g, P, k, pose) {
    const map = this.map, house = this.house, now = this.now();
    // temporary obstacles: people (a red disc and column), live-depth blockers (amber), closed doorways seen in flight (red)
    for (const t of map.temps?.values?.() ?? []) {
      if (t.kind === "change") continue;
      const red = t.kind === "person" || t.source === "door leaf", fl = this.floor(t.x ?? t.polygon?.[0][0], t.y ?? t.polygon?.[0][1]);
      const foot = t.polygon ?? (Number.isFinite(t.r) ? ring(t.x, t.y, 0, t.r, 20).map(([x, y]) => [x, y]) : null);
      if (!foot) continue;
      g.beginPath();
      if (this.shape3(g, P, foot.map(([x, y]) => [x, y, fl + 0.01]))) {
        g.fillStyle = red ? "rgba(239,68,68,0.25)" : "rgba(245,158,11,0.25)";
        g.fill();
        g.strokeStyle = red ? "#f87171" : "#fbbf24";
        g.lineWidth = 1.5 * k;
        g.stroke();
      }
    }
    // home pad
    if (house.home) {
      const z = this.floor(house.home.x, house.home.y) + 0.01;
      g.beginPath();
      if (this.shape3(g, P, ring(house.home.x, house.home.y, z, 0.15))) {
        g.fillStyle = "rgba(56,189,248,0.25)";
        g.fill();
        g.strokeStyle = "#38bdf8";
        g.lineWidth = 2 * k;
        g.stroke();
      }
      const q = P.px([house.home.x, house.home.y, z]);
      if (q) label(g, "H", q[0], q[1], k, "#e0f2fe", 11, true);
    }
    // where it has been
    const flight = this.flightId ?? this.memory?.flight?.id ?? this.memory?.flights?.at(-1)?.id ?? null, live = !this.flightId && !!this.memory?.flight;
    const tr = this.flightId ? this.flightTrail ?? [] : flight ? this.memory.trail({ flightId: flight })
      : (this.truth?.()?.trail ?? []).map((p, i, a) => ({ ...p, z: p.z ?? this.floor(p.x, p.y) + 1, t: now - (a.length - i) * 100, sigma: 0 }));
    const { segments, lost } = trailSegments(tr, { now, live: live || !flight });
    g.lineCap = "round";
    for (const s of segments) {
      g.beginPath();
      if (!this.line3(g, P, [s.a.x, s.a.y, s.a.z], [s.b.x, s.b.y, s.b.z])) continue;
      g.globalAlpha = s.alpha;
      g.strokeStyle = s.color;
      g.lineWidth = 3 * k;
      g.stroke();
    }
    g.globalAlpha = 1;
    this.drawn = { trail: segments.length, lost: lost.length, flight };
    for (const s of lost) {
      const q = P.px([s.x, s.y, s.z]);
      if (q) label(g, "✕", q[0], q[1], k, "#ef4444", 14, true);
    }
    // where it will go: the path, its stops
    const path = this.path;
    if (path?.length > 1) {
      g.beginPath();
      for (let i = 1; i < path.length; i++) this.line3(g, P, [path[i - 1][0], path[i - 1][1], path[i - 1][2] ?? 1], [path[i][0], path[i][1], path[i][2] ?? 1]);
      g.setLineDash([7 * k, 5 * k]);
      g.strokeStyle = "rgba(255,216,77,0.95)";
      g.lineWidth = 2.5 * k;
      g.stroke();
      g.setLineDash([]);
      const end = path.at(-1), q = P.px([end[0], end[1], end[2] ?? 1]);
      if (q) {
        g.fillStyle = "#ffd84d";
        g.beginPath();
        g.arc(q[0], q[1], 5 * k, 0, 7);
        g.fill();
      }
    }
    for (const s of this.plan?.stops ?? []) {
      if (!Number.isFinite(s.x)) continue;
      const q = P.px([s.x, s.y, this.floor(s.x, s.y) + 1.0]);
      if (q) label(g, `${s.name ?? ""}${s.seconds != null ? ` · ${Math.round(s.seconds)} s` : ""}`, q[0], q[1] - 12 * k, k, "#fde68a", 10.5);
    }
    // what it saw, and changes waiting for a decision
    const seen = this.memory ? this.memory.sightings.filter((s) => s.flight === flight && Number.isFinite(s.x)) : [];
    const pins = [...seen.map((s) => ({ kind: "sighting", id: s.id, x: s.x, y: s.y, z: s.z, label: s.label })),
      ...(this.memory ? [] : this.findings.map((f, i) => ({ kind: "finding", id: i, x: f.x, y: f.y, z: f.z, label: f.label })))];
    for (const p of pins) this.pin(g, P, k, p, PIN[p.label] ?? "#cbd5e1", p.label);
    for (const c of this.memory?.changes ?? []) if (c.status === "suspected" && Number.isFinite(c.x)) this.pin(g, P, k, { kind: "change", id: c.id, x: c.x, y: c.y, z: c.z }, CHANGE, "!", true);
    // the drone now, and in the simulator where it really is (white outline)
    const tp = this.truth?.();
    this.drawn.truth = !!(tp && Number.isFinite(tp.x) && pose?.source !== "truth"
      && this.drone(g, P, k, { ...tp, z: Number.isFinite(tp.z) ? tp.z : pose?.z ?? this.floor(tp.x, tp.y) + 0.05 }, null));
    if (pose) this.drone(g, P, k, pose, STATUS[pose.status] ?? STATUS.ok);
  }

  pin(g, P, k, p, color, text, change = false) {
    const fl = this.floor(p.x, p.y), top = Math.max(fl + 0.35, p.z ?? fl + 0.5) + 0.25, base = P.px([p.x, p.y, fl]), head = P.px([p.x, p.y, top]);
    if (!head) return;
    if (base) {
      g.strokeStyle = color;
      g.lineWidth = 1.5 * k;
      g.beginPath();
      g.moveTo(base[0], base[1]);
      g.lineTo(head[0], head[1]);
      g.stroke();
    }
    const r = clamp(30 / head[2], 6, 10) * k;
    g.fillStyle = color;
    g.beginPath();
    if (change) (g.moveTo(head[0], head[1] - r * 1.2), g.lineTo(head[0] + r, head[1]), g.lineTo(head[0], head[1] + r * 1.2), g.lineTo(head[0] - r, head[1]), g.closePath());
    else g.arc(head[0], head[1], r, 0, 7);
    g.fill();
    g.strokeStyle = "rgba(5,8,12,0.8)";
    g.lineWidth = k;
    g.stroke();
    label(g, change ? "!" : text[0].toUpperCase(), head[0], head[1], k, "#111", 9.5, true, false);
    if (!change && head[2] < 5) label(g, text, head[0], head[1] - r - 8 * k, k, "#fde68a", 10);
    this.hits.push({ kind: p.kind, id: p.id, u: head[0], v: head[1], x: p.x, y: p.y, z: p.z });
  }

  drone(g, P, k, p, color) {
    const fl = this.floor(p.x, p.y), at = P.px([p.x, p.y, p.z]);
    if (!at) return false;
    if (color && p.sigma > 0 && p.source !== "truth") {
      g.beginPath();
      if (this.shape3(g, P, ring(p.x, p.y, p.z, Math.min(3, 2 * p.sigma)))) {
        g.setLineDash([4 * k, 3 * k]);
        g.strokeStyle = color;
        g.lineWidth = 1.5 * k;
        g.globalAlpha = 0.8;
        g.stroke();
        g.setLineDash([]);
        g.globalAlpha = 1;
      }
    }
    const foot = P.px([p.x, p.y, fl]);
    if (foot && color) {
      g.strokeStyle = "rgba(255,216,77,0.5)";
      g.setLineDash([2 * k, 3 * k]);
      g.lineWidth = k;
      g.beginPath();
      g.moveTo(foot[0], foot[1]);
      g.lineTo(at[0], at[1]);
      g.stroke();
      g.setLineDash([]);
      g.fillStyle = "rgba(0,0,0,0.35)";
      g.beginPath();
      if (this.shape3(g, P, ring(p.x, p.y, fl + 0.005, 0.09, 12))) g.fill();
    }
    // an arrow in the drone's plane, 0.18 m long, pointing where it looks
    const c = Math.cos(p.yaw ?? 0), s = Math.sin(p.yaw ?? 0), L = 0.16, Wd = 0.09;
    const pts = [[L, 0], [-L * 0.7, Wd], [-L * 0.35, 0], [-L * 0.7, -Wd]].map(([a, b]) => [p.x + a * c - b * s, p.y + a * s + b * c, p.z]);
    g.beginPath();
    if (!this.shape3(g, P, pts)) return false;
    if (color) {
      g.fillStyle = color;
      g.fill();
      g.strokeStyle = "rgba(0,0,0,0.7)";
      g.lineWidth = k;
      g.stroke();
      if (at[2] > 3) label(g, "drone", at[0], at[1] + 16 * k, k, color, 10.5, true);
      this.hits.push({ kind: "drone", id: "drone", u: at[0], v: at[1], x: p.x, y: p.y, z: p.z });
    } else {
      g.strokeStyle = "rgba(255,255,255,0.9)";
      g.lineWidth = 1.5 * k;
      g.stroke();
    }
    return true;
  }

  // The words over the view: where the drone is and where it's going; the picture's source (short: a narrow view).
  caption(where, next) {
    if (this.ui.where.textContent !== where) this.ui.where.textContent = where;
    if (this.ui.next.textContent !== next) this.ui.next.textContent = next;
    this.ui.next.hidden = !next;
  }

  badge(useBg, bg) {
    const pose = this.pose(), map = this.map;
    let where = "Position unknown: set the drone on its home pad";
    if (pose) {
      const room = map.roomAt(pose.x, pose.y)?.name, up = pose.z - this.floor(pose.x, pose.y);
      const word = [sigmaWord(pose.sigma), pose.stale && "no fresh camera fix"].filter(Boolean).join(", "); // as the localizer's "degraded"
      where = pose.status === "lost" ? `Position lost${room ? `: last in ${room}` : ""}`
        : `${room ? `In ${room}` : "Off the map"}, ${Math.max(0, up).toFixed(1)} m up${pose.source === "truth" ? "" : `, ±${Math.round(2 * pose.sigma * 100)} cm${word ? ` (${word})` : ""}`}`;
    }
    const stops = this.plan?.stops?.filter((s) => s.name) ?? [], next = this.goingHome ? "Going home to the pad"
      : stops.length ? `Next: ${stops.slice(0, 3).map((s) => s.name).join(" → ")}${this.plan.seconds ? ` (about ${Math.round(this.plan.seconds)} s)` : ""}`
      : this.path?.length > 1 ? "Flying a planned path" : "";
    this.caption(where, next);
    const why = this.holdReason(), text = useBg ? (this.mode() === "real" && this.flying() ? `3D scan, as of ${new Date(bg.at).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}: it's busy finding the drone while it flies` : "3D scan")
      : why === "flying" ? "Drawing: the 3D scan is busy finding the drone while it flies"
      : why === "outside" ? (this.follow ? "Drawing: no room for the camera here; drag to turn the view" : "Drawing: the 3D scan shows from inside a room; zoom in or press Follow")
      : why ? `Drawing: ${why}` : "Drawing";
    if (this.ui.badgeLong.textContent !== text) (this.ui.badgeLong.textContent = text), (this.ui.badgeShort.textContent = useBg ? "3D scan" : "Drawing"), this.ui.badge.setAttribute("title", text);
  }

  // ---------------------------------------------------------------------------------------------- controls

  buildUi() {
    const host = this.canvas.parentElement, btn = (text, title, fn, extra = {}) => h("button", { type: "button", title, "aria-label": /^\w/.test(text) ? `${text}: ${title}` : title, onclick: fn, ...extra }, text);
    host?.classList.add("v3d-host");
    const ui = (this.ui = {});
    ui.follow = btn("Follow", "follow the drone", () => this.setFollow(!this.follow), { "aria-pressed": "true" });
    ui.cloud = btn("Unseen", "show the space nobody has seen (pink): the drone never flies there", () => this.toggleCloud(), { "aria-pressed": "false" });
    ui.keyBtn = btn("Key", "what the colours mean", () => this.toggleKey(), { "aria-expanded": "false" });
    ui.where = h("span", { class: "v3d-where" });
    ui.next = h("span", { class: "v3d-next" });
    ui.badgeLong = h("span", { class: "v3d-long" });
    ui.badgeShort = h("span", { class: "v3d-short", "aria-hidden": "true" });
    ui.badge = h("span", { class: "v3d-badge", role: "status" }, ui.badgeLong, ui.badgeShort);
    ui.key = h("div", { class: "v3d-key", hidden: true },
      h("p", {}, h("i", { class: "v3d-sw", style: `background:${STATUS.ok}` }), "The drone (dashed ring: where it could be); ", h("i", { class: "v3d-sw", style: `background:${STATUS.degraded}` }),
        "orange when it's unsure where it is (±50 cm or more, or no fresh camera fix), ", h("i", { class: "v3d-sw", style: `background:${STATUS.lost}` }), "red when it has lost its position. Tap it to follow it."),
      h("p", {}, "Its trail, by how sure it was of its position: ", SIGMA_STEPS.flatMap(([, c, w], i) => [i ? ", " : "", h("span", { class: "v3d-chip" }, h("i", { class: "v3d-sw", style: `background:${c}` }), w)]), ". Older parts are fainter; ✕ marks where it lost its position."),
      h("p", {}, h("i", { class: "v3d-sw dash" }), "Where it will fly next"),
      h("p", {}, h("i", { class: "v3d-sw", style: `background:${CHANGE}` }), "A change from the 3D scan waiting for your decision (tap it)"),
      h("p", {}, h("i", { class: "v3d-sw", style: "background:#f472b6" }), "Person, ", h("i", { class: "v3d-sw", style: "background:#f59e0b" }), "cat, ", h("i", { class: "v3d-sw", style: "background:#a3e635" }), "dog it saw"),
      h("p", {}, h("i", { class: "v3d-sw", style: "background:rgba(239,68,68,.6)" }), "No-fly zone or someone in the way; ", h("i", { class: "v3d-sw", style: "background:#4ade80" }), "open doorway, ", h("i", { class: "v3d-sw", style: "background:#f87171" }), "closed"),
      h("p", {}, h("i", { class: "v3d-sw", style: `background:rgba(${PINK.join()},.5)` }), "Unseen space: the drone treats it like a wall"));
    ui.root = h("div", { class: "v3d-ui" },
      h("div", { class: "v3d-cap" }, ui.where, ui.next),
      h("div", { class: "v3d-bar", role: "toolbar", "aria-label": "3D view" }, ui.follow, btn("House", "show the whole house", () => this.home()), ui.cloud,
        btn("+", "Zoom in", () => this.zoom(0.8), { class: "v3d-zoom" }), btn("−", "Zoom out", () => this.zoom(1.25), { class: "v3d-zoom" }), ui.keyBtn),
      ui.key, ui.badge);
    host?.append(ui.root);
  }

  toggleCloud() {
    this.cloudOn = !this.cloudOn;
    this.ui.cloud.setAttribute("aria-pressed", String(this.cloudOn));
  }

  toggleKey() {
    this.keyOn = !this.keyOn;
    this.ui.key.hidden = !this.keyOn;
    this.ui.keyBtn.setAttribute("aria-expanded", String(this.keyOn));
  }

  home() {
    this.setFollow(false);
    this.cam = this.overview();
  }

  zoom(f) {
    if (!this.cam) return;
    this.cam = { ...this.cam, dist: clamp(this.cam.dist * f, 0.4, 60) };
    this.moved();
  }

  orbit(dAz, dEl) {
    if (!this.cam) return;
    this.cam = { ...this.cam, az: wrapAngle(this.cam.az + dAz), el: clamp(this.cam.el + dEl, 4 * DEG, 85 * DEG) };
    this.moved();
  }

  pan(dx, dy) {
    if (!this.cam) return;
    const s = (this.cam.dist * Math.tan((VIEW3D.hfov * DEG) / 2) * 2) / Math.max(1, this.canvas.width), a = this.cam.az;
    const right = [Math.sin(a), -Math.cos(a)], fwd = [Math.cos(a), Math.sin(a)], t = this.cam.target;
    this.setFollow(false);
    this.cam = { ...this.cam, target: [t[0] - (dx * right[0] - dy * fwd[0]) * s, t[1] - (dx * right[1] - dy * fwd[1]) * s, t[2]] };
    this.moved();
  }

  bind() {
    const c = this.canvas, ac = (this.ac = new AbortController()), o = { signal: ac.signal }, pts = new Map();
    let drag = null;
    const px = (e) => { const r = c.getBoundingClientRect(), d = c.width / Math.max(1, r.width); return [(e.clientX - r.left) * d, (e.clientY - r.top) * d]; };
    c.addEventListener("pointerdown", (e) => {
      c.setPointerCapture(e.pointerId);
      pts.set(e.pointerId, px(e));
      drag = { at: px(e), moved: false, pan: e.shiftKey || e.button === 2, pinch: pts.size === 2 ? this.spread(pts) : null };
    }, o);
    c.addEventListener("pointermove", (e) => {
      if (!pts.has(e.pointerId) || !drag) return;
      const p = px(e), prev = pts.get(e.pointerId), dpr = window.devicePixelRatio || 1;
      pts.set(e.pointerId, p);
      if (Math.hypot(p[0] - drag.at[0], p[1] - drag.at[1]) > 6 * dpr) drag.moved = true;
      if (pts.size === 2) {
        const s = this.spread(pts);
        if (drag.pinch) this.zoom(drag.pinch.d / Math.max(1, s.d));
        if (drag.pinch) this.pan((s.c[0] - drag.pinch.c[0]) / 2, (s.c[1] - drag.pinch.c[1]) / 2);
        drag.pinch = s;
      } else if (drag.pan) this.pan(p[0] - prev[0], p[1] - prev[1]);
      else this.orbit(-(p[0] - prev[0]) * 0.006 / dpr, (p[1] - prev[1]) * 0.005 / dpr);
    }, o);
    const up = (e) => {
      const was = drag;
      pts.delete(e.pointerId);
      if (!pts.size) drag = null;
      else if (drag) drag.pinch = null;
      if (e.type === "pointerup" && was && !was.moved && !pts.size) this.pick(px(e));
      if (!pts.size && document.activeElement === c) c.blur(); // a tap doesn't keep the keys (the simulator's arrows)
    };
    c.addEventListener("pointerup", up, o);
    c.addEventListener("pointercancel", up, o);
    c.addEventListener("contextmenu", (e) => e.preventDefault(), o);
    c.addEventListener("wheel", (e) => {
      const page = document.scrollingElement;
      if (page.scrollHeight > page.clientHeight + 1 && !e.ctrlKey && !e.metaKey && !e.altKey && matchMedia("(max-width: 900px)").matches) return;
      e.preventDefault();
      this.zoom(Math.exp(e.deltaY * 0.0015));
    }, { ...o, passive: false });
    c.addEventListener("keydown", (e) => {
      if (!c.matches(":focus-visible")) return;
      const step = { ArrowLeft: [8 * DEG, 0], ArrowRight: [-8 * DEG, 0], ArrowUp: [0, -5 * DEG], ArrowDown: [0, 5 * DEG] }[e.key];
      if (step) this.orbit(...step);
      else if (e.key === "+" || e.key === "=") this.zoom(0.8);
      else if (e.key === "-" || e.key === "_") this.zoom(1.25);
      else if (e.key === "0") this.home();
      else if (e.key === "f" || e.key === "F") this.setFollow(!this.follow);
      else return;
      e.preventDefault();
      e.stopPropagation();
    }, o);
  }

  spread(pts) {
    const [a, b] = [...pts.values()];
    return { d: Math.hypot(a[0] - b[0], a[1] - b[1]), c: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] };
  }

  pick([u, v]) {
    const r = 16 * (window.devicePixelRatio || 1);
    let best = null, bd = r;
    for (const hit of this.hits) {
      const d = Math.hypot(hit.u - u, hit.v - v);
      if (d < bd) [best, bd] = [hit, d];
    }
    if (best?.kind === "drone") return this.setFollow(true); // the drone: follow it (not a pin: stays in 3D)
    if (best) this.emit("select", { kind: best.kind, id: best.id, x: best.x, y: best.y, z: best.z });
  }

  // How it has been doing: renders, skipped (busy, flying), last render time (ms).
  stats() {
    return { renders: this.renders, skipped: { ...this.skipped }, renderMs: Math.round(this.renderMs), background: this.bg ? "scan" : "drawing" };
  }

  dispose() {
    this.disposed = true;
    clearTimeout(this.timer);
    this.setMissions(null);
    this.ac.abort();
    this.bg?.bitmap.close?.();
    this.bg = null;
    this.ui.root.remove();
    this.canvas.parentElement?.classList.remove("v3d-host");
  }
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
