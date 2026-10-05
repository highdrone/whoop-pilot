// Where the drone is in the house frame H (docs/HOME-DRONE.md): x, y on the plan, z up, yaw CCW from +x.
// Sources: "truth" (the simulator's own pose), "odometry" (dead reckoning from the controller's optical-flow velocity
// and heading, from the last reset), "fused" (the same plus absolute fixes: the home pad, later splat PnP and tags).
// An EKF on s = [x, y, yaw, vx, vy, k, b]: velocity in H, k the flow scale (true velocity = k x the controller's flow
// velocity, whose depth is a guess), b the heading drift (rad/s); and the height's own filter on v = [z, vz, bz, ab, kz]
// (nothing couples it to the plan's position): the climb rate vz, driven by the throttle against the hover throttle (the
// thrust's physics, AZ: ab the acceleration that model misses, a hover estimate a little off) where the controller says
// what it sends, and read by the flow's climb rate as kz vz + bz (kz its scale, as unknown as the scene's depth; bz its
// short-lived bias), each reading weighed by the flow's quality and by how well its climb agrees with the fixes' heights,
// and refused when it disagrees with the rest (the flow read +0.8 m/s while the drone sank; in the twin it reads 0.4-0.6 x
// the climb rate, ±0.3 m/s); vision fixes' heights at their own σ (at least LOC.zFix; 1-4 cm off in the twin), softly
// gated, and their consistency (zNis) scaling both. Each controller tick predicts with the heading change and the
// throttle; on the ground nothing moves, so nothing grows. Measurements are kept with the tick
// they describe (8 s of history: a relocalization can take seconds): the flow velocity (FLOW_LAG + videoDelay old) and
// timestamped absolute fixes (video is late too). A new one goes on its tick and every tick since is replayed with all of
// its measurements; a fix older than the history is refused. sigma (1σ horizontal, the larger axis) grows with distance
// flown (k), turns and time without flow; status: ok < 0.25 m <= degraded < 0.5 m <= lost. `flown`: the drone has flown
// since the last reset, so a lost pose on the ground can't be taken for the home pad.
// Conflict: fixes the gate rejects are kept, carried to now with odometry. Fixes that agree with each other but not with
// the filter are a second hypothesis (pose().conflict, and sigma covers both, so status degrades or turns lost and Safety
// holds); the filter re-anchors on them only when at least CONFLICT.need of them agree, span CONFLICT.span ms and
// CONFLICT.move m or CONFLICT.turn of motion (different views), and a confirming solve from another viewpoint (fix with
// confirm: a render 0.5 m away, or a relocalization) lands on the same pose. Repeated wrong solves of one view never win.
// Honest when the fixes pause (a GPU that can't keep up, a fast turn, a blank wall): the small "aided" noise needs a fix
// within AIDED.fresh ms as well as the rate; turning widens the velocity and position noise (the whoop drifts in ways the
// flow can't see, and the flow stops reading sideways); a slow video's flow counts once per frame, and less for its longer
// span; accepted fixes that keep landing further off than σ said (their normalized innovation, NIS, above 1 on average)
// scale the process noise up. With
// vision the position source (a splat fix since the last reset, or expectVision: the session runs vision localization, so
// a flight whose tracking never locks on counts from its take-off: visionActive()), a flight without one for LOC.stale ms
// (LOC.staleTurn while turning faster than LOC.turn rad/s or the flow reads loose: the turn's drift is what fixes see)
// reports "degraded" whatever σ says (safety.js slows, then holds), and without one for LOC.visionLost ms "lost": a hold
// on the flow alone drifts while σ grows slowly (the flow says it's still), and the vision side relocalizes only when lost
// (visionBack(ms): the lost clock allows the time safety.js spends turning back to where the camera last placed itself,
// at most SAFETY.backFor), or sooner when asked: askFix() (safety.js: the hold has no turn to make, or the turn back
// brought no fix) makes needFix() true until a vision fix lands, and the vision side relocalizes then as it does when lost
// ("need-fix" event). On the ground
// with vision fixes coming but refused (someone moved the drone: the fixes disagree with a pose the ground model keeps
// still), σ grows by LOC.groundDoubt m/s after 2 s, until the gate takes them.
import { Emitter, clamp, wrapAngle } from "../util.js";
import { headingFromYaw } from "../house/frames.js";

import { MASK } from "../protocol.js";

const N = 7, X = 0, Y = 1, PSI = 2, VX = 3, VY = 4, K = 5, B = 6;
const Z = 0, VZ = 1, BZ = 2, AB = 3, KZ = 4; // the height's filter
const DEG = Math.PI / 180;
const HISTORY_MS = 8000;
const FLOW_LAG = 100; // ms: the controller's flow velocity smoothing
export const LOC = { ok: 0.25, lost: 0.5, truthSigma: 0.02, history: HISTORY_MS, stale: 1500, staleTurn: 700, turn: 0.3, visionLost: 5000, zFix: 0.05, vision: ["splat", "tags"], groundDoubt: 0.05 };
// keep: ms a rejected fix counts; agree: how many joint sigmas apart two may be; drift: odometry error per metre flown
// between them; show: agreeing fixes before the pose reports the conflict.
export const CONFLICT = { keep: 6000, need: 3, span: 1000, move: 0.3, turn: 30 * Math.PI / 180, agree: 2, drift: 0.08, show: 2 };
// accel: velocity random walk (m/s per √s); it also stands for the flow's own errors (scene depth, k), so while splat
// fixes come at 3 Hz or more (k calibrated, velocity seen by the fixes) the smaller `aided` values apply (with them
// sigma stays under 0.1 m at 3-5 Hz of 5 cm fixes, and above the error). xy: 0.06 for x and y alike (an earlier version
// gave y the height's noise and the height none; sigma, the larger axis, is what grew, so missions see the same growth).
// spin, spinXY: velocity (m/s per √s) and position (m per √s) noise per rad/s of yaw rate while unaided (turning, the whoop
// drifts in ways the flow can't see: 0.65 m in a second at 1.2 rad/s in the twin); with fixes coming they see it, and more
// noise there let repeated wrong solves at one spot pass as agreeing with the odometry (test-loc's conflict case).
const Q = { xy: 0.06, xyAided: 0.02, yaw: 1 * DEG, turn: 0.04, accel: 0.85, aided: 0.3, k: 0.004, b: 0.0004, spin: 0.2, spinXY: 0 };
// The height (the simulator's whoop: thrust ~ throttle^1.6, so 9.81 x 1.6 = 15.7 m/s² per share of the hover throttle
// near hover, drag 1.4/s; a fit to seed 11's flight, its throttle a little ahead of the thrust, said 12.1 and 0.75: with
// those the model missed fast sinks, the climb rate 0.15 m/s behind and the height 0.15 m, and 0.8 kept a take-off's hop
// in the climb rate): gain (m/s² per share of the hover throttle; 9.81 x how thrust grows with throttle) and drag (1/s); q:
// the climb rate's random walk with it (m/s² per √s: gusts, the motors' lag), free: without it (no throttle known: the
// flow alone moves the height); ab: how fast what the model misses may change (m/s² per √s), ab0 its prior (m/s²: a
// hover throttle 10% off is 1.2), abMax; bz: the flow's
// climb-rate bias (m/s) and how long it lasts (s); kz: its scale's prior (1 ± kz0), drift per √s and range; z: the
// height's own random walk (m per √s); still: the climb rate on the ground (m/s); flow: the flow climb rate's noise over the
// horizontal flow's (weighed less, the browser twin's poor flow helped a little, 0.07 against 0.08 m p95 in a replay of seed
// 11, and the Node simulator's good one was missed: sparse noisy fixes yanked the height 0.36 m off), and at least flowFit x how far
// its mean climb rate between two fixes check s apart ([least, most]) is from theirs (flowZ, m/s rms, averaged over flowN
// checks: 0.1-0.15 m/s in the browser twin, a few cm/s in the Node simulator's flow); gate: χ² (1 dof) over which a
// flow climb rate is refused, fixGate: over which a fix's height counts less (its noise scaled by d²/fixGate); nis: the
// most the fixes' height consistency scales the height's noise by.
export const AZ = { gain: 15, drag: 1.4, q: 0.15, free: 1, ab: 0.15, ab0: 1.5, abMax: 4, bz: 0.15, bzTau: 1, kz0: 0.5, kz: 0.01, kzRange: [0.2, 2.5], z: 0.02, still: 0.02, flow: 1, flowFit: 7, check: [1, 2], flowN: 10, gate: 9, fixGate: 16, nis: 9 };
// ms, fixes used in it: splat tracking at 3 Hz or more (a rejection or two allowed), never 2 Hz; and the last within `fresh` ms
// (two 3 Hz fixes missed; 400 left 3 Hz tracking with a rejection unaided often enough to break σ < 0.1)
const AIDED = { window: 2500, fixes: 6, fresh: 700 };
const NIS = { n: 8, max: 4 }; // accepted fixes the average runs over; the most the process noise is scaled by
// the flow's noise: x √(ticks per video frame), x its span over `span` s (a slow video's frames compare views further
// apart: larger motions, read short)
const FLOW = { span: 0.12, tick: 1000 / 30 };
const CHI2 = { 2: 13.8, 3: 16.3, 4: 18.5 }; // 99.9%

const zeros = (r, c) => Array.from({ length: r }, () => new Array(c).fill(0));
const eye = (n, v = 1) => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? v : 0)));
const mul = (A, B) => A.map((row) => B[0].map((_, j) => row.reduce((a, v, k) => a + v * B[k][j], 0)));
const tr = (A) => A[0].map((_, j) => A.map((row) => row[j]));
const copy = (A) => A.map((r) => r.slice());

function inv(A) {
  const n = A.length, M = A.map((r, i) => [...r, ...eye(n)[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c] || 1e-12;
    for (let j = 0; j < 2 * n; j++) M[c][j] /= d;
    for (let r = 0; r < n; r++) if (r !== c && M[r][c]) for (let j = 0, f = M[r][c]; j < 2 * n; j++) M[r][j] -= f * M[c][j];
  }
  return M.map((r) => r.slice(n));
}

class Ekf {
  constructor() {
    this.s = [0, 0, 0, 0, 0, 1, 0];
    this.P = eye(N, 1e4);
    [this.P[K][K], this.P[B][B]] = [0.25 ** 2, (0.2 * DEG) ** 2];
    this.v = [0, 0, 0, 0, 1];
    this.Pv = eye(5, 0);
    [this.Pv[Z][Z], this.Pv[VZ][VZ], this.Pv[BZ][BZ], this.Pv[AB][AB], this.Pv[KZ][KZ]] = [1e4, 1, AZ.bz ** 2, AZ.ab0 ** 2, AZ.kz0 ** 2];
  }
  // qs: the process noise's scale (NIS); the yaw rate (dh / dt) widens velocity and position noise. acc: the throttle's
  // acceleration (AZ; null: not known), zq: the height's noise scale, floor: the floor under the estimate (the height
  // stays over it).
  predict({ dt, dh, still, aided, qs = 1, doubt = 0, acc = null, zq = 1, floor = null }) {
    const { s, P } = this;
    if (still) {
      s[PSI] = wrapAngle(s[PSI] - dh);
      P[PSI][PSI] += (Q.turn * dh) ** 2;
      if (doubt) for (const i of [X, Y, PSI]) P[i][i] = (Math.sqrt(P[i][i]) + doubt * dt) ** 2; // moved on the ground, maybe
      return;
    }
    s[X] += s[VX] * dt;
    s[Y] += s[VY] * dt;
    s[PSI] = wrapAngle(s[PSI] - dh - s[B] * dt);
    const F = eye(N);
    [F[X][VX], F[Y][VY], F[PSI][B]] = [dt, dt, -dt];
    const P2 = mul(mul(F, P), tr(F)), w = aided ? 0 : Math.abs(dh) / Math.max(dt, 1e-3); // fixes coming: they see the turn's drift
    const qxy = ((aided ? Q.xyAided : Q.xy) ** 2 * qs + (Q.spinXY * w) ** 2) * dt, qv = ((aided ? Q.aided : Q.accel) ** 2 * qs + (Q.spin * w) ** 2) * dt;
    const q = [qxy, qxy, Q.yaw ** 2 * dt + (Q.turn * dh) ** 2, qv, qv, Q.k ** 2 * dt, Q.b ** 2 * dt];
    q.forEach((v, i) => (P2[i][i] += v));
    this.P = P2;
    this.predictZ(dt, acc, zq, floor);
  }
  // The height: z += vz dt; vz += (acc + ab - drag vz) dt with the throttle known, else a random walk the flow steers.
  predictZ(dt, acc, zq, floor) {
    const v = this.v, phys = acc != null, drag = phys ? AZ.drag : 0, fb = Math.exp(-dt / AZ.bzTau);
    const F = eye(5);
    [F[Z][VZ], F[VZ][VZ], F[VZ][AB], F[BZ][BZ]] = [dt, 1 - drag * dt, phys ? dt : 0, fb];
    v[Z] += v[VZ] * dt;
    v[VZ] += ((phys ? acc + v[AB] : 0) - drag * v[VZ]) * dt;
    v[BZ] *= fb;
    const P2 = mul(mul(F, this.Pv), tr(F)), q = [AZ.z ** 2 * dt, (phys ? AZ.q : AZ.free) ** 2 * dt, AZ.bz ** 2 * (1 - fb * fb), phys ? AZ.ab ** 2 * dt : 0, AZ.kz ** 2 * dt];
    q.forEach((x, i) => (P2[i][i] += x * zq));
    this.Pv = P2;
    if (floor != null && v[Z] < floor) [v[Z], v[VZ]] = [floor, Math.max(0, v[VZ])]; // on the floor, not in it
  }
  // One scalar measurement of the height's filter: innovation y, row h, noise r; gate (χ²): refused over it ("soft":
  // counted less instead). -> d²
  zUpdate(y, h, r, { gate = Infinity, soft = false } = {}) {
    const v = this.v, P = this.Pv, Ph = P.map((row) => row.reduce((a, x, j) => a + x * h[j], 0)), S0 = h.reduce((a, x, i) => a + x * Ph[i], 0), d2 = (y * y) / (S0 + r);
    if (d2 > gate && !soft) return d2;
    const S = S0 + r * (soft && d2 > gate ? d2 / gate : 1), k = Ph.map((x) => x / S);
    for (let i = 0; i < 5; i++) v[i] += k[i] * y;
    this.Pv = P.map((row, i) => row.map((x, j) => x - k[i] * Ph[j]));
    for (let i = 0; i < 5; i++) for (let j = 0; j < i; j++) this.Pv[i][j] = this.Pv[j][i] = (this.Pv[i][j] + this.Pv[j][i]) / 2;
    [v[AB], v[KZ]] = [clamp(v[AB], -AZ.abMax, AZ.abMax), clamp(v[KZ], ...AZ.kzRange)];
    return d2;
  }
  // The flow's climb rate (m/s) = kz vz + bz.
  climb(vz, sigma, zq = 1) {
    const v = this.v;
    return this.zUpdate(vz - (v[KZ] * v[VZ] + v[BZ]), [0, v[KZ], 1, 0, v[VZ]], sigma * sigma * zq, { gate: AZ.gate });
  }
  // A height fix (m, σ).
  fixZ(z, sigma) {
    return this.zUpdate(z - this.v[Z], [1, 0, 0, 0, 0], sigma * sigma, { gate: AZ.fixGate, soft: true });
  }
  // Generic update: innovation y = z - h(s), H the Jacobian rows, R the noise. angle: indices of y that are angles.
  update(y, H, R, { angle = [], gate = Infinity } = {}) {
    angle.forEach((i) => (y[i] = wrapAngle(y[i])));
    const PHt = mul(this.P, tr(H)), S = mul(H, PHt).map((r, i) => r.map((v, j) => v + R[i][j])), Si = inv(S);
    const d2 = y.reduce((a, yi, i) => a + yi * Si[i].reduce((b, v, j) => b + v * y[j], 0), 0);
    if (d2 > gate) return d2;
    const Kg = mul(PHt, Si);
    for (let i = 0; i < N; i++) this.s[i] += Kg[i].reduce((a, v, j) => a + v * y[j], 0);
    this.s[PSI] = wrapAngle(this.s[PSI]);
    this.s[K] = clamp(this.s[K], 0.3, 3);
    const IKH = eye(N).map((r, i) => r.map((v, j) => v - Kg[i].reduce((a, kv, m) => a + kv * H[m][j], 0)));
    this.P = mul(mul(IKH, this.P), tr(IKH)).map((r, i) => r.map((v, j) => v + Kg[i].reduce((a, kv, m) => a + kv * R[m][m] * Kg[j][m], 0)));
    return d2;
  }
  // Flow velocity (forward, right in the body frame) = R(yaw)^T v / k.
  flow([f, r], sigma) {
    const { s } = this, c = Math.cos(s[PSI]), sn = Math.sin(s[PSI]), k = s[K];
    const fh = c * s[VX] + sn * s[VY], rh = sn * s[VX] - c * s[VY];
    const H = zeros(2, N);
    H[0][PSI] = (-sn * s[VX] + c * s[VY]) / k;
    H[1][PSI] = (c * s[VX] + sn * s[VY]) / k;
    [H[0][VX], H[0][VY], H[1][VX], H[1][VY]] = [c / k, sn / k, sn / k, -c / k];
    [H[0][K], H[1][K]] = [-fh / k ** 2, -rh / k ** 2];
    this.update([f - fh / k, r - rh / k], H, [[sigma[0] ** 2, 0], [0, sigma[1] ** 2]]);
  }
  still() {
    const H = zeros(2, N);
    [H[0][VX], H[1][VY]] = [1, 1];
    this.update([-this.s[VX], -this.s[VY]], H, [[0.02 ** 2, 0], [0, 0.02 ** 2]]);
    this.zUpdate(-this.v[VZ], [0, 1, 0, 0, 0], AZ.still ** 2);
  }
  sigma() {
    const a = this.P[X][X], c = this.P[Y][Y], b = this.P[X][Y];
    return Math.sqrt(Math.max(0, (a + c) / 2 + Math.sqrt(((a - c) / 2) ** 2 + b * b)));
  }
  snapshot() {
    return { s: this.s.slice(), P: copy(this.P), v: this.v.slice(), Pv: copy(this.Pv) };
  }
  restore({ s, P, v, Pv }) {
    this.s = s.slice();
    this.P = copy(P);
    this.v = v.slice();
    this.Pv = copy(Pv);
  }
}

export class Localizer extends Emitter {
  constructor({ ctl, map = null, house = null, settings = null }) {
    super();
    Object.assign(this, { ctl, map, house, settings });
    this.source = settings?.get?.("locSource") || "fused";
    this.truthFn = null;
    this.kf = new Ekf();
    this.known = false; // reset() at least once
    this.lost = false; // once σ passes LOC.lost only an absolute fix or a reset brings the pose back
    this.flown = false;
    this.ticks = []; // { t, dt, dh, still, acc, floor, flows: [], fixes: [], pre: ekf snapshot before this tick }
    this.trail = []; // reported poses { t, x, y, z, yaw, sigma }
    this.fixes = { used: 0, rejected: 0, last: null, lastUsed: null, times: [], conflicts: 0, anchored: 0 };
    this.conflict = null; // { since, cands: [rejected fixes with the filter's estimate at their time], view }
    this.vision = null; // { since, last, yaw } (performance.now ms; H yaw there) of the vision fixes used since the last reset
    this.expectVision = false; // the session runs vision localization (set by it): a flight counts as vision-led from its take-off
    this.airborneAt = null; // performance.now ms of this flight's take-off
    this.turnedAt = -Infinity; // last turn faster than LOC.turn (or a loose flow)
    this.backMs = 0; // ms safety.js spent turning back to the last fix's view since it (visionBack())
    this.fixAsked = null; // performance.now ms safety.js asked for a relocalization (askFix()), until a vision fix lands
    this.nis = 1; // accepted fixes' normalized innovation, averaged (see NIS)
    this.zNis = 1; // the same for their heights (AZ.nis)
    this.zFixes = []; // recent fixes' heights { t (frame ms), z }, against which the flow's climb rate is checked (flowCheck())
    this.flowZ = null; // how far the flow's climb rate (its scale and bias taken off) is from the fixes' heights, m/s rms
    this.lastHeading = ctl.est.heading;
    this.unsub = ctl.on("tick", ({ dt, now }) => this.step(dt, now));
  }

  dispose() {
    this.unsub?.();
  }

  setMap(map, house = this.house) {
    Object.assign(this, { map, house });
  }

  setSource(source) {
    if (!["truth", "odometry", "fused"].includes(source)) throw new Error(`unknown localization source "${source}"`);
    this.source = source;
  }

  setTruth(fn) {
    this.truthFn = fn;
  }

  // A known pose: the home pad before take-off, or a relocalization (in flight the velocity starts from the flow's).
  // k, b and the height's bz, ab and kz (learned) carry over.
  reset({ x, y, yaw, z, sigma = 0.05, yawSigma = 3 * DEG }) {
    const kf = this.kf, floor = this.map?.floorAt(x, y) ?? 0, e = this.ctl.est, flying = this.ctl.isFlying(), moving = flying && e.flowQ > 0.2;
    const k = kf.s[K], c = Math.cos(yaw), sn = Math.sin(yaw), v = moving ? [k * (c * e.vx + sn * e.vy), k * (sn * e.vx - c * e.vy)] : [0, 0];
    kf.s = [x, y, yaw, v[0], v[1], kf.s[K], kf.s[B]];
    const P = eye(N), sv = moving ? 0.2 : 0.05;
    [P[X][X], P[Y][Y], P[PSI][PSI], P[VX][VX], P[VY][VY], P[K][K], P[B][B]] = [sigma ** 2, sigma ** 2, yawSigma ** 2, sv ** 2, sv ** 2, kf.P[K][K], kf.P[B][B]];
    kf.P = P;
    kf.v = [z ?? floor + (flying ? 1 : 0), 0, kf.v[BZ], kf.v[AB], kf.v[KZ]];
    const Pv = eye(5, 0);
    [Pv[Z][Z], Pv[VZ][VZ], Pv[BZ][BZ], Pv[AB][AB], Pv[KZ][KZ]] = [(z == null ? 0.3 : sigma) ** 2, (flying ? 0.3 : 0.05) ** 2, kf.Pv[BZ][BZ], kf.Pv[AB][AB], kf.Pv[KZ][KZ]];
    kf.Pv = Pv;
    Object.assign(this, { known: true, lost: false, flown: false, ticks: [], conflict: null, vision: null, backMs: 0, fixAsked: null, lastHeading: this.ctl.est.heading });
    this.emit("pose", this.pose());
  }

  // The position is unknown again (e.g. switching from the simulator to the real drone): the pad or a fix brings it back.
  // The ticks stay (without their fixes), so a relocalization from a frame taken before now is carried to now.
  forget() {
    for (const t of this.ticks) t.fixes = [];
    Object.assign(this, { known: false, lost: false, conflict: null, vision: null, backMs: 0, fixAsked: null });
    this.emit("pose", this.pose());
  }

  // Absolute pose at time t (ms, performance.now clock; default now): { x, y, z?, yaw?, sigma, yawSigma?, source, reloc?,
  // confirm? } plus anything to show with it (splat fixes: inliers, rms, ms). reloc: replace the estimate (a relocalization)
  // instead of passing the Mahalanobis gate. confirm: a solve from another viewpoint; if it agrees with a ready conflict
  // (see above) the filter re-anchors on it, else it is an ordinary fix. Ignored in "odometry"; in "truth" it is only
  // counted. Returns whether it was used; fixes.last.why says why not ("source", "stale": older than the history, "gate").
  fix({ x, y, z, yaw, sigma = 0.1, yawSigma = 5 * DEG, t = performance.now(), source = "fix", reloc = false, confirm = false, ...info }) {
    const last = (this.fixes.last = { x, y, z, yaw, sigma, yawSigma, t, source, ...info, at: performance.now(), used: false, why: null });
    if (this.source !== "fused") return (last.why = "source"), false;
    const ticks = this.ticks;
    if (ticks.length && ticks[0].t - t > 50) return this.reject(last, "stale"); // older than the history: can't be carried to now
    if (!this.known) {
      // the pose at t, carried through the ticks since (a relocalization is a few hundred ms late, the drone turning)
      const i = this.tickAt(t);
      this.reset({ x, y, yaw: yaw ?? 0, z, sigma, yawSigma: yaw == null ? Math.PI : yawSigma });
      if (i >= 0 && i < ticks.length - 1) {
        this.ticks = ticks.slice(i + 1);
        this.replay(0, null);
      }
      return this.used(last);
    }
    const i = Math.max(0, this.tickAt(t));
    // The gate looks at x, y and yaw; a height that comes with them is from the same solution: the height's filter takes
    // it at the fix's σ (at least LOC.zFix), softly gated (a far one counts less, never nothing: the height must not run
    // away from the fixes), with the noise the fixes' height consistency says (zScale).
    const rows = [[X, x, sigma], [Y, y, sigma], ...(yaw != null ? [[PSI, yaw, yawSigma]] : [])], zs = z != null ? [z, Math.max(sigma, LOC.zFix) * Math.sqrt(this.zScale)] : null;
    const now = this.kf.snapshot();
    if (ticks.length) {
      this.kf.restore(ticks[i].pre);
      this.advance(ticks[i]);
    }
    const s = this.kf.s, est = { x: s[X], y: s[Y], yaw: s[PSI] }, me = { x, y, yaw, sigma, yawSigma, t, est };
    // Lost (and no second hypothesis), a relocalization, or a confirmed conflict: take this one.
    const anchor = reloc || (!this.conflict && this.status(this.kf.sigma()) === "lost") || (confirm && this.confirms(me));
    const f = { rows, z: zs, reloc: anchor }, gate = anchor ? Infinity : CHI2[rows.length], d2 = this.apply(f, gate);
    if (d2 > gate) {
      this.fixes.lastReject = { at: last.at, d2: +d2.toFixed(1), dx: +(x - s[X]).toFixed(3), dy: +(y - s[Y]).toFixed(3), dyaw: yaw != null ? +wrapAngle(yaw - s[PSI]).toFixed(3) : null, sigma: +this.kf.sigma().toFixed(3) };
      this.kf.restore(now);
      if (yaw != null) this.disagree(me);
      return this.reject(last, "gate");
    }
    if (!anchor) this.nis += (Math.min(20, d2 / rows.length) - this.nis) / NIS.n;
    if (!anchor && zs) this.zNis += (Math.min(20, this.kf.zd2 ?? 1) - this.zNis) / NIS.n;
    if (zs && !reloc) this.flowCheck(t, z, zs[1]);
    if (ticks.length) {
      ticks[i].fixes.push(f);
      this.replay(i + 1, null);
    }
    if (anchor && this.conflict && !reloc) this.fixes.anchored++;
    this.conflict = null;
    this.lost = false;
    return this.used(last);
  }

  // The flow's climb rate against the fixes' heights: its mean (scale and bias taken off) over the ticks between this fix and
  // one AZ.check s before it, against their difference over that time, less what the two fixes' own noise (σ, m) explains
  // -> flowZ (m/s rms), the floor under the flow climb rate's noise (AZ.flowFit x it): a flow that disagrees with vision
  // counts for less, one that agrees as much as its quality says.
  flowCheck(t, z, sigma) {
    const zf = this.zFixes.filter((f) => t - f.t < 1000 * AZ.check[1] + 500), v = this.kf.v;
    zf.push({ t, z, sigma });
    this.zFixes = zf;
    const o = zf.find((f) => t - f.t >= 1000 * AZ.check[0] && t - f.t <= 1000 * AZ.check[1]);
    if (!o) return;
    let sum = 0, n = 0;
    for (const k of this.ticks) if (k.t > o.t && k.t <= t && !k.still) for (const m of k.flows) if (m.vz != null) (sum += (m.vz - v[BZ]) / v[KZ]), n++;
    if (n < 10) return;
    const T = (t - o.t) / 1000, e2 = Math.max(0, (sum / n - (z - o.z) / T) ** 2 - (sigma ** 2 + o.sigma ** 2) / T ** 2); // (the mean climb rate: a slow video's frames don't land on every tick)
    this.flowZ = Math.sqrt((this.flowZ ?? Math.sqrt(e2)) ** 2 + (e2 - (this.flowZ ?? Math.sqrt(e2)) ** 2) / AZ.flowN);
  }

  // ---------------------------------------------------------------- conflicts

  // A rejected fix as a candidate pose now (or at time `at`): moved as the filter's estimate moved since it was taken.
  carry(c, at = null) {
    const e = at == null ? { x: this.kf.s[X], y: this.kf.s[Y], yaw: this.kf.s[PSI] } : this.poseAt(at), r = wrapAngle(c.yaw - c.est.yaw), dx = e.x - c.est.x, dy = e.y - c.est.y;
    return { x: c.x + Math.cos(r) * dx - Math.sin(r) * dy, y: c.y + Math.sin(r) * dx + Math.cos(r) * dy, yaw: wrapAngle(c.yaw + e.yaw - c.est.yaw), sigma: c.sigma + CONFLICT.drift * Math.hypot(dx, dy), yawSigma: c.yawSigma };
  }
  agree(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y) <= CONFLICT.agree * Math.hypot(a.sigma, b.sigma) && Math.abs(wrapAngle(a.yaw - b.yaw)) <= CONFLICT.agree * Math.hypot(a.yawSigma, b.yawSigma) + 2 * DEG;
  }

  disagree(me) {
    const c = (this.conflict ??= { since: me.t, cands: [], view: null });
    c.cands.push(me);
    this.review();
  }

  // The largest group of rejected fixes that agree with each other (carried to now) -> conflict.view: { x, y, yaw (the
  // second hypothesis now), n, apart (m from the estimate), ready (enough of them, over enough time and motion) } or null.
  review() {
    const c = this.conflict;
    if (!c) return;
    const tNow = this.ticks.at(-1)?.t ?? performance.now();
    c.cands = c.cands.filter((k) => tNow - k.t < CONFLICT.keep);
    if (!c.cands.length) return void (this.conflict = null);
    const now = c.cands.map((k) => this.carry(k));
    let best = [];
    now.forEach((a, i) => {
      const g = now.flatMap((b, j) => (this.agree(a, b) ? [j] : []));
      if (g.length > best.length) best = g;
    });
    const ks = best.map((j) => c.cands[j]), ps = best.map((j) => now[j]), n = ks.length, mean = (k) => ps.reduce((a, p) => a + p[k], 0) / n;
    const yaw = Math.atan2(ps.reduce((a, p) => a + Math.sin(p.yaw), 0), ps.reduce((a, p) => a + Math.cos(p.yaw), 0)), x = mean("x"), y = mean("y");
    let moved = 0, turned = 0;
    for (const a of ks) for (const b of ks) [moved, turned] = [Math.max(moved, Math.hypot(a.est.x - b.est.x, a.est.y - b.est.y)), Math.max(turned, Math.abs(wrapAngle(a.est.yaw - b.est.yaw)))];
    const span = Math.max(...ks.map((k) => k.t)) - Math.min(...ks.map((k) => k.t)), was = c.view;
    c.view = n < CONFLICT.show ? null : { x, y, yaw, n, sigma: Math.min(...ps.map((p) => p.sigma)), apart: Math.hypot(x - this.kf.s[X], y - this.kf.s[Y]), span, moved, turned,
      ready: n >= CONFLICT.need && span >= CONFLICT.span && (moved >= CONFLICT.move || turned >= CONFLICT.turn) };
    if (c.view && !was) {
      this.fixes.conflicts++;
      this.emit("conflict", { estimate: { x: this.kf.s[X], y: this.kf.s[Y], yaw: this.kf.s[PSI] }, candidate: { x, y, yaw }, ...c.view });
    }
  }

  // A confirming fix (another viewpoint) agrees with a ready conflict at its own time.
  confirms(me) {
    const v = this.conflict?.view;
    if (!v?.ready) return false;
    const ks = this.conflict.cands.map((k) => this.carry(k, me.t)).filter((p) => Math.hypot(p.x - me.x, p.y - me.y) < 1);
    return ks.filter((p) => this.agree(p, me)).length >= CONFLICT.need;
  }

  used(last) {
    const fx = this.fixes;
    fx.used++;
    last.used = true;
    fx.lastUsed = last;
    fx.times.push(last.at);
    while (fx.times.length && fx.times[0] < last.at - 5000) fx.times.shift();
    if (LOC.vision.includes(last.source)) [this.vision, this.backMs, this.fixAsked] = [{ since: this.vision?.since ?? last.at, last: last.at, yaw: last.yaw ?? this.kf.s[PSI] }, 0, null];
    this.emit("fix", last);
    return true;
  }

  // Vision (splat PnP, tags) is the position source: fused, with a vision fix used since the last reset (or expected).
  visionActive() {
    return this.source === "fused" && (!!this.vision || this.expectVision);
  }
  // When the vision clock last started (performance.now ms): the last vision fix, or the take-off when expected and none
  // came since. null: never.
  visionSince() {
    const t = Math.max(this.vision?.last ?? -Infinity, this.expectVision && this.ctl.isFlying() ? this.airborneAt ?? -Infinity : -Infinity);
    return Number.isFinite(t) ? t : null;
  }
  // safety.js has turned back toward the view of the last fix for `ms`: the lost clock allows that long more.
  visionBack(ms) {
    this.backMs = Math.max(this.backMs, ms);
  }
  // A relocalization now, not at "lost" (safety.js: tracking won't come back by itself): needFix() until a vision fix lands.
  askFix(now = performance.now()) {
    if (this.fixAsked != null || !this.visionActive()) return;
    this.fixAsked = now;
    this.emit("need-fix", { t: now });
  }
  needFix() {
    return this.fixAsked != null;
  }
  // The process noise's scale from the fixes' consistency (1: they land within σ as they should).
  get qScale() {
    return clamp(this.nis, 1, NIS.max);
  }
  // The height's noise scale (process and fixes alike) from their heights' consistency.
  get zScale() {
    return clamp(this.zNis, 1, AZ.nis);
  }

  // For the HUD and safety.js: { age (ms since the last fix used; Infinity: none), rate (fixes used per second over 5 s),
  // source, inliers, sigma (of that fix), used, rejected, conflict (pose().conflict), vision (visionActive()), visionAge (ms
  // since the last vision fix), nis (the fixes' normalized innovation, about 1 when σ is honest) }.
  fixQuality(now = performance.now()) {
    const fx = this.fixes, u = fx.lastUsed;
    return { age: u ? now - u.at : Infinity, rate: fx.times.filter((t) => t > now - 5000).length / 5, source: u?.source ?? null, inliers: u?.inliers ?? null,
      sigma: u?.sigma ?? null, used: fx.used, rejected: fx.rejected, conflict: this.conflict?.view ?? null, vision: this.visionActive(),
      visionAge: now - (this.visionSince() ?? -Infinity), nis: +this.nis.toFixed(2) };
  }

  apply({ rows, z = null, reloc }, gate = Infinity) {
    const kf = this.kf;
    if (reloc) {
      for (const [i] of rows) for (let j = 0; j < N; j++) kf.P[i][j] = kf.P[j][i] = i === j ? Math.max(kf.P[i][i], i === PSI ? 0.5 : 1) : 0;
      if (z) for (let j = 0; j < 5; j++) kf.Pv[Z][j] = kf.Pv[j][Z] = j === Z ? Math.max(kf.Pv[Z][Z], 1) : 0;
    }
    const H = rows.map(([i]) => Object.assign(new Array(N).fill(0), { [i]: 1 })), R = rows.map(([, , sg], i) => rows.map((_, j) => (i === j ? sg * sg : 0)));
    const d2 = kf.update(rows.map(([i, v]) => v - kf.s[i]), H, R, { angle: rows.flatMap(([i], j) => (i === PSI ? [j] : [])), gate });
    if (d2 <= gate && z) kf.zd2 = kf.fixZ(z[0], z[1]);
    return d2;
  }

  // The latest tick at or before t (-1: none).
  tickAt(t) {
    let i = this.ticks.length - 1;
    while (i >= 0 && this.ticks[i].t > t) i--;
    return i;
  }

  // Ticks i.. again with their measurements, from the state before tick i (or from the current state: from = null).
  replay(i, from = this.ticks[i].pre) {
    if (from) this.kf.restore(from);
    for (let j = i; j < this.ticks.length; j++) {
      this.ticks[j].pre = this.kf.snapshot();
      this.advance(this.ticks[j]);
    }
  }

  reject(last, why) {
    this.fixes.rejected++;
    last.why = why;
    return false;
  }

  // One controller tick: heading and height change, and what the flow says about velocity. The flow velocity describes
  // the drone FLOW_LAG + videoDelay ms ago (late video, the controller's smoothing), so it goes on that tick.
  step(dt, now) {
    const e = this.ctl.est, flying = this.ctl.isFlying();
    const dh = e.heading - this.lastHeading;
    this.lastHeading = e.heading;
    if (flying && this.source !== "truth") this.flown = true;
    this.airborneAt = flying ? this.airborneAt ?? now : null;
    if (flying && (Math.abs(dh) / Math.max(dt, 1e-3) > LOC.turn || e.rotating)) this.turnedAt = now;
    const fx = this.fixes, aided = now - (fx.lastUsed?.at ?? -Infinity) < AIDED.fresh && fx.times.reduce((n, t) => n + (t > now - AIDED.window), 0) >= AIDED.fixes;
    // on the ground, vision fixes coming but refused for 2 s: the drone may have been moved (picked up, nudged, bumped)
    const doubt = !flying && this.visionActive() && fx.last?.why === "gate" && now - fx.last.at < 1000 && now - (fx.lastUsed?.at ?? -Infinity) > 2000 ? LOC.groundDoubt : 0;
    const tick = { t: now, dt, dh, still: !flying, aided, doubt, qs: this.qScale, zq: this.zScale, acc: flying ? this.thrust() : null, floor: this.map?.floorAt?.(this.kf.s[X], this.kf.s[Y]) ?? null,
      flows: [], fixes: [], pre: this.kf.snapshot() };
    this.advance(tick);
    this.ticks.push(tick);
    while (this.ticks.length && this.ticks[0].t < now - HISTORY_MS) this.ticks.shift();
    const per = this.ctl.perception, f = per?.latest?.flow, i = this.tickAt(now - FLOW_LAG - this.ctl.videoDelay);
    if (flying && f && now - f.t < 250 && e.flowQ > 0.2 && i >= 0) {
      // While spinning the controller stops reading sideways drift (forward still comes from the image's zoom). A slow
      // video's frame counts once, across the ticks it is read on.
      const reps = Math.max(1, 1000 / clamp(per.fps || 30, 1, 30) / FLOW.tick);
      const k = Math.sqrt(reps) * Math.max(1, (f.span ?? 0.1) / FLOW.span), zs = Math.max(((2 * AZ.flow * (0.1 + 0.3 * Math.abs(e.vz))) / Math.max(0.3, e.flowQ)) * k, AZ.flowFit * (this.flowZ ?? 0));
      this.ticks[i].flows.push({ v: [e.vx, e.vy], vz: e.vz, zs, q: e.flowQ, loose: e.rotating, k });
      this.replay(i);
    }
    if (this.source === "truth" && this.truthFn) this.pin(this.truthFn());
    if (this.known && !(this.kf.sigma() < LOC.lost) && this.source !== "truth") this.lost = true;
    if (this.visionActive() && flying && now - (this.visionSince() ?? -Infinity) > LOC.visionLost + this.backMs) this.lost = true;
    if (this.conflict) this.review();
    const p = this.pose();
    this.trail.push({ t: now, x: p.x, y: p.y, z: p.z, yaw: p.yaw, sigma: p.sigma });
    while (this.trail.length && this.trail[0].t < now - HISTORY_MS) this.trail.shift();
    this.emit("pose", p);
  }

  // The flow velocity is smoothed and its errors (scene depth) persist for several frames: its noise is set for that.
  advance(tick) {
    const kf = this.kf;
    kf.predict(tick);
    if (tick.still) kf.still();
    for (const m of tick.flows) {
      kf.flow(m.v, m.v.map((v, i) => ((m.loose && i ? 0.6 : 0) + (2 * (0.1 + 0.3 * Math.abs(v))) / Math.max(0.3, m.q)) * (m.k ?? 1)));
      if (m.vz != null) kf.climb(m.vz, m.zs, tick.zq ?? 1);
    }
    for (const f of tick.fixes) this.apply(f);
  }

  // Keep the filter on the truth (sim), so switching sources does not jump.
  pin(t) {
    if (!t) return;
    const kf = this.kf;
    [kf.s[X], kf.s[Y], kf.s[PSI], kf.v[Z]] = [t.x, t.y, t.yaw, t.z];
    for (const i of [X, Y, PSI])
      for (let j = 0; j < N; j++) kf.P[i][j] = kf.P[j][i] = i === j ? (i === PSI ? DEG : LOC.truthSigma) ** 2 : 0;
    for (let j = 0; j < 5; j++) kf.Pv[Z][j] = kf.Pv[j][Z] = j === Z ? LOC.truthSigma ** 2 : 0;
    this.known = true;
    this.lost = false;
    this.conflict = null;
  }

  status(sigma) {
    if (!this.known || this.lost || !(sigma < LOC.lost)) return "lost";
    return sigma < LOC.ok ? "ok" : "degraded";
  }

  // { x, y, z, yaw, sigma, zSigma, status, source, room, v: [vx, vy], conflict?, stale? }. In a conflict the pose stays the
  // filter's, its sigma covers the second hypothesis too (half the gap plus that one's sigma) and `conflict` is
  // { x, y, yaw, n, apart, ready } (see review()). stale: ms since the last vision fix, when vision is the source and the
  // drone has flown LOC.stale ms without one (then "ok" reads "degraded").
  pose() {
    const t = this.source === "truth" && this.truthFn?.();
    const s = this.kf.s, v = !t && this.conflict?.view, sigma = Math.max(this.kf.sigma(), v ? v.apart / 2 + v.sigma : 0);
    const now = performance.now(), age = !t && this.visionActive() && this.ctl.isFlying() ? now - (this.visionSince() ?? -Infinity) : 0;
    const stale = age > (now - this.turnedAt < 500 ? LOC.staleTurn : LOC.stale), st = this.status(sigma);
    const p = t
      ? { x: t.x, y: t.y, z: t.z, yaw: t.yaw, sigma: LOC.truthSigma, zSigma: LOC.truthSigma, status: "ok" }
      : { x: s[X], y: s[Y], z: this.kf.v[Z], yaw: s[PSI], sigma, zSigma: Math.sqrt(this.kf.Pv[Z][Z]), status: stale && st === "ok" ? "degraded" : st, ...(stale && { stale: Math.round(age) }),
        ...(v && { conflict: { x: v.x, y: v.y, yaw: v.yaw, n: v.n, apart: v.apart, ready: v.ready } }) };
    return { ...p, source: t ? "truth" : this.source, room: this.map?.roomAt(p.x, p.y)?.id ?? null, v: [s[VX], s[VY]] };
  }

  // The reported pose at an earlier time (ms), e.g. when a late video frame was taken.
  poseAt(t) {
    const h = this.trail;
    if (!h.length || t >= h.at(-1).t) return this.pose();
    if (t <= h[0].t) return { ...h[0] };
    let i = h.length - 1;
    while (i > 0 && h[i - 1].t > t) i--;
    const a = h[i - 1], b = h[i], u = (t - a.t) / (b.t - a.t || 1);
    return { t, x: a.x + (b.x - a.x) * u, y: a.y + (b.y - a.y) * u, z: a.z + (b.z - a.z) * u, yaw: a.yaw + wrapAngle(b.yaw - a.yaw) * u, sigma: b.sigma };
  }

  // The smallest σ reported over the last `ms`: what the fixes keep bringing it back to (between fixes it saws up), for
  // planning (a plan sized for a peak between two fixes found no way through a doorway).
  sigmaFloor(ms = 2000) {
    const h = this.trail, end = h.at(-1)?.t;
    let s = Infinity;
    for (let i = h.length - 1; i >= 0 && end - h[i].t <= ms; i--) s = Math.min(s, h[i].sigma);
    return Number.isFinite(s) ? s : this.pose().sigma;
  }

  // Velocity of the reported pose over the last `ms` (H frame, m/s): the truth's in the simulator, else the filter's.
  velocity(ms = 300) {
    const h = this.trail, b = h.at(-1);
    if (!b) return [0, 0];
    let i = h.length - 1;
    while (i > 0 && b.t - h[i - 1].t <= ms) i--;
    const a = h[i], dt = (b.t - a.t) / 1000;
    return dt > 0.05 ? [(b.x - a.x) / dt, (b.y - a.y) / dt] : [0, 0];
  }

  // The height and climb rate: { z (m, H), vz (m/s), zSigma, vzSigma }: the simulator's truth (vz over its last 150 ms) in
  // "truth", else the height's filter.
  height(ms = 150) {
    if (this.source === "truth" && this.truthFn) {
      const h = this.trail, b = h.at(-1), t = this.truthFn();
      let i = h.length - 1;
      while (i > 0 && b.t - h[i - 1].t <= ms) i--;
      const dt = b ? (b.t - h[i].t) / 1000 : 0;
      return { z: t.z, vz: dt > 0.05 ? (b.z - h[i].z) / dt : 0, zSigma: LOC.truthSigma, vzSigma: 0.05 };
    }
    const v = this.kf.v, P = this.kf.Pv;
    return { z: v[Z], vz: v[VZ], zSigma: Math.sqrt(P[Z][Z]), vzSigma: Math.sqrt(P[VZ][VZ]) };
  }

  // The acceleration (m/s²) the throttle reaching the motors gives over the hover throttle (AZ), or null when the
  // controller doesn't say what it sends or its hover estimate (the pilot's throttle in co-pilot, the controller's in full
  // auto), tilted by the attitude commanded.
  thrust() {
    const c = this.ctl, thr = c.mask & MASK.thr ? c.out?.thr : c.tel?.sticks?.thr, hover = c.hover?.estimate?.(c.tel?.vbat), e = c.est;
    if (!(thr >= 0) || !(hover > 0)) return null;
    return AZ.gain * ((thr * Math.cos(e.pitchAngle ?? 0) * Math.cos(e.rollAngle ?? 0)) / hover - 1);
  }

  // The controller heading (clockwise, continuous) that points the drone along H yaw `yaw`, nearest the current one.
  headingFor(yaw, pose = this.pose()) {
    const h = this.ctl.est.heading;
    return headingFromYaw(yaw, { yaw0: pose.yaw, heading0: h }, h);
  }

  get scale() {
    return this.kf.s[K];
  }
}
