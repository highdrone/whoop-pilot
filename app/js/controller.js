// Flight controller: runs at ~30 Hz, turns the active behavior's setpoints ("fly forward at 0.3 m/s,
// turn toward this heading") into stick commands, and sends them to the radio (real or simulated).
//
// What it knows about the quad comes from: radio telemetry (battery, heading, the pilot's sticks,
// the AI switch) and image motion from the FPV camera. There is no GPS, no rangefinder and usually
// no barometer on a whoop, so velocities are estimates with an unknown scale; the gains are gentle.
import { Emitter, DEG, clamp, wrapAngle, actualRate, actualRateInverse } from "./util.js";
import { MASK } from "./protocol.js";
import { Hold } from "./behaviors.js";

const TICK_MS = 33;
const NOMINAL_DEPTH = 2.5; // m, typical distance to what the camera sees indoors
// The height hold on the localizer's height and climb rate (safety.height(): carried to now by the throttle's physics, so
// not as late as the video): m/s of climb per m off the height held, throttle per m/s off the climb rate (on the flow's own
// 1.0 and 0.12, eased for the video's delay). A gentler hold let gusts move a scan 0.19 m off its height in the simulator.
export const HEIGHT = { kz: 1.5, kv: 0.2 };

export class HoverModel {
  constructor(key) {
    this.key = `whoopPilot.hover.${key}`;
    this.k = 1.75; // hover throttle x pack voltage
    this.n = 0;
    try {
      const v = JSON.parse(localStorage.getItem(this.key));
      if (v && v.k > 0.8 && v.k < 3.2) Object.assign(this, { k: v.k, n: v.n || 0 });
    } catch {}
  }
  estimate(vbat) {
    const v = vbat > 2.8 && vbat < 4.6 ? vbat : 3.9;
    return clamp(this.k / v, 0.15, 0.8);
  }
  // Adjust the model by a throttle offset (at this voltage).
  shift(dThr, vbat) {
    this.learn(this.estimate(vbat) + dThr, vbat, 1);
  }
  learn(thr, vbat, weight) {
    if (!(vbat > 2.8 && vbat < 4.6)) return;
    this.k = clamp(this.k + (thr * vbat - this.k) * weight, 0.8, 3.2);
    if (++this.n % 120 === 0) {
      try {
        localStorage.setItem(this.key, JSON.stringify({ k: this.k, n: this.n }));
      } catch {}
    }
  }
}

// Learns how the quad's own rotation shows up as image motion (y ~ k . x), so the rest can be read as
// drift. Depends on the real camera's field of view and tilt and on the Betaflight rates, so it is
// calibrated in flight rather than assumed. Recursive least squares with forgetting.
class RotationModel {
  constructor(k0, bounds) {
    this.k = [...k0];
    this.k0 = [...k0];
    this.bounds = bounds;
    const n = k0.length;
    this.P = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 0.02 : 0)));
  }
  predict(x) {
    return x.reduce((a, xi, i) => a + xi * this.k[i], 0);
  }
  update(x, y, lambda = 0.995) {
    const n = x.length;
    const Px = this.P.map((row) => row.reduce((a, v, j) => a + v * x[j], 0));
    const denom = lambda + x.reduce((a, xi, i) => a + xi * Px[i], 0);
    const gain = Px.map((v) => v / denom);
    const err = y - this.predict(x);
    for (let i = 0; i < n; i++) {
      const [lo, hi] = this.bounds[i];
      this.k[i] = clamp(this.k[i] + gain[i] * err, lo, hi);
    }
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) this.P[i][j] = (this.P[i][j] - gain[i] * Px[j]) / lambda;
  }
}

export class FlightController extends Emitter {
  constructor({ settings, perception }) {
    super();
    this.settings = settings;
    this.perception = perception;
    this.radio = null;
    this.tel = null;
    this.telT = 0;
    this.seq = 0;
    this.est = { heading: 0, vx: 0, vy: 0, vz: 0, px: 0, py: 0, z: 0, flowQ: 0, ttc: Infinity, pitchAngle: 0, rollAngle: 0, rotating: false };
    this.out = { roll: 0, pitch: 0, yaw: 0, thr: 0 };
    this.mask = 0;
    this.behavior = null;
    this.hold = new Hold();
    this.airborne = false; // full-auto: set by take-off, cleared by landing or the pilot taking over
    this.posRef = null;
    this.zRef = null;
    this.zLoc = false; // the height hold follows the localizer's height (safety.height()), not the flow's
    this.iThr = this.iVx = this.iVy = 0;
    this.headingHist = []; // { t, h (heading estimate), c (integrated commanded turn), p (pitch angle) }
    this.att = { raw: null, cont: 0, seen: false, changedAt: 0 };
    this.headingFix = 0;
    this.rateScale = 1; // actual turn rate / what the configured Betaflight rates predict (learned)
    this.attLog = [];
    this.rotX = null; // image x-motion per (yaw rate, roll rate)
    this.rotY = null; // image y-motion per pitch rate
    this.lastTick = performance.now();
    this.lastRequest = {};
    this.stableSince = 0;
    this.status = "No radio";
    this.hover = new HoverModel("sim");
    this.safety = null; // safety.js: filters every setpoint and can take over (hold, land in place)
    // Co-pilot with the safety layer attached: after a behavior ends in the air the AI keeps holding position (as full auto
    // does) until the pilot takes a stick (the radio script hands it over), the AI switch goes off, or a new command runs.
    this.pilotHold = false;
    this.handHold = Object.assign(new Hold(), { label: "Holding position (move a stick to take over)" });
  }

  attach(radio, mode) {
    this.unsub?.();
    this.radio = radio;
    this.tel = null;
    this.hover = new HoverModel(mode);
    this.unsub = radio.on("telemetry", (t) => this.onTelemetry(t));
    this.abort("Switched video/radio source");
    this.airborne = false;
  }

  start() {
    clearInterval(this.timer);
    this.timer = setInterval(() => this.tick(), TICK_MS);
  }

  get autonomy() {
    return this.settings.get("autonomy");
  }

  get telemetryFresh() {
    return this.tel && performance.now() - this.telT < 1000;
  }

  // True when the quad is (probably) in the air.
  isFlying() {
    if (this.autonomy === "full" && this.airborne) return true;
    const t = this.tel;
    if (!t || !this.armed()) return false;
    return t.sticks.thr > 0.22;
  }

  armed() {
    const fm = this.tel?.fm || "";
    return fm !== "" && !fm.includes("*");
  }

  // Why the AI can't fly right now, or null if it can.
  blocker() {
    const t = this.tel;
    if (!this.radio || !this.telemetryFresh) return "The radio isn't connected (no telemetry).";
    if (this.autonomy === "observer") return "Autonomy is set to Observer, so I only watch and talk.";
    if (!t.engaged) return "The AI switch on the radio is off.";
    if (t.failsafe) return "The radio is in failsafe. Flip the AI switch off and on again.";
    if (!t.linkOk) return "The radio isn't receiving my commands.";
    const o = t.override;
    if (o.roll || o.pitch || o.yaw || (this.autonomy === "full" && o.thr)) return "The pilot moved the sticks and has control. Flip the AI switch off and on to hand it back.";
    return null;
  }

  onTelemetry(t) {
    const prev = this.tel;
    this.tel = t;
    this.telT = performance.now();
    this.emit("telemetry", t);
    if (prev && prev.engaged && !t.engaged) {
      this.abort("AI switch turned off");
      this.airborne = false;
    }
    if (prev && !prev.failsafe && t.failsafe) this.abort("radio failsafe");
    // Heading from the flight controller (gyro-integrated, drifts slowly), used to correct ours.
    const y = t.att?.yaw;
    if (typeof y === "number" && (y !== 0 || t.att.pitch !== 0 || t.att.roll !== 0)) this.att.seen = true;
    if (!this.att.seen) return;
    const now = performance.now();
    if (this.att.raw === null) {
      this.att.raw = y;
      this.att.cont = y;
      this.est.heading = y;
      this.headingHist = [];
      return;
    }
    const changed = Math.abs(y - this.att.raw) > 1e-4;
    this.att.cont += wrapAngle(y - this.att.raw);
    this.att.raw = y;
    if (changed) {
      this.att.changedAt = now;
      this.learnRateScale(now);
    }
    const then = this.headingAt(now - 150);
    const gain = changed ? 0.5 : now - this.att.changedAt > 1000 ? 0.2 : 0;
    // Blend corrections in over ~0.4 s instead of jumping (jumps would look like turning).
    if (gain) this.headingFix = gain * (this.att.cont - then);
  }

  headingAt(t) {
    return this.historyAt(t).h;
  }

  // On the ground: the flight controller's heading as it is now, without the rest of a correction still being blended in
  // (the simulator's Reset turns the drone at once; a flight controller started again with a battery reports a new zero).
  // The pad button takes it before the localizer takes the pad's heading, so the rest of the blend isn't read as a turn
  // (it had turned a rehearsal's estimate 130° off the pad, and vision couldn't place a single view from there).
  snapHeading() {
    if (this.isFlying() || this.att.raw === null) return;
    Object.assign(this, { headingFix: 0, headingHist: [] });
    this.est.heading = this.att.cont;
  }

  // Compare how far the flight controller says we turned with how far our commands should have
  // turned us, across whole turns (from standstill to standstill, so telemetry lag cancels out).
  // Fixes turn accuracy when the rates in Settings don't match the quad.
  learnRateScale(now) {
    const TEL_LAG = 150;
    const c = this.historyAt(now - TEL_LAG).c;
    const rest = Math.abs(c - this.historyAt(now - TEL_LAG - 400).c) < 2 * DEG;
    this.attLog.push({ t: now, y: this.att.cont, c, rest });
    while (this.attLog.length && this.attLog[0].t < now - 6000) this.attLog.shift();
    if (!rest) return;
    const old = this.attLog.find((a) => a.rest && now - a.t >= 800);
    if (!old) return;
    const dTel = this.att.cont - old.y;
    const dCmd = c - old.c;
    if (Math.abs(dCmd) < 30 * DEG || Math.sign(dTel) !== Math.sign(dCmd)) return;
    const ratio = clamp(dTel / dCmd, 0.6, 1.6);
    this.rateScale = clamp(this.rateScale * (1 + 0.5 * (ratio - 1)), 0.5, 2);
    this.attLog = [this.attLog[this.attLog.length - 1]];
  }

  historyAt(t) {
    const h = this.headingHist;
    if (!h.length) return { h: this.est.heading, c: this.cmdHeading || 0, p: 0, q: 0 };
    if (t <= h[0].t) return h[0];
    for (let i = h.length - 1; i >= 0; i--) {
      if (h[i].t <= t) {
        const a = h[i];
        const b = h[i + 1];
        if (!b) return a;
        const k = (t - a.t) / (b.t - a.t);
        return { h: a.h + (b.h - a.h) * k, c: a.c + (b.c - a.c) * k, p: a.p + (b.p - a.p) * k, q: a.q + (b.q - a.q) * k };
      }
    }
    return h[0];
  }

  get videoDelay() {
    return Math.max(0, Number(this.settings.get("videoDelay")) || 0);
  }

  // How much gentler height control must be for the video delay (climb rate is read from the video).
  get heightSlowdown() {
    return 1 + this.videoDelay / 120;
  }

  // Heading the quad had when the frame now on screen was captured (video arrives late).
  imageHeading() {
    const t = this.perception.latest.t || performance.now();
    return this.headingAt(t - this.videoDelay);
  }

  // Run a behavior until it finishes; resolves with { ok, text, data }.
  run(behavior, signal) {
    if (this.behavior) this.finish(this.behavior, { ok: false, text: "Interrupted by a new action." });
    return new Promise((resolve) => {
      behavior._resolve = resolve;
      behavior.t0 = performance.now();
      this.behavior = behavior;
      behavior.start?.(this);
      if (signal) {
        if (signal.aborted) return this.finish(behavior, { ok: false, aborted: true, text: "Stopped." });
        signal.addEventListener("abort", () => this.finish(behavior, { ok: false, aborted: true, text: "Stopped." }), { once: true });
      }
      this.emit("behavior", behavior);
    });
  }

  finish(behavior, result) {
    if (!behavior._resolve) return;
    const resolve = behavior._resolve;
    behavior._resolve = null;
    behavior.stop?.(this);
    if (this.behavior === behavior) this.behavior = null;
    this.hold.heading = this.handHold.heading = undefined; // the idle hover keeps whatever heading we ended up at
    if (behavior !== this.handHold) this.pilotHold = !!this.safety && this.autonomy === "copilot" && this.isFlying();
    resolve(result);
    this.emit("behavior", this.behavior);
  }

  abort(reason) {
    if (this.behavior) this.finish(this.behavior, { ok: false, aborted: true, text: `Stopped: ${reason}.` });
  }

  // Ask the human pilot for help (co-pilot mode): spoken in real life, obeyed by the sim's pilot.
  askPilot(kind, text) {
    const now = performance.now();
    if (now - (this.lastRequest[kind] ?? -Infinity) < 4000) return;
    this.lastRequest[kind] = now;
    this.emit("pilot-request", { kind, text });
  }

  tick() {
    const now = performance.now();
    const gap = now - this.lastTick;
    const dt = clamp(gap / 1000, 0.005, 0.1);
    this.lastTick = now;
    const s = this.settings.all();
    const t = this.tel;
    this.estimate(dt, now, s);
    this.emit("tick", { dt, now, gap }); // the localizer integrates here, before behaviors read the pose

    const blocked = this.blocker();
    if (blocked || !this.safety || !this.isFlying() || this.autonomy !== "copilot") this.pilotHold = false;
    let b = this.behavior;
    if (blocked && b) {
      this.finish(b, { ok: false, aborted: true, text: blocked });
      b = null;
    }
    const take = !blocked && this.safety?.takeover;
    if (take && b !== take) {
      if (b) this.finish(b, { ok: false, aborted: true, safety: true, text: `Stopped: ${take.reason}.` });
      this.run(take);
      b = take;
    }
    if (!b && !blocked && this.autonomy === "full" && this.airborne) b = this.hold; // keep hovering between commands
    else if (!b && !blocked && this.pilotHold) b = this.handHold;
    if (!b || blocked) {
      this.mask = 0;
      this.out = { roll: 0, pitch: 0, yaw: 0, thr: t ? t.sticks.thr : 0 };
      this.status = blocked || "Ready";
      this.send();
      this.learnFromPilot(dt, now);
      return;
    }

    let res;
    try {
      res = b.update(this, dt, now) || {};
    } catch (e) {
      console.error(e);
      res = { done: { ok: false, text: `Internal error: ${e.message}` } };
    }
    if (res.done && b !== this.hold && b !== this.handHold) {
      this.finish(b, res.done);
      res = this.hold.update(this, dt, now);
    }
    this.status = b.label || b.name;
    this.mask = MASK.roll | MASK.pitch | MASK.yaw | (this.autonomy === "full" ? MASK.thr : 0);
    const sp = this.safety ? this.safety.filter(res.sp || {}, { ctl: this, dt, now, gap, behavior: b }) : res.sp || {};
    this.stabilize(sp, res.thr, dt, s);
    this.send();
  }

  estimate(dt, now, s) {
    const t = this.tel;
    const e = this.est;
    const yawStick = this.mask & MASK.yaw ? this.out.yaw : t ? t.sticks.yaw : 0;
    const flying = this.isFlying() || !!this.mask;
    const rateCW = flying ? actualRate(yawStick, s.yawCenter, s.yawMax) * DEG * this.rateScale : 0;
    e.heading += rateCW * dt;
    if (this.headingFix) {
      const step = this.headingFix * Math.min(1, dt / 0.4);
      e.heading += step;
      this.headingFix -= step;
    }
    this.cmdHeading = (this.cmdHeading || 0) + rateCW * dt; // pure integral of commanded turns (no telemetry jumps)

    // Attitude we are commanding (angle mode) - used to separate rotation from translation in the image.
    const angleLim = s.angleLimit * DEG;
    const pitchCmd = (this.mask & MASK.pitch ? this.out.pitch : t ? t.sticks.pitch : 0) * angleLim;
    const rollCmd = (this.mask & MASK.roll ? this.out.roll : t ? t.sticks.roll : 0) * angleLim;
    const k = Math.min(1, dt / 0.06);
    e.pitchAngle += (pitchCmd - e.pitchAngle) * k;
    e.rollAngle += (rollCmd - e.rollAngle) * k;
    this.headingHist.push({ t: now, h: e.heading, c: this.cmdHeading, p: e.pitchAngle, q: e.rollAngle });
    while (this.headingHist.length && this.headingHist[0].t < now - 3000) this.headingHist.shift();

    const f = this.perception.latest.flow;
    const fresh = f && now - f.t < 250;
    if (fresh && f.quality > 0.15) {
      // The image is videoDelay old and compares two frames f.span apart: compare it with how we
      // were turning and pitching over that same stretch of time.
      const span = f.span || 0.1;
      const then = this.historyAt(f.t - this.videoDelay);
      const before = this.historyAt(f.t - this.videoDelay - span * 1000);
      const rate = (then.c - before.c) / span;
      const pitchRate = (then.p - before.p) / span;
      const rollRate = (then.q - before.q) / span;
      const hfov = s.hfov * DEG;
      const w = this.perception.latest.width || 16;
      const h = this.perception.latest.height || 9;
      const vfov = (hfov * h) / w;
      if (!this.rotX || this.rotX.hfov !== hfov) {
        this.rotX = Object.assign(new RotationModel([-1 / hfov, 0], [[-2 / hfov, -0.4 / hfov], [-1 / hfov, 1 / hfov]]), { hfov });
        this.rotY = new RotationModel([-1 / vfov], [[-2 / vfov, -0.4 / vfov]]);
      }
      // Calibrate while we are clearly rotating (translation is small next to rotation then).
      if (Math.hypot(rate, rollRate) > 0.35 && f.quality > 0.4) this.rotX.update([rate, rollRate], f.dx);
      if (Math.abs(pitchRate) > 0.35 && f.quality > 0.4) this.rotY.update([pitchRate], f.dy);
      const a = Math.min(1, dt / 0.12) * Math.min(1, f.quality * 1.5);
      // While turning fast, sideways image motion is mostly rotation: don't read drift from it.
      e.rotating = Math.abs(rate) > 25 * DEG;
      const dxTrans = f.dx - this.rotX.predict([rate, rollRate]);
      const dyTrans = f.dy - this.rotY.predict([pitchRate]);
      if (!e.rotating) e.vy += (clamp(-dxTrans * hfov * NOMINAL_DEPTH, -3, 3) - e.vy) * a;
      else e.vy -= e.vy * Math.min(1, dt / 0.5);
      e.vz += (clamp(dyTrans * vfov * NOMINAL_DEPTH, -3, 3) - e.vz) * a;
      e.vx += (clamp(f.div * NOMINAL_DEPTH, -3, 3) - e.vx) * a * 0.6; // zoom is the noisiest signal
      e.ttc = f.div > 0.08 ? 1 / f.div : Infinity;
      e.flowQ = f.quality;
    } else {
      const decay = Math.min(1, dt / 0.6);
      e.vx -= e.vx * decay;
      e.vy -= e.vy * decay;
      e.vz -= e.vz * decay;
      e.ttc = Infinity;
      e.flowQ = 0;
    }

    // Dead-reckoned odometry (scale is only roughly right, but holding it still holds position).
    if (this.isFlying()) {
      const c = Math.cos(e.heading);
      const sn = Math.sin(e.heading);
      e.px += (e.vx * c - e.vy * sn) * dt;
      e.py += (e.vx * sn + e.vy * c) * dt;
      e.z += e.vz * dt;
    }
  }

  // Setpoints -> sticks.
  // sp: { vx, vy (m/s, body frame; omit both to hold position), vz (m/s; omit to hold height), z (m, H: the height to hold,
  //       on the localizer's height, while the hold follows it: safety.height()),
  //       yawRate (rad/s, + = right) | heading (rad), maxYawRate? (rad/s: the safety layer's turn limit, e.g. while vision
  //       fixes need a steady view) }
  stabilize(sp, thrOverride, dt, s) {
    const e = this.est;
    const maxTilt = s.maxTilt;
    let yawRate = sp.yawRate ?? 0;
    if (sp.heading !== undefined) yawRate = clamp(2.2 * wrapAngle(sp.heading - e.heading), -1.2, 1.2);
    const yawMax = Math.min(s.maxYawRate * DEG, sp.maxYawRate ?? Infinity);
    yawRate = clamp(yawRate, -yawMax, yawMax);
    const yaw = actualRateInverse(yawRate / this.rateScale / DEG, s.yawCenter, s.yawMax);

    // Horizontal: velocity commands, or hold the odometry point we had when velocity commands stopped.
    let vx = sp.vx;
    let vy = sp.vy;
    const turningFast = Math.abs(yawRate) > 25 * DEG || e.rotating;
    if (vx === undefined && vy === undefined && turningFast) {
      // Hold still (velocity damping) while spinning; re-anchor the position once the turn ends.
      this.posRef = null;
      vx = vy = 0;
    } else if (vx === undefined && vy === undefined) {
      if (!this.posRef) this.posRef = { x: e.px, y: e.py };
      const ex = this.posRef.x - e.px;
      const ey = this.posRef.y - e.py;
      const c = Math.cos(e.heading);
      const sn = Math.sin(e.heading);
      const k = 0.9 / (1 + this.videoDelay / 250); // late video: a stiffer hold swings past the point
      vx = clamp(k * (ex * c + ey * sn), -0.3, 0.3);
      vy = clamp(k * (-ex * sn + ey * c), -0.3, 0.3);
    } else {
      this.posRef = null;
      vx = vx ?? 0;
      vy = vy ?? 0;
    }
    const good = e.flowQ > 0.25;
    // The flow's velocity assumes a scene NOMINAL_DEPTH away; while splat fixes aid the localizer, the scale it learnt
    // (safety.flowScale()) makes it the true one, so braking doesn't weaken in front of a far background.
    const k = this.safety?.flowScale?.() ?? 1, evx = k * e.vx, evy = k * e.vy;
    if (good) {
      // Integrators cancel the whoop's steady drift (accelerometer trim, prop wash).
      this.iVx = clamp(this.iVx + (vx - evx) * dt * 0.05, -0.05, 0.05);
      this.iVy = clamp(this.iVy + (vy - evy) * dt * 0.05, -0.05, 0.05);
    }
    // The 0.11 feedforward alone holds the commanded speed; the feedback on (late) image motion is eased off with delay,
    // or speed changes overshoot by half.
    const kv = 0.17 / (1 + this.videoDelay / 250);
    const pitch = clamp(0.11 * vx + kv * (vx - evx) + this.iVx, -maxTilt, maxTilt);
    const roll = clamp(0.11 * vy + kv * (vy - evy) + this.iVy, -maxTilt, maxTilt);

    let thr = this.tel ? this.tel.sticks.thr : 0;
    if (this.autonomy === "full") {
      const vbat = this.tel?.vbat;
      const hover = this.hover.estimate(vbat);
      const tilt = Math.max(0.75, Math.cos(e.pitchAngle) * Math.cos(e.rollAngle));
      // The height and climb rate held: the localizer's while it knows them (safety.height(): vision fixes and the
      // throttle's physics keep them), else the controller's own, dead reckoned from the flow's climb rate (which misreads:
      // +0.8 m/s while the drone sank, and drifts: a scan climbed 1.1 m on it). A switch keeps the height held where it was.
      const L = this.safety?.height?.(), loc = !!L?.lead, z = loc ? L.z : e.z, vz = loc ? L.vz : e.vz;
      if (loc !== this.zLoc && this.zRef !== null && L) this.zRef += loc ? L.z - e.z : e.z - L.z;
      this.zLoc = loc;
      if (thrOverride !== undefined) {
        thr = thrOverride;
        this.zRef = z;
      } else {
        // Height: climb/sink rate commands, or hold the estimated height. The flow's climb rate comes from video that is
        // videoDelay old, so on it the loop gets gentler as the delay grows (at 0.12 per m/s it oscillates past ~0.2 s); the
        // localizer's is carried to now (the throttle's physics), so on it the loop keeps its pace (a gentle one let gusts
        // move a scan 0.35 m).
        const slow = loc ? 1 : this.heightSlowdown;
        let vzCmd = sp.vz;
        if (vzCmd === undefined || vzCmd === 0) {
          if (loc && sp.z !== undefined) this.zRef = sp.z; // the safety layer's height to hold, on the localizer's height
          else if (this.zRef === null) this.zRef = z;
          vzCmd = clamp(((loc ? HEIGHT.kz : 1.0) / slow) * (this.zRef - z), -0.35, 0.35);
        } else this.zRef = z;
        const err = vzCmd - vz;
        this.iThr = clamp(this.iThr + (err * dt * 0.08) / slow, -0.15, 0.15);
        thr = hover / tilt + ((loc ? HEIGHT.kv : 0.12) / slow) * err + this.iThr;
        // Once settled, move what the integrator learned into the hover model.
        if (sp.vz === undefined && good && Math.abs(vz) < 0.1 && Math.abs(this.zRef - z) < 0.15) {
          const moved = this.iThr * 0.01;
          this.hover.shift(moved, vbat);
          this.iThr -= moved;
        }
      }
      thr = clamp(thr, 0, 0.85);
    }

    // Smooth the sticks a little (slew-rate limits).
    const o = this.out;
    const step = (cur, target, rate) => cur + clamp(target - cur, -rate * dt, rate * dt);
    this.out = {
      roll: step(o.roll, roll, 2.5),
      pitch: step(o.pitch, pitch, 2.5),
      yaw: step(o.yaw, yaw, 5),
      thr: this.autonomy === "full" ? step(o.thr, thr, 1.6) : thr,
    };
  }

  // Reset odometry and hold references (e.g. at take-off).
  resetHold() {
    const e = this.est;
    e.px = e.py = e.z = 0;
    this.posRef = null;
    this.zRef = null;
    this.iThr = this.iVx = this.iVy = 0;
  }

  // Learn the hover throttle from the pilot's thumb while they fly steadily.
  learnFromPilot(dt, now) {
    const t = this.tel;
    const e = this.est;
    if (!t || !this.armed() || t.sticks.thr < 0.15 || e.flowQ < 0.3 || Math.abs(e.vz) > 0.12) {
      this.stableSince = now;
      return;
    }
    if (now - this.stableSince > 1000) this.hover.learn(t.sticks.thr, t.vbat, 0.01);
  }

  send() {
    if (!this.radio) return;
    this.seq = (this.seq + 1) % 10000;
    const vbat = this.tel?.vbat;
    this.radio.send({ seq: this.seq, mask: this.mask, ...this.out, hover: this.hover.estimate(vbat) });
  }

  // Bearing of a normalized image x position (0..1), radians, + = right. Equidistant fisheye model.
  bearingOf(x) {
    return (x - 0.5) * this.settings.get("hfov") * DEG;
  }

  // Elevation of a normalized image y position relative to the horizon (rad, + = up).
  elevationOf(y) {
    const s = this.settings.all();
    const w = this.perception.latest.width || 16;
    const h = this.perception.latest.height || 9;
    const vfov = (s.hfov * DEG * h) / w;
    return (0.5 - y) * vfov + s.uptilt * DEG - this.est.pitchAngle;
  }
}
