// Flight behaviors. Each one is ticked by the FlightController (~30 Hz) and returns setpoints:
//   { sp: { vx, vy, vz (m/s), yawRate (rad/s, + = right) | heading (rad) }, thr?: direct throttle }
// Leave out vx/vy to hold position and vz to hold height (see FlightController.stabilize).
// or { done: { ok, text, data? } } when finished. Headings are continuous (not wrapped), + = clockwise.
// With a house map (PathFollow, DoorTransit, ScanAt, ReturnHome, Land at a point) positions come from the localizer
// (nav/localizer.js) in the house frame H: x, y on the plan, yaw counter-clockwise; setpoints stay in the body frame.
import { clamp, DEG, wrapAngle } from "./util.js";
import { plan } from "./house/planner.js";
import { safeSpot, staticClearance, cruiseSpeed, withoutTemps, polyDist } from "./nav/avoid.js";

const seconds = (b, now) => (now - b.t0) / 1000;

// Things that live on the floor: the camera is tilted up, so the quad has to fly low to see them.
export const FLOOR_LABELS = ["cat", "dog", "backpack", "sports ball", "teddy bear", "handbag", "suitcase", "bowl"];

// Get down to roughly knee height (full auto) or ask the pilot to (co-pilot). Returns a setpoint while
// still descending, else null.
function getLow(b, ctl, labels, now) {
  if (!labels.some((l) => FLOOR_LABELS.includes(l))) return null;
  if (ctl.autonomy !== "full") {
    if (!b.askedLow) ctl.askPilot("low", "Fly low, about knee height, so I can see the floor.");
    b.askedLow = true;
    return null;
  }
  if (b.lowered || ctl.est.z < 0.6 || seconds(b, now) > 3) {
    b.lowered = true;
    return null;
  }
  return { sp: { vz: -0.3 } };
}

export class Hold {
  constructor(duration = Infinity) {
    this.name = "hold";
    this.duration = duration;
    this.label = Number.isFinite(duration) ? `Hovering ${duration}s` : "Holding position";
  }
  start(ctl) {
    this.heading = ctl.est.heading;
  }
  update(ctl, dt, now) {
    if (this.heading === undefined) this.heading = ctl.est.heading;
    if (this.t0 && seconds(this, now) >= this.duration) return { done: { ok: true, text: `Hovered for ${this.duration} s.` } };
    return { sp: { heading: this.heading } };
  }
}

export class Turn {
  constructor(degrees) {
    this.name = "turn";
    this.deg = degrees;
    this.label = `Turning ${degrees >= 0 ? "right" : "left"} ${Math.abs(Math.round(degrees))}°`;
  }
  start(ctl) {
    this.target = ctl.est.heading + this.deg * DEG;
    this.settle = 0;
  }
  update(ctl, dt, now) {
    const err = this.target - ctl.est.heading;
    this.settle = Math.abs(err) < 6 * DEG ? this.settle + dt : 0;
    if (this.settle > 0.25) return { done: { ok: true, text: `Turned ${this.deg >= 0 ? "right" : "left"} ${Math.abs(Math.round(this.deg))}°.` } };
    if (seconds(this, now) > 3 + Math.abs(this.deg) / 40) return { done: { ok: false, text: "The turn timed out (heading estimate may be off)." } };
    return { sp: { yawRate: clamp(2.2 * err, -1.5, 1.5) } };
  }
}

export class Move {
  constructor(direction, meters, speed) {
    this.name = "move";
    this.dir = direction;
    this.meters = meters;
    this.speed = speed;
    this.label = `Moving ${direction} ~${meters.toFixed(1)} m`;
  }
  start(ctl) {
    this.heading = ctl.est.heading;
    this.duration = this.meters / this.speed + 0.5;
  }
  update(ctl, dt, now) {
    const t = seconds(this, now);
    const moved = `about ${this.meters.toFixed(1)} m ${this.dir}`;
    if (t >= this.duration) return { done: { ok: true, text: `Moved ${moved} (distance is estimated from time).` } };
    if (this.dir === "forward" && t > 0.5 && ctl.est.ttc < 0.9) {
      return { done: { ok: false, text: `Stopped early after ${t.toFixed(1)} s: something is close ahead.` } };
    }
    const v = this.speed;
    const sp = { heading: this.heading };
    if (this.dir === "forward") Object.assign(sp, { vx: v, vy: 0 });
    if (this.dir === "back") Object.assign(sp, { vx: -v, vy: 0 });
    if (this.dir === "right") Object.assign(sp, { vx: 0, vy: v });
    if (this.dir === "left") Object.assign(sp, { vx: 0, vy: -v });
    if (this.dir === "up") sp.vz = 0.35;
    if (this.dir === "down") sp.vz = -0.35;
    return { sp };
  }
}

// Full-auto take-off: spool up, punch out of ground effect, settle into a hover.
export class TakeOff {
  constructor() {
    this.name = "takeoff";
    this.label = "Taking off";
  }
  start(ctl) {
    this.heading = ctl.est.heading;
    ctl.resetHold();
  }
  update(ctl, dt, now) {
    const t = seconds(this, now);
    const hover = ctl.hover.estimate(ctl.tel?.vbat);
    if (t < 0.4) return { sp: { heading: this.heading }, thr: hover * 0.7 * (t / 0.4) }; // spool up
    ctl.airborne = true;
    if (t < 0.7) return { sp: { heading: this.heading }, thr: hover * 1.2 }; // hop out of ground effect
    if (t < 2.2) return { sp: { vz: 0.5, heading: this.heading } }; // climb
    if (t < 3.2) return { sp: { heading: this.heading } }; // settle
    return { done: { ok: true, text: "Airborne and hovering (roughly 1 m up)." } };
  }
}

// Full-auto landing: descend gently, cut throttle on touchdown (or after a few seconds of descent). The safety layer's
// speed cap (nav/avoid.js) may hold the descent over something ("something below"): that isn't a touchdown, and that time
// doesn't count toward the cut; held LAND.blocked s, it ends without landing ({ ok: false, blocked }) for the caller to
// pick another spot, except a forced landing (onto: the battery is empty), which then sets down on it slowly.
export const LAND = { blocked: 2, touchAfter: 1 };
export class Land {
  // at: [x, y] in H with a localizer: hold over that point on the way down.
  constructor({ at = null, localizer = null, label = "Landing", onto = false } = {}) {
    this.name = "land";
    this.label = label;
    Object.assign(this, { at, loc: localizer, onto });
  }
  start(ctl) {
    this.heading = ctl.est.heading;
    Object.assign(this, { still: 0, held: 0, heldAll: 0, touch: false, cutAt: null, startThr: null });
  }
  update(ctl, dt, now) {
    const t = seconds(this, now);
    if (this.cutAt === null) {
      const held = ctl.safety?.avoid?.last?.vertical === "something below"; // slowed or stopped over something
      this.held = held && !(ctl.est.vz < -0.08) ? this.held + dt : 0;
      this.heldAll += held ? dt : 0;
      if (this.held > LAND.blocked && !this.onto) return { done: { ok: false, blocked: true, text: "Something is below me, so I didn't land there." } };
      this.touch ||= this.onto && this.held > LAND.touchAfter;
      const f = ctl.perception.latest.flow;
      const quiet = f && Math.abs(f.dy) < 0.02 && Math.abs(f.dx) < 0.02 && Math.abs(f.div) < 0.03;
      this.still = t > 1.5 && quiet && !held ? this.still + dt : 0;
      if (this.still > 0.5 || t - this.heldAll > 5) {
        this.cutAt = t;
        this.startThr = ctl.out.thr;
      } else {
        const P = this.at && this.loc?.pose();
        return { sp: { vz: -0.35, heading: this.heading, ...(this.touch && { touchdown: true }), ...(P && P.status !== "lost" && toward(P, this.at, 0.8, 0.15)) } };
      }
    }
    const k = (t - this.cutAt) / 0.6;
    if (k >= 1) {
      ctl.airborne = false;
      return { done: { ok: true, text: "Landed. Motors at idle - the pilot can disarm." } };
    }
    return { sp: { heading: this.heading }, thr: this.startThr * (1 - k) * 0.8 };
  }
}

// Rotate until one of the given detector labels is seen a few frames in a row, then center it.
export class Search {
  constructor(labels, maxDegrees = 390) {
    this.name = "search";
    this.targetLabels = Array.isArray(labels) ? labels : [labels];
    this.maxDeg = maxDegrees;
    this.label = `Looking for ${this.targetLabels.join(" / ")}`;
  }
  start(ctl) {
    this.startHeading = ctl.est.heading;
    this.hits = 0;
  }
  update(ctl, dt, now) {
    const low = getLow(this, ctl, this.targetLabels, now);
    if (low) return low;
    let det = null;
    if (ctl.perception.frameAge < 400) {
      for (const l of this.targetLabels) {
        const d = ctl.perception.find(l, 0.4)[0];
        if (d && (!det || d.score > det.score)) det = d;
      }
    }
    this.hits = det ? this.hits + 1 : Math.max(0, this.hits - 1);
    if (det && this.hits >= 3) {
      // The video is a little late: aim at where the object was relative to our heading back then.
      this.found = { label: det.label, box: det.box, trackId: det.trackId, heading: ctl.imageHeading() + ctl.bearingOf(det.box.x + det.box.w / 2) };
    }
    if (this.found) {
      const err = this.found.heading - ctl.est.heading;
      this.settle = Math.abs(err) < 8 * DEG ? (this.settle || 0) + dt : 0;
      if (this.settle > 0.15 + ctl.videoDelay / 1000) {
        const { label, box, trackId } = this.found;
        return { done: { ok: true, text: `Found the ${label}; it's straight ahead.`, data: { label, box, trackId } } };
      }
      return { sp: { yawRate: clamp(2 * err, -1, 1) } };
    }
    const what = this.targetLabels.length > 1 ? "any of them" : `a ${this.targetLabels[0]}`;
    const turned = Math.abs(ctl.est.heading - this.startHeading) / DEG;
    if (turned > this.maxDeg) return { done: { ok: false, text: `Turned all the way around and didn't see ${what}.` } };
    if (seconds(this, now) > 25) return { done: { ok: false, text: `Search timed out without seeing ${what}.` } };
    return { sp: { yawRate: 40 * DEG } };
  }
}

// Visual servoing on a detected object: turn to center it, fly until it looks the right size.
// follow=true keeps going until the time runs out (and searches if the target is lost).
// Locks on to the detector's track id (from Search, or the first one it sees), so another cat doesn't take over.
export class Approach {
  constructor(label, { size = 0.3, follow = false, duration = 60, maxSpeed = 0.45, trackId = null } = {}) {
    this.name = follow ? "follow" : "approach";
    this.targetLabel = label;
    this.firstTrack = trackId;
    this.size = size;
    this.follow = follow;
    this.duration = duration;
    this.maxSpeed = maxSpeed;
    this.label = `${follow ? "Following" : "Approaching"} ${label}`;
  }
  start() {
    this.box = null;
    this.trackId = this.firstTrack;
    this.lastSeen = performance.now();
    this.lastBearing = 0;
    this.searchTurn = 0;
    this.close = 0;
    this.seenFor = 0;
  }
  update(ctl, dt, now) {
    const low = getLow(this, ctl, [this.targetLabel], now);
    if (low) return low;
    const fresh = ctl.perception.frameAge < 400;
    const dets = fresh ? ctl.perception.find(this.targetLabel, 0.35) : [];
    let det = this.trackId != null ? dets.find((d) => d.trackId === this.trackId) : null;
    if (!det && dets.length && (this.trackId == null || now - this.lastSeen > 700)) {
      det = dets[0];
      if (this.box && dets.length > 1) {
        const c = center(this.box);
        det = dets.reduce((a, b) => (dist(center(a.box), c) < dist(center(b.box), c) ? a : b));
      }
      this.trackId = det.trackId ?? null;
    }
    if (det) {
      this.box = this.box ? blend(this.box, det.box, 0.5) : { ...det.box };
      this.lastSeen = now;
      this.searchTurn = 0;
      this.seenFor += dt;
      this.targetHeading = ctl.imageHeading() + ctl.bearingOf(this.box.x + this.box.w / 2);
    }
    const t = seconds(this, now);
    if (this.follow && t > this.duration) return { done: { ok: true, text: `Followed the ${this.targetLabel} for ${Math.round(t)} s.` } };
    if (!this.follow && t > 45) return { done: { ok: false, text: `Couldn't reach the ${this.targetLabel} within 45 s.` } };

    const lostMs = now - this.lastSeen;
    if (lostMs > 700) {
      this.box = null;
      this.searchTurn += 50 * DEG * dt;
      if (this.searchTurn > 2 * Math.PI || lostMs > 15000) return { done: { ok: false, text: `Lost sight of the ${this.targetLabel}.` } };
      return { sp: { yawRate: (this.lastBearing >= 0 ? 1 : -1) * 50 * DEG } };
    }
    if (!this.box) return { sp: { yawRate: 0 } };

    const [, cy] = center(this.box);
    const bearing = this.targetHeading - ctl.est.heading; // where the target is now, allowing for video delay
    this.lastBearing = bearing;
    const err = (this.size - this.box.h) / this.size; // > 0: still too far
    let vx = clamp(0.8 * err, -0.3, this.maxSpeed);
    if (Math.abs(bearing) > 25 * DEG) vx = Math.min(vx, 0.05);
    if (vx > 0 && ctl.est.ttc < 0.8 && this.box.h < this.size * 0.6) vx = 0; // something else is closer than the target
    let vz;
    if (ctl.autonomy === "full") {
      // Keep the target a little below us (the camera is tilted up, so this is not image center).
      vz = clamp(1.2 * (ctl.elevationOf(cy) + 12 * DEG), -0.3, 0.3);
      if (ctl.est.z < 0.25) vz = Math.max(vz, 0);
    } else if (this.box.h < this.size * 0.8 && this.box.y + this.box.h > 0.97 && now - (this.askedAt || -1e9) > 8000) {
      // Only while it's still small and cut off by the bottom edge: close up, low is expected.
      this.askedAt = now;
      ctl.askPilot("lower", `Go a little lower - the ${this.targetLabel} is at the bottom of my view.`);
    } else if (cy < 0.16 && now - (this.askedAt || -1e9) > 8000) {
      this.askedAt = now;
      ctl.askPilot("higher", `Go a little higher - the ${this.targetLabel} is at the top of my view.`);
    }

    if (!this.follow) {
      this.close = Math.abs(err) < 0.18 && Math.abs(bearing) < 10 * DEG ? this.close + dt : 0;
      if (this.close > 0.6) return { done: { ok: true, text: `Arrived in front of the ${this.targetLabel}.` } };
    }
    return { sp: { vx, vy: 0, vz, yawRate: clamp(2 * bearing, -1.3, 1.3) } };
  }
}

// 360° scan in steps, grabbing a snapshot + detections at each heading (for Claude to choose a direction).
export class LookAround {
  constructor(steps = 8) {
    this.name = "look";
    this.steps = steps;
    this.label = "Looking around";
  }
  start(ctl) {
    this.base = ctl.est.heading;
    this.i = 0;
    this.phase = "settle";
    this.phaseT = performance.now();
    this.frames = [];
  }
  update(ctl, dt, now) {
    const stepRad = (2 * Math.PI) / this.steps;
    const target = this.base + this.i * stepRad;
    const err = target - ctl.est.heading;
    if (this.phase === "turn") {
      if (Math.abs(err) < 6 * DEG) {
        this.phase = this.i >= this.steps ? "done" : "settle";
        this.phaseT = now;
      }
      if (seconds(this, now) > 25) return { done: { ok: false, text: "Look-around timed out.", data: { frames: this.frames } } };
      return { sp: { yawRate: clamp(2.2 * err, -1.4, 1.4) } };
    }
    if (this.phase === "settle" && now - this.phaseT > 400 + ctl.videoDelay) {
      this.frames.push({
        angle: Math.round((this.i * 360) / this.steps),
        image: ctl.perception.snapshot({ maxWidth: 320, quality: 0.7 }),
        detections: ctl.perception.latest.detections.map((d) => ({ label: d.label, score: d.score, box: d.box })),
      });
      this.i++;
      this.phase = "turn";
    }
    if (this.phase === "done") {
      return { done: { ok: true, text: `Looked around in ${this.steps} directions and turned back to the start.`, data: { frames: this.frames } } };
    }
    return { sp: { heading: target } };
  }
}

// Wait (neutral sticks) until a condition holds, e.g. the pilot has taken off in co-pilot mode.
export class WaitFor {
  constructor(label, test, timeout, okText, failText) {
    Object.assign(this, { name: "wait", label, test, timeout, okText, failText });
  }
  update(ctl, dt, now) {
    if (this.test(ctl)) return { done: { ok: true, text: this.okText } };
    if (seconds(this, now) > this.timeout) return { done: { ok: false, text: this.failText } };
    return { sp: { yawRate: 0 } };
  }
}

// ---- with a house map ----

// An H-frame velocity as body-frame setpoints for a drone at H yaw psi (vy + = right).
export const toBody = (vx, vy, psi) => ({ vx: Math.cos(psi) * vx + Math.sin(psi) * vy, vy: Math.sin(psi) * vx - Math.cos(psi) * vy });
// Fly toward an H point (P-control, capped), as body-frame setpoints.
export function toward(P, [x, y], gain = 0.8, vmax = 0.25) {
  const ex = x - P.x, ey = y - P.y, L = Math.hypot(ex, ey), v = Math.min(vmax, gain * L);
  return L < 1e-6 ? { vx: 0, vy: 0 } : toBody((ex / L) * v, (ey / L) * v, P.yaw);
}
const lostDone = { done: { ok: false, lost: true, text: "Lost track of where I am." } };
// Full auto: toward height z on the localizer's, while that is sure enough (else the controller holds its own: chasing a
// height estimate 1 m off flew a drone into the ceiling): σz at most HOLD_SIGMA[1] while the controller holds the
// localizer's height itself (ctl.zLoc: the two follow the same height, as sure as safety.js SAFETY.zLead; at 0.2 a σz that
// saws between fixes switched the chase off half the time and a leg stayed 0.4 m over its path), else HOLD_SIGMA[0].
export const HOLD_SIGMA = [0.2, 0.3];
const heightHold = (ctl, P, z) => (ctl.autonomy === "full" && z != null && P.z != null && !(P.zSigma > HOLD_SIGMA[ctl.zLoc ? 1 : 0]) ? { vz: clamp((1.2 / ctl.heightSlowdown) * (z - P.z), -0.3, 0.3) } : {});

// Regulated pure pursuit along a planner path [[x, y, z]]: face the course, slow near obstacles, on heading error, near
// doorways and the goal, and when less sure of the position. It hands the way back to whoever planned it (done with
// blocked: true, to plan again) when the path ahead (2 m) has become tighter than when it was planned (a temporary
// obstacle: a person, something new the live depth found, a suspected change), after 2 s of the safety layer holding it
// back from a person, after 3 s of nav/avoid.js holding it still (something the map lacks ahead), after 2 s more than
// 0.45 m off the path (pushed off it: the way on from there may be through a wall; more if it started off), or after 5 s of something close
// ahead (time to contact) the map doesn't explain. Turns faster than
// 20°/s happen only nearly stopped (the controller can't read sideways drift while spinning). The last 0.35 m is plain
// position control on the goal, settling longer when the pose is unsure. In full auto it also flies the path's height,
// climbing early (to the highest point of the next metre, holding back until it is up there) and sinking late.
export class PathFollow {
  constructor(path, { localizer, map, vmax = 0.5, lookahead = 0.6, goalTol = 0.15, doors = [], need = map.lethal(), label = "Following the path" } = {}) {
    Object.assign(this, { name: "path", label, path, loc: localizer, map, vmax, lookahead, goalTol, doors, need });
  }
  start() {
    const p = this.path;
    this.s = p.map(() => 0);
    for (let i = 1; i < p.length; i++) this.s[i] = this.s[i - 1] + Math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1]);
    this.cl0 = p.map((q) => Math.min(this.need, this.map.clearance(q[0], q[1], q[2])));
    const P = this.loc.pose();
    Object.assign(this, { i: 0, blockedFor: 0, personFor: 0, personAt: -Infinity, heldFor: 0, offFor: 0, settle: 0, hold: null, off0: Math.hypot(p[0][0] - P.x, p[0][1] - P.y) });
  }
  // The first point within `ahead` m whose clearance dropped below what it had when the path was planned, or -1.
  narrowed(from, ahead = 2.0) {
    const { path, s, cl0 } = this;
    for (let k = from; k < path.length && s[k] - s[from] < ahead; k++) if (this.map.clearance(path[k][0], path[k][1], path[k][2]) < cl0[k] - 0.03) return k;
    return -1;
  }
  update(ctl, dt, now) {
    const P = this.loc.pose(), { path, s } = this, n = path.length, goal = path[n - 1];
    if (P.status === "lost") return lostDone;
    let best = this.i, bd = Infinity;
    for (let k = this.i; k < n && s[k] <= s[this.i] + 1.0; k++) {
      const d = Math.hypot(path[k][0] - P.x, path[k][1] - P.y);
      if (d < bd) [bd, best] = [d, k];
    }
    this.i = best;
    this.offFor = bd > Math.max(0.45, this.off0 + 0.15) ? this.offFor + dt : 0;
    if (this.offFor > 2) return { done: { ok: false, blocked: true, text: "I was pushed off the way." } };
    const k = this.narrowed(best);
    if (k >= 0) return { done: { ok: false, blocked: true, at: path[k], text: "Something new is in the way." } };
    const dGoal = Math.hypot(goal[0] - P.x, goal[1] - P.y), near = s[n - 1] - s[best] < 0.6 && dGoal < (this.hold == null ? 0.35 : 0.6);
    if (!near) this.hold = null;
    this.settle = near && dGoal < this.goalTol ? this.settle + dt : Math.max(0, this.settle - dt);
    if (this.settle > (P.sigma > 0.08 ? 1.2 : 0.3)) return { done: { ok: true, text: "Arrived.", data: { error: dGoal } } }; // unsure: let a fix land first
    if ((now - this.t0) / 1000 > 20 + s[n - 1] / 0.12) return { done: { ok: false, text: "Took too long to follow the path." } };
    if (ctl.safety?.personHeld) this.personAt = now; // the safety layer won't let us closer to someone
    this.personFor = now - this.personAt < 1000 ? this.personFor + dt : 0;
    if (this.personFor > 2) return { done: { ok: false, blocked: true, person: true, text: "A person is in the way." } };
    const av = ctl.safety?.avoid?.last;
    this.heldFor = av?.held && now - av.t < 200 ? this.heldFor + dt : 0;
    if (this.heldFor > 3) return { done: { ok: false, blocked: true, held: true, text: "Something the map lacks is in the way." } };
    if (near) {
      this.hold ??= ctl.est.heading;
      return { sp: { ...toward(P, goal, 1.0, 0.2), heading: this.hold, ...heightHold(ctl, P, goal[2]) } };
    }
    let j = best, zUp = path[best][2];
    while (j < n - 1 && s[j] - s[best] < this.lookahead) j++;
    for (let k = best; k < n && s[k] - s[best] < 1.0; k++) zUp = Math.max(zUp, path[k][2]);
    const err = wrapAngle(Math.atan2(path[j][1] - P.y, path[j][0] - P.x) - P.yaw), full = ctl.autonomy === "full";
    const clear = this.map.clearance(P.x, P.y, P.z), door = this.doors.some((d) => Math.hypot(d.mid[0] - P.x, d.mid[1] - P.y) < 0.8);
    let v = this.vmax * clamp((clear - 0.15) / 0.4, 0.4, 1) * clamp(1 - Math.abs(err) / (50 * DEG), 0, 1) * clamp(1.15 - P.sigma, 0.6, 1);
    v = Math.min(v, 0.1 + 0.7 * dGoal, door ? 0.3 : Infinity);
    if (full && P.z != null) v *= clamp(1 - (zUp - P.z + (P.zSigma ?? 0) - 0.08) / 0.15, 0, 1);
    if (ctl.est.ttc < 0.8) {
      v = 0;
      if ((this.blockedFor += dt) > 5) return { done: { ok: false, blocked: true, text: "Something I can't see on the map is in the way." } };
    } else this.blockedFor = 0;
    // Turning faster than ~25°/s blinds the controller to sideways drift, so while moving turns stay slower than that.
    const moving = v > 0.08 || Math.hypot(ctl.est.vx, ctl.est.vy) > 0.12;
    const turn = moving ? { yawRate: clamp(-2.2 * err, -20 * DEG, 20 * DEG) } : { heading: ctl.est.heading - err };
    return { sp: { vx: v * Math.cos(err), vy: -v * Math.sin(err), ...turn, ...heightHold(ctl, P, Math.max(zUp, path[j][2])) } };
  }
}

// Through a doorway (a plan() door: { id, mid, pre, post }): line up at the pre-door waypoint facing across, check that
// nothing is close ahead and that the doorway fits the drone at the current uncertainty (±2σ), then cross straight to
// the post-door waypoint at 0.25 m/s.
export class DoorTransit {
  constructor(door, { localizer, map, speed = 0.25, label = "Going through the doorway" } = {}) {
    Object.assign(this, { name: "door", label, door, loc: localizer, map, speed });
  }
  start() {
    const { pre, post } = this.door, L = Math.hypot(post[0] - pre[0], post[1] - pre[1]);
    Object.assign(this, { phase: "align", since: performance.now(), dwell: 0, blockedFor: 0, L, u: [(post[0] - pre[0]) / L, (post[1] - pre[1]) / L] });
    this.course = Math.atan2(this.u[1], this.u[0]);
  }
  update(ctl, dt, now) {
    const P = this.loc.pose(), { pre, post, mid } = this.door, err = wrapAngle(this.course - P.yaw), heading = ctl.est.heading - err;
    if (P.status === "lost") return lostDone;
    const z = this.map.floorAt(...mid) != null ? P.z : null, phaseT = (now - this.since) / 1000;
    const next = (phase) => Object.assign(this, { phase, since: now, dwell: 0 });
    if (this.phase === "align") {
      const off = Math.hypot(pre[0] - P.x, pre[1] - P.y);
      this.dwell = Math.abs(err) < 10 * DEG && off < 0.2 ? this.dwell + dt : 0;
      if (this.dwell > 0.3 || (phaseT > 6 && Math.abs(err) < 25 * DEG && off < 0.4)) next("check");
      else if (phaseT > 8) return { done: { ok: false, text: "Couldn't line up with the doorway." } };
      return { sp: { ...toward(P, pre, 0.8, 0.2), heading } };
    }
    if (this.phase === "check") {
      const sg = Math.min(P.sigma, this.loc.sigmaFloor?.(1000) ?? P.sigma), need = this.map.o.droneRadius + 2 * sg, fits = this.map.clearance(mid[0], mid[1], z) >= need; // the σ fixes keep, not a peak between two
      // too tight on the map as built, or something on it now (a closed door, a person): planned again around it
      if (!fits && staticClearance(this.map, mid[0], mid[1], z ?? this.map.floorAt(...mid) + 1) >= need) return { done: { ok: false, blocked: true, text: "Something is in the doorway." } };
      if (!fits) return { done: { ok: false, text: `The doorway is too tight for how sure I am of my position (±${(2 * sg).toFixed(2)} m).` } };
      this.dwell = ctl.est.ttc > 1.2 && !personAhead(ctl, 0.3) ? this.dwell + dt : 0;
      if (this.dwell > 0.2) next("cross");
      else if (phaseT > 4) return { done: { ok: false, blocked: true, text: "The doorway looks blocked." } };
      return { sp: { ...toward(P, pre, 0.8, 0.2), heading } };
    }
    const along = (P.x - pre[0]) * this.u[0] + (P.y - pre[1]) * this.u[1];
    if (along >= this.L - 0.1) return { done: { ok: true, text: "Through the doorway." } };
    if (phaseT > this.L / 0.08 + 5) return { done: { ok: false, text: "Took too long to get through the doorway." } };
    const k = Math.min(this.L, Math.max(0, along) + 0.4), aim = [pre[0] + this.u[0] * k, pre[1] + this.u[1] * k];
    if (ctl.est.ttc < 0.6 || personAhead(ctl, 0.45)) {
      if ((this.blockedFor += dt) > 4) return { done: { ok: false, blocked: true, text: "Something is in the doorway." } };
      return { sp: { heading } };
    }
    this.blockedFor = 0;
    return { sp: { ...toward(P, aim, 2.0, this.speed), heading } };
  }
}

function personAhead(ctl, h) {
  return ctl.perception.frameAge < 500 && ctl.perception.find("person", 0.4).some((d) => d.box.h > h && Math.abs(d.box.x + d.box.w / 2 - 0.5) < 0.25);
}

// Height of what the detector finds (m), for range from box size.
export const TALL = { person: 1.7, cat: 0.28, dog: 0.55, couch: 0.85, chair: 0.9, bed: 0.6, "dining table": 0.75 };

// Range (m) to a detection: from the feet line (box bottom on the floor, hc = camera height above that floor), else the
// head line (a person's known height), else the box height.
export function rangeOf(det, ctl, hc) {
  const lat = ctl.perception.latest, vfov = ctl.settings.get("hfov") * DEG * ((lat.height || 3) / (lat.width || 4));
  const tall = TALL[det.label] ?? 0.5, top = det.box.y, bottom = det.box.y + det.box.h, elB = ctl.elevationOf(bottom), elT = ctl.elevationOf(top);
  if (bottom < 0.97 && elB < -3 * DEG && hc > 0.1) return hc / Math.tan(-elB);
  if (top > 0.03 && det.label === "person" && elT > 3 * DEG && tall > hc + 0.1) return (tall - hc) / Math.tan(elT);
  return tall / (2 * Math.tan(Math.max(0.01, (det.box.h * vfov) / 2)));
}

// The nearest a detected person could be (m), for the standoff. With the feet in the picture, rangeOf(). Without them
// (the box runs off the bottom of the frame: close, sitting, lying, a child) the feet are nearer than where the frame's
// bottom edge meets the floor. The head line (a standing adult) counts only if the width it implies is an adult's
// (<= 0.65 m; then also at most what 0.45 m across implies); else (a seated person's head above a low drone, a child)
// the box is taken as narrow as anyone (0.3 m).
export function nearRange(det, ctl, hc) {
  const b = det.box;
  if (b.y + b.h < 0.97) return rangeOf(det, ctl, hc);
  const eb = ctl.elevationOf(1), elT = ctl.elevationOf(b.y), t = Math.tan((Math.min(1, b.w) * ctl.settings.get("hfov") * DEG) / 2);
  const head = b.y > 0.03 && elT > 3 * DEG && TALL.person > hc + 0.1 ? (TALL.person - hc) / Math.tan(elT) : Infinity;
  return Math.min(eb < -DEG && hc > 0.1 ? hc / Math.tan(-eb) : Infinity, 2 * head * t <= 0.65 ? Math.min(head, 0.45 / (2 * t)) : 0.3 / (2 * t));
}

// The best guess of a person's range (m), for where to remember them: rangeOf() with the feet in the picture; without
// them, the head line of a standing adult (else the width of one, 0.5 m), kept between nearRange() and where the frame's
// bottom edge meets the floor (the feet are nearer than that).
export function bestRange(det, ctl, hc) {
  const b = det.box;
  if (b.y + b.h < 0.97) return rangeOf(det, ctl, hc);
  const elT = ctl.elevationOf(b.y), t = Math.tan((Math.min(1, b.w) * ctl.settings.get("hfov") * DEG) / 2);
  const head = b.y > 0.03 && elT > 3 * DEG && TALL.person > hc + 0.1 ? (TALL.person - hc) / Math.tan(elT) : null;
  return clamp(head ?? 0.5 / (2 * t), nearRange(det, ctl, hc), edgeRange(det, ctl, hc));
}
// Where the frame's bottom edge meets the floor hc below the camera (m): a box cut by that edge stands nearer.
export const edgeRange = (det, ctl, hc) => ((eb) => (eb < -DEG && hc > 0.1 ? hc / Math.tan(-eb) : 10))(ctl.elevationOf(1));

// fn(det, ctl, hc) (rangeOf, nearRange, bestRange, edgeRange) against the floor where that range lands along yaw from P
// (rooms have their own floors: a person in a room a step down is farther than the drone's own floor says).
export function floorRange(fn, det, ctl, map, P, yaw) {
  const eye = P.z + 0.025;
  let floor = map.floorAt(P.x, P.y) ?? 0, r = fn(det, ctl, eye - floor);
  for (let i = 0; i < 2; i++) {
    const f = map.floorAt(P.x + r * Math.cos(yaw), P.y + r * Math.sin(yaw));
    if (f == null || Math.abs(f - floor) < 0.02) break;
    r = fn(det, ctl, eye - (floor = f));
  }
  return r;
}

// Where on the map a detection is: bearing from the box centre and the pose when the frame was taken (video is
// late), range from rangeOf() against the floor where it lands (rooms have their own floors; a box cut by the frame's
// bottom edge stands nearer than where that edge meets the floor: `cut`, a rougher sighting), cut at the first wall
// that every ray across the box meets. A wall far short of the range (under LOCATE_CUT of it) is one the camera sees
// through (an arch or an opening the map lacks): then the range stands, pulled back to the last point inside a room,
// rather than putting them in front of the drone.
const LOCATE_CUT = 0.35;
export function locate(det, { ctl, localizer, map, t = ctl.perception.latest.t }) {
  const P = localizer.poseAt(t - ctl.videoDelay), yaw = P.yaw - ctl.bearingOf(det.box.x + det.box.w / 2), low = det.box.y + det.box.h >= 0.97;
  const fn = !low ? rangeOf : (d, c, hc) => Math.min(rangeOf(d, c, hc), edgeRange(d, c, hc));
  let r = clamp(floorRange(fn, det, ctl, map, P, yaw), 0.2, 10);
  const along = (a) => { // the first wall along bearing a, and the last point inside a room within r
    let first = null, last = 0.2;
    for (let d = 0.1; d < r; d += map.o.cell / 2) {
      const k = map.idx(P.x + d * Math.cos(a), P.y + d * Math.sin(a));
      if (k >= 0 && map.room[k] >= 0 && !map.wall[k]) last = d;
      else first ??= Math.max(0.2, d - 0.1);
    }
    return first == null ? [r, r] : [first, last];
  };
  const rays = [yaw, ...[det.box.x, det.box.x + det.box.w].map((u) => P.yaw - ctl.bearingOf(u))].map(along);
  const cut = Math.max(...rays.map((q) => q[0]));
  r = cut >= LOCATE_CUT * r ? cut : along(yaw)[1];
  const x = P.x + r * Math.cos(yaw), y = P.y + r * Math.sin(yaw);
  return { label: det.label, score: det.score, trackId: det.trackId, box: det.box, x, y, range: r, cut: low, room: map.roomAt(x, y)?.id ?? null, t };
}

// Look around at a spot { x, y, headings: [H yaw] } (planner viewpoints): hold the spot, face each heading, let the
// late video catch up, then watch for `look` s. Every detection of `labels` becomes a map sighting (locate()); one
// snapshot per heading. onSighting(sighting) returning true ends the scan early (a confirmed find). A heading counts as
// looked at (frames[i].looked) only with SCAN_FRAMES fresh frames from the person detector while it worked (ground
// truth, or loaded and not failed); else data.why says what was wrong (the fallback detector is said too). With vision
// the position source, each turn ends with a dwell until a fix lands (SCAN.fixAge, at most SCAN.settle ms): the turn
// starves the fixes (safety.js slows it too), the next one mustn't start on a stale position.
export const SCAN_FRAMES = 4;
// With vision, turns in steps of at most `step`, a dwell after each until a fix under fixAge ms old (at most `settle` ms; no
// fix by then: the camera can't place itself facing that way, a blank wall, glare: that heading is skipped, not turned
// further into). dwell, look: s a dwell and a heading's look take, typically (missions' scan budget).
export const SCAN = { fixAge: 300, settle: 1500, step: 35 * DEG, dwell: 0.35, look: 0.8 };
function detecting(per) {
  const d = per.detector;
  if (per.source?.detections || !d) return { ok: true };
  if (per.detectorFailed) return { ok: false, why: "the person detector failed to load" };
  if (!d.ready) return { ok: false, why: "the person detector wasn't running yet" };
  return { ok: true, note: d.backend === "mediapipe" ? "with the fallback detector, which misses more" : "" };
}
export class ScanAt {
  constructor(spot, { localizer, map, labels = ["person", "cat", "dog"], look = 0.8, minScore = 0.4, onSighting = null, snapshots = true, label = "Looking around" } = {}) {
    Object.assign(this, { name: "scan", label, spot, loc: localizer, map, labels, look, minScore, onSighting, snapshots });
  }
  start() {
    Object.assign(this, { i: 0, phase: "turn", since: performance.now(), frames: [], sightings: [], seen: 0, good: 0, why: null, vision: !!this.loc.visionActive?.() });
  }
  update(ctl, dt, now) {
    const P = this.loc.pose(), hs = this.spot.headings;
    if (P.status === "lost") return lostDone;
    const finish = (early) => ({ done: { ok: true, text: early ? "Found it." : `Looked around in ${hs.length} directions.`, data: { frames: this.frames, sightings: this.sightings, early, why: this.why } } });
    if (this.i >= hs.length) return finish(false);
    // after a heading skipped for want of a fix, turns go the other way round (not back across what the camera can't place)
    let err = wrapAngle(hs[this.i] - P.yaw);
    if (this.away && Math.sign(err) === this.away && Math.abs(err) > 8 * DEG) err -= 2 * Math.PI * this.away;
    const sp = { ...toward(P, [this.spot.x, this.spot.y], 0.8, 0.25), heading: ctl.est.heading - err };
    if ((now - this.t0) / 1000 > 6 + hs.length * (this.vision ? 9 : 5)) return { done: { ok: false, text: "The look-around timed out.", data: { frames: this.frames, sightings: this.sightings } } };
    if (this.phase === "turn") {
      if (this.vision && Math.abs(err) > SCAN.step + 8 * DEG) { // a step of the turn, then a dwell for a fix
        this.via ??= wrapAngle(P.yaw + Math.sign(err) * SCAN.step);
        this.turning = Math.sign(err);
        const e2 = wrapAngle(this.via - P.yaw);
        if (Math.abs(e2) < 8 * DEG) Object.assign(this, { phase: "settle", since: now, via: null, mid: true });
        return { sp: { ...sp, heading: ctl.est.heading - e2 } };
      }
      // (a step's target left over from the turn here would steer the next turn toward it: into the heading just skipped)
      if (Math.abs(err) < 8 * DEG) Object.assign(this, { phase: this.vision ? "settle" : "look", since: now, mid: false, via: null });
      return { sp };
    }
    if (this.phase === "settle") {
      if (!(this.loc.fixQuality?.(now).visionAge > SCAN.fixAge)) Object.assign(this, { phase: this.mid ? "turn" : "look", since: now, mid: false });
      else if (now - this.since > SCAN.settle) { // no fix facing this way: skip the heading (turning on would face further into it)
        this.why ??= "the camera couldn't place itself facing some ways";
        this.frames.push({ yaw: hs[this.i], image: null, looked: false, detections: [] });
        Object.assign(this, { i: this.i + 1, phase: "turn", since: now, mid: false, good: 0, away: this.turning || this.away, via: null });
      }
      return { sp };
    }
    const waited = now - this.since - 300 - ctl.videoDelay, lat = ctl.perception.latest;
    if (waited > 0 && lat.t !== this.seen && lat.t - ctl.videoDelay > this.since) {
      this.seen = lat.t;
      const det = detecting(ctl.perception);
      if (det.ok && ctl.perception.frameAge < 400) this.good++;
      else if (!det.ok) this.why = det.why;
      if (det.note) this.note = det.note;
      for (const d of lat.detections) {
        if (!this.labels.includes(d.label) || d.score < this.minScore) continue;
        const s = locate(d, { ctl, localizer: this.loc, map: this.map, t: lat.t });
        this.sightings.push(s);
        if (this.onSighting?.(s, ctl)) return finish(true);
      }
    }
    if (waited > this.look * 1000) {
      const looked = this.good >= SCAN_FRAMES;
      if (!looked) this.why ??= "the video stalled";
      this.frames.push({ yaw: hs[this.i], image: this.snapshots ? ctl.perception.snapshot({ maxWidth: 320, quality: 0.7 }) : null, looked, ...(this.note && { note: this.note }),
        detections: lat.detections.map((d) => ({ label: d.label, score: d.score, box: d.box })) });
      Object.assign(this, { i: this.i + 1, phase: "turn", since: now, good: 0 });
    }
    return { sp };
  }
}

// A braking measurement (missions.js "calibrate"): along H yaw `yaw`, `speed` m/s for `run` m, then a stop (velocity 0:
// the controller's own braking), watching the localizer's position and velocity along the way. -> data { lag (s from the
// stop command to 10% slower: the video's delay is in it, as the velocity comes from the video), v0 (m/s at the command),
// travel (m from the command to standstill, as the position estimate saw it), decel (m/s²: the braking that, after lag at
// v0, stops in that travel: nav/avoid.js stopDistance(v0, lag, decel) = travel) }.
export class BrakeTest {
  constructor({ localizer, yaw, speed = 0.3, run = 2 }) {
    Object.assign(this, { name: "brake", label: "Measuring my braking", loc: localizer, yaw, speed, run });
  }
  start() {
    Object.assign(this, { from: this.loc.pose(), phase: "go", trace: [] });
  }
  update(ctl, dt, now) {
    const P = this.loc.pose(), u = [Math.cos(this.yaw), Math.sin(this.yaw)], v = this.loc.velocity(150), along = v[0] * u[0] + v[1] * u[1], t = (now - this.t0) / 1000;
    if (P.status === "lost") return lostDone;
    const heading = ctl.est.heading - wrapAngle(this.yaw - P.yaw);
    if (this.phase === "go") {
      const gone = (P.x - this.from.x) * u[0] + (P.y - this.from.y) * u[1];
      const last = [...this.trace.filter((q) => q.t > now - 300).map((q) => q.v), along]; // the speed at the stop command (its mean over 0.3 s)
      if (gone >= this.run || t > 12) Object.assign(this, { phase: "stop", stopAt: now, v0: last.reduce((a, b) => a + b, 0) / last.length });
      this.trace.push({ t: now, v: along, s: gone });
      return { sp: { vx: this.speed, vy: 0, heading } };
    }
    this.trace.push({ t: now, v: along, s: (P.x - this.from.x) * u[0] + (P.y - this.from.y) * u[1] });
    if (now - this.stopAt < 3000) return { sp: { vx: 0, vy: 0, heading } };
    const after = this.trace.filter((q) => q.t >= this.stopAt), v0 = this.v0, q90 = after.find((q) => q.v < 0.9 * v0), q10 = after.find((q) => q.v < 0.1 * v0);
    if (!(v0 > 0.15) || !q90 || !q10 || q10.t <= q90.t) return { done: { ok: false, text: `the run was too slow or the stop didn't show (${v0.toFixed(2)} m/s).` } };
    const lag = (q90.t - this.stopAt) / 1000, travel = Math.max(...after.map((q) => q.s)) - after[0].s, decel = (v0 * v0) / (2 * Math.max(0.02, travel - v0 * lag));
    return { done: { ok: true, text: "Measured.", data: { decel, lag, v0, travel } } };
  }
}

// Plan home from where we are and fly it; in full auto land on the pad (holding over it), in co-pilot hover there and
// ask the pilot to land. A safety takeover when the battery runs low, and the last step of every mission. It doesn't give
// up while the battery allows: a blocked way (a person, something new) or no plan is tried again after a wait (2 s, then
// 2 s longer each time, up to 10 s) with a wider berth where there is room for one (σ + 0.05 a try, up to 0.45; else the
// usual σ); once the battery (battery(): seconds
// of flying left) only covers landing nearby, or after `patience` s of trying without a battery reading, it flies to the
// nearest safe spot (open floor, known free, away from temporary obstacles) and lands there (co-pilot: asks the pilot to),
// saying why (blocker(): what blocks the way in words, if known). say(text): progress for the mission log. While it
// waits it faces what blocks the way (lookAt(): [x, y]), so the change detector can see a false change plainly and drop
// it. passable(): ids of suspected changes it may pass (Claude called them no change, surely): with nothing else in the
// way and the drone able to see (avoid.eyes()), it plans through them and flies past slowly (avoid.pass: a creep near
// them, the live depth's caps still on). closer(): what only the live depth says blocks the way (a doorway it sees closed,
// a wedge it sees ahead: missions.js closer()) { id, spot, face, text, seconds }: before a wait, it flies to spot (about 1 m
// in front of it, at passVmax) and faces it for `seconds` (a false one is seen through and goes), then plans again; each
// at most twice, and a look isn't a try. The trip is budgeted at the speed it really flies (nav/avoid.js cruise()).
export const HOME_TRY = { wait: 2, waitMax: 10, berth: 0.05, sigmaMax: 0.45, landTime: 8, budget: 1.3, reserve: 10, passVmax: 0.25, relook: 2 };
export class ReturnHome {
  // sigma: m, or a function of now (the planning σ as the position estimate improves or worsens: one frozen at a bad
  // moment can leave no way home through a doorway for good)
  // comfy: () => σ for a first plan kept out of the safety layer's buffer (missions.comfySigma()), if it reaches the pad
  constructor({ localizer, map, home, land = true, alt = 1.0, sigma = 0.15, vmax = 0.5, label = "Flying home", battery = null, patience = 90, say = null, blocker = null, lookAt = null, passable = null, comfy = null, closer = null }) {
    Object.assign(this, { name: "home", label, loc: localizer, map, home, land, alt, sigmaOf: typeof sigma === "function" ? sigma : () => sigma, vmax, battery, patience, say, blocker, lookAt, passable, comfy, closer });
    this.passed = [];
  }
  get sigma() {
    return this.sigmaOf();
  }
  start(ctl) {
    Object.assign(this, { tries: 0, why: "", phase: "fly", child: null, since: performance.now(), looked: new Map() });
    this.go(ctl, this.since);
  }
  stop() {
    this.unpass();
  }
  unpass() {
    for (const id of this.passed) this.avoid?.unpass(id);
    this.passed = [];
  }
  // Plan home (wider each try, but never to a goal moved off the pad) and follow it, or wait. Blocked only by suspected
  // changes it may pass (or inside one: the plan escapes it, the safety layer would not let it): through them, slowly.
  go(ctl, now) {
    this.unpass();
    const P = this.loc.pose(), wide = Math.min(HOME_TRY.sigmaMax, this.sigma + HOME_TRY.berth * this.tries), climb = ctl.autonomy === "full", to = (m, sigma) => plan(m, [P.x, P.y], [this.home.x, this.home.y], { alt: this.alt, sigma, climb });
    const off = (q) => !q.ok || Math.hypot(q.to[0] - this.home.x, q.to[1] - this.home.y) > 0.25;
    let sigma = Math.max(wide, this.comfy?.() ?? 0), map = this.map, p = to(map, sigma);
    if (off(p) && sigma > wide) p = to(map, (sigma = wide));
    if (off(p) && wide > this.sigma) p = to(map, (sigma = this.sigma));
    const inside = [...(map.temps?.values() ?? [])].some((t) => t.kind === "change" && (t.polygon ? polyDist(P.x, P.y, t.polygon) : Math.hypot(P.x - t.x, P.y - t.y) - t.r) < 0);
    const avoid = (this.avoid = ctl.safety?.avoid), ids = (!p.ok || inside) && avoid?.eyes().ok ? this.passable?.() ?? [] : [];
    if (ids.length && (p = to((map = withoutTemps(this.map, ids)), sigma)).ok) {
      for (const id of ids) avoid.pass(id);
      this.passed = ids;
      this.say?.("The way home is blocked by something Claude saw no change in: passing it slowly, watching with live depth.");
    }
    if (!p.ok) return this.blocked(p.reason, ctl, now);
    if (Math.hypot(p.to[0] - this.home.x, p.to[1] - this.home.y) > 0.25) return this.wait("something is on the home pad", now); // the plan's goal moved off it
    this.child = this.started(new PathFollow(p.path, { localizer: this.loc, map, vmax: this.passed.length ? HOME_TRY.passVmax : this.vmax, goalTol: 0.12, doors: p.doors, need: this.map.lethal(sigma), label: this.label }), ctl, now);
    this.phase = "fly";
  }
  // Blocked: first a closer look at what only the live depth says is in the way (each thing at most twice; not a try), else a wait.
  blocked(why, ctl, now) {
    const c = this.closer?.(), n = c ? this.looked.get(c.id) ?? 0 : 2;
    if (n >= 2) return this.wait(why, now);
    this.looked.set(c.id, n + 1);
    Object.assign(this, { phase: "look", peek: c });
    this.say?.(`The way home is blocked (${c.text.replace(/^(?!I\b)./, (q) => q.toLowerCase())}): taking a closer look${c.spot ? " from in front of it" : ""}.`);
    const P = this.loc.pose(), p = c.spot && plan(this.map, [P.x, P.y], c.spot, { alt: this.alt, sigma: this.sigma, climb: ctl.autonomy === "full" });
    this.child = this.started(p?.ok && p.length > 0.2 ? new PathFollow(p.path, { localizer: this.loc, map: this.map, vmax: HOME_TRY.passVmax, goalTol: 0.15, doors: p.doors, need: this.map.lethal(this.sigma), label: "Flying in front of it for a closer look" }) : this.facer(c), ctl, now);
  }
  facer(c) {
    const P = this.loc.pose();
    return new ScanAt({ x: P.x, y: P.y, headings: [Math.atan2(c.face[1] - P.y, c.face[0] - P.x)] }, { localizer: this.loc, map: this.map, labels: [], look: c.seconds ?? HOME_TRY.relook, snapshots: false, label: "Taking a closer look" });
  }
  // Wait (2 s, longer each try), facing what blocks the way if it is known.
  wait(why, now) {
    this.why = this.blocker?.()?.replace(/^(?!I\b)./, (c) => c.toLowerCase()) ?? why;
    this.phase = "wait";
    this.until = now + 1000 * Math.min(HOME_TRY.waitMax, HOME_TRY.wait * (1 + this.tries++));
    const at = this.lookAt?.(), P = this.loc.pose();
    this.facing = at && P.status !== "lost" ? { yaw: Math.atan2(at[1] - P.y, at[0] - P.x) } : null;
    this.say?.(`The way home is blocked (${this.why}); ${this.facing ? "looking at it, then " : ""}trying again with more room.`);
  }
  waitSp(ctl) {
    const P = this.loc.pose();
    return { sp: { heading: this.facing && P.status !== "lost" ? ctl.est.heading - wrapAngle(this.facing.yaw - P.yaw) : ctl.est.heading } };
  }
  started(b, ctl, now) {
    b.start(ctl);
    b.t0 = now;
    return b;
  }
  // Out of time to keep trying: the battery covers the trip home (or, while it's blocked, 1.5 x the straight line, at the
  // speed it really flies) only with the margin, or patience is up without a battery reading.
  outOfTime(ctl, now) {
    const P = this.loc.pose(), left = this.battery?.();
    if (left == null) return this.tries > 0 && (now - this.since) / 1000 > this.patience;
    const home = (1.5 * Math.hypot(this.home.x - P.x, this.home.y - P.y)) / cruiseSpeed(ctl, now) + HOME_TRY.landTime;
    return this.tries > 0 && left < HOME_TRY.budget * home + HOME_TRY.reserve;
  }
  // The nearest safe spot, then down there.
  settle(ctl, now) {
    const P = this.loc.pose(), spot = safeSpot(this.map, P, { sigma: this.sigma });
    this.phase = "spot";
    if (!spot) return this.landHere(ctl, now, "there's no clear spot nearby either");
    this.spot = spot;
    this.say?.(`I can't get home (${this.why}), so I'm landing on a clear spot nearby.`);
    this.child = this.started(new PathFollow(spot.path.path, { localizer: this.loc, map: this.map, vmax: 0.35, goalTol: 0.12, need: this.map.lethal(this.sigma), label: "Flying to a clear spot to land" }), ctl, now);
  }
  landHere(ctl, now, more = "") {
    const P = this.loc.pose();
    this.phase = "spotLand";
    this.landedAt = { x: P.x, y: P.y, more };
    if (ctl.autonomy !== "full") {
      ctl.askPilot("land", "I can't get home. Please land here.");
      return { done: { ok: false, safeLanded: true, text: `I couldn't get home (${this.why}), so I asked the pilot to land where I am${more ? `; ${more}` : ""}.` } };
    }
    // the battery decides now: on whatever is below if need be (slowly)
    this.child = this.started(new Land({ at: [P.x, P.y], localizer: this.loc, label: "Landing: I can't get home", onto: true }), ctl, now);
    return null;
  }
  update(ctl, dt, now) {
    if (this.phase === "wait") {
      if (this.outOfTime(ctl, now)) return this.settle(ctl, now) ?? { sp: { heading: ctl.est.heading } };
      if (now >= this.until) this.go(ctl, now);
      return this.phase === "wait" ? this.waitSp(ctl) : { sp: { heading: ctl.est.heading } };
    }
    const r = this.child.update(ctl, dt, now);
    if (!r.done) return r;
    if (r.done.lost) return r;
    const next = () => (this.phase === "wait" ? this.waitSp(ctl) : { sp: { heading: ctl.est.heading } });
    if (this.phase === "look") { // in front of it (or not: then from where it got to), the look; then plan again
      if (this.child.name === "path") return (this.child = this.started(this.facer(this.peek), ctl, now)), next();
      return this.go(ctl, now), next();
    }
    if (this.phase === "fly") {
      this.unpass();
      if (!r.done.ok) {
        if (this.outOfTime(ctl, now)) return this.settle(ctl, now) ?? { sp: { heading: ctl.est.heading } };
        this.blocked(r.done.text.replace(/\.$/, "").replace(/^(?!I\b)./, (c) => c.toLowerCase()), ctl, now); // "I was pushed off the way" keeps its I
        return next();
      }
      if (!this.land) return { done: { ok: true, text: "Back at the home pad." } };
      if (ctl.autonomy !== "full") {
        ctl.askPilot("land", "We're over the home pad. Please land.");
        return { done: { ok: true, text: "Over the home pad; asked the pilot to land." } };
      }
      this.phase = "land";
      this.child = this.started(new Land({ at: [this.home.x, this.home.y], localizer: this.loc, label: "Landing on the home pad" }), ctl, now);
      return { sp: { heading: ctl.est.heading } };
    }
    if (this.phase === "land" && !r.done.ok) { // something on the home pad after all (the descent was held over it)
      this.why = "something is on the home pad";
      return this.settle(ctl, now) ?? { sp: { heading: ctl.est.heading } };
    }
    if (this.phase === "spot") {
      if (!r.done.ok) return this.landHere(ctl, now, "the clear spot couldn't be reached") ?? { sp: { heading: ctl.est.heading } };
      return this.landHere(ctl, now) ?? { sp: { heading: ctl.est.heading } };
    }
    if (this.phase === "spotLand") {
      const room = this.map.roomAt(this.landedAt.x, this.landedAt.y)?.name, at = room ? ` in ${/^(room|bedroom|bathroom)\s*\d/i.test(room) ? room : `the ${room}`}` : "", more = this.landedAt.more;
      return { done: { ok: false, safeLanded: true, text: `I couldn't get home (${this.why}), so I landed${more ? ` where I was${at}: ${more}` : `${at} on a clear spot instead`}.` } };
    }
    return { done: { ok: true, text: "Landed on the home pad." } };
  }
}

const center = (b) => [b.x + b.w / 2, b.y + b.h / 2];
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
const blend = (a, b, k) => ({ x: a.x + (b.x - a.x) * k, y: a.y + (b.y - a.y) * k, w: a.w + (b.w - a.w) * k, h: a.h + (b.h - a.h) * k });
