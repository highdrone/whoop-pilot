// Safety layer between the behaviors and the sticks (docs/HOME-DRONE.md). The controller passes every setpoint through
// filter() and flies `takeover` (a behavior) instead of the mission when this layer needs the drone:
// - geofence: the pose ± 2σ must be free on the map (map.free with the drone radius + 2σ). Velocity toward an
//   obstacle is cut when the drone is about to come closer than that plus a 10 cm buffer, and inside the buffer it is
//   pushed back out along the clearance gradient, proportionally and with an integral that leaks away once clear
//   (positions from the localizer, so this also beats a steady bias in the flow the controller holds position by).
//   A pose off the rooms (in a wall, out through a doorway) is pushed hardest, toward the nearest free cell. Static things
//   (walls, furniture, unknown space) push hard; temporary ones (what live depth found, suspected changes) softly, never
//   toward a static one the drone is near. The push comes after the speed caps, so nothing looking ahead (a blocked way, an
//   image looming) can cancel it: a hover drifting toward a wall while return home waits is pushed off it.
// - height (full auto): the controller holds the localizer's height and climb rate (height(): vision fixes and the
//   throttle's physics keep them) while its σz is under zLead, else its own (dead reckoned from the flow's climb rate, which
//   misreads); while a behavior holds height the safety layer gives it the height to hold (sp.z), at least 0.5 m over the
//   floor and no higher than the map's top band. On its own height, the safety layer climbs back when the localizer's
//   height sinks under the height held (a hover sank onto a cabinet), and well over it (a scan climbed 1.1 m into a door
//   head) brings the drone back down. Over every band the map has (it knows nothing up there), the drone holds still and
//   comes down, with no geofence push (its clearances would be a lower height's).
// - vision fixes (the position from the camera, nav/localizer.js visionActive()): turns slow down so tracking keeps up;
//   fixes too sparse (over 1 s old, under 2 a second): at most 0.15 m/s; none for 2.5 s: hold position, turned back to
//   where the camera last placed itself (facing on into a blank wall, none would come), or, the last fix facing this way
//   too (the scene there changed, the height estimate drifted), toward the most open side; no fix 1 s after facing there
//   (at once with no turn back to make): the vision side is asked to relocalize now (localizer.askFix()), not at "lost".
//   Until one lands, or the position is lost. The real drone's video under 12 frames a second (a machine too busy): a
//   hold after 2 s, a landing after 5 s.
// - localization lost (σ > 0.5 m, or no vision fix for 5 s): hold; full auto lands in place after 3 s, co-pilot asks the
//   pilot to take over; found again before that (full auto): home.
// - battery: sag-compensated voltage or time left against the trip home (x1.3 + 10 s) flies home and lands; 3.3 V under
//   load lands in place (co-pilot: asks the pilot).
// - people: hold position (backing away is allowed) while a person's box fills more than half the frame or they may be
//   closer than 1.5 m (plus the stopping distance; nearRange(), the nearest they could be: sitting, lying and children
//   too; 1.0 m from someone seated or lying still while the drone creeps, which it does within 1.5 m of them). Where they
//   probably are (bestRange(), on the floor of the room the range lands in), once seen on 2 frames in a row (a one-frame
//   false detection never closes a doorway), becomes a temporary no-fly zone of 1.5 m (seated or lying still: 1.0 m) on
//   the map (avoid.addTemp), which the planner routes around and the drone only backs out of, for 5 s after the last
//   sighting (25 s if they dropped out of the bottom of the frame), until they are seen elsewhere, or until the camera
//   sees that spot in plain view with nobody there. Someone walking gets a zone along their way for the next
//   3 s (where they will be). Someone inside the standoff (they came closer) makes the drone back away gently.
// - obstacles (nav/avoid.js): the speed toward anything on the map, a temporary obstacle, unknown space or live depth
//   is capped so the drone can stop in time (climbing and sinking too); something the camera sees ahead that the scan
//   lacks and the flow's time to contact slow it too. On the real drone it flies slowly while it can't see what the scan
//   lacks (avoid.eyes(): reason "flying slowly: ...").
// - the picture (real drone): a video frame that isn't the camera's picture (black bars, a 16:9 crop of the 4:3 O4) gives
//   wrong bearings and ranges to people: said once, and every person in the picture then holds the drone.
// - tick watchdog: a tick more than 200 ms late (a throttled tab) means hovering for a second.
// The radio script's own failsafe (sticks back, slow descent after 0.3 s without packets) stays the last line of
// defence. This never arms or disarms.
import { Emitter, clamp, wrapAngle, DEG } from "./util.js";
import { MASK } from "./protocol.js";
import { Hold, Land, ReturnHome, toBody, nearRange, bestRange, edgeRange, floorRange } from "./behaviors.js";
import { plan, nearestFree } from "./house/planner.js";
import { Avoid, staticClearance, staticView, blockers, polyDist, freeRun } from "./nav/avoid.js";
import { pictureProblem } from "./vision/depth.js";

export const SAFETY = {
  buffer: 0.1, lookahead: 0.7, pushGain: 3, pushI: 3, pushLeak: 1.5, pushMax: 0.5,
  pushTravel: 0.5, // m a push may carry the drone from where it began without getting it out of the buffer: then it stops
  // (it was sliding along furniture under the band flown into a corner: 1.5 m at 0.5 m/s, a contact at 0.47 m/s, seed 12)
  holdLost: 3, // s before landing in place (full auto)
  homeVolts: 3.7, landVolts: 3.3, landAfter: 1.5, budget: 1.3, reserve: 10, landTime: 8, // the trip home at avoid.cruise()
  standoff: 1.5, personBox: 0.5, faintBox: 0.15, personMemory: 5, personLost: 25, personSee: 6, personJoin: 1.2, jitter: 200, jitterHover: 1000,
  seated: 1.0, seatedAspect: 1.8, seatedTall: 1.35, creepNear: 0.12, // someone seated or lying still (a whole box this much taller
  // than wide at most, and no taller than seatedTall m where its range is known): their zone and, while the drone creeps (it
  // does within the standoff of them), its hold; m/s. A standing adult's box is 2.4-3.2 times as tall as wide (to verify on
  // the first real flights)
  zoneFrames: 2, // frames in a row a person is seen on before they are remembered on the map
  fixSlow: 1000, fixRate: 2, fixHold: 2500, sparse: 0.15, // vision fixes: older (ms) or rarer (per s): at most `sparse` m/s; older: hold
  pushFresh: 400, pushStale: 0.2, // the geofence's push with the last vision fix older (ms): at most this (m/s), down to none at
  // fixHold: a push on a pose the flow can't follow (a turn) flew a drone 0.6 m at 0.56 m/s off a wall only its estimate was
  // near, into a cabinet; one on a pose a metre off steers by nothing
  backFor: 2000, backPast: 15 * DEG, // ms of turning back to where the camera last placed itself (and rad past it) the lost clock
  // allows on top of its 5 s
  askAfter: 1000, // ms facing where the hold turned with no fix before the vision side is asked to relocalize
  askSigma: 0.15, // m: an estimate this unsure as the hold begins has drifted past what tracking finds: asked at once, the turn back too
  paceFps: 12, paceHold: 2000, paceLand: 5000, // the real drone: video under this many frames a second for paceHold ms holds still,
  // for paceLand ms lands in place (full auto; co-pilot: asks the pilot)
  yawVision: 0.6, yawStale: 0.3, yawStaleAge: 600, // rad/s turns with vision the position source; slower once a fix is yawStaleAge ms old
  zTrust: 0.15, zGain: 0.8, zI: 1.0, zMax: 0.25, zBand: 0.08, // a sink is caught on the localizer's height while its σz is under
  // zTrust (m), once more than zBand m under the height held; gains, m/s
  zLead: 0.3, zRate: 2, // m, fixes a second: the controller's height hold follows the localizer's height (height()) while its
  // σz is under zLead and fixes come at zRate or more (5 s average): with the throttle's physics and vision fixes it is better
  // than the flow's (seed 11 replayed: 0.06 m p95 against 0.22 m; the controller's own drifted 0.2-1.1 m on the flow's climb
  // rate) and a pause widens it slowly; on sparse ones the flow's, with its gentler hold (1 a second, ±0.1 m: the drone
  // bobbed 0.3 m chasing them)
  zOver: 0.12, zDown: 0.3, // m over the height held from which it is brought back down (with the integral too), at most this fast (m/s)
  minHover: 0.5, // m over the floor a held hover keeps at least (a take-off the controller thought reached 1 m, really 0.3 m)
  bearing: 6 * DEG, // beyond a box's half-width: a sighting along the bearing of a remembered spot can be them
  scaleFor: 10000, scaleMargin: 0.85, // ms a whole view's range and box width scale a cut box's range; the share of it the hold uses
  clearFor: 600, clearFrames: 3, // ms and frames a remembered spot must be in plain view with nobody there before it goes
  walk: 0.35, walkAhead: 3, // m/s from which a person counts as walking; s of their way ahead their zone covers
  backOff: 0.35, backFrom: 0.4, // m/s at most when backing away from someone inside the standoff + backFrom m
};

// 1S LiPo/LiHV resting voltage -> charge (rough), and the sag of a 65 mm whoop at full thrust (~16 A x 50 mΩ).
const SOC = [[3.3, 0], [3.5, 0.05], [3.6, 0.1], [3.65, 0.15], [3.7, 0.25], [3.75, 0.35], [3.8, 0.45], [3.85, 0.55], [3.95, 0.7], [4.05, 0.85], [4.2, 1]];
const SAG = 0.8;
const interp = (v, t) => (v <= t[0][0] ? t[0][1] : v >= t.at(-1)[0] ? t.at(-1)[1] : ((i) => t[i - 1][1] + ((v - t[i - 1][0]) / (t[i][0] - t[i - 1][0])) * (t[i][1] - t[i - 1][1]))(t.findIndex((p) => p[0] >= v)));

// Throttle the motors get: ours in full auto, the pilot's otherwise.
const throttle = (ctl) => (ctl.mask & MASK.thr ? ctl.out.thr : ctl.tel?.sticks?.thr ?? 0);

export class Safety extends Emitter {
  constructor({ map, localizer, settings, house = null, alerts = null }) {
    super();
    Object.assign(this, { map, loc: localizer, ctl: localizer.ctl, settings, house, alerts });
    this.takeover = null;
    this.reason = "";
    this.vComp = null;
    this.lowSince = null;
    this.lostSince = null;
    this.hoverUntil = -Infinity;
    this.person = null; // the nearest person seen within reach: { x, y (H, nearest plausible), yaw (seen at), t }
    this.avoid = new Avoid({ map, localizer, settings }); // speed caps and temporary obstacles (people too)
    this.personIds = 0;
    this.personHeld = false; // this tick's setpoint was cut for a person (PathFollow gives up after a while)
    this.floor = null; // the last floor under the pose
    this.pushed = [0, 0]; // the geofence push's integral (H frame, m/s)
    this.homeTrip = { t: -Infinity, s: Infinity };
    this.zHold = null; // the height held (full auto), m
    this.fixHold = null; // the heading held while waiting for a vision fix
    this.seenTracks = new Map(); // a person's track (or bearing) -> { n frames in a row, t }
    this.tripped = new Set(); // once per flight
    this.homeOptions = null; // () => more ReturnHome options for SafeHome (the mission runner's planning: missions.js)
    this.loc.on("pose", (p) => this.watch(p));
  }

  setMap(map, house = this.house) {
    Object.assign(this, { map, house });
    this.avoid.setMap(map);
  }

  status() {
    return { ok: !this.reason && !this.takeover, reason: this.takeover?.reason ?? this.reason };
  }

  // Battery: { vbat (V, as measured), vComp (V, sag-compensated, smoothed), soc (0-1, rough), secondsLeft (to 10%) }.
  battery() {
    const v = this.ctl.tel?.vbat;
    if (!(v > 2.5)) return null;
    const soc = interp(this.vComp ?? v, SOC);
    return { vbat: v, vComp: this.vComp ?? v, soc, secondsLeft: Math.max(0, soc - 0.1) * (Number(this.settings?.get?.("flightSeconds")) || 240) };
  }

  // Seconds to fly home and land from p (planner length at the speed the drone really flies: avoid.cruise()), refreshed
  // once a second.
  timeHome(p, now) {
    const h = this.house?.home;
    if (!h || !this.map) return 0;
    if (now - this.homeTrip.t > 1000) {
      const r = plan(this.map, [p.x, p.y], [h.x, h.y], { sigma: 0.15, climb: false }), v = this.avoid.cruise(now).v;
      this.homeTrip = { t: now, v, s: (r.ok ? r.length : Math.hypot(h.x - p.x, h.y - p.y) * 1.5) / v + SAFETY.landTime };
    }
    return this.homeTrip.s;
  }

  trip(kind, reason, takeover = null) {
    if (takeover) this.takeover = Object.assign(takeover, { reason, kind, safety: this });
    if (this.tripped.has(kind)) return;
    this.tripped.add(kind);
    this.emit("trip", { kind, reason, action: takeover?.name ?? "warn" });
    this.alerts?.notify({ text: reason[0].toUpperCase() + reason.slice(1) + ".", urgency: "high", kind: `safety-${kind}`, pose: this.loc.pose() });
  }

  // Every tick (the localizer's "pose"): battery and localization, which need action even without a mission.
  watch(p) {
    const ctl = this.ctl, now = performance.now(), full = ctl.autonomy === "full", v = ctl.tel?.vbat;
    this.avoid.expire(now);
    if (v > 2.5) {
      const comp = v + SAG * Math.max(0, throttle(ctl)) ** 1.6;
      this.vComp = this.vComp == null ? comp : this.vComp + (comp - this.vComp) * 0.02;
    }
    if (ctl.safety !== this || !ctl.isFlying()) { // detached (demo apartment, no position yet) acts like the ground
      if (this.takeover && !this.takeover.landing) this.takeover = null;
      this.tripped.clear();
      Object.assign(this, { lostSince: null, lowSince: null, reason: "", picture: "", slowSince: null, paced: false });
      this.avoid.flown = null; // the speed flown is per flight
      return;
    }
    const real = this.settings?.get?.("mode") === "real", per = ctl.perception;
    this.picture = real ? pictureProblem(per) : "";
    if (this.picture) this.trip("picture", `${this.picture}, so I can't tell how far people are`);
    // the real drone's video too slow to fly by (a machine too busy): hold, then land in place
    const slow = real && !!per && (!(per.frameAge < 500) || per.fps < SAFETY.paceFps);
    this.slowSince = slow ? this.slowSince ?? now : null;
    this.paced = slow && now - this.slowSince > SAFETY.paceHold;
    if (slow && now - this.slowSince > SAFETY.paceLand) { // co-pilot: the pilot lands it, so the alert says so
      const fps = `${per.fps} frames a second`;
      if (full) this.trip("pace-land", `the video is too slow to fly by (${fps}), so I'm landing where I am`, this.takeover?.landing ? this.takeover : new SafeLand());
      else if (!this.tripped.has("pace-land")) (ctl.askPilot("land", "The video is too slow for me to fly by. Please take over and land."), this.trip("pace-land", `the video is too slow for me to fly by (${fps}): please take over and land`));
    }
    if (v > 2.5) {
      this.lowSince = v <= SAFETY.landVolts ? (this.lowSince ?? now) : null;
      const b = this.battery();
      if (this.lowSince != null && now - this.lowSince > SAFETY.landAfter * 1000) {
        if (full) this.trip("battery-land", "the battery is empty, landing now", this.takeover?.landing ? this.takeover : new SafeLand());
        else if (!this.tripped.has("battery-land")) ctl.askPilot("land", "The battery is empty. Land now.");
        this.tripped.add("battery-land");
      } else if (this.house?.home && p.status !== "lost" && !this.tripped.has("battery-home") && !["land", "home"].includes(ctl.behavior?.name)) { // landing, or going home already
        const home = this.timeHome(p, now), short = b.secondsLeft < SAFETY.budget * home + SAFETY.reserve;
        if (b.vComp <= SAFETY.homeVolts || short)
          this.trip("battery-home", `the battery is low (${b.vComp.toFixed(2)} V, about ${Math.round(b.secondsLeft)} s left), flying home`,
            new SafeHome(this));
      }
    }
    if (p.status === "lost") {
      this.lostSince ??= now;
      if (full && now - this.lostSince > SAFETY.holdLost * 1000) this.trip("lost-land", "I'm still lost, so I'm landing where I am", this.takeover?.landing ? this.takeover : new SafeLand());
      else if (!this.takeover) {
        this.trip("lost", "I've lost track of where I am, holding still", new SafeHold());
        if (!full) ctl.askPilot("land", "I've lost track of where I am. Please take over and land.");
      }
    } else if (this.lostSince != null) {
      this.lostSince = null;
      const held = this.takeover?.kind === "lost";
      if (held) this.takeover = null;
      this.tripped.delete("lost");
      // found again (the vision side relocalized) after the mission was stopped: home, not a hover where it stopped
      if (held && full && this.house?.home && ctl.isFlying?.()) this.trip("found", "I know where I am again, so I'm flying home", new SafeHome(this, "Flying home: I know where I am again"));
    }
  }

  // Someone in the current picture may be inside the standoff (a box over half the frame, or nearRange() under it): on the
  // ground, before a take-off (the camera sees them; the standoff can't be kept from a pad someone stands beside).
  personClose(ctl = this.ctl) {
    const per = ctl.perception, p = this.loc.pose(), hc = p.z + 0.025 - (this.map?.floorAt(p.x, p.y) ?? 0);
    return per?.frameAge < 500 && per.find("person", 0.4).some((d) => d.box.h > SAFETY.personBox || nearRange(d, ctl, hc) < SAFETY.standoff);
  }

  // The controller's setpoint filter (state: { ctl, dt, now, gap, behavior }): the person standoff, the vision fixes' limits,
  // the height hold, the speed caps (avoid.limit) on what the behavior asks, then the geofence's push (after the caps:
  // nothing looking ahead cancels it). Over every band the map has (its clearances are a lower height's there), no push:
  // the drone holds still and comes down.
  filter(sp, { ctl, now, dt = 0.033, gap = 0 }) {
    this.reason = "";
    this.personHeld = false;
    if (!ctl.isFlying()) return Object.assign(this, { pushed: [0, 0], zHold: null, fixHold: null }), sp;
    if (gap > SAFETY.jitter) this.hoverUntil = now + SAFETY.jitterHover;
    if (now < this.hoverUntil) {
      this.reason = "the app was too slow for a moment";
      return sp.heading !== undefined ? { heading: sp.heading } : { yawRate: 0 };
    }
    const p = this.loc.pose();
    if (p.status === "lost" || !this.map) return sp;
    const floor = (this.floor = this.map.floorAt(p.x, p.y) ?? this.floor ?? 0); // off the rooms: the one we came from
    let out = this.keepHeight(this.vision(this.standoff(sp, ctl, p, floor, now), ctl, now), ctl, p, floor, dt);
    const top = this.map.o.bands.at(-1), high = p.z - floor > top + (this.map.o.bandHalf ?? 0) && p.zSigma < SAFETY.zTrust;
    if (out.vz > 0 && p.z - floor > top + 0.2) out = { ...out, vz: 0 };
    if (high) { // over every band the map has (it knows nothing up there: a door head, a lamp): hold still and come down; no push
      const { vx, vy, ...rest } = out;
      out = { ...rest, vz: Math.min(rest.vz ?? 0, -SAFETY.zDown) };
    }
    const capped = this.avoid.limit(out, p, now); // climbing and sinking capped too (the height hold's as well)
    if (capped !== out && !this.reason && (this.avoid.last.blind || Math.hypot(capped.vx ?? 0, capped.vy ?? 0) < 0.5 * Math.hypot(out.vx ?? 0, out.vy ?? 0))) this.reason = this.avoid.last.why || this.avoid.last.vertical;
    if (high) return (this.reason ||= `too high for the map: holding still ${ctl.autonomy === "full" ? "and coming down" : "until you bring me down"}`), capped; // co-pilot: the pilot has the throttle
    return this.geofence(capped, p, floor, dt, now);
  }

  // Vision the position source (localizer.visionActive()): turns at most yawVision rad/s (yawStale once the last fix is
  // yawStaleAge ms old), so tracking keeps a steady view; fixes too sparse (older than fixSlow ms, under fixRate a second):
  // at most `sparse` m/s; none for fixHold ms: hold position and turn back (at yawVision) to where the camera last placed
  // itself (a blank wall, glare, a dark corner: facing on into it, no fix comes back) or, with no turn back to make (the
  // last fix faced this way: the scene there changed, or the height estimate drifted off what the render shows), toward
  // the most open side (openSide()), and hold that heading until one lands (or the position is lost: the localizer's lost
  // clock allows the turn, at most backFor ms). No turn back to make, or no fix askAfter ms after facing there: the vision
  // side is asked to relocalize (localizer.askFix()), at once too when σ is over askSigma as the hold begins (the estimate
  // has drifted past what tracking finds from where it thinks it is). The real drone's video too slow to fly by (under paceFps for
  // paceHold ms, watch()): hold still, position and heading.
  vision(sp, ctl, now) {
    if (this.paced) {
      this.reason = `holding still: the video is too slow to fly by (${ctl.perception?.fps ?? 0} frames a second)`;
      const { vx, vy, yawRate, heading, ...rest } = sp;
      return { ...rest, heading: (this.paceHeading ??= ctl.est.heading) };
    }
    this.paceHeading = null;
    const q = this.loc.visionActive?.() && this.loc.fixQuality(now);
    this.sparse = q && (q.visionAge > SAFETY.fixSlow || q.rate < SAFETY.fixRate) ? SAFETY.sparse : Infinity;
    this.pushCap = q && q.visionAge > SAFETY.pushFresh ? SAFETY.pushStale * clamp((SAFETY.fixHold - q.visionAge) / (SAFETY.fixHold - SAFETY.pushFresh), 0, 1) : Infinity;
    if (!q) return (this.fixHold = null), sp;
    const age = q.visionAge, out = { ...sp, maxYawRate: Math.min(sp.maxYawRate ?? Infinity, age > SAFETY.yawStaleAge ? SAFETY.yawStale : SAFETY.yawVision) };
    if (age > SAFETY.fixHold) {
      // back to the heading of the last fix and a little past it (that heading was the edge of what the camera can place);
      // already facing that way: toward the most open side, and a relocalization asked for now
      const yaw = this.loc.vision?.yaw, P = this.loc.pose(), to = Number.isFinite(yaw) ? this.loc.headingFor(yaw, P) : ctl.est.heading, d = wrapAngle(to - ctl.est.heading);
      if (!this.fixHold) {
        const same = Math.abs(d) <= 8 * DEG;
        this.fixHold = { heading: same ? this.openSide(P, ctl) : to + Math.sign(d) * SAFETY.backPast, t: now, same };
        if (same || P.sigma > SAFETY.askSigma) this.loc.askFix?.(now); // (seed 11: 0.4 m off at σ 0.21, asked after the turn: lost first)
      }
      const back = Math.abs(wrapAngle(ctl.est.heading - this.fixHold.heading)) > 8 * DEG;
      if (back) this.loc.visionBack?.(Math.min(now - this.fixHold.t, SAFETY.backFor)); // the lost clock allows the turn
      else if (now - (this.fixHold.facing ??= now) > SAFETY.askAfter) this.loc.askFix?.(now);
      this.reason = `holding still: no camera position fix for ${(age / 1000).toFixed(0)} s${back ? (this.fixHold.same ? ", turning to see more of the room" : ", turning back to where it last placed itself") : ""}`;
      const { vx, vy, yawRate, heading, ...rest } = out;
      return { ...rest, heading: this.fixHold.heading, maxYawRate: Math.min(sp.maxYawRate ?? Infinity, SAFETY.yawVision) };
    }
    this.fixHold = null;
    const v = Math.hypot(out.vx ?? 0, out.vy ?? 0);
    if ((age > SAFETY.fixSlow || q.rate < SAFETY.fixRate) && v > SAFETY.sparse) {
      this.reason = `flying slowly: the camera's position fixes are too sparse (${q.rate.toFixed(1)} a second)`;
      return { ...out, vx: ((out.vx ?? 0) * SAFETY.sparse) / v, vy: ((out.vy ?? 0) * SAFETY.sparse) / v };
    }
    return out;
  }

  // The controller heading toward the most open side, 60-120° either way from the pose's yaw (the longest free run on the
  // map without temporary obstacles; the smaller turn when alike): another view, away from what the camera can't place.
  openSide(P, ctl = this.ctl) {
    const map = staticView(this.map), need = map.o.droneRadius + 2 * P.sigma;
    let best = { run: -Infinity, yaw: P.yaw };
    for (const a of [60, -60, 90, -90, 120, -120]) {
      const yaw = P.yaw + a * DEG, run = freeRun(map, P, [Math.cos(yaw), Math.sin(yaw)], need) - Math.abs(a) / 600;
      if (run > best.run) best = { run, yaw };
    }
    return ctl.est.heading - wrapAngle(best.yaw - P.yaw); // (headings turn clockwise)
  }

  // Full auto, while the behavior holds height (no vz: a scan, a hover, a wait): a climb back when the localizer's height
  // (vision fixes keep it; the controller's own is dead reckoned and drifts with the flow's climb-rate error: a hover sank
  // 0.27 m onto a cabinet) is more than SAFETY.zBand under the height held, with an integral for a steady sink (leaking
  // away otherwise); a way back down (at most zDown, the caps on sinking still on) once it is more than zOver over it
  // (the same drift the other way climbed a scan from 1.0 to 2.1 m, into a door head the map's bands don't reach), so a
  // noisy height can't sink the drone; the height held at least SAFETY.minHover over the floor (only landings go lower) and
  // no higher than the map's top band. (A climb wherever the space above was clearer ratcheted a drone in a doorway up to
  // its head.)
  // While the controller holds the localizer's height itself (ctl.zLoc: height()), the height held goes to it as sp.z, the
  // same limits on it, and no climb or sink of its own (its integral kept a small climb rate going for seconds after a
  // dip, so the controller, flying that rate, held no height at all: a scan wandered 0.3 m).
  keepHeight(sp, ctl, p, floor, dt = 0.033) {
    if (ctl.autonomy !== "full" || !ctl.airborne || (sp.vz !== undefined && sp.vz !== 0) || sp.touchdown || !(p.zSigma < SAFETY.zTrust)) return Object.assign(this, { zHold: null, zI: 0 }), sp;
    this.zHold = clamp(this.zHold ?? p.z, floor + SAFETY.minHover, floor + Math.max(SAFETY.minHover, this.map?.o?.bands?.at(-1) ?? Infinity));
    if (ctl.zLoc) return (this.zI = 0), { ...sp, z: this.zHold };
    const below = this.zHold - SAFETY.zBand - p.z, above = p.z - this.zHold - SAFETY.zOver, e = below > 0 ? below : above > 0 ? -above : 0, slow = ctl.heightSlowdown ?? 1;
    this.zI = e ? clamp((this.zI ?? 0) + (SAFETY.zI / slow) * e * dt, -SAFETY.zDown, SAFETY.zMax) : (this.zI ?? 0) * Math.exp(-dt / 2);
    const vz = clamp((SAFETY.zGain / slow) * e + this.zI, -SAFETY.zDown, SAFETY.zMax);
    return Math.abs(vz) < 0.02 ? sp : { ...sp, vz };
  }

  // The height the controller holds (full auto): the localizer's { z, vz, zSigma, lead } with lead while its σz is under
  // zLead and fixes come at zRate (vision fixes and the throttle's physics keep it; nav/localizer.js height()), else the
  // controller holds its own. Never on the simulator's truth: the real drone has no such height, and the simulator flies as
  // it would (on its flow).
  height(now = performance.now()) {
    const h = this.loc.known && this.loc.height?.();
    if (!h || !Number.isFinite(h.z) || !Number.isFinite(h.vz)) return null;
    return { ...h, lead: h.zSigma < SAFETY.zLead && this.loc.source !== "truth" && (this.loc.fixQuality?.(now)?.rate ?? 0) >= SAFETY.zRate };
  }

  // The flow scale the localizer learnt (true velocity / the controller's flow velocity, whose scene depth is a guess) while
  // splat fixes aid it (about 3 Hz or more), else 1: the controller's velocity loop reads the flow with it.
  flowScale(now = performance.now()) {
    const q = this.loc.fixQuality?.(now);
    return q && q.rate >= 2.4 && q.age < 1500 && Number.isFinite(this.loc.scale) ? clamp(this.loc.scale, 0.3, 3) : 1;
  }

  // Standoff: the trigger range includes the stopping distance at the current speed (and the video's lateness), so the
  // drone comes to rest 1.5 m from the nearest a person could be (on the drone's floor or theirs, whichever is nearer);
  // for a second after the last close sighting it holds: inside 1.5 m only backing away passes, between 1.5 m and the
  // stopping distance only moving closer is cut (passing by them goes on). A box cut by the frame's bottom edge after a
  // whole view of the same person (scaleFor ms) takes its range from how much wider the box got since (their width
  // doesn't change: scaleMargin of that for the hold); else the nearest plausible. That near estimate is only for the
  // hold: everyone seen within 6 m is remembered where they probably are (bestRange on the floor their range lands on)
  // as a temporary obstacle of radius 1.5 m: the planner routes around it and, inside it, only backing away passes.
  standoff(sp, ctl, p, floor, now) {
    const per = ctl.perception, m = this.person, speed = Math.hypot(...(this.loc.velocity?.() ?? [0, 0])), creeping = speed <= SAFETY.creepNear + 0.03;
    const fresh = m && now - m.t < 1000;
    const reach = (R) => R + (fresh ? 0.5 : speed * (0.4 + ctl.videoDelay / 1000));
    if (per.frameAge < 500 && per.latest.t !== this.seenFrame) {
      this.seenFrame = per.latest.t;
      const P = this.loc.poseAt?.(per.latest.t - ctl.videoDelay) ?? p, seen = new Set();
      let near = Infinity;
      for (const d of per.find("person", 0.4)) {
        const yaw = P.yaw - ctl.bearingOf(d.box.x + d.box.w / 2), at = (fn) => floorRange(fn, d, ctl, this.map, P, yaw);
        const lo = Math.min(nearRange(d, ctl, P.z + 0.025 - floor), at(nearRange)), cut = d.box.y + d.box.h >= 0.97 ? [lo, Math.max(lo, at(edgeRange))] : null;
        const scaled = cut && this.scaled(d, P, yaw, cut, now), rb = scaled ? clamp(scaled, ...cut) : Math.max(lo, at(bestRange));
        // not the camera's picture: bearings and ranges are wrong, so anyone in it is taken to be right here
        const r = this.picture ? 0.5 : Math.min(scaled ? clamp(scaled * SAFETY.scaleMargin, ...cut) : lo, d.box.h > SAFETY.personBox ? 0.5 : Infinity);
        const seated = !cut && this.seated(d, per, rb), R = seated && creeping ? SAFETY.seated : SAFETY.standoff;
        if (r < reach(R) && r < near) {
          this.person = { x: P.x + r * Math.cos(yaw), y: P.y + r * Math.sin(yaw), yaw, r, R, t: now };
          near = r;
        }
        if (rb < SAFETY.personSee && !this.picture && this.confirmed(d, yaw, now)) seen.add(this.remember(d, P, yaw, rb, cut, now, seated).id);
      }
      this.seenClear(P, seen, ctl, now);
    }
    // Back away from where people inside the standoff probably are (never faster than backOff; the speed cap keeps it off walls).
    let bx = 0, by = 0;
    for (const t of this.avoid.temps.values()) {
      const L = t.kind === "person" ? Math.hypot(p.x - t.x, p.y - t.y) : Infinity;
      const k = SAFETY.backOff * clamp(((t.r ?? SAFETY.standoff) + SAFETY.backFrom - L) / (2 * SAFETY.backFrom), 0, 1);
      if (k > 0 && L > 0.05) [bx, by] = [bx + ((p.x - t.x) / L) * k, by + ((p.y - t.y) / L) * k];
    }
    const back = (out) => {
      if (Math.hypot(bx, by) < 0.02) return out;
      this.reason = "backing away from a person";
      const h = toBody(out.vx ?? 0, out.vy ?? 0, p.yaw);
      return { ...out, ...toBody(h.vx + bx, h.vy + by, p.yaw) };
    };
    if (sp.vx === undefined && sp.vy === undefined) return back(sp);
    // within the standoff of someone seated or lying still: a creep (their zone and hold are the smaller SAFETY.seated)
    const v0 = Math.hypot(sp.vx ?? 0, sp.vy ?? 0), sat = [...this.avoid.temps.values()].some((t) => t.kind === "person" && t.seated && Math.hypot(p.x - t.x, p.y - t.y) < SAFETY.standoff + 0.2);
    if (sat && v0 > SAFETY.creepNear) [sp, this.reason] = [{ ...sp, vx: ((sp.vx ?? 0) * SAFETY.creepNear) / v0, vy: ((sp.vy ?? 0) * SAFETY.creepNear) / v0 }, "slowly past someone sitting"];
    const q = this.person && now - this.person.t < 1000 ? this.person : this.avoid.tempsAt(p.x, p.y).find((t) => t.kind === "person");
    if (!q) return back(sp);
    // Only backing off passes (from where they were last seen; they may have moved since); outside the standoff itself,
    // anything that doesn't close in.
    const dx = q.x - p.x, dy = q.y - p.y, L = Math.hypot(dx, dy);
    const yaw = q === this.person ? q.yaw : Math.atan2(dy, dx), h = toBody(sp.vx ?? 0, sp.vy ?? 0, p.yaw), u = [Math.cos(yaw), Math.sin(yaw)]; // body -> H (the same reflection both ways)
    const closing = h.vx * u[0] + h.vy * u[1], v = Math.hypot(h.vx, h.vy);
    if (closing < -0.3 * v) return back(sp);
    this.reason = "a person is close";
    if (q === this.person && q.r >= q.R) {
      const ox = h.vx - Math.max(0, closing) * u[0], oy = h.vy - Math.max(0, closing) * u[1];
      this.personHeld = Math.hypot(ox, oy) < 0.3 * v;
      return back({ ...sp, ...toBody(ox, oy, p.yaw) });
    }
    this.personHeld = true;
    const { vx, vy, ...rest } = sp;
    return back(rest);
  }

  // A person seen from P along yaw at range rb: their no-fly zone on the map, moved with them (the same track, else the
  // nearest within personJoin m, else one along the same bearing). A box cut by the frame's bottom edge only bounds the
  // range (cut: [nearest, farthest]): a spot already remembered along this bearing within those bounds is them, and stays
  // put; a spot only ever seen cut is replaced by the first whole view. One that dropped out of the bottom of the frame is
  // kept longer. Standing: at the median of the last 2 s of sightings (one view's range can be 0.5 m off). Walking (a
  // least-squares fit over those): a capsule from where they are to where they will be in walkAhead s. -> the zone
  remember(d, P, yaw, rb, cut, now, seated = false) {
    const b = d.box, faint = !!cut && b.h < SAFETY.faintBox, dist = (t) => Math.hypot(t.x - P.x, t.y - P.y);
    const mine = [...this.avoid.temps.values()].filter((t) => t.kind === "person"), free = mine.filter((t) => t.trackId == null || d.trackId == null || t.trackId === d.trackId);
    const half = (b.w * this.ctl.settings.get("hfov") * DEG) / 2 + SAFETY.bearing, along = (t) => Math.abs(wrapAngle(Math.atan2(t.y - P.y, t.x - P.x) - yaw)) < half;
    const prior = cut && free.filter((t) => along(t) && dist(t) > cut[0] - 0.2 && dist(t) < cut[1] + 0.2).sort((a, c) => Math.abs(dist(a) - rb) - Math.abs(dist(c) - rb))[0];
    const r = prior ? clamp(dist(prior), cut[0], cut[1]) : rb;
    let x = P.x + r * Math.cos(yaw), y = P.y + r * Math.sin(yaw);
    const same = prior || (d.trackId != null && mine.find((t) => t.trackId === d.trackId)) ||
      free.find((t) => Math.hypot(t.x - x, t.y - y) < SAFETY.personJoin + (t.v ? Math.hypot(...t.v) * SAFETY.walkAhead : 0)) || (!cut && free.find((t) => t.cut && along(t)));
    const onlyCut = !!cut && (!same || same.cut), floor = this.map.floorAt(x, y) ?? this.floor ?? 0;
    const keep = 1000 * (faint ? SAFETY.personLost : SAFETY.personMemory), hist = [...(same && !(same.cut && !cut) ? same.hist : []).filter((h) => now - h.t < 2000), { t: now, x, y }];
    const v = velocity(hist), ahead = Math.hypot(...v) > SAFETY.walk ? [x + v[0] * SAFETY.walkAhead, y + v[1] * SAFETY.walkAhead] : null;
    const still = !ahead && (cut ? !!same?.seated : seated), R = still ? SAFETY.seated : SAFETY.standoff; // a cut view keeps what a whole one saw
    if (!ahead) [x, y] = ["x", "y"].map((k) => hist.map((h) => h[k]).sort((a, c) => a - c)[hist.length >> 1]);
    return this.avoid.addTemp({ id: same?.id ?? `person-${++this.personIds}`, kind: "person", trackId: d.trackId ?? same?.trackId ?? null, x, y, r: R, ...(ahead && { polygon: capsule([x, y], ahead, R) }),
      v, hist, zMin: floor - 0.1, zMax: floor + 2.2, until: now + Math.max(keep, cut && same?.faint ? same.until - now : 0), faint: faint || (!!cut && !!same?.faint), cut: onlyCut, clear: 0, seated: still,
      ref: cut ? same?.ref ?? null : { r: rb, box: { ...b }, t: now } }, now);
  }

  // Seated or lying: a whole box (off every edge of the frame) no taller than SAFETY.seatedAspect times its width, and, at
  // range r (m, horizontal) where known, no taller than SAFETY.seatedTall m.
  seated(d, per, r = null) {
    const b = d.box, W = per.latest.width || 4, H = per.latest.height || 3, ctl = this.ctl;
    if (!(b.y > 0.03 && b.x > 0.01 && b.x + b.w < 0.99 && (b.h * H) / Math.max(1e-6, b.w * W) < SAFETY.seatedAspect)) return false;
    const tall = Number.isFinite(r) && ctl.elevationOf ? r * (Math.tan(ctl.elevationOf(b.y)) - Math.tan(ctl.elevationOf(b.y + b.h))) : 0;
    return !(tall > SAFETY.seatedTall);
  }

  // Remembered on the map only once seen on SAFETY.zoneFrames frames in a row: the same track (one already remembered goes
  // on being moved), or without tracks along about the same bearing. A one-frame false detection (a poster, a coat, a
  // render's glitch) never closes a doorway; the hold for someone close doesn't wait.
  confirmed(d, yaw, now) {
    const key = d.trackId != null ? `t${d.trackId}` : `b${Math.round(yaw / (10 * DEG))}`, was = this.seenTracks.get(key), n = was && now - was.t < 700 ? was.n + 1 : 1;
    this.seenTracks.set(key, { n, t: now });
    if (this.seenTracks.size > 64) for (const [k, v] of this.seenTracks) if (now - v.t > 2000) this.seenTracks.delete(k);
    return n >= SAFETY.zoneFrames || (d.trackId != null && [...this.avoid.temps.values()].some((t) => t.kind === "person" && t.trackId === d.trackId));
  }

  // A cut box's range from the box width of the last whole view of the same person (their track, or the spot remembered
  // along this bearing within the cut bounds), or null (none in scaleFor ms, or either box cut at the sides).
  scaled(d, P, yaw, [lo, hi], now) {
    const b = d.box, half = (b.w * this.ctl.settings.get("hfov") * DEG) / 2, side = (q) => q.x <= 0.01 || q.x + q.w >= 0.99;
    if (side(b)) return null;
    const ok = (t) => t.kind === "person" && t.ref && now - t.ref.t < SAFETY.scaleFor && !side(t.ref.box);
    const t = [...this.avoid.temps.values()].find((t) => ok(t) && (d.trackId != null && t.trackId === d.trackId ||
      (t.trackId == null || d.trackId == null) && Math.abs(wrapAngle(Math.atan2(t.y - P.y, t.x - P.x) - yaw)) < half + SAFETY.bearing && Math.hypot(t.x - P.x, t.y - P.y) < hi + 0.3));
    if (!t) return null;
    const tan = (w) => Math.tan((w * this.ctl.settings.get("hfov") * DEG) / 2);
    return (t.ref.r * tan(t.ref.box.w)) / tan(b.w);
  }

  // Remembered people whose spot is now in plain view (inside the picture, the floor there above the frame's bottom edge,
  // nothing on the map in between up to sitting height) with nobody seen along it: gone after SAFETY.clearFor ms and
  // SAFETY.clearFrames such frames in a row.
  seenClear(P, seen, ctl, now) {
    const map = this.map, half = (ctl.settings.get("hfov") * DEG) / 2 - 8 * DEG, occ = map.occ[map.bandOf(0.6)];
    for (const t of [...this.avoid.temps.values()]) {
      if (t.kind !== "person" || seen.has(t.id)) continue;
      const dx = t.x - P.x, dy = t.y - P.y, L = Math.hypot(dx, dy), floor = map.floorAt(t.x, t.y);
      let clear = floor != null && L > 0.5 && L < 5 && Math.abs(wrapAngle(Math.atan2(dy, dx) - P.yaw)) < half && L > edgeRange(null, ctl, P.z + 0.025 - floor) + 0.3;
      for (let i = 1, n = Math.ceil(L / map.o.cell); clear && i < n; i++) {
        const k = map.idx(P.x + (dx * i) / n, P.y + (dy * i) / n);
        if (k < 0 || map.room[k] < 0 || map.wall[k] || occ[k]) clear = false;
      }
      t.clearSince = clear ? t.clearSince ?? now : null;
      t.clear = clear ? (t.clear ?? 0) + 1 : 0;
      if (t.clear >= SAFETY.clearFrames && now - t.clearSince >= SAFETY.clearFor) this.avoid.removeTemp(t.id);
    }
  }

  // The geofence's clearances at (x, y), the worst over the heights zs: [static, temporary]. Static: HomeMap.base under the
  // height rules, with the voxels (walls, furniture, unknown space); temporary: things the live depth found and suspected
  // changes, as discs and polygons (not people's no-fly zones: they stop an approach and never push, as a push out of one
  // can run the drone into a wall; not one return home is passing).
  geoClear(x, y, zs) {
    const m = this.map;
    let st = Infinity, tm = Infinity;
    for (const z of zs) {
      st = Math.min(st, m.base ? staticClearance(m, x, y, z) : m.clearance(x, y, z));
      for (const t of m.temps?.values() ?? []) {
        if (t.kind === "person" || this.avoid.passing.has(t.id) || z < (t.zMin ?? -Infinity) || z > (t.zMax ?? Infinity)) continue;
        tm = Math.min(tm, t.polygon ? polyDist(x, y, t.polygon) : Math.max(0, Math.hypot(x - t.x, y - t.y) - t.r));
      }
    }
    return [st, tm];
  }

  // Static things push hard (with the integral), temporary ones softly and never toward a static one the drone is near; the
  // push keeps to the caps that hold in every direction (the real drone blind or its braking unmeasured, sparse vision fixes).
  // A push alone (the behavior holding still) that has carried the drone SAFETY.pushTravel m from where it began, still
  // inside the buffer, is following a ridge of the clearance, not out of it: it stops there.
  geofence(sp, p, floor, dt, now = performance.now()) {
    const map = this.map, I = this.pushed, off = map.floorAt(p.x, p.y) == null;
    map.sync?.();
    // Height is uncertain too: the worst of the bands within ±2σz.
    const top = floor + map.o.bands.at(-1), dz = 2 * (p.zSigma ?? 0), zs = [p.z - dz, p.z, p.z + dz].map((z) => clamp(z, floor + 0.3, top));
    const need = map.o.droneRadius + 2 * p.sigma, edge = need + SAFETY.buffer, d = map.o.cell, at = (x, y) => this.geoClear(x, y, zs);
    const [cs, ct] = off ? [0, Infinity] : at(p.x, p.y), leak = Math.exp(-dt / SAFETY.pushLeak);
    if (cs >= edge) [I[0], I[1]] = [I[0] * leak, I[1] * leak];
    this.pushFrom = cs < edge && Math.hypot(sp.vx ?? 0, sp.vy ?? 0) < 0.05 ? this.pushFrom ?? { x: p.x, y: p.y } : null; // the push alone moving it
    const astray = !!this.pushFrom && Math.hypot(p.x - this.pushFrom.x, p.y - this.pushFrom.y) > SAFETY.pushTravel;
    if (astray) [I[0], I[1]] = [0, 0];
    if (cs >= edge + 0.45 && ct >= edge + 0.45 && Math.hypot(...I) < 0.01) return sp; // out of reach within the lookahead at mission speeds
    if (off || cs < need || ct < need) this.reason = "too close to something on the map"; // pose ± 2σ not free
    const grad = (k) => {
      const gx = at(p.x + d, p.y)[k] - at(p.x - d, p.y)[k], gy = at(p.x, p.y + d)[k] - at(p.x, p.y - d)[k], L = Math.hypot(gx, gy);
      return L > 1e-6 && Number.isFinite(L) ? [gx / L, gy / L] : null;
    };
    let gs = grad(0), gt = ct < edge + 0.45 ? grad(1) : null, L;
    if (!gs && cs < edge) {
      // deep in a wall (or flat clearance): toward the nearest free cell
      const q = nearestFree(staticView(map), [p.x, p.y], { band: map.bandOf(p.z - floor), sigma: 0, maxR: 1 }); // temporary obstacles off, voxels on
      if (q && (L = Math.hypot(q[0] - p.x, q[1] - p.y)) > 1e-6) gs = [(q[0] - p.x) / L, (q[1] - p.y) / L];
    }
    if (gs && gt && cs < edge + 0.15) { // a temporary obstacle's push never leans toward the wall the drone is near
      const dot = gt[0] * gs[0] + gt[1] * gs[1];
      if (dot < 0) gt = (([x, y]) => ((L = Math.hypot(x, y)) > 0.2 ? [x / L, y / L] : null))([gt[0] - dot * gs[0], gt[1] - dot * gs[1]]);
    }
    const c0 = Math.cos(p.yaw), s0 = Math.sin(p.yaw), bx = sp.vx ?? 0, by = sp.vy ?? 0;
    let vx = c0 * bx + s0 * by, vy = s0 * bx - c0 * by; // body (forward, right) -> H
    for (const [g, k] of [[gs, 0], [gt, 1]]) {
      if (!g) continue;
      const ahead = off ? 0 : at(p.x + vx * SAFETY.lookahead, p.y + vy * SAFETY.lookahead)[k], into = vx * g[0] + vy * g[1];
      if (into < 0 && ahead < edge) [vx, vy] = [vx - into * g[0], vy - into * g[1]];
    }
    // the integral only while holding still (a drift or sink a hover must beat); a path flown inside the buffer (a climb over
    // something with the height unsure brings the band below into it) winds it up into a push off the way
    if (gs && cs < edge && Math.hypot(bx, by) < 0.05) [I[0], I[1]] = [I[0] + gs[0] * SAFETY.pushI * (edge - cs) * dt, I[1] + gs[1] * SAFETY.pushI * (edge - cs) * dt];
    else if (Math.hypot(bx, by) >= 0.05) [I[0], I[1]] = [I[0] * leak, I[1] * leak];
    const wound = Math.hypot(I[0], I[1]);
    if (wound > SAFETY.pushMax) [I[0], I[1]] = [(I[0] * SAFETY.pushMax) / wound, (I[1] * SAFETY.pushMax) / wound]; // no more than the push may be
    // Push back by how far inside the buffer we are or are about to be (the pose's own velocity: drift counts too).
    const vel = this.loc.velocity?.() ?? [0, 0], push = (g, c) => (g ? SAFETY.pushGain * Math.max(0, edge - c + Math.max(0, -(vel[0] * g[0] + vel[1] * g[1])) * SAFETY.lookahead) : 0);
    const ks = push(gs, cs), kt = push(gt, ct), px = (gs ? gs[0] * ks : 0) + (gt ? gt[0] * kt : 0) + I[0], py = (gs ? gs[1] * ks : 0) + (gt ? gt[1] * kt : 0) + I[1], m = Math.hypot(px, py);
    const most = astray ? 0 : Math.min(SAFETY.pushMax, this.avoid.speedCap(now).v, this.sparse ?? Infinity, this.pushCap ?? Infinity), cap = m > most ? most / m : 1;
    if (m * cap < 0.005 && sp.vx === undefined && sp.vy === undefined) return sp;
    // never faster than the behavior asked or the push alone
    const ox = vx + px * cap, oy = vy + py * cap, fast = Math.max(Math.hypot(bx, by), m * cap), k2 = Math.min(1, fast / (Math.hypot(ox, oy) || 1));
    return { ...sp, ...toBody(ox * k2, oy * k2, p.yaw) };
  }
}

// Velocity (m/s) of [{ t (ms), x, y }] by least squares, from 4 points over at least 0.5 s; else [0, 0].
function velocity(h) {
  if (h.length < 4 || h.at(-1).t - h[0].t < 500) return [0, 0];
  const n = h.length, mt = h.reduce((a, p) => a + p.t, 0) / n, mx = h.reduce((a, p) => a + p.x, 0) / n, my = h.reduce((a, p) => a + p.y, 0) / n;
  const tt = h.reduce((a, p) => a + (p.t - mt) ** 2, 0);
  return [(1000 * h.reduce((a, p) => a + (p.t - mt) * (p.x - mx), 0)) / tt, (1000 * h.reduce((a, p) => a + (p.t - mt) * (p.y - my), 0)) / tt];
}

// A stadium of radius r around the segment a-b, as a polygon.
function capsule(a, b, r, n = 6) {
  const th = Math.atan2(b[1] - a[1], b[0] - a[0]), out = [];
  for (let i = 0; i <= n; i++) out.push([b[0] + r * Math.cos(th - Math.PI / 2 + (Math.PI * i) / n), b[1] + r * Math.sin(th - Math.PI / 2 + (Math.PI * i) / n)]);
  for (let i = 0; i <= n; i++) out.push([a[0] + r * Math.cos(th + Math.PI / 2 + (Math.PI * i) / n), a[1] + r * Math.sin(th + Math.PI / 2 + (Math.PI * i) / n)]);
  return out;
}

// What the safety layer flies instead of the mission.
class SafeHold extends Hold {
  constructor() {
    super();
    this.label = "Holding: lost track of position";
  }
  update(ctl, dt, now) {
    if (this.safety.takeover !== this) return { done: { ok: true, text: "Found my position again." } };
    return super.update(ctl, dt, now);
  }
}

class SafeLand extends Land {
  constructor() {
    super({ label: "Landing where I am", onto: true });
    this.landing = true;
  }
  update(ctl, dt, now) {
    const r = super.update(ctl, dt, now);
    if (r.done && this.safety.takeover === this) this.safety.takeover = null;
    return r;
  }
}

// What temporary obstacle blocks the way home from P, in words (only one on the way: the plan without them passes it), or null.
export function homeBlocker(map, P, home, sigma = 0.17) {
  const t = home && blockers(map, [P.x, P.y], [home.x, home.y], { sigma })?.on[0];
  if (!t) return null;
  const room = map.roomAt(...(Number.isFinite(t.x) ? [t.x, t.y] : t.polygon[0]))?.name, at = room ? ` in ${/^(room|bedroom|bathroom)\s*\d/i.test(room) ? room : `the ${room}`}` : "";
  return t.kind === "person" ? `A person${at} is in the way` : t.kind === "change" ? `Something changed${at} and blocks the way` : t.source === "door leaf" ? `A doorway${at} looks closed` : `Something the map lacks is in the way${at}`;
}

// Home and land (co-pilot: home, then the pilot is asked to land); a blocked way is tried again (ReturnHome) until the
// battery only covers landing on a clear spot nearby. Planned as the missions plan (safety.homeOptions(), the mission
// runner's: its σ, a comfy first plan, facing what blocks the way, passing a change Claude called no change).
class SafeHome extends ReturnHome {
  constructor(safety, label = "Battery low: flying home") {
    super({ localizer: safety.loc, map: safety.map, home: safety.house.home, label, battery: () => safety.battery()?.secondsLeft, sigma: () => clamp(2 * safety.loc.pose().sigma + 0.08, 0.17, 0.4),
      blocker: () => homeBlocker(safety.map, safety.loc.pose(), safety.house.home), ...safety.homeOptions?.(), say: (text) => safety.emit("status", { text }) });
  }
  update(ctl, dt, now) {
    const r = super.update(ctl, dt, now);
    if (r.done && this.safety.takeover === this)
      this.safety.takeover = r.done.ok || r.done.safeLanded || ctl.autonomy !== "full" ? null : Object.assign(new SafeLand(), { safety: this.safety, reason: this.reason, kind: "battery-land" });
    if (this.phase === "land" || this.phase === "spotLand") this.landing = true;
    return r;
  }
}

