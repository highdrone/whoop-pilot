// Physics for the BetaFPV Meteor65 Pro II (DJI O4): a 36 g 1S 65 mm brushless whoop flown in Betaflight ANGLE mode.
// Inputs are what the flight controller receives: roll/pitch/yaw in [-1, 1], thr in [0, 1].
import { WALL_H } from "./world.js";
import { actualRate } from "../util.js";

const G = 9.81;
const DEG = Math.PI / 180;

export const SIM_DRONE = {
  mass: 0.036, // kg with the O4 air unit and a 480 mAh pack (the model is in accelerations; this is for reference)
  angleLimit: 60, // deg at full stick (Betaflight angle_limit)
  yawCenter: 70, // deg/s, Betaflight "Actual" rates center sensitivity
  yawMax: 670, // deg/s at full stick
  twr: 4.0, // thrust-to-weight at full throttle on a full pack; the controller's hover prior (0.45) assumes this
  thrExpo: 1.6, // thrust ~ throttle^1.6
  radius: 0.05, // m, the ducts' outer corners (= HomeMap droneRadius)
  capacityAh: 0.48, // LAVA 1S 480 mAh
  resistance: 0.05, // ohms, pack + wiring sag
  baseA: 0.4, // A armed, plus motorA x thrust: hovering ~5.8 A, 3.3 V under load after ~4 min, flat at ~4.8 min
  motorA: 16,
  standbyA: 0.8, // A disarmed after the first arm: the O4 in low-power standby (~3 W); before it the pack isn't plugged in
};

export class SimDrone {
  constructor(rand = Math.random) {
    this.rand = rand;
    this.reset(7.0, 4.25, 90);
  }

  // z: the floor under the start (a house's rooms have their own floors); heights are absolute, like the world's.
  reset(x, y, headingDeg, z = 0) {
    Object.assign(this, {
      x, y, z, vx: 0, vy: 0, vz: 0,
      floorZ: z,
      yaw: headingDeg * DEG, // CCW from +x (world convention)
      yawRate: 0, // CCW rad/s
      roll: 0, pitch: 0, // rad; +roll = right side down, +pitch = nose down
      armed: false, crashed: false, airborne: false,
      powered: false, // the pack goes in when the pilot arms: an app left open on the pad doesn't drain it
      soc: 1, vbat: 4.35, current: 0,
      thr: 0,
      gust: { x: 0, y: 0, z: 0 },
    });
    // Every whoop drifts a little in angle mode (accelerometer trim); pick a direction for this flight.
    const a = this.rand() * Math.PI * 2;
    this.trimDrift = { x: Math.cos(a) * 0.1, y: Math.sin(a) * 0.1 };
    this.lastImpact = 0;
  }

  arm() {
    if (this.crashed || this.thr > 0.05) return false;
    this.armed = this.powered = true;
    return true;
  }

  disarm() {
    this.armed = false;
  }

  step(dt, sticks, world) {
    const lim = SIM_DRONE.angleLimit * DEG;
    const lag = Math.min(1, dt / 0.06);
    this.thr = sticks.thr;
    const armed = this.armed && !this.crashed;

    // Attitude follows the sticks (angle mode) with a short lag; only while airborne.
    const tRoll = armed && this.airborne ? sticks.roll * lim : 0;
    const tPitch = armed && this.airborne ? sticks.pitch * lim : 0;
    this.roll += (tRoll - this.roll) * lag;
    this.pitch += (tPitch - this.pitch) * lag;
    const rateCW = armed && this.airborne ? actualRate(sticks.yaw, SIM_DRONE.yawCenter, SIM_DRONE.yawMax) * DEG : 0;
    this.yawRate += (-rateCW - this.yawRate) * Math.min(1, dt / 0.05);
    this.yaw += this.yawRate * dt;

    // Battery: open-circuit voltage by charge, sagging under load.
    const thrustFrac = armed ? Math.pow(Math.max(0, sticks.thr), SIM_DRONE.thrExpo) : 0;
    this.current = armed ? SIM_DRONE.baseA + SIM_DRONE.motorA * thrustFrac : this.powered ? SIM_DRONE.standbyA : 0;
    this.soc = Math.max(0, this.soc - (this.current * dt) / 3600 / SIM_DRONE.capacityAh);
    let ocv = 3.45 + 0.9 * this.soc;
    if (this.soc < 0.1) ocv -= (0.1 - this.soc) * 3;
    this.vbat = ocv - this.current * SIM_DRONE.resistance;

    // Thrust. Ground effect adds a little lift in the first ~15 cm.
    const floor = (this.floorZ = world.floorAt(this.x, this.y) ?? this.floorZ);
    const ceiling = world.ceilingAt(this.x, this.y) ?? WALL_H;
    const agl = this.z - floor;
    const ge = agl < 0.15 ? 1 + 0.12 * (1 - agl / 0.15) : 1;
    const accT = G * SIM_DRONE.twr * (this.vbat / 4.2) ** 2 * thrustFrac * ge;
    const cr = Math.cos(this.roll);
    const aUp = accT * Math.cos(this.pitch) * cr;
    const aFwd = accT * Math.sin(this.pitch) * cr;
    const aRight = accT * Math.sin(this.roll);

    // Turbulence: slow random gusts (Ornstein-Uhlenbeck) plus the trim drift.
    const tau = 1.2;
    for (const k of ["x", "y", "z"]) {
      const sigma = k === "z" ? 0.2 : 0.25;
      this.gust[k] += (-this.gust[k] / tau) * dt + sigma * Math.sqrt((2 * dt) / tau) * gauss(this.rand);
    }
    const fx = Math.cos(this.yaw);
    const fy = Math.sin(this.yaw);
    const rx = Math.sin(this.yaw);
    const ry = -Math.cos(this.yaw);
    const flying = this.airborne && armed;
    let ax = fx * aFwd + rx * aRight - 1.1 * this.vx + (flying ? this.gust.x + this.trimDrift.x : 0);
    let ay = fy * aFwd + ry * aRight - 1.1 * this.vy + (flying ? this.gust.y + this.trimDrift.y : 0);
    let az = aUp - G - 1.4 * this.vz + (flying ? this.gust.z : 0);

    if (!this.airborne) {
      // Sitting on the floor until thrust beats gravity.
      if (az > 0.3) this.airborne = true;
      else {
        this.vx *= 0.8;
        this.vy *= 0.8;
        this.vz = 0;
        ax = ay = az = 0;
      }
    }

    this.vx += ax * dt;
    this.vy += ay * dt;
    this.vz += az * dt;
    this.x += this.vx * dt;
    this.y += this.vy * dt;
    this.z += this.vz * dt;

    // Floor and ceiling.
    if (this.z <= floor) {
      const hit = -this.vz;
      this.z = floor;
      this.vz = 0;
      if (this.airborne && (!armed || aUp < G * 0.98)) {
        this.airborne = false;
        this.impact(hit, dt);
      }
    }
    if (this.z > ceiling - 0.06) {
      this.z = ceiling - 0.06;
      if (this.vz > 0) {
        this.impact(this.vz, dt);
        this.vz = -0.3 * this.vz;
      }
    }

    // Walls and furniture: whoops bounce, hard hits flip it over.
    const hit = world.collide(this, SIM_DRONE.radius, 0.05);
    if (hit > 0) this.impact(hit, dt);
    if (!armed && !this.airborne) this.yawRate = 0;
  }

  impact(speed, dt) {
    this.lastImpact = speed;
    if (speed > 4.0 && this.armed) {
      this.crashed = true;
      this.armed = false;
    }
  }
}

function gauss(rand) {
  const u = Math.max(1e-9, rand());
  const v = rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
