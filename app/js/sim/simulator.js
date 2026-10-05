// The simulator stands in for the whole real rig: quad + radio (with the aibrg.lua bridge) + pilot.
// The app talks to `sim.radio` exactly like it talks to the USB radio.
// Worlds: the demo apartment (world.js, the default), or a real house (twin-world.js: { house, map } from house/), whose
// camera is the splat twin's render of the drone's view (twin/, 4:3 equidistant O4 lens) with ground-truth boxes from
// its oracle, or without a twin the layered raycast camera. Frames from either go through the same video-delay queue.
// Realism for vision tests (augment.js): blur, exposure, noise, compression, an OSD and a lens error on the twin's frames.
// Scene changes (addObstacle, setDoor) go into the collision world and, as props, into the twin.
import { World, mulberry32 } from "./world.js";
import { augmenter, augmentOptions, simLens } from "./augment.js";
import { makeHouseWorld } from "./twin-world.js";
import { SimDrone, SIM_DRONE } from "./drone.js";
import { CameraRenderer } from "./render.js";
import { Emitter, DEG, clamp, wrapAngle } from "../util.js";
import { MASK } from "../protocol.js";
import { GPU, BUDGET } from "../vision/budget.js";

export const MIXER_PERIOD = 0.05; // s: EdgeTX runs mixer scripts every 50 ms
const TWIN_VIEW = { width: 640, height: 480, prio: "camera" }; // the twin's queue serves localization and live depth first
const ORACLE_VIEW = { width: 320, height: 240, prio: "camera" };
export const SIM_VIEW = { width: 320, height: 240 }; // the raycast cameras, in both worlds: the O4 lens's native 4:3
const MIN_VISIBLE = 0.35; // oracle visibleFraction from which an actor counts as detected

// Same rules as radio/SCRIPTS/MIXES/aibrg.lua, in normalized stick units. The script runs every `period` s: packets
// that arrive in between are read at its next run (the last one wins), and with the AI switch on the channels carry
// its outputs from that run; with the switch off they are the pilot's sticks (the mixes, not the script).
export class BridgeLogic {
  constructor({ period = MIXER_PERIOD } = {}) {
    this.cmd = { seq: 0, mask: 0, roll: 0, pitch: 0, thr: 0, yaw: 0, hover: 0 };
    this.lastRx = -1e9;
    this.engaged = this.failsafe = this.hadLink = false;
    this.ovr = { roll: false, pitch: false, thr: false, yaw: false };
    this.thrAtEngage = 0;
    this.fsStart = 0;
    this.fsThr = 0;
    this.rxOk = 0;
    this.period = period;
    this.nextRun = -Infinity;
    this.inbox = null;
    this.out = null;
  }

  receive(cmd) {
    this.inbox = cmd;
  }

  // pilot: { roll, pitch, yaw, thr, aiSwitch }. Returns the sticks the flight controller gets.
  outputs(p, t, beep) {
    if (t >= this.nextRun - 1e-9) {
      this.nextRun = t - this.nextRun < this.period ? this.nextRun + this.period : t + this.period;
      this.out = this.run(p, t, beep);
    }
    return p.aiSwitch ? this.out : { roll: p.roll, pitch: p.pitch, thr: p.thr, yaw: p.yaw };
  }

  // One run of the script.
  run(p, t, beep) {
    if (this.inbox) {
      this.cmd = this.inbox;
      this.inbox = null;
      this.lastRx = t;
      this.rxOk++;
    }
    const linkOk = t - this.lastRx < 0.3;
    this.linkOk = linkOk;
    if (p.aiSwitch && !this.engaged) {
      Object.assign(this, { engaged: true, failsafe: false, hadLink: false, thrAtEngage: p.thr });
      this.ovr = { roll: false, pitch: false, thr: false, yaw: false };
      beep(1400, 60);
      beep(1900, 60, 80);
    } else if (!p.aiSwitch && this.engaged) {
      this.engaged = this.failsafe = false;
      beep(900, 120);
    }
    const c = this.cmd;
    if (this.engaged) {
      if (linkOk) this.hadLink = true;
      if (this.hadLink && !this.failsafe && !linkOk) {
        this.failsafe = true;
        this.fsStart = t;
        this.fsThr = c.mask & MASK.thr && c.thr > 0.06 ? Math.max(0, Math.min(c.thr, c.hover) - 0.078) : 0;
        beep(600, 400);
      }
      if (Math.abs(p.roll) > 0.29) this.ovr.roll = true;
      if (Math.abs(p.pitch) > 0.29) this.ovr.pitch = true;
      if (Math.abs(p.yaw) > 0.29) this.ovr.yaw = true;
      if (Math.abs(p.thr - this.thrAtEngage) > 0.125) this.ovr.thr = true;
    }
    const out = { roll: p.roll, pitch: p.pitch, thr: p.thr, yaw: p.yaw };
    if (this.engaged) {
      if (this.failsafe) {
        if (c.mask & MASK.thr && !this.ovr.thr) out.thr = this.failsafeThrottle(t);
      } else {
        for (const [axis, bit] of Object.entries(MASK)) if (c.mask & bit && !this.ovr[axis]) out[axis] = c[axis];
      }
    }
    return out;
  }

  failsafeThrottle(t) {
    if (this.fsThr <= 0) return 0;
    const dt = t - this.fsStart;
    if (dt < 3) return this.fsThr;
    const k = (dt - 3) / 1.5;
    return k >= 1 ? 0 : this.fsThr * (1 - k);
  }
}

class SimRadio extends Emitter {
  constructor(sim) {
    super();
    this.sim = sim;
    this.kind = "sim";
    this.connected = true;
    this.telemetry = null;
  }
  send(cmd) {
    this.sim.bridge.receive(cmd, this.sim.time);
  }
}

export class Simulator extends Emitter {
  // house + map (a finalized HomeMap): fly in that house; twin (twin/twin.js Twin, optional): its camera.
  // detections: "truth" (the renderer's ground truth or the twin's oracle) or "detector" (frames only: the app's
  // detector runs on them). mixerPeriod: the radio script's run interval (0: every physics step). augment: see augment.js.
  constructor({
    hfov = 127, uptilt = 20, seed = (Date.now() % 1e6) | 0, headless = false,
    house = null, map = null, twin = null, detections = "truth", mixerPeriod = MIXER_PERIOD, augment = null,
  } = {}) {
    super();
    this.seed = seed;
    this.headless = headless;
    this.hfovDeg = hfov;
    this.uptilt = uptilt * DEG;
    this.detections = detections;
    this.bridge = new BridgeLogic({ period: mixerPeriod });
    this.radio = new SimRadio(this);
    this.pilot = { roll: 0, pitch: 0, yaw: 0, thr: 0, aiSwitch: false, armSwitch: false, ptt: false };
    // A simulated human holding altitude, so co-pilot mode can be tried without a real pilot.
    this.assist = { on: false, targetZ: 1.0, i: 0, landing: false, takeoff: false };
    this.keys = new Set();
    this.time = 0;
    this.nextTelem = 0;
    this.attHistory = [];
    this.lastAtt = { pitch: 0, roll: 0, yaw: 0 };
    this.nextAtt = 0;
    this.gyroDrift = 0;
    this.trail = [];
    this.videoDelay = 0; // ms; emulates the latency of the real video path (e.g. DJI goggles -> Mac)
    this.stats = { frames: 0, fps: 0, renderMs: 0 };
    this.augment = augmentOptions(augment);
    this.augmenter = augmenter(this.augment, seed + 7, mulberry32(seed + 7));
    this.useWorld(house ? makeHouseWorld(house, map, { seed }) : new World(seed), twin);
  }

  get house() {
    return this.world.house ?? null;
  }

  // Fly in a house ({ house, map, twin? }), or back in the demo apartment (no house). The drone starts on the pad.
  setHouse({ house = null, map = null, twin = null } = {}) {
    this.useWorld(house ? makeHouseWorld(house, map, { seed: this.seed }) : new World(this.seed), twin);
    this.emit("world", { kind: this.world.kind, house });
  }

  // Use the splat twin's camera (null: the raycast camera).
  setTwin(twin) {
    this.twin = twin;
    this.twinBusy = false;
    if (twin) twin.setLens(this.twinLens()).catch(() => {});
    this.frameQueue = [];
    this.syncProps();
  }

  // The twin's lens: the O4 at the settings' field of view and uptilt, plus augment.lensError.
  twinLens() {
    return simLens(this.hfovDeg, this.uptilt / DEG, this.augment?.lensError);
  }

  // Realism on the twin's frames from now on (augment.js; null: off).
  setAugment(augment) {
    this.augment = augmentOptions(augment);
    this.augmenter?.dispose?.();
    this.augmenter = augmenter(this.augment, this.seed + 7, mulberry32(this.seed + 7));
    this.twin?.setLens(this.twinLens()).catch(() => {});
  }

  useWorld(world, twin = null) {
    this.world = world;
    this.drone = new SimDrone(world.rand);
    this.renderer = this.headless ? null : new CameraRenderer(world, { ...SIM_VIEW, hfovDeg: this.hfovDeg });
    this.framePool = [];
    this.video = null; // the frame the app currently "receives": { canvas, detections, id, t (when it was taken) }
    this.props = new Map(); // scene changes the twin shows: id -> twin prop
    this.setTwin(world.kind === "house" ? twin : null);
    this.reset();
  }

  setCamera({ hfov, uptilt }) {
    if (hfov) this.renderer?.setFov((this.hfovDeg = hfov));
    if (uptilt !== undefined) this.uptilt = uptilt * DEG;
    this.twin?.setLens(this.twinLens()).catch(() => {});
  }

  reset() {
    const s = this.world.start;
    this.drone.reset(s.x, s.y, s.heading, this.world.floorAt(s.x, s.y) ?? 0);
    Object.assign(this.pilot, { roll: 0, pitch: 0, yaw: 0, thr: 0, armSwitch: false, aiSwitch: false });
    Object.assign(this.assist, { on: false, targetZ: 1.0, i: 0, landing: false, takeoff: false });
    this.yaw0 = this.drone.yaw;
    this.gyroDrift = 0;
    Object.assign(this, { attHistory: [], nextAtt: 0 }); // a flight controller started again: its new zero comes at once
    this.trail = [];
    this.emit("reset");
  }

  // What the app asks the (simulated) human to do in co-pilot mode.
  pilotRequest(kind) {
    const a = this.assist;
    if (kind === "takeoff") {
      this.pilot.armSwitch = true;
      a.on = true;
      a.landing = false;
      a.takeoff = true;
      a.targetZ = 1.0;
    } else if (kind === "land") {
      a.on = true;
      a.landing = true;
    } else if (kind === "higher") {
      a.targetZ = clamp(a.targetZ + 0.35, 0.3, 2.0);
    } else if (kind === "lower") {
      a.targetZ = clamp(a.targetZ - 0.3, 0.35, 2.0);
    } else if (kind === "low") {
      a.targetZ = Math.min(a.targetZ, 0.5);
    }
  }

  // Flip the virtual switches the way a pilot would before handing over.
  prepareForMission({ copilot }) {
    const p = this.pilot;
    this.assist.on = copilot;
    if (!p.armSwitch) {
      p.thr = 0;
      p.armSwitch = true;
    }
    // Re-engage to clear overrides left over from manual flying (switch off, then on).
    if (p.aiSwitch && Object.values(this.bridge.ovr).some(Boolean)) {
      p.aiSwitch = false;
      this.bridge.run(p, this.time, () => {});
    }
    p.aiSwitch = true;
  }

  keyDown(code) {
    this.keys.add(code);
    if (code === "KeyE") this.pilot.armSwitch = !this.pilot.armSwitch;
    if (code === "KeyQ") this.pilot.aiSwitch = !this.pilot.aiSwitch;
    if (code === "KeyR") this.reset();
  }

  keyUp(code) {
    this.keys.delete(code);
  }

  step(dtReal) {
    // rAF timestamps can be slightly behind performance.now(), so the first dt may be negative.
    const dt = Math.min(Math.max(dtReal, 0), 0.05);
    if (dt === 0) return;
    this.time += dt;
    this.readKeys(dt);
    const n = Math.max(1, Math.ceil(dt / (1 / 240)));
    const h = dt / n;
    const beep = (freq, ms, delay = 0) => this.emit("tone", { freq, ms, delay });
    for (let i = 0; i < n; i++) {
      this.assistStep(h);
      const out = this.bridge.outputs(this.pilot, this.time, beep);
      const d = this.drone;
      if (this.pilot.armSwitch && !d.armed && !d.crashed && out.thr < 0.05) d.arm();
      if (!this.pilot.armSwitch && d.armed) d.disarm();
      d.step(h, out, this.world);
      this.world.step(h, d);
      this.lastOut = out;
    }
    this.gyroDrift += 0.05 * DEG * dt;
    this.recordAttitude();
    if (this.time >= this.nextTelem) {
      this.nextTelem = this.time + 0.1;
      this.emitTelemetry();
    }
    if (!this.trail.length || Math.hypot(this.trail.at(-1).x - this.drone.x, this.trail.at(-1).y - this.drone.y) > 0.08) {
      this.trail.push({ x: this.drone.x, y: this.drone.y });
      if (this.trail.length > 400) this.trail.shift();
    }
  }

  readKeys(dt) {
    const k = this.keys;
    const p = this.pilot;
    const axis = (neg, pos, amount) => (k.has(pos) ? amount : 0) - (k.has(neg) ? amount : 0);
    p.pitch = axis("ArrowDown", "ArrowUp", 0.35);
    p.roll = axis("ArrowLeft", "ArrowRight", 0.35);
    p.yaw = axis("KeyA", "KeyD", 0.45);
    this.handsOn = !!(p.pitch || p.roll);
    const up = axis("KeyS", "KeyW", 1);
    if (this.assist.on) this.assist.targetZ = clamp(this.assist.targetZ + up * 0.6 * dt, 0.2, 2.2);
    else p.thr = clamp(p.thr + up * 0.5 * dt, 0, 1);
  }

  // Simulated human thumbs: the throttle holds a target height above the floor using its eyes (true altitude).
  assistStep(h) {
    const a = this.assist;
    const d = this.drone;
    const agl = d.z - d.floorZ;
    if (!a.on || !this.pilot.armSwitch || d.crashed) return;
    if (!d.armed || (!d.airborne && !a.takeoff)) {
      this.pilot.thr = 0; // a pilot arms with the throttle down and waits to be asked to take off
      return;
    }
    if (a.landing) {
      if (!d.airborne) {
        this.pilot.thr = 0;
        this.pilot.armSwitch = false;
        a.landing = a.takeoff = false;
        return;
      }
      a.targetZ = Math.max(-0.3, Math.min(a.targetZ, agl) - 0.4 * h);
    }
    const ge = agl < 0.15 ? 1 + 0.12 * (1 - agl / 0.15) : 1;
    const need = 1 / (SIM_DRONE.twr * (Math.max(3, d.vbat) / 4.2) ** 2 * ge);
    const hover = need ** (1 / SIM_DRONE.thrExpo);
    const err = a.targetZ - agl;
    a.i = clamp(a.i + err * h * 0.05, -0.05, 0.05);
    this.pilot.thr = clamp(hover + 0.12 * err - 0.1 * d.vz + a.i + (d.airborne ? 0 : 0.08), 0, 0.9);
    // The other thumb stops the drift while the AI isn't steering (small sticks: never enough to take over from it).
    if (d.airborne && !this.handsOn) {
      const c = Math.cos(d.yaw), s = Math.sin(d.yaw);
      this.pilot.pitch = clamp(-(c * d.vx + s * d.vy), -0.25, 0.25);
      this.pilot.roll = clamp(-(s * d.vx - c * d.vy), -0.25, 0.25);
    }
  }

  recordAttitude() {
    const d = this.drone;
    // Betaflight heading: clockwise positive, zero where it booted, slowly drifting without a compass.
    const heading = wrapAngle(-(d.yaw - this.yaw0) + this.gyroDrift);
    this.attHistory.push({ t: this.time, pitch: -d.pitch, roll: d.roll, yaw: heading });
    while (this.attHistory.length && this.attHistory[0].t < this.time - 0.5) this.attHistory.shift();
    // ELRS telemetry: attitude arrives ~5 times a second, ~120 ms late.
    if (this.time >= this.nextAtt) {
      this.nextAtt = this.time + 0.2;
      const late = this.attHistory.find((s) => s.t >= this.time - 0.12) || this.attHistory.at(-1);
      this.lastAtt = { pitch: late.pitch, roll: late.roll, yaw: late.yaw };
    }
  }

  emitTelemetry() {
    const d = this.drone;
    const b = this.bridge;
    const p = this.pilot;
    const t = {
      t: performance.now(),
      source: "sim",
      bridge: true,
      seq: b.cmd.seq || 0,
      engaged: b.engaged,
      linkOk: !!b.linkOk,
      failsafe: b.failsafe,
      ptt: p.ptt,
      override: { ...b.ovr },
      sticks: { roll: p.roll, pitch: p.pitch, thr: p.thr, yaw: p.yaw },
      vbat: Math.round(d.vbat * 10) / 10, // CRSF battery has 0.1 V resolution
      lq: 100,
      rssi: -35 - Math.round(Math.hypot(d.x - this.world.start.x, d.y - this.world.start.y) * 2),
      att: { ...this.lastAtt },
      alt: 0,
      fm: d.crashed ? "!ERR*" : d.armed ? "STAB" : "STAB*",
    };
    this.radio.telemetry = t;
    this.radio.emit("telemetry", t);
  }

  // The drone camera: optical axis pitched up by the uptilt (the raycast renderer's convention).
  cameraPose() {
    const d = this.drone;
    const z = Math.max(d.floorZ + 0.035, d.z + 0.025);
    return { x: d.x, y: d.y, z, yaw: d.yaw, pitch: this.uptilt - d.pitch, roll: d.crashed ? Math.PI : d.roll };
  }

  // Call about every frame: renders (the twin: one frame in flight, asynchronously) and delivers the newest frame
  // that is at least videoDelay old.
  renderCamera() {
    const now = performance.now();
    if (this.twin) this.renderTwin(now);
    else {
      const t0 = performance.now(), res = this.renderer.render(this.cameraPose());
      this.stats.renderMs = +(performance.now() - t0).toFixed(1);
      this.queueFrame(now, res.canvas, this.detections === "truth" ? res.detections : null);
    }
    this.deliver(now);
  }

  // At most GPU.cap("camera") frames a second (vision/budget.js: 24 while vision localization or live depth runs), served
  // ahead of live depth while under that rate (localization and depth work on these frames), after it when over. The next
  // frame renders while this one is augmented (the augmenter's worker keeps their order; two waiting at most).
  renderTwin(now) {
    if (this.twinBusy || this.augmenting > 1 || !GPU.take("camera", (this.camSlot ??= {}), now)) return;
    const twin = this.twin, c = this.cameraPose(), truth = this.detections === "truth", behind = GPU.rate("camera", now) < Math.min(GPU.cap("camera", now), BUDGET.cameraFps) * 0.95;
    const pose = { x: c.x, y: c.y, z: c.z, yaw: c.yaw, pitch: this.drone.pitch, roll: c.roll }; // twin: body pose, lens uptilt
    this.twinBusy = true;
    if (this.world.actorSpecs) twin.setActors(this.world.actorSpecs()).catch(() => {});
    const motion = this.augmenter && this.motion();
    const prio = behind ? "cameraBehind" : "camera";
    Promise.all([twin.render(pose, { ...TWIN_VIEW, prio }), truth ? twin.oracle(pose, { ...ORACLE_VIEW, prio }) : null])
      .then(async ([raw, gt]) => {
        if (twin !== this.twin) return raw.close();
        this.twinBusy = false;
        let bmp = raw;
        if (this.augmenter) {
          this.augmenting = (this.augmenting ?? 0) + 1;
          try { bmp = await this.augmenter.apply(raw, motion); } finally { raw.close(); this.augmenting--; }
        }
        if (twin === this.twin) {
          this.stats.renderMs = twin.stats.renderMs;
          GPU.mark("camera", performance.now() - now);
          this.queueFrame(now, bmp, gt && this.oracleDetections(gt, pose));
          this.deliver(performance.now());
        }
        bmp.close();
      })
      .catch((e) => {
        if (twin !== this.twin) return;
        this.twin = null;
        // disposed by its owner (a world change) while a frame was in flight: not a failure
        if (e.message !== "twin disposed") this.emit("error", `The splat camera failed, so the simulator shows the simple one: ${e.message.split("\n")[0]}`);
      })
      .finally(() => twin === this.twin && (this.twinBusy = false));
  }

  // What the augmenter needs: the camera's turn and pitch rates, its focal length, the battery and flight time.
  motion() {
    const d = this.drone, h = this.attHistory, a = h.at(-1), b = h.find((s) => s.t >= this.time - 0.05) ?? a;
    const dt = a && b && a.t > b.t ? a.t - b.t : 0;
    return { yawRate: d.yawRate ?? 0, pitchRate: dt ? (a.pitch - b.pitch) / dt : 0, fx: TWIN_VIEW.width / 2.2, vbat: d.vbat, seconds: this.time };
  }

  // Twin oracle -> detector-style boxes: actors at least 35% visible, their visible part.
  oracleDetections(gt, pose) {
    const at = new Map((this.world.actorSpecs?.() ?? []).map((a) => [a.id, a]));
    return gt
      .filter((a) => a.visibleFraction >= MIN_VISIBLE && a.visibleBox)
      .map((a) => {
        const p = at.get(a.id);
        const dist = p ? Math.hypot(p.x - pose.x, p.y - pose.y) : undefined;
        return { label: a.kind, score: +(0.55 + 0.4 * a.visibleFraction).toFixed(2), box: a.visibleBox, dist, actor: a.id, trackId: a.id };
      });
  }

  queueFrame(t, image, detections) {
    let c = this.framePool.pop();
    const { width, height } = image;
    if (!c || c.width !== width || c.height !== height) c = Object.assign(document.createElement("canvas"), { width, height });
    c.getContext("2d").drawImage(image, 0, 0, width, height);
    this.frameQueue.push({ t, canvas: c, detections });
  }

  deliver(now) {
    let pick = -1;
    for (let i = this.frameQueue.length - 1; i >= 0; i--) {
      if (now - this.frameQueue[i].t >= this.videoDelay) {
        pick = i;
        break;
      }
    }
    if (pick < 0) return;
    for (const f of this.frameQueue.splice(0, pick)) this.framePool.push(f.canvas);
    const f = this.frameQueue[0];
    if (this.video?.canvas === f.canvas) return;
    this.video = { canvas: f.canvas, detections: f.detections, id: (this.video?.id || 0) + 1, t: f.t };
    const s = this.stats, w = (s.window ??= []);
    w.push(now);
    while (w[0] < now - 1000) w.shift();
    s.frames++;
    s.fps = w.length;
  }

  // ---------------------------------------------------------------- scene changes (house worlds)

  // A box that was not there when the house was captured: { id, x, y, w, d, h (m), z (bottom; default the floor), yaw,
  // color }. In the collision world and, as a twin prop, in the picture. Returns the prop.
  addObstacle({ id, x, y, w = 0.4, d = 0.4, h = 0.8, z, yaw = 0, color = "#8a6b4a" }) {
    if (!this.world.addObstacle) throw new Error("scene changes need a house world");
    const prop = { id, kind: "box", x, y, z: z ?? this.world.floorAt(x, y) ?? 0, w, d, h, yaw, color };
    this.world.addObstacle(prop);
    this.props.set(id, prop);
    this.syncProps();
    return prop;
  }

  removeObstacle(id) {
    this.world.removeSolid?.(id);
    this.props.delete(id);
    this.syncProps();
  }

  // Close (open = false) or open a doorway of the house: a door leaf across it in the collision world and the twin.
  setDoor(id, open) {
    if (!this.world.setDoor) throw new Error("scene changes need a house world");
    const leaf = this.world.setDoor(id, open), key = `door:${id}`;
    if (leaf) this.props.set(key, { id: key, kind: "panel", ...leaf, color: "#c9b79c" });
    else this.props.delete(key);
    this.syncProps();
    return !!leaf;
  }

  // The twin draws the props (twin.setProps, MAP3D's part); a twin without it shows the house as captured.
  syncProps() {
    const twin = this.twin;
    if (!twin) return;
    if (typeof twin.setProps !== "function") {
      if (this.props.size && !this.warnedProps) this.emit("error", "This twin can't show scene changes yet (no setProps): the obstacles are only in the collision world.");
      this.warnedProps = this.props.size > 0;
      return;
    }
    twin.setProps([...this.props.values()]).catch(() => {});
  }

  truth() {
    const d = this.drone;
    const cat = this.world.cat;
    return {
      drone: { x: d.x, y: d.y, z: d.z, agl: d.z - d.floorZ, yaw: d.yaw, armed: d.armed, crashed: d.crashed, airborne: d.airborne,
        vbat: d.vbat, soc: d.soc },
      cat: cat ? { x: cat.x, y: cat.y, state: cat.state } : { x: NaN, y: NaN, state: "none" },
      actors: this.world.actors?.list.map((a) => ({ id: a.id, kind: a.kind, x: a.x, y: a.y, z: a.z, state: a.state })) ?? [],
      room: this.world.roomAt(d.x, d.y)?.name || "",
      world: this.world.kind,
      trail: this.trail,
    };
  }
}
