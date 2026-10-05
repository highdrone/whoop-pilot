import { Emitter } from "./util.js";

const KEY = "whoopPilot.settings.v1";

// The Meteor65 Pro II's DJI O4 lens until calibrated: 4:3, equidistant fisheye, 159 deg diagonal, so 127 x 95 deg
// (angles are linear in the image, so hfov = 0.8 x diagonal); the camera mount tilts it up 15-35 deg.
export const LENS = Object.freeze({ hfov: 127, vfov: 95, diagonal: 159, aspect: 4 / 3, uptilt: 20, uptiltMin: 15, uptiltMax: 35 });

export const DEFAULTS = {
  mode: "sim", // "sim" | "real"
  autonomy: "copilot", // "observer" | "copilot" | "full"
  apiKey: "",
  model: "claude-opus-5",
  effort: "low",
  cameraId: "",
  hfov: LENS.hfov, // degrees across the image width (O4: 127 at 4:3; analog 173° diagonal cams ~150)
  uptilt: LENS.uptilt, // camera tilt, degrees (LENS.uptiltMin..uptiltMax)
  videoDelay: 0, // ms from camera to this app (Goggles 3 via the helper ~90, window capture ~150-300)
  crop: null, // part of the video frame that is the drone's view, normalized {x, y, w, h}
  angleLimit: 60, // Betaflight angle_limit
  yawCenter: 70, // Betaflight Actual rates: center sensitivity (deg/s)
  yawMax: 670, // Betaflight Actual rates: max rate (deg/s)
  maxTilt: 0.3, // stick fraction the AI may use for roll/pitch
  maxYawRate: 110, // deg/s
  speed: 0.35, // m/s cruise for moves (estimated)
  followDistance: "medium",
  handsFree: false,
  wakeWord: "drone",
  speak: true,
  voiceName: "",
  homeNotes: "",
  detector: "rfdetr", // "rfdetr" (WebGPU, falls back to MediaPipe on its own) | "mediapipe"
  detectorQuality: "fast", // "fast" | "accurate"
  simMode: "demo", // simulator world: "demo" (the apartment) | "house" (houseId, with its splat twin when it loads)
  houseId: "", // the active house: OPFS /houses/<id>/ (house/store.js)
  simDetections: "truth", // in the simulator: "truth" (ground-truth boxes) | "detector" (run the detector on its frames)
  simActors: true, // people and pets in the simulated house
  locSource: "fused", // the real drone's position: "fused" | "odometry"
  startOnPad: true, // a mission without a position starts from the home pad if the drone hasn't flown since
  patrolAlerts: true, // a person found on patrol is an alert (speech, Mac notification, ntfy)
  ntfyTopic: "", // phone pushes through ntfy (empty: off); ntfyServer overrides https://ntfy.sh
  ntfyServer: "",
  // Wave C (docs/HOME-DRONE.md)
  aiVision: "ask", // images of the house to Claude (survey, change and detection checks): "ask" | "on" | "off"
  aiBudget: 0.5, // US$ per flight for those checks (a survey asks with its own estimate)
  locVision: true, // splat localization (nav/splatloc.js) on the real drone
  simLoc: "truth", // the simulator flying the house: "truth" (its true pose) | "vision" (splat localization from its own video)
  simAugment: false, // the simulator's twin frames with blur, noise, compression, an OSD and a lens error (sim/augment.js)
  simRehearse: false, // "Rehearse the real flight": simLoc "vision", simAugment, live depth on and the real drone's speed caps (nav/avoid.js)
  avoid: true, // speed caps and replanning from the 3D map and live depth (nav/avoid.js)
  depthModel: "dav2s", // monocular depth (vision/depth.js): Depth Anything V2 Small
  brake: null, // { decel, react, v0, at }: the real drone's braking, measured by the "calibrate" mission (nav/avoid.js)
  simBrake: null, // the same measured in the simulator: the simulator's only (nav/avoid.js brakeKey)
  flightSeconds: 240, // a full pack's flight time (safety.js battery estimate)
  aiModel: "", // Claude's picture checks and the survey (ai/*); empty: the brain's model
  memoryDays: 90, // flight memory (memory/memory.js) keeps this many days...
  memoryFlights: 200, // ...and at most this many flights
};

const AI_VISION = ["ask", "on", "off"];

// Stored values older settings wrote that are now wrong: the O4 lens was entered as 140 deg wide.
const MIGRATE = { hfov: [140, LENS.hfov] };

export class Settings extends Emitter {
  constructor() {
    super();
    this.values = { ...DEFAULTS };
    try {
      Object.assign(this.values, JSON.parse(localStorage.getItem(KEY) || "{}"));
    } catch {}
    if (!this.values.lensModel) {
      for (const [k, [from, to]] of Object.entries(MIGRATE)) if (this.values[k] === from) this.values[k] = to;
      this.values.lensModel = 1;
    }
    this.values.uptilt = clampUptilt(this.values.uptilt);
    this.values.aiVision = checked("aiVision", this.values.aiVision);
    this.values.aiBudget = checked("aiBudget", this.values.aiBudget);
  }
  get(key) {
    return this.values[key];
  }
  all() {
    return this.values;
  }
  set(key, value) {
    if (key === "uptilt") value = clampUptilt(value);
    value = checked(key, value);
    if (this.values[key] === value) return;
    this.values[key] = value;
    try {
      localStorage.setItem(KEY, JSON.stringify(this.values));
    } catch {}
    this.emit("change", { key, value });
  }
}

// Images leave the Mac only with an answer the app understands; a budget is a non-negative number of dollars.
function checked(key, v) {
  if (key === "aiVision") return AI_VISION.includes(v) ? v : DEFAULTS.aiVision;
  if (key === "aiBudget") return Number.isFinite(+v) && +v >= 0 ? +v : DEFAULTS.aiBudget;
  return v;
}

function clampUptilt(v) {
  const n = Number(v);
  return Number.isFinite(n) ? Math.max(LENS.uptiltMin, Math.min(LENS.uptiltMax, n)) : LENS.uptilt;
}
