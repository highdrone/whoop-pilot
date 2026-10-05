// Flies the simulated whoop with the real flight controller and behaviors, headless and faster than
// real time. Camera motion and detections are synthesized from the simulator's ground truth (with
// noise and an unknown scene depth), so this checks the control logic, not the vision.
// The radio bridge runs every 50 ms like the EdgeTX mixer script. test-house-sim.mjs reuses rig() in a real house.
// Usage: cd tools && npm test
import assert from "node:assert/strict";
import { fileURLToPath } from "node:url";

let clock = 0;
Object.defineProperty(globalThis, "performance", { value: { now: () => clock }, configurable: true, writable: true });
export const now = () => clock;

const { Simulator, SIM_VIEW } = await import("../app/js/sim/simulator.js");
const { FlightController } = await import("../app/js/controller.js");
const { DEFAULTS, LENS } = await import("../app/js/settings.js");
const B = await import("../app/js/behaviors.js");
const { DEG, wrapAngle } = await import("../app/js/util.js");
const { mulberry32, WALL_H } = await import("../app/js/sim/world.js");

// house, map: fly in that house (twin-world.js) instead of the demo apartment.
export function rig({ autonomy = "full", seed = 7, videoDelay = 0, realCamera = false, rateError = 1, house, map, aspect = SIM_VIEW.height / SIM_VIEW.width } = {}) {
  const values = { ...DEFAULTS, autonomy, videoDelay };
  // rateError > 1: the quad actually turns faster than the app's Betaflight-rate settings say.
  values.yawCenter /= rateError;
  values.yawMax /= rateError;
  const settings = { get: (k) => values[k], all: () => values, set: (k, v) => (values[k] = v), on() {} };
  const sim = new Simulator({ headless: true, seed, house, map });
  const rand = mulberry32(seed + 1);
  const perception = new FakePerception(sim, values, rand, realCamera, aspect);
  const ctl = new FlightController({ settings, perception });
  ctl.attach(sim.radio, "test");
  ctl.on("pilot-request", ({ kind }) => sim.pilotRequest(kind));
  sim.prepareForMission({ copilot: autonomy === "copilot" });
  let n = 0;
  const step = (controllerAlive = true) => {
    clock += 1000 / 60;
    sim.step(1 / 60);
    if (n++ % 2 === 0) {
      perception.update();
      if (controllerAlive) ctl.tick();
    }
  };
  const run = async (behavior, maxSeconds = 60) => {
    let result = null;
    ctl.run(behavior).then((r) => (result = r));
    for (let t = 0; t < maxSeconds && !result; t += 1 / 60) {
      step();
      await Promise.resolve();
    }
    return result || { ok: false, text: "test timeout" };
  };
  const wait = (seconds, alive = true) => {
    for (let t = 0; t < seconds; t += 1 / 60) step(alive);
  };
  wait(0.5);
  return { sim, ctl, run, wait, values, perception };
}

// Ground-truth "camera": image motion and object boxes as the real pipeline would report them. aspect: frame
// height / width (the simulator's own frames: the O4's 4:3 in both worlds). Walls hide targets; in a house so does
// anything else solid at the target's mid height (furniture boxes, door headers), unless it sits on that furniture.
export class FakePerception {
  constructor(sim, settings, rand, realCamera, aspect = SIM_VIEW.height / SIM_VIEW.width) {
    Object.assign(this, { sim, settings, rand, realCamera, aspect, W: 320, H: Math.round(320 * aspect) });
    this.prevRoll = 0;
    this.latest = { t: 0, detections: [], flow: null, width: this.W, height: this.H };
    this.prevPitch = 0;
    this.queue = []; // frames in flight: the app sees them settings.videoDelay ms late
  }
  get frameAge() {
    return clock - this.latest.t;
  }
  find(label, min = 0.4) {
    return this.latest.detections.filter((d) => d.label === label && d.score >= min);
  }
  snapshot() {
    return null;
  }
  update() {
    const d = this.sim.drone;
    const hfov = this.settings.hfov * DEG;
    const vfov = hfov * this.aspect;
    const dt = 1 / 30;
    const fx = Math.cos(d.yaw);
    const fy = Math.sin(d.yaw);
    const vFwd = d.vx * fx + d.vy * fy;
    const vRight = d.vx * fy - d.vy * fx;
    const depth = 1.4 + 1.6 * this.rand(); // the real scene depth is unknown to the controller
    const pitchRate = (d.pitch - this.prevPitch) / dt;
    this.prevPitch = d.pitch;
    const rollRate = (d.roll - this.prevRoll) / dt;
    this.prevRoll = d.roll;
    // A real camera tilted up by a: yaw pans it by cos(a) and body roll pans it by sin(a).
    const a = this.realCamera ? this.settings.uptilt * DEG : 0;
    const pan = -d.yawRate * Math.cos(a) + rollRate * Math.sin(a); // rad/s, + = view swings right
    const noise = () => (this.rand() - 0.5) * 0.02;
    const onGround = !d.airborne;
    const flow = onGround && d.z - d.floorZ < 0.02
      ? { dx: noise(), dy: noise(), div: noise(), rot: 0, quality: 0.8, t: clock }
      : {
          dx: -vRight / (hfov * depth) - pan / hfov + noise(),
          dy: d.vz / (vfov * depth) - pitchRate / vfov + noise(),
          div: vFwd / depth + noise(),
          rot: 0,
          quality: 0.8,
          t: clock,
        };
    this.queue.push({ detections: this.detect(hfov, vfov), flow, born: clock });
    while (this.queue.length > 1 && clock - this.queue[1].born >= this.settings.videoDelay) this.queue.shift();
    const f = this.queue[0];
    if (clock - f.born >= this.settings.videoDelay) {
      this.latest = { t: clock, detections: f.detections, flow: f.flow && { ...f.flow, t: clock }, width: this.W, height: this.H };
    }
  }
  detect(hfov, vfov) {
    const d = this.sim.drone;
    const camPitch = this.settings.uptilt * DEG - d.pitch;
    const out = [];
    for (const s of this.sim.world.sprites()) {
      if (!s.label) continue;
      const dx = s.x - d.x;
      const dy = s.y - d.y;
      const dist = Math.hypot(dx, dy);
      if (dist < 0.15) continue;
      const rel = wrapAngle(Math.atan2(dy, dx) - d.yaw);
      const half = Math.atan2(s.w, dist);
      if (Math.abs(rel) + half * 0.5 > hfov / 2) continue;
      const world = this.sim.world, eye = d.z + 0.025, mid = (s.z0 + s.z1) / 2;
      const wall = world.castRay(d.x, d.y, dx / dist, dy / dist);
      if (wall && wall.dist < dist) continue;
      if (world.kind === "house" && !s.depthBias) {
        // the sight line at the target's distance minus its size, at the height it passes there
        const k = Math.max(0, dist - s.r - 0.05), z = eye + ((mid - eye) * k) / dist;
        const hit = world.castRay(d.x, d.y, dx / dist, dy / dist, k, z);
        if (hit && (hit.seg.kind === "box" || hit.seg.kind === "header" || hit.seg.kind === "door")) continue;
      }
      const top = 0.5 - (Math.atan2(s.z1 - d.z, dist) - camPitch) / vfov;
      const bot = 0.5 - (Math.atan2(s.z0 - d.z, dist) - camPitch) / vfov;
      const y0 = Math.max(0, top);
      const y1 = Math.min(1, bot);
      if (y1 - y0 < 0.04) continue;
      const cx = 0.5 - rel / hfov;
      const w = (2 * half) / hfov;
      out.push({ label: s.label, score: 0.85, box: { x: cx - w / 2, y: y0, w, h: y1 - y0 } });
    }
    return out;
  }
}

// What the app asks for: tools.js SIZES.cat.medium, the cat's height as a fraction of the frame's.
export const CAT_SIZE = 0.22;

export const trueHeading = (sim) => -sim.drone.yaw; // clockwise, radians (continuous)

const tests = {
  async "full auto: takes off, hovers roughly in place, lands"() {
    const { sim, ctl, run } = rig();
    const r = await run(new B.TakeOff(), 10);
    assert.ok(r.ok, r.text);
    assert.ok(sim.drone.airborne, "airborne after take-off");
    const start = { x: sim.drone.x, y: sim.drone.y };
    const h = await run(new B.Hold(8), 12);
    assert.ok(h.ok);
    const z = sim.drone.z;
    const drift = Math.hypot(sim.drone.x - start.x, sim.drone.y - start.y);
    console.log(`      hover: z=${z.toFixed(2)} m, drift ${drift.toFixed(2)} m in 8 s, hover model ${ctl.hover.estimate(sim.drone.vbat).toFixed(3)}`);
    assert.ok(z > 0.35 && z < WALL_H - 0.3, `height ${z.toFixed(2)}`);
    assert.ok(drift < 1.5, `drifted ${drift.toFixed(2)} m`);
    const l = await run(new B.Land(), 12);
    assert.ok(l.ok, l.text);
    assert.ok(!sim.drone.airborne, "on the ground");
    assert.ok(!sim.drone.crashed, "not crashed");
  },

  async "full auto: turns 90 degrees right and 135 left"() {
    const { sim, run } = rig();
    await run(new B.TakeOff(), 10);
    let h0 = trueHeading(sim);
    let r = await run(new B.Turn(90), 10);
    assert.ok(r.ok, r.text);
    let turned = (trueHeading(sim) - h0) / DEG;
    console.log(`      turned ${turned.toFixed(0)}° (asked 90)`);
    assert.ok(Math.abs(turned - 90) < 15, `turned ${turned.toFixed(1)}°`);
    h0 = trueHeading(sim);
    r = await run(new B.Turn(-135), 10);
    turned = (trueHeading(sim) - h0) / DEG;
    console.log(`      turned ${turned.toFixed(0)}° (asked -135)`);
    assert.ok(Math.abs(turned + 135) < 18, `turned ${turned.toFixed(1)}°`);
  },

  async "full auto: moves forward about the requested distance"() {
    const { sim, run } = rig();
    await run(new B.TakeOff(), 10);
    await run(new B.Turn(-90), 10); // face west, into the open living room
    const p0 = { x: sim.drone.x, y: sim.drone.y, yaw: sim.drone.yaw };
    const r = await run(new B.Move("forward", 1.5, 0.35), 12);
    await run(new B.Hold(1.5), 3);
    const dx = sim.drone.x - p0.x;
    const dy = sim.drone.y - p0.y;
    const along = dx * Math.cos(p0.yaw) + dy * Math.sin(p0.yaw);
    console.log(`      moved ${along.toFixed(2)} m forward (asked 1.5): ${r.text}`);
    assert.ok(along > 0.7 && along < 2.8, `moved ${along.toFixed(2)} m`);
  },

  async "full auto: finds the cat, approaches it, then follows it"() {
    const { sim, run } = rig({ seed: 3 });
    sim.world.cat.state = "sit";
    sim.world.cat.timer = 60;
    Object.assign(sim.world.cat, { x: 8.8, y: 6.6, z: 0 });
    await run(new B.TakeOff(), 10);
    const f = await run(new B.Search("cat"), 25);
    assert.ok(f.ok, f.text);
    const a = await run(new B.Approach("cat", { size: CAT_SIZE }), 45);
    const d = Math.hypot(sim.world.cat.x - sim.drone.x, sim.world.cat.y - sim.drone.y);
    console.log(`      approach: "${a.text}" ending ${d.toFixed(2)} m from the cat, drone z=${sim.drone.z.toFixed(2)}`);
    assert.ok(a.ok, a.text);
    assert.ok(d > 0.45 && d < 2.2, `distance ${d.toFixed(2)}`);
    // let the cat walk away and follow it
    sim.world.cat.startWandering();
    const fol = await run(new B.Approach("cat", { size: CAT_SIZE, follow: true, duration: 20 }), 25);
    const d2 = Math.hypot(sim.world.cat.x - sim.drone.x, sim.world.cat.y - sim.drone.y);
    console.log(`      follow: "${fol.text}", ${d2.toFixed(2)} m from the cat at the end`);
    assert.ok(!sim.drone.crashed, "no crash while following");
  },

  async "with 250 ms of video delay (DJI goggles -> Mac): hovers, finds and follows the cat"() {
    const { sim, run } = rig({ seed: 5, videoDelay: 250 });
    Object.assign(sim.world.cat, { x: 8.8, y: 6.6, z: 0, state: "sit", timer: 60 });
    await run(new B.TakeOff(), 10);
    const start = { x: sim.drone.x, y: sim.drone.y };
    await run(new B.Hold(6), 8);
    const drift = Math.hypot(sim.drone.x - start.x, sim.drone.y - start.y);
    const f = await run(new B.Search("cat"), 30);
    assert.ok(f.ok, f.text);
    sim.world.cat.startWandering();
    const fol = await run(new B.Approach("cat", { size: CAT_SIZE, follow: true, duration: 20 }), 25);
    const d = Math.hypot(sim.world.cat.x - sim.drone.x, sim.world.cat.y - sim.drone.y);
    console.log(`      delayed video: hover drift ${drift.toFixed(2)} m in 6 s; follow "${fol.text}", ${d.toFixed(2)} m from the cat`);
    assert.ok(drift < 1.5, `drifted ${drift.toFixed(2)} m`);
    assert.ok(!sim.drone.crashed);
  },

  async "real-camera geometry and mis-set rates: calibrates itself and still turns and holds"() {
    const { sim, ctl, run } = rig({ seed: 11, realCamera: true, rateError: 1.2 });
    await run(new B.TakeOff(), 10);
    for (const deg of [90, -90, 120, -120]) {
      await run(new B.Turn(deg), 10);
      await run(new B.Hold(1), 2); // missions pause between moves; complete turns are what it learns from
    }
    const h0 = trueHeading(sim);
    const r = await run(new B.Turn(90), 10);
    const turned = (trueHeading(sim) - h0) / DEG;
    const start = { x: sim.drone.x, y: sim.drone.y };
    await run(new B.Hold(8), 10);
    const drift = Math.hypot(sim.drone.x - start.x, sim.drone.y - start.y);
    const hfov = DEFAULTS.hfov * DEG;
    const learned = -ctl.rotX.k[0] * hfov * ctl.rateScale; // image pan per commanded turn, all included
    const truth = 1.2 * Math.cos(DEFAULTS.uptilt * DEG);
    console.log(`      learned turn-rate scale ${ctl.rateScale.toFixed(2)} (truth 1.20), yaw->image gain ${learned.toFixed(2)} (truth ${truth.toFixed(2)}); turned ${turned.toFixed(0)}° (asked 90), hover drift ${drift.toFixed(2)} m`);
    assert.ok(r.ok, r.text);
    assert.ok(Math.abs(ctl.rateScale - 1.2) < 0.1, "turn-rate scale learned");
    assert.ok(Math.abs(learned - truth) < 0.2, "rotation model converged");
    assert.ok(Math.abs(turned - 90) < 12, `turned ${turned.toFixed(0)}`);
    assert.ok(drift < 1.5, `drift ${drift.toFixed(2)}`);
  },

  async "co-pilot: asks the (simulated) pilot to take off, then steers"() {
    const { sim, ctl, run } = rig({ autonomy: "copilot" });
    ctl.askPilot("takeoff", "take off please");
    const w = await run(new B.WaitFor("wait", (c) => c.isFlying(), 25, "flying", "not flying"), 26);
    assert.ok(w.ok, w.text);
    await run(new B.Hold(2), 4);
    const h0 = trueHeading(sim);
    const r = await run(new B.Turn(60), 10);
    const turned = (trueHeading(sim) - h0) / DEG;
    console.log(`      co-pilot turn ${turned.toFixed(0)}° (asked 60), z=${sim.drone.z.toFixed(2)}`);
    assert.ok(r.ok && Math.abs(turned - 60) < 15);
    assert.ok(sim.drone.z > 0.5, "pilot kept altitude");
  },

  async "the cat approach at the app's own size (0.22 of the frame), seeds 1-12, in the simulator's 4:3 frames"() {
    assert.equal(SIM_VIEW.width / SIM_VIEW.height, LENS.aspect, "the simulator renders the lens's own shape");
    const out = [];
    for (let seed = 1; seed <= 12; seed++) {
      const { sim, run } = rig({ seed });
      Object.assign(sim.world.cat, { x: 8.8, y: 6.6, z: 0, state: "sit", timer: 60 });
      await run(new B.TakeOff(), 10);
      const f = await run(new B.Search("cat"), 25);
      const a = f.ok ? await run(new B.Approach("cat", { size: CAT_SIZE }), 45) : f;
      out.push({ seed, ok: a.ok, text: a.text, d: Math.hypot(sim.world.cat.x - sim.drone.x, sim.world.cat.y - sim.drone.y) });
    }
    const bad = out.filter((r) => !r.ok);
    console.log(`      ${12 - bad.length}/12 arrived, ${Math.min(...out.map((r) => r.d)).toFixed(2)}-${Math.max(...out.map((r) => r.d)).toFixed(2)} m from the cat${bad.length ? `; not: ${bad.map((r) => `${r.seed} "${r.text}"`).join(", ")}` : ""}`);
    assert.ok(bad.length <= 1, `${bad.length} of 12 approaches failed`);
  },

  "battery: the pack drains once it is plugged in (the first arm), so a demo left open stays charged"() {
    const sim = new Simulator({ headless: true, seed: 1 });
    for (let t = 0; t < 30 * 60; t += 0.05) sim.step(0.05); // half an hour on the pad, disarmed
    const idle = sim.drone.soc;
    sim.pilotRequest("takeoff");
    for (let t = 0; t < 10; t += 1 / 60) sim.step(1 / 60);
    const loaded = sim.drone.vbat, flying = sim.drone.airborne;
    sim.pilotRequest("land");
    for (let t = 0; t < 10; t += 1 / 60) sim.step(1 / 60);
    const landed = sim.drone.soc;
    for (let t = 0; t < 10 * 60; t += 0.05) sim.step(0.05); // ten minutes disarmed after the flight: the O4 on standby
    const standby = landed - sim.drone.soc;
    console.log(`      ${Math.round(idle * 100)}% after 30 min unarmed; hovering at ${loaded.toFixed(2)} V; then ${Math.round(standby * 100)}% in 10 min of standby`);
    assert.equal(idle, 1);
    assert.ok(flying && loaded > 3.9, `hover on a fresh pack: ${loaded.toFixed(2)} V`);
    assert.ok(!sim.drone.armed && standby > 0.2 && standby < 0.35, `standby used ${standby}`);
  },

  async "perception: the detector's status goes out whole (download progress included), warnings as errors"() {
    const restore = fakeCanvas();
    try {
      const { Perception } = await import("../app/js/perception.js");
      const p = new Perception(), status = [], errors = [];
      p.on("status", (s) => status.push(s));
      p.on("error", (t) => errors.push(t));
      for (const pct of [1, 2, 3]) p.detector.emit("status", { text: `Downloading the detector model (once): ${pct}% of 108 MB`, level: "info", progress: pct / 100 });
      p.detector.emit("status", { text: "Detector: RF-DETR on WebGPU.", level: "info" });
      p.detector.emit("status", { text: "RF-DETR detector unavailable (no WebGPU); loading MediaPipe instead.", level: "warn" });
      assert.deepEqual(status.map((s) => s.progress), [0.01, 0.02, 0.03, undefined]);
      assert.equal(status[3].text, "Detector: RF-DETR on WebGPU.");
      assert.deepEqual(errors, ["RF-DETR detector unavailable (no WebGPU); loading MediaPipe instead."]);
    } finally {
      restore();
    }
  },

  async "perception: its frame rate falls to 0 within a second of the frames stopping, and starts again with a new source"() {
    const restore = fakeCanvas();
    try {
      const { Perception } = await import("../app/js/perception.js");
      const p = new Perception(), canvas = document.createElement("canvas");
      let id = 0;
      const source = (kind = "sim") => ({ kind, ready: () => true, element: () => canvas, region: () => ({ sx: 0, sy: 0, sw: 320, sh: 240 }), frameId: () => id, detections: () => [] });
      p.setSource(source());
      for (let i = 0; i < 60; i++) ((clock += 1000 / 30), id++, p.process());
      const running = p.fps;
      for (let i = 0; i < 15; i++) ((clock += 1000 / 30), p.process()); // the same frame: nothing new
      const half = p.fps;
      clock += 700;
      p.process();
      assert.ok(Math.abs(running - 30) <= 1 && Math.abs(half - 15) <= 1, `${running} fps running, ${half} half a second after the last frame`);
      assert.equal(p.fps, 0, "no frame for over a second");
      id++, p.process();
      assert.equal(p.fps, 1);
      p.setSource(source("camera"));
      assert.equal(p.fps, 0, "a new source counts its own frames");
    } finally {
      restore();
    }
  },

  async "radio failsafe: if the app stops, the quad descends and idles instead of flying away"() {
    const { sim, run, wait } = rig();
    await run(new B.TakeOff(), 10);
    await run(new B.Hold(2), 4);
    assert.ok(sim.drone.airborne);
    wait(7, false); // controller stops sending
    assert.ok(sim.bridge.failsafe, "bridge latched failsafe");
    assert.ok(!sim.drone.airborne, "on the ground");
    assert.ok(!sim.drone.crashed, `soft landing (last impact ${sim.drone.lastImpact.toFixed(2)} m/s)`);
  },
};

// Enough of Canvas 2D for the renderers and Perception: image data kept, drawing calls ignored (sprites come out blank).
export function fakeCanvas() {
  const ctx = (c) => new Proxy({
    createImageData: (w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }),
    getImageData: (x, y, w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }),
    createLinearGradient: () => ({ addColorStop() {} }),
    canvas: c,
  }, { get: (o, k) => (k in o ? o[k] : () => {}), set: () => true });
  const had = globalThis.document;
  globalThis.document = { createElement: () => { const c = { width: 300, height: 150 }; c.getContext = () => ctx(c); return c; } };
  return () => (globalThis.document = had);
}

export async function runTests(tests) {
  let failed = 0;
  for (const [name, fn] of Object.entries(tests)) {
    try {
      await fn();
      console.log(`  ok  ${name}`);
    } catch (e) {
      failed++;
      console.log(`FAIL  ${name}\n      ${e.message}`);
    }
  }
  console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed`);
  process.exit(failed ? 1 : 0);
}

if (process.argv[1] === fileURLToPath(import.meta.url)) await runTests(tests);
