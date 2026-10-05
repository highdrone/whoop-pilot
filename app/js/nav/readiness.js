// Ready to fly? The O4 sits on the ground for only about 2.5 min with its battery in before it overheats (O4.standby), so
// everything that can happen before the battery goes in does (prepare(): the vision models downloaded and loaded, the 3D
// map, the position database; the camera calibrated from a pad turn recorded earlier, nav/replay.js), and with the battery
// in only the camera picture is found in the goggles' video (a few seconds, by itself) and the pad check runs (the pad
// button, about 3 s). The O4's clock runs from the first sign of power: the radio's battery reading (vbat and link
// quality in the telemetry) or the goggles' video (the O4 streams only with its battery in; from the video it's "about":
// it boots for a while first). It starts again only on a sign the battery went out: the radio saying "no drone" with no
// video meanwhile, a fresh pack's voltage, or neither sign for a minute. A pause of one alone isn't: the goggles sleep
// when taken off, the radio's USB gets re-plugged. items() lists what is done and what isn't as pre-flight checklist
// items (missions.preflightCheck's shape: { id, phase "before" | "battery", ok, level "block" | "warn" | "info", text,
// fix?: { label, action } }, action "prepare" (run(action) does it), "calibrate" (Settings -> Recordings: a recorded pad
// turn) or "pad" (the pad button)).
//   const ready = new Readiness({ splat, depth, perception, settings, prepare3D });   // prepare3D({ onProgress }): main.js's 3D map
//   ready.attach(perception);                      // the battery-in clock's video side (the radio side: ctl "telemetry")
//   await ready.prepare({ onProgress });           // onProgress({ text, progress 0-1, step }): one line that updates
//   ready.items({ mode: "real" });                 // sync; "change" events when something moved
import { Emitter } from "../util.js";
import { MODELS, isCached } from "../vision/models.js";
import { DEPTH_MODELS, pictureProblem } from "../vision/depth.js";

// s on the ground before it overheats; warn with this many left; ms of the radio saying "no drone" (link quality 0 or no
// battery reading) with no video since = battery out; a reading this many volts over the last one = a fresh pack (unless
// the drone flew in the last `landed` ms: a pack recovers from its sag on landing; or the video never stopped); ms
// between frames that still count as one run of video; ms with neither video nor a battery reading = battery out
export const O4 = { standby: 150, warnLeft: 60, unplugged: 3000, freshV: 0.15, landed: 10000, live: 2000, gap: 60000 };
// What prepare() does, in order, with weights for its one progress line (about MB to download, or seconds of work).
export const PREPARE = [
  { id: "vision", text: "the vision worker (XFeat, OpenCV)", weight: 5 },
  { id: "dino", text: "the relocalization model (DINOv2, 44 MB once)", weight: 44 },
  { id: "depth", text: "the depth model (Depth Anything V2, 99 MB once)", weight: 99 },
  { id: "detector", text: "the person and pet detector (RF-DETR, 108 MB once)", weight: 108 },
  { id: "map3d", text: "the 3D map", weight: 25 },
  { id: "posdb", text: "the position database", weight: 30 },
];
const clock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;

export class Readiness extends Emitter {
  constructor({ splat = null, depth = null, perception = null, settings = null, prepare3D = null, has3D = null, ctl = null } = {}) {
    super();
    Object.assign(this, { splat, depth, perception, settings, prepare3D, has3D, cached: {}, running: null, failed: {} });
    this.power = { since: null, by: null, seen: -Infinity, radio: -Infinity, frame: -Infinity, run: -Infinity, off: null, last: null, flew: -Infinity };
    this.watchBattery(ctl ?? splat?.ctl);
  }
  // New parts (a house switch, the real drone's twin): { splat, depth, perception, settings, prepare3D, has3D (() => bool), ctl }.
  set(parts) {
    Object.assign(this, parts);
    if (parts.ctl || parts.splat) this.watchBattery(parts.ctl ?? this.splat?.ctl);
    this.emit("change");
  }

  // The clock's video side: the real camera's frames.
  attach(perception) {
    this.unsub?.();
    this.perception = perception;
    this.unsub = perception.on("frame", () => perception.source && perception.source.kind !== "sim" && this.frame());
  }
  frame(now = performance.now()) {
    const p = this.power, out = this.out(now);
    if (now - p.frame > O4.live) p.run = now;
    p.frame = now;
    this.powered(now, "video", out);
  }
  // The clock's radio side: the controller's "telemetry" (vbat, lq) from the real radio.
  watchBattery(ctl) {
    if (ctl === this.ctl && this.offTel) return;
    this.offTel?.();
    this.ctl = ctl;
    this.offTel = ctl?.on?.("telemetry", (t) => this.battery(t));
  }
  battery(t, now = performance.now()) {
    const p = this.power, v = t?.vbat;
    if (t?.source === "sim") return;
    if (this.ctl?.isFlying?.()) p.flew = now;
    if (!(v > 2.5 && (t.lq ?? 1) > 0)) return void (p.off ??= now); // the radio says "no drone"
    const fresh = p.last != null && v > p.last + O4.freshV && now - p.flew > O4.landed && !this.ran(p.radio, now);
    this.powered(now, "radio", this.out(now) || fresh);
    Object.assign(p, { radio: now, last: v, off: null });
  }
  powered(now, by, restart) {
    const p = this.power;
    if (p.since == null || restart) Object.assign(p, { since: now, by, off: null }), this.emit("change");
    p.seen = now;
  }
  // The video ran without a break from t to now.
  ran(t, now) {
    return this.power.run <= t && now - this.power.frame <= O4.live;
  }
  // The battery is out: the radio has said "no drone" for O4.unplugged with no video since, or no sign of power for O4.gap.
  out(now) {
    const p = this.power;
    return (p.off != null && now - p.off > O4.unplugged && p.frame < p.off) || now - p.seen > O4.gap;
  }
  // { seconds (with the battery in so far), left, approx (timed from the video, or the radio's reading paused), by
  // ("radio" | "video": what started it) } or null (no battery in).
  standby(now = performance.now()) {
    const p = this.power;
    if (p.since == null || this.out(now)) return null;
    const seconds = (now - p.since) / 1000;
    return { seconds, left: O4.standby - seconds, approx: p.by === "video" || now - p.radio > O4.unplugged, by: p.by };
  }

  // What is downloaded already (Cache Storage), and the position database for this house and lens (the stored one): call
  // it when the checklist is shown (prepare() does at its end).
  async refresh() {
    const s = this.splat, depthKey = this.settings?.get?.("depthModel") ?? "dav2s", gpu = !!globalThis.navigator?.gpu;
    const [dino, depth, detector] = await Promise.all([isCached(MODELS[gpu ? "dinov2-s" : "dinov2-s-q8"]), isCached(DEPTH_MODELS[depthKey] ?? DEPTH_MODELS.dav2s), isCached(MODELS["rfdetr-n"])].map((p) => p.catch(() => false)));
    if (s && !s.db && s.store && s.house) await s.loadDb?.().catch(() => null);
    this.cached = { dino, depth, detector };
    this.emit("change");
    return this.cached;
  }

  // Each part's state: "done" | "missing" | "running" | "failed".
  state(id) {
    const s = this.splat, f = s?.features;
    const done = {
      vision: !!f?.alive,
      dino: !!f?.info?.dinoLoaded || !!this.cached.dino,
      depth: !!this.depth?.loaded || !!this.cached.depth,
      detector: !!this.perception?.detector?.ready || !!this.cached.detector,
      map3d: this.has3D ? !!this.has3D() : true,
      posdb: !!s?.db && (!s.dbKey || !s.features || s.db.meta?.key === s.dbKey()),
    }[id];
    return done ? "done" : this.running?.step === id ? "running" : this.failed[id] ? "failed" : "missing";
  }

  // The checklist's vision part for mode "real" | "sim" (sim: only while the simulator localizes from its own video).
  items({ mode = this.settings?.get?.("mode") ?? "sim", now = performance.now() } = {}) {
    const s = this.splat, real = mode === "real", vision = !!s && (real ? this.settings?.get?.("locVision") !== false : this.settings?.get?.("simLoc") === "vision");
    if (!vision) return [];
    const out = [], add = (id, phase, ok, level, text, fix) => out.push({ id, phase, ok, level: ok ? "info" : level, text, ...(fix && !ok && { fix }) });
    const prep = { label: "Prepare now", action: "prepare" }, run = this.running;
    const need = ["vision", "dino", ...(this.settings?.get?.("avoid") !== false ? ["depth"] : []), "detector"], missing = need.filter((id) => this.state(id) !== "done");
    add("models", "before", !missing.length, real ? "block" : "warn", !missing.length ? "Vision models: downloaded and ready."
      : run && need.includes(run.step) ? `Getting ready: ${run.text}` : `Get the vision models ready before the battery goes in: ${missing.map((id) => PREPARE.find((p) => p.id === id).text).join(", ")}.`, prep);
    const cpu = s.cpuOnly?.() || (globalThis.navigator && !navigator.gpu ? "this browser has no WebGPU: vision localization would run on the CPU, too slow for the real drone" : "");
    add("webgpu", "before", !cpu, real ? "block" : "warn", cpu ? `${cpu[0].toUpperCase()}${cpu.slice(1)}.` : "Vision runs on the graphics chip (WebGPU).");
    const db = this.state("posdb");
    add("posdb", "before", db === "done", real ? "block" : "warn", db === "done" ? `Position database for this house: built (${s.db?.n ?? "?"} views).`
      : run?.step === "posdb" ? `Building the position database: ${run.text}` : "Build the position database for this house (about 30 s, before the battery goes in).", prep);
    if (!real) return out;
    const c = s.calib?.fx ? s.calib : null;
    add("calib", "before", !!s.calibOk, "block", s.calibOk ? `Camera calibrated (${c.rms ?? "?"} px${c.verified?.from ? `, ${c.verified.from}` : ""}).`
      : c && !c.verified ? "The camera calibration wasn't checked on the pad: calibrate again."
      : c?.region && s.region ? "The video's framing changed since the camera was calibrated: calibrate again."
      : "Calibrate the camera before the battery goes in: record a slow turn of the drone on its pad, then Settings → Recordings → Calibrate the camera from this flight.",
      { label: "Calibrate the camera", action: "calibrate" });
    const per = this.perception, src = per?.source, video = !!src?.ready?.() && src.kind !== "sim", pic = video ? pictureProblem(per) : "", st = per?.pictureState?.();
    const frame = video && !pic ? (s.framingOf ? s.framingOf(per.region?.() ?? src.region(), st === "found" || st === "whole") : s.framing ?? "") : ""; // what vision needs
    add("picture", "battery", video && !pic && !frame, "block", !video ? "No video from the drone yet: battery in, goggles on." : pic ? `The video isn't the camera's picture yet: ${pic}.`
      : frame ? `Vision is paused: ${frame}.` : `Camera picture found in the video${st === "found" ? ` (${per.region().sw}x${per.region().sh}, black bars taken off)` : ""}.`);
    const trust = s.trust?.(), pad = s.pad, same = !!pad && pad.key === s.lensKey?.();
    add("padcheck", "battery", trust === "verified", "block", trust === "verified" ? pad?.text ?? "The pad check passed."
      : cpu ? `The pad check can't pass here: ${cpu}.`
      : pad && !same ? "The pad check was for another camera setup (the lens or the picture in the video changed since): put the drone on its pad and press the pad button again."
      : pad && !pad.ok ? pad.text : "Put the drone on its home pad and press the pad button: the camera checks itself against the 3D scan (about 3 s).", { label: "Pad check", action: "pad" });
    const sb = this.standby(now), how = sb?.approx ? "about " : "";
    if (sb) add("standby", "battery", sb.left > O4.warnLeft, "warn", sb.left > 0 ? `Battery in for ${how}${clock(sb.seconds)}${sb.by === "video" ? " (timed from the video)" : ""}: about ${clock(sb.left)} before the O4 overheats on the ground.`
      : `The O4 has been on the ground with its battery in for ${how}${clock(sb.seconds)}: it may shut down to cool. Unplug the battery for a few minutes.`);
    return out;
  }

  // run("prepare") (the checklist's fix button).
  run(action, opts) {
    return action === "prepare" ? this.prepare(opts) : Promise.reject(new Error(`"${action}" is done in the app's screens`));
  }

  // Everything before the battery: each part not done yet, in PREPARE's order. onProgress({ step, text, progress (0-1
  // over all of it) }); it goes on past a failure and says which failed. -> { ok, failed: { [step]: why }, seconds }.
  prepare({ onProgress, signal } = {}) {
    return (this.preparing ??= this.#prepare({ onProgress, signal }).finally(() => ((this.preparing = null), (this.running = null), this.emit("change"))));
  }
  async #prepare({ onProgress, signal }) {
    const t0 = performance.now(), steps = PREPARE.filter((p) => this.applies(p.id)), total = steps.reduce((a, p) => a + p.weight, 0);
    let before = 0;
    this.failed = {};
    for (const p of steps) {
      if (signal?.aborted) throw new DOMException("Aborted", "AbortError");
      if (this.state(p.id) === "done" && p.id !== "dino") (before += p.weight);
      else {
        const say = (frac, extra = "") => {
          this.running = { step: p.id, text: `${p.text}${extra}`, progress: (before + p.weight * Math.min(1, frac)) / total };
          onProgress?.(this.running);
          this.emit("change");
        };
        say(0);
        try {
          await this.step(p.id, say, signal);
        } catch (e) {
          if (e?.name === "AbortError") throw e;
          this.failed[p.id] = String(e?.message ?? e).split("\n")[0];
        }
        before += p.weight;
      }
    }
    await this.refresh().catch(() => {});
    const failed = this.failed;
    onProgress?.({ step: null, text: Object.keys(failed).length ? `Not ready: ${Object.entries(failed).map(([k, v]) => `${PREPARE.find((p) => p.id === k).text}: ${v}`).join("; ")}` : "Ready for the battery.", progress: 1 });
    return { ok: !Object.keys(failed).length, failed, seconds: +((performance.now() - t0) / 1000).toFixed(1) };
  }
  applies(id) {
    return id === "depth" ? !!this.depth && this.settings?.get?.("avoid") !== false : id === "detector" ? !!this.perception?.detector : id === "map3d" ? !!this.prepare3D : !!this.splat;
  }
  async step(id, say, signal) {
    const s = this.splat, pct = (l, n) => say(n ? l / n : 0, n ? ` ${Math.floor((100 * l) / n)}%` : "");
    if (id === "vision") return s.ready();
    if (id === "dino") {
      const off = s.on("status", (m) => m.key === "vision-model" && say(m.progress, ` ${Math.round(100 * m.progress)}%`));
      try { return await s.dino(); } finally { off?.(); }
    }
    if (id === "depth") return this.depth.warm(pct);
    if (id === "detector") {
      const d = this.perception.detector, off = d.on?.("status", (m) => m.progress != null && say(m.progress, ` ${Math.round(100 * m.progress)}%`));
      try { return await d.load(); } finally { off?.(); }
    }
    if (id === "map3d") return this.prepare3D({ onProgress: (p) => p?.total && pct(p.done, p.total), signal });
    if (id === "posdb") {
      for (const t0 = Date.now(); !s.twin && Date.now() - t0 < 60000; ) await new Promise((r) => setTimeout(r, 250)); // the real drone's twin loads meanwhile
      if (!s.twin) throw new Error("the 3D scan isn't loaded");
      // a build already running (the app starts one on its own) reports to its first caller only: follow its "db" events
      const off = s.on?.("db", (p) => p.total && say(p.done / p.total, ` (${p.done} of ${p.total} views${p.paused ? ", paused while flying" : ""})`));
      try { return await s.ensureDb({ signal }); } finally { off?.(); }
    }
  }

  dispose() {
    this.unsub?.();
    this.offTel?.();
  }
}
