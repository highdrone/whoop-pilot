// Checking a recorded flight's localization (tools/recorder.mjs keeps /rec/<id>/: meta.json, video.h264 + video.jsonl,
// telemetry.jsonl, commands.jsonl): its own goggles video goes through the localizer and the splat localizer again, on
// the recording's clock, against the twin of the house. A real flight has no truth, so the home pad is its truth: the
// flight starts there (the pad button: a "pad" command) and ends there (landed on it, or put back by hand), and the pose
// after landing must be back on it. The app runs it in a worker (replay-worker.js: its own clock, twin and vision worker,
// nothing shared with a flight); tools/loc-check.html runs replayRecording() in its page; tools/loc-replay.mjs lists
// recordingProblems() under Node.
//   const report = await checkRecording({ id, houseId, onProgress, signal });   // plain words: report.title, report.lines
//   const { calib, text } = await calibrateFromRecording({ id, houseId, onProgress, signal });   // a pad turn recorded earlier
import { Emitter, wrapAngle } from "../util.js";
import { Localizer } from "./localizer.js";
import { SplatLocalizer } from "./splatloc.js";
import { OsdMask } from "./osdmask.js";
import { droneLens, DEG, CAM_DZ } from "./lens.js";
import { H264Player } from "../goggles/player.js";
import { serverSession } from "../goggles/source.js";
import { DEFAULTS } from "../settings.js";
import { MASK } from "../protocol.js";

// What a good replay shows (loc-check's targets): fixes per second while flying, the pose after landing and the first
// fix on the ground against the pad (m), the share of video decoded.
export const REPLAY = { fixRate: 3, closure: 0.1, groundFix: 0.05, decoded: 0.9, rate: 5, lag: 150 };

const r3 = (v) => +(+v).toFixed(3);
const quant = (v, f) => { const a = v.filter(Number.isFinite).sort((x, y) => x - y); return a.length ? r3(a[Math.min(a.length - 1, Math.floor(f * a.length))]) : null; };
const stats = (v) => ({ median: quant(v, 0.5), p95: quant(v, 0.95), max: v.length ? r3(Math.max(...v)) : null, n: v.length });
const posErr = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
const yawErr = (a, b) => Math.abs(wrapAngle(a.yaw - b.yaw)) / DEG;
const jsonl = (text) => text.split("\n").filter(Boolean).map((l) => JSON.parse(l));

// Telemetry samples (main.js telemetrySample) as the localizer's controller: heading, flow velocity, climb rate and the
// attitude it commanded; flying as FlightController.isFlying (armed, throttle up).
export class ReplayCtl extends Emitter {
  constructor(videoDelay) {
    super();
    Object.assign(this, { videoDelay, est: { heading: 0, vx: 0, vy: 0, vz: 0, flowQ: 0, rotating: false, pitchAngle: 0, rollAngle: 0 }, tel: null, hist: [], perception: { latest: { flow: null } }, last: null });
  }
  isFlying() {
    const fm = this.tel?.fm || "";
    return fm !== "" && !fm.includes("*") && (this.tel?.sticks?.thr ?? 0) > 0.22;
  }
  historyAt(t) {
    const h = this.hist;
    let i = h.length - 1;
    while (i > 0 && h[i].t > t) i--;
    return h[i] ?? { p: 0, q: 0 };
  }
  apply(s, t) {
    const lim = DEFAULTS.angleLimit * DEG, axis = (k) => (s.mask & MASK[k] && s.out ? s.out[k] : s.sticks?.[k] ?? 0) * lim, dt = this.last == null ? 0.1 : (t - this.last) / 1000;
    const e = this.est, dh = (s.est?.heading ?? e.heading) - e.heading;
    Object.assign(e, { heading: s.est?.heading ?? e.heading, vx: s.est?.vx ?? 0, vy: s.est?.vy ?? 0, vz: s.est?.vz ?? 0, flowQ: s.est?.flowQ ?? 0, rotating: Math.abs(dh) / Math.max(dt, 0.02) > 0.5, pitchAngle: axis("pitch"), rollAngle: axis("roll") });
    this.tel = { fm: s.fm, sticks: s.sticks };
    this.perception.latest.flow = s.est?.flowQ > 0 ? { t } : null;
    this.hist.push({ t, p: e.pitchAngle, q: e.rollAngle });
    while (this.hist.length > 60) this.hist.shift();
    if (this.last != null && dt > 0) this.emit("tick", { dt: Math.min(dt, 0.3), now: t });
    this.last = t;
  }
}

// A recording from the server (needs its session token). -> { id, meta, index, video (bytes), telemetry, commands }.
export async function loadRecording(id) {
  const s = await serverSession(), get = (f) => fetch(`/rec/${encodeURIComponent(id)}/${f}`, { headers: { "X-Whoop-Token": s?.token ?? "" } }).then((r) => { if (!r.ok) throw new Error(`the recording's ${f} can't be read (HTTP ${r.status})`); return r; });
  const [meta, index, video, telemetry, commands] = await Promise.all([get("meta.json").then((r) => r.json()), get("video.jsonl").then((r) => r.text()).then(jsonl),
    get("video.h264").then((r) => r.arrayBuffer()).then((b) => new Uint8Array(b)), get("telemetry.jsonl").then((r) => r.text()).then(jsonl), get("commands.jsonl").then((r) => r.text()).then(jsonl).catch(() => [])]);
  return { id, meta, index, video, telemetry, commands };
}

const flyingSample = (s) => s.fm && !s.fm.includes("*") && s.sticks?.thr > 0.22;

// What stops a recording being checked, before replaying it (plain words; [] when nothing). videoBytes: the size of
// video.h264 (the index must fit in it).
export function recordingProblems({ meta, index, telemetry: tel, commands: cmds }, videoBytes = Infinity) {
  const pad = cmds.find((c) => c.tool === "pad"), start = tel.find((s) => s.pose?.status === "ok" && !flyingSample(s));
  return [
    !index.length && "it has no video (the goggles weren't streaming to the Mac)",
    index.length && index.at(-1).off + index.at(-1).bytes > videoBytes && "its video file is shorter than its index (the recording was cut off)",
    !tel.length && "it has no telemetry",
    tel.length && !tel.some((s) => s.est?.heading != null) && "its telemetry has no heading or motion estimates",
    !meta.house && "it wasn't flying a house (no house in it)",
    !pad && !start && "it has no pad reset and no position: press the pad button before take-off next time",
    !tel.some((s) => s.truth) && !pad && "a real flight is checked against the home pad: press the pad button before take-off and land on the pad at the end",
    meta.app?.mode !== "sim" && !meta.lens?.region && !cmds.some((c) => c.tool === "picture") && "the camera picture's place in the video wasn't recorded (record again with this version of the app)",
  ].filter(Boolean);
}

// The recording's video through the H.264 player, one frame at a time: each(frame (VideoFrame, closed after), t (the
// recording's ms)). Waits for the decoder rather than racing it.
async function decodeAll(rec, each, { signal } = {}) {
  const frames = [], tsT = new Map(), player = new H264Player({ onFrame: (vf) => (frames.push(vf), waiter?.()) });
  let waiter = null;
  const take = async (vf) => {
    const t = tsT.get(vf.timestamp);
    tsT.delete(vf.timestamp);
    try { if (t != null) await each(vf, t); } finally { vf.close(); }
  };
  try {
    for (const u of rec.index) {
      if (signal?.aborted) throw new DOMException("Aborted", "AbortError");
      player.push(rec.video.subarray(u.off, u.off + u.bytes));
      tsT.set(player.ts, u.t);
      while (player.decoder?.decodeQueueSize > 4 && !frames.length) await new Promise((r) => player.decoder.addEventListener("dequeue", r, { once: true }));
      if (!frames.length && player.decoder?.decodeQueueSize) await Promise.race([new Promise((r) => (waiter = r)), new Promise((r) => setTimeout(r, 500))]);
      while (frames.length) await take(frames.shift());
    }
    await player.decoder?.flush().catch(() => {});
    while (frames.length) await take(frames.shift());
  } finally {
    for (const f of frames) f.close();
    player.close();
  }
  return player.stats;
}

// The camera picture of a recording: the last "picture" command that is the O4's 4:3 picture, else meta.lens.region if it
// is one or the framing its calibration was made for (a recording started before the picture was found stores the whole
// 16:9 stream there), else null (the replay takes the whole frame when that is 4:3, else says it can't tell where the
// picture is).
const is43 = (r) => r?.sw > 0 && Math.abs(r.sw / r.sh / (4 / 3) - 1) <= 0.005;
const same = (a, b) => !!a && !!b && ["sx", "sy", "sw", "sh"].every((k) => Math.abs(a[k] - b[k]) <= 2);
export const regionOf = (rec) => {
  const m = rec.meta.lens?.region;
  return rec.commands.findLast((c) => c.tool === "picture" && is43(c.args))?.args ?? (is43(m) || same(m, rec.meta.lens?.calib?.region) ? m : null);
};

// rec: { meta, index [{t, bytes, off}], video (bytes), telemetry, commands }. delay: ms from the camera to the server's
// receive time. The pose starts from the pad (a "pad" command, else the first grounded sample whose recorded pose was
// ok); without one the localizer starts lost and relocalizes (db: a RelocDb). Frames are cut to the recorded picture region.
// Runs on the recording's clock: performance.now() is the recording's time while it runs (a page or a worker of its
// own), and each fix lands `lag` ms after its frame, as a live one would. -> numbers: errors against the recorded truth
// (simulator) or pose, and against the pad.
export async function replayRecording(rec, { twin, house, map, features = null, db = null, calib = rec.meta.lens?.calib ?? null, delay = rec.meta.app?.videoDelay ?? 100, rate = REPLAY.rate, lag = REPLAY.lag, relocStart = false, region = null, onProgress, signal } = {}) {
  const tel = rec.telemetry.filter((s) => s.t), T0 = tel[0].t, vt = (t) => t - T0 + 10000, realNow = performance.now.bind(performance);
  let clock = vt(T0), splat = null;
  performance.now = () => clock;
  try {
    const lens = droneLens(calib, rec.meta.lens?.uptilt ?? 20);
    region ??= regionOf(rec);
    const ctl = new ReplayCtl(delay), loc = new Localizer({ ctl, map, house });
    splat = new SplatLocalizer({ twin, localizer: loc, lens, house, map, ctl, features });
    loc.setSource("fused");
    splat.db = db;
    const fix = loc.fix.bind(loc), held = [];
    let grabbed = null;
    loc.fix = (f) => (clock >= grabbed + lag ? fix(f) : (held.push({ f, due: grabbed + lag }), true));
    const pad = !relocStart && rec.commands.find((c) => c.tool === "pad" && c.args?.x != null);
    const start = relocStart ? null : pad ? { t: pad.t, ...pad.args } : tel.find((s) => s.pose?.status === "ok" && !flyingSample(s))?.pose;
    const startT = pad ? pad.t : tel.find((s) => s.pose === start)?.t;
    let ti = 0, lastTry = -Infinity, started = false, foundAt = null, usedRegion = null, groundFix = null, flew = false;
    splat.on("fix", (f) => !flew && !ctl.isFlying() && start && groundFix == null && (groundFix = r3(Math.hypot(f.pose.x - start.x, f.pose.y - start.y))));
    const samples = [], tries = [], w0 = realNow();
    const truthAt = (t) => { let i = tel.findIndex((s) => s.t >= t); if (i < 0) i = tel.length - 1; return tel[i].truth ?? null; };
    const advance = (t) => {
      for (; ti < tel.length && tel[ti].t <= t; ti++) {
        clock = vt(tel[ti].t);
        if (!started && start && tel[ti].t >= startT) (started = true), loc.reset({ x: start.x, y: start.y, yaw: start.yaw });
        ctl.apply(tel[ti], clock);
        flew ||= ctl.isFlying();
        while (held.length && clock >= held[0].due) fix(held.shift().f);
        const p = loc.pose(), tr = tel[ti].truth, rp = tel[ti].pose;
        if (loc.known) foundAt ??= clock;
        if (ti % 3 === 0) samples.push({ t: clock, status: p.status, known: loc.known, sigma: p.sigma, flying: ctl.isFlying(),
          err: tr ? posErr(p, tr) : null, yawErr: tr ? yawErr(p, tr) : null, err3: tr ? Math.hypot(p.x - tr.x, p.y - tr.y, p.z - tr.z) : null, vsRecorded: rp && rp.status !== "lost" ? posErr(p, rp) : null });
      }
    };
    const stat = await decodeAll(rec, async (vf, t) => {
      if (t - lastTry < 1000 / rate) return;
      advance(t);
      clock = Math.max(clock, vt(t));
      lastTry = t;
      const pr = region ?? (Math.abs(vf.displayWidth / vf.displayHeight / (4 / 3) - 1) <= 0.005 ? { sx: 0, sy: 0, sw: vf.displayWidth, sh: vf.displayHeight } : null);
      if (!pr) throw new Error(`the recording's frames are ${vf.displayWidth}x${vf.displayHeight}, not the camera's 4:3 picture, and it doesn't say where the picture is in them`);
      usedRegion = pr;
      const bmp = await createImageBitmap(vf, pr.sx, pr.sy, pr.sw, pr.sh, { resizeWidth: 640, resizeHeight: Math.round((640 * pr.sh) / pr.sw) });
      const lost = loc.pose().status === "lost", tc = vt(t) - delay, wr = realNow();
      grabbed = clock;
      if ((lost && !splat.db) || (lost && t - (tries.findLast((x) => x.reloc)?.t ?? -Infinity) < 2000)) return bmp.close();
      const r = await (lost ? splat.relocalize(bmp, tc) : splat.track(bmp, tc)), tr = truthAt(t - delay);
      tries.push({ t, reloc: lost, ok: r.ok, reason: r.reason, refused: r.refused ?? null, inliers: r.inliers, ms: realNow() - wr, err: r.pose && tr ? posErr(r.pose, tr) : null });
      if (tries.length % 10 === 0) onProgress?.({ seconds: (t - T0) / 1000, of: (tel.at(-1).t - T0) / 1000, tries: tries.length });
    }, { signal });
    advance(Infinity);
    const flying = samples.filter((s) => s.flying && foundAt != null && s.t >= foundAt), ok = tries.filter((x) => x.ok && !x.reloc), known = flying.filter((s) => s.known && s.status !== "lost");
    const seconds = (tel.at(-1).t - T0) / 1000, fly = flying.length * 0.3, endPose = loc.pose(), solved = tries.filter((x) => x.ok || x.refused), endTruth = tel.at(-1).truth;
    const lastFlying = samples.findLastIndex((s) => s.flying), landedAfter = lastFlying >= 0 && lastFlying < samples.length - 1;
    return {
      region: usedRegion, pad: start ? { x: start.x, y: start.y } : null, groundFixVsPad: groundFix,
      endVsPad: start && landedAfter ? r3(Math.hypot(endPose.x - start.x, endPose.y - start.y)) : null, endStatus: endPose.status,
      endErr: endTruth && landedAfter ? r3(posErr(endPose, endTruth)) : null, truthVsPad: endTruth && start ? r3(Math.hypot(endTruth.x - start.x, endTruth.y - start.y)) : null,
      gated: solved.length ? r3(solved.filter((x) => x.refused === "gate").length / solved.length) : null, conflicts: loc.fixes.conflicts, anchored: loc.fixes.anchored, delay: splat.delayCheck(), videoDelay: delay,
      id: rec.id ?? null, label: rec.meta.label, seconds: +seconds.toFixed(1), flyingSeconds: +fly.toFixed(1), wallSeconds: +((realNow() - w0) / 1000).toFixed(1), units: rec.index.length, decoded: stat.decoded, decoder: stat.start,
      start: pad ? "pad command" : start ? "first pose on the ground" : "relocalization", foundAfter: foundAt == null ? null : r3((foundAt - vt(T0)) / 1000), truth: !!tel[0].truth,
      error: stats(known.map((s) => s.err).filter((v) => v != null)), error3d: stats(known.map((s) => s.err3).filter((v) => v != null)), yawDeg: stats(known.map((s) => s.yawErr).filter((v) => v != null)),
      vsRecorded: stats(known.map((s) => s.vsRecorded).filter((v) => v != null)), sigma: stats(known.map((s) => s.sigma)),
      status: flying.reduce((a, s) => ((a[s.status] = (a[s.status] ?? 0) + 1), a), {}), fixRate: +(ok.length / Math.max(1, fly)).toFixed(2), tries: tries.length, accepted: ok.length,
      relocs: tries.filter((x) => x.reloc).map((x) => ({ t: r3((x.t - T0) / 1000), ok: x.ok, err: x.err != null ? r3(x.err) : null, ms: Math.round(x.ms) })),
      rejects: Object.entries(tries.filter((x) => !x.ok).reduce((a, x) => ((a[x.reason?.replace(/\d+/g, "#")] = (a[x.reason?.replace(/\d+/g, "#")] ?? 0) + 1), a), {})),
      inliers: stats(ok.map((x) => x.inliers)), msPerFix: stats(tries.map((x) => x.ms)),
    };
  } finally {
    performance.now = realNow;
    if (!features) splat?.dispose();
  }
}

// replayRecording()'s numbers -> the plain report: { ok, verdict "pass" | "fail", title, lines: [{ id, ok (true, false or
// null: for information), text }], numbers }. Without truth (a real flight) the pad is the truth.
export function judgeReplay(out) {
  const cm = (m) => `${Math.round(m * 100)} cm`, lines = [], line = (id, ok, text) => lines.push({ id, ok, text });
  const e = out.truth ? out.error : out.vsRecorded, lost = out.status?.lost ?? 0;
  line("video", out.decoded >= REPLAY.decoded * out.units, `Video: ${out.decoded} of ${out.units} frames decoded (${out.seconds} s, ${out.flyingSeconds} s of it flying).`);
  line("picture", out.region ? true : null, out.region ? `The camera picture in the video: ${out.region.sw}x${out.region.sh} at ${out.region.sx}, ${out.region.sy}.` : "The video is the camera picture itself.");
  line("start", out.start === "pad command" ? true : out.start === "relocalization" ? false : null, out.start === "pad command" ? "The flight started on the home pad (the pad button was pressed)."
    : out.start === "relocalization" ? "The flight didn't start from the pad, so the check had to find the drone from the video first." : "The flight started from a position the app already had (not the pad button).");
  line("found", out.foundAfter != null, out.foundAfter == null ? "The camera never matched the 3D scan: check the lighting and calibrate the camera." : `Position known from ${out.foundAfter} s.`);
  line("fixes", out.fixRate >= REPLAY.fixRate, `Position fixes from the camera: ${out.fixRate} a second while flying (at least ${REPLAY.fixRate} needed), ${out.inliers?.median ?? 0} matched points each, ${Math.round(out.msPerFix?.median ?? 0)} ms.`);
  line("lost", !lost, lost ? `The position was lost for about ${(lost * 0.3).toFixed(1)} s of the flight.` : "The position was never lost.");
  if (out.truth) line("error", e.median != null && e.median < 0.1 && e.p95 < 0.25, `Against the simulator's truth: ${cm(e.median ?? 0)} typical, ${cm(e.p95 ?? 0)} at worst (95%).`);
  if (!out.truth) {
    line("closure", out.endVsPad != null && out.endVsPad < REPLAY.closure, out.endVsPad == null ? "It didn't end on the ground after flying, so the pad can't be checked: land on the pad (or put the drone back on it) before stopping the recording."
      : `Back on the ground the position was ${cm(out.endVsPad)} from the pad (under ${cm(REPLAY.closure)} needed).`);
    if (out.groundFixVsPad != null) line("ground", out.groundFixVsPad < REPLAY.groundFix, `Before take-off the camera put the drone ${cm(out.groundFixVsPad)} from the pad (under ${cm(REPLAY.groundFix)} needed).`);
  }
  if (out.conflicts) line("conflicts", null, `The camera and dead reckoning disagreed ${out.conflicts} time${out.conflicts > 1 ? "s" : ""} (settled ${out.anchored ?? 0}).`);
  if (out.delay) line("delay", Math.abs(out.delay.offset) <= 30, Math.abs(out.delay.offset) <= 30 ? `The video delay setting (${out.videoDelay} ms) fits the turns.` : `The video delay looks like ${out.delay.videoDelay} ms, not ${out.videoDelay} ms: set Video delay to ${out.delay.videoDelay} ms in Settings.`);
  const ok = lines.every((l) => l.ok !== false), bad = lines.filter((l) => l.ok === false);
  return { ok, verdict: ok ? "pass" : "fail", title: ok ? "The camera found the drone's position all through this flight." : `The camera's position check failed on this flight (${bad.length} problem${bad.length > 1 ? "s" : ""}): ${bad[0].text}`, lines, numbers: out };
}

// Lens calibration from a recording of the pad turn (the drone on its home pad after the pad button, turned slowly by hand
// through a full circle, then held still; nothing flies): a frame every `step` of the recorded heading for
// SplatLocalizer.calibrateFrames(), and the frames held still afterwards (else the ones between) as the check: solved
// where the drone sits with the new lens, they must land on the pad (SPLAT.pad.pos). Done before the battery goes in
// next time, so the O4's short time on the ground goes to the pad check only. -> { ok, calib (calib.json, with the
// recording's picture region and the check as verified), samples, rounds, verify, text } or throws with plain words.
export async function calibrateFromRecording(rec, { twin, house, map, features = null, calib = null, delay = rec.meta.app?.videoDelay ?? 100, step = 30 * DEG, of = 12, onProgress, signal } = {}) {
  const tel = rec.telemetry.filter((s) => s.t), pad = rec.commands.find((c) => c.tool === "pad" && c.args?.x != null), region = regionOf(rec);
  if (!pad) throw new Error("this recording has no pad button press: record the pad turn after pressing the pad button");
  const up = tel.find((s) => s.t > pad.t && flyingSample(s))?.t ?? Infinity, ground = tel.filter((s) => s.t >= pad.t && s.t < up && s.est?.heading != null);
  if (ground.length < 10) throw new Error("this recording has no time on the pad after the pad button");
  const h0 = ground[0].est.heading, yawAt = (t) => { const s = ground.reduce((a, b) => (Math.abs(b.t - t) < Math.abs(a.t - t) ? b : a)); return wrapAngle(pad.args.yaw - (s.est.heading - h0)); };
  const turned = ground.reduce((a, s, i) => a + (i ? Math.abs(wrapAngle(s.est.heading - ground[i - 1].est.heading)) : 0), 0);
  if (turned < Math.PI) throw new Error(`the drone was turned only ${Math.round(turned / DEG)}° on the pad: turn it slowly all the way round`);
  const floor = map?.floorAt(pad.args.x, pad.args.y) ?? 0, body = (yaw) => ({ x: pad.args.x, y: pad.args.y, z: floor + 0.01, yaw, pitch: 0, roll: 0 });
  const fit = [], still = [], between = [], osd = new OsdMask(), oc = new OffscreenCanvas(osd.width, osd.height).getContext("2d", { willReadFrequently: true });
  let last = null, n = 0;
  const steady = (t) => { const w = ground.filter((s) => s.t > t - 1000 && s.t <= t); return w.length > 3 && Math.abs(wrapAngle(w.at(-1).est.heading - w[0].est.heading)) < 2 * DEG; };
  await decodeAll(rec, async (vf, t) => {
    if (t < pad.t || t >= up) return;
    if (n++ % 3 === 0) { // the overlay mask learns from the turning picture (osdmask.js), as the live calibration's does
      const pr = region ?? { sx: 0, sy: 0, sw: vf.displayWidth, sh: vf.displayHeight };
      oc.drawImage(vf, pr.sx, pr.sy, pr.sw, pr.sh, 0, 0, osd.width, osd.height);
      const px = oc.getImageData(0, 0, osd.width, osd.height).data, g = new Float32Array(osd.width * osd.height);
      for (let i = 0; i < g.length; i++) g[i] = 0.299 * px[4 * i] + 0.587 * px[4 * i + 1] + 0.114 * px[4 * i + 2];
      osd.add(g);
    }
    const tc = t - delay, yaw = yawAt(tc), keep = fit.length < of && (last == null || Math.abs(wrapAngle(yaw - last)) >= step);
    const late = steady(tc) && fit.length >= 4 && still.length < 6, mid = !keep && !late && between.length < 6 && last != null && Math.abs(wrapAngle(yaw - last)) >= step / 2;
    if (!keep && !late && !mid) return;
    const pr = region ?? { sx: 0, sy: 0, sw: vf.displayWidth, sh: vf.displayHeight }, image = await createImageBitmap(vf, pr.sx, pr.sy, pr.sw, pr.sh, { resizeWidth: 640, resizeHeight: Math.round((640 * pr.sh) / pr.sw) });
    if (keep) (fit.push({ image, body: body(yaw), grounded: true }), (last = yaw), onProgress?.({ phase: "turn", samples: fit.length, of }));
    else (late ? still : between).push({ image, yaw });
  }, { signal });
  let splat = null;
  const close = () => ([...fit, ...still, ...between].forEach((f) => f.image.close?.()), !features && splat?.dispose());
  try {
    if (fit.length < 4) throw new Error(`only ${fit.length} views from the turn: turn the drone slowly, all the way round`);
    splat = new SplatLocalizer({ twin, localizer: { pose: () => body(pad.args.yaw), poseAt: () => body(pad.args.yaw), source: "fused" }, lens: droneLens(calib, rec.meta.lens?.uptilt ?? 20), house, map, features });
    onProgress?.({ phase: "fit", samples: fit.length, of });
    const b = fit[0].body, { calib: c, rounds, samples } = await splat.calibrateFrames(fit, { pad: { x: b.x, y: b.y, z: b.z + 0.025 } });
    splat.setLens(droneLens(c));
    onProgress?.({ phase: "check", samples: fit.length, of });
    const checks = (still.length >= 4 ? still : [...still, ...between]).slice(0, 6), rows = [];
    for (const f of checks) rows.push(await splat.padSolve(await createImageBitmap(f.image), body(f.yaw)));
    const acc = rows.filter((r) => r.ok), med = acc.length ? acc.map((r) => r.err).sort((x, y) => x - y)[acc.length >> 1] : null, P = splat.o.pad;
    const verify = { ok: acc.length >= Math.min(P.accept, rows.length) && rows.length >= 3 && med <= P.pos, accepted: acc.length, n: rows.length, medianErr: med == null ? null : r3(med) };
    if (!verify.ok) throw new Error(`the calibration from this recording didn't check out on the pad (${acc.length} of ${rows.length} views matched${med == null ? "" : `, ${Math.round(med * 100)} cm from the pad`})`);
    const out = { ...c, region: region ? { ...region, kind: rec.meta.app?.mode === "sim" ? "sim" : "goggles" } : null, verified: { err: verify.medianErr, accepted: verify.accepted, at: new Date().toISOString(), from: `recording ${rec.id ?? ""}`.trim() },
      ...(osd.ready && osd.coverage() <= 0.3 && { osdMask: osd.toJSON() }) }; // with the overlay mask, flights don't learn it again
    return { ok: true, calib: out, samples, rounds, verify, text: `The camera is calibrated from the recorded pad turn (${c.rms} px, checked on the pad within ${Math.round(verify.medianErr * 100)} cm). Next: battery in, drone on its pad, press the pad button.` };
  } finally {
    close();
  }
}

// ---------------------------------------------------------------- the app's side: a worker of its own

// Runs a check or a calibration in replay-worker.js (its own clock, twin and vision worker; the house, map, 3D map,
// calib.json and relocalization database come from the browser's store). op: "check" | "calibrate"; the recording by
// id from the server, or rec (loadRecording()'s shape, e.g. one made in a check page).
function inWorker(op, { id, rec, houseId, onProgress, signal, ...options }) {
  return new Promise((resolve, reject) => {
    const w = new Worker(new URL("./replay-worker.js", import.meta.url), { type: "module" }), end = (fn, v) => (w.terminate(), signal?.removeEventListener("abort", abort), fn(v));
    const abort = () => end(reject, new DOMException("Aborted", "AbortError"));
    if (signal?.aborted) return abort();
    signal?.addEventListener("abort", abort, { once: true });
    w.onmessage = ({ data: m }) => (m.type === "progress" ? onProgress?.(m.progress) : m.type === "done" ? end(resolve, m.result) : end(reject, new Error(m.error)));
    w.onerror = (e) => end(reject, new Error(e.message || "the replay worker failed"));
    w.postMessage({ op, id, rec, houseId, options });
  });
}

// The recording `id` (from /rec/list) replayed against the house `houseId` (stored): judgeReplay()'s plain report, plus
// problems (recordingProblems: why it can't be checked; then lines is empty and ok false). onProgress({ text, ... }).
export const checkRecording = (args) => inWorker("check", args);
// A recorded pad turn -> calibrateFromRecording()'s { ok, calib, text }; the caller stores calib.json
// (store.writeFile(houseId, "calib.json", JSON.stringify(calib))) and hands it to the app (splat.setCalib, depth.setCalib).
export const calibrateRecording = (args) => inWorker("calibrate", args);
