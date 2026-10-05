// "Check localization on this flight", for Settings → Recordings (docs/HOME-DRONE.md, Wave C; the PANELS contract; critic
// ui 12): first what the recording's files say at once (video, flight data, this house, the pad button at the start, the
// camera's picture found in the goggles' video, landed before it stopped: tools/loc-replay.mjs's checks), then the
// flight replayed against the 3D scan (nav/replay.js checkRecording: the vision localizer on the recorded video, in a
// worker of its own) as a plain pass/fail report: pictures decoded, position found, fixes a second, never lost, back on
// the pad (or against the simulator's truth), disagreements, the video delay. Then, after a real flight's replay with
// nothing wrong in its files, "Calibrate the camera from this flight". Never in the air: a check is refused, and one
// running stops, when the drone takes off (flying()).
//   new RecheckPanel(el, { recordings, house, settings, replay, calibrate, flying })
// recordings: an array or async () => /rec/list's rows; replay(id, { onProgress, signal }) -> nav/replay.js's report
// ({ ok, title, lines: [{ id, ok: true | false | null, text }], problems }), or loc-check's replay numbers (replayChecks()
// words them); default: checkRecording({ id, houseId }). calibrate(id): what the button does (else the "calibrate" { id }
// event); fetchRec(id) -> { meta, telemetry, commands, index } (default: the /rec/<id>/ files).
import { Emitter } from "../util.js";
import { put, watchFlying, onVisible, plainWords } from "./changecard.js";
import { h, fmtDate, fmtDuration } from "./dom.js";

const cm = (m) => `${Math.round(m * 100)} cm`;
const PAD_BUTTON = '"The drone is on its home pad"';
// "the pad button" (other modules' words) as the button is labelled.
export const padWords = (t) => String(t).replace(/press the pad button/g, `press ${PAD_BUTTON}`).replace(/after the pad button/g, `after pressing ${PAD_BUTTON}`)
  .replace(/no pad button press/g, `no press of ${PAD_BUTTON}`).replace(/\(the pad button was pressed\)/g, `(${PAD_BUTTON} was pressed)`).replace(/the pad button/g, PAD_BUTTON);
const flyingSample = (s) => !!s.fm && !s.fm.includes("*") && (s.sticks?.thr ?? 0) > 0.22;

// What the files say, before any replay: [{ id, ok, level: "block" | "warn" | "info", text }].
export function recordingChecks({ meta = {}, telemetry = [], commands = [], index = [] }, { houseId = null } = {}) {
  const span = (a) => (a.length > 1 ? (a.at(-1).t - a[0].t) / 1000 : 0), sim = meta.app?.mode === "sim" || telemetry.some((s) => s.truth);
  const pad = commands.find((c) => c.tool === "pad"), region = meta.lens?.region ?? commands.findLast((c) => c.tool === "picture" && c.args?.sw)?.args;
  const out = [];
  const add = (id, ok, level, text) => out.push({ id, ok, level: ok ? "info" : level, text });
  add("video", index.length > 0, "block", index.length ? `Video: ${index.length} pictures over ${Math.round(span(index))} s.` : "No video: the goggles weren't sending video to the Mac while it recorded.");
  add("telemetry", telemetry.some((s) => s.est?.heading != null), "block", telemetry.length ? `Flight data: ${Math.round(span(telemetry))} s.` : "No flight data was recorded.");
  add("house", !houseId || meta.house === houseId, "block", meta.house === houseId || !houseId ? "Recorded in this house." : meta.house ? "Recorded in another house: load that house to check it." : "Recorded without a house loaded: there's no 3D scan to check it against.");
  add("pad", !!pad || sim, "block", pad ? `Started from the home pad (${PAD_BUTTON} was pressed).` : sim ? "A simulator flight: checked against its true position."
    : `${PAD_BUTTON} wasn't pressed before take-off. The check needs the flight to start and end on the home pad: that's how it knows the truth.`);
  add("picture", sim || !!region, "warn", region ? `The camera's picture was found in the goggles' video (${Math.round(region.sw)}×${Math.round(region.sh)}).` : sim ? "Simulator video: no goggles framing to find."
    : "The camera's picture wasn't found in the goggles' video while recording: the check will look for it itself.");
  const last = telemetry.at(-1);
  add("landed", !last || !flyingSample(last), "warn", !last || !flyingSample(last) ? "Landed before the recording stopped." : "The recording stopped in the air: land on the pad first, then stop recording.");
  return out;
}

// loc-check's replay numbers in words (see tools/loc-check.js replay()).
export function replayChecks(out, { videoDelay = null } = {}) {
  const res = [], add = (id, ok, level, text) => res.push({ id, ok, level: ok ? "info" : level, text });
  add("decoded", out.decoded >= 0.9 * out.units, "block", `${out.decoded} of ${out.units} video pictures decoded.`);
  add("found", out.foundAfter != null, "block", out.foundAfter != null ? `Position found ${out.foundAfter.toFixed(1)} s after the start (${out.start === "pad command" ? "from the home pad" : out.start}).` : "The position was never found from the video.");
  add("rate", out.fixRate >= 3, "block", `${out.fixRate.toFixed(1)} camera position fixes a second in flight (needs 3 or more).`);
  const lost = out.status?.lost ?? 0, all = Object.values(out.status ?? {}).reduce((a, b) => a + b, 0);
  add("lost", !lost, "block", lost ? `Lost its position for about ${Math.round((lost / Math.max(1, all)) * (out.flyingSeconds ?? 0))} s of the flight.` : "Never lost its position in flight.");
  if (out.truth && out.error?.n) add("truth", out.error.median < 0.1 && out.error.p95 < 0.25, "block", `Off from the simulator's true position by ${cm(out.error.median)} typically, ${cm(out.error.p95)} at worst (95% of the time; needs under 10 and 25 cm).`);
  if (!out.truth) {
    add("pad-end", out.endVsPad != null && out.endVsPad < 0.1, "block", out.endVsPad == null ? "It didn't land back on the pad, so the end can't be checked: land on the pad next time."
      : `Back on the pad: the position after landing was ${cm(out.endVsPad)} from it (needs under 10 cm).`);
    if (out.groundFixVsPad != null) add("pad-start", out.groundFixVsPad < 0.05, "block", `First camera fix on the pad: ${cm(out.groundFixVsPad)} off (needs under 5 cm).`);
  }
  if (out.conflicts) add("conflicts", out.anchored >= out.conflicts, "warn", `The camera and the drone's own sense of motion disagreed ${out.conflicts} time${out.conflicts === 1 ? "" : "s"}${out.anchored ? `; settled ${out.anchored}` : ""}.`);
  if (out.delay?.n) {
    const set = videoDelay ?? out.delay.videoDelay + out.delay.offset, off = Math.abs(out.delay.offset);
    add("delay", off <= 30, "warn", off <= 30 ? `The video delay setting fits (${set} ms).` : `The video seems to arrive ${out.delay.videoDelay} ms late, not ${set} ms: set Video delay to ${out.delay.videoDelay} in Settings → Real drone.`);
  }
  return res;
}

// Any replay report as check lines: nav/replay.js's (problems, or lines with ok null for information), a list of checks,
// or loc-check's numbers.
export function reportChecks(r, o = {}) {
  if (r.problems?.length && !r.lines?.length) return r.problems.map((p, i) => ({ id: `problem${i}`, ok: false, level: "block", text: padWords(`${p.charAt(0).toUpperCase()}${p.slice(1).replace(/\.?$/, ".")}`) }));
  if (r.lines) return r.lines.map((l) => ({ id: l.id, ok: l.ok !== false, level: l.ok === false ? "block" : "info", info: l.ok == null, text: padWords(l.text) }));
  return r.checks ?? replayChecks(r, o);
}

// One check's verdict: { ok, text }. The replay's own title only when it agrees (every line passed): else the panel's words,
// so a missing pad press or a warning isn't covered by "found the position all through". Not checked (stopped, failed):
// says so, never a pass.
export function runVerdict(run) {
  if (run.error) return { ok: false, text: `Not checked: ${run.error}` };
  const own = verdict(run.checks);
  return { ...own, text: own.ok && run.title && run.replay?.ok !== false && run.checks.every((c) => c.ok) ? run.title : own.text };
}
// Calibrating from it makes sense: a real flight replayed to the end, its files fine.
export const canCalibrateFrom = (run) => !!run && run.state === "done" && !run.error && !!run.replay && !run.sim && !run.checks.some((c) => !c.ok && c.level === "block" && c.file);

// -> { ok, text }: ok when nothing at "block" level failed.
export function verdict(checks) {
  const bad = checks.filter((c) => !c.ok && c.level === "block"), warn = checks.filter((c) => !c.ok && c.level === "warn");
  if (bad.length) return { ok: false, text: `Not good enough yet: ${bad[0].text.replace(/\.$/, "")}${bad.length > 1 ? `; ${bad.length - 1} more below` : ""}.` };
  return { ok: true, text: warn.length ? `Localization works on this flight, with ${warn.length} thing${warn.length === 1 ? "" : "s"} to fix.` : "Localization works on this flight." };
}

async function fetchFiles(id) {
  const { serverSession } = await import("../goggles/source.js"), s = await serverSession();
  if (!s) throw new Error("Whoop Pilot's server isn't running");
  const get = (f) => fetch(`/rec/${encodeURIComponent(id)}/${f}`, { headers: { "X-Whoop-Token": s.token } }).then((r) => (r.ok ? r.text() : ""));
  const lines = (t) => t.split("\n").filter(Boolean).map((l) => JSON.parse(l));
  const [meta, telemetry, commands, index] = await Promise.all([get("meta.json").then((t) => (t ? JSON.parse(t) : {})), get("telemetry.jsonl").then(lines), get("commands.jsonl").then(lines), get("video.jsonl").then(lines)]);
  return { meta, telemetry, commands, index };
}
// The server's recordings (/rec/list rows), newest first; [] without the server.
export const listRecordings = () => fetch("/rec/list").then((r) => (r.ok ? r.json() : { recordings: [] })).then((j) => j.recordings ?? []).catch(() => []);

export class RecheckPanel extends Emitter {
  constructor(el, { recordings = listRecordings, house = null, settings = null, replay = null, calibrate = null, fetchRec = fetchFiles, flying = () => false } = {}) {
    super();
    replay ??= (id, o) => import("../nav/replay.js").then((m) => m.checkRecording({ id, houseId: this.house?.id, ...o }));
    Object.assign(this, { el, recordings, house, settings, replay, calibrate, fetchRec, flying, recs: null, open: null, runs: new Map(), ac: null, seq: 0 });
    el.classList.add("rck");
    this.offs = [watchFlying(flying, (air) => {
      if (air && this.ac) (this.stopWhy = "flying"), this.ac.abort();
      this.render();
    }), onVisible(el, () => this.refresh())];
    this.refresh();
  }

  setHouse(house) {
    this.house = house;
    this.render();
  }

  async refresh() {
    const seq = ++this.seq, recs = typeof this.recordings === "function" ? await this.recordings().catch(() => this.recs ?? []) : this.recordings;
    if (seq !== this.seq) return;
    this.recs = (recs ?? []).filter((r) => !r.recording);
    this.render();
  }

  async check(id) {
    if (this.disposed) return;
    this.ac?.abort();
    const ac = (this.ac = new AbortController()), run = { id, files: null, checks: [], replay: null, state: "files", progress: null, error: null, sim: false };
    this.open = id;
    this.runs.set(id, run);
    this.stopWhy = null;
    if (this.flying()) {
      Object.assign(run, { state: "done", error: "the drone is flying. The check uses the computer's graphics, which finding the drone needs in the air: land first." });
      return void ((this.ac = null), this.render());
    }
    this.render();
    try {
      const files = await this.fetchRec(id);
      run.sim = files.meta?.app?.mode === "sim" || (files.telemetry ?? []).some((s) => s.truth);
      run.checks = recordingChecks(files, { houseId: this.house?.id ?? null }).map((c) => ({ ...c, file: true }));
      const blocked = run.checks.some((c) => !c.ok && c.level === "block" && c.id !== "pad");
      if (blocked || !this.replay) run.state = "done";
      else {
        run.state = "replay";
        this.render();
        const out = await this.replay(id, { signal: ac.signal, onProgress: (p) => ((run.progress = p), this.open === id && this.renderProgress(run)) });
        if (ac.signal.aborted) throw new DOMException("Aborted", "AbortError");
        run.replay = out;
        run.title = out.title ? padWords(out.title) : null;
        run.checks = [...run.checks.filter((c) => !c.ok), ...reportChecks(out, { videoDelay: this.settings?.get?.("videoDelay") ?? null })]; // the replay's lines say what passed
        run.state = "done";
      }
    } catch (e) {
      const why = this.stopWhy === "flying" ? "the drone took off, so the check stopped (it runs on the ground only)."
        : e?.name === "AbortError" ? "you stopped it."
        : `${plainWords(e, "the replay stopped before the end, so this says nothing about localization yet")}. Reload the page and try again.`;
      Object.assign(run, { state: "done", error: why.replace(/^./, (c) => c.toLowerCase()) });
    }
    if (this.ac === ac) this.ac = null;
    this.render();
  }

  renderProgress(run) {
    const slot = this.el.querySelector(".rck-progress");
    if (!slot) return;
    const p = run.progress ?? {}, frac = p.done != null && p.total ? p.done / p.total : p.seconds != null && p.of ? p.seconds / p.of : null;
    put(slot, h("progress", { max: 1, value: frac ?? undefined, "aria-label": "Replay progress" }), h("small", {}, p.text ?? "Replaying the flight against the 3D scan…"));
  }

  render() {
    if (this.disposed) return;
    if (!this.recs) return put(this.el, h("p", { class: "note" }, "Looking for recordings…"));
    if (!this.recs.length)
      return put(this.el, h("p", { class: "note" }, "No recordings yet. Record a flight (Record, at the top) that starts and ends on the home pad, then check it here."));
    put(this.el,
      h("p", { class: "hint" }, "Replays a recorded flight's video against the 3D scan, as the drone would in the air, and says whether it would have known where it was. Start and land on the home pad: that's how the check knows the truth."),
      this.flying() && h("p", { class: "note warn-text" }, "Land first: the check runs on the ground only (finding the drone needs the computer's graphics in the air)."),
      h("ul", { class: "rck-list" }, this.recs.map((r) => this.row(r))));
  }

  row(r) {
    const run = this.runs.get(r.id), open = this.open === r.id, busy = run && run.state !== "done", air = this.flying();
    const v = run?.state === "done" ? runVerdict(run) : null;
    const lines = (cs) => h("ul", { class: "rck-checks" }, cs.map((c) => h("li", { "data-ok": String(c.ok), "data-level": c.level },
      h("span", { class: "rck-mark", "data-info": String(!!c.info), "aria-label": c.info ? "note" : c.ok ? "passed" : c.level === "block" ? "failed" : "warning" }, c.info ? "i" : c.ok ? "✓" : c.level === "block" ? "✗" : "!"), c.text)));
    return h("li", { class: "rck-rec", "data-open": String(open) },
      h("div", { class: "rck-h" },
        h("span", { class: "rck-name" }, h("strong", {}, r.label || "flight"), h("small", {}, `${r.started ? fmtDate(r.started) : r.id}${r.durationMs ? ` · ${fmtDuration(r.durationMs)}` : ""}${r.house && this.house && r.house !== this.house.id ? " · another house" : ""}`)),
        busy ? h("button", { type: "button", class: "btn ghost small", onclick: () => this.ac?.abort() }, "Stop")
          : h("button", { type: "button", class: "btn small", disabled: air, title: air ? "Land first" : "", onclick: () => this.check(r.id) }, run ? "Check again" : "Check localization")),
      open && run && h("div", { class: "rck-body" },
        run.state === "files" && h("p", { class: "hint" }, "Reading the recording…"),
        run.state === "replay" && h("div", { class: "rck-progress", role: "status", "aria-live": "polite" }, h("progress", { max: 1, "aria-label": "Replay progress" }), h("small", {}, "Replaying the flight against the 3D scan (about as long as the flight)…")),
        v && h("p", { class: "rck-verdict", "data-ok": String(v.ok), role: "status" }, v.text.replace(/^./, (c) => c.toUpperCase())),
        run.checks.length > 0 && (run.error ? h("details", { class: "rck-files" }, h("summary", {}, "What the files say"), lines(run.checks)) : lines(run.checks)),
        canCalibrateFrom(run) && h("div", { class: "row wrap" },
          h("button", { type: "button", class: "btn ghost small", onclick: () => (this.calibrate ? this.calibrate(r.id) : this.emit("calibrate", { id: r.id })) }, "Calibrate the camera from this flight"))));
  }

  dispose() {
    this.seq++;
    this.disposed = true;
    this.offs.forEach((f) => f());
    this.ac?.abort();
    put(this.el);
    this.el.classList.remove("rck");
  }
}
