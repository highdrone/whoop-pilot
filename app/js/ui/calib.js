// Calibrate the drone's camera, for Settings → House (docs/HOME-DRONE.md, Wave C; the PANELS contract): the lens in use
// (calib.json or the O4's standard one), then one of two ways: from a recorded flight that starts with a slow turn on the
// home pad (no battery time spent; recommended), or live on the pad ("The drone is on its home pad", a slow full turn by
// hand, hold still: against the O4's ~2.5 min on the ground; the real drone only). Progress with Stop, then the result:
// error in pixels, field of view, uptilt, and a before/after picture (the 3D scan from the home pad through the new lens,
// with where the old lens model put each point and where the new one does), Save (calib.json through house/store.js, or
// save()) or Discard. A live calibration is in use as soon as it is made (SplatLocalizer.calibrate applies it): Discard,
// Stop, a failure, another house or closing the panel put the saved lens back (restore()). Nothing runs in the air.
//   new CalibPanel(el, { twin, house, settings, lens: await import("../nav/lens.js"), recordings, calibrateLive, calibrateRecording, save, restore, current, flying, mode })
// recordings: an array or async () => [{ id, label, started, durationMs, house }]; calibrateLive({ onProgress, signal })
// (SplatLocalizer.calibrate) and calibrateRecording(id, { onProgress, signal }) -> { calib, verify? }; save(calib); restore():
// the saved lens back in use (session.applyCalib); current: calib.json now (or async () => it); flying(), mode() ("sim" |
// "real"; default settings' mode). refreshRecordings() after a recording stops; pickRecording(id) preselects one.
// Events: "saved" { calib }.
import { Emitter } from "../util.js";
import { put, watchFlying, onVisible, plainWords } from "./changecard.js";
import { h, fmtDate, fmtDuration } from "./dom.js";
import { intrinsics, unproject, project, fieldOfView, resolveLens } from "../twin/lens.js";
import { openStore } from "../house/store.js";
import * as navLens from "../nav/lens.js";
import { listRecordings, padWords } from "./recheck.js";

const PAD_BUTTON = '"The drone is on its home pad"';
const PHASE = { turn: "Turn the drone slowly", fit: "Fitting the lens to the 3D scan…", check: "Checking it on the pad…", decode: "Reading the recording…", match: "Matching the pictures to the 3D scan…" };

// Where the old lens model and the new one put the same directions: a grid over the new lens's picture (w x h), each
// point { u, v } (new) with the old one's { u0, v0 }; max and mean shift in pixels.
export function lensShift(before, after, w = 640, h = 480, step = 8) {
  const K0 = intrinsics(before, w, h), K1 = intrinsics(after, w, h), points = [];
  let max = 0, sum = 0;
  for (let j = 0; j <= step; j++)
    for (let i = 0; i <= step; i++) {
      const u = (i / step) * (w - 1) + 0.5, v = (j / step) * (h - 1) + 0.5, d = unproject(K1, u, v), p = d && project(K0, d);
      if (!p) continue;
      const s = Math.hypot(p[0] - u, p[1] - v);
      points.push({ u, v, u0: p[0], v0: p[1], shift: s });
      max = Math.max(max, s);
      sum += s;
    }
  return { points, max, mean: points.length ? sum / points.length : 0, width: w, height: h };
}

// calib.json -> words: { quality: "good" | "usable" | "poor", rms, fov "129° × 96°", uptilt, text }.
export function calibSummary(calib, lensOf) {
  if (!calib?.fx) return { quality: null, text: "Not calibrated: using the O4's standard lens (127° × 95°, tilted up 20°). Vision position fixes get wider error bars until it is." };
  const lens = lensOf(calib), f = fieldOfView(lens, calib.width, calib.height), rms = calib.rms;
  const quality = rms == null ? "usable" : rms <= 1 ? "good" : rms <= 2 ? "usable" : "poor";
  const words = { good: "a good fit", usable: "usable", poor: "a poor fit: try again in better light, turning more slowly" }[quality];
  const details = `${rms != null ? `${rms.toFixed(2)} px error (${words})` : words}, ${Math.round(f.h)}° × ${Math.round(f.v)}° field of view, camera tilted up ${(calib.uptiltDeg ?? 20).toFixed(1)}°${calib.verified ? ", checked on the pad" : ""}`;
  return { quality, rms, fov: `${Math.round(f.h)}° × ${Math.round(f.v)}°`, uptilt: calib.uptiltDeg, details,
    text: `Calibrated${calib.verified?.at ? ` ${fmtDate(calib.verified.at)}` : ""}${calib.source === "recording" ? " from a recording" : ""}: ${details}.` };
}

export class CalibPanel extends Emitter {
  constructor(el, { twin = null, house = null, map = null, settings = null, lens = navLens, recordings = listRecordings, calibrateLive = null, calibrateRecording = null, save = null, restore = null,
    current = null, flying = () => false, mode = null } = {}) {
    super();
    calibrateRecording ??= (id, o) => import("../nav/replay.js").then((m) => m.calibrateRecording({ id, houseId: this.house?.id, ...o }));
    mode ??= () => settings?.get?.("mode") ?? "real";
    Object.assign(this, { el, twin, house, map, settings, lensMod: lens, recordings, calibrateLive, calibrateRecording, saveFn: save, restoreFn: restore, currentFn: current, flying, mode });
    Object.assign(this, { calib: null, result: null, state: "idle", progress: null, ac: null, error: null, recs: [], pick: null, seq: 0, stopWhy: null });
    el.classList.add("cal");
    this.offs = [watchFlying(flying, (air) => {
      if (air && this.state === "running") (this.stopWhy = "flying"), this.ac?.abort();
      this.render();
    }), onVisible(el, () => this.refreshRecordings())];
    this.load();
  }

  // Another house: anything unsaved goes (a live result's lens too). The same house (a new map or twin): only those.
  setHouse(house, twin = this.twin, map = this.map) {
    if (house && house.id === this.house?.id) return void (Object.assign(this, { house, twin, map }), this.render());
    this.forget();
    Object.assign(this, { house, twin, map, error: null, calib: null });
    this.load();
  }

  lensOf(calib) {
    return this.lensMod.droneLens(calib, this.settings?.get?.("uptilt") ?? 20);
  }

  async load() {
    const seq = ++this.seq;
    try {
      const c = typeof this.currentFn === "function" ? await this.currentFn() : this.currentFn ?? (this.house && (await this.readStored()));
      const recs = typeof this.recordings === "function" ? await this.recordings() : this.recordings ?? [];
      if (seq !== this.seq) return;
      this.calib = c ?? null;
      this.setRecs(recs);
    } catch (e) {
      this.error = `Couldn't read the calibration: ${plainWords(e, "reload the page and try again")}.`;
    }
    this.render();
  }

  setRecs(recs) {
    this.recs = (recs ?? []).filter((r) => !r.recording && (!r.house || !this.house || r.house === this.house.id));
    if (!this.recs.some((r) => r.id === this.pick)) this.pick = this.recs[0]?.id ?? null;
  }

  // The server's recordings again (one just made shows up), keeping the one picked.
  async refreshRecordings() {
    if (typeof this.recordings !== "function" || this.state === "running") return;
    const seq = this.seq, recs = await this.recordings().catch(() => null);
    if (!recs || seq !== this.seq || this.state === "running") return;
    const before = this.recs.map((r) => r.id).join();
    this.setRecs(recs);
    if (this.recs.map((r) => r.id).join() !== before) this.render();
  }

  // "Calibrate the camera from this flight" (the localization check): that recording, picked.
  async pickRecording(id) {
    this.pick = id;
    await this.refreshRecordings();
    if (this.recs.some((r) => r.id === id)) this.pick = id;
    this.render();
    this.el.querySelector("select")?.scrollIntoView?.({ block: "nearest" });
  }

  async readStored() {
    const b = await (await openStore()).readFile(this.house.id, "calib.json");
    return b ? JSON.parse(new TextDecoder().decode(b)) : null;
  }

  render() {
    if (this.disposed) return;
    const s = calibSummary(this.calib, (c) => this.lensOf(c)), busy = this.state === "running", air = this.flying(), sim = this.mode() === "sim";
    const recSelect = h("select", { "aria-label": "Recording", disabled: busy || !this.recs.length, onchange: (e) => (this.pick = e.target.value), onfocus: () => this.refreshRecordings() },
      this.recs.length ? this.recs.map((r) => h("option", { value: r.id, selected: r.id === this.pick }, `${r.label || "flight"} · ${r.started ? fmtDate(r.started) : r.id}${r.durationMs ? ` · ${fmtDuration(r.durationMs)}` : ""}`))
        : h("option", {}, "No recordings yet"));
    put(this.el,
      h("p", { class: "cal-now", "data-quality": s.quality ?? "none" }, s.text),
      h("p", { class: "hint" }, "The drone finds its position by matching its camera picture to the 3D scan, so the app must know the camera's lens exactly. Calibrate once, and again if the camera's tilt changes."),
      h("div", { class: "cal-ways" },
        h("section", { class: "cal-way" },
          h("h4", {}, "From a recorded flight ", h("span", { class: "badge" }, "recommended")),
          h("p", { class: "hint" }, `Record a flight (Record, at the top) that starts on the home pad: press ${PAD_BUTTON}, turn the drone slowly by hand through a full circle, then fly. No battery time is spent calibrating.`),
          h("div", { class: "row" }, recSelect,
            h("button", { type: "button", class: "btn small", disabled: busy || air || !this.recs.length || !this.calibrateRecording, onclick: () => this.run("recording") }, "Calibrate from it"))),
        h("section", { class: "cal-way" },
          h("h4", {}, "Live on the pad"),
          h("ol", { class: "cal-steps" }, h("li", {}, `Battery in, goggles on, the drone on its home pad: press ${PAD_BUTTON}.`), h("li", {}, "Press Start, then turn the drone slowly by hand through a full circle (about 20 s)."),
            h("li", {}, "Hold it still for a few seconds while it checks.")),
          sim ? h("p", { class: "hint warn-text" }, "Live calibration needs the real drone's camera: switch to Real drone first. In the simulator, the camera needs no calibration.")
            : h("p", { class: "hint warn-text" }, "The O4 camera overheats about 2½ minutes after the battery goes in: be ready before you plug it in."),
          h("div", { class: "row" }, h("button", { type: "button", class: "btn small", disabled: busy || air || sim || !this.calibrateLive, onclick: () => this.run("live") }, "Start")))),
      air && h("p", { class: "note warn-text" }, "Land first: calibration needs the drone on its pad."),
      busy && this.renderProgress(),
      this.error && h("p", { class: "warn-text", role: "alert" }, this.error),
      this.result && this.renderResult());
  }

  renderProgress() {
    const p = this.progress ?? {}, frac = p.of ? Math.min(1, (p.samples ?? 0) / p.of) : p.done != null && p.total ? p.done / p.total : null;
    return h("div", { class: "import", "data-state": "running" },
      h("p", {}, h("b", {}, PHASE[p.phase] ?? "Calibrating…"), p.phase === "turn" && p.of ? ` ${p.samples ?? 0} of ${p.of} views` : ""),
      h("progress", { max: 1, value: frac ?? undefined, "aria-label": "Calibration progress" }),
      p.text && h("p", { class: "hint", role: "status" }, p.text),
      h("div", { class: "row" }, h("button", { type: "button", class: "btn ghost small", onclick: () => ((this.stopWhy = "you"), this.ac?.abort()) }, "Stop")));
  }

  async run(way) {
    if (this.state === "running") return;
    if (this.flying()) return void ((this.error = "Land first: calibration needs the drone on its pad."), this.render());
    if (way === "live" && this.mode() === "sim") return void ((this.error = "Live calibration needs the real drone's camera: switch to Real drone first."), this.render());
    this.forget();
    Object.assign(this, { state: "running", error: null, progress: { phase: way === "live" ? "turn" : "decode" }, ac: new AbortController(), stopWhy: null });
    this.render();
    try {
      const onProgress = (p) => ((this.progress = { ...this.progress, ...p }), this.render());
      const r = way === "live" ? await this.calibrateLive({ onProgress, signal: this.ac.signal }) : await this.calibrateRecording(this.pick, { onProgress, signal: this.ac.signal });
      if (this.stopWhy) throw new DOMException("Aborted", "AbortError");
      if (!r?.calib?.fx) throw new Error("no calibration came back");
      this.result = { calib: { source: way === "live" ? "pad" : "recording", ...r.calib }, verify: r.verify ?? null, way, picture: null };
      this.picture();
    } catch (e) {
      if (way === "live") this.restoreFn?.(); // whatever it got to, the saved lens is the one in use
      this.error = this.stopWhy === "flying" ? "Stopped: the drone took off. The lens is unchanged."
        : e?.name === "AbortError" || this.stopWhy ? "Stopped: the lens is unchanged."
        : `Calibration didn't work: ${padWords(plainWords(e, "the check stopped before the end (reload the page and try again)"))}. The lens is unchanged.`;
    }
    this.state = "idle";
    this.ac = null;
    this.render();
  }

  renderResult() {
    const { calib, verify, way } = this.result, s = calibSummary(calib, (c) => this.lensOf(c)), was = this.lensOf(this.calib), now = this.lensOf(calib), shift = lensShift(was, now, calib.width, calib.height);
    const canvas = h("canvas", { class: "cal-pic", width: 640, height: 480, role: "img",
      "aria-label": `Before and after: the biggest shift between the old and the new lens model is ${Math.round(shift.max)} pixels` });
    this.drawPicture(canvas, shift);
    return h("div", { class: "cal-result", "data-quality": s.quality },
      h("p", {}, h("b", {}, { good: "Good calibration", usable: "Usable calibration", poor: "Poor calibration" }[s.quality]), `: ${s.details}. `,
        h("b", {}, way === "live" ? "In use now until you save or discard it." : "Not saved yet.")),
      verify && h("p", { class: "hint" }, `Checked on the pad: ${verify.accepted} of ${verify.n} views matched, ${(verify.medianErr * 100).toFixed(0)} cm from the pad.`),
      canvas,
      h("p", { class: "hint" }, `Grey dots: where the ${this.calib?.fx ? "old calibration" : "standard lens"} put points of the picture; yellow: where the new one does. Biggest shift ${Math.round(shift.max)} px, average ${Math.round(shift.mean)} px (the picture is ${calib.width} px wide).`),
      s.quality === "poor" && h("p", { class: "warn-text" }, "Saving a poor calibration is allowed, but position fixes will be less sure: try again with more light and a slower turn."),
      h("div", { class: "row" },
        h("button", { type: "button", class: "btn", onclick: () => this.save() }, "Save it"),
        h("button", { type: "button", class: "btn ghost", onclick: () => this.discard() }, way === "live" ? "Discard (back to the saved lens)" : "Discard")));
  }

  discard() {
    this.forget();
    this.render();
  }

  // An unsaved result goes; a live one's lens with it (the saved lens back in use).
  forget() {
    const r = this.result;
    if (!r) return;
    this.result = null;
    r.picture?.close?.();
    if (r.way === "live") this.restoreFn?.();
  }
  // The 3D scan from the home pad through the new lens (once), under the dots.
  async picture() {
    const r = this.result, home = this.house?.home;
    if (!this.twin || !home || !r) return;
    try {
      const lens = resolveLens(this.lensOf(r.calib)), z = (this.map?.floorAt?.(home.x, home.y) ?? 0) + 0.05;
      const bmp = await this.twin.render({ x: home.x, y: home.y, z, yaw: home.yaw ?? 0, pitch: 0, roll: 0 }, { width: 320, height: 240, lens, actors: false, props: false });
      if (this.result !== r) return bmp.close?.();
      r.picture = bmp;
      this.render();
    } catch {}
  }

  drawPicture(c, shift) {
    const g = c.getContext("2d"), kx = c.width / shift.width, ky = c.height / shift.height, pic = this.result?.picture;
    g.fillStyle = "#0a0e13";
    g.fillRect(0, 0, c.width, c.height);
    if (pic) {
      g.globalAlpha = 0.85;
      g.drawImage(pic, 0, 0, c.width, c.height);
      g.globalAlpha = 1;
    }
    g.lineWidth = 2;
    for (const p of shift.points) {
      const [x, y, x0, y0] = [p.u * kx, p.v * ky, p.u0 * kx, p.v0 * ky];
      g.strokeStyle = "rgba(255,216,77,0.8)";
      g.beginPath();
      g.moveTo(x0, y0);
      g.lineTo(x, y);
      g.stroke();
      g.fillStyle = "rgba(203,213,225,0.9)";
      g.beginPath();
      g.arc(x0, y0, 3.5, 0, 7);
      g.fill();
      g.fillStyle = "#ffd84d";
      g.beginPath();
      g.arc(x, y, 4.5, 0, 7);
      g.fill();
    }
  }

  async save() {
    const calib = this.result?.calib;
    if (!calib) return;
    try {
      if (this.saveFn) await this.saveFn(calib);
      else await (await openStore()).writeFile(this.house.id, "calib.json", JSON.stringify(calib));
      this.calib = calib;
      this.result?.picture?.close?.();
      this.result = null;
      this.error = null;
      this.emit("saved", { calib });
    } catch (e) {
      this.error = `Couldn't save it: ${plainWords(e, "the house's storage didn't take it")}. ${this.result.way === "live" ? "It stays in use until you discard it or reload the page." : ""}`.trim();
    }
    this.render();
  }

  dispose() {
    this.seq++;
    this.disposed = true;
    this.offs.forEach((f) => f());
    this.stopWhy ??= "closed";
    this.ac?.abort();
    this.forget();
    put(this.el);
    this.el.classList.remove("cal");
  }
}
