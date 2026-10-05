// Record flight: a recording on the server (tools/recorder.mjs through whoop.mjs: the goggles' video as received, plus
// what this page posts) with the page's telemetry about 10 times a second, sent in batches every second, and every
// command (what you said or clicked, Claude's tool calls, missions). The server stops by itself at its caps, when the
// disk fills or when the page goes quiet for 120 s; a status poll notices and says why. Recordings are listed in
// Settings → Recordings. Hidden when the app runs without its Node server (serverSession() is null).
import { Emitter } from "../util.js";
import { serverSession, serverPost } from "../goggles/source.js";
import { h, fmtBytes, fmtDate, fmtDuration, fmtClock } from "./dom.js";

export class FlightRecorder extends Emitter {
  // sample() -> one telemetry object (t added here); context() -> /rec/start's body ({ label, house, lens, app }).
  constructor({ button, list, sample, context }) {
    super();
    Object.assign(this, { button, list, sample, context, rec: null, tele: [], cmds: [], lastSample: 0, timers: [], busy: false });
    button.addEventListener("click", () => (this.rec ? this.stop("stopped by you") : this.start()));
  }

  async init() {
    const s = await serverSession();
    this.button.hidden = !s;
    if (s?.recording) this.resume(await fetch("/rec/status").then((r) => r.json()).catch(() => null));
    this.render();
    return !!s;
  }

  get recording() {
    return !!this.rec;
  }

  tick(now) {
    if (!this.rec || now - this.lastSample < 100) return;
    this.lastSample = now;
    this.tele.push({ t: Date.now(), ...this.sample() });
  }

  command(tool, args = {}, source = "ui") {
    if (this.rec) this.cmds.push({ t: Date.now(), tool, args, source });
  }

  async start() {
    if (this.busy) return;
    this.busy = true;
    try {
      this.resume(await serverPost("/rec/start", this.context()));
      this.emit("started", this.rec);
    } catch (e) {
      this.emit("error", e.status === 409 ? "The server is already recording." : `Couldn't start recording: ${e.message}`);
    } finally {
      this.busy = false;
      this.render();
    }
  }

  resume(status) {
    if (!status?.recording) return;
    this.rec = { ...status, since: Date.now() - (status.durationMs ?? 0) };
    this.timers.forEach(clearInterval);
    this.timers = [setInterval(() => this.flush(), 1000), setInterval(() => this.poll(), 3000), setInterval(() => this.render(), 500)];
  }

  async flush() {
    if (!this.rec) return;
    const [tele, cmds] = [this.tele.splice(0), this.cmds.splice(0)];
    try {
      await serverPost("/rec/telemetry", tele);
      if (cmds.length) await serverPost("/rec/command", cmds);
    } catch (e) {
      if (e.status === 409) await this.poll();
      else console.warn("recorder", e.message);
    }
  }

  async poll() {
    const s = await fetch("/rec/status").then((r) => r.json()).catch(() => null);
    if (!s || !this.rec) return;
    if (s.recording) Object.assign(this.rec, s);
    else this.ended(s.last ?? {});
  }

  async stop(reason) {
    if (!this.rec || this.busy) return;
    this.busy = true;
    try {
      await this.flush();
      this.ended(await serverPost("/rec/stop", { reason }), true);
    } catch (e) {
      if (e.status === 409) await this.poll();
      else this.emit("error", `Couldn't stop recording: ${e.message}`);
    } finally {
      this.busy = false;
      this.render();
    }
  }

  ended(last, byUs = false) {
    this.timers.forEach(clearInterval);
    this.timers = [];
    this.rec = null;
    this.tele = [];
    this.cmds = [];
    this.emit("stopped", { ...last, byUs });
    this.render();
    this.refreshList();
  }

  render() {
    const b = this.button, r = this.rec;
    b.classList.toggle("on", !!r);
    b.setAttribute("aria-pressed", String(!!r));
    b.querySelector("span").textContent = r ? `${fmtClock(Date.now() - r.since)}${r.bytes ? ` · ${fmtBytes(r.bytes)}` : ""}` : "Record";
    b.title = r ? "Stop recording this flight" : "Record this flight: the goggles' video, telemetry and every command, kept on this Mac";
    b.setAttribute("aria-label", r ? "Stop recording this flight" : "Record this flight"); // at phone width only its dot shows
  }

  async refreshList() {
    if (!this.list) return;
    const j = await fetch("/rec/list").then((r) => (r.ok ? r.json() : null)).catch(() => null);
    const recs = j?.recordings ?? [];
    if (!j) return this.list.replaceChildren(h("p", { class: "note" }, "Recordings need Whoop Pilot's server: start it with start.command."));
    if (!recs.length)
      return this.list.replaceChildren(h("p", { class: "note" }, "No recordings yet. Record (at the top) keeps a flight's goggles video, telemetry and commands on this Mac."));
    this.list.replaceChildren(h("table", { class: "recs" },
      h("thead", {}, h("tr", {}, ["Flight", "Length", "Size", "Ended"].map((t) => h("th", {}, t)))),
      h("tbody", {}, recs.map((r) => h("tr", {},
        h("td", {}, h("strong", {}, r.label || "flight"), h("br"), h("small", {}, r.started ? fmtDate(r.started) : r.id)),
        h("td", {}, r.recording ? "recording" : r.durationMs != null ? fmtDuration(r.durationMs) : "—"),
        h("td", {}, fmtBytes(Object.values(r.files ?? {}).reduce((a, b) => a + b, 0))),
        h("td", {}, r.stopReason ?? "—"),
      ))),
    ));
  }
}
