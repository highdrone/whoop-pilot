// Flight recorder: one folder per sortie with the goggles' video exactly as received and what the page
// saw and did, for lens calibration and localization replay.
//   video.h264       Annex-B access units as the goggles sent them (no keyframes: bootstrap.js starts a decoder)
//   video.jsonl      per unit {t, bytes, off}: receive time (Unix epoch ms, sub-ms), size, offset in video.h264
//   telemetry.jsonl  posted by the page, one JSON object per line (t added when missing)
//   commands.jsonl   likewise
//   meta.json        label, app, house, lens and whatever else /rec/start got, plus the server's side; final at stop
// Stops by itself at the size cap, the duration cap, when the disk gets down to RESERVE free (checked every
// second: other programs fill it too), or when the page has posted nothing for idleMs (closed tab).
import fs from "node:fs";
import fsp from "node:fs/promises";
import os from "node:os";
import path from "node:path";

export const RECORDINGS = path.join(os.homedir(), "Library", "Application Support", "WhoopPilot", "recordings");
const RESERVE = 1 << 30; // leave this much of the disk free
const STALL = 64 << 20; // video bytes waiting for the disk before units are dropped
const okName = (s) => /^[\w][\w.-]*$/.test(s);
const slug = (s) => String(s || "").toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-+|-+$/g, "").slice(0, 40) || "flight";
const stamp = (d) => new Date(d - d.getTimezoneOffset() * 60e3).toISOString().slice(0, 19).replace(/:/g, "-");
const now = () => Math.round((performance.timeOrigin + performance.now()) * 100) / 100;
const gb = (n) => `${(n / 2 ** 30).toFixed(1)} GB`;
const statfsFree = (dir) => fsp.statfs(dir).then(({ bavail, bsize }) => bavail * bsize);
export const httpError = (status, message) => Object.assign(new Error(message), { status });

async function writeJson(file, obj) {
  await fsp.writeFile(file + ".tmp", JSON.stringify(obj, null, 2) + "\n");
  await fsp.rename(file + ".tmp", file);
}

export class Recorder {
  constructor({ dir = RECORDINGS, maxBytes = 2 * 2 ** 30, maxMs = 20 * 60e3, idleMs = 120e3, free = statfsFree, log = () => {}, context = () => ({}) } = {}) {
    Object.assign(this, { dir, maxBytes, maxMs, idleMs, free, log, context, rec: null, last: null, starting: null });
  }

  get recording() {
    return !!this.rec;
  }

  // A stop that arrives while this is still opening the folder waits for it, then stops it.
  async start(opts = {}) {
    if (this.rec || this.starting) throw httpError(409, `Already recording (${this.rec?.id ?? "starting"}).`);
    this.starting = this.open(opts);
    try {
      return await this.starting;
    } finally {
      this.starting = null;
    }
  }

  async open({ label, ...meta }) {
    await fsp.mkdir(this.dir, { recursive: true });
    const avail = await this.free(this.dir);
    if (avail - RESERVE < 100e6) throw httpError(507, `Only ${gb(avail)} free on this Mac's disk: free some space to record.`);
    const cap = Math.min(this.maxBytes, avail - RESERVE);
    const t0 = new Date();
    let id = `${stamp(t0)}-${slug(label)}`;
    for (let n = 2; ; n++) {
      try {
        await fsp.mkdir(path.join(this.dir, id));
        break;
      } catch (e) {
        if (e.code !== "EEXIST" || n > 99) throw e;
        id = `${stamp(t0)}-${slug(label)}-${n}`;
      }
    }
    const dir = path.join(this.dir, id);
    const open = (f) => fs.createWriteStream(path.join(dir, f), { flags: "wx" }).on("error", (e) => this.end(`write error: ${e.message}`));
    const rec = (this.rec = {
      id, dir, label: String(label || ""), t0: t0.getTime(), cap, meta, bytes: 0, lastPost: Date.now(),
      video: { units: 0, bytes: 0, dropped: 0 }, telemetry: 0, commands: 0,
      out: { video: open("video.h264"), index: open("video.jsonl"), telemetry: open("telemetry.jsonl"), commands: open("commands.jsonl") },
      at: { start: this.context() },
    });
    await writeJson(path.join(dir, "meta.json"), this.meta(rec));
    this.timer = setInterval(() => this.check(), 1000);
    this.log(`Recording ${id} (cap ${gb(cap)}, ${this.maxMs / 60e3} min).`);
    return this.status();
  }

  unit(u) {
    const r = this.rec;
    if (!r) return;
    const v = r.video;
    if (r.out.video.writableLength > STALL) return void v.dropped++;
    const line = `{"t":${now()},"bytes":${u.length},"off":${v.bytes}}\n`;
    r.out.video.write(u);
    r.out.index.write(line);
    v.units++;
    v.bytes += u.length;
    r.bytes += u.length + line.length;
    if (r.bytes > r.cap) this.end(r.cap < this.maxBytes ? "disk nearly full" : "size cap");
  }

  // kind: "telemetry" | "commands"; items: an array (or one object) of JSON values.
  lines(kind, items) {
    const r = this.rec;
    if (!r) throw httpError(409, "Not recording.");
    r.lastPost = Date.now();
    const list = Array.isArray(items) ? items : [items];
    let s = "";
    for (const it of list) s += JSON.stringify(it && typeof it === "object" && !Array.isArray(it) && it.t == null ? { t: Date.now(), ...it } : it) + "\n";
    r.out[kind].write(s);
    r[kind] += list.length;
    r.bytes += Buffer.byteLength(s);
    if (r.bytes > r.cap) this.end("size cap");
    return { recording: true, id: r.id, written: list.length };
  }

  check() {
    const r = this.rec;
    if (!r) return;
    if (Date.now() - r.t0 > this.maxMs) return this.end("time cap");
    if (Date.now() - r.lastPost > this.idleMs) return this.end(`nothing from the page for ${this.idleMs / 1000} s`);
    this.freeCheck ??= this.free(this.dir)
      .then((n) => n < RESERVE && this.rec === r && this.end("disk nearly full"), () => {})
      .finally(() => (this.freeCheck = null));
  }

  // Stops from inside (caps, errors): whoever comes second finds nothing to stop.
  end(reason) {
    if (this.rec) this.stop(reason).catch((e) => this.log(`Recorder: ${e.message}`));
  }

  async stop(reason = "stopped") {
    if (!this.rec && this.starting) await this.starting.catch(() => {});
    const r = this.rec;
    if (!r) throw httpError(409, "Not recording.");
    this.rec = null;
    clearInterval(this.timer);
    r.ended = Date.now();
    r.reason = reason;
    r.at.stop = this.context();
    await Promise.all(Object.values(r.out).map((s) => new Promise((done) => (s.closed ? done() : s.end(done)))));
    await writeJson(path.join(r.dir, "meta.json"), this.meta(r)).catch((e) => this.log(`meta.json: ${e.message}`));
    this.last = this.summary(r);
    this.log(`Recording ${r.id} stopped (${reason}): ${((r.ended - r.t0) / 1000).toFixed(0)} s, ${r.video.units} video units, ${(r.bytes / 1e6).toFixed(1)} MB.`);
    return { recording: false, ...this.last };
  }

  summary(r) {
    return {
      id: r.id, label: r.label, started: new Date(r.t0).toISOString(), durationMs: (r.ended ?? Date.now()) - r.t0, bytes: r.bytes,
      video: { ...r.video }, telemetry: r.telemetry, commands: r.commands, ...(r.reason && { stopReason: r.reason }),
    };
  }

  meta(r) {
    const { app = null, house = null, lens = null, ...extra } = r.meta;
    return {
      ...extra, ...this.summary(r), app, house, lens,
      ended: r.ended ? new Date(r.ended).toISOString() : null,
      clock: "t fields are Unix epoch milliseconds; video units carry the server's receive time",
      files: { video: "video.h264", index: "video.jsonl", telemetry: "telemetry.jsonl", commands: "commands.jsonl" },
      limits: { maxBytes: r.cap, maxMs: this.maxMs, idleMs: this.idleMs },
      server: { node: process.version, platform: `${process.platform} ${os.release()}`, ...r.at.start.server },
      goggles: { start: r.at.start.goggles ?? null, stop: r.at.stop?.goggles ?? null },
    };
  }

  status() {
    return this.rec ? { recording: true, ...this.summary(this.rec), cap: this.rec.cap } : { recording: false, last: this.last };
  }

  async list() {
    const out = [];
    for (const e of await fsp.readdir(this.dir, { withFileTypes: true }).catch(() => [])) {
      if (!e.isDirectory() || !okName(e.name)) continue;
      const dir = path.join(this.dir, e.name);
      const meta = await fsp.readFile(path.join(dir, "meta.json"), "utf8").then(JSON.parse).catch(() => null);
      const files = {};
      for (const f of await fsp.readdir(dir).catch(() => [])) if (okName(f) && !f.endsWith(".tmp")) files[f] = (await fsp.stat(path.join(dir, f)).catch(() => null))?.size ?? null;
      out.push({ id: e.name, recording: this.rec?.id === e.name, label: meta?.label ?? null, started: meta?.started ?? null, durationMs: meta?.durationMs ?? null, stopReason: meta?.stopReason ?? null, house: meta?.house ?? null, video: meta?.video ?? null, files });
    }
    return out.sort((a, b) => b.id.localeCompare(a.id));
  }

  // The real path of one file of one recording, or null.
  async file(id, name) {
    if (!okName(id) || !okName(name)) return null;
    const root = await fsp.realpath(this.dir).catch(() => null);
    const p = root && (await fsp.realpath(path.join(root, id, name)).catch(() => null));
    return p && p.startsWith(path.join(root, id) + path.sep) ? p : null;
  }
}
