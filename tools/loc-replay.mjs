// Flight recordings for localization replay (the app's Settings -> Recordings check, app/js/nav/replay.js, or
// tools/loc-check.html?only=replay&rec=<id>): lists what the recorder kept
// and checks that a recording can be replayed (video units, telemetry with heading and flow, a pad reset or a first
// pose, the lens and the picture region in a 16:9 goggles stream; without the simulator's truth, a flight that starts
// and lands on the home pad, which is then the truth), then prints the page to open. --add <folder> copies a recording's files (meta.json, video.h264,
// video.jsonl, telemetry.jsonl, commands.jsonl; loc-check saves its synthetic one as c-loc-rec-*) in as a new one.
// Usage: node loc-replay.mjs [id] [--dir <recordings>] [--add <folder> [--prefix c-loc-rec-]] [--port 8790]
import fs from "node:fs";
import path from "node:path";
import { RECORDINGS } from "./recorder.mjs";
import { recordingProblems } from "../app/js/nav/replay.js";

const args = process.argv.slice(2), opt = (k, d) => { const i = args.indexOf(k); return i >= 0 ? args.splice(i, 2)[1] : d; };
const dir = opt("--dir", RECORDINGS), add = opt("--add", null), prefix = opt("--prefix", ""), port = opt("--port", "8790");
const FILES = ["meta.json", "video.h264", "video.jsonl", "telemetry.jsonl", "commands.jsonl"];
const jsonl = (f) => (fs.existsSync(f) ? fs.readFileSync(f, "utf8").split("\n").filter(Boolean).map((l) => JSON.parse(l)) : []);

if (add) {
  const meta = JSON.parse(fs.readFileSync(path.join(add, `${prefix}meta.json`), "utf8"));
  const id = `${new Date().toISOString().slice(0, 19).replace(/:/g, "-")}-${String(meta.label || "replay").toLowerCase().replace(/[^a-z0-9]+/g, "-").slice(0, 40)}`;
  fs.mkdirSync(path.join(dir, id), { recursive: true });
  for (const f of FILES) {
    const src = path.join(add, prefix + f);
    fs.writeFileSync(path.join(dir, id, f), fs.existsSync(src) ? fs.readFileSync(src) : "");
  }
  console.log(`added ${id}`);
  args.unshift(id);
}

const ids = fs.existsSync(dir) ? fs.readdirSync(dir).filter((d) => fs.existsSync(path.join(dir, d, "meta.json"))).sort().reverse() : [];
if (!args[0]) {
  if (!ids.length) console.log(`No recordings in ${dir}. Record a flight in the app (Record, at the top), then run this again.`);
  for (const id of ids) {
    const m = JSON.parse(fs.readFileSync(path.join(dir, id, "meta.json"), "utf8"));
    console.log(`${id}  ${((m.durationMs ?? 0) / 1000).toFixed(0)} s, ${m.video?.units ?? 0} video units, ${m.telemetry ?? 0} telemetry samples, house ${m.house ?? "-"}`);
  }
  process.exit(0);
}

const id = args[0], d = path.join(dir, id);
if (!fs.existsSync(path.join(d, "meta.json"))) throw new Error(`no recording ${id} in ${dir}`);
const meta = JSON.parse(fs.readFileSync(path.join(d, "meta.json"), "utf8")), index = jsonl(path.join(d, "video.jsonl")), tel = jsonl(path.join(d, "telemetry.jsonl")), cmds = jsonl(path.join(d, "commands.jsonl"));
const video = fs.existsSync(path.join(d, "video.h264")) ? fs.statSync(path.join(d, "video.h264")).size : 0;
const span = (a) => (a.length > 1 ? (a.at(-1).t - a[0].t) / 1000 : 0);
const flying = tel.filter((s) => s.fm && !s.fm.includes("*") && s.sticks?.thr > 0.22), pad = cmds.find((c) => c.tool === "pad");
const start = tel.find((s) => s.pose?.status === "ok" && !flying.includes(s));
const problems = recordingProblems({ meta, index, telemetry: tel, commands: cmds }, video); // the app's Settings -> Recordings check says the same
console.log(JSON.stringify({
  id, label: meta.label, house: meta.house, lens: meta.lens, videoDelay: meta.app?.videoDelay ?? null,
  video: { units: index.length, bytes: video, seconds: +span(index).toFixed(1), fps: +(index.length / Math.max(1e-3, span(index))).toFixed(1) },
  telemetry: { samples: tel.length, seconds: +span(tel).toFixed(1), hz: +(tel.length / Math.max(1e-3, span(tel))).toFixed(1), flyingSeconds: +(flying.length / Math.max(1e-3, tel.length / Math.max(1e-3, span(tel)))).toFixed(1),
    truth: tel.some((s) => s.truth) },
  start: pad ? "pad command" : start ? "first pose on the ground" : "relocalization", problems,
}, null, 2));
console.log(`\nReplay it (whoop.mjs running): http://localhost:${port}/tools/loc-check.html?only=replay&rec=${encodeURIComponent(id)}${meta.app?.videoDelay == null ? "&recDelay=<ms from the camera to the Mac, e.g. 60>" : ""}`);
