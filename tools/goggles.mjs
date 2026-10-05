#!/usr/bin/env node
// DJI Goggles 3 live view from the command line: diagnostics for the goggles side of Whoop Pilot.
// The app itself gets goggles video from its server (whoop.mjs, started by start.command), which runs
// the same code (goggles-helper.mjs); stop that server first, the goggles take one client at a time.
//
//   node goggles.mjs probe              connect for 9 s and report what the goggles send
//   node goggles.mjs capture [seconds]  save the live H.264 to goggles-capture.h264 and analyze it
//   node goggles.mjs analyze [file]     the same analysis on a saved capture (no goggles needed)
//   node goggles.mjs serve              stream status and video on http://localhost:8791 (/ and /stream)
//   --wifi             the goggles' Wi-Fi hotspot instead of USB (join their network first)
//   --peer IP[:port]   the goggles' address (192.168.60.2 on USB, 192.168.2.1 on Wi-Fi)
//   --local-port N     local UDP port on Wi-Fi (default 9003, 0 = any free one; the goggles answer to it)
//
// On the goggles (menu names from the DJI Goggles 3 manual), USB: Settings > About > OTG Wired Connection
// off, and Share Liveview to Mobile Device via Wi-Fi on (despite the name it also enables the USB live view).
// Wi-Fi: Share Liveview to Mobile Device via Wi-Fi on (push the 5D button back for the shortcut menu).
// Protocol details: dji-goggles-lab (https://github.com/soldnenz/dji-goggles-lab) and orbit-rs.
import http from "node:http";
import fs from "node:fs";
import path from "node:path";
import { Goggles, GogglesHelper, PORT, explain } from "./goggles-helper.mjs";
import { describeUnit, codecString, nalUnits, nalBytes, NAL } from "../app/js/goggles/h264.js";

const HTTP_PORT = 8791;
const WHOOP = "http://127.0.0.1:8790"; // whoop.mjs
const OLD_STALL = 94; // video packets after which the goggles stopped with the old echo-style ACK
const log = (...a) => console.log(new Date().toISOString().slice(11, 23), ...a);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const opts = { wifi: false, peer: null, localPort: PORT };
const positional = [];
for (let i = 2; i < process.argv.length; i++) {
  const a = process.argv[i];
  if (a === "--wifi") opts.wifi = true;
  else if (a === "--usb") opts.wifi = false;
  else if (a === "--peer") opts.peer = process.argv[++i];
  else if (a === "--local-port") opts.localPort = Number(process.argv[++i]);
  else if (a === "-h" || a === "--help") usage();
  else if (a.startsWith("-")) usage(`Unknown option ${a}`);
  else positional.push(a);
}
const [cmd = "probe", arg] = positional;
const goggles = new Goggles({ ...opts, log });

function usage(msg) {
  if (msg) console.error(msg);
  console.log("usage: node goggles.mjs probe | capture [seconds] | analyze [file] [seconds] | serve   [--wifi] [--peer IP[:port]] [--local-port N]");
  process.exit(msg ? 2 : 0);
}

// The goggles serve one LiveView client: with Whoop Pilot's server running, this one would fight it.
async function whoopRunning() {
  const r = await fetch(`${WHOOP}/goggles/status`, { signal: AbortSignal.timeout(800) }).catch(() => null);
  const j = r?.ok && (await r.json().catch(() => null));
  if (j?.app !== "whoop-pilot-goggles") return false;
  log(`Whoop Pilot's server is running and has the goggles (${j.state}${j.device ? `, ${j.device}` : ""}). Its status: ${WHOOP.replace("127.0.0.1", "localhost")}/goggles/status. Close its start.command window to use this tool.`);
  return true;
}

const exitOnDown = (e) => {
  log("USB link error:", e.message);
  process.exit(1);
};

function report(session, secs) {
  const s = session.stats;
  log(`After ${secs} s: session ${session.state} (id 0x${session.session.toString(16)}${s.serial ? `, goggles serial ${s.serial}` : ""}), ${s.units} frames (${(s.bytes / 1e6).toFixed(1)} MB), ${s.videoPackets} video packets, ${s.duplicates} duplicates, ${s.resendRequests} resend requests, ${s.lostPackets} packets given up, ${s.droppedUnits} frames dropped, ${s.acks} ACKs sent; the goggles' window is ${s.outstanding} packets ahead of ours, ${s.stalls} stalls.`);
  log(s.videoPackets > 200
    ? `Video kept flowing past ${OLD_STALL} packets, where the old echo-style ACK stalled: the ACK windows work.`
    : s.videoPackets ? `Only ${s.videoPackets} video packets. If it stopped near ${OLD_STALL}, the goggles aren't accepting our ACKs.` : explain(session.state));
  log("stats:", JSON.stringify(s));
}

// Access units of an Annex-B file: each ends with its AUD, as the goggles send them.
function splitUnits(b) {
  const units = [];
  let from = 0;
  for (const n of nalUnits(b)) if (n.type === NAL.AUD) {
    units.push(b.subarray(from, n.end));
    from = n.end;
  }
  if (from < b.length) units.push(b.subarray(from));
  return units;
}

function analyze(units, seconds) {
  const hist = {};
  let idr = 0;
  let intra = 0;
  let recovery = 0;
  let sps = null;
  let bytes = 0;
  const idrAt = [];
  units.forEach((u, i) => {
    bytes += u.length;
    const d = describeUnit(u);
    d.types.forEach((t) => (hist[t] = (hist[t] || 0) + 1));
    if (d.idr) {
      idr++;
      idrAt.push(i);
    }
    if (d.intra && !d.idr) intra++;
    if (d.recovery) recovery++;
    if (d.sps && !sps) sps = codecString(u, d.sps);
  });
  const gaps = idrAt.slice(1).map((v, i) => v - idrAt[i]);
  console.log(`
${units.length} units in ${seconds}s (${(units.length / seconds).toFixed(1)} fps), ${((bytes * 8) / seconds / 1e6).toFixed(1)} Mbit/s
codec: ${sps || "no SPS seen"}
NAL types: ${Object.entries(hist).map(([t, n]) => `${t}:${n}`).join("  ")}   (1 slice, 5 IDR, 6 SEI, 7 SPS, 8 PPS, 9 AUD)
IDR frames: ${idr}${gaps.length ? `, every ~${Math.round(gaps.reduce((a, b) => a + b, 0) / gaps.length)} frames` : ""}
all-intra non-IDR frames: ${intra}
recovery-point SEIs: ${recovery}
=> ${idr ? "The app can start decoding at each IDR." : intra ? "No IDRs, but intra frames exist: the app starts there."
    : "No IDR or I frames. That's normal for these goggles: they refresh the picture a few rows per frame (intra refresh) and never send a keyframe, so a plain player shows garbage until a full refresh cycle has passed. The app bootstraps its decoder with a synthetic keyframe (app/js/goggles/bootstrap.js) and starts on the first P frame."}`);
}

// <capture>-bootstrapped.h264: the app's decoder bootstrap (its best candidate) in front of the stream
// from the first slice that follows an SPS/PPS, so ffplay starts cleanly. Needs app/js/goggles/bootstrap.js.
async function writeBootstrapped(units, capture) {
  let bootstrapUnits;
  try {
    ({ bootstrapUnits } = await import("../app/js/goggles/bootstrap.js"));
  } catch {
    return null;
  }
  let sps;
  let pps;
  let slice;
  let from = 0;
  for (let i = 0; i < units.length && !slice; i++) {
    for (const n of nalUnits(units[i])) {
      if (n.type === NAL.SPS) sps ??= nalBytes(units[i], n);
      else if (n.type === NAL.PPS) pps ??= nalBytes(units[i], n);
      else if ((n.type === NAL.SLICE || n.type === NAL.IDR) && sps && pps && !slice) {
        slice = nalBytes(units[i], n);
        from = i;
      }
    }
  }
  if (!slice) return null;
  let boot;
  try {
    boot = bootstrapUnits(sps, pps, slice)[0];
  } catch (e) {
    log(`bootstrap.js couldn't build a start unit: ${e.message}`);
    return null;
  }
  const file = capture.replace(/(\.h264)?$/, "-bootstrapped.h264");
  const out = fs.createWriteStream(file);
  out.write(boot.data);
  for (const u of units.slice(from)) out.write(u);
  await new Promise((r) => out.end(r));
  log(`Bootstrap variant "${boot.name}" (${boot.data.length} bytes) in front of ${units.length - from} units.`);
  return file;
}

async function playable(units, file) {
  const boot = await writeBootstrapped(units, file);
  log(boot
    ? `Also ${boot}: decoder bootstrap + stream. Play it with: ffplay ${path.basename(boot)}`
    : `Play it with: ffplay -f h264 ${path.basename(file)} (garbage at first is expected: no keyframes)`);
}

process.on("SIGINT", () => process.exit(0));

if ((cmd === "probe" || cmd === "capture" || cmd === "serve") && (await whoopRunning())) process.exit(1);

if (cmd === "probe") {
  let conn;
  try {
    conn = await goggles.connect(() => {}, { onDown: exitOnDown, waitMs: 10000 });
  } catch (e) {
    console.log(e.message);
    process.exit(1);
  }
  await sleep(9000); // past the session's 8 s no-answer verdict
  report(conn.session, 9);
  process.exit(conn.session.state === "live" ? 0 : 1);
} else if (cmd === "capture") {
  const seconds = Number(arg) || 10;
  const units = [];
  const file = path.resolve("goggles-capture.h264");
  const out = fs.createWriteStream(file);
  let t0 = null;
  let conn;
  try {
    conn = await goggles.connect((u) => {
      t0 ??= Date.now();
      units.push(u);
      out.write(u);
    }, { onDown: exitOnDown });
  } catch (e) {
    console.log(e.message);
    process.exit(1);
  }
  const started = Date.now();
  for (;;) {
    await sleep(200);
    if (t0 && Date.now() - t0 > seconds * 1000) break;
    if (!t0 && Date.now() - started > 30000) {
      log(`No video after 30 s (session ${conn.session.state}). ${explain(conn.session.state)}`);
      process.exit(1);
    }
  }
  conn.close();
  await new Promise((r) => out.end(r));
  analyze(units, seconds);
  report(conn.session, seconds);
  log(`Saved ${file}`);
  await playable(units, file);
  process.exit(0);
} else if (cmd === "analyze") {
  const file = path.resolve(arg || "goggles-capture.h264");
  if (!fs.existsSync(file)) {
    console.log(`No such file: ${file}. Record one first: node goggles.mjs capture 10`);
    process.exit(1);
  }
  const units = splitUnits(new Uint8Array(fs.readFileSync(file)));
  const seconds = Number(positional[2]) || units.length / 60; // the goggles send 60 fps
  analyze(units, seconds);
  await playable(units, file);
  process.exit(0);
} else if (cmd === "serve") {
  const helper = new GogglesHelper({ ...opts, log });
  // Diagnostics only (curl, a terminal): no CORS, and browsers are turned away; the app uses whoop.mjs.
  const hosts = new Set([`localhost:${HTTP_PORT}`, `127.0.0.1:${HTTP_PORT}`]);
  const server = http.createServer((req, res) => {
    res.setHeader("Cache-Control", "no-store");
    const site = req.headers["sec-fetch-site"];
    if (!hosts.has(req.headers.host) || req.headers.origin || (site && site !== "none")) {
      res.writeHead(403, { "Content-Type": "text/plain" });
      return res.end("This diagnostics server answers terminals only. Whoop Pilot gets goggles video from start.command's server.\n");
    }
    if (req.url === "/stream") return helper.attach(req, res);
    res.writeHead(200, { "Content-Type": "application/json" });
    res.end(JSON.stringify(helper.status()));
  });
  server.on("error", (e) => {
    if (e.code !== "EADDRINUSE") throw e;
    log(`Port ${HTTP_PORT} is taken (another goggles.mjs serve?). Quit that program and try again.`);
    process.exit(1);
  });
  server.listen(HTTP_PORT, "127.0.0.1", () => log(`Goggles diagnostics on http://localhost:${HTTP_PORT} (status) and /stream over ${opts.wifi ? "Wi-Fi" : "USB"}.`));
  helper.start();
} else {
  usage(`Unknown command ${cmd}`);
}
