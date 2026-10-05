// DJI Goggles 3 as a video source for Whoop Pilot, with no viewer app:
//   goggles -> RNDIS over USB-C (or UDP over the goggles' Wi-Fi) -> LiveView session -> H.264 -> WebCodecs -> canvas.
// Chrome can't do the USB part itself: the goggles' RNDIS control interface is USB class 0xE0
// (wireless controller), on the WebUSB spec's protected-class list and in Chromium's
// WebUsbServiceImpl::GetProtectedInterfaceClasses, so claimInterface() is a SecurityError. So the app's
// own server (tools/whoop.mjs, Node) talks to the goggles and streams the H.264 to us at HELPER_URL on
// this origin, and the page decodes it in a worker (js/goggles/worker.js). Direct WebUSB stays as a
// fallback for goggles or firmware that expose the network interface with a class Chrome allows.
import { GogglesPipeline } from "./pipeline.js";
import { Emitter } from "../util.js";

export const DJI_VENDOR_ID = 0x2ca3;
export const HELPER_URL = "/goggles"; // tools/whoop.mjs: /goggles/status, /goggles/<token>/stream

// The server's per-launch session ({ token, build, goggles, recording, features }), or null when the app
// is served without it (the Python fallback). Cached; { fresh: true } asks again (the server restarted).
let session = null;
export async function serverSession({ fresh = false } = {}) {
  if (fresh || !session) session = fetch("/api/session", { cache: "no-store" }).then((r) => (r.ok ? r.json() : null)).catch(() => null);
  const s = await session;
  if (!s) session = null;
  return s;
}

// POST JSON to the server with the session token (recorder: /rec/start, /rec/stop, /rec/telemetry,
// /rec/command). Retries once with a fresh token after a 403; throws with .status on errors.
export async function serverPost(path, body = {}) {
  for (const fresh of [false, true]) {
    const s = await serverSession({ fresh });
    if (!s) throw Object.assign(new Error("Whoop Pilot's server isn't running: double-click start.command (needs Node.js)."), { status: 0 });
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json", "X-Whoop-Token": s.token }, body: JSON.stringify(body) });
    if (r.status === 403 && !fresh) continue;
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw Object.assign(new Error(j.error || `${path}: HTTP ${r.status}`), { status: r.status, body: j });
    return j;
  }
}

const POLL_MS = 2500;
const DIRECT_KEY = "whoopPilot.gogglesDirectUsb"; // set once direct WebUSB got past opening the device
// Statuses while the worker has the chain running. The others: off, error, no-helper.
const RUNNING = new Set(["connecting", "no-goggles", "handshake", "no-answer", "live", "no-video", "lost"]);

export class GogglesSource extends Emitter {
  constructor() {
    super();
    this.kind = "goggles";
    this.label = "DJI Goggles 3";
    this.canvas = document.createElement("canvas");
    this.canvas.width = 16;
    this.canvas.height = 9;
    this.ctx = this.canvas.getContext("2d");
    this.frames = 0; // painted since the page loaded: frameId()
    this.liveFrames = 0; // painted on this connection
    this.lastPaint = 0;
    this.decoded = null; // performance.now() when the painted frame came out of the decoder
    this.crop = null; // the user's crop, normalized {x, y, w, h}
    this.picture = null; // the camera's picture perception found in the stream (element px), see region()
    this.status = "off";
    this.via = null; // "helper" | "usb"
    this.decoderState = "waiting";
    this.stats = {};
    this.lastError = null;
    this.watching = false;
    if ("usb" in navigator) {
      navigator.usb.addEventListener("disconnect", (e) => {
        if (this.device && e.device === this.device) this.fail("The goggles were unplugged.");
      });
    }
  }

  static get webUsbSupported() {
    return "usb" in navigator;
  }

  // Direct WebUSB reached the goggles once in this browser, so it's worth retrying without asking.
  get directUsbWorked() {
    try {
      return localStorage.getItem(DIRECT_KEY) === "1";
    } catch {
      return false;
    }
  }

  get active() {
    return RUNNING.has(this.status);
  }

  get transport() {
    return this.via === "usb" ? "direct USB" : this.stats.transport === "wifi" ? "Wi-Fi" : "USB";
  }

  setStatus(s) {
    this.status = s;
    this.emit("status", s);
  }

  onEvent(e) {
    if (e.type === "state") {
      this.decoderState = e.decoder;
      this.stats = e.stats || {};
      if (e.state === "off" || !this.active) return; // we set off/error/no-helper ourselves; ignore stale reports
      if (this.via === "usb" && e.state !== "connecting") {
        try {
          localStorage.setItem(DIRECT_KEY, "1");
        } catch {}
      }
      if (e.state !== this.status) this.setStatus(e.state);
      else this.emit("status", e.state);
    } else if (e.type === "log") this.emit("log", e.msg);
    else if (e.type === "error") this.onError(e.msg);
  }

  onError(msg) {
    if (this.via === "helper") {
      // The helper went away: keep watching for it instead of sitting on an error.
      this.stop();
      this.setStatus("no-helper");
      this.emit("error", "Lost the goggles video from Whoop Pilot's server (its start.command window closed?). Start it again; the app reconnects by itself.");
      return;
    }
    if (/protected|won't open/i.test(msg)) {
      msg = "Chrome can't claim this USB interface: it's class 0xE0, which WebUSB blocks for every web page (expected on Goggles 3). Use the helper in Whoop Pilot's server: double-click start.command.";
      try {
        localStorage.removeItem(DIRECT_KEY);
      } catch {}
    }
    this.stop();
    this.lastError = msg;
    this.status = "error";
    this.emit("error", msg);
    this.emit("status", this.status);
  }

  // decodedAt: when the decoder put the frame out (this page's clock; the worker sends its absolute time).
  paint(frame, decodedAt = performance.now()) {
    const w = frame.displayWidth;
    const h = frame.displayHeight;
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    this.ctx.drawImage(frame, 0, 0, w, h);
    frame.close();
    this.frames++;
    this.liveFrames++;
    this.lastPaint = performance.now();
    this.decoded = Math.min(this.lastPaint, Math.max(this.decoded ?? -Infinity, decodedAt));
  }

  worker() {
    if (!this.w) {
      this.w = new Worker(new URL("./worker.js", import.meta.url), { type: "module" });
      this.w.onmessage = ({ data }) => (data.type === "frame" ? this.paint(data.frame, data.at != null ? data.at - performance.timeOrigin : undefined) : this.onEvent(data));
    }
    return this.w;
  }

  // The goggles helper's status JSON, or null when the server doesn't answer (or has no helper).
  async helperInfo() {
    try {
      const r = await fetch(`${HELPER_URL}/status`, { cache: "no-store", signal: AbortSignal.timeout(1500) });
      return r.ok ? await r.json() : null;
    } catch {
      return null;
    }
  }

  // The normal way in: the server does USB or Wi-Fi outside Chrome and streams H.264 to the worker.
  // Resolves false (status "no-helper") when the server or its goggles helper isn't there.
  async connectHelper() {
    this.stop();
    this.via = "helper";
    this.lastError = null;
    this.liveFrames = 0;
    this.setStatus("connecting");
    const info = await this.helperInfo();
    const s = info && (await serverSession({ fresh: true })); // a fresh token: the server may have restarted since
    if (!s) {
      this.setStatus("no-helper");
      return false;
    }
    this.stats = { ...(info.stats || {}), transport: info.transport };
    this.worker().postMessage({ cmd: "start-helper", url: `${location.origin}${HELPER_URL}/${s.token}` }); // the worker fetches <url>/stream
    return true;
  }

  // While the goggles UI is showing, look for the helper every few seconds and connect as soon as
  // it's there, so starting the server (start.command) is all the user has to do.
  watchHelper(on) {
    if (on === this.watching) return;
    this.watching = on;
    clearInterval(this.pollTimer);
    this.pollTimer = on ? setInterval(() => this.poll(), POLL_MS) : null;
    if (on) this.poll();
  }

  async poll() {
    if (this.active || this.polling) return;
    this.polling = true;
    const info = await this.helperInfo();
    this.polling = false;
    if (!this.watching || this.active) return;
    if (!info) {
      if (this.status === "off") this.setStatus("no-helper");
      return;
    }
    if (await this.connectHelper()) this.emit("connected");
  }

  // Asks Chrome for the goggles directly (needs a click the first time). Works only when the
  // goggles' network interface has a class Chrome allows; Goggles 3 get a SecurityError.
  async connectUsb({ prompt = true } = {}) {
    const dev = prompt
      ? await navigator.usb.requestDevice({ filters: [{ vendorId: DJI_VENDOR_ID }] })
      : (await navigator.usb.getDevices()).find((d) => d.vendorId === DJI_VENDOR_ID);
    if (!dev) return false;
    this.stop();
    this.via = "usb";
    this.device = dev;
    this.lastError = null;
    this.liveFrames = 0;
    this.setStatus("connecting");
    this.worker().postMessage({ cmd: "start-usb", filter: { vendorId: dev.vendorId, productId: dev.productId, serialNumber: dev.serialNumber } });
    return true;
  }

  // Runs the chain on the page with any WebUSB-like device (used by tests with a fake goggles).
  async startInPage(device) {
    this.stop();
    this.via = "usb";
    this.liveFrames = 0;
    this.pipeline = new GogglesPipeline({ onFrame: (f) => this.paint(f), onEvent: (e) => this.onEvent(e) });
    this.reportTimer = setInterval(() => this.pipeline?.report(), 500);
    this.status = "connecting";
    await this.pipeline.startUsb(device);
  }

  fail(msg) {
    this.stop();
    this.onEvent({ type: "error", msg });
  }

  stop() {
    this.w?.postMessage({ cmd: "stop" });
    clearInterval(this.reportTimer);
    this.pipeline?.stop();
    this.pipeline = null;
    this.device = null;
    if (this.status !== "error") this.status = "off";
  }

  // One-line status for the UI.
  describe() {
    const s = this.stats;
    const p = s.player || {};
    switch (this.status) {
      case "off": return "Not connected";
      case "no-helper": return "No goggles helper: Whoop Pilot's server isn't answering, or this copy of the app runs without Node.js (simulator only). Install Node.js (brew install node), double-click start.command and keep its window open; the app connects by itself.";
      case "connecting": return this.via === "usb" ? "Opening the goggles' USB link…" : "Connecting to the helper…";
      case "no-goggles":
        if (s.error) return s.fatal ? `Goggles helper can't start: ${s.error}` : `Helper found the goggles but can't open them: ${s.error}`;
        return s.transport === "wifi"
          ? "Helper running, waiting for the goggles on Wi-Fi: join their network (Share Liveview to Mobile Device via Wi-Fi on)."
          : "Helper running, no goggles on USB: plug them in with a USB-C data cable (goggles on, Settings → About → OTG Wired Connection off).";
      case "handshake": return `Link up over ${this.transport}, calling the goggles…`;
      case "no-answer": return s.transport === "wifi"
        ? "The goggles don't answer. Is the Mac on the goggles' Wi-Fi, Share Liveview to Mobile Device via Wi-Fi on, the drone powered, the goggles on your head (they sleep when taken off) and out of Real View and the album?"
        : "The goggles don't answer. Is the drone powered, are the goggles on your head (they sleep when taken off) and out of Real View and the album, with Share Liveview to Mobile Device via Wi-Fi on (it also enables USB) and OTG Wired Connection off?";
      case "no-video": return "Connected, but no picture: power the drone and check the goggles show its camera.";
      case "lost": return "Goggles went quiet, reconnecting…";
      case "error": return this.lastError || "Error";
      default:
        if (!s.units) return `Goggles answered over ${this.transport}, waiting for video…`;
        if (p.start === "failed") return `The decoder took none of the start pictures${p.lastError ? ` (${p.lastError})` : ""}. Click Connect goggles to try again, or replug the drone battery.`;
        if (this.decoderState !== "running" || !this.liveFrames) {
          return p.start
            ? `Starting the decoder (${p.variant ? `bootstrap ${p.variant}${p.start === "bootstrap-software" ? ", software decoder" : ""}` : p.start})…`
            : `Receiving video over ${this.transport}, waiting for the goggles' stream parameters…`;
        }
        return `Live over ${this.transport}: ${this.liveFrames} frames${p.errors ? `, ${p.errors} decoder restarts` : ""}${s.droppedUnits ? `, ${s.droppedUnits} dropped` : ""}${s.lostPackets ? `, ${s.lostPackets} packets lost` : ""}`;
    }
  }

  // Video source interface used by Perception and the HUD.
  ready() {
    return this.active && performance.now() - this.lastPaint < 1000;
  }
  element() {
    return this.canvas;
  }
  // The user's crop, or the whole stream (the Goggles 3: 1920x1080 with the O4's 4:3 picture pillarboxed in it).
  baseRegion() {
    const w = this.canvas.width;
    const h = this.canvas.height;
    const c = this.crop;
    if (!c || c.w < 0.02 || c.h < 0.02) return { sx: 0, sy: 0, sw: w, sh: h };
    return { sx: Math.round(c.x * w), sy: Math.round(c.y * h), sw: Math.round(c.w * w), sh: Math.round(c.h * h) };
  }
  // The camera picture Perception found inside baseRegion() (perception.js PictureFrame), or null.
  setPicture(rect) {
    this.picture = rect;
  }
  // The drone's video: the picture when it lies inside baseRegion(), else baseRegion(). Flow, the detector, depth,
  // vision localization, the HUD and Claude's snapshots all use this.
  region() {
    const b = this.baseRegion(), p = this.picture;
    return p && p.sx >= b.sx && p.sy >= b.sy && p.sx + p.sw <= b.sx + b.sw && p.sy + p.sh <= b.sy + b.sh ? { sx: p.sx, sy: p.sy, sw: p.sw, sh: p.sh } : b;
  }
  // A crop drawn on the shown video (normalised in region(): the HUD draws the picture once it is found) -> the crop to
  // store (normalised in the whole stream, as baseRegion() reads it).
  cropFromView(c) {
    const r = this.region(), w = this.canvas.width, h = this.canvas.height;
    return c && w && h ? { x: (r.sx + c.x * r.sw) / w, y: (r.sy + c.y * r.sh) / h, w: (c.w * r.sw) / w, h: (c.h * r.sh) / h } : null;
  }
  frameId() {
    return this.frames;
  }
  // When the frame now on the canvas was decoded (performance.now() ms), or null before the first one.
  decodedAt() {
    return this.decoded;
  }
}
