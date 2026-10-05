// DJI Goggles 3 live view under Node, for Whoop Pilot's server (whoop.mjs) and the goggles.mjs CLI.
//
// Chrome can't claim the goggles' USB network interface itself (the RNDIS control interface is USB
// class 0xE0, which WebUSB refuses for every web page), so the app's own code (app/js/goggles/*) runs
// here: over USB (node-usb + our RNDIS/IP stack) or over the goggles' Wi-Fi hotspot (a plain UDP
// socket). GogglesHelper keeps a LiveView session up, reconnects, and fans the H.264 out to stream
// clients ([type u8][length u32le][payload]; 1 = Annex-B access unit, 2 = JSON status) and to "unit"
// listeners (the flight recorder).
import os from "node:os";
import dgram from "node:dgram";
import { EventEmitter } from "node:events";
import { RndisLink, describeDevice } from "../app/js/goggles/rndis.js";
import { NetStack } from "../app/js/goggles/netstack.js";
import { LiveviewSession, HOST_IP, PORT } from "../app/js/goggles/liveview.js";

export { PORT };
export const DJI = 0x2ca3;
const BACKLOG = 8 << 20; // bytes queued for a stream client before its units are dropped (a stalled tab must not eat the Mac's memory)
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const hex4 = (n) => n.toString(16).padStart(4, "0");
const ipOf = (s) => s.split(".").map(Number);

export const frame = (type, payload) => {
  const b = Buffer.alloc(5 + payload.length);
  b[0] = type;
  b.writeUInt32LE(payload.length, 1);
  Buffer.from(payload.buffer, payload.byteOffset, payload.byteLength).copy(b, 5);
  return b;
};

export const explain = (state) =>
  ({
    handshake: "Still waiting for the goggles to answer the handshake.",
    "no-answer": "The goggles don't answer the handshake. Drone powered and linked? Goggles on your head (their screen and the shared view stop when they're taken off)? Real View or the album open (both pause sharing)? Settings right (Share Liveview to Mobile Device via Wi-Fi on for both; USB: OTG Wired Connection off; the Mac on their network)?",
    "no-video": "The goggles answer but send no video: power the drone and check the goggles show its camera.",
    live: "Video is flowing.",
    lost: "The goggles went quiet.",
  })[state] || state;

// On Wi-Fi the Mac is on the goggles' network, so the OS does Ethernet/IP and a socket is all we
// need. The session addresses the goggles by their USB address; datagrams go to `peer` instead, and
// the goggles answer whatever port we send from (the reference clients even use an ephemeral one).
class UdpLink {
  constructor(peer, { localPort = PORT, log = () => {} } = {}) {
    Object.assign(this, { peer, localPort, log, sock: null, handler: null, sendErrors: 0 });
  }
  open() {
    return new Promise((resolve, reject) => {
      const sock = dgram.createSocket({ type: "udp4", recvBufferSize: 4 << 20 });
      sock.on("message", (m, r) => this.handler?.(ipOf(r.address), r.port, new Uint8Array(m.buffer, m.byteOffset, m.length)));
      sock.on("error", (e) => {
        if (this.sock) return this.log(`UDP socket error: ${e.message}`);
        sock.close();
        if (e.code === "EADDRINUSE" && this.localPort) {
          this.log(`UDP port ${this.localPort} is taken; using any free port.`);
          this.localPort = 0;
          resolve(this.open());
        } else reject(e);
      });
      sock.bind(this.localPort, "0.0.0.0", () => {
        this.sock = sock;
        this.localPort = sock.address().port;
        resolve(this);
      });
    });
  }
  bindUdp(port, handler) {
    this.handler = handler;
  }
  sendUdp(dstIp, srcPort, dstPort, payload) {
    this.sock?.send(payload, this.peer.port, this.peer.host, (e) => {
      if (e && !this.sendErrors++) this.log(`Can't send to ${this.peer.host}: ${e.code || e.message}. Is the Mac on the goggles' Wi-Fi?`);
    });
  }
  close() {
    this.sock?.close();
    this.sock = null;
  }
}

const localAddressNear = (host) => {
  const net = host.split(".").slice(0, 3).join(".") + ".";
  for (const list of Object.values(os.networkInterfaces())) for (const a of list) if (a.family === "IPv4" && !a.internal && a.address.startsWith(net)) return a.address;
  return null;
};

let webusb = null;
async function usbDevices() {
  if (!webusb) {
    let WebUSB;
    try {
      ({ WebUSB } = await import("usb"));
    } catch (e) {
      throw Object.assign(new Error(`The "usb" package isn't installed or didn't build (${e.message}). Run: cd tools && npm install`), { fatal: true });
    }
    webusb = new WebUSB({ allowAllDevices: true });
  }
  return webusb.getDevices();
}

// How to reach the goggles. devices() lists WebUSB-like devices (node-usb by default; tests pass a fake).
export class Goggles {
  constructor({ wifi = false, peer = null, localPort = PORT, devices = usbDevices, log = () => {} } = {}) {
    const [host, port] = (peer || (wifi ? "192.168.2.1" : "192.168.60.2")).split(":");
    Object.assign(this, { wifi, redirect: !!peer, peer: { host, port: Number(port) || PORT }, localPort, devices, log, lastFound: null });
    this.transport = wifi ? "wifi" : "usb";
  }

  async openWifi() {
    const { peer, log } = this;
    const local = localAddressNear(peer.host);
    if (!local && !peer.host.startsWith("127.")) log(`No interface has an address near ${peer.host}. Join the goggles' Wi-Fi first (Share Liveview to Mobile Device via Wi-Fi on, then the Mac's Wi-Fi menu). Trying anyway…`);
    const link = await new UdpLink(peer, { localPort: this.localPort, log }).open();
    log(`UDP ${local || "0.0.0.0"}:${link.localPort} -> ${peer.host}:${peer.port}`);
    return { udp: link, close: () => link.close(), label: `Wi-Fi, goggles at ${peer.host}` };
  }

  async openUsb({ waitMs, onDown, signal }) {
    const { log } = this;
    const t0 = Date.now();
    let dev;
    let told = false;
    while (!(dev = (await this.devices()).find((d) => d.vendorId === DJI))) {
      if (!told) log("No DJI goggles on USB. Plug them into the Mac with a USB-C data cable (goggles on, OTG Wired Connection off)…");
      told = true;
      if (Date.now() - t0 > waitMs) throw new Error("No DJI goggles on USB.");
      if (signal?.aborted) throw new Error("Stopped.");
      await sleep(500);
    }
    const found = describeDevice(dev);
    if (found !== this.lastFound) log("Found:\n" + found);
    this.lastFound = found;
    const link = new RndisLink(dev, { log });
    const { mac } = await link.open();
    const net = new NetStack({ mac, ip: HOST_IP, sendFrame: (f) => link.sendFrame(f), log });
    link.start((f) => net.handleFrame(f), onDown);
    net.announce();
    const close = () => {
      try {
        link.close()?.catch?.(() => {});
      } catch {}
    };
    // LiveviewSession addresses the goggles by their USB address; --peer redirects that.
    const udp = this.redirect ? { bindUdp: (p, h) => net.bindUdp(p, h), sendUdp: (ip, sp, dp, payload) => net.sendUdp(ipOf(this.peer.host), sp, this.peer.port, payload) } : net;
    return { udp, close, label: `${dev.productName || "DJI"} ${hex4(dev.vendorId)}:${hex4(dev.productId)} on USB` };
  }

  // Opens the transport and starts a LiveView session on it. The goggles may still be silent after
  // this: watch session.state. onDown fires when the USB link dies.
  async connect(onUnit, { onDown = () => {}, waitMs = 60000, signal } = {}) {
    const t = this.wifi ? await this.openWifi() : await this.openUsb({ waitMs, onDown, signal });
    const session = new LiveviewSession(t.udp);
    session.on("state", (s) => this.log("session:", s));
    session.on("log", (m) => this.log(m));
    session.on("unit", onUnit);
    session.start();
    return {
      session,
      label: t.label,
      close: () => {
        session.stop();
        t.close();
      },
    };
  }
}

// Keeps the goggles connected for as long as it runs. Emits "unit" (Uint8Array access unit) and "status".
export class GogglesHelper extends EventEmitter {
  constructor(opts = {}) {
    super();
    this.goggles = new Goggles(opts);
    this.log = opts.log || (() => {});
    this.clients = new Set();
    this.session = this.device = this.conn = this.lastError = null;
  }

  get transport() {
    return this.goggles.transport;
  }

  status() {
    const { session, transport, lastError: err } = this;
    const error = !session && err && (err.fatal || Date.now() - err.at < 6000) ? err.msg : null;
    return {
      app: "whoop-pilot-goggles",
      state: session ? session.state : "no-goggles",
      stats: { ...(session ? session.stats : {}), transport, error, fatal: !!err?.fatal },
      transport,
      peer: `${this.goggles.peer.host}:${this.goggles.peer.port}`,
      device: this.device,
      clients: this.clients.size,
    };
  }

  push() {
    const f = frame(2, Buffer.from(JSON.stringify(this.status())));
    for (const c of this.clients) c.write(f);
    this.emit("status");
  }

  unit(u) {
    this.emit("unit", u);
    if (!this.clients.size) return;
    const f = frame(1, u);
    for (const c of this.clients) if (c.writableLength < BACKLOG) c.write(f);
  }

  // A stream client: status first, then units as they arrive.
  attach(req, res) {
    res.writeHead(200, { "Content-Type": "application/octet-stream" });
    this.clients.add(res);
    res.write(frame(2, Buffer.from(JSON.stringify(this.status()))));
    req.on("close", () => this.clients.delete(res));
  }

  start() {
    if (this.abort) return this;
    this.abort = new AbortController();
    this.abort.signal.addEventListener("abort", () => this.down?.(new Error("stopped")));
    this.timer = setInterval(() => this.push(), 2000);
    this.loop(this.abort.signal);
    return this;
  }

  async loop(signal) {
    while (!signal.aborted) {
      const gone = new Promise((r) => (this.down = r));
      let conn;
      try {
        conn = await this.goggles.connect((u) => this.unit(u), { onDown: this.down, waitMs: Infinity, signal });
      } catch (e) {
        if (signal.aborted) break;
        this.log(e.message);
        this.lastError = { msg: e.message.split("\n")[0], at: Date.now(), fatal: !!e.fatal };
        this.push();
        if (e.fatal) break;
        await sleep(2000);
        continue;
      }
      if (signal.aborted) {
        conn.close();
        break;
      }
      this.lastError = null;
      this.conn = conn;
      this.session = conn.session;
      this.device = conn.label;
      this.session.on("state", () => this.push());
      this.push();
      const e = await gone;
      if (!signal.aborted) this.log(`Goggles link lost (${e.message}); waiting for them to come back.`);
      conn.close();
      this.session = this.device = this.conn = null;
      this.push();
      await sleep(500);
    }
  }

  stop() {
    this.abort?.abort();
    clearInterval(this.timer);
    this.conn?.close();
    for (const c of this.clients) c.end();
    this.clients.clear();
  }
}
