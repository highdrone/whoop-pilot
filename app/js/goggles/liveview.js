// DJI Goggles 3 "LiveView Sharing" session over UDP: handshake, acknowledgements with loss recovery,
// and reassembly of the video into H.264 Annex-B access units.
//
// Protocol as documented by dji-goggles-lab (https://github.com/soldnenz/dji-goggles-lab,
// docs/02-ip-liveview.md; verified on Goggles 3 and matching the SquirrelReceiver client) and orbit-rs
// (MIT, https://github.com/planktonwhc/orbit-rs), checked against a capture from our own goggles. This
// is an independent implementation.
//   header, both directions: u16 length | 0x8000, u16 session, u16 seq, u8 type, u8 XOR of bytes 0-6
//   types: 0 handshake, 1 the goggles' windows + DUML (~10 Hz), 2 video (seq steps by 8), 4 our ACK
//   handshake: a 48-byte template; its u16 at offset 8 seeds every window. The goggles answer with a
//     9-byte type 0 (or, if that is lost, their first type 1/2 for our session proves the link).
//   type 1: [8] the video seq the goggles believe we've received through (our last ACK's start),
//     [10] the last video seq they sent, [16]/[24] the type-3/5 windows, [32] DUML length, DUML from 34
//     (now and then one from the goggles carrying their serial number). Nothing in it is echoed.
//   video: [8]/[10] the same two windows, [16] frame id, [17] part count (bits 0-6) and part index
//     bit 0 (bit 7), [18] part index bits 1-5, H.264 from byte 20
//   ACK: u16 video received-through, u16 highest video received, u16 n, n missing seqs (they must start
//     at the first hole or the goggles ignore the list), then the type-3 and type-5 windows as
//     (start, end, 0), u16 0, u16 DUML length. The goggles stop sending video once about 128 KiB
//     (~94 full packets) is outstanding past the received-through seq, so the ACKs must keep it moving.
import { Emitter } from "../util.js";

export const GOGGLES_IP = [192, 168, 60, 2];
export const HOST_IP = [192, 168, 60, 1];
export const PORT = 9003;
export const TYPE = { HANDSHAKE: 0, DATA: 1, VIDEO: 2, ACK: 4 };
const STEP = 8;
const MAX_RESEND = 16;

// Handshake as sent by the SquirrelReceiver client (documented in dji-goggles-lab); session and seed
// are filled in.
const HANDSHAKE = [
  0x30, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x64, 0x00, 0xc0, 0x05, 0x14, 0x00, 0x00, 0x0a,
  0x00, 0x64, 0x00, 0x64, 0x00, 0xc0, 0x05, 0x14, 0x00, 0x00, 0x64, 0x00, 0x14, 0x00, 0x64, 0x00, 0xc0, 0x05, 0x14, 0x00,
  0x00, 0x64, 0x00, 0x01, 0x01, 0x04, 0x0a, 0x02,
];

const xor7 = (p) => p[0] ^ p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5] ^ p[6];
const u16 = (p, o) => p[o] | (p[o + 1] << 8);
const put16 = (p, o, v) => {
  p[o] = v & 255;
  p[o + 1] = (v >> 8) & 255;
};
// How far seq a is ahead of seq b on the 16-bit ring (negative if behind).
export const ahead = (a, b) => {
  const d = (a - b) & 0xffff;
  return d < 0x8000 ? d : d - 0x10000;
};

function seal(p) {
  put16(p, 0, p.length | 0x8000);
  p[7] = xor7(p);
  return p;
}

export function buildHandshake(session, seed) {
  const p = Uint8Array.from(HANDSHAKE);
  put16(p, 2, session);
  put16(p, 8, seed & 0xfff8);
  return seal(p);
}

export function buildAck(session, { start, end, resend = [], t3 = [start, start], t5 = [start, start], duml = null }) {
  const n = resend.length;
  const p = new Uint8Array(8 + 6 + 2 * n + 16 + (duml ? duml.length : 0));
  put16(p, 2, session);
  p[6] = TYPE.ACK;
  put16(p, 8, start);
  put16(p, 10, end);
  put16(p, 12, n);
  let o = 14;
  for (const s of resend) put16(p, (o += 2) - 2, s);
  put16(p, o, t3[0]);
  put16(p, o + 2, t3[1]);
  put16(p, o + 6, t5[0]);
  put16(p, o + 8, t5[1]);
  if (duml) {
    put16(p, o + 14, duml.length);
    p.set(duml, o + 16);
  }
  return seal(p);
}

// Type byte of a well-formed packet for this session, else -1.
export function packetType(p, session) {
  if (p.length < 8 || u16(p, 2) !== session) return -1;
  if ((u16(p, 0) & 0x7fff) !== p.length || xor7(p) !== p[7]) return -1;
  return p[6];
}

export function parseVideo(p) {
  if (p.length < 20) return null;
  return { seq: u16(p, 4), frame: p[16], total: p[17] & 0x7f, index: (p[17] >> 7) | ((p[18] & 0x1f) << 1), payload: p.subarray(20) };
}

// The goggles' type-1 report. Short ones (the goggles send 34 bytes without DUML) parse with zeros.
export function parseData(p) {
  if (p.length < 12) return null;
  const at = (o) => (p.length >= o + 2 ? u16(p, o) : 0);
  return { start: u16(p, 8), end: u16(p, 10), t3: [at(16), at(18)], t5: [at(24), at(26)], duml: p.subarray(34, 34 + at(32)) };
}

// The goggles' serial number if a DUML blob carries it: the only 8+ run of letters and digits.
export function serialIn(duml) {
  const m = /[0-9A-Za-z]{8,}/.exec(String.fromCharCode(...duml.subarray(0, 512)));
  return m ? m[0] : "";
}

// What we've received of the video sequence: `start` is the last seq with everything up to it
// received, `end` the highest seen. `missing()` lists the holes in between, first hole first.
export class ReceiveWindow {
  constructor(seed) {
    this.seed = seed;
    this.reset();
  }
  reset() {
    this.armed = false;
    this.start = this.end = this.seed;
    this.have = new Set();
    this.holes = new Map(); // seq -> when we first knew it was missing
  }
  // Returns false for a duplicate or a packet from before the window.
  push(seq, now = 0) {
    if (!this.armed) {
      this.armed = true;
      this.start = this.end = seq;
      return true;
    }
    const d = ahead(seq, this.start);
    if (d <= 0 || this.have.has(seq)) return false;
    if (d > 0x4000) {
      // far ahead: the goggles moved on without us, start over from here
      this.have.clear();
      this.holes.clear();
      this.start = this.end = seq;
      return true;
    }
    if (ahead(seq, this.end) > 0) {
      for (let s = (this.end + STEP) & 0xffff; s !== seq; s = (s + STEP) & 0xffff) this.holes.set(s, now);
      this.end = seq;
    } else this.holes.delete(seq);
    this.have.add(seq);
    this.advance();
    return true;
  }
  advance() {
    let next = (this.start + STEP) & 0xffff;
    while (this.have.delete(next)) {
      this.start = next;
      next = (next + STEP) & 0xffff;
    }
  }
  missing(max = MAX_RESEND) {
    const out = [];
    for (let s = (this.start + STEP) & 0xffff, i = 0; ahead(s, this.end) < 0 && out.length < max && i < 512; s = (s + STEP) & 0xffff, i++) {
      if (!this.have.has(s)) out.push(s);
    }
    return out;
  }
  // Treat the holes at the front of the window that have stayed open longer than maxAge as lost for
  // good, so the window (and the goggles' send budget) can move on. Returns how many.
  expire(now, maxAge) {
    let n = 0;
    while (this.start !== this.end) {
      const hole = (this.start + STEP) & 0xffff;
      if (now - this.holes.get(hole) <= maxAge) break;
      this.holes.delete(hole);
      this.start = hole;
      this.advance();
      n++;
    }
    return n;
  }
}

// Collects video parts into access units and hands them out in frame order. A frame that is missing
// parts is held for a moment in case they're resent, then skipped.
export class FrameAssembler {
  constructor({ holdMs = 80 } = {}) {
    this.holdMs = holdMs;
    this.frames = new Map();
    this.next = -1;
    this.dropped = 0;
  }
  push(v, now) {
    if (!v.total || v.index >= v.total) return [];
    if (this.next < 0) this.next = v.frame;
    const d = (v.frame - this.next) & 255;
    if (d >= 128) {
      // Behind us: a late resend of a frame already handed out or skipped, unless it's further back
      // than the goggles' send window can reach, which means the stream restarted its numbering.
      if (d >= 256 - 32) return [];
      this.reset();
      this.next = v.frame;
    }
    let f = this.frames.get(v.frame);
    if (!f || f.total !== v.total) {
      f = { parts: new Array(v.total).fill(null), total: v.total, have: 0, firstAt: now };
      this.frames.set(v.frame, f);
    }
    if (!f.parts[v.index]) {
      f.parts[v.index] = v.payload.slice();
      f.have++;
    }
    return this.drain(now);
  }
  drain(now) {
    const out = [];
    while (this.frames.size) {
      const f = this.frames.get(this.next);
      if (f && f.have === f.total) {
        out.push(join(f.parts));
        this.frames.delete(this.next);
        this.next = (this.next + 1) & 255;
        continue;
      }
      // The frame we need is incomplete or missing. Skip it once a later frame exists and we've
      // waited long enough (or too much is piling up).
      let since = Infinity;
      for (const [id, g] of this.frames) if (id !== this.next) since = Math.min(since, g.firstAt);
      if (since === Infinity) break;
      if (now - (f ? f.firstAt : since) < this.holdMs && this.frames.size < 32) break;
      if (f) this.frames.delete(this.next);
      this.dropped++;
      this.next = (this.next + 1) & 255;
    }
    return out;
  }
  reset() {
    this.frames.clear();
    this.next = -1;
  }
}

function join(parts) {
  let n = 0;
  for (const p of parts) n += p.length;
  const out = new Uint8Array(n);
  let o = 0;
  for (const p of parts) {
    out.set(p, o);
    o += p.length;
  }
  return out;
}

// Emits: "state" ("handshake" | "no-answer" | "live" | "no-video" | "lost"), "unit" (Uint8Array
// Annex-B access unit), "log".
// giveUpMs: how long a hole may stay open before we move the window past it. The goggles stop sending
// once ~128 KiB is outstanding past our received-through seq (~50 ms at full bitrate), so a hole they
// never refill must not hold the window for long. Our resend request leaves within 10 ms of noticing
// the hole and a resend takes a few ms over USB; one that arrives after we gave up still reaches the
// assembler, which holds the frame for holdMs.
export class LiveviewSession extends Emitter {
  constructor(net, { lostAfter = 3000, noVideoAfter = 2500, ackEvery = 33, holdMs = 80, giveUpMs = 80 } = {}) {
    super();
    this.net = net;
    this.lostAfter = lostAfter;
    this.noVideoAfter = noVideoAfter;
    this.ackEvery = ackEvery;
    this.holdMs = holdMs;
    this.giveUpMs = giveUpMs;
    this.session = 0;
    this.state = "idle";
    this.timer = null;
    // outstanding: video packets the goggles report sent that we haven't received (in flight or lost);
    // theirWindow: the goggles' own report (received-through per our ACKs, last sent)
    this.stats = { acks: 0, videoPackets: 0, duplicates: 0, resendRequests: 0, lostPackets: 0, units: 0, bytes: 0, droppedUnits: 0, stalls: 0, outstanding: 0, theirWindow: [0, 0], serial: "" };
    net.bindUdp(PORT, (src, srcPort, data) => this.onDatagram(data));
  }

  start() {
    this.stop();
    const r = new Uint16Array(2);
    crypto.getRandomValues(r);
    this.session = r[0] | 1; // odd, as the reference client's
    this.seed = (r[1] & 0xfff8) || 0x1000;
    this.window = new ReceiveWindow(this.seed);
    this.assembler = new FrameAssembler({ holdMs: this.holdMs });
    this.handshake = buildHandshake(this.session, this.seed);
    this.lastAny = this.lastData = this.lastVideo = this.lastAck = this.lastHandshake = 0;
    this.startedAt = Date.now();
    this.setState("handshake");
    this.timer = setInterval(() => this.tick(), Math.min(this.ackEvery, 50));
    this.tick();
  }

  stop() {
    clearInterval(this.timer);
    this.timer = null;
  }

  setState(s) {
    if (s === this.state) return;
    this.state = s;
    this.emit("state", s);
  }

  send(p) {
    this.net.sendUdp(GOGGLES_IP, PORT, PORT, p);
  }

  // Before any video (and again after a stall) the window sits on the seed, as the reference client's
  // ACKs do; the type-3/5 windows always do.
  sendAck(resend = []) {
    const w = this.window;
    this.send(buildAck(this.session, { start: w.start, end: w.end, resend, t3: [this.seed, this.seed], t5: [this.seed, this.seed] }));
    this.stats.acks++;
    if (resend.length) this.stats.resendRequests++;
    this.lastAck = Date.now();
  }

  linked() {
    return this.state !== "handshake" && this.state !== "no-answer";
  }

  tick() {
    const now = Date.now();
    if (!this.linked()) {
      if (now - this.lastHandshake >= 250) {
        this.send(this.handshake);
        this.lastHandshake = now;
      }
      if (now - this.startedAt > 8000) this.setState("no-answer");
      return;
    }
    if (now - this.lastAny > this.lostAfter) {
      this.setState("lost");
      this.emit("log", "Goggles went quiet; reconnecting.");
      this.start();
      return;
    }
    if (this.window.armed && now - this.lastVideo > 1000) {
      // Video stopped (drone off or out of range). Start the window over when it comes back.
      this.window.reset();
      this.assembler.reset();
      this.stats.stalls++;
      this.emit("log", "Video stalled.");
    }
    if (this.state === "live" && now - (this.lastVideo || this.linkedAt) > this.noVideoAfter) this.setState("no-video");
    this.stats.lostPackets += this.window.expire(now, this.giveUpMs);
    for (const u of this.assembler.drain(now)) this.deliver(u);
    this.stats.droppedUnits = this.assembler.dropped;
    if (now - this.lastAck >= this.ackEvery) this.sendAck(this.window.missing());
  }

  onDatagram(p) {
    if (!this.timer) return;
    const type = packetType(p, this.session);
    if (type < 0) return;
    const now = Date.now();
    this.lastAny = now;
    if (!this.linked()) {
      // Their 9-byte answer, or (should it get lost) their first report or video for our session.
      if (type !== TYPE.HANDSHAKE && type !== TYPE.DATA && type !== TYPE.VIDEO) return;
      this.emit("log", `Goggles answered (session 0x${this.session.toString(16)}).`);
      this.linkedAt = now;
      this.setState("live");
      if (type === TYPE.HANDSHAKE) {
        this.sendAck();
        return;
      }
    }
    if (type === TYPE.DATA) {
      this.lastData = now;
      const d = parseData(p);
      if (d) {
        this.stats.theirWindow = [d.start, d.end];
        this.stats.outstanding = this.window.armed ? Math.max(0, ahead(d.end, this.window.end) >> 3) : 0;
        if (d.duml.length) this.stats.serial = serialIn(d.duml) || this.stats.serial;
      }
      this.sendAck(this.window.missing());
      return;
    }
    if (type !== TYPE.VIDEO) return;
    const v = parseVideo(p);
    if (!v) return;
    this.stats.videoPackets++;
    const w = this.window;
    const filled = w.armed && ahead(v.seq, w.end) < 0; // inside the window: a resend
    const fresh = w.push(v.seq, now);
    if (fresh) this.lastVideo = now;
    else this.stats.duplicates++;
    // A packet the window has already given up on may still complete a frame the assembler holds.
    const units = this.assembler.push(v, now);
    this.stats.droppedUnits = this.assembler.dropped;
    this.stats.lostPackets += w.expire(now, this.giveUpMs);
    // ACK at the end of each frame, and quickly (but not for every packet) while something is missing
    // or when a resend just moved our received-through seq, which frees the goggles' send budget.
    const missing = w.missing();
    if (fresh && (units.length || v.index === v.total - 1 || ((missing.length || filled) && now - this.lastAck >= 10))) this.sendAck(missing);
    for (const u of units) this.deliver(u);
  }

  deliver(unit) {
    this.stats.units++;
    this.stats.bytes += unit.length;
    if (this.state !== "live") this.setState("live");
    this.emit("unit", unit);
  }
}
