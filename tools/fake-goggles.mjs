// A pretend DJI Goggles 3 in "LiveView Sharing" mode, implementing the WebUSB device API: RNDIS
// control + bulk data, ARP, IPv4 (it fragments big datagrams like a real Linux stack) and the
// LiveView UDP session (handshake, window reports, video that stops unless ACKs move the window,
// resends on request).
// Used by test-goggles.mjs to exercise app/js/goggles/* end to end without hardware. Where the real
// device's rules are known (from a capture of our goggles and the dji-goggles-lab notes) it is at
// least as strict: the send budget is ~128 KiB outstanding past the ACKed seq, checked per frame; a
// resend list is only honoured when it starts at the first hole; an ACK is dropped when its lengths
// don't add up, its start is off the seq grid, its end is before its start or past what was sent, or
// its resend list is longer than the reference client's 16 or isn't an increasing run of on-grid
// seqs inside (start, end].

const u32 = (b, o) => (b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) >>> 0;
const u16 = (p, o) => p[o] | (p[o + 1] << 8);
const xor7 = (p) => p[0] ^ p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5] ^ p[6];
const put16 = (p, o, v) => {
  p[o] = v & 255;
  p[o + 1] = (v >> 8) & 255;
};
const ahead = (a, b) => {
  const d = (a - b) & 0xffff;
  return d < 0x8000 ? d : d - 0x10000;
};
const HOST_MAC = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55]; // what the device tells the host its NIC MAC is
const GOG_MAC = [0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0xee];
const GOG_IP = [192, 168, 60, 2];
export const FAKE_SERIAL = "1234ABCD5678EF";

// DJI DUML frame: 55 | len(10 bits) + version 1 | crc8 | sender | receiver | seq u16le | flags | set | id |
// payload | crc16le.  CRC8: init 0x77, reflected poly 0x8c. CRC16: init 0x3692, poly 0x8408.
function crc8(b, n) {
  let c = 0x77;
  for (let i = 0; i < n; i++) {
    c ^= b[i];
    for (let k = 0; k < 8; k++) c = c & 1 ? (c >> 1) ^ 0x8c : c >> 1;
  }
  return c;
}

function crc16(b, n) {
  let c = 0x3692;
  for (let i = 0; i < n; i++) {
    c ^= b[i];
    for (let k = 0; k < 8; k++) c = c & 1 ? (c >> 1) ^ 0x8408 : c >> 1;
  }
  return c;
}

function buildDuml({ sender, receiver, seq, flags, set, id, payload }) {
  const len = payload.length + 13;
  const f = new Uint8Array(len);
  f[0] = 0x55;
  f[1] = len & 0xff;
  f[2] = ((len >> 8) & 0x03) | 0x04;
  f[3] = crc8(f, 3);
  f.set([sender, receiver, seq & 0xff, (seq >> 8) & 0xff, flags, set, id], 4);
  f.set(payload, 11);
  const c = crc16(f, len - 2);
  f[len - 2] = c & 0xff;
  f[len - 1] = c >> 8;
  return f;
}

function csum(b, s, e, init = 0) {
  let sum = init;
  for (let i = s; i < e - 1; i += 2) sum += (b[i] << 8) | b[i + 1];
  if ((e - s) % 2) sum += b[e - 1] << 8;
  while (sum > 0xffff) sum = (sum & 0xffff) + (sum >>> 16);
  return sum;
}

export class FakeGoggles {
  // partSize 1452 makes 1472-byte datagrams like the real goggles; bigger ones exercise IP fragments.
  // budget: bytes of video the goggles send ahead of the app's ACKs (a real one stopped at 128967
  // bytes / 94 packets). lossRate drops first sends. resend false: never honour resend lists.
  // serialEvery: put the serial-number DUML in every n-th window report (the real one: ~every 10 s).
  // answer false: the 9-byte handshake answer never arrives (lost on the link); streaming starts anyway.
  constructor({ units = [], fps = 60, partSize = 1452, mtu = 1500, budget = 131072, lossRate = 0, resend = true, serialEvery = 100, answer = true, classes = [0x02, 0x02, 0xff] } = {}) {
    Object.assign(this, { units, fps, partSize, mtu, budget, lossRate, resend, serialEvery, answer });
    let r = 12345;
    this.rand = () => ((r = (r * 1103515245 + 12345) & 0x7fffffff) / 0x80000000);
    this.vendorId = 0x2ca3;
    this.productId = 0x0020;
    this.productName = "DJI Goggles 3";
    this.manufacturerName = "DJI";
    this.opened = false;
    this.configurations = [
      {
        configurationValue: 1,
        interfaces: [
          { interfaceNumber: 0, alternates: [{ alternateSetting: 0, interfaceClass: classes[0], interfaceSubclass: classes[1], interfaceProtocol: classes[2], endpoints: [{ endpointNumber: 1, direction: "in", type: "interrupt", packetSize: 8 }] }] },
          { interfaceNumber: 1, alternates: [{ alternateSetting: 0, interfaceClass: 0x0a, interfaceSubclass: 0, interfaceProtocol: 0, endpoints: [{ endpointNumber: 2, direction: "in", type: "bulk", packetSize: 512 }, { endpointNumber: 3, direction: "out", type: "bulk", packetSize: 512 }] }] },
        ],
      },
    ];
    this.configuration = null;
    this.ctrlQueue = [];
    this.inQueue = [];
    this.inWaiters = [];
    this.stats = { openers: 0, acksSent: 0, dataSent: 0, acksReceived: 0, badAcks: 0, foreignAcks: 0, ignoredResendLists: 0, resendsAsked: 0, videoSent: 0, videoBytes: 0, lost: 0, resent: 0, stalledTicks: 0, unitsSent: 0, ipFragments: 0 };
    this.firstAck = null; // raw bytes of the first ACK of the current session
    this.lastAck = null; // parsed
    this.session = null;
    this.hostIp = null;
    this.ipId = 1;
    this.dumlSeq = 0;
    this.paused = false;
  }

  // ---- WebUSB surface ----
  async open() {
    this.opened = true;
  }
  async close() {
    this.opened = false;
    this.stopStreaming();
  }
  async selectConfiguration(v) {
    this.configuration = this.configurations.find((c) => c.configurationValue === v);
  }
  async claimInterface() {}
  async releaseInterface() {}
  async selectAlternateInterface() {}
  async clearHalt() {}

  async controlTransferOut(setup, data) {
    const m = new Uint8Array(data.buffer ? data.buffer.slice(data.byteOffset ?? 0, (data.byteOffset ?? 0) + data.byteLength) : data);
    const type = u32(m, 0);
    const id = u32(m, 8);
    const reply = (t, words, payload) => {
      const len = 8 + 4 * words.length + (payload ? payload.length : 0);
      const b = new Uint8Array(len);
      const v = new DataView(b.buffer);
      v.setUint32(0, t, true);
      v.setUint32(4, len, true);
      words.forEach((w, i) => v.setUint32(8 + 4 * i, w, true));
      if (payload) b.set(payload, 8 + 4 * words.length);
      this.ctrlQueue.push(b);
    };
    if (type === 2) reply(0x80000002, [id, 0, 1, 0, 0, 0, 1, 1600, 0, 0, 0]); // INIT_CMPLT
    else if (type === 4) reply(0x80000004, [id, 0, 6, 16], new Uint8Array(HOST_MAC)); // QUERY_CMPLT (MAC)
    else if (type === 5) {
      this.filter = u32(m, 28);
      reply(0x80000005, [id, 0]);
    }
    return { status: "ok", bytesWritten: m.length };
  }

  async controlTransferIn() {
    const r = this.ctrlQueue.shift();
    return { status: "ok", data: r ? new DataView(r.buffer) : new DataView(new ArrayBuffer(1)) };
  }

  async transferOut(ep, data) {
    const b = data instanceof Uint8Array ? data : new Uint8Array(data);
    if (u32(b, 0) !== 1) return { status: "ok" };
    const off = 8 + u32(b, 8);
    this.onEthernet(b.subarray(off, off + u32(b, 12)));
    return { status: "ok", bytesWritten: b.length };
  }

  transferIn() {
    if (this.inQueue.length) return Promise.resolve(this.wrapIn(this.inQueue.shift()));
    return new Promise((resolve) => this.inWaiters.push(resolve));
  }

  wrapIn(frame) {
    const b = new Uint8Array(44 + frame.length);
    const v = new DataView(b.buffer);
    v.setUint32(0, 1, true);
    v.setUint32(4, b.length, true);
    v.setUint32(8, 36, true);
    v.setUint32(12, frame.length, true);
    b.set(frame, 44);
    return { status: "ok", data: new DataView(b.buffer) };
  }

  deliver(frame) {
    if (this.filter === undefined) return; // host hasn't set a packet filter yet: device stays quiet
    const w = this.inWaiters.shift();
    if (w) w(this.wrapIn(frame));
    else this.inQueue.push(frame);
  }

  // ---- the goggles' network side ----
  onEthernet(f) {
    const type = (f[12] << 8) | f[13];
    if (type === 0x0806) {
      const op = (f[20] << 8) | f[21];
      const tpa = [...f.subarray(38, 42)];
      this.hostMac = [...f.subarray(22, 28)];
      if (op === 1 && tpa.join() === GOG_IP.join()) {
        const r = new Uint8Array(42);
        r.set(this.hostMac, 0);
        r.set(GOG_MAC, 6);
        r.set([0x08, 0x06, 0, 1, 8, 0, 6, 4, 0, 2], 12);
        r.set(GOG_MAC, 22);
        r.set(GOG_IP, 28);
        r.set(f.subarray(22, 28), 32);
        r.set(f.subarray(28, 32), 38);
        this.deliver(r);
      }
      return;
    }
    if (type !== 0x0800) return;
    const ip = f.subarray(14);
    if (ip[9] !== 17 || ip.slice(16, 20).join() !== GOG_IP.join()) return;
    this.hostIp = [...ip.subarray(12, 16)];
    const udp = ip.subarray((ip[0] & 15) * 4);
    const dstPort = (udp[2] << 8) | udp[3];
    if (dstPort !== 9003) return;
    const payload = udp.subarray(8, (udp[4] << 8) | udp[5]);
    this.onSession(payload.slice());
  }

  onSession(p) {
    const len = u16(p, 0) & 0x7fff;
    if (p.length < 8 || len !== p.length || p[7] !== xor7(p)) return;
    if (p[6] === 0 && p.length === 48) {
      // A new handshake replaces whatever session was running, like the real goggles.
      this.stats.openers++;
      this.session = u16(p, 2);
      this.seed = u16(p, 8);
      this.firstAck = this.lastAck = null;
      const ack = new Uint8Array([0x09, 0x80, p[2], p[3], 0, 0, 0, 0, 0x01]);
      ack[7] = xor7(ack);
      if (this.answer) this.sendUdp(ack);
      this.stats.acksSent++;
      this.startStreaming();
      return;
    }
    if (p[6] !== 4) return;
    if (u16(p, 2) !== this.session) this.stats.foreignAcks++;
    else this.onAck(p);
  }

  // Our ACK: video (start, end, n, n seqs), type-3 and type-5 windows (start, end, 0), 0, DUML length.
  onAck(p) {
    const n = u16(p, 12);
    const tail = 14 + 2 * n;
    const start = u16(p, 8);
    const end = u16(p, 10);
    const list = [];
    for (let i = 0; i < n && 14 + 2 * i + 2 <= p.length; i++) list.push(u16(p, 14 + 2 * i));
    const onGrid = (s) => !((s - this.seed) & 7);
    const listOk = list.every((s, i) => onGrid(s) && ahead(s, start) > 0 && ahead(s, end) <= 0 && (!i || ahead(s, list[i - 1]) > 0));
    if (p.length < tail + 16 || p.length !== tail + 16 + u16(p, tail + 14) || !onGrid(start) || ahead(end, start) < 0 || ahead(end, this.seq) > 0 || n > 16 || !listOk) {
      this.stats.badAcks++;
      return;
    }
    this.stats.acksReceived++;
    this.lastAck = { start, end, list, t3: [u16(p, tail), u16(p, tail + 2)], t5: [u16(p, tail + 6), u16(p, tail + 8)] };
    this.firstAck ??= p.slice();
    // the window moves up to what the app has received, never past what we sent
    while (ahead(start, this.acked) > 0 && ahead(start, this.seq) <= 0) {
      this.acked = (this.acked + 8) & 0xffff;
      this.unacked -= this.history.get(this.acked)?.length ?? 0;
    }
    if (!n) return;
    this.stats.resendsAsked += n;
    if (list[0] !== ((this.acked + 8) & 0xffff)) {
      this.stats.ignoredResendLists++; // like the real firmware: the list must start at the first hole
      return;
    }
    if (!this.resend) return;
    for (const s of list) {
      const pkt = this.history.get(s);
      if (pkt && ahead(s, this.acked) > 0) {
        this.sendUdp(pkt);
        this.stats.resent++;
      }
    }
  }

  startStreaming() {
    this.stopStreaming();
    this.seq = this.acked = this.seed;
    this.unacked = 0;
    this.history = new Map();
    let tick = 0;
    let unitId = 1;
    this.timer = setInterval(() => {
      if (tick++ % Math.max(1, Math.round(this.fps / 10)) === 0) this.sendData();
      if (!this.units.length || this.paused) return;
      const unit = this.units[this.stats.unitsSent % this.units.length];
      const parts = Math.max(1, Math.ceil(unit.length / this.partSize));
      // Like the real goggles: a frame only goes out if it fits the unacknowledged budget.
      if (this.unacked + unit.length + 20 * parts > this.budget) {
        this.stats.stalledTicks++;
        return;
      }
      this.sendUnit(unit, unitId, parts);
      unitId = (unitId + 1) & 0xff;
      this.stats.unitsSent++;
    }, 1000 / this.fps);
  }

  stopStreaming() {
    clearInterval(this.timer);
  }

  // Video stops (drone off) while the window reports keep coming.
  pauseVideo(paused = true) {
    this.paused = paused;
  }

  // Windows + DUML, about 10 times a second. The DUML (when present) is the goggles' own
  // 0x1b -> app, set 7, id 0x94 frame carrying the serial number, as captured.
  sendData() {
    const s = this.session;
    let duml = new Uint8Array(0);
    if (this.stats.dataSent % this.serialEvery === 0) {
      const payload = new Uint8Array(31);
      payload.set([0x01, 0x2b, 0x23, 0x02, 0x00, 0x03, 0x00]);
      payload.set([...FAKE_SERIAL].map((c) => c.charCodeAt(0)), 7);
      duml = buildDuml({ sender: 0x1b, receiver: 0x02, seq: this.dumlSeq++, flags: 0, set: 7, id: 0x94, payload });
    }
    const p = new Uint8Array(34 + duml.length);
    put16(p, 0, p.length | 0x8000);
    p.set([s & 255, s >> 8, 0, 0, 0x01], 2);
    put16(p, 8, this.acked);
    put16(p, 10, this.seq);
    for (const o of [16, 24]) {
      put16(p, o, this.seed);
      put16(p, o + 2, this.seed);
    }
    put16(p, 32, duml.length);
    p.set(duml, 34);
    p[7] = xor7(p);
    this.stats.dataSent++;
    this.sendUdp(p);
  }

  sendUnit(unit, unitId, parts) {
    const s = this.session;
    for (let i = 0; i < parts; i++) {
      const payload = unit.subarray(i * this.partSize, (i + 1) * this.partSize);
      this.seq = (this.seq + 8) & 0xffff;
      const p = new Uint8Array(20 + payload.length);
      put16(p, 0, p.length | 0x8000);
      p.set([s & 255, s >> 8], 2);
      put16(p, 4, this.seq);
      p[6] = 0x02;
      put16(p, 8, this.acked);
      put16(p, 10, this.seq);
      p[16] = unitId;
      p[17] = (parts & 0x7f) | ((i & 1) << 7);
      p[18] = (i >> 1) & 0x1f;
      p.set(payload, 20);
      p[7] = xor7(p);
      this.history.set(this.seq, p);
      this.history.delete((this.seq - 8 * 1024) & 0xffff);
      this.unacked += p.length;
      this.stats.videoSent++;
      this.stats.videoBytes += p.length;
      if (this.lossRate && this.rand() < this.lossRate) {
        this.stats.lost++;
        continue;
      }
      this.sendUdp(p);
    }
  }

  sendUdp(payload) {
    const udpLen = 8 + payload.length;
    const dgram = new Uint8Array(udpLen);
    dgram.set([0x23, 0x2b, 0x23, 0x2b, udpLen >> 8, udpLen & 255, 0, 0], 0); // 9003 -> 9003, no checksum
    dgram.set(payload, 8);
    // IPv4, fragmented to the MTU like a real stack
    const maxData = Math.floor((this.mtu - 20) / 8) * 8;
    const id = this.ipId++ & 0xffff;
    for (let off = 0; off < dgram.length; off += maxData) {
      const chunk = dgram.subarray(off, off + maxData);
      const more = off + maxData < dgram.length;
      const f = new Uint8Array(14 + 20 + chunk.length);
      f.set(this.hostMac || [255, 255, 255, 255, 255, 255], 0);
      f.set(GOG_MAC, 6);
      f[12] = 0x08;
      const ip = 14;
      const total = 20 + chunk.length;
      const fo = (off / 8) | (more ? 0x2000 : 0);
      f.set([0x45, 0, total >> 8, total & 255, id >> 8, id & 255, fo >> 8, fo & 255, 64, 17, 0, 0], ip);
      f.set(GOG_IP, ip + 12);
      f.set(this.hostIp || [192, 168, 60, 1], ip + 16);
      const c = ~csum(f, ip, ip + 20) & 0xffff;
      f[ip + 10] = c >> 8;
      f[ip + 11] = c & 255;
      f.set(chunk, ip + 20);
      if (more || off) this.stats.ipFragments++;
      this.deliver(f);
    }
  }
}
