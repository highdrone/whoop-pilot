// Just enough Ethernet / ARP / IPv4 / UDP to talk to the goggles over the RNDIS link: answers ARP
// for our address, resolves the goggles' MAC, reassembles fragmented IP datagrams (video datagrams
// are bigger than one Ethernet frame) and sends/receives UDP.

const ETH_IP = 0x0800;
const ETH_ARP = 0x0806;
const BROADCAST = new Uint8Array([255, 255, 255, 255, 255, 255]);

const ipStr = (ip) => ip.join(".");
const sameIp = (b, o, ip) => b[o] === ip[0] && b[o + 1] === ip[1] && b[o + 2] === ip[2] && b[o + 3] === ip[3];

function checksum(bytes, start, end, initial = 0) {
  let sum = initial;
  for (let i = start; i < end - 1; i += 2) sum += (bytes[i] << 8) | bytes[i + 1];
  if ((end - start) % 2) sum += bytes[end - 1] << 8;
  while (sum > 0xffff) sum = (sum & 0xffff) + (sum >>> 16);
  return sum;
}

export class NetStack {
  // mac: our MAC (from RNDIS), ip: our IPv4 as [a,b,c,d], sendFrame(frame: Uint8Array)
  constructor({ mac, ip, sendFrame, log = () => {} }) {
    this.mac = mac;
    this.ip = ip;
    this.sendFrame = sendFrame;
    this.log = log;
    this.arp = new Map(); // "a.b.c.d" -> mac
    this.arpWaiters = new Map();
    this.udp = new Map(); // port -> handler(srcIp, srcPort, payload)
    this.frags = new Map();
    this.ipId = Math.floor(Math.random() * 0xffff);
    this.stats = { udpIn: 0, fragmented: 0, dropped: 0 };
  }

  bindUdp(port, handler) {
    this.udp.set(port, handler);
  }

  announce() {
    this.sendArp(1, BROADCAST, this.ip, new Uint8Array(6), this.ip); // gratuitous ARP
  }

  handleFrame(f) {
    if (f.length < 14) return;
    const type = (f[12] << 8) | f[13];
    if (type === ETH_ARP) this.handleArp(f);
    else if (type === ETH_IP) this.handleIp(f.subarray(14), f.subarray(6, 12));
  }

  handleArp(f) {
    if (f.length < 42) return;
    const op = (f[20] << 8) | f[21];
    const sha = f.slice(22, 28);
    const spa = [f[28], f[29], f[30], f[31]];
    if (spa.some((x) => x)) {
      this.arp.set(ipStr(spa), sha);
      const waiters = this.arpWaiters.get(ipStr(spa));
      if (waiters) {
        this.arpWaiters.delete(ipStr(spa));
        waiters.forEach((w) => w(sha));
      }
    }
    if (op === 1 && sameIp(f, 38, this.ip)) this.sendArp(2, sha, this.ip, sha, spa);
  }

  sendArp(op, dstMac, spa, tha, tpa) {
    const f = new Uint8Array(42);
    f.set(dstMac, 0);
    f.set(this.mac, 6);
    f[12] = 0x08;
    f[13] = 0x06;
    f.set([0, 1, 8, 0, 6, 4, 0, op], 14);
    f.set(this.mac, 22);
    f.set(spa, 28);
    f.set(tha, 32);
    f.set(tpa, 38);
    this.sendFrame(f);
  }

  // MAC for an IP on the link; falls back to broadcast if nobody answers.
  async resolve(ip) {
    const key = ipStr(ip);
    if (this.arp.has(key)) return this.arp.get(key);
    for (let attempt = 0; attempt < 3; attempt++) {
      const got = new Promise((resolve) => {
        const list = this.arpWaiters.get(key) || [];
        list.push(resolve);
        this.arpWaiters.set(key, list);
        setTimeout(() => resolve(null), 300);
      });
      this.sendArp(1, BROADCAST, this.ip, new Uint8Array(6), ip);
      const mac = await got;
      if (mac) return mac;
    }
    this.log(`No ARP reply from ${key}; sending to broadcast`);
    return BROADCAST;
  }

  handleIp(p, srcMac) {
    if (p.length < 20 || p[0] >> 4 !== 4) return;
    const ihl = (p[0] & 15) * 4;
    const total = (p[2] << 8) | p[3];
    if (total > p.length || total < ihl) return;
    if (!sameIp(p, 16, this.ip) && !(p[16] === 255 && p[19] === 255)) return;
    const src = [p[12], p[13], p[14], p[15]];
    if (!this.arp.has(ipStr(src)) && srcMac) this.arp.set(ipStr(src), srcMac.slice());
    const flags = p[6] >> 5;
    const fragOff = (((p[6] & 0x1f) << 8) | p[7]) * 8;
    let payload = p.subarray(ihl, total);
    if (flags & 1 || fragOff) {
      payload = this.reassemble(src, (p[4] << 8) | p[5], p[9], fragOff, !!(flags & 1), payload);
      if (!payload) return;
    }
    if (p[9] === 17) this.handleUdp(src, payload);
  }

  reassemble(src, id, proto, offset, more, data) {
    const key = `${ipStr(src)}/${id}/${proto}`;
    let e = this.frags.get(key);
    if (!e) {
      e = { parts: [], total: -1, t: Date.now() };
      this.frags.set(key, e);
      if (this.frags.size > 64) this.expireFrags();
    }
    e.parts.push({ offset, data: data.slice() });
    if (!more) e.total = offset + data.length;
    if (e.total < 0) return null;
    let have = 0;
    for (const part of e.parts) have += part.data.length;
    if (have < e.total) return null;
    this.frags.delete(key);
    const out = new Uint8Array(e.total);
    for (const part of e.parts) if (part.offset + part.data.length <= e.total) out.set(part.data, part.offset);
    this.stats.fragmented++;
    return out;
  }

  expireFrags() {
    const now = Date.now();
    for (const [k, e] of this.frags) {
      if (now - e.t > 2000) {
        this.frags.delete(k);
        this.stats.dropped++;
      }
    }
  }

  handleUdp(src, u) {
    if (u.length < 8) return;
    const srcPort = (u[0] << 8) | u[1];
    const dstPort = (u[2] << 8) | u[3];
    const len = Math.min(u.length, (u[4] << 8) | u[5]);
    const h = this.udp.get(dstPort);
    this.stats.udpIn++;
    if (h) h(src, srcPort, u.subarray(8, len));
  }

  async sendUdp(dstIp, srcPort, dstPort, payload) {
    const dstMac = await this.resolve(dstIp);
    const udpLen = 8 + payload.length;
    const f = new Uint8Array(14 + 20 + udpLen);
    f.set(dstMac, 0);
    f.set(this.mac, 6);
    f[12] = 0x08;
    f[13] = 0x00;
    const ip = 14;
    this.ipId = (this.ipId + 1) & 0xffff;
    f.set([0x45, 0, (20 + udpLen) >> 8, (20 + udpLen) & 255, this.ipId >> 8, this.ipId & 255, 0x40, 0, 64, 17, 0, 0], ip);
    f.set(this.ip, ip + 12);
    f.set(dstIp, ip + 16);
    const hc = ~checksum(f, ip, ip + 20) & 0xffff;
    f[ip + 10] = hc >> 8;
    f[ip + 11] = hc & 255;
    const u = ip + 20;
    f.set([srcPort >> 8, srcPort & 255, dstPort >> 8, dstPort & 255, udpLen >> 8, udpLen & 255, 0, 0], u);
    f.set(payload, u + 8);
    // UDP checksum over the pseudo-header + datagram
    let pseudo = checksum(f, ip + 12, ip + 20); // src + dst
    pseudo += 17 + udpLen;
    let uc = ~checksum(f, u, u + udpLen, pseudo) & 0xffff;
    if (uc === 0) uc = 0xffff;
    f[u + 6] = uc >> 8;
    f[u + 7] = uc & 255;
    return this.sendFrame(f);
  }
}
