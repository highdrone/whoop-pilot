// RNDIS host over the WebUSB API. With "LiveView Sharing" on, DJI goggles show up over USB as an
// RNDIS network adapter; macOS has no driver for that, so we speak RNDIS ourselves. Works with
// Chrome's navigator.usb and with node-usb's WebUSB implementation (tools/goggles-helper.mjs).
//
// Spec: Microsoft "Remote NDIS Specification" (control messages over CDC SEND_ENCAPSULATED_COMMAND /
// GET_ENCAPSULATED_RESPONSE, Ethernet frames over bulk endpoints wrapped in REMOTE_NDIS_PACKET_MSG).

const MSG = { PACKET: 1, INIT: 2, HALT: 3, QUERY: 4, SET: 5, RESET: 6, INDICATE_STATUS: 7, KEEPALIVE: 8 };
const CMPLT = 0x80000000;
const OID_802_3_PERMANENT_ADDRESS = 0x01010101;
const OID_GEN_CURRENT_PACKET_FILTER = 0x0001010e;
const PACKET_FILTER = 0x2d; // directed | all-multicast | broadcast | promiscuous (same as Linux rndis_host)
const HOST_MAX_TRANSFER = 16384;
const PACKET_HEADER = 44;

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const u32 = (b, o) => (b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) >>> 0;

function message(type, requestId, words, payload) {
  const len = 8 + 4 * words.length + (payload ? payload.length : 0);
  const b = new Uint8Array(len);
  const v = new DataView(b.buffer);
  v.setUint32(0, type, true);
  v.setUint32(4, len, true);
  v.setUint32(8, requestId, true);
  words.slice(1).forEach((w, i) => v.setUint32(12 + 4 * i, w >>> 0, true));
  if (payload) b.set(payload, 8 + 4 * words.length);
  return b;
}

// RNDIS control interface: CDC (02/02/ff, Linux gadgets), wireless (e0/01/03) or misc (ef/04/01).
function isRndisControl(a) {
  return (
    (a.interfaceClass === 0x02 && a.interfaceSubclass === 0x02 && a.interfaceProtocol === 0xff) ||
    (a.interfaceClass === 0xe0 && a.interfaceSubclass === 0x01 && a.interfaceProtocol === 0x03) ||
    (a.interfaceClass === 0xef && a.interfaceSubclass === 0x04 && a.interfaceProtocol === 0x01)
  );
}

export function findRndisInterfaces(config) {
  let ctrl = null;
  let data = null;
  for (const iface of config.interfaces) {
    for (const a of iface.alternates) {
      if (!ctrl && isRndisControl(a)) ctrl = { number: iface.interfaceNumber, alt: a };
      if (!data && a.interfaceClass === 0x0a) {
        const epIn = a.endpoints.find((e) => e.direction === "in" && e.type === "bulk");
        const epOut = a.endpoints.find((e) => e.direction === "out" && e.type === "bulk");
        if (epIn && epOut) data = { number: iface.interfaceNumber, alt: a, epIn, epOut };
      }
    }
  }
  return ctrl && data ? { ctrl, data } : null;
}

// Human-readable dump of a device's descriptors (for troubleshooting).
export function describeDevice(dev) {
  const hex = (n) => n.toString(16).padStart(2, "0");
  const lines = [`${hex(dev.vendorId)}${hex(dev.productId)} ${dev.manufacturerName || ""} ${dev.productName || ""}`.trim()];
  for (const c of dev.configurations) {
    lines.push(`config ${c.configurationValue}${dev.configuration?.configurationValue === c.configurationValue ? " (active)" : ""}`);
    for (const i of c.interfaces) {
      for (const a of i.alternates) {
        const eps = a.endpoints.map((e) => `${e.direction}${e.endpointNumber}:${e.type}/${e.packetSize}`).join(" ");
        lines.push(`  if ${i.interfaceNumber} alt ${a.alternateSetting} class ${hex(a.interfaceClass)}/${hex(a.interfaceSubclass)}/${hex(a.interfaceProtocol)} ${eps}`);
      }
    }
  }
  return lines.join("\n");
}

export class RndisLink {
  constructor(device, { log = () => {} } = {}) {
    this.dev = device;
    this.log = log;
    this.requestId = 0;
    this.running = false;
    this.onFrame = null;
    this.onError = null;
    this.stats = { rxFrames: 0, txFrames: 0, rxBytes: 0 };
  }

  async open() {
    const dev = this.dev;
    if (!dev.opened) await dev.open();
    if (!dev.configuration) await dev.selectConfiguration(dev.configurations[0].configurationValue);
    let found = findRndisInterfaces(dev.configuration);
    if (!found) {
      for (const cfg of dev.configurations) {
        if (findRndisInterfaces(cfg)) {
          await dev.selectConfiguration(cfg.configurationValue);
          found = findRndisInterfaces(dev.configuration);
          break;
        }
      }
    }
    if (!found) throw new Error(`No RNDIS network interface on this device. Are the goggles on with Share Liveview to Mobile Device via Wi-Fi on and OTG Wired Connection off?\n${describeDevice(dev)}`);
    this.ctrl = found.ctrl;
    this.data = found.data;
    await dev.claimInterface(this.ctrl.number);
    await dev.claimInterface(this.data.number);
    if (this.data.alt.alternateSetting) await dev.selectAlternateInterface(this.data.number, this.data.alt.alternateSetting);
    this.maxPacket = this.data.epOut.packetSize || 512;

    const init = await this.command(message(MSG.INIT, ++this.requestId, [0, 1, 0, HOST_MAX_TRANSFER]), MSG.INIT);
    if (u32(init, 12) !== 0) throw new Error(`RNDIS initialize failed (status 0x${u32(init, 12).toString(16)})`);
    this.deviceMaxTransfer = u32(init, 36);
    const mac = await this.query(OID_802_3_PERMANENT_ADDRESS, 48);
    if (mac.length < 6) throw new Error("RNDIS: no MAC address");
    this.mac = mac.slice(0, 6);
    const filter = new Uint8Array(4);
    new DataView(filter.buffer).setUint32(0, PACKET_FILTER, true);
    await this.set(OID_GEN_CURRENT_PACKET_FILTER, filter);
    this.log(`RNDIS up (max transfer ${this.deviceMaxTransfer}, MAC ${[...this.mac].map((b) => b.toString(16).padStart(2, "0")).join(":")})`);
    return { mac: this.mac };
  }

  ctrlOut(msg) {
    return this.dev.controlTransferOut({ requestType: "class", recipient: "interface", request: 0x00, value: 0, index: this.ctrl.number }, msg);
  }

  async ctrlIn() {
    const r = await this.dev.controlTransferIn({ requestType: "class", recipient: "interface", request: 0x01, value: 0, index: this.ctrl.number }, 1025);
    if (r.status !== "ok" || !r.data || r.data.byteLength < 8) return null;
    return new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength).slice();
  }

  // Send a control message and poll for its completion (like Linux rndis_host does).
  async command(msg, type) {
    const id = u32(msg, 8);
    await this.ctrlOut(msg);
    for (let i = 0; i < 120; i++) {
      const r = await this.ctrlIn().catch(() => null);
      if (r) {
        const t = u32(r, 0);
        if (t === ((type | CMPLT) >>> 0) && u32(r, 8) === id) return r;
        await this.handleUnsolicited(r);
      }
      await sleep(i < 10 ? 5 : 25);
    }
    throw new Error(`RNDIS: no reply to message type ${type}`);
  }

  async handleUnsolicited(r) {
    if (u32(r, 0) === MSG.KEEPALIVE) await this.ctrlOut(message((MSG.KEEPALIVE | CMPLT) >>> 0, u32(r, 8), [0, 0]));
  }

  async query(oid, len) {
    const r = await this.command(message(MSG.QUERY, ++this.requestId, [0, oid, len, 20, 0], new Uint8Array(len)), MSG.QUERY);
    if (u32(r, 12) !== 0) throw new Error(`RNDIS query 0x${oid.toString(16)} failed`);
    const n = u32(r, 16);
    const off = 8 + u32(r, 20);
    return r.slice(off, off + n);
  }

  async set(oid, value) {
    const r = await this.command(message(MSG.SET, ++this.requestId, [0, oid, value.length, 20, 0], value), MSG.SET);
    if (u32(r, 12) !== 0) throw new Error(`RNDIS set 0x${oid.toString(16)} failed`);
  }

  // Start receiving Ethernet frames. Several transfers stay queued so the device never waits on us.
  start(onFrame, onError) {
    this.onFrame = onFrame;
    this.onError = onError;
    this.running = true;
    for (let i = 0; i < 8; i++) this.pump();
    this.keepaliveTimer = setInterval(() => this.pollControl(), 1000);
  }

  async pump() {
    const ep = this.data.epIn.endpointNumber;
    const size = Math.max(HOST_MAX_TRANSFER, this.deviceMaxTransfer || 0);
    while (this.running) {
      let r;
      try {
        r = await this.dev.transferIn(ep, size);
      } catch (e) {
        if (this.running) {
          this.running = false;
          this.onError?.(e);
        }
        return;
      }
      if (r.status === "stall") {
        await this.dev.clearHalt("in", ep).catch(() => {});
        continue;
      }
      if (r.data && r.data.byteLength) this.parseIn(new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength));
    }
  }

  // One transfer may hold several REMOTE_NDIS_PACKET_MSGs back to back.
  parseIn(buf) {
    let off = 0;
    while (off + PACKET_HEADER <= buf.length) {
      const type = u32(buf, off);
      const len = u32(buf, off + 4);
      if (type !== MSG.PACKET || len < PACKET_HEADER || off + len > buf.length) break;
      const dataOff = off + 8 + u32(buf, off + 8);
      const dataLen = u32(buf, off + 12);
      if (dataOff + dataLen <= off + len) {
        this.stats.rxFrames++;
        this.stats.rxBytes += dataLen;
        this.onFrame?.(buf.subarray(dataOff, dataOff + dataLen));
      }
      off += len;
    }
  }

  sendFrame(frame) {
    const len = PACKET_HEADER + frame.length;
    // A transfer that is an exact multiple of the packet size would need a zero-length packet to
    // end it; add one spare byte instead (the device goes by MessageLength), as Linux does.
    const buf = new Uint8Array(len + (len % this.maxPacket === 0 ? 1 : 0));
    const v = new DataView(buf.buffer);
    v.setUint32(0, MSG.PACKET, true);
    v.setUint32(4, len, true);
    v.setUint32(8, PACKET_HEADER - 8, true);
    v.setUint32(12, frame.length, true);
    buf.set(frame, PACKET_HEADER);
    this.stats.txFrames++;
    return this.dev.transferOut(this.data.epOut.endpointNumber, buf).catch((e) => this.onError?.(e));
  }

  async pollControl() {
    if (!this.running || this.polling) return;
    this.polling = true;
    try {
      for (let i = 0; i < 4; i++) {
        const r = await this.ctrlIn().catch(() => null);
        if (!r) break;
        await this.handleUnsolicited(r);
      }
    } finally {
      this.polling = false;
    }
  }

  async close() {
    this.running = false;
    clearInterval(this.keepaliveTimer);
    try {
      await this.ctrlOut(message(MSG.HALT, ++this.requestId, [0]));
    } catch {}
    try {
      await this.dev.releaseInterface(this.data.number);
      await this.dev.releaseInterface(this.ctrl.number);
    } catch {}
    try {
      await this.dev.close();
    } catch {}
  }
}
