// End-to-end test of the goggles USB video stack (app/js/goggles/*) against a fake Goggles 3:
// RNDIS bring-up, ARP, IP fragment reassembly, the LiveView handshake, ACKs that keep the goggles'
// send budget moving, loss recovery, video reassembly, stalls, and reconnecting after the link goes
// quiet. Wire bytes are checked against dji-goggles-lab's reference client and a capture from real
// Goggles 3.
import assert from "node:assert/strict";
import { FakeGoggles, FAKE_SERIAL } from "./fake-goggles.mjs";
import { RndisLink, findRndisInterfaces } from "../app/js/goggles/rndis.js";
import { NetStack } from "../app/js/goggles/netstack.js";
import { LiveviewSession, HOST_IP, buildHandshake, buildAck, packetType, parseVideo, parseData, serialIn, ahead, ReceiveWindow, FrameAssembler } from "../app/js/goggles/liveview.js";
import { describeUnit, nalUnits, codecString } from "../app/js/goggles/h264.js";

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const hex = (b) => [...b].map((x) => x.toString(16).padStart(2, "0")).join(" ");
const bytes = (s) => Uint8Array.from(s.split(" ").map((x) => parseInt(x, 16)));

// Fake Annex-B units of realistic sizes for the goggles' stream (P frames 2-30 KB, some bigger).
function makeUnits(n) {
  const units = [];
  for (let i = 0; i < n; i++) {
    const size = i % 30 === 0 ? 60000 : 2000 + ((i * 7919) % 28000);
    const u = new Uint8Array(size);
    let o = 0;
    if (i % 30 === 0) {
      u.set([0, 0, 0, 1, 0x67, 0x64, 0x00, 0x2a, 0xac, 0x2b], 0); // SPS: High, level 4.2
      u.set([0, 0, 0, 1, 0x68, 0xee, 0x3c, 0x80], 10); // PPS
      u.set([0, 0, 0, 1, 0x65, 0x88, 0x84], 18); // IDR slice
      o = 25;
    } else {
      u.set([0, 0, 0, 1, 0x41, 0x9a, 0x00], 0); // non-IDR slice
      o = 7;
    }
    for (let k = o; k < size; k++) u[k] = ((k * 31 + i) % 251) + 1; // no accidental start codes
    units.push(u);
  }
  return units;
}

async function connect(fake, opts) {
  const link = new RndisLink(fake);
  const { mac } = await link.open();
  const net = new NetStack({ mac, ip: HOST_IP, sendFrame: (f) => link.sendFrame(f) });
  link.start((f) => net.handleFrame(f), (e) => {
    throw e;
  });
  net.announce();
  const session = new LiveviewSession(net, opts);
  const got = [];
  const states = [];
  session.on("unit", (u) => got.push(u));
  session.on("state", (s) => states.push(s));
  session.start();
  return { link, net, session, got, states, mac };
}

async function until(test, ms, what) {
  const t0 = Date.now();
  while (!test()) {
    if (Date.now() - t0 > ms) throw new Error(`timed out waiting for ${what}`);
    await sleep(10);
  }
}

async function finish(c, fake) {
  fake.stopStreaming();
  c.session.stop();
  await c.link.close();
}

// Every delivered unit is one of `units`, in order (some may be skipped).
function assertInOrder(got, units) {
  let j = 0;
  got.forEach((u, i) => {
    while (j < units.length && !(units[j].length === u.length && units[j].every((b, k) => b === u[k]))) j++;
    assert.ok(j < units.length, `unit ${i} (${u.length} bytes) is not the next unit of the stream`);
    j++;
  });
}

// A session on a stub network, fed datagrams by hand.
function bench(opts) {
  const net = { sent: [], bindUdp: (port, h) => (net.h = h), sendUdp: (ip, sp, dp, p) => net.sent.push(p) };
  const s = new LiveviewSession(net, { ackEvery: 1000, ...opts });
  s.start();
  const answer = new Uint8Array([0x09, 0x80, s.session & 255, s.session >> 8, 0, 0, 0, 0, 0x01]);
  answer[7] = answer[0] ^ answer[1] ^ answer[2] ^ answer[3] ^ answer[4] ^ answer[5] ^ answer[6];
  net.h(null, 9003, answer);
  const sealed = (body, session = s.session) => {
    const p = Uint8Array.from(body);
    p[0] = p.length & 255;
    p[1] = (p.length >> 8) | 0x80;
    p[2] = session & 255;
    p[3] = session >> 8;
    p[7] = p[0] ^ p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5] ^ p[6];
    return p;
  };
  return { net, s, sealed };
}

const tests = {
  "handshake and ACK bytes match the reference client; real goggles packets parse"() {
    // reference bytes from dji-goggles-lab's implementation of the SquirrelReceiver protocol
    assert.equal(
      hex(buildHandshake(0x1234, 0xe9d0)),
      "30 80 34 12 00 00 00 96 d0 e9 64 00 64 00 c0 05 14 00 00 0a 00 64 00 64 00 c0 05 14 00 00 64 00 14 00 64 00 c0 05 14 00 00 64 00 01 01 04 0a 02",
    );
    assert.equal(
      hex(buildAck(0x71f5, { start: 0xf5e0, end: 0xf5f8, resend: [0xf5e8, 0xf5f0], t3: [0xf5e0, 0xf5e0], t5: [0xf5e0, 0xf5e0] })),
      "22 80 f5 71 00 00 04 22 e0 f5 f8 f5 02 00 e8 f5 f0 f5 e0 f5 e0 f5 00 00 e0 f5 e0 f5 00 00 00 00 00 00",
    );
    assert.equal(hex(buildAck(0x71f5, { start: 0xe9d0, end: 0xe9d0 })), "1e 80 f5 71 00 00 04 1e d0 e9 d0 e9 00 00 d0 e9 d0 e9 00 00 d0 e9 d0 e9 00 00 00 00 00 00");
    // captured from a Goggles 3: handshake answer, window reports, first parts of a frame
    assert.equal(packetType(bytes("09 80 5b 49 00 00 00 9b 01"), 0x495b), 0);
    const report = bytes("22 80 5b 49 00 00 01 b1 e0 f5 d0 f8 00 00 00 00 e0 f5 e0 f5 00 00 00 00 e0 f5 e0 f5 00 00 00 00 00 00");
    assert.equal(packetType(report, 0x495b), 1);
    assert.equal(packetType(report, 0x495c), -1);
    assert.deepEqual({ ...parseData(report), duml: undefined }, { start: 0xf5e0, end: 0xf8d0, t3: [0xf5e0, 0xf5e0], t5: [0xf5e0, 0xf5e0], duml: undefined });
    assert.equal(parseData(report).duml.length, 0);
    // the 78-byte kind carries a DUML from the goggles (0x1b -> app, set 7, id 0x94) with the serial
    const fake = new FakeGoggles();
    fake.session = 0x495b;
    fake.seed = fake.seq = fake.acked = 0xf5e0;
    fake.sendUdp = (p) => (fake.sentData = p);
    fake.sendData();
    assert.equal(fake.sentData.length, 78);
    assert.equal(hex(fake.sentData.subarray(0, 8)), "4e 80 5b 49 00 00 01 dd", "header as captured");
    assert.equal(hex(fake.sentData.subarray(32, 45)), "2c 00 55 2c 04 36 1b 02 00 00 00 07 94", "DUML length and frame as captured");
    assert.equal(serialIn(parseData(fake.sentData).duml), FAKE_SERIAL);
    const v0 = new Uint8Array(1472);
    v0.set(bytes("c0 85 5b 49 e8 f5 02 48 e0 f5 e8 f5 00 00 00 00 01 0d 00 00 00 00 00 01"));
    const v1 = new Uint8Array(1472);
    v1.set(bytes("c0 85 5b 49 f0 f5 02 50 e0 f5 f0 f5 00 00 00 00 01 8d 00 00 8d 1d 90 07"));
    const v2 = bytes("c0 85 5b 49 f8 f5 02 58 e0 f5 f8 f5 00 00 00 00 01 0d 01 00 e8 c8 6b 72");
    assert.equal(packetType(v0, 0x495b), 2);
    assert.deepEqual({ ...parseVideo(v0), payload: undefined }, { seq: 0xf5e8, frame: 1, total: 13, index: 0, payload: undefined });
    assert.equal(parseVideo(v1).index, 1);
    assert.equal(parseVideo(v2).index, 2);
    assert.equal(hex(parseVideo(v0).payload.subarray(0, 4)), "00 00 00 01");
    assert.deepEqual({ ...parseVideo(bytes("18 80 5b 49 00 00 02 00 00 00 00 00 00 00 00 00 05 c0 1f 00 00 00 00 00")), payload: undefined }, { seq: 0, frame: 5, total: 64, index: 63, payload: undefined }, "6-bit part index");
    // the fake goggles refuse ACKs no firmware could accept, so the tests below prove we never send one
    fake.seq = 0xf5e0 + 32;
    fake.history = new Map();
    fake.unacked = 0;
    const ack = (o) => fake.onAck(buildAck(0x495b, o));
    ack({ start: 0xf5e0, end: 0xf5e0 + 8, resend: [0xf5e0 + 16] }); // beyond end
    ack({ start: 0xf5e0, end: 0xf5e0 + 16, resend: [0xf5e0 + 16, 0xf5e0 + 8] }); // not increasing
    ack({ start: 0xf5e0 + 16, end: 0xf5e0 + 8 }); // end before start
    ack({ start: 0xf5e0 + 4, end: 0xf5e0 + 8 }); // off the grid
    ack({ start: 0xf5e0, end: 0xf5e0 + 40 }); // past what was sent
    assert.deepEqual([fake.stats.badAcks, fake.stats.acksReceived], [5, 0]);
    ack({ start: 0xf5e0, end: 0xf5e0 + 16, resend: [0xf5e0 + 8] });
    assert.deepEqual([fake.stats.badAcks, fake.stats.acksReceived], [5, 1]);
  },

  "receive window: advances in order, lists holes first-hole-first, gives up on old holes together"() {
    const w = new ReceiveWindow(0xfff0);
    assert.deepEqual([w.start, w.end, w.armed], [0xfff0, 0xfff0, false]);
    w.push(0xfff8, 0);
    w.push(0x0000, 0); // wraps
    assert.deepEqual([w.start, w.end], [0x0000, 0x0000]);
    w.push(0x0010, 0); // 0x0008 lost
    w.push(0x0020, 0); // 0x0018 lost
    assert.deepEqual([w.start, w.end], [0x0000, 0x0020]);
    assert.deepEqual(w.missing(), [0x0008, 0x0018]);
    assert.equal(w.push(0x0010, 0), false, "duplicate");
    w.push(0x0008, 5); // resent
    assert.deepEqual([w.start, w.missing()], [0x0010, [0x0018]]);
    assert.equal(w.expire(50, 100), 0, "a young hole is kept");
    assert.deepEqual([w.start, w.missing()], [0x0010, [0x0018]]);
    assert.equal(w.expire(101, 100), 1, "an old one is given up");
    assert.deepEqual([w.start, w.missing()], [0x0020, []]);
    assert.equal(w.push(0x0018, 102), false, "late packet from before the window");
    // a burst: three consecutive packets lost at once are given up together, not one per period
    w.push(0x0040, 200); // 0x28, 0x30, 0x38 lost
    w.push(0x0050, 250); // 0x48 lost later
    assert.deepEqual(w.missing(), [0x0028, 0x0030, 0x0038, 0x0048]);
    assert.equal(w.expire(301, 100), 3);
    assert.deepEqual([w.start, w.end, w.missing()], [0x0040, 0x0050, [0x0048]]);
    assert.equal(w.expire(351, 100), 1);
    assert.deepEqual([w.start, w.holes.size, w.have.size], [0x0050, 0, 0], "nothing left behind");
    // the resend list is capped at 16 and always starts at the first hole
    w.push(0x00f0, 400);
    assert.equal(w.missing().length, 16);
    assert.equal(w.missing()[0], 0x0058);
    assert.deepEqual(w.missing(1), [0x0058]);
    // far ahead: the goggles moved on, start over from there
    w.push(0x8000, 500);
    assert.deepEqual([w.start, w.end, w.missing(), w.holes.size], [0x8000, 0x8000, [], 0]);
  },

  "frame assembler: out-of-order parts, holds a gap briefly, then skips it; 8-bit frame ids wrap"() {
    const a = new FrameAssembler({ holdMs: 50 });
    const part = (frame, total, index, byte) => ({ frame, total, index, payload: new Uint8Array([byte]) });
    assert.deepEqual(a.push(part(7, 2, 1, 2), 0), []);
    assert.deepEqual(a.push(part(7, 2, 0, 1), 1).map((u) => [...u]), [[1, 2]]);
    assert.deepEqual(a.push(part(8, 2, 0, 3), 2), []); // frame 8 part 1 lost
    assert.deepEqual(a.push(part(9, 1, 0, 4), 3), [], "frame 9 waits for 8");
    assert.deepEqual(a.push(part(8, 2, 1, 5), 20).map((u) => [...u]), [[3, 5], [4]], "resent part arrives in time");
    assert.deepEqual(a.push(part(10, 2, 0, 6), 30), []); // part 1 never comes
    assert.deepEqual(a.push(part(11, 1, 0, 7), 31), []);
    assert.deepEqual(a.drain(100).map((u) => [...u]), [[7]], "frame 10 skipped after the hold");
    assert.equal(a.dropped, 1);
    assert.deepEqual(a.push(part(10, 2, 1, 8), 101), [], "late part of a skipped frame is ignored");
    assert.deepEqual(a.push(part(12, 0, 0, 9), 102), [], "no parts: ignored");
    assert.deepEqual(a.push(part(12, 2, 2, 9), 102), [], "index past the count: ignored");
    const b = new FrameAssembler({ holdMs: 50 });
    assert.deepEqual(b.push(part(254, 1, 0, 1), 0).map((u) => [...u]), [[1]]);
    assert.deepEqual(b.push(part(0, 1, 0, 3), 1), [], "frame 0 waits for 255");
    assert.deepEqual(b.push(part(255, 1, 0, 2), 2).map((u) => [...u]), [[2], [3]]);
    assert.deepEqual(b.push(part(1, 1, 0, 4), 3).map((u) => [...u]), [[4]]);
    assert.deepEqual(b.push(part(250, 1, 0, 5), 4), [], "a frame a few back (a late resend) is ignored");
    // numbering that jumps back further than any late resend could be: the stream restarted
    assert.deepEqual(b.push(part(200, 1, 0, 6), 5).map((u) => [...u]), [[6]], "picked up at once");
    assert.deepEqual(b.push(part(201, 1, 0, 7), 6).map((u) => [...u]), [[7]]);
    assert.deepEqual(b.push(part(2, 1, 0, 8), 7), [], "and the old numbering is now behind");
  },

  "finds the RNDIS interfaces whichever class codes the goggles use"() {
    for (const classes of [[0x02, 0x02, 0xff], [0xe0, 0x01, 0x03], [0xef, 0x04, 0x01]]) {
      const f = new FakeGoggles({ classes });
      assert.ok(findRndisInterfaces(f.configurations[0]), classes.join("/"));
    }
  },

  async "streams well past the goggles' send budget and reassembles video byte-exact"() {
    const units = makeUnits(60);
    const fake = new FakeGoggles({ units, fps: 120, serialEvery: 3 });
    const c = await connect(fake);
    assert.deepEqual([...c.mac], [0x02, 0x11, 0x22, 0x33, 0x44, 0x55], "MAC from RNDIS query");
    assert.equal(fake.filter, 0x2d, "packet filter set");
    await until(() => c.got.length >= 60, 10000, "60 video units");
    await finish(c, fake);
    const { session, seed } = c.session;
    assert.equal(hex(fake.firstAck), hex(buildAck(session, { start: seed, end: seed })), "the first ACK is the reference's (seed, seed, 0, seed, seed, 0, seed, seed, 0, 0, 0)");
    assert.deepEqual([fake.lastAck.t3, fake.lastAck.t5], [[seed, seed], [seed, seed]], "type-3/5 windows stay on the seed");
    assert.ok(fake.stats.videoBytes > 5 * fake.budget, `sent ${fake.stats.videoBytes} bytes, several budgets' worth`);
    assert.equal(fake.stats.stalledTicks, 0, "the ACKs kept the budget moving");
    for (let i = 0; i < 60; i++) assert.deepEqual(c.got[i], units[i], `unit ${i}`);
    assert.equal(fake.stats.badAcks, 0, "no malformed ACKs");
    assert.ok(fake.stats.acksReceived > 60);
    assert.deepEqual([c.session.stats.lostPackets, c.session.stats.duplicates, c.session.stats.droppedUnits], [0, 0, 0]);
    assert.equal(c.session.stats.serial, FAKE_SERIAL, "serial from the goggles' DUML");
    assert.ok(c.session.stats.outstanding <= 2, `outstanding ${c.session.stats.outstanding}`);
    assert.ok(c.states.includes("live"));
  },

  async "big datagrams arrive as IP fragments and are reassembled"() {
    const units = makeUnits(10);
    const fake = new FakeGoggles({ units, fps: 120, partSize: 8000 });
    const c = await connect(fake);
    await until(() => c.got.length >= 10, 8000, "10 video units");
    await finish(c, fake);
    for (let i = 0; i < 10; i++) assert.deepEqual(c.got[i], units[i], `unit ${i}`);
    assert.ok(fake.stats.ipFragments > 50, "IP fragmentation was exercised");
  },

  async "recovers lost packets by asking for resends, first hole first"() {
    const units = makeUnits(90);
    const fake = new FakeGoggles({ units, fps: 120, lossRate: 0.03 });
    const c = await connect(fake);
    await until(() => c.got.length >= 90, 15000, "90 video units");
    await finish(c, fake);
    assert.ok(fake.stats.lost > 20, `${fake.stats.lost} packets lost on the way`);
    assert.ok(fake.stats.resent >= fake.stats.lost * 0.9, `resent ${fake.stats.resent} of ${fake.stats.lost}`);
    assert.equal(fake.stats.ignoredResendLists, 0, "every resend list started at the first hole");
    assert.equal(fake.stats.badAcks, 0);
    assert.equal(c.session.stats.droppedUnits, 0, "no frames given up");
    assert.equal(c.session.stats.lostPackets, 0, "no holes given up");
    for (let i = 0; i < 90; i++) assert.deepEqual(c.got[i], units[i], `unit ${i}`);
  },

  async "ACKs stay well under a few hundred per second at 60 fps with loss"() {
    const fake = new FakeGoggles({ units: makeUnits(120), fps: 60, lossRate: 0.03 });
    const c = await connect(fake);
    await until(() => c.got.length >= 5, 5000, "first units");
    const acks0 = fake.stats.acksReceived;
    const t0 = Date.now();
    await sleep(1500);
    const rate = ((fake.stats.acksReceived - acks0) * 1000) / (Date.now() - t0);
    await finish(c, fake);
    assert.ok(rate < 400, `${rate.toFixed(0)} ACKs/s`);
    assert.ok(rate > 60, `${rate.toFixed(0)} ACKs/s: at least one per frame`);
  },

  async "goggles that never resend: lost frames are skipped and the stream keeps flowing"() {
    const units = makeUnits(150); // more than the run consumes, so the fake never cycles back to unit 0
    const fake = new FakeGoggles({ units, fps: 60, lossRate: 0.03, resend: false });
    const c = await connect(fake);
    const t0 = Date.now();
    await until(() => c.got.length + c.session.stats.droppedUnits >= 119, 6000, "the stream to get through");
    const took = Date.now() - t0;
    await finish(c, fake);
    assert.equal(fake.stats.resent, 0);
    assert.ok(fake.stats.resendsAsked > 0, "we did ask");
    assert.equal(fake.stats.ignoredResendLists, 0, "every resend list started at the first hole");
    assert.ok(c.session.stats.droppedUnits > 5, `${c.session.stats.droppedUnits} frames dropped`);
    assert.ok(c.session.stats.lostPackets >= fake.stats.lost * 0.8, `gave up on ${c.session.stats.lostPackets} of ${fake.stats.lost} lost packets`);
    assert.ok(c.got.length >= 50, `${c.got.length} frames still delivered`);
    assert.ok(took < 3500, `took ${took} ms for 2 s of video: holes must not hold the goggles' budget for long`);
    assert.ok(fake.stats.stalledTicks < 80, `goggles stalled ${fake.stats.stalledTicks} ticks`);
    assertInOrder(c.got, units);
  },

  async "without our ACKs the goggles stop after their budget (so the ACKs matter)"() {
    const fake = new FakeGoggles({ units: makeUnits(30), fps: 120 });
    const c = await connect(fake);
    await until(() => c.got.length >= 5, 5000, "first units");
    c.net.bindUdp(9003, () => {}); // stop answering anything
    c.session.stop();
    await sleep(800);
    const sent = fake.stats.videoSent;
    await sleep(500);
    assert.equal(fake.stats.videoSent, sent, "goggles stopped sending video");
    assert.ok(fake.stats.stalledTicks > 20);
    assert.ok(fake.unacked > fake.budget - 60000 && fake.unacked <= fake.budget, `${fake.unacked} bytes outstanding`);
    fake.stopStreaming();
    await c.link.close();
  },

  async "video pause: the window falls back to the seed like the reference, and video resumes cleanly"() {
    const units = makeUnits(30);
    const fake = new FakeGoggles({ units, fps: 60 });
    const c = await connect(fake, { noVideoAfter: 400 });
    await until(() => c.got.length >= 5, 5000, "first units");
    fake.pauseVideo();
    await until(() => c.session.stats.stalls === 1, 3000, "the stall");
    assert.equal(c.session.state, "no-video");
    await sleep(150); // a window report and its ACK
    assert.deepEqual([fake.lastAck.start, fake.lastAck.end, fake.lastAck.list], [fake.seed, fake.seed, []], "ACKs carry the seed windows while stalled");
    assert.ok(c.session.stats.acks > 0);
    const before = c.got.length;
    fake.pauseVideo(false);
    await until(() => c.got.length >= before + 10, 5000, "video after the pause");
    await finish(c, fake);
    assert.equal(c.session.state, "live");
    assert.equal(fake.stats.stalledTicks, 0, "the goggles' budget was never confused by the seed ACKs");
    assert.deepEqual([fake.stats.badAcks, c.session.stats.droppedUnits], [0, 0]);
    for (let i = 0; i < c.got.length; i++) assert.deepEqual(c.got[i], units[i], `unit ${i}`);
  },

  async "reconnects with a new session when the goggles go quiet"() {
    const fake = new FakeGoggles({ units: makeUnits(10), fps: 60 });
    const c = await connect(fake, { lostAfter: 800, noVideoAfter: 400 });
    await until(() => c.got.length >= 3, 5000, "first units");
    const firstSession = fake.session;
    fake.stopStreaming(); // goggles go silent
    await until(() => fake.stats.openers >= 2, 4000, "a new handshake");
    await until(() => c.got.length >= 10, 5000, "video after reconnect");
    assert.notEqual(fake.session, firstSession, "new session id");
    assert.ok(c.states.includes("lost"));
    await finish(c, fake);
  },

  async "links on the goggles' first report or video when their handshake answer is lost"() {
    // reference client and SquirrelReceiver: repeat type-0 until a type-1/2 for our session arrives
    const units = makeUnits(20);
    const fake = new FakeGoggles({ units, fps: 60, answer: false });
    const c = await connect(fake);
    await until(() => c.got.length >= 10, 5000, "video without a handshake answer");
    await finish(c, fake);
    assert.equal(c.session.state, "live");
    assert.ok(fake.stats.openers <= 2, `${fake.stats.openers} handshakes`);
    assert.ok(!c.states.includes("no-answer"));
    assert.equal(fake.stats.badAcks, 0);
    for (let i = 0; i < 10; i++) assert.deepEqual(c.got[i], units[i], `unit ${i}`);
    const [start, end] = c.session.stats.theirWindow;
    assert.ok(ahead(end, start) >= 0 && ahead(fake.seq, end) >= 0 && ahead(start, fake.seed) > 0, `the goggles' own window is reported: ${start}..${end}`);
  },

  async "a resend that fills a hole is ACKed promptly even when its frame was already skipped"() {
    const { net, s, sealed } = bench({ holdMs: 20, giveUpMs: 500 });
    assert.equal(s.session & 1, 1, "odd session id like the reference client");
    const video = (seq, frame, total, index, byte) => sealed([0, 0, 0, 0, seq & 255, seq >> 8, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, frame, (total & 0x7f) | ((index & 1) << 7), (index >> 1) & 0x1f, 0, byte]);
    const acks = () => net.sent.filter((p) => p[6] === 4).length;
    const last = () => {
      const p = net.sent.at(-1);
      return { start: p[8] | (p[9] << 8), end: p[10] | (p[11] << 8), n: p[12] | (p[13] << 8) };
    };
    await sleep(15);
    net.h(null, 9003, video(0x08, 1, 3, 0, 1));
    net.h(null, 9003, video(0x18, 1, 3, 2, 3)); // 0x10 lost; the last part of the frame asks for it
    assert.deepEqual([acks(), last()], [2, { start: 0x08, end: 0x18, n: 1 }]);
    await sleep(15);
    net.h(null, 9003, video(0x20, 2, 1, 0, 4)); // complete, but waits behind frame 1
    assert.deepEqual([acks(), s.stats.units], [3, 0]);
    await until(() => s.stats.units === 1, 1000, "frame 2 after frame 1 is skipped");
    assert.deepEqual([s.stats.droppedUnits, acks()], [1, 3]);
    net.h(null, 9003, video(0x10, 1, 3, 1, 2)); // the resend: too late for its frame, but it frees the goggles' budget
    assert.deepEqual([acks(), last(), s.stats.units], [4, { start: 0x20, end: 0x20, n: 0 }, 1]);
    s.stop();
  },

  "malformed, foreign-session and post-stop datagrams are ignored without side effects"() {
    const { net, s, sealed } = bench();
    assert.equal(s.state, "live");
    assert.equal(net.sent.length, 2, "handshake, then the first ACK");
    const snap = () => [s.stats.acks, s.stats.videoPackets, s.window.start, s.window.end];
    const before = snap();
    s.lastAny = 0;
    let r = 1;
    const rand = () => ((r = (r * 1103515245 + 12345) & 0x7fffffff) & 255);
    for (let len = 0; len < 64; len++) net.h(null, 9003, Uint8Array.from({ length: len }, rand)); // noise
    net.h(null, 9003, sealed(new Array(34).fill(0).map((_, i) => (i === 6 ? 0x01 : 0)), s.session ^ 1)); // another session's report
    net.h(null, 9003, sealed([0, 0, 0, 0, 0, 0, 0x00, 0, 0x01], s.session ^ 1)); // another session's handshake answer
    const bad = sealed(new Array(34).fill(0).map((_, i) => (i === 6 ? 0x01 : 0)));
    bad[7] ^= 1; // XOR wrong
    net.h(null, 9003, bad);
    const long = sealed(new Array(34).fill(0).map((_, i) => (i === 6 ? 0x01 : 0)));
    long[0] = 40; // length field disagrees with the datagram
    net.h(null, 9003, long);
    assert.deepEqual(snap(), before, "nothing changed");
    assert.equal(s.lastAny, 0, "none of it counts as hearing from the goggles");
    net.h(null, 9003, sealed([0, 0, 0, 0, 0, 0, 0x02, 0, 0, 0, 0, 0])); // ours, but a video header cut short
    assert.deepEqual(snap(), before, "ignored");
    assert.ok(s.lastAny > 0, "though it does count as hearing from them");
    // short but valid window report (the reference accepts 12 bytes): ACKed, DUML length beyond the end: fine
    net.h(null, 9003, sealed([0, 0, 0, 0, 0, 0, 0x01, 0, 1, 2, 3, 4]));
    net.h(null, 9003, sealed([0, 0, 0, 0, 0, 0, 0x01, 0, ...new Array(24).fill(0), 0xff, 0xff, 0x55, 0x55]));
    assert.equal(s.stats.acks, 3);
    // video with no parts or an index past the count arms the window but yields nothing
    net.h(null, 9003, sealed([0, 0, 0, 0, 0x08, 0, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0x00, 0x00, 0, 0xaa]));
    net.h(null, 9003, sealed([0, 0, 0, 0, 0x10, 0, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0x82, 0x1f, 0, 0xaa]));
    assert.deepEqual([s.window.armed, s.window.start, s.window.end, s.stats.videoPackets, s.stats.units], [true, 0x0010, 0x0010, 2, 0]);
    s.stop();
    net.h(null, 9003, sealed(new Array(34).fill(0).map((_, i) => (i === 6 ? 0x01 : 0))));
    assert.equal(s.stats.acks, 3, "no ACKs after stop()");
  },

  "H.264 inspection: NAL types, IDR, codec string"() {
    const [idr, p] = makeUnits(2);
    const a = describeUnit(idr);
    assert.deepEqual(a.types, [7, 8, 5]);
    assert.ok(a.idr && a.sps && a.pps);
    assert.equal(codecString(idr, a.sps), "avc1.64002a");
    const b = describeUnit(p);
    assert.deepEqual(b.types, [1]);
    assert.ok(!b.idr);
    assert.equal(nalUnits(p).length, 1);
  },
};

// ---------------------------------------------------------------- the Goggles 3's framing, headless

// Grey pictures on a stand-in 2D canvas: drawImage samples each output pixel's box in the source (3 x 3 points, as a
// canvas scales down), a same-size draw copies; getImageData gives RGBA. Enough for perception.js's picture finder, flow
// and crops; toDataURL says what was drawn.
class FakeImage {
  constructor(w, h, g = new Uint8Array(w * h)) { Object.assign(this, { width: w, height: h, displayWidth: w, displayHeight: h, g }); }
  close() { this.closed = true; }
}
class FakeCanvas {
  constructor(w = 300, h = 150) { Object.assign(this, { _w: w, _h: h, g: new Uint8Array(w * h), drawn: [] }); }
  get width() { return this._w; }
  set width(v) { this._w = v; this.g = new Uint8Array(this._w * this._h); }
  get height() { return this._h; }
  set height(v) { this._h = v; this.g = new Uint8Array(this._w * this._h); }
  getContext() { return (this.ctx ??= new FakeCtx(this)); }
  toDataURL() { return `data:image/jpeg;base64,${this.width}x${this.height}`; }
}
class FakeCtx {
  constructor(canvas) { this.canvas = canvas; }
  drawImage(img, ...a) {
    const [sx, sy, sw, sh, dx, dy, dw, dh] = a.length === 4 ? [0, 0, img.width, img.height, ...a] : a, c = this.canvas, W = img.width, src = img.g;
    c.drawn.push({ sx, sy, sw, sh, dw, dh });
    if (sw === dw && sh === dh) {
      for (let y = 0; y < dh; y++) c.g.set(src.subarray((sy + y) * W + sx, (sy + y) * W + sx + sw), (dy + y) * c.width + dx);
      return;
    }
    for (let y = 0; y < dh; y++) {
      const y0 = sy + (y * sh) / dh, fy = sh / dh / 3;
      for (let x = 0; x < dw; x++) {
        const x0 = sx + (x * sw) / dw, fx = sw / dw / 3;
        let s = 0;
        for (let j = 0; j < 3; j++) for (let i = 0; i < 3; i++) s += src[Math.min(img.height - 1, Math.floor(y0 + (j + 0.5) * fy)) * W + Math.min(W - 1, Math.floor(x0 + (i + 0.5) * fx))];
        c.g[(dy + y) * c.width + dx + x] = s / 9;
      }
    }
  }
  getImageData(x, y, w, h) {
    const c = this.canvas, data = new Uint8ClampedArray(w * h * 4);
    for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) { const v = c.g[(y + j) * c.width + x + i], k = 4 * (j * w + i); data[k] = data[k + 1] = data[k + 2] = v; data[k + 3] = 255; }
    return { data, width: w, height: h };
  }
}

// The Goggles 3 stream: 1920x1080, the O4's 4:3 picture at x0 (textured, panning; its left `darkLeft` px stay dark),
// black bars beside it, white OSD text over the bars and the picture.
// wide: the O4 at 16:9 (the scene fills the stream, no bars).
function goggleFrames({ x0 = 240, darkLeft = 0, wide = false } = {}) {
  const W = 1920, H = 1080, TW = 2600, tex = new Uint8Array(TW * H);
  let seed = 17;
  const rnd = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
  for (let y = 0; y < H; y++) for (let x = 0; x < TW; x++) tex[y * TW + x] = 110 + 70 * Math.sin(x / 23) * Math.cos(y / 31) + 30 * rnd();
  const osd = [[40, 30, 170, 36], [900, 30, 120, 36], [1710, 30, 170, 36], [40, 1010, 170, 36], [1700, 1010, 180, 36], [1690, 200, 100, 400], [1310, 840, 480, 60]];
  return (k) => {
    const g = new Uint8Array(W * H), off = (k * 7) % (TW - 1920);
    if (wide) for (let y = 0; y < H; y++) g.set(tex.subarray(y * TW + off, y * TW + off + W), y * W);
    else for (let y = 0; y < H; y++) {
      g.set(tex.subarray(y * TW + off, y * TW + off + 1440), y * W + x0);
      if (darkLeft) g.fill(3, y * W + x0, y * W + x0 + darkLeft);
      for (let x = 0; x < x0; x += 97) g[y * W + x] = 3;
    }
    for (const [ox, oy, w, h] of osd) for (let y = oy; y < oy + h; y++) for (let x = ox; x < ox + w; x++) if ((x + y) % 3) g[y * W + x] = 235;
    return new FakeImage(W, H, g);
  };
}

// The app's goggles source and perception on a simulated clock, `seconds` of 30 fps video.
async function framingRun({ seconds = 8, frameAt: custom = null, ...frames } = {}) {
  let clock = 1000;
  const realPerf = globalThis.performance, perf = { now: () => clock, timeOrigin: realPerf.timeOrigin };
  Object.defineProperty(globalThis, "performance", { value: perf, configurable: true, writable: true });
  globalThis.document ??= { createElement: () => new FakeCanvas() };
  globalThis.OffscreenCanvas ??= FakeCanvas;
  try {
    const { GogglesSource } = await import("../app/js/goggles/source.js"), { Perception } = await import("../app/js/perception.js"), { pictureProblem } = await import("../app/js/vision/depth.js");
    const g = new GogglesSource(), per = new Perception(), seen = [], events = [], frameAt = custom ?? goggleFrames(frames);
    g.status = "live";
    per.detector = { ready: true, loading: null, on() {}, load: async () => {}, detect: (src, w, h, o) => (seen.push({ w, h, region: o.region, element: o.element }), []) };
    per.setSource(g);
    const find = per.findPicture.bind(per);
    per.looks = [];
    per.findPicture = (src) => (per.looks.push(per.frameCount), find(src));
    per.on("frame", (f) => events.push({ t: f.t, decoded: f.decoded, region: f.region, picture: f.picture, w: per.latest.width, problem: pictureProblem(per) }));
    const timeline = [];
    for (let k = 0; k < seconds * 30; k++) {
      clock += 1000 / 30;
      g.paint(frameAt(k), clock - 4 - (k % 3)); // decoded a few ms before it is painted
      per.process();
      timeline.push({ t: (k + 1) / 30, region: g.region(), state: per.pictureState() });
    }
    return { g, per, seen, events, timeline, pictureProblem };
  } finally {
    Object.defineProperty(globalThis, "performance", { value: realPerf, configurable: true, writable: true });
  }
}

const PIC = { sx: 240, sy: 0, sw: 1440, sh: 1080 };

tests["Goggles 3 framing: the O4's 4:3 picture pillarboxed in 1920x1080 (OSD on the bars) is found within 6 s and held; flow, the detector, depth's check and Claude's snapshot use the 1440 px picture; decodedAt is monotonic and never ahead"] = async () => {
  const { g, per, seen, events, timeline, pictureProblem } = await framingRun();
  const first = timeline.find((s) => JSON.stringify(s.region) === JSON.stringify(PIC)), after = timeline.filter((s) => s.t >= first?.t);
  console.log(`      found ${JSON.stringify(g.region())} after ${first?.t.toFixed(2)} s; regions after that: ${new Set(after.map((s) => JSON.stringify(s.region))).size}; problems seen: ${[...new Set(events.map((e) => e.problem))].map((p) => p || "none").join(" | ")}`);
  assert.ok(first && first.t <= 6, `the picture within 6 s (${first?.t})`);
  assert.ok(after.every((s) => JSON.stringify(s.region) === JSON.stringify(PIC) && s.state === "found"), "held steady once found");
  assert.deepEqual(g.baseRegion(), { sx: 0, sy: 0, sw: 1920, sh: 1080 });
  assert.match(events[0].problem, /checking the video for black bars/);
  assert.equal(pictureProblem(per), "");
  assert.deepEqual([per.latest.width, per.latest.height], [1440, 1080], "flow's work frame is the picture");
  assert.deepEqual(per.work.drawn.at(-1), { ...PIC, dw: 320, dh: 240 });
  assert.deepEqual(seen.at(-1).region, PIC, "the detector's crop");
  const ev = events.at(-1);
  assert.ok(ev.picture && JSON.stringify(ev.region) === JSON.stringify(PIC));
  assert.ok(events.every((e, i) => e.decoded <= e.t && (i === 0 || e.decoded >= events[i - 1].decoded)), "decodedAt: never ahead of the frame, monotonic");
  const gaps = per.looks.slice(1).map((f, i) => f - per.looks[i]);
  assert.ok(gaps.slice(0, 20).every((d) => d === 5) && gaps.at(-1) === 30, `looked every 5 frames, then every 30 once settled: ${gaps.join(" ")}`);
  const shot = [], ce = document.createElement;
  g.lastPaint = performance.now(); // the run's clock is gone: the video is live now
  document.createElement = () => { const c = new FakeCanvas(); shot.push(c); return c; };
  try { per.snapshot({ maxWidth: 720 }); } finally { document.createElement = ce; }
  assert.deepEqual([shot[0].width, shot[0].height, shot[0].drawn[0].sx, shot[0].drawn[0].sw], [720, 540, 240, 1440], "Claude's snapshot is the picture");
};

tests["Goggles 3 framing: a picture whose left edge stays dark is still placed centred (not slid by the dark strip); a user crop the picture doesn't fit in wins; the held picture moves only after the finder agrees for a while"] = async () => {
  const { g, per } = await framingRun({ seconds: 6, darkLeft: 150 });
  console.log(`      dark left 150 px: ${JSON.stringify(g.region())}`);
  assert.deepEqual(g.region(), PIC);
  g.crop = { x: 0.5, y: 0, w: 0.5, h: 1 };
  assert.deepEqual(g.region(), { sx: 960, sy: 0, sw: 960, sh: 1080 }, "the crop, which the picture doesn't fit in");
  g.crop = null;
  const { PICTURE } = await import("../app/js/perception.js"), A = per.applied, moved = { ...A, sx: A.sx + 10 }, near = { ...A, sx: A.sx + 3 };
  assert.equal(per.steady(near), A);
  const got = Array.from({ length: PICTURE.hold.n }, () => per.steady(moved));
  assert.ok(got.slice(0, -1).every((p) => p === A) && got.at(-1) === moved, "moves on the n-th disagreeing check");
  assert.equal(per.steady(null), per.applied, "a dark spell keeps the picture");
  // decodedAt from the worker's absolute time; a stamp from the future is clamped
  g.paint(new FakeImage(1920, 1080), performance.now() + 50);
  assert.ok(g.decodedAt() <= performance.now());
};

tests["Goggles 3 framing: when the stream's layout changes (the O4 switched between 4:3 and 16:9) the held picture is let go within a few seconds, not after its long history (16-22 s); a crop drawn on the shown picture is stored in stream fractions"] = async () => {
  const boxed = goggleFrames(), wide = goggleFrames({ wide: true }), sw = [10, 20], frameAt = (k) => ((k < sw[0] * 30 || k >= sw[1] * 30) ? boxed : wide)(k);
  const { g, timeline } = await framingRun({ seconds: 30, frameAt });
  const when = (from, test) => timeline.find((s) => s.t > from && test(s))?.t - from;
  const toWhole = when(sw[0], (s) => s.state === "whole" && s.region.sw === 1920), back = when(sw[1], (s) => s.state === "found" && JSON.stringify(s.region) === JSON.stringify(PIC));
  const stale = timeline.filter((s) => s.t > sw[0] && s.t < sw[0] + toWhole && s.state === "found").length / 30;
  console.log(`      4:3 -> 16:9: the whole stream after ${toWhole?.toFixed(1)} s (the old picture kept ${stale.toFixed(1)} s); 16:9 -> 4:3: the picture after ${back?.toFixed(1)} s`);
  assert.ok(toWhole <= 7 && stale <= 2.5, `16:9 noticed in ${toWhole} s, old crop for ${stale} s`);
  assert.ok(back <= 7, `4:3 again in ${back} s`);
  // the HUD shows region() (the picture once found): a box drawn over its right half is stored in the stream's fractions
  const c = g.cropFromView({ x: 0.5, y: 0, w: 0.48, h: 1 });
  assert.deepEqual([c.x * 1920, c.w * 1920, c.y, c.h].map((v) => +v.toFixed(1)), [960, 691.2, 0, 1]);
};

let failed = 0;
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    const where = e.stack?.split("\n").find((l) => l.includes("test-goggles.mjs")) || "";
    console.log(`FAIL  ${name}\n      ${e.message.split("\n").join("\n      ")}\n      ${where.trim()}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed`);
process.exit(failed ? 1 : 0);
