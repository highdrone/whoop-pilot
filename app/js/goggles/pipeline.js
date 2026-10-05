// The whole goggles video chain, independent of where it runs (a worker for real devices, the
// page for tests): USB (RNDIS) or the helper's HTTP stream -> LiveView session -> H.264 decoder.
import { RndisLink } from "./rndis.js";
import { NetStack } from "./netstack.js";
import { LiveviewSession, HOST_IP } from "./liveview.js";
import { H264Player } from "./player.js";

// onFrame(VideoFrame) must close the frame. onEvent({ type: "state"|"log"|"error", ... }).
export class GogglesPipeline {
  constructor({ onFrame, onEvent }) {
    this.onEvent = onEvent;
    this.player = new H264Player({ onFrame, onState: () => this.report() });
    this.state = "off";
  }

  setState(s) {
    this.state = s;
    this.report();
  }

  report() {
    this.onEvent({ type: "state", state: this.state, decoder: this.player.state, stats: this.stats() });
  }

  stats() {
    return { ...(this.session?.stats || this.helperStats || {}), player: { ...this.player.stats, lastError: this.player.lastError } };
  }

  async startUsb(device) {
    this.stop();
    this.setState("connecting");
    const link = new RndisLink(device, { log: (msg) => this.onEvent({ type: "log", msg }) });
    this.link = link;
    const { mac } = await link.open();
    const net = new NetStack({ mac, ip: HOST_IP, sendFrame: (f) => link.sendFrame(f), log: (msg) => this.onEvent({ type: "log", msg }) });
    link.start(
      (f) => net.handleFrame(f),
      (e) => this.fail(`Goggles USB link lost: ${e.message}`),
    );
    net.announce();
    const session = new LiveviewSession(net);
    session.on("state", (s) => this.setState(s));
    session.on("log", (msg) => this.onEvent({ type: "log", msg }));
    session.on("unit", (u) => {
      this.player.push(u);
      this.kickIfStuck();
    });
    this.session = session;
    this.kicks = 0;
    this.waitingSince = Date.now();
    session.start();
  }

  // Video arrives but the player has nothing to start from: no parameter sets yet (a fresh session
  // makes the goggles resend them) or every bootstrap picture was refused. A few tries after 5 s of
  // that, then leave it to the user (Connect goggles, or replug the drone battery). Bootstrapping
  // itself is the player's job and takes a moment, so it doesn't count as stuck; nor does the
  // stream's lack of keyframes. Through the helper there is no session of ours to restart, so only
  // the start pictures are retried, and missing parameter sets are just waited for.
  kickIfStuck() {
    const now = Date.now();
    const p = this.player;
    const failed = p.stats.start === "failed";
    if (p.state === "running" || (p.sps && p.pps && !failed) || (!this.session && !failed)) {
      this.waitingSince = now;
      return;
    }
    if (now - this.waitingSince < 5000 || this.kicks >= 3) return;
    this.kicks++;
    this.waitingSince = now;
    const why = failed ? "the decoder refused every bootstrap picture" : "no SPS/PPS yet";
    this.onEvent({ type: "log", msg: `Video arrives but ${why}; ${this.session ? "restarting the LiveView session" : "trying the start pictures again"} (try ${this.kicks}/3).` });
    p.retryStart();
    this.session?.start();
  }

  // Helper stream from tools/whoop.mjs (/goggles/<token>/stream): [type u8][length u32le][payload]; 1 = H.264 unit, 2 = JSON status.
  async startHelper(url) {
    this.stop();
    this.setState("connecting");
    this.kicks = 0;
    this.waitingSince = Date.now();
    const ac = new AbortController();
    this.helperAbort = ac;
    let res;
    try {
      res = await fetch(`${url}/stream`, { signal: ac.signal });
    } catch {
      this.fail("Whoop Pilot's server isn't running: double-click start.command.");
      return;
    }
    const reader = res.body.getReader();
    let buf = new Uint8Array(0);
    try {
      for (;;) {
        const { value, done } = await reader.read();
        if (done || ac.signal.aborted) break;
        const merged = new Uint8Array(buf.length + value.length);
        merged.set(buf);
        merged.set(value, buf.length);
        buf = merged;
        let off = 0;
        while (buf.length - off >= 5) {
          const type = buf[off];
          const len = (buf[off + 1] | (buf[off + 2] << 8) | (buf[off + 3] << 16) | (buf[off + 4] << 24)) >>> 0;
          if (buf.length - off - 5 < len) break;
          const payload = buf.subarray(off + 5, off + 5 + len);
          off += 5 + len;
          if (type === 1) {
            this.player.push(payload.slice());
            this.kickIfStuck();
          } else if (type === 2) {
            const msg = JSON.parse(new TextDecoder().decode(payload));
            this.helperStats = msg.stats;
            if (msg.state && msg.state !== this.state) this.setState(msg.state);
          }
        }
        buf = buf.slice(off);
      }
    } catch (e) {
      if (!ac.signal.aborted) this.fail(`Goggles helper stream ended: ${e.message}`);
      return;
    }
    if (!ac.signal.aborted) this.fail("Goggles helper stream ended.");
  }

  fail(msg) {
    this.stop();
    this.state = "error";
    this.onEvent({ type: "error", msg });
    this.report();
  }

  stop() {
    this.session?.stop();
    this.link?.close();
    this.helperAbort?.abort();
    this.session = this.link = this.helperAbort = null;
    this.state = "off"; // before the player's close() reports, so nothing stale from this session goes out
    this.player.close();
  }
}
