// Decodes the goggles' H.264 units with WebCodecs (hardware on the Mac) and hands out VideoFrames.
// A real IDR or all-intra frame is the best start (an intra frame gets a recovery-point SEI so Chrome
// accepts it). The goggles' stream has neither, it is intra refresh only, so it normally starts on a
// synthetic gray picture instead (bootstrap.js) that the refresh wave repaints within a second or
// two. The bootstrap variants are tried in turn, first on the default (hardware) decoder, then on
// Chrome's software one, which is more forgiving.
import { describeUnit, codecString, nalBytes, concat, RECOVERY_SEI } from "./h264.js";
import { bootstrapUnits } from "./bootstrap.js";

const BACKLOG = 40; // queued chunks, about 2/3 s at 60 fps. decodeQueueSize only drains between tasks, and one helper read or assembler drain can push up to 32 units at once.
const PATIENCE = 30; // units after a bootstrap with at most the gray picture out: the decoder eats the stream silently, next variant

const same = (a, b) => !!a && !!b && a.length === b.length && a.every((x, i) => x === b[i]);

export class H264Player {
  // onFrame(VideoFrame) must close the frame.
  constructor({ onFrame, onState = () => {} }) {
    this.onFrame = onFrame;
    this.onState = onState;
    this.decoder = null;
    this.state = "waiting"; // waiting for a starting frame | running
    this.sps = null;
    this.pps = null;
    this.codec = null;
    this.ts = 0;
    this.attempts = null; // bootstrap starts for the current SPS/PPS, best first: [{ name, software }] (pictures are rebuilt for each start)
    this.attempt = 0; // the one in use, or the next to try
    this.startKind = null; // "stream" (IDR, intra or recovery point) | "bootstrap"
    this.startUnits = 0; // units fed since the last start
    this.startDecoded = 0; // frames out since the last start
    // start: "idr" | "intra" | "recovery" | "bootstrap-<variant>" | "bootstrap-software" | "failed"
    this.stats = { units: 0, keyframes: 0, idr: 0, intra: 0, decoded: 0, errors: 0, waitingUnits: 0, skipped: 0, start: null, variant: null };
    this.lastError = null;
  }

  setState(s) {
    if (s === this.state) return;
    this.state = s;
    this.onState(s);
  }

  push(unit) {
    this.stats.units++;
    const info = describeUnit(unit);
    if (info.sps) this.setParams("sps", nalBytes(unit, info.sps), codecString(unit, info.sps));
    if (info.pps) this.setParams("pps", nalBytes(unit, info.pps));
    if (info.idr) this.stats.idr++;
    if (info.intra && !info.idr) this.stats.intra++;
    this.ts += 16667;
    if (!info.slices) return; // parameter sets alone: kept above; as a chunk they make the software decoder fail

    if (this.state === "running" && this.decoder?.state === "configured") {
      if (this.decoder.decodeQueueSize > BACKLOG) {
        // Decoding fell behind: drop frames until the next place we can restart cleanly.
        this.stats.skipped++;
        if (info.idr || info.intra) this.restartAt(unit, info);
        else this.setState("waiting");
        return;
      }
      if (!(this.startKind === "bootstrap" && this.startDecoded < 2 && this.startUnits >= PATIENCE)) {
        this.startUnits++;
        this.decode(unit, info.idr ? "key" : "delta");
        return;
      }
      this.nextAttempt(`no video after ${PATIENCE} frames`);
    }
    if (!this.sps || !this.pps) {
      this.stats.waitingUnits++;
      return;
    }
    if (info.idr || info.intra || info.recovery) this.restartAt(unit, info);
    else this.bootstrap(unit, info);
  }

  setParams(key, nal, codec) {
    if (same(nal, this[key])) {
      if (this.stats.start === "failed") this.retryStart(); // stream resent its parameter sets (sharing toggled, battery replugged): try again
      return;
    }
    if (this.state === "running") this.setState("waiting"); // the decoder only sees slices, so restart it on the new sets
    this[key] = nal;
    if (codec) this.codec = codec;
    this.attempts = null; // new parameter sets: new bootstrap pictures, and the hardware gets another go
    this.attempt = 0;
  }

  restartAt(unit, info) {
    this.configure("no-preference");
    this.startKind = "stream";
    this.stats.start = info.idr ? "idr" : info.intra ? "intra" : "recovery";
    this.stats.variant = null;
    const params = concat(info.sps ? new Uint8Array() : this.sps, info.pps ? new Uint8Array() : this.pps);
    const start = info.idr || info.recovery ? concat(params, unit) : concat(params, RECOVERY_SEI, unit);
    this.stats.keyframes++;
    this.setState("running");
    this.startUnits++;
    this.decode(start, "key");
  }

  // No real starting frame: put a gray picture of our own in front of this P frame.
  bootstrap(unit, info) {
    if (this.attempts && !this.attempts[this.attempt]) {
      this.stats.start = "failed";
      this.stats.waitingUnits++;
      return;
    }
    // Built for this P frame every time: a cached picture's frame_num would sit behind the stream and leave a gap.
    let variants = [];
    try {
      variants = bootstrapUnits(this.sps, this.pps, nalBytes(unit, info.firstSlice));
    } catch (e) {
      this.lastError = `Can't build a bootstrap picture for this stream: ${e.message}`;
    }
    this.attempts ??= [false, true].flatMap((software) => variants.map(({ name }) => ({ name, software })));
    const a = this.attempts[this.attempt];
    const picture = a && variants.find((v) => v.name === a.name);
    if (!picture) {
      this.stats.start = "failed";
      this.stats.waitingUnits++;
      return;
    }
    this.configure(a.software ? "prefer-software" : "no-preference");
    this.startKind = "bootstrap";
    this.stats.start = a.software ? "bootstrap-software" : `bootstrap-${a.name}`;
    this.stats.variant = a.name;
    this.stats.keyframes++;
    this.setState("running");
    this.decode(picture.data, "key", this.ts - 16667);
    if (this.state !== "running") return;
    this.startUnits++;
    this.decode(unit, "delta");
  }

  // The bootstrap in use didn't take: on to the next variant, then the software decoder.
  nextAttempt(why) {
    this.attempt++;
    this.lastError = why; // once none is left, stats.start turns "failed" on the next unit
    this.setState("waiting");
  }

  // Back to the first bootstrap variant: on every close() (new connection), on parameter sets
  // resent after a failure, and when the pipeline restarts the session.
  retryStart() {
    this.attempts = null; // rebuilt from the next P frame, so the gray picture's frame_num precedes it
    this.attempt = 0;
    if (this.stats.start === "failed") this.stats.start = null;
  }

  fail(msg) {
    this.stats.errors++;
    if (this.startKind === "bootstrap" && this.startDecoded < 2) return this.nextAttempt(msg);
    this.lastError = msg;
    this.setState("waiting");
  }

  configure(hardwareAcceleration) {
    if (this.decoder && this.decoder.state !== "closed") this.decoder.close();
    this.startUnits = this.startDecoded = 0;
    this.decoder = new VideoDecoder({
      output: (frame) => {
        this.stats.decoded++;
        this.startDecoded++;
        this.onFrame(frame);
      },
      error: (e) => this.fail(String(e?.message || e)),
    });
    this.decoder.configure({ codec: this.codec || "avc1.640028", optimizeForLatency: true, hardwareAcceleration });
  }

  decode(data, type, timestamp = this.ts) {
    try {
      this.decoder.decode(new EncodedVideoChunk({ type, timestamp, data }));
    } catch (e) {
      this.fail(String(e?.message || e));
    }
  }

  close() {
    try {
      if (this.decoder && this.decoder.state !== "closed") this.decoder.close();
    } catch {}
    this.decoder = null;
    this.startKind = null;
    this.retryStart();
    this.stats.start = this.stats.variant = null;
    this.setState("waiting");
  }
}
