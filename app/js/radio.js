// USB link to the radio: the TX15 shows up as a USB serial port (EdgeTX "USB Serial (VCP)") and the
// aibrg.lua mixer script on the radio reads our commands and writes telemetry back. Chrome/Edge only.
import { Emitter } from "./util.js";
import { encodeCommand, decodeTelemetry, explainTelemetry } from "./protocol.js";

const encoder = new TextEncoder();

export class SerialRadio extends Emitter {
  constructor() {
    super();
    this.kind = "serial";
    this.port = null;
    this.writer = null;
    this.reader = null;
    this.telemetry = null;
    this.writing = false;
    this.badLines = 0;
    this.onDisconnect = (e) => {
      if (e.target === this.port) this.closed("The radio's USB cable was unplugged.");
    };
  }

  static get supported() {
    return "serial" in navigator;
  }

  get connected() {
    return !!this.port;
  }

  // True if the radio script is actually talking to us (not just an open port).
  get bridgeAlive() {
    return !!this.telemetry && performance.now() - this.telemetry.t < 1500;
  }

  // prompt=false reconnects to a port the user already granted, without a picker.
  async connect({ prompt = true } = {}) {
    let port;
    if (prompt) port = await navigator.serial.requestPort();
    else [port] = await navigator.serial.getPorts();
    if (!port) return false;
    await port.open({ baudRate: 115200 });
    this.port = port;
    this.goodLines = this.badLines = 0;
    this.warned = false;
    this.writer = port.writable.getWriter();
    navigator.serial.addEventListener("disconnect", this.onDisconnect);
    this.readLoop(port);
    this.emit("status", "connected");
    return true;
  }

  async readLoop(port) {
    const decoder = new TextDecoder();
    let buf = "";
    while (this.port === port && port.readable) {
      const reader = port.readable.getReader();
      this.reader = reader;
      try {
        for (;;) {
          const { value, done } = await reader.read();
          if (done) break;
          buf += decoder.decode(value, { stream: true });
          let i;
          while ((i = buf.search(/[\r\n]/)) >= 0) {
            const line = buf.slice(0, i);
            buf = buf.slice(i + 1);
            if (line) this.onLine(line);
          }
          if (buf.length > 1024) {
            this.diagnose(`no line breaks in 1 KB, so not text from the aibrg script`, buf);
            buf = "";
          }
        }
      } catch (e) {
        if (this.port === port) this.closed(`Radio connection lost: ${e.message}`);
        return;
      } finally {
        reader.releaseLock();
      }
    }
  }

  onLine(line) {
    const t = decodeTelemetry(line);
    if (!t) {
      if (++this.badLines >= 20) this.diagnose(explainTelemetry(line), line);
      return;
    }
    this.goodLines++;
    t.t = performance.now();
    t.source = "radio";
    t.bridge = true;
    this.telemetry = t;
    this.emit("telemetry", t);
  }

  // Data but no telemetry for a while: say once what arrives. It tells a wrong USB-VCP mode (or the
  // wrong model selected) from a problem in the script itself.
  diagnose(why, sample) {
    if (this.goodLines || this.warned) return;
    this.warned = true;
    const shown = sample.slice(0, 140).replace(/[^\x20-\x7e]/g, (c) => `\\x${c.charCodeAt(0).toString(16).padStart(2, "0")}`);
    this.emit("error", `The radio sends data, but not the aibrg script's telemetry (${why}): “${shown}”. Check SYS → Hardware → USB-VCP is LUA and the model with the aibrg mixer script is selected.`);
  }

  send(cmd) {
    if (!this.writer || this.writing) return; // drop a frame rather than queue stale commands
    this.writing = true;
    this.writer
      .write(encoder.encode(encodeCommand(cmd)))
      .catch((e) => this.closed(`Couldn't write to the radio: ${e.message}`))
      .finally(() => (this.writing = false));
  }

  closed(reason) {
    const port = this.port;
    this.port = null;
    this.writer = null;
    this.telemetry = null;
    navigator.serial.removeEventListener("disconnect", this.onDisconnect);
    port?.close().catch(() => {});
    this.emit("status", "disconnected");
    if (reason) this.emit("error", reason);
  }

  async disconnect() {
    const port = this.port;
    if (!port) return;
    this.port = null;
    try {
      await this.reader?.cancel();
      this.writer?.releaseLock();
      await port.close();
    } catch {}
    this.writer = null;
    this.telemetry = null;
    navigator.serial.removeEventListener("disconnect", this.onDisconnect);
    this.emit("status", "disconnected");
  }
}
