// Telling people what the drone found or what went wrong: speech, a Mac notification (the Notification API, once
// permission was given from a click: requestPermission()), an optional push to a phone through ntfy (settings
// "ntfyTopic", optional "ntfyServer"; the snapshot goes as an attachment, so the frame leaves the house: use a long
// random topic or a self-hosted server), and an event log in IndexedDB (snapshot, room, pose) for the map and history
// UI. Every channel is best effort and none waits for another; notify() reports which ones worked.
// An alert about a detection ({ label, score, box }) that is uncertain (inspector.needsCheck: a low score or a label new
// to this house) waits for Claude's look first (ai/inspect.js), at most ALERT_WAIT, and never for the consent prompt (the
// UI hears "consent-needed" instead): "(confirmed by Claude)" when Claude is sure, "(Claude: likely)" when it leans that
// way, and without Claude (no key, no consent yet, over budget, too slow) "(unconfirmed)". One Claude is sure is wrong
// still goes out when it is about a person or urgent (at default priority, "(Claude thinks it's a coat on a chair)"):
// one doubtful look at a small, dim picture never silences an intruder alert; other ones (a pet) are only logged, at
// low urgency. The log stays here (ui/events.js reads events()) and every logged event is also noted in the house's
// flight memory (setMemory).
import { Emitter } from "./util.js";

const DB = "whoopPilot.events", STORE = "events", KEEP = 500;
export const ALERT_WAIT = 8000; // ms an uncertain alert waits for Claude's look
const PRIORITY = { low: 2, default: 3, high: 5 };
const TAGS = { high: "rotating_light", default: "eyes", low: "information_source" };

// HTTP header text: printable ASCII as is, anything else RFC 2047 encoded (ntfy decodes it).
function header(t) {
  const s = String(t).replace(/[\r\n]+/g, " ");
  if (/^[\x20-\x7e]*$/.test(s)) return s;
  let bin = "";
  for (const b of new TextEncoder().encode(s)) bin += String.fromCharCode(b);
  return `=?UTF-8?B?${btoa(bin)}?=`;
}
const jpegBytes = (b64) => Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));

export class Alerts extends Emitter {
  constructor({ settings = null, speak = null, fetch = globalThis.fetch?.bind(globalThis), Notification = globalThis.Notification, indexedDB = globalThis.indexedDB, memory = null, inspector = null } = {}) {
    super();
    Object.assign(this, { settings, speak, fetch, N: Notification, idb: indexedDB, houseMemory: memory, inspector, alertWait: ALERT_WAIT });
    this.memory = []; // the log without IndexedDB (Node, private windows)
    this.db = null;
  }

  // Ask for notification permission: from a click (Chrome quietly blocks prompts without one, and nobody may be there).
  async requestPermission() {
    const N = this.N;
    if (!N || N.permission !== "default") return N?.permission ?? "unsupported";
    return N.requestPermission();
  }

  // The house's flight memory (memory/memory.js) and Claude's second look (ai/inspect.js); null for none.
  setMemory(memory) {
    this.houseMemory = memory;
  }
  setInspector(inspector) {
    this.inspector = inspector;
  }

  // { text, image (base64 JPEG), room, urgency: "low" | "default" | "high", kind, pose, and for a detection: label, score,
  // box (normalized), trackId, x, y } -> { spoke, notified, pushed, logged, confirmation }. confirmation: "confirmed"
  // (Claude saw it too), "likely", "rejected" (Claude is sure it isn't: a person or urgent one still goes out, at default
  // priority; others are logged only), "unconfirmed" (needed a check, didn't get one), or null (no check needed).
  async notify({ text, image = null, room = null, urgency = "default", kind = "alert", pose = null, label = null, score = null, box = null, trackId = null, x = null, y = null }) {
    const check = label && this.inspector?.needsCheck?.({ label, score })
      ? await this.inspector.confirmDetection(image, label, score, { box, ask: false, deadline: this.alertWait }).catch((e) => ({ status: "unconfirmed", reason: e?.message ?? String(e) }))
      : null;
    const confirmation = check?.status ?? null, rejected = confirmation === "rejected", still = rejected && (label === "person" || urgency === "high");
    const ev = this.event({ text, image, room, urgency: rejected ? (still ? "default" : "low") : urgency, kind, pose, label, score, trackId, x, y, check });
    const out = { spoke: false, notified: false, pushed: false, logged: false, ...(check && { confirmation }) };
    if (rejected && !still) return Object.assign(out, { logged: await this.log(ev) });
    if (this.speak && this.settings?.get?.("speak") !== false) {
      this.speak(text);
      out.spoke = true;
    }
    const [notified, pushed, logged] = await Promise.allSettled([this.desktop(ev), this.push(ev), this.log(ev)]);
    if (pushed.status === "rejected") this.emit("error", `Phone push failed: ${pushed.reason?.message}`);
    return Object.assign(out, { notified: notified.value === true, pushed: pushed.value === true, logged: logged.value === true });
  }

  // check: inspector.confirmDetection's answer; the event's text then says what Claude made of it.
  event({ text, image = null, room = null, urgency = "default", kind = "event", pose = null, label = null, score = null, trackId = null, x = null, y = null, check = null }) {
    const p = pose && { x: +pose.x.toFixed(2), y: +pose.y.toFixed(2), yaw: +pose.yaw.toFixed(3), sigma: +pose.sigma.toFixed(3) };
    const s = check?.status;
    const said = s === "confirmed" ? " (confirmed by Claude)" : s === "likely" ? " (Claude: likely)" : s === "rejected" ? ` (Claude thinks it's ${check.actual ? check.actual.charAt(0).toLowerCase() + check.actual.slice(1) : `not a ${label}`})`
      : s === "unconfirmed" ? " (unconfirmed)" : "";
    const ev = { t: Date.now(), kind, text: String(text ?? "") + said, room, urgency, pose: p, image };
    if (label) Object.assign(ev, { label, score, trackId, x: x ?? p?.x ?? null, y: y ?? p?.y ?? null, confirmed: s === "confirmed" ? true : s === "rejected" ? false : null, confirmation: s ?? null,
      ...(check && { claude: { status: s, actual: check.actual ?? null, confidence: check.confidence ?? null, why: check.why ?? check.reason ?? null } }) });
    return ev;
  }

  // Only with permission already granted: asking without a click can wait forever on a prompt nobody answers.
  async desktop(ev) {
    const N = this.N;
    if (N?.permission !== "granted") return false;
    try {
      new N("Whoop Pilot", { body: ev.text, tag: ev.kind, requireInteraction: ev.urgency === "high", silent: ev.urgency === "low" });
      return true;
    } catch {
      return false;
    }
  }

  async push(ev) {
    const topic = String(this.settings?.get?.("ntfyTopic") ?? "").trim();
    if (!topic || !this.fetch) return false;
    const server = String(this.settings?.get?.("ntfyServer") || "https://ntfy.sh").replace(/\/+$/, "");
    const headers = { Title: "Whoop Pilot", Priority: String(PRIORITY[ev.urgency] ?? 3), Tags: TAGS[ev.urgency] ?? TAGS.default };
    const url = `${server}/${encodeURIComponent(topic)}`;
    const res = ev.image
      ? await this.fetch(url, { method: "PUT", headers: { ...headers, Filename: "whoop.jpg", Message: header(ev.text) }, body: jpegBytes(ev.image) })
      : await this.fetch(url, { method: "POST", headers, body: ev.text });
    if (!res.ok) throw new Error(`ntfy answered ${res.status}`);
    return true;
  }

  // The event log, without speech or push (findings during a mission); also noted in the flight memory.
  async log(ev) {
    if (!ev.t) ev = this.event(ev);
    this.emit("event", ev);
    try {
      this.houseMemory?.note?.(ev);
    } catch (e) {
      console.warn("memory note", e);
    }
    const db = await this.open();
    if (!db) {
      this.memory.push(ev);
      if (this.memory.length > KEEP) this.memory.shift();
      return true;
    }
    return new Promise((resolve) => {
      const tx = db.transaction(STORE, "readwrite"), store = tx.objectStore(STORE);
      store.add(ev);
      const count = store.count();
      count.onsuccess = () => {
        if (count.result <= KEEP) return;
        const cur = store.openCursor();
        let drop = count.result - KEEP;
        cur.onsuccess = () => {
          const c = cur.result;
          if (c && drop-- > 0) (c.delete(), c.continue());
        };
      };
      tx.oncomplete = () => resolve(true);
      tx.onerror = tx.onabort = () => resolve(false);
    });
  }

  // Newest first.
  async events({ limit = 100 } = {}) {
    const db = await this.open();
    if (!db) return this.memory.slice(-limit).reverse();
    return new Promise((resolve) => {
      const out = [], req = db.transaction(STORE).objectStore(STORE).openCursor(null, "prev");
      req.onsuccess = () => {
        const c = req.result;
        if (c && out.length < limit) (out.push({ id: c.key, ...c.value }), c.continue());
        else resolve(out);
      };
      req.onerror = () => resolve(out);
    });
  }

  async clear() {
    this.memory = [];
    const db = await this.open();
    if (db) await new Promise((r) => ((db.transaction(STORE, "readwrite").objectStore(STORE).clear().onsuccess = r)));
  }

  open() {
    if (!this.idb) return Promise.resolve(null);
    return (this.db ??= new Promise((resolve) => {
      const req = this.idb.open(DB, 1);
      req.onupgradeneeded = () => req.result.createObjectStore(STORE, { autoIncrement: true });
      req.onsuccess = () => resolve(req.result);
      req.onerror = req.onblocked = () => resolve(null);
    }));
  }
}
