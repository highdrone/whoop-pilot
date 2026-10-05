// The Events list beside the mission log: what missions found and what alerts.js sent (its stored log, newest first),
// each with its snapshot; a click on a snapshot opens it large. And Settings → Alerts: Mac notification permission,
// an optional ntfy topic for phone pushes, and a test alert.
import { h, fmtDate } from "./dom.js";

const urls = new WeakMap();

// A snapshot (base64 JPEG as perception.snapshot() makes them, a data or http URL, a Blob, canvas or ImageBitmap) as
// something an <img> can show.
export function snapshotUrl(img) {
  if (!img) return null;
  if (typeof img === "string") return img.length > 256 && /^[A-Za-z0-9+/]+=*$/.test(img.slice(0, 256)) ? `data:image/jpeg;base64,${img}` : img;
  if (typeof img.src === "string" && img.src) return img.src;
  if (urls.has(img)) return urls.get(img);
  let url = null;
  if (img instanceof Blob) url = URL.createObjectURL(img);
  else if (img.width && img.height) {
    const c = img.toDataURL ? img : Object.assign(document.createElement("canvas"), { width: img.width, height: img.height });
    if (c !== img) c.getContext("2d").drawImage(img, 0, 0);
    url = c.toDataURL("image/jpeg", 0.85);
  }
  urls.set(img, url);
  return url;
}

// alerts.js log entries and mission findings in one shape.
export const normalizeEvent = (e) => ({
  t: [e.t, e.time, e.ts].find((v) => v > 1e12) ?? Date.now(),
  text: e.text ?? (e.label ? `${e.label}${e.score ? ` (${Math.round(e.score * 100)}%)` : ""}` : "Event"),
  image: e.image ?? e.snapshot ?? null,
  room: e.room ?? null,
  urgency: e.urgency ?? (e.label === "person" ? "high" : "default"),
  label: e.label ?? null,
});

// A mission finding that alerts.js also logged (same picture, or its label within 5 s).
const same = (a, b) => (a.image && a.image === b.image) || (!!a.label && Math.abs(a.t - b.t) < 5000 && b.text.toLowerCase().includes(a.label.toLowerCase()));

export class EventsList {
  constructor(list, { dialog, roomName = (r) => r, onCount = () => {} }) {
    Object.assign(this, { list, dialog, roomName, onCount, alerts: null, local: [], stored: [], unsub: null });
    dialog.querySelector("[data-close]").addEventListener("click", () => dialog.close());
  }

  setAlerts(alerts) {
    this.unsub?.();
    this.alerts = alerts;
    this.unsub = alerts?.on?.("event", () => this.refresh()) ?? null;
    return this.refresh();
  }

  add(e) {
    this.local.push(normalizeEvent(e));
    if (this.local.length > 100) this.local.shift();
    this.render();
  }

  async refresh() {
    try {
      this.stored = ((await this.alerts?.events?.()) ?? []).map(normalizeEvent);
    } catch (e) {
      console.warn("alerts log", e);
    }
    this.render();
  }

  render() {
    const all = [...this.stored, ...this.local.filter((e) => !this.stored.some((s) => same(e, s)))].sort((a, b) => b.t - a.t);
    this.onCount(all.length);
    if (!all.length) return this.list.replaceChildren(h("p", { class: "note center" }, "Nothing yet. What missions find shows up here, with a picture."));
    this.list.replaceChildren(...all.slice(0, 80).map((e) => {
      const url = snapshotUrl(e.image), room = e.room && this.roomName(e.room);
      return h("article", { class: `event ${e.urgency}` },
        url && h("button", { type: "button", class: "snap", "aria-label": "Show the picture large", onclick: () => this.show(url, e, room) }, h("img", { src: url, alt: "" })),
        h("div", {}, h("div", { class: "t" }, e.text), h("small", {}, [fmtDate(e.t), room].filter(Boolean).join(" · "))),
      );
    }));
  }

  show(url, e, room) {
    this.dialog.querySelector("img").src = url;
    this.dialog.querySelector("p").textContent = [e.text, room, fmtDate(e.t)].filter(Boolean).join(" · ");
    this.dialog.showModal();
  }
}

// A suspected change, plainly: what and where (memory.changeText), the live and expected pictures, Claude's opinion, and
// Confirm / Dismiss / It was a person or pet (memory.resolveChange). Used when ui/changecard.js isn't there; same API.
export class SimpleChangeCard {
  constructor(el, { memory }) {
    Object.assign(this, { el, memory, handlers: [] });
    el.classList.add("simple-card");
  }
  on(ev, fn) {
    const it = [ev, fn];
    this.handlers.push(it);
    return () => (this.handlers = this.handlers.filter((x) => x !== it));
  }
  emit(ev, data) {
    this.handlers.forEach(([e, f]) => e === ev && f(data));
  }
  show(id) {
    const m = this.memory, c = m?.changes?.find((x) => x.id === id);
    if (!c) return this.el.replaceChildren(h("p", { class: "note" }, "That change isn't in the flight memory any more."));
    const pic = (b, alt) => b && h("figure", {}, h("img", { src: snapshotUrl(b), alt }), h("figcaption", {}, alt));
    const cl = c.claude, verdict = c.passing ? "Claude thinks it was a person or pet: kept as an obstacle while in view."
      : cl?.status === "answered" ? `Claude: ${cl.what || cl.verdict} (${Math.round((cl.confidence ?? 0) * 100)}% sure)${cl.why ? `: ${cl.why}` : ""}.`
      : cl?.waiting ? "Waiting for your OK to ask Claude." : cl?.reason ? `Not checked by Claude: ${cl.reason}.` : "";
    const done = (status, o) => {
      m.resolveChange(c.id, status, null, o);
      this.emit("resolved", { id: c.id, status });
    };
    this.el.replaceChildren(
      h("div", { class: "row" }, h("strong", { style: "flex: 1" }, `${m.changeText(c)[0].toUpperCase()}${m.changeText(c).slice(1)}.`),
        h("button", { type: "button", class: "icon-btn", "aria-label": "Close", onclick: () => this.emit("close") }, "✕")),
      h("div", { class: "change-pics" }, pic(c.evidence?.live, "What the camera saw"), pic(c.evidence?.expected, "What the 3D scan expected")),
      verdict && h("p", { class: "note" }, verdict),
      c.status !== "suspected" ? h("p", { class: "note" }, `Already ${c.status}.`) : h("div", { class: "row wrap" },
        h("button", { type: "button", class: "btn small", onclick: () => done("confirmed") }, "It's real: keep it on the map"),
        h("button", { type: "button", class: "btn ghost small", onclick: () => done("dismissed") }, "Nothing changed"),
        h("button", { type: "button", class: "btn ghost small", onclick: () => done("dismissed", { transient: true }) }, "It was a person or pet")));
  }
  dispose() {
    this.el.replaceChildren();
  }
}

// Settings → Alerts. alerts() returns the Alerts instance or null (not loaded); tests fall back to a plain
// notification.
export function bindAlertSettings(root, { settings, alerts, log }) {
  const $ = (s) => root.querySelector(s);
  const perm = $("#btnNotifyPerm"), state = $("#notifyPermState"), topic = $("#setNtfy");
  const show = () => {
    const p = "Notification" in window ? Notification.permission : "unsupported";
    state.textContent = { granted: "Allowed.", denied: "Blocked: allow notifications for this page in Chrome's site settings.", default: "Not asked yet.",
      unsupported: "This browser can't show notifications." }[p];
    perm.disabled = p !== "default";
  };
  show();
  perm.addEventListener("click", async () => {
    await Notification.requestPermission().catch(() => {});
    show();
  });
  topic.value = settings.get("ntfyTopic") || "";
  topic.addEventListener("change", () => settings.set("ntfyTopic", topic.value.trim()));
  $("#btnNtfyNew").addEventListener("click", () => {
    const rnd = [...crypto.getRandomValues(new Uint8Array(15))].map((b) => "abcdefghijkmnpqrstuvwxyz23456789"[b % 32]).join("");
    topic.value = `whoop-${rnd}`;
    settings.set("ntfyTopic", topic.value);
  });
  $("#btnTestAlert").addEventListener("click", async () => {
    const a = alerts();
    try {
      if (a) await a.notify({ text: "Test alert from Whoop Pilot.", urgency: "low" });
      else if ("Notification" in window && Notification.permission === "granted") new Notification("Whoop Pilot", { body: "Test alert from Whoop Pilot." });
      log("info", a ? "Test alert sent." : "Test notification shown. Phone alerts work once missions are available in this build.");
    } catch (e) {
      log("error", `The test alert failed: ${e.message}`);
    }
  });
}
