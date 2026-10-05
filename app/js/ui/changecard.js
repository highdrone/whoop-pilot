// One difference from the 3D scan, for a person to decide on, in flight or after (docs/HOME-DRONE.md, Wave C; the PANELS
// contract): what the drone's camera saw against what the scan shows from the same spot, the change in words
// (memory.changeText), Claude's verdict, what it does to the flight until someone decides, and Confirm, Dismiss, "It was a
// person or pet" (memory.resolveChange) and Ask Claude (inspect.confirmChange with apply false: Claude only advises). What
// Ask Claude sends, to whom and about what it costs is on the card; unless Claude's picture checks are "on" (and not
// turned down with "Not now"), the first tap asks "Send these 2 pictures?" in place and only the Send tap sends them.
// Two taps from a change pin: open, decide. Opening never takes the keyboard focus, so Space, L and Esc stay with the
// flight. On a phone, "Show on map" folds the card to one line (title, Decide) so the map shows. Pictures no longer in
// the memory's RAM (older changes) are read back from its store; a cleared memory closes the card.
//   const card = new ChangeCard(el, { memory, inspect, settings });  card.show(id);  card.on("resolved", ({ id, status }) => …)
// Events: "resolved" { id, status, transient }, "show" { id, x, y, z } (Show on map), "close".
import { Emitter } from "../util.js";
import { h } from "./dom.js";
import { when } from "../memory/memory.js";
import { imageSize } from "../ai/claude.js";
import { CHANGE_PROMPT } from "../ai/inspect.js";

const BLOCKS = new Set(["obstacle", "moved", "door-closed"]);
// el's children: kids flattened, without null, undefined and false (as dom.js h() takes them).
export const put = (el, ...kids) => el.replaceChildren(...kids.flat(Infinity).filter((k) => k != null && k !== false));
// One object URL per picture (Blob) while a panel shows the same memory, so re-renders don't reload or flicker them.
export class BlobUrls {
  map = new Map();
  url(blob) {
    if (!this.map.has(blob)) this.map.set(blob, URL.createObjectURL(blob));
    return this.map.get(blob);
  }
  clear() {
    this.map.forEach((u) => URL.revokeObjectURL(u));
    this.map.clear();
  }
}
// Calls onChange(now) whenever flying() turns true or false (polled: the panels have no controller events). -> stop().
export function watchFlying(flying, onChange, ms = 1000) {
  let was = !!flying();
  const t = setInterval(() => { const now = !!flying(); if (now !== was) onChange((was = now)); }, ms);
  return () => clearInterval(t);
}
// Calls fn each time el comes into view (a tab or a dialog opened). -> stop().
export function onVisible(el, fn) {
  if (typeof IntersectionObserver === "undefined") return () => {};
  const io = new IntersectionObserver((es) => es.some((e) => e.isIntersecting) && fn());
  io.observe(el);
  return () => io.disconnect();
}
// An error's message as plain words, or `fallback` when it reads like a program's (workers, stack words, HTTP codes).
export const plainWords = (e, fallback) => {
  const m = String(e?.message ?? e ?? "").trim().replace(/\.+$/, "");
  return !m || /worker|twin\b|abort|undefined|null|NaN|TypeError|ReferenceError|is not a function|Cannot read|HTTP \d|fetch|\.js\b|json/i.test(m) ? fallback : m;
};
const cap = (s) => (s ? s.charAt(0).toUpperCase() + s.slice(1) : "");
const pct = (v) => `${Math.round(v * 100)}% sure`;
export const centsText = (cost) => (cost == null ? "" : cost < 0.01 ? "less than $0.01" : `about $${cost.toFixed(2)}`);
// What Ask Claude sends, to whom, and the cost, in a sentence.
export const askText = (cost) => `Ask Claude sends these 2 pictures to Anthropic's Claude API with your API key${cost != null ? ` (${centsText(cost)})` : ""}; Claude only advises.`;

export const STATUS_WORDS = { suspected: "Not decided yet", confirmed: "Confirmed", dismissed: "Dismissed" };

// Claude's look at a change, in a sentence: { tone: "warn" | "ok" | "muted" | "info", text }.
export function verdictText(c, { vision = "ask" } = {}) {
  const v = c?.claude;
  if (c?.status === "suspected" && c.passing) return { tone: "info", text: "Claude thinks a person or pet: kept as an obstacle while in view." };
  if (v?.status === "answered") {
    const what = v.what ? ` (${v.what})` : "";
    const head = { "real-change": `Claude: a real change${what}`, "no-change": "Claude: nothing has changed there", "person-or-pet": "Claude: a person or pet passing by",
      unclear: "Claude couldn't tell from the pictures" }[v.verdict] ?? `Claude: ${v.verdict}`;
    const advice = v.suggest === "confirmed" ? " It suggests confirming it." : v.suggest === "dismissed" ? " It suggests dismissing it." : "";
    return { tone: v.verdict === "real-change" ? "warn" : v.verdict === "unclear" ? "muted" : "ok",
      text: `${head}, ${pct(v.confidence)}.${v.why ? ` ${v.why.replace(/\.?$/, ".")}` : ""}${advice}${v.small ? " The pictures were small, so this is only advice." : ""}` };
  }
  if (v?.status === "unconfirmed")
    return { tone: "muted", text: v.waiting ? "Claude hasn't looked yet: it's waiting for your OK to send pictures." : `Claude didn't check it: ${String(v.reason ?? "no answer").replace(/\.$/, "")}.` };
  if (vision === "off") return { tone: "muted", text: "Claude's picture checks are off (Settings → Brain)." };
  return { tone: "muted", text: "Claude hasn't looked at it." };
}

// What the change does to flying, now.
export function effectText(c, { world = "real" } = {}) {
  const door = c.kind?.startsWith("door");
  if (c.status === "suspected") {
    if (c.stale && BLOCKS.has(c.kind)) return "The camera looked at it for 10 s and didn't see it: no longer blocking the way (it blocks again if seen).";
    if (c.kind === "door-closed") return "Until someone decides, the drone won't fly through that doorway.";
    if (BLOCKS.has(c.kind)) return "Until someone decides, the drone flies around it as if it were there.";
    return "Until someone decides, the drone goes by the 3D scan there.";
  }
  if (c.status === "confirmed") {
    if (world === "sim") return "In the simulator it stays on the map as an obstacle; your house's map isn't changed.";
    return { "door-closed": "The doorway is closed on the map: missions go another way.", "door-open": "The doorway is open on the map again.",
      gone: "Its space is free on the map again (only what the scan had there: space nothing has seen stays off limits)." }[c.kind]
      ?? "It's on the map as a no-fly spot, and in the 3D map.";
  }
  if (c.transient) return "It was something passing: nothing changes on the map.";
  if (c.by === "user") return `Nothing changes on the map, and the drone won't raise ${door ? "this doorway" : "this spot"} again unless it looks different.`;
  return "Nothing changes on the map. It may be raised again if the drone sees it again.";
}

// Who decided, and when: "Confirmed by you 3 min ago".
export function decidedText(c, now = Date.now()) {
  if (c.status === "suspected") return `Seen ${c.n === 1 ? "once" : `${c.n} times`}, last ${when(c.last, now)}.`;
  const by = { user: "by you", claude: "by Claude", detector: "when the drone looked again", passing: "after the person or pet left" }[c.by] ?? "";
  return `${STATUS_WORDS[c.status]} ${by} ${when(c.resolvedAt ?? c.last, now)}.`.replace(/\s+/g, " ");
}

export class ChangeCard extends Emitter {
  constructor(el, { memory = null, inspect = null, settings = null, world = null, now = Date.now } = {}) {
    super();
    Object.assign(this, { el, memory: null, inspect, settings, world, now, id: null, pics: new BlobUrls(), busy: false, note: "", offs: [], confirmAsk: false, collapsed: false, costs: new Map(), stored: new Map() });
    el.classList.add("chg-host");
    this.setMemory(memory);
  }

  setMemory(memory) {
    this.offs.forEach((f) => f());
    if (memory !== this.memory) (this.pics.clear(), this.stored.clear());
    this.memory = memory;
    const again = (e) => e?.id === this.id && this.render();
    const changed = (e = {}) => (e.phase === "cleared" ? this.id && (this.hide(), this.emit("close")) : (e.change?.id ?? e.id) === this.id && this.render());
    this.offs = memory?.on ? [memory.on("change", changed), memory.on("annotate", again)] : [];
    if (this.id && !this.change) this.hide();
  }

  get change() {
    return this.memory?.changes?.find((c) => c.id === this.id) ?? null;
  }

  show(id) {
    this.id = id;
    Object.assign(this, { note: "", confirmAsk: false, collapsed: false });
    this.el.hidden = false;
    this.render();
    return !!this.change;
  }

  hide() {
    this.id = null;
    this.pics.clear();
    put(this.el);
    this.el.hidden = true;
  }

  picture(blob, alt, caption) {
    const img = blob instanceof Blob ? h("img", { src: this.pics.url(blob), alt }) : h("div", { class: "chg-nopic" }, "No picture");
    return h("figure", { class: "chg-pic" }, img, h("figcaption", {}, caption));
  }

  // Its pictures: in RAM, else read back once from the memory's store (memory.picture), then drawn again.
  evidence(c) {
    if (c.evidence?.live || !this.memory?.picture) return c.evidence ?? null;
    if (!this.stored.has(c.id)) {
      this.stored.set(c.id, null);
      this.memory.picture(c).then((e) => e && (this.stored.set(c.id, e), this.id === c.id && this.render())).catch(() => {});
    }
    return this.stored.get(c.id) ?? c.evidence ?? null;
  }

  render() {
    const c = this.change, m = this.memory;
    if (!c) return put(this.el, h("p", { class: "note" }, "This change is no longer in the flight memory."));
    const ev = this.evidence(c);
    const vision = this.settings?.get?.("aiVision") ?? "ask", v = verdictText(c, { vision }), world = this.world ?? m.world ?? "real";
    const room = c.room ? m.roomName(c.room) : null, close = h("button", { type: "button", class: "chg-x", "aria-label": "Close", title: "Close", onclick: () => (this.hide(), this.emit("close")) }, "×");
    if (this.collapsed) return put(this.el, h("article", { class: "chg-card", "data-status": c.status, "data-collapsed": "true", "aria-label": "Change from the 3D scan" },
      h("header", { class: "chg-h" }, h("strong", { class: "chg-title" }, cap(m.changeText(c))),
        h("button", { type: "button", class: "btn small", onclick: () => ((this.collapsed = false), this.render()) }, c.status === "suspected" ? "Decide" : "Open"), close)));
    const act = (label, title, fn, cls = "") => h("button", { type: "button", class: `btn small ${cls}`, title, disabled: this.busy, onclick: fn }, label);
    const ask = this.canAsk(c, vision), cost = ask ? this.cost(c) : null;
    const actions = c.status === "suspected" ? [
      act("Confirm: it's there", "Keep it on the map: the drone flies around it from now on", () => this.resolve("confirmed"), "chg-yes"),
      act("Dismiss: not real", "Nothing is there: the drone goes by the 3D scan again", () => this.resolve("dismissed")),
      act("It was a person or pet", "Something passing: forget it", () => this.resolve("dismissed", { transient: true })),
      ask && !this.confirmAsk && act(c.claude?.status === "answered" ? "Ask Claude again" : "Ask Claude", askText(cost), () => this.askTap(vision), "ghost"),
    ] : c.status === "confirmed"
      ? [act("Undo: it isn't there", "Take it off the map again", () => this.resolve("dismissed", { note: "Undone by you." }))]
      : [act("It is there after all", "Put it on the map", () => this.resolve("confirmed"))];
    put(this.el, h("article", { class: "chg-card", "data-status": c.status, "aria-label": "Change from the 3D scan" },
      h("header", { class: "chg-h" },
        h("strong", { class: "chg-title" }, cap(m.changeText(c))),
        h("span", { class: "chg-badge", "data-status": c.status }, STATUS_WORDS[c.status]),
        close),
      h("p", { class: "chg-meta" }, decidedText(c, this.now()), room ? ` ${room}.` : "", c.note ? ` ${c.note}` : ""),
      h("div", { class: "chg-pics" }, this.picture(ev?.live, "What the drone's camera saw", "What the drone saw"),
        this.picture(ev?.expected, "What the 3D scan shows from the same spot", "What the 3D scan shows")),
      ev?.cut && h("p", { class: "chg-meta" }, "Only part of it was in view (dark where the picture ends)."),
      h("p", { class: "chg-verdict", "data-tone": v.tone }, v.text),
      h("p", { class: "chg-effect" }, effectText(c, { world })),
      h("div", { class: "chg-actions" }, actions,
        Number.isFinite(c.x) && act("Show on map", "Centre the map on it", () => this.showOnMap(c), "ghost")),
      c.status === "suspected" && ask && (this.confirmAsk
        ? h("div", { class: "chg-ask", role: "group", "aria-label": "Send the pictures to Claude?" },
          h("p", {}, `Send these 2 pictures to Claude? They go to Anthropic's Claude API with your API key${cost != null ? `, ${centsText(cost)}` : ""}. Claude only advises: you decide.`),
          h("div", { class: "chg-actions" }, act("Send them", "Send the two pictures to Claude now", () => ((this.confirmAsk = false), this.ask())),
            act("Not now", "Don't send anything", () => ((this.confirmAsk = false), this.render()), "ghost")))
        : h("p", { class: "chg-hint" }, askText(cost))),
      h("p", { class: "chg-note", role: "status", "aria-live": "polite" }, this.note)));
  }

  // Ask Claude: straight away when its picture checks are "on" (and weren't turned down since); else "Send these 2 pictures?" first.
  askTap(vision) {
    if (vision === "on" && !this.inspect?.claude?.declined) return this.ask();
    this.confirmAsk = true;
    this.render();
  }

  // About what one check of this change costs (its two pictures at their size, the prompt, the answer), or null.
  cost(c) {
    const claude = this.inspect?.claude;
    if (!claude?.estimate) return null;
    if (!this.costs.has(c.id)) {
      this.costs.set(c.id, null);
      Promise.all([imageSize(c.evidence?.live), imageSize(c.evidence?.expected)]).then((sz) => {
        const img = (s) => ({ type: "image", width: s?.width ?? 640, height: s?.height ?? 480 });
        this.costs.set(c.id, claude.estimate({ system: CHANGE_PROMPT, content: [img(sz[0]), img(sz[1]), { type: "text", text: "x".repeat(400) }], expectOut: 500 }).cost);
        if (this.id === c.id) this.render();
      }).catch(() => {});
    }
    return this.costs.get(c.id);
  }

  // Show on map: on a phone the card folds to a line so the map it centred shows.
  showOnMap(c) {
    if (typeof matchMedia === "function" && matchMedia("(max-width: 900px)").matches) (this.collapsed = true), this.render();
    this.emit("show", { id: c.id, x: c.x, y: c.y, z: c.z });
  }

  canAsk(c, vision) {
    return !!this.inspect && vision !== "off" && !!c.evidence?.live && !!c.evidence?.expected && !!this.inspect.claude?.configured;
  }

  resolve(status, { transient = false, note = null } = {}) {
    const c = this.change;
    if (!c) return;
    const r = this.memory.resolveChange(c.id, status, note ?? (transient ? "A person or pet, said by you." : status === "dismissed" ? "Dismissed by you." : "Confirmed by you."), { by: "user", transient });
    this.note = r ? (status === "confirmed" ? "Confirmed." : transient ? "Forgotten: it was something passing." : "Dismissed.") : "Couldn't change it.";
    this.render();
    if (r) this.emit("resolved", { id: c.id, status, transient });
  }

  async ask() {
    const c = this.change;
    if (!c || this.busy) return;
    this.busy = true;
    this.note = "Asking Claude: sending the two pictures…";
    this.render();
    try {
      const r = await this.inspect.confirmChange(c, { apply: false, approved: true });
      this.note = r.status === "answered" ? "" : `Claude didn't answer: ${r.reason}.`;
    } catch (e) {
      this.note = `Claude didn't answer: ${e.message}.`;
    } finally {
      this.busy = false;
      if (this.id) this.render();
    }
  }

  dispose() {
    this.offs.forEach((f) => f());
    this.offs = [];
    this.hide();
  }
}
