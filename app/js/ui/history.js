// Where the drone has been and what it saw (docs/HOME-DRONE.md, Wave C; the PANELS contract), from the flight memory
// (memory/memory.js): a question box (memory.answer: plain words built only from the records, with the sighting's
// picture), the changes from the 3D scan waiting for a decision (each opens a ChangeCard), the decided ones, "Last seen"
// for people, the cat, the dog and anything else it named (picture, where, when), and the flights by day (length,
// distance, rooms, what it saw, how it ended, how much of each room its camera looked at, position lost; the full report
// folded) whose trail a click shows on the map and the 3D view. Older flights' trails, no longer in RAM, are read back
// from the memory's store (trailOf reads only that flight's chunks) for their losses: one at a time, on the ground only,
// the newest HISTORY.readBack of them per memory, keeping only the counts; a flight record's own `losses` comes first
// (only records older than that count lack it).
//   const hist = new HistoryPanel(el, { memory, map, inspect, settings, flying });
//   hist.on("trail", ({ flightId }) => { mapView.setTrail(...); view3d.setFlight(flightId); });   // flightId null: none
//   hist.on("select", ({ kind: "sighting" | "change", id, x, y, z }) => …)
import { Emitter } from "../util.js";
import { h } from "./dom.js";
import { when, duration } from "../memory/memory.js";
import { ChangeCard, STATUS_WORDS, verdictText, put, BlobUrls } from "./changecard.js";

export const HISTORY = { readBack: 6 };
const DAY = 864e5;
const DAYS = ["Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"];
const MONTHS = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
const midnight = (t) => new Date(new Date(t).setHours(0, 0, 0, 0)).getTime();
const hm = (t) => { const d = new Date(t); return `${String(d.getHours()).padStart(2, "0")}:${String(d.getMinutes()).padStart(2, "0")}`; };
const cap = (s) => (s ? s.charAt(0).toUpperCase() + s.slice(1) : "");
const plural = (n, w, ws = `${w}s`) => `${n} ${n === 1 ? w : ws}`;
export const LABEL_ORDER = ["person", "cat", "dog"];
const ICON = { person: "P", cat: "C", dog: "D" };
const ASK = ["Where did you last see the cat?", "What changed?", "Where have you been?"];

// "Today", "Yesterday", "Monday", "28 Sep" (local time).
export function dayLabel(t, now = Date.now()) {
  const days = Math.round((midnight(now) - midnight(t)) / DAY), d = new Date(t);
  return days <= 0 ? "Today" : days === 1 ? "Yesterday" : days < 7 ? DAYS[d.getDay()] : `${d.getDate()} ${MONTHS[d.getMonth()]}${d.getFullYear() !== new Date(now).getFullYear() ? ` ${d.getFullYear()}` : ""}`;
}

// Newest first, by day: [{ day, flights }].
export function groupFlights(flights, now = Date.now()) {
  const out = [];
  for (const f of [...flights].sort((a, b) => b.t0 - a.t0)) {
    const day = dayLabel(f.t0, now);
    if (out.at(-1)?.day === day) out.at(-1).flights.push(f);
    else out.push({ day, flights: [f] });
  }
  return out;
}

// A flight's trail: in RAM, else read back from the memory's store (older flights), else null (not known).
export async function flightTrail(memory, flightId) {
  if (!memory || !flightId) return null;
  const tr = memory.trails?.get(flightId);
  if (tr) return tr;
  return (await memory.trailOf?.(flightId).catch(() => null)) ?? null;
}

const firstSentence = (t) => (t ? (String(t).trim().match(/^.*?[.!?](?=\s|$)/)?.[0] ?? String(t).trim()) : null);

// One flight in plain words: { time, length, distance, rooms (names, in order), seen (things), lost (times; null when its
// trail isn't at hand), sim, what, live, outcome (how it ended), looked (how much of each room the camera saw), report }.
// trail: the flight's samples when not in RAM (flightTrail), or just its count of losses.
export function flightSummary(memory, f, now = Date.now(), trail = memory.trails?.get(f.id) ?? null) {
  const secs = ((f.t1 ?? now) - f.t0) / 1000, live = f.t1 == null;
  const rooms = f.rooms.map((id) => memory.roomName(id)).filter((n, i, a) => n !== a[i - 1]);
  const seen = memory.sightings.filter((s) => s.flight === f.id), things = [...new Set(seen.map((s) => memory.thing(s)))];
  const lost = Array.isArray(trail) ? trail.filter((s) => s.lost).length : Number.isFinite(trail) ? trail : f.losses ?? null, changes = memory.changes.filter((c) => c.flight === f.id).length;
  const looked = !live && f.inspection ? memory.inspectionText?.(f.inspection) || null : null;
  return { time: hm(f.t0), length: duration(secs), distance: `${(f.distance ?? 0).toFixed(1)} m`, rooms, seen: things, sightings: seen.length, lost, changes,
    sim: f.world === "sim" || f.kind === "sim", live, what: f.missions?.length ? f.missions.slice(0, 2).join("; ") : f.mission ?? null, summary: f.summary ?? null,
    outcome: live ? null : firstSentence(f.summary), looked, measured: f.inspection?.measured === true, report: live ? null : f.report ?? null };
}

// What it saw, in words: the things, or (finished, with the camera's share measured) that nobody was noticed; else nothing.
export function seenWords(s) {
  if (s.seen.length) return `saw ${s.seen.join(", ")}`;
  return !s.live && s.measured ? "no people or pets noticed in what it looked at" : null;
}

// The latest sighting of each label: people, the cat and the dog first, then the rest, newest first.
export function lastSeenList(memory) {
  const latest = new Map();
  for (const s of memory.sightings) if (!latest.has(s.label) || latest.get(s.label).t <= s.t) latest.set(s.label, s);
  const rank = (l) => (LABEL_ORDER.includes(l) ? LABEL_ORDER.indexOf(l) : LABEL_ORDER.length);
  return [...latest.values()].sort((a, b) => rank(a.label) - rank(b.label) || b.t - a.t);
}

// "The cat: in the Kitchen near the sofa, 2 h ago (on patrol; Claude confirmed)".
export function sightingText(memory, s, now = Date.now()) {
  const claude = { confirmed: "Claude confirmed", likely: "Claude thought it likely", rejected: "Claude doubted it" }[s.claude];
  const bits = [s.mission && (s.mission === "patrol" ? "on patrol" : `while asked to ${s.mission}`), s.n > 1 && `seen ${s.n} times`, claude].filter(Boolean);
  return `${cap(memory.thing(s))} ${memory.where(s)}, ${when(s.t, now)}${bits.length ? ` (${bits.join("; ")})` : ""}.`;
}

export class HistoryPanel extends Emitter {
  constructor(el, { memory = null, map = null, inspect = null, settings = null, now = Date.now, flying = () => false } = {}) {
    super();
    Object.assign(this, { el, memory: null, map, inspect, settings, now, flying, flightId: null, open: null, pics: new BlobUrls(), offs: [], timer: 0, losses: new Map(), reading: null });
    el.classList.add("hist");
    const input = h("input", { type: "text", class: "hist-q", placeholder: "Ask, e.g. where did you last see the cat?", "aria-label": "Ask about earlier flights", spellcheck: false });
    this.input = input;
    this.answerEl = h("div", { class: "hist-answer", role: "status", "aria-live": "polite" });
    this.changesEl = h("section", { class: "hist-sec" });
    this.seenEl = h("section", { class: "hist-sec" });
    this.flightsEl = h("section", { class: "hist-sec" });
    put(el,
      h("form", { class: "hist-ask", onsubmit: (e) => (e.preventDefault(), this.ask(input.value)) }, input, h("button", { type: "submit", class: "btn small" }, "Ask")),
      h("div", { class: "hist-chips" }, ASK.map((q) => h("button", { type: "button", class: "chip", onclick: () => ((input.value = q), this.ask(q)) }, q))),
      this.answerEl, this.changesEl, this.seenEl, this.flightsEl,
      h("p", { class: "hint" }, "The flight memory keeps pictures of people and pets on this Mac only. Settings → Memory sets how long."));
    this.setMemory(memory);
  }

  setMemory(memory) {
    this.offs.forEach((f) => f());
    if (memory !== this.memory) (this.pics.clear(), this.card?.dispose(), (this.card = null), (this.open = null), this.losses.clear());
    this.memory = memory;
    const soon = () => (this.timer ||= setTimeout(() => ((this.timer = 0), this.refresh()), 250));
    this.offs = memory?.on ? ["flight", "sighting", "change", "annotate", "named"].map((e) => memory.on(e, soon)) : []; // named: the house came after the memory
    if (this.flightId && !memory?.flights.some((f) => f.id === this.flightId)) this.setFlight(null);
    put(this.answerEl);
    this.refresh();
  }

  setMap(map) {
    this.map = map;
  }

  setFlight(flightId) {
    this.flightId = flightId;
    this.renderFlights();
    this.emit("trail", { flightId });
  }

  ask(q) {
    q = String(q ?? "").trim();
    if (!q) return;
    if (!this.memory) return put(this.answerEl, h("p", { class: "note" }, "No house is loaded, so there's no flight memory to ask."));
    const a = this.memory.answer(q, this.now()), s = a.sighting;
    put(this.answerEl, h("div", { class: "hist-reply" }, s?.snapshot instanceof Blob && this.thumb(s.snapshot, `${this.memory.thing(s)}, as the drone saw it`), h("p", {}, a.text),
      s && Number.isFinite(s.x) && h("button", { type: "button", class: "btn ghost small", onclick: () => this.emit("select", { kind: "sighting", id: s.id, x: s.x, y: s.y, z: s.z }) }, "Show on map")));
  }

  thumb(blob, alt) {
    return h("img", { class: "hist-thumb", src: this.pics.url(blob), alt });
  }

  refresh() {
    const m = this.memory;
    if (!m) {
      for (const s of [this.changesEl, this.seenEl, this.flightsEl]) put(s);
      this.changesEl.append(h("p", { class: "note" }, "Load a house to see what the drone remembers about it."));
      return;
    }
    this.renderChanges();
    this.renderSeen();
    this.renderFlights();
  }

  renderChanges() {
    const m = this.memory, open = m.changes.filter((c) => c.status === "suspected").sort((a, b) => b.last - a.last);
    const decided = m.changes.filter((c) => c.status !== "suspected").sort((a, b) => (b.resolvedAt ?? b.last) - (a.resolvedAt ?? a.last));
    const vision = this.settings?.get?.("aiVision") ?? "ask";
    const row = (c) => {
      const pic = c.evidence?.live instanceof Blob ? this.thumb(c.evidence.live, "What the drone saw") : h("div", { class: "hist-thumb blank", "aria-hidden": "true" }, "!");
      const isOpen = this.open === c.id;
      return h("li", { class: "hist-change", "data-status": c.status },
        h("button", { type: "button", class: "hist-row", "aria-expanded": String(isOpen), onclick: () => this.toggleChange(c.id) }, pic,
          h("span", { class: "hist-text" }, h("strong", {}, cap(m.changeText(c))),
            h("small", {}, `${STATUS_WORDS[c.status]} · ${c.status === "suspected" ? `last seen ${when(c.last, this.now())}` : when(c.resolvedAt ?? c.last, this.now())}`),
            c.status === "suspected" && !isOpen && h("small", { class: "hist-verdict" }, verdictText(c, { vision }).text)),
          h("span", { class: "hist-go", "aria-hidden": "true" }, isOpen ? "▾" : "▸")),
        isOpen && this.cardHost(c.id));
    };
    put(this.changesEl,
      h("h4", {}, "Changes from the 3D scan ", h("span", { class: "count" }, open.length || "")),
      open.length ? h("ul", { class: "hist-list" }, open.map(row))
        : h("p", { class: "note" }, m.changes.length ? "Nothing waiting for a decision." : "No differences from the 3D scan noticed yet. When the drone sees something new (a box, a closed door), it shows here with pictures."),
      decided.length > 0 && h("details", { class: "hist-more", open: decided.some((c) => c.id === this.open) }, h("summary", {}, `Decided (${decided.length})`), h("ul", { class: "hist-list" }, decided.map(row))));
  }

  cardHost(id) {
    if (this.card?.id === id) return this.card.render(), this.card.el; // kept across refreshes (its note, a check running)
    const host = h("div", { class: "hist-card" });
    this.card?.dispose();
    this.card = new ChangeCard(host, { memory: this.memory, inspect: this.inspect, settings: this.settings, now: this.now });
    this.card.on("resolved", (e) => this.emit("resolved", e));
    this.card.on("show", (e) => this.emit("select", { kind: "change", ...e }));
    this.card.on("close", () => this.toggleChange(id));
    this.card.show(id);
    return host;
  }

  toggleChange(id) {
    this.open = this.open === id ? null : id;
    if (!this.open) (this.card?.dispose(), (this.card = null));
    this.renderChanges();
    const c = this.memory.changes.find((q) => q.id === id);
    if (this.open && c) this.emit("select", { kind: "change", id, x: c.x, y: c.y, z: c.z });
  }

  renderSeen() {
    const m = this.memory, list = lastSeenList(m), now = this.now();
    put(this.seenEl, h("h4", {}, "Last seen"),
      list.length ? h("ul", { class: "hist-seen" }, list.map((s) => h("li", {},
        h("button", { type: "button", class: "hist-tile", title: "Show where on the map", onclick: () => this.emit("select", { kind: "sighting", id: s.id, x: s.x, y: s.y, z: s.z }) },
          s.snapshot instanceof Blob ? this.thumb(s.snapshot, `${m.thing(s)}, as the drone saw it`) : h("span", { class: "hist-thumb blank", "data-label": s.label, "aria-hidden": "true" }, ICON[s.label] ?? "?"),
          h("span", { class: "hist-text" }, h("strong", {}, cap(s.label === "person" ? "a person" : s.label)), h("small", {}, sightingText(m, s, now)))))))
        : h("p", { class: "note" }, "Nothing seen yet. People, pets and things the drone recognises on its flights show here with a picture."));
  }

  // A finished flight's trail in RAM, else its losses: its record's, read back already, or (the newest HISTORY.readBack,
  // on the ground) read back now, one at a time, the list drawn again once they are all in. null: not known.
  trailOf(f) {
    const m = this.memory;
    if (m.trails?.has(f.id) || f.t1 == null) return m.trails?.get(f.id) ?? null;
    if (Number.isFinite(f.losses) || !m.trailOf) return f.losses ?? null;
    if (!this.losses.has(f.id) && this.losses.size < HISTORY.readBack) this.losses.set(f.id, null);
    return this.losses.get(f.id) ?? null;
  }

  readBack() {
    const m = this.memory, todo = () => [...this.losses].filter(([, n]) => n == null).map(([id]) => id);
    if (this.reading || !m || this.flying() || !todo().length) return;
    this.reading = (async () => {
      for (let id; (id = todo()[0]) && m === this.memory && !this.flying(); ) {
        const tr = await flightTrail(m, id);
        if (m === this.memory) this.losses.set(id, tr ? tr.filter((s) => s.lost).length : -1);
      }
    })().finally(() => {
      this.reading = null;
      if (m === this.memory) this.renderFlights();
    });
  }

  renderFlights() {
    const m = this.memory;
    if (!m) return;
    const now = this.now(), groups = groupFlights(m.flights, now);
    put(this.flightsEl, h("h4", {}, "Flights ", h("span", { class: "count" }, m.flights.length || "")),
      groups.length ? groups.map((g) => h("div", { class: "hist-day" }, h("h5", {}, g.day), h("ul", { class: "hist-list" }, g.flights.map((f) => {
        const n = this.trailOf(f), s = flightSummary(m, f, now, n === -1 ? null : n), on = this.flightId === f.id;
        return h("li", { class: "hist-fl" }, h("button", { type: "button", class: "hist-flight", "aria-pressed": String(on), title: on ? "Hide this flight's path" : "Show this flight's path on the map and in 3D",
          onclick: () => this.setFlight(on ? null : f.id) },
          h("span", { class: "hist-when" }, s.time, s.live && h("em", {}, " flying now")),
          h("span", { class: "hist-text" },
            h("strong", {}, `${s.length} · ${s.distance}${s.rooms.length ? ` · ${s.rooms.join(" → ")}` : ""}`),
            h("small", {}, [s.what && cap(s.what), seenWords(s), s.changes && plural(s.changes, "change"), s.lost && `position lost ${plural(s.lost, "time")}`].filter(Boolean).join(" · ")),
            s.outcome && h("small", { class: "hist-outcome" }, s.outcome),
            s.looked && h("small", {}, s.looked)),
          s.sim && h("span", { class: "badge" }, "simulator"),
          h("span", { class: "hist-go", "aria-hidden": "true" }, on ? "●" : "○")),
          s.report && h("details", { class: "hist-more hist-report" }, h("summary", {}, "More"), h("p", {}, s.report)));
      })))) : h("p", { class: "note" }, "No flights yet. Each flight's path, the rooms it went through and what it saw show here."));
    this.readBack();
  }

  dispose() {
    clearTimeout(this.timer);
    this.offs.forEach((f) => f());
    this.offs = [];
    this.card?.dispose();
    this.pics.clear();
    this.memory = null;
    put(this.el);
    this.el.classList.remove("hist");
  }
}
