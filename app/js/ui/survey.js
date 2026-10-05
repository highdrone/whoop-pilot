// Claude surveys the house, for Settings → House (docs/HOME-DRONE.md, Wave C; the PANELS contract): first what it costs
// (estimateSurvey: pictures and dollars, at most), then the OK (with Claude's picture checks on "ask" or "on", starting it
// is the consent for these pictures; "off" refuses), progress with Stop, then the review: per room the suggested name
// (editable), how much of the floor and ceiling the pictures saw and what they missed; landmarks and hazards each with
// its crop from the picture Claude boxed it in, ticked or not (new names at >= 0.6 that the user didn't set, new landmarks
// and hazards at >= 0.5; ones the map already has are shown as such), hazards as the no-fly zones they would become.
// Apply hands onApply({ rooms, landmarks, keepouts, accept, res, apply }) the accepted items (source "claude"); apply(house)
// is ai/survey.js applySurvey with the user's names, for editHouse(id, apply), then vox.applyObjects and saveVoxels.
// Then the real cost. A survey stops when the drone takes off (flying()); the review stays through a new map of the same
// house (setHouse), and throwing it away asks first (it was paid for).
//   new SurveyPanel(el, { house, map, vox, twin, settings, claude, onApply })   // survey, estimate: ai/survey.js's by default
//   twin: a Twin or an async function that loads one (released with releaseTwin(twin) after the survey).
import { Emitter } from "../util.js";
import { put, watchFlying, plainWords } from "./changecard.js";
import { h } from "./dom.js";
import { applySurvey, hazardKeepout, surveyHouse, estimateSurvey } from "../ai/survey.js";

export const PRETICK = { room: 0.6, item: 0.5 };
const HAZARD_WORDS = {
  "ceiling-fan": "Ceiling fan", "hanging-lamp": "Hanging lamp", plant: "Plant", curtain: "Curtain or blind cord", cable: "Loose cable", glass: "Glass",
  mirror: "Mirror", "pet-bowl": "Pet bowl", stairs: "Stairs", candle: "Candle", "open-flame": "Open flame", "shelf-edge": "Thin shelf", other: "Hazard",
};
const CALLOUT = {
  "ceiling-fan": "The drone keeps out of the whole column under the fan: its downdraft knocks a 36 g drone out of the air.",
  glass: "The camera and the 3D scan see through glass, so the map may think it's open space: accept this to block it.",
  mirror: "A mirror shows a room that isn't there: accept this to block it.",
  stairs: "Stairs change the floor height: the drone keeps away.",
  "open-flame": "Heat rises: blocked up to a metre above it.", candle: "Heat rises: blocked up to a metre above it.",
};
const cap = (s) => (s ? s.charAt(0).toUpperCase() + s.slice(1) : "");
const money = (v) => `$${(v ?? 0).toFixed(2)}`;
const pct = (v) => `${Math.round((v ?? 0) * 100)}%`;
const heights = (k) => (k.zMin == null ? "every height" : `${Math.max(0, k.zMin).toFixed(1)} to ${k.zMax.toFixed(1)} m up`);

const low = (v) => String(v ?? "").trim().toLowerCase();
const clean = (t) => String(t ?? "").trim().replace(/\.+$/, "");
// What the user sees ticked when the review opens: { rooms: Set(ids), landmarks: Set(i), hazards: Set(i) }. Rooms: a new
// name at >= 0.6 the user didn't set, and no other room has or is offered (two rooms of one name can't be told apart).
// Landmarks and hazards: new ones at >= 0.5; of landmarks with one name in one room only the surest ("go to the chair"),
// and none whose name the map already has in that room (a third "chair" makes "go to the chair" no clearer).
export function pretick(res, house, map = null) {
  const userNamed = new Set(house.rooms.filter((r) => r.nameSource === "user").map((r) => r.id));
  const ok = res.rooms.filter((r) => !r.error && r.suggestedName && r.confidence >= PRETICK.room && !userNamed.has(r.id) && low(r.suggestedName) !== low(r.name));
  const rooms = ok.filter((r) => !takenName(res, house, r)).map((r) => r.id);
  const best = new Map();
  res.landmarks.forEach((l, i) => {
    if (l.known || l.confidence < PRETICK.item || onMap(house, map, l.name, l.room)) return;
    const k = `${low(l.name)}|${l.room}`;
    if (!best.has(k) || res.landmarks[best.get(k)].confidence < l.confidence) best.set(k, i);
  });
  const hazards = res.hazards.map((it, i) => (!it.known && it.confidence >= PRETICK.item ? i : -1)).filter((i) => i >= 0);
  return { rooms: new Set(rooms), landmarks: new Set(best.values()), hazards: new Set(hazards) };
}
// A landmark of that name in that room already on the map.
export const onMap = (house, map, name, room) => (house.landmarks ?? []).some((m) => low(m.name) === low(name) && (m.room ?? map?.roomAt?.(m.x, m.y)?.id) === room);
// Another room's name (now, or Claude's suggestion for it) equal to the one suggested for r.
export const takenName = (res, house, r) => house.rooms.some((q) => q.id !== r.id && low(q.name) === low(r.suggestedName))
  || res.rooms.some((q) => q.id !== r.id && !q.error && low(q.suggestedName) === low(r.suggestedName));

// The accepted items as they go into house.json (source "claude"), and the call that writes them.
// picks: { rooms: Map(id -> name), landmarks: Map(i -> name), hazards: Set(i) }.
export function surveyChanges(house, res, picks, { map = null } = {}) {
  const edited = { ...res, rooms: res.rooms.map((r) => (picks.rooms.has(r.id) ? { ...r, suggestedName: picks.rooms.get(r.id) } : r)),
    landmarks: res.landmarks.map((l, i) => (picks.landmarks.has(i) ? { ...l, name: picks.landmarks.get(i) } : l)) };
  const accept = { rooms: [...picks.rooms.keys()], landmarks: [...picks.landmarks.keys()], hazards: [...picks.hazards] };
  const floor = (hz) => map?.floorAt?.(hz.x, hz.y) ?? house.rooms.find((r) => r.id === hz.room)?.floorZ ?? 0;
  return {
    rooms: accept.rooms.map((id) => edited.rooms.find((r) => r.id === id)).filter((r) => r?.suggestedName && house.rooms.find((q) => q.id === r.id)?.nameSource !== "user")
      .map((r) => ({ id: r.id, name: r.suggestedName, kind: r.kind, nameSource: "claude" })),
    landmarks: accept.landmarks.map((i) => edited.landmarks[i]).filter(Boolean).map((l) => ({ name: l.name, room: l.room, x: l.x, y: l.y, z: l.z, source: "claude", confidence: l.confidence })),
    keepouts: accept.hazards.map((i) => res.hazards[i]).filter(Boolean).map((hz) => hazardKeepout(hz, floor(hz))),
    accept, res: edited, apply: (target) => applySurvey(target, edited, accept, { map }),
  };
}

// A crop of the picture an item was boxed in, as a canvas (the box outlined, some room around it).
async function cropOf(view, box, alt, w = 112, ht = 84) {
  const c = h("canvas", { width: w * 2, height: ht * 2, class: "svy-crop", role: "img", "aria-label": alt });
  if (!view?.image || !box) return c;
  const bmp = await createImageBitmap(new Blob([Uint8Array.from(atob(view.image), (ch) => ch.charCodeAt(0))], { type: "image/jpeg" }));
  const g = c.getContext("2d"), [x0, y0, x1, y1] = box, bw = x1 - x0, bh = y1 - y0, side = Math.max(bw * 1.5, (bh * 1.5 * w) / ht, 96);
  const sw = Math.min(view.width, side), sh = Math.min(view.height, (side * ht) / w);
  const sx = Math.max(0, Math.min(view.width - sw, (x0 + x1) / 2 - sw / 2)), sy = Math.max(0, Math.min(view.height - sh, (y0 + y1) / 2 - sh / 2));
  const kx = c.width / sw, ky = c.height / sh;
  g.drawImage(bmp, sx, sy, sw, sh, 0, 0, c.width, c.height);
  bmp.close();
  g.strokeStyle = "#ffd84d";
  g.lineWidth = 3;
  g.strokeRect((x0 - sx) * kx, (y0 - sy) * ky, bw * kx, bh * ky);
  return c;
}

export class SurveyPanel extends Emitter {
  constructor(el, { house = null, map = null, vox = null, twin = null, settings, claude = null, survey = surveyHouse, estimate = estimateSurvey, onApply = null, releaseTwin = null, flying = () => false } = {}) {
    super();
    Object.assign(this, { el, settings, claude, survey, estimateFn: estimate, onApply, releaseTwin, flying, state: "idle", res: null, picks: null, ac: null, progress: null, applied: null, error: null });
    el.classList.add("svy");
    this.offs = [settings?.on?.("change", ({ key }) => {
      if (["aiModel", "model"].includes(key)) this.est = null;
      if (this.state === "idle" && ["aiVision", "apiKey", "aiModel", "model"].includes(key)) this.render();
    }), watchFlying(flying, (air) => {
      if (air && this.state === "running") this.stop("flying");
      else if (this.state === "idle") this.render();
    })].filter(Boolean);
    this.setHouse(house, map, vox, twin);
  }

  // Another house: what was going on goes. The same house (a new map after a door change, the 3D map, a twin): those only;
  // a running survey, its review or what was applied stay.
  setHouse(house, map, vox = null, twin = this.twin) {
    const same = house && house.id === this.house?.id;
    if (!same && this.state === "running") this.stop("house");
    Object.assign(this, { house, map, vox, twin }, !same && { state: "idle", res: null, picks: null, names: {}, applied: null, error: null, confirming: false });
    this.est = null;
    if (!same || this.state === "idle") this.render();
  }

  stop(why) {
    this.stopWhy = why;
    this.ac?.abort();
  }

  estimate() {
    if (!this.house || !this.map) return null;
    return (this.est ??= this.estimateFn({ house: this.house, map: this.map, settings: this.settings, model: this.claude?.model }));
  }

  // Why it can't start now, in words, or "".
  blocker() {
    if (!this.house || !this.map) return "Load a house first.";
    if (!this.claude?.configured) return "Add a Claude API key in Settings → Brain first.";
    if (this.settings?.get("aiVision") === "off") return "Claude's picture checks are off (Settings → Brain): turn them on to Ask or On to run a survey.";
    if (this.flying()) return "Land first: the survey uses the computer's graphics for a few minutes.";
    if (!this.twin) return "The 3D scan of this house isn't stored, so there's nothing to show Claude: import the capture again.";
    return "";
  }

  render() {
    if (this.disposed) return;
    const go = { idle: () => this.renderIdle(), running: () => this.renderRunning(), review: () => this.renderReview(), done: () => this.renderDone() }[this.state];
    go();
  }

  renderIdle() {
    const est = this.estimate(), why = this.blocker();
    if (!est) return put(this.el, h("p", { class: "note" }, "Load a house to have Claude survey it."));
    const t = est.total;
    put(this.el,
      h("p", {}, "Claude looks at pictures of your 3D scan, room by room, and suggests room names, landmarks you can send the drone to (\"the sofa\"), and hazards for a small drone (ceiling fans, plants, cables, glass) as no-fly zones. Nothing changes until you accept it."),
      h("div", { class: "svy-cost" }, h("b", {}, `About ${money(t.cost)}`), h("span", {}, ` for ${t.rooms} room${t.rooms === 1 ? "" : "s"} (${t.images} pictures); at most ${money(t.maxCost)} (${t.maxImages} pictures) if the floor or ceiling needs more views.`)),
      h("details", { class: "svy-rooms" }, h("summary", {}, "Per room"), h("ul", {}, est.rooms.map((r) => h("li", {}, `${r.name}: ${r.views} pictures, about ${money(r.cost)} (at most ${money(r.maxCost)})`)))),
      h("p", { class: "hint" }, `Starting sends these pictures of your house (renders of the 3D scan, no people) to Anthropic's Claude API with your API key, using ${est.model}. It takes a few minutes; you can stop it.`),
      this.error && h("p", { class: "warn-text" }, this.error),
      why ? h("p", { class: "note warn-text" }, why) : null,
      h("div", { class: "row" }, h("button", { type: "button", class: "btn", disabled: !!why, onclick: () => this.start() }, `Ask Claude to survey (about ${money(t.cost)})`)));
  }

  async start() {
    if (this.blocker() || this.state === "running") return;
    const est = this.estimate();
    Object.assign(this, { state: "running", error: null, stopWhy: null, asked: est.rooms.map((r) => ({ id: r.id, name: r.name })), progress: { text: "Loading the 3D scan…", index: 0, total: est.rooms.length }, ac: new AbortController() });
    this.render();
    let twin = null;
    try {
      twin = typeof this.twin === "function" ? await this.twin() : this.twin;
      const res = await this.survey({ house: this.house, map: this.map, vox: this.vox, twin, settings: this.settings, claude: this.claude, estimate: est, approved: true,
        signal: this.ac.signal, onProgress: (p) => this.onProgress(p) });
      if (!res.ok) throw new Error(res.stopped ?? res.rooms.find((r) => r.error)?.error ?? "no answer");
      this.review(res, false);
    } catch (e) {
      Object.assign(this, { state: "idle", error: this.stopWhy === "flying" ? "Stopped: the drone took off. Nothing was changed."
        : this.stopWhy || /^stopped$/.test(e.message) ? "Stopped: nothing was changed." : `The survey didn't finish: ${plainWords(e, "Claude didn't answer")}.` });
    } finally {
      if (twin && typeof this.twin === "function") this.releaseTwin?.(twin);
      this.ac = null;
      this.render();
    }
  }

  // A survey's result to review (also one Claude's survey_house tool ran: ToolBox.setAI's reviewSurvey).
  review(res, render = true) {
    const pre = pretick(res, this.house, this.map);
    if (render) Object.assign(this, { stopWhy: null, asked: null, confirming: false });
    Object.assign(this, { res, state: "review", error: null, names: {}, picks: { rooms: new Map([...pre.rooms].map((id) => [id, res.rooms.find((r) => r.id === id).suggestedName])),
      landmarks: new Map([...pre.landmarks].map((i) => [i, res.landmarks[i].name])), hazards: pre.hazards } });
    this.emit("review", res);
    if (render) this.render();
  }

  onProgress(p) {
    if (this.flying()) this.stop("flying");
    this.progress = { ...this.progress, ...(p.index != null && { index: p.index, total: p.total }), text: p.text ?? this.progress.text, phase: p.phase };
    if (this.state === "running" && !this.disposed) this.renderRunning();
  }

  renderRunning() {
    const p = this.progress, frac = p.total ? (p.index + (p.phase === "room" ? 1 : p.phase === "ask" ? 0.6 : 0.2)) / p.total : 0;
    put(this.el, h("div", { class: "import", "data-state": "running" },
      h("p", {}, h("b", {}, `Surveying: room ${Math.min(p.total, p.index + 1)} of ${p.total}`)),
      h("progress", { max: 1, value: Math.min(1, frac), "aria-label": "Survey progress" }),
      h("p", { class: "hint", role: "status", "aria-live": "polite" }, p.text)),
      h("div", { class: "row" }, h("button", { type: "button", class: "btn ghost", onclick: () => this.stop("you") }, "Stop")));
  }

  renderReview() {
    const { res, house, picks } = this, cost = res.cost, rooms = res.rooms.filter((r) => !r.error), failed = res.rooms.filter((r) => r.error);
    // Stopped here (Stop, take-off): the room it was on and those after it have no suggestions, said once in plain words.
    const cut = this.stopWhy ? failed.filter((r) => /abort|stopped/i.test(r.error)) : [], left = this.stopWhy ? (this.asked ?? []).filter((e) => !res.rooms.some((r) => r.id === e.id)) : [];
    const missing = [...cut, ...left].map((r) => r.name), others = failed.filter((r) => !cut.includes(r));
    const views = (it) => res.views.find((v) => v.room === it.views?.[0]?.room && v.index === it.views[0].view);
    const roomName = (id) => house.rooms.find((r) => r.id === id)?.name ?? id;
    const tick = (checked, label, fn) => h("input", { type: "checkbox", checked, "aria-label": label, onchange: (e) => (fn(e.target.checked), this.renderCount()) });
    const item = (kind, it, i) => {
      const crop = h("div", { class: "svy-crop-slot" });
      const on = kind === "landmark" ? picks.landmarks.has(i) : picks.hazards.has(i), title = kind === "landmark" ? it.name : HAZARD_WORDS[it.kind] ?? cap(it.kind);
      cropOf(views(it), it.views?.[0]?.box, `Where Claude saw the ${kind === "landmark" ? it.name : title.toLowerCase()}`).then((c) => put(crop, c), () => {});
      const ko = kind === "hazard" ? hazardKeepout(it, this.map?.floorAt?.(it.x, it.y) ?? 0) : null;
      const names = (this.names.landmarks ??= new Map()), box = tick(on, `Accept ${title}`, (v) => (kind === "landmark" ? (v ? picks.landmarks.set(i, names.get(i) ?? it.name) : picks.landmarks.delete(i)) : v ? picks.hazards.add(i) : picks.hazards.delete(i)));
      return h("li", { class: "svy-item", "data-kind": kind },
        h("label", { class: "svy-tick" }, box),
        crop,
        h("div", { class: "svy-what" },
          kind === "landmark" // a name typed is a name wanted: it ticks the item
            ? h("input", { type: "text", value: names.get(i) ?? it.name, "aria-label": "Landmark name", spellcheck: false,
              oninput: (e) => (names.set(i, e.target.value.trim() || it.name), picks.landmarks.set(i, names.get(i)), (box.checked = true), this.renderCount()) })
            : h("strong", {}, title),
          h("small", {}, `${roomName(it.room)} · ${pct(it.confidence)} sure${it.views?.length > 1 ? ` · seen in ${it.views.length} pictures` : ""}`),
          it.known && h("small", { class: "svy-known" }, `Already on the map as "${it.known}".`),
          kind === "landmark" && !it.known && (onMap(house, this.map, it.name, it.room)
            ? h("small", { class: "svy-known" }, `The map already has a "${it.name}" in this room: give this one its own name (say "${it.name} by the window") to keep it.`)
            : res.landmarks.some((o, j) => j !== i && low(o.name) === low(it.name) && o.room === it.room)
              && h("small", { class: "svy-known" }, `More than one "${it.name}" here: name each (say "${it.name} by the window") to send the drone to it.`)),
          kind === "hazard" && h("small", {}, `${it.why ? `${cap(it.why)}. ` : ""}No-fly zone ${(2 * ko.r).toFixed(1)} m across, ${heights(ko)}.`),
          CALLOUT[it.kind] && h("small", { class: "svy-callout" }, CALLOUT[it.kind])));
    };
    put(this.el,
      h("p", { class: "svy-sum" }, h("b", {}, `Claude looked at ${rooms.length} room${rooms.length === 1 ? "" : "s"} for ${money(cost.cost)}`), ` (estimated ${money(cost.estimate)}). Tick what to keep; nothing is on the map yet.`),
      others.length > 0 && h("p", { class: "warn-text" }, `Not surveyed: ${others.map((r) => `${r.name} (${clean(plainWords(r.error, "Claude didn't answer"))})`).join("; ")}.`),
      this.stopWhy ? h("p", { class: "warn-text" }, `${this.stopWhy === "flying" ? "The survey stopped when the drone took off" : "You stopped the survey"}${missing.length ? `: no suggestions for ${missing.join(", ")}` : ""}.`)
        : res.stopped && h("p", { class: "warn-text" }, `Stopped early: ${clean(plainWords(res.stopped, "Claude stopped answering"))}.`),
      h("h4", {}, "Rooms"),
      h("ul", { class: "svy-list" }, rooms.map((r) => {
        const own = house.rooms.find((q) => q.id === r.id)?.nameSource === "user", on = picks.rooms.has(r.id), names = (this.names.rooms ??= new Map());
        const box = Object.assign(tick(on, `Rename ${r.name}`, (v) => (v ? picks.rooms.set(r.id, names.get(r.id) ?? r.suggestedName) : picks.rooms.delete(r.id))), { disabled: own });
        return h("li", { class: "svy-room" },
          h("label", { class: "svy-tick" }, box),
          h("div", { class: "svy-what" },
            h("span", { class: "svy-rename" }, h("span", {}, `${r.name} → `),
              h("input", { type: "text", value: names.get(r.id) ?? r.suggestedName ?? "", disabled: own, "aria-label": `New name for ${r.name}`, spellcheck: false,
                oninput: (e) => (names.set(r.id, e.target.value.trim() || r.suggestedName), picks.rooms.set(r.id, names.get(r.id)), (box.checked = true), this.renderCount()) })),
            h("small", {}, `${cap(r.kind ?? "room")}, ${pct(r.confidence)} sure${own ? " · you named this room, so it keeps your name" : ""}`),
            takenName(res, house, r) && h("small", { class: "warn-text" }, "Another room is called that (or Claude suggests it for one): give this room its own name."),
            r.seen && h("small", {}, `The pictures saw ${pct(r.seen.floor)} of the floor and ${pct(r.seen.ceiling)} of the ceiling${r.more ? ` (${r.more} from extra spots)` : ""}.`),
            r.unseen?.text && h("small", { class: "warn-text" }, `Not seen: ${r.unseen.text.replace(/^[^:]+:\s*/, "")}. Hazards there may be missing.`)));
      })),
      h("h4", {}, "Landmarks ", h("span", { class: "muted" }, "places you can send the drone to")),
      res.landmarks.length ? h("ul", { class: "svy-list" }, res.landmarks.map((l, i) => item("landmark", l, i))) : h("p", { class: "note" }, "None found."),
      h("h4", {}, "Hazards ", h("span", { class: "muted" }, "proposed no-fly zones")),
      res.hazards.length ? h("ul", { class: "svy-list" }, res.hazards.map((hz, i) => item("hazard", hz, i))) : h("p", { class: "note" }, "None found."),
      this.error && h("p", { class: "warn-text" }, this.error),
      h("div", { class: "row wrap svy-apply" },
        this.countEl = h("button", { type: "button", class: "btn", onclick: () => this.apply() }),
        this.confirming
          ? h("span", { class: "svy-confirm", role: "group", "aria-label": "Throw the survey away?" }, `Throw away this ${money(cost.cost)} survey? `,
            h("button", { type: "button", class: "btn ghost small danger-text", onclick: () => this.discard() }, "Throw away"),
            h("button", { type: "button", class: "btn ghost small", onclick: () => ((this.confirming = false), this.render()) }, "Keep"))
          : h("button", { type: "button", class: "btn ghost", onclick: () => ((this.confirming = true), this.render()) }, "Discard")));
    this.renderCount();
  }

  renderCount() {
    const n = this.picks.rooms.size + this.picks.landmarks.size + this.picks.hazards.size;
    this.countEl.textContent = n ? `Put ${n} on the map` : "Nothing ticked";
    this.countEl.disabled = !n || !this.onApply;
  }

  async apply() {
    const changes = surveyChanges(this.house, this.res, this.picks, { map: this.map });
    this.countEl.disabled = true;
    try {
      const out = await this.onApply?.(changes);
      Object.assign(this, { state: "done", applied: { rooms: changes.rooms.length, landmarks: changes.landmarks.length, keepouts: changes.keepouts.length, out } });
      this.emit("applied", changes);
    } catch (e) {
      this.error = `Couldn't put it on the map: ${plainWords(e, "the house couldn't be saved")}. Nothing changed; try again.`;
    }
    this.render();
  }

  discard() {
    Object.assign(this, { state: "idle", res: null, picks: null, error: null, confirming: false, stopWhy: null });
    this.render();
  }

  renderDone() {
    const a = this.applied, c = this.res.cost, glass = this.res.hazards.some((hz, i) => this.picks.hazards.has(i) && /glass|mirror/.test(hz.kind));
    const parts = [a.rooms && `${a.rooms} room name${a.rooms === 1 ? "" : "s"}`, a.landmarks && `${a.landmarks} landmark${a.landmarks === 1 ? "" : "s"}`, a.keepouts && `${a.keepouts} no-fly zone${a.keepouts === 1 ? "" : "s"}`].filter(Boolean);
    put(this.el, h("div", { class: "import", "data-state": "done" },
      h("p", {}, h("b", {}, `On the map: ${parts.join(", ")}.`)),
      glass && h("p", { class: "hint" }, "The glass and mirrors are blocked in the 3D map too."),
      h("p", { class: "hint" }, `This survey cost ${money(c.cost)} (estimated ${money(c.estimate)}, ${c.requests} request${c.requests === 1 ? "" : "s"} to ${c.model}).`)),
      h("div", { class: "row" }, h("button", { type: "button", class: "btn ghost small", onclick: () => this.discard() }, "Done")));
  }

  dispose() {
    this.disposed = true;
    this.stop("closed");
    this.offs.forEach((f) => f?.());
    put(this.el);
    this.el.classList.remove("svy");
  }
}
