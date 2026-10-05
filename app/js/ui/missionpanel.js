// The Missions panel: Go to / Look in a room or place, Find people, Check on pets, Patrol, Return home, Stop; the
// running mission's phase, and the battery budget (flight time left against the way home). It only builds mission
// objects (missions.js MissionRunner.run(m), docs/HOME-DRONE.md) and emits "run" m, "stop" and "pad" (the drone sits
// on its home pad: reset the position there); main.js runs them.
import { Emitter } from "../util.js";
import { h } from "./dom.js";

export const MISSIONS = {
  people: { kind: "searchFor", target: "person" },
  pets: { kind: "checkOnPets" },
  patrol: { kind: "patrol" },
  home: { kind: "returnHome" },
};

// The places "Go to" offers besides rooms: your landmarks, then furniture and fixtures from the capture, each name once.
// A mission names the place and the runner flies next to the nearest one called that (yours before the capture's), so
// count says how many it picks from.
export function places(house) {
  const out = new Map(), rank = (l) => (l.source === "user" ? 0 : l.source === "roomplan" ? 1 : 2);
  for (const l of [...(house.landmarks ?? [])].sort((a, b) => rank(a) - rank(b))) {
    const key = l.name.trim().toLowerCase(), first = out.get(key);
    if (!Number.isFinite(l.x) || /ceiling|fan|light|curtain|shade|vent|speaker|device|header|beam|column|stair|step/i.test(l.name)) continue;
    if (!first) out.set(key, { ...l, count: 1 });
    else if ((l.source === "user") === (first.source === "user")) first.count++;
  }
  return [...out.values()];
}

// What a button does with the choice in the list (t: { room } or { place }); places go by name, so the runner can
// find free space beside the thing and turn to face it.
export function missionFor(which, t) {
  if (which !== "goTo" && which !== "lookIn") return { ...MISSIONS[which] };
  if (!t) return null;
  if (which === "lookIn") return { kind: "lookIn", room: t.room?.id ?? t.place.room };
  return { kind: "goTo", target: t.room ? t.room.id : t.place.name };
}

export function describeMission(m, house) {
  const room = (id) => house?.rooms.find((r) => r.id === id || r.name === id)?.name ?? id;
  const where = (t) => (typeof t === "object" ? t.name ?? `${t.x.toFixed(1)}, ${t.y.toFixed(1)} m` : room(t));
  switch (m.kind) {
    case "goTo": return `Go to ${where(m.target)}`;
    case "lookIn": return `Look in ${room(m.room)}`;
    case "searchFor": return m.target === "person" ? "Find people" : `Search for ${m.target}`;
    case "checkOnPets": return "Check on the pets";
    case "patrol": return m.rooms?.length ? `Patrol ${m.rooms.map(room).join(", ")}` : "Patrol the house";
    case "returnHome": return "Return home";
    case "calibrate": return "Measure braking";
    default: return m.kind;
  }
}

export class MissionPanel extends Emitter {
  constructor(root) {
    super();
    const $ = (s) => root.querySelector(s);
    Object.assign(this, { root, select: $("#missionTarget"), state: $("#missionState"), note: $("#missionNote"), budget: $("#missionBudget"), pad: $("#btnOnPad"), why: $("#missionWhy") });
    Object.assign(this, { house: null, targets: [], blocked: "", ok: false });
    root.querySelectorAll("[data-mission]").forEach((b) => b.addEventListener("click", () => this.press(b.dataset.mission)));
    $("#btnMissionStop").addEventListener("click", () => this.emit("stop"));
    this.pad.addEventListener("click", () => this.emit("pad"));
    this.setContext({});
  }

  // available: the mission runner is loaded; reason: why not, shown instead of the buttons' work.
  setContext({ house = null, available = false, reason = "" } = {}) {
    this.house = house;
    const rooms = house?.rooms ?? [];
    this.targets = [...rooms.map((r) => ({ room: r })), ...(house ? places(house) : []).map((l) => ({ place: l }))];
    const where = (p) => (p.count > 1 ? `nearest of ${p.count}` : rooms.find((r) => r.id === p.room)?.name ?? "?");
    const opt = (t, i) => h("option", { value: i }, t.room ? t.room.name : `${t.place.name} (${where(t.place)})`);
    const keep = this.select.value;
    this.select.replaceChildren(
      !rooms.length && h("option", { value: "" }, "No house yet"),
      rooms.length > 0 && h("optgroup", { label: "Rooms" }, this.targets.map((t, i) => t.room && opt(t, i))),
      this.targets.some((t) => t.place) && h("optgroup", { label: "Places" }, this.targets.map((t, i) => t.place && opt(t, i))),
    );
    if (keep && this.targets[keep]) this.select.value = keep;
    const ok = (this.ok = !!house && available);
    this.enable();
    this.note.hidden = ok;
    this.note.textContent = !house ? "Add your house in Settings → House to fly missions in it." : reason;
    this.root.dataset.ready = ok ? "1" : "0";
  }

  // The real drone isn't ready ("Ready to fly?" has a blocking item): missions wait, with why; Return home stays.
  setBlocked(text = "") {
    if (text === this.blocked) return;
    this.blocked = text;
    this.why.hidden = !text || !this.ok;
    this.why.textContent = text ? `Not ready to fly a mission: ${text}` : "";
    this.enable();
  }

  enable() {
    this.root.querySelectorAll("[data-mission], #missionTarget").forEach((b) => (b.disabled = !this.ok || (!!this.blocked && b.dataset.mission !== "home")));
    if (this.why) this.why.hidden = !this.blocked || !this.ok;
  }

  press(which) {
    const m = missionFor(which, this.targets[this.select.value]);
    if (m) this.emit("run", m);
  }

  setStatus(s) {
    this.state.textContent = s ? [s.phase, s.text].filter(Boolean).join(": ") : "Idle";
    this.root.classList.toggle("running", !!s);
  }

  setBudget(text, level = "ok") {
    this.budget.textContent = text;
    this.budget.dataset.level = level;
  }
}
