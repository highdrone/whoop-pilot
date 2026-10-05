// "Ready to fly?": the real drone's checklist, first on the page in real mode (and in "Rehearse the real flight"). Items
// are missions.js preflightCheck()'s (docs/HOME-DRONE.md: { id, ok, level: "block" | "warn" | "info", text, fix?: { label,
// action }, when }) with the vision side's merged in (nav/readiness.js: phase instead of when), plus the radio lines only
// the page knows; where the page knows better (no video, the radio not connected, a camera never calibrated) its item
// replaces the runner's. Without preflightCheck (an older missions.js) shellChecks() says what the page can tell. They
// split into what to do before the battery goes in and what needs it in, with a clock from the first video frame (unless
// the vision side's "standby" item keeps that clock): the O4 overheats after about 2.5 minutes armed on the ground. And
// the banner that asks before pictures of the house go to Claude: one line, never the keyboard (Space, L and Esc keep
// working), never over the flight view.
import { Emitter } from "../util.js";
import { h, fmtClock } from "./dom.js";

export const READY = { standbyS: 150, warnS: 90, flyingEvery: 5000 };

// Items that need the battery in (video, the pad check, the radio link): by id, unless an item says group itself.
const BATTERY = /video|picture|pad|battery|angle|switch|link|lq|heading|delay|standby/i;
export const groupOf = (it) => it.group ?? it.when ?? it.phase ?? (BATTERY.test(it.id) ? "battery" : "before");

// Every fix.action the items carry (missions.js, nav/readiness.js, the page's) -> what the page does (main.js readyFix).
export const FIXES = {
  radio: "radio", goggles: "goggles", house: "house", "import-house": "house", home: "home", map3d: "map3d", "build-3d": "map3d",
  db: "db", "build-db": "db", models: "prepare", prepare: "prepare", calib: "calib", calibrate: "calib", "calibrate-camera": "calib",
  pad: "pad", "crop-video": "crop", "vision-on": "vision", brake: "flight", "measure-braking": "flight", flight: "flight",
  brain: "brain", "ai-settings": "brain", charge: "charge",
};

// The first item that stops a mission, or null.
export const blocking = (items) => items.find((it) => !it.ok && it.level === "block") ?? null;

// In the air the list is checked again only while something blocks missions, every READY.flyingEvery ms (preflightCheck's
// coverage report costs the control loop as the 3D map grows): whether to keep the last list now.
export const keepReady = ({ flying, items, at, now }) => !!flying && (!blocking(items) || now - at < READY.flyingEvery);

// Mission-side items from missions.preflightCheck when it is there, the page's own otherwise; the radio's always (missions
// can't see them); one per id: the mission runner's, unless the page's item with that id says override and isn't ready.
// switchAsks: the radio is linked and only its AI switch is off: a mission asks for the switch as it starts (the page's
// "switch" line says so), so the runner's radio block doesn't hold the buttons (missions.run still refuses if it stays off).
export function readyItems({ missions = null, mode = "real", extra = {}, page = [], switchAsks = false }) {
  let runner = null;
  try {
    runner = missions?.preflightCheck?.({ mode, ...extra }) ?? null;
  } catch (e) {
    runner = [{ id: "preflight", ok: false, level: "warn", text: `The pre-flight check failed: ${e.message}` }];
  }
  const out = [], at = new Set(), mine = new Map(page.map((p) => [p.id, p])), base = runner ?? page.filter((p) => !p.radio), r = base.findIndex((it) => it?.id === "radio");
  // the radio's own lines right after the radio item (connect it, then its script), else at the end
  const radio = page.filter((p) => p.radio), list = r < 0 ? [...base, ...radio] : [...base.slice(0, r + 1), ...radio, ...base.slice(r + 1)];
  for (const it of list) {
    if (!it || at.has(it.id)) continue;
    at.add(it.id);
    const own = mine.get(it.id);
    if (switchAsks && it.id === "radio" && !it.radio && !it.ok) out.push({ ...it, ok: true, level: "info", text: "Radio connected over USB.", fix: undefined });
    else out.push(own && own !== it && own.override && !own.ok ? { ...it, ...own } : it);
  }
  return out;
}

// The radio's lines (only the page sees the USB port and the flight mode). r: { supported, connected, script, angle, engaged }.
// The USB link and the radio's script come before the battery (the radio needs no drone), next to each other.
export function radioChecks(r = {}) {
  const out = [], add = (id, ok, level, text, fix = null, more = {}) => out.push({ id, ok, level, text, radio: true, ...(fix && { fix }), ...more });
  add("radio", !!r.connected, "block", !r.supported ? "This browser can't talk to the radio over USB: use Chrome." : r.connected ? "Radio connected over USB." : "Connect the radio over USB-C.",
    r.supported && !r.connected ? { label: "Connect the radio (USB)", action: "radio" } : null, { override: !r.connected, ...(!r.connected && { group: "before" }) });
  add("radio-script", !!r.script, "block", r.script ? "The radio's AI script is talking." : "The radio's AI script isn't answering (setup guide, step 1).", null, { group: "before" });
  add("angle", !!r.angle, "block", r.angle ? "Quad in ANGLE (self-level) mode." : "Put the quad in ANGLE (self-level) mode.");
  add("switch", !!r.engaged, "info", r.engaged ? "AI switch on." : "AI switch off: flip it on when the app asks.");
  return out;
}

// The page's radio blocks for missions.preflightCheck (HomeSession pageChecks): only what isn't ready, and not the radio
// link itself (missions checks that one).
export const radioBlocks = (r) => radioChecks(r).filter((it) => !it.ok && it.level === "block" && it.id !== "radio");

// What the page knows. s: { house, home (pad set), vox: { usable, why } | null, building, coverage (report|null), webgpu,
// detector: { ready, backend, failed }, depth: { ready, on }, splat: { enabled, calibOk, db, trust, padcheck (its text) } | null, calib,
// memory: { open, stored }, ai: { key, vision, budget }, brake: { measured }, radio: { supported, connected, script, angle,
// engaged }, video: { ready, picture, fps }, vbat, flying }.
export function shellChecks(s) {
  const out = radioChecks(s.radio), add = (id, ok, level, text, fix = null, more = {}) => out.push({ id, ok, level, text, ...(fix && { fix }), ...more });
  if (!s.house) {
    add("house", false, "block", "No house yet: import your capture first.", { label: "Open House settings", action: "house" });
    return out;
  }
  add("house", true, "info", "Your house is loaded.");
  add("home", !!s.home, "block", s.home ? "Home pad set on the map." : "Set the home pad on the map (the Home tool).", s.home ? null : { label: "Set it", action: "home" });
  const cov = s.coverage?.score;
  add("map3d", !!s.vox?.usable, "block", s.vox?.usable ? `3D map built${Number.isFinite(cov) ? `: ${Math.round(cov)}% of the flying space is known` : ""}.`
    : s.building ? "The 3D map is being built…" : `No 3D map${s.vox?.why ? ` (${s.vox.why})` : ""}: real missions need it.`,
    s.vox?.usable || s.building ? null : { label: "Build it", action: "map3d" });
  add("webgpu", !!s.webgpu, "block", s.webgpu ? "WebGPU works: vision runs fast enough." : "This browser has no WebGPU: vision would be too slow for real missions. Use Chrome on this Mac.");
  const d = s.detector ?? {};
  add("models", !!d.ready && !!s.depth?.ready, "warn", d.ready && s.depth?.ready ? `The vision models are ready${d.backend ? ` (${d.backend})` : ""}.`
    : `Vision models still to load: ${[!d.ready && "the person detector", !s.depth?.ready && "live depth"].filter(Boolean).join(" and ")} (they download once).`);
  const sp = s.splat;
  if (sp?.enabled) {
    const never = !s.calib?.fx; // a first calibration happens on the pad with the battery in (the guide's step 8)
    add("calib", !!sp.calibOk, "block", sp.calibOk ? "Camera calibrated for this video." : never
      ? "Calibrate the camera: the drone on its pad, a slow turn by hand (Settings → House → Camera calibration), or later from a recording of that turn."
      : "Calibrate the camera again (on the pad, or from a recorded flight).",
      sp.calibOk ? null : { label: "Calibrate the camera", action: "calib" }, { override: !sp.calibOk && never, ...(never && { group: "battery" }) });
    add("db", !!sp.db, "block", sp.db ? "Position database built." : "Build the position database (about 30 s, on the ground).", sp.db ? null : { label: "Build it", action: "db" });
    // the vision side's own words when it isn't verified (another lens or picture, on the CPU, why it failed): an old pass's
    // text would read like a pass
    add("padcheck", sp.trust === "verified", "block", sp.trust === "verified" ? "The camera matched the 3D scan on the pad."
      : sp.padcheck ?? "Put the drone on its home pad and press “The drone is on its home pad”.",
      sp.trust === "verified" ? null : { label: "It's on the pad", action: "pad" });
  } else add("vision", false, "block", "Vision position is off (Settings → Flight): real missions need it.", { label: "Settings", action: "flight" });
  const v = s.video ?? {};
  add("picture", !!v.ready && v.picture !== "looking", v.ready ? "warn" : "block", !v.ready ? "No video from the drone yet: goggles on and plugged in, battery in."
    : v.picture === "looking" ? "Looking for the camera picture in the video…" : `Video from the goggles${v.fps ? ` (${v.fps} fps)` : ""}.`,
    v.ready ? null : { label: "Connect goggles", action: "goggles" }, { override: !v.ready, group: "battery" });
  // without video the machine's pace and the camera's fixes can't be judged yet (the picture line blocks meanwhile; the
  // runner still refuses on its own items)
  if (!v.ready) {
    add("pace", false, "info", "Whether this computer keeps up is checked once the video runs.", null, { override: true, group: "battery" });
    add("fix", false, "info", "Camera position fixes start once the video shows.", null, { override: true, group: "battery" });
  }
  add("battery", s.vbat > 3.7, s.vbat ? "warn" : "info", s.vbat ? `Battery ${s.vbat.toFixed(2)} V${s.vbat > 3.7 ? "" : ": charge it first"}.` : "Battery: waits for the radio's telemetry.");
  add("brake", !!s.brake?.measured, "warn", s.brake?.measured ? "Braking measured." : "Braking not measured yet: the drone flies at most 0.3 m/s.",
    s.brake?.measured ? null : { label: "How", action: "brake" });
  add("memory", !!s.memory?.open, "info", s.memory?.open ? (s.memory.stored ? "Flight memory open." : "Flight memory works, but this browser won't keep it.") : "Flight memory not open yet.");
  const ai = s.ai ?? {};
  add("ai", true, "info", !ai.key ? "No Claude key: no picture checks (Settings → Brain)." : ai.vision === "off" ? "Claude's picture checks are off."
    : `Claude's picture checks: ${ai.vision === "on" ? "on" : "asks first"}, up to $${Number(ai.budget ?? 0).toFixed(2)} a flight.`, { label: "Change", action: "brain" });
  return out;
}

export class ReadyList extends Emitter {
  constructor(root) {
    super();
    const $ = (s) => root.querySelector(s);
    Object.assign(this, { root, state: $("#readyState"), before: $("#readyBefore"), battery: $("#readyBattery"), timer: $("#readyTimer"), key: "", since: null, flying: false });
  }

  // items as readyItems() returns them; video: the real video shows a picture now; flying: the drone is in the air.
  render(items, { video = false, flying = false, now = performance.now() } = {}) {
    if (flying !== this.flying) {
      this.flying = flying;
      if (flying) this.root.open = false;
    }
    this.since = video && !flying && !items.some((it) => it.id === "standby") ? (this.since ?? now) : null; // one overheat clock
    const s = this.since == null ? null : (now - this.since) / 1000;
    this.timer.textContent = s == null ? "" : `· ${fmtClock(s * 1000)} since the video started`;
    this.timer.dataset.level = s == null ? "" : s > READY.standbyS ? "bad" : s > READY.warnS ? "warn" : "";
    const block = items.filter((it) => !it.ok && it.level === "block"), warn = items.filter((it) => !it.ok && it.level === "warn");
    this.state.textContent = block.length ? `${block.length} to fix` : warn.length ? `Ready, ${warn.length} to note` : "Ready";
    this.root.dataset.state = block.length ? "block" : warn.length ? "warn" : "ok";
    const key = JSON.stringify(items.map((it) => [it.id, it.ok, it.level, it.text, it.fix?.label]));
    if (key === this.key) return;
    this.key = key;
    const row = (it) => h("li", { "data-ok": it.ok ? "yes" : "no", "data-level": it.level },
      h("span", { class: "t" }, it.text),
      !it.ok && it.fix && h("button", { type: "button", class: "btn ghost small", onclick: () => this.emit("fix", it) }, it.fix.label));
    // what to fix first (blocks, then warnings, then notes), and what is ready folded into one line
    const rank = { block: 0, warn: 1, info: 2 }, todo = (list) => list.filter((it) => !it.ok).sort((a, b) => rank[a.level] - rank[b.level]);
    const group = (g) => {
      const list = items.filter((it) => groupOf(it) === g), done = list.filter((it) => it.ok);
      return [...todo(list).map(row), done.length > 0 && h("li", { class: "ready-done", "data-ok": "yes", "data-level": "ok" },
        h("details", {}, h("summary", {}, `${done.length} ready`), h("ul", {}, done.map((it) => h("li", {}, it.text)))))];
    };
    this.before.replaceChildren(...group("before").filter(Boolean));
    this.battery.replaceChildren(...group("battery").filter(Boolean));
  }
}

const PURPOSE = { change: "check a change the drone noticed", detection: "check an uncertain person or pet alert", survey: "survey the house" };

// Claude's "consent-needed" ({ purpose }) -> a one-line banner under the flight view: Allow once, Always allow (remembered),
// Not now, and under Details the whole story and Never. claude.answerConsent() takes the answer. It shows without
// focusing anything.
export class ConsentBanner {
  constructor(root, { claude, settings }) {
    Object.assign(this, { root, claude, settings, purposes: new Set() });
    root.querySelectorAll("[data-answer]").forEach((b) => b.addEventListener("click", () => this.answer(b.dataset.answer)));
    claude?.on("consent-needed", ({ purpose }) => this.show(purpose));
    claude?.on("consent", () => this.hide());
  }

  budget() {
    return Number(this.settings.get("aiBudget") ?? 0.5).toFixed(2);
  }

  short() {
    const why = [...this.purposes].map((p) => PURPOSE[p] ?? p);
    return `Claude wants to see pictures of your house${why.length ? ` to ${why.join(" and ")}` : ""} (up to $${this.budget()} a flight).`;
  }

  text() {
    return `To confirm changes the drone notices and uncertain person or pet alerts, Whoop Pilot sends small pictures (from the drone camera and `
      + `your 3D scan) to Anthropic's Claude API with your API key, up to $${this.budget()} per flight. “Allow once” lets the waiting check through; `
      + `“Always allow” remembers it (Settings → Brain changes it). Voice commands always include the current camera view.`;
  }

  show(purpose) {
    if (purpose) this.purposes.add(purpose);
    this.root.querySelector("[data-short]").textContent = this.short();
    this.root.querySelector("[data-text]").textContent = this.text();
    this.root.hidden = false;
  }

  hide() {
    this.purposes.clear();
    this.root.hidden = true;
  }

  // "on" (always allow, remembered), "once", "no" (not now), "off" (never)
  answer(a) {
    this.claude?.answerConsent(a === "no" ? false : a);
    this.hide();
  }
}
