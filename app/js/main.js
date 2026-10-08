import { Settings } from "./settings.js";
import { Simulator } from "./sim/simulator.js";
import { makeHouseWorld } from "./sim/twin-world.js";
import { Twin } from "./twin/twin.js";
import { Perception, simSource, CameraSource, ScreenSource } from "./perception.js";
import { FlightController } from "./controller.js";
import { ToolBox } from "./tools.js";
import { Agent } from "./agent.js";
import { LocalPlanner } from "./localplanner.js";
import { Voice } from "./voice.js";
import { SerialRadio } from "./radio.js";
import { GogglesSource, serverSession } from "./goggles/source.js";
import { openStore } from "./house/store.js";
import { plan } from "./house/planner.js";
import { houseFrame } from "./house/frames.js";
import { buildVoxels, voxelCentres } from "./house/voxels.js";
import { coverageReport } from "./house/coverage.js";
import { HouseMemory } from "./memory/memory.js";
import { Claude } from "./ai/claude.js";
import { Inspector } from "./ai/inspect.js";
import { estimateSurvey, surveyHouse, applySurvey, surveySummary } from "./ai/survey.js";
import { summarizeFlight } from "./ai/recall.js";
import { drawFpv, drawMinimap, statusChips } from "./hud.js";
import { MapView } from "./ui/mapview.js";
import { HousePanel } from "./ui/housepanel.js";
import { MissionPanel, describeMission } from "./ui/missionpanel.js";
import { EventsList, bindAlertSettings, snapshotUrl, normalizeEvent, SimpleChangeCard } from "./ui/events.js";
import { FlightRecorder } from "./ui/recorder.js";
import { Turns, handOver as handOverTo } from "./ui/autonomy.js";
import { HomeSession, plainError } from "./ui/session.js";
import { ReadyList, ConsentBanner, readyItems, shellChecks, radioBlocks, blocking, keepReady, FIXES } from "./ui/ready.js";
import { fmtClock } from "./ui/dom.js";
import { DEG, clamp } from "./util.js";

// Autonomy modules (docs/HOME-DRONE.md, "Wave B" and "Wave C" contracts), used when they are there: the page works
// without them. Then the house panels ("Wave C as wired"): each one's place on the page says so when it is missing.
const optional = (path) => import(path).catch((e) => (console.info(`${path}: not available (${e.message})`), null));
const [LOC, MIS, SAF, ALR, SPL, DEP, CHG, LENS, RPL] = await Promise.all(["./nav/localizer.js", "./missions.js", "./safety.js", "./alerts.js",
  "./nav/splatloc.js", "./vision/depth.js", "./nav/changes.js", "./nav/lens.js", "./nav/replay.js"].map(optional));
const [V3D, HIS, SUR, CAL, COV, CHC, RCK] = await Promise.all(["./ui/view3d.js", "./ui/history.js", "./ui/survey.js", "./ui/calib.js", "./ui/coverage.js",
  "./ui/changecard.js", "./ui/recheck.js"].map(optional));

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => [...document.querySelectorAll(sel)];
const r3 = (v) => (Number.isFinite(v) ? Math.round(v * 1000) / 1000 : v);

const FLIGHT_S = 240; // usable flight on a full 480 mAh pack (docs/HOME-DRONE.md: about 4 min)

const settings = new Settings();
const perception = new Perception();
const ctl = new FlightController({ settings, perception });
const voice = new Voice(settings);
const alerts = make("Alerts", () => ALR && new ALR.Alerts({ settings, voice, speak: (t) => voice.speak(t) }));
const tools = new ToolBox({ ctl, perception, settings, alerts, speak: (t) => droneSays(t) });
const agent = new Agent({ tools, settings, ctl });
const local = new LocalPlanner({ tools, ctl, settings });
const serial = new SerialRadio();
const camera = new CameraSource();
const screen = new ScreenSource();
const goggles = new GogglesSource();
let sim = null;
let source = null;
let cropMode = null; // while dragging a crop rectangle on the FPV view
let planner = null; // the planner running the current mission
const subtitles = [];

// Claude's picture checks (survey, changes, uncertain detections): consent through a banner that never takes the keyboard,
// never asked in the air; the Inspector checks each new suspected change once.
const claude = new Claude({ settings, agent });
const inspector = new Inspector({ claude, watch: true });
alerts?.setInspector?.(inspector);
claude.setConsent(null, { canAsk: () => !ctl.isFlying() });
claude.on("error", (m) => log("error", m));

// The active house and everything that flies it (ui/session.js): session.house, .map (HomeMap), .ortho (orthophoto
// bitmap), .twin (the simulator's splat camera), .vox (the 3D map), .memory, and the localizer, safety layer and missions.
let store = null;
const houseStore = () => (store ??= openStore().catch((e) => ((store = null), Promise.reject(e))));
const session = new HomeSession({ ctl, perception, settings, store: houseStore, edit: (id, fn) => editHouse(id, fn), alerts, tools, inspector, sim: () => sim,
  pageChecks: () => radioBlocks(radioState()), mods: { LOC, SAF, MIS, SPL, DEP, CHG, HouseMemory, Twin, buildVoxels, voxelCentres, houseFrame, coverageReport } });
const home = session;
let localizer = null;
let safety = null;
let missions = null;
let missionState = null; // the running mission's last status
const turns = new Turns(); // a stop cancels commands still waiting for the hand-over

function make(what, fn) {
  try {
    return fn() || null;
  } catch (e) {
    console.error(e);
    queueMicrotask(() => log("error", `${what} didn't start: ${e.message}`));
    return null;
  }
}

// ---------------------------------------------------------------- mode & sources

function setMode(mode) {
  stopMission("switched mode");
  settings.set("mode", mode);
  document.body.dataset.mode = mode;
  $$("#modeSeg button").forEach((b) => b.classList.toggle("on", b.dataset.mode === mode));
  if (mode === "sim") {
    if (!sim) {
      sim = new Simulator({ hfov: settings.get("hfov"), uptilt: settings.get("uptilt"), detections: settings.get("simDetections"), augment: settings.get("simAugment") || null });
      sim.videoDelay = settings.get("videoDelay");
      sim.on("tone", ({ freq, ms, delay }) => beep(freq, ms, delay));
      sim.on("error", (m) => simError(m));
      if (home.house && settings.get("simMode") === "house") setSimWorld();
    }
    session.dropLocTwin(); // the real drone's twin: the simulator has its own
    source = simSource(sim);
    ctl.attach(sim.radio, "sim");
  } else {
    source = goggles.active ? goggles : screen.stream ? screen : camera;
    ctl.attach(serial, "real");
    localizer?.forget(); // where the simulator left it says nothing about the real drone
    if (source === camera && settings.get("cameraId") && !camera.stream) startCamera(settings.get("cameraId"));
    if (source === camera && GogglesSource.webUsbSupported && goggles.directUsbWorked) goggles.connectUsb({ prompt: false }).then((ok) => ok && useGoggles()).catch(() => {});
    if (SerialRadio.supported && !serial.connected) serial.connect({ prompt: false }).catch(() => {});
  }
  perception.setSource(source);
  session.sync();
}

async function startCamera(deviceId) {
  try {
    screen.stop();
    goggles.stop();
    await camera.start(deviceId);
    camera.crop = settings.get("crop");
    settings.set("cameraId", deviceId || "");
    useRealVideo(camera);
    log("info", `Video: ${camera.label}`);
    await fillCameraList();
  } catch (e) {
    log("error", `Couldn't open the video source: ${e.message}`);
  }
}

// Window capture: e.g. iPhone Mirroring with DJI Fly's live view. Chrome asks which window to use.
async function startScreenCapture() {
  try {
    await screen.start();
    camera.stop();
    goggles.stop();
    screen.crop = settings.get("crop");
    screen.onEnded = () => {
      log("error", "Window capture ended - the drone video is gone.");
      refreshPanels(true);
    };
    useRealVideo(screen);
    log("info", `Video: capturing “${screen.label}”. Use “Crop to video” so only the camera picture is used.`);
    if (!settings.get("videoDelay")) {
      settings.set("videoDelay", 250);
      log("info", "Assuming about 250 ms of video delay for goggles-to-Mac video (Settings → Camera).");
    }
  } catch (e) {
    if (e.name !== "NotAllowedError") log("error", `Couldn't capture a window: ${e.message}`);
  }
}

// DJI Goggles 3 straight into the app, no viewer app (see js/goggles/). Normally through Whoop Pilot's server, which
// does the USB (or Wi-Fi) part outside Chrome; "direct" tries WebUSB, which only works for goggles that expose their
// network interface with a class Chrome allows (not Goggles 3).
async function startGoggles({ direct = false } = {}) {
  try {
    const ok = direct ? await goggles.connectUsb() : await goggles.connectHelper();
    if (ok) useGoggles();
    else if (goggles.status === "no-helper") log("info", goggles.describe());
  } catch (e) {
    if (e.name !== "NotFoundError") log("error", goggles.lastError || `Goggles: ${e.message}`);
  }
}

function useGoggles() {
  camera.stop();
  screen.stop();
  goggles.crop = settings.get("crop");
  useRealVideo(goggles);
  log("info", goggles.via === "usb"
    ? "Goggles: opening the USB link directly. Video appears once the drone is powered and the goggles show a picture."
    : "Goggles connected through the server. Video appears once the goggles are plugged in, the drone is powered and the goggles show a picture.");
  if (!settings.get("videoDelay") || settings.get("videoDelay") === 250) settings.set("videoDelay", 90);
}

// The server is polled while the goggles UI is showing (real mode, or the Camera tab) and no other real video
// source is in use, so running start.command is all it takes.
function syncGogglesWatch() {
  const cameraTab = $("#settings").open && !$("#tabVideo").hidden;
  goggles.watchHelper((settings.get("mode") === "real" || cameraTab) && !camera.stream && !screen.stream);
}

goggles.on("connected", useGoggles);
goggles.on("status", () => refreshPanels(true));
goggles.on("log", (m) => console.log("[goggles]", m));
goggles.on("error", (m) => log("error", m));

function useRealVideo(src) {
  if (settings.get("mode") !== "real") return;
  source = src;
  perception.setSource(src);
  refreshPanels(true);
}

// Crop: drag a rectangle on the FPV view around the actual camera picture.
function startCrop() {
  if (!source || source.kind === "sim" || !source.ready()) return log("error", "No video from the drone yet: connect the goggles first (Settings → Camera).");
  source.crop = null;
  cropMode = { dragging: false };
  $("#settings").close();
  log("info", "Drag a box around the drone's camera picture on the video view. Esc cancels.");
}

function setupCropDrag() {
  const c = $("#fpv");
  const pos = (e) => {
    const r = c.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  };
  c.addEventListener("pointerdown", (e) => {
    if (!cropMode) return;
    const [x, y] = pos(e);
    Object.assign(cropMode, { dragging: true, x0: x, y0: y, x1: x, y1: y });
    c.setPointerCapture(e.pointerId);
  });
  c.addEventListener("pointermove", (e) => {
    if (!cropMode?.dragging) return;
    [cropMode.x1, cropMode.y1] = pos(e);
  });
  c.addEventListener("pointerup", () => {
    if (!cropMode?.dragging) return;
    const f = lastFrameRect;
    const m = cropMode;
    cropMode = null;
    const x = (Math.min(m.x0, m.x1) - f.fx) / f.fw;
    const y = (Math.min(m.y0, m.y1) - f.fy) / f.fh;
    const w = Math.abs(m.x1 - m.x0) / f.fw;
    const h = Math.abs(m.y1 - m.y0) / f.fh;
    const box = w > 0.05 && h > 0.05 ? { x: Math.max(0, x), y: Math.max(0, y), w: Math.min(1 - Math.max(0, x), w), h: Math.min(1 - Math.max(0, y), h) } : null;
    const crop = box && (source.cropFromView?.(box) ?? box); // the box is drawn on the picture the HUD shows (region()): kept in the stream's terms
    settings.set("crop", crop);
    camera.crop = screen.crop = goggles.crop = crop;
    perception.setSource(source);
    log("info", crop ? "Cropped the video to your box." : "Crop cleared.");
  });
}

// ---------------------------------------------------------------- your house

// Loads a house from this browser's storage and makes it the active one ("" for none).
let houseGen = 0;
async function useHouse(id) {
  const gen = ++houseGen;
  settings.set("houseId", id || "");
  if (!id) return setHouse(null, null, null);
  try {
    const s = await houseStore();
    const house = await s.loadHouse(id);
    if (!house) throw new Error("it's no longer stored in this browser");
    const map = await s.loadMap(house);
    const png = house.orthophoto && (await s.readFile(id, "orthophoto.png"));
    const ortho = png ? await createImageBitmap(new Blob([png], { type: "image/png" })) : null;
    if (gen !== houseGen) return;
    setHouse(house, map, ortho);
    if (map.stale) log("info", "This house has no stored 3D scan, so its map has walls and no-fly zones but no furniture. Use its capture again in Settings → House.");
  } catch (e) {
    console.error(e);
    if (gen !== houseGen) return;
    log("error", `Couldn't load your house: ${e.message}`);
    settings.set("houseId", "");
    setHouse(null, null, null);
  }
}

function setHouse(house, map, ortho) {
  stopMission("the house changed");
  document.body.dataset.house = house ? "1" : "";
  mapView.setHouse(house, map, ortho);
  session.setHouse(house, map, ortho);
  if (sim && (house || sim.world.kind === "house")) setSimWorld();
  panelsForHouse();
}

// Applies an edit to a stored house (the active one in place) and saves it.
async function editHouse(id, fn) {
  const s = await houseStore();
  const active = home.house?.id === id;
  const house = active ? home.house : await s.loadHouse(id);
  fn(house);
  await s.saveHouse(house);
  if (!active) return;
  const named = (rooms) => rooms.forEach((r) => (r.name = house.rooms.find((q) => q.id === r.id)?.name ?? r.name));
  named(home.map.rooms);
  home.map.finalize();
  if (sim?.world.house === house) {
    named(sim.world.rooms);
    sim.world.start = sim.world.startPose();
  }
  mapView.invalidate();
  session.sync();
}

// The simulator's world: the demo apartment, or the active house (its splat twin as the camera once that loads).
let worldGen = 0;
async function setSimWorld() {
  if (!sim) return;
  const gen = ++worldGen, house = settings.get("simMode") === "house" ? home.house : null, old = home.twin;
  stopMission("the simulator world changed");
  sim.setTwin(null);
  session.setSimTwin(null); // vision and live depth let go of it before it goes
  old?.dispose();
  if (!house) {
    if (sim.world.kind !== "demo") sim.setHouse({});
    return session.sync();
  }
  const cast = settings.get("simActors") === false ? { cast: [] } : {};
  sim.useWorld(makeHouseWorld(house, home.map, { seed: sim.seed, ...cast }));
  ctl.airborne = false;
  session.sync();
  // the drone starts again on its pad, and so does a position the simulator's truth doesn't give (People & pets switched
  // mid-flight had left the estimate in the air where the drone was: vision looked from there and no mission could start)
  if (localizer && localizer.source !== "truth" && house.home) setTimeout(() => gen === worldGen && !ctl.isFlying() && session.pad(), 150);
  if (!house.splatFile) return log("info", "No 3D scan is stored for this house, so the simulator shows its simple camera. Use the capture again in Settings → House.");
  const line = log("info", "Loading your house's 3D scan into the simulator camera…");
  try {
    const bytes = await (await houseStore()).readFile(house.id, house.splatFile);
    if (!bytes) throw new Error("the scan file is missing");
    const twin = await Twin.create({ splat: bytes, house });
    if (gen !== worldGen) return twin.dispose();
    sim.setTwin(twin);
    session.setSimTwin(twin);
    line.querySelector(".t").textContent = "The simulator camera now shows your house's 3D scan.";
  } catch (e) {
    if (gen === worldGen) log("error", `The 3D scan didn't load, so the simulator shows its simple camera: ${e.message}`);
  }
}

// ---------------------------------------------------------------- autonomy: position, safety, missions

// The session made or moved the localizer, safety layer and missions (the first time: the mission UI hooks on).
function onBuilt(s) {
  const first = !missions && s.missions;
  ({ localizer, safety, missions } = s);
  if (first) {
    missions.on("status", (st) => missionStatus(st));
    missions.on("finding", (f) => events.add({ ...f, t: Date.now(), text: `${cap(f.label)} ${f.where ?? `in the ${f.roomName ?? roomName(f.room)}`}${f.near ? `, near the ${f.near}` : ""}.` }));
    missions.on("done", (r) => {
      finishMissionStep(r?.ok !== false);
      missionState = null;
      missionPanel.setStatus(null);
    });
  }
  mapView.set({ localizer, missions });
}

const inHouse = () => session.inHouse();

// After the session synced (mode, world, twin, settings): what the page shows.
function syncUi() {
  const on = inHouse();
  document.body.dataset.map = on ? "house" : settings.get("mode") === "real" ? "none" : "demo";
  $("#minimap").hidden = on;
  $("#mapTitle").textContent = on ? home.house.name : "Apartment";
  syncMapCanvas();
  syncViews();
  const reason = !missions ? "Missions aren't part of this version of the app yet."
    : !on ? "The simulator is flying the demo apartment: switch it to My house to fly missions there." : "";
  missionPanel.setContext({ house: home.house, available: !!missions && on, reason });
  mapView.set({ vox: home.vox, memory: session.memory, coverage: () => session.coverage({ maxAge: 30000 }) });
  document.body.dataset.rehearse = session.rehearsing() ? "1" : "";
  syncPanelTwins();
  refreshPanels(true);
}

function missionStatus(s) {
  if (s?.phase === "done") return; // the "done" event closes the mission's last step
  missionState = s;
  missionPanel.setStatus(s);
  const text = [cap(s?.phase), s?.text].filter(Boolean).join(": ");
  if (!text || text === missionStep?.dataset.text) return;
  finishMissionStep(true);
  missionStep = log("step", text, { pending: true, own: true });
  missionStep.dataset.text = text;
}

let missionStep = null;
function finishMissionStep(ok, detail = "") {
  const el = missionStep;
  missionStep = null;
  if (!el) return;
  el.classList.remove("pending");
  el.classList.add(ok ? "ok" : "fail");
  el.querySelector(".detail").textContent = detail;
}

async function runMission(m, origin = "ui") {
  if (!missions || !inHouse()) return;
  if (settings.get("autonomy") === "observer") return droneSays("I'm in observer mode, so I won't fly. Switch to Co-pilot or Full auto for missions.");
  const block = m.kind !== "returnHome" && blocking(readyCheck());
  if (block) return droneSays(`I'm not ready to fly a mission: ${block.text}`);
  stopMission();
  const what = describeMission(m, home.house);
  log("pilot", what);
  subtitle("pilot", what);
  recorder.command("mission", m, origin);
  if (!(await handOver())) return;
  planner = missions;
  missionStatus({ phase: "starting", text: what });
  try {
    const r = await missions.run(m);
    if (r?.summary) droneSays(r.summary);
  } catch (e) {
    finishMissionStep(false, e.message);
    log("error", `The mission stopped: ${e.message}`);
  } finally {
    if (planner === missions) planner = null;
    missionState = null;
    missionPanel.setStatus(null);
    if (!missions.busy) mapView.setPath(null);
    events.refresh();
  }
}

// Battery against the way home: flight time left (the safety layer's estimate from the sag-compensated voltage when it
// is loaded, else the simulator's charge or the plain voltage) and the time to fly the planned path home.
function updateBudget() {
  const t = ctl.tel, real = settings.get("mode") === "real", d = sim?.drone, now = performance.now();
  const b = safety?.battery(), soc = b ? b.soc : !real && d ? d.soc : t?.vbat ? clamp((t.vbat - 3.3) / (4.15 - 3.3), 0, 1) : null;
  if (soc == null) return missionPanel.setBudget("Battery: waiting for the radio's telemetry.", "na");
  const left = (b ? b.secondsLeft : soc * (settings.get("flightSeconds") || FLIGHT_S)) * 1000, p = inHouse() && localizer?.pose(), pad = home.house?.home;
  let back = null;
  if (p && pad && p.status !== "lost") {
    if (safety) back = safety.timeHome(p, now) * 1000;
    else {
      const route = plan(home.map, [p.x, p.y], [pad.x, pad.y]);
      if (route.ok) back = ((route.length / (settings.get("speed") || 0.35)) + 8) * 1000;
    }
  }
  const v = t?.vbat ? `${t.vbat.toFixed(1)} V, ` : "";
  missionPanel.setBudget(`Battery ${v}about ${fmtClock(left)} of flight left${back != null ? ` · ${fmtClock(back)} to fly home` : ""}`,
    back != null && left < 1.3 * back + 10e3 ? "bad" : soc < 0.35 ? "warn" : "ok");
}

const cap = (s) => (s ? s[0].toUpperCase() + s.slice(1) : s);
const roomName = (id) => home.house?.rooms.find((r) => r.id === id || r.name === id)?.name ?? id;

// ---------------------------------------------------------------- commands

async function runCommand(raw, origin = "ui") {
  const text = raw.trim();
  if (!text) return;
  log("pilot", text);
  subtitle("pilot", text);
  recorder.command("say", { text }, origin);
  const t = text.toLowerCase().replace(/[.,!?]/g, "").trim();

  // Safety words act instantly, without waiting for the AI.
  if (/^(stop|stop stop|abort|freeze|hold|hover|wait|cancel|hold on)$/.test(t)) {
    stopMission("pilot said stop");
    droneSays(settings.get("autonomy") === "full" && ctl.isFlying() ? "Stopping. Holding here." : "Stopping.");
    return;
  }
  if (/^(land|land now|come down)$/.test(t)) {
    stopMission("landing");
    return startPlanner(local, "land");
  }
  if (!(await handOver())) return;
  return startPlanner(agent.configured ? agent : local, text);
}

// Before the AI flies (ui/autonomy.js): false when a stop or a newer command came in while it waited.
const handOver = () => handOverTo({ turns, settings, sim, ctl, say: droneSays });

async function startPlanner(p, text) {
  stopMission();
  planner = p;
  await p.run(text);
  if (planner === p) planner = null;
}

function stopMission(reason) {
  turns.stop();
  agent.stop();
  local.stop();
  missions?.stop?.();
  ctl.abort(reason || "stopped");
  planner = null;
}

for (const p of [agent, local]) {
  p.on("say", (text) => droneSays(text));
  p.on("error", (msg) => {
    log("error", msg);
    voice.speak("I hit a problem. Check the log.");
  });
  p.on("status", (s) => {
    $("#brainState").textContent = s === "thinking" ? "Thinking…" : s === "acting" ? "Flying…" : "";
    document.body.classList.toggle("busy", s !== "idle");
  });
  p.on("tool", ({ name, input, phase, result }) => {
    if (phase === "start") {
      log("step", describeTool(name, input), { pending: true });
      recorder.command(name, input, p === agent ? "claude" : "local");
    } else finishStep(result);
  });
}

// After each Claude call: what it was for, tokens used, how much came from the prompt cache, and roughly what it cost;
// the session's spend split into voice commands and picture checks (Settings → Brain), and this flight's checks (HUD).
const kTok = (n) => (n >= 1000 ? `${(n / 1000).toFixed(1)}K` : `${n}`);
const PURPOSE = { survey: "survey", change: "change check", detection: "detection check", recall: "memory question", summary: "flight summary" };
const PICTURES = new Set(["survey", "change", "detection"]);
const costs = { command: 0, checks: 0, memory: 0, session: 0 };
agent.on("usage", (u) => {
  const inTok = u.input + u.cacheWrite + u.cacheRead;
  const money = u.cost === null ? "" : ` · about $${u.cost.toFixed(3)} (this session $${u.session.cost.toFixed(2)})`;
  const what = u.purpose ? ` (${PURPOSE[u.purpose] ?? u.purpose})` : "";
  log("info", `Claude${what}: ${u.requests} call${u.requests === 1 ? "" : "s"}, ${kTok(inTok)} tokens in (${Math.round(u.cachedShare * 100)}% from cache), ${kTok(u.output)} out${money}`);
  if (u.cost != null) costs[!u.purpose ? "command" : PICTURES.has(u.purpose) ? "checks" : "memory"] += u.cost;
  costs.session = u.session.cost;
  showCosts();
});
claude.on("usage", () => showCosts());

function showCosts() {
  const $s = (v) => `$${v.toFixed(2)}`;
  $("#aiCosts").textContent = costs.session ? `This session: ${$s(costs.session)} (voice commands ${$s(costs.command)}, picture checks ${$s(costs.checks)}`
    + `${costs.memory ? `, memory questions and flight summaries ${$s(costs.memory)}` : ""}). `
    + `This flight's Claude budget: ${$s(claude.flight.spent)} of ${$s(claude.budget)} spent.` : "No Claude calls yet this session.";
}

function describeTool(name, input = {}) {
  switch (name) {
    case "take_off": return "Take off";
    case "land": return "Land";
    case "hover": return `Hover ${input.seconds ?? ""}s`;
    case "turn": return `Turn ${input.degrees >= 0 ? "right" : "left"} ${Math.abs(Math.round(input.degrees))}°`;
    case "move": return `Move ${input.direction} ${Number(input.meters).toFixed(1)} m`;
    case "look_around": return "Look around (360°)";
    case "fly_toward": return `Fly toward point (${(+input.x).toFixed(2)}, ${(+input.y).toFixed(2)}) ${Number(input.meters).toFixed(1)} m`;
    case "find": return `Find ${input.object}`;
    case "approach": return `Approach ${input.object}`;
    case "follow": return `Follow ${input.object} for ${input.seconds}s`;
    case "snapshot": return "Look at the camera";
    case "say": return `Say “${input.text}”`;
    case "remember": return `Remember: ${input.note}`;
    case "where_am_i": return "Where am I?";
    case "go_to": return `Go to ${input.place}`;
    case "look_in": return `Look in ${input.room}`;
    case "search_for": return `Search for ${input.target}`;
    case "patrol": return "Patrol";
    case "return_home": return "Return home";
    case "set_home": return "Set the home pad here";
    case "mark_landmark": return `Mark “${input.name}”`;
    case "notify": return `Alert: ${input.text}`;
    case "recall": return `Remember: ${input.question}`;
    case "what_changed": return "What changed?";
    case "survey_house": return "Survey the house";
    default: return name;
  }
}

function droneSays(text) {
  if (!text) return;
  log("drone", text);
  subtitle("drone", text);
  voice.speak(text);
}

ctl.on("pilot-request", ({ kind, text }) => {
  if (settings.get("mode") === "sim") {
    sim.pilotRequest(kind);
    log("info", `Asked the (simulated) pilot: ${text}`);
  } else droneSays(text);
});

// ---------------------------------------------------------------- log & subtitles

let pendingStep = null;

function log(kind, text, { pending = false, own = false } = {}) {
  const box = $("#log");
  const el = document.createElement("div");
  el.className = `msg ${kind}${pending ? " pending" : ""}`;
  if (kind === "step") el.innerHTML = `<span class="icon"></span><span class="t"></span><div class="detail"></div>`;
  else el.innerHTML = `<span class="t"></span>`;
  el.querySelector(".t").textContent = text;
  box.appendChild(el);
  while (box.children.length > 200) box.firstChild.remove();
  box.scrollTop = box.scrollHeight;
  if (pending && !own) pendingStep = el;
  return el;
}

function finishStep(result) {
  const el = pendingStep;
  pendingStep = null;
  if (!el) return;
  el.classList.remove("pending");
  el.classList.add(result?.isError ? "fail" : "ok");
  el.querySelector(".detail").textContent = (result?.text || "").split("\n")[0];
  $("#log").scrollTop = $("#log").scrollHeight;
}

function subtitle(who, text) {
  subtitles.push({ who, text: who === "pilot" ? `“${text}”` : text, t: performance.now() });
  if (subtitles.length > 4) subtitles.shift();
}

// Model downloads and other long jobs are one line each that updates (key: "detector", "depth", "vision", "db", "map3d").
const progressLines = new Map();
function progressLine(key, text, value = null) {
  const el = progressLines.get(key);
  if (key === "map3d" || key === "db") refreshHouseTools(key === "map3d" ? text : null, key === "db" ? text : null);
  if (text == null) return progressLines.delete(key);
  if (el?.isConnected) el.querySelector(".t").textContent = text;
  else progressLines.set(key, log("info", text));
}
perception.on("status", (s) => (s.progress != null ? progressLine("detector", s.text, s.progress) : (progressLines.delete("detector"), log("info", s.text))));
perception.on("error", (m) => log("error", m));

// ---------------------------------------------------------------- HUD

function hudState() {
  const t = ctl.tel;
  const s = settings.all();
  const now = performance.now();
  const chips = [];
  const blocked = ctl.blocker();
  if (!t) chips.push({ text: settings.get("mode") === "sim" ? "SIM" : "NO RADIO", color: "#475569" });
  else if (t.failsafe) chips.push({ text: "FAILSAFE", color: "#dc2626" });
  else if (!t.engaged) chips.push({ text: "AI OFF · pilot flying", color: "#475569" });
  else if (blocked && /pilot moved/.test(blocked)) chips.push({ text: "PILOT OVERRIDE", color: "#ea580c" });
  else if (ctl.mask) chips.push({ text: "AI FLYING", color: "#16a34a" });
  else chips.push({ text: "AI READY", color: "#0e7490" });
  chips.push({ text: { observer: "Observer", copilot: "Co-pilot", full: "Full auto" }[s.autonomy], color: "#1f2937" });
  const right = [], real = settings.get("mode") === "real", sp = session.splat;
  if (recorder.recording) right.push({ text: "● REC", color: "#b91c1c" });
  if (t?.vbat) right.push({ text: `${t.vbat.toFixed(1)} V`, color: t.vbat < 3.4 ? "#dc2626" : t.vbat < 3.6 ? "#d97706" : "#1f2937" });
  if (t && real) right.push({ text: `LQ ${t.lq}%`, color: t.lq < 50 ? "#dc2626" : "#1f2937" });
  const pose = inHouse() && localizer?.pose();
  const st = statusChips({ pose, known: !!localizer?.known, flown: !!localizer?.flown, flying: ctl.isFlying(), vision: !!sp?.enabled, trust: sp?.trust?.(), fix: localizer?.fixQuality?.(),
    why: ctl.safety ? safety?.status().reason : "", real, brake: safety?.avoid?.brake?.(), video: real && source?.ready?.() ? { fps: perception.fps, picture: perception.pictureState?.() } : null,
    ai: { spent: claude.flight.spent, budget: claude.budget }, attached: !!ctl.safety });
  right.push(...st.right);
  let banner = null;
  if (sim && settings.get("mode") === "sim" && sim.drone.crashed) banner = { text: "Crashed! Press R to reset the sim", color: "rgba(220,38,38,0.92)" };
  else if (t?.failsafe) banner = { text: "Radio failsafe — flip the AI switch off and on", color: "rgba(220,38,38,0.92)" };
  else banner = st.banner;
  const b = ctl.behavior;
  const busyText = $("#brainState").textContent;
  const missionText = missionState && [cap(missionState.phase), missionState.text].filter(Boolean).join(": ");
  return {
    detections: perception.latest.detections,
    targets: b ? [b.targetLabel, ...(b.targetLabels || [])].filter(Boolean) : [],
    chips,
    rightChips: right,
    action: b?.label ? `▶ ${b.label}` : missionText ? `◆ ${missionText}` : busyText ? `● ${busyText}` : "",
    notice: st.notice,
    listening: voice.listening,
    subtitles: subtitles
      .map((x) => ({ ...x, alpha: Math.max(0, Math.min(1, (9000 - (now - x.t)) / 1500)) }))
      .filter((x) => x.alpha > 0),
    banner,
    hfov: s.hfov,
    uptilt: s.uptilt,
    pitchDeg: ctl.est.pitchAngle / DEG,
    rollDeg: ctl.est.rollAngle / DEG,
    noVideoText: settings.get("mode") === "real" ? (source === goggles && goggles.status !== "off" ? goggles.describe() : "No video yet") : "",
    noVideoHint: settings.get("mode") === "real" && !(source === goggles && goggles.status !== "off")
      ? "Goggles: double-click start.command, plug the goggles into the Mac over USB-C and power the drone" : "",
  };
}

// ---------------------------------------------------------------- panels

let lastPanels = 0;
let lastBudget = 0;

function refreshPanels(force) {
  const now = performance.now();
  if (!force && now - lastPanels < 200) return;
  lastPanels = now;
  const t = ctl.tel;
  const s = settings.all();
  const real = s.mode === "real";

  pill("#pillRadio", real ? (serial.bridgeAlive ? "ok" : serial.connected ? "warn" : "off") : "ok",
    real ? (serial.bridgeAlive ? "Radio" : serial.connected ? "Radio: no script" : "Radio") : "Sim radio");
  const videoName = () => ({ screen: "Video (window)", goggles: goggles.via === "usb" ? "Goggles USB" : "Goggles" })[source.kind] || "Video";
  pill("#pillVideo", source?.ready() ? "ok" : source === goggles && goggles.active ? "warn" : "off",
    real ? (source?.ready() ? videoName() : source === goggles && goggles.active ? "Goggles…" : "No video") : "Sim camera");
  syncGogglesWatch();
  const gs = $("#gogglesStatus");
  gs.textContent = goggles.describe();
  gs.dataset.state = goggles.status;
  pill("#pillBrain", agent.configured ? "ok" : "warn", agent.configured ? "Claude" : "Offline brain");
  pill("#pillAI", !t ? "off" : t.failsafe ? "bad" : t.engaged ? (ctl.mask ? "live" : "ok") : "off",
    !t ? "AI switch ?" : t.failsafe ? "Failsafe" : t.engaged ? (ctl.mask ? "AI flying" : "AI armed") : "AI switch off");

  $("#vBatt").textContent = t?.vbat ? `${t.vbat.toFixed(2)} V` : "—";
  $("#vLink").textContent = t ? `${t.lq}% / ${t.rssi} dBm` : "—";
  $("#vFm").textContent = t ? t.fm || "—" : "—";
  $("#vHeading").textContent = `${Math.round((((ctl.est.heading / DEG) % 360) + 360) % 360)}°`;
  $("#vVel").textContent = `${ctl.est.vx.toFixed(2)} / ${ctl.est.vy.toFixed(2)} / ${ctl.est.vz.toFixed(2)}`;
  $("#vFlow").textContent = `${Math.round(ctl.est.flowQ * 100)}% · ${perception.fps} fps`;
  $("#vHover").textContent = `${Math.round(ctl.hover.estimate(t?.vbat) * 100)}%`;
  $("#vStatus").textContent = ctl.status;

  if (sim && !real) {
    const tr = sim.truth(), house = tr.world === "house";
    $("#simArm").classList.toggle("on", sim.pilot.armSwitch);
    $("#simAi").classList.toggle("on", sim.pilot.aiSwitch);
    $("#simAssist").classList.toggle("on", sim.assist.on);
    $("#simActors").classList.toggle("on", s.simActors !== false);
    $("#simActors").hidden = !house;
    $("#simRehearse").hidden = !house;
    $$("#simWorldSeg button").forEach((b) => b.classList.toggle("on", b.dataset.world === (house ? "house" : "demo")));
    $("#simInfo").textContent = `${tr.room || "—"} · height ${tr.drone.agl.toFixed(2)} m · battery ${Math.round(tr.drone.soc * 100)}%${house ? "" : ` · cat: ${tr.cat.state}`}`;
  }
  if (real) {
    check("#ckSerial", SerialRadio.supported ? serial.connected : null, SerialRadio.supported ? "" : "Use Chrome or Edge (Web Serial)");
    check("#ckScript", serial.bridgeAlive);
    check("#ckSwitch", !!t?.engaged);
    check("#ckAngle", t ? /STAB|ANGL|HOR/.test(t.fm) : false);
    check("#ckVideo", camera.ready() || screen.ready() || goggles.ready());
    check("#ckBrain", agent.configured);
    $("#btnRadio").textContent = serial.connected ? "Disconnect radio" : "Connect radio (USB)";
  }
  // the pad button wherever the position isn't the simulator's truth (the real drone, a rehearsal)
  missionPanel.pad.hidden = !(inHouse() && localizer && localizer.source !== "truth");
  missionPanel.pad.disabled = ctl.isFlying() || !!missions?.busy;
  $$("#autonomySeg button").forEach((b) => b.classList.toggle("on", b.dataset.autonomy === s.autonomy));
  $("#simRehearse").classList.toggle("on", !!s.simRehearse);
  updateChangesButton();
  if (force || now - lastBudget > 1000) {
    lastBudget = now;
    updateBudget();
    updateReady();
    syncWakeLock();
  }
}

// ---------------------------------------------------------------- ready to fly, changes, the screen

// "Ready to fly?" (real mode, and a rehearsal of it): missions.preflightCheck's items (with the vision side's and, on the
// real drone, the radio's blocks merged in by the session) and the page's own (ui/ready.js), once a second on the ground;
// a blocking one keeps the mission buttons off (Return home stays) with its reason.
let readyCache = [];
const checklistOn = () => settings.get("mode") === "real" || session.rehearsing();
function readyCheck() {
  if (!checklistOn()) return (readyCache = []);
  const real = settings.get("mode") === "real", switchAsks = real && ctl.telemetryFresh && !!ctl.tel && !ctl.tel.engaged; // handOver() asks for it
  const items = readyItems({ missions, mode: real ? "real" : "sim", extra: { depth: session.depth, claude, session, vox: home.vox }, page: real ? shellChecks(readySnapshot()) : [], switchAsks });
  return (readyCache = real ? items : items.filter((it) => it.id !== "radio")); // a rehearsal: the simulated pilot hands over by itself
}

function radioState() {
  const t = ctl.tel;
  return { supported: SerialRadio.supported, connected: serial.connected, script: serial.bridgeAlive, angle: t ? /STAB|ANGL|HOR/.test(t.fm) : false, engaged: !!t?.engaged };
}

function readySnapshot() {
  const t = ctl.tel, sp = session.splat, d = perception.detector;
  return {
    house: home.house, home: !!home.house?.home, vox: home.vox ? { usable: true } : { usable: false, why: session.voxWhy }, building: !!session.building,
    coverage: home.vox && session.coverage({ maxAge: 60000 }), webgpu: !!navigator.gpu, detector: { ready: !!d?.ready, backend: d?.backend ?? null, failed: !!perception.detectorFailed },
    depth: { ready: !!session.depthReady, on: settings.get("avoid") !== false },
    splat: sp && { enabled: sp.enabled, calibOk: sp.calibOk, db: !!sp.db, trust: sp.trust?.(), padcheck: session.readiness().find((it) => it.id === "padcheck")?.text },
    calib: session.calib, memory: { open: !!session.memory, stored: !!session.memory?.stored },
    ai: { key: !!settings.get("apiKey"), vision: settings.get("aiVision"), budget: settings.get("aiBudget") }, brake: safety?.avoid?.brake?.() ?? { measured: !!settings.get("brake") },
    radio: radioState(), video: { ready: !!source?.ready?.() && source.kind !== "sim", picture: perception.pictureState?.(), fps: perception.fps }, vbat: t?.vbat, flying: ctl.isFlying(),
  };
}

// In the air the list is folded and checked again only while something blocks missions, every 5 s (ready.js keepReady).
let readyAt = 0;
function updateReady() {
  const on = checklistOn(), flying = ctl.isFlying(), now = performance.now(), keep = keepReady({ flying, items: readyCache, at: readyAt, now });
  if (on && !keep) readyAt = now;
  const items = !on ? [] : keep ? readyCache : readyCheck();
  if (on) {
    readyList.render(items, { video: !!source?.ready?.() && source.kind !== "sim", flying });
    if (!flying && readyList.root.open) session.refreshReadiness();
  }
  missionPanel.setBlocked(on ? blocking(items)?.text ?? "" : "");
}

function readyFix(it) {
  const a = it.fix?.action;
  if (typeof a === "function") return Promise.resolve(a()).catch((e) => log("error", e.message));
  switch (FIXES[a]) {
    case "radio": return serial.connected ? log("info", it.text) : $("#btnRadio").click();
    case "goggles": return startGoggles();
    case "house": return openSettings("tabHouse");
    case "calib": return openSettings("tabHouse", "#calib");
    case "home": return mapView.setTool("home");
    case "map3d": return home.vox ? openSettings("tabHouse", "#coverage") : session.load3D();
    case "db": return buildDb();
    case "prepare": return prepare();
    case "pad": return missionPanel.emit("pad");
    case "crop": return startCrop();
    case "vision": return settings.set("locVision", true), settings.set("avoid", true);
    case "flight": return openSettings("tabFlight");
    case "brain": return openSettings("tabBrain");
    case "charge": return log("info", "Charge or swap the battery, then put it back in.");
    default: log("info", it.text);
  }
}

// Everything before the battery goes in (nav/readiness.js prepare(): the vision models, the 3D map, the position database),
// as one log line that updates.
function prepare() {
  const sp = session.splat;
  if (!sp?.prepare) return session.sync();
  if (ctl.isFlying()) return log("info", "That is done on the ground: after landing.");
  return sp.prepare({ onProgress: (p) => progressLine("prepare", p.step ? `Getting ready: ${p.text}` : null, p.progress) })
    .then((r) => log(r.ok ? "info" : "error", !r.ok ? `Not ready: ${Object.values(r.failed).join("; ")}.`
      : `${settings.get("mode") === "real" ? "Ready for the battery" : "Ready: the models, the 3D map and the position database are in place"} (${r.seconds} s).`))
    .catch((e) => e?.name !== "AbortError" && log("error", `Getting ready stopped: ${e.message}`));
}

// After landing: the memory's report of the flight (fixed rules), and on request Claude's few sentences about it (text
// only: no pictures go).
function flightReport(f, memory = session.memory) {
  if (!f?.report) return;
  const el = log("info", `After the flight: ${f.report}`);
  if (!claude.configured || settings.get("aiVision") === "off" || !memory) return;
  const b = Object.assign(document.createElement("button"), { type: "button", className: "btn ghost small", textContent: "Short summary from Claude" });
  b.addEventListener("click", async () => {
    Object.assign(b, { disabled: true, textContent: "Asking Claude…" });
    const r = await summarizeFlight({ claude, memory, flight: f }).catch((e) => ({ ok: false, reason: e.message }));
    b.remove();
    if (r.ok) log("drone", r.text);
    else log("error", `No summary from Claude: ${plainError(r.reason ?? "", "it didn't answer")}.`);
  });
  el.append(b);
}

// The open changes (suspected, not resolved) as a button over the flight view: two taps to the change card. "kept clear
// of": the ones the drone flies around now (the others the camera looked at for a while and didn't see).
function updateChangesButton() {
  const c = inHouse() ? session.openChanges() : null, n = c?.open ?? 0, b = $("#hudChanges"), kept = c?.blocking ?? n;
  b.hidden = !n;
  if (n) b.textContent = `${n} possible change${n === 1 ? "" : "s"}${kept < n ? ` (${kept ? `${kept} kept clear of` : "none in the way now"})` : ""} · check`;
}

function showChange(id) {
  const m = session.memory;
  id ??= m?.changes?.findLast((c) => c.status === "suspected")?.id;
  if (!m || !id) return;
  const card = mount("changeCard");
  if (!card) return;
  $("#changeCard").hidden = false;
  card.show(id);
}

// Screen Wake Lock while the drone is armed or flying (a sleeping screen hides the window: missions would stop).
let wakeLock = null, wakeAsking = false;
async function syncWakeLock() {
  const want = ctl.isFlying() || (settings.get("mode") === "real" ? ctl.armed() : !!sim?.pilot.armSwitch);
  if (!want && wakeLock) return void (wakeLock.release().catch(() => {}), (wakeLock = null));
  if (!want || wakeLock || wakeAsking || document.hidden || !navigator.wakeLock) return;
  wakeAsking = true;
  try {
    wakeLock = await navigator.wakeLock.request("screen");
    wakeLock.addEventListener("release", () => (wakeLock = null));
  } catch {}
  wakeAsking = false;
}

function pill(sel, state, text) {
  const el = $(sel);
  el.dataset.state = state;
  el.querySelector("span").textContent = text;
}

function check(sel, ok, note = "") {
  const el = $(sel);
  el.dataset.ok = ok === null ? "na" : ok ? "yes" : "no";
  const n = el.querySelector("small");
  if (n) n.textContent = note;
}

// Stick gauges: pilot sticks (grey) vs. what goes to the quad (yellow).
function drawSticks() {
  const c = $("#sticks");
  const dpr = window.devicePixelRatio || 1;
  const w = Math.round(c.clientWidth * dpr);
  const h = Math.round(c.clientHeight * dpr);
  if (c.width !== w || c.height !== h) Object.assign(c, { width: w, height: h });
  const g = c.getContext("2d");
  g.clearRect(0, 0, w, h);
  const t = ctl.tel;
  const size = Math.max(0, Math.min(w / 2 - 16 * dpr, h - 16 * dpr));
  const pilot = t?.sticks || { roll: 0, pitch: 0, yaw: 0, thr: 0 };
  const m = ctl.mask;
  const out = {
    roll: m & 1 ? ctl.out.roll : pilot.roll,
    pitch: m & 2 ? ctl.out.pitch : pilot.pitch,
    thr: m & 4 ? ctl.out.thr : pilot.thr,
    yaw: m & 8 ? ctl.out.yaw : pilot.yaw,
  };
  const gauges = [
    [w / 4, (p) => [p.yaw, p.thr * 2 - 1], "THR / YAW"],
    [(3 * w) / 4, (p) => [p.roll, p.pitch], "PITCH / ROLL"],
  ];
  for (const [cx, pick, name] of gauges) {
    const cy = h / 2 - 4 * dpr;
    g.strokeStyle = "rgba(148,163,184,0.35)";
    g.lineWidth = dpr;
    g.strokeRect(cx - size / 2, cy - size / 2, size, size);
    g.beginPath();
    g.moveTo(cx - size / 2, cy);
    g.lineTo(cx + size / 2, cy);
    g.moveTo(cx, cy - size / 2);
    g.lineTo(cx, cy + size / 2);
    g.stroke();
    for (const [p, color, r] of [[pilot, "#94a3b8", 5], [out, "#ffd84d", 4]]) {
      const [x, y] = pick(p);
      g.fillStyle = color;
      g.beginPath();
      g.arc(cx + (x * size) / 2, cy - (y * size) / 2, r * dpr, 0, 7);
      g.fill();
    }
    g.fillStyle = "#64748b";
    g.font = `${9 * dpr}px ui-sans-serif, system-ui, sans-serif`;
    g.textAlign = "center";
    g.fillText(name, cx, h - 2 * dpr);
  }
}

// ---------------------------------------------------------------- main loop

let lastT = performance.now();
let lastRender = 0;
let lastFrameRect = { fx: 0, fy: 0, fw: 1, fh: 1 };
// A frame that throws must not stop the loop (the simulator, the views and the recorder live in it): log each distinct
// error once and carry on.
const frameErrors = new Set();
function frame(now) {
  requestAnimationFrame(frame);
  try {
    step(now);
  } catch (e) {
    const key = `${e?.message}`.slice(0, 200);
    if (frameErrors.has(key)) return;
    frameErrors.add(key);
    console.error(e);
    log("error", `Something went wrong drawing the screen (${key}). The app keeps running.`);
  }
}
// One frame of the page: the simulator, perception, the views (a check in a hidden page calls it from a timer).
function step(now) {
  const dt = Math.max(0, (now - lastT) / 1000);
  lastT = Math.max(lastT, now);
  if (settings.get("mode") === "sim" && sim) {
    sim.step(dt);
    if (now - lastRender > 30) {
      sim.renderCamera();
      lastRender = now;
    }
  }
  perception.process();
  lastFrameRect = drawFpv($("#fpv"), source, { ...hudState(), cropDrag: cropMode?.dragging ? cropMode : null });
  const map = document.body.dataset.map;
  if (map === "demo" && sim) drawMinimap($("#minimap"), sim);
  else if (map === "house") mapView.draw();
  drawSticks();
  refreshPanels();
  recorder.tick(now);
}

// ---------------------------------------------------------------- audio beeps (sim radio)

let audio = null;
function beep(freq, ms, delay = 0) {
  try {
    audio ||= new AudioContext();
    const o = audio.createOscillator();
    const gain = audio.createGain();
    o.frequency.value = freq;
    gain.gain.value = 0.04;
    o.connect(gain).connect(audio.destination);
    const t0 = audio.currentTime + delay / 1000;
    o.start(t0);
    o.stop(t0 + ms / 1000);
  } catch {}
}

// ---------------------------------------------------------------- settings dialog

// A control bound to a setting both ways (a preset or the app may change the setting too).
const bound = new Map();
function bindSetting(sel, key, parse = (v) => v) {
  const el = $(sel);
  const isCheck = el.type === "checkbox";
  const show = (val) => (isCheck ? (el.checked = !!val) : (el.value = val ?? ""));
  show(settings.get(key));
  bound.set(key, (val) => document.activeElement !== el && show(val));
  el.addEventListener(isCheck || el.tagName === "SELECT" ? "change" : "input", () => {
    settings.set(key, parse(isCheck ? el.checked : el.value));
  });
}

async function fillCameraList() {
  const sel = $("#setCamera");
  try {
    const cams = await CameraSource.list();
    sel.innerHTML = `<option value="">— choose a video source —</option>` +
      cams.map((c, i) => `<option value="${c.deviceId}">${escapeHtml(c.label || `Camera ${i + 1}`)}</option>`).join("");
    sel.value = settings.get("cameraId") || "";
  } catch {
    sel.innerHTML = `<option value="">Camera access unavailable</option>`;
  }
}

function fillVoices() {
  const sel = $("#setVoice");
  const voices = (voice.voices || []).filter((v) => /^en/i.test(v.lang));
  sel.innerHTML = `<option value="">Default voice</option>` + voices.map((v) => `<option>${escapeHtml(v.name)}</option>`).join("");
  sel.value = settings.get("voiceName") || "";
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]);
}

function openSettings(tab, anchor = null) {
  fillCameraList();
  fillVoices();
  if (!$("#settings").open) $("#settings").showModal();
  showTab(tab || $("#settings .tabs button.on")?.dataset.tab || "tabBrain");
  if (anchor) requestAnimationFrame(() => $(anchor)?.scrollIntoView({ block: "start" }));
}

function showTab(id) {
  $$("#settings .tabs button").forEach((x) => {
    x.classList.toggle("on", x.dataset.tab === id);
    x.setAttribute("aria-selected", String(x.dataset.tab === id));
  });
  $$("#settings .tab").forEach((x) => (x.hidden = x.id !== id));
  $("#settings .tabs button.on")?.scrollIntoView?.({ inline: "nearest", block: "nearest" });
  if (id === "tabHouse") {
    housePanel.refresh();
    refreshHouseTools();
  }
  if (id === "tabRecordings") {
    recorder.refreshList();
    mount("recheck");
  }
  if (id === "tabFlight") showBrake();
  if (id === "tabMemory") showMemory();
  syncGogglesWatch();
}

// Settings → Memory: what this house's memory holds (memory.ram()), in words.
function showMemory(note = "") {
  const m = session.memory, r = m?.ram?.(), mb = (b) => (b >= 1e6 ? `${(b / 1e6).toFixed(1)} MB` : `${Math.round(b / 1e3)} KB`), n = (k, w) => `${k} ${w}${k === 1 ? "" : "s"}`;
  $("#memoryState").textContent = note || (!m ? "No flight memory is open (it opens with your house)."
    : `${n(r?.flights ?? m.flights?.length ?? 0, "flight")} remembered for this house${m.world === "sim" ? " in the simulator" : ""}`
      + `${r ? `, ${n(r.sightings, "sighting")} and ${n(r.changes, "change")}; ${n(r.pictures, "picture")} in use (${mb(r.bytes)}), people and pets included` : ""}.`);
  $("#btnForgetPictures").disabled = $("#btnForget").disabled = !m;
}

// Settings → House: the 3D map's state and the position database's (progress lines pass their text).
let voxStats = null;
function refreshHouseTools(map3dText = null, dbText = null) {
  $("#houseTools").hidden = !home.house;
  if (!$("#settings").open || !home.house || $("#tabHouse").hidden) return;
  ["coverage", "survey", "calib"].forEach(mount); // also right after the first import, while the tab shows
  const vox = home.vox, sp = session.splat;
  if (sp?.twin) dbHint = null;
  if (vox && (voxStats?.vox !== vox || voxStats.version !== vox.version)) {
    const score = session.coverage({ maxAge: 60000 })?.score;
    const known = Number.isFinite(score) ? `: ${Math.round(score)}% of the space the drone can fly in is known` : "";
    voxStats = { vox, version: vox.version, text: `Built${known}. The drone never flies where the scan didn't see.` };
  }
  $("#map3dState").textContent = map3dText ?? (vox ? voxStats.text : session.building ? "Building the 3D map…" : `Not built${session.voxWhy ? `: ${session.voxWhy}` : ""}.`);
  $("#btnRebuild3d").textContent = vox ? "Build the 3D map again" : "Build the 3D map";
  $("#dbState").textContent = dbText ?? dbHint ?? (sp?.db ? "Built for this house and camera." : sp?.twin ? "Not built yet (about 30 s)."
    : "It is built from the 3D scan once vision position is on: with the real drone, or in the simulator with Position from its own camera.");
}

// The position database needs the twin vision uses: the real drone's (it loads by itself in real mode) or a rehearsal's.
let dbHint = null;
function buildDb() {
  const sp = session.splat, real = settings.get("mode") === "real", built = !!sp?.db && (!sp.dbKey || sp.db.meta?.key === sp.dbKey());
  if (ctl.isFlying()) return log("info", "The position database is built on the ground: after landing.");
  dbHint = !sp ? "This version of the app can't build it." : built ? "Built for this house and camera." : sp.twin ? null
    : real ? "Waiting for the 3D scan to load for the real drone: it builds by itself then (about 30 s)."
    : "Switch to Real drone (or turn on “Rehearse the real flight” in the simulator) and it builds by itself, about 30 s.";
  if (dbHint) {
    log("info", `Position database: ${dbHint}`);
    if (real) session.sync();
    return openSettings("tabHouse", "#dbState");
  }
  session.ensureDb();
}

function showBrake() {
  const b = settings.get("brake");
  $("#brakeState").textContent = b?.decel ? `Measured: slows at ${b.decel} m/s², ${b.react} s to react.` : "Not measured: the real drone flies at most 0.3 m/s.";
}

function setupSettings() {
  bindSetting("#setKey", "apiKey", (v) => v.trim());
  bindSetting("#setModel", "model", (v) => v.trim() || "claude-opus-5");
  bindSetting("#setEffort", "effort");
  bindSetting("#setSpeed", "speed", Number);
  bindSetting("#setTilt", "maxTilt", Number);
  bindSetting("#setYawRate", "maxYawRate", Number);
  bindSetting("#setHfov", "hfov", Number);
  bindSetting("#setUptilt", "uptilt", Number);
  bindSetting("#setDelay", "videoDelay", (v) => Math.max(0, Math.min(1000, Number(v) || 0)));
  bindSetting("#setDetector", "detector");
  bindSetting("#setDetectorQuality", "detectorQuality");
  bindSetting("#setSimDetections", "simDetections");
  bindSetting("#setAiVision", "aiVision");
  bindSetting("#setAiBudget", "aiBudget", (v) => Math.max(0, Number(v) || 0));
  bindSetting("#setAiModel", "aiModel", (v) => v.trim());
  bindSetting("#setAvoid", "avoid");
  bindSetting("#setLocVision", "locVision");
  bindSetting("#setSimLoc", "simLoc");
  bindSetting("#setSimAugment", "simAugment");
  bindSetting("#setMemoryDays", "memoryDays", (v) => Math.max(1, Math.round(Number(v) || 90)));
  bindSetting("#setMemoryFlights", "memoryFlights", (v) => Math.max(10, Math.round(Number(v) || 200)));
  // the memory's limits apply once a value is entered (not at every keystroke: a shorter limit deletes at once)
  const retain = () => session.memory?.setRetention?.({ keepDays: settings.get("memoryDays"), keepFlights: settings.get("memoryFlights") })?.then?.(() => showMemory());
  $("#setMemoryDays").addEventListener("change", retain);
  $("#setMemoryFlights").addEventListener("change", retain);
  const rehearse = $("#setRehearse");
  rehearse.checked = !!settings.get("simRehearse");
  rehearse.addEventListener("change", () => setRehearse(rehearse.checked));
  $("#btnRebuild3d").addEventListener("click", () => (ctl.isFlying() ? log("info", "I'll build the 3D map after landing.") : session.load3D({ rebuild: !!home.vox })));
  $("#btnBuildDb").addEventListener("click", buildDb);
  $("#btnPrepare").addEventListener("click", prepare);
  $("#btnPrepareReady").addEventListener("click", prepare);
  $("#btnBrake").addEventListener("click", () => {
    $("#settings").close();
    runMission({ kind: "calibrate" });
  });
  let forgetArmed = 0;
  $("#btnForget").addEventListener("click", async () => {
    const b = $("#btnForget"), m = session.memory;
    if (!m) return;
    if (Date.now() - forgetArmed > 4000) return void ((forgetArmed = Date.now()), (b.textContent = "Delete for good? Click again"));
    forgetArmed = 0;
    b.textContent = "Delete this house's flight memory";
    await m.clear();
    showMemory("Deleted: flights, sightings with their pictures, and changes.");
    panels.history?.setMemory?.(m);
  });
  let picturesArmed = 0;
  $("#btnForgetPictures").addEventListener("click", async () => {
    const b = $("#btnForgetPictures"), m = session.memory;
    if (!m?.clearPictures) return;
    if (Date.now() - picturesArmed > 4000) return void ((picturesArmed = Date.now()), (b.textContent = "Delete every picture? Click again"));
    picturesArmed = 0;
    b.textContent = "Delete the pictures only";
    await m.clearPictures();
    showMemory("Deleted every picture (people, pets, changes). Where and when it saw them stays.");
    panels.history?.setMemory?.(m);
  });
  $("#btnScreen").addEventListener("click", startScreenCapture);
  $("#btnGoggles").addEventListener("click", () => startGoggles());
  $("#btnGogglesUsb").addEventListener("click", () => startGoggles({ direct: true }));
  $("#btnGoggles2").addEventListener("click", () => startGoggles());
  $("#btnCrop").addEventListener("click", startCrop);
  $("#btnCropReset").addEventListener("click", () => {
    settings.set("crop", null);
    camera.crop = screen.crop = goggles.crop = null;
    perception.setSource(source);
  });
  bindSetting("#setYawCenter", "yawCenter", Number);
  bindSetting("#setYawMax", "yawMax", Number);
  bindSetting("#setAngle", "angleLimit", Number);
  bindSetting("#setSpeak", "speak");
  bindSetting("#setHandsFree", "handsFree");
  bindSetting("#setWake", "wakeWord", (v) => v.trim().toLowerCase() || "drone");
  bindSetting("#setVoice", "voiceName");
  bindSetting("#setNotes", "homeNotes");
  $("#setCamera").addEventListener("change", (e) => e.target.value && startCamera(e.target.value));
  $("#btnCamRefresh").addEventListener("click", async () => {
    // Asking for any camera once unlocks device names.
    try {
      const s = await navigator.mediaDevices.getUserMedia({ video: true });
      s.getTracks().forEach((t) => t.stop());
    } catch {}
    fillCameraList();
  });
  $("#btnKeyShow").addEventListener("click", () => {
    const k = $("#setKey");
    k.type = k.type === "password" ? "text" : "password";
  });
  $("#btnResetHover").addEventListener("click", () => {
    for (const m of ["sim", "real"]) localStorage.removeItem(`whoopPilot.hover.${m}`);
    ctl.attach(settings.get("mode") === "sim" ? sim.radio : serial, settings.get("mode"));
    log("info", "Hover calibration reset.");
  });
  const syncGeometry = () => (perception.geometry = { ...perception.geometry, hfov: settings.get("hfov"), uptilt: settings.get("uptilt") });
  syncGeometry();
  settings.on("change", ({ key, value }) => {
    bound.get(key)?.(value);
    if (key === "hfov" || key === "uptilt") syncGeometry();
    if (key === "uptilt") session.applyCalib();
    if (["simLoc", "locVision", "avoid", "locSource", "simRehearse"].includes(key)) session.sync();
    if (key === "simAugment") setAugment(value);
    if (key === "simRehearse") $("#setRehearse").checked = !!value;
    if (key === "brake") showBrake();
    if (key === "aiVision" || key === "aiBudget" || key === "apiKey") showCosts();
    if ((key === "hfov" || key === "uptilt") && sim) sim.setCamera({ hfov: settings.get("hfov"), uptilt: settings.get("uptilt") });
    if (key === "hfov") mapView.fov = value;
    if (key === "videoDelay" && sim) sim.videoDelay = value;
    if (key === "videoDelay" && document.activeElement !== $("#setDelay")) $("#setDelay").value = value;
    if (key === "handsFree") voice.setHandsFree(value);
    if (key === "apiKey") refreshPanels(true);
    if (key === "homeNotes" && document.activeElement !== $("#setNotes")) $("#setNotes").value = value;
    if (key === "detector" || key === "detectorQuality") perception.detector.configure({ backend: settings.get("detector"), quality: settings.get("detectorQuality") });
    if (key === "simDetections" && sim) {
      sim.detections = value;
      if (settings.get("mode") === "sim") perception.setSource(source);
    }
  });
  $("#btnSettings").addEventListener("click", () => openSettings());
  $("#btnCloseSettings").addEventListener("click", () => $("#settings").close());
  $("#settings").addEventListener("close", syncGogglesWatch);
  $$("#settings .tabs button").forEach((b) => b.addEventListener("click", () => showTab(b.dataset.tab)));
  bindAlertSettings($("#tabAlerts"), { settings, alerts: () => alerts, log });
}

// ---------------------------------------------------------------- map, missions, events, recorder

const mapView = new MapView($("#mapview"), { fov: settings.get("hfov"), truth: () => simTruth(), actors: () => simActors() });
const housePanel = new HousePanel($("#tabHouse"), {
  app: { store: houseStore, active: () => home.house?.id ?? "", use: useHouse, edit: editHouse, session: () => serverSession() },
});
const missionPanel = new MissionPanel($("#missions"));
const events = new EventsList($("#events"), { dialog: $("#snapDialog"), roomName, onCount: (n) => ($("#eventCount").textContent = n ? String(n) : "") });
const recorder = new FlightRecorder({
  button: $("#btnRecord"),
  list: $("#recList"),
  sample: () => telemetrySample(),
  context: () => ({
    label: `${settings.get("mode") === "sim" ? "sim" : "flight"}${home.house && inHouse() ? `-${home.house.name}` : ""}`,
    house: inHouse() ? home.house.id : null,
    // the camera's 4:3 picture once found ("dark": region() is still the whole 16:9 stream)
    lens: { uptilt: settings.get("uptilt"), fov: settings.get("hfov"), calib: session.calib, region: ["found", "whole"].includes(perception.pictureState?.()) ? perception.region() : null },
    app: { mode: settings.get("mode"), autonomy: settings.get("autonomy"), model: settings.get("model"), simWorld: sim?.world.kind ?? null, detector: settings.get("detector") },
  }),
});
let swapped = false;
const readyList = new ReadyList($("#ready"));
readyList.on("fix", readyFix);
const consent = new ConsentBanner($("#aiConsent"), { claude, settings });

// ---------------------------------------------------------------- the house panels (ui/view3d, history, survey, calib,
// coverage, changecard, recheck): made when first shown, made again when what they show is replaced (house, memory).

const panels = {};
const recordings = async () => (await fetch("/rec/list").then((r) => (r.ok ? r.json() : null)).catch(() => null))?.recordings ?? [];
const view3dTwin = () => session.depthTwin() ?? home.twin;
const surveyTwin = async () => home.twin ?? (home.house?.splatFile
  ? Twin.create({ splat: await (await houseStore()).readFile(home.house.id, home.house.splatFile), house: home.house }) : null);
const releaseTwin = (t) => t && t !== home.twin && t !== session.locTwin && t.dispose?.();
// The twin is busy with what flies the drone (vision fixes, live depth): the 3D view waits (the twin's own queue puts the
// simulator's camera first).
const twinBusy = () => !!(session.splat?.busy || session.depth?.inFlight);
const flying = () => ctl.isFlying();
const PANELS = {
  view3d: () => V3D && new V3D.View3D($("#view3d"), { house: home.house, twin: view3dTwin(), vox: home.vox, map: home.map, localizer, missions, memory: session.memory,
    truth: () => simTruth(), mode: () => settings.get("mode"), flying, busy: twinBusy }),
  history: () => HIS && new HIS.HistoryPanel($("#history"), { memory: session.memory, map: home.map, inspect: inspector, settings, flying }),
  coverage: () => COV && new COV.CoveragePanel($("#coverage"), { house: home.house, map: home.map, vox: home.vox, flying,
    coverageReport: (o = {}) => (!o.vox || o.vox === home.vox ? session.coverage() : coverageReport(o)) }),
  survey: () => SUR && new SUR.SurveyPanel($("#survey"), { house: home.house, map: home.map, vox: home.vox, twin: surveyTwin, settings, claude, survey: surveyHouse,
    estimate: estimateSurvey, onApply: applyFromSurvey, releaseTwin, flying }),
  // restore: the saved lens back on vision and live depth after a Discard, a Stop or a failed live calibration
  calib: () => CAL && new CAL.CalibPanel($("#calib"), { twin: view3dTwin(), house: home.house, map: home.map, settings, lens: LENS, recordings, flying,
    calibrateLive: session.splat && ((o) => session.splat.calibrate(o)), calibrateRecording: RPL?.calibrateRecording && calibrateFromRecording,
    save: (c) => session.saveCalib(c), restore: () => session.applyCalib(), current: () => session.calib, mode: () => settings.get("mode") }),
  recheck: () => RCK && new RCK.RecheckPanel($("#recheck"), { recordings, house: home.house, settings, flying,
    replay: RPL?.checkRecording && ((id, o = {}) => RPL.checkRecording({ id, houseId: home.house?.id, ...o })),
    calibrate: (id) => (openSettings("tabHouse", "#calib"), (panels.calib ?? mount("calib"))?.pickRecording?.(id)) }),
  changeCard: () => session.memory && new (CHC?.ChangeCard ?? SimpleChangeCard)($("#changeCardBody"), { memory: session.memory, inspect: inspector, settings }),
};
const MISSING = { view3d: "the 3D view", history: "the flight history", coverage: "the coverage report", survey: "Claude's survey", calib: "camera calibration", recheck: "the localization check" };

// A recorded pad turn -> calib.json (nav/replay.js in a worker, with its own twin): stored and used from now on.
async function calibrateFromRecording(id, o = {}) {
  const r = await RPL.calibrateRecording({ id, houseId: home.house?.id, ...o });
  if (!r?.calib) throw new Error(r?.text ?? "that recording couldn't calibrate the camera");
  return r;
}

function mount(name) {
  if (panels[name]) return panels[name];
  let p = null;
  try {
    p = PANELS[name]() || null;
  } catch (e) {
    console.error(e);
    log("error", `${MISSING[name] ?? name} didn't open: ${e.message}`);
  }
  const host = $(`#${name === "changeCard" ? "changeCardBody" : name}`);
  if (!p && MISSING[name] && host && !host.childElementCount && host.tagName !== "CANVAS")
    host.append(Object.assign(document.createElement("p"), { className: "note", textContent: `This version of the app doesn't have ${MISSING[name]} yet.` }));
  if (p) wirePanel(name, p);
  return (panels[name] = p);
}
function unmount(name) {
  panels[name]?.dispose?.();
  delete panels[name];
}
function remount(...names) {
  for (const n of names) if (n in panels) (unmount(n), mount(n));
}

function wirePanel(name, p) {
  const select = ({ kind, id, x, y } = {}) => {
    if (kind === "change" && name !== "history") return showChange(id); // History opens its own card: the map shows where
    const sg = kind === "sighting" ? session.memory?.sightings?.find((q) => q.id === id) : null;
    if (Number.isFinite(x ?? sg?.x)) {
      showMap();
      mapView.focusOn(x ?? sg.x, y ?? sg.y);
    }
  };
  p.on?.("select", select);
  p.on?.("error", (e) => console.warn(name, e));
  if (name === "history") p.on?.("trail", ({ flightId } = {}) => {
    mapView.showTrail(flightId);
    panels.view3d?.setFlight?.(flightId);
    setLayer("trail", true);
  });
  if (name === "coverage") p.on?.("show", ({ x, y } = {}) => {
    $("#settings").close();
    showMap();
    setLayer("gaps", true);
    mapView.focusOn(x, y);
  });
  if (name === "survey") p.on?.("review", () => refreshPanels(true));
  if (name === "view3d" && mapView.trailFlight) p.setFlight?.(mapView.trailFlight);
  if (name === "changeCard") {
    p.on?.("resolved", () => (refreshPanels(true), setTimeout(() => ($("#changeCard").hidden = true), 1500)));
    p.on?.("close", () => ($("#changeCard").hidden = true));
    p.on?.("show", ({ x, y } = {}) => (showMap(), mapView.focusOn(x, y)));
  }
}

// A new house (or none): every panel shows the new one.
function panelsForHouse() {
  const { house, map, vox } = home, P = panels;
  $("#changeCard").hidden = true;
  P.view3d?.setHouse?.(house, view3dTwin(), vox, map);
  P.history?.setMap?.(map);
  P.coverage?.setHouse?.(house, map, vox);
  P.survey?.setHouse?.(house, map, vox, surveyTwin);
  P.calib?.setHouse?.(house, view3dTwin(), map);
  P.recheck?.setHouse?.(house);
  refreshHouseTools();
}

// The twin the 3D view and calibration render with changed (the real drone's loaded or went, the simulator's world): they
// take the new one (never keep one that is disposed).
function syncPanelTwins() {
  const t = view3dTwin(), { view3d: v, calib: c } = panels;
  if (v && v.twin !== t) v.setHouse?.(home.house, t, home.vox, home.map);
  if (c && c.twin !== t && c.state !== "running") c.setHouse?.(home.house, t, home.map);
}

// The map (2D) in sight: the 3D view off, the map in the big view if it is small and the page narrow.
function showMap() {
  if (show3d) (show3d = false), syncMapCanvas();
  $("#mapview").scrollIntoView?.({ block: "nearest" });
}

let show3d = false;
function syncMapCanvas() {
  const on = document.body.dataset.map === "house", three = on && show3d && !!panels.view3d;
  $("#mapview").hidden = !on || three;
  $("#view3dBox").hidden = !three; // with the 3D view's own controls and caption
  $("#btnView3d").setAttribute("aria-pressed", String(three));
  panels.view3d?.invalidate?.();
}

// Claude's survey from a panel or the survey_house tool: approve the estimate (a modal: only on the ground), review, apply.
function confirmSurvey(est, { signal } = {}) {
  const d = $("#surveyDialog");
  $("#surveyDialogText").textContent = `${est.text}`;
  if (!d.open) d.showModal();
  return new Promise((resolve) => {
    const ac = new AbortController(), o = { signal: ac.signal }, done = (v) => (ac.abort(), d.open && d.close(), resolve(v));
    $("#surveyYes").addEventListener("click", () => done(true), o);
    $("#surveyNo").addEventListener("click", () => done(false), o);
    d.addEventListener("cancel", () => done(false), o);
    signal?.addEventListener("abort", () => done(false), o);
  });
}

function reviewSurvey(res) {
  openSettings("tabHouse", "#survey");
  const p = mount("survey");
  if (p?.review) p.review(res);
  else if (p?.showResult) p.showResult(res);
  else log("info", surveySummary(res, home.house));
}

// What the user accepted from Claude's survey: { res, accept } (applySurvey's), or the panel's { rooms, landmarks, keepouts }.
async function applyFromSurvey(sel = {}) {
  const id = home.house?.id;
  if (!id) return;
  const near = (a, b) => a.name?.toLowerCase() === b.name?.toLowerCase() && Math.hypot(a.x - b.x, a.y - b.y) < 1;
  if (sel.apply) await editHouse(id, sel.apply);
  else if (sel.res && sel.accept) await editHouse(id, (h) => applySurvey(h, sel.res, sel.accept, { map: home.map }));
  else
    await editHouse(id, (h) => {
      for (const r of sel.rooms ?? []) {
        const room = h.rooms.find((q) => q.id === r.id), name = r.name ?? r.suggestedName;
        if (room && name && room.nameSource !== "user") Object.assign(room, { name, nameSource: "claude", ...(r.kind && { kind: r.kind }) });
      }
      const lms = (sel.landmarks ?? []).map((l) => ({ ...l, source: "claude" }));
      h.landmarks = [...(h.landmarks ?? []).filter((m) => !(m.source === "claude" && lms.some((l) => near(l, m)))), ...lms];
      h.keepouts = [...(h.keepouts ?? []), ...(sel.keepouts ?? []).map((k) => ({ ...k, source: "claude" }))];
    });
  await session.cleanVox();
  home.vox?.applyObjects?.(home.house);
  await session.saveVox();
  mapView.invalidate();
  housePanel.refresh();
  log("info", "Claude's suggestions you accepted are on the map: room names, places and no-fly zones (Settings → House lists them).");
}

tools.setAI({ claude, twin: surveyTwin, releaseTwin, confirmSurvey, reviewSurvey, onProgress: (p) => p?.text && progressLine("survey", p.text, p.done != null && p.total ? p.done / p.total : null) });

// The simulator's O4 look on or off between two frames of its splat camera (a frame in flight keeps the look it began with).
function setAugment(on) {
  if (!sim) return;
  if (sim.twinBusy) return void setTimeout(() => setAugment(settings.get("simAugment")), 20);
  sim.setAugment(on || null);
}

// "Rehearse the real flight": the simulator's position from its own camera (with the O4's blur, noise and a lens error),
// live depth on, the person detector on its pictures as on the real drone, and the "Ready to fly?" list's vision rules.
function setRehearse(on) {
  settings.set("simRehearse", !!on);
  settings.set("simLoc", on ? "vision" : "truth");
  settings.set("simAugment", !!on);
  settings.set("simDetections", on ? "detector" : "truth");
  if (on) settings.set("avoid", true);
  if (!on) return log("info", "Rehearsal off: the simulator knows its exact position again.");
  if (sim?.world.kind === "house" && !sim.twin) setSimWorld(); // its 3D-scan camera, again
  const caps = safety?.avoid?.real ? "the real drone's speed limits apply" : "the real drone's speed limits aren't in this simulator yet";
  log("info", `Rehearsing the real flight: the simulator's position now comes from its own camera (with the O4's blur, noise and a lens error), `
    + `live depth and the person detector run on its pictures, missions start only when the real drone would, and ${caps}.`);
}

// The simulator's trouble in plain words; a rehearsal can't go on without its 3D-scan camera.
function simError(m) {
  log("error", plainError(m, "The simulator's 3D-scan camera stopped, so it shows the simple camera."));
  if (session.rehearsing() && !sim?.twin)
    log("error", "The rehearsal needs that 3D-scan camera: missions won't start until it runs again (turn the rehearsal off and on, or pick My house again).");
}

const simTruth = () => {
  if (settings.get("mode") !== "sim" || sim?.world.kind !== "house") return null;
  const d = sim.drone;
  return { x: d.x, y: d.y, z: d.z, yaw: d.yaw, trail: sim.trail };
};
const simActors = () => (settings.get("mode") === "sim" && sim?.world.kind === "house" ? sim.truth().actors : null);

function telemetrySample() {
  const t = ctl.tel, e = ctl.est, p = inHouse() ? localizer?.pose() : null, d = settings.get("mode") === "sim" ? sim?.drone : null;
  return {
    mode: settings.get("mode"),
    engaged: t?.engaged ?? null, failsafe: t?.failsafe ?? null, fm: t?.fm ?? null, vbat: t?.vbat ?? null, lq: t?.lq ?? null,
    sticks: t?.sticks ?? null, mask: ctl.mask, out: ctl.mask ? { roll: r3(ctl.out.roll), pitch: r3(ctl.out.pitch), thr: r3(ctl.out.thr), yaw: r3(ctl.out.yaw) } : null,
    att: t?.att ?? null,
    est: { heading: r3(e.heading), vx: r3(e.vx), vy: r3(e.vy), vz: r3(e.vz), flowQ: r3(e.flowQ) },
    pose: p ? { x: r3(p.x), y: r3(p.y), z: r3(p.z), yaw: r3(p.yaw), sigma: r3(p.sigma), status: p.status, source: p.source } : null,
    truth: d ? { x: r3(d.x), y: r3(d.y), z: r3(d.z), yaw: r3(d.yaw), soc: r3(d.soc), crashed: d.crashed } : null,
    behavior: ctl.behavior?.label ?? null,
    mission: missionState?.phase ?? null,
  };
}

// The map and the camera view trade places (the big stage area and the small map card).
function syncViews() {
  const big = $("#mainView"), slot = $("#mapSlot"), maps = [$("#minimap"), $("#mapview"), $("#view3dBox")];
  const on = swapped && document.body.dataset.map !== "none";
  if (on) {
    big.append(...maps);
    slot.append($("#fpv"));
  } else {
    big.append($("#fpv"));
    slot.append(...maps);
  }
  $("#btnSwapView").setAttribute("aria-pressed", String(on));
}

let markAt = null;
function setupMap() {
  $("#btnSwapView").addEventListener("click", () => {
    swapped = !swapped;
    syncViews();
  });
  $$("#mapTools [data-tool]").forEach((b) => b.addEventListener("click", () => {
    mapView.setTool(b.dataset.tool);
    $("#mapview").focus();
  }));
  $$("#mapTools [data-zoom]").forEach((b) => b.addEventListener("click", () => {
    const z = b.dataset.zoom;
    if (z === "fit") mapView.fit();
    else mapView.zoomBy(z === "in" ? 1.3 : 1 / 1.3);
  }));
  mapView.on("tool", (tool) => $$("#mapTools [data-tool]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.tool === tool))));
  mapView.on("home", async ({ x, y, yaw }) => {
    if (!home.map.roomAt(x, y)) return log("error", "Put the home pad inside a room.");
    await editHouse(home.house.id, (h) => (h.home = { x: r3(x), y: r3(y), yaw: r3(yaw), source: "user" }));
    mapView.setTool("pan");
    log("info", `Home pad set in ${home.map.roomAt(x, y).name}. Missions start and end there.`);
  });
  mapView.on("landmark", (p) => {
    markAt = p;
    $("#markForm").hidden = false;
    $("#markName").value = "";
    $("#markName").focus();
  });
  $("#markForm").addEventListener("submit", async (e) => {
    e.preventDefault();
    const name = $("#markName").value.trim(), p = markAt;
    $("#markForm").hidden = true;
    markAt = null;
    if (!name || !p) return;
    await editHouse(home.house.id, (h) => h.landmarks.push({ name, room: p.room, x: r3(p.x), y: r3(p.y), z: r3(home.map.floorAt(p.x, p.y) ?? 0), source: "user" }));
    log("info", `Landmark “${name}” added. Try “go to the ${name}”.`);
    $("#mapview").focus();
  });
  $("#markCancel").addEventListener("click", () => {
    $("#markForm").hidden = true;
    markAt = null;
    $("#mapview").focus();
  });
  mapView.on("keepout", async ({ polygon }) => {
    await editHouse(home.house.id, (h) => h.keepouts.push({ kind: "user", polygon, source: "user" }));
    log("info", "No-fly zone added. The drone plans around it at every height.");
  });
  mapView.on("goto", (p) => {
    mapView.setTool("pan");
    if (!p.room) return log("error", "That spot is outside the rooms the map knows.");
    runMission({ kind: "goTo", target: { x: r3(p.x), y: r3(p.y) } });
  });
  mapView.on("select", (f) => {
    sideTab("events");
    const url = snapshotUrl(f.snapshot);
    if (url) events.show(url, normalizeEvent(f), f.room && roomName(f.room));
  });
  mapView.on("change", (id) => showChange(id));
  mapView.on("gap", (gap) => log("info", `${gap.text}${gap.suggestion?.text ? ` ${gap.suggestion.text}` : ""}`));
  $$("#mapLegend [data-layer]").forEach((c) => {
    c.checked = !!mapView.layers[c.dataset.layer];
    c.addEventListener("change", () => mapView.setLayer(c.dataset.layer, c.checked));
  });
  $("#mapKey").addEventListener("toggle", fitMapKey);
  window.addEventListener("resize", fitMapKey);
  $("#btnView3d").addEventListener("click", () => {
    show3d = !show3d;
    if (show3d && !mount("view3d")) {
      show3d = false;
      log("info", "This version of the app doesn't have the 3D view yet.");
    }
    syncMapCanvas();
  });
  $("#hudChanges").addEventListener("click", () => showChange());
}

// The open map key stays inside the map card (which clips) and scrolls there.
function fitMapKey() {
  const k = $("#mapKey"), ul = k.querySelector("ul");
  if (!k.open) return;
  const bottom = k.closest(".map-card").getBoundingClientRect().bottom, top = ul.getBoundingClientRect().top;
  ul.style.maxHeight = `${Math.max(64, Math.floor(bottom - top - 6))}px`;
}

function sideTab(which) {
  const tabs = { log: ["#log", "#tabLogBtn"], events: ["#events", "#tabEventsBtn"], history: ["#history", "#tabHistoryBtn"] };
  for (const [k, [panel, btn]] of Object.entries(tabs)) {
    $(panel).hidden = k !== which;
    $(btn).setAttribute("aria-selected", String(k === which));
  }
  if (which === "events") events.refresh();
  if (which === "history" && !mount("history")) $("#history").replaceChildren(Object.assign(document.createElement("p"), { className: "note center",
    textContent: session.memory ? "This version of the app doesn't have the flight history yet."
      : "The flight history opens with your house (Settings → House), in the simulator's My house or with the real drone." }));
}

function setupMissions() {
  alerts?.on("error", (m) => log("error", m));
  missionPanel.on("run", (m) => runMission(m));
  missionPanel.on("stop", () => runCommand("stop"));
  missionPanel.on("pad", () => {
    const checking = session.canPadCheck(), why = session.pad(), h = home.house?.home, real = settings.get("mode") === "real";
    if (why) return log("error", why);
    recorder.command("pad", { x: h.x, y: h.y, yaw: h.yaw ?? Math.PI / 2 }, "ui");
    log("info", checking ? "Position set to the home pad. Checking the camera against the 3D scan from here…"
      : real && session.splat?.enabled ? "Position set to the home pad. The camera check needs the video: press this again once it shows (battery in, goggles on)."
      : "Position set to the home pad. Take off from there.");
  });
  $("#tabLogBtn").addEventListener("click", () => sideTab("log"));
  $("#tabEventsBtn").addEventListener("click", () => sideTab("events"));
  $("#tabHistoryBtn").addEventListener("click", () => sideTab("history"));
  events.setAlerts(alerts);
  recorder.on("started", () => log("info", "Recording this flight."));
  recorder.on("error", (m) => log("error", m));
  recorder.on("stopped", ({ stopReason, durationMs, byUs }) => {
    const len = durationMs ? ` (${fmtClock(durationMs)})` : "";
    panels.calib?.refreshRecordings?.();
    panels.recheck?.refresh?.();
    if (byUs) return log("info", `Recording saved${len}. Settings → Recordings lists it.`);
    const text = `Recording stopped${len}: ${stopReason || "the server stopped it"}.`;
    log("error", text);
    if (alerts) alerts.notify({ text, urgency: "default", kind: "recorder" });
    else if (window.Notification?.permission === "granted") new Notification("Whoop Pilot", { body: text });
  });
  recorder.init();
  // the camera's picture in the video, as the recording's replay needs it: at the start and whenever it moves
  let pictureKey = "";
  perception.on("frame", ({ region: r } = {}) => {
    const k = recorder.recording && settings.get("mode") === "real" && r ? `${r.sx},${r.sy},${r.sw},${r.sh}` : "";
    if (k && k !== pictureKey) recorder.command("picture", { sx: r.sx, sy: r.sy, sw: r.sw, sh: r.sh }, "app");
    pictureKey = k || (recorder.recording ? pictureKey : "");
  });
}

// ---------------------------------------------------------------- controls

function setupControls() {
  $$("#modeSeg button").forEach((b) => b.addEventListener("click", () => setMode(b.dataset.mode)));
  $$("#autonomySeg button").forEach((b) =>
    b.addEventListener("click", () => {
      if (b.dataset.autonomy === settings.get("autonomy")) return;
      stopMission("changed autonomy");
      settings.set("autonomy", b.dataset.autonomy);
      if (ctl.airborne && b.dataset.autonomy !== "full") ctl.airborne = false;
      log("info", `Autonomy: ${b.textContent.trim()}. ${b.title}`);
      refreshPanels(true);
    }),
  );
  const form = $("#cmdForm");
  form.addEventListener("submit", (e) => {
    e.preventDefault();
    const text = $("#cmd").value;
    $("#cmd").value = "";
    runCommand(text);
  });
  $$(".chip[data-cmd]").forEach((c) => c.addEventListener("click", () => runCommand(c.dataset.cmd)));
  $("#btnStop").addEventListener("click", () => runCommand("stop"));
  $("#btnLand").addEventListener("click", () => runCommand("land"));

  const mic = $("#btnMic");
  if (!voice.supported) {
    mic.disabled = true;
    mic.title = "Speech recognition isn't available in this browser. Use Chrome.";
  }
  const down = (e) => {
    e.preventDefault();
    voice.startPushToTalk();
  };
  const up = () => voice.endPushToTalk();
  mic.addEventListener("pointerdown", down);
  mic.addEventListener("pointerup", up);
  mic.addEventListener("pointerleave", up);

  voice.on("command", (text) => {
    $("#cmd").placeholder = "Say or type a command…";
    runCommand(text, "voice");
  });
  voice.on("interim", (text) => ($("#cmd").placeholder = text ? `🎙 ${text}` : "Listening…"));
  voice.on("state", (on) => {
    mic.classList.toggle("on", on && voice.mode === "ptt");
    if (on && voice.mode === "ptt") $("#cmd").placeholder = "Listening…";
    if (!on) {
      $("#cmd").placeholder = "Say or type a command…";
      if (settings.get("handsFree")) setTimeout(() => !voice.listening && voice.setHandsFree(true), 300);
    }
  });
  voice.on("error", (msg) => log("error", msg));

  // Radio push-to-talk button (reported by the radio script).
  let lastPtt = false;
  ctl.on("telemetry", (t) => {
    if (t.ptt && !lastPtt) voice.startPushToTalk();
    if (!t.ptt && lastPtt) voice.endPushToTalk();
    lastPtt = t.ptt;
  });

  const typing = () => ["INPUT", "TEXTAREA", "SELECT"].includes(document.activeElement?.tagName) || $("#settings").open || $("#snapDialog").open;
  window.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
      if ($("#settings").open || $("#snapDialog").open) return;
      if (!$("#markForm").hidden && $("#markForm").contains(document.activeElement)) return $("#markCancel").click();
      if (cropMode) {
        cropMode = null;
        if (source && source.kind !== "sim") source.crop = settings.get("crop");
        return;
      }
      runCommand("stop");
      return;
    }
    if (typing()) return;
    if (e.code === "Space") {
      e.preventDefault();
      if (!e.repeat) voice.startPushToTalk();
      return;
    }
    if (e.code === "KeyL" && !e.repeat) return runCommand("land");
    if (e.code === "KeyR" && settings.get("mode") === "sim" && !e.metaKey && !e.ctrlKey) return void (!e.repeat && simReset());
    if (settings.get("mode") === "sim" && sim && !e.metaKey && !e.ctrlKey) {
      if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(e.code)) e.preventDefault();
      if (!e.repeat) sim.keyDown(e.code);
    }
  });
  window.addEventListener("keyup", (e) => {
    if (e.code === "Space") voice.endPushToTalk();
    sim?.keyUp(e.code);
  });

  $("#simArm").addEventListener("click", () => sim && (sim.pilot.armSwitch = !sim.pilot.armSwitch));
  $("#simAi").addEventListener("click", () => sim && (sim.pilot.aiSwitch = !sim.pilot.aiSwitch));
  $("#simAssist").addEventListener("click", () => sim && (sim.assist.on = !sim.assist.on));
  $("#simReset").addEventListener("click", simReset);
  $$("#simWorldSeg button").forEach((b) => b.addEventListener("click", () => {
    if (b.dataset.world === "house" && !home.house) {
      log("info", "Add your house first: Settings → House.");
      return openSettings("tabHouse");
    }
    settings.set("simMode", b.dataset.world);
    setSimWorld();
  }));
  $("#simActors").addEventListener("click", () => {
    settings.set("simActors", settings.get("simActors") === false);
    setSimWorld();
  });
  $("#simRehearse").addEventListener("click", () => setRehearse(!settings.get("simRehearse")));

  $("#btnRadio").addEventListener("click", async () => {
    if (!SerialRadio.supported) return log("error", "This browser can't talk to USB serial devices. Open the app in Chrome or Edge.");
    try {
      if (serial.connected) await serial.disconnect();
      else if (await serial.connect()) log("info", "Radio port open. Waiting for the aibrg script to talk…");
    } catch (e) {
      if (e.name !== "NotFoundError") log("error", `Couldn't open the radio: ${e.message}`);
    }
    refreshPanels(true);
  });
  serial.on("error", (msg) => log("error", msg));
  serial.on("status", () => refreshPanels(true));

  document.addEventListener("visibilitychange", () => {
    if (document.hidden && (planner || ctl.behavior)) {
      stopMission("the app window was hidden");
      log("error", "Mission stopped: keep this window visible while the AI flies (browsers slow down hidden tabs).");
      alerts?.notify({ text: "Mission stopped: the Whoop Pilot window was hidden. Keep it visible while the AI flies.", urgency: "high", kind: "hidden" });
    }
    if (!document.hidden) syncWakeLock();
  });
}

// The simulator back on its home pad; where its position comes from its camera, the estimate too (as the pad button does).
function simReset() {
  stopMission("sim reset");
  ctl.airborne = false;
  sim?.reset();
  if (inHouse() && localizer && localizer.source !== "truth" && home.house?.home) setTimeout(() => {
    const why = session.pad();
    log(why ? "error" : "info", why || "Reset: the drone and its position estimate are back on the home pad.");
  }, 150);
}

// A map layer on or off, its legend box with it.
function setLayer(name, on) {
  mapView.setLayer(name, on);
  $$(`#mapLegend [data-layer=${name}]`).forEach((c) => (c.checked = !!on));
}

// ---------------------------------------------------------------- start

function setupSession() {
  session.on("log", ({ level, text }) => log(level === "error" ? "error" : "info", text));
  session.on("progress", ({ key, text, value }) => progressLine(key, text, value));
  session.on("built", onBuilt);
  session.on("sync", syncUi);
  session.on("vox", (vox) => {
    mapView.set({ vox });
    refreshHouseTools();
    panels.coverage?.setHouse?.(home.house, home.map, vox);
    if (panels.survey && panels.survey.state !== "running" && panels.survey.state !== "review") panels.survey.setHouse?.(home.house, home.map, vox, surveyTwin);
    panels.view3d?.setHouse?.(home.house, view3dTwin(), vox, home.map);
  });
  session.on("memory", (m) => {
    mapView.set({ memory: m });
    $("#changeCard").hidden = true;
    panels.history?.setMemory?.(m);
    panels.view3d?.setMemory?.(m);
    if (panels.changeCard?.setMemory && m) panels.changeCard.setMemory(m);
    else remount("changeCard");
  });
  session.on("changes", () => updateChangesButton());
  session.on("twin", syncPanelTwins);
  session.on("takeoff", () => (mapView.newFlight(), panels.view3d?.newFlight?.()));
  session.on("house", () => (mapView.invalidate(), panels.view3d?.invalidate?.()));
  session.on("map", ({ house, map }) => (mapView.setHouse(house, map, home.ortho), panelsForHouse()));
  session.on("calib", () => refreshHouseTools());
  session.on("ready", () => refreshPanels(true)); // the vision checklist moved (a download, the battery clock)
  session.on("pad", () => mapView.invalidate()); // the pad check may have re-seated the pose on the camera's
  session.on("flight", (f) => flightReport(f));
}

setupSession();
setupSettings();
setupControls();
setupCropDrag();
setupMap();
setupMissions();
ctl.start();
setMode(settings.get("mode") || "sim");
if (settings.get("handsFree")) voice.setHandsFree(true);
log("info", agent.configured
  ? "Ready. Hold Space (or the mic button) and say a command, like “find the cat”."
  : "Ready. Offline brain: simple commands work (“take off”, “find the cat”, “go to the kitchen”). Add a Claude API key in Settings for real understanding.");
if (settings.get("houseId")) useHouse(settings.get("houseId"));
requestAnimationFrame(frame);
if ("serviceWorker" in navigator) navigator.serviceWorker.register("sw.js").catch(() => {});
// for debugging in the console
window.whoop = { settings, ctl, perception, agent, local, tools, goggles, useGoggles, home, session, claude, inspector, panels, mapView, recorder, useHouse, runMission, runCommand,
  step, get localizer() { return localizer; }, get missions() { return missions; }, get safety() { return safety; }, get sim() { return sim; } };
