// The app's panels (app/js/ui/) without a browser, on the house fixture (fixtures/house): the map's frame (orthophoto
// placement, fit, zoom about a point, the pixel <-> plan round trip), room labels inside their own rooms, importing a
// capture through the server's /house-files/ route with progress (the same house as from the folder, no request for a
// file it doesn't have), the import's plain-language checks, the mission panel's places and mission objects (the Wave B
// contract shapes), the map dropping a finished mission's path, event snapshots and times, and the flight recorder's
// batches to /rec/*. Then main.js's rules around the autonomy modules (ui/autonomy.js), flown in the simulator with the
// real localizer, safety layer and mission runner: the safety layer only acts while attached, the real drone's safety
// layer waits for a known position, the home-pad button refuses in flight, a stop cancels a command waiting for the
// hand-over, and Go to a place from the panel flies beside it.
// Usage: cd tools && node test-ui.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { rig } from "./test-sim.mjs"; // (puts performance.now on the simulator's clock)

const I = await import("../app/js/house/import.js");
const MV = await import("../app/js/ui/mapview.js");
const HP = await import("../app/js/ui/housepanel.js");
const MP = await import("../app/js/ui/missionpanel.js");
const EV = await import("../app/js/ui/events.js");
const D = await import("../app/js/ui/dom.js");
const AU = await import("../app/js/ui/autonomy.js");
const B = await import("../app/js/behaviors.js");
const { Localizer } = await import("../app/js/nav/localizer.js");
const { Safety } = await import("../app/js/safety.js");
const { MissionRunner } = await import("../app/js/missions.js");
const { Emitter } = await import("../app/js/util.js");
const SESSION = await import("../app/js/ui/session.js");
const RD = await import("../app/js/ui/ready.js");
const HUD = await import("../app/js/hud.js");
const SET = await import("../app/js/settings.js");
const CHGM = await import("../app/js/nav/changes.js");

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const { house, map } = await I.importCapture(fsSource(FIXTURE));
const near = (a, b, tol, what) => assert.ok(Math.abs(a - b) <= tol, `${what}: ${a} vs ${b} (±${tol})`);

// whoop.mjs's /house-files/<project>/<path>: files, JSON listings for folders, 404 otherwise (each one a console
// error in the browser, so `missing` should stay empty).
function houseFiles(dir, id) {
  const calls = [], missing = [];
  const fetch = async (url) => {
    calls.push(url);
    const m = /^\/house-files\/([^/]+)\/(.*)$/.exec(url);
    assert.ok(m && decodeURIComponent(m[1]) === id, `asked for another project: ${url}`);
    const rel = m[2].split("/").map(decodeURIComponent).join("/"), p = path.join(dir, rel);
    if (!fs.existsSync(p)) return missing.push(url), new Response(JSON.stringify({ error: "Not found." }), { status: 404 });
    if (fs.statSync(p).isDirectory())
      return Response.json({ entries: fs.readdirSync(p).map((name) => ({ name, kind: fs.statSync(path.join(p, name)).isDirectory() ? "dir" : "file" })) });
    const bytes = fs.readFileSync(p), CHUNK = 64 << 10; // streamed in network-sized chunks
    const body = new ReadableStream({
      start(c) {
        for (let i = 0; i < bytes.length; i += CHUNK) c.enqueue(new Uint8Array(bytes.subarray(i, i + CHUNK)));
        c.close();
      },
    });
    return new Response(body, { headers: { "Content-Length": String(bytes.length) } });
  };
  return { fetch, calls, missing };
}

// Steps the simulator until `promise` settles (missions await between controller ticks).
async function drive(r, promise, maxSeconds) {
  let out = null;
  promise.then((v) => (out = { v }), (e) => (out = { e }));
  for (let t = 0; t < maxSeconds && !out; t += 1 / 60) {
    r.wait(1 / 60);
    await new Promise((res) => setImmediate(res));
  }
  if (out?.e) throw out.e;
  return out ? out.v : { ok: false, summary: "test timeout" };
}

// main.js's autonomy stack on a rig: the localizer's source as main sets it, the safety layer behind
// onlyWhenAttached, and ctl.safety synced on every pose (registered before the safety layer's own listener, as main
// does). world: { inHouse, sim } as main's inHouse() and mode say.
function autonomy(r, { source, world, home = house }) {
  const { ctl, sim } = r, settings = ctl.settings, notes = [], trips = [], asks = [];
  const localizer = new Localizer({ ctl, map, house: home, settings });
  localizer.setSource(source);
  if (source === "truth") localizer.setTruth(() => ({ x: sim.drone.x, y: sim.drone.y, z: sim.drone.z, yaw: sim.drone.yaw }));
  let safety = null;
  localizer.on("pose", () => (ctl.safety = safety && AU.safetyWanted({ ...world, localizer }) ? safety : null));
  safety = AU.onlyWhenAttached(new Safety({ map, localizer, settings, house: home, alerts: { notify: (e) => notes.push(e) } }));
  safety.on("trip", (t) => trips.push(t.kind));
  ctl.on("pilot-request", (q) => asks.push(q.kind));
  return { localizer, safety, notes, trips, asks };
}

const tests = {
  "map frame: orthophoto extent, fit, zoom about a point, pixels <-> plan"() {
    const o = { x0: -2.14, y0: -0.41, res: 0.0148, width: 503, height: 452 }, r = MV.orthoRect(o);
    near(r.x1, -2.14 + 503 * 0.0148, 1e-9, "x1");
    near(r.y1, -0.41 + 452 * 0.0148, 1e-9, "y1 (row 0)");
    const b = MV.houseBounds(house);
    for (const rm of house.rooms) for (const [x, y] of rm.outline) assert.ok(x > b.x0 && x < b.x1 && y > b.y0 && y < b.y1, "rooms inside the bounds");
    const t = MV.viewTransform({ W: 800, H: 600, bounds: b, pad: 10 });
    assert.ok(t.X(b.x0) >= 10 - 1e-6 && t.X(b.x1) <= 790 + 1e-6 && t.Y(b.y1) >= 10 - 1e-6 && t.Y(b.y0) <= 590 + 1e-6, "the house fits");
    assert.ok(t.Y(b.y1) < t.Y(b.y0), "y goes up the page");
    for (const [x, y] of [[0, 0], [3.2, 4.1], [-1.5, 6]]) {
      const [hx, hy] = t.toH(t.X(x), t.Y(y));
      near(hx, x, 1e-9, "x round trip");
      near(hy, y, 1e-9, "y round trip");
    }
    // Zooming about a pixel keeps the plan point under it in place (MapView.zoomBy).
    const at = [600, 200], h = t.toH(...at), z = 2.5, f = 1 / z;
    const center = [h[0] + (t.cx - h[0]) * f, h[1] + (t.cy - h[1]) * f];
    const t2 = MV.viewTransform({ W: 800, H: 600, bounds: b, zoom: z, center, pad: 10 });
    near(t2.X(h[0]), at[0], 1e-6, "zoom keeps x");
    near(t2.Y(h[1]), at[1], 1e-6, "zoom keeps y");
    near(t2.s, t.s * z, 1e-9, "scale");
  },

  "room labels sit inside their own room, also where outlines overlap"() {
    const labels = MV.roomLabels(map);
    assert.equal(labels.length, house.rooms.length);
    for (const l of labels) assert.equal(map.roomAt(l.x, l.y)?.id, l.id, `${l.id} label at (${l.x.toFixed(2)}, ${l.y.toFixed(2)})`);
  },

  async "import through /house-files/ with progress: the same house as from the folder"() {
    const { fetch, calls, missing } = houseFiles(FIXTURE, "My capture 1");
    const progress = [];
    const src = HP.houseSource({ id: "My capture 1" }, (e) => progress.push(e), fetch);
    const r = await I.importCapture(src);
    assert.equal(r.house.id, house.id);
    assert.deepEqual(r.map.stats(), map.stats());
    assert.ok(calls.every((u) => u.startsWith("/house-files/My%20capture%201/")), "project name encoded");
    assert.ok(!calls.some((u) => u.endsWith("splat.ply")), "never fetches the PLY when centres are there");
    const big = progress.filter((p) => p.path.endsWith("splat-centres.bin"));
    assert.ok(big.length >= 2 && big.at(-1).loaded === big.at(-1).total, "progress up to the whole file");
    assert.ok(big.every((p, i) => !i || p.loaded > big[i - 1].loaded), "progress only grows");
    assert.equal(await src.read("outputs/nothing.json"), null);
    assert.equal(await src.read("work/nowhere/x.json"), null);
    assert.equal(await src.list("outputs/none"), null);
    assert.deepEqual((await src.list("outputs/plans")).sort(), fs.readdirSync(path.join(FIXTURE, "outputs/plans")).sort());
    assert.deepEqual(missing, [], "asked for files the capture doesn't have (a 404 and a console error each)");
    const listings = calls.filter((u) => u.endsWith("/"));
    assert.equal(new Set(listings).size, listings.length, "each folder listed once");
  },

  "the import's checks in plain words"() {
    const lines = HP.checkSummary(house);
    assert.ok(lines.some((l) => /tape-measure checks \(they shrink the scan by 1\.2%\)/.test(l)), lines.join(" | "));
    assert.ok(lines.some((l) => /walls and the floor plan agree/.test(l)));
    assert.ok(lines.some((l) => /6 AprilTags in the scan; tag 0 is the home pad/.test(l)));
    assert.ok(lines.some((l) => /of \d+ doorways are wide and tall enough/.test(l)));
    assert.ok(HP.checkSummary({ ...house, frame: { f: 1 }, checks: {}, tags: [], doors: [] }).some((l) => /No tape-measure correction/.test(l)));
  },

  "mission panel: places and missions in the contract's shapes"() {
    const ps = MP.places(house);
    assert.ok(ps.length > 0 && ps.every((p) => Number.isFinite(p.x) && Number.isFinite(p.y)));
    assert.ok(!ps.some((p) => /fan|stair|step|curtain|light/i.test(p.name)), "no fixtures or keep-outs to fly to");
    const keys = ps.map((p) => p.name.toLowerCase());
    assert.equal(new Set(keys).size, keys.length, "each name once (the runner picks the nearest one called that)");
    const named = (n) => house.landmarks.filter((l) => l.name.toLowerCase() === n && Number.isFinite(l.x)).length;
    for (const p of ps) assert.equal(p.count, named(p.name.toLowerCase()), `${p.name}: how many it picks from`);
    const mine = { ...house, landmarks: [{ name: "Chair", room: "r2", x: 1, y: 2, z: 0, source: "user" }, { name: "cat bed", room: "r1", x: 1, y: 2, z: 0, source: "user" }, ...house.landmarks] };
    assert.deepEqual(MP.places(mine).slice(0, 2).map((p) => [p.name, p.count]), [["Chair", 1], ["cat bed", 1]], "yours first, and yours win over the capture's");
    // Places go by name, so the runner finds free space beside the thing and faces it (not raw x, y inside it).
    const bed = { place: MP.places(mine).find((p) => p.name === "cat bed") };
    assert.deepEqual(MP.missionFor("goTo", bed), { kind: "goTo", target: "cat bed" });
    assert.deepEqual(MP.missionFor("lookIn", bed), { kind: "lookIn", room: "r1" });
    assert.deepEqual(MP.missionFor("goTo", { room: house.rooms[1] }), { kind: "goTo", target: house.rooms[1].id });
    assert.deepEqual(MP.missionFor("people"), MP.MISSIONS.people);
    assert.notEqual(MP.missionFor("people"), MP.MISSIONS.people, "a copy");
    assert.equal(MP.missionFor("goTo", undefined), null);
    assert.equal(MP.describeMission(MP.missionFor("goTo", bed), mine), "Go to cat bed");
    const kinds = new Set(["goTo", "lookIn", "patrol", "searchFor", "checkOnPets", "returnHome"]);
    for (const m of Object.values(MP.MISSIONS)) assert.ok(kinds.has(m.kind), m.kind);
    assert.deepEqual(MP.MISSIONS.people, { kind: "searchFor", target: "person" });
    const r2 = house.rooms.find((r) => r.id === "r2");
    assert.equal(MP.describeMission({ kind: "goTo", target: "r2" }, house), `Go to ${r2.name}`);
    assert.equal(MP.describeMission({ kind: "goTo", target: { x: 1, y: 2, name: "cat bed" } }, house), "Go to cat bed");
    assert.equal(MP.describeMission({ kind: "goTo", target: { x: 1, y: 2 } }, house), "Go to 1.0, 2.0 m");
    assert.equal(MP.describeMission({ kind: "lookIn", room: "r1" }, house), "Look in Living room");
    assert.equal(MP.describeMission({ kind: "patrol" }, house), "Patrol the house");
  },

  "map: a mission's path shows while it runs and goes when it ends"() {
    const had = globalThis.document;
    globalThis.document = { createElement: () => ({}) };
    try {
      const canvas = { setAttribute() {}, addEventListener() {} }, missions = new Emitter();
      const view = new MV.MapView(canvas, { missions });
      missions.emit("path", { path: [[0, 0, 1], [1, 1, 1]] });
      assert.equal(view.path.length, 2);
      missions.emit("finding", { label: "person", x: 1, y: 1 });
      missions.emit("done", { ok: false, summary: "Stopped." });
      assert.equal(view.path, null, "no dashed plan left on the map after a stop");
      assert.equal(view.findings.length, 1, "findings stay");
    } finally {
      globalThis.document = had;
    }
  },

  "events: base64 snapshots become pictures, mission-relative times become now"() {
    const b64 = "/9j/" + "A".repeat(400);
    assert.equal(EV.snapshotUrl(b64), `data:image/jpeg;base64,${b64}`);
    assert.equal(EV.snapshotUrl("data:image/png;base64,iVBOR"), "data:image/png;base64,iVBOR");
    assert.equal(EV.snapshotUrl("/rec/x/frame.jpg"), "/rec/x/frame.jpg");
    assert.ok(EV.snapshotUrl(new Blob(["x"], { type: "image/jpeg" })).startsWith("blob:"));
    const before = Date.now(), e = EV.normalizeEvent({ label: "person", score: 0.83, t: 12, snapshot: b64, room: "r1" });
    assert.ok(e.t >= before, "t: 12 s into the mission is not 1970");
    assert.equal(e.text, "person (83%)");
    assert.equal(e.urgency, "high");
    assert.equal(EV.normalizeEvent({ t: 1790000000000, text: "x", urgency: "low" }).t, 1790000000000);
  },

  "formatting"() {
    assert.equal(D.fmtBytes(75781049), "76 MB");
    assert.equal(D.fmtBytes(1.2e9), "1.2 GB");
    assert.equal(D.fmtBytes(512), "1 KB");
    assert.equal(D.fmtClock(125e3), "2:05");
    assert.equal(D.fmtDuration(32218), "32 s");
    assert.equal(D.fmtDuration(125e3), "2 min 05 s");
  },

  async "recorder: start, telemetry and command batches, a stop by the server's cap"() {
    const posts = [];
    let recording = false;
    globalThis.fetch = async (url, o = {}) => {
      if (url === "/api/session") return Response.json({ token: "t0k" });
      if (url === "/rec/status") return Response.json(recording ? { recording: true, durationMs: 1000, bytes: 10 } : { recording: false, last: { stopReason: "time cap", durationMs: 1200e3 } });
      if (url === "/rec/list") return Response.json({ recordings: [] });
      assert.equal(o.headers["X-Whoop-Token"], "t0k");
      const body = JSON.parse(o.body);
      posts.push([url, body]);
      if (url === "/rec/start") return (recording = true), Response.json({ recording: true, id: "r1", durationMs: 0 });
      return Response.json({ recording: true, written: Array.isArray(body) ? body.length : 1 });
    };
    const { FlightRecorder } = await import("../app/js/ui/recorder.js");
    const span = { textContent: "" };
    const button = { hidden: false, title: "", addEventListener() {}, setAttribute() {}, classList: { toggle() {} }, querySelector: () => span };
    let n = 0;
    const rec = new FlightRecorder({ button, list: null, sample: () => ({ n: n++ }), context: () => ({ label: "sim-test", house: "h1" }) });
    const stopped = [];
    rec.on("stopped", (s) => stopped.push(s));
    await rec.start();
    assert.ok(rec.recording);
    assert.deepEqual(posts[0], ["/rec/start", { label: "sim-test", house: "h1" }]);
    for (let t = 0; t <= 1000; t += 16) rec.tick(1e6 + t);
    rec.command("mission", { kind: "returnHome" }, "ui");
    await rec.flush();
    const tele = posts.find(([u]) => u === "/rec/telemetry")[1], cmds = posts.find(([u]) => u === "/rec/command")[1];
    assert.ok(tele.length >= 9 && tele.length <= 11, `about 10 samples a second, got ${tele.length}`);
    assert.ok(tele.every((s) => s.t > 1.7e12 && Number.isInteger(s.n)), "epoch times");
    assert.deepEqual(cmds.map(({ tool, args, source }) => [tool, args, source]), [["mission", { kind: "returnHome" }, "ui"]]);
    recording = false;
    await rec.poll();
    assert.ok(!rec.recording);
    assert.equal(stopped[0].stopReason, "time cap");
    assert.equal(stopped[0].byUs, false);
    assert.equal(span.textContent, "Record");
  },

  async "safety layer: quiet while the simulator flies the demo apartment, though a house is loaded"() {
    for (const attached of [false, true]) {
      const r = rig({ autonomy: "full", seed: 7 });
      const world = attached ? { inHouse: true, sim: false } : { inHouse: false, sim: true }; // attached: as if it applied
      const a = autonomy(r, { source: "fused", world });
      if (attached) a.localizer.known = true; // the fixture house's pose, never reset: lost as soon as it flies
      const res = await r.run(new B.TakeOff(), 20);
      r.wait(6);
      if (!attached) {
        assert.ok(res.ok, res.text);
        assert.equal(r.ctl.safety, null);
        assert.deepEqual([a.trips, a.notes, a.asks], [[], [], []], "no trips, alerts or requests to land");
        assert.ok(r.sim.drone.airborne, "still flying");
      } else assert.ok(a.trips.includes("lost") && a.notes.length, "the same layer attached does act (so the gate is what keeps it quiet)");
    }
  },

  async "safety layer on the real drone: waits for a known position, which the pad button sets on the ground only"() {
    const r = rig({ house, map, autonomy: "full", seed: 7 });
    r.sim.world.actors.list = [];
    const a = autonomy(r, { source: "fused", world: { inHouse: true, sim: false } }); // real mode: fused, never reset
    const take = await r.run(new B.TakeOff(), 20), hold = await r.run(new B.Hold(4), 6);
    assert.ok(take.ok && hold.ok, `${take.text} / ${hold.text}`);
    assert.equal(r.ctl.safety, null, "no position yet: flights work as without a house");
    assert.deepEqual([a.trips, a.notes], [[], []]);
    const before = a.localizer.pose();
    assert.match(AU.padReset({ ctl: r.ctl, missions: null, localizer: a.localizer, pad: house.home }), /Land on the home pad first/);
    assert.ok(!a.localizer.known && a.localizer.pose().x === before.x, "no reset in the air");
    assert.match(AU.padReset({ ctl: r.ctl, missions: { busy: true }, localizer: a.localizer, pad: house.home }), /Land/, "nor during a mission");
    const land = await r.run(new B.Land(), 20);
    assert.ok(land.ok && !r.ctl.isFlying(), land.text);
    assert.equal(AU.padReset({ ctl: r.ctl, missions: { busy: false }, localizer: a.localizer, pad: house.home }), null);
    const p = a.localizer.pose();
    assert.ok(a.localizer.known && p.status === "ok" && Math.hypot(p.x - house.home.x, p.y - house.home.y) < 1e-9, "at the pad");
    assert.equal(r.ctl.safety, a.safety, "attached as soon as the position is known");
    assert.match(AU.padReset({ ctl: r.ctl, localizer: a.localizer, pad: null }), /Set the home pad/);
  },

  "the simulator's Reset turns the drone at once: the pad button takes the flight controller's heading as it is, so the rest of the controller's blend isn't read as a turn"() {
    const r = rig({ house, map, autonomy: "full", seed: 7 });
    r.sim.world.actors.list = [];
    const a = autonomy(r, { source: "fused", world: { inHouse: true, sim: true } });
    r.sim.drone.yaw += 2.3; // landed turned away from the pad's heading
    r.wait(3);
    r.sim.reset(); // back on the pad, facing its way: the flight controller's heading starts again
    r.wait(0.15); // main.js presses the pad button 150 ms after
    assert.equal(AU.padReset({ ctl: r.ctl, missions: null, localizer: a.localizer, pad: house.home }), null);
    r.wait(3);
    const off = (Math.abs(Math.atan2(Math.sin(a.localizer.pose().yaw - r.sim.drone.yaw), Math.cos(a.localizer.pose().yaw - r.sim.drone.yaw))) * 180) / Math.PI;
    assert.ok(off < 3, `the estimate's heading is ${off.toFixed(1)}° off the drone's`);
  },

  "main.js: a new simulator world (People & pets switched, My house) puts the estimate back on the pad with the drone, as Reset does"() {
    const main = fs.readFileSync(path.join(import.meta.dirname, "../app/js/main.js"), "utf8"), body = (name) => main.slice(main.indexOf(`function ${name}(`), main.indexOf("\n}\n", main.indexOf(`function ${name}(`)));
    for (const f of ["setSimWorld", "simReset"]) assert.match(body(f), /localizer\.source !== "truth"[\s\S]{0,200}session\.pad\(\)/, f);
    assert.ok(body("setSimWorld").indexOf("session.pad()") > body("setSimWorld").indexOf("sim.useWorld("), "after the new world puts the drone on its pad");
  },

  async "hand-over: a stop, or a newer command, while a command waits cancels it"() {
    const turns = new AU.Turns(), values = { autonomy: "copilot", mode: "sim" }, settings = { get: (k) => values[k] };
    const prepared = [], sim = { prepareForMission: (o) => prepared.push(o) }, said = [], say = (t) => said.push(t);
    const go = (ctl = {}) => AU.handOver({ turns, settings, sim, ctl, say });
    let p = go();
    turns.stop(); // runCommand("stop") 30 ms in: stopMission()
    assert.equal(await p, false, "stopped while the simulated pilot armed");
    assert.equal(await go(), true);
    assert.deepEqual(prepared, [{ copilot: true }, { copilot: true }]);
    const older = go(), newer = go();
    assert.deepEqual([await older, await newer], [false, true], "the newer command flies");
    values.mode = "real";
    const ctl = { telemetryFresh: true, tel: { engaged: false } }, t0 = Date.now();
    p = go(ctl);
    setTimeout(() => turns.stop(), 100);
    assert.equal(await p, false);
    assert.ok(Date.now() - t0 < 1000, "the wait for the AI switch ends at the stop, not after 10 s");
    assert.deepEqual(said, ["Flip the AI switch on the radio to hand me control."]);
    p = go(ctl);
    setTimeout(() => (ctl.tel.engaged = true), 100);
    assert.equal(await p, true, "the pilot flipped the switch");
    values.autonomy = "observer";
    assert.equal(await go({ telemetryFresh: true, tel: { engaged: false } }), true, "observer: nothing to hand over");
  },

  async "Go to a place from the panel: flies beside it and faces it, also when it sits in a no-fly zone"() {
    const fan = map.keepouts.find((k) => k.kind === "fan");
    const mine = { ...house, landmarks: [{ name: "cat bed", room: map.roomAt(fan.x, fan.y)?.id, x: fan.x, y: fan.y, z: 0, source: "user" }, ...house.landmarks] };
    const r = rig({ house, map, autonomy: "full", seed: 7 });
    r.sim.world.actors.list = [];
    const a = autonomy(r, { source: "truth", world: { inHouse: true, sim: true }, home: mine });
    const missions = new MissionRunner({ ctl: r.ctl, map, house: mine, localizer: a.localizer, perception: r.perception, settings: r.ctl.settings });
    const m = MP.missionFor("goTo", { place: MP.places(mine).find((p) => p.name === "cat bed") });
    const res = await drive(r, missions.run(m), 90);
    assert.ok(res.ok, res.summary);
    assert.match(res.summary, /Arrived at the cat bed/);
    const d = r.sim.drone, off = Math.hypot(d.x - fan.x, d.y - fan.y);
    assert.ok(off > fan.r, `outside the fan's no-fly zone (${off.toFixed(2)} m from its centre, r ${fan.r} m)`);
    const facing = Math.atan2(fan.y - d.y, fan.x - d.x), err = Math.abs(Math.atan2(Math.sin(d.yaw - facing), Math.cos(d.yaw - facing)));
    assert.ok(err < 0.35, `facing the cat bed (${(err * 57.3).toFixed(0)}° off)`);
    assert.deepEqual(a.trips, []);
  },

  "files: no long lines, no paths of this Mac in what users read"() {
    const dir = path.join(import.meta.dirname, "..", "app");
    for (const f of ["js/ui/mapview.js", "js/ui/housepanel.js", "js/ui/missionpanel.js", "js/ui/events.js", "js/ui/recorder.js", "js/ui/dom.js", "js/ui/autonomy.js", "js/ui/session.js",
      "js/ui/ready.js", "js/hud.js", "js/settings.js", "js/main.js", "sw.js", "index.html", "setup.html", "../README.md"]) {
      const text = fs.readFileSync(path.join(dir, f), "utf8");
      assert.ok(!/\/Users\/|\/private\/|Library\/Application Support|SiteSpec Projects\//.test(text), `${f} names a path on this Mac`);
      if (f.endsWith(".js")) text.split("\n").forEach((l, i) => assert.ok(l.length <= 200, `${f}:${i + 1} is ${l.length} characters`));
    }
  },

  "page: every element main.js and its panels look up exists, the house panels have their places, the consent banner is never modal"() {
    const app = path.join(import.meta.dirname, "..", "app"), html = fs.readFileSync(path.join(app, "index.html"), "utf8");
    const ids = new Set([...html.matchAll(/\sid="([^"]+)"/g)].map((m) => m[1]));
    const used = new Set();
    for (const f of ["js/main.js", "js/ui/ready.js", "js/ui/missionpanel.js", "js/ui/housepanel.js", "js/ui/events.js"])
      for (const m of fs.readFileSync(path.join(app, f), "utf8").matchAll(/\$\$?\("#([A-Za-z][\w-]*)|querySelector(?:All)?\("#([A-Za-z][\w-]*)/g)) used.add(m[1] ?? m[2]);
    const missing = [...used].filter((id) => !ids.has(id));
    assert.deepEqual(missing, [], `ids the code uses but index.html lacks: ${missing.join(", ")}`);
    for (const id of ["ready", "readyBefore", "readyBattery", "aiConsent", "hudChanges", "view3d", "history", "coverage", "survey", "calib", "recheck", "changeCard", "changeCardBody", "mapLegend"])
      assert.ok(ids.has(id), `#${id} (a panel's place) is missing`);
    assert.match(html, /<canvas id="view3d"/, "the 3D view draws on a canvas in the map card");
    assert.match(html, /href="css\/panels\.css"/, "the panels' stylesheet is linked");
    const banner = html.match(/<div class="consent" id="aiConsent"[^>]*>/)?.[0] ?? "";
    assert.ok(banner && !/aria-modal="true"/.test(banner) && !/<dialog[^>]*id="aiConsent"/.test(html), "the consent banner is a plain region, not a dialog");
    assert.ok(!/aria-modal="true"/.test(html.match(/<div class="change-card"[^>]*>/)?.[0] ?? ""), "the change card is not modal either");
    const ready = fs.readFileSync(path.join(app, "js/ui/ready.js"), "utf8"), consent = ready.slice(ready.indexOf("class ConsentBanner"));
    assert.ok(!/\.focus\(|showModal|inert/.test(consent), "the banner never takes the keyboard");
    const stage = html.slice(html.indexOf('<section class="stage">'), html.indexOf("</section>"));
    assert.ok(stage.includes('id="aiConsent"') && /data-answer="once">Allow once/.test(banner + stage) && /data-answer="on">Always allow/.test(stage),
      "the banner is one line under the flight view, and says what each answer does");
  },

  "layout: the 3D view's controls hide with it, the change card and the phone's Stop bar leave Stop and Land in reach, labels for phones"() {
    const app = path.join(import.meta.dirname, "..", "app"), read = (f) => fs.readFileSync(path.join(app, f), "utf8");
    const html = read("index.html"), main = read("js/main.js"), css = read("css/app.css");
    assert.match(html, /<div class="v3d-box" id="view3dBox" hidden><canvas id="view3d"/, "View3D puts its controls in the canvas's parent: a box that hides and moves with it");
    assert.match(main, /\$\("#view3dBox"\)\.hidden = /);
    assert.ok(!/\$\("#view3d"\)\.hidden/.test(main) && /maps = \[[^\]]*\$\("#view3dBox"\)/.test(main), "the box is what syncViews moves and syncMapCanvas hides");
    assert.match(css, /\.change-card \{[^}]*right: calc\(var\(--side-w\)/, "the change card sits over the stage, clear of the side column's Stop and Land");
    assert.match(css, /grid-template-columns: minmax\(0, 1fr\) var\(--side-w\)/);
    const narrow = css.slice(css.indexOf("@media (max-width: 900px)"), css.indexOf("@media (max-width: 560px)"));
    assert.match(narrow, /\.actions \{[^}]*position: fixed[^}]*bottom: 0/, "at phone width Stop and Land stay at the bottom of the screen");
    assert.match(narrow, /\.change-card \{[^}]*bottom: calc\(var\(--actions-h\)/, "and the change card sits above them");
    assert.match(html.match(/<button class="rec-btn"[^>]*>/)[0], /aria-label="Record this flight"/, "the record button has a name where only its dot shows");
    assert.ok(!/class="[^"]*real-only[^"]*" type="button" id="btnOnPad"/.test(html), "the pad button also shows in a rehearsal");
    assert.match(html, /<details class="map-key" id="mapKey">[\s\S]*No-fly[\s\S]*Home pad[\s\S]*really is[\s\S]*Found this flight[\s\S]*People and pets now/, "the map's key");
  },

  "settings: every key the page reads or binds has a default"() {
    const { DEFAULTS } = SET, app = path.join(import.meta.dirname, "..", "app", "js"), keys = new Set(), files = [];
    const walk = (d) => fs.readdirSync(d, { withFileTypes: true }).forEach((e) => (e.isDirectory() ? walk(path.join(d, e.name)) : e.name.endsWith(".js") && files.push(path.join(d, e.name))));
    walk(app);
    for (const f of files) {
      const text = fs.readFileSync(f, "utf8"), mine = /main\.js$|ui\/session\.js$/.test(f);
      for (const m of text.matchAll(/settings\??\.(?:get|set)\??\.?\(\s*"([a-zA-Z]+)"/g)) keys.add(m[1]);
      for (const m of text.matchAll(/bindSetting\("[^"]+",\s*"([a-zA-Z]+)"/g)) keys.add(m[1]);
      if (mine) for (const m of text.matchAll(/\bs\.get\("([a-zA-Z]+)"\)/g)) keys.add(m[1]);
    }
    const missing = [...keys].filter((k) => !(k in DEFAULTS));
    assert.deepEqual(missing, [], `no default: ${missing.join(", ")}`);
    assert.equal(DEFAULTS.simRehearse, false);
  },

  "first-flight steps: the House tab, the setup guide and the README say the same twelve"() {
    const root = path.join(import.meta.dirname, ".."), read = (f) => fs.readFileSync(path.join(root, f), "utf8");
    const html = (t) => [...(t.match(/<ol class="steps">([\s\S]*?)<\/ol>/)?.[1] ?? "").matchAll(/<li><b>([^<]+)<\/b>/g)].map((m) => m[1].trim());
    const md = [...(read("README.md").match(/## Your path to the first real flight([\s\S]*?)\n## /)?.[1] ?? "").matchAll(/^\d+\. \*\*([^*]+)\*\*/gm)].map((m) => m[1].trim());
    const house = html(read("app/index.html")), setup = html(read("app/setup.html"));
    assert.equal(house.length, 12, house.join(" | "));
    assert.deepEqual(setup, house, "setup.html");
    assert.deepEqual(md, house, "README.md");
    for (const t of [read("app/index.html"), read("app/setup.html"), read("README.md")]) assert.match(t, /about 3 minutes/, "the honest limits are said too");
  },

  "HUD chips: where it is, what vision says, why it is slow"() {
    const pose = (o = {}) => ({ x: 0, y: 0, z: 1, yaw: 0, sigma: 0.06, status: "ok", source: "fused", ...o });
    const texts = (s) => HUD.statusChips(s).right.map((c) => c.text);
    assert.deepEqual(texts({ pose: pose(), known: false, real: true }), ["NO POSITION · SET ON PAD"]);
    assert.match(HUD.statusChips({ pose: pose(), known: false, real: true }).notice, /home pad/);
    assert.deepEqual(texts({ pose: pose({ status: "lost" }), known: true, flying: true }), ["POSITION LOST · HOLDING"]);
    assert.deepEqual(texts({ pose: pose(), known: true, vision: true, trust: "advisory", fix: { age: 200, rate: 4 } }), ["POS ±0.12 m", "VISION: PAD CHECK NEEDED"]);
    assert.deepEqual(texts({ pose: pose(), known: true, vision: true, trust: "verified", fix: { age: 200, rate: 3.8 } }), ["POS ±0.12 m", "VISION 3.8/s"]);
    assert.deepEqual(texts({ pose: pose(), known: true, vision: true, trust: "verified", fix: { age: 4000, rate: 0 } })[1], "NO VISION FIX");
    assert.deepEqual(texts({ pose: pose({ conflict: { apart: 0.62 } }), known: true, vision: true, trust: "verified", fix: { age: 100 } })[1], "VISION DISAGREES 0.6 m");
    assert.deepEqual(texts({ pose: pose({ source: "truth" }), known: true }), [], "the simulator's truth needs no position chip");
    assert.deepEqual(texts({ pose: null, real: true, video: { fps: 12, picture: "looking" } }), ["VIDEO: FINDING THE PICTURE"]);
    assert.deepEqual(texts({ ai: { spent: 0.12, budget: 0.5 } }), ["AI $0.12 of $0.50"]);
    assert.equal(HUD.statusChips({ pose: pose(), known: true, flying: true, why: "flying slowly: no live depth yet" }).notice, "flying slowly: no live depth yet");
    assert.equal(HUD.statusChips({ pose: pose(), known: true, flying: true, why: "a person is close" }).notice, "Holding back: a person is close");
    assert.match(HUD.statusChips({ pose: pose(), known: true, flying: true, real: true, brake: { measured: false } }).notice, /0\.3 m\/s until braking is measured/);
  },

  "Ready to fly: the runner's items with the radio's, grouped, blocks first; the page's no-video beats the runner's ok"() {
    const page = RD.shellChecks({ radio: { supported: true, connected: false }, house: house, home: true, vox: { usable: true }, webgpu: true, video: { ready: false } });
    assert.ok(page.find((i) => i.id === "radio" && !i.ok && i.level === "block" && i.fix?.action === "radio"));
    assert.ok(page.find((i) => i.id === "picture" && !i.ok && i.fix?.action === "goggles"));
    const runner = [{ id: "house", ok: true, level: "info", text: "House: yours.", when: "before" }, { id: "picture", ok: true, level: "info", text: "Video: the camera's picture is found.", when: "battery" },
      { id: "calibration", ok: false, level: "block", text: "Calibrate the camera first.", when: "before", fix: { label: "Calibrate", action: "calibrate-camera" } },
      { id: "braking", ok: false, level: "warn", text: "Braking not measured.", when: "battery" }];
    const items = RD.readyItems({ missions: { preflightCheck: () => runner }, page });
    assert.equal(items.find((i) => i.id === "picture").ok, false, "no video is no video, whatever the runner thinks");
    const noVideo = RD.readyItems({ missions: { preflightCheck: () => [{ id: "picture", ok: false, level: "block", text: "No video from the drone yet.", fix: { label: "Crop the video", action: "crop-video" } }] }, page });
    assert.equal(noVideo.find((i) => i.id === "picture").fix.action, "goggles", "with no video the fix is the goggles, not a crop");
    assert.equal(items.find((i) => i.id === "radio").fix.label, "Connect the radio (USB)");
    assert.ok(items.some((i) => i.id === "radio-script") && items.some((i) => i.id === "angle"), "the radio lines only the page knows");
    assert.equal(items.filter((i) => i.id === "house").length, 1);
    assert.equal(RD.blocking(items).id, "picture", "the first block in the list");
    assert.equal(RD.blocking(items.filter((i) => i.id !== "picture")).id, "calibration");
    assert.deepEqual([RD.groupOf(runner[1]), RD.groupOf(runner[2]), RD.groupOf({ id: "pad" }), RD.groupOf({ id: "map3d" })], ["battery", "before", "battery", "before"]);
    assert.deepEqual([RD.groupOf({ id: "standby", phase: "battery" }), RD.groupOf({ id: "warmup", phase: "battery" }), RD.groupOf({ id: "padcheck-db", phase: "before" })],
      ["battery", "battery", "before"], "the vision side's phase, over the id's guess");
    const never = RD.shellChecks({ radio: {}, house, home: true, vox: { usable: true }, splat: { enabled: true, calibOk: false }, calib: null }).find((i) => i.id === "calib");
    assert.ok(never.override && RD.groupOf(never) === "battery" && /Settings → House/.test(never.text), "a first calibration: on the pad, battery in, where the guide says");
    assert.equal(RD.radioBlocks({ supported: true, connected: true, script: true, angle: false }).map((i) => i.id).join(), "angle", "missions get the radio's blocks, not the link");
    const thrown = RD.readyItems({ missions: { preflightCheck: () => { throw new Error("boom"); } }, page: [] });
    assert.match(thrown[0].text, /boom/);
    const none = RD.readyItems({ missions: null, page });
    assert.ok(none.some((i) => i.id === "map3d") && none.some((i) => i.id === "radio"), "without preflightCheck the page's own list");
    assert.ok(RD.shellChecks({ radio: {}, house: null })[4].fix.action === "house", "no house: import it first");
  },

"Ready to fly: every fix button the checklist can show does something in main.js"() {
    const js = path.join(import.meta.dirname, "..", "app", "js"), read = (f) => fs.readFileSync(path.join(js, f), "utf8"), actions = new Set();
    for (const f of ["missions.js", "nav/readiness.js", "ui/ready.js"]) for (const m of read(f).matchAll(/action: "([a-z0-9-]+)"/g)) actions.add(m[1]);
    for (const a of ["prepare", "calibrate", "pad", "models", "calibrate-camera", "build-db", "radio", "goggles"]) assert.ok(actions.has(a), `${a} is an action somewhere`);
    assert.deepEqual([...actions].filter((a) => !RD.FIXES[a]), [], "actions without a page action");
    const main = read("main.js"), fix = main.slice(main.indexOf("function readyFix"), main.indexOf("function prepare("));
    assert.deepEqual([...new Set(Object.values(RD.FIXES))].filter((v) => !fix.includes(`case "${v}":`)), [], "page actions readyFix has no case for");
  },

  "Ready to fly: the radio's USB and script before the battery, side by side; the AI switch alone holds nothing; in the air kept unless a block"() {
    const runner = [{ id: "house", ok: true, level: "info", text: "House.", when: "before" }, { id: "radio", ok: false, level: "block", text: "The radio isn't connected (no telemetry).", when: "battery",
      fix: { label: "Radio help", action: "radio" } }, { id: "posdb", ok: false, level: "block", text: "Build the database.", when: "before" }];
    const items = RD.readyItems({ missions: { preflightCheck: () => runner }, page: RD.radioChecks({ supported: true, connected: false }) });
    assert.deepEqual(["radio", "radio-script"].map((id) => RD.groupOf(items.find((i) => i.id === id))), ["before", "before"], "connect the radio, then its script: no battery needed");
    assert.deepEqual(items.map((i) => i.id).slice(1, 4), ["radio", "radio-script", "angle"], "the radio's own lines right after it, not after everything else");
    const off = [{ id: "radio", ok: false, level: "block", text: "The AI switch on the radio is off.", when: "battery" }];
    const linked = RD.radioChecks({ supported: true, connected: true, script: true, angle: true, engaged: false }), pre = { preflightCheck: () => off };
    assert.equal(RD.blocking(RD.readyItems({ missions: pre, page: linked }))?.id, "radio");
    const asks = RD.readyItems({ missions: pre, page: linked, switchAsks: true });
    assert.equal(RD.blocking(asks), null, "the mission asks for the switch as it starts (handOver), so the switch alone doesn't hold the buttons");
    assert.match(asks.find((i) => i.id === "switch").text, /flip it on when the app asks/);
    const busy = [{ id: "pace", ok: false, level: "block", text: "The computer is too busy to fly safely (10 frames a second).", when: "battery" },
      { id: "fix", ok: false, level: "block", text: "No position fix from the camera in the last second: keep the camera's view on the room.", when: "battery" }];
    const page = (ready) => RD.shellChecks({ radio: {}, house, home: true, vox: { usable: true }, webgpu: true, video: { ready } });
    const noVideo = RD.readyItems({ missions: { preflightCheck: () => busy }, page: page(false) });
    assert.ok(noVideo.filter((i) => i.id === "pace" || i.id === "fix").every((i) => i.level === "info" && /video/.test(i.text)), "no video: no 'too busy' or 'face the room' advice (stale frame rates)");
    assert.deepEqual(RD.readyItems({ missions: { preflightCheck: () => busy }, page: page(true) }).filter((i) => i.level === "block" && /^(pace|fix)$/.test(i.id)).map((i) => i.id), ["pace", "fix"], "with video they count");
    const block = [{ id: "x", ok: false, level: "block" }], fine = [{ id: "x", ok: true, level: "info" }];
    assert.equal(RD.keepReady({ flying: false, items: fine, at: 0, now: 1 }), false, "on the ground: checked every time");
    assert.equal(RD.keepReady({ flying: true, items: fine, at: 0, now: 600000 }), true, "in the air, nothing blocking: not checked again");
    assert.deepEqual([RD.keepReady({ flying: true, items: block, at: 0, now: 1000 }), RD.keepReady({ flying: true, items: block, at: 0, now: RD.READY.flyingEvery })], [true, false],
      "a block in the air: checked again every few seconds");
  },

  "main.js: the panels follow the session's twin, the checklist rests in the air, Reset and the pad, the House panels after an import, the panels' options"() {
    const app = path.join(import.meta.dirname, "..", "app"), main = fs.readFileSync(path.join(app, "js/main.js"), "utf8"), css = fs.readFileSync(path.join(app, "css/app.css"), "utf8");
    const fn = (name) => {
      const i = main.indexOf(`function ${name}(`);
      assert.ok(i >= 0, `main.js has ${name}()`);
      return main.slice(i, main.indexOf("\n}\n", i));
    };
    assert.match(fn("syncUi"), /syncPanelTwins\(\)/, "every session sync hands the 3D view and calibration the current twin");
    assert.match(main, /session\.on\("twin", syncPanelTwins\)/, "and before the real drone's twin is disposed");
    assert.match(fn("updateReady"), /keep = keepReady\(\{ flying, items: readyCache, at: readyAt, now \}\)/);
    assert.match(fn("updateReady"), /keep \? readyCache : readyCheck\(\)/, "in the air the cached list, not preflightCheck every second");
    assert.match(fn("readyCheck"), /switchAsks/);
    assert.match(fn("runMission"), /blocking\(readyCheck\(\)\)[\s\S]*await handOver\(\)/, "missions refuse on the list's blocks before the hand-over");
    assert.match(fn("simReset"), /session\.pad\(\)/, "Reset puts the estimate back on the pad where the camera drives it");
    assert.match(fn("refreshHouseTools"), /\["coverage", "survey", "calib"\]\.forEach\(mount\)/, "the House panels mount right after the first import");
    const panels = main.slice(main.indexOf("const PANELS = {"), main.indexOf("const MISSING ="));
    assert.match(panels, /calib: [\s\S]*restore: \(\) => session\.applyCalib\(\)[\s\S]*mode: \(\) => settings\.get\("mode"\)/, "a discarded calibration puts the saved lens back");
    assert.match(panels, /recheck: [\s\S]*flying,[\s\S]*pickRecording\?\.\(id\)/, "the recheck stops at take-off and calibrates from the flight it checked");
    assert.match(panels, /coverage: [^\n]*flying/);
    assert.match(panels, /history: [^\n]*flying/);
    assert.match(main, /panels\.calib\?\.refreshRecordings\?\.\(\);\s*panels\.recheck\?\.refresh\?\.\(\);/, "a recording that stops shows up in both panels");
    assert.match(main.slice(main.indexOf("const simTruth")), /return \{ x: d\.x, y: d\.y, z: d\.z,/, "the 3D view gets the simulated drone's height");
    assert.match(main, /\$\("#mapKey"\)\.addEventListener\("toggle", fitMapKey\)/);
    assert.match(css, /\.map-key ul \{[^}]*max-height: [^;]+;[^}]*overflow-y: auto/, "the open map key scrolls inside the map card (which clips)");
    assert.match(fn("setupCropDrag"), /source\.cropFromView\?\.\(box\)/, "a crop drawn on the picture is kept in the stream's terms");
  },

  "map layers: the trail fades and colours by how sure, lost marked; the 3D layer's height"() {
    const s = [{ x: 0, y: 0, sigma: 0.05 }, { x: 1, y: 0, sigma: 0.05 }, { x: 2, y: 0, sigma: 0.2 }, { t: 1, lost: true }, { x: 3, y: 0, sigma: 0.4 }, { x: 4, y: 0, sigma: 0.4 }];
    const { segs, lost } = MV.trailSegments(s);
    assert.equal(segs.length, 3);
    assert.deepEqual(lost, [[2, 0]]);
    assert.deepEqual(segs.map((g) => g.rgb[0]), [74, 250, 248], "green, yellow, red");
    assert.ok(segs[0].alpha < segs[2].alpha && segs[0].alpha >= 0.25 && segs[2].alpha <= 0.95, "older fades");
    assert.equal(MV.layerHeight(null, null), 1.0);
    assert.equal(MV.layerHeight({ z: 0.05 }, 0), 1.0, "on the ground: a usual flying height");
    assert.equal(MV.layerHeight({ z: 1.34 }, 0.1), 1.2);
    assert.equal(MV.layerHeight({ z: 3 }, 0), 1.6);
    assert.ok(Object.keys(MV.LAYERS).every((k) => typeof MV.LAYERS[k] === "boolean"));
    assert.equal(MV.LAYERS.gaps, false, "gaps show when the coverage report asks (they looked like no-fly zones)");
    const mv = Object.assign(Object.create(MV.MapView.prototype), { findings: [], memory: { flight: { id: "f1" } } });
    for (const f of [{ label: "person", room: "r1", x: 1, y: 1 }, { label: "person", room: "r1", x: 2, y: 1 }, { label: "cat", room: "r1", x: 1, y: 2 }, { label: "person", room: "r2", x: 5, y: 1 }]) mv.addFinding(f);
    assert.deepEqual(mv.findings.map((f) => `${f.label}@${f.room}:${f.x}`), ["person@r1:2", "cat@r1:1", "person@r2:5"], "the latest per label and room");
    assert.equal(mv.findingsFlight, "f1");
    mv.newFlight();
    assert.deepEqual([mv.findings.length, mv.trailFlight], [0, null], "a take-off starts the map's findings again");
  },

  async "house session: switching houses and simulator/real lets go of everything; depth changes the house only from video it trusts"() {
    const made = { twins: [], memories: [], depthStarts: 0 }, values = { mode: "sim", simLoc: "truth", locVision: true, avoid: true, memoryDays: 90, memoryFlights: 200 };
    const settings = { get: (k) => values[k], set: (k, v) => (values[k] = v) };
    const ctl = Object.assign(new Emitter(), { safety: null, flying: false, isFlying() { return this.flying; } });
    const perception = Object.assign(new Emitter(), { geometry: {}, latest: { luma: 120 } });
    class Loc extends Emitter {
      constructor(o) { super(); Object.assign(this, o, { known: true, src: "fused" }); }
      setMap(map, house) { Object.assign(this, { map, house }); }
      setSource(s) { this.src = s; }
      setTruth(f) { this.truth = f; }
      pose() { return { x: 0, y: 0, z: 1, yaw: 0, sigma: 0.05, status: "ok", source: this.src }; }
      fixQuality() { return { age: this.fixAge ?? Infinity, visionAge: this.fixAge ?? Infinity, rate: 3 }; }
      reset(p) { this.at = p; }
    }
    class Saf extends Emitter {
      constructor(o) { super(); Object.assign(this, o, { avoid: { ingested: 0, ingest() { this.ingested++; }, setDepthStatus() {} } }); }
      setMap(map) { this.map = map; }
    }
    class Spl extends Emitter {
      constructor(o) { super(); Object.assign(this, o, { enabled: false, twin: null, db: { meta: { key: "k" } }, trusted: "advisory", parts: {}, refreshed: 0, verified: 0 }); }
      checklist(parts) { Object.assign(this.parts, parts ?? {}); return (this.list ??= Object.assign(new Emitter(), { refresh: async () => this.refreshed++ })); }
      readiness() { return this.enabled ? [{ id: "posdb", phase: "before", ok: false, level: "warn", text: "Build the position database.", fix: { label: "Prepare now", action: "prepare" } }] : []; }
      verifyOnPad() { this.verified++; return Promise.resolve(); }
      attach(p) { this.unsub = p.on("frame", () => {}); }
      setHouse(house, map) { Object.assign(this, { house, map }); }
      setTwin(t) { this.twin = t; }
      setCalib(c) { this.calib = c; }
      dbKey() { return "k"; }
      trust() { return this.trusted; }
    }
    class Dep extends Emitter {
      constructor(o) { super(); Object.assign(this, o, { estimator: null }); }
      start() { made.depthStarts++; this.estimator = {}; this.unsub = perception.on("frame", () => {}); return Promise.resolve({}); }
      stop() { this.estimator = null; this.unsub?.(); }
      setTwin(t) { this.twin = t; }
      setCalib() {}
    }
    class Chg extends Emitter {
      constructor(o) { super(); Object.assign(this, o, { n: 0 }); }
      setMap(map) { this.map = map; }
      setMemory(m) { this.memory = m; }
      setTrust(t) { this.trust = t; }
      ingest() { this.n++; }
    }
    class Mis extends Emitter {
      constructor(o) { super(); Object.assign(this, o); }
      setHouse(o) { Object.assign(this, o); }
      stop() {}
    }
    const fakeMap = (id) => ({ id, temps: new Map(), setVoxels(v) { this.vox = v; } });
    const vox = (id) => ({ id, version: 1, built: { usable: true }, n: 10, res: 0.05, integrated: 0, seen: 0, refresh: async () => {}, integrate() { this.integrated++; }, markSeen() { this.seen++; },
      counts: () => ({ free: 5 }), forgetFlight() { this.forgot = true; } });
    const store = { loadVoxels: async (h) => vox(h.id), saveVoxels: async () => (made.saved = (made.saved ?? 0) + 1), readFile: async (id, name) => (name === "calib.json" ? null : new ArrayBuffer(8)),
      saveHouse: async () => {}, loadMap: async (h) => fakeMap(h.id) };
    const HouseMemory = { open: async (id, o) => { const m = Object.assign(new Emitter(), { id, world: o.world, stored: true, changes: [], paused: [], setHouse(x) { this.house = x.house ?? null; }, seenPaused(w) { this.paused.push(w); },
      openChanges: () => ({ open: 2, blocking: 1, stale: 1, people: 0 }) }); made.memories.push(m); return m; } };
    const Twin = { create: async () => { const t = { disposed: false, dispose() { this.disposed = true; }, pixels: async () => ({}) }; made.twins.push(t); return t; } };
    const angle = { id: "angle", ok: false, level: "block", text: "Put the quad in ANGLE (self-level) mode.", radio: true };
    const S = new SESSION.HomeSession({ ctl, perception, settings, store: async () => store, sim: () => ({ world: { kind: "house" }, drone: { x: 1, y: 2, z: 1, yaw: 0 } }), pageChecks: () => [angle],
      mods: { LOC: { Localizer: Loc }, SAF: { Safety: Saf }, MIS: { MissionRunner: Mis }, SPL: { SplatLocalizer: Spl }, DEP: { LiveDepth: Dep, mapDepth: (f) => f.depth }, CHG: { ChangeDetector: Chg, changeTrust: CHGM.changeTrust },
        HouseMemory, Twin, buildVoxels: async () => vox("built"), voxelCentres: async () => null, houseFrame: () => ({}), coverageReport: () => ({ score: 90 }) } });
    const tick = () => new Promise((r) => setTimeout(r, 0));
    const houses = ["A", "B", "C"].map((id) => ({ id, name: id, splatFile: "splat.spz", frame: { f: 1, Yf: 0 }, home: { x: 0, y: 0, yaw: 0 } }));
    await S.setHouse(houses[0], fakeMap("A"));
    await tick();
    assert.equal(S.vox.id, "A");
    assert.equal(S.localizer.src, "truth", "the simulator's truth while simLoc is truth");
    assert.ok(S.splat.parts.depth === S.depth && S.splat.parts.has3D() === true && typeof S.splat.parts.prepare3D === "function",
      "the vision checklist sees live depth's model and the 3D map, and can build it");
    assert.ok(S.trust && S.changes.trust === S.trust, "change detection and the session judge video by one rule (nav/changes.js changeTrust)");
    let readyEvents = 0;
    S.on("ready", () => readyEvents++);
    S.splat.list.emit("change");
    assert.equal(readyEvents, 1, "the vision checklist's changes reach the page");
    const progress = [];
    S.on("progress", (p) => p.key === "vision" && progress.push([p.text, p.value]));
    S.splatSaid({ key: "vision-model", text: "Downloading DINOv2: 97% of 44 MB", progress: 0.97 });
    S.splatSaid({ key: "vision-model", text: "Downloading DINOv2: 100% of 44 MB", progress: 1 });
    S.splatSaid({ key: "vision-model", text: "Downloading DINOv2: 100% of 44 MB", progress: 1 });
    assert.deepEqual(progress, [["Downloading DINOv2: 97% of 44 MB", 0.97], ["Downloading DINOv2: 100% of 44 MB", 1], [null, null]], "the download line ends on its 100%, not stuck at 97%");
    assert.deepEqual(S.openChanges(), { open: 2, blocking: 1, stale: 1, people: 0 }, "the HUD's count: open, and how many the drone keeps clear of now");
    const simTwin = { pixels: async () => ({}), dispose() {} };
    S.setSimTwin(simTwin);
    assert.equal(S.depthTwin(), simTwin);
    await tick();
    assert.equal(made.depthStarts, 1, "live depth starts with a twin");
    const listeners = () => [...perception._handlers.values()].reduce((a, s) => a + s.size, 0) + [...ctl._handlers.values()].reduce((a, s) => a + s.size, 0);
    const base = listeners();
    // the simulator's frames are trusted: changes, voxels and what the camera saw
    ctl.safety = S.safety;
    S.depthFrame({ pose: S.localizer.pose(), lens: {}, depth: [], width: 4, height: 3, t: performance.now() });
    assert.deepEqual([S.safety.avoid.ingested, S.changes.n, S.vox.integrated, S.vox.seen, S.simTouched], [1, 1, 1, 1, true]);
    assert.deepEqual(S.readiness(), [], "nothing more to check in the simulator with its exact position");
    // a rehearsal: vision drives the position, so the real drone's rules (a fresh fix before video changes the house; its
    // vision items block missions; the simulator's scan camera must run)
    Object.assign(values, { simLoc: "vision", simRehearse: true });
    S.sync();
    assert.ok(S.splat.enabled && S.rehearsing());
    assert.equal(S.localizer.expectVision, true, "the localizer counts its clocks from take-off when vision runs");
    S.localizer.fixAge = 2500;
    assert.match(S.untrusted(), /fresh camera position fix/, "a stale fix in a rehearsal: the video changes nothing");
    const said = [];
    S.on("log", (l) => /Change detection paused/.test(l.text) && said.push(l.text));
    for (let i = 0; i < 3; i++) S.depthFrame({ pose: S.localizer.pose(), lens: {}, depth: [], width: 4, height: 3, t: performance.now() });
    assert.deepEqual([S.safety.avoid.ingested, S.changes.n], [4, 1]);
    assert.deepEqual(said, ["Change detection paused: no fresh camera position fix."], "said once, not at every frame");
    assert.deepEqual(S.memory.paused, Array(3).fill("no fresh camera position fix"), "the flight report learns why the camera's share wasn't measured");
    ["the position isn't sure enough", "no fresh camera position fix", "the position isn't sure enough"].forEach((w) => S.paused(w));
    assert.equal(said.length, 2, "reasons that come and go are each said once a flight");
    const rehearsal = S.readiness();
    assert.equal(rehearsal.find((i) => i.id === "posdb").level, "block", "vision items block missions in a rehearsal");
    assert.ok(rehearsal.some((i) => i.id === "fix" && !i.ok && i.level === "block") && rehearsal.some((i) => i.id === "sim-camera" && i.level === "block"));
    assert.ok(!rehearsal.some((i) => i.id === "angle"), "the radio's lines are the real drone's");
    S.localizer.fixAge = 300;
    assert.equal(S.untrusted(), "");
    assert.ok(!S.readiness().some((i) => i.id === "fix" && !i.ok));
    Object.assign(values, { simLoc: "truth", simRehearse: false });
    S.sync();
    assert.equal(S.localizer.expectVision, false);
    S.localizer.fixAge = undefined;
    const touched = S.vox;
    // the real drone: its own twin, a real memory; untrusted video only feeds avoidance
    values.mode = "real";
    S.setSimTwin(null);
    await tick();
    await tick();
    await tick();
    assert.equal(made.twins.length, 1, "the real drone's localization twin");
    assert.equal(S.locTwin, made.twins[0]);
    assert.equal(S.memory.world, "real");
    assert.equal(S.localizer.src, "fused");
    assert.ok(S.vox !== touched && S.vox.id === "A" && S.vox.integrated === 0, "the simulator's flight left the 3D map: the stored one again");
    assert.ok(S.readiness().some((i) => i.id === "angle"), "missions see the page's radio blocks on the real drone (voice and Claude too)");
    assert.ok(S.splat.refreshed > 0, "the checklist looks at the downloads in real mode");
    S.pad();
    assert.equal(S.splat.verified, 0, "no pad check without video");
    perception.source = { kind: "goggles", ready: () => true };
    let padResults = 0;
    S.on("pad", () => padResults++);
    S.pad();
    assert.equal(S.splat.verified, 1, "the pad check with the video");
    await tick();
    assert.equal(padResults, 1, "its result reaches the page (the map redraws: the pose may have been re-seated)");
    perception.source = undefined;
    ctl.safety = S.safety;
    S.depthFrame({ pose: S.localizer.pose(), lens: {}, depth: [], width: 4, height: 3, t: performance.now() });
    assert.deepEqual([S.safety.avoid.ingested, S.changes.n], [5, 1], "no change detection before calibration and the pad check");
    assert.match(S.untrusted(), /calibrated and checked/);
    S.calib = { verified: true };
    S.splat.trusted = "verified";
    S.localizer.fixAge = 300;
    assert.equal(S.untrusted(), "");
    S.depthFrame({ pose: S.localizer.pose(), lens: {}, depth: [], width: 4, height: 3, t: performance.now() });
    assert.equal(S.changes.n, 2, "calibrated, checked on the pad, a fresh fix: changes count");
    S.localizer.fixAge = 2500;
    assert.match(S.untrusted(), /fresh camera position fix/);
    // house switches: the old twin goes, memories are let go, no listener piles up
    const firstReal = S.memory;
    for (const h of [houses[1], houses[2], houses[0], houses[1]]) {
      await S.setHouse(h, fakeMap(h.id));
      for (let i = 0; i < 4; i++) await tick();
    }
    assert.ok(made.twins.slice(0, -1).every((t) => t.disposed), "every earlier localization twin was disposed");
    assert.ok(!made.twins.at(-1).disposed && S.locTwin === made.twins.at(-1));
    assert.equal(firstReal.house, null, "the old memory let go of the house (and its obstacles on the map)");
    assert.equal(listeners(), base, "no listeners pile up on perception or the controller");
    const left = (e) => [...e._handlers.values()].reduce((a, h) => a + h.size, 0);
    assert.ok(made.memories.filter((m) => m !== S.memory).every((m) => left(m) === 0), "the session stopped listening to every memory it let go");
    assert.ok(left(S.memory) > 0);
    assert.equal(S.memory.id, "B");
    // back to the simulator: the real drone's twin goes
    values.mode = "sim";
    let goneFirst = null;
    S.on("twin", () => (goneFirst ??= !made.twins.at(-1).disposed && !S.locTwin));
    S.dropLocTwin();
    assert.equal(goneFirst, true, "the panels hear the twin go before it is disposed");
    S.sync();
    await tick();
    assert.ok(made.twins.at(-1).disposed && !S.locTwin);
    assert.equal(S.memory.world, "sim");
    // landing: forget what flights alone found, save (not the simulator's)
    let tookOff = 0;
    S.on("takeoff", () => tookOff++);
    ctl.flying = true;
    S.tick();
    assert.equal(tookOff, 1);
    ctl.flying = false;
    const saved = made.saved ?? 0;
    S.simTouched = false;
    S.tick();
    assert.ok(S.vox.forgot);
    await tick();
    assert.equal(made.saved, saved + 1);
    S.dispose();
  },
};

let failed = 0;
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name}\n      ${e.stack?.split("\n").slice(0, 3).join("\n      ")}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed`);
process.exit(failed ? 1 : 0);
