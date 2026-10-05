// The wave C panels' pure parts without a browser (app/js/ui/view3d.js, history.js, changecard.js, survey.js, coverage.js,
// calib.js, recheck.js) on the house fixture (fixtures/house): the 3D view's camera against twin/lens.js's own math (the
// target in the middle, a metre aside where the pinhole puts it, near-plane clipping), the trail's colours by σ and its
// fading, lost markers and gaps, the camera kept inside the rooms, the unseen-space haze only at flying heights inside the
// rooms (every 20 cm block with unseen space in it has its dot), the chase camera turned round the drone next to a wall;
// the history's days, flights in words (rooms in order, what it saw, positions lost, how it ended, what the camera looked
// at) and "last seen" order from a real HouseMemory, an older flight's trail read back from the store; a change card's
// verdicts, effects, who decided and what Ask Claude sends; the survey's pre-ticks (not a name the map has in that room)
// and what Apply writes (source "claude", the user's names kept, applySurvey's result); coverage rows (gaps never "well
// covered" alone, shares of the open space) and what flights added (flight-only free space, what the camera saw); the
// calibration's before/after shift and words; a recording's checks and a replay's report, its verdict agreeing with its
// lines (no pad press: never a pass) and when calibrating from it is offered.
// Usage: cd tools && node test-panels.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";

const I = await import("../app/js/house/import.js");
const V = await import("../app/js/house/voxels.js");
const { coverageReport } = await import("../app/js/house/coverage.js");
const { HouseMemory } = await import("../app/js/memory/memory.js");
const { applySurvey, hazardKeepout } = await import("../app/js/ai/survey.js");
const { intrinsics, project, pinholeLens, DRONE_LENS, DEG } = await import("../app/js/twin/lens.js");
const { droneCamera } = await import("../app/js/twin/pose.js");
const { droneLens } = await import("../app/js/nav/lens.js");
const V3 = await import("../app/js/ui/view3d.js");
const HI = await import("../app/js/ui/history.js");
const CC = await import("../app/js/ui/changecard.js");
const SV = await import("../app/js/ui/survey.js");
const CO = await import("../app/js/ui/coverage.js");
const CA = await import("../app/js/ui/calib.js");
const RC = await import("../app/js/ui/recheck.js");

const ROOT = path.join(import.meta.dirname, "..");
const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const near = (a, b, tol, what) => assert.ok(Math.abs(a - b) <= tol, `${what}: ${a} vs ${b} (±${tol})`);
const { house, map, points } = await I.importCapture(fsSource(FIXTURE), { keepPoints: true });
const vox = await V.buildVoxels({ house, map, centres: points });
const T0 = new Date(2026, 9, 4, 15, 0, 0).getTime(); // a fixed "now": Sunday 4 Oct 2026, 15:00 local
const room = (id) => house.rooms.find((r) => r.id === id);
const centre = (id) => { const o = room(id).outline; return [o.reduce((a, p) => a + p[0], 0) / o.length, o.reduce((a, p) => a + p[1], 0) / o.length]; };

// A memory with three flights: yesterday's patrol through the rooms (σ growing, a lost stretch), today's search that saw the
// cat twice and a person, and a simulator flight; changes suspected, confirmed and dismissed.
async function filledMemory() {
  let now = T0 - 864e5 - 3600e3;
  const m = await HouseMemory.open(house.id, { indexedDB: null, now: () => now });
  m.setHouse({ house, map, vox: null, save: null });
  const fly = (fromId, toId, { t0, lost = null, sigma = (i) => 0.05 } = {}) => {
    const [ax, ay] = centre(fromId), [bx, by] = centre(toId), n = 40;
    for (let i = 0; i <= n; i++) {
      now = t0 + i * 500;
      const k = i / n, x = ax + (bx - ax) * k, y = ay + (by - ay) * k;
      const gone = lost && i >= lost[0] && i < lost[1];
      m.samplePose(gone ? { status: "lost" } : { x, y, z: 1.0, yaw: Math.atan2(by - ay, bx - ax), sigma: sigma(i), status: "ok", room: map.roomAt(x, y)?.id ?? null }, now);
    }
  };
  m.startFlight({ kind: "patrol", mission: { kind: "patrol" } }, now);
  fly("r1", "r2", { t0: now, sigma: (i) => 0.04 + i * 0.006 });
  fly("r2", "r3", { t0: now + 500, lost: [10, 16] });
  m.endFlight("Patrolled 3 rooms.", now + 500);
  now = T0 - 600e3;
  m.startFlight({ kind: "searchFor", mission: { kind: "searchFor", target: "cat" } }, now);
  fly("r1", "r3", { t0: now });
  const [cx, cy] = centre("r3");
  m.addSighting({ label: "cat", room: "r3", x: cx, y: cy, z: 0.3, score: 0.8, sigma: 0.05, t: now - 8000 });
  m.addSighting({ label: "person", room: "r1", x: centre("r1")[0], y: centre("r1")[1], z: 1.2, score: 0.9, sigma: 0.05, t: now - 15000 });
  const box = m.addChange({ kind: "obstacle", x: cx + 0.3, y: cy, z: 0.3, size: 0.4 }, now - 4000);
  const door = m.addChange({ kind: "door-closed", door: house.doors[0].id, x: house.doors[0].a[0], y: house.doors[0].a[1], z: 1 }, now - 3000);
  const ghost = m.addChange({ kind: "obstacle", x: centre("r2")[0], y: centre("r2")[1], z: 0.5, size: 0.3 }, now - 2000);
  m.endFlight("Found the cat.", now);
  now = T0 - 120e3;
  const sim = await HouseMemory.open(house.id, { indexedDB: null, world: "sim", now: () => now });
  return { m, sim, box, door, ghost, setNow: (t) => (now = t) };
}

const tests = {
  "History words the records again when the memory gets its house after the panel (the simulator's world switched: no 'r3' left in the text)"() {
    const src = fs.readFileSync(path.join(ROOT, "app", "js", "ui", "history.js"), "utf8");
    assert.match(src, /\["flight", "sighting", "change", "annotate", "named"\]\.map\(\(e\) => memory\.on\(e, soon\)\)/);
  },
  "3D view: the orbit camera agrees with the twin's pinhole math (target in the middle, 1 m aside at fx/dist, behind the camera null)"() {
    const W = 640, H = 360, cam = { target: [1, 2, 1], az: 30 * DEG, el: 20 * DEG, dist: 3 }, P = V3.projector(cam, W, H, 70);
    const mid = P.px(cam.target);
    near(mid[0], W / 2, 1e-6, "u of the target");
    near(mid[1], H / 2, 1e-6, "v of the target");
    near(mid[2], 3, 1e-9, "depth of the target");
    const pose = V3.orbitPose(cam), c = droneCamera(pose, 0), K = intrinsics(pinholeLens(70), W, H);
    for (const Q of [[2, 2.5, 0.4], [0, 3, 1.6], [1.5, 1, 0]]) {
      const r = Q.map((v, k) => v - c.p[k]), q = [0, 1, 2].map((j) => c.R[0][j] * r[0] + c.R[1][j] * r[1] + c.R[2][j] * r[2]), want = project(K, q), got = P.px(Q);
      near(got[0], want[0], 1e-6, "u vs twin/lens project");
      near(got[1], want[1], 1e-6, "v vs twin/lens project");
    }
    // level camera looking along +x: a point 1 m to the camera's right (−y in H) at the target's distance
    const L = V3.projector({ target: [0, 0, 1], az: 0, el: 0, dist: 2 }, W, H, 90), right = L.px([0, -1, 1]);
    near(right[0], W / 2 + (W / 2) * (1 / 2), 1e-6, "1 m right at 2 m with a 90° pinhole");
    assert.equal(L.px([-3, 0, 1]), null, "behind the camera");
    const seg = V3.clipSeg(L.toCam([-3, 0, 1]), L.toCam([2, 0, 1]));
    near(seg[0][2], V3.VIEW3D.near, 1e-9, "a segment through the camera is cut at the near plane");
    assert.equal(V3.clipPoly([[0, 0, -1], [1, 0, -1], [1, 1, -2]]).length, 0, "a polygon all behind is dropped");
    console.log(`      target at (${mid[0].toFixed(1)}, ${mid[1].toFixed(1)}); 1 m right at 2 m: u ${right[0].toFixed(1)} of ${W}`);
  },

  "3D view: the trail fades with age, takes its colour from σ, breaks at gaps and marks where the position was lost"() {
    const s = (t, sigma, extra = {}) => ({ t, x: t / 1000, y: 0, z: 1, sigma, ...extra });
    const tr = [s(0, 0.05), s(500, 0.05), s(1000, 0.2), s(1500, 0.4), s(2000, 0.9), { t: 2500, lost: true }, { t: 3000, lost: true }, s(3500, 0.05), s(4000, 0.05), s(12000, 0.05), s(12500, 0.05)];
    const { segments, lost } = V3.trailSegments(tr, { live: false });
    assert.deepEqual(segments.map((g) => g.color), ["#4ade80", "#facc15", "#f87171", "#f87171", "#4ade80", "#4ade80"], "colours by σ (as the map's), nothing across the loss or the 8 s gap");
    assert.ok(segments.every((g, i) => i === 0 || g.alpha >= segments[i - 1].alpha), "older is fainter");
    near(segments[0].alpha, 0.2 + 0.8 * (500 / 12500), 1e-3, "the oldest piece");
    near(segments.at(-1).alpha, 1, 1e-9, "the newest piece");
    assert.equal(lost.length, 1);
    assert.equal(lost[0].t, 2000, "lost: marked at the last sure sample");
    const live = V3.trailSegments([s(0, 0.05), s(500, 0.05), s(119500, 0.05), s(120000, 0.05)], { now: 120000, live: true });
    assert.ok(live.segments[0].alpha < 0.25 && live.segments.at(-1).alpha === 1, "live: two minutes old is nearly gone");
    assert.equal(V3.sigmaColor(0.099), "#4ade80");
    assert.equal(V3.sigmaColor(0.1), "#facc15");
    assert.equal(V3.sigmaColor(0.25), "#f87171");
    // the words give ±2σ, as the caption and the HUD do: σ 0.08 is "±16 cm", inside "±20 cm or better"
    assert.deepEqual(V3.SIGMA_STEPS.map((s) => s[2]), ["sure (±20 cm or better)", "fairly sure (±20 to 50 cm)", "unsure (more than ±50 cm)"]);
    assert.deepEqual([0.08, 0.12, 0.3].map(V3.sigmaWord), ["", "fairly sure", "unsure"]);
    assert.notEqual(V3.STATUS.degraded, "#f59e0b", "the unsure drone isn't the cat pin's colour");
  },

  "3D view: the camera stays inside the rooms for a render; the unseen haze is only flying-height space inside the rooms"() {
    const [x, y] = centre("r1"), fl = map.floorAt(x, y);
    assert.ok(V3.insideRooms(map, vox, [x, y, fl + 1.2]), "the middle of the living room at 1.2 m");
    assert.ok(!V3.insideRooms(map, vox, [x, y, fl + 5]), "above the ceiling");
    assert.ok(!V3.insideRooms(map, vox, [x, y, fl - 0.2]), "under the floor");
    assert.ok(!V3.insideRooms(map, vox, [-50, -50, 1]), "off the map");
    const cloud = V3.unknownCloud(vox, map), n = cloud.length / 4, step = V3.VIEW3D.cloud.step, blocks = new Set();
    assert.ok(n > 50 && n <= V3.VIEW3D.cloud.max, `${n} points`);
    for (let i = 0; i < cloud.length; i += 4) {
      const k = map.idx(cloud[i], cloud[i + 1]), up = cloud[i + 2] - map.floorZ[k];
      assert.ok(k >= 0 && map.room[k] >= 0 && up >= 0.3 - 1e-6 && up <= 1.6 + 1e-6, `point ${i / 4} at ${up.toFixed(2)} m`);
      assert.equal(vox.state(cloud[i], cloud[i + 1], cloud[i + 2]), V.UNKNOWN);
      assert.ok(cloud[i + 3] > 0 && cloud[i + 3] <= 1, "the block's unseen share");
      const j = vox.idx(cloud[i], cloud[i + 1], cloud[i + 2]), c = j % vox.nx, r = Math.floor(j / vox.nx) % vox.ny, l = Math.floor(j / (vox.nx * vox.ny));
      blocks.add(`${Math.floor(l / step)},${Math.floor(r / step)},${Math.floor(c / step)}`);
    }
    // every block with an unseen voxel at flying height inside a room has its dot (not only blocks whose corner is unseen)
    const want = new Set();
    for (let i = 0; i < vox.n; i++) {
      if (vox.st[i] !== V.UNKNOWN) continue;
      const [x, y, z] = vox.center(i), k = map.idx(x, y);
      if (k < 0 || map.room[k] < 0 || map.wall[k] || z - map.floorZ[k] < 0.3 || z - map.floorZ[k] > 1.6) continue;
      const c = i % vox.nx, r = Math.floor(i / vox.nx) % vox.ny, l = Math.floor(i / (vox.nx * vox.ny));
      want.add(`${Math.floor(l / step)},${Math.floor(r / step)},${Math.floor(c / step)}`);
    }
    assert.equal(blocks.size, n, "one dot per block");
    assert.deepEqual([...want].filter((b) => !blocks.has(b)), [], `${want.size} blocks have unseen space; all drawn`);
    // next to a wall, facing away from it: pulled in only, the chase camera stays in the wall; turned round the drone, it's in the room
    const o = room("r1").outline, [a, b] = [o[0], o[1]], mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2], [cx, cy] = centre("r1");
    const inward = Math.atan2(cy - mid[1], cx - mid[0]), at = [mid[0] + 0.15 * Math.cos(inward), mid[1] + 0.15 * Math.sin(inward)], fz = map.floorAt(...at) + 0.05;
    const cam = { target: [at[0], at[1], fz], az: inward, el: V3.VIEW3D.chase.el, dist: V3.VIEW3D.chase.dist }, posOf = V3.View3D.prototype.posOf;
    const pulled = [];
    for (let d = cam.dist; d >= V3.VIEW3D.chase.min - 1e-9; d -= 0.15) pulled.push(V3.insideRooms(map, vox, posOf({ ...cam, dist: d })));
    assert.ok(!pulled.some(Boolean), "behind the drone, against the wall: outside at every distance");
    const fit = V3.View3D.prototype.fit.call({ map, vox, posOf }, cam);
    assert.ok(fit && V3.insideRooms(map, vox, posOf(fit)), "fit() finds a spot inside the room");
    const path = [[...centre("r1"), 1.0], [...centre("r2"), 1.0]], solid = V3.solidNear(vox, path);
    for (let i = 0; i < solid.length; i += 3) assert.equal(vox.state(solid[i], solid[i + 1], solid[i + 2]), V.OCCUPIED);
    console.log(`      ${n} unseen blocks drawn (all ${want.size}), ${solid.length / 3} solid voxels near a path; next to a wall the camera turned ${Math.round(((fit.az - cam.az) * 180) / Math.PI)}° to ${fit.dist.toFixed(2)} m`);
  },

  async "history: flights by day, in words (rooms in order, what it saw, losses), last seen people and pets first"() {
    const { m, sim } = await filledMemory();
    assert.equal(HI.dayLabel(T0 - 3600e3, T0), "Today");
    assert.equal(HI.dayLabel(T0 - 864e5, T0), "Yesterday");
    assert.equal(HI.dayLabel(T0 - 3 * 864e5, T0), "Thursday");
    assert.equal(HI.dayLabel(T0 - 20 * 864e5, T0), "14 Sep");
    const groups = HI.groupFlights(m.flights, T0);
    assert.deepEqual(groups.map((g) => [g.day, g.flights.length]), [["Today", 1], ["Yesterday", 1]]);
    const patrol = HI.flightSummary(m, m.flights[0], T0), search = HI.flightSummary(m, m.flights[1], T0);
    assert.equal(patrol.rooms[0], "Living room");
    assert.equal(patrol.rooms.at(-1), "Room 3");
    assert.ok(patrol.rooms.includes("Room 2") && patrol.rooms.every((r, i) => r !== patrol.rooms[i - 1]), patrol.rooms.join(" → "));
    assert.equal(patrol.lost, 1, "one lost stretch");
    assert.match(patrol.what ?? patrol.summary, /patrol/i);
    assert.deepEqual(search.seen.sort(), ["a person", "the cat"].sort());
    assert.equal(search.changes, 3);
    assert.ok(!search.sim && !search.live);
    const seen = HI.lastSeenList(m);
    assert.deepEqual(seen.map((s) => s.label), ["person", "cat"]);
    const words = HI.sightingText(m, seen[1], T0);
    assert.match(words, /^The cat in Room 3( near the [a-z ]+)?, 10 min ago \(while asked to search for cat\)\.$/);
    sim.startFlight({ kind: "sim" }, T0 - 60e3);
    const simLive = HI.flightSummary(sim, sim.flights[0], T0);
    assert.ok(simLive.sim && simLive.live, "a simulator flight, flying now");
    assert.equal(HI.seenWords(simLive), null, "nothing seen yet in a live flight: no 'saw nobody'");
    // how it ended and what the camera looked at, for a finished flight
    assert.equal(patrol.outcome, "Patrolled 3 rooms.");
    const stopped = { ...m.flights[0], summary: "Stopped: stopped by you. Flew 2 m.", inspection: { measured: true, share: 0.4, rooms: [{ id: "r1", name: "Living room", share: 0.13, unmapped: 0, points: 50, unseen: null }] } };
    const st = HI.flightSummary(m, stopped, T0);
    assert.equal(st.outcome, "Stopped: stopped by you.");
    assert.match(st.looked, /13% of the Living room/);
    assert.equal(HI.seenWords(st), "no people or pets noticed in what it looked at");
    assert.equal(HI.seenWords({ ...st, measured: false }), null, "no measure of what it looked at: nothing said");
    // an older flight's trail no longer in RAM: read back from the store (trailOf), its loss counted
    const kept = m.trails.get(m.flights[0].id), old = { ...m, trails: new Map(), trailOf: async (id) => (id === m.flights[0].id ? kept : []), roomName: (id) => m.roomName(id), thing: (s) => m.thing(s) };
    assert.equal(HI.flightSummary(old, { ...m.flights[0], losses: undefined }, T0).lost, null, "not in RAM, a record from before memory kept losses: not known yet (not 0)");
    const back = await HI.flightTrail(old, m.flights[0].id);
    assert.equal(HI.flightSummary(old, m.flights[0], T0, back).lost, 1, "read back: one lost stretch");
    console.log(`      ${groups.map((g) => g.day).join(", ")}; "${patrol.length} · ${patrol.distance} · ${patrol.rooms.join(" → ")}"; "${words}"`);
  },

  async "history reads older flights' losses back one at a time, the newest few, never in the air, counts only; a take-off and a stale change in words"() {
    const flights = Array.from({ length: 40 }, (_, i) => ({ id: `f${i}`, t0: T0 - (40 - i) * 3600e3, t1: T0 - (40 - i) * 3600e3 + 120e3, rooms: [] }));
    let calls = 0, open = 0, most = 0, air = true, renders = 0;
    const memory = { flights, sightings: [], changes: [], roomName: (id) => id, thing: (x) => x.label, trails: new Map([["f39", [{ x: 0, y: 0 }, { lost: true }]]]),
      trailOf: async () => (calls++, (most = Math.max(most, ++open)), await new Promise((r) => setTimeout(r, 2)), open--, [{ x: 0, y: 0 }, { lost: true }, { lost: true }]) };
    const panel = Object.assign(Object.create(HI.HistoryPanel.prototype), { memory, losses: new Map(), reading: null, flying: () => air, renderFlights() { renders++; this.readBack(); } });
    const rows = () => [...flights].reverse().map((f) => panel.trailOf(f));
    rows();
    panel.readBack();
    assert.deepEqual([calls, panel.reading], [0, null], "nothing read back in the air (each read is the whole store, on the control loop's thread)");
    air = false;
    panel.readBack();
    await panel.reading;
    const n = HI.HISTORY.readBack, got = rows();
    assert.deepEqual([calls, most, renders], [n, 1, 1], "the newest few, one at a time, the list drawn again once");
    assert.ok(Array.isArray(got[0]) && got.slice(1, 1 + n).every((v) => v === 2) && got.slice(1 + n).every((v) => v === null), "RAM's trail, then counts, then not known");
    assert.equal(HI.flightSummary(memory, flights[38], T0, got[1]).lost, 2);
    assert.equal(panel.trailOf({ ...flights[0], losses: 3 }), 3, "a flight record's own count needs no read-back");
    // a take-off: the 3D view drops the past flight History showed and this flight's findings start again
    const v = { findings: [{ x: 1 }], flightId: "f3", setFlight(id) { this.shown = id; }, invalidate() {} };
    V3.View3D.prototype.newFlight.call(v);
    assert.deepEqual([v.findings, v.shown], [[], null]);
    // the narrow card's badge: short words of its own (not the long text cut at a colon, which held a clock time)
    const ui = { badgeLong: { textContent: "" }, badgeShort: { textContent: "" }, badge: { setAttribute() {} } };
    const view = { pose: () => null, caption() {}, holdReason: () => "", mode: () => "real", flying: () => true, ui, map: null };
    V3.View3D.prototype.badge.call(view, true, { at: Date.parse("2026-10-04T17:51:00") });
    assert.deepEqual([ui.badgeShort.textContent, /^3D scan, as of .+: it's busy/.test(ui.badgeLong.textContent)], ["3D scan", true]);
    V3.View3D.prototype.badge.call({ ...view, holdReason: () => "flying" }, false, null);
    assert.equal(ui.badgeShort.textContent, "Drawing");
    assert.match(CC.effectText({ status: "suspected", kind: "obstacle", stale: true }), /looked at it for 10 s and didn't see it: no longer blocking the way/);
  },

  async "change card: Claude's verdict, what the change does to flying and who decided, in words"() {
    const { m, box, door, ghost, setNow } = await filledMemory();
    assert.equal(CC.verdictText(box).text, "Claude hasn't looked at it.");
    assert.equal(CC.verdictText(box, { vision: "off" }).text, "Claude's picture checks are off (Settings → Brain).");
    m.annotateChange(box.id, { claude: { status: "unconfirmed", reason: "waiting for your OK to Claude's picture checks", waiting: true } });
    assert.match(CC.verdictText(box).text, /waiting for your OK/);
    m.annotateChange(box.id, { claude: { status: "answered", verdict: "real-change", kind: "obstacle", what: "cardboard box", confidence: 0.86, why: "a brown box on the floor that the scan doesn't have", suggest: "confirmed", small: false } });
    const v = CC.verdictText(box);
    assert.equal(v.tone, "warn");
    assert.equal(v.text, "Claude: a real change (cardboard box), 86% sure. A brown box on the floor that the scan doesn't have. It suggests confirming it.".replace("A brown", "a brown"));
    m.annotateChange(ghost.id, { claude: { status: "answered", verdict: "no-change", kind: null, what: "", confidence: 0.93, why: "the same wall, darker", suggest: "dismissed", small: true } });
    assert.match(CC.verdictText(ghost).text, /^Claude: nothing has changed there, 93% sure\. the same wall, darker\. It suggests dismissing it\. The pictures were small/);
    assert.equal(CC.effectText(box), "Until someone decides, the drone flies around it as if it were there.");
    assert.equal(CC.effectText(door), "Until someone decides, the drone won't fly through that doorway.");
    m.passing(ghost.id, { note: "Claude: a dog" });
    assert.equal(CC.verdictText(ghost).text, "Claude thinks a person or pet: kept as an obstacle while in view.");
    setNow(T0);
    m.resolveChange(box.id, "confirmed", "yes", { by: "user" });
    assert.equal(CC.effectText(box), "It's on the map as a no-fly spot, and in the 3D map.");
    assert.equal(CC.effectText(box, { world: "sim" }), "In the simulator it stays on the map as an obstacle; your house's map isn't changed.");
    assert.equal(CC.decidedText(box, T0 + 3 * 60e3), "Confirmed by you 3 min ago.");
    m.resolveChange(door.id, "dismissed", "no", { by: "user" });
    assert.match(CC.effectText(door), /won't raise this doorway again/);
    m.resolveChange(ghost.id, "dismissed", "dog", { by: "user", transient: true });
    assert.equal(CC.effectText(ghost), "It was something passing: nothing changes on the map.");
    assert.match(CC.decidedText({ status: "suspected", n: 3, last: T0 - 120e3 }, T0), /^Seen 3 times, last 2 min ago\.$/);
    // Ask Claude says what it sends, to whom and about what it costs (on the card, not only in a tooltip)
    assert.equal(CC.askText(0.012), "Ask Claude sends these 2 pictures to Anthropic's Claude API with your API key (about $0.01); Claude only advises.");
    assert.equal(CC.centsText(0.004), "less than $0.01");
    assert.equal(CC.plainWords(new Error("twin worker failed"), "the check stopped"), "the check stopped", "a program's words aren't shown");
    assert.equal(CC.plainWords(new Error("only 3 views: turn the drone slowly."), "x"), "only 3 views: turn the drone slowly");
  },

  "survey: pre-ticks (new names at ≥ 0.6 the user didn't set and no other room has, new things at ≥ 0.5, one per name and room) and Apply's items (source claude, the user's edits) match applySurvey"() {
    const h0 = structuredClone(house);
    h0.rooms[1].nameSource = "user";
    h0.rooms[1].name = "Den";
    const [x1, y1] = centre("r1"), [x3, y3] = centre("r3");
    const res = {
      ok: true, cost: { cost: 0.31, estimate: 0.35, requests: 3, model: "claude-opus-5" }, views: [],
      rooms: [{ id: "r1", name: "Living room", suggestedName: "Lounge", kind: "living room", confidence: 0.9 }, { id: "r2", name: "Den", suggestedName: "Office", kind: "office", confidence: 0.95 },
        { id: "r3", name: "Room 3", suggestedName: "Bedroom", kind: "bedroom", confidence: 0.5 }],
      landmarks: [{ name: "piano", room: "r1", x: x1, y: y1, z: 0.4, r: 0.6, confidence: 0.9, views: [] }, { name: "lamp", room: "r1", x: x1 + 1, y: y1, z: 1.2, r: 0.2, confidence: 0.4, views: [] },
        { name: "tv", room: "r1", x: x1 - 1, y: y1, z: 1, r: 0.4, confidence: 0.9, views: [], known: "tv" }],
      hazards: [{ kind: "ceiling-fan", room: "r3", x: x3, y: y3, z: 2.3, r: 0.5, zMin: 2.2, zMax: 2.4, why: "a fan", confidence: 0.8, views: [] },
        { kind: "glass", room: "r1", x: x1, y: y1 + 1, z: 1, r: 0.4, zMin: 0.5, zMax: 1.8, why: "a glass door", confidence: 0.45, views: [] }],
    };
    const pre = SV.pretick(res, h0);
    assert.deepEqual([...pre.rooms], ["r1"], "r2 the user named; r3 under 0.6");
    assert.deepEqual([...pre.landmarks], [0], "the lamp under 0.5; the tv already known");
    assert.deepEqual([...pre.hazards], [0]);
    const twice = { ...res, rooms: [res.rooms[0], res.rooms[1], { ...res.rooms[2], suggestedName: "lounge", confidence: 0.9 }, { id: "r4", name: "Hall", suggestedName: "Living Room", confidence: 0.9 }],
      landmarks: [...res.landmarks, { ...res.landmarks[0], x: x1 + 0.8, confidence: 0.7 }, { ...res.landmarks[0], room: "r3", confidence: 0.6 }] };
    // a name the map already has in that room isn't ticked (a third "chair" makes "go to the chair" no clearer)
    const withPiano = { ...h0, landmarks: [...(h0.landmarks ?? []), { name: "Piano", room: "r1", x: x1, y: y1, source: "user" }] };
    assert.deepEqual([...SV.pretick(res, withPiano, map).landmarks], [], "the map has a piano in the living room");
    assert.ok(SV.onMap(withPiano, map, "piano", "r1") && !SV.onMap(withPiano, map, "piano", "r3"));
    assert.deepEqual([...SV.pretick({ ...res, landmarks: [{ ...res.landmarks[0], name: "chair" }] }, h0, map).landmarks], [], "the scan's own chairs in the living room");
    const pre2 = SV.pretick(twice, h0);
    assert.deepEqual([...pre2.rooms], [], "two rooms offered one name, and a room's name another room has, are not ticked");
    assert.deepEqual([...pre2.landmarks].sort(), [0, 4], "of two pianos in one room only the surer; one in another room too");
    assert.ok(SV.takenName(twice, h0, twice.rooms[3]) && !SV.takenName(res, h0, res.rooms[0]));
    const picks = { rooms: new Map([["r1", "Family room"], ["r2", "Office"]]), landmarks: new Map([[0, "big sofa"]]), hazards: new Set([0, 1]) };
    const ch = SV.surveyChanges(h0, res, picks, { map });
    assert.deepEqual(ch.rooms, [{ id: "r1", name: "Family room", kind: "living room", nameSource: "claude" }], "the user's room name stays");
    assert.deepEqual(ch.landmarks.map((l) => [l.name, l.source]), [["big sofa", "claude"]]);
    assert.ok(ch.keepouts.every((k) => k.source === "claude") && ch.keepouts[0].kind === "fan" && ch.keepouts[1].kind === "glass");
    assert.deepEqual(ch.keepouts[0], hazardKeepout(res.hazards[0], map.floorAt(x3, y3)));
    const a = structuredClone(h0), b = structuredClone(h0);
    const done = ch.apply(a);
    applySurvey(b, ch.res, ch.accept, { map });
    assert.deepEqual(a, b, "apply(house) is applySurvey with the user's names");
    assert.deepEqual(done, { rooms: 1, landmarks: 1, keepouts: 2 });
    assert.equal(a.rooms[0].name, "Family room");
    assert.equal(a.rooms[1].name, "Den");
    assert.ok(a.landmarks.some((l) => l.name === "big sofa" && l.source === "claude"));
  },

  "coverage: rows worst first in words, and what flights added (flight-only free space, what the camera saw)"() {
    const v = V.VoxelMap.load(vox.serialize()), report = coverageReport({ house, map, vox: v });
    const rows = CO.coverageRows(report);
    assert.equal(rows.length, house.rooms.length);
    assert.ok(rows.every((r, i) => i === 0 || r.unknown <= rows[i - 1].unknown), "worst first");
    for (const r of rows) assert.ok(r.known + r.unknown === 100 && /(m³|litres)$/.test(r.unknownM3) && r.words.length > 5, JSON.stringify(r));
    // a room with gaps is never just "well covered"; known is a share of the open space, as the score is
    const fake = { rooms: [{ id: "a", name: "A", knownFreePct: 84, unknownPct: 8, occupiedPct: 8, unknownM3: 2.7, hiddenM3: 0, flyableM3: 20, gaps: [{}, {}] }, { id: "b", name: "B", knownFreePct: 90, unknownPct: 2, gaps: [], unknownM3: 0.1, hiddenM3: 0, flyableM3: 9 }] };
    const fr = CO.coverageRows(fake);
    assert.deepEqual(fr.map((r) => [r.known, r.unknown, r.words]), [[91, 9, "Well covered: 2 gaps worth a rescan (optional)"], [98, 2, "Well covered"]]);
    assert.match(CO.scoreWords(report.score), /^(Good|Mostly known|Patchy|Poor)/);
    const todo = CO.rescans(report.suggestions);
    assert.ok(todo.length <= report.suggestions.length && new Set(todo.map((t) => t.text)).size === todo.length, "one item per thing to do");
    assert.equal(todo.reduce((a, t) => a + t.gaps, 0), report.suggestions.reduce((a, s) => a + s.gaps, 0), "every gap still counted");
    assert.ok(todo.every((t, i) => i === 0 || t.gaps <= todo[i - 1].gaps));
    const before = CO.flightAdditions(v, map);
    assert.ok(!before.flown && before.flightM3 === 0, "nothing flown yet");
    // a flight's live depth frees a 30 cm cube of unknown space in the living room and the camera looks around from its middle
    const [x, y] = centre("r1"), fl = map.floorAt(x, y);
    let freed = 0;
    for (let i = 0; i < v.n && freed < 216; i++) {
      const [cx, cy, cz] = v.center(i);
      if (v.st[i] === V.UNKNOWN && map.roomAt(cx, cy)?.id === "r1" && cz - (map.floorAt(cx, cy) ?? 0) > 0.4 && cz - (map.floorAt(cx, cy) ?? 0) < 1.5) (v.set(i, -40, V.FLAG.FLIGHT), freed++);
    }
    v.markSeen({ x, y, z: fl + 1, yaw: 0, sigma: 0.03, status: "ok" }, DRONE_LENS, T0, { width: 320, height: 240 });
    v.markSeen({ x, y, z: fl + 1, yaw: Math.PI, sigma: 0.03, status: "ok" }, DRONE_LENS, T0 + 1000, { width: 320, height: 240 });
    const after = CO.flightAdditions(v, map), r1 = after.rooms.find((r) => r.id === "r1");
    near(r1.flightM3, freed * 0.05 ** 3, 1e-3, "flight-only free space");
    assert.ok(r1.seenPct > 5 && r1.lastSeen === T0 + 1000, `seen ${r1.seenPct}%, last ${r1.lastSeen}`);
    assert.ok(after.flown);
    console.log(`      score ${report.score}%: ${rows.map((r) => `${r.name} ${r.known}% known, ${r.words}`).join("; ")}; a flight added ${r1.flightM3} m³, the camera saw ${r1.seenPct}% of the living room`);
  },

  "calibration: the before/after shift (none for the same lens, the edges move most for a new focal length) and its words"() {
    const std = droneLens(null, 20), same = CA.lensShift(std, std);
    near(same.max, 0, 1e-6, "same lens");
    const calib = { model: "equidistant", fx: 255, fy: 255, cx: 322, cy: 238, k: [0.02, -0.01, 0, 0], uptiltDeg: 22.4, width: 640, height: 480, rms: 0.62, source: "pad", verified: { at: "2026-10-04T12:00:00Z" } };
    const s = CA.lensShift(std, droneLens(calib, 20));
    const mid = s.points.find((p) => Math.abs(p.u - 320.5) < 1 && Math.abs(p.v - 240.5) < 1), corner = s.points.find((p) => p.u < 1 && p.v < 1);
    assert.ok(s.max > 5 && corner.shift > mid.shift, `max ${s.max.toFixed(1)} px, middle ${mid.shift.toFixed(1)}, corner ${corner.shift.toFixed(1)}`);
    const words = CA.calibSummary(calib, (c) => droneLens(c, 20));
    assert.equal(words.quality, "good");
    assert.match(words.text, /^Calibrated .*: 0\.62 px error \(a good fit\), \d+° × \d+° field of view, camera tilted up 22\.4°, checked on the pad\.$/);
    assert.match(CA.calibSummary(null).text, /^Not calibrated/);
    assert.equal(CA.calibSummary({ ...calib, rms: 3.1 }, (c) => droneLens(c, 20)).quality, "poor");
    console.log(`      a new focal length moves the picture's edges by up to ${s.max.toFixed(0)} px (middle ${mid.shift.toFixed(1)} px); "${words.text}"`);
  },

  "recordings: the files' checks (pad, picture, this house, landed) and a replay's report (loc-check's numbers or nav/replay.js's lines) as pass/fail words"() {
    const tel = (t, extra = {}) => ({ t, fm: "STAB", sticks: { thr: 0 }, est: { heading: 0 }, ...extra });
    const real = { meta: { house: house.id, app: { mode: "real" }, lens: {} }, index: [{ t: 0 }, { t: 30000 }], telemetry: [tel(0), tel(30000)], commands: [] };
    const c1 = RC.recordingChecks(real, { houseId: house.id }), byId = (cs, id) => cs.find((c) => c.id === id);
    assert.ok(!byId(c1, "pad").ok && byId(c1, "pad").level === "block", "no pad button: the truth is missing");
    assert.ok(!byId(c1, "picture").ok && byId(c1, "picture").level === "warn");
    assert.ok(byId(c1, "house").ok && byId(c1, "landed").ok);
    const c2 = RC.recordingChecks({ ...real, commands: [{ tool: "pad", args: { x: 0, y: 0, yaw: 0 } }, { tool: "picture", args: { sx: 240, sy: 0, sw: 1440, sh: 1080 } }],
      telemetry: [tel(0), tel(30000, { fm: "ANGLE", sticks: { thr: 0.5 } })] }, { houseId: house.id });
    assert.ok(byId(c2, "pad").ok && byId(c2, "picture").ok && /1440×1080/.test(byId(c2, "picture").text));
    assert.ok(!byId(c2, "landed").ok, "stopped in the air");
    assert.ok(!byId(RC.recordingChecks(real, { houseId: "other" }), "house").ok);
    // loc-check's replay numbers from a real run (the synthetic recording, out/c-loc-result.json)
    const out = { region: { sx: 0, sy: 0, sw: 640, sh: 480 }, pad: { x: 0.007, y: 0.161 }, groundFixVsPad: 0.007, endVsPad: 0.185, endStatus: "ok", gated: 0, conflicts: 0, anchored: 0, delay: null,
      seconds: 62.4, flyingSeconds: 59.4, units: 1870, decoded: 1870, start: "pad command", foundAfter: 1, truth: true, error: { median: 0.041, p95: 0.124, max: 0.172, n: 198 },
      status: { ok: 198 }, fixRate: 5.03, accepted: 299 };
    const r1 = RC.replayChecks(out), v1 = RC.verdict(r1);
    assert.ok(v1.ok, v1.text);
    assert.match(r1.find((c) => c.id === "truth").text, /^Off from the simulator's true position by 4 cm typically, 12 cm at worst/);
    const bad = RC.replayChecks({ ...out, truth: false, endVsPad: 0.185, fixRate: 1.2, status: { ok: 150, lost: 48 }, conflicts: 2, anchored: 1, delay: { n: 40, offset: -60, videoDelay: 160, rmsDeg: 1.1 } }, { videoDelay: 100 });
    const v2 = RC.verdict(bad);
    assert.ok(!v2.ok && /^Not good enough yet: 1\.2 camera position fixes a second/.test(v2.text), v2.text);
    assert.match(bad.find((c) => c.id === "pad-end").text, /19 cm from it \(needs under 10 cm\)/);
    assert.match(bad.find((c) => c.id === "delay").text, /160 ms late, not 100 ms/);
    assert.match(bad.find((c) => c.id === "lost").text, /^Lost its position for about 14 s/);
    // nav/replay.js's report: lines with ok true, false or null (information), or problems before any replay
    const vr = RC.reportChecks({ ok: false, title: "t", lines: [{ id: "video", ok: true, text: "Video fine." }, { id: "start", ok: null, text: "Started elsewhere." }, { id: "fixes", ok: false, text: "Too few fixes." }] });
    assert.deepEqual(vr.map((c) => [c.ok, c.level, !!c.info]), [[true, "info", false], [true, "info", true], [false, "block", false]]);
    assert.ok(!RC.verdict(vr).ok);
    // the verdict agrees with its lines: no pad press and a passing replay is not a pass; a check that didn't finish isn't either
    const pass = { ok: true, title: "The camera found the drone's position all through this flight.", lines: [{ id: "video", ok: true, text: "Video fine." }, { id: "fixes", ok: true, text: "5 a second." }] };
    const noPad = { id: "r", state: "done", error: null, sim: false, replay: pass, title: pass.title, checks: [...c1.filter((c) => !c.ok).map((c) => ({ ...c, file: true })), ...RC.reportChecks(pass)] };
    const nv = RC.runVerdict(noPad);
    assert.ok(!nv.ok && nv.text !== pass.title && /home pad is pressed|wasn't pressed/.test(nv.text), nv.text);
    assert.ok(!RC.canCalibrateFrom(noPad), "no pad press: nothing to calibrate from");
    const good = { ...noPad, checks: RC.reportChecks(pass) };
    assert.deepEqual(RC.runVerdict(good), { ok: true, text: pass.title });
    assert.ok(RC.canCalibrateFrom(good) && !RC.canCalibrateFrom({ ...good, sim: true }), "a real flight replayed, files fine: calibrate offered (never for the simulator)");
    const warned = { ...good, checks: [{ id: "landed", ok: false, level: "warn", file: true, text: "Stopped in the air." }, ...RC.reportChecks(pass)] };
    assert.match(RC.runVerdict(warned).text, /with 1 thing to fix/);
    const failed = { ...good, replay: null, error: "the replay stopped before the end, so this says nothing about localization yet. Reload the page and try again." };
    assert.ok(!RC.runVerdict(failed).ok && /^Not checked: /.test(RC.runVerdict(failed).text) && !RC.canCalibrateFrom(failed));
    // other modules' "the pad button" as the button is labelled
    assert.equal(RC.padWords("this recording has no time on the pad after the pad button"), 'this recording has no time on the pad after pressing "The drone is on its home pad"');
    assert.equal(RC.padWords("set the drone on the home pad and press the pad button first"), 'set the drone on the home pad and press "The drone is on its home pad" first');
    const vp = RC.reportChecks({ ok: false, problems: ["it has no video (the goggles weren't streaming to the Mac)"], lines: [] });
    assert.deepEqual(vp.map((c) => [c.ok, c.level, c.text]), [[false, "block", "It has no video (the goggles weren't streaming to the Mac)."]]);
    console.log(`      synthetic run: "${v1.text}"; a poor one: "${v2.text}"`);
  },

  "no user-facing text in the panels names a file path or a developer tool"() {
    for (const f of ["view3d", "history", "changecard", "survey", "coverage", "calib", "recheck"]) {
      const src = fs.readFileSync(path.join(ROOT, "app", "js", "ui", `${f}.js`), "utf8");
      const strings = [...src.matchAll(/(["`])((?:(?!\1)[^\\]|\\.)*)\1/g)].map((m) => m[2]).filter((s) => /\s/.test(s) && s.length > 20);
      for (const s of strings) assert.ok(!/\/Users\/|\.mjs|\.json\b|localhost|console|undefined|NaN/.test(s.replace(/calib\.json/g, "")), `${f}.js: "${s.slice(0, 80)}"`);
    }
  },
};

let failed = 0;
const t0 = performance.now();
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name}\n      ${e.message.split("\n").slice(0, 6).join("\n      ")}\n      ${e.stack?.split("\n").find((l) => l.includes("test-panels.mjs:"))?.trim() ?? ""}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed (${((performance.now() - t0) / 1000).toFixed(1)} s)`);
process.exit(failed ? 1 : 0);
