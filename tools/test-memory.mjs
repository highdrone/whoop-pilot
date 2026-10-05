// The house's flight memory (app/js/memory/memory.js) on the capture fixture, with a stand-in IndexedDB (FakeIDB below:
// stores, transactions, structured clones, kept across reopening): flights and their 2 Hz trails, sightings and their
// merging, lastSeen in people's words, recall's plain-text answers (deterministic, from the records only), changes from
// suspected to confirmed (the house's keep-outs and doors, the map) or dismissed (and not raised again at that spot unless
// it changes again), suspected ones on the map as temporary obstacles, toggles (closed, open, closed again), what Claude
// may resolve on its own (a weak suspicion too, never a well-evidenced blocker), freeing only voxels something marked
// occupied, people and pets passing, the simulator's own memory, one live memory per house, honest positions (lost,
// unsure), door questions, retention, and alerts.js noting what it logs.
// Usage: cd tools && node test-memory.mjs
process.env.TZ = "UTC";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";

const I = await import("../app/js/house/import.js");
const { plan } = await import("../app/js/house/planner.js");
const { HomeMap } = await import("../app/js/house/homemap.js");
const { HouseMemory, MEMORY, labelOf, parseSince, when, missionText, clip } = await import("../app/js/memory/memory.js");
const { Alerts } = await import("../app/js/alerts.js");
const { VoxelMap, FLAG, UNKNOWN, FREE, OCCUPIED } = await import("../app/js/house/voxels.js");

const FIXTURE = path.join(import.meta.dirname, "fixtures", "house");
const fsSource = (dir) => ({
  name: path.basename(dir),
  read: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readFileSync(path.join(dir, p)) : null),
  list: (p) => (fs.existsSync(path.join(dir, p)) ? fs.readdirSync(path.join(dir, p)) : []),
});
const fixture = () => I.importCapture(fsSource(FIXTURE));
const { house, map } = await fixture();
const name = (id) => house.rooms.find((r) => r.id === id).name;

// IndexedDB, as much of it as memory.js uses: open with an upgrade, object stores with a keyPath, transactions over
// several stores, put / delete / getAll (all, or a key range) requests firing onsuccess, oncomplete after them. Values
// are structured clones; `read` counts the records handed out.
globalThis.IDBKeyRange ??= { bound: (lower, upper) => ({ lower, upper, includes: (k) => k >= lower && k <= upper }) };
export class FakeIDB {
  dbs = new Map();
  open(name, version) {
    const rq = {};
    setTimeout(() => {
      let db = this.dbs.get(name);
      const upgrade = !db || version > db.version;
      if (!db) this.dbs.set(name, (db = new FakeDB()));
      rq.result = db;
      if (upgrade) (db.version = version), rq.onupgradeneeded?.();
      rq.onsuccess?.();
    });
    return rq;
  }
}
class FakeDB {
  stores = new Map();
  read = 0;
  objectStoreNames = { contains: (n) => this.stores.has(n) };
  createObjectStore(name, { keyPath }) {
    this.stores.set(name, { keyPath, data: new Map() });
  }
  transaction(names, mode = "readonly") {
    return new FakeTx(this, [].concat(names), mode);
  }
}
class FakeTx {
  constructor(db, names, mode) {
    Object.assign(this, { db, names, mode, pending: 0, ended: false });
    setTimeout(() => this.settle());
  }
  objectStore(n) {
    if (!this.names.includes(n)) throw new Error(`store ${n} not in this transaction`);
    const s = this.db.stores.get(n), rw = () => { if (this.mode !== "readwrite") throw new Error("read-only transaction"); };
    return {
      put: (v) => this.req(() => (rw(), s.data.set(v[s.keyPath], structuredClone(v)), v[s.keyPath])),
      delete: (k) => this.req(() => (rw(), s.data.delete(k), undefined)),
      getAll: (range) => this.req(() => [...s.data].filter(([k]) => !range || range.includes(k)).map(([, v]) => (this.db.read++, structuredClone(v)))),
      get: (k) => this.req(() => structuredClone(s.data.get(k))),
    };
  }
  req(fn) {
    const rq = {};
    this.pending++;
    queueMicrotask(() => {
      try {
        rq.result = fn();
        rq.onsuccess?.();
      } catch (e) {
        rq.error = this.error = e;
        rq.onerror?.();
      }
      this.pending--;
      this.settle();
    });
    return rq;
  }
  settle() {
    if (this.pending || this.ended) return;
    this.ended = true;
    setTimeout(() => (this.error ? this.onerror?.() : this.oncomplete?.()));
  }
}

const H = 36e5, DAY = 24 * H, T0 = Date.UTC(2026, 9, 3, 14, 0, 0); // Saturday 3 Oct 2026, 14:00 UTC
const clock = (t = T0) => {
  const c = { t, now: () => c.t, at: (t) => ((c.t = t), c) };
  return c;
};
const jpeg = (n = 4) => Buffer.from([0xff, 0xd8, ...Array(n).fill(7), 0xff, 0xd9]).toString("base64");

// Fly a path: samples every 100 ms at 0.3 m/s, z 1 m above the floor; returns the time at the end.
function fly(m, pathPts, c, { lostAt = null } = {}) {
  const pts = [];
  for (let i = 1; i < pathPts.length; i++) {
    const [a, b] = [pathPts[i - 1], pathPts[i]], n = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / 0.03));
    for (let k = 0; k < n; k++) pts.push([a[0] + ((b[0] - a[0]) * k) / n, a[1] + ((b[1] - a[1]) * k) / n]);
  }
  pts.push(pathPts.at(-1));
  pts.forEach(([x, y], i) => {
    c.t += 100;
    const lost = lostAt && i >= lostAt[0] && i < lostAt[1];
    m.samplePose(lost ? { status: "lost" } : { x, y, z: (map.floorAt(x, y) ?? 0) + 1, yaw: 0, sigma: 0.08, status: "ok", room: map.roomAt(x, y)?.id ?? null }, c.t);
  });
  return c.t;
}

const tests = {
  async "a flight keeps a 2 Hz trail with σ and the rooms in order; it is all there again after reopening"() {
    const idb = new FakeIDB(), c = clock();
    const m = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now });
    assert.ok(m.stored && !(await HouseMemory.open(house.id, { indexedDB: null })).stored, "stored, or only until the page closes (the UI says so)");
    m.setHouse({ house, map });
    const route = plan(map, [house.home.x, house.home.y], [0.86, 3.46], { alt: 1.0 });
    assert.ok(route.ok, route.reason);
    const events = [];
    m.on("flight", (f) => events.push(f.t1 == null ? "start" : "end"));
    const f = m.startFlight({ kind: "patrol", mission: { kind: "patrol" } });
    const t1 = fly(m, route.path, c, { lostAt: [20, 40] });
    m.endFlight("Patrolled and landed.", t1);
    const trail = m.trail({ flightId: f.id }), secs = (t1 - T0) / 1000;
    assert.ok(Math.abs(trail.length - (secs * 2 - 3)) <= 2, `${trail.length} samples in ${secs} s (2 Hz, one marker for 2 s lost)`);
    assert.equal(trail.filter((s) => s.lost).length, 1, "a lost stretch is one marker");
    assert.ok(trail.every((s) => s.lost || (Number.isFinite(s.sigma) && s.room)), "σ and room on every sample");
    const rooms = [...new Set(route.path.map(([x, y]) => map.roomAt(x, y)?.id).filter(Boolean))];
    assert.deepEqual(f.rooms, rooms, "rooms in the order flown");
    assert.ok(Math.abs(f.distance - (route.length - 0.6)) < 0.25, `distance ${f.distance} vs path ${route.length.toFixed(2)} minus the 0.6 m lost stretch`);
    assert.deepEqual(events, ["start", "end"]);
    assert.equal(f.mission, "patrol");
    assert.equal(await HouseMemory.open(house.id, { indexedDB: idb, now: c.now }), m, "the memory already open for this house");
    await m.close();
    const again = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now });
    assert.notEqual(again, m, "closed: read again from storage");
    assert.equal(again.flights.length, 1);
    assert.deepEqual(again.trail({ flightId: f.id }), trail, "trail stored in chunks and read back");
    assert.deepEqual(again.flights[0].rooms, f.rooms);
    const other = await HouseMemory.open("another-house", { indexedDB: idb, now: c.now });
    assert.equal(other.flights.length, 0, "one house's memory only");
  },

  async "a long flight's report keeps whole sentences: every mission's words up to the cap, how it ended, never a sentence cut mid-way"() {
    const c = clock(T0), m = await HouseMemory.open("long-report-house", { indexedDB: null, now: c.now });
    m.setHouse({ house, map });
    m.startFlight({ kind: "patrol", mission: { kind: "patrol" } });
    fly(m, [[0.0, 0.6], [0.9, 2.2]], c);
    const words = ["A person in Room 2 is in the way and I can't get around it; I waited 20 s.", "I couldn't look into Room 3: a person in Room 2 is in the way and I can't get around it; I waited 20 s.",
      "Found a person in the Living room near the chair.", "The battery is too low for that (about 114 s of flying left; that leg and the way home need about 142 s at the 0.10 m/s I can fly now). Before that I found the dog in the Living room. Landed on the home pad."];
    const f = m.endFlight(`${words.join(" ")} (180 s in the air)`, c.t + 1000);
    assert.ok(f.summary.endsWith("(180 s in the air)") && f.report.includes("Landed on the home pad."), f.report);
    const long = `${words.join(" ")} `.repeat(3);
    m.startFlight({ kind: "patrol", mission: { kind: "patrol" } });
    fly(m, [[0.9, 2.2], [0.0, 0.6]], c);
    const g = m.endFlight(long, c.t + 1000);
    assert.ok(g.summary.length <= 800 && /\.$/.test(g.summary) && long.startsWith(g.summary), g.summary.slice(-80));
    assert.doesNotMatch(g.report, /[;(,]\.|left;\./, "no sentence cut mid-way");
    assert.equal(clip("word ".repeat(100), 50).slice(-1), "…");
  },

  async "a page closed mid-flight leaves a flight that ends at its last sample"() {
    const idb = new FakeIDB(), c = clock();
    const m = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now });
    m.setHouse({ house, map });
    m.startFlight({ kind: "manual" });
    const end = fly(m, [[0.5, 3], [1.5, 3]], c);
    await m.close(); // the page closes: what was stored stays, the flight is still open in storage
    const again = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now });
    assert.ok(again.flights[0].t1 <= end && end - again.flights[0].t1 <= MEMORY.chunk * MEMORY.trailMs, "ended at the last stored sample (trails are stored every 10 s)");
    assert.match(again.flights[0].summary, /closed during this flight/);
  },

  async "sightings: one per label and spot (or track) within 15 s; lastSeen understands what people call them"() {
    const c = clock(), m = await HouseMemory.open(house.id, { indexedDB: null, now: c.now });
    m.setHouse({ house, map });
    m.startFlight({ kind: "searchFor", mission: { kind: "searchFor", target: "cat" } });
    const a = m.addSighting({ label: "cat", x: 1.9, y: 3.0, z: 0.3, score: 0.6, snapshot: jpeg(), confirmed: true, t: c.t });
    const b = m.addSighting({ label: "cat", x: 2.1, y: 3.2, z: 0.3, score: 0.8, snapshot: jpeg(8), t: c.t + 2000 });
    const d = m.addSighting({ label: "cat", trackId: null, x: 2.0, y: 3.1, score: 0.7, t: c.t + 4000 });
    assert.equal(a, b);
    assert.equal(a, d);
    assert.equal(a.n, 3);
    assert.equal(a.score, 0.8, "best score");
    assert.equal(a.snapshot.size, 12, "the best-scored snapshot, as a JPEG blob");
    assert.equal(a.snapshot.type, "image/jpeg");
    assert.equal(a.room, map.roomAt(2, 3.1).id);
    const tracked = [m.addSighting({ label: "person", trackId: 7, x: 0.5, y: 1, t: c.t + 5000 }), m.addSighting({ label: "person", trackId: 7, x: 2.5, y: 1, t: c.t + 6000 })];
    assert.equal(tracked[0], tracked[1], "the same track, though it moved 2 m");
    const later = m.addSighting({ label: "cat", x: -1.2, y: 3.2, score: 0.9, t: c.t + 60000 });
    assert.notEqual(later, a, "a minute later elsewhere: another sighting");
    assert.equal(m.addSighting({ label: "cat", x: 0, y: 0, snapshot: "A".repeat(400000), t: c.t + 70000 }).snapshot, null, "a huge snapshot isn't kept");
    assert.equal(m.lastSeen("the kitty").id, m.sightings.at(-1).id);
    assert.equal(m.lastSeen("Someone?").label, "person");
    assert.equal(m.lastSeen("my keys"), null);
    for (const [w, l] of [["kittens", "cat"], ["the puppy", "dog"], ["anybody", "person"], ["glasses", "glasse"], ["boxes", "boxe"], ["keys", "key"], ["glass", "glass"]]) assert.equal(labelOf(w), l, w);
    assert.equal(a.mission, "search for cat");
  },

  async "recall answers in plain words from the records only, the same way every time"() {
    const c = clock(T0 - DAY - 3 * H), m = await HouseMemory.open(house.id, { indexedDB: null, now: c.now });
    m.setHouse({ house, map });
    // Yesterday at 11:00: a patrol, the cat in Room 3 near the stairs.
    m.startFlight({ kind: "patrol", mission: { kind: "patrol" } });
    fly(m, [[0.0, 0.6], [-0.9, 1.1], [-1.2, 3.1]], c);
    m.addSighting({ label: "cat", x: -1.3, y: 3.6, z: 0.1, score: 0.7, snapshot: jpeg(), t: c.t });
    m.endFlight("Patrolled Room 2 and Room 3.", c.t + 1000);
    // Today at 13:50: to the sofa, the cat beside it twice, a person.
    c.at(T0 - 10 * 60e3);
    m.startFlight({ kind: "goTo", mission: { kind: "goTo", target: "sofa" } });
    fly(m, [[0.0, 0.6], [0.9, 2.2], [1.2, 3.4]], c);
    m.addSighting({ label: "cat", x: 1.8, y: 3.0, z: 0.4, score: 0.8, snapshot: jpeg(6), t: c.t });
    m.addSighting({ label: "cat", x: 1.9, y: 3.1, z: 0.4, score: 0.9, t: c.t + 1000 });
    m.note({ kind: "finding", label: "cat", x: 1.9, y: 3.0, text: "The cat in the Living room.", confirmation: "confirmed", t: c.t + 1500 });
    m.addSighting({ label: "person", x: 2.5, y: 1.0, z: 1.0, score: 0.9, t: c.t + 3000 });
    m.note({ kind: "mission", text: "Arrived at the sofa." }, c.t + 4000);
    m.endFlight("Arrived at the sofa.", c.t + 5000);
    c.at(T0);
    const cat = m.recall("Where did you last see the cat?");
    const lr = name(map.roomAt(1.9, 3.0).id), r3 = name(map.roomAt(-1.3, 3.6).id);
    assert.match(cat, new RegExp(`^I last saw the cat in the ${lr} near the sofa, \\d+ min ago \\(while asked to go to sofa, seen 2 times\\), confirmed by Claude\\. Before that: in ${r3}`));
    assert.match(cat, /yesterday at 11:0\d\. 2 sightings in all\.$/);
    assert.equal(m.recall("where did you last see the cat"), cat, "deterministic");
    assert.equal(m.answer("where's the cat").sighting.snapshot.size, 10, "the sighting (and its picture) for Claude");
    assert.match(m.recall("have you seen my keys?"), /^I haven't seen "my keys" on any flight I remember \(2 flights since yesterday at 11:00\)\.$/);
    assert.match(m.recall("have you seen the sofa"), /^The map has the sofa in the Living room; no flight I remember noted it \(2 flights since yesterday at 11:00\)\.$/);
    assert.match(m.recall("when did you last see someone"), /^I last saw a person in /);
    assert.match(m.recall(`When were you last in ${r3}?`), new RegExp(`^I was last in ${r3} yesterday at 11:\\d\\d, (briefly|for about \\d+ s), on the flight yesterday at 11:00; 1 of 2 flights went there\\. The last thing I saw there: the cat`));
    const been = m.recall("where have you been?");
    assert.match(been, /^The flight \d+ min ago: \d+ s, \d+\.\d m, .+ → .+; Arrived at the sofa; saw the cat in .+, a person in .+\. The flight yesterday at 11:00: /);
    assert.match(m.recall("how many flights so far?"), /^I remember 2 flights since yesterday at 11:00: \d+ s in the air, \d+ m flown, most of it in .+; 3 sightings \(cat 2, person 1\); 0 changes/);
    assert.match(m.recall("anything else?"), /^The flight \d+ min ago: .* Last seen: a person in .*; the cat in /);
    for (const q of ["where did you last see the cat", "where have you been", "how many flights"]) assert.ok(!/\(-?\d+\.\d+, -?\d+\.\d+\)/.test(m.recall(q)), "no coordinates");
    const tl = m.timeline({ limit: 20 });
    assert.deepEqual(tl.slice(0, 3).map((e) => e.kind), ["landed", "mission", "sighting"]);
    assert.ok(tl.every((e, i) => !i || tl[i - 1].t >= e.t), "newest first");
  },

  async "a confirmed obstacle becomes a keep-out (source change) the map honours; gone takes it away"() {
    const { house, map } = await fixture();
    const c = clock(), m = await HouseMemory.open(house.id, { indexedDB: null, now: c.now }), saved = [], changed = [];
    m.setHouse({ house, map, save: (h) => saved.push(h) });
    m.on("house", (e) => changed.push(e));
    const [x, y] = [0.9, 2.4], floor = map.floorAt(x, y);
    assert.ok(map.free(x, y, floor + 1.0), "free before");
    const seen = [];
    m.on("change", (r) => seen.push(r.status));
    const ch = m.addChange({ id: "change-a", kind: "obstacle", x, y, z: floor + 0.4, size: 0.5, zMin: floor, zMax: floor + 0.8, evidence: { live: jpeg(), expected: jpeg() } });
    assert.equal(ch.id, "change-a", "the detector's id, so it can drop its temporary obstacle");
    assert.equal(ch.status, "suspected");
    assert.equal(ch.evidence.live.type, "image/jpeg");
    assert.ok(!map.free(x, y, floor + 0.6), "suspected: a temporary obstacle on the map (memory owns it)");
    assert.ok(map.temps.has("change-a"));
    const tp = { ...map.temps.get("change-a") };
    m.resolveChange("change-a", "confirmed", "a box");
    assert.ok(!map.temps.has("change-a"), "confirmed: the keep-out takes over");
    assert.deepEqual(seen, ["suspected", "confirmed"]);
    const ko = house.keepouts.find((k) => k.source === "change");
    assert.deepEqual([ko.kind, ko.x, ko.y, ko.r, ko.reach, ko.change], ["change", x, y, 0.85, 0.25, "change-a"], "as large as it blocked while suspected: its place's error stays in");
    assert.deepEqual([ko.r, ko.zMin, ko.zMax], [tp.r, Math.round(100 * tp.zMin) / 100, Math.round(100 * tp.zMax) / 100]);
    assert.ok(!map.free(x, y, floor + 0.6), "blocked at the box's height (map finalized)");
    assert.equal(saved.length, 1);
    assert.equal(changed[0].rebuild, false);
    const g = m.addChange({ kind: "gone", x: x + 0.1, y, z: floor + 0.4, size: 0.5 });
    m.resolveChange(g.id, "confirmed");
    assert.ok(!house.keepouts.some((k) => k.source === "change"), "gone: the change keep-out is removed");
    assert.ok(map.free(x, y, floor + 0.6));
    assert.match(m.whatChanged(), /^2 changes\. Confirmed: a new obstacle in the Living room.*about 50 cm across \(seen once, last just now; a box\); something in the scan is gone in the Living room/);
  },

  async "confirmed in the real world, a new obstacle still blocks the whole of it: the browser runs' reports (0.19-0.57 m off) as keep-out and map; gone frees it all again"() {
    const { house, map } = await fixture(), vox = VoxelMap.forMap(map), B = { x: 0.9, y: 2.4, w: 0.5, d: 0.5 }, floor = map.floorAt(B.x, B.y), z = floor + 0.4;
    for (let i = 0; i < vox.n; i++) if (vox.st[i] === UNKNOWN) vox.set(i, -32, FLAG.CARVED); // known free around it
    // where tools/depth-check.html put the 0.5 m box (report minus box) and how large it said it was
    const runs = { clean: [0, 0.2, 0.37], aug: [-0.27, 0.03, 0.26], "aug + lamp": [0.01, 0.19, 0.58], night: [-0.04, 0.22, 0.66], "people + aug": [-0.35, 0.13, 0.28], clean2: [0.19, 0, 0.5], aug2: [0.31, 0, 0.5] };
    const share = (blocked) => {
      let n = 0, inside = 0;
      for (let i = 0; i < 20; i++) for (let j = 0; j < 20; j++) n++, (inside += blocked(B.x - B.w / 2 + ((i + 0.5) * B.w) / 20, B.y - B.d / 2 + ((j + 0.5) * B.d) / 20) ? 1 : 0);
      return inside / n;
    };
    const vox0 = vox.st.slice(), lines = [];
    for (const [name, [dx, dy, size]] of Object.entries(runs)) {
      const m = await HouseMemory.open(`${house.id}:real-${name}`, { indexedDB: null });
      m.setHouse({ house, map, vox, save: () => {} });
      const c = m.addChange({ kind: "obstacle", x: B.x + dx, y: B.y + dy, z, size, zMin: floor, zMax: floor + 0.8, sigma: 0.25 });
      const tp = { ...map.temps.get(c.id) };
      m.resolveChange(c.id, "confirmed", "a box");
      const ko = house.keepouts.find((k) => k.change === c.id), inKo = share((x, y) => Math.hypot(x - ko.x, y - ko.y) <= ko.r), onMap = share((x, y) => map.clearance(x, y, z) === 0);
      lines.push(`${name} ${Math.round(100 * inKo)}%/${Math.round(100 * onMap)}%`);
      assert.ok(!map.temps.has(c.id) && ko.r === tp.r, `${name}: the keep-out is the suspected circle (r ${ko.r})`);
      assert.ok(inKo >= 0.95 && Math.hypot(B.x - ko.x, B.y - ko.y) <= ko.r, `${name}: ${Math.round(100 * inKo)}% of the box inside the keep-out`);
      assert.ok(onMap >= 0.95, `${name}: ${Math.round(100 * onMap)}% of the box blocked on the map`);
      assert.ok(vox.state(c.x, c.y, z) === OCCUPIED, "occupied in the voxels where the reports put it");
      const g = m.addChange({ kind: "gone", x: c.x, y: c.y, z, size, of: c.id });
      m.resolveChange(g.id, "confirmed");
      assert.ok(!house.keepouts.some((k) => k.source === "change") && map.clearance(B.x, B.y, z) > 0, `${name}: gone, the box's place is free again`);
      assert.ok(vox.st.every((s, i) => s === vox0[i]), `${name}: and the voxels are as before`);
      m.close?.();
    }
    console.log(`      box in the keep-out / blocked on the map: ${lines.join(", ")}`);
  },

  async "a confirmed closed door isn't passable any more and the map is rebuilt without it"() {
    const { house, map } = await fixture();
    const m = await HouseMemory.open(house.id, { indexedDB: null }), events = [];
    m.setHouse({ house, map });
    m.on("house", (e) => events.push(e));
    const d = house.doors.find((q) => q.passable && q.rooms[0] === "r2" && q.rooms[1] === "r3"), mid = [(d.a[0] + d.b[0]) / 2, (d.a[1] + d.b[1]) / 2];
    const from = [-0.94, 1.06], to = [-1.24, 3.16], before = plan(map, from, to, { alt: 1.0 });
    assert.ok(before.doors.some((q) => q.id === d.id), "Room 2 to Room 3 through their doorway");
    const ch = m.addChange({ kind: "door-closed", x: mid[0] + 0.2, y: mid[1], z: 1, size: d.width });
    assert.match(m.whatChanged(), /Not confirmed yet: the doorway between Room 2 and Room 3 is closed/);
    assert.ok(map.temps.has(ch.id), "suspected: the doorway is a temporary obstacle");
    m.resolveChange(ch.id, "confirmed");
    assert.equal(d.passable, false);
    assert.equal(events[0].rebuild, true, "doors are cut when the map is built: rebuild it");
    assert.ok(events[0].saved instanceof Promise, "await the save before reloading anything");
    assert.ok(m.needsRebuild);
    assert.ok(map.temps.has(`door:${d.id}`), "until the rebuild (after landing) the closed doorway stays an obstacle on this map");
    const still = plan(map, from, to, { alt: 1.0 });
    assert.ok(!still.ok || !still.doors.some((q) => q.id === d.id), "the current map already avoids it");
    const rebuilt = new HomeMap(house).finalize(), after = plan(rebuilt, from, to, { alt: 1.0 });
    assert.ok(!after.ok || !after.doors.some((q) => q.id === d.id), "no longer through that doorway");
    m.setHouse({ house, map: rebuilt });
    assert.ok(!m.needsRebuild && !rebuilt.temps.has(`door:${d.id}`), "a map built from the house has the door as it is");
    const open = m.addChange({ kind: "door-open", x: mid[0], y: mid[1], z: 1, size: d.width });
    m.resolveChange(open.id, "confirmed");
    assert.equal(d.passable, true);
    // The 3D map: a confirmed closed door fills its doorway, a confirmed open one puts back what was there (voxels.js).
    const vox = new VoxelMap({ x0: mid[0] - 0.5, y0: mid[1] - 0.5, z0: -0.1, nx: 20, ny: 20, nz: 50 }), at = vox.idx(mid[0], mid[1], 1.0), was = vox.st[at];
    m.setHouse({ house, map: rebuilt, vox });
    const shut = m.addChange({ kind: "door-closed", x: mid[0], y: mid[1], z: 1, size: d.width });
    m.resolveChange(shut.id, "confirmed");
    assert.ok(shut.applied.voxels > 0 && vox.st[at] === OCCUPIED, "the closed doorway is occupied in the voxels");
    const again = m.addChange({ kind: "door-open", x: mid[0], y: mid[1], z: 1, size: d.width });
    m.resolveChange(again.id, "confirmed");
    assert.equal(vox.st[at], was, "open again: the doorway as it was before it closed (never seen: still unknown, not free)");
    // A door the scan caught closed opens when the pilot confirms it open.
    const shy = house.doors.find((q) => q !== d && q.passable && q.rooms[1]);
    Object.assign(shy, { passable: false });
    const smid = [(shy.a[0] + shy.b[0]) / 2, (shy.a[1] + shy.b[1]) / 2], o2 = m.addChange({ kind: "door-open", x: smid[0], y: smid[1], z: 1, size: shy.width, door: shy.id });
    assert.equal(m.resolveChange(o2.id, "dismissed", null, { by: "claude" })?.status, "dismissed", "Claude may only wave away an opening");
    const o3 = m.addChange({ kind: "door-open", x: smid[0], y: smid[1], z: 1, size: shy.width, door: shy.id });
    assert.equal(m.resolveChange(o3.id, "confirmed", null, { by: "claude" }), null, "Claude never opens a door on its own");
    m.resolveChange(o3.id, "confirmed");
    assert.equal(shy.passable, true, "the pilot's confirmation opens it");
  },

  async "dismissed changes aren't raised again at that spot until it changes again; transient ones never stop anything"() {
    const c = clock(), m = await HouseMemory.open(house.id, { indexedDB: null, now: c.now }), events = [];
    m.setHouse({ house, map });
    m.on("change", (r) => events.push(`${r.id}:${r.status}`));
    const p = { kind: "obstacle", x: 1.0, y: 2.0, z: 0.4, size: 0.4 };
    const a = m.addChange({ ...p, id: "c1" });
    const merged = m.addChange({ ...p, id: "c2", x: 1.1 });
    assert.equal(merged, a, "seen again: the same change");
    assert.equal(a.n, 2);
    m.resolveChange("c1", "dismissed", "the laundry basket, it's fine");
    assert.equal(m.addChange({ ...p, id: "c3", x: 0.95 }), null, "same spot, same size: not raised again");
    assert.deepEqual(events, ["c1:suspected", "c1:suspected", "c2:merged", "c1:dismissed", "c3:dismissed"], "the detector hears about every id it used");
    assert.equal(m.addChange({ ...p, kind: "gone" }).status, "suspected", "a report without an id is fine too");
    assert.equal(a.suppressed, 1);
    assert.equal(m.addChange({ ...p, id: "c6", signature: "x" }), null, "no signature on the dismissed one: still the same");
    assert.ok(m.addChange({ ...p, id: "c4", size: 1.2 }), "much bigger: something else, raised");
    assert.ok(m.addChange({ ...p, id: "c5", kind: "moved" }), "another kind: raised");
    m.noteUnchanged({ x: 1.0, y: 2.0, r: 0.3 }, c.t + 1000);
    assert.equal(m.changes.find((q) => q.id === "c4").status, "dismissed", "a suspected change no longer there is dismissed...");
    assert.equal(m.changes.find((q) => q.id === "c4").transient, true, "...as transient");
    c.t += 2000;
    const back = m.addChange({ ...p, id: "c7" });
    assert.ok(back && back.status === "suspected", "the spot looked as the scan expects in between, so a new report counts");
    m.resolveChange("c7", "dismissed", "the dog", { transient: true });
    assert.ok(m.addChange({ ...p, id: "c8" }), "dismissed as transient: never suppresses");
  },

  async "retention: keepDays and keepFlights, with their trails and sightings, in storage too"() {
    const idb = new FakeIDB(), c = clock(T0 - 10 * DAY);
    const m = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now, keepDays: 5, keepFlights: 3 });
    m.setHouse({ house, map });
    for (let i = 0; i < 6; i++) {
      c.at(T0 - (10 - i * 2) * DAY);
      m.startFlight({ kind: `f${i}` });
      fly(m, [[0.5, 3], [1.5, 3]], c);
      m.addSighting({ label: "cat", x: 1, y: 3, t: c.t });
      m.endFlight(`flight ${i}`, c.t);
    }
    c.at(T0);
    m.addChange({ kind: "obstacle", x: 1, y: 2, size: 0.3 }, T0 - 9 * DAY);
    m.prune(T0);
    assert.deepEqual(m.flights.map((f) => f.kind), ["f3", "f4", "f5"], "the newest 3 within 5 days");
    assert.equal(m.sightings.length, 3, "their sightings only");
    assert.equal(m.changes.length, 0, "a suspected change older than keepDays goes");
    await m.close();
    const again = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now, keepDays: 5, keepFlights: 3 });
    assert.equal(again.flights.length, 3);
    assert.equal(again.trails.size, 3);
    const trails = idb.dbs.get("whoop-memory").stores.get("trails").data;
    assert.ok([...trails.values()].every((ch) => again.flights.some((f) => f.id === ch.flight)), "dropped flights' trail chunks deleted from storage");
    await again.clear();
    assert.equal(idb.dbs.get("whoop-memory").stores.get("flights").data.size, 0, "clear() empties storage");
  },

  async "alerts.js notes what it logs: findings update the sighting, other events go in the timeline"() {
    const c = clock(Date.now() - 1000), m = await HouseMemory.open(house.id, { indexedDB: null, now: c.now });
    m.setHouse({ house, map });
    const alerts = new Alerts({ settings: { get: () => undefined }, fetch: null, Notification: undefined, indexedDB: undefined, memory: m });
    m.startFlight({ kind: "patrol" });
    c.t += 300;
    const s = m.addSighting({ label: "person", trackId: 3, x: 2.5, y: 1.0, score: 0.9, confirmed: true });
    await alerts.log({ text: "A person in the Living room.", kind: "finding", label: "person", trackId: 3, room: s.room, pose: { x: 1, y: 1, yaw: 0, sigma: 0.1 } });
    assert.equal(m.sightings.length, 1, "the same sighting");
    await alerts.notify({ text: "Battery low, flying home.", urgency: "default", kind: "safety" });
    assert.deepEqual(m.timeline({ limit: 5 }).map((e) => e.kind).slice(0, 2), ["safety", "sighting"]);
    assert.equal((await alerts.events()).length, 2, "alerts keeps its own log for the Events list");
    assert.match(m.recall("anything new?"), /Last seen: a person/);
  },

  async "toggles: a door closed, opened and closed again (or a box back after it was gone) is a new change, not the old confirmation"() {
    const { house, map } = await fixture();
    const m = await HouseMemory.open(house.id, { indexedDB: null });
    m.setHouse({ house, map });
    const d = house.doors.find((q) => q.id === "r2:w1-o1+r3:w2-o1"), at = { x: (d.a[0] + d.b[0]) / 2, y: (d.a[1] + d.b[1]) / 2, z: 1.2, size: d.width };
    const closed = m.addChange({ kind: "door-closed", ...at, id: "d1" });
    m.resolveChange("d1", "confirmed");
    assert.equal(d.passable, false);
    assert.equal(m.addChange({ kind: "door-closed", ...at, id: "d1b" }), closed, "closed again while it is confirmed closed: that confirmation, seen again");
    m.resolveChange(m.addChange({ kind: "door-open", ...at, id: "d2" }).id, "confirmed");
    assert.equal(d.passable, true);
    const again = m.addChange({ kind: "door-closed", ...at, id: "d3" });
    assert.ok(again.status === "suspected" && again !== closed, "closed after a confirmed open: a new change");
    assert.ok(map.temps.has("d3"), "on the map until someone answers");
    m.resolveChange("d3", "confirmed");
    assert.equal(d.passable, false);
    const p = { x: 3.0, y: 4.0, z: 0.4, size: 0.4 }, box = m.addChange({ kind: "obstacle", ...p });
    m.resolveChange(box.id, "confirmed");
    m.resolveChange(m.addChange({ kind: "gone", ...p }).id, "confirmed");
    assert.ok(!house.keepouts.some((k) => k.change === box.id));
    const back = m.addChange({ kind: "obstacle", ...p });
    assert.ok(back.status === "suspected" && back !== box, "back after a confirmed gone: new");
    m.resolveChange(back.id, "confirmed");
    assert.ok(house.keepouts.some((k) => k.change === back.id), "and a keep-out again once confirmed");
  },

  async "Claude resolves only in the safe direction (a new blocker, or keeping what the scan has); its dismissals stop nothing"() {
    const { house, map } = await fixture();
    const m = await HouseMemory.open(house.id, { indexedDB: null });
    m.setHouse({ house, map });
    const ob = m.addChange({ kind: "obstacle", x: 1.0, y: 2.0, size: 0.4 });
    assert.equal(m.resolveChange(ob.id, "dismissed", "no", { by: "claude" }), null, "a blocker isn't waved away on Claude's word");
    const gone = m.addChange({ kind: "gone", x: 3.0, y: 4.0, size: 0.4 }), moved = m.addChange({ kind: "moved", x: 3.6, y: 1.0, size: 0.4, from: { x: 3.0, y: 1.5, z: 0.4 } });
    const open = m.addChange({ kind: "door-open", x: -1.84, y: 2.0, size: 0.5 });
    for (const c of [gone, moved, open]) assert.equal(m.resolveChange(c.id, "confirmed", "yes", { by: "claude" }), null, `${c.kind}: nothing freed on Claude's word`);
    assert.equal(m.resolveChange(ob.id, "confirmed", "a box", { by: "claude" }).status, "confirmed", "a new blocker: yes");
    assert.equal(m.resolveChange(gone.id, "dismissed", "still there", { by: "claude" }).status, "dismissed", "keeping what the scan has: yes");
    assert.equal(m.addChange({ kind: "gone", x: 3.0, y: 4.0, size: 0.4 })?.status, "suspected", "Claude's dismissal suppresses nothing");
    assert.equal(m.resolveChange(ob.id, "dismissed", "changed my mind", { by: "claude" }), null, "nor undoes what was confirmed");
  },

  async "a weak suspicion (nav/changes.js: the pose's error explains most of it) Claude may dismiss, and its weak reports there stay down that flight; a well-evidenced report is raised, makes a weak record well-evidenced, and Claude can't wave it away"() {
    const { house, map } = await fixture();
    const c = clock(), m = await HouseMemory.open(`${house.id}:weak`, { indexedDB: null, now: c.now });
    m.setHouse({ house, map });
    m.startFlight({ kind: "patrol" });
    const A = { kind: "obstacle", x: 1.0, y: 2.0, z: 1.1, size: 0.4, sigma: 0.25 }, B = { ...A, x: 3.0, y: 4.0 }, D = { ...A, x: 1.0, y: 4.4 };
    const w = m.addChange({ ...A, weak: true }, (c.t += 1000));
    assert.ok(w.weak && map.temps.has(w.id), "weak, and on the map like any suspected blocker until someone says");
    assert.equal(m.resolveChange(w.id, "dismissed", "Claude: nothing new (70% sure).", { by: "claude" })?.status, "dismissed", "Claude may drop a weak suspicion");
    assert.ok(!map.temps.has(w.id), "off the map");
    assert.equal(m.addChange({ ...A, x: 1.05, weak: true }, (c.t += 1000)), null, "its weak reports there: not raised again this flight");
    const s = m.addChange({ ...A, x: 1.05 }, (c.t += 1000));
    assert.ok(s?.status === "suspected" && !s.weak, "a well-evidenced report there is");
    assert.equal(m.resolveChange(s.id, "dismissed", "no", { by: "claude" }), null, "and Claude can't wave that away");
    const b = m.addChange({ ...B, weak: true }, (c.t += 1000)), b2 = m.addChange({ ...B, x: 3.1 }, (c.t += 1000));
    assert.ok(b2 === b && b.n === 2 && b.weak === false, "a well-evidenced report of a weak record: no longer weak");
    assert.equal(m.resolveChange(b.id, "dismissed", "no", { by: "claude" }), null);
    const d = m.addChange({ ...D, weak: true }, (c.t += 1000));
    m.resolveChange(d.id, "dismissed", "Claude: nothing new (70% sure).", { by: "claude" });
    m.endFlight("Landed.", (c.t += 1000));
    m.startFlight({ kind: "patrol" }, (c.t += 60000));
    const again = m.addChange({ ...D, weak: true }, (c.t += 1000));
    assert.ok(again?.status === "suspected" && again.id !== d.id, "the next flight raises it again (Claude is asked again)");
    assert.equal(m.addChange({ kind: "gone", x: 3.0, y: 1.0, size: 0.4 }, (c.t += 1000)).weak, undefined, "only reports that say so are weak");
    for (const id of [...map.temps.keys()]) map.removeTemp(id);
  },

  async "a confirmed gone frees only what something marked occupied: unknown space stays unknown, the plan's stays"() {
    const { house, map } = await fixture();
    const m = await HouseMemory.open(house.id, { indexedDB: null }), vox = new VoxelMap({ x0: 2.5, y0: 3.5, z0: -0.1, nx: 20, ny: 20, nz: 30 });
    const surface = vox.idx(2.86, 4.0, 0.3), inside = vox.idx(3.0, 4.0, 0.3), planned = vox.idx(3.1, 4.1, 0.3);
    vox.set(surface, 64, FLAG.CAPTURE);
    vox.set(planned, 64, FLAG.PLAN);
    m.setHouse({ house, map, vox });
    const g = m.addChange({ kind: "gone", x: 3.0, y: 4.0, z: 0.4, size: 0.4, zMin: 0, zMax: 0.8 });
    m.resolveChange(g.id, "confirmed");
    assert.deepEqual([vox.st[surface], vox.st[inside], vox.st[planned]], [FREE, UNKNOWN, OCCUPIED]);
    assert.equal(g.applied.freed, 1);
    const ob = m.addChange({ kind: "obstacle", x: 3.0, y: 4.0, z: 0.4, size: 0.4, zMin: 0, zMax: 0.8 });
    m.resolveChange(ob.id, "confirmed");
    assert.equal(vox.st[inside], OCCUPIED, "a confirmed obstacle marks its box occupied");
  },

  async "a change Claude takes for a person or pet blocks until a minute after it was last in view, reported or seen, then is dismissed as transient"() {
    const { house, map } = await fixture();
    const c = clock(), m = await HouseMemory.open(house.id, { indexedDB: null, now: c.now });
    map.clock = () => c.t;
    m.setHouse({ house, map });
    const ch = m.addChange({ kind: "obstacle", x: 0.9, y: 2.4, z: 0.85, size: 0.5, zMin: 0, zMax: 1.7 });
    m.passing(ch.id, { note: "Claude: the dog (92% sure)." });
    const t = map.temps.get(ch.id);
    assert.deepEqual([t.kind, t.until], ["person", Infinity], "a person-like obstacle: memory takes it off");
    assert.ok(!map.free(0.9, 2.4, 1.0), "still blocked: a still person the detector missed is not flown through");
    c.t += 40000;
    m.addChange({ kind: "obstacle", x: 0.92, y: 2.4, z: 0.85, size: 0.5 });
    assert.equal(ch.passing.until, c.t + MEMORY.passingMs, "reported again: a minute more");
    c.t += 50000;
    m.expirePassing();
    assert.equal(ch.status, "suspected");
    c.t += 5000;
    assert.equal(m.seen(ch.id), ch, "the detector's blobs there");
    assert.deepEqual([ch.passing.until, ch.last], [c.t + MEMORY.passingMs, c.t]);
    // In flight: sure samples with the spot in view keep it; looking away (or unsure) doesn't.
    m.startFlight({ kind: "patrol" });
    const look = (yaw, sigma = 0.05, x = -1.0) => m.samplePose({ x, y: 2.4, z: 1, yaw, sigma, status: "ok" });
    for (let i = 0; i < 300; i++) (c.t += 500), look(0); // 150 s facing it, 1.9 m away
    assert.equal(ch.status, "suspected", "in view the whole time: still there");
    assert.equal(ch.passing.until, c.t + MEMORY.passingMs);
    for (let i = 0; i < 100; i++) (c.t += 500), look(0, 0.4); // unsure position: proves nothing
    for (let i = 0; i < 20; i++) (c.t += 500), look(Math.PI); // looking away
    assert.deepEqual([ch.status, ch.transient, ch.by], ["dismissed", true, "passing"], "a minute after it was last in view");
    assert.ok(map.free(0.9, 2.4, 1.0) && !map.temps.has(ch.id));
    assert.ok(m.addChange({ kind: "obstacle", x: 0.9, y: 2.4, z: 0.85, size: 0.5 }), "never suppresses anything");
    assert.equal(m.seen(ch.id), null, "resolved: nothing to renew");
    const far = m.addChange({ kind: "obstacle", x: 4.0, y: 2.4, z: 0.85, size: 0.5 });
    m.passing(far.id);
    for (let i = 0; i < 130; i++) (c.t += 500), look(0, 0.05, -1.5); // 5.5 m away: beyond what the detector compares
    assert.equal(far.status, "dismissed", "too far to tell");
  },

  async "the simulator has its own memory: its sightings aren't real ones, its changes never edit or save the house"() {
    const idb = new FakeIDB(), { house, map } = await fixture(), saved = [];
    const real = await HouseMemory.open(house.id, { indexedDB: idb }), sim = await HouseMemory.open(house.id, { indexedDB: idb, world: "sim" });
    assert.notEqual(real, sim);
    real.setHouse({ house, map, save: (h) => saved.push(h) });
    sim.setHouse({ house, map, save: (h) => saved.push(h) });
    sim.startFlight({ kind: "patrol", mission: { kind: "patrol" } });
    sim.addSighting({ label: "cat", x: 2.0, y: 3.0, score: 0.8 });
    sim.endFlight("Patrolled.");
    assert.equal(real.lastSeen("cat"), null, "a simulated cat is never last seen for real");
    assert.match(sim.recall("where did you last see the cat?"), /^I last saw the cat in the Living room near the sofa, just now \(on patrol, in the simulator\)/);
    assert.equal(sim.timeline().find((e) => e.kind === "flight").text, "Took off in the simulator: patrol.");
    assert.match(sim.recall("where have you been?"), /^The flight just now in the simulator: /);
    const d = house.doors.find((q) => q.id === "r2:w1-o1+r3:w2-o1"), ch = sim.addChange({ kind: "door-closed", x: (d.a[0] + d.b[0]) / 2, y: (d.a[1] + d.b[1]) / 2, z: 1, size: d.width });
    sim.resolveChange(ch.id, "confirmed");
    assert.deepEqual([d.passable, saved.length, house.keepouts.some((k) => k.source === "change")], [true, 0, false], "the house is untouched");
    assert.ok(map.temps.has(ch.id), "the simulator's closed door stays on its map as a temporary obstacle");
    sim.setHouse({});
    assert.ok(!map.temps.has(ch.id), "and comes off when the simulator lets go of the map");
    await real.close();
    await sim.close();
    assert.equal((await HouseMemory.open(house.id, { indexedDB: idb })).sightings.length, 0, "stored apart too");
    assert.equal((await HouseMemory.open(house.id, { indexedDB: idb, world: "sim" })).sightings.length, 1);
  },

  async "where it is, honestly: lost and unsure positions say so; hovering adds no distance; a room counts after two sure samples"() {
    const c = clock(), m = await HouseMemory.open(house.id, { indexedDB: null, now: c.now });
    m.setHouse({ house, map });
    m.startFlight({ kind: "manual" });
    const at = (x, y, sigma = 0.08, status = "ok") => ((c.t += 500), m.samplePose({ x, y, z: 1, yaw: 0, sigma, status, room: map.roomAt(x, y)?.id ?? null }, c.t));
    const jit = (i, a) => a * Math.sin(i * 12.9898 + 78.233 * a);
    at(-0.9, 1.0), at(-0.9, 1.05), at(1.5, 3.0), at(-1.2, 3.3), at(-1.2, 3.35);
    assert.deepEqual(m.flight.rooms, ["r2", "r3"], "the Living room, one sample of it, doesn't count");
    let before = m.flight.distance;
    for (let i = 0; i < 20; i++) at(-1.2 + jit(i, 0.3), 3.3 + jit(i + 7, 0.3), 0.4, "degraded");
    assert.equal(m.flight.distance, before, "an unsure hover adds nothing");
    assert.match(m.flightSoFar(), /^This flight: \d+ s so far, [\d.]+ m flown, took off in Room 2, then Room 3; now probably in Room 3 \(position unsure, ±0\.8 m\)\.$/);
    before = m.flight.distance;
    for (let i = 0; i < 20; i++) at(-1.2 + jit(i, 0.03), 3.3 + jit(i + 3, 0.03), 0.08);
    assert.ok(m.flight.distance - before < 0.2, `a sure hover: ${(m.flight.distance - before).toFixed(2)} m`);
    assert.match(m.flightSoFar(), /took off in Room 2, now in Room 3\.$/);
    at(NaN, NaN, 0, "lost");
    c.t += 12000;
    assert.match(m.flightSoFar(), /; position lost now \(last known in Room 3, 1\d s ago\) \(position lost 1 time in all\)\.$/);
    m.addSighting({ label: "cat", x: 1.8, y: 3.0, sigma: 0.4 });
    m.addSighting({ label: "dog", x: 0.55, y: 2.2, sigma: 0.12 });
    m.addSighting({ label: "person", x: 2.5, y: 4.0, sigma: 0.05 });
    assert.match(m.recall("where's the cat?"), /^I last saw the cat probably in the Living room/);
    assert.match(m.recall("where's the dog?"), /^I last saw the dog probably in the Living room/, "within σ of Room 2");
    assert.match(m.recall("where was the person?"), /^I last saw a person in the Living room/);
  },

  async "a flight record keeps where the position was lost (lists need no trail); an older trail is read back by its own key range, not the whole store; a spot off every room is near the nearest room or outside the mapped rooms, never a hallway"() {
    const idb = new FakeIDB(), c = clock();
    const m = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now, ramSamples: 50 });
    m.setHouse({ house, map });
    const f1 = m.startFlight({ kind: "patrol" });
    fly(m, [[0.0, 0.6], [0.9, 2.2], [1.2, 3.4]], c, { lostAt: [20, 30] });
    m.endFlight("Patrolled.", (c.t += 1000));
    const before = m.trails.get(f1.id), lostIdx = before.findIndex((s) => s.lost), spot = before[lostIdx - 1];
    assert.deepEqual([f1.losses, f1.lostAt], [1, [{ t: before[lostIdx].t, x: spot.x, y: spot.y, room: spot.room }]], "one loss, where it was last known");
    assert.match(f1.report, /The position was lost 1 time\./);
    for (let i = 0; i < 6; i++) (m.startFlight({ kind: "patrol" }), fly(m, [[0.0, 0.6], [0.9, 2.2]], c), m.endFlight("Flew.", (c.t += 1000)));
    await m.flush();
    assert.ok(!m.trails.has(f1.id) && m.flights[0].losses === 1 && m.flights.slice(1).every((f) => f.losses === 0), "out of RAM, its record still says");
    const db = idb.dbs.get("whoop-memory"), stored = db.stores.get("trails").data.size, n = Math.ceil(f1.samples / MEMORY.chunk), r0 = db.read;
    const back = await m.trailOf(f1.id);
    assert.deepEqual([back.length, back.filter((s) => s.lost).length, db.read - r0], [f1.samples, 1, n], `read ${db.read - r0} of ${stored} stored chunks`);
    assert.deepEqual(await m.trailOf("f-none"), []);
    // a record stored before flights kept their losses gets them from its trail on opening
    await m.close();
    const rec = [...db.stores.get("flights").data.values()].find((f) => f.id === f1.id);
    delete rec.losses, delete rec.lostAt;
    const again = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now, ramSamples: 50 });
    assert.deepEqual([again.flights[0].losses, again.flights[0].lostAt], [1, f1.lostAt]);
    again.setHouse({ house, map });
    // off every room: the nearest room within MEMORY.nearRoom, or outside the mapped rooms
    assert.equal(again.roomName(null), "outside the mapped rooms");
    again.startFlight({ kind: "patrol" });
    fly(again, [[0.0, 0.6], [0.3, 0.4]], c);
    c.t += 500, again.samplePose({ x: 1.0, y: -0.3, z: 1, yaw: 0, sigma: 0.05, status: "ok", room: null }, c.t);
    assert.match(again.flightSoFar(), /; now near the Living room\.$/);
    const by = again.addSighting({ label: "person", x: 1.0, y: -0.3, z: 1, sigma: 0.05, t: c.t });
    const away = again.addSighting({ label: "dog", x: 9, y: 9, z: 0.3, sigma: 0.05, t: c.t });
    assert.equal(by.room, null);
    assert.match(again.where(by), /^near the Living room( by the [a-z ]+)?$/);
    assert.equal(again.where(away), "outside the mapped rooms");
    const f = again.endFlight("Patrolled.", (c.t += 1000)), words = `${f.report} ${again.recall("where's the dog?")} ${again.timeline().map((e) => e.text).join(" ")}`;
    assert.match(f.report, /Saw a person near the Living room( by the [a-z ]+)?, the dog outside the mapped rooms\./);
    assert.ok(!/hallway/.test(words), words);
    console.log(`      ${f.report}`);
    await again.close();
  },

  async "door questions are answered from what flights saw there; rooms from what was seen in them"() {
    const { house, map } = await fixture(), c = clock();
    house.landmarks.push({ name: "front door", room: "r2", x: 0.02, y: -0.1, z: 1, source: "user" });
    const m = await HouseMemory.open(house.id, { indexedDB: null, now: c.now });
    m.setHouse({ house, map });
    assert.equal(m.recall("is the front door open?"), "The doorway from Room 2 to outside: open in the 3D scan; no flight I remember went near it, so I don't know how it is now.");
    m.startFlight({ kind: "manual" });
    fly(m, [[0.0, 0.9], [0.0, 0.4]], c);
    m.addSighting({ label: "cat", x: 2.0, y: 3.2, t: c.t });
    m.endFlight("Flew.", c.t);
    c.t += 5 * 60e3;
    assert.equal(m.recall("Is the front door closed?"), "The doorway from Room 2 to outside: open in the 3D scan; I last flew near it 5 min ago and no change was reported there.");
    const d = house.doors.find((q) => q.id === "r2:w1-o1+r3:w2-o1"), ch = m.addChange({ kind: "door-closed", x: (d.a[0] + d.b[0]) / 2, y: (d.a[1] + d.b[1]) / 2, z: 1, size: d.width });
    assert.equal(m.recall("is the door between Room 2 and Room 3 open?"), "The doorway between Room 2 and Room 3: looked closed just now, not confirmed yet.");
    m.resolveChange(ch.id, "confirmed");
    assert.equal(m.recall("is the door between room 3 and room 2 shut"), "The doorway between Room 2 and Room 3: confirmed closed just now.");
    assert.match(m.recall("is the door open?"), /^I can't tell which door you mean; the map's doorways: the doorway from the Living room to outside; /);
    assert.equal(m.recall("what did you see in the living room?"), "I haven't been in the Living room on any flight I remember (1 flight). From outside it I saw the cat there, 5 min ago.");
    assert.match(m.recall("what did you find in room 2"), /^I was last in Room 2 5 min ago, (briefly|for about \d+ s), on the flight 5 min ago; 1 of 1 flight went there\.$/);
    assert.match(m.recall("anything in Room 3?"), /^I haven't been in Room 3 on any flight I remember/);
  },

  async "bounded RAM: pictures beyond the newest per label, the open changes' and ramPictures stay in storage (read back on asking); old trails too, and questions about them still answered"() {
    const idb = new FakeIDB(), c = clock(T0 - 2 * DAY);
    const m = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now, ramPictures: 5, ramSamples: 50 });
    m.setHouse({ house, map });
    for (let i = 0; i < 8; i++) {
      c.t += 3600e3;
      m.startFlight({ kind: "patrol" });
      fly(m, [[0.0, 0.6], [0.9, 2.2], [1.2, 3.4]], c);
      for (let k = 0; k < 3; k++) m.addSighting({ label: k ? "person" : "cat", x: 1.8 + k, y: 3.0, snapshot: jpeg(20 + i), t: (c.t += 20000) });
      m.endFlight("Patrolled.", (c.t += 1000));
    }
    await m.flush();
    const ram = m.ram(), withPic = m.sightings.filter((s) => s.snapshot).length;
    console.log(`      ${m.sightings.length} sightings, ${withPic} pictures in RAM (${ram.bytes} bytes), ${ram.samples} of ${m.flights.reduce((a, f) => a + f.samples, 0)} trail samples in RAM`);
    assert.ok(withPic <= 5 + 2 && m.sightings.every((s) => s.pic), "the rest in storage");
    assert.ok(ram.samples <= 50 + m.flights.at(-1).samples && m.trails.size < 8);
    const old = m.sightings[0];
    assert.equal(old.snapshot, null);
    assert.equal((await m.picture(old)).size, 24, "read back from storage");
    assert.equal((await m.trailOf(m.flights[0].id)).length, m.flights[0].samples, "an old trail read back");
    assert.match(m.recall(`When were you last in ${name(map.roomAt(0.0, 0.6).id)}?`), /on the flight .*; 8 of 8 flights went there\./, "rooms from the flights' index, not their trails");
    await m.close();
    const again = await HouseMemory.open(house.id, { indexedDB: idb, now: c.now, ramPictures: 5 });
    assert.ok(again.sightings.filter((s) => s.snapshot).length <= 7 && again.lastSeen("cat").snapshot?.size === 31, "reopened: only the newest pictures fetched");
    await again.clearPictures();
    assert.ok(!(await again.picture(again.sightings[0])) && idb.dbs.get("whoop-memory").stores.get("pictures").data.size === 0, "every picture forgotten on asking");
    const f0 = again.flights[0].id;
    assert.ok(await again.deleteFlight(f0));
    assert.ok(!again.flights.some((f) => f.id === f0) && ![...idb.dbs.get("whoop-memory").stores.get("trails").data.values()].some((t) => t.flight === f0), "a flight forgotten with its trail");
    await again.setRetention({ keepFlights: 2 });
    assert.equal(again.flights.length, 2);
    await again.clear();
  },

  async "recall in people's words: time windows, what people call rooms, people and pets, how many times"() {
    const { house, map } = await fixture(), c = clock(T0 - DAY - 3 * H);
    house.rooms.find((r) => r.id === map.roomAt(1.9, 3.0).id).kind = "living room";
    const m = await HouseMemory.open(`${house.id}:words`, { indexedDB: null, now: c.now });
    m.setHouse({ house, map });
    m.startFlight({ kind: "patrol" });
    fly(m, [[0.0, 0.6], [0.9, 2.2], [1.2, 3.4]], c);
    m.addSighting({ label: "dog", x: 1.8, y: 3.0, t: c.t });
    m.endFlight("Patrolled.", c.t + 1000);
    c.at(T0 - 4 * H); // today 10:00
    m.startFlight({ kind: "patrol" });
    fly(m, [[0.0, 0.6], [0.9, 2.2], [1.2, 3.4]], c);
    m.addSighting({ label: "cat", x: 1.8, y: 3.0, t: c.t });
    m.addSighting({ label: "cat", x: -1.3, y: 3.6, t: c.t + 60e3 });
    m.addSighting({ label: "person", x: 2.5, y: 1.0, t: c.t + 90e3 });
    m.endFlight("Patrolled.", c.t + 100e3);
    c.at(T0);
    assert.match(m.recall("did you see the cat this morning?"), /^Yes, this morning: I saw the cat 2 times; the last in Room 3/);
    assert.match(m.recall("did you see the dog today?"), /^Not today\. I last saw the dog in the Living room near the sofa, yesterday at 1\d:\d\d\./);
    assert.match(m.recall("did you see the dog last night?"), /^Not last night \(I didn't fly last night\)\./);
    assert.match(m.recall("how many times did you see the cat?"), /^I saw the cat 2 times in all; the last in Room 3/);
    assert.match(m.recall("where are the pets?"), /I last saw the cat in Room 3.*I last saw the dog in the Living room/);
    assert.match(m.recall("have you seen my husband?"), /^I last saw a person in /);
    assert.match(m.recall("anything in the lounge?"), /^I was last in the Living room today at 10:\d\d/, "the lounge is the living room");
    assert.match(m.recall("did you see the cat in the lounge today?"), /^Yes, today: I saw the cat in the Living room once, near the sofa, today at 10:\d\d\./);
    assert.match(m.recall("where did you fly yesterday?"), /^1 flight yesterday: The flight yesterday at 11:00: /);
    assert.match(m.recall("have you flown in the last hour?"), /^I didn't fly in the last hour\. The last flight was today at 10:00/);
    assert.equal(m.recall("where did you last see the cat"), m.recall("where did you last see the cat"), "deterministic");
    // the way people ask: never "I haven't seen ..." about what the memory holds
    for (const q of ["Where was the cat last seen?", "Where is the cat now?", "Where was the cat last spotted?", "Where's the kitty at the moment?"])
      assert.match(m.recall(q), /^I last saw the cat in Room 3/, q);
    for (const q of ["Is anyone home?", "Did my husband come home?", "Is there someone in the house?", "Was anybody around?"]) assert.match(m.recall(q), /^I last saw a person in /, q);
    assert.match(m.recall("Who was in the lounge today?"), /^(Yes, today: I saw a person in the Living room|Not today\. I haven't seen anyone in the Living room)/);
    assert.match(m.recall("Is the dog around?"), /^I last saw the dog in the Living room/);
    for (const q of ["Who's home?", "Who is home?", "Who was here today?", "Who is in the house?", "Who's around?", "Where is everybody?", "Is the house empty?", "Is no one home?", "Who's in the house right now?", "Who did you see?"])
      assert.match(m.recall(q), /^(I last saw a person in |Yes, today: I saw a person once)/, q);
    assert.match(m.recall("Did you see who came home?"), /^I last saw a person in /);
    assert.doesNotMatch(m.recall("What's around?"), /haven't seen "/, "no subject left: not asked about \"around\"");
    assert.match(m.recall("Are my keys in the house?"), /^I haven't seen "my keys" on/, "the whole house is no room");
    assert.match(m.recall("Is the sofa in the house?"), /^The map has the sofa in the Living room/);
    house.landmarks.push({ name: "Cat tower", x: 1.8, y: 3.2, room: map.roomAt(1.8, 3.2).id });
    assert.match(m.recall("Where is the cat tower?"), /^The map has the cat tower in the Living room; no flight/, "a thing the map has, not the cat");
    assert.match(m.recall("Have you seen anything unusual?"), /^No differences from the 3D scan/, "unusual: the changes");
    assert.match(m.recall("Where is the sofa?"), /^The map has the sofa in the Living room; no flight I remember noted it/, "a thing the map has is what is asked about");
    assert.match(m.recall("Is the dog's bed in Room 3?"), /^I haven't seen "dog bed" in Room 3/, "a dog's bed is not the dog");
    assert.match(m.recall("have you seen my keys?"), /^I haven't seen "my keys"/, "what it never saw stays unseen");
    assert.match(m.recall("where were my keys last seen?"), /^I haven't seen "my keys" on/, "the words around what it asks about aren't it");
  },

  async "after landing: how much of each room the camera saw (vox.seenAt) and a plain report of the flight"() {
    const { house, map } = await fixture(), c = clock(T0);
    const vox = VoxelMap.forMap(map), R = map.roomAt(1.2, 3.4).id, others = house.rooms.filter((r) => r.id !== R);
    for (let i = 0; i < vox.n; i++) {
      const [x, y, z] = vox.center(i), f = map.floorAt(x, y);
      if (f != null && map.roomAt(x, y) && z > f + 0.1 && z < f + 1.9) vox.set(i, -32, FLAG.CARVED);
    }
    const m = await HouseMemory.open(`${house.id}:inspect`, { indexedDB: null, now: c.now });
    m.setHouse({ house, map, vox });
    m.startFlight({ kind: "patrol", mission: { kind: "patrol" } });
    fly(m, [[0.0, 0.6], [0.9, 2.2], [1.2, 3.4]], c);
    // the camera saw the room it flew in except its part near the floor, and nothing of the others
    vox.seen = new Float32Array(vox.n);
    vox.epoch = c.t;
    for (let i = 0; i < vox.n; i++) {
      const [x, y, z] = vox.center(i);
      if (map.roomAt(x, y)?.id === R && z > (map.floorAt(x, y) ?? 0) + 0.3) vox.seen[i] = 1;
    }
    m.addSighting({ label: "cat", x: 1.8, y: 3.0, t: c.t });
    m.addChange({ kind: "obstacle", x: 1.0, y: 2.0, z: 0.4, size: 0.4 }, c.t);
    const f = m.endFlight("Patrolled the Living room.", (c.t += 1000));
    const lr = f.inspection.rooms.find((r) => r.id === R);
    console.log(`      ${f.report}`);
    assert.ok(lr.share > 0.6 && lr.share < 0.95 && lr.unseen.low && /near the floor/.test(lr.unseen.where), `the Living room ${lr.share}, the floor unseen`);
    assert.ok(others.every((o) => f.inspection.rooms.find((r) => r.id === o.id).share === 0));
    assert.match(f.report, /^Flight: \d+ s, \d+\.\d m, .*Patrolled the Living room\. Saw the cat in the Living room near the sofa\. The camera saw \d+% of the Living room \(not near the floor.*\); none of Room 2 and Room 3\. 1 possible change to check: a new obstacle in /);
    assert.equal(m.inspection(null), null);
  },

  async "honest about what the camera saw: space the 3D map doesn't have counts as not seen (told apart); nothing marked this flight is said as not measured, with why"() {
    const { house, map } = await fixture(), c = clock(T0);
    const vox = VoxelMap.forMap(map), R = map.roomAt(1.2, 3.4).id;
    // the 3D map has the rooms' space free from 0.7 m up; below that (near the floor, behind furniture) it doesn't know
    for (let i = 0; i < vox.n; i++) {
      const [x, y, z] = vox.center(i), f = map.floorAt(x, y);
      if (f != null && map.roomAt(x, y) && z > f + 0.7 && z < f + 1.9) vox.set(i, -32, FLAG.CARVED);
    }
    const m = await HouseMemory.open(`${house.id}:honest`, { indexedDB: null, now: c.now });
    m.setHouse({ house, map, vox });
    m.startFlight({ kind: "patrol" });
    fly(m, [[0.0, 0.6], [0.9, 2.2], [1.2, 3.4]], c);
    vox.seen = new Float32Array(vox.n);
    vox.epoch = c.t;
    for (let i = 0; i < vox.n; i++) if (vox.st[i] === FREE && map.roomAt(...vox.center(i).slice(0, 2))?.id === R) vox.seen[i] = 1; // every free voxel of it
    const f = m.endFlight("Patrolled.", (c.t += 1000)), lr = f.inspection.rooms.find((r) => r.id === R);
    console.log(`      ${m.inspectionText(f.inspection)}`);
    assert.ok(f.inspection.measured && lr.share < 0.75 && lr.unmapped > 0.25, `not "100%": ${lr.share} seen, ${lr.unmapped} unmapped`);
    assert.match(f.report, /The camera saw \d+% of the Living room \(\d+% of it isn't in the 3D map; not near the floor/);
    // a flight whose camera picture wasn't trusted (ui/session.js untrusted(): nothing marked seen)
    m.startFlight({ kind: "patrol" }, (c.t += 60000));
    fly(m, [[0.0, 0.6], [0.9, 2.2], [1.2, 3.4]], c);
    m.seenPaused("the pad check hasn't passed yet");
    const g = m.endFlight("Patrolled the Living room.", (c.t += 1000));
    console.log(`      ${g.report}`);
    assert.deepEqual([g.inspection.measured, g.inspection.why], [false, "the pad check hasn't passed yet"]);
    assert.match(g.report, /What the camera saw wasn't measured on this flight \(its picture wasn't trusted: the pad check hasn't passed yet\)\./);
    assert.ok(!/none of/.test(g.report), "never 'the camera saw none of the rooms'");
  },

  async "forgetting every flight takes the changes' obstacles off the map at once and tells whoever shows them"() {
    const { house, map } = await fixture(), c = clock(T0);
    const m = await HouseMemory.open(`${house.id}:forget`, { indexedDB: new FakeIDB(), now: c.now }), heard = [];
    m.setHouse({ house, map });
    m.on("change", (e) => heard.push(e.phase));
    const r = m.addChange({ kind: "obstacle", x: 1.0, y: 2.0, z: 0.4, size: 0.4 });
    assert.ok(map.temps.has(r.id) && !map.free(1.0, 2.0, 1.0));
    await m.clear();
    assert.ok(!map.temps.has(r.id) && map.free(1.0, 2.0, 1.0) && heard.at(-1) === "cleared" && m.openChanges().open === 0);
  },

  async "a memory from before pictures were stored apart (version 1): opened, its pictures move out of the records at once (RAM bounded), its flights know their length (their trails can be forgotten), and forgetting pictures reaches storage"() {
    const idb = new FakeIDB(), key = `${house.id}:v1`, c = clock(T0);
    await new Promise((res) => {
      const rq = idb.open("whoop-memory", 1);
      rq.onupgradeneeded = () => ["flights", "trails", "sightings", "changes", "events"].forEach((s) => rq.result.createObjectStore(s, { keyPath: "key" }));
      rq.onsuccess = res;
    });
    const db = idb.dbs.get("whoop-memory"), put = (store, v) => db.stores.get(store).data.set(v.key, { ...v, house: key, key: v.key });
    const blob = (n) => new Blob([Buffer.from(jpeg(n), "base64")], { type: "image/jpeg" });
    for (let i = 0; i < 12; i++) {
      const id = `f${i}`, t0 = T0 - (12 - i) * H;
      put("flights", { key: `${key}|${id}`, id, kind: "patrol", world: "real", t0, t1: t0 + 30000, rooms: [], distance: 3, seen: 1, summary: "Patrolled.", missions: [] });
      for (let k = 0; k < 3; k++) put("trails", { key: `${key}|${id}|${k}`, flight: id, seq: k, t: t0 + k * 10000, s: Array.from({ length: 20 }, (_, j) => ({ t: t0 + k * 10000 + j * 500, x: 1, y: 2, z: 1, yaw: 0, sigma: 0.05, room: null })) });
      put("sightings", { key: `${key}|s${i}`, id: `s${i}`, flight: id, t0: t0 + 1000, t: t0 + 1000, n: 1, label: i % 2 ? "cat" : "person", x: 1, y: 2, z: 0.5, snapshot: blob(30 + i) });
    }
    put("changes", { key: `${key}|c0`, id: "c0", flight: "f11", kind: "obstacle", x: 1, y: 2, z: 0.4, size: 0.4, status: "suspected", t: T0 - H, last: T0 - H, n: 1, evidence: { live: blob(5), expected: blob(6) } });
    const m = await HouseMemory.open(house.id + ":v1", { indexedDB: idb, now: c.now, ramPictures: 2, ramSamples: 100 });
    await m.flush();
    const stored = (s) => [...db.stores.get(s).data.values()];
    console.log(`      opened: ${m.ram().pictures} pictures in RAM, ${stored("pictures").length} in their own store, ${stored("sightings").filter((r) => r.snapshot).length} left inside records`);
    assert.ok(m.ram().pictures <= 2 + 2 + 2, "RAM bounded at once");
    assert.ok(stored("pictures").length === 13 && stored("sightings").every((r) => !r.snapshot && r.pic) && !stored("changes")[0].evidence);
    assert.ok(m.flights.every((f) => f.samples === 60) && !m.trails.has("f0"));
    assert.equal((await m.picture(m.sightings[0])).size, 34, "read back from the pictures store");
    await m.deleteFlight("f0");
    assert.ok(!stored("trails").some((t) => t.flight === "f0"), "a forgotten flight's trail is gone from storage too");
    await m.clearPictures();
    assert.ok(!stored("pictures").length && stored("sightings").every((r) => !r.snapshot && r.pic === false));
    await m.close();
    const again = await HouseMemory.open(house.id + ":v1", { indexedDB: idb, now: c.now });
    assert.ok(again.sightings.every((s) => !s.snapshot) && !again.lastSeen("person").snapshot && !(await again.picture(again.lastSeen("cat"))), "the pictures stay forgotten");
  },

  async "words: times, spans, missions"() {
    assert.equal(when(T0 - 30e3, T0), "just now");
    assert.equal(when(T0 - 12 * 60e3, T0), "12 min ago");
    assert.equal(when(T0 - 3 * H, T0), "today at 11:00");
    assert.equal(when(T0 - DAY, T0), "yesterday at 14:00");
    assert.equal(when(T0 - 3 * DAY, T0), "on Wednesday at 14:00");
    assert.equal(when(T0 - 20 * DAY, T0), "on 13 Sep at 14:00");
    assert.equal(parseSince("yesterday", T0), Date.UTC(2026, 9, 2));
    assert.equal(parseSince("2 hours", T0), T0 - 2 * H);
    assert.equal(parseSince("last flight", T0, 123), 123);
    assert.equal(parseSince(null, T0), 0);
    assert.equal(parseSince("2026-10-01T00:00:00Z", T0), Date.UTC(2026, 9, 1));
    assert.equal(missionText({ kind: "goTo", target: "the Kitchen" }), "go to the Kitchen");
    assert.equal(missionText({ kind: "goTo", target: { x: 1, y: 2 } }), "go to a point on the map");
    assert.equal(missionText({ kind: "checkOnPets" }), "check on the pets");
    assert.equal(missionText("checkOnPets"), "check on the pets", "a mission's kind alone, in words");
    const named = [], mm = await HouseMemory.open("named-house", { indexedDB: null });
    mm.on("named", (h) => named.push(h?.id ?? null));
    mm.setHouse({ house, map }), mm.setHouse({ house, map }), mm.setHouse({});
    assert.deepEqual(named, [house.id, null], "'named' when the house (its names) arrives or goes, not again for the same house");
    const rid = house.rooms[0].id, rname = house.rooms[0].name, the = /^(room|bedroom|bathroom)\s*\d/i.test(rname) ? rname : `the ${rname}`;
    assert.equal(missionText({ kind: "lookIn", room: rid }, (id) => (id === rid ? rname : id)), `look in ${the}`, "a room's id as the house calls it");
    assert.equal(missionText({ kind: "goTo", target: rid }, (id) => (id === rid ? rname : id)), `go to ${the}`);
    assert.equal(missionText({ kind: "goTo", target: "sofa" }, (id) => (id === rid ? rname : id)), "go to sofa");
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
    console.log(`FAIL  ${name}\n      ${e.message.split("\n").slice(0, 6).join("\n      ")}\n      ${e.stack?.split("\n").find((l) => l.includes("test-memory.mjs:"))?.trim() ?? ""}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed (${((performance.now() - t0) / 1000).toFixed(1)} s)`);
process.exit(failed ? 1 : 0);
