// What the drone remembers about one house (docs/HOME-DRONE.md, "Wave C contracts"): every flight with its trail (2 Hz:
// position, σ, room), what it saw (people, pets, other labels, with a small snapshot), the differences from the 3D scan
// that flights noticed (suspected → confirmed or dismissed; a confirmed one changes the house: a door's `passable`, or a
// keep-out with source "change"), and the other things that happened (alerts, missions). IndexedDB `whoop-memory`, one
// store per kind, everything also held in memory so questions are answered synchronously; without IndexedDB (Node,
// private windows) it lives in memory only.
//   const memory = await HouseMemory.open(house.id);  memory.setHouse({ house, map, vox, save });
//   memory.startFlight({ kind: "sim", mission }); memory.samplePose(localizer.pose()); memory.endFlight("Patrolled.");
//   memory.addSighting({ label: "cat", room, x, y, z, score, snapshot });  memory.lastSeen("the cat");
//   memory.addChange({ kind: "obstacle", x, y, z, size, evidence: { live, expected } }); memory.resolveChange(id, "confirmed");
//   memory.recall("where did you last see the cat?") -> plain text for Claude (deterministic: same records, same words).
// One memory per house and world: HouseMemory.open(id, { world: "sim" }) keeps the simulator's flights, sightings and
// changes apart from the real ones (a simulated cat is never "last seen" for real), and the simulator's confirmed changes
// stay on its map as temporary obstacles: they never edit or save the house. open() hands back the memory already open
// for that house and world (one live flight record), until close().
// Positions are only as good as the localizer: rooms visited, distance flown and "now in" count sure samples (σ under
// MEMORY.sure); a lost or unsure position is said as such ("position lost now; last known in Room 3, 12 s ago",
// "probably in the Kitchen"), and so are sightings with a large σ or within σ of another room. A spot off every room is
// "near" the nearest room (MEMORY.nearRoom) or "outside the mapped rooms", never a room the house doesn't have. Each
// flight record keeps its losses (count) and lostAt (where: the last sample before each), so lists need no trail.
// Changes: memory owns their life. A suspected change that blocks (obstacle, moved, door-closed) is a temporary obstacle
// on the HomeMap (under the change's id) until it is confirmed or dismissed, also after a reload (setHouse puts them back).
// One Claude thinks is a person or pet (passing()) stays an obstacle until MEMORY.passingMs after the camera last had it
// in view (a sure pose sample within MEMORY.view m and MEMORY.viewHalf of the heading: nav/changes.js never reports a
// spot it already reported, and says when the spot looks as the scan expects again: noteUnchanged), after it was last
// reported or after seen(id) (the detector's blobs there), then is dismissed as transient. A report at the spot of a suspected change is that change seen again; one matching
// what is already confirmed there (no later confirmed opposite: door-open after door-closed, gone after obstacle) is that
// confirmation seen again; else it is new. A change the user dismissed is not raised again for the same spot (same kind,
// about the same size) until noteUnchanged() saw that spot as the scan expects it, or the detector's signature differs;
// one Claude dismissed as weak isn't raised again by weak reports in that flight; no other dismissal suppresses anything.
// Claude may only resolve a change in the safe direction: confirm one that adds a blocker (obstacle, door-closed) or
// dismiss one that would free space (gone, door-open), and dismiss a weak suspicion (weak: what nav/changes.js saw of it
// a pose error or a misread could make: mostly explained by the pose's error, just over or under a scanned surface, or at
// a faint layer; a report that isn't weak makes the record not weak): a well-evidenced blocker always waits for the user. noteUnchanged() at a confirmed change raises its opposite (the new
// box is gone again, the closed door is open).
// Freeing (gone, moved, door-open) frees only voxels something marked occupied: unknown space stays unknown.
// Events: "flight" (t1 null: started), "sighting", "change" { ...record, change: record, phase } with phase "suspected"
// (new: the only one worth a Claude check), "seen" (reported again), "merged" ({ id: the detector's id, into }),
// "suppressed" (a dismissed spot; addChange returns null), "confirmed", "dismissed"; "annotate" (Claude's notes on a
// change: never "change", so a listener can't loop on it); and "house" ({ house, change, rebuild, saved }: the house was
// edited and is being saved (saved: a promise); rebuild: a door changed, so the HomeMap must be built again from the
// house once landed (needsRebuild stays true until setHouse gets a new map): until then a closed door is a temporary
// obstacle on the current map); "named" (setHouse: the rooms' and doorways' names to word the records with).
import { Emitter } from "../util.js";

export const MEMORY = {
  db: "whoop-memory",
  trailMs: 500, // 2 Hz
  chunk: 20, // trail samples per stored record (10 s)
  sightingWindow: 15000, // ms: the same label at the same spot (or track) within this is one sighting
  near: 1.0, // m
  changeNear: 0.5, // m: changes closer than this (or half their size) are the same spot
  keepDays: 90,
  keepFlights: 200,
  keepSightings: 5000,
  keepEvents: 2000,
  keepResolvedDays: 365, // confirmed and dismissed changes (dismissals are what stops a change being raised again)
  snapshotMax: 200000, // bytes: larger snapshots aren't kept
  sure: 0.25, // m: a position with σ under this counts for rooms visited, distance flown and plain "in" (else "probably in")
  passingMs: 60000, // a change Claude takes for a person or pet blocks this long after it was last in view, then is dismissed
  view: 5, viewHalf: 0.9, // m, rad: the camera has a spot in view (as far as nav/changes.js compares depth; ~100° across)
  // A new report is an open suspected change of the same kind when their outlines are within twice the larger placement
  // error (σ, at least placeErr) and the record, grown to hold every report, stays within maxReach of its middle. The
  // floor is sim-tuned: placed boxes came out 0.19-0.37 m off in tools/depth-check.html, with σ under 0.15 (re-measure
  // on the first real flights).
  placeErr: 0.25, maxReach: 0.75, // m
  staleView: 10000, // ms: a suspected change the camera had in plain view this long in a flight (trusted frames: unseen())
  // without seeing it again stops blocking (seen again: blocks again; off view or untrusted, nothing counts)
  ramPictures: 60, // snapshots and evidence pictures held in RAM besides the newest per label and the open changes'
  ramSamples: 20000, // trail samples held in RAM (about 40 four-minute flights); older trails are read back when asked for
  inspectStep: 0.25, inspectHeights: [0.15, 0.5, 0.9, 1.3, 1.7], // m: the lattice a room's inspection is measured on
  nearRoom: 1.5, // m: a spot off every room this close to one is "near" it, else "outside the mapped rooms"
  lostSpots: 20, // where the position was lost, kept per flight (f.lostAt; f.losses counts them all)
};
const OFF_ROOMS = "outside the mapped rooms";
const STORES = ["flights", "trails", "sightings", "changes", "events", "pictures"];
const RECORDS = STORES.slice(0, 5); // read whole at open; pictures one by one (picture())
const PIC = { sightings: "snapshot", changes: "evidence" }; // the field each store keeps its pictures in (stored apart)
const DAY = 864e5;
const BLOCKS = new Set(["obstacle", "moved", "door-closed"]);
const ADDS = new Set(["obstacle", "door-closed"]); // blocks something new and frees nothing: Claude may confirm these
const family = (kind) => (kind.startsWith("door") ? "door" : "thing");
const OPPOSITE = { obstacle: "gone", moved: "gone", gone: "obstacle", "door-closed": "door-open", "door-open": "door-closed" };
// a dismissal that stops report c being raised again: the user's; Claude's of a weak suspicion, for weak reports in its flight
const suppresses = (r, c, flight) => r.status === "dismissed" && !r.transient && (r.by === "user" || (r.by === "claude" && r.weak && !!c.weak && !!flight && r.flight === flight));
const r2 = (v) => (Number.isFinite(v) ? Math.round(v * 100) / 100 : null);
const r3 = (v) => (Number.isFinite(v) ? Math.round(v * 1000) / 1000 : null);
// At most n characters, cut after the last whole sentence (else a word, with "…"): a cut mid-sentence read "(about 114 s
// of flying left;." in the flight's report.
export const clip = (t, n) => {
  const s = String(t);
  if (s.length <= n) return s;
  const c = s.slice(0, n - 1), e = c.lastIndexOf(". ");
  return e > n / 2 ? c.slice(0, e + 1) : `${c.slice(0, Math.max(1, c.lastIndexOf(" "))).replace(/[,;:(\s]+$/, "")}…`;
};
const norm = (t) => String(t ?? "").toLowerCase().replace(/[^a-z0-9 ]/g, " ").replace(/\s+/g, " ").trim();
const theRoom = (name) => (/^(room|bedroom|bathroom)\s*\d/i.test(name) ? name : `the ${name}`);
const plural = (n, w, ws = `${w}s`) => `${n} ${n === 1 ? w : ws}`;
const MISSION_WORDS = { goTo: "go to", lookIn: "look in", searchFor: "search for", checkOnPets: "check on the pets", returnHome: "return home", patrol: "patrol" };
// A mission object (missions.js) or text -> a few words: "go to the Kitchen", "patrol".
export function missionText(m, room = (id) => id) {
  if (!m) return null;
  if (typeof m !== "object") return MISSION_WORDS[m] ?? String(m).slice(0, 200);
  const named = (id) => (room(id) !== id ? theRoom(room(id)) : id); // a room's id ("r3") as the house calls it ("the Office")
  const what = typeof m.target === "string" ? named(m.target) : m.room ? named(m.room) : m.rooms?.length ? m.rooms.map(named).join(", ") : m.target ? "a point on the map" : "";
  return `${MISSION_WORDS[m.kind] ?? m.kind ?? "mission"}${what ? ` ${what}` : ""}`;
}

// What people call the detector's labels.
const LABELS = {
  person: new RegExp(`^(${["person|persons|people|someone|somebody|anyone|anybody|everyone|human|humans|man|woman|men|women|kid|kids|child|children",
    "intruder|intruders|visitor|visitors|guest|guests|stranger|strangers|burglar|wife|husband|partner|son|daughter|mom|mum|dad|mother|father",
    "baby|family|boyfriend|girlfriend|roommate|flatmate|everybody|nobody|no one|folks|teen|teens|teenager|teenagers|grandma|grandpa|nanny|babysitter"].join("|")})$`),
  cat: /^(cat|cats|kitty|kitten|kittens)$/,
  dog: /^(dog|dogs|pup|pups|puppy|puppies|doggy|doggo|doggie)$/,
};
export function labelOf(what) {
  const w = norm(what).replace(/^(the|a|an|my|our|any|some) /, "");
  for (const [label, re] of Object.entries(LABELS)) if (re.test(w)) return label;
  return w.replace(/(?<=[a-z]{2}[^s])s$/, "");
}
const PETS = /^(?:the |my |our |any )?(pets?|animals?)$/;
const OWNED = /^(?:bed|beds|bowl|bowls|food|toy|toys|door|flap|box|cage|crate|kennel|house|leash|lead|collar|tree|basket|dish|water|gate|hair|fur|room|rooms)$/; // "the dog ___": a thing, not the dog
// What a question asks about, from the words after its verb: the room (room: the words naming it) and the words around a
// subject taken off, also when nothing else is left ("the cat last seen" -> "cat", "anyone home" -> "anyone", "home" -> "",
// "anyone in the house right now" -> "anyone": the whole house is no room).
const TRAIL = new RegExp(`(?:^| )(?:${["last|lately|recently|anywhere|before|earlier|at all|seen|spotted|noticed|found|now|right now|at the moment|currently",
  "home|at home|around|about|here|in here|around here|there|inside|indoors|again|go|gone|went|going|been|come|came|back",
  "(?:in|at|around|inside) (?:the|my|our) (?:house|home|place|flat|apartment)"].join("|")})$`);
function subjectOf(said, room) {
  let s = (room ? said.replace(new RegExp(`\\s*\\b(?:(?:in|at|inside|near|by) )?(?:the )?${room}\\b.*$`), "") : said).replace(/^(?:in the|in|the|a|an) /, "").trim();
  for (let t = s.replace(TRAIL, ""); t !== s; t = s.replace(TRAIL, "")) s = t;
  return s;
}

// What people call rooms, against a room's name or the kind a survey gave it (rooms[].kind).
const ROOM_WORDS = {
  "living room": /\b(living ?room|lounge|family room|front room|sitting room|den)\b/, kitchen: /\b(kitchen|kitchenette)\b/,
  "dining room": /\bdining\b/, bedroom: /\b(bed ?room|master)\b/, bathroom: /\b(bath ?room|toilet|loo|wc|washroom|restroom)\b/,
  hallway: /\b(hall|hallway|corridor|landing)\b/, entrance: /\b(entrance|entry|foyer|porch)\b/, office: /\b(office|study)\b/,
  laundry: /\b(laundry|utility room)\b/, "kids room": /\b(kids room|nursery|playroom)\b/,
};

// A time window in a question: "today", "yesterday", "this morning", "this afternoon", "this evening", "last night",
// "this week", "in the last 2 hours" -> { from, to, words, re } (local time) or null.
const NUM = { a: 1, an: 1, one: 1, two: 2, three: 3, four: 4, five: 5, six: 6, ten: 10, twelve: 12 };
export function windowOf(q, now = Date.now()) {
  const m0 = midnight(now), H = 36e5;
  const fixed = [[/\bthis morning\b/, m0, m0 + 12 * H], [/\bthis afternoon\b/, m0 + 12 * H, m0 + 18 * H], [/\b(this evening|tonight)\b/, m0 + 18 * H, m0 + DAY],
    [/\b(last night|overnight)\b/, m0 - 6 * H, m0 + 6 * H], [/\btoday\b/, m0, m0 + DAY], [/\byesterday\b/, m0 - DAY, m0], [/\b(this|past|last) week\b/, now - 7 * DAY, now + 1]];
  for (const [re, from, to] of fixed) {
    const m = q.match(re);
    if (m) return { from, to, words: m[0], re };
  }
  const m = q.match(/\b(?:in the )?(?:last|past) (?:(\d+|an?|one|two|three|four|five|six|ten|twelve) )?(hours?|minutes?|mins?)\b/);
  if (!m) return null;
  const n = m[1] ? NUM[m[1]] ?? +m[1] : 1, ms = n * (m[2].startsWith("h") ? H : 6e4);
  return { from: now - ms, to: now + 1, words: `in the last ${n === 1 ? (m[2].startsWith("h") ? "hour" : "minute") : `${n} ${m[2].startsWith("h") ? "hours" : "minutes"}`}`, re: new RegExp(m[0]) };
}

// "12 min ago", "today at 14:05", "yesterday at 09:30", "on Monday at 18:00", "on 2 Oct at 08:15" (local time).
const DAYS = ["Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"];
const MONTHS = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
const midnight = (t) => new Date(new Date(t).setHours(0, 0, 0, 0)).getTime();
export function when(t, now = Date.now()) {
  const s = (now - t) / 1000, d = new Date(t), hm = `${String(d.getHours()).padStart(2, "0")}:${String(d.getMinutes()).padStart(2, "0")}`;
  if (s < 60) return "just now";
  if (s < 3600) return `${Math.round(s / 60)} min ago`;
  const days = Math.round((midnight(now) - midnight(t)) / DAY);
  if (days === 0) return `today at ${hm}`;
  if (days === 1) return `yesterday at ${hm}`;
  if (days < 7) return `on ${DAYS[d.getDay()]} at ${hm}`;
  return `on ${d.getDate()} ${MONTHS[d.getMonth()]} at ${hm}`;
}
export const duration = (s) => (s < 60 ? `${Math.round(s)} s` : s < 3600 ? `${Math.floor(s / 60)} min${Math.round(s % 60) ? ` ${Math.round(s % 60)} s` : ""}` : `${(s / 3600).toFixed(1)} h`);

// "yesterday", "today", "last flight", "3 hours", "2 days ago", "this week", an ISO date or epoch ms -> epoch ms (0: all).
export function parseSince(since, now = Date.now(), lastFlightStart = 0) {
  if (since == null || since === "") return 0;
  if (Number.isFinite(+since) && +since > 1e12) return +since;
  const s = norm(since);
  if (!s || /^(ever|always|all|anything|the beginning|start)$/.test(s)) return 0;
  if (/\b(last|previous|this) (flight|time|sortie|mission)\b/.test(s)) return lastFlightStart;
  if (/\btoday\b/.test(s)) return midnight(now);
  if (/\byesterday\b/.test(s)) return midnight(now) - DAY;
  if (/\b(this|last|past) week\b/.test(s)) return now - 7 * DAY;
  if (/\b(this|last|past) month\b/.test(s)) return now - 30 * DAY;
  const m = s.match(/(\d+(?:\.\d+)?|an?|one|two|three|four|five|six|seven|ten) ?(min|minute|h|hr|hour|d|day|week)s?\b/);
  if (m) {
    const n = { a: 1, an: 1, one: 1, two: 2, three: 3, four: 4, five: 5, six: 6, seven: 7, ten: 10 }[m[1]] ?? +m[1];
    return now - n * { min: 6e4, minute: 6e4, h: 36e5, hr: 36e5, hour: 36e5, d: DAY, day: DAY, week: 7 * DAY }[m[2]];
  }
  const d = Date.parse(since);
  return Number.isFinite(d) ? d : 0;
}

// A snapshot as a small JPEG Blob: base64 (perception.snapshot()), a data URL or a Blob. Anything else, or too big: null.
function jpegBlob(img, max = MEMORY.snapshotMax) {
  if (!img) return null;
  if (typeof Blob !== "undefined" && img instanceof Blob) return img.size <= max ? img : null;
  if (typeof img !== "string") return null;
  const b64 = img.startsWith("data:") ? img.slice(img.indexOf(",") + 1) : img;
  if (b64.length * 0.75 > max || !/^[A-Za-z0-9+/]+=*$/.test(b64.slice(0, 64))) return null;
  return new Blob([Uint8Array.from(atob(b64), (c) => c.charCodeAt(0))], { type: "image/jpeg" });
}

const done = (tx) => new Promise((resolve, reject) => {
  tx.oncomplete = () => resolve(true);
  tx.onerror = tx.onabort = () => reject(tx.error ?? new Error("memory write failed"));
});
const result = (rq) => new Promise((resolve, reject) => {
  rq.onsuccess = () => resolve(rq.result);
  rq.onerror = () => reject(rq.error);
});
function openDb(idb, name) {
  if (!idb) return Promise.resolve(null);
  return new Promise((resolve) => {
    const rq = idb.open(name, 2); // 2: pictures apart from their records
    rq.onupgradeneeded = () => {
      for (const s of STORES) if (!rq.result.objectStoreNames.contains(s)) rq.result.createObjectStore(s, { keyPath: "key" });
    };
    rq.onsuccess = () => resolve(rq.result);
    rq.onerror = rq.onblocked = () => resolve(null);
  });
}

export class HouseMemory extends Emitter {
  static registry = new WeakMap(); // indexedDB -> Map(house key -> Promise<HouseMemory>): one live memory per house and world

  // options: indexedDB (a stand-in, or null for memory only: never shared), world ("real" | "sim"), now (ms clock),
  // keepDays, keepFlights, ...MEMORY. The memory already open for this house and world comes back as it is.
  static open(houseId, { indexedDB = globalThis.indexedDB, world = "real", ...o } = {}) {
    const key = world === "sim" ? `${houseId}:sim` : houseId;
    const reg = indexedDB && (HouseMemory.registry.get(indexedDB) ?? HouseMemory.registry.set(indexedDB, new Map()).get(indexedDB));
    if (reg?.has(key)) return reg.get(key);
    const p = HouseMemory.load(houseId, { indexedDB, world, ...o });
    reg?.set(key, p);
    p.catch(() => reg?.delete(key));
    return p;
  }

  static async load(houseId, { indexedDB, world, now = Date.now, ...o }) {
    const m = new HouseMemory(houseId, { now, world, ...o }), key = m.key;
    m.idb = indexedDB;
    m.db = await openDb(indexedDB, m.o.db).catch(() => null);
    if (m.db) {
      const tx = m.db.transaction(RECORDS, "readonly");
      const all = await Promise.all(RECORDS.map((s) => result(tx.objectStore(s).getAll())));
      const mine = all.map((list) => list.filter((r) => r.house === key));
      [m.flights, , m.sightings, m.changes, m.events] = mine.map((l) => l.sort((a, b) => (a.t0 ?? a.t) - (b.t0 ?? b.t)));
      for (const c of mine[1].sort((a, b) => a.seq - b.seq)) (m.trails.get(c.flight) ?? m.trails.set(c.flight, []).get(c.flight)).push(...c.s);
      // A flight still open from a page that closed mid-flight ends at its last sample.
      for (const f of m.flights) if (f.t1 == null) Object.assign(f, { t1: m.trails.get(f.id)?.at(-1)?.t ?? f.t0, summary: f.summary ?? "The app closed during this flight." }), m.save("flights", f);
      for (const f of m.flights) if (!f.roomSecs || f.samples == null) (m.indexFlight(f, m.trails.get(f.id) ?? []), (f.samples = m.trails.get(f.id)?.length ?? 0), m.save("flights", f));
      for (const f of m.flights) if (f.losses == null) (m.lossesOf(f, m.trails.get(f.id) ?? []), m.save("flights", f));
      // pictures stored inside their records (before version 2) move to the pictures store now
      for (const [store, list] of [["sightings", m.sightings], ["changes", m.changes]])
        for (const r of list) if (r.pic == null && (r.snapshot || r.evidence?.live)) (m.dirty.add(r), m.save(store, r));
    }
    m.seq = m.flights.length + m.sightings.length + m.changes.length + m.events.length;
    m.prune();
    m.trimTrails();
    m.trimPictures();
    await m.fetchPictures();
    return m;
  }

  constructor(houseId, { now = Date.now, world = "real", ...o } = {}) {
    super();
    this.houseId = houseId;
    this.world = world === "sim" ? "sim" : "real";
    this.key = this.world === "sim" ? `${houseId}:sim` : houseId; // the records' house field and storage key prefix
    this.now = now;
    this.o = { ...MEMORY, ...o };
    Object.assign(this, { db: null, idb: null, flights: [], trails: new Map(), sightings: [], changes: [], events: [], flight: null, seq: 0 });
    Object.assign(this, { house: null, map: null, vox: null, saveHouse: null, saved: Promise.resolve() });
    this.onMap = new Map(); // id -> what it is (kind|until) of the temporary obstacles this put on this.map
    this.staleDoors = new Map(); // door id -> the change that closed it, while this.map was built before it
    this.queue = new Map(); // "store|key" -> record | null (delete), written together on the next microtask
    this.writing = Promise.resolve();
    this.dirty = new WeakSet(); // records whose picture changed since it was stored
  }

  // false: IndexedDB couldn't open (a private window, storage blocked): this memory lasts only until the page closes.
  get stored() {
    return !!this.db;
  }

  // Stored and closed; the next open() reads it again.
  async close() {
    await this.flush();
    const reg = this.idb && HouseMemory.registry.get(this.idb);
    if (reg && (await reg.get(this.key)?.catch(() => null)) === this) reg.delete(this.key);
    this.setHouse({});
    this.db?.close?.();
    this.db = null;
  }

  // The house the changes apply to (and the names come from): its HomeMap (finalized again after a keep-out changes; the
  // suspected changes that block go on it as temporary obstacles), the VoxelMap and save(house) to store it. A new map
  // (built from the house as it is now) ends needsRebuild. setHouse({}) takes this memory's obstacles off the map.
  setHouse({ house = null, map = null, vox = null, save = null } = {}) {
    if (map !== this.map) {
      for (const id of this.onMap.keys()) this.map?.removeTemp?.(id);
      this.onMap.clear();
      this.staleDoors.clear();
    }
    const named = house !== this.house;
    Object.assign(this, { house, map, vox, saveHouse: save });
    this.expirePassing();
    this.syncTemps();
    if (named) this.emit("named", house);
  }

  // A door changed since the map was built: build the HomeMap again from the house (after landing).
  get needsRebuild() {
    return this.staleDoors.size > 0;
  }

  // The temporary obstacles on the map: every suspected change that blocks (one Claude takes for a person or pet as a
  // person, until expirePassing() dismisses it), doors closed since the map was built, and in the simulator its confirmed
  // blockers.
  syncTemps(t = this.now()) {
    const map = this.map;
    if (!map?.addTemp) return;
    const want = new Map();
    for (const r of this.changes) {
      if (r.status !== "suspected" || !BLOCKS.has(r.kind)) continue;
      const stale = !this.blocking(r);
      if (stale !== !!r.stale) (r.stale = stale), this.emit("annotate", r);
      if (!stale) want.set(r.id, r);
    }
    if (this.world === "sim") for (const r of this.changes) if (r.status === "confirmed" && BLOCKS.has(r.kind) && this.confirmedAt(r) === r) want.set(r.id, r);
    for (const [door, r] of this.staleDoors) want.set(`door:${door}`, { ...r, door });
    const temps = new Map([...want].map(([id, r]) => {
      const tp = this.tempOf(r);
      return [id, tp && (r.status === "suspected" && r.passing ? { ...tp, kind: "person" } : tp)];
    }));
    const sig = (tp) => `${tp.kind}|${tp.until}|${tp.x}|${tp.y}|${tp.r}`;
    for (const [id, s] of this.onMap) if (!temps.get(id) || sig(temps.get(id)) !== s) (map.removeTemp(id), this.onMap.delete(id));
    for (const [id, tp] of temps) if (tp && !this.onMap.has(id)) (map.addTemp({ ...tp, id }), this.onMap.set(id, sig(tp)));
  }

  // Does a suspected change block the way now? Not once the camera had its spot in plain view for staleView of the running
  // flight without seeing it again (the detector's unseen(): trusted frames only): one false report must not close the
  // way home for the rest of a sortie; seen again, it blocks again. On the ground, and off view, it blocks.
  blocking(r) {
    return !(this.flight && !r.passing && (r.unseen ?? 0) >= this.o.staleView);
  }
  // Open suspected changes for the HUD and the change cards: { open, blocking, stale, people } counts.
  openChanges() {
    const open = this.changes.filter((c) => c.status === "suspected"), block = open.filter((c) => BLOCKS.has(c.kind) && this.blocking(c));
    return { open: open.length, blocking: block.length, stale: open.filter((c) => BLOCKS.has(c.kind) && !this.blocking(c)).length, people: open.filter((c) => c.passing).length };
  }
  // The detector had the spot of suspected change `id` in plain view for `ms` on a trusted frame and saw nothing there.
  unseen(id, ms, t = this.now()) {
    const r = this.changes.find((c) => c.id === id);
    if (!r || r.status !== "suspected" || !this.flight) return null;
    const was = this.blocking(r);
    r.unseen = (r.unseen ?? 0) + Math.max(0, ms);
    if (was !== this.blocking(r)) this.syncTemps(t);
    return r;
  }

  // How sure a change's place is (m): its σ, at least placeErr.
  err(r) {
    return Math.max(r.sigma ?? 0, this.o.placeErr);
  }
  // Every report of a thing as [x, y, half size] (one record holds them all after merges).
  spotsOf(r) {
    return r.spots ?? [[r.x, r.y, (r.size ?? 0.3) / 2]];
  }
  // How far from its place a thing's reports reach (at least its half size).
  reach(r) {
    return Math.max((r.size ?? 0.3) / 2, ...this.spotsOf(r).map(([x, y, s]) => Math.hypot(x - r.x, y - r.y) + s));
  }

  // A doorway: the door's line, 0.15 m (and its depth) to either side; anything else: a disc around every report of it,
  // its place's error twice over and 0.1 m more.
  tempOf(r) {
    const d = r.kind.startsWith("door") ? this.doorOf(r) : null;
    if (d) {
      const L = Math.hypot(d.b[0] - d.a[0], d.b[1] - d.a[1]) || 1, n = [-(d.b[1] - d.a[1]) / L, (d.b[0] - d.a[0]) / L], w = (d.depth ?? 0) + 0.15;
      return { kind: "change", change: r.kind, polygon: [[d.a, w], [d.b, w], [d.b, -w], [d.a, -w]].map(([p, s]) => [p[0] + s * n[0], p[1] + s * n[1]]), until: Infinity };
    }
    if (!Number.isFinite(r.x)) return null;
    return { kind: "change", change: r.kind, x: r.x, y: r.y, r: r2(Math.max(0.15, this.reach(r) + 0.1 + 2 * this.err(r))), until: Infinity,
      ...(r.zMin != null && r.zMax != null && { zMin: r.zMin - 0.1, zMax: r.zMax + 0.1 }) };
  }

  doorOf(c) {
    const doors = this.house?.doors ?? [];
    return doors.find((q) => q.id === c.door) ?? nearestDoor(doors, c.x, c.y, 1.2);
  }

  // ---- storage ----

  id(prefix, t) {
    return `${prefix}${t.toString(36)}${(this.seq++).toString(36)}`;
  }
  // A record as it is now (a copy goes to storage); its picture (PIC) apart, under "<key>|pic", when it changed (setPic).
  save(store, rec) {
    rec.key ??= `${this.key}|${rec.id ?? `${rec.flight}|${rec.seq}`}`;
    rec.house = this.key;
    const k = PIC[store];
    if (!k) return this.write(store, rec.key, rec);
    if (this.dirty.has(rec)) {
      this.dirty.delete(rec);
      rec.pic = !!rec[k];
      this.write("pictures", `${rec.key}|pic`, rec.pic ? { key: `${rec.key}|pic`, house: this.key, pic: rec[k] } : null);
    }
    this.write(store, rec.key, { ...rec, [k]: null });
  }
  setPic(rec, k, v) {
    rec[k] = v;
    this.dirty.add(rec);
  }
  // A sighting's snapshot or a change's evidence ({ live, expected }), from RAM or storage (older ones live only there).
  async picture(rec) {
    const k = rec && (rec.label ? "snapshot" : "evidence");
    if (!rec || rec[k] || !rec.pic || !this.db) return rec?.[k] ?? null;
    const got = await result(this.db.transaction(["pictures"], "readonly").objectStore("pictures").get(`${rec.key}|pic`)).catch(() => null);
    return got?.pic ?? null;
  }
  // Pictures held in RAM: the newest sighting's per label, the open changes', and the newest ramPictures others; the rest
  // stay in storage only (without storage, up to four times as many, then gone).
  keepPictures() {
    const newest = new Set([...new Map(this.sightings.map((s) => [s.label, s])).values()]), keep = new Set([...newest, ...this.changes.filter((c) => c.status === "suspected")]);
    const rest = [...this.sightings, ...this.changes].filter((r) => !keep.has(r)).sort((a, b) => b.t - a.t);
    rest.slice(0, this.o.ramPictures).forEach((r) => keep.add(r));
    return { keep, drop: rest.slice(this.db ? this.o.ramPictures : 4 * this.o.ramPictures) };
  }
  trimPictures() {
    for (const r of this.keepPictures().drop) {
      const k = r.label ? "snapshot" : "evidence";
      if (r[k] && (!this.db || !this.dirty.has(r))) r[k] = null;
    }
  }
  async fetchPictures() {
    if (!this.db) return;
    const want = [...this.keepPictures().keep].filter((r) => r.pic && !r[r.label ? "snapshot" : "evidence"]);
    await Promise.all(want.map(async (r) => (r[r.label ? "snapshot" : "evidence"] = await this.picture(r))));
  }
  // Trails held in RAM: the running flight's and the newest flights' up to ramSamples; older ones are read back (trailOf).
  trimTrails() {
    let n = 0;
    for (const f of [...this.flights].reverse()) {
      const tr = this.trails.get(f.id);
      if (!tr || f === this.flight) continue;
      if ((n += tr.length) > (this.db ? 1 : 4) * this.o.ramSamples) this.trails.delete(f.id);
    }
  }
  // One flight's chunks are keyed "<house>|<flight>|<n>" (save()): one key range of the store, the per-flight index (a new
  // IndexedDB index would bump the database's version, which another tab holding it open blocks).
  async trailOf(flightId) {
    if (this.trails.has(flightId) || !this.db) return this.trails.get(flightId) ?? [];
    const at = `${this.key}|${flightId}|`, range = IDBKeyRange.bound(at, `${at}\uffff`);
    const chunks = await result(this.db.transaction(["trails"], "readonly").objectStore("trails").getAll(range)).catch(() => []);
    return chunks.sort((a, b) => a.seq - b.seq).flatMap((c) => c.s);
  }
  // What this memory holds in RAM: trail samples, pictures and their bytes.
  ram() {
    let samples = 0, pictures = 0, bytes = 0;
    for (const tr of this.trails.values()) samples += tr.length;
    for (const r of [...this.sightings, ...this.changes])
      for (const b of [r.snapshot, r.evidence?.live, r.evidence?.expected]) if (b) (pictures++, (bytes += b.size ?? 0));
    return { samples, pictures, bytes, flights: this.flights.length, sightings: this.sightings.length, changes: this.changes.length };
  }
  drop(store, key) {
    this.write(store, key, null);
  }
  write(store, key, rec) {
    if (!this.db) return;
    const first = !this.queue.size;
    this.queue.set(`${store}|${key}`, { store, key, rec });
    if (first) this.writing = this.writing.then(() => this.commit());
  }
  async commit() {
    const ops = [...this.queue.values()];
    this.queue.clear();
    if (!ops.length) return;
    try {
      const tx = this.db.transaction([...new Set(ops.map((o) => o.store))], "readwrite");
      for (const { store, key, rec } of ops) rec ? tx.objectStore(store).put(rec) : tx.objectStore(store).delete(key);
      await done(tx);
    } catch (e) {
      this.emit("error", `Couldn't save the flight memory: ${e?.message ?? e}`);
    }
  }
  // Resolves when everything so far is stored.
  flush() {
    return this.writing;
  }

  // Keep keepDays and at most keepFlights flights (their trails and sightings go with them), keepSightings sightings,
  // keepEvents events; suspected changes for keepDays, resolved ones for keepResolvedDays.
  prune(now = this.now()) {
    const { keepDays, keepFlights, keepSightings, keepEvents, keepResolvedDays } = this.o, old = now - keepDays * DAY;
    const keep = new Set(this.flights.filter((f) => f.t0 >= old || f === this.flight).slice(-keepFlights).map((f) => f.id));
    if (this.flight) keep.add(this.flight.id);
    for (const f of this.flights.filter((f) => !keep.has(f.id))) {
      this.drop("flights", f.key);
      const n = Math.ceil((this.trails.get(f.id)?.length ?? f.samples ?? 0) / this.o.chunk);
      for (let i = 0; i < n; i++) this.drop("trails", `${this.key}|${f.id}|${i}`);
      this.trails.delete(f.id);
    }
    this.flights = this.flights.filter((f) => keep.has(f.id));
    const cut = (list, store, ok, max = Infinity) => {
      const kept = list.filter(ok).slice(-max), set = new Set(kept);
      for (const r of list) if (!set.has(r)) (this.drop(store, r.key), r.pic && this.drop("pictures", `${r.key}|pic`));
      return kept;
    };
    this.sightings = cut(this.sightings, "sightings", (s) => s.t >= old && (s.flight === "ground" || keep.has(s.flight)), keepSightings);
    this.events = cut(this.events, "events", (e) => e.t >= old, keepEvents);
    this.changes = cut(this.changes, "changes", (c) => (c.status === "suspected" ? c.last >= old : (c.resolvedAt ?? c.last) >= now - keepResolvedDays * DAY));
    this.syncTemps();
  }

  // Settings → Memory: keep days and flights (settings memoryDays, memoryFlights), pruned at once.
  setRetention({ keepDays = this.o.keepDays, keepFlights = this.o.keepFlights } = {}) {
    Object.assign(this.o, { keepDays: Math.max(1, +keepDays || MEMORY.keepDays), keepFlights: Math.max(1, Math.round(+keepFlights || MEMORY.keepFlights)) });
    this.prune();
    return this.flush();
  }

  // One flight forgotten: its trail, its sightings and their pictures (not the running one).
  async deleteFlight(id) {
    const f = this.flights.find((q) => q.id === id);
    if (!f || f === this.flight) return false;
    this.drop("flights", f.key);
    for (let i = 0; i < Math.ceil((this.trails.get(f.id)?.length ?? f.samples ?? 0) / this.o.chunk); i++) this.drop("trails", `${this.key}|${f.id}|${i}`);
    for (const s of this.sightings.filter((s) => s.flight === id)) this.drop("sightings", s.key), s.pic && this.drop("pictures", `${s.key}|pic`);
    this.flights = this.flights.filter((q) => q !== f);
    this.sightings = this.sightings.filter((s) => s.flight !== id);
    this.trails.delete(id);
    await this.flush();
    return true;
  }

  // Every picture forgotten (snapshots of people and pets, change evidence); the records stay, without them.
  async clearPictures() {
    for (const [store, list] of [["sightings", this.sightings], ["changes", this.changes]])
      for (const r of list) if (r.pic !== false || r[PIC[store]]) (this.setPic(r, PIC[store], null), this.save(store, r));
    await this.flush();
  }

  async clear() {
    for (const [store, list] of [["flights", this.flights], ["sightings", this.sightings], ["changes", this.changes], ["events", this.events]])
      for (const r of list) (this.drop(store, r.key), r.pic && this.drop("pictures", `${r.key}|pic`));
    for (const f of this.flights) for (let i = 0; i < Math.ceil((this.trails.get(f.id)?.length ?? f.samples ?? 0) / this.o.chunk); i++) this.drop("trails", `${this.key}|${f.id}|${i}`);
    Object.assign(this, { flights: [], sightings: [], changes: [], events: [], flight: null, run: null });
    this.trails.clear();
    this.syncTemps(); // the changes' obstacles off the map (doors the house now has closed stay until it is rebuilt)
    this.emit("change", { phase: "cleared", change: null });
    await this.flush();
  }

  // ---- names ----

  roomName(id) {
    return this.house?.rooms?.find((r) => r.id === id)?.name ?? (id ? String(id) : OFF_ROOMS);
  }
  // "in the Kitchen"; off every room "near the Kitchen" (the nearest within nearRoom m of x, y) or "outside the mapped rooms".
  placeText(room, x, y) {
    if (room) return `in ${theRoom(this.roomName(room))}`;
    let near = null, bd = this.o.nearRoom;
    if (Number.isFinite(x)) for (const r of this.house?.rooms ?? []) {
      const d = r.outline?.length > 2 ? outlineDist(x, y, r.outline) : Infinity;
      if (d < bd) [near, bd] = [r, d];
    }
    return near ? `near ${theRoom(near.name)}` : OFF_ROOMS;
  }
  roomAt(x, y) {
    return Number.isFinite(x) ? this.map?.roomAt?.(x, y)?.id ?? null : null;
  }
  nearby(x, y, within = 1.5) {
    let best = null, bd = within;
    for (const l of this.house?.landmarks ?? []) {
      const d = Math.hypot(l.x - x, l.y - y);
      if (d < bd && l.name.length <= 30) [best, bd] = [l.name.toLowerCase(), d];
    }
    return best;
  }
  // "in the Kitchen near the sofa" (off every room "near the Kitchen by the sofa", "outside the mapped rooms near the
  // sofa"), or "probably ..." when the position was unsure (σ: the drone's, for a sighting) or within σ of another room (a
  // change's σ is how well its place is known: only this).
  where(r) {
    const near = r.near ?? (Number.isFinite(r.x) && this.nearby(r.x, r.y)), place = this.placeText(r.room, r.x, r.y);
    return `${this.unsure(r) ? "probably " : ""}${place}${near ? ` ${r.room || place === OFF_ROOMS ? "near" : "by"} the ${near}` : ""}`;
  }
  unsure(r) {
    const s = r.sigma ?? 0;
    if (s >= this.o.sure && !r.kind) return true;
    if (!(s > 0) || !Number.isFinite(r.x) || !this.map) return false;
    return [[1, 0], [-1, 0], [0, 1], [0, -1]].some(([a, b]) => {
      const q = this.roomAt(r.x + a * s, r.y + b * s);
      return q && q !== r.room;
    });
  }

  // ---- flights and trails ----

  // kind: what started it ("patrol", "manual", ...; "sim" | "real" also fine); mission: what it was for (text), if known.
  // A running flight ends first. The memory's world goes on the record.
  startFlight({ kind = "flight", mission = null } = {}, t = this.now()) {
    if (this.flight) this.endFlight(null, t);
    const f = (this.flight = { id: this.id("f", t), kind, world: this.world, mission: missionText(mission, (id) => this.roomName(id)), missions: [], t0: t, t1: null, rooms: [], distance: 0, seen: 0, summary: null,
      roomSecs: {}, roomLast: {}, near: {}, losses: 0, lostAt: [], inspection: null, report: null });
    this.flights.push(f);
    this.trails.set(f.id, []);
    this.save("flights", f);
    for (const c of this.changes) if (c.status === "suspected") c.unseen = 0;
    this.syncTemps(t);
    this.emit("flight", f);
    return f;
  }

  endFlight(summary = null, t = this.now()) {
    const f = this.flight;
    if (!f) return null;
    this.flight = null;
    const trail = this.trails.get(f.id);
    this.settle(f);
    if (trail.length % this.o.chunk) this.saveChunk(f, Math.floor(trail.length / this.o.chunk));
    Object.assign(f, { t1: t, distance: r2(f.distance), summary: summary ? clip(summary, 800) : f.summary }); // every mission's words, how it ended last
    f.inspection = this.inspection(f);
    f.report = this.flightReport(f, t);
    this.save("flights", f);
    this.expirePassing(t);
    this.prune(t);
    this.trimTrails();
    this.emit("flight", f);
    return f;
  }

  saveChunk(f, i) {
    const s = this.trails.get(f.id).slice(i * this.o.chunk, (i + 1) * this.o.chunk);
    this.save("trails", { flight: f.id, seq: i, t: s[0].t, s });
    this.save("flights", f);
  }

  // The flight's index, so questions about it need no trail in RAM: seconds and the last sure sample per room, the last
  // sure sample within 1.5 m of each doorway.
  indexFlight(f, trail) {
    Object.assign(f, { roomSecs: {}, roomLast: {}, near: {} });
    trail.forEach((s, i) => this.index(f, s, trail[i - 1]));
  }
  index(f, s, prev) {
    if (prev && this.sure(prev) && prev.room && s.t - prev.t <= 2000) f.roomSecs[prev.room] = (f.roomSecs[prev.room] ?? 0) + (s.t - prev.t) / 1000;
    if (!this.sure(s)) return;
    if (s.room) f.roomLast[s.room] = s.t;
    for (const d of this.house?.doors ?? []) if (Math.hypot(s.x - (d.a[0] + d.b[0]) / 2, s.y - (d.a[1] + d.b[1]) / 2) <= 1.5) f.near[d.id] = s.t;
  }

  // Where the position was lost: each lost stretch (one trail marker) counted in f.losses, the sample before it (the last
  // known spot) in f.lostAt (the first lostSpots).
  noteLoss(f, s, prev) {
    f.losses = (f.losses ?? 0) + 1;
    if ((f.lostAt ??= []).length < this.o.lostSpots) f.lostAt.push({ t: s.t, x: prev?.x ?? null, y: prev?.y ?? null, room: prev?.room ?? null });
  }
  lossesOf(f, trail) {
    Object.assign(f, { losses: 0, lostAt: [] });
    trail.forEach((s, i) => s.lost && this.noteLoss(f, s, trail[i - 1]));
  }

  // A localizer pose { x, y, z, yaw, sigma, status, room } at 2 Hz while a flight runs; a lost position is one marker.
  samplePose(pose, t = this.now()) {
    const f = this.flight;
    if (!f || !pose) return false;
    const trail = this.trails.get(f.id), last = trail.at(-1), lost = pose.status === "lost" || !Number.isFinite(pose.x);
    if ((last && t - last.t < this.o.trailMs) || (lost && last?.lost)) return false;
    const s = lost ? { t, lost: true } : { t, x: r2(pose.x), y: r2(pose.y), z: r2(pose.z ?? 0), yaw: r3(pose.yaw ?? 0), sigma: r2(pose.sigma ?? 0), room: pose.room ?? this.roomAt(pose.x, pose.y) };
    trail.push(s);
    f.samples = trail.length;
    if (lost) this.noteLoss(f, s, last);
    this.track(f, s);
    this.index(f, s, last);
    if (trail.length % this.o.chunk === 0) this.saveChunk(f, trail.length / this.o.chunk - 1);
    if (this.sure(s)) for (const c of this.changes) if (c.status === "suspected" && c.passing && this.inView(s, c)) this.renew(c, t);
    this.expirePassing(t);
    this.syncTemps(t);
    return true;
  }

  inView(s, c) {
    const dx = c.x - s.x, dy = c.y - s.y, d = Math.hypot(dx, dy), a = Math.atan2(dy, dx) - s.yaw;
    return d <= this.o.view && (d < 0.3 || Math.abs(Math.atan2(Math.sin(a), Math.cos(a))) <= this.o.viewHalf);
  }

  sure(s) {
    return !!s && !s.lost && (s.sigma ?? 0) < this.o.sure;
  }

  // Rooms visited and distance flown, from sure samples only: a room once two samples in a row put it there; distance
  // from an anchor that moves on only when the position has gone further than its own noise (hovering adds nothing),
  // plus the last bit when the position is lost or the flight ends.
  track(f, s) {
    const run = this.run?.flight === f.id ? this.run : (this.run = { flight: f.id, room: null, n: 0, anchor: null, last: null });
    if (!this.sure(s)) {
      if (s.lost) this.settle(f);
      return Object.assign(run, { room: null, n: 0 });
    }
    run.n = s.room && s.room === run.room ? run.n + 1 : 1;
    run.room = s.room;
    if (s.room && run.n >= 2 && f.rooms.at(-1) !== s.room) (f.rooms.push(s.room), this.save("flights", f));
    const a = run.anchor, step = a ? Math.hypot(s.x - a.x, s.y - a.y) : 0;
    run.last = s;
    if (!a) run.anchor = s;
    else if (step > Math.max(0.1, 2 * Math.max(s.sigma ?? 0, a.sigma ?? 0))) (f.distance += step), (run.anchor = s);
  }
  settle(f) {
    const run = this.run?.flight === f.id ? this.run : null, a = run?.anchor, b = run?.last;
    if (a && b && b !== a) f.distance += Math.hypot(b.x - a.x, b.y - a.y);
    if (run) Object.assign(run, { anchor: null, last: null });
  }

  // Samples (oldest first) of one flight, or of every flight since `since` (ms).
  trail({ since = 0, flightId = null } = {}) {
    if (flightId) return (this.trails.get(flightId) ?? []).filter((s) => s.t >= since);
    return this.flights.filter((f) => (f.t1 ?? Infinity) >= since).flatMap((f) => (this.trails.get(f.id) ?? []).filter((s) => s.t >= since));
  }

  // Seconds per room in a trail (gaps over 2 s between samples don't count).
  roomTimes(trail) {
    const out = new Map();
    for (let i = 1; i < trail.length; i++) {
      const a = trail[i - 1], dt = (trail[i].t - a.t) / 1000;
      if (this.sure(a) && a.room && dt <= 2) out.set(a.room, (out.get(a.room) ?? 0) + dt);
    }
    return out;
  }

  // ---- sightings ----

  // { label, trackId, room, x, y, z, sigma (m: how sure the drone's position was), score, snapshot (base64 JPEG or Blob),
  // confirmed (the caller's: e.g. seen on enough frames), claude ("confirmed" | "likely" | "rejected" | "unconfirmed":
  // Claude's look, from alerts), near, mission, text, t }. The same label at the same spot or on the same track within
  // sightingWindow is one sighting (seen n times, best score kept, the surest position's σ).
  addSighting({ label, trackId = null, room = null, x = null, y = null, z = null, sigma = null, score = null, snapshot = null, confirmed = null, claude = null, near = null, mission = null, text = null, t = this.now() }) {
    const L = labelOf(label);
    if (!L) return null;
    const flight = this.flight?.id ?? "ground";
    const same = this.sightings.findLast((s) => s.label === L && s.flight === flight && t - s.t < this.o.sightingWindow
      && ((trackId != null && s.trackId === trackId) || (Number.isFinite(x) && Number.isFinite(s.x) ? Math.hypot(s.x - x, s.y - y) < this.o.near : s.room === room)));
    const shot = jpegBlob(snapshot, this.o.snapshotMax);
    let s;
    if (same) {
      s = same;
      const w = s.n;
      if (Number.isFinite(x)) Object.assign(s, Number.isFinite(s.x) ? { x: r2((s.x * w + x) / (w + 1)), y: r2((s.y * w + y) / (w + 1)) } : { x: r2(x), y: r2(y) });
      if (Number.isFinite(z)) s.z = r2(z);
      if (Number.isFinite(sigma)) s.sigma = r2(Number.isFinite(s.sigma) ? Math.min(s.sigma, sigma) : sigma);
      if (shot && (score ?? 0) >= (s.score ?? 0)) this.setPic(s, "snapshot", shot);
      Object.assign(s, { t, n: s.n + 1, score: r2(Math.max(s.score ?? 0, score ?? 0)), trackId: trackId ?? s.trackId, room: room ?? this.roomAt(s.x, s.y) ?? s.room });
      if (confirmed != null) s.confirmed = confirmed;
      if (claude) s.claude = claude;
    } else {
      s = { id: this.id("s", t), flight, t0: t, t, n: 1, label: L, trackId, room: room ?? this.roomAt(x, y), x: r2(x), y: r2(y), z: r2(z), sigma: Number.isFinite(sigma) ? r2(sigma) : null, score: r2(score), snapshot: null,
        confirmed, claude, near: near && String(near).slice(0, 40), mission: missionText(mission, (id) => this.roomName(id)) ?? this.flight?.mission ?? null, text: text && String(text).slice(0, 200) };
      if (shot) this.setPic(s, "snapshot", shot);
      this.sightings.push(s);
      if (this.flight) this.flight.seen++;
    }
    this.save("sightings", s);
    this.trimPictures();
    this.emit("sighting", s);
    return s;
  }

  // The latest sighting of a label ("cat", "the kitty", "someone") or of anything whose label or text has the words.
  lastSeen(what) {
    const L = labelOf(what);
    if (!L) return null;
    const ws = L.split(" ");
    return this.sightings.findLast((s) => s.label === L) ?? this.sightings.findLast((s) => ws.every((w) => `${s.label} ${s.text ?? ""}`.includes(w))) ?? null;
  }

  // Was this label seen before now: on another flight, or first seen more than sightingWindow ago on this one (so the
  // sighting a mission just recorded for the finding it is about to alert doesn't count).
  seenBefore(what, t = this.now()) {
    const L = labelOf(what), flight = this.flight?.id ?? "ground";
    return this.sightings.some((s) => s.label === L && (s.flight !== flight || t - s.t0 >= this.o.sightingWindow));
  }

  // ---- changes ----

  sameSpot(a, b) {
    const tol = Math.max(this.o.changeNear, 0.5 * Math.max(a.size ?? 0, b.size ?? 0));
    return Math.hypot(a.x - b.x, a.y - b.y) <= tol && (a.z == null || b.z == null || Math.abs(a.z - b.z) <= Math.max(0.5, tol));
  }
  sameChange(a, b) {
    return a.kind === b.kind && this.sameSpot(a, b);
  }
  // The latest confirmed change of the same family (doors, things) at a spot: how it is now, as far as anyone confirmed.
  confirmedAt(spot, fam = family(spot.kind)) {
    return this.changes.findLast((r) => r.status === "confirmed" && family(r.kind) === fam && this.sameSpot(r, spot)) ?? null;
  }
  emitChange(r, phase, extra = null) {
    this.emit("change", { ...r, ...extra, change: r, phase });
  }

  // { id?, kind: "obstacle" | "door-closed" | "door-open" | "moved" | "gone", x, y, z, room, size, zMin?, zMax?, door?,
  // from? (moved), what?, signature?, of? (the confirmed change this undoes), weak?, evidence?: { live, expected } } -> the
  // suspected change (new, or one at the same spot seen once more), the confirmed one it matches, or null when it matches
  // a spot the user dismissed.
  addChange(c, t = this.now()) {
    const ch = { kind: c.kind ?? "obstacle", x: r2(c.x), y: r2(c.y), z: r2(c.z), size: r2(c.size ?? 0.3), sigma: Number.isFinite(c.sigma) ? r2(c.sigma) : null };
    this.expirePassing(t);
    const again = (r, merge = false) => {
      // another report of an open thing: its place the mean of the reports, every report kept (spots: what it blocks reaches
      // them all), the larger size and height span
      if (merge && family(r.kind) === "thing" && Number.isFinite(ch.x)) {
        const w = r.n, spots = [...this.spotsOf(r), [ch.x, ch.y, ch.size / 2]];
        Object.assign(r, { x: r2((r.x * w + ch.x) / (w + 1)), y: r2((r.y * w + ch.y) / (w + 1)), size: Math.max(r.size ?? 0, ch.size), spots: tidySpots(spots),
          sigma: ch.sigma == null ? r.sigma : Math.min(r.sigma ?? Infinity, ch.sigma) },
          c.zMin != null && r.zMin != null && { zMin: Math.min(r.zMin, r2(c.zMin)) }, c.zMax != null && r.zMax != null && { zMax: Math.max(r.zMax, r2(c.zMax)) });
      }
      Object.assign(r, { last: t, n: r.n + 1, unseen: 0 }, merge && r.weak && !c.weak && { weak: false });
      if (c.evidence && r.status === "suspected") this.setPic(r, "evidence", this.evidence(c.evidence));
      if (r.passing && r.status === "suspected") r.passing = { ...r.passing, until: t + this.o.passingMs };
      this.save("changes", r);
      this.syncTemps(t);
      this.emitChange(r, "seen");
      if (c.id && String(c.id) !== r.id) this.emitChange(r, "merged", { id: String(c.id), status: "merged", into: r.id });
      return r;
    };
    const door = c.door ?? (family(ch.kind) === "door" ? this.doorOf(ch)?.id : null) ?? null;
    const open = this.changes.findLast((r) => r.status === "suspected" && this.merges(r, { ...ch, door }));
    if (open) return again(open, true);
    const known = this.confirmedAt(ch);
    if (known && BLOCKS.has(known.kind) === BLOCKS.has(ch.kind)) return again(known);
    const gone = this.changes.findLast((r) => suppresses(r, c, this.flight?.id) && this.sameChange(r, ch) && !(r.clearedAt > r.resolvedAt)
      && Math.abs((r.size ?? 0) - ch.size) <= Math.max(0.2, 0.5 * (r.size ?? 0)) && (c.signature == null || r.signature == null || c.signature === r.signature));
    if (gone) {
      gone.suppressed = (gone.suppressed ?? 0) + 1;
      gone.last = t;
      this.save("changes", gone);
      this.emitChange(gone, "suppressed", { ...ch, id: c.id ? String(c.id) : gone.id, status: "dismissed", suppressed: true, matches: gone.id });
      return null;
    }
    const r = { id: c.id && !this.changes.some((q) => q.id === String(c.id)) ? String(c.id) : this.id("c", t), flight: this.flight?.id ?? "ground", ...ch, room: c.room ?? this.roomAt(c.x, c.y),
      zMin: r2(c.zMin), zMax: r2(c.zMax), door, from: c.from ?? null, what: c.what ? String(c.what).slice(0, 120) : null, signature: c.signature ?? null, of: c.of ?? null,
      status: "suspected", t, last: t, n: 1, evidence: null, ...(c.weak && { weak: true }) };
    if (c.evidence) this.setPic(r, "evidence", this.evidence(c.evidence));
    this.changes.push(r);
    this.save("changes", r);
    this.syncTemps(t);
    this.trimPictures();
    this.emitChange(r, "suspected");
    return r;
  }

  // A new report is this open suspected change: the same kind, and the same doorway, or (things) their outlines within
  // twice the larger placement error (err()) at about the same height, and the record grown to hold both reports still
  // within maxReach of its place (two things a gap apart stay two records, each blocking where it was seen).
  merges(r, ch) {
    if (r.kind !== ch.kind) return false;
    if (family(ch.kind) === "door" && r.door && ch.door) return r.door === ch.door;
    const near = ((r.size ?? 0) + (ch.size ?? 0)) / 2 + 2 * Math.max(this.err(r), this.err(ch)), w = r.n, mx = (r.x * w + ch.x) / (w + 1), my = (r.y * w + ch.y) / (w + 1);
    return Math.hypot(r.x - ch.x, r.y - ch.y) <= near && (r.z == null || ch.z == null || Math.abs(r.z - ch.z) <= Math.max(0.6, near))
      && [...this.spotsOf(r), [ch.x, ch.y, ch.size / 2]].every(([x, y, s]) => Math.hypot(x - mx, y - my) + s <= this.o.maxReach);
  }

  // The detector's evidence: the live and expected pictures (JPEG blobs), where the change is in them (box) and whether
  // the best view still cut it at the picture's edge (cut).
  evidence(e) {
    return e ? { live: jpegBlob(e.live, this.o.snapshotMax), expected: jpegBlob(e.expected, this.o.snapshotMax), box: e.box ?? null, cut: !!e.cut } : null;
  }

  // Claude's opinion and similar notes on a change, without resolving it ("annotate", not "change").
  annotateChange(id, fields) {
    const r = this.changes.find((c) => c.id === id);
    if (!r) return null;
    Object.assign(r, fields);
    this.save("changes", r);
    this.emit("annotate", r);
    return r;
  }

  // status "confirmed": the house changes (see apply()); "dismissed": not lasting or not real (by "user": not raised again
  // for this spot; transient: it was a person, a pet or something passing). Either way it is off the map as a temporary
  // obstacle. A confirmed change dismissed later (the user changed their mind) is undone, as far as the house goes.
  // by "claude": only the safe direction (confirm a new blocker, dismiss a change that would free space or a weak
  // suspicion), else null.
  resolveChange(id, status, note = null, { by = "user", transient = false } = {}, t = this.now()) {
    const r = this.changes.find((c) => c.id === id);
    if (!r || !["confirmed", "dismissed"].includes(status) || r.status === status) return null;
    if (by === "claude" && (r.status !== "suspected" || (status === "confirmed" ? !ADDS.has(r.kind) : BLOCKS.has(r.kind) && !r.weak))) return null;
    const was = r.status;
    Object.assign(r, { status, note: note && String(note).slice(0, 300), by, resolvedAt: t, transient: status === "dismissed" && transient });
    if (status === "confirmed") r.applied = this.apply(r);
    else if (was === "confirmed") r.applied = this.apply(r, true);
    this.save("changes", r);
    this.syncTemps();
    this.emitChange(r, status);
    return r;
  }

  // Claude takes a suspected change for a person or pet: it stays an obstacle (as a person) until ttl after it was last in
  // view, reported or seen(), then is dismissed as transient, unless seen gone first (noteUnchanged) or resolved by someone.
  passing(id, { ttl = this.o.passingMs, by = "claude", note = null } = {}, t = this.now()) {
    const r = this.changes.find((c) => c.id === id);
    if (!r || r.status !== "suspected") return null;
    r.passing = { until: t + ttl, by, note: note && String(note).slice(0, 200) };
    this.save("changes", r);
    this.syncTemps();
    this.emit("annotate", r);
    return r;
  }
  // A person or pet still there: passingMs more (stored every 10 s at most: the evidence pictures go with the record).
  renew(c, t = this.now()) {
    const until = t + this.o.passingMs, was = c.passing.until;
    c.passing = { ...c.passing, until };
    if (Math.floor(until / 1e4) !== Math.floor(was / 1e4)) this.save("changes", c);
  }
  // The detector still sees the change `id` (the record's id: its memoryId): last seen now, and one taken for a person or
  // pet blocks a while longer. -> the record or null.
  seen(id, t = this.now()) {
    const r = this.changes.find((c) => c.id === id);
    if (!r || r.status !== "suspected") return null;
    Object.assign(r, { last: t, unseen: 0 });
    if (r.passing) this.renew(r, t);
    if (r.stale) this.syncTemps(t);
    return r;
  }
  expirePassing(t = this.now()) {
    for (const c of this.changes.filter((c) => c.status === "suspected" && c.passing?.until <= t))
      this.resolveChange(c.id, "dismissed", c.passing.note ?? "A person or pet; nothing there any more.", { by: "passing", transient: true }, t);
  }

  // The change detector saw this spot as the scan expects (id: the suspected change it looked at, wherever its place is):
  // a dismissed change there may be raised again later, a suspected one there was something passing (dismissed as
  // transient), and what was confirmed there no longer is (its opposite is raised: the new box is gone again, the closed
  // door is open).
  noteUnchanged({ x, y, z = null, r = 0.3, id = null }, t = this.now()) {
    const spot = { x, y, z, size: 2 * r };
    for (const c of [...this.changes]) {
      if (c.id !== id && !this.sameSpot(c, spot)) continue;
      if (c.status === "dismissed") (c.clearedAt = t), this.save("changes", c);
      else if (c.status === "suspected" && c.kind !== "gone" && !c.of) this.resolveChange(c.id, "dismissed", "No longer there.", { by: "detector", transient: true }, t);
    }
    for (const fam of ["thing", "door"]) {
      const k = this.confirmedAt(spot, fam);
      if (k) this.addChange({ kind: OPPOSITE[k.kind], x: k.x, y: k.y, z: k.z, size: k.size, zMin: k.zMin, zMax: k.zMax, door: k.door, what: k.what, of: k.id }, t);
    }
  }

  // A confirmed change applied to the house (undo: what it did taken back): a door's passable flag and its doorway in the
  // voxels (rebuild the map; until then a closed door is a temporary obstacle on this one); for an obstacle a keep-out
  // (kind "change", source "change") as large as what it blocked while suspected (tempOf: confirming says it is there,
  // not where exactly, so its place's error stays in) and occupied voxels where its reports put it (reach); for "gone",
  // the change keep-outs there (or of the change it undoes) removed, with what their changes made occupied, and the voxels
  // something marked occupied freed (vox.mark: unknown ones stay unknown, the plan's stay). In the simulator nothing in the
  // house changes (syncTemps keeps its blockers on its map). Returns what it did, or null.
  apply(r, undo = false) {
    const h = this.house;
    if (!h) return null;
    if (this.world === "sim") return { sim: true };
    let out = null, rebuild = false, keepouts = false;
    const box = this.boxOf(r, r.kind === "gone" ? null : this.reach(r)), mark = (b, state) => this.vox?.mark?.(b, state, { source: "change" }) ?? null;
    if (family(r.kind) === "door") {
      const d = this.doorOf(r), open = (r.kind === "door-open") !== undo;
      if (!d) return null;
      if (!open && d.passable !== false) {
        Object.assign(d, { passable: false, closedBy: r.id });
        this.staleDoors.set(d.id, r);
        rebuild = true;
      } else if (open && d.passable === false && (d.closedBy || !undo)) {
        d.passable = true;
        delete d.closedBy;
        this.staleDoors.delete(d.id);
        rebuild = true;
      }
      out = { door: d.id, passable: d.passable, voxels: rebuild ? this.vox?.door?.(d, d.passable) ?? null : null };
    } else if (undo) {
      // the voxels as they were: an obstacle's put back exactly (and a moved thing's old place filled again); what a "gone"
      // freed has no record, so its box is filled (it may block some space that was free)
      const before = h.keepouts?.length ?? 0, moved = r.kind === "moved" && Number.isFinite(r.from?.x);
      h.keepouts = (h.keepouts ?? []).filter((k) => k.change !== r.id);
      keepouts = h.keepouts.length !== before;
      out = { removed: before - h.keepouts.length, ...(r.kind === "gone" ? { occupied: mark(box, "occupied") } : { freed: this.vox?.mark?.(box, "free", { scan: false, source: "change" }) ?? null }),
        ...(moved && { occupied: mark(this.boxOf({ ...r, ...r.from, zMin: null, zMax: null }), "occupied") }) };
    } else if (r.kind === "gone") {
      const off = (h.keepouts ?? []).filter((k) => k.source === "change" && (k.change === r.of || Math.hypot(k.x - r.x, k.y - r.y) <= Math.max(this.o.changeNear, k.reach ?? k.r ?? 0)));
      h.keepouts = (h.keepouts ?? []).filter((k) => !off.includes(k));
      keepouts = off.length > 0;
      // what those changes made occupied goes back first (a merged record reaches past this report's box)
      const was = off.map((k) => this.changes.find((c) => c.id === k.change)).filter((c) => c && c.kind !== "gone");
      const restored = was.reduce((n, c) => n + (this.vox?.mark?.(this.boxOf(c, this.reach(c)), "free", { scan: false, source: "change" }) ?? 0), 0);
      out = { removed: off.length, freed: mark(box, "free"), ...(restored && { restored }) };
    } else {
      const tp = this.tempOf(r) ?? { x: r.x, y: r.y, r: 0.15 };
      const ko = { kind: "change", x: tp.x, y: tp.y, r: tp.r, reach: r2(this.reach(r)), source: "change", change: r.id, room: r.room, name: r.what ?? "new obstacle",
        ...(tp.zMin != null && { zMin: r2(tp.zMin), zMax: r2(tp.zMax) }) };
      h.keepouts = [...(h.keepouts ?? []).filter((k) => k.change !== r.id), ko];
      keepouts = true;
      // a moved thing: its old place freed first, so where the two boxes overlap the new obstacle stays
      const freed = r.kind === "moved" && Number.isFinite(r.from?.x) ? mark(this.boxOf({ ...r, ...r.from, zMin: null, zMax: null }), "free") : undefined;
      out = { keepout: ko, occupied: mark(box, "occupied"), ...(freed !== undefined && { freed }) };
    }
    if (keepouts && this.map?.house === h) this.map.finalize();
    this.saved = Promise.resolve(this.saveHouse?.(h)).catch((e) => this.emit("error", `Couldn't save the house after a change: ${e?.message ?? e}`));
    this.emit("house", { house: h, change: r, rebuild, saved: this.saved });
    return out;
  }

  // A change's space for the voxels: its size across (or reach: as far as its reports go), its heights (or its size up
  // from the floor).
  boxOf(r, reach = null) {
    const s = reach ?? Math.max(0.1, r.size ?? 0.3) / 2, floor = this.map?.floorAt?.(r.x, r.y) ?? 0;
    const z0 = r.zMin ?? (r.z != null ? r.z - s : floor), z1 = r.zMax ?? (r.z != null ? r.z + s : floor + 2 * s);
    return { min: [r.x - s, r.y - s, z0], max: [r.x + s, r.y + s, z1] };
  }

  // ---- other events (alerts, missions) ----

  // { kind, text, room, urgency, label?, score?, confirmed?, x?, y?, pose?, trackId? }. An event with a label is a
  // sighting (it updates the matching one's confirmed state, or adds one); others go in the timeline.
  note(ev, t = ev?.t ?? this.now()) {
    if (!ev) return null;
    const x = ev.x ?? ev.pose?.x ?? null, y = ev.y ?? ev.pose?.y ?? null;
    if (ev.label) {
      const L = labelOf(ev.label);
      const s = this.sightings.findLast((q) => q.label === L && Math.abs(t - q.t) < this.o.sightingWindow
        && ((ev.trackId != null && q.trackId === ev.trackId) || (Number.isFinite(x) && Number.isFinite(q.x) ? Math.hypot(q.x - x, q.y - y) < 2 * this.o.near : !ev.room || q.room === ev.room)));
      const claude = ev.confirmation ?? null;
      if (s) {
        if (claude) Object.assign(s, { claude }, claude === "rejected" && { confirmed: false }), this.save("sightings", s), this.emit("sighting", s);
        return s;
      }
      return this.addSighting({ label: L, trackId: ev.trackId ?? null, room: ev.room ?? null, x, y, z: ev.z ?? ev.pose?.z ?? null, score: ev.score ?? null, snapshot: ev.image ?? null,
        confirmed: claude === "rejected" ? false : null, claude, text: ev.text, t });
    }
    const e = { id: this.id("e", t), t, kind: ev.kind ?? "event", text: String(ev.text ?? "").slice(0, 300), room: ev.room ?? this.roomAt(x, y), urgency: ev.urgency ?? "default",
      x: r2(x), y: r2(y), flight: this.flight?.id ?? null, confirmed: ev.confirmed ?? null };
    if (e.kind === "mission" && this.flight) (this.flight.missions.push(e.text.slice(0, 80)), this.save("flights", this.flight));
    this.events.push(e);
    this.save("events", e);
    return e;
  }

  // ---- reading it back ----

  // Newest first: { t, kind: "flight" | "landed" | "sighting" | "change" | <event kind>, text, ref }.
  timeline({ since = 0, limit = 50 } = {}, now = this.now()) {
    const out = [];
    for (const f of this.flights) {
      if (f.t0 >= since) out.push({ t: f.t0, kind: "flight", text: `Took off${simFlight(f) ? " in the simulator" : ""}${f.mission ? `: ${f.mission}` : !["flight", "sim", "real"].includes(f.kind) ? ` (${f.kind})` : ""}.`, ref: f });
      if (f.t1 != null && f.t1 >= since) out.push({ t: f.t1, kind: "landed", text: this.flightLine(f, now), ref: f });
    }
    for (const s of this.sightings) if (s.t >= since) out.push({ t: s.t, kind: "sighting", text: `Saw ${this.thing(s)} ${this.where(s)}${confirmText(s.claude)}.`, ref: s });
    for (const c of this.changes) if (c.t >= since) out.push({ t: c.t, kind: "change", text: `${cap(this.changeText(c))} (${c.status}).`, ref: c });
    for (const e of this.events) if (e.t >= since) out.push({ t: e.t, kind: e.kind, text: e.text, ref: e });
    return out.sort((a, b) => b.t - a.t).slice(0, limit);
  }

  stats() {
    const labels = {}, rooms = {};
    for (const s of this.sightings) labels[s.label] = (labels[s.label] ?? 0) + 1;
    for (const f of this.flights) for (const [id, sec] of Object.entries(f.roomSecs ?? {})) rooms[id] = Math.round((rooms[id] ?? 0) + sec);
    const changes = { suspected: 0, confirmed: 0, dismissed: 0 };
    for (const c of this.changes) changes[c.status]++;
    return { flights: this.flights.length, seconds: Math.round(this.flights.reduce((a, f) => a + ((f.t1 ?? this.now()) - f.t0) / 1000, 0)),
      distance: r2(this.flights.reduce((a, f) => a + f.distance, 0)), sightings: this.sightings.length, labels, changes, rooms, since: this.flights[0]?.t0 ?? null };
  }

  thing(s) {
    return s.label === "person" ? "a person" : `the ${s.label}`;
  }

  flightLine(f, now = this.now()) {
    const secs = ((f.t1 ?? now) - f.t0) / 1000, rooms = f.rooms.map((id) => this.roomName(id)).filter((n, i, a) => n !== a[i - 1]);
    const seen = this.sightings.filter((s) => s.flight === f.id), things = [...new Set(seen.map((s) => `${this.thing(s)} ${this.where(s)}`))].slice(0, 3);
    return `${f.t1 == null ? "This flight" : `The flight ${when(f.t0, now)}`}${simFlight(f) ? " in the simulator" : ""}: ${duration(secs)}, ${r2(f.distance).toFixed(1)} m${rooms.length ? `, ${rooms.join(" → ")}` : ""}`
      + `${f.missions?.length ? `; ${f.missions.slice(0, 3).map((t) => t.replace(/\.$/, "")).join("; ")}` : f.mission ? `; ${f.mission}` : ""}${things.length ? `; saw ${things.join(", ")}` : ""}`
      + `${f.summary && f.t1 != null && !f.missions?.length ? `; ${f.summary.replace(/\.$/, "")}` : ""}.`;
  }

  // How much of each room the drone camera saw on a flight (vox.markSeen since its take-off): of the room's space (a
  // lattice of inspectStep m at inspectHeights above its floor, less what the 3D map has occupied), the share known free
  // whose last sighting is from this flight; space the 3D map doesn't have (unknown: the camera can't look through it)
  // counts as not seen and is told apart (unmapped). Where most of what it didn't see is. measured false when nothing was
  // marked seen this flight (the camera's picture wasn't trusted, f.paused: seenPaused()). null without the 3D map.
  // -> { measured: true, rooms: [{ id, name, share, unmapped, points, unseen: { x, y, z, low, where } | null }], share }
  //  | { measured: false, why, rooms: [], share: null }
  inspection(f = this.flight ?? this.flights.at(-1)) {
    const vox = this.vox, map = this.map;
    if (!f || !vox?.state || !map?.roomAt) return null;
    const step = this.o.inspectStep, rooms = [], seenAt = vox.seen ? (x, y, z) => vox.seenAt(x, y, z) : () => null;
    let all = 0, saw = 0;
    for (const room of this.house?.rooms ?? []) {
      const xs = room.outline.map((p) => p[0]), ys = room.outline.map((p) => p[1]), miss = [];
      let n = 0, hit = 0, unknown = 0;
      for (let x = Math.min(...xs) + step / 2; x < Math.max(...xs); x += step)
        for (let y = Math.min(...ys) + step / 2; y < Math.max(...ys); y += step) {
          if (map.roomAt(x, y)?.id !== room.id) continue;
          const floor = map.floorAt(x, y) ?? room.floorZ ?? 0, ceil = map.ceilingAt?.(x, y) ?? floor + 2.4;
          for (const h of this.o.inspectHeights) {
            const st = floor + h > ceil - 0.1 ? 2 : vox.state(x, y, floor + h); // voxels.js UNKNOWN 0, FREE 1, OCCUPIED 2
            if (st === 2) continue;
            n++;
            if (st === 1 && seenAt(x, y, floor + h) >= f.t0) hit++;
            else (miss.push([x, y, floor + h, h]), (unknown += st === 0));
          }
        }
      if (!n) continue;
      all += n, saw += hit;
      const share = hit / n, mid = (k) => miss.reduce((a, p) => a + p[k], 0) / miss.length;
      const unseen = miss.length && share < 0.9 ? { x: r2(mid(0)), y: r2(mid(1)), z: r2(mid(2)), low: miss.filter((p) => p[3] < 0.6).length > miss.length / 2 } : null;
      if (unseen) unseen.where = [unseen.low && "near the floor", this.nearby(unseen.x, unseen.y) && `around the ${this.nearby(unseen.x, unseen.y)}`].filter(Boolean).join(", ");
      rooms.push({ id: room.id, name: room.name, share: r2(share), unmapped: r2(unknown / n), points: n, unseen });
    }
    if (!saw) return { measured: false, why: f.paused ?? null, rooms: [], share: null };
    return { measured: true, rooms, share: r2(saw / all) };
  }

  // The running flight's camera picture isn't trusted now (why: ui/session.js untrusted()), so what it sees isn't marked:
  // the report says so instead of "the camera saw none of the rooms".
  seenPaused(why) {
    if (this.flight && why) this.flight.paused = String(why).slice(0, 120);
  }

  // "The camera saw 82% of the Living room, 64% of Room 2 (30% of it isn't in the 3D map; not near the floor); none of
  // Room 3." or "What the camera saw wasn't measured on this flight (...)."
  inspectionText(ins) {
    if (ins?.measured === false) return `What the camera saw wasn't measured on this flight${ins.why ? ` (its picture wasn't trusted: ${ins.why})` : ""}.`;
    if (!ins?.rooms?.length) return "";
    const seen = ins.rooms.filter((r) => r.share >= 0.05), none = ins.rooms.filter((r) => r.share < 0.05);
    const bits = (r) => [r.unmapped >= 0.05 && `${Math.round(r.unmapped * 100)}% of it isn't in the 3D map`, r.share >= 0.05 && r.unseen?.where && `not ${r.unseen.where}`].filter(Boolean);
    const one = (r, what) => `${what}${bits(r).length ? ` (${bits(r).join("; ")})` : ""}`;
    const list = (a) => (a.length > 1 ? `${a.slice(0, -1).join(", ")} and ${a.at(-1)}` : a[0]);
    const noneText = list(none.map((r) => one(r, theRoom(r.name))));
    return `${seen.length ? `The camera saw ${list(seen.map((r) => one(r, `${Math.round(r.share * 100)}% of ${theRoom(r.name)}`)))}` : `The camera saw none of ${noneText}`}${seen.length && none.length ? `; none of ${noneText}` : ""}.`;
  }

  // After landing, in plain words and the same way every time: how long, where, what it saw, how much of each room the
  // camera looked at, the changes waiting for an answer and how sure the position was.
  flightReport(f, now = this.now()) {
    const secs = ((f.t1 ?? now) - f.t0) / 1000, rooms = f.rooms.map((id) => theRoom(this.roomName(id))).filter((n, i, a) => n !== a[i - 1]);
    const seen = this.sightings.filter((s) => s.flight === f.id), things = [...new Set(seen.map((s) => `${this.thing(s)} ${this.where(s)}`))];
    const lost = f.losses ?? 0, during = (c) => c.t >= f.t0 && c.t <= (f.t1 ?? now);
    const open = this.changes.filter((c) => during(c) && c.status === "suspected"), confirmed = this.changes.filter((c) => during(c) && c.status === "confirmed");
    const out = [`${simFlight(f) ? "Simulator flight" : "Flight"}: ${duration(secs)}, ${f.distance.toFixed(1)} m${rooms.length ? `, ${rooms.join(" → ")}` : ", without a sure position"}.`];
    if (f.summary) out.push(cap(f.summary.trim().replace(/([^.…])$/, "$1.")));
    out.push(things.length ? `Saw ${things.slice(0, 4).join(", ")}${things.length > 4 ? ` and ${things.length - 4} more` : ""}.` : "No people or pets seen.");
    const ins = this.inspectionText(f.inspection);
    if (ins) out.push(ins);
    if (open.length) out.push(`${plural(open.length, "possible change")} to check: ${open.slice(0, 3).map((c) => this.changeText(c)).join("; ")}.`);
    if (confirmed.length) out.push(`Confirmed: ${confirmed.slice(0, 3).map((c) => this.changeText(c)).join("; ")}.`);
    if (lost) out.push(`The position was lost ${plural(lost, "time")}.`);
    return out.join(" ");
  }

  // Where it has been this flight, for where_am_i: rooms in order, time, distance.
  flightSoFar(now = this.now()) {
    const f = this.flight;
    if (!f) {
      const last = this.flights.at(-1);
      return last ? `Not flying now; ${this.flightLine(last, now).replace(/^The flight/, "the last flight was")}` : "No flights remembered yet.";
    }
    const tr = this.trails.get(f.id) ?? [], last = tr.at(-1), lost = f.losses ?? 0, rooms = f.rooms.map((id) => this.roomName(id));
    const known = tr.findLast((s) => this.sure(s) && s.room), name = (s) => theRoom(this.roomName(s.room));
    let path;
    if (this.sure(last) && last.room && last.room === f.rooms.at(-1))
      path = rooms.length > 1 ? `took off in ${theRoom(rooms[0])}${rooms.length > 2 ? `, then ${rooms.slice(1, -1).map(theRoom).join(", ")}` : ""}, now in ${theRoom(rooms.at(-1))}` : `all of it in ${theRoom(rooms[0])}`;
    else {
      const been = rooms.length ? `took off in ${theRoom(rooms[0])}${rooms.length > 1 ? `, then ${rooms.slice(1).map(theRoom).join(", ")}` : ""}` : "no sure position yet";
      const at = !last ? "" : last.lost ? `position lost now${known ? ` (last known ${known.room ? `in ${name(known)}` : "off the rooms"}, ${duration((now - known.t) / 1000)} ago)` : ""}`
        : `now ${this.sure(last) ? "" : "probably "}${this.placeText(last.room, last.x, last.y)}${this.sure(last) ? "" : ` (position unsure, ±${(2 * last.sigma).toFixed(1)} m)`}`;
      path = [been, at].filter(Boolean).join("; ");
    }
    return `This flight${this.world === "sim" ? " (in the simulator)" : ""}: ${duration((now - f.t0) / 1000)} so far, ${f.distance.toFixed(1)} m flown, ${path}${lost ? ` (position lost ${plural(lost, "time")} in all)` : ""}.`;
  }

  changeText(c) {
    const where = this.where(c), size = c.size ? `, about ${c.size < 1 ? `${Math.round(c.size * 100)} cm` : `${c.size.toFixed(1)} m`} across` : "";
    const door = () => {
      const d = this.doorOf(c);
      return d ? `the doorway between ${d.rooms.map((id) => (id ? theRoom(this.roomName(id)) : "outside")).join(" and ")}` : `a doorway ${where}`;
    };
    switch (c.kind) {
      case "door-closed": return `${door()} is closed`;
      case "door-open": return `${door()} is open`;
      case "moved": return `${c.what ? `the ${c.what}` : "something"} moved ${where}${size}`;
      case "gone": return `${c.what ? `the ${c.what}` : c.of ? "the obstacle confirmed earlier" : "something in the scan"} is gone ${where}`;
      default: return `${c.what ? `a new ${c.what}` : "a new obstacle"} ${where}${size}`;
    }
  }

  // The changes since `since` (anything parseSince() reads), suspected first.
  whatChanged(since = null, now = this.now()) {
    const t0 = parseSince(since, now, this.lastFlightStart()), list = this.changes.filter((c) => c.last >= t0 || (c.status === "suspected" && !since));
    const span = since ? ` since ${typeof since === "string" ? since : when(t0, now)}` : "";
    if (!list.length) return `No differences from the 3D scan noticed${span || " on any flight I remember"}.`;
    const line = (c) => {
      const cl = c.claude?.status === "answered" ? `; Claude: ${c.claude.what || c.claude.verdict}, ${Math.round(c.claude.confidence * 100)}% sure` : "";
      return `${this.changeText(c)} (seen ${c.n === 1 ? "once" : `${c.n} times`}, last ${when(c.last, now)}${cl}${c.note ? `; ${c.note}` : ""})`;
    };
    const by = (s) => list.filter((c) => c.status === s);
    const out = [];
    if (by("suspected").length) out.push(`Not confirmed yet: ${by("suspected").map(line).join("; ")}.`);
    if (by("confirmed").length) out.push(`Confirmed: ${by("confirmed").map(line).join("; ")}.`);
    if (by("dismissed").length) out.push(`Dismissed (not real or not lasting): ${by("dismissed").map((c) => this.changeText(c)).join("; ")}.`);
    return `${plural(list.length, "change")}${span}. ${out.join(" ")}`;
  }

  lastFlightStart() {
    return (this.flight ?? this.flights.at(-1))?.t0 ?? 0;
  }

  // Questions about earlier flights -> plain text built only from the records (Claude gets it as a tool result).
  recall(question = "", now = this.now()) {
    return this.answer(question, now).text;
  }

  // recall() plus the sighting it is about (its snapshot can go to Claude with the text). Deterministic: rooms by name or
  // by what people call them (ROOM_WORDS, against the name or a survey's kind), people and pets by what people call them
  // ("my son", "the pets"), a time window ("today", "this morning", "in the last 2 hours") and "how many times".
  answer(question = "", now = this.now()) {
    let q = norm(String(question).replace(/\b(where|what|who|how)['’]s\b/gi, "$1 is").replace(/\b(\w+)['’]s\b/g, "$1"));
    const win = windowOf(q, now);
    if (win) q = q.replace(win.re, " ").replace(/\s+/g, " ").trim();
    const { room, words } = this.roomIn(q) ?? {};
    if (/\bdoors?\b|\bdoorways?\b/.test(q) && /\b(open|opened|closed|shut|locked)\b/.test(q)) return { text: this.doorText(q, now) };
    if (/\b(chang|different|differ|moved|missing)/.test(q) || (/\b(unusual|strange|weird|odd|out of place)\b/.test(q) && !this.named(q))) return { text: this.whatChanged(q.match(/\bsince (.+)$/)?.[1] ?? (win ? win.from : null), now) };
    // "where was the cat last seen", "is anyone home", "did my husband come home", "who was in the lounge": what after
    // the verb, less the room and the words around it; a thing the map has stays asked about ("the cat bowl"); "who"
    // means anyone; else a label or a sighting's label anywhere in the question ("is the house empty": anyone).
    const said = q.match(/\b(?:see|seen|saw|spot|spotted|notice|noticed|find|found|is|are|was|were)\b (.+)$/)?.[1];
    let rest = said && !/^(you|i|we|it|there)\b/.test(said) ? subjectOf(said, room && words) : "";
    if (/\bwho\b/.test(q) && !this.knows(rest)) rest = "anyone";
    if (!this.knows(rest) && !this.landmark(rest)) rest = this.named(q) ?? (/\b(empty|alone)\b/.test(q) ? "anyone" : rest);
    const count = /\bhow (many|often)\b/.test(q);
    if (rest && PETS.test(rest)) return this.petsText(now, room, win, count);
    if (rest && !/^(?:anything|something|stuff|things?|lately|recently)$/.test(rest)) return this.seenText(rest, now, room, { win, count });
    if (room) return { text: this.roomText(room, now) };
    if (/\b(how many|how much|how long|stats|total|altogether)\b/.test(q)) return { text: this.statsText(now) };
    if (/\b(flight|flights|flew|flown|fly|been|went|trail|path|route|where have you|where did you)\b/.test(q)) return { text: this.flightsText(now, win) };
    return { text: this.digest(now) };
  }

  // A subject recall can answer about: a label people use (LABELS), the pets, or a label a sighting has.
  knows(what) {
    const L = what && labelOf(what);
    return !!L && (!!LABELS[L] || PETS.test(what) || this.sightings.some((s) => s.label === L));
  }
  // A landmark of the map's that the words name ("cat bowl", "the sofa") -> it or null.
  landmark(what, room = null) {
    const L = what && labelOf(what), all = L ? (this.house?.landmarks ?? []).filter((m) => norm(m.name) === L || norm(m.name).includes(L)) : [];
    return all.find((m) => room && (m.room ?? this.roomAt(m.x, m.y)) === room.id) ?? all[0] ?? null;
  }
  // The first thing a question names: a sighting's label (longest first: "teddy bear"), the pets, or a word for a person,
  // cat or dog; not one that only says whose a thing is ("the dog bed", "the cat flap"). -> words or null
  named(q) {
    const labels = [...new Set(this.sightings.map((s) => s.label))].sort((a, b) => b.length - a.length);
    const names = (l) => ((m) => !!m && !OWNED.test(m[1] ?? ""))(q.match(new RegExp(`\\b${l}s?\\b(?: (\\w+))?`)));
    return labels.find(names) ?? (/\b(pets?|animals?)\b/.test(q) ? "pets" : null) ?? q.split(" ").find((w, i, ws) => LABELS[labelOf(w)] && !OWNED.test(ws[i + 1] ?? "")) ?? null;
  }

  // The room a question names: by name first, then by what people call it. -> { room, words } | null
  roomIn(q) {
    const rooms = this.house?.rooms ?? [];
    for (const r of rooms) if (new RegExp(`\\b${norm(r.name)}\\b`).test(q)) return { room: r, words: norm(r.name) };
    for (const [kind, re] of Object.entries(ROOM_WORDS)) {
      const m = q.match(re), r = m && rooms.find((r) => r.kind === kind || re.test(norm(r.name)));
      if (r) return { room: r, words: m[0] };
    }
    return null;
  }

  // room: only sightings there (and where it was seen last, if elsewhere); win: only within that time (else when it was
  // last seen at all); count: how many sightings.
  seenText(what, now = this.now(), room = null, { win = null, count = false } = {}) {
    const L = labelOf(what), any = this.lastSeen(what), n = this.flights.length, name = LABELS[L] ? (L === "person" ? "anyone" : `the ${L}`) : `"${what}"`;
    if (win || count) {
      const ws = L.split(" "), is = (q) => q.label === L || (!LABELS[L] && ws.every((w) => `${q.label} ${q.text ?? ""}`.includes(w)));
      const hits = this.sightings.filter((q) => is(q) && (!room || q.room === room.id) && (!win || (q.t >= win.from && q.t < win.to))), s = hits.at(-1);
      const where = room ? ` in ${theRoom(room.name)}` : "", near = s && (s.near ?? (Number.isFinite(s.x) && this.nearby(s.x, s.y)));
      // in the room asked about: only where in it ("near the sofa"), not the room again
      const at = s && (room && s.room === room.id ? [this.unsure(s) && "probably", near && `near the ${near}`].filter(Boolean).join(" ") : this.where(s));
      const head = s && `${win ? `Yes, ${win.words}: ` : ""}I saw ${this.thing(s)}${where} ${hits.length === 1 ? "once" : `${hits.length} times${win ? "" : " in all"}; the last`}`;
      if (s) return { sighting: s, text: `${hits.length === 1 ? [head, at, when(s.t, now)].filter(Boolean).join(", ") : `${head}${at ? ` ${at}` : ""}, ${when(s.t, now)}`}${confirmText(s.claude)}.` };
      const flew = !win || this.flights.some((f) => f.t0 < win.to && (f.t1 ?? now) >= win.from), base = this.seenText(what, now, room);
      return { ...base, text: `${win ? `Not ${win.words}${flew ? "" : ` (I didn't fly ${win.words})`}. ` : ""}${base.text}` };
    }
    const s = room ? this.sightings.findLast((q) => q.room === room.id && (q.label === L || q === any)) ?? null : any;
    if (!s) {
      const l = this.landmark(what, room), since = n ? ` (${plural(n, "flight")} since ${when(this.flights[0].t0, now)})` : "";
      const also = room && any ? ` I last saw ${this.thing(any)} ${this.where(any)}, ${when(any.t, now)}.` : "";
      if (l) {
        const lr = l.room ?? this.roomAt(l.x, l.y), lm = l.name.replace(/^the /i, "").split(" ").map((w) => (/^[A-Z0-9]{2,4}$/.test(w) ? w : w.toLowerCase())).join(" ");
        return { text: `The map has the ${lm} ${this.placeText(lr, l.x, l.y)}${room && lr !== room.id ? `, not in ${theRoom(room.name)}` : ""}; no flight I remember noted it${since}.${also}`, sighting: room ? any : null };
      }
      return { text: `I haven't seen ${name}${room ? ` in ${theRoom(room.name)}` : ""} on any flight I remember${since}.${also}`, sighting: room ? any : null };
    }
    const f = this.flights.find((q) => q.id === s.flight), earlier = this.sightings.filter((o) => o.label === s.label && o !== s);
    const prev = earlier.findLast((o) => o.room !== s.room || o.flight !== s.flight);
    const m = (s.mission ?? f?.mission)?.replace(/\.$/, "");
    const bits = [m && (m === "patrol" ? "on patrol" : `while asked to ${m}`), simFlight(f ?? { world: this.world }) && "in the simulator", s.n > 1 && `seen ${s.n} times`].filter(Boolean);
    return { sighting: s, text: `I last saw ${this.thing(s)} ${this.where(s)}, ${when(s.t, now)}${bits.length ? ` (${bits.join(", ")})` : ""}${confirmText(s.claude)}.`
      + `${prev ? ` Before that: ${this.where(prev)}, ${when(prev.t, now)}.` : ""}${earlier.length ? ` ${plural(earlier.length + 1, "sighting")} in all.` : ""}` };
  }

  // "the pets": the cat and the dog, each.
  petsText(now, room, win, count) {
    const each = ["cat", "dog"].map((l) => this.seenText(l, now, room, { win, count })), seen = each.filter((a) => a.sighting);
    return { sighting: seen.sort((a, b) => b.sighting.t - a.sighting.t)[0]?.sighting ?? null, text: each.map((a) => a.text).join(" ") };
  }

  roomText(room, now = this.now()) {
    let last = null, flight = null, visits = 0;
    for (const f of this.flights) if (f.roomLast?.[room.id] != null) (visits++, (last = f.roomLast[room.id]), (flight = f));
    const seen = this.sightings.filter((s) => s.room === room.id).at(-1);
    if (last == null) return `I haven't been in ${theRoom(room.name)} on any flight I remember (${plural(this.flights.length, "flight")}).${seen ? ` From outside it I saw ${this.thing(seen)} there, ${when(seen.t, now)}.` : ""}`;
    const secs = flight.roomSecs?.[room.id] ?? 0;
    return `I was last in ${theRoom(room.name)} ${when(last, now)}, ${secs < 2 ? "briefly" : `for about ${duration(secs)}`}, on the flight ${when(flight.t0, now)}; ${visits} of ${plural(this.flights.length, "flight")} went there.`
      + `${seen ? ` The last thing I saw there: ${this.thing(seen)}, ${when(seen.t, now)}.` : ""}`;
  }

  // "is the front door open?": which door (a landmark naming it, the rooms named, front/outside for the doorways off the
  // map), then what was seen there: a change (confirmed or not), or when a flight last went near it, or that none did.
  doorText(q, now = this.now()) {
    const doors = this.house?.doors ?? [], rooms = (this.house?.rooms ?? []).filter((r) => new RegExp(`\\b${norm(r.name)}\\b`).test(q));
    const mark = (this.house?.landmarks ?? []).find((l) => /door/i.test(l.name) && q.includes(norm(l.name)));
    const outside = /\b(front|main|entrance|entry|outside|outer|exterior|back|garden)\b/.test(q);
    let list = mark ? [nearestDoor(doors, mark.x, mark.y, 1.5)].filter(Boolean)
      : rooms.length >= 2 ? doors.filter((d) => rooms.slice(0, 2).every((r) => d.rooms.includes(r.id)))
      : outside ? doors.filter((d) => !d.rooms[1] && (!rooms.length || d.rooms.includes(rooms[0].id)))
      : rooms.length ? doors.filter((d) => d.rooms.includes(rooms[0].id)) : [];
    if (!list.length) {
      const all = doors.map((d) => this.doorName(d));
      return `I can't tell which door you mean${all.length ? `; the map's doorways: ${[...new Set(all)].slice(0, 8).join("; ")}` : ""}.`;
    }
    return list.slice(0, 3).map((d) => this.doorState(d, now)).join(" ") + (list.length > 3 ? ` (and ${plural(list.length - 3, "more doorway")})` : "");
  }

  doorName(d) {
    return `the doorway ${d.rooms[1] ? `between ${theRoom(this.roomName(d.rooms[0]))} and ${theRoom(this.roomName(d.rooms[1]))}` : `from ${theRoom(this.roomName(d.rooms[0]))} to outside`}`;
  }

  doorState(d, now = this.now()) {
    const name = cap(this.doorName(d)), at = (c) => c.door === d.id || this.doorOf(c) === d;
    const c = this.changes.findLast((c) => family(c.kind) === "door" && c.status !== "dismissed" && at(c)), state = (c) => (c.kind === "door-open" ? "open" : "closed");
    let passed = null;
    for (const f of this.flights) if (f.near?.[d.id] > (passed?.t ?? -Infinity)) passed = { t: f.near[d.id] };
    const since = (t) => (passed && passed.t > t ? `; I last flew near it ${when(passed.t, now)} and no change was reported there since` : "");
    if (c?.status === "confirmed") return `${name}: confirmed ${state(c)} ${when(c.resolvedAt ?? c.last, now)}${since(c.resolvedAt ?? c.last)}.`;
    if (c) return `${name}: looked ${state(c)} ${when(c.last, now)}, not confirmed yet.`;
    const scan = d.passable === false ? "closed" : "open";
    return passed ? `${name}: ${scan} in the 3D scan; I last flew near it ${when(passed.t, now)} and no change was reported there.`
      : `${name}: ${scan} in the 3D scan; no flight I remember went near it, so I don't know how it is now.`;
  }

  flightsText(now = this.now(), win = null) {
    if (!this.flights.length) return "I don't remember any flights yet.";
    const list = win ? this.flights.filter((f) => f.t0 < win.to && (f.t1 ?? now) >= win.from) : this.flights;
    if (!list.length) return `I didn't fly ${win.words}. ${this.flightLine(this.flights.at(-1), now).replace(/^The flight/, "The last flight was")}`;
    const recent = list.slice(-3).reverse().map((f) => this.flightLine(f, now)), more = list.length - recent.length;
    if (win) return `${plural(list.length, "flight")} ${win.words}: ${recent.join(" ")}${more > 0 ? ` And ${plural(more, "earlier one")}.` : ""}`;
    return `${recent.join(" ")}${more > 0 ? ` And ${plural(more, "earlier flight")} since ${when(this.flights[0].t0, now)}.` : ""}`;
  }

  statsText(now = this.now()) {
    const s = this.stats();
    if (!s.flights && !s.sightings) return "I don't remember any flights yet.";
    const labels = Object.entries(s.labels).sort((a, b) => b[1] - a[1]).map(([l, n]) => `${l} ${n}`).join(", ");
    const top = Object.entries(s.rooms).sort((a, b) => b[1] - a[1])[0];
    return `I remember ${plural(s.flights, "flight")}${s.since ? ` since ${when(s.since, now)}` : ""}: ${duration(s.seconds)} in the air, ${s.distance.toFixed(0)} m flown`
      + `${top ? `, most of it in ${theRoom(this.roomName(top[0]))}` : ""}; ${plural(s.sightings, "sighting")}${labels ? ` (${labels})` : ""}; `
      + `${plural(this.changes.length, "change")} from the scan (${s.changes.suspected} waiting, ${s.changes.confirmed} confirmed, ${s.changes.dismissed} dismissed).`;
  }

  digest(now = this.now()) {
    if (!this.flights.length && !this.sightings.length) return "I don't remember any flights yet.";
    const out = [this.flights.length ? this.flightLine(this.flights.at(-1), now) : ""];
    const latest = [...new Set(this.sightings.map((s) => s.label))].map((l) => this.sightings.findLast((s) => s.label === l)).sort((a, b) => b.t - a.t).slice(0, 4);
    if (latest.length) out.push(`Last seen: ${latest.map((s) => `${this.thing(s)} ${this.where(s)} ${when(s.t, now)}`).join("; ")}.`);
    const open = this.changes.filter((c) => c.status === "suspected").length;
    if (open) out.push(`${plural(open, "possible change")} from the scan not confirmed yet.`);
    return out.filter(Boolean).join(" ");
  }
}

const cap = (s) => s.charAt(0).toUpperCase() + s.slice(1);
const confirmText = (claude) => (claude === "confirmed" ? ", confirmed by Claude" : claude === "likely" ? ", which Claude thought likely" : claude === "rejected" ? ", though Claude doubted it" : "");
const simFlight = (f) => f?.world === "sim" || f?.kind === "sim";

// The smallest disc [x, y, r] around two discs.
export function enclose(a, b) {
  const d = Math.hypot(b[0] - a[0], b[1] - a[1]);
  if (d + b[2] <= a[2]) return a;
  if (d + a[2] <= b[2]) return b;
  const R = (d + a[2] + b[2]) / 2, k = (R - a[2]) / d;
  return [r2(a[0] + k * (b[0] - a[0])), r2(a[1] + k * (b[1] - a[1])), Math.ceil(R * 100 + 1) / 100];
}
// A thing's reports as discs, none inside another, at most 8 (the nearest two then one disc around both).
function tidySpots(spots) {
  const out = [];
  for (const a of spots) {
    if (out.some((b) => Math.hypot(a[0] - b[0], a[1] - b[1]) + a[2] <= b[2])) continue;
    for (let i = out.length - 1; i >= 0; i--) if (Math.hypot(a[0] - out[i][0], a[1] - out[i][1]) + out[i][2] <= a[2]) out.splice(i, 1);
    out.push([r2(a[0]), r2(a[1]), Math.ceil(a[2] * 100) / 100]);
  }
  while (out.length > 8) {
    let pair = [0, 1], bd = Infinity;
    for (let i = 0; i < out.length; i++) for (let j = i + 1; j < out.length; j++) {
      const d = Math.hypot(out[i][0] - out[j][0], out[i][1] - out[j][1]);
      if (d < bd) [pair, bd] = [[i, j], d];
    }
    out.splice(pair[0], 1, enclose(out[pair[0]], out.splice(pair[1], 1)[0]));
  }
  return out;
}

// Distance from (x, y) to a polygon's outline.
function outlineDist(x, y, poly) {
  let best = Infinity;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [ax, ay] = poly[j], dx = poly[i][0] - ax, dy = poly[i][1] - ay, k = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy || 1)));
    best = Math.min(best, Math.hypot(ax + k * dx - x, ay + k * dy - y));
  }
  return best;
}

export function nearestDoor(doors, x, y, within = Infinity) {
  let best = null, bd = within;
  for (const d of doors) {
    const ax = d.b[0] - d.a[0], ay = d.b[1] - d.a[1], L2 = ax * ax + ay * ay || 1, k = Math.max(0, Math.min(1, ((x - d.a[0]) * ax + (y - d.a[1]) * ay) / L2));
    const dist = Math.hypot(d.a[0] + k * ax - x, d.a[1] + k * ay - y);
    if (dist < bd) [best, bd] = [d, dist];
  }
  return best;
}
