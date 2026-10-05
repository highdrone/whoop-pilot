// Suspected changes to the house (docs/HOME-DRONE.md, "Wave C contracts"): each live depth frame (vision/depth.js,
// metric and aligned at the localized pose) against the twin's empty-house view from the same pose ({ actors: false,
// props: false }), on a coarse grid with people and pets masked out (their boxes, padded). The expected view is first
// shifted onto the live one (a small pose, lens or uptilt error shows as a shift of the whole picture). A cell is
// nearer than expected (something new, a closed door) or farther (something gone, an open door) when its depth, after a
// smooth bias surface is taken off (monocular depth bends whole walls by 10-30%), is nearer (farther) than anything the
// twin expects in and around it, and, with the pictures, it also looks different (differences(), appearance(): grain,
// compression and exposure of the O4's video don't count). Such cells make blobs; each blob is a sighting along a ray
// from the camera with a range that can be far off for a new thing (monocular depth: 30-60% under the O4's video), and
// sightings of one thing from several places pin it down by their rays (ChangeDetector.track). One seen on
// CHANGES.frames frames over CHANGES.span ms, placed to within CHANGES.place m, at least CHANGES.minSize across, whose
// sightings agree in 3D (CHANGES.agree of them pass the place) from viewpoints at least CHANGES.baseline m apart (turning
// on the spot triangulates nothing), is a suspected change { id, kind: "obstacle" | "door-closed" | "door-open" | "moved"
// | "gone", x, y, z, room, size, sigma, zMin, zMax, door?, from? (moved), evidence: { live, expected, box } (JPEG blobs,
// in the browser, the blob outlined; box: where), t }; one that fills a doorway is a door, on its plane.
// The 3D scan has a say (map.vox, house/voxels.js, as captured: what flights alone found left out): a blob counts only
// where it contradicts what the scan knew. Something new must stand in space the scan saw free (in unknown space the scan
// never knew what was there, and the planner already keeps out); something gone must have been a surface the scan had (a
// smear or floater the twin renders is not one); never at a faint layer (FLAG.FAINT: glass, an aquarium, a glossy top,
// sheer curtains), where the twin's expected depth means little. Below the trust input (trust(frame): an uncalibrated
// camera, no pad check, no fresh vision fix; changeTrust() builds the app's) ingest() does nothing.
// Something moving (a person the detector missed) never lines up from view to view, so it never builds up; one
// staying put whom the detector boxed on some frames and missed on others isn't reported either (until they go): a
// candidate CHANGES.boxed of whose sightings point into a person's or pet's box of a frame within CHANGES.boxedNear ms of
// them, at about the range the box's live depth gives (the detector saw a person there just before or after), is them; one
// placed where the camera itself was while seeing it is no solid thing either. A change goes to memory.addChange()
// (memory/memory.js keeps it, puts the blocking kinds on the map as temporary obstacles until they are confirmed or
// dismissed, and says null for a spot the user dismissed); without a memory this puts them on the map itself (through
// nav/avoid.js) until resolve(). A reported change seen again tells the memory
// (memory.seen(id), at most every CHANGES.renew ms: one Claude took for a person or pet keeps blocking while it stays). A spot
// reported before that now looks as the scan expects (on CHANGES.frames frames) goes to memory.noteUnchanged(). Frames
// are used only from a good pose (σ under CHANGES.maxSigma, its height's σz under CHANGES.maxZSigma) and when most of the
// view agrees with the twin (else the pose or the alignment is off, not the house).
// The pose's error has its say too (heldUp()): the expected view is rendered where the drone thinks it is, and a height
// 0.2 m off makes a furniture top's edge stand where the render sees past it (or the other way round), from every
// viewpoint alike, so no 3D agreement can tell. A differing cell counts only if it still differs from the expected view
// seen from the camera moved up and down by 2 σz (σz at least CHANGES.zSigma, but for a pose as sure as the simulator's
// truth: CHANGES.exact) and sideways by 2 σ, and a blob's live points
// must stand in space the scan saw free from all of those; cells by an edge of the expected view (its depth jumps there)
// must also stand up to CHANGES.edgeZ more height error. A blob where the pose's error explains most of the differing
// cells it is part of, a new one most of whose points are just over (or under) a surface the scan has off the floor (where
// a larger height error than swept would leave what is left of a phantom along a furniture top's edge), or one touching a
// faint layer is weak evidence: a candidate seen mostly so needs CHANGES.weakFrames frames over CHANGES.weakSpan ms and is
// reported weak (Claude calling it no change may drop it: ai/inspect.js); never a well-evidenced one.
import { Emitter } from "../util.js";
import { intrinsics, unproject, project } from "../twin/lens.js";
import { droneCamera } from "../twin/pose.js";
import { enclose } from "../memory/memory.js";

// house/voxels.js's states and flags (not imported: the depth worker loads this module for differences())
const UNKNOWN = 0, FREE = 1, OCCUPIED = 2, CARVED = 4 | 8, FAINT = 128;

export const CHANGES = {
  grid: 35, // cells across the image (square cells)
  abs: 0.15, rel: 0.2, // a cell's depth differs when it is off the bias surface by more than max(abs, rel x expected)
  colour: 30, ncc: 0.3, texture: [1.5, 6], spread: 2, light: 4, // and it looks different: a mean colour `colour` levels off
  // the expected cell's and its neighbours' (after the brightness and colour of the light around it: a gain per channel,
  // the median over the block of `light` x `light` cells it is in and the 8 around), gradients (blurred over 2 x spread + 1 half-size px) correlating below ncc over
  // 3x3 cells where both are textured, or texture in one and not the other (gradient energy over texture[1], under
  // texture[0] x its picture's noise floor)
  minConf: 0.5,
  maxRange: 5, // m: farther pixels aren't compared
  minCells: 3, // per blob, at least 2 cells across both ways
  maxSigma: 0.1,
  zSigma: 0.1, maxZSigma: 0.15, // m: the height's σ taken when a frame (or the trust input) says less or nothing (vision
  // fixes count heights at σz >= 0.1 m; rehearsals measured 0.2-0.3 m errors at σz 0.1), and above which frames are skipped
  exact: 0.025, // m: a pose this sure (σ: the simulator's own truth, 0.02; vision's in flight was 0.03-0.11) is as sure of its height
  jump: 0.25, edgeZ: 0.1, weakZ: 0.2, // an edge of the expected view: its depth jumps by this share between samples 2-4 px
  // apart; cells by one also stand up to edgeZ m more height error; a new thing's points within 2 σz + weakZ m over (or
  // under: a shelf, the ceiling) a surface the scan has, off the floor, are weak evidence (perched())
  weakFrames: 12, weakSpan: 3000, // what a candidate seen mostly as weak evidence needs: frames, ms
  maxOff: 0.3, // share of differing cells above which the frame is skipped
  frames: 6, farFrames: 9, span: 1500, // farther than expected needs more: monocular depth often reads glass, mirrors and
  // thin things as far
  baseline: 0.3, agree: 0.6, keepViews: 60, // m between the farthest two viewpoints whose sightings agree in 3D; the share of
  // sightings that must pass the fitted place; sightings kept per candidate
  scanShare: 0.5, faintShare: 0.1, scanRays: 48, // a blob's rays (up to scanRays of them) that must contradict the 3D scan,
  // and at most this share touching a faint layer (sim-tuned)
  fixAge: 1000, // ms: on the real drone (and the simulator flying on vision), changes only within this of an applied vision
  // fix (changeTrust)
  shift: 16, // px: the largest shift of the expected view onto the live one searched
  depthErr: 0.5, bearing: 0.02, // a new thing's live depth error (share of the range; frames may say: f.depthErr; nav/avoid.js
  // uses the same: 60-100% at worst in tools/depth-check.html on the twin's renders with the O4's blur, noise and
  // compression added) and a blob's bearing error (rad)
  place: 0.15, // m: a candidate is reported once its place is known this well (views from different sides)
  placeErr: 0.25, // m: the least σ a report says it has: placed boxes came out 0.19-0.37 m off in tools/depth-check.html
  // with σ under `place` (sim-tuned, like depthErr, place and the scan shares: re-measure on the first real recordings)
  sameRay: 0.17, // rad: views of a candidate this close in direction share their depth error
  minSize: 0.25, // m across
  body: 0.1, // m: the drone's own half size (a new thing the camera was inside of while seeing it isn't one)
  forget: 8000, // ms without a sighting
  door: 0.25, doorRel: 0.15, doorShare: 0.6, // a door change: in a doorway on most frames (its live depth on the doorway's
  // plane: in front of it within max(door m, doorRel x the range), the aligned depth's own error beside the jambs, not a
  // new thing's; behind it as far as a new thing's depth can read too far: a closed door is new to the scan), placed on
  // that plane (within max(door m, 2 sigma), at most 0.5 m), and, in the median view, at least doorShare of the
  // opening in the picture closed (opened) on its plane; else something in or in front of the doorway
  pad: 0.15, // mask padding, share of the box
  movedWithin: 3, // m between the gone and the new place of one thing
  evidence: 400, // px: the long side of an evidence picture at least
  renew: 3000, // ms: a reported change seen again tells the memory at most this often (it keeps blocking)
  boxed: 2, boxedNear: 1000, // sightings of a candidate pointing into people and pet boxes of frames boxedNear ms from them:
  // it is one of them
};

let ids = 0;

export class ChangeDetector extends Emitter {
  // render(pose, view): the twin's empty-house picture ({ width, height, data } RGBA, e.g. twin.pixels) for the evidence
  // at the size of the frames' hires picture; avoid: nav/avoid.js (its temporary obstacles, without a memory), else
  // map.addTemp directly; trust(frame) -> true | false | { ok, why } (changeTrust()), none: every frame.
  constructor({ map, house, memory = null, avoid = null, render = null, trust = null }) {
    super();
    Object.assign(this, { map, house, avoid, render, trust, untrusted: "" });
    this.candidates = [];
    this.reported = []; // what this reported: { ...change, memoryId, near, agree, renewed, spots? (later reports of the same thing) }
    this.watch = new Map(); // the memory's other open changes' reports looked at: "id|i" -> { x, y, z, size, agree }
    this.boxed = []; // the last frames with people or pets boxed: [{ t, pose, lens, width, height, boxes }]
    this.stats = { frames: 0, used: 0, skipped: { pose: 0, height: 0, off: 0, trust: 0 }, blobs: 0, scan: 0, unagreed: 0, changes: 0, poseErr: 0 };
    this.setMemory(memory);
  }

  setTrust(trust) {
    this.trust = trust;
  }

  setMap(map, house = this.house) {
    Object.assign(this, { map, house, candidates: [] });
  }

  // memory/memory.js "change" { ...record, change, phase }: resolved, it is no longer ours (a merged report stays reported).
  // A new memory (another house or world) starts afresh.
  setMemory(memory) {
    if (memory === this.memory) return;
    this.unsub?.();
    Object.assign(this, { memory, reported: this.memory ? [] : this.reported });
    this.watch.clear();
    this.unsub = memory?.on?.("change", (e = {}) => (e.phase === "cleared" ? (this.reported = []) : /^(confirmed|dismissed)$/.test(e.phase ?? e.status) && this.forget((e.change ?? e).id)));
  }

  dispose() {
    this.unsub?.();
  }

  // One frame: { t, pose (H, sigma, zSigma?, status), width, height, lens, depth, conf?, expected, expectedRgb?, boxes?
  // (people and pets, normalised in this view), mask? (1 = ignore: the OSD), rgb? ({ width, height, data }: the rectified
  // live picture) }; without the pose's zSigma, the trust input's (changeTrust: the localizer's now). Returns this frame's
  // blobs.
  ingest(f, now = performance.now()) {
    this.stats.frames++;
    const tr = this.trust ? this.trust(f) : true;
    if (!(tr === true || tr?.ok)) return (this.stats.skipped.trust++, (this.untrusted = tr?.why ?? "not trusted"), []);
    this.untrusted = "";
    if (!f.expected || !(f.pose.sigma <= CHANGES.maxSigma) || (f.pose.status && f.pose.status !== "ok")) return (this.stats.skipped.pose++, []);
    const zs = Math.max(f.pose.zSigma ?? tr?.zSigma ?? 0, f.pose.sigma <= CHANGES.exact ? f.pose.sigma : CHANGES.zSigma);
    if (zs > CHANGES.maxZSigma) return (this.stats.skipped.height++, []);
    const made = (f.grid ??= differences(f)), fr = made.shift ? { ...f, expected: made.expected, expectedRgb: made.expectedRgb } : f;
    if (made.off > CHANGES.maxOff) return (this.stats.skipped.off++, []);
    this.stats.used++;
    const grid = heldUp(made.lv ? made : differences(f), fr, { z: zs, xy: f.pose.sigma }); // f.grid stays as made (nav/avoid.js reads it)
    this.stats.poseErr += grid.posed;
    if (fr.boxes?.length) (this.boxed.push({ t: now, pose: fr.pose, lens: fr.lens, width: fr.width, height: fr.height, boxes: fr.boxes.map((b) => ({ ...b, d: boxDepth(fr, b) })) }), this.boxed.length > 60 && this.boxed.shift());
    const all = blobsOf(grid, fr, this.map, this.house), blobs = all.filter((b) => !b.scan || b.scan.ok);
    this.stats.blobs += blobs.length;
    this.stats.scan += all.length - blobs.length;
    for (const b of blobs) this.track(b, fr, now);
    this.unchanged(grid, fr, now);
    this.candidates = this.candidates.filter((c) => now - c.t < CHANGES.forget);
    return blobs;
  }

  // A blob into the candidate it lines up with (its ray passes the candidate's place within the blob's size and the
  // errors across it, at a range within its depth error), else a new one. Each candidate is a least-squares point: every
  // sighting a ray from the camera with errors across it (ss sideways, su up and down) and a range along it (sd: monocular
  // depth of a new thing is often 30-60% off, the twin's expected depth is not), so views from different sides fix the
  // place and views along one line leave it uncertain (sigma, m). A new thing's depth error is much the same in every
  // view along one line (flying at it, the model reads it the same way), so a view whose ray is within CHANGES.sameRay of
  // k earlier ones adds range with its sd x (1 + k): views along one line add up to little more than one, and only views
  // from different sides pin the place down. Blobs at a reported change are that change seen again.
  track(b, f, now) {
    const fit = (q) => {
      const v = [0, 1, 2].map((i) => q.X[i] - b.c[i]), a = v[0] * b.r[0] + v[1] * b.r[1] + v[2] * b.r[2], e = 2 * Math.min(1, q.sigma ?? 0) + q.size / 2 + 0.15;
      return Math.max(Math.abs(v[0] * b.s[0] + v[1] * b.s[1]) / (2 * Math.min(b.ss, 1) + e), Math.abs(a - b.d) / (2.5 * b.sd + e));
    };
    // a door change is seen again only in its own doorway (it is as wide as the doorway: anything near would match)
    const again = this.reported.find((r) => r.near === b.near && (r.door ? b.door?.id === r.door : spotsOf(r).some((p) => fit({ X: [p.x, p.y, p.z], size: p.size }) < 1)));
    if (again) return void this.renew(again, now);
    const c = this.candidates.filter((q) => !q.done && q.near === b.near && fit(q) < 1).sort((p, q) => fit(p) - fit(q))[0];
    const same = c && b.near ? c.views.filter((v) => v.r[0] * b.r[0] + v.r[1] * b.r[1] + v.r[2] * b.r[2] > Math.cos(CHANGES.sameRay)).length : 0;
    const sd = b.sd * (1 + same), Ai = [0, 1, 2].map((i) => [0, 1, 2].map((j) => (b.s[i] * b.s[j]) / b.ss ** 2 + (b.u[i] * b.u[j]) / b.su ** 2 + (b.r[i] * b.r[j]) / sd ** 2));
    const Bi = [0, 1, 2].map((i) => Ai[i].reduce((s, v, j) => s + v * b.c[j], 0) + (b.r[i] * b.d) / sd ** 2), w = 1 / Math.max(0.5, b.d) ** 2, ws = b.cut ? 0 : w;
    const door = b.door && { id: b.door.id, n: 1, cover: b.door.cover ? [b.door.cover] : [] };
    const edged = b.cut || b.cutV, shot = b.cells * (edged ? 0.3 : 1); // the evidence: the view with the most of it in the picture
    const view = { c: b.c, r: b.r, s: b.s, u: b.u, d: b.d, sd: b.sd, ss: Math.min(b.ss, 1), su: Math.min(b.su, 1), t: now };
    if (!c) return void this.candidates.push({ ...b, X: [b.x, b.y, b.z], A: Ai, B: Bi, w, wSize: ws, shot, n: 1, t0: now, t: now, frame: f, from: f.pose, base: 0,
      sigma: Infinity, views: [view], doors: door ? { [door.id]: door } : {}, cuts: +edged, cutBest: edged, weaks: +!!b.weak });
    c.views.push(view);
    if (c.views.length > CHANGES.keepViews) c.views.shift();
    c.A = c.A.map((row, i) => row.map((v, j) => v + Ai[i][j]));
    c.B = c.B.map((v, i) => v + Bi[i]);
    const inv = inv3(c.A);
    if (inv) {
      c.X = inv.map((row) => row.reduce((s, v, j) => s + v * c.B[j], 0));
      const [a, d, e] = [inv[0][0], inv[1][1], inv[0][1]];
      c.sigma = Math.sqrt((a + d) / 2 + Math.sqrt(((a - d) / 2) ** 2 + e * e)); // the longer axis of its place on the plan
    }
    [c.x, c.y, c.z] = c.X;
    if (door) { // how much of a doorway each view saw it cover (fills() takes the median view, never a union of them)
      const h = c.doors[door.id];
      c.doors[door.id] = h ? { ...h, n: h.n + 1, cover: [...h.cover, ...door.cover] } : door;
    }
    const W = c.w + w, mix = (k) => (c[k] * c.w + b[k] * w) / W; // a blob cut at the sides is narrower than the thing
    c.size = c.wSize ? (c.size * c.wSize + b.size * ws) / (c.wSize + ws) : ws ? b.size : Math.max(c.size, b.size);
    Object.assign(c, { zMin: mix("zMin"), zMax: mix("zMax"), w: W, wSize: c.wSize + ws, n: c.n + 1, cuts: c.cuts + edged, weaks: (c.weaks ?? 0) + !!b.weak, t: now, room: this.map.roomAt(c.x, c.y)?.id ?? null });
    if (shot >= c.shot) Object.assign(c, { shot, frame: f, px: b.px, cutBest: edged });
    c.base = Math.max(c.base, Math.hypot(f.pose.x - c.from.x, f.pose.y - c.from.y));
    // seen mostly as weak evidence (what the pose's error doesn't quite explain, or at a faint layer): more of it
    const dr = this.fills(c), weak = !dr && 2 * c.weaks > c.n;
    if (!c.done && (dr || c.room) && c.n >= (weak ? CHANGES.weakFrames : c.near ? CHANGES.frames : CHANGES.farFrames) && now - c.t0 >= (weak ? CHANGES.weakSpan : CHANGES.span)
      && (dr || (c.size >= CHANGES.minSize && c.sigma <= CHANGES.place)) && c.base >= CHANGES.baseline) {
      if (dr || this.agreed(c)) this.report(c, dr, now);
      else this.stats.unagreed++;
    }
  }

  // Do the sightings agree in 3D? At least CHANGES.agree of them pass the fitted place (sideways within twice their
  // bearing error, the thing's half size and 0.1 m; up and down within twice theirs and half its height; along the ray
  // within 2.5 sd and 0.1 m), and the viewpoints of those that do are at least CHANGES.baseline m apart (two views from one
  // spot, or along one line from far off, place nothing).
  agreed(c) {
    const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2], h = Math.max(0, (c.zMax ?? 0) - (c.zMin ?? 0)) / 2;
    const ok = c.views.filter((v) => {
      const p = [0, 1, 2].map((i) => c.X[i] - v.c[i]);
      return Math.abs(dot(p, v.s)) <= 2 * v.ss + c.size / 2 + 0.1 && Math.abs(dot(p, v.u)) <= 2 * v.su + h + 0.1 && Math.abs(dot(p, v.r) - v.d) <= 2.5 * v.sd + 0.1;
    });
    if (ok.length < CHANGES.agree * c.views.length) return false;
    const apart = (p, q) => Math.hypot(p.c[0] - q.c[0], p.c[1] - q.c[1], p.c[2] - q.c[2]);
    for (let i = 0; i < ok.length; i++) for (let j = i + 1; j < ok.length; j++) if (apart(ok[i], ok[j]) >= CHANGES.baseline) return true;
    return false;
  }

  // The doorway a candidate was seen in on most of its frames, if it lies on the doorway's plane (something standing in
  // front of a doorway doesn't, once views from two places have placed it) and the median view there had at least
  // doorShare of the opening in its picture closed (opened): then it is a door, at the middle of what changed on the
  // doorway's plane, sill to head. -> { id, x, y, z, size, zMin, zMax, room } | null
  fills(c) {
    const mid = (a) => [...a].sort((p, q) => p - q)[a.length >> 1];
    for (const h of Object.values(c.doors)) {
      const dr = this.house?.doors?.find((q) => q.id === h.id);
      if (!dr || 2 * h.n <= c.n || h.cover.length < 3) continue;
      const ex = dr.b[0] - dr.a[0], ey = dr.b[1] - dr.a[1], wide = Math.hypot(ex, ey), u = Math.max(0, Math.min(1, ((c.x - dr.a[0]) * ex + (c.y - dr.a[1]) * ey) / wide ** 2));
      if (Math.hypot(c.x - dr.a[0] - u * ex, c.y - dr.a[1] - u * ey) > Math.max(CHANGES.door, Math.min(2 * c.sigma, 0.5))) continue;
      if (mid(h.cover.map((v) => v.share)) < CHANGES.doorShare) continue;
      const s = [mid(h.cover.map((v) => v.s[0])), mid(h.cover.map((v) => v.s[1]))], m = (s[0] + s[1]) / 2, x = dr.a[0] + m * ex, y = dr.a[1] + m * ey, [z0, z1] = [dr.sillZ ?? 0, dr.headZ ?? 2];
      return { id: dr.id, x, y, z: (z0 + z1) / 2, size: (s[1] - s[0]) * wide, zMin: z0, zMax: z1, room: this.map.roomAt(x, y)?.id ?? c.room };
    }
    return null;
  }

  // How many of a candidate's sightings point (along their ray, at their range) into a person's or pet's box (padded by
  // CHANGES.pad) of a frame taken within CHANGES.boxedNear ms of them, at about the range of the box's live depth.
  boxedViews(c) {
    const inBox = (q, X) => {
      const K = intrinsics(q.lens, q.width, q.height), cam = droneCamera(q.pose, q.lens.uptiltDeg ?? 0), v = [0, 1, 2].map((a) => X[a] - cam.p[a]), L = Math.hypot(...v);
      const p = [0, 1, 2].map((a) => cam.R[0][a] * v[0] + cam.R[1][a] * v[1] + cam.R[2][a] * v[2]), px = p[2] > 0.2 && project(K, p);
      return px && q.boxes.some((b) => { const u = px[0] / q.width, w = px[1] / q.height; return u > b.x - CHANGES.pad * b.w && u < b.x + b.w * (1 + CHANGES.pad) && w > b.y - CHANGES.pad * b.h && w < b.y + b.h * (1 + CHANGES.pad) && !(Math.abs(b.d - L) > Math.max(0.5, 0.3 * L)); });
    };
    return (c.views ?? []).filter((v) => this.boxed.some((q) => Math.abs(q.t - v.t) <= CHANGES.boxedNear && inBox(q, [0, 1, 2].map((a) => v.c[a] + v.r[a] * v.d)))).length;
  }

  async report(c, dr, now) {
    // a person (or pet) the detector boxed there on other frames just now: not a change (asked again on its next sighting)
    if (!dr && c.near && this.boxedViews(c) >= CHANGES.boxed) return void (this.stats.people = (this.stats.people ?? 0) + 1);
    // the camera was inside it while seeing it (within its half size and CHANGES.body, at a height it spans): no solid thing
    // (rehearsals placed some at the drone's own turning spot; asked again on its next sighting)
    const inside = (v) => Math.hypot(v.c[0] - c.x, v.c[1] - c.y) < c.size / 2 + CHANGES.body && v.c[2] > c.zMin - CHANGES.body && v.c[2] < c.zMax + CHANGES.body;
    if (!dr && c.near && (c.views ?? []).some(inside)) return void (this.stats.flown = (this.stats.flown ?? 0) + 1);
    c.done = true;
    if (dr) Object.assign(c, dr, { door: dr.id });
    // A doorway is reported once. A thing whose outline is within twice the larger σ of a reported thing's (any report of
    // it) is that thing again: no second change is raised, but the report still goes to the memory (or our own obstacle),
    // where what the thing blocks grows to hold every report of it; a thing further off is a thing of its own.
    const sig = (q) => (Number.isFinite(q.sigma) ? q.sigma : 0);
    if (dr && this.reported.some((r) => r.near === c.near && r.door === dr.id)) return;
    const same = (!dr && this.reported.find((r) => r.near === c.near && !r.door
      && spotsOf(r).some((p) => Math.hypot(p.x - c.x, p.y - c.y) <= (p.size + c.size) / 2 + 2 * Math.max(sig(p), sig(c))))) || null;
    // one placed where a report of it already is (within CHANGES.place, no bigger) adds nothing: that change seen again
    if (same && spotsOf(same).some((p) => Math.hypot(p.x - c.x, p.y - c.y) <= CHANGES.place && c.size <= p.size + 0.1)) return void this.renew(same, now);
    // The other half of a moved thing: a candidate of the other kind nearby in the same room, about as big.
    const other = !dr && !same && this.candidates.find((q) => q !== c && q.near !== c.near && q.n >= 3 && q.room === c.room && !q.done && !this.fills(q) &&
      Math.hypot(q.x - c.x, q.y - c.y) < CHANGES.movedWithin && Math.max(q.size, c.size) < 2 * Math.min(q.size, c.size));
    if (other) other.done = true;
    const to = c.near ? c : other, from = c.near ? other : c;
    const kind = other ? "moved" : dr ? (c.near ? "door-closed" : "door-open") : c.near ? "obstacle" : "gone";
    // Heights: the place's within what was seen of it (a blob cut at the frame's top or bottom says nothing about it, so
    // its fitted height can be anywhere: then the middle); a new thing stands on the floor (its foot may be out of view).
    const at = to ?? from, r2 = (v) => +v.toFixed(2), zIn = (q) => (q.z >= q.zMin && q.z <= q.zMax ? q.z : (q.zMin + q.zMax) / 2);
    const zMin = Math.min(at.zMin, (!dr && to ? this.map.floorAt(at.x, at.y) : null) ?? Infinity), sigma = dr || !Number.isFinite(at.sigma) ? null : r2(at.sigma);
    // a new thing seen mostly cut at the picture's edge is wider than it looked; σ said at least placeErr (it is optimistic)
    const size = at === to && 2 * to.cuts > to.n ? Math.max(at.size, 2 * CHANGES.minSize) : at.size;
    const weak = !dr && 2 * at.weaks > at.n; // what blocks (or is gone) seen mostly as weak evidence
    const change = { id: `change-${Date.now().toString(36)}-${++ids}`, kind, x: r2(at.x), y: r2(at.y), z: r2(zIn(at)), room: at.room, size: r2(size),
      sigma: sigma == null ? null : Math.max(sigma, CHANGES.placeErr), zMin: r2(zMin), zMax: r2(at.zMax), ...(dr && { door: dr.id }), ...(weak && { weak }),
      ...(other && { from: { x: r2(from.x), y: r2(from.y), z: r2(zIn(from)) } }), t: Date.now(), source: "live depth" };
    const mine = { ...change, sigma, memoryId: null, near: !!to, agree: 0, renewed: now };
    if (same) (same.spots ??= []).push(mine);
    else this.reported.push(mine); // before anything async: the next frame's blobs here are this change seen again
    change.evidence = await this.evidence(at).catch(() => null);
    let rec;
    try {
      rec = this.memory?.addChange ? await this.memory.addChange(change) : undefined;
    } catch (e) {
      console.warn(`memory.addChange: ${e.message}`);
    }
    mine.memoryId = rec?.id ?? null;
    if (rec === null) return; // a spot the user dismissed: not raised again
    // where it belongs: a report of a change already reported (the memory merged it into one), or a change of its own
    const home = (this.memory ? rec && this.reported.find((r) => r !== mine && r.memoryId === rec.id) : same) || null;
    if (home !== same) {
      if (same) same.spots = same.spots.filter((p) => p !== mine);
      else this.reported = this.reported.filter((r) => r !== mine);
      if (home) (home.spots ??= []).push(mine);
      else this.reported.push(mine);
    }
    if (home) return void (!this.memory && this.cover(home, now));
    this.stats.changes++;
    if (!this.memory && to && kind !== "door-open") this.cover(mine, now);
    this.emit("change", { ...change, memoryId: mine.memoryId });
  }

  // Without a memory, what a reported thing blocks, on the map (through nav/avoid.js): one disc around a disc at each of
  // its reports as large as it, its place's error (σ, at least placeErr; a door's is its doorway's) twice over and 0.1 m
  // (as memory.tempOf()).
  cover(r, now) {
    const ps = spotsOf(r), disc = (p) => [p.x, p.y, Math.max(0.15, p.size / 2 + 0.1 + (p.door ? 0 : 2 * Math.max(p.sigma ?? 0, CHANGES.placeErr)))];
    const [x, y, rr] = ps.map(disc).reduce(enclose);
    const temp = { id: r.id, kind: "change", change: r.kind, x, y, r: rr, zMin: Math.min(...ps.map((p) => p.zMin)) - 0.1, zMax: Math.max(...ps.map((p) => p.zMax)) + 0.1, until: Infinity };
    if (this.avoid) this.avoid.addTemp(temp, now);
    else this.map.addTemp?.(temp);
  }

  // A reported change seen again: the memory hears of it (at most every CHANGES.renew ms) and keeps it blocking.
  renew(r, now) {
    if (!r.memoryId || now - r.renewed < CHANGES.renew) return;
    r.renewed = now;
    try {
      this.memory?.seen?.(r.memoryId);
    } catch (e) {
      console.warn(`memory.seen: ${e.message}`);
    }
  }

  // Confirmed (the memory updates the house) or dismissed: off the map either way, and no longer ours.
  resolve(id) {
    if (this.avoid) this.avoid.removeTemp(id);
    else this.map.removeTemp?.(id);
    this.forget(id);
  }
  forget(id) {
    this.reported = this.reported.filter((r) => r.id !== id && r.memoryId !== id);
  }

  // The open changes this frame sees (every report of each: ours, and the memory's others, from earlier flights). Only a
  // spot in plain view counts (the scan has nothing nearer along that ray): one hidden behind a wall says nothing. Nothing
  // different at any of a change's reports in view: the memory hears for how long (memory.unseen(): one the camera keeps
  // not seeing stops blocking). Each report as the scan expects on CHANGES.frames frames running: memory.noteUnchanged().
  // As expected: the depth there (bias taken off) within CHANGES.rel of the scan's, and it looks the same; not just "not
  // nearer than anything around it" (a closed door's middle can be as near as the jambs beside it). A spot is looked at
  // over the heights it was reported at (heightsOf()), and counts as seen only when its lower part is in view: a box 0.8 m
  // tall whose report was fitted at 1.3 m (its depth read far) had been "no longer there" from a view over its top.
  unchanged(grid, f, now) {
    const dt = Math.min(500, Math.max(0, now - (this.usedAt ?? now))), mine = new Set(this.reported.map((r) => r.memoryId).filter(Boolean)), keys = new Set();
    this.usedAt = now;
    const looks = this.reported.map((r) => ({ id: r.memoryId, spots: spotsOf(r) }));
    for (const c of this.memory?.changes ?? []) {
      if (c.status !== "suspected" || mine.has(c.id) || !Number.isFinite(c.x)) continue;
      const spots = (this.memory.spotsOf?.(c) ?? [[c.x, c.y, (c.size ?? 0.3) / 2]]).map(([x, y, h], i) => {
        const key = `${c.id}|${i}`, w = this.watch.get(key) ?? { agree: 0 };
        keys.add(key);
        return this.watch.set(key, Object.assign(w, { x, y, z: c.z ?? 1, zMin: c.zMin, zMax: c.zMax, size: 2 * h })).get(key);
      });
      looks.push({ id: c.id, spots });
    }
    for (const key of this.watch.keys()) if (!keys.has(key)) this.watch.delete(key);
    if (!looks.length) return;
    const K = intrinsics(f.lens, f.width, f.height), cam = droneCamera(f.pose, f.lens.uptiltDeg ?? 0), R = cam.R;
    for (const { id, spots } of looks) {
      let quiet = false, differs = false;
      for (const p of spots) {
        const hs = heightsOf(p), low = hs[0] + 0.4 * (hs.at(-1) - hs[0]);
        let seen = 0, lowSeen = false, odd = false, off = false;
        for (const z of hs) {
          const v = [p.x - cam.p[0], p.y - cam.p[1], z - cam.p[2]], c = [0, 1, 2].map((a) => R[0][a] * v[0] + R[1][a] * v[1] + R[2][a] * v[2]);
          const px = c[2] > 0.3 && c[2] < CHANGES.maxRange && project(K, c);
          if (!px || px[0] < 0 || px[1] < 0 || px[0] >= f.width || px[1] >= f.height) continue;
          const k = ((px[1] / grid.C) | 0) * grid.gw + ((px[0] / grid.C) | 0), e = f.expected[(px[1] | 0) * f.width + (px[0] | 0)];
          if (grid.n[k] < (grid.C * grid.C) / 4 || !(e >= Math.hypot(...v) - p.size / 2 - 0.25)) continue;
          (seen++, (lowSeen ||= z <= low + 1e-6));
          (odd ||= !!(grid.near[k] || grid.far[k] || grid.looks?.[k])), (off ||= !(Math.abs(grid.res[k]) < Math.log(1 + CHANGES.rel)));
        }
        if (!seen) continue;
        (differs ||= odd), (quiet ||= !odd && lowSeen);
        p.agree = odd || off ? 0 : p.agree + (lowSeen ? 1 : 0);
      }
      if (quiet && !differs && id) this.memory?.unseen?.(id, dt);
      const [p0] = spots;
      if (spots.every((p) => p.agree >= CHANGES.frames) && spots.some((p) => p.agree === CHANGES.frames))
        this.memory?.noteUnchanged?.({ x: p0.x, y: p0.y, z: p0.z, r: Math.max(0.15, p0.size / 2), id });
    }
  }

  // Live and expected crops of the blob as JPEG blobs (in the browser; null elsewhere): from the frame's hires picture
  // (else its rgb) and the twin's empty-house view at that size (render; else the frame's expectedRgb, scaled), 3 times
  // the blob and at least CHANGES.evidence px on the long side, the blob outlined on both and kept off the crop's edge (a
  // blob at the frame's edge: the crop goes up to a quarter past it, dark there). box: the blob in the crops (normalised
  // [x0, y0, x1, y1]); cut: the best view still had it at the frame's edge (only part of it in the pictures).
  async evidence(c) {
    const f = c.frame, pic = f?.hires ?? f?.rgb;
    if (!pic || typeof OffscreenCanvas === "undefined") return null;
    const [x0, y0, x1, y1] = c.px, cx = (x0 + x1) / 2, cy = (y0 + y1) / 2, k = pic.width / f.width;
    const w = Math.min(f.width, Math.max(3 * (x1 - x0), 4 * (y1 - y0), 0.35 * f.width)), h = Math.min(f.height, (w * 3) / 4);
    const bx = Math.max(-w / 4, Math.min(f.width - (3 * w) / 4, cx - w / 2)), by = Math.max(-h / 4, Math.min(f.height - (3 * h) / 4, cy - h / 2)), up = Math.max(1, CHANGES.evidence / (w * k));
    const W = Math.round(w * k * up), H = Math.round(h * k * up), box = [(x0 - bx) / w, (y0 - by) / h, (x1 - bx) / w, (y1 - by) / h].map((v) => +Math.min(1, Math.max(0, v)).toFixed(3));
    const sx0 = Math.max(0, bx), sy0 = Math.max(0, by), sx1 = Math.min(f.width, bx + w), sy1 = Math.min(f.height, by + h); // the part inside the frame
    const jpeg = async (px) => {
      const s = px.width / f.width, src = new OffscreenCanvas(px.width, px.height), out = new OffscreenCanvas(W, H), g = out.getContext("2d");
      src.getContext("2d").putImageData(new ImageData(new Uint8ClampedArray(px.data), px.width, px.height), 0, 0);
      g.fillStyle = "#1c1c1c";
      g.fillRect(0, 0, W, H);
      g.imageSmoothingQuality = "high";
      g.drawImage(src, sx0 * s, sy0 * s, (sx1 - sx0) * s, (sy1 - sy0) * s, ((sx0 - bx) / w) * W, ((sy0 - by) / h) * H, ((sx1 - sx0) / w) * W, ((sy1 - sy0) / h) * H);
      g.strokeStyle = "#ffd400";
      g.lineWidth = Math.max(2, W / 200);
      g.strokeRect(box[0] * W, box[1] * H, (box[2] - box[0]) * W, (box[3] - box[1]) * H);
      return out.convertToBlob({ type: "image/jpeg", quality: 0.85 });
    };
    const view = { width: pic.width, height: pic.height, lens: f.lens, actors: false, props: false };
    const expected = (this.render && (await Promise.resolve(this.render(f.pose, view)).catch(() => null))) || f.expectedRgb;
    return { live: await jpeg(pic), expected: expected && (await jpeg(expected)), box, cut: !!c.cutBest };
  }
}

// Per grid cell: usable pixels (n), the depth residual (ln live/expected less a robust quadratic surface over the image),
// whether it looks different from the expected picture, and nearer / farther than the expected envelope (the nearest and
// farthest expected depth in it and its 8 neighbours), after the expected view is shifted onto the live one; lv: the live
// depth's log less the bias surface, tol: the log tolerance (heldUp() tests the cells again with them).
// -> { C, gw, gh, n, res, looks, near, far, lv, tol, used, off, shift, expected, expectedRgb }
export function differences(f) {
  const { width: w, height: h, depth, conf } = f, C = Math.max(2, Math.round(w / CHANGES.grid)), gw = Math.ceil(w / C), gh = Math.ceil(h / C), N = gw * gh;
  const shift = register(f), expected = shift ? moved(f.expected, w, h, shift, 1, NaN) : f.expected, expectedRgb = shift ? { ...f.expectedRgb, data: moved(f.expectedRgb.data, w, h, shift, 4, 0) } : f.expectedRgb;
  const mask = f.mask ? Uint8Array.from(f.mask) : new Uint8Array(w * h);
  for (const b of f.boxes ?? []) {
    const px = b.w * CHANGES.pad, py = b.h * CHANGES.pad;
    for (let v = Math.max(0, Math.floor((b.y - py) * h)); v < Math.min(h, Math.ceil((b.y + b.h + py) * h)); v++)
      mask.fill(1, Math.max(0, Math.floor((b.x - px) * w)) + v * w, Math.min(w, Math.ceil((b.x + b.w + px) * w)) + v * w);
  }
  const vals = Array.from({ length: N }, () => []), lives = Array.from({ length: N }, () => []), exps = Array.from({ length: N }, () => []);
  const eMin = new Float32Array(N).fill(Infinity), eMax = new Float32Array(N).fill(-Infinity);
  for (let v = 0; v < h; v++)
    for (let u = 0; u < w; u++) {
      const i = v * w + u, a = depth[i], e = expected[i], k = ((v / C) | 0) * gw + ((u / C) | 0);
      if (e > 0) [eMin[k], eMax[k]] = [Math.min(eMin[k], e), Math.max(eMax[k], e)];
      if (mask[i] || !(a > 0) || !(e > 0) || Math.min(a, e) > CHANGES.maxRange || (conf && conf[i] < CHANGES.minConf)) continue;
      vals[k].push(Math.log(a / e));
      lives[k].push(a);
      exps[k].push(e);
    }
  const median = (a) => a.sort((x, y) => x - y)[a.length >> 1];
  const n = new Uint16Array(N), rc = new Float32Array(N), lm = new Float32Array(N), em = new Float32Array(N), min = (C * C) / 4, cells = [];
  for (let k = 0; k < N; k++) {
    n[k] = vals[k].length;
    if (n[k] < min) continue;
    [rc[k], lm[k], em[k]] = [median(vals[k]), median(lives[k]), median(exps[k])];
    cells.push(k);
  }
  const fit = smoothFit(cells, rc, n, gw, gh), res = new Float32Array(N), looks = appearance({ ...f, expectedRgb }, C, gw, gh, mask);
  const near = new Uint8Array(N), far = new Uint8Array(N), lv = new Float32Array(N).fill(NaN), tl = new Float32Array(N);
  let off = 0;
  for (const k of cells) {
    let lo = Infinity, hi = -Infinity;
    for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
      const c = (k % gw) + dc, r = ((k / gw) | 0) + dr, j = r * gw + c;
      if (c >= 0 && r >= 0 && c < gw && r < gh && eMax[j] > 0) [lo, hi] = [Math.min(lo, eMin[j]), Math.max(hi, eMax[j])];
    }
    const live = Math.log(lm[k]) - fit(k), tol = Math.max(Math.log(1 + CHANGES.rel), CHANGES.abs / em[k]), seen = !looks || looks[k];
    [res[k], lv[k], tl[k]] = [rc[k] - fit(k), live, tol];
    near[k] = live < Math.log(lo) - tol && seen ? 1 : 0;
    far[k] = live > Math.log(hi) + tol && seen ? 1 : 0;
    off += near[k] | far[k];
  }
  return { C, gw, gh, n, res, looks, near, far, lv, tol: tl, used: cells.length, off: cells.length ? off / cells.length : 0, shift, expected, expectedRgb };
}

// The differences that stand up to the pose's error: the expected view (the grid's, shifted as differences() shifted it)
// seen again from the camera moved up and down by 2 σz and σz, and sideways by 2 σ (xy), z-buffered every C/2 px (a moved
// view's hidden surfaces don't count, and where it would see what the render had hidden, no far cell stands); a near
// (far) cell stays so only if its live depth is still nearer (farther) than everything any of those views expects in it
// (in it only: differences() took the cells around it, unmoved, for a few px of misregistration). Cells by an edge of the
// expected view (edge: its depth jumps by CHANGES.jump between neighbouring samples in or around the cell) also stand up
// to 2 σz + CHANGES.edgeZ up and down (less for a surer height: an exact one needs nothing more).
// -> the grid with near, far, edge, near0 and far0 (as differences() found them), moves and wide (the offsets, for
// scanSays()), reach (2 σz + CHANGES.weakZ: perched()), posed (cells the pose's error explains)
export function heldUp(grid, f, { z = CHANGES.zSigma, xy = 0 } = {}) {
  const { C, gw, gh, near, far, lv, tol } = grid, N = gw * gh, cells = [];
  for (let k = 0; k < N; k++) if (near[k] || far[k]) cells.push(k);
  const dz = 2 * z, dl = 2 * (xy ?? 0), R = droneCamera(f.pose, f.lens.uptiltDeg ?? 0).R, fw = Math.hypot(R[0][2], R[1][2]) || 1, sx = -R[1][2] / fw, sy = R[0][2] / fw;
  const moves = [[0, 0, 0], ...[dz, -dz, dz / 2, -dz / 2].map((h) => [0, 0, h]), ...(dl > 0.01 ? [[sx * dl, sy * dl, 0], [-sx * dl, -sy * dl, 0]] : [])], dw = dz + CHANGES.edgeZ * Math.min(1, z / CHANGES.zSigma), wide = [[0, 0, dw], [0, 0, -dw]];
  const out = { ...grid, near: new Uint8Array(N), far: new Uint8Array(N), edge: new Uint8Array(N), near0: near, far0: far, moves, wide, reach: dz + CHANGES.weakZ, posed: 0 };
  if (!cells.length) return out;
  const { width: w, height: h } = f, E = grid.expected ?? f.expected, s = Math.max(1, C >> 1), ws = Math.ceil(w / s), hs = Math.ceil(h / s), K = intrinsics(f.lens, w, h), { rays, cell } = samples(K, s, C, gw);
  const lo = new Float32Array(N).fill(Infinity), hi = new Float32Array(N).fill(-Infinity), loW = new Float32Array(N).fill(Infinity), hiW = new Float32Array(N).fill(-Infinity);
  const jump = new Uint8Array(N), jr = 1 + CHANGES.jump, zb = new Float32Array(ws * hs), at = new Float32Array(3 * ws * hs), pin = K.model === "pinhole";
  const Ej = new Float32Array(ws * hs); // the expected range at each sample
  for (let v = 0, j = 0; v < h; v += s) for (let u = 0; u < w; u += s, j++) Ej[j] = E[v * w + u];
  for (let j = 0; j < ws * hs; j++) // (no allocations per sample or bin below: this runs on the page's thread every frame)
    for (let side = 0; side < 2; side++) {
      const q = side ? j + ws : j % ws < ws - 1 ? j + 1 : -1, r = Ej[q] / Ej[j];
      if (q >= 0 && q < ws * hs && Ej[j] > 0 && Ej[q] > 0 && !(r <= jr && r >= 1 / jr)) jump[cell[j]] = jump[cell[q]] = 1;
    }
  const lines = new Int16Array(2 * Math.max(ws, hs)), byEdge = (k) => {
    for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
      const c = (k % gw) + dc, r = ((k / gw) | 0) + dr;
      if (c >= 0 && r >= 0 && c < gw && r < gh && jump[r * gw + c]) return 1;
    }
    return 0;
  };
  for (const k of cells) out.edge[k] = byEdge(k);
  for (const [t, L, H] of [...moves.slice(1).map((t) => [t, lo, hi]), ...(cells.some((k) => out.edge[k]) ? wide.map((t) => [t, loW, hiW]) : [])]) {
    const tc = [0, 1, 2].map((a) => R[0][a] * t[0] + R[1][a] * t[1] + R[2][a] * t[2]);
    zb.fill(Infinity);
    at.fill(NaN);
    for (let v = 0, j = 0; v < h; v += s) // each sample in the moved view: bin x, y and range
      for (let u = 0; u < w; u += s, j++) {
        const e = Ej[j], x = rays[3 * j] * e - tc[0], y = rays[3 * j + 1] * e - tc[1], z = rays[3 * j + 2] * e - tc[2];
        if (!(e > 0) || !(z > 0.05)) continue;
        const px = pin ? null : project(K, [x, y, z]);
        if (pin) (at[3 * j] = (K.cx + (K.fx * x) / z) / s), (at[3 * j + 1] = (K.cy + (K.fy * y) / z) / s);
        else if (px) (at[3 * j] = px[0] / s), (at[3 * j + 1] = px[1] / s);
        at[3 * j + 2] = Math.sqrt(x * x + y * y + z * z);
      }
    const put = (x, y, d) => {
      if (!(x >= 0 && y >= 0 && x < ws && y < hs)) return;
      const b = (y | 0) * ws + (x | 0);
      if (d < zb[b]) zb[b] = d;
    };
    // z-buffered as a surface: each sample, and the bins between it and the next one along its row and column when they
    // are on one surface (no jump between them in the render), so a surface the move magnifies keeps no gaps
    for (let j = 0; j < ws * hs; j++) {
      const x = at[3 * j], y = at[3 * j + 1], d = at[3 * j + 2];
      if (!(d > 0) || !(x === x)) continue;
      put(x, y, d);
      const e = Ej[j];
      for (let side = 0; side < 2; side++) {
        const q = side ? j + ws : (j % ws) < ws - 1 ? j + 1 : -1;
        if (q < 0 || q >= ws * hs) continue;
        const x2 = at[3 * q], y2 = at[3 * q + 1], d2 = at[3 * q + 2], r = Ej[q] / e;
        if (!(d2 > 0) || !(x2 === x2) || !(r <= jr && r >= 1 / jr)) continue;
        const n = Math.min(16, Math.ceil(Math.max(Math.abs(x2 - x), Math.abs(y2 - y))));
        for (let i = 1; i < n; i++) put(x + ((x2 - x) * i) / n, y + ((y2 - y) * i) / n, d + ((d2 - d) * i) / n);
      }
    }
    // Nothing landed in a bin: a gap between samples (both neighbours across it landed: their farther); else, with samples
    // on both sides of it the way the move moves the picture (up and down: its column; sideways: its row), what the moved
    // camera would see behind what hid it in the render: anything as far, so no far cell there; else past the render's
    // edge: nothing known (a furniture top under the render's view: the 3D scan's check over the same moves still tells
    // one within them, scanSays(); one further off is perched()).
    const up = Math.abs(tc[1]) >= Math.abs(tc[0]), n = up ? ws : hs; // lines[l], lines[n + l]: the first and last bin landed on in line l
    lines.fill(32767, 0, n).fill(-1, n, 2 * n);
    for (let b = 0; b < zb.length; b++) {
      if (zb[b] === Infinity) continue;
      const l = up ? b % ws : (b / ws) | 0, pos = up ? (b / ws) | 0 : b % ws;
      if (pos < lines[l]) lines[l] = pos;
      if (pos > lines[n + l]) lines[n + l] = pos;
    }
    for (let b = 0; b < zb.length; b++) {
      const k = cell[b], bu = b % ws, bv = (b / ws) | 0;
      let d = zb[b];
      if (d === Infinity) {
        const l = bu > 0 ? zb[b - 1] : Infinity, r = bu < ws - 1 ? zb[b + 1] : Infinity, a = bv > 0 ? zb[b - ws] : Infinity, c = bv < hs - 1 ? zb[b + ws] : Infinity;
        d = Math.min(l < Infinity && r < Infinity ? Math.max(l, r) : Infinity, a < Infinity && c < Infinity ? Math.max(a, c) : Infinity);
      }
      const line = up ? bu : bv, pos = up ? bv : bu;
      if (d < Infinity) (L[k] = Math.min(L[k], d)), (H[k] = Math.max(H[k], d));
      else if (pos > lines[line] && pos < lines[n + line]) H[k] = Infinity;
    }
  }
  for (const k of cells) {
    const edge = out.edge[k], l = edge ? Math.min(lo[k], loW[k]) : lo[k], m = edge ? Math.max(hi[k], hiW[k]) : hi[k];
    out.near[k] = near[k] && !(lv[k] >= Math.log(l) - tol[k]) ? 1 : 0;
    out.far[k] = far[k] && !(lv[k] <= Math.log(m) + tol[k]) ? 1 : 0;
    out.posed += (near[k] | far[k]) - (out.near[k] | out.far[k]);
  }
  return out;
}

// The unit rays (camera frame) of every s-th pixel of a view and the grid cell each sample's bin is in (cached per view).
const sampleCache = new Map();
function samples(K, s, C, gw) {
  const key = `${K.model}|${K.width}x${K.height}|${K.fx}|${K.fy}|${K.cx}|${K.cy}|${s}|${C}`;
  if (!sampleCache.has(key)) {
    const ws = Math.ceil(K.width / s), hs = Math.ceil(K.height / s), rays = new Float32Array(3 * ws * hs), cell = new Int32Array(ws * hs);
    for (let v = 0, j = 0; v < K.height; v += s)
      for (let u = 0; u < K.width; u += s, j++) {
        rays.set(unproject(K, u + 0.5, v + 0.5) ?? [NaN, NaN, NaN], 3 * j);
        cell[j] = ((v / C) | 0) * gw + ((u / C) | 0);
      }
    if (sampleCache.size > 8) sampleCache.clear();
    sampleCache.set(key, { rays, cell });
  }
  return sampleCache.get(key);
}

const gray = (px, w, h) => {
  const y = new Float32Array(w * h), g = new Float32Array(w * h);
  for (let i = 0; i < w * h; i++) y[i] = 0.3 * px[4 * i] + 0.59 * px[4 * i + 1] + 0.11 * px[4 * i + 2];
  for (let v = 1; v < h - 1; v++) for (let u = 1; u < w - 1; u++) { const i = v * w + u; g[i] = Math.abs(y[i + 1] - y[i - 1]) + Math.abs(y[i + w] - y[i - w]); }
  return g;
};

// The shift { sx, sy } (px) of the expected picture that best lines its gradients up with the live one's (normalised
// correlation, coarse to fine over halved images, up to CHANGES.shift px), or null without both pictures, for no shift,
// or when nothing lines up.
function register(f) {
  const L = f.rgb, E = f.expectedRgb, { width: w, height: h } = f;
  if (!L || !E || E.width !== w || L.width !== w) return null;
  const pyr = (g) => {
    const out = [{ g, w, h }];
    for (let l = 1; l < 3; l++) {
      const p = out[l - 1], w2 = p.w >> 1, h2 = p.h >> 1, d = new Float32Array(w2 * h2);
      for (let v = 0; v < h2; v++) for (let u = 0; u < w2; u++) d[v * w2 + u] = (p.g[2 * v * p.w + 2 * u] + p.g[2 * v * p.w + 2 * u + 1] + p.g[(2 * v + 1) * p.w + 2 * u] + p.g[(2 * v + 1) * p.w + 2 * u + 1]) / 4;
      out.push({ g: d, w: w2, h: h2 });
    }
    return out;
  };
  const A = pyr(gray(L.data, w, h)), B = pyr(gray(E.data, w, h));
  const ncc = ({ g: a, w: W, h: H }, b, sx, sy) => {
    let n = 0, sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    const m = 2;
    for (let v = Math.max(m, sy + m); v < Math.min(H - m, H + sy - m); v++)
      for (let u = Math.max(m, sx + m); u < Math.min(W - m, W + sx - m); u++) {
        const x = a[v * W + u], y = b.g[(v - sy) * W + (u - sx)];
        n++, (sa += x), (sb += y), (saa += x * x), (sbb += y * y), (sab += x * y);
      }
    const va = saa / n - (sa / n) ** 2, vb = sbb / n - (sb / n) ** 2;
    return n > 50 && va > 0 && vb > 0 ? (sab / n - (sa / n) * (sb / n)) / Math.sqrt(va * vb) : -1;
  };
  let sx = 0, sy = 0, best = -1;
  for (const [l, r] of [[2, Math.ceil(CHANGES.shift / 4)], [1, 1], [0, 1]]) {
    const cx = l === 2 ? 0 : sx * 2, cy = l === 2 ? 0 : sy * 2;
    best = -1;
    for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
      const c = ncc(A[l], B[l], cx + dx, cy + dy);
      if (c > best) [best, sx, sy] = [c, cx + dx, cy + dy];
    }
  }
  return best > 0.3 && (sx || sy) ? { sx, sy, ncc: best } : null;
}

// A running box filter of radius r along one axis (step 1: rows, step w: columns) of a w x h image.
function blur(a, w, h, step, r) {
  const out = new Float32Array(a.length), n = step === 1 ? w : h, lines = step === 1 ? h : w;
  for (let l = 0; l < lines; l++) {
    const base = step === 1 ? l * w : l;
    let s = 0;
    for (let i = 0; i < Math.min(n, r + 1); i++) s += a[base + i * step];
    for (let i = 0; i < n; i++) {
      out[base + i * step] = s / (Math.min(n - 1, i + r) - Math.max(0, i - r) + 1);
      if (i + r + 1 < n) s += a[base + (i + r + 1) * step];
      if (i - r >= 0) s -= a[base + (i - r) * step];
    }
  }
  return out;
}

// An image (k values per pixel) moved by { sx, sy }: out(u, v) = in(u - sx, v - sy), `fill` where nothing comes from.
function moved(src, w, h, { sx, sy }, k, fill) {
  const out = new src.constructor(src.length).fill(fill);
  for (let v = Math.max(0, sy); v < Math.min(h, h + sy); v++) out.set(src.subarray(((v - sy) * w + Math.max(0, -sx)) * k, ((v - sy) * w + Math.min(w, w - sx)) * k), (v * w + Math.max(0, sx)) * k);
  return out;
}

// A quadratic surface over the image fitted to the cells' log ratios (weights: pixels; Huber, 3 rounds). -> k => value
function smoothFit(cells, r, n, gw, gh) {
  const feat = (k) => {
    const u = (2 * ((k % gw) + 0.5)) / gw - 1, v = (2 * (((k / gw) | 0) + 0.5)) / gh - 1;
    return [1, u, v, u * u, u * v, v * v];
  };
  let c = [0, 0, 0, 0, 0, 0];
  const at = (k) => feat(k).reduce((s, x, i) => s + x * c[i], 0);
  if (cells.length < 12) return () => 0;
  for (let round = 0; round < 3; round++) {
    const res = cells.map((k) => Math.abs(r[k] - at(k))).sort((a, b) => a - b), scale = Math.max(0.02, 1.4826 * res[res.length >> 1]);
    const A = Array.from({ length: 6 }, () => new Array(6).fill(0)), b = new Array(6).fill(0);
    for (const k of cells) {
      const x = feat(k), e = Math.abs(r[k] - at(k)), w = n[k] * (round && e > 1.5 * scale ? (1.5 * scale) / e : 1);
      for (let i = 0; i < 6; i++) {
        b[i] += w * x[i] * r[k];
        for (let j = 0; j < 6; j++) A[i][j] += w * x[i] * x[j];
      }
    }
    c = solve(A, b) ?? c;
  }
  return at;
}

function solve(A, b) {
  const n = b.length, M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let p = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(M[r][col]) > Math.abs(M[p][col])) p = r;
    if (Math.abs(M[p][col]) < 1e-12) return null;
    [M[col], M[p]] = [M[p], M[col]];
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col] / M[col][col];
      for (let j = col; j <= n; j++) M[r][j] -= f * M[col][j];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}

// Per cell: does the live picture look different from the expected one (f.rgb vs f.expectedRgb, the same view)? It has to
// hold up against the O4's video (grain, compression, exposure and white balance that hunt, motion blur) and against a
// few px of misregistration (pose, lens), and against light unlike the capture's (a lamp, a shadow, the evening): on
// both pictures at half size, the cell's mean colour against the expected cell's and its 8 neighbours' after a gain per
// channel (the median over the cells around it, 1-2 blocks of CHANGES.light cells each way: lighting is smooth, things
// have edges),
// and the structure of the gradients
// (their correlation over the cell and its 8 neighbours) where both are textured above their own picture's noise floor,
// or texture in one where the other is flat. null without the pictures.
function appearance(f, C, gw, gh, mask) {
  const L = f.rgb, E = f.expectedRgb;
  if (!L || !E || E.width !== f.width || L.width !== f.width) return null;
  const { width: w, height: h } = f, N = gw * gh, cl = new Float64Array(3 * N), ce = new Float64Array(3 * N), cn = new Float64Array(N);
  for (let v = 0; v < h; v++)
    for (let u = 0; u < w; u++) {
      const i = v * w + u, k = ((v / C) | 0) * gw + ((u / C) | 0);
      if (mask[i]) continue;
      cn[k]++;
      for (let c = 0; c < 3; c++) (cl[3 * k + c] += L.data[4 * i + c]), (ce[3 * k + c] += E.data[4 * i + c]);
    }
  const cells = [];
  for (let k = 0; k < N; k++) if (cn[k] >= 4) (cells.push(k), [0, 1, 2].forEach((c) => ((cl[3 * k + c] /= cn[k]), (ce[3 * k + c] /= cn[k]))));
  const med = (a) => a.sort((x, y) => x - y)[a.length >> 1] ?? 1, R = CHANGES.light;
  const gain0 = [0, 1, 2].map((c) => med(cells.filter((k) => ce[3 * k + c] > 20).map((k) => cl[3 * k + c] / ce[3 * k + c])));
  // the light's gain per channel around each cell: per block of R x R cells, the median over it and the 8 around it
  const bw = Math.ceil(gw / R), bh = Math.ceil(gh / R), blk = Array.from({ length: 3 * bw * bh }, () => []), lit = new Float32Array(3 * bw * bh), bOf = (k) => ((((k / gw) | 0) / R) | 0) * bw + (((k % gw) / R) | 0);
  for (const k of cells) for (let q = 0; q < 3; q++) if (ce[3 * k + q] > 20) blk[3 * bOf(k) + q].push(cl[3 * k + q] / ce[3 * k + q]);
  for (let b = 0; b < bw * bh; b++)
    for (let q = 0; q < 3; q++) {
      const rs = [];
      for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
        const c = (b % bw) + dc, r = ((b / bw) | 0) + dr;
        if (c >= 0 && r >= 0 && c < bw && r < bh) rs.push(...blk[3 * (r * bw + c) + q]);
      }
      lit[3 * b + q] = rs.length >= 8 ? med(rs) : gain0[q];
    }
  // gradients on half-size greys (2x2 means: half the grain), summed per cell
  const w2 = w >> 1, h2 = h >> 1, half = (px) => {
    const y = new Float32Array(w2 * h2), g = new Float32Array(w2 * h2);
    for (let v = 0; v < h2; v++)
      for (let u = 0; u < w2; u++) {
        let s = 0;
        for (const [a, b] of [[0, 0], [1, 0], [0, 1], [1, 1]]) {
          const i = 4 * ((2 * v + b) * w + 2 * u + a);
          s += 0.3 * px[i] + 0.59 * px[i + 1] + 0.11 * px[i + 2];
        }
        y[v * w2 + u] = s / 4;
      }
    for (let v = 1; v < h2 - 1; v++) for (let u = 1; u < w2 - 1; u++) { const i = v * w2 + u; g[i] = Math.abs(y[i + 1] - y[i - 1]) + Math.abs(y[i + w2] - y[i - w2]); }
    return blur(blur(g, w2, h2, 1, CHANGES.spread), w2, h2, w2, CHANGES.spread); // edges a few px apart still line up
  };
  const gl = half(L.data), ge = half(E.data), S = Array.from({ length: 6 }, () => new Float64Array(N));
  for (let v = 1; v < h2 - 1; v++)
    for (let u = 1; u < w2 - 1; u++) {
      if (mask[2 * v * w + 2 * u]) continue;
      const i = v * w2 + u, k = (((2 * v) / C) | 0) * gw + (((2 * u) / C) | 0), a = gl[i], b = ge[i];
      S[0][k]++, (S[1][k] += a), (S[2][k] += b), (S[3][k] += a * a), (S[4][k] += b * b), (S[5][k] += a * b);
    }
  const pct = (a, q) => a.sort((x, y) => x - y)[Math.floor(q * (a.length - 1))] ?? 0, en = cells.filter((k) => S[0][k] > 0);
  const floorL = pct(en.map((k) => S[3][k] / S[0][k]), 0.2), floorE = pct(en.map((k) => S[4][k] / S[0][k]), 0.2);
  const out = new Uint8Array(N);
  for (const k of cells) {
    const t = [0, 0, 0, 0, 0, 0], c0 = k % gw, r0 = (k / gw) | 0;
    let dc = Infinity;
    for (let dr = -1; dr <= 1; dr++)
      for (let dq = -1; dq <= 1; dq++) {
        const c = c0 + dq, r = r0 + dr, j = r * gw + c;
        if (c < 0 || r < 0 || c >= gw || r >= gh) continue;
        for (let q = 0; q < 6; q++) t[q] += S[q][j];
        if (cn[j] >= 4) dc = Math.min(dc, [0, 1, 2].reduce((s, q) => s + Math.abs(cl[3 * k + q] - lit[3 * bOf(k) + q] * ce[3 * j + q]), 0) / 3);
      }
    if (t[0] < 4) continue;
    const ea = t[3] / t[0], eb = t[4] / t[0], ra = ea / (floorL + 10), rb = eb / (floorE + 10), [lo, hi] = CHANGES.texture;
    const va = ea - (t[1] / t[0]) ** 2, vb = eb - (t[2] / t[0]) ** 2, ncc = va > 1 && vb > 1 ? (t[5] / t[0] - (t[1] / t[0]) * (t[2] / t[0])) / Math.sqrt(va * vb) : 1;
    out[k] = dc > CHANGES.colour || (ra > hi && rb > hi && ncc < CHANGES.ncc) || (Math.max(ra, rb) > hi && Math.min(ra, rb) < lo) ? 1 : 0;
  }
  return out;
}

// Connected differing cells (4-neighbours) at least 2 cells across both ways, each as a sighting from the camera c: the
// unit ray r through its pixels, the range d along it (the live depth's median when nearer, the expected one's when
// farther: what is missing) and its error sd (f.depthErr: the share a new thing's live depth can be off), the errors
// across it (ss sideways along s, su up and down along u: a blob cut by the frame's edge says little that way), its
// width across the ray (cut: at the sides) and a first place c + r d. door: the doorway whose plane its rays cross at
// its range (t), and, when the picture holds enough of that doorway's opening, cover: the share of it (where the scan saw
// through it, for a nearer blob; where it saw its leaf, for a farther one) that is now the other way, and where along it
// (s, 0-1 from a to b), for the candidate to add up. scan: what the 3D scan says (scanSays(), with map.vox; else null),
// from the camera moved as heldUp() moved it (grid.moves; most of its cells by an edge, grid.edge: also grid.wide). posed:
// the share of the differing cells it is part of (as differences() found them: grid.near0, far0) the pose's error
// explains; weak: at least half, or (near) most of its points perched() on a scanned surface, or it touches a faint layer.
// -> [{ near, c, r, s, u, d, sd, ss, su, cut, x, y, z, size, zMin, zMax, room, door, scan, posed, weak, cells, px: [u0, v0, u1, v1] }]
export function blobsOf(grid, f, map, house) {
  const { gw, gh, C } = grid, K = intrinsics(f.lens, f.width, f.height), cam = droneCamera(f.pose, f.lens.uptiltDeg ?? 0), R = cam.R, out = [];
  const mid = (a) => [...a].sort((x, y) => x - y)[a.length >> 1], q = (a, s) => [...a].sort((x, y) => x - y)[Math.min(a.length - 1, Math.floor(s * a.length))];
  for (const near of [true, false]) {
    const on = near ? grid.near : grid.far, was = (near ? grid.near0 : grid.far0) ?? on, seen = new Uint8Array(gw * gh);
    for (let k0 = 0; k0 < gw * gh; k0++) {
      if (!on[k0] || seen[k0]) continue;
      const cells = [k0];
      seen[k0] = 1;
      for (let i = 0; i < cells.length; i++)
        for (const d of [1, -1, gw, -gw]) {
          const k = cells[i] + d;
          if (k >= 0 && k < gw * gh && on[k] && !seen[k] && Math.abs((k % gw) - (cells[i] % gw)) <= 1) (seen[k] = 1), cells.push(k);
        }
      const cs = cells.map((k) => k % gw), rs = cells.map((k) => (k / gw) | 0);
      if (cells.length < CHANGES.minCells || Math.max(...cs) === Math.min(...cs) || Math.max(...rs) === Math.min(...rs)) continue;
      const rays = [], ds = [], lives = [], exps = [];
      for (const k of cells)
        for (let v = ((k / gw) | 0) * C; v < Math.min(f.height, ((k / gw) | 0) * C + C); v += 2)
          for (let u = (k % gw) * C; u < Math.min(f.width, (k % gw) * C + C); u += 2) {
            const i = v * f.width + u, a = f.depth[i], e = f.expected[i], tol = Math.max(CHANGES.abs, CHANGES.rel * e);
            if (!(a > 0) || !(e > 0) || (near ? !(a < e - tol) : !(a > e + tol))) continue;
            const p = unproject(K, u + 0.5, v + 0.5), L = Math.hypot(...p);
            rays.push([0, 1, 2].map((j) => (R[j][0] * p[0] + R[j][1] * p[1] + R[j][2] * p[2]) / L));
            ds.push(near ? a : e);
            lives.push(a), exps.push(e);
          }
      if (rays.length < 6) continue;
      const m = [0, 1, 2].map((j) => rays.reduce((s, r) => s + r[j], 0)), Lm = Math.hypot(...m), r = m.map((v) => v / Lm), c = cam.p, d = mid(ds);
      const h = Math.hypot(r[0], r[1]), side = [-r[1] / h, r[0] / h, 0], up = [side[1] * r[2], -side[0] * r[2], side[0] * r[1] - side[1] * r[0]];
      const sd = near ? Math.max(0.05, (f.depthErr ?? CHANGES.depthErr) * d) : 0.05 + 0.05 * d, sl = 0.03 + CHANGES.bearing * d + (f.pose.sigma ?? 0);
      const lat = rays.map((ray, i) => ds[i] * (ray[0] * side[0] + ray[1] * side[1])), zs = rays.map((ray, i) => c[2] + ds[i] * ray[2]);
      const edge = (a, hi) => Math.min(...a) <= 0 || Math.max(...a) >= hi, cut = edge(cs, gw - 1), cutV = edge(rs, gh - 1);
      let door = null;
      for (const dr of house?.doors ?? []) {
        const hit = (ray) => onDoor(dr, c, ray), at = rays.map(hit).filter(Boolean), t = at.length >= 0.8 * rays.length ? mid(at.map((p) => p.t)) : NaN;
        // On the doorway's plane: in front of it, within the aligned depth's own error there (the jambs beside it are in the
        // scan; a new thing's depth error that way would take anything standing in front of a doorway for a door); behind
        // it, as far as a new thing's depth can read too far (a closed door is new to the scan).
        const tol = Math.max(CHANGES.door, CHANGES.doorRel * t), err = near ? f.depthErr ?? CHANGES.depthErr : 0;
        if (!(d > t - tol && d < t * (1 + err) + tol) || (door && Math.abs(t - d) >= Math.abs(door.t - d))) continue;
        door = { dr, t, err };
      }
      if (door) { // the doorway nearest its depth: how much of its opening is the other way in this picture
        const cv = doorCover(f, K, cam, door.dr, near, door.err);
        door = { id: door.dr.id, t: door.t, cover: cv.n >= 12 && { share: cv.share, s: cv.s } };
      }
      const edgy = !!grid.edge && 2 * cells.filter((k) => grid.edge[k]).length >= cells.length, moves = [...(grid.moves ?? [[0, 0, 0]]), ...(edgy ? grid.wide ?? [] : [])];
      const x = c[0] + r[0] * d, y = c[1] + r[1] * d, z = c[2] + r[2] * d, scan = map.vox ? scanSays(map.vox, c, rays, lives, exps, near, moves) : null, posed = 1 - cells.length / spread(was, cells, gw, gh);
      out.push({ near, c, r, s: side, u: up, d, sd, ss: cut ? 10 : sl, su: cutV ? 10 : sl, cut, cutV, x, y, z, size: q(lat, 0.95) - q(lat, 0.05), zMin: q(zs, 0.05), zMax: q(zs, 0.95),
        room: map.roomAt(x, y)?.id ?? null, door, scan, posed, weak: posed >= 0.5 || (near && !!map.vox && perched(map, c, rays, lives, grid.reach ?? 2 * CHANGES.zSigma + CHANGES.weakZ) >= 0.5) || scan?.faint > 0, cells: cells.length, px: [Math.min(...cs) * C, Math.min(...rs) * C, (Math.max(...cs) + 1) * C, (Math.max(...rs) + 1) * C] });
    }
  }
  return out;
}

// How many cells of mask m are connected (4-neighbours) to `cells` (in it).
function spread(m, cells, gw, gh) {
  const seen = new Set(cells), todo = [...cells];
  while (todo.length) {
    const k = todo.pop(), c = k % gw;
    for (const j of [c > 0 ? k - 1 : -1, c < gw - 1 ? k + 1 : -1, k - gw, k + gw]) if (j >= 0 && j < gw * gh && m[j] && !seen.has(j)) seen.add(j), todo.push(j);
  }
  return seen.size;
}

// The share of a new thing's points (up to CHANGES.scanRays of its rays from c, at their live depth) within `reach` m over
// or under a surface the 3D scan has, off the floor (0.15 m over it): where a height error larger than swept would put a
// furniture top's or a shelf's edge (what is left of a phantom along it), where monocular depth misreads a plain ceiling
// (a patch of it read nearer: a rehearsal's "new thing" at 2 m), and a real thing on a table too.
function perched(map, c, rays, lives, reach) {
  const vox = map.vox, step = Math.max(1, Math.floor(rays.length / CHANGES.scanRays)), res = vox.res;
  let n = 0, near = 0;
  for (let k = 0; k < rays.length; k += step) {
    const r = rays[k], x = c[0] + r[0] * lives[k], y = c[1] + r[1] * lives[k], z = c[2] + r[2] * lives[k], floor = map.floorAt(x, y) ?? 0;
    n++;
    for (let t = res, hit = false; t <= reach && !hit; t += res)
      for (const h of [z - t, z + t]) if (!hit && h > floor + 0.15 && (scanAt(vox, x, y, h) & 3) === OCCUPIED) (hit = true), near++;
  }
  return n ? near / n : 0;
}

// What the 3D scan says about a blob, over up to CHANGES.scanRays of its rays (from c, unit, with the live and expected
// depth along each): something new (near) contradicts the scan where its live point stands in space the scan saw free,
// seen from c moved by each of `moves` (the pose's error: heldUp()); something gone (far) where the scan had a surface at
// the end of the twin's expected depth (within a voxel or two). A faint layer at any of those live points (near) or at the
// expected surface (far: glass, an aquarium, a mirror) makes the ray unreliable. -> { ok, against, faint, n } (shares of n)
export function scanSays(vox, c, rays, lives, exps, near, moves = [[0, 0, 0]]) {
  const step = Math.max(1, Math.floor(rays.length / CHANGES.scanRays)), res = vox.res;
  let n = 0, against = 0, faint = 0;
  for (let k = 0; k < rays.length; k += step) {
    const r = rays[k], at = (t, m = [0, 0, 0]) => scanAt(vox, c[0] + m[0] + r[0] * t, c[1] + m[1] + r[1] * t, c[2] + m[2] + r[2] * t);
    const pts = near ? moves.map((m) => at(lives[k], m)) : [-res, 0, res, 2 * res].map((dt) => at(exps[k] + dt));
    n++;
    if (pts.some((s) => s & SCAN_FAINT)) faint++;
    else if (near ? pts.every((s) => (s & 3) === FREE) : pts.some((s) => (s & 3) === OCCUPIED)) against++;
  }
  return { ok: n > 0 && against >= CHANGES.scanShare * n && faint <= CHANGES.faintShare * n, against: n ? against / n : 0, faint: n ? faint / n : 0, n };
}

// A voxel as the scan has it (UNKNOWN | FREE | OCCUPIED, plus SCAN_FAINT): what flights alone found left out (durableAt),
// so a new thing the flight's live depth has already filled in is still where the scan saw free space.
const SCAN_FAINT = 4;
function scanAt(vox, x, y, z) {
  const i = vox.idx(x, y, z);
  if (i < 0) return UNKNOWN;
  const [lo, fl] = vox.durableAt ? vox.durableAt(i) : [vox.lo[i], vox.flags[i]];
  return (lo === vox.lo[i] ? vox.st[i] : fl & CARVED ? FREE : UNKNOWN) | (fl & FAINT ? SCAN_FAINT : 0);
}

// The app's trust input for a ChangeDetector (and for whatever else video may change: ui/session.js): the real drone's
// frames only with the camera calibrated and checked for this framing (calib.verified), vision positioning on and past
// the pad check (splat.trust() "verified"), a vision fix applied within CHANGES.fixAge ms and a sure pose; the simulator
// flying on its own video (settings simLoc "vision": "Rehearse the real flight") the same but the calibration (it has no
// calib.json) and the pad check (its video is the twin's); the simulator on its true pose, always. calib: calib.json or a
// function returning it. zSigma: the frame's height σ, else the localizer's now (frames timed by the localizer's trail
// carry none). -> (frame?) => { ok, why, zSigma } (no frame: the localizer's pose now)
export function changeTrust({ settings, splat, localizer, calib }) {
  return (f) => {
    const real = settings.get("mode") === "real", zSigma = f?.pose?.zSigma ?? localizer?.pose?.()?.zSigma;
    if (!real && settings.get("simLoc") !== "vision") return { ok: true, why: "", zSigma };
    const cb = typeof calib === "function" ? calib() : calib, q = localizer?.fixQuality?.(), p = f?.pose ?? localizer?.pose?.();
    const why = real && !cb?.verified ? "the camera isn't calibrated and checked yet" : !splat?.enabled ? "vision positioning is off"
      : real && splat.trust?.() !== "verified" ? "the pad check hasn't passed yet" : !(q?.age < CHANGES.fixAge) ? "no fresh camera position fix"
      : !(p?.sigma <= CHANGES.maxSigma) || (p.status && p.status !== "ok") ? "the position isn't sure enough" : "";
    return { ok: !why, why, zSigma };
  };
}

// A reported change and its later reports (the same thing seen again from elsewhere).
const spotsOf = (r) => [r, ...(r.spots ?? [])];
// The heights a reported spot is looked at: its fitted height alone when its reports span 0.3 m or less, else evenly from
// 0.1 m over its foot (a new thing's is the floor) to its top, at most 6.
const heightsOf = (p) => {
  const lo = p.zMin ?? p.z, hi = p.zMax ?? p.z;
  if (!(hi - lo > 0.3)) return [p.z];
  const n = Math.min(6, Math.ceil((hi - lo) / 0.25) + 1);
  return Array.from({ length: n }, (_, i) => lo + 0.1 + ((hi - lo - 0.1) * i) / (n - 1));
};

// The doorway's opening (sill to head, 12 x 10 samples) in a frame's picture, where the scan saw through it (near:
// something new may close it) or saw its leaf (far: it may have opened), people and pets (f.boxes) left out: n samples, and
// the share of them now the other way (near: live depth on the plane, not in front of it, no farther behind it than a new
// thing's depth error err reaches, and nearer than the scan through it; far: beyond the leaf), and where along it (s, 0-1
// from a to b) those are. -> { n, share, s: [s0, s1] }
export function doorCover(f, K, cam, dr, near, err) {
  const R = cam.R, c = cam.p, ex = dr.b[0] - dr.a[0], ey = dr.b[1] - dr.a[1], [z0, z1] = [dr.sillZ ?? 0, dr.headZ ?? 2], ss = [];
  let n = 0;
  for (let i = 0; i < 12; i++)
    for (let j = 0; j < 10; j++) {
      const sv = (i + 0.5) / 12, P = [dr.a[0] + sv * ex - c[0], dr.a[1] + sv * ey - c[1], z0 + ((j + 0.5) / 10) * (z1 - z0) - c[2]], cc = [0, 1, 2].map((a) => R[0][a] * P[0] + R[1][a] * P[1] + R[2][a] * P[2]);
      const px = cc[2] > 0.1 && project(K, cc);
      if (!px || !(px[0] >= 0 && px[1] >= 0 && px[0] < f.width && px[1] < f.height) || f.boxes?.some((b) => px[0] / f.width > b.x && px[0] / f.width < b.x + b.w && px[1] / f.height > b.y && px[1] / f.height < b.y + b.h)) continue;
      const k = (px[1] | 0) * f.width + (px[0] | 0), L = Math.hypot(...P), e = f.expected[k], a = f.depth[k], tol = Math.max(CHANGES.door, CHANGES.doorRel * L);
      if (!(a > 0) || !(near ? e > L + tol : Math.abs(e - L) < tol)) continue;
      n++;
      if (near ? a > L - tol && a < Math.min(e - tol, L * (1 + err) + tol) : a > L + tol) ss.push(sv);
    }
  return { n, share: n ? ss.length / n : 0, s: ss.length ? [Math.min(...ss), Math.max(...ss)] : [0, 0] };
}

// Where a ray from c meets a doorway's plane: { t (along the ray), s (0-1 along a-b, a little beyond counts), z } | null.
function onDoor(dr, c, ray) {
  const ex = dr.b[0] - dr.a[0], ey = dr.b[1] - dr.a[1], den = ray[0] * ey - ray[1] * ex;
  if (Math.abs(den) < 1e-6) return null;
  const t = ((dr.a[0] - c[0]) * ey - (dr.a[1] - c[1]) * ex) / den, s = ((dr.a[0] - c[0]) * ray[1] - (dr.a[1] - c[1]) * ray[0]) / den;
  return t > 0 && s > -0.05 && s < 1.05 ? { t, s, z: c[2] + t * ray[2] } : null;
}

// The nearer live depth (25th percentile) in the middle half of a box: the person or pet, not the background around them.
function boxDepth(f, b) {
  const ds = [];
  for (let v = Math.max(0, Math.floor(b.y * f.height)); v < Math.min(f.height, Math.ceil((b.y + b.h) * f.height)); v += 2)
    for (let u = Math.max(0, Math.floor((b.x + 0.25 * b.w) * f.width)); u < Math.min(f.width, Math.ceil((b.x + 0.75 * b.w) * f.width)); u += 2) if (f.depth[v * f.width + u] > 0) ds.push(f.depth[v * f.width + u]);
  return ds.length >= 4 ? ds.sort((p, q) => p - q)[ds.length >> 2] : NaN;
}

// The inverse of a 3x3 matrix, or null.
function inv3(m) {
  const [[a, b, c], [d, e, f], [g, h, i]] = m, A = e * i - f * h, B = f * g - d * i, Cc = d * h - e * g, det = a * A + b * B + c * Cc;
  if (Math.abs(det) < 1e-12) return null;
  return [[A / det, (c * h - b * i) / det, (b * f - c * e) / det], [B / det, (a * i - c * g) / det, (c * d - a * f) / det], [Cc / det, (b * g - a * h) / det, (a * e - b * d) / det]];
}
