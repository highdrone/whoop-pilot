// Claude surveys the house from its 3D scan (docs/HOME-DRONE.md, "Wave C contracts"): per room 4-8 level pinhole views
// (90°, 768x576, no actors or props) rendered by the twin from the planner's viewpoints, chosen to see the whole room,
// plus up to SURVEY.maxExtra views looking down at the floor or up at the ceiling, which a level camera 1.3 m up misses
// within about 1.7 m (its vertical view is 74°): planned so the floor and ceiling are covered too (projected through the
// lens, not just in plan). What the views really saw is measured from their depth (floor and ceiling hits); while a
// room's measured share is short of floorCoverage (ceilingCoverage), up to SURVEY.maxMore views pitched at what is still
// unseen are added from other spots in the room (furniture hides floor the plan can't tell), each kept only if its depth
// shows at least moreGain more. One Claude vision call per room (the views, the room's plan, a JSON-schema answer) for
// what kind of room it is, the landmarks a person would name and the hazards for a 65 mm whoop, each boxed in one view;
// each box lifted to 3D with the twin's expected depth at that view; duplicates across views and rooms merged. What no
// view saw is said per room (rooms[].seen, rooms[].unseen, and in the summary). Nothing is applied: the UI shows the
// result and applySurvey() writes what the user accepts. (vox, in the contract's signature, isn't needed: the twin's
// depth is the scan's own surface.)
//   const est = estimateSurvey({ house, map, settings });              // before asking the user: tokens and dollars
//   const res = await surveyHouse({ house, map, twin, settings, claude, approved: true, estimate: est, onProgress });
//   applySurvey(house, res, { rooms: ["r2"], landmarks: [0, 3], hazards: [1] }); map.finalize(); save(house);
import { viewpoints, roomCenter } from "../house/planner.js";
import { pinholeLens, intrinsics, unproject, project } from "../twin/lens.js";
import { droneCamera } from "../twin/pose.js";
import { text, image, imageTokens, textTokens, toJpegBase64 } from "./claude.js";
import { costOf } from "../agent.js";

export const SURVEY = {
  width: 768, height: 576, hfov: 90, alt: 1.3, // camera: about chest height
  minViews: 4, maxViews: 8, spots: 3, range: 7, coverage: 0.97, // level views
  maxExtra: 6, down: 40, up: 50, // degrees: views pitched down at the floor and up at the ceiling, when the level ones miss them
  floorCoverage: 0.9, ceilingCoverage: 0.6, minGain: 0.04, // shares wanted, and the least share of the floor a planned pitched view adds
  maxMore: 4, moreTries: 10, moreGain: 0.03, spotStep: 0.6, spotClear: 0.3, // views from other spots (m apart, m from anything)
  // while the measured shares are short: at most, depth renders tried, the least share a kept one adds
  depthScale: 0.5, // the depth render is half the picture's size
  near: 0.6, // m: the same name (or hazard kind) closer than this (or their radii) is one thing
  outTokens: 1500, roomTokens: 450, // the estimate's output (thinking included) and per-room text
  minOpen: 1.2, // m: a level view whose median depth is nearer (a wall or a door in its face) is swapped for the next best
  seenCell: 0.2, // m: the grid floor and ceiling coverage is measured on
  unseenMin: 0.2, // m²: smaller unseen patches aren't mentioned
};
export const HAZARDS = ["ceiling-fan", "hanging-lamp", "plant", "curtain", "cable", "glass", "mirror", "pet-bowl", "stairs", "candle", "open-flame", "shelf-edge", "other"];
export const ROOM_KINDS = ["living room", "kitchen", "dining room", "bedroom", "bathroom", "hallway", "entrance", "office", "laundry", "stairwell", "garage", "closet", "kids room", "other"];
const SEE_THROUGH = new Set(["glass", "mirror"]); // the scan shows what is behind (or reflected in) them: placed on their wall
const DEG = Math.PI / 180;
const r2 = (v) => Math.round(v * 100) / 100;
const theRoom = (name) => (/^(room|bedroom|bathroom)\s*\d/i.test(name) ? name : `the ${name}`);
const SAME = { couch: "sofa", settee: "sofa", television: "tv", telly: "tv", fridge: "refrigerator", "dining table": "table" };
const nameKey = (n) => { const k = String(n).toLowerCase().replace(/^(the|a|an) /, "").replace(/[^a-z0-9 ]/g, "").trim(); return SAME[k] ?? k; };

export const SURVEY_PROMPT = `You are mapping a home for a tiny indoor drone (a 65 mm, 36 g whoop with ducted propellers) that will fly through it on its own. The pictures are renders of a 3D scan of the home (a Gaussian splat), taken from inside one room about 1.3 m above the floor with a 90° wide pinhole lens, ${SURVEY.width}x${SURVEY.height} pixels. Most views are level; some look down at the floor or up at the ceiling (each view's line says which). They look like photos but can be soft, smeared or have holes where the scan was thin; thin things (cables, cords, fan blades, glass) may be faint. A mirror shows a reflected room and glass shows what is behind it: box the mirror or the glass itself.

For the room you are shown, report:
1. room: what kind of room it is (kind) and a short name a person would use for it (name: "Kitchen", "Main bedroom", "Hallway"). If you can't tell, use kind "other" and a plain name. confidence 0..1.
2. landmarks: things a person would name when sending the drone somewhere: "sofa", "kitchen island", "cat tree", "front door", "tv", "dining table", "bed", "desk", "fireplace", "bookshelf", "window". Large, fixed, distinctive things, not small clutter. Short lowercase names without "the".
3. hazards for the drone: ceiling-fan; hanging-lamp (pendants, chandeliers); plant (leaves catch propellers); curtain (sheer fabric, blind cords); cable (loose cables and cords at any height); glass (glass doors, partitions and tables: the drone's camera sees through them); mirror; pet-bowl (water); stairs; candle or open-flame (a lit fireplace, candles); shelf-edge (thin shelves at flying height); other (anything else a small drone could hit or get tangled in: say what in why).

For every landmark and hazard give the view number where it is clearest (largest and least hidden) and its box in that view's pixels, [x0, y0, x1, y1], with (0, 0) the top-left corner and (${SURVEY.width}, ${SURVEY.height}) the bottom-right; box just the object, tightly. Report each object once even when several views show it. Things in another room seen through a doorway are fine (they are placed where they really are). confidence 0..1: how sure you are that it is there and is what you say. why: a few words. Leave out what you can't make out rather than guess.`;

const BOX = { type: "array", items: { type: "number" } };
export const SURVEY_SCHEMA = {
  type: "object", additionalProperties: false, required: ["room", "landmarks", "hazards"],
  properties: {
    room: { type: "object", additionalProperties: false, required: ["kind", "name", "confidence"],
      properties: { kind: { type: "string", enum: ROOM_KINDS }, name: { type: "string" }, confidence: { type: "number" } } },
    landmarks: { type: "array", items: { type: "object", additionalProperties: false, required: ["name", "view", "box", "confidence"],
      properties: { name: { type: "string" }, view: { type: "integer" }, box: BOX, confidence: { type: "number" } } } },
    hazards: { type: "array", items: { type: "object", additionalProperties: false, required: ["kind", "view", "box", "confidence", "why"],
      properties: { kind: { type: "string", enum: HAZARDS }, view: { type: "integer" }, box: BOX, confidence: { type: "number" }, why: { type: "string" } } } },
  },
};

// ---- views ----

const polyArea = (p) => Math.abs(p.reduce((a, [x, y], i) => a + x * p[(i + 1) % p.length][1] - p[(i + 1) % p.length][0] * y, 0) / 2);
const angleTo = (a, b) => Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
const toCam = (cam, P) => { const r = [0, 1, 2].map((k) => P[k] - cam.p[k]); return [0, 1, 2].map((k) => cam.R[0][k] * r[0] + cam.R[1][k] * r[1] + cam.R[2][k] * r[2]); };
const camToH = (cam, v) => [0, 1, 2].map((i) => cam.p[i] + cam.R[i][0] * v[0] + cam.R[i][1] * v[1] + cam.R[i][2] * v[2]);

// Poses { x, y, z, yaw, pitch (+ nose down), roll: 0, room, look: "level" | "down" | "up", planned: { level, floor,
// ceiling } (shares in plan: what the depth shows is measured in surveyHouse) } that together see the room: from up to `spots` of the planner's viewpoints (or the room's
// most open spot), 8 level headings each, greedy cover of the room's cells in plan (walls and tall furniture hide), at
// least minViews and at most maxViews; then views pitched down (4 headings) and up, while they add at least minGain of
// the floor (or ceiling) that no view shows yet when projected through the lens (floor under furniture doesn't count),
// up to maxExtra.
export function surveyViews(map, roomId, o = {}) {
  const { alt, hfov, minViews, maxViews, spots: maxSpots, range, coverage, maxExtra, down, up, floorCoverage, ceilingCoverage, minGain, width: W, height: H } = { ...SURVEY, ...o };
  const ri = map.rooms.findIndex((r) => r.id === roomId);
  if (ri < 0) return [];
  const vps = viewpoints(map, roomId, "person", { alt: 1.0 }).points.map((p) => [p.x, p.y]);
  const spots = (vps.length ? vps : [roomCenter(map, roomId, { alt: 1.0 })].filter(Boolean)).slice(0, maxSpots);
  if (!spots.length) return [];
  const tall = map.occ?.[map.bandOf(1.4)], low = map.occ?.[map.levels.length - 1], { cell } = map.o, sees = (p, q) => seesIn(map, p, q, range);
  let cells = 0;
  for (let k = 0; k < map.N; k++) cells += map.room[k] === ri ? 1 : 0;
  const step = Math.max(4, Math.round(Math.sqrt((cells * cell * cell) / 500) / cell)), targets = [], floors = [], ceils = [];
  for (let r = 0; r < map.H; r += step)
    for (let c = 0; c < map.W; c += step) {
      const k = r * map.W + c;
      if (map.room[k] !== ri || map.wall[k]) continue;
      const [x, y] = map.center(c, r);
      targets.push([x, y]);
      if (!low?.[k] && !tall?.[k]) floors.push([x, y, map.floorZ[k]]);
      if (Number.isFinite(map.ceilZ?.[k])) ceils.push([x, y, map.ceilZ[k]]);
    }
  const half = (hfov / 2 - 2) * DEG, cands = [];
  for (const s of spots) {
    const vis = targets.map((t, j) => (sees(s, t) ? j : -1)).filter((j) => j >= 0);
    for (let i = 0; i < 8; i++) {
      const yaw = Math.atan2(Math.sin((i * Math.PI) / 4), Math.cos((i * Math.PI) / 4));
      cands.push({ x: s[0], y: s[1], yaw, pitch: 0, look: "level", covers: vis.filter((j) => angleTo(Math.atan2(targets[j][1] - s[1], targets[j][0] - s[0]), yaw) <= half) });
    }
  }
  const covered = new Uint8Array(targets.length), chosen = [];
  let seen = 0;
  while (chosen.length < maxViews) {
    let best = null, gain = 0;
    for (const c of cands) {
      if (chosen.includes(c)) continue;
      const g = c.covers.reduce((a, j) => a + (covered[j] ? 0 : 1), 0) + c.covers.length * 1e-4; // ties: the one that sees more
      if (g > gain) [best, gain] = [c, g];
    }
    if (!best || (gain < 1 && chosen.length >= minViews) || (seen / Math.max(1, targets.length) >= coverage && chosen.length >= minViews)) break;
    chosen.push(best);
    for (const j of best.covers) if (!covered[j]) (covered[j] = 1), seen++;
  }
  // The floor and ceiling each view shows, through the lens (a level camera misses the floor near it).
  const K = intrinsics(pinholeLens(hfov), W, H), z0 = (x, y) => camZ(map, x, y, alt);
  const losCache = new Map(), los = (s, list, j) => {
    const key = `${s[0]},${s[1]},${list === floors ? "f" : "c"}`;
    if (!losCache.has(key)) losCache.set(key, list.map((t) => sees(s, t)));
    return losCache.get(key)[j];
  };
  const shows = (c, list) => inPicture(K, W, H, { ...c, z: z0(c.x, c.y) }, list).filter((j) => los([c.x, c.y], list, j));
  const fl = new Uint8Array(floors.length), ce = new Uint8Array(ceils.length), share = (a) => (a.length ? a.reduce((n, v) => n + v, 0) / a.length : 1);
  for (const c of chosen) (shows(c, floors).forEach((j) => (fl[j] = 1)), shows(c, ceils).forEach((j) => (ce[j] = 1)));
  const extra = [];
  for (const s of spots)
    for (let i = 0; i < 4; i++) {
      const yaw = Math.atan2(Math.sin((i * Math.PI) / 2 + Math.PI / 4), Math.cos((i * Math.PI) / 2 + Math.PI / 4));
      for (const [look, pitch] of [["down", down * DEG], ["up", -up * DEG]]) {
        const c = { x: s[0], y: s[1], yaw, pitch, look };
        Object.assign(c, { floor: shows(c, floors), ceil: shows(c, ceils) });
        extra.push(c);
      }
    }
  for (let n = 0; n < maxExtra; n++) {
    const needF = share(fl) < floorCoverage, needC = share(ce) < ceilingCoverage;
    if (!needF && !needC) break;
    let best = null, gain = 0;
    for (const c of extra) {
      if (chosen.includes(c)) continue;
      const g = (needF ? c.floor.reduce((a, j) => a + (fl[j] ? 0 : 1), 0) / Math.max(1, floors.length) : 0) + (needC ? c.ceil.reduce((a, j) => a + (ce[j] ? 0 : 1), 0) / Math.max(1, ceils.length) : 0);
      if (g > gain) [best, gain] = [c, g];
    }
    if (!best || gain < minGain) break;
    chosen.push(best);
    best.floor.forEach((j) => (fl[j] = 1));
    best.ceil.forEach((j) => (ce[j] = 1));
  }
  const look = { level: 0, down: 1, up: 2 }, planned = { level: r2(seen / Math.max(1, targets.length)), floor: r2(share(fl)), ceiling: r2(share(ce)) };
  chosen.sort((a, b) => look[a.look] - look[b.look] || spots.findIndex((s) => s[0] === a.x && s[1] === a.y) - spots.findIndex((s) => s[0] === b.x && s[1] === b.y) || a.yaw - b.yaw);
  return chosen.map((c) => ({ ...poseAt(map, c, roomId, alt), planned }));
}

// The camera's height at (x, y): alt above the floor, below the ceiling.
const camZ = (map, x, y, alt = SURVEY.alt) => { const f = map.floorAt(x, y) ?? 0; return Math.min(f + alt, (map.ceilingAt(x, y) ?? f + 2.4) - 0.25); };
const poseAt = (map, c, room, alt = SURVEY.alt) => ({ x: r2(c.x), y: r2(c.y), z: r2(camZ(map, c.x, c.y, alt)), yaw: +c.yaw.toFixed(4), pitch: +c.pitch.toFixed(4), roll: 0, room, look: c.look });
// From p to q in plan within range, past no wall and nothing tall (1.4 m band).
function seesIn(map, p, q, range = SURVEY.range) {
  const L = Math.hypot(q[0] - p[0], q[1] - p[1]), n = Math.ceil(L / map.o.cell), tall = map.occ?.[map.bandOf(1.4)];
  if (L > range) return false;
  for (let i = 1; i < n; i++) {
    const k = map.idx(p[0] + ((q[0] - p[0]) * i) / n, p[1] + ((q[1] - p[1]) * i) / n);
    if (k < 0 || map.wall[k] || tall?.[k]) return false;
  }
  return true;
}
// Indexes of the points [x, y, z] the pose { x, y, z, yaw, pitch } has in its picture.
function inPicture(K, W, H, c, pts) {
  const cam = droneCamera({ x: c.x, y: c.y, z: c.z, yaw: c.yaw, pitch: c.pitch, roll: 0 }, 0), out = [];
  pts.forEach((t, j) => {
    const v = toCam(cam, t), px = v[2] > 0.1 && project(K, v);
    if (px && px[0] >= 0 && px[0] <= W && px[1] >= 0 && px[1] <= H) out.push(j);
  });
  return out;
}

// More views of a room whose measured floor (ceiling) share is short: from open spots (spotClear m from anything at the
// camera's height) every spotStep m in the room, away from the spots used, 8 headings looking down (and up); the one
// that shows the most still unseen (projected, in plan's sight) first. -> [{ x, y, yaw, pitch, look }]
function moreSpots(map, roomId, used) {
  map.sync?.();
  const ri = map.rooms.findIndex((r) => r.id === roomId), cl = map.clear?.[map.bandOf(SURVEY.alt)], step = Math.max(1, Math.round(SURVEY.spotStep / map.o.cell)), out = [];
  if (ri < 0 || !cl) return out;
  for (let r = step >> 1; r < map.H; r += step)
    for (let c = step >> 1; c < map.W; c += step) {
      const k = r * map.W + c, [x, y] = map.center(c, r);
      if (map.room[k] !== ri || map.wall[k] || !(cl[k] >= SURVEY.spotClear) || used.some((u) => Math.hypot(u[0] - x, u[1] - y) < SURVEY.spotStep)) continue;
      for (let i = 0; i < 8; i++)
        for (const [look, pitch] of [["down", SURVEY.down * DEG], ["up", -SURVEY.up * DEG]]) out.push({ x, y, yaw: Math.atan2(Math.sin((i * Math.PI) / 4), Math.cos((i * Math.PI) / 4)), pitch, look });
    }
  return out;
}
function bestMore(map, cands, targets, grid, needF, needC) {
  const K = intrinsics(pinholeLens(SURVEY.hfov), SURVEY.width, SURVEY.height), sight = new Map();
  const open = (list, seen) => list.filter((t) => !seen[t.g]);
  const fl = needF ? open(targets.floor, grid.floor) : [], ce = needC ? open(targets.ceil, grid.ceil) : [];
  const gain = (c, list, n) => {
    if (!list.length) return 0;
    const key = `${c.x},${c.y},${list === fl ? "f" : "c"}`;
    if (!sight.has(key)) sight.set(key, list.map((t) => seesIn(map, [c.x, c.y], t.p)));
    const ok = sight.get(key);
    return inPicture(K, SURVEY.width, SURVEY.height, { ...c, z: camZ(map, c.x, c.y) }, list.map((t) => t.p)).filter((j) => ok[j]).length / n;
  };
  let best = null;
  for (const c of cands) {
    if ((c.look === "down" && !needF) || (c.look === "up" && !needC)) continue;
    const g = (c.look === "down" ? gain(c, fl, targets.floor.length) : 0) + (c.look === "up" ? gain(c, ce, targets.ceil.length) : 0);
    if (!best || g > best.gain) best = { c, gain: g };
  }
  return best;
}

// ---- pixels to the house ----

// A view: { pose, lens, width, height, depth (Float32Array, metres along each ray, rows top-down, NaN where thin),
// dw, dh (the depth's size) }. The box [x0, y0, x1, y1] (picture pixels, or fractions) -> the thing on the box's central
// ray, in H. A box drawn around a thing also holds what is behind it (the wall, the floor through a chair) and may be
// partly hidden by something in front, so its depth is a cluster of the middle half's depths (a window 15% of the
// distance deep: a sofa 5 m away is one): the largest, unless that one is the background (as deep as what shows just
// outside the box, above and beside it) and a nearer one holds at least half as much and shows less outside the box than
// it (a fan under the ceiling, a chair before a wall: the thing ends at its box, the wall goes on; a door frame in front
// of a cabinet against a wall goes on too, so the cabinet stays). The point is that cluster's near side (30th
// percentile) moved back along the view by about half the thing's width (its centre, not its front; at most 0.4 m),
// never into the background (a thing as deep as its background, flat on a wall, isn't moved). r is half the box's
// width there, zMin and zMax the 2nd and 98th percentile heights of the box's points at about that depth.
// surface: true (glass, mirrors: the depth inside shows what is behind or reflected) uses the box's edges (the frame,
// the wall around it) instead, without moving back.
export function liftBox(view, box, { surface = false } = {}) {
  const { pose, lens, width: W, height: H, depth, dw = W, dh = H } = view;
  let [x0, y0, x1, y1] = box.map(Number);
  if ([x0, y0, x1, y1].every((v) => v >= 0 && v <= 1.0001)) [x0, y0, x1, y1] = [x0 * W, y0 * H, x1 * W, y1 * H]; // fractions after all
  [x0, x1] = [Math.max(0, Math.min(x0, x1)), Math.min(W, Math.max(x0, x1))];
  [y0, y1] = [Math.max(0, Math.min(y0, y1)), Math.min(H, Math.max(y0, y1))];
  if (!(x1 - x0 >= 1 && y1 - y0 >= 1)) return null;
  const sx = dw / W, sy = dh / H, at = (u, v) => depth[v * dw + u];
  const U0 = Math.max(0, Math.floor(x0 * sx)), U1 = Math.min(dw - 1, Math.max(U0, Math.ceil(x1 * sx) - 1)), V0 = Math.max(0, Math.floor(y0 * sy)), V1 = Math.min(dh - 1, Math.max(V0, Math.ceil(y1 * sy) - 1));
  // The box's edges (a band 12% of its size wide: the frame of a mirror, the wall around it), the background (a band just
  // outside the box, above it and beside its upper two thirds) and all around it (a band 15% of its size wide).
  const bu = Math.max(1, Math.round(0.12 * (U1 - U0 + 1))), bv = Math.max(1, Math.round(0.12 * (V1 - V0 + 1))), ring = [], back = [], around = [];
  for (let v = V0; v <= V1; v++)
    for (let u = U0; u <= U1; u++) if ((u < U0 + bu || u > U1 - bu || v < V0 + bv || v > V1 - bv) && at(u, v) > 0.05) ring.push(at(u, v));
  const ou = Math.max(2, Math.round(0.1 * (U1 - U0 + 1))), ov = Math.max(2, Math.round(0.1 * (V1 - V0 + 1))), vMid = V0 + Math.round((2 * (V1 - V0)) / 3);
  for (let v = Math.max(0, V0 - ov); v <= vMid; v++)
    for (let u = Math.max(0, U0 - ou); u <= Math.min(dw - 1, U1 + ou); u++) if ((v < V0 || u < U0 || u > U1) && at(u, v) > 0.05) back.push(at(u, v));
  const au = Math.max(2, Math.round(0.15 * (U1 - U0 + 1))), av = Math.max(2, Math.round(0.15 * (V1 - V0 + 1)));
  for (let v = Math.max(0, V0 - av); v <= Math.min(dh - 1, V1 + av); v++)
    for (let u = Math.max(0, U0 - au); u <= Math.min(dw - 1, U1 + au); u++) if ((v < V0 || v > V1 || u < U0 || u > U1) && at(u, v) > 0.05) around.push(at(u, v));
  const pct = (a, q) => (a.length ? [...a].sort((p, q2) => p - q2)[Math.min(a.length - 1, Math.floor(a.length * q))] : null);
  const bg = pct(back, 0.5);
  let d, near;
  if (surface) {
    d = near = pct(ring, 0.2);
    if (d == null) return null;
  } else {
    const ds = [];
    const [a0, a1] = [Math.floor((x0 + (x1 - x0) / 4) * sx), Math.ceil((x1 - (x1 - x0) / 4) * sx)], [b0, b1] = [Math.floor((y0 + (y1 - y0) / 4) * sy), Math.ceil((y1 - (y1 - y0) / 4) * sy)];
    for (let v = Math.max(0, b0); v <= Math.min(dh - 1, Math.max(b0, b1 - 1)); v++)
      for (let u = Math.max(0, a0); u <= Math.min(dw - 1, Math.max(a0, a1 - 1)); u++) if (at(u, v) > 0.05) ds.push(at(u, v));
    if (!ds.length) return null;
    ds.sort((a, b) => a - b);
    // Clusters: windows of depth that hold the most samples, each one's start past the previous one's end.
    const clusters = [];
    for (let i = 0; i < ds.length; ) {
      let best = { lo: i, n: 0 };
      for (let k = i, j = i; k < ds.length && ds[k] <= ds[i] + 0.6; k++) {
        const top = ds[k] + Math.max(0.25, 0.15 * ds[k]);
        while (j < ds.length && ds[j] <= top) j++;
        if (j - k > best.n) best = { lo: k, n: j - k };
      }
      clusters.push(best);
      i = best.lo + best.n;
    }
    const at30 = (c) => ds[c.lo + Math.floor(c.n * 0.3)], biggest = clusters.reduce((a, c) => (c.n > a.n ? c : a));
    const isBack = (c) => bg != null && at30(c) >= bg - Math.max(0.3, 0.1 * bg);
    const outside = (c) => around.filter((x) => x >= ds[c.lo] - 0.05 && x <= ds[c.lo + c.n - 1] + 0.05).length / Math.max(1, around.length);
    const fore = isBack(biggest) && clusters.find((c) => c !== biggest && !isBack(c) && c.n >= Math.max(0.15 * ds.length, 0.5 * biggest.n) && outside(c) < outside(biggest));
    const pick = fore || biggest;
    [d, near] = [at30(pick), ds[pick.lo]];
  }
  const K = intrinsics(lens, W, H), cam = droneCamera(pose, lens.uptiltDeg ?? 0), uc = (x0 + x1) / 2, vc = (y0 + y1) / 2, ray = unproject(K, uc, vc);
  const zc = d * ray[2], w = ((x1 - x0) / K.fx) * zc, h = ((y1 - y0) / K.fy) * zc;
  const p = camToH(cam, [((uc - K.cx) / K.fx) * zc, ((vc - K.cy) / K.fy) * zc, zc]), dir = [p[0] - cam.p[0], p[1] - cam.p[1]], L = Math.hypot(...dir) || 1;
  const shift = !surface && bg != null && bg - d > 0.3 ? Math.min(0.5 * w, 0.4, bg - d - 0.05) : 0; // flat on the wall: none
  const c = [p[0] + (shift * dir[0]) / L, p[1] + (shift * dir[1]) / L, p[2]];
  // The thing's own points: the whole box, from its nearest depth to about its size behind (not the wall behind it).
  const far = near + Math.max(0.25, 0.75 * Math.min(w, h)), zs = [], step = Math.max(1, Math.round(Math.sqrt(((x1 - x0) * sx * (y1 - y0) * sy) / 1500)));
  for (let v = V0; v <= V1; v += step)
    for (let u = U0; u <= U1; u += step) {
      const dd = at(u, v);
      if (dd >= near - 0.1 && dd <= far) zs.push(camToH(cam, unproject(K, (u + 0.5) / sx, (v + 0.5) / sy).map((q) => q * dd))[2]);
    }
  zs.sort((a, b) => a - b);
  const zMin = zs.length ? zs[Math.floor(zs.length * 0.02)] : c[2] - h / 2, zMax = zs.length ? zs[Math.min(zs.length - 1, Math.floor(zs.length * 0.98))] : c[2] + h / 2;
  return { x: c[0], y: c[1], z: c[2], r: Math.max(0.05, w / 2), width: w, height: h, zMin: Math.min(zMin, c[2]), zMax: Math.max(zMax, c[2]), d: d + shift, bg,
    toward: [cam.p[0] - c[0], cam.p[1] - c[1]], from: [cam.p[0], cam.p[1]], across: [-dir[1] / L, dir[0] / L] };
}

// From `from` along the 2D direction `dir` (unit): metres to the first wall cell of the map (Infinity within `max`).
function wallAlong(map, from, dir, max = 12) {
  const c = map.o.cell / 2;
  for (let t = c; t <= max; t += c) {
    const k = map.idx(from[0] + t * dir[0], from[1] + t * dir[1]);
    if (k < 0) return t;
    if (map.wall[k]) return t;
  }
  return Infinity;
}

// The wall's direction at (x, y): the main axis of the wall cells within 0.5 m (null without enough of them).
function wallDir(map, x, y) {
  const c = map.o.cell, pts = [];
  for (let dy = -0.5; dy <= 0.5; dy += c)
    for (let dx = -0.5; dx <= 0.5; dx += c) {
      const k = map.idx(x + dx, y + dy);
      if (k >= 0 && map.wall[k] && dx * dx + dy * dy <= 0.25) pts.push([dx, dy]);
    }
  if (pts.length < 6) return null;
  const mx = pts.reduce((a, p) => a + p[0], 0) / pts.length, my = pts.reduce((a, p) => a + p[1], 0) / pts.length;
  let sxx = 0, syy = 0, sxy = 0;
  for (const [a, b] of pts) (sxx += (a - mx) ** 2), (syy += (b - my) ** 2), (sxy += (a - mx) * (b - my));
  const th = 0.5 * Math.atan2(2 * sxy, sxx - syy);
  return [Math.cos(th), Math.sin(th)];
}

// Merge things with the same key closer than `near` (or their radii): confidence-weighted position, the largest extent,
// confidence 1 − Π(1 − c) (capped at 0.99), every view that showed it.
export function dedupe(items, key, near = SURVEY.near) {
  const out = [];
  for (const it of [...items].sort((a, b) => b.confidence - a.confidence)) {
    const g = out.find((o) => key(o) === key(it) && Math.hypot(o.x - it.x, o.y - it.y) <= Math.max(near, Math.min(1.5, o.r + it.r)) && Math.abs(o.z - it.z) < 1.2);
    if (!g) {
      out.push({ ...it, views: [...it.views] });
      continue;
    }
    const w = g.confidence + it.confidence || 1;
    Object.assign(g, {
      x: (g.x * g.confidence + it.x * it.confidence) / w, y: (g.y * g.confidence + it.y * it.confidence) / w, z: (g.z * g.confidence + it.z * it.confidence) / w,
      r: Math.max(g.r, it.r), zMin: Math.min(g.zMin, it.zMin), zMax: Math.max(g.zMax, it.zMax),
      confidence: Math.min(0.99, 1 - (1 - g.confidence) * (1 - it.confidence)), views: [...g.views, ...it.views],
    });
  }
  return out;
}

// ---- prompts ----

const area = (map, room) => {
  const i = map.rooms.findIndex((q) => q.id === room.id);
  return i >= 0 ? map.room.reduce((n, v) => n + (v === i ? 1 : 0), 0) * map.o.cell ** 2 : polyArea(room.outline);
};
const ceiling = (room) => (room.ceiling?.z ?? room.floorZ + 2.4) - room.floorZ;

// The whole house in a few lines for the (cached, same for every room) system prompt.
export function houseContext(house, map) {
  const name = (id) => house.rooms.find((r) => r.id === id)?.name ?? "outside";
  const doors = [...new Set((house.doors ?? []).filter((d) => d.passable && d.rooms?.[1]).map((d) => [name(d.rooms[0]), name(d.rooms[1])].sort().join(" and ")))];
  return `\n\nThe home: ${house.rooms.map((r) => `${r.name} (${Math.round(area(map, r))} m², ceiling about ${ceiling(r).toFixed(1)} m)`).join("; ")}.`
    + `${doors.length ? ` Doorways join ${doors.join("; ")}.` : ""} Room names like "Room 2" are placeholders from the scanner.`;
}

function roomContext(house, map, room) {
  const name = (id) => (id ? house.rooms.find((r) => r.id === id)?.name ?? "another room" : "outside");
  const doors = (house.doors ?? []).filter((d) => d.passable && d.rooms?.includes(room.id)).map((d) => name(d.rooms[0] === room.id ? d.rooms[1] : d.rooms[0]));
  const counts = new Map();
  for (const l of house.landmarks ?? []) if (l.room === room.id && l.source !== "claude") counts.set(l.name.toLowerCase(), (counts.get(l.name.toLowerCase()) ?? 0) + 1);
  const objs = [...counts].map(([n, c]) => (c > 1 ? `${n} x${c}` : n));
  const kos = (map.keepouts ?? house.keepouts ?? []).filter((k) => map.roomAt(k.x ?? k.polygon?.[0]?.[0], k.y ?? k.polygon?.[0]?.[1])?.id === room.id).map((k) => (k.kind === "fan" ? "ceiling fan" : k.kind));
  return `This is ${room.name} (${Math.round(area(map, room))} m², floor to ceiling about ${ceiling(room).toFixed(1)} m).`
    + `${doors.length ? ` Doorways to: ${[...new Set(doors)].join(", ")}.` : ""}${objs.length ? ` Objects the scan already lists here: ${objs.join(", ")}.` : ""}`
    + `${kos.length ? ` Already known no-fly zones here: ${[...new Set(kos)].join(", ")}.` : ""}`;
}

function viewText(map, room, v, k, n) {
  const r = map.roomAt(v.x + 2.5 * Math.cos(v.yaw), v.y + 2.5 * Math.sin(v.yaw)), look = v.look === "down" ? ", looking down at the floor" : v.look === "up" ? ", looking up at the ceiling" : "";
  return `View ${k} of ${n}: facing ${!r ? "a wall" : r.id === room.id ? `into ${theRoom(room.name)}` : `toward ${theRoom(r.name)}`}${look}.`;
}

// ---- what the views saw ----

// A grid (SURVEY.seenCell) of the floor and ceiling cells some view's depth reached: a point from 60 cm below that spot's
// floor to 30 cm above it (the floor, or what lies on it), or within 60 cm of its ceiling.
function seenGrid(map) {
  const c = SURVEY.seenCell, W = Math.ceil((map.W * map.o.cell) / c), H = Math.ceil((map.H * map.o.cell) / c);
  return { c, W, H, floor: new Uint8Array(W * H), ceil: new Uint8Array(W * H), at: (x, y) => { const i = Math.floor((x - map.x0) / c), j = Math.floor((y - map.y0) / c); return i < 0 || j < 0 || i >= W || j >= H ? -1 : j * W + i; } };
}
// The grid cells of the floor and ceiling a view's depth reached: { floor: Set, ceil: Set }.
function reach(map, grid, view) {
  const { pose, lens, width: W, height: H, depth, dw, dh } = view, K = intrinsics(lens, W, H), cam = droneCamera(pose, lens.uptiltDeg ?? 0), out = { floor: new Set(), ceil: new Set() };
  for (let v = 0; v < dh; v += 2)
    for (let u = 0; u < dw; u += 2) {
      const d = depth[v * dw + u];
      if (!(d > 0.05 && d < 12)) continue;
      const P = camToH(cam, unproject(K, ((u + 0.5) / dw) * W, ((v + 0.5) / dh) * H).map((q) => q * d)), k = map.idx(P[0], P[1]), g = grid.at(P[0], P[1]);
      if (k < 0 || g < 0 || map.room[k] < 0) continue;
      const f = P[2] - map.floorZ[k], up = P[2] - map.ceilZ[k]; // a scan's floor and ceiling are soft: often 20-50 cm off the plan's
      if (f >= -0.6 && f <= 0.3) out.floor.add(g); // the floor, or what lies on it
      else if (Math.abs(up) <= 0.6) out.ceil.add(g); // further out: a hole in the scan seen through, neither
    }
  return out;
}
const mark = (grid, hit) => (hit.floor.forEach((g) => (grid.floor[g] = 1)), hit.ceil.forEach((g) => (grid.ceil[g] = 1)));
// A room's cells of the grid: its floor (not under furniture) and its ceiling, { g, p: [x, y, z] } each.
function targetsOf(map, roomId, grid) {
  const ri = map.rooms.findIndex((r) => r.id === roomId), low = map.occ?.[map.levels.length - 1], tall = map.occ?.[map.bandOf(1.4)], out = { floor: [], ceil: [] };
  for (let g = 0; g < grid.W * grid.H; g++) {
    const x = map.x0 + ((g % grid.W) + 0.5) * grid.c, y = map.y0 + (((g / grid.W) | 0) + 0.5) * grid.c, k = map.idx(x, y);
    if (k < 0 || map.room[k] !== ri || map.wall[k]) continue;
    if (!low?.[k] && !tall?.[k]) out.floor.push({ g, p: [x, y, map.floorZ[k]] });
    if (Number.isFinite(map.ceilZ?.[k])) out.ceil.push({ g, p: [x, y, map.ceilZ[k]] });
  }
  return out;
}
const seenShare = (list, seen) => (list.length ? list.filter((t) => seen[t.g]).length / list.length : 1);
// The room's floor (not under furniture) and ceiling no view saw: shares, m², and the patches of at least unseenMin in words.
function unseenOf(map, house, roomId, grid, views) {
  const { c } = grid, a = c * c, tg = targetsOf(map, roomId, grid);
  const out = { floor: 0, floorShare: 0, ceiling: 0, ceilingShare: 0, patches: [], text: "" }, room = house.rooms.find((r) => r.id === roomId);
  for (const [what, seen, list] of [["floor", grid.floor, tg.floor], ["ceiling", grid.ceil, tg.ceil]]) {
    const target = new Uint8Array(grid.W * grid.H), n = list.length;
    let miss = 0;
    for (const t of list) (target[t.g] = 1), (miss += seen[t.g] ? 0 : 1);
    out[what] = r2(miss * a);
    out[`${what}Share`] = n ? r2(miss / n) : 0;
    const done = new Uint8Array(target.length);
    for (let g0 = 0; g0 < target.length; g0++) {
      if (!target[g0] || seen[g0] || done[g0]) continue;
      const part = [g0];
      done[g0] = 1;
      for (let q = 0; q < part.length; q++) {
        const g = part[q], i = g % grid.W, j = (g / grid.W) | 0;
        for (let dj = -1; dj <= 1; dj++)
          for (let di = -1; di <= 1; di++) {
            const h = (j + dj) * grid.W + i + di;
            if (i + di >= 0 && i + di < grid.W && j + dj >= 0 && j + dj < grid.H && target[h] && !seen[h] && !done[h]) (done[h] = 1), part.push(h);
          }
      }
      if (part.length * a < SURVEY.unseenMin) continue;
      const x = map.x0 + (part.reduce((s, g) => s + (g % grid.W), 0) / part.length + 0.5) * c, y = map.y0 + (part.reduce((s, g) => s + ((g / grid.W) | 0), 0) / part.length + 0.5) * c;
      out.patches.push({ what, x: r2(x), y: r2(y), m2: r2(part.length * a), where: whereIn(map, house, x, y, views, what) });
    }
  }
  out.patches.sort((p, q) => q.m2 - p.m2);
  if (out.patches.length) out.text = `${theRoom(room?.name ?? roomId)}: ${out.patches.slice(0, 4).map((p) => `about ${p.m2.toFixed(1)} m² of ${p.what} ${p.where}`).join(", ")}`;
  return out;
}
function whereIn(map, house, x, y, views, what) {
  let best = null, bd = 1.5;
  for (const l of house.landmarks ?? []) if (Math.hypot(l.x - x, l.y - y) < bd && l.name.length <= 30) [best, bd] = [l.name.toLowerCase(), Math.hypot(l.x - x, l.y - y)];
  if (best) return `${what === "ceiling" ? "above" : /ceiling|light|lamp|fan|chandelier|pendant/.test(best) ? "below" : "near"} the ${best}`;
  if (views.some((v) => Math.hypot(v.x - x, v.y - y) < 1.2)) return `${what === "floor" ? "below" : "above"} where the pictures were taken`;
  for (let dy = -0.6; dy <= 0.6; dy += 0.1) for (let dx = -0.6; dx <= 0.6; dx += 0.1) if (map.wall[map.idx(x + dx, y + dy)]) return "along a wall";
  return "in the open";
}

// ---- estimate, survey ----

const modelOf = (settings) => settings?.get?.("aiModel") || settings?.get?.("model") || "claude-opus-5";
const price = (model, u) => costOf(model, u) ?? costOf("claude-fable-5", u);

// -> { model, perImage (tokens), rooms: [{ id, name, views, input, output, cacheWrite, cacheRead, cost, maxViews, maxCost }],
// total: { rooms, images, input, output, cost, maxImages, maxCost }, text }. views: the planned pictures; maxViews and
// maxCost with every view the measured floor or ceiling may add (SURVEY.maxMore a room).
export function estimateSurvey({ house, map, settings = null, rooms = null, model = modelOf(settings) }) {
  const per = imageTokens(SURVEY.width, SURVEY.height, model), sys = textTokens(SURVEY_PROMPT + houseContext(house, map));
  const list = (rooms ?? house.rooms.map((r) => r.id)).map((id) => house.rooms.find((r) => r.id === id || r.name === id)).filter(Boolean);
  const out = list.map((r, i) => {
    const views = surveyViews(map, r.id).length, u = { input: views * per + SURVEY.roomTokens, output: SURVEY.outTokens, cacheWrite: i ? 0 : sys, cacheRead: i ? sys : 0 };
    return { id: r.id, name: r.name, views, ...u, cost: price(model, u), maxViews: views + SURVEY.maxMore, maxCost: price(model, { ...u, input: u.input + SURVEY.maxMore * per }) };
  });
  const sum = (k) => out.reduce((a, r) => a + r[k], 0);
  const total = { rooms: out.length, images: sum("views"), input: sum("input") + sum("cacheWrite") + sum("cacheRead"), output: sum("output"), cost: sum("cost"), maxImages: sum("maxViews"), maxCost: sum("maxCost") };
  return { model, perImage: per, width: SURVEY.width, height: SURVEY.height, rooms: out, total,
    text: `${total.rooms} room${total.rooms === 1 ? "" : "s"}, ${total.images} pictures of the 3D scan (${SURVEY.width}x${SURVEY.height}, about ${per} tokens each): `
      + `about ${Math.round(total.input / 100) / 10}K tokens in and ${Math.round(total.output / 100) / 10}K out, about $${total.cost.toFixed(2)} with ${model}`
      + ` (at most ${total.maxImages} pictures and $${total.maxCost.toFixed(2)} if the floor or ceiling needs pictures from more spots).` };
}

// -> { ok, rooms: [{ id, name, suggestedName, kind, confidence, views }], landmarks: [{ name, room, x, y, z, r, confidence,
//   views: [{ room, view, box }], known? }], hazards: [{ kind, room, x, y, z, r, zMin, zMax, why, confidence, views, known? }],
//   views: [{ room, index, pose, image, width, height }], cost: { estimate, cost, input, output, cacheRead, cacheWrite,
//   requests, model, rooms: [{ id, cost }] }, unplaced, stopped }
// approved: the user OK'd the estimate (which says pictures go to Claude). limit: dollars (default twice the estimate).
export async function surveyHouse({ house, map, vox = null, twin, settings = null, claude, onProgress = () => {}, signal = null, rooms = null,
  estimate = null, approved = false, limit = null, encode = (px) => toJpegBase64(px) }) {
  const est = estimate ?? estimateSurvey({ house, map, settings, rooms, model: claude.model });
  const res = { ok: false, rooms: [], landmarks: [], hazards: [], views: [], unplaced: 0, stopped: null,
    cost: { estimate: r2x(est.total.cost), cost: 0, input: 0, output: 0, cacheRead: 0, cacheWrite: 0, requests: 0, model: claude.model, rooms: [] } };
  if (!twin) return { ...res, stopped: "the 3D scan isn't loaded" };
  const permit = await claude.permit("survey", { approved });
  if (!permit.ok) return { ...res, stopped: permit.reason };
  const scope = { limit: limit ?? Math.max(0.05, 2 * est.total.cost, 1.2 * (est.total.maxCost ?? 0)), spent: 0 }, system = SURVEY_PROMPT + houseContext(house, map);
  const lens = pinholeLens(SURVEY.hfov), W = SURVEY.width, H = SURVEY.height, dw = Math.round(W * SURVEY.depthScale), dh = Math.round(H * SURVEY.depthScale);
  const marks = [], hazards = [], grid = seenGrid(map);
  for (const [i, e] of est.rooms.entries()) {
    if (signal?.aborted) res.stopped ??= "stopped";
    if (res.stopped) break;
    const room = house.rooms.find((r) => r.id === e.id), poses = surveyViews(map, room.id), same = (a, b) => a.x === b.x && a.y === b.y && a.yaw === b.yaw && a.pitch === b.pitch;
    const level = poses.filter((p) => p.look === "level"), pitched = poses.filter((p) => p.look !== "level");
    const spare = surveyViews(map, room.id, { minViews: 16, maxViews: 16, coverage: 2, maxExtra: 0 }).filter((p) => !poses.some((q) => same(p, q)));
    onProgress({ phase: "render", room: room.id, name: room.name, index: i, total: est.rooms.length, text: `Rendering ${poses.length} views of ${theRoom(room.name)}…` });
    const views = [], skipped = [], depthOf = (pose) => twin.depth(pose, { width: dw, height: dh, lens, actors: false, props: false });
    const add = async (pose, depth) => views.push({ room: room.id, index: views.length + 1, pose, lens, width: W, height: H, depth, dw, dh,
      image: await encode(await twin.pixels(pose, { width: W, height: H, lens, actors: false, props: false })) });
    for (const pose of [...level, ...spare]) {
      if (views.length >= level.length) break;
      const depth = await depthOf(pose), open = medianDepth(depth);
      if (open < SURVEY.minOpen) skipped.push({ pose, depth, open });
      else await add(pose, depth);
    }
    // A narrow room may have nothing better: its most open ones then, up to minViews.
    for (const v of skipped.sort((a, b) => b.open - a.open)) if (views.length < Math.min(level.length, SURVEY.minViews)) await add(v.pose, v.depth);
    const swapped = skipped.length - (views.length - views.filter((v) => !skipped.some((k) => k.pose === v.pose)).length);
    for (const pose of pitched) await add(pose, await depthOf(pose));
    for (const v of views) mark(grid, reach(map, grid, v));
    if (swapped) onProgress({ phase: "render", room: room.id, name: room.name, index: i, total: est.rooms.length, text: `${swapped} view${swapped > 1 ? "s" : ""} of ${theRoom(room.name)} faced a wall up close: used others.` });
    // Short of the floor (ceiling) wanted, as the depth measured: views from other spots at what is still unseen.
    const tg = targetsOf(map, room.id, grid), cands = moreSpots(map, room.id, poses.map((p) => [p.x, p.y]));
    let more = 0;
    for (let tries = 0; more < SURVEY.maxMore && tries < SURVEY.moreTries && !signal?.aborted; tries++) {
      const needF = seenShare(tg.floor, grid.floor) < SURVEY.floorCoverage, needC = seenShare(tg.ceil, grid.ceil) < SURVEY.ceilingCoverage;
      const best = (needF || needC) && bestMore(map, cands, tg, grid, needF, needC);
      if (!best || best.gain < SURVEY.moreGain) break;
      cands.splice(cands.indexOf(best.c), 1);
      const pose = { ...poseAt(map, best.c, room.id), more: true }, depth = await depthOf(pose), hit = reach(map, grid, { pose, lens, width: W, height: H, depth, dw, dh });
      const fresh = (list, seen, got) => list.filter((t) => !seen[t.g] && got.has(t.g)).length / Math.max(1, list.length);
      if ((needF ? fresh(tg.floor, grid.floor, hit.floor) : 0) + (needC ? fresh(tg.ceil, grid.ceil, hit.ceil) : 0) < SURVEY.moreGain) continue;
      mark(grid, hit);
      await add(pose, depth);
      more++;
    }
    const seen = { floor: r2(seenShare(tg.floor, grid.floor)), ceiling: r2(seenShare(tg.ceil, grid.ceil)) };
    if (more) onProgress({ phase: "render", room: room.id, name: room.name, index: i, total: est.rooms.length,
      text: `${more} more view${more > 1 ? "s" : ""} of ${theRoom(room.name)} from other spots, for floor or ceiling the first ones missed: ${Math.round(100 * seen.floor)}% of the floor and ${Math.round(100 * seen.ceiling)}% of the ceiling seen.` });
    onProgress({ phase: "ask", room: room.id, name: room.name, index: i, total: est.rooms.length, text: `Asking Claude about ${theRoom(room.name)}…` });
    const content = [text(`${roomContext(house, map, room)} ${views.length} views follow.`)];
    for (const v of views) content.push(text(viewText(map, room, v.pose, v.index, views.length)), { ...image(v.image), width: W, height: H });
    content.push(text(`Survey ${theRoom(room.name)}: the room, its landmarks and the hazards for the drone.`));
    const r = await claude.call({ purpose: "survey", system, content, schema: SURVEY_SCHEMA, expectOut: SURVEY.outTokens, scope, approved, signal });
    if (r.usage) for (const k of ["input", "output", "cacheRead", "cacheWrite", "requests"]) res.cost[k] += r.usage[k];
    if (r.cost != null) (res.cost.cost += r.cost), res.cost.rooms.push({ id: room.id, cost: r2x(r.cost) });
    if (!r.ok) {
      res.rooms.push({ id: room.id, name: room.name, suggestedName: null, kind: null, confidence: 0, views: views.length, more, error: r.reason });
      if (r.budget || r.consent || r.stopped || signal?.aborted) res.stopped = r.reason;
      continue;
    }
    for (const v of views) res.views.push({ room: v.room, index: v.index, pose: (({ planned, ...p }) => p)(v.pose), image: v.image, width: W, height: H });
    const d = r.data;
    res.rooms.push({ id: room.id, name: room.name, suggestedName: String(d.room?.name ?? "").trim().slice(0, 40) || null, kind: d.room?.kind ?? null,
      confidence: clamp01(d.room?.confidence), views: views.length, more });
    // Glass and mirrors: at the box's edges (the frame, the wall around it), and on the wall when the view's ray meets one
    // there or before (the scan shows the room behind the glass, or the reflection, beyond it), with the wall's line.
    const place = (it, kind = null) => {
      const v = views[Math.round(it.view) - 1], b = Array.isArray(it.box) && it.box.length === 4 && it.box.every(Number.isFinite) ? it.box : null, glass = SEE_THROUGH.has(kind);
      let p = v && b && liftBox(v, b, { surface: glass }), line = null;
      if (!p) return (res.unplaced++, null);
      if (glass) {
        const L = Math.hypot(...p.toward) || 1, u = [-p.toward[0] / L, -p.toward[1] / L], wall = wallAlong(map, p.from, u);
        if (Number.isFinite(wall) && L > wall - 0.3) {
          const t = Math.max(0.2, wall - 0.1), a = wallDir(map, p.from[0] + wall * u[0], p.from[1] + wall * u[1]) ?? p.across, h = p.width / 2;
          p = { ...p, x: p.from[0] + t * u[0], y: p.from[1] + t * u[1], toward: [p.from[0] - (p.from[0] + t * u[0]), p.from[1] - (p.from[1] + t * u[1])] };
          line = [[r2(p.x - a[0] * h), r2(p.y - a[1] * h)], [r2(p.x + a[0] * h), r2(p.y + a[1] * h)]];
        }
      }
      const t = Math.hypot(...p.toward) || 1, inRoom = map.roomAt(p.x, p.y) ?? map.roomAt(p.x + (0.25 * p.toward[0]) / t, p.y + (0.25 * p.toward[1]) / t); // on a wall: step back
      if (!inRoom) return (res.unplaced++, null); // outside every room: through a window, or a bad box
      return { room: inRoom.id, x: p.x, y: p.y, z: p.z, r: p.r, zMin: p.zMin, zMax: p.zMax, confidence: clamp01(it.confidence), views: [{ room: room.id, view: v.index, box: b.map((x) => Math.round(x)) }], ...(line && { line }) };
    };
    for (const l of d.landmarks ?? []) {
      const p = place(l), name = String(l.name ?? "").trim().toLowerCase().replace(/^the /, "").slice(0, 40);
      if (p && name) marks.push({ name, ...p });
    }
    for (const h of d.hazards ?? []) {
      const p = place(h, h.kind);
      if (p && HAZARDS.includes(h.kind)) hazards.push({ kind: h.kind, why: String(h.why ?? "").slice(0, 120), ...p });
    }
    onProgress({ phase: "room", room: room.id, name: room.name, index: i, total: est.rooms.length, text: `${room.name}: ${d.room?.name ?? "?"}, ${d.landmarks?.length ?? 0} landmarks, ${d.hazards?.length ?? 0} hazards.` });
  }
  const round = (o) => ({ ...o, x: r2(o.x), y: r2(o.y), z: r2(o.z), r: r2(o.r), zMin: r2(o.zMin), zMax: r2(o.zMax), confidence: r2(o.confidence) });
  // Known: the scan's (or an earlier accepted survey's) landmark of that name, a keep-out of that kind, about there.
  res.landmarks = dedupe(marks, (o) => nameKey(o.name)).map(round).map((l) => {
    const k = (house.landmarks ?? []).filter((m) => sameLandmark(m, l)).sort((a, b) => (a.source === "claude") - (b.source === "claude"))[0];
    return { name: l.name, room: map.roomAt(l.x, l.y)?.id ?? l.room, x: l.x, y: l.y, z: l.z, r: l.r, confidence: l.confidence, views: l.views, ...(k && { known: k.name }) };
  });
  const kos = [...(house.keepouts ?? []), ...(map.keepouts ?? [])];
  res.hazards = dedupe(hazards, (o) => o.kind).map(round).map((h) => {
    const ko = kos.find((k) => sameKeepout(k, h));
    return { kind: h.kind, room: map.roomAt(h.x, h.y)?.id ?? h.room, x: h.x, y: h.y, z: h.z, r: h.r, zMin: h.zMin, zMax: h.zMax, why: h.why, confidence: h.confidence, views: h.views,
      ...(h.line && { line: h.line }), ...(ko && { known: ko.kind }) };
  });
  for (const r of res.rooms) if (!r.error) (r.unseen = unseenOf(map, house, r.id, grid, res.views.filter((v) => v.room === r.id).map((v) => v.pose))), (r.seen = { floor: r2(1 - r.unseen.floorShare), ceiling: r2(1 - r.unseen.ceilingShare) });
  res.cost.cost = r2x(res.cost.cost);
  res.ok = res.rooms.some((r) => !r.error);
  onProgress({ phase: "done", text: surveySummary(res, house) });
  return res;
}
const r2x = (v) => Math.round(v * 10000) / 10000;
// Median of a depth render (NaN where the scan is too thin to say counts as far).
function medianDepth(depth) {
  const step = Math.max(1, Math.floor(depth.length / 4000)), ds = [];
  for (let i = 0; i < depth.length; i += step) ds.push(Number.isFinite(depth[i]) ? depth[i] : Infinity);
  return ds.sort((a, b) => a - b)[ds.length >> 1];
}
const clamp01 = (v) => Math.max(0, Math.min(1, Number(v) || 0));

// For Claude (the survey_house tool) and the log: what was proposed and what it cost.
export function surveySummary(res, house) {
  const name = (id) => house?.rooms?.find((r) => r.id === id)?.name ?? id;
  if (!res.ok) return `The survey didn't run: ${res.stopped ?? res.rooms.find((r) => r.error)?.error ?? "no answer"}.`;
  const rooms = res.rooms.filter((r) => !r.error).map((r) => `${r.name}: ${r.kind}${r.suggestedName && r.suggestedName !== r.name ? `, "${r.suggestedName}"` : ""} (${Math.round(r.confidence * 100)}%)`);
  const marks = res.landmarks.filter((l) => !l.known).map((l) => `${l.name} (${name(l.room)})`), known = res.landmarks.length - marks.length;
  const haz = res.hazards.map((h) => `${h.kind.replace(/-/g, " ")} in ${theRoom(name(h.room))}${h.known ? " (already on the map)" : ""}`);
  const unseen = res.rooms.filter((r) => r.unseen?.text).map((r) => r.unseen.text);
  return `Surveyed ${plural(rooms.length, "room")} for $${res.cost.cost.toFixed(3)} (estimated $${res.cost.estimate.toFixed(3)}). Rooms: ${rooms.join("; ")}.`
    + ` Landmarks: ${marks.length ? marks.join(", ") : "none new"}${known ? ` (and ${known} the map already had)` : ""}. Hazards: ${haz.length ? haz.join(", ") : "none"}.`
    + `${unseen.length ? ` Not seen in the pictures (hazards there may be missing): ${unseen.join("; ")}.` : ""}`
    + `${res.stopped ? ` Stopped early: ${res.stopped}.` : ""} Nothing is on the map yet: the pilot reviews the suggestions in Settings → House.`;
}
const plural = (n, w) => `${n} ${n === 1 ? w : `${w}s`}`;

// ---- applying what the user accepted ----

export const KEEPOUT_KIND = { "ceiling-fan": "fan" };
const keepoutKind = (kind) => KEEPOUT_KIND[kind] ?? kind.replace(/-/g, " ");
// A keep-out k for the hazard (or the keep-out) h: the same kind (as hazardKeepout names it) within 0.8 m or k's reach.
const sameKeepout = (k, h) => Number.isFinite(k.x) && (k.kind === keepoutKind(h.kind) || k.kind === h.kind) && Math.hypot(k.x - h.x, k.y - h.y) < Math.max(0.8, k.r ?? 0);
const sameLandmark = (m, l) => nameKey(m.name) === nameKey(l.name) && Math.hypot(m.x - l.x, m.y - l.y) < 1.0;
const MARGIN = { "ceiling-fan": 0.4, "hanging-lamp": 0.3, plant: 0.25, curtain: 0.25, cable: 0.2, glass: 0.3, mirror: 0.3, "pet-bowl": 0.15, stairs: 0.3, candle: 0.4, "open-flame": 0.5, "shelf-edge": 0.2, other: 0.25 };

// A hazard as a keep-out (source "claude"): fans to the floor (downwash), hanging lamps from 0.4 m below them, flames with
// 1 m of heat above, floor things (cables, bowls) up to 0.5 m; glass, mirrors, stairs and the rest at every height. One
// with a line (glass or a mirror on a wall) is a panel along it, its margin to either side (x, y, r: its middle and reach).
export function hazardKeepout(h, floorZ = 0) {
  const k = { kind: keepoutKind(h.kind), x: r2(h.x), y: r2(h.y), r: r2(Math.max(h.kind === "ceiling-fan" ? 0.6 : 0.15, h.r + (MARGIN[h.kind] ?? 0.25))), source: "claude", room: h.room, why: h.why };
  if (h.line) {
    const m = MARGIN[h.kind] ?? 0.25, [[ax, ay], [bx, by]] = h.line, L = Math.hypot(bx - ax, by - ay) || 1, u = [(bx - ax) / L, (by - ay) / L], n = [-u[1], u[0]];
    const A = [ax - m * u[0], ay - m * u[1]], B = [bx + m * u[0], by + m * u[1]];
    Object.assign(k, { x: r2((ax + bx) / 2), y: r2((ay + by) / 2), r: r2(L / 2 + m), polygon: [[A, m], [B, m], [B, -m], [A, -m]].map(([p, t]) => [r2(p[0] + t * n[0]), r2(p[1] + t * n[1])]) });
  }
  const z = (zMin, zMax) => Object.assign(k, { zMin: r2(zMin), zMax: r2(zMax) });
  switch (h.kind) {
    case "ceiling-fan": return z(floorZ - 0.1, h.zMax + 0.3);
    case "hanging-lamp": return z(h.zMin - 0.4, h.zMax + 0.3);
    case "candle": case "open-flame": return z(floorZ - 0.1, h.zMax + 1.0);
    case "plant": case "curtain": case "shelf-edge": return z(Math.min(floorZ - 0.1, h.zMin - 0.2), h.zMax + 0.3);
    case "cable": case "pet-bowl": return z(floorZ - 0.1, Math.max(floorZ + 0.5, h.zMax + 0.2));
    default: return k;
  }
}

// accept: { rooms: [room ids to rename], landmarks: [indexes into res.landmarks], hazards: [indexes into res.hazards] }.
// Rooms the user named themselves keep their names. Earlier Claude landmarks and keep-outs of the same thing (what `known`
// matches: within 1 m, or the keep-out's reach) are replaced.
export function applySurvey(house, res, accept = {}, { map = null } = {}) {
  const done = { rooms: 0, landmarks: 0, keepouts: 0 };
  for (const id of accept.rooms ?? []) {
    const r = res.rooms.find((q) => q.id === id), room = house.rooms.find((q) => q.id === id);
    if (!r?.suggestedName || !room || room.nameSource === "user") continue;
    Object.assign(room, { name: r.suggestedName, nameSource: "claude", kind: r.kind });
    done.rooms++;
  }
  for (const i of accept.landmarks ?? []) {
    const l = res.landmarks[i];
    if (!l) continue;
    house.landmarks = [...(house.landmarks ?? []).filter((m) => !(m.source === "claude" && sameLandmark(m, l))),
      { name: l.name, room: l.room, x: l.x, y: l.y, z: l.z, source: "claude", confidence: l.confidence }];
    done.landmarks++;
  }
  for (const i of accept.hazards ?? []) {
    const h = res.hazards[i];
    if (!h) continue;
    const k = hazardKeepout(h, map?.floorAt?.(h.x, h.y) ?? house.rooms.find((r) => r.id === h.room)?.floorZ ?? 0);
    house.keepouts = [...(house.keepouts ?? []).filter((o) => !(o.source === "claude" && sameKeepout(o, h))), k];
    done.keepouts++;
  }
  return done;
}

export { roomContext, viewText };
