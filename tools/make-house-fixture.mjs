// Cuts tools/fixtures/house/ from a real SiteSpec capture (read-only): the plan files slimmed down (ceiling maps
// cropped to their rooms, measurements dropped), every 10th capture camera, and instead of the 76 MB splat a
// "splat centres" file (import.js, WPC1) of the most opaque surface splats. The layout mirrors a project folder,
// so importCapture reads the fixture like the real thing. thin-centres.bin adds every splat down to 5% opacity in
// a few boxes around thin things (the coffee table's base, a chair's legs, the stair railing; thin-centres.json)
// for the 3D map's tests: the browser builds from all of them, the fixture's main file keeps only the opaque ones.
// Usage: node make-house-fixture.mjs <SiteSpec project folder> [--keep 250000] [--thin-only]
import fs from "node:fs";
import path from "node:path";
import { readSplatPoints, writeCentres } from "../app/js/house/import.js";
import { houseFrame } from "../app/js/house/frames.js";

const src = process.argv[2] ?? process.env.HOUSE_CAPTURE;
if (!src) throw new Error("usage: node make-house-fixture.mjs <SiteSpec project folder> [--keep 250000] [--thin-only]");
const keep = Number(process.argv[process.argv.indexOf("--keep") + 1]) || 250000;
const out = path.join(import.meta.dirname, "fixtures", "house");
const json = (p) => JSON.parse(fs.readFileSync(path.join(src, p), "utf8"));
const write = (p, v) => {
  fs.mkdirSync(path.dirname(path.join(out, p)), { recursive: true });
  fs.writeFileSync(path.join(out, p), typeof v === "string" || ArrayBuffer.isView(v) ? v : v instanceof ArrayBuffer ? new Uint8Array(v) : JSON.stringify(v, null, 1));
  console.log(`  ${p}  ${(fs.statSync(path.join(out, p)).size / 1024).toFixed(0)} KB`);
};

function cropCeiling(m) {
  const cm = m.ceiling_map;
  if (!cm?.heights) return cm;
  const xs = m.outline.map((p) => p[0]), ys = m.outline.map((p) => p[1]);
  const i0 = Math.max(0, Math.floor((Math.min(...xs) - cm.x0) / cm.cell) - 1), i1 = Math.floor((Math.max(...xs) - cm.x0) / cm.cell) + 1;
  const j0 = Math.max(0, Math.floor((Math.min(...ys) - cm.y0) / cm.cell) - 1), j1 = Math.floor((Math.max(...ys) - cm.y0) / cm.cell) + 1;
  return { x0: cm.x0 + i0 * cm.cell, y0: cm.y0 + j0 * cm.cell, cell: cm.cell, heights: cm.heights.slice(j0, j1 + 1).map((r) => r.slice(i0, i1 + 1)) };
}
const slimModel = ({ source, ...m }) => ({ ...m, ceiling_map: cropCeiling(m), source: { outline: source?.outline, scale: source?.scale } });

// Boxes in H ([x0, y0, z0], [x1, y1, z1]) around thin things in the reference capture.
const THIN = [
  { name: "coffee table (X base, rim)", min: [2.63, 2.65, -0.1], max: [3.83, 3.85, 1.0] },
  { name: "chair (legs)", min: [1.77, 4.78, -0.1], max: [2.77, 5.78, 1.2] },
  { name: "stair railing (balusters)", min: [-1.05, 3.6, -0.1], max: [-0.7, 6.4, 1.3] },
];
async function thin() {
  const frame = houseFrame({ site: json("site.json"), roomsAuto: json("work/room/rooms.auto.json") });
  const name = ["splat.spz", "splat.splat"].find((n) => fs.existsSync(path.join(src, "outputs", n)));
  const pts = await readSplatPoints(fs.readFileSync(path.join(src, "outputs", name)), name, { minOpacity: 13, maxScale: 0.3 });
  const idx = [];
  for (let i = 0; i < pts.n; i++) {
    const h = frame.toH([pts.xyz[3 * i], pts.xyz[3 * i + 1], pts.xyz[3 * i + 2]]);
    if (THIN.some((b) => h.every((v, k) => v >= b.min[k] && v < b.max[k]))) idx.push(i);
  }
  const pick = { n: idx.length, xyz: new Float32Array(3 * idx.length), opacity: new Uint8Array(idx.length), scale: new Float32Array(idx.length) };
  idx.forEach((i, j) => {
    pick.xyz.set(pts.xyz.subarray(3 * i, 3 * i + 3), 3 * j);
    pick.opacity[j] = pts.opacity[i];
    pick.scale[j] = pts.scale[i];
  });
  write("outputs/thin-centres.bin", writeCentres(pick));
  write("outputs/thin-centres.json", { note: "every splat with opacity >= 13/255 and scale <= 0.3 m inside these boxes (H)", boxes: THIN, n: pick.n });
}
if (process.argv.includes("--thin-only")) {
  await thin();
  process.exit(0);
}

for (const p of ["outputs", "work", "site.json", "project.json"]) fs.rmSync(path.join(out, p), { recursive: true, force: true }); // check.html stays
console.log(`writing ${out}`);
const plans = path.join(src, "outputs", "plans");
const roomsFile = fs.readdirSync(plans).find((n) => n.endsWith("-rooms.json"));
const rooms = json(path.join("outputs", "plans", roomsFile));
write("outputs/plans/house-rooms.json", { ...rooms, rooms: rooms.rooms.map(({ measurements, confidence, ...r }) => ({ ...r, model: slimModel(r.model) })) });
const ortho = json(path.join("outputs", "plans", roomsFile.replace("-rooms.json", "-orthophoto.json")));
write("outputs/plans/house-orthophoto.json", ortho);
write("outputs/plans/manifest.json", { rooms: rooms.rooms.map((r) => ({ id: r.id, name: r.name })), files: { json: { name: "house-rooms.json" } }, orthophoto: ortho });
write("site.json", json("site.json"));
const project = json("project.json");
write("project.json", { id: project.id, name: project.name });
const auto = json("work/room/rooms.auto.json");
write("work/room/rooms.auto.json", { rooms: auto.rooms.map((r) => ({ id: r.id, model: slimModel(r.model) })) });
if (fs.existsSync(path.join(src, "work/room/ai_scene.json"))) {
  const ai = json("work/room/ai_scene.json");
  write("work/room/ai_scene.json", { version: ai.version, rooms: Object.fromEntries(Object.entries(ai.rooms).map(([id, r]) => [id, { room_id: r.room_id, name: r.name, room_type: r.room_type, summary: r.summary, fixtures: r.fixtures }])) });
}
if (fs.existsSync(path.join(src, "work/room/iphone.json"))) {
  const ip = json("work/room/iphone.json");
  write("work/room/iphone.json", { version: ip.version, captures: ip.captures.map((c) => ({ folder: c.folder, name: c.name, scale: c.scale, rooms: c.rooms.map((r) => ({ index: r.index, name: r.name, model: { floor_y: r.model?.floor_y }, fixtures: r.fixtures })) })) });
}
const scene = json("outputs/scene.json");
write("outputs/scene.json", { units: scene.units, up: scene.up, convention: scene.convention, cameras: scene.cameras.filter((_, i) => i % 10 === 0), tags: scene.tags, alignment: scene.alignment, splat: scene.splat });

// The most opaque surface splats (largest axis <= 0.2 m), in the capture frame W.
const splatName = ["splat.spz", "splat.splat"].find((n) => fs.existsSync(path.join(src, "outputs", n)));
const t0 = performance.now();
const pts = await readSplatPoints(fs.readFileSync(path.join(src, "outputs", splatName)), splatName, { maxScale: 0.2 });
const hist = new Uint32Array(256);
for (const a of pts.opacity) hist[a]++;
let cut = 255;
for (let n = hist[255]; cut > 0 && n + hist[cut - 1] <= keep; ) n += hist[--cut];
const idx = [];
for (let i = 0; i < pts.n; i++) if (pts.opacity[i] >= cut) idx.push(i);
const pick = { n: idx.length, xyz: new Float32Array(3 * idx.length), opacity: new Uint8Array(idx.length), scale: new Float32Array(idx.length) };
idx.forEach((i, j) => {
  pick.xyz.set(pts.xyz.subarray(3 * i, 3 * i + 3), 3 * j);
  pick.opacity[j] = pts.opacity[i];
  pick.scale[j] = pts.scale[i];
});
write("outputs/splat-centres.bin", writeCentres(pick));
console.log(`${splatName}: ${pts.total} splats, ${pts.n} with scale <= 0.2 m, kept ${pick.n} with opacity >= ${cut}/255 (${(performance.now() - t0).toFixed(0)} ms)`);
await thin();
