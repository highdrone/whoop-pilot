// replay.js's checks in a worker of their own: performance.now() here is the recording's clock while one runs, and the
// twin and the vision worker are this worker's, so nothing is shared with the app's flight. { op: "check" | "calibrate",
// id (a recording), houseId, options } -> { type: "progress", progress } ... then { type: "done", result } | { type: "error", error }.
import { loadRecording, recordingProblems, replayRecording, judgeReplay, calibrateFromRecording } from "./replay.js";
import { RelocDb } from "./splatloc.js";
import { openStore } from "../house/store.js";
import { Twin } from "../twin/twin.js";

const say = (progress) => self.postMessage({ type: "progress", progress });

async function run({ op, id, rec: given, houseId, options = {} }) {
  say({ text: "Reading the recording and the house…" });
  const store = await openStore(await navigator.storage.getDirectory()), house = await store.loadHouse(houseId);
  if (!house) throw new Error("the house isn't stored in this browser");
  const rec = given ?? (await loadRecording(id)), problems = recordingProblems(rec, rec.video.length);
  if (op === "check" && problems.some((p) => !/pad button|position/.test(p)))
    return { ok: false, verdict: "can't check", title: `This recording can't be checked: ${problems[0]}.`, problems, lines: [] };
  if (house.id !== rec.meta.house) throw new Error("this recording was made in another house");
  const map = await store.loadMap(house), vox = await store.loadVoxels(house).catch(() => null);
  if (vox) map.setVoxels?.(vox);
  const read = async (name) => { const b = await store.readFile(house.id, name); return b && JSON.parse(new TextDecoder().decode(b)); };
  const calib = options.calib !== undefined ? options.calib : (await read("calib.json")) ?? rec.meta.lens?.calib ?? null;
  const splat = house.splatFile && (await store.readFile(house.id, house.splatFile));
  if (!splat) throw new Error("the house's 3D scan isn't stored in this browser");
  say({ text: "Loading the 3D scan…" });
  const twin = await Twin.create({ splat, house });
  try {
    if (op === "calibrate") {
      const r = await calibrateFromRecording(rec, { twin, house, map, calib: options.fresh ? null : calib, onProgress: (p) => say({ ...p, text: { turn: `Picking views from the turn (${p.samples})`, fit: "Fitting the lens…", check: "Checking it on the pad…" }[p.phase] }) });
      return r;
    }
    const dbBytes = await store.readFile(house.id, "reloc.bin"), db = dbBytes && RelocDb.load(dbBytes);
    const out = await replayRecording(rec, { twin, house, map, db, calib, ...options, onProgress: (p) => say({ ...p, text: `Replaying the flight: ${Math.round(p.seconds)} of ${Math.round(p.of)} s` }) });
    return { ...judgeReplay(out), problems };
  } finally {
    await twin.dispose();
  }
}

self.onmessage = ({ data }) => run(data).then((result) => self.postMessage({ type: "done", result }), (e) => self.postMessage({ type: "error", error: String(e?.message || e) }));
