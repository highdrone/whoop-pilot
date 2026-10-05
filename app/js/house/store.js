// Houses kept in the origin-private file system (OPFS): /houses/<id>/ holds house.json, the splat (its name in
// house.splatFile: splat.spz, .splat or .ply), splat-centres.bin when the capture had one, orthophoto.png,
// occupancy.bin (the HomeMap cache), voxels.bin (the 3D map, voxels.js, with what flights added) and calib.json.
// The origin is fixed (http://localhost:8790), since another port is another origin with its own, empty OPFS.
// root: an OPFS directory (the default) or memoryDir() in Node tests; both are FileSystemDirectoryHandle-shaped.
// User edits carry source: "user" (the home pad, keep-outs, landmarks) or nameSource: "user" (a renamed room), so
// importing the same capture again (same id) keeps them.

import { houseFrame } from "./frames.js";
import { HomeMap } from "./homemap.js";
import { CENTRES, geometryH } from "./import.js";
import { VoxelMap, voxelKey } from "./voxels.js";

const enc = new TextEncoder(), dec = new TextDecoder();

export async function openStore(root) {
  let persisted = false;
  if (!root) {
    root = await navigator.storage.getDirectory();
    persisted = (await navigator.storage.persisted()) || (await navigator.storage.persist()); // else Chrome may evict it under storage pressure
  }
  const houses = await root.getDirectoryHandle("houses", { create: true });
  const dir = (id, create = false) => houses.getDirectoryHandle(id, { create });
  const missing = (e) => e?.name === "NotFoundError" || e?.name === "TypeMismatchError";

  async function readFile(id, name) {
    try {
      return await (await (await (await dir(id)).getFileHandle(name)).getFile()).arrayBuffer();
    } catch (e) {
      if (missing(e)) return null;
      throw e;
    }
  }
  async function writeFile(id, name, data) {
    const w = await (await (await dir(id, true)).getFileHandle(name, { create: true })).createWritable();
    const raw = data instanceof ArrayBuffer || ArrayBuffer.isView(data);
    await w.write(typeof data === "string" ? enc.encode(data) : raw ? data : enc.encode(JSON.stringify(data)));
    await w.close();
  }
  async function loadHouse(id) {
    const b = await readFile(id, "house.json");
    return b && JSON.parse(dec.decode(b));
  }

  return {
    persisted,
    readFile,
    writeFile,
    loadHouse,
    async listHouses() {
      const out = [];
      for await (const [id, h] of houses.entries()) {
        if (h.kind !== "directory") continue;
        const house = await loadHouse(id).catch(() => null);
        if (house) out.push({ id, name: house.name ?? id, kind: house.source?.kind, rooms: house.rooms?.length ?? 0, saved: house.saved ?? 0 });
      }
      return out.sort((a, b) => b.saved - a.saved);
    },
    // files: { "splat.spz": bytes, "orthophoto.png": bytes, "occupancy.bin": bytes, "calib.json": {...}, ... }
    async saveHouse(house, files = {}) {
      house.saved = Date.now();
      for (const [name, data] of Object.entries(files)) if (data != null) await writeFile(house.id, name, data);
      await writeFile(house.id, "house.json", JSON.stringify(house)); // last, so a listed house has its files
      return house.id;
    },
    // An importCapture() result: the house (with the user's edits to an earlier import of it), the splat under its
    // own name, the splat centres, the orthophoto and the map's cache.
    async saveImport({ house, splat, centres, orthophoto, map }) {
      const before = await loadHouse(house.id).catch(() => null);
      if (before && keepUserEdits(house, before)) map?.finalize(); // the user's keep-outs into the map too
      house.splatFile = splat?.name ?? null;
      return this.saveHouse(house, {
        ...(splat && { [splat.name]: splat.bytes }),
        ...(centres && { [CENTRES]: centres.bytes }),
        ...(orthophoto && { "orthophoto.png": orthophoto.bytes }),
        ...(map && { "occupancy.bin": map.occupancyBytes() }),
      });
    },
    // The house's HomeMap from its occupancy cache, or when that is missing or made for another grid, from the
    // stored splat (then the cache is written again). stale: true only when no splat is stored either: the map then
    // has walls and keep-outs but no furniture.
    async loadMap(house, options) {
      const map = new HomeMap(house, options), cache = await readFile(house.id, "occupancy.bin");
      if (cache && map.loadOccupancy(cache)) return Object.assign(map.finalize(), { stale: false, rebuilt: false });
      const centres = await readFile(house.id, CENTRES), name = centres ? CENTRES : house.splatFile;
      const bytes = centres ?? (name && (await readFile(house.id, name)));
      if (!bytes) return Object.assign(map.finalize(), { stale: true, rebuilt: false });
      const g = await geometryH(bytes, name, houseFrame({ f: house.frame.f, Yf: house.frame.Yf }));
      map.addSplats(g.xyz, g.opacity, g.scale).finalize();
      await writeFile(house.id, "occupancy.bin", map.occupancyBytes());
      return Object.assign(map, { stale: false, rebuilt: true });
    },
    // The 3D map, flight evidence included. loadVoxels returns null when there is none or it was built for another version of
    // the house (rooms, frame or capture changed, or a new voxel format): build it again (buildVoxels) and save it.
    saveVoxels: (house, vox) => writeFile(house.id, "voxels.bin", vox.serialize()),
    async loadVoxels(house) {
      const bytes = await readFile(house.id, "voxels.bin");
      if (!bytes) return null;
      try {
        const vox = VoxelMap.load(bytes);
        return vox.key === voxelKey(house) ? vox : null;
      } catch {
        return null;
      }
    },
    async deleteHouse(id) {
      try {
        await houses.removeEntry(id, { recursive: true });
        return true;
      } catch (e) {
        if (missing(e)) return false;
        throw e;
      }
    },
  };
}

// Carries the user's edits from an earlier import of the same capture into a new one; returns how many.
function keepUserEdits(house, before) {
  const user = (x) => x?.source === "user";
  let kept = 0;
  if (user(before.home)) [house.home, kept] = [before.home, kept + 1];
  for (const key of ["keepouts", "landmarks"]) {
    const mine = (before[key] ?? []).filter(user);
    house[key].push(...mine);
    kept += mine.length;
  }
  for (const r of house.rooms) {
    const old = before.rooms?.find((q) => q.id === r.id && q.nameSource === "user");
    if (old) [r.name, r.nameSource, kept] = [old.name, "user", kept + 1];
  }
  return kept;
}

// In-memory stand-in for an OPFS directory, with just what the store uses.
export function memoryDir(name = "") {
  const items = new Map();
  const get = (n, create, make, kind) => {
    let e = items.get(n);
    if (!e && create) items.set(n, (e = make(n)));
    if (!e) throw new DOMException(`${n} not found`, "NotFoundError");
    if (e.kind !== kind) throw new DOMException(`${n} is not a ${kind}`, "TypeMismatchError");
    return e;
  };
  return {
    kind: "directory",
    name,
    getDirectoryHandle: async (n, { create = false } = {}) => get(n, create, memoryDir, "directory"),
    getFileHandle: async (n, { create = false } = {}) => get(n, create, memoryFile, "file"),
    async removeEntry(n) {
      if (!items.delete(n)) throw new DOMException(`${n} not found`, "NotFoundError");
    },
    async *entries() {
      yield* items.entries();
    },
    async *keys() {
      yield* items.keys();
    },
  };
}

function memoryFile(name) {
  let data = new Uint8Array(0);
  return {
    kind: "file",
    name,
    getFile: async () => new File([data], name),
    async createWritable() {
      const parts = [];
      return {
        write: async (d) => parts.push(d instanceof ArrayBuffer ? d.slice(0) : d.buffer.slice(d.byteOffset, d.byteOffset + d.byteLength)),
        close: async () => (data = new Uint8Array(await new Blob(parts).arrayBuffer())),
      };
    },
  };
}
