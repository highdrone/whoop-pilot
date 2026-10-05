// The 3D map's heavy lifting off the main thread (voxels.js inWorker): { id, op: "edt", mask, nx, ny, nz, res, cap } ->
// { id, dist } (VoxelMap.refresh), { id, op: "carve", grid, lo, st, flags, faint, cams, o } -> { id, progress } messages,
// then { id, lo, st, flags, faint, visible } (buildVoxels: the capture cameras' rays).
import { edt3, VoxelMap, carveFromCameras } from "./voxels.js";

self.onmessage = ({ data: m }) => {
  try {
    if (m.op === "edt") {
      const dist = edt3(m.mask, m.nx, m.ny, m.nz, m.res, m.cap);
      self.postMessage({ id: m.id, dist }, [dist.buffer]);
    } else if (m.op === "carve") {
      const vox = Object.assign(new VoxelMap(m.grid), { lo: m.lo, st: m.st, flags: m.flags });
      const visible = carveFromCameras(vox, m.cams, m.o, m.faint, (done) => self.postMessage({ id: m.id, progress: done }));
      self.postMessage({ id: m.id, lo: vox.lo, st: vox.st, flags: vox.flags, faint: m.faint, visible },
        [vox.lo.buffer, vox.st.buffer, vox.flags.buffer, m.faint.buffer, visible.buffer]);
    } else throw new Error(`unknown op ${m.op}`);
  } catch (e) {
    self.postMessage({ id: m.id, error: String(e?.stack || e) });
  }
};
