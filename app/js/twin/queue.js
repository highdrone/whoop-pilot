// The twin worker's job order (vision/budget.js): state changes (lens, actors, props) first, then localization, live
// depth, the simulator's camera, evidence pictures, the 3D view, map and database building; among equals the oldest. The
// simulator's camera under its frame rate ("cameraBehind") goes between localization and depth: both work on its frames.
// A job moves up one level for every AGE_MS it has waited, so a busy flight delays a 3D view but never drops it.
export const PRIO = { state: -1, loc: 0, cameraBehind: 0.5, depth: 1, camera: 2, evidence: 3, view: 4, build: 5 };
export const AGE_MS = 1000;

// Takes the next job out of list ([{ prio, at }]) at time t, or null.
export function nextJob(list, t) {
  let best = -1, bv = Infinity;
  for (let i = 0; i < list.length; i++) {
    const j = list[i], p = j.prio ?? PRIO.evidence, v = p - (p >= 0 ? (t - j.at) / AGE_MS : 0);
    if (v < bv) [best, bv] = [i, v];
  }
  return best < 0 ? null : list.splice(best, 1)[0];
}

// What a job's result hands over instead of copying (postMessage's transfer list): every ImageBitmap and typed array's
// buffer in it, at any depth, each once (one listed twice is a DataCloneError). Only those count: the oracle's boxes
// carry a `pixels` count, pair()'s { pixels: { data }, depth } the arrays.
export function transferOf(result) {
  const out = new Set(), walk = (v) => {
    if (typeof ImageBitmap !== "undefined" && v instanceof ImageBitmap) out.add(v);
    else if (ArrayBuffer.isView(v)) { if (v.buffer instanceof ArrayBuffer) out.add(v.buffer); }
    else if (v && typeof v === "object") for (const w of Array.isArray(v) ? v : Object.values(v)) walk(w);
  };
  walk(result);
  return [...out];
}
