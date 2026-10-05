// The house's splat digital twin, page side (docs/HOME-DRONE.md). All rendering runs in twin-worker.js.
//   const twin = await Twin.create({ splat, house });          // splat: Blob | ArrayBuffer | URL; house.frame = { f, Yf }
//                                                               // (or frame: houseFrame(...) from house/frames.js)
//   const bmp = await twin.render(poseH, { width: 640, height: 480 });   // drone camera (DRONE_LENS by default)
//   await twin.setActors([{ id: "cat", kind: "cat", x, y, z, yaw, pose: "walk" }]);
//   const gt = await twin.oracle(poseH);    // [{ id, kind, box, visibleBox, visibleFraction, pixels, visiblePixels }]
//   await twin.setProps([{ id: "door1", kind: "panel", x, y, z, w: 0.8, h: 2, yaw, color: 0xd8d0c0 }]);   // not in the splat
//   const depths = await twin.depthBatch(poses, { width: 96, height: 96, lens: { model: "pinhole", hfovDeg: 90 } });
// Poses are H-frame drone body poses { x, y, z, yaw, pitch, roll } (sim/drone.js convention); the lens carries the
// camera uptilt. A view is { width, height, lens?, ss?, sync?, actors?, props?, visibleT?, minCoverage? }: lens replaces
// the current lens for that call (a lens with a model takes its missing fields from that model's defaults, so
// { model: "pinhole", hfovDeg: 90 } has no uptilt; one without a model adjusts the current lens), ss 2 supersamples a pinhole view, sync
// false skips waiting for the GPU (pipelining; timings then undercount), actors false leaves the actors out of
// render/pixels/depth, props false the props (props.js: closed doors, new obstacles) out of those and the oracle (the
// expected view of the house as captured), visibleT is the oracle's visibility threshold (0.1), minCoverage the splat
// opacity a depth pixel needs (0.05; NaN below).
// The actors are code-built figures (a person, a brown tabby cat, a black and tan shepherd dog; actors.js): exact ground
// truth for the oracle, depth and occlusion, and only partly a stand-in for real ones in front of the detector. RF-DETR-N
// finds them as their kind at >= 0.4 in 58 of 60 person views, 54 of 80 cat and 41 of 80 dog views (standing and
// walking pets mostly; sitting dogs read as teddy bears, lying and rear views of pets are hit and miss).
// tools/twin-check.html?det=...&detGrid=1 measures it.
// One worker serves everyone, by priority (vision/budget.js): a view's prio (PRIO's names) puts its job ahead of lower
// ones waiting; state changes (lens, actors, props) go first. pixelsDepth() renders the colour picture and the expected
// depth from one placement and one GPU wait (localization and live depth want both). pending(prio) counts this page's
// jobs waiting or running at that priority or above (a 3D view keeps one render of its own in flight, at "view").
import { PRIO } from "./queue.js";
export { PRIO };
export { DRONE_LENS, pinholeLens, resolveLens, intrinsics, project, unproject, fieldOfView } from "./lens.js";
export { poseFromCapture, droneCamera } from "./pose.js";

// without a view.prio: render() is a picture for people (the 3D view), pixels() and depth() evidence or the survey
const DEFAULT_PRIO = { render: "view", pixels: "evidence", depth: "evidence", oracle: "camera", depthBatch: "build", pair: "evidence" };

export class Twin {
  #worker;
  #seq = 0;
  #pending = new Map();
  ordered = true; // false: every job first come, first served (GPU.enabled false: tools/loc-check.html?only=bench&budget=0)
  stats = {};
  info = {};

  constructor(worker) {
    this.#worker = worker;
    worker.onmessage = ({ data: { id, result, error, stats } }) => {
      const p = this.#pending.get(id);
      this.#pending.delete(id);
      if (stats) this.stats = stats;
      if (error) p?.reject(new Error(error)); else p?.resolve(result);
    };
    worker.onerror = (e) => this.#failAll(new Error(e.message || "twin worker failed"));
  }

  // options: resortDist (m moved before re-sorting, 0.1), quality (atlas px per lens px, 1), minOpacity (prune
  // fainter splats, 0 = keep all; 0.05 keeps 75% and is ~25% faster).
  static async create({ splat, house, frame = house?.frame, lens, workerUrl = new URL("./twin-worker.js", import.meta.url), ...options }) {
    if (!frame) throw new Error("Twin.create needs house.frame { f, Yf }");
    const twin = new Twin(new Worker(workerUrl, { type: "module" }));
    const src = typeof splat === "string" || splat instanceof URL ? new URL(splat, location.href).href : splat;
    try {
      twin.info = await twin.#call("init", { splat: src, frame: { f: frame.f, Yf: frame.Yf }, lens, ...options }, src instanceof ArrayBuffer ? [src] : []);
    } catch (e) {
      twin.#worker.terminate(); // it already holds a WebGL context
      twin.#worker = null;
      throw e;
    }
    return twin;
  }

  render(pose, view = {}) { return this.#call("render", { pose, view }); }            // ImageBitmap
  // { pixels: { width, height, data }, depth: Float32Array } at the same pose: depthView (default view) for the depth.
  pixelsDepth(pose, view = {}, depthView = view) { return this.#call("pair", { pose, view, depthView }); }
  pixels(pose, view = {}) { return this.#call("pixels", { pose, view }); }            // { width, height, data: RGBA top-down }
  depth(pose, view = {}) { return this.#call("depth", { pose, view }); }              // Float32Array, metres along each ray, actors included
  oracle(pose, view = {}) { return this.#call("oracle", { pose, view }); }
  depthBatch(poses, view = {}) { return this.#call("depthBatch", { poses, view }); }  // [Float32Array], one per pose, as depth()
  setLens(lens) { return this.#call("setLens", lens, [], "state"); }
  setActors(actors) { return this.#call("setActors", actors, [], "state"); }
  setProps(props) { return this.#call("setProps", props, [], "state"); }   // [{ id, kind: "box"|"panel", x, y, z, w, d, h, yaw, color }] -> count
  setOptions(options) { return this.#call("setOptions", options, [], "state"); }

  // This page's jobs waiting or running at priority prio (a PRIO name) or above; no prio: all of them.
  pending(prio) {
    const max = PRIO[prio] ?? Infinity;
    let n = 0;
    for (const p of this.#pending.values()) if (p.prio <= max) n++;
    return n;
  }
  get busy() {
    return this.#pending.size > 0;
  }

  async dispose() {
    if (!this.#worker) return;
    await this.#call("dispose").catch(() => {});
    this.#worker.terminate();
    this.#worker = null;
    this.#failAll(new Error("twin disposed"));
  }

  #call(op, args, transfer = [], prioName = args?.view?.prio ?? DEFAULT_PRIO[op] ?? "state") {
    if (!this.#worker) return Promise.reject(new Error("twin disposed"));
    const prio = !this.ordered && prioName !== "state" ? PRIO.evidence : PRIO[prioName] ?? PRIO.evidence;
    return new Promise((resolve, reject) => {
      const id = ++this.#seq;
      this.#pending.set(id, { resolve, reject, prio });
      this.#worker.postMessage({ id, op, args, prio }, transfer);
    });
  }

  #failAll(err) {
    for (const p of this.#pending.values()) p.reject(err);
    this.#pending.clear();
  }
}
