// The splat twin's geometry (app/js/twin): lens models and the view layout the worker warps from, the W/H/three.js
// frames and drone poses from docs/HOME-DRONE.md, the procedural actors' sizes and placement, and the props (boxes and
// door panels that are not in the splat) and the page side of setProps / depthBatch. The GPU side (Spark renders, PSNR,
// oracle, props in the image and depth, depthBatch timing) is checked in Chrome by tools/twin-check.html.
import assert from "node:assert/strict";
import { DRONE_LENS, pinholeLens, resolveLens, intrinsics, project, unproject, layout, fieldOfView, FACES, DEG } from "../app/js/twin/lens.js";
import { droneCamera, poseFromCapture, mul3 } from "../app/js/twin/pose.js";
import { rootMatrix, hToThree, houseFrame } from "../app/js/house/frames.js";
import { Actors } from "../app/js/twin/actors.js";
import { Props, propSize } from "../app/js/twin/props.js";
import * as THREE from "../app/vendor/three/build/three.module.js";

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ""} ${a} vs ${b} (tol ${tol})`);
const nearVec = (a, b, tol, msg) => a.forEach((v, i) => near(v, b[i], tol, `${msg ?? ""}[${i}]`));
const col = (R, j) => [R[0][j], R[1][j], R[2][j]];
const rot = (ax, t) => { const c = Math.cos(t), s = Math.sin(t); return ax === "x" ? [[1, 0, 0], [0, c, -s], [0, s, c]] : ax === "y" ? [[c, 0, s], [0, 1, 0], [-s, 0, c]] : [[c, -s, 0], [s, c, 0], [0, 0, 1]]; };
let seed = 7;
const rand = () => ((seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296);

// Ray a layout entry samples: inverts the LUT's atlas mapping back to a view's tangent coordinates.
function lutRay(lay, x, j) {
  const o = ((lay.height - 1 - j) * lay.width + x) * 4, u = lay.lut[o] * lay.atlasW, v = lay.lut[o + 1] * lay.atlasH;
  const view = lay.views.find((w) => u >= w.x0 && u <= w.x0 + w.w);
  const a = view.aMin + ((u - view.x0) / view.w) * (view.aMax - view.aMin), b = view.bMax - (v / view.h) * (view.bMax - view.bMin);
  const d = [0, 1, 2].map((i) => view.f[i] + a * view.r[i] + b * view.d[i]), n = Math.hypot(...d);
  return { d: d.map((c) => c / n), rangeFactor: lay.lut[o + 2], n, valid: lay.lut[o + 3] };
}

const tests = {
  "O4 lens: 159 deg diagonal equidistant is 127 x 95 deg at 4:3"() {
    const fov = fieldOfView(DRONE_LENS, 640, 480);
    near(fov.diag, 159, 0.01, "diag");
    near(fov.h, 127.2, 0.1, "h");
    near(fov.v, 95.4, 0.1, "v");
    near(fieldOfView(pinholeLens(90), 320, 240).h, 90, 1e-9, "pinhole h");
  },

  "16:9 output is the centre crop of the 4:3 O4 lens (same focal length, 127 x 72 deg)"() {
    const fov = fieldOfView(DRONE_LENS, 320, 180), K = intrinsics(DRONE_LENS, 320, 180);
    near(fov.h, 127.2, 0.1, "h");
    near(fov.v, 71.5, 0.1, "v");
    near(K.fx, intrinsics(DRONE_LENS, 320, 240).fx, 1e-9, "fx");
    near(fieldOfView(DRONE_LENS, 1280, 720).h, fieldOfView(DRONE_LENS, 640, 480).h, 1e-9, "720p");
    near(fieldOfView({ ...DRONE_LENS, aspect: null }, 640, 360).diag, 159, 1e-6, "aspect null fits the diagonal to the output");
    const lay = layout(DRONE_LENS, 320, 180);
    for (let i = 3; i < lay.lut.length; i += 4) assert.equal(lay.lut[i], 1);
  },

  "a lens with a model takes that model's defaults (no inherited uptilt); one without a model adjusts the current lens"() {
    assert.deepEqual(resolveLens({ model: "pinhole", hfovDeg: 90 }, DRONE_LENS), pinholeLens(90));
    assert.deepEqual(resolveLens({ model: "pinhole", hfovDeg: 70 }, { ...DRONE_LENS, uptiltDeg: 35 }).uptiltDeg, 0);
    assert.deepEqual(resolveLens({ model: "equidistant" }, pinholeLens(60)), { ...DRONE_LENS });
    assert.deepEqual(resolveLens({ uptiltDeg: 0 }, DRONE_LENS), { ...DRONE_LENS, uptiltDeg: 0 });
    assert.deepEqual(resolveLens({ hfovDeg: 60 }, pinholeLens(90)), pinholeLens(60));
    assert.deepEqual(resolveLens(undefined, pinholeLens(80)), pinholeLens(80));
    const a = intrinsics({ model: "pinhole", hfovDeg: 90 }, 320, 240), b = intrinsics(pinholeLens(90), 320, 240);
    assert.deepEqual(a, b);
  },

  async "Twin.create rejects and terminates its worker when init fails"() {
    const made = [];
    globalThis.Worker = class {
      constructor(url, o) { this.url = String(url); this.type = o.type; this.terminated = 0; made.push(this); }
      postMessage({ id, op }) { setTimeout(() => this.onmessage({ data: op === "init" ? { id, error: "Error: splat.spz: HTTP 404" } : { id, result: true } })); }
      terminate() { this.terminated++; }
    };
    try {
      const { Twin } = await import("../app/js/twin/twin.js");
      await assert.rejects(Twin.create({ splat: new ArrayBuffer(8), frame: { f: 1, Yf: 0 } }), /HTTP 404/);
      assert.equal(made.length, 1);
      assert.equal(made[0].terminated, 1);
      assert.match(made[0].url, /twin-worker\.js$/);
      await assert.rejects(Twin.create({ splat: new ArrayBuffer(8) }), /house.frame/);
      assert.equal(made.length, 1, "no worker without a frame");
    } finally { delete globalThis.Worker; }
  },

  "project and unproject invert each other (equidistant, Kannala-Brandt, pinhole)"() {
    const lenses = [DRONE_LENS, { ...DRONE_LENS, k1: 0.03, k2: -0.01, k3: 0.002, k4: -0.0004, cx: 0.51, cy: 0.48 }, pinholeLens(90, { cx: 0.47 })];
    for (const lens of lenses) {
      const K = intrinsics(lens, 640, 480);
      for (let i = 0; i < 200; i++) {
        const u = rand() * 640, v = rand() * 480, d = unproject(K, u, v);
        near(Math.hypot(...d), 1, 1e-12, "unit ray");
        nearVec(project(K, d), [u, v], 1e-6, `${lens.model} round trip`);
      }
    }
  },

  "drone layout: five cube faces (no back face), every pixel covered, LUT rays exact"() {
    const lay = layout(DRONE_LENS, 640, 480);
    assert.deepEqual(lay.views.map((v) => v.name).sort(), ["down", "front", "left", "right", "up"]);
    assert.ok(lay.atlasW <= 4096 && lay.atlasH <= 1024, `atlas ${lay.atlasW}x${lay.atlasH}`);
    let invalid = 0;
    for (let i = 3; i < lay.lut.length; i += 4) if (!lay.lut[i]) invalid++;
    assert.equal(invalid, 0, "all 4:3 pixels are inside the 159 deg lens");
    const K = intrinsics(DRONE_LENS, 640, 480);
    for (let k = 0; k < 400; k++) {
      const x = Math.floor(rand() * 640), j = Math.floor(rand() * 480), want = unproject(K, x + 0.5, j + 0.5), got = lutRay(lay, x, j);
      near(Math.acos(Math.min(1, want[0] * got.d[0] + want[1] * got.d[1] + want[2] * got.d[2])) / DEG, 0, 1e-3, `ray at ${x},${j}`);
      near(got.rangeFactor, got.n, 1e-4, "range factor = distance per unit view depth");
    }
    for (const v of lay.views) assert.ok(v.x0 + v.w <= lay.atlasW && v.h <= lay.atlasH, `view ${v.name} inside the atlas`);
  },

  "pinhole layout: one view, 2x supersampled texels average exact 2x2 blocks"() {
    const lay = layout(pinholeLens(90), 320, 240, { ss: 2 });
    assert.equal(lay.views.length, 1);
    assert.deepEqual([lay.atlasW, lay.atlasH], [640, 480]);
    for (const [x, j] of [[0, 0], [319, 239], [100, 37]]) {
      const o = ((239 - j) * 320 + x) * 4;
      near(lay.lut[o] * 640, 2 * x + 1, 1e-3, "u on a texel corner");
      near(lay.lut[o + 1] * 480, 480 - (2 * j + 1), 1e-3, "v on a texel corner");
    }
  },

  "the splat root matrix (house/frames.js, column-major as the worker loads it) maps W to H in three.js"() {
    const hf = houseFrame({ f: 0.9879, Yf: 0.2483 }), M = new THREE.Matrix4().fromArray(rootMatrix(hf.f, hf.Yf));
    for (const W of [[1.2, -0.7, 3.4], [0, 0, 0], [-1.8, 0.25, 6.5]]) {
      const v = new THREE.Vector3(...W).applyMatrix4(M);
      nearVec([v.x, v.y, v.z], hToThree(hf.toH(W)), 1e-12, "root");
    }
    assert.ok(Math.abs(M.determinant() - hf.f ** 3) < 1e-12, "a rotation (no mirror) scaled by f");
  },

  "drone camera axes: yaw CCW from +x, uptilt raises the view, +pitch noses down, +roll right side down"() {
    const P = { x: 1, y: 2, z: 1, yaw: 0, pitch: 0, roll: 0 };
    let c = droneCamera(P, 0);
    nearVec(col(c.R, 2), [1, 0, 0], 1e-12, "forward +x");
    nearVec(col(c.R, 0), [0, -1, 0], 1e-12, "image right = -y (right of the nose)");
    nearVec(col(c.R, 1), [0, 0, -1], 1e-12, "image down = -z");
    nearVec(col(droneCamera({ ...P, yaw: Math.PI / 2 }, 0).R, 2), [0, 1, 0], 1e-12, "yaw 90 looks +y");
    near(Math.asin(col(droneCamera(P, 20).R, 2)[2]) / DEG, 20, 1e-9, "uptilt elevation");
    near(Math.asin(col(droneCamera({ ...P, pitch: 5 * DEG }, 20).R, 2)[2]) / DEG, 15, 1e-9, "pitch nose down");
    assert.ok(col(droneCamera({ ...P, roll: 10 * DEG }, 0).R, 0)[2] < -0.1, "roll right side down tips image right downward");
    c = droneCamera(P, 0);
    nearVec(c.pThree, [1, 1, -2], 1e-12, "three.js position");
    nearVec(col(c.RThree, 2), [1, 0, 0], 1e-12, "three.js forward");
  },

  "a capture camera (world_from_cam, W) converts to an H pose that renders the same view"() {
    const frame = houseFrame({ f: 0.9879, Yf: 0.2483 }), A = [[1, 0, 0], [0, 0, 1], [0, -1, 0]];
    for (let i = 0; i < 50; i++) {
      const R = mul3(mul3(rot("y", rand() * 6.28), rot("x", (rand() - 0.5) * 2.5)), rot("z", (rand() - 0.5) * 0.6));
      const cam = { rotation: R, position: [rand() * 4, -rand() * 2, rand() * 5] }, pose = poseFromCapture(cam, frame), c = droneCamera(pose, 0);
      nearVec(c.p, frame.toH(cam.position), 1e-12, "centre");
      const want = mul3(A, R);
      for (let r = 0; r < 3; r++) nearVec(c.R[r], want[r], 1e-9, `rotation row ${r}`);
    }
  },

  "actors: sizes, ground contact, poses and placement in three.js"() {
    const actors = new Actors();
    const box = (id) => new THREE.Box3().setFromObject(actors.items.get(id).rig.root, true);
    const put = (spec) => actors.set([{ id: "a", x: 0, y: 0, z: 0, yaw: 0, ...spec }]);
    const cases = [
      ["person", "stand", 1.65, 1.8], ["person", "walk", 1.6, 1.8], ["person", "sit", 1.15, 1.35], ["person", "lie", 0.15, 0.45],
      ["cat", "stand", 0.45, 0.6], ["cat", "walk", 0.45, 0.6], ["cat", "sit", 0.3, 0.45], ["cat", "lie", 0.18, 0.3], // stand, walk: tail up
      ["dog", "stand", 0.65, 0.8], ["dog", "walk", 0.65, 0.8], ["dog", "sit", 0.65, 0.85], ["dog", "lie", 0.4, 0.55],
    ];
    const lowest = (o) => { o.updateMatrixWorld(true); const b = new THREE.Box3(); o.traverse((m) => m.isMesh && b.union(new THREE.Box3().setFromObject(m, true))); return b.min.y; };
    for (const [kind, pose, lo, hi] of cases) {
      put({ kind, pose });
      const b = box("a"), rig = actors.items.get("a").rig;
      assert.ok(b.max.y >= lo && b.max.y <= hi, `${kind} ${pose} height ${b.max.y.toFixed(3)}`);
      near(b.min.y, 0, 1e-6, `${kind} ${pose} rests on the floor`);
      if (kind !== "person" && pose !== "lie") {
        const down = rig.legs.filter((leg) => lowest(leg.at(-1)) < 0.015).length;
        assert.ok(down >= (pose === "walk" ? 2 : 4), `${kind} ${pose}: ${down} paws on the floor`);
      }
      if (kind === "cat" && pose === "sit") near(lowest(rig.trunk.children.find((m) => m.isMesh)), 0, 0.01, "a sitting cat sits on its haunches");
    }
    put({ kind: "person", pose: "stand" });
    const L = box("a").max.x - box("a").min.x;
    actors.set([{ id: "a", kind: "person", x: 2, y: 3, z: 0.25, yaw: Math.PI / 2, pose: "lie" }]);
    const b = box("a"), c = b.getCenter(new THREE.Vector3());
    near(b.min.y, 0.25, 1e-6, "rests on its floor z");
    assert.ok(b.max.z - b.min.z > 1.5 && b.max.x - b.min.x < 0.7, "lying along H y after yaw 90 deg (three.js -Z)");
    near(c.x, 2, 0.3, "x"); near(-c.z, 3, 1.0, "y");
    assert.ok(L < 0.7, "standing person is shallow front to back");
    for (const kind of ["person", "cat", "dog"]) { // one mesh per joint, vertex colours and fur pattern strengths
      put({ kind });
      const { meshes } = actors.items.get("a");
      assert.ok(meshes.length <= 17, `${kind}: ${meshes.length} draw calls`);
      assert.ok(meshes.every((m) => m.geometry.attributes.color && m.geometry.attributes.pat && m.material === meshes[0].material), `${kind}: baked parts share one material`);
    }
    actors.set([{ id: "p", kind: "person", x: 0, y: 0 }, { id: "c", kind: "cat", x: 1, y: 0 }]);
    assert.deepEqual(actors.list().map((a) => [a.spec.id, a.index, a.idMat.uniforms.id.value]), [["p", 1, 1], ["c", 2, 2]]);
    actors.set([{ id: "c", kind: "dog", x: 1, y: 0 }]);
    assert.equal(actors.items.size, 1);
    assert.equal(actors.root.children.length, 1);
    assert.equal(actors.items.get("c").spec.kind, "dog");
    actors.use("id");
    assert.ok(actors.items.get("c").meshes.every((m) => m.material.uniforms?.id.value === 1), "id materials");
    actors.use("shade");
    assert.ok(actors.items.get("c").meshes.every((m) => m.material.isMeshLambertMaterial), "shaded materials");
    actors.set(["a", "b", "c", "d", "e", "f"].map((id, i) => ({ id, kind: i % 2 ? "cat" : "person", x: i, y: 0 })));
    const all = actors.list(), group = all.slice(4);
    actors.use("sil", group);
    assert.deepEqual(all.map((a) => a.rig.root.visible), [false, false, false, false, true, true], "only the group is drawn");
    group.forEach((a, j) => a.meshes.forEach((m) => {
      assert.match(m.material.fragmentShader, new RegExp(`vec4\\(${[0, 1, 2, 3].map((k) => (k === j ? "1\\.0" : "0\\.0")).join(",")}\\)`), "channel j");
      assert.equal(m.material.blendEquation, THREE.MaxEquation);
      assert.equal(m.material.depthTest, false);
    }));
    actors.use("shade");
    assert.ok(all.every((a) => a.rig.root.visible), "all drawn again");
  },
};

tests["props: boxes and door panels sit where the H spec says (footprint, bottom, yaw), and swap materials per pass"] = () => {
  const props = new Props(), yaw = 0.6;
  assert.equal(props.set([
    { id: "door", kind: "panel", x: 1.0, y: 2.0, z: 0.25, w: 0.8, h: 2.0, yaw, color: 0xd8d0c0 },
    { id: "crate", kind: "box", x: -1, y: 0.5, z: 0, w: 0.4, d: 0.3, h: 0.5 },
  ]), 2);
  assert.deepEqual(propSize({ kind: "panel", w: 0.8, h: 2 }), { w: 0.8, d: 0.04, h: 2 }, "a panel is 4 cm thick");
  const door = props.items.get("door").mesh, box3 = new THREE.Box3().setFromObject(door), [cy, cx] = [Math.cos(yaw), Math.sin(yaw)];
  near(box3.min.y, 0.25, 1e-6, "bottom at z");
  near(box3.max.y, 2.25, 1e-6, "top at z + h");
  // The panel's corners in H: its long side runs along yaw (CCW from +x), centred on (x, y).
  const corners = [-1, 1].flatMap((a) => [-1, 1].map((b) => new THREE.Vector3(a * 0.5, 0, b * 0.5).applyMatrix4(door.matrixWorld)));
  for (const c of corners) {
    const [x, y] = [c.x, -c.z], along = (x - 1) * cy + (y - 2) * cx, across = -(x - 1) * cx + (y - 2) * cy;
    near(Math.abs(along), 0.4, 1e-6, "half the width along yaw");
    near(Math.abs(across), 0.02, 1e-6, "half the thickness across");
  }
  const crate = new THREE.Box3().setFromObject(props.items.get("crate").mesh);
  nearVec([crate.min.x, -crate.max.z, crate.min.y], [-1.2, 0.35, 0], 1e-6, "box min (H)");
  nearVec([crate.max.x, -crate.min.z, crate.max.y], [-0.8, 0.65, 0.5], 1e-6, "box max (H)");
  const shade = props.items.get("door").mat;
  assert.ok(shade.isMeshLambertMaterial && shade.color.getHex() !== 0, "lit, coloured");
  const s = { fragmentShader: "#include <colorspace_fragment>" };
  shade.onBeforeCompile(s);
  assert.match(s.fragmentShader, /sRGBTransferOETF/, "encoded to sRGB like the actors (the atlas holds sRGB)");
  props.use("id");
  assert.ok(door.material.isMeshBasicMaterial && door.material.color.getHex() === 0 && props.root.visible, "id pass: black (no actor), still hiding what is behind");
  props.use("sil");
  assert.equal(props.root.visible, false, "whole-silhouette pass: not drawn");
  props.use("shade");
  assert.ok(props.root.visible && door.material === shade);
  props.set([{ id: "door", kind: "panel", x: 1, y: 2, z: 0.25, w: 0.8, h: 2, yaw, color: 0x203040 }]);
  assert.equal(props.root.children.length, 1, "the crate is gone");
  assert.equal(props.items.get("door").mesh.material.color.getHexString(), new THREE.Color(0x203040).getHexString(), "recoloured");
  assert.deepEqual(props.list().map((p) => p.id), ["door"]);
};

tests["setProps and depthBatch go to the worker, one message each"] = async () => {
  const sent = [];
  globalThis.Worker = class {
    constructor() { this.terminate = () => {}; }
    postMessage(m, transfer) {
      sent.push({ ...m, transfer });
      const result = m.op === "init" ? { splats: 1 } : m.op === "depthBatch" ? m.args.poses.map(() => new Float32Array(4)) : m.args.length;
      setTimeout(() => this.onmessage({ data: { id: m.id, result } }));
    }
  };
  try {
    const { Twin } = await import("../app/js/twin/twin.js");
    const twin = await Twin.create({ splat: new ArrayBuffer(8), frame: { f: 1, Yf: 0 } });
    assert.equal(await twin.setProps([{ id: "a", kind: "box", x: 0, y: 0, z: 0, w: 1, d: 1, h: 1 }]), 1);
    const poses = [0, 1, 2].map((k) => ({ x: k, y: 0, z: 1, yaw: 0 })), view = { width: 2, height: 2, lens: pinholeLens(90), props: false };
    const out = await twin.depthBatch(poses, view);
    assert.equal(out.length, 3);
    assert.deepEqual(sent.at(-1).args, { poses, view });
    assert.deepEqual(sent.map((m) => m.op), ["init", "setProps", "depthBatch"]);
  } finally { delete globalThis.Worker; }
};

tests["the twin's queue: localization, then live depth, then the camera, evidence, the 3D view and map building; state changes first; a waiting job moves up a level a second; prio goes with each call and pending() counts by it"] = async () => {
  const { nextJob, PRIO, AGE_MS } = await import("../app/js/twin/queue.js");
  const order = (jobs, t = 0) => { const l = jobs.map((j, i) => ({ id: i, at: 0, ...j })), out = []; for (let j; (j = nextJob(l, t)); ) out.push(j.id); return out; };
  assert.deepEqual(order([{ prio: PRIO.view }, { prio: PRIO.camera }, { prio: PRIO.loc }, { prio: PRIO.depth }, { prio: PRIO.state }, { prio: PRIO.camera }, { prio: PRIO.build }]), [4, 2, 3, 1, 5, 0, 6]);
  // a 3D view queued 2.5 s ago goes before a camera frame queued now (it isn't starved), not before localization
  assert.deepEqual(order([{ prio: PRIO.camera, at: 2500 }, { prio: PRIO.view, at: 0 }, { prio: PRIO.loc, at: 2500 }], 2500), [2, 1, 0]);
  assert.equal(AGE_MS, 1000);
  const sent = [];
  globalThis.Worker = class {
    constructor() { this.terminate = () => {}; }
    postMessage(m) {
      sent.push(m);
      if (m.op === "init") setTimeout(() => this.onmessage({ data: { id: m.id, result: { splats: 1 } } }));
      else this.last = m;
    }
  };
  try {
    const { Twin } = await import("../app/js/twin/twin.js");
    const twin = await Twin.create({ splat: new ArrayBuffer(8), frame: { f: 1, Yf: 0 } }), pose = { x: 0, y: 0, z: 1, yaw: 0 };
    twin.pixelsDepth(pose, { width: 4, height: 3, prio: "loc" }, { width: 2, height: 2 });
    twin.render(pose, { width: 4, height: 3 });
    twin.render(pose, { width: 4, height: 3, prio: "camera" });
    twin.setProps([]);
    twin.depth(pose, { prio: "depth" });
    assert.deepEqual(sent.slice(1).map((m) => [m.op, m.prio]), [["pair", PRIO.loc], ["render", PRIO.view], ["render", PRIO.camera], ["setProps", PRIO.state], ["depth", PRIO.depth]]);
    assert.deepEqual([twin.pending("loc"), twin.pending("depth"), twin.pending("camera"), twin.pending(), twin.busy], [2, 3, 4, 5, true], "pending counts state changes as the most urgent");
    twin.ordered = false; // GPU.enabled off: first come, first served
    twin.render(pose, { prio: "loc" });
    assert.equal(sent.at(-1).prio, PRIO.evidence);
  } finally { delete globalThis.Worker; }
};

tests["the worker's results go back whole: an oracle's boxes (with their pixel counts) are copied, pictures and depth handed over once each; the worker uses this list"] = async () => {
  const { transferOf } = await import("../app/js/twin/queue.js"), { readFile } = await import("node:fs/promises");
  const post = (result) => structuredClone(result, { transfer: transferOf(result) }); // what self.postMessage does with the list
  const oracle = [{ id: "person-1", kind: "person", box: { x: 0.1, y: 0.2, w: 0.1, h: 0.5 }, visibleBox: { x: 0.1, y: 0.3, w: 0.1, h: 0.4 }, visibleFraction: 0.8, pixels: 1234, visiblePixels: 987 },
    { id: "cat", kind: "cat", box: null, visibleBox: null, visibleFraction: 0, pixels: 40, visiblePixels: 0 }];
  assert.deepEqual(transferOf(oracle), []);
  assert.deepEqual(post(oracle), oracle);
  const pair = { pixels: { width: 2, height: 1, data: new Uint8ClampedArray(8) }, depth: new Float32Array(2) }, bufs = transferOf(pair);
  assert.deepEqual(bufs, [pair.pixels.data.buffer, pair.depth.buffer]);
  assert.equal(post(pair).depth.length, 2);
  assert.equal(pair.depth.byteLength, 0, "handed over, not copied");
  const batch = [new Float32Array(4), new Float32Array(4)], shared = new Float32Array(8), halves = [shared.subarray(0, 4), shared.subarray(4)];
  assert.equal(transferOf(batch).length, 2);
  assert.equal(transferOf(halves).length, 1, "one buffer once");
  assert.equal(post(halves)[1].length, 4);
  for (const plain of [3, true, null, undefined, { splats: 1, gpu: "x" }, { model: "equidistant", k: [0, 0, 0, 0] }]) assert.deepEqual(transferOf(plain), []);
  globalThis.ImageBitmap = class {};
  try {
    const bmp = new ImageBitmap();
    assert.deepEqual(transferOf([bmp, { bmp }]), [bmp]);
  } finally { delete globalThis.ImageBitmap; }
  const worker = await readFile(new URL("../app/js/twin/twin-worker.js", import.meta.url), "utf8");
  assert.ok(/import \{[^}]*\btransferOf\b[^}]*\} from "\.\/queue\.js"/.test(worker) && !/const transferOf\s*=/.test(worker), "twin-worker.js posts with queue.js's transferOf");
};

tests["vendored Spark: every bundled third-party module is credited with its licence in VENDOR.md"] = async () => {
  const { readFile } = await import("node:fs/promises"), dir = new URL("../app/vendor/spark/", import.meta.url);
  const js = await readFile(new URL("spark.module.js", dir), "utf8"), notes = await readFile(new URL("VENDOR.md", dir), "utf8");
  const bundled = [...new Set([...js.matchAll(/#region node_modules\/([^/]+)\//g)].map((m) => m[1]))];
  assert.deepEqual(bundled, ["fflate"]);
  for (const name of bundled) assert.match(notes, new RegExp(`${name}[^\\n]*\\(MIT\\)[\\s\\S]*Copyright \\(c\\)`), `${name} credited`);
};

let failed = 0;
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name}\n      ${e.stack?.split("\n").slice(0, 3).join("\n      ")}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed`);
process.exit(failed ? 1 : 0);
