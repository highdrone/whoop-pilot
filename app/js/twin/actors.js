// Simulated people and pets for the twin: articulated figures built here (no third-party models), posed stand / walk /
// sit / lie and composited with the splats by depth.
// Spec: { id, kind: "person"|"cat"|"dog", x, y, z (ground contact, H), yaw (rad, CCW from +x), pose, phase (walk
// cycle, rad), color? (person: { skin, shirt, pants, hair, shoe }; cat, dog: the keys of QUAD[kind].color) }. Figures
// face +x at yaw 0. Built in three.js world axes: +X forward, +Y up, -Z left.
// Each joint is one mesh: its parts baked into one geometry with vertex colours and a fur pattern strength, so a figure
// is about 15 draw calls.
import * as THREE from "../../vendor/three/build/three.module.js";

// Lambert with a fur grain and tabby bands in the joint's coordinates (strengths in the pat attribute), output encoded
// to sRGB: the atlas holds sRGB values (splats blend in sRGB).
const NOISE = `float h3(vec3 p) { return fract(sin(dot(p, vec3(127.1, 311.7, 74.7))) * 43758.5453); }
float vnoise(vec3 p) {
  vec3 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(mix(h3(i), h3(i + vec3(1, 0, 0)), f.x), mix(h3(i + vec3(0, 1, 0)), h3(i + vec3(1, 1, 0)), f.x), f.y),
    mix(mix(h3(i + vec3(0, 0, 1)), h3(i + vec3(1, 0, 1)), f.x), mix(h3(i + vec3(0, 1, 1)), h3(i + vec3(1, 1, 1)), f.x), f.y), f.z);
}`;
function figureMaterial() {
  const m = new THREE.MeshLambertMaterial({ vertexColors: true });
  m.onBeforeCompile = (s) => {
    s.vertexShader = s.vertexShader.replace("#include <common>", "#include <common>\nattribute vec2 pat;\nvarying vec2 vPat;\nvarying vec3 vP;")
      .replace("#include <begin_vertex>", "#include <begin_vertex>\nvP = position; vPat = pat;");
    s.fragmentShader = s.fragmentShader.replace("#include <common>", `#include <common>\nvarying vec2 vPat;\nvarying vec3 vP;\n${NOISE}`)
      .replace("#include <color_fragment>", `#include <color_fragment>
  float grain = (0.8 + 0.4 * vnoise(vP * 180.0) * vnoise(vP * 23.0 + 7.0)) * (0.8 + 0.4 * vnoise(vP * 19.0 + 3.0));
  float band = smoothstep(0.25, 0.75, 0.5 + 0.5 * sin(dot(vP, vec3(55.0, 38.0, 6.0)) + 4.0 * vnoise(vP * 9.0)));
  diffuseColor.rgb *= mix(1.0, grain, vPat.x) * (1.0 - 0.6 * vPat.y * band);`)
      .replace("#include <normal_fragment_maps>", "#include <normal_fragment_maps>\n  diffuseColor.rgb *= mix(1.0, 0.4 + 0.6 * abs(dot(normal, normalize(vViewPosition))), vPat.x * 0.8);")
      .replace("#include <colorspace_fragment>", "gl_FragColor = sRGBTransferOETF( gl_FragColor );");
  };
  return m;
}
const FLAT_VS = "#include <common>\n#include <skinning_pars_vertex>\nvarying float vz;\nvoid main() {\n#include <skinbase_vertex>\n#include <begin_vertex>\n#include <skinning_vertex>\n#include <project_vertex>\nvz = -mvPosition.z;\n}";
const flat = (frag, o) => new THREE.ShaderMaterial({ vertexShader: FLAT_VS, fragmentShader: `varying float vz;\n${frag}`, ...o });
export const idMaterial = (id) => flat("uniform float id;\nvoid main() { gl_FragColor = vec4(id / 255.0, 0.0, 0.0, 1.0); }", { uniforms: { id: { value: id } } });
// Float pass: view depth with full coverage (expected depth) and 1 in b, which the splats in front attenuate to the
// actor's transmittance.
export const depthMaterial = () => flat("void main() { gl_FragColor = vec4(vz, 1.0, 1.0, 1.0); }");
// Whole silhouettes, four actors per pass: actor j of a group lights channel j, MAX blending, no depth test.
const SIL = [0, 1, 2, 3].map((c) => flat(`void main() { gl_FragColor = vec4(${[0, 1, 2, 3].map((k) => (k === c ? "1.0" : "0.0"))}); }`,
  { blending: THREE.CustomBlending, blendEquation: THREE.MaxEquation, depthTest: false, depthWrite: false }));

// Part geometry, transformed into its joint's frame: ellipsoid radii, a limb hanging along -Y from (0, 0, 0) tapering
// from r0 to r1 over len, a cone (ears).
const M4 = new THREE.Matrix4(), QT = new THREE.Quaternion(), EU = new THREE.Euler(), ONE = new THREE.Vector3(1, 1, 1), TV = new THREE.Vector3();
const put = (g, at = [0, 0, 0], rot = [0, 0, 0]) => g.applyMatrix4(M4.compose(TV.set(...at), QT.setFromEuler(EU.set(...rot)), ONE));
const ell = ([rx, ry, rz], at, rot, seg = 20) => put(new THREE.SphereGeometry(1, seg, Math.ceil(seg * 0.75)).scale(rx, ry, rz), at, rot);
function limb(r0, r1, len, at, rot, seg = 14) {
  const pts = [];
  for (let i = 0; i <= 6; i++) { const a = (-Math.PI / 2) * (1 - i / 6); pts.push(new THREE.Vector2(r1 * Math.cos(a), -len + r1 * Math.sin(a))); }
  for (let i = 0; i <= 6; i++) { const a = (Math.PI / 2) * (i / 6); pts.push(new THREE.Vector2(Math.max(r0 * Math.cos(a), 0), r0 * Math.sin(a))); }
  return put(new THREE.LatheGeometry(pts, seg), at, rot);
}
const cone = (r, h, at, rot, seg = 12) => put(new THREE.ConeGeometry(r, h, seg), at, rot);

// One mesh for a joint: parts [geometry, colour (hex, or (position, normal) => hex), fur grain, tabby bands].
const CV = new THREE.Color(), PV = new THREE.Vector3(), NV = new THREE.Vector3();
function bake(joint, parts, mat) {
  let nv = 0, ni = 0;
  for (const [g] of parts) { nv += g.attributes.position.count; ni += g.index.count; }
  const pos = new Float32Array(nv * 3), nor = new Float32Array(nv * 3), col = new Float32Array(nv * 3), pat = new Float32Array(nv * 2), idx = new Uint32Array(ni);
  let v = 0, i = 0;
  for (const [g, color, grain = 0, bands = 0] of parts) {
    const P = g.attributes.position, N = g.attributes.normal;
    pos.set(P.array, v * 3);
    nor.set(N.array, v * 3);
    for (let k = 0; k < P.count; k++) {
      CV.set(typeof color === "function" ? color(PV.fromBufferAttribute(P, k), NV.fromBufferAttribute(N, k)) : color).toArray(col, (v + k) * 3);
      pat[(v + k) * 2] = grain;
      pat[(v + k) * 2 + 1] = bands;
    }
    for (let k = 0; k < g.index.count; k++) idx[i + k] = g.index.array[k] + v;
    v += P.count; i += g.index.count;
    g.dispose();
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  geo.setAttribute("normal", new THREE.BufferAttribute(nor, 3));
  geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
  geo.setAttribute("pat", new THREE.BufferAttribute(pat, 2));
  geo.setIndex(new THREE.BufferAttribute(idx, 1));
  joint.add(new THREE.Mesh(geo, mat));
}
const joint = (parent, x, y, z) => { const g = new THREE.Group(); g.position.set(x, y, z); parent.add(g); return g; };

function person(c, mat) {
  const C = { skin: 0xd9a383, shirt: 0x2f6db5, pants: 0x34465f, hair: 0x2e2018, shoe: 0x1c1c1c, ...c };
  const lip = new THREE.Color(C.skin).multiplyScalar(0.75).getHex(), brow = new THREE.Color(C.hair).multiplyScalar(0.8).getHex();
  const root = new THREE.Group(), body = joint(root, 0, 0, 0), hips = joint(body, 0, 0.95, 0), spine = joint(hips, 0, 0.06, 0);
  bake(hips, [[ell([0.11, 0.13, 0.175], [0, 0, 0]), C.pants, 0.5], [ell([0.115, 0.03, 0.172], [0, 0.07, 0]), 0x2a2a2a]], mat);
  // torso: waist 0 -> chest -> shoulders, 0.6 deep for its width
  const prof = [[0, -0.02], [0.15, -0.02], [0.155, 0.08], [0.18, 0.22], [0.19, 0.33], [0.17, 0.42], [0.11, 0.47], [0, 0.48]].map(([r, y]) => new THREE.Vector2(r, y));
  bake(spine, [
    [new THREE.LatheGeometry(prof, 18).scale(0.62, 1, 1), C.shirt, 0.3],
    [ell([0.07, 0.075, 0.075], [0, 0.4, -0.17]), C.shirt, 0.3], [ell([0.07, 0.075, 0.075], [0, 0.4, 0.17]), C.shirt, 0.3],
    [limb(0.052, 0.055, 0.12, [0.0, 0.58, 0]), C.skin],
  ], mat);
  const head = joint(spine, 0, 0.66, 0);
  const face = [
    [ell([0.098, 0.118, 0.083], [0.01, 0, 0]), C.skin], [ell([0.07, 0.055, 0.062], [0.035, -0.065, 0]), C.skin],
    [ell([0.105, 0.098, 0.09], [-0.016, 0.036, 0]), C.hair, 0.6],
    [ell([0.022, 0.026, 0.014], [0.103, -0.012, 0], [0, 0, -0.25]), C.skin], [ell([0.006, 0.007, 0.026], [0.099, -0.05, 0]), lip],
  ];
  for (const z of [-1, 1]) face.push(
    [ell([0.007, 0.009, 0.015], [0.093, 0.016, 0.035 * z]), 0xf2eee8], [ell([0.004, 0.0075, 0.0075], [0.0975, 0.016, 0.036 * z]), 0x2b1a10],
    [ell([0.006, 0.0045, 0.021], [0.092, 0.041, 0.036 * z]), brow], [ell([0.014, 0.028, 0.01], [-0.004, 0.0, 0.083 * z]), C.skin]);
  bake(head, face, mat);
  const rig = { root, body, hips, spine, head };
  for (const [s, z] of [["L", -1], ["R", 1]]) {
    const sh = (rig["shoulder" + s] = joint(spine, 0, 0.44, 0.215 * z));
    bake(sh, [[limb(0.06, 0.056, 0.12), C.shirt, 0.3], [limb(0.047, 0.04, 0.29), C.skin]], mat);
    const el = (rig["elbow" + s] = joint(sh, 0, -0.29, 0));
    bake(el, [[limb(0.042, 0.032, 0.25), C.skin], [ell([0.042, 0.06, 0.02], [0.005, -0.3, 0]), C.skin]], mat);
    const hip = (rig["hip" + s] = joint(hips, 0, 0, 0.09 * z));
    bake(hip, [[limb(0.085, 0.06, 0.45), C.pants, 0.5]], mat);
    const kn = (rig["knee" + s] = joint(hip, 0, -0.45, 0));
    bake(kn, [[limb(0.06, 0.045, 0.40), C.pants, 0.5], [ell([0.13, 0.045, 0.052], [0.05, -0.43, 0]), C.shoe]], mat);
  }
  return rig;
}

function posePerson(r, pose, ph) {
  for (const k of ["shoulderL", "shoulderR", "elbowL", "elbowR", "hipL", "hipR", "kneeL", "kneeR", "body", "spine", "head"]) r[k].rotation.set(0, 0, 0);
  r.body.position.set(0, 0, 0);
  r.shoulderL.rotation.x = 0.08; r.shoulderR.rotation.x = -0.08;
  r.elbowL.rotation.z = r.elbowR.rotation.z = 0.12;
  if (pose === "walk") {
    const s = Math.sin(ph);
    r.hipL.rotation.z = 0.42 * s; r.hipR.rotation.z = -0.42 * s;
    r.kneeL.rotation.z = -0.5 * Math.max(0, -Math.cos(ph)); r.kneeR.rotation.z = -0.5 * Math.max(0, Math.cos(ph));
    r.shoulderL.rotation.z = -0.35 * s; r.shoulderR.rotation.z = 0.35 * s;
    r.elbowL.rotation.z = r.elbowR.rotation.z = 0.3;
  } else if (pose === "sit") { // on a seat about 0.45 m high
    r.hipL.rotation.z = r.hipR.rotation.z = Math.PI / 2;
    r.kneeL.rotation.z = r.kneeR.rotation.z = -Math.PI / 2;
    r.shoulderL.rotation.z = r.shoulderR.rotation.z = 0.35;
    r.elbowL.rotation.z = r.elbowR.rotation.z = 0.8;
  } else if (pose === "lie") { // on the back, head toward -X
    r.body.rotation.z = Math.PI / 2;
    r.body.position.x = 0.85;
  }
}

// Cats and dogs share one quadruped rig with different proportions (metres). Joints: trunk (centre of the chest-loin
// line), neck at the chest, head, shoulders and hips with two (front) or three (hind) leg segments, a tail of four.
// paint(part, position, normal) picks each vertex's colour key: a brown tabby cat with a cream belly, and a black and
// tan shepherd dog (the saddle, muzzle and ears are what make it read as a dog rather than a plush toy).
const QUAD = {
  cat: {
    color: { fur: 0x8f7a62, belly: 0xe6d9c2, nose: 0xc98c8c, eye: 0xb3ae3c, ear: 0x7a6650, inner: 0xd6a4a0 }, grain: 0.8, bands: 1,
    paint: (part, p, n) => (part === "trunk" ? (n.y < -0.35 || (p.x > 0.12 && n.x > 0.6) ? "belly" : "fur") : part === "neck" ? (n.x > 0.5 ? "belly" : "fur") : "fur"),
    trunk: [[[0.085, 0.078, 0.062], [0.1, 0.005, 0]], [[0.13, 0.068, 0.06], [-0.01, 0, 0]], [[0.078, 0.074, 0.066], [-0.115, 0.004, 0]]],
    neck: { at: [0.15, 0.03], r: [0.042, 0.038], len: 0.05, tilt: 0.95 },
    head: { skull: [0.05, 0.047, 0.056], cheeks: [[0.032, 0.03, 0.06], [0.012, -0.014, 0]], muzzle: [[0.022, 0.018, 0.028], [0.04, -0.022, 0], "belly"], nose: [[0.006, 0.005, 0.009], [0.061, -0.009, 0]],
      eye: { r: [0.009, 0.014, 0.014], at: [0.037, 0.01, 0.023], pupil: [0.003, 0.011, 0.0035] }, ear: { r: 0.026, h: 0.05, at: [-0.004, 0.05, 0.03], tilt: 0.32, seg: 3 } },
    front: { at: [0.1, -0.035, 0.03], seg: [[0.024, 0.018, 0.085], [0.017, 0.015, 0.075]], paw: [0.022, 0.012, 0.017], muscle: [[0.035, 0.05, 0.026], [0, -0.025, 0]] },
    hind: { at: [-0.115, 0.002, 0.04], seg: [[0.04, 0.022, 0.09], [0.019, 0.015, 0.085], [0.015, 0.013, 0.05]], paw: [0.024, 0.012, 0.017], muscle: [[0.05, 0.065, 0.032], [0.005, -0.035, 0]] },
    tail: { at: [-0.185, 0.03], r: [0.017, 0.012], len: 0.075 },
    sit: { tilt: 0.65, thigh: 1.25, shin: -1.3, front: 0 }, // body tilt, hind thigh and shin angles from vertical, front legs forward
  },
  dog: {
    color: { fur: 0xae7a42, belly: 0xc99a62, dark: 0x221c18, nose: 0x111010, eye: 0x24180f, ear: 0x2a221c, inner: 0x5a4030 }, grain: 0.6, bands: 0,
    paint: (part, p, n) => part === "trunk" ? (n.y > 0.3 && p.x < 0.16 && p.x > -0.26 ? "dark" : n.y < -0.4 ? "belly" : "fur")
      : part === "tail" ? (n.x < -0.2 || n.y > 0.4 ? "dark" : "fur") : part === "neck" ? (n.x < -0.4 ? "dark" : "fur")
      : part === "head" ? (p.x > 0.11 || (p.x > 0.05 && p.y < -0.03) ? "dark" : n.y > 0.6 && p.x < 0.05 ? "dark" : "fur") : "fur",
    trunk: [[[0.16, 0.15, 0.105], [0.13, -0.01, 0]], [[0.21, 0.11, 0.1], [-0.05, 0.012, 0]], [[0.13, 0.12, 0.105], [-0.2, 0.0, 0]]],
    neck: { at: [0.22, 0.06], r: [0.09, 0.07], len: 0.13, tilt: 1.0, collar: 0xb3262e },
    head: { skull: [0.1, 0.09, 0.085], cheeks: [[0.07, 0.06, 0.09], [0.03, -0.032, 0]], snout: [0.056, 0.036, 0.13, [0.055, -0.035, 0]], nose: [[0.025, 0.021, 0.03], [0.205, -0.028, 0]],
      eye: { r: [0.013, 0.014, 0.015], at: [0.075, 0.024, 0.046] }, ear: { r: 0.05, h: 0.115, at: [-0.035, 0.105, 0.05], tilt: 0.32, seg: 4 }, tongue: [[0.04, 0.007, 0.024], [0.15, -0.082, 0]] },
    front: { at: [0.17, -0.06, 0.072], seg: [[0.05, 0.036, 0.18], [0.034, 0.03, 0.17]], paw: [0.042, 0.02, 0.034], muscle: [[0.065, 0.11, 0.055], [0.0, -0.03, 0]] },
    hind: { at: [-0.2, 0.016, 0.072], seg: [[0.07, 0.042, 0.2], [0.04, 0.03, 0.19], [0.03, 0.028, 0.11]], paw: [0.042, 0.02, 0.034], muscle: [[0.085, 0.12, 0.062], [0.01, -0.05, 0]] },
    tail: { at: [-0.31, 0.04], r: [0.04, 0.03], len: 0.095 },
    sit: { tilt: 0.6, thigh: 0.5, shin: -1.5, front: 0.22 }, // rump just off the floor: upright, RF-DETR sees a teddy bear
  },
};
const HIND_REST = [0.55, -1.15, 0.6]; // hip, knee, hock: thigh forward, shin back, metatarsus near vertical

function quadruped(kind, c, mat) {
  const P = QUAD[kind], C = { ...P.color, ...c }, paint = (part) => (p, n) => C[P.paint(part, p, n)];
  const root = new THREE.Group(), body = joint(root, 0, 0, 0), trunk = joint(body, 0, 0, 0);
  bake(trunk, P.trunk.map(([r, at]) => [ell(r, at, [0, 0, 0], 24), paint("trunk"), P.grain, P.bands]), mat);
  const neck = joint(trunk, P.neck.at[0], P.neck.at[1], 0);
  bake(neck, [[limb(P.neck.r[0], P.neck.r[1], P.neck.len, [0, 0, 0], [0, 0, Math.PI]), paint("neck"), P.grain, P.bands * 0.6],
    ...(P.neck.collar ? [[new THREE.TorusGeometry(P.neck.r[1] * 1.08, 0.012, 8, 24).rotateX(Math.PI / 2).translate(0, P.neck.len * 0.72, 0), P.neck.collar]] : [])], mat);
  const neckEnd = joint(neck, 0, P.neck.len, 0), head = joint(neckEnd, 0, 0, 0), H = P.head;
  const parts = [[ell(H.skull, [0, 0, 0], [0, 0, 0], 24), paint("head"), P.grain, P.bands * 0.5], [ell(...H.cheeks), paint("head"), P.grain, P.bands * 0.3], [ell(...H.nose), C.nose]];
  if (H.muzzle) parts.push([ell(H.muzzle[0], H.muzzle[1]), C[H.muzzle[2]], P.grain * 0.6]);
  if (H.snout) parts.push([limb(H.snout[0], H.snout[1], H.snout[2], H.snout[3], [0, 0, Math.PI / 2], 16), paint("head"), P.grain * 0.6]);
  if (H.tongue) parts.push([ell(...H.tongue), 0xd77a82]);
  for (const z of [-1, 1]) {
    const E = H.eye, [ex, ey, ez] = E.at, A = H.ear;
    parts.push([ell(E.r, [ex, ey, ez * z], [0, -0.45 * z, 0], 12), C.eye], [ell([0.003, 0.003, 0.003], [ex + E.r[0] * 0.8, ey + E.r[1] * 0.4, ez * z - 0.002 * z], [0, 0, 0], 6), 0xffffff]);
    if (E.pupil) parts.push([ell(E.pupil, [ex + E.r[0] * 0.7, ey, ez * z], [0, -0.45 * z, 0], 8), 0x0d0b08]);
    parts.push([cone(A.r, A.h, [A.at[0], A.at[1], A.at[2] * z], [z * A.tilt, A.seg === 3 ? Math.PI / 6 : Math.PI / 4, 0], A.seg), C.ear, P.grain],
      [cone(A.r * 0.6, A.h * 0.7, [A.at[0] + A.r * 0.3, A.at[1] - A.h * 0.08, A.at[2] * z], [z * A.tilt, A.seg === 3 ? Math.PI / 6 : Math.PI / 4, 0], A.seg), C.inner]);
  }
  bake(head, parts, mat);
  const rig = { root, body, trunk, neck, head, legs: [], P, kind };
  const leg = (parent, at, { seg, paw, muscle }) => {
    const js = [];
    let j = joint(parent, ...at);
    seg.forEach(([r0, r1, len], i) => {
      js.push(j);
      bake(j, [[limb(r0, r1, len), paint("leg"), P.grain, P.bands * 0.8], ...(i === 0 ? [[ell(...muscle), paint("leg"), P.grain, P.bands * 0.8]] : []),
        ...(i === seg.length - 1 ? [[ell(paw, [paw[0] * 0.35, -len - paw[1] * 0.3, 0]), paint("leg"), P.grain]] : [])], mat);
      j = joint(j, 0, -len, 0);
    });
    return js;
  };
  for (const z of [-1, 1]) rig.legs.push(leg(trunk, [P.front.at[0], P.front.at[1], P.front.at[2] * z], P.front));
  for (const z of [-1, 1]) rig.legs.push(leg(trunk, [P.hind.at[0], P.hind.at[1], P.hind.at[2] * z], P.hind));
  rig.tail = [];
  let t = joint(trunk, P.tail.at[0], P.tail.at[1], 0);
  for (let i = 0; i < 4; i++) {
    const r = P.tail.r[0] + ((P.tail.r[1] - P.tail.r[0]) * i) / 3;
    bake(t, [[limb(r, r * 0.92, P.tail.len, [0, 0, 0], [0, 0, 0], 10), paint("tail"), P.grain, P.bands]], mat);
    rig.tail.push(t);
    t = joint(t, 0, -P.tail.len, 0);
  }
  return rig;
}

// Leg order: front left, front right, hind left, hind right. Angles about z: + swings the foot forward.
function poseQuad(r, pose, ph) {
  const cat = r.kind === "cat";
  r.body.rotation.set(0, 0, 0); r.body.position.set(0, 0, 0);
  r.neck.rotation.set(0, 0, -r.P.neck.tilt); r.head.rotation.set(0, 0, r.P.neck.tilt);
  const set = (leg, angles) => leg.forEach((j, i) => j.rotation.set(0, 0, angles[i] ?? 0));
  for (const l of r.legs.slice(0, 2)) set(l, [0.05, -0.05]);
  for (const l of r.legs.slice(2)) set(l, HIND_REST);
  // tail: angles about z (in the body's plane), then curls about x (sideways, once the base lies along the floor)
  const tail = (a, curl = [0, 0, 0, 0]) => r.tail.forEach((j, i) => j.rotation.set(curl[i], 0, a[i]));
  tail(cat ? [-2.75, -0.2, -0.3, -0.4] : [-0.75, 0.1, 0.15, -0.25]);
  if (pose === "walk") {
    const s = Math.sin(ph), c = Math.cos(ph);
    set(r.legs[0], [0.35 * s, -0.3 * Math.max(0, c)]); set(r.legs[3], [HIND_REST[0] + 0.3 * s, HIND_REST[1] - 0.25 * Math.max(0, c), HIND_REST[2]]);
    set(r.legs[1], [-0.35 * s, -0.3 * Math.max(0, -c)]); set(r.legs[2], [HIND_REST[0] - 0.3 * s, HIND_REST[1] - 0.25 * Math.max(0, -c), HIND_REST[2]]);
    r.head.rotation.z = r.P.neck.tilt - 0.2;
  } else if (pose === "sit") { // chest up on straight front legs, hind legs folded, hind feet flat
    const S = r.P.sit, a = S.tilt;
    r.body.rotation.z = a;
    r.head.rotation.z = r.P.neck.tilt - a + 0.15;
    for (const l of r.legs.slice(0, 2)) set(l, [S.front - a, 0]);
    for (const l of r.legs.slice(2)) set(l, [S.thigh - a, S.shin - S.thigh, Math.PI / 2 - S.shin]); // hind feet flat, pointing forward
    tail([-1.45 - a, 0, 0, 0], cat ? [0, 0.75, 0.75, 0.6] : [0, 0.5, 0.4, 0.3]);
  } else if (pose === "lie") { // sphinx: front legs forward on the floor, hind legs folded under
    for (const l of r.legs.slice(0, 2)) set(l, [-0.9, Math.PI / 2 + 0.9]); // elbows down, forearms forward on the floor
    for (const l of r.legs.slice(2)) set(l, [1.35, -2.75, Math.PI / 2 + 1.4]);
    r.head.rotation.z = r.P.neck.tilt + 0.15;
    tail([-1.5, 0, 0, 0], cat ? [0, 0.7, 0.7, 0.5] : [0, 0.4, 0.3, 0.2]);
  }
}

const box = new THREE.Box3();

export class Actors {
  constructor() {
    this.root = new THREE.Group();
    this.items = new Map(); // id -> { spec, rig, meshes, shadeMats, idMat }
  }

  set(list) {
    if (list.length > 254) throw new Error("at most 254 actors"); // ids are 8-bit, 0 = none
    const keep = new Set();
    list.forEach((spec, i) => {
      keep.add(spec.id);
      let a = this.items.get(spec.id);
      if (a && (a.spec.kind !== spec.kind || JSON.stringify(a.spec.color) !== JSON.stringify(spec.color))) { this.#drop(a); a = null; }
      if (!a) {
        const mat = figureMaterial(), rig = spec.kind === "person" ? person(spec.color, mat) : quadruped(spec.kind, spec.color, mat);
        const meshes = [];
        rig.root.traverse((o) => o.isMesh && meshes.push(o));
        a = { rig, meshes, shadeMats: meshes.map((m) => m.material), idMat: idMaterial(0) };
        this.items.set(spec.id, a);
        this.root.add(rig.root);
      }
      a.spec = spec;
      a.index = i + 1;
      a.idMat.uniforms.id.value = i + 1;
      const o = a.rig.root;
      o.position.set(spec.x, spec.z ?? 0, -spec.y);
      o.rotation.set(0, spec.yaw ?? 0, 0);
      (spec.kind === "person" ? posePerson : poseQuad)(a.rig, spec.pose ?? "stand", spec.phase ?? 0);
      a.rig.body.position.y -= box.setFromObject(o, true).min.y - o.position.y; // lowest point on the floor
    });
    for (const [id, a] of this.items) if (!keep.has(id)) this.#drop(a);
  }

  // "shade" for the camera image, "id" for depth-tested ids, "sil" for the whole silhouettes of only (at most 4).
  use(kind, only) {
    for (const a of this.items.values()) {
      const j = only ? only.indexOf(a) : 0, m = kind === "id" ? a.idMat : kind === "sil" ? SIL[j] : null;
      a.rig.root.visible = j >= 0;
      a.meshes.forEach((mesh, i) => (mesh.material = m ?? a.shadeMats[i]));
    }
  }

  list() { return [...this.items.values()].sort((a, b) => a.index - b.index); }

  #drop(a) {
    this.root.remove(a.rig.root);
    this.items.delete(a.spec.id);
    for (const m of a.meshes) m.geometry.dispose();
    for (const m of new Set(a.shadeMats)) m.dispose();
    a.idMat.dispose();
  }
}
