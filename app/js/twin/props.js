// Things that are not in the splat, drawn with it: the simulator's closed doors and new obstacles (docs/HOME-DRONE.md,
// "Wave C contracts"). Spec: { id, kind: "box" | "panel", x, y (centre of the footprint, H), z (bottom, H), w (along yaw),
// d (across; a panel defaults to 4 cm), h, yaw (rad, CCW from +x), color (hex or CSS) }. Opaque, so they hide the splats
// and actors behind them in the image, the expected depth and the oracle.
import * as THREE from "../../vendor/three/build/three.module.js";

const PANEL = 0.04;
const box = new THREE.BoxGeometry(1, 1, 1);
const NONE = new THREE.MeshBasicMaterial({ color: 0x000000 }); // id pass: occludes, id 0

// Lambert, output encoded to sRGB like the actors' (the atlas holds sRGB values; splats blend in sRGB).
function shade(color) {
  const m = new THREE.MeshLambertMaterial({ color: new THREE.Color(color ?? 0x8a8070) });
  m.onBeforeCompile = (s) => {
    s.fragmentShader = s.fragmentShader.replace("#include <colorspace_fragment>", "gl_FragColor = sRGBTransferOETF( gl_FragColor );");
  };
  return m;
}

export const propSize = (p) => ({ w: p.w ?? 1, d: p.d ?? (p.kind === "panel" ? PANEL : 1), h: p.h ?? 1 });

export class Props {
  constructor() {
    this.root = new THREE.Group();
    this.items = new Map(); // id -> { spec, mesh, mat }
  }

  set(list) {
    const keep = new Set();
    for (const spec of list) {
      keep.add(spec.id);
      let it = this.items.get(spec.id);
      if (it && it.spec.color !== spec.color) {
        it.mat.dispose();
        it.mat = it.mesh.material = shade(spec.color);
      }
      if (!it) {
        const mat = shade(spec.color);
        it = { mat, mesh: new THREE.Mesh(box, mat) };
        this.items.set(spec.id, it);
        this.root.add(it.mesh);
      }
      it.spec = spec;
      const { w, d, h } = propSize(spec), m = it.mesh;
      m.scale.set(w, h, d);
      m.position.set(spec.x, (spec.z ?? 0) + h / 2, -spec.y); // three.js world = (x_H, z_H, -y_H)
      m.rotation.set(0, spec.yaw ?? 0, 0);
      m.updateMatrixWorld(true);
    }
    for (const [id, it] of this.items)
      if (!keep.has(id)) {
        this.root.remove(it.mesh);
        it.mat.dispose();
        this.items.delete(id);
      }
    return this.items.size;
  }

  // "shade" for the camera image, "id" for the actor id pass (black: hides actors, names none), "sil" hides them.
  use(kind) {
    this.root.visible = kind !== "sil";
    for (const it of this.items.values()) it.mesh.material = kind === "id" ? NONE : it.mat;
  }

  list() {
    return [...this.items.values()].map((it) => it.spec);
  }
}
