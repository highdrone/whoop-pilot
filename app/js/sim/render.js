// Fisheye FPV camera for the simulator: a column raycaster with per-pixel floor, ceiling and wall
// texturing. Angles map linearly to pixels in both axes (equidistant fisheye), which matches how
// the app converts image positions back into bearings on the real camera.
// A world with castAll() (twin-world.js, a real house) is drawn in layers: each column walks every face its ray crosses
// up to the first full-height wall, so door headers, furniture boxes (top, sides, underside) and floors at different
// heights show; the demo apartment keeps the single-wall path.
import { WALL_H } from "./world.js";
import { paintSprites } from "./sprites.js";

const DEG = Math.PI / 180;
const FAR = 30;

export class CameraRenderer {
  constructor(world, { width = 320, height = 240, hfovDeg = 127 } = {}) {
    this.world = world;
    this.W = width;
    this.H = height;
    this.setFov(hfovDeg);
    this.frame = document.createElement("canvas");
    this.frame.width = width;
    this.frame.height = height;
    this.fctx = this.frame.getContext("2d");
    this.image = this.fctx.createImageData(width, height);
    this.canvas = document.createElement("canvas"); // final output with roll applied
    this.canvas.width = width;
    this.canvas.height = height;
    this.ctx = this.canvas.getContext("2d");
    this.zpix = new Float32Array(width * height); // horizontal distance of what each pixel shows
    this.idbuf = new Int16Array(width * height);
    this.tanE = new Float32Array(height);
    this.textures = paintSprites();
    this.noiseSeed = 12345;
    this.noise = 0.05; // sensor/compression noise (0.09 looks like analog video)
    this.flip = new Map();
  }

  setFov(hfovDeg) {
    this.hfov = hfovDeg * DEG;
    this.vfov = (this.hfov * this.H) / this.W;
  }

  // cam: { x, y, z, yaw (rad, CCW), pitch (rad, optical axis above horizontal), roll (rad, right side down) }
  render(cam) {
    const { W, H, world, idbuf, tanE } = this;
    const px = this.image.data;
    idbuf.fill(0);
    for (let r = 0; r < H; r++) {
      const e = cam.pitch + (0.5 - (r + 0.5) / H) * this.vfov;
      tanE[r] = Math.tan(Math.max(-1.55, Math.min(1.55, e)));
    }
    const sprites = world.sprites();
    this.noiseSeed = (this.noiseSeed * 1103515245 + 12345) >>> 0;
    const shade = { seed: this.noiseSeed, glitch: Math.random() < 0.03 ? Math.floor(Math.random() * H) : -1 };
    if (world.castAll) this.layered(cam, px, shade, sprites.length);
    else this.flat(cam, px, shade);

    const boxes = this.drawSprites(cam, px, sprites);
    this.fctx.putImageData(this.image, 0, 0);

    // Roll the image like the real camera would: body roll turns the picture about the point
    // straight ahead on the horizon (below center, since the camera is tilted up).
    const g = this.ctx;
    const py = H * (0.5 + cam.pitch / this.vfov);
    g.save();
    g.fillStyle = "#000";
    g.fillRect(0, 0, W, H);
    g.translate(W / 2, py);
    g.rotate(-cam.roll);
    g.scale(1.12, 1.12);
    g.drawImage(this.frame, -W / 2, -py);
    g.restore();
    return { canvas: this.canvas, detections: boxes };
  }

  pixel(px, i, r, fog, shade) {
    shade.seed = (shade.seed * 1664525 + 1013904223) >>> 0;
    const n = this.noise ? ((shade.seed >>> 24) - 128) * this.noise + (r === shade.glitch ? 30 : 0) : 0;
    px[i] = OR * fog + n;
    px[i + 1] = OG * fog + n;
    px[i + 2] = OB * fog + n;
    px[i + 3] = 255;
  }

  // The demo apartment: one full-height wall per column, flat floor and ceiling.
  flat(cam, px, shade) {
    const { W, H, world, zpix, tanE } = this;
    for (let c = 0; c < W; c++) {
      const a = cam.yaw + (0.5 - (c + 0.5) / W) * this.hfov;
      const ca = Math.cos(a);
      const sa = Math.sin(a);
      const hit = world.castRay(cam.x, cam.y, ca, sa);
      const d = hit ? hit.dist : FAR;
      const hx = cam.x + ca * (d - 0.01);
      const hy = cam.y + sa * (d - 0.01);
      const room = world.roomAt(hx, hy) || world.rooms[0];
      const decor = hit ? hit.seg.decor.filter((k) => k.room === room.id) : [];
      const u = hit ? hit.u : 0;
      for (let r = 0; r < H; r++) {
        const t = tanE[r];
        const h = cam.z + d * t;
        let dist;
        if (h >= 0 && h <= WALL_H) {
          wallColor(room, decor, u, h);
          dist = d;
        } else if (h > WALL_H) {
          dist = (WALL_H - cam.z) / t;
          ceilingColor(world, cam.x + ca * dist, cam.y + sa * dist);
        } else {
          dist = cam.z / -t;
          floorColor(world, cam.x + ca * dist, cam.y + sa * dist);
        }
        zpix[r * W + c] = d; // sprites are hidden by walls only, as they always were here
        this.pixel(px, (r * W + c) * 4, r, 1 / (1 + 0.07 * dist), shade);
      }
    }
  }

  // A house: per column the faces along the ray, nearest first, and the region between each pair (room floor and
  // ceiling, or the top or underside of the furniture box the ray is over or under). Pixels of labelled furniture get
  // ids after the sprites' for the ground-truth boxes.
  layered(cam, px, shade, nSprites) {
    const { W, H, world, zpix, idbuf, tanE } = this;
    const regs = [];
    for (let c = 0; c < W; c++) {
      const a = cam.yaw + (0.5 - (c + 0.5) / W) * this.hfov;
      const ca = Math.cos(a);
      const sa = Math.sin(a);
      const hits = world.castAll(cam.x, cam.y, ca, sa, FAR);
      const n = hits.length;
      for (let k = 0; k <= n; k++) {
        const d0 = k ? hits[k - 1].dist : 0, d1 = k < n ? hits[k].dist : FAR, m = (d0 + d1) / 2;
        regs[k] = world.regionAt(cam.x + ca * m, cam.y + sa * m);
      }
      for (let r = 0; r < H; r++) {
        const t = tanE[r];
        let dist = FAR, id = 0;
        rgb(18, 18, 20);
        for (let k = 0; k <= n; k++) {
          const g = regs[k], d0 = k ? hits[k - 1].dist : 0, d1 = k < n ? hits[k].dist : FAR, h0 = cam.z + d0 * t, box = g.box;
          const top = box && h0 >= box.zMax - 1e-6, under = box && h0 <= box.zMin + 1e-6;
          const F = top ? box.zMax : g.floor, C = under ? box.zMin : g.ceil;
          if (t < 0 && cam.z > F && (F - cam.z) / t < d1) {
            dist = (F - cam.z) / t;
            const x = cam.x + ca * dist, y = cam.y + sa * dist;
            if (top) (itemColor(box, x, y, 1), (id = box.item ? nSprites + box.item.index + 1 : 0));
            else floorColor(world, x, y);
            break;
          }
          if (t > 0 && cam.z < C && (C - cam.z) / t < d1) {
            dist = (C - cam.z) / t;
            if (under) (itemColor(box, 0, 0, 0.6), (id = box.item ? nSprites + box.item.index + 1 : 0));
            else ceilingColor(world, cam.x + ca * dist, cam.y + sa * dist);
            break;
          }
          if (k === n) break;
          const s = hits[k].seg, h = cam.z + d1 * t;
          if (h < s.zMin || h > s.zMax) continue;
          dist = d1;
          if (s.item) (itemColor(s.item, hits[k].u, h, s.shade), (id = s.item.label ? nSprites + s.item.index + 1 : 0));
          else if (s.kind === "riser") rgb(150, 112, 74);
          else if (s.kind === "fan") rgb(232, 230, 222);
          else {
            const room = g.room || world.rooms[0];
            wallColor(room, s.decor.length ? s.decor.filter((d) => d.room === room.id) : s.decor, hits[k].u, h - room.floorZ);
          }
          break;
        }
        const j = r * W + c;
        zpix[j] = dist;
        idbuf[j] = id;
        this.pixel(px, j * 4, r, 1 / (1 + 0.07 * dist), shade);
      }
    }
  }

  drawSprites(cam, px, sprites) {
    const { W, H, zpix, idbuf } = this;
    const list = [];
    for (let k = 0; k < sprites.length; k++) {
      const s = sprites[k];
      const dx = s.x - cam.x;
      const dy = s.y - cam.y;
      const dist = Math.hypot(dx, dy);
      if (dist < 0.12) continue;
      const rel = wrap(Math.atan2(dy, dx) - cam.yaw);
      const half = Math.atan2(s.w, dist);
      if (Math.abs(rel) - half > this.hfov / 2) continue;
      const cc = (0.5 - rel / this.hfov) * W;
      const hp = (half / this.hfov) * W;
      const rTop = (0.5 - (Math.atan2(s.z1 - cam.z, dist) - cam.pitch) / this.vfov) * H;
      const rBot = (0.5 - (Math.atan2(s.z0 - cam.z, dist) - cam.pitch) / this.vfov) * H;
      list.push({ s, k, dist, sort: dist - (s.depthBias || 0), cL: cc - hp, cR: cc + hp, rTop, rBot });
    }
    list.sort((a, b) => b.sort - a.sort);

    const drawn = new Int32Array(sprites.length);
    for (const it of list) {
      const { s, k, dist, cL, cR, rTop, rBot } = it;
      const tex = this.textures[s.sprite];
      if (!tex) continue;
      let flip = false;
      if (s.heading !== undefined) {
        // Face the direction the cat is walking, as seen from the camera.
        const side = Math.cos(s.heading) * Math.sin(cam.yaw) - Math.sin(s.heading) * Math.cos(cam.yaw);
        flip = Math.abs(side) > 0.2 ? side < 0 : this.flip.get(k) || false;
        this.flip.set(k, flip);
      }
      const fog = 1 / (1 + 0.07 * dist);
      const c0 = Math.max(0, Math.ceil(cL));
      const c1 = Math.min(W - 1, Math.floor(cR));
      const r0 = Math.max(0, Math.ceil(rTop));
      const r1 = Math.min(H - 1, Math.floor(rBot));
      const near = dist - (this.world.castAll ? s.depthBias || 0 : 0); // a house's pets and people on furniture boxes
      for (let c = c0; c <= c1; c++) {
        let u = (c + 0.5 - cL) / (cR - cL);
        if (flip) u = 1 - u;
        const tx = Math.min(tex.w - 1, (u * tex.w) | 0);
        for (let r = r0; r <= r1; r++) {
          if (near >= zpix[r * W + c] + 0.02) continue;
          const v = (r + 0.5 - rTop) / (rBot - rTop);
          const ty = Math.min(tex.h - 1, (v * tex.h) | 0);
          const ti = (ty * tex.w + tx) * 4;
          if (tex.data[ti + 3] < 128) continue;
          const i = (r * W + c) * 4;
          px[i] = tex.data[ti] * fog;
          px[i + 1] = tex.data[ti + 1] * fog;
          px[i + 2] = tex.data[ti + 2] * fog;
          idbuf[r * W + c] = k + 1;
          drawn[k]++;
        }
      }
    }

    // Ground-truth "detections" from what is actually visible in the frame: sprites, then a house's labelled furniture.
    const items = this.world.castAll ? this.world.items : [];
    const all = items.length ? [...sprites, ...items] : sprites;
    const n = all.length;
    const minX = new Int32Array(n).fill(W);
    const minY = new Int32Array(n).fill(H);
    const maxX = new Int32Array(n).fill(-1);
    const maxY = new Int32Array(n).fill(-1);
    const seen = new Int32Array(n);
    for (let r = 0; r < H; r++) {
      for (let c = 0; c < W; c++) {
        const id = idbuf[r * W + c] - 1;
        if (id < 0) continue;
        seen[id]++;
        if (c < minX[id]) minX[id] = c;
        if (c > maxX[id]) maxX[id] = c;
        if (r < minY[id]) minY[id] = r;
        if (r > maxY[id]) maxY[id] = r;
      }
    }
    const out = [];
    for (let k = 0; k < n; k++) {
      const s = all[k];
      if (!s.label || seen[k] < 25 || (k < sprites.length && seen[k] < drawn[k] * 0.3)) continue;
      out.push({
        label: s.label,
        score: Math.min(0.97, 0.62 + 0.3 * Math.min(1, seen[k] / 1500)),
        box: { x: minX[k] / W, y: minY[k] / H, w: (maxX[k] - minX[k] + 1) / W, h: (maxY[k] - minY[k] + 1) / H },
        dist: Math.hypot(s.x - cam.x, s.y - cam.y),
      });
    }
    return out;
  }
}

function wrap(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

// Texture coordinates are world metres, negative in a house west or south of its origin: % would keep the sign.
const mod = (v, p) => v - Math.floor(v / p) * p;

function hash2(x, y) {
  let h = (x * 374761393 + y * 668265263) | 0;
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}

// Color functions write their result here to avoid allocating an array per pixel.
let OR = 0;
let OG = 0;
let OB = 0;
function rgb(r, g, b) {
  OR = r;
  OG = g;
  OB = b;
}

// Furniture from the house map: its colour, a little grain, and the face's shade.
function itemColor(item, a, b, k) {
  const [r, g, bl] = item.color, f = k * (0.9 + 0.1 * hash2(Math.floor(a * 25), Math.floor(b * 25)));
  rgb(r * f, g * f, bl * f);
}

function wallColor(room, decor, u, v) {
  for (const d of decor) {
    if (u < d.u0 || u > d.u1 || v < d.v0 || v > d.v1) continue;
    const edge = Math.min(u - d.u0, d.u1 - u, v - d.v0, d.v1 - v);
    if (d.kind === "window") {
      if (edge < 0.05 || Math.abs(u - (d.u0 + d.u1) / 2) < 0.025) return rgb(244, 244, 240);
      const k = (v - d.v0) / (d.v1 - d.v0);
      return rgb(120 + 70 * k, 175 + 45 * k, 230);
    }
    if (d.kind === "door") {
      if (edge < 0.06) return rgb(235, 232, 225);
      if (Math.abs(u - (d.u1 - 0.12)) < 0.03 && Math.abs(v - 1.0) < 0.05) return rgb(210, 180, 60);
      return rgb(120, 78, 48);
    }
    if (d.kind === "clock") {
      const cu = (d.u0 + d.u1) / 2;
      const cv = (d.v0 + d.v1) / 2;
      const rr = Math.hypot(u - cu, v - cv);
      if (rr > 0.24) break;
      if (rr > 0.21) return rgb(40, 40, 40);
      if ((Math.abs(u - cu) < 0.012 && v > cv && v - cv < 0.16) || (Math.abs(v - cv) < 0.012 && u > cu && u - cu < 0.12)) return rgb(30, 30, 30);
      return rgb(250, 250, 245);
    }
    if (edge < 0.05) return rgb(92, 64, 40); // picture frame
    const q = hash2(Math.floor(u * 6), Math.floor(v * 6));
    return rgb(80 + 170 * q, 120 + 100 * (1 - q), 90 + 140 * hash2(Math.floor(v * 5), 7));
  }
  if (v < 0.09) return rgb(236, 232, 224); // baseboard
  const [r, g, b] = room.wall;
  if (room.style === "stripes") {
    const f = Math.floor(u / 0.17) % 2 ? 1 : 0.9;
    return rgb(r * f, g * f, b * f);
  }
  if (room.style === "tiles" && v > 0.92 && v < 1.55) {
    const gu = mod(u, 0.15);
    const gv = mod(v - 0.92, 0.15);
    if (gu < 0.012 || gv < 0.012) return rgb(168, 168, 160);
    const alt = (Math.floor(u / 0.15) + Math.floor((v - 0.92) / 0.15)) % 2;
    return alt ? rgb(232, 242, 250) : rgb(200, 220, 238);
  }
  const f = 0.93 + 0.07 * hash2(Math.floor(u * 18), Math.floor(v * 18));
  return rgb(r * f, g * f, b * f);
}

function floorColor(world, x, y) {
  const room = world.roomAt(x, y);
  if (!room) return rgb(60, 60, 60);
  if (room.id === "living" && x > 6.8 && x < 8.8 && y > 4.6 && y < 6.9) {
    // rug
    const edge = Math.min(x - 6.8, 8.8 - x, y - 4.6, 6.9 - y);
    if (edge < 0.12) return rgb(230, 200, 120);
    return (Math.floor(x / 0.25) + Math.floor(y / 0.25)) % 2 ? rgb(150, 48, 60) : rgb(120, 36, 52);
  }
  switch (room.floor) {
    case "tile": {
      const gx = mod(x, 0.3);
      const gy = mod(y, 0.3);
      if (gx < 0.015 || gy < 0.015) return rgb(140, 140, 136);
      return (Math.floor(x / 0.3) + Math.floor(y / 0.3)) % 2 ? rgb(226, 226, 220) : rgb(178, 184, 192);
    }
    case "carpet": {
      const f = 0.86 + 0.14 * hash2(Math.floor(x * 30), Math.floor(y * 30));
      return rgb(118 * f, 104 * f, 150 * f);
    }
    case "planks": {
      const row = Math.floor(x / 0.16);
      const plank = Math.floor((y + hash2(row, 3) * 1.4) / 1.2);
      if (mod(x, 0.16) < 0.01) return rgb(95, 70, 45);
      const f = 0.82 + 0.18 * hash2(row, plank);
      return rgb(178 * f, 142 * f, 102 * f);
    }
    default: {
      const row = Math.floor(y / 0.16);
      const plank = Math.floor((x + hash2(row, 9) * 1.4) / 1.2);
      if (mod(y, 0.16) < 0.01) return rgb(70, 46, 30);
      const f = 0.8 + 0.2 * hash2(row, plank);
      return rgb(150 * f, 104 * f, 68 * f);
    }
  }
}

function ceilingColor(world, x, y) {
  const room = world.roomAt(x, y);
  if (room) {
    const cx = (room.x0 + room.x1) / 2;
    const cy = (room.y0 + room.y1) / 2;
    if (Math.hypot(x - cx, y - cy) < 0.22) return rgb(255, 252, 235);
  }
  return rgb(232, 230, 224);
}
