// Simulated apartment: floor plan, furniture and a cat. The demo world, and the interface twin-world.js (a real house)
// also gives the Simulator: rooms, segments, furniture, start, bounds, roomAt, floorAt, ceilingAt, castRay, collide,
// sprites, step, cat.
// Units are meters. World frame: x east, y north, z up. Yaw is CCW from +x.

export const WALL_H = 2.5;

export const ROOMS = [
  { id: "living", name: "living room", x0: 4, y0: 3.5, x1: 10, y1: 8, floor: "wood", wall: [198, 186, 160], style: "stripes" },
  { id: "kitchen", name: "kitchen", x0: 4, y0: 0, x1: 10, y1: 3.5, floor: "tile", wall: [206, 222, 212], style: "tiles" },
  { id: "bedroom", name: "bedroom", x0: 0, y0: 4.5, x1: 4, y1: 8, floor: "carpet", wall: [148, 170, 212], style: "plain" },
  { id: "hall", name: "hallway", x0: 0, y0: 0, x1: 4, y1: 4.5, floor: "planks", wall: [222, 204, 164], style: "plain" },
];

// Wall segments. Gaps are doorways: hall-kitchen (x=4, y 1..2), bedroom-living (x=4, y 6.2..7.2),
// hall-bedroom (y=4.5, x 1..2) and the wide kitchen-living opening (y=3.5, x 5.5..8).
const SEGMENTS = [
  [0, 0, 10, 0], [10, 0, 10, 8], [10, 8, 0, 8], [0, 8, 0, 0],
  [4, 0, 4, 1], [4, 2, 4, 6.2], [4, 7.2, 4, 8],
  [4, 3.5, 5.5, 3.5], [8, 3.5, 10, 3.5],
  [0, 4.5, 1, 4.5], [2, 4.5, 4, 4.5],
];

// Windows / pictures painted on walls: segment index, the room they face, u range along the
// segment (m from its first point) and v range (height).
const DECOR = [
  { seg: 2, room: "living", u0: 1.2, u1: 3.2, v0: 0.9, v1: 2.1, kind: "window" },
  { seg: 0, room: "kitchen", u0: 7.4, u1: 8.9, v0: 1.25, v1: 2.05, kind: "window" },
  { seg: 3, room: "bedroom", u0: 1.0, u1: 2.4, v0: 1.1, v1: 1.8, kind: "picture" },
  { seg: 5, room: "living", u0: 3.0, u1: 3.9, v0: 1.2, v1: 1.9, kind: "picture" },
  { seg: 1, room: "living", u0: 5.3, u1: 6.3, v0: 1.5, v1: 2.2, kind: "picture" },
  { seg: 1, room: "kitchen", u0: 1.2, u1: 2.4, v0: 1.3, v1: 1.8, kind: "clock" },
  { seg: 3, room: "hall", u0: 5.2, u1: 6.4, v0: 0, v1: 2.05, kind: "door" }, // front door
];

// Furniture. label = what an object detector would call it (COCO names), or null.
// w = half width of the billboard, r = collision radius, z0..z1 = vertical extent.
export const FURNITURE = [
  { label: "couch", sprite: "couch", x: 6.1, y: 5.8, w: 0.95, r: 0.45, z0: 0, z1: 0.85 },
  { label: "tv", sprite: "tv", x: 9.8, y: 5.8, w: 0.55, r: 0.2, z0: 0.45, z1: 1.2 },
  { label: null, sprite: "tvstand", x: 9.75, y: 5.8, w: 0.6, r: 0.25, z0: 0, z1: 0.45 },
  { label: "potted plant", sprite: "plant", x: 9.5, y: 7.5, w: 0.3, r: 0.25, z0: 0, z1: 1.15 },
  { label: "chair", sprite: "armchair", x: 7.9, y: 7.4, w: 0.4, r: 0.35, z0: 0, z1: 0.9 },
  { label: "refrigerator", sprite: "fridge", x: 9.55, y: 0.45, w: 0.4, r: 0.38, z0: 0, z1: 1.85 },
  { label: "oven", sprite: "oven", x: 7.9, y: 0.35, w: 0.36, r: 0.32, z0: 0, z1: 0.92 },
  { label: null, sprite: "counter", x: 8.75, y: 0.35, w: 0.4, r: 0.3, z0: 0, z1: 0.92 },
  { label: "sink", sprite: "sink", x: 6.7, y: 0.35, w: 0.45, r: 0.32, z0: 0, z1: 1.1 },
  { label: null, sprite: "counter", x: 5.4, y: 0.35, w: 0.5, r: 0.3, z0: 0, z1: 0.92 },
  { label: "microwave", sprite: "microwave", x: 5.4, y: 0.35, w: 0.25, r: 0.1, z0: 0.92, z1: 1.2 },
  { label: "dining table", sprite: "table", x: 6.4, y: 2.0, w: 0.7, r: 0.5, z0: 0, z1: 0.76 },
  { label: "chair", sprite: "chair", x: 6.4, y: 2.75, w: 0.24, r: 0.2, z0: 0, z1: 0.92 },
  { label: "chair", sprite: "chair", x: 6.4, y: 1.25, w: 0.24, r: 0.2, z0: 0, z1: 0.92 },
  { label: "bed", sprite: "bed", x: 1.6, y: 6.9, w: 1.0, r: 0.8, z0: 0, z1: 0.62 },
  { label: "potted plant", sprite: "plant", x: 3.55, y: 7.55, w: 0.28, r: 0.22, z0: 0, z1: 1.0 },
  { label: "chair", sprite: "chair", x: 3.4, y: 5.1, w: 0.24, r: 0.2, z0: 0, z1: 0.92 },
  { label: "potted plant", sprite: "plant", x: 0.45, y: 0.45, w: 0.3, r: 0.25, z0: 0, z1: 1.2 },
  { label: "backpack", sprite: "backpack", x: 3.55, y: 0.35, w: 0.2, r: 0.15, z0: 0, z1: 0.45 },
];

// Spots where the cat likes to nap (on top of furniture).
const NAP_SPOTS = [
  { x: 6.05, y: 5.55, z: 0.42 }, // couch
  { x: 1.8, y: 6.9, z: 0.6 }, // bed
  { x: 9.2, y: 4.2, z: 0 }, // living room floor
];

export class World {
  kind = "demo";
  start = { x: 7.0, y: 4.3, heading: 90 };
  bounds = { x0: 0, y0: 0, x1: 10, y1: 8 };

  constructor(seed = 1) {
    this.rooms = ROOMS;
    this.furniture = FURNITURE;
    this.segments = SEGMENTS.map(([ax, ay, bx, by], i) => {
      const dx = bx - ax;
      const dy = by - ay;
      const len = Math.hypot(dx, dy);
      return { i, ax, ay, bx, by, dx, dy, len, decor: DECOR.filter((d) => d.seg === i) };
    });
    this.rand = mulberry32(seed);
    this.cat = new Cat(this);
  }

  roomAt(x, y) {
    for (const r of this.rooms) if (x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1) return r;
    return null;
  }
  floorAt() {
    return 0;
  }
  ceilingAt() {
    return WALL_H;
  }

  // Nearest wall hit along a ray. Returns { dist, seg, u } or null.
  castRay(ox, oy, dx, dy, maxDist = 50) {
    let best = maxDist;
    let hit = null;
    let hitT = 0;
    for (const s of this.segments) {
      const denom = dx * s.dy - dy * s.dx;
      if (Math.abs(denom) < 1e-12) continue;
      const wx = s.ax - ox;
      const wy = s.ay - oy;
      const t = (wx * dy - wy * dx) / denom; // along segment, 0..1
      if (t < 0 || t > 1) continue;
      const u = (wx * s.dy - wy * s.dx) / denom; // along ray
      if (u > 1e-6 && u < best) {
        best = u;
        hit = s;
        hitT = t;
      }
    }
    return hit ? { dist: best, seg: hit, u: hitT * hit.len } : null;
  }

  // Push a circle out of walls and furniture. Returns the impact speed (m/s, 0 if none).
  collide(body, radius, height = 0.05) {
    let impact = 0;
    for (const s of this.segments) {
      // closest point on segment
      const t = Math.max(0, Math.min(1, ((body.x - s.ax) * s.dx + (body.y - s.ay) * s.dy) / (s.len * s.len)));
      const px = s.ax + t * s.dx;
      const py = s.ay + t * s.dy;
      impact = Math.max(impact, pushOut(body, px, py, radius));
    }
    for (const f of this.furniture) {
      if (body.z > f.z1 + height || body.z + height < f.z0) continue;
      const d = Math.hypot(body.x - f.x, body.y - f.y);
      if (d < f.r + radius && d > 1e-6) {
        const nx = (body.x - f.x) / d;
        const ny = (body.y - f.y) / d;
        const vn = (body.vx || 0) * nx + (body.vy || 0) * ny;
        body.x = f.x + nx * (f.r + radius);
        body.y = f.y + ny * (f.r + radius);
        if (vn < 0 && body.vx !== undefined) {
          body.vx -= 1.35 * vn * nx;
          body.vy -= 1.35 * vn * ny;
          impact = Math.max(impact, -vn);
        }
      }
    }
    return impact;
  }

  // Everything the camera can see as a billboard: furniture plus the cat.
  sprites() {
    return this.cat ? [...this.furniture, this.cat.sprite()] : this.furniture;
  }

  step(dt, drone) {
    this.cat.update(dt, drone);
  }
}

export function pushOut(body, px, py, radius) {
  const dx = body.x - px;
  const dy = body.y - py;
  const d = Math.hypot(dx, dy);
  if (d >= radius || d < 1e-9) return 0;
  const nx = dx / d;
  const ny = dy / d;
  body.x = px + nx * radius;
  body.y = py + ny * radius;
  if (body.vx === undefined) return 0;
  const vn = body.vx * nx + body.vy * ny;
  if (vn >= 0) return 0;
  body.vx -= 1.35 * vn * nx; // restitution 0.35
  body.vy -= 1.35 * vn * ny;
  return -vn;
}

// A cat with simple moods: napping, wandering, sitting, and fleeing from a drone that gets too close.
class Cat {
  constructor(world) {
    this.world = world;
    this.rand = world.rand;
    const spot = NAP_SPOTS[0];
    Object.assign(this, { x: spot.x, y: spot.y, z: spot.z, vx: 0, vy: 0, heading: Math.PI, state: "nap", timer: 6 + this.rand() * 8 });
    this.target = null;
    this.moving = false;
  }

  sprite() {
    return {
      label: "cat",
      sprite: this.moving ? "catwalk" : "cat",
      x: this.x,
      y: this.y,
      w: 0.24,
      r: 0.15,
      z0: this.z,
      z1: this.z + (this.moving ? 0.28 : 0.3),
      heading: this.heading,
      depthBias: this.z > 0 ? 0.35 : 0, // draw in front of the couch / bed it lies on
    };
  }

  pickTarget() {
    const rooms = this.world.rooms.filter((r) => r.id === "living" || r.id === "kitchen" || (this.rand() < 0.3 && r.id !== "hall"));
    const r = rooms[Math.floor(this.rand() * rooms.length)];
    for (let i = 0; i < 20; i++) {
      const x = r.x0 + 0.5 + this.rand() * (r.x1 - r.x0 - 1);
      const y = r.y0 + 0.5 + this.rand() * (r.y1 - r.y0 - 1);
      if (this.world.furniture.every((f) => Math.hypot(f.x - x, f.y - y) > f.r + 0.3)) return { x, y };
    }
    return { x: 8, y: 4.5 };
  }

  update(dt, drone) {
    const d = drone ? Math.hypot(drone.x - this.x, drone.y - this.y) : 99;
    const threat = drone && drone.airborne && d < 0.75 && drone.z < 1.4;
    if (threat && this.state !== "flee") {
      this.state = "flee";
      this.timer = 1.2 + this.rand() * 1.2;
      this.heading = Math.atan2(this.y - drone.y, this.x - drone.x) + (this.rand() - 0.5) * 0.8;
      this.z = 0;
    }
    let speed = 0;
    switch (this.state) {
      case "nap":
        this.timer -= dt * (drone && drone.airborne && d < 1.6 ? 4 : 1); // a buzzing drone nearby wakes it up sooner
        if (this.timer <= 0) this.startWandering();
        break;
      case "sit":
        this.timer -= dt;
        if (this.timer <= 0) {
          if (this.rand() < 0.25) {
            const spot = NAP_SPOTS[Math.floor(this.rand() * NAP_SPOTS.length)];
            this.target = { x: spot.x, y: spot.y, napZ: spot.z };
            this.state = "walk";
          } else this.startWandering();
        }
        break;
      case "walk": {
        const dx = this.target.x - this.x;
        const dy = this.target.y - this.y;
        const dist = Math.hypot(dx, dy);
        this.heading = Math.atan2(dy, dx);
        speed = 0.38;
        // Nap spots on furniture: the cat hops up once it is next to it.
        if (dist < 0.12 || (this.target.napZ > 0 && dist < 1.2)) {
          if (this.target.napZ !== undefined) {
            Object.assign(this, { x: this.target.x, y: this.target.y, z: this.target.napZ, state: "nap", timer: 15 + this.rand() * 25 });
          } else {
            this.state = "sit";
            this.timer = 2 + this.rand() * 6;
          }
          speed = 0;
        }
        break;
      }
      case "flee":
        speed = 1.5;
        this.timer -= dt;
        if (this.timer <= 0) {
          this.state = "sit";
          this.timer = 3 + this.rand() * 4;
        }
        break;
    }
    this.vx = Math.cos(this.heading) * speed;
    this.vy = Math.sin(this.heading) * speed;
    this.moving = speed > 0;
    if (speed > 0) {
      this.x += this.vx * dt;
      this.y += this.vy * dt;
      const before = this.heading;
      if (this.world.collide(this, 0.16, 0.3) > 0 && this.state === "flee") this.heading = before + Math.PI / 2;
      if (this.state === "walk" && this.stuck(dt)) this.startWandering();
    }
  }

  startWandering() {
    this.z = 0;
    this.target = this.pickTarget();
    this.state = "walk";
    this.progress = { x: this.x, y: this.y, t: 0 };
  }

  // True if the cat has barely moved for a couple of seconds (blocked by furniture).
  stuck(dt) {
    const p = this.progress || (this.progress = { x: this.x, y: this.y, t: 0 });
    p.t += dt;
    if (p.t < 2.5) return false;
    const moved = Math.hypot(this.x - p.x, this.y - p.y);
    Object.assign(p, { x: this.x, y: this.y, t: 0 });
    return moved < 0.2;
  }
}

export function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
