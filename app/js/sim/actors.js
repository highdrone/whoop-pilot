// People and pets for the house simulator (twin-world.js). They walk between rooms on planner paths (house/planner.js
// at hip height, so through the doorways and around the furniture the map knows), stand about, sit (people on the
// plan's sofas and chairs, facing out), lie down and nap (pets on the floor or a sofa), and the cat bolts from a drone
// that comes close. One seed gives the same day every time. Their positions feed the raycast camera (sprites()) and
// the splat twin (twinSpecs() -> twin.setActors(), twin/actors.js specs).
import { plan } from "../house/planner.js";
import { mulberry32 } from "./world.js";

export const SEAT = 0.42; // m above the floor: sofa and chair seats

export const CAST = [
  { id: "person1", kind: "person" },
  { id: "person2", kind: "person", color: { shirt: 0xb5452f, pants: 0x3b3b40, hair: 0x6b4423, skin: 0xc68e6a } },
  { id: "cat", kind: "cat" },
  { id: "dog", kind: "dog" },
];

// speed m/s, collision radius, planning altitude band and σ (the planner keeps 0.05 + σ from obstacles), stride m,
// sprite half width and height per pose.
const KIND = {
  person: { speed: 0.8, radius: 0.2, alt: 0.6, sigma: 0.15, stride: 1.3,
    look: { stand: ["person", 0.26, 1.72], walk: ["personwalk", 0.3, 1.72], sit: ["personsit", 0.3, 1.25], lie: ["personlie", 0.9, 0.32] } },
  cat: { speed: 0.38, radius: 0.15, alt: 0.6, sigma: 0.08, stride: 0.45,
    look: { stand: ["cat", 0.24, 0.3], walk: ["catwalk", 0.24, 0.28], sit: ["cat", 0.24, 0.3], lie: ["cat", 0.24, 0.22] } },
  dog: { speed: 0.65, radius: 0.22, alt: 0.6, sigma: 0.12, stride: 0.8,
    look: { stand: ["dog", 0.36, 0.6], walk: ["dogwalk", 0.38, 0.58], sit: ["dog", 0.34, 0.62], lie: ["doglie", 0.4, 0.3] } },
};
const POSE = { stand: "stand", walk: "walk", flee: "walk", sit: "sit", lie: "lie", nap: "lie" };

export class HouseActors {
  constructor(world, { seed = 1, cast = CAST } = {}) {
    this.list = cast.map((spec, i) => new Actor(world, spec, mulberry32((seed * 2654435761 + (i + 1) * 40503) >>> 0)));
    this.acc = 0;
  }
  get cat() {
    return this.list.find((a) => a.kind === "cat") ?? null;
  }
  // Called per physics substep; people and pets move at 60 Hz.
  step(dt, drone) {
    if ((this.acc += dt) < 1 / 60) return;
    for (const a of this.list) a.update(this.acc, drone);
    this.acc = 0;
  }
  sprites() {
    return this.list.map((a) => a.sprite());
  }
  twinSpecs() {
    return this.list.map((a) => a.spec());
  }
}

class Actor {
  constructor(world, spec, rand) {
    Object.assign(this, { world, map: world.map, rand, id: spec.id, kind: spec.kind, color: spec.color, P: KIND[spec.kind] });
    Object.assign(this, { yaw: rand() * 2 * Math.PI, phase: 0, path: null, goal: null, seat: null, onTop: 0, lastFloor: 0 });
    [this.x, this.y] = this.spot();
    this.z = this.floor();
    this.state = "stand";
    this.timer = 2 + rand() * 6;
  }

  // The demo cat's names, so scenarios can set and read either world's cat the same way.
  get heading() {
    return this.yaw;
  }
  set heading(v) {
    this.yaw = v;
  }
  get pose() {
    return POSE[this.state] ?? "stand";
  }
  get moving() {
    return this.state === "walk" || this.state === "flee";
  }

  floor() {
    return (this.lastFloor = this.map.floorAt(this.x, this.y) ?? this.lastFloor);
  }

  // A random free spot (for this one's size, at hip height) in a random room.
  spot() {
    const { map } = this, cl = map.clear[map.bandOf(this.P.alt)], need = map.lethal(this.P.sigma) + 0.05;
    for (let i = 0; i < 80; i++) {
      const rm = map.rooms[Math.floor(this.rand() * map.rooms.length)], o = rm.outline;
      const xs = o.map((p) => p[0]), ys = o.map((p) => p[1]), [x0, y0] = [Math.min(...xs), Math.min(...ys)];
      const x = x0 + this.rand() * (Math.max(...xs) - x0), y = y0 + this.rand() * (Math.max(...ys) - y0);
      const k = map.idx(x, y);
      if (k >= 0 && map.room[k] === rm.index && cl[k] >= need) return [x, y];
    }
    return [this.x ?? map.x0, this.y ?? map.y0];
  }

  update(dt, drone) {
    const d = drone ? Math.hypot(drone.x - this.x, drone.y - this.y) : 99;
    if (this.kind === "cat" && this.state !== "flee" && drone?.airborne && d < 0.75 && drone.z - this.floor() < 1.4) {
      this.getUp();
      const away = Math.atan2(this.y - drone.y, this.x - drone.x) + (this.rand() - 0.5) * 0.8;
      Object.assign(this, { state: "flee", path: null, timer: 1.2 + this.rand() * 1.2, yaw: away });
    }
    if (this.state === "walk") this.follow(dt);
    else if (this.state === "flee") this.flee(dt);
    else if ((this.timer -= dt * (this.state === "nap" && drone?.airborne && d < 1.6 ? 4 : 1)) <= 0) this.next();
    this.z = this.floor() + this.onTop;
  }

  // What to do next: people sit on a seat, lie on a sofa or stand somewhere; the cat naps on a sofa or the floor or
  // sits; the dog sits or lies down. Each one walks there on a planned path.
  next() {
    this.getUp();
    const r = this.rand(), items = this.world.items, seats = items.filter((i) => i.seat), sofas = seats.filter((i) => i.label === "couch");
    const pick = (a) => a[Math.floor(this.rand() * a.length)];
    let goal;
    if (this.kind === "person")
      goal = r < 0.4 && seats.length ? { seat: pick(seats), then: "sit" }
        : r < 0.48 && sofas.length ? { seat: pick(sofas), then: "lie" } : { then: "stand" };
    else if (this.kind === "cat") goal = r < 0.25 && sofas.length ? { seat: pick(sofas), then: "nap" } : { then: r < 0.65 ? "sit" : "nap" };
    else goal = { then: r < 0.5 ? "sit" : "lie" };
    this.goTo(goal);
  }

  // Put it somewhere (tests, scenarios): sitting there for `timer` seconds.
  place(x, y, state = "sit", timer = 60) {
    Object.assign(this, { x, y, state, timer, path: null, seat: null, onTop: 0 });
    this.z = this.floor();
  }

  startWandering() {
    this.getUp();
    return this.goTo({ then: "sit" });
  }

  goTo(goal) {
    const to = goal.seat ? [goal.seat.x, goal.seat.y] : this.spot();
    const p = plan(this.map, [this.x, this.y], to, { alt: this.P.alt, sigma: this.P.sigma, climb: false, snap: 1.0 });
    if (!p.ok || this.world.blocked(this.x, this.y, p.path[0][0], p.path[0][1])) {
      Object.assign(this, { state: "stand", timer: 1 + this.rand() * 2 });
      return false;
    }
    Object.assign(this, { path: p.path, i: 0, goal, state: "walk" });
    return true;
  }

  follow(dt) {
    let step = this.P.speed * dt;
    while (step > 0 && this.path) {
      const [tx, ty] = this.path[this.i], dx = tx - this.x, dy = ty - this.y, L = Math.hypot(dx, dy);
      if (L > 1e-6) this.yaw = Math.atan2(dy, dx);
      if (L > step) {
        this.x += (dx / L) * step;
        this.y += (dy / L) * step;
        break;
      }
      [this.x, this.y, step] = [tx, ty, step - L];
      if (++this.i >= this.path.length) this.arrive();
    }
    this.phase = (this.phase + ((this.P.speed * dt) / this.P.stride) * 2 * Math.PI) % (2 * Math.PI);
  }

  // At a seat: a person sits at the front of it facing out; lying and napping are on top of it, along it.
  arrive() {
    const g = this.goal, s = g.seat;
    this.path = null;
    if (s) {
      const dx = this.x - s.x, dy = this.y - s.y, L = Math.hypot(dx, dy) || 1;
      this.seat = { item: s, from: [this.x, this.y] };
      if (g.then === "sit") Object.assign(this, { x: s.x + (dx / L) * 0.2, y: s.y + (dy / L) * 0.2, yaw: Math.atan2(dy, dx) });
      else Object.assign(this, { x: s.x, y: s.y, yaw: s.axis + (this.rand() < 0.5 ? 0 : Math.PI), onTop: SEAT });
    }
    const [lo, hi] = { stand: [4, 20], sit: [this.kind === "person" ? 20 : 3, 60], lie: [20, 80], nap: [15, 60] }[g.then];
    Object.assign(this, { state: g.then, timer: lo + this.rand() * (hi - lo) });
  }

  getUp() {
    if (!this.seat) return;
    [this.x, this.y] = this.seat.from;
    Object.assign(this, { seat: null, onTop: 0 });
  }

  // Straight away from the drone, bouncing off walls and furniture, then sit.
  flee(dt) {
    const v = 1.5, body = { x: this.x + Math.cos(this.yaw) * v * dt, y: this.y + Math.sin(this.yaw) * v * dt, z: this.floor() + 0.35 };
    if (this.world.collide(body, this.P.radius, 0.05) > 0 || Math.hypot(body.x - this.x, body.y - this.y) < v * dt * 0.5) this.yaw += Math.PI / 2;
    [this.x, this.y] = [body.x, body.y];
    this.phase = (this.phase + ((v * dt) / this.P.stride) * 2 * Math.PI) % (2 * Math.PI);
    if ((this.timer -= dt) <= 0) Object.assign(this, { state: "sit", timer: 3 + this.rand() * 4 });
  }

  sprite() {
    const [sprite, w, h] = this.P.look[this.pose];
    return { label: this.kind, sprite, x: this.x, y: this.y, w, r: this.P.radius, z0: this.z, z1: this.z + h,
      heading: this.kind === "person" && !this.moving ? undefined : this.yaw, depthBias: this.seat ? 0.8 : 0, actor: this.id };
  }

  spec() {
    const { id, kind, x, y, z, yaw, pose, phase, color } = this;
    return { id, kind, x, y, z, yaw, pose, phase, ...(color && { color }) };
  }
}
