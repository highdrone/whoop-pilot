// Capture frame W <-> house frame H (docs/HOME-DRONE.md, "Frames and units").
// W: the splat's own frame (splat files, scene.json, work/room/rooms.auto.json): OpenCV axes (x right, y DOWN,
//    z forward), metres before the tape correction, origin = AprilTag 0.
// H: plan x, y (rooms.json, orthophoto) and z up from the lowest floor, tape-corrected metres.
//    p_H = (f·x_W, f·z_W, f·(Yf − y_W)).  three.js world = (x_H, z_H, −y_H).

import { wrapAngle } from "../util.js";

// site.json scale_correction.factor, trusted only inside 0.9..1.1 (SiteSpec's own rule).
export function siteScale(site) {
  const f = Number(site?.scale_correction?.factor ?? 1);
  return f >= 0.9 && f <= 1.1 ? f : 1;
}

// Yf: the lowest floor's y_W (y points down, so the max floor_y).
export function lowestFloor({ roomsAuto, rooms, scene }, f = 1) {
  if (roomsAuto?.rooms?.length) return { Yf: Math.max(...roomsAuto.rooms.map((r) => r.model.floor_y)), floorSource: "rooms.auto.json" };
  if (rooms?.rooms?.length) return { Yf: Math.max(...rooms.rooms.map((r) => r.model.floor_y)) / f, floorSource: "rooms.json" };
  if (scene?.splat?.bounds_p98) return { Yf: scene.splat.bounds_p98[1], floorSource: "splat p98" };
  return { Yf: 0, floorSource: "none" };
}

// W -> three.js as a column-major Matrix4 for the SplatMesh's parent: 180° about X, uniform scale f, f·Yf up.
// The splat itself stays in W (baking the turn in would need its SH rotated).
export const rootMatrix = (f, Yf) => [f, 0, 0, 0, 0, -f, 0, 0, 0, 0, -f, 0, 0, f * Yf, 0, 1];

export const hToThree = ([x, y, z]) => [x, z, -y];
export const threeToH = ([X, Y, Z]) => [X, -Z, Y];

// f and Yf can be forced (tests, a bare splat with a measured floor).
export function houseFrame({ site, roomsAuto, rooms, scene, f, Yf } = {}) {
  f ??= siteScale(site);
  const floor = Yf === undefined ? lowestFloor({ roomsAuto, rooms, scene }, f) : { Yf, floorSource: "given" };
  Yf = floor.Yf;
  const fY = f * Yf;
  const toH = ([x, y, z]) => [f * x, f * z, fY - f * y];
  // Rotation W -> H is A = [[1,0,0],[0,0,1],[0,-1,0]]; R is row-major, columns = body axes in W.
  const rotToH = (R) => [R[0].slice(), R[2].slice(), R[1].map((v) => -v)];
  return {
    f,
    Yf,
    floorSource: floor.floorSource,
    threeRoot: rootMatrix(f, Yf),
    toH,
    toW: ([x, y, z]) => [x / f, Yf - z / f, y / f],
    rotToH,
    // scene.json camera (rotation = world_from_rig, OpenCV camera axes; position = rig centre) -> pose in H.
    camToH(R, t) {
      const RH = rotToH(R);
      const fwd = [RH[0][2], RH[1][2], RH[2][2]];
      return { p: toH(t), R: RH, yaw: Math.atan2(fwd[1], fwd[0]), pitch: Math.asin(Math.max(-1, Math.min(1, fwd[2]))) };
    },
    // Packed xyz triples in W -> a new Float32Array in H.
    pointsToH(xyz) {
      const out = new Float32Array(xyz.length);
      for (let i = 0; i < xyz.length; i += 3) {
        out[i] = f * xyz[i];
        out[i + 1] = f * xyz[i + 2];
        out[i + 2] = fY - f * xyz[i + 1];
      }
      return out;
    },
    // rooms.json heights are above that room's own floor; add this (floor_y is already ×f there).
    roomZ: (floorY) => fY - floorY,
  };
}

// H yaw ψ (CCW from +x, like sim/drone.js) <-> the controller's est.heading (clockwise-positive, zeroed at boot,
// never wrapped). ref pairs one moment where both are known: { yaw0, heading0 } (the home pad, a fix).
export const yawFromHeading = (heading, { yaw0 = 0, heading0 = 0 } = {}) => wrapAngle(yaw0 - (heading - heading0));
// The heading closest to `near` (usually the current est.heading) so a setpoint never asks for a full turn.
export function headingFromYaw(yaw, { yaw0 = 0, heading0 = 0 } = {}, near = heading0) {
  return near + wrapAngle(heading0 - (yaw - yaw0) - near);
}
