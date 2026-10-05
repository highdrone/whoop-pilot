// Drone poses and cameras for the twin. Pure math, row-major 3x3 arrays. Frames (W capture, H house, three.js world
// = (x_H, z_H, -y_H)) come from house/frames.js, per docs/HOME-DRONE.md.
// Drone pose in H: { x, y, z, yaw (rad, CCW from +x), pitch (rad, + nose down), roll (rad, + right side down) },
// the same convention as sim/drone.js. Body frame: x forward, y left, z up. The camera looks along body x,
// pitched up by the lens uptilt.
import { DEG } from "./lens.js";
import { hToThree } from "../house/frames.js";

export const mul3 = (A, B) => A.map((row) => [0, 1, 2].map((j) => row[0] * B[0][j] + row[1] * B[1][j] + row[2] * B[2][j]));
const T3 = (A) => [0, 1, 2].map((i) => [A[0][i], A[1][i], A[2][i]]);
const rotX = (t) => [[1, 0, 0], [0, Math.cos(t), -Math.sin(t)], [0, Math.sin(t), Math.cos(t)]];
const rotY = (t) => [[Math.cos(t), 0, Math.sin(t)], [0, 1, 0], [-Math.sin(t), 0, Math.cos(t)]];
const rotZ = (t) => [[Math.cos(t), -Math.sin(t), 0], [Math.sin(t), Math.cos(t), 0], [0, 0, 1]];

// H <- body (FLU)
export const bodyRotation = ({ yaw = 0, pitch = 0, roll = 0 }) => mul3(mul3(rotZ(yaw), rotY(pitch)), rotX(roll));
const BODY_FROM_CV = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]; // columns: OpenCV camera axes in the body frame
const THREE_FROM_H = [[1, 0, 0], [0, 0, 1], [0, -1, 0]];

// Camera of a drone pose: position and rotation (columns = OpenCV camera axes) in H and in three.js world.
export function droneCamera(pose, uptiltDeg = 0) {
  const R = mul3(mul3(bodyRotation(pose), rotY(-uptiltDeg * DEG)), BODY_FROM_CV);
  const p = [pose.x, pose.y, pose.z];
  return { p, R, pThree: hToThree(p), RThree: mul3(THREE_FROM_H, R) };
}

// H pose of a scene.json camera (world_from_cam rotation, OpenCV, centre in W) under a houseFrame(): body forward
// = optical axis, so it renders exactly with uptiltDeg 0.
export function poseFromCapture(cam, hf) {
  const R = mul3(hf.rotToH(cam.rotation), T3(BODY_FROM_CV));
  const [x, y, z] = hf.toH(cam.position);
  return { x, y, z, yaw: Math.atan2(R[1][0], R[0][0]), pitch: Math.asin(Math.max(-1, Math.min(1, -R[2][0]))), roll: Math.atan2(R[2][1], R[2][2]) };
}
