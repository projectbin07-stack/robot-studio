// Depth camera (a lower-priority extra): a coarse point cloud from rays
// cast through a pinhole grid at the camera link, against the floor and
// the obstacles extruded to their heights. No noise, no lens model, no
// image: it shows what the camera's field of view covers, including
// obstacles below the LiDAR's scan plane.

import { raySegmentDistance } from "../../../vendor/lidar-slam-lab/src/geometry.js";
import { mul3, mulVec, rpyToMatrix } from "../../../vendor/urdf-inspector/src/core/math.js";
import { obstacles } from "./world.js";

/** @typedef {import("./se2.js").Pose2} Pose2 */
/** @typedef {import("./world.js").SimWorld} SimWorld */
/** @typedef {import("./robot.js").SimSensor} SimSensor */
/** @typedef {import("./settings.js").DepthSettings} DepthSettings */

/**
 * Points (world frame) the camera sees. The camera looks along its link's
 * +x axis (REP 103 body convention, not the optical frame).
 * @param {SimWorld} world
 * @param {Pose2} base true base pose
 * @param {number} baseHeight
 * @param {SimSensor} sensor
 * @param {DepthSettings} s
 * @returns {{ points: Float32Array, count: number }}
 */
export function depthCloud(world, base, baseHeight, sensor, s) {
  const { segments, heights } = obstacles(world);
  const R = mul3(rpyToMatrix([0, 0, base.theta]), sensor.mount.R);
  const c = Math.cos(base.theta);
  const sn = Math.sin(base.theta);
  const m = sensor.mount.p;
  const ox = base.x + c * m[0] - sn * m[1];
  const oy = base.y + sn * m[0] + c * m[1];
  const oz = baseHeight + m[2];
  const th = Math.tan((s.hfovDeg * Math.PI) / 180 / 2);
  const tv = (th * s.height) / s.width;
  const out = new Float32Array(s.width * s.height * 3);
  let n = 0;
  for (let v = 0; v < s.height; v++)
    for (let u = 0; u < s.width; u++) {
      const a = th * (1 - (2 * (u + 0.5)) / s.width);
      const b = tv * (1 - (2 * (v + 0.5)) / s.height);
      const d = mulVec(R, [1, a, b]);
      const len = Math.hypot(d[0], d[1], d[2]);
      let best = s.maxRange / len;
      let hit = false;
      if (d[2] < 0) {
        const t = -oz / d[2];
        if (t < best) {
          best = t;
          hit = true;
        }
      }
      for (let k = 0; k < segments.length; k++) {
        const t = raySegmentDistance(ox, oy, d[0], d[1], segments[k]);
        if (!(t < best)) continue;
        const z = oz + t * d[2];
        if (z >= 0 && z <= heights[k]) {
          best = t;
          hit = true;
        }
      }
      if (!hit) continue;
      out[n++] = ox + best * d[0];
      out[n++] = oy + best * d[1];
      out[n++] = oz + best * d[2];
    }
  return { points: out, count: n / 3 };
}
