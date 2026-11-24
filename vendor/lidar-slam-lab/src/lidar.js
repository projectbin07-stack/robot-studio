// Simulated 2D LiDAR: ray casting against the true walls, Gaussian range
// noise and random dropouts.

import { castRay } from "./geometry.js";

/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./geometry.js").Segment} Segment */
/** @typedef {import("./rng.js").Rng} Rng */

/**
 * @typedef {object} LidarConfig
 * @property {number} beams Number of beams per scan (>= 2).
 * @property {number} fov Field of view in radians, at most 2*pi.
 * @property {number} maxRange Maximum range in metres.
 * @property {number} rangeNoise Standard deviation of range noise in metres.
 * @property {number} dropout Probability in [0, 1] that a beam returns nothing.
 */

/**
 * One scan. Angles are in the robot frame. A range of NaN means "no return"
 * (dropout, or nothing within maxRange).
 * @typedef {{ angles: Float64Array, ranges: Float64Array, maxRange: number }} Scan
 */

/** @type {Readonly<LidarConfig>} */
export const DEFAULT_LIDAR = Object.freeze({
  beams: 180,
  fov: 2 * Math.PI,
  maxRange: 8,
  rangeNoise: 0.02,
  dropout: 0.03,
});

/**
 * Beam angles in the robot frame. A full circle is split into equal steps
 * without repeating the first beam; a partial fan spans -fov/2 to +fov/2.
 * @param {number} beams
 * @param {number} fov
 * @returns {Float64Array}
 */
export function beamAngles(beams, fov) {
  if (!Number.isInteger(beams) || beams < 2) throw new RangeError("beams must be an integer >= 2");
  if (!(fov > 0 && fov <= 2 * Math.PI + 1e-9)) throw new RangeError("fov must be in (0, 2*pi]");
  const out = new Float64Array(beams);
  const full = fov >= 2 * Math.PI - 1e-9;
  const step = full ? fov / beams : fov / (beams - 1);
  const start = full ? 0 : -fov / 2;
  for (let i = 0; i < beams; i++) out[i] = start + i * step;
  return out;
}

/**
 * Simulate one scan from the true pose.
 * @param {readonly Segment[]} walls
 * @param {Pose} pose
 * @param {LidarConfig} config
 * @param {Rng} rng
 * @returns {Scan}
 */
export function simulateScan(walls, pose, config, rng) {
  const angles = beamAngles(config.beams, config.fov);
  const ranges = new Float64Array(angles.length);
  for (let i = 0; i < angles.length; i++) {
    const truth = castRay(walls, pose.x, pose.y, pose.theta + angles[i], config.maxRange);
    // Draw both samples every time so that changing one setting does not
    // reshuffle the noise of every later beam.
    const u = rng.next();
    const n = rng.gaussian();
    if (!Number.isFinite(truth) || u < config.dropout) {
      ranges[i] = NaN;
      continue;
    }
    const r = truth + config.rangeNoise * n;
    ranges[i] = r > 0 && r <= config.maxRange ? r : NaN;
  }
  return { angles, ranges, maxRange: config.maxRange };
}

/**
 * World coordinates of every valid return when the scan is placed at `pose`.
 * @param {Scan} scan
 * @param {Pose} pose
 * @returns {Float64Array} Interleaved [x0, y0, x1, y1, ...].
 */
export function scanEndpoints(scan, pose) {
  let n = 0;
  for (const r of scan.ranges) if (!Number.isNaN(r)) n++;
  const out = new Float64Array(2 * n);
  let k = 0;
  for (let i = 0; i < scan.ranges.length; i++) {
    const r = scan.ranges[i];
    if (Number.isNaN(r)) continue;
    const a = pose.theta + scan.angles[i];
    out[k++] = pose.x + r * Math.cos(a);
    out[k++] = pose.y + r * Math.sin(a);
  }
  return out;
}
