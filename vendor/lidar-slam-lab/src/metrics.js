// Error measures comparing the estimate with the ground truth.

import { normalizeAngle } from "./geometry.js";
import { distanceTransform } from "./distance.js";
import { inGrid, worldToCell } from "./grid.js";

/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./geometry.js").Segment} Segment */
/** @typedef {import("./grid.js").OccupancyGrid} OccupancyGrid */
/** @typedef {import("./grid.js").GridGeometry} GridGeometry */

/**
 * @param {Pose} a
 * @param {Pose} b
 * @returns {number} Euclidean distance between the positions in metres.
 */
export function positionError(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

/**
 * @param {Pose} a
 * @param {Pose} b
 * @returns {number} Absolute heading difference in radians, in [0, pi].
 */
export function headingError(a, b) {
  return Math.abs(normalizeAngle(a.theta - b.theta));
}

/**
 * Root-mean-square position error over two equally long trajectories
 * (the absolute trajectory error without alignment, since both start at the
 * same known pose).
 * @param {readonly Pose[]} truth
 * @param {readonly Pose[]} estimate
 * @returns {number}
 */
export function trajectoryRmse(truth, estimate) {
  if (truth.length !== estimate.length) throw new Error("trajectories differ in length");
  if (truth.length === 0) return 0;
  let sum = 0;
  for (let i = 0; i < truth.length; i++) {
    const e = positionError(truth[i], estimate[i]);
    sum += e * e;
  }
  return Math.sqrt(sum / truth.length);
}

/**
 * Mark every grid cell that a wall passes through.
 * @param {GridGeometry} grid
 * @param {readonly Segment[]} walls
 * @returns {Uint8Array}
 */
export function rasterizeWalls(grid, walls) {
  const mask = new Uint8Array(grid.width * grid.height);
  const step = grid.resolution / 4;
  for (const [x1, y1, x2, y2] of walls) {
    const n = Math.max(1, Math.ceil(Math.hypot(x2 - x1, y2 - y1) / step));
    for (let k = 0; k <= n; k++) {
      const [i, j] = worldToCell(grid, x1 + ((x2 - x1) * k) / n, y1 + ((y2 - y1) * k) / n);
      if (inGrid(grid, i, j)) mask[j * grid.width + i] = 1;
    }
  }
  return mask;
}

/**
 * @typedef {object} MapScore
 * @property {number} occupiedCells Cells the estimate believes are occupied.
 * @property {number} precision Fraction of those within `tolerance` of a true
 *   wall (NaN if there are none). Low precision means smeared or doubled walls.
 */

/**
 * Compare the estimated map with the true walls.
 * @param {OccupancyGrid} grid
 * @param {readonly Segment[]} walls
 * @param {number} tolerance Metres.
 * @param {number} [threshold] Log-odds above which a cell counts as occupied.
 * @returns {MapScore}
 */
export function mapPrecision(grid, walls, tolerance, threshold = 1) {
  const truth = rasterizeWalls(grid, walls);
  const dist = distanceTransform(truth, grid.width, grid.height);
  const tolCells = tolerance / grid.resolution;
  let occupiedCells = 0;
  let near = 0;
  for (let k = 0; k < grid.logOdds.length; k++) {
    if (grid.logOdds[k] <= threshold) continue;
    occupiedCells++;
    if (dist[k] <= tolCells) near++;
  }
  return { occupiedCells, precision: occupiedCells === 0 ? NaN : near / occupiedCells };
}
