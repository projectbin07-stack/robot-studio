// Log-odds occupancy grid with the inverse sensor model from Probabilistic
// Robotics (chapter 9): cells a beam passes through become more likely free,
// the cell where it ends becomes more likely occupied.

/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./lidar.js").Scan} Scan */

/**
 * @typedef {object} GridParams
 * @property {number} hit Log-odds added to the cell containing a return.
 * @property {number} miss Log-odds added to each cell a beam passes through (negative).
 * @property {number} min Lower clamp, keeps cells able to change their mind.
 * @property {number} max Upper clamp.
 */

/** @type {Readonly<GridParams>} */
export const DEFAULT_GRID_PARAMS = Object.freeze({ hit: 0.85, miss: -0.4, min: -4, max: 4 });

/**
 * @typedef {object} OccupancyGrid
 * @property {number} resolution Cell size in metres.
 * @property {number} width Cells along x.
 * @property {number} height Cells along y.
 * @property {number} originX World x of the lower-left corner of cell (0, 0).
 * @property {number} originY World y of the lower-left corner of cell (0, 0).
 * @property {Float32Array} logOdds Row-major, index = j * width + i, 0 = unknown.
 * @property {GridParams} params
 */

/**
 * The geometric part of a grid, shared with likelihood fields and masks.
 * @typedef {Pick<OccupancyGrid, "resolution" | "width" | "height" | "originX" | "originY">} GridGeometry
 */

/**
 * @param {number} widthM
 * @param {number} heightM
 * @param {number} resolution
 * @param {number} [originX]
 * @param {number} [originY]
 * @param {GridParams} [params]
 * @returns {OccupancyGrid}
 */
export function createGrid(
  widthM,
  heightM,
  resolution,
  originX = 0,
  originY = 0,
  params = DEFAULT_GRID_PARAMS,
) {
  if (!(resolution > 0)) throw new RangeError("resolution must be positive");
  const width = Math.ceil(widthM / resolution - 1e-9);
  const height = Math.ceil(heightM / resolution - 1e-9);
  if (width < 1 || height < 1) throw new RangeError("grid must be at least one cell");
  return {
    resolution,
    width,
    height,
    originX,
    originY,
    logOdds: new Float32Array(width * height),
    params: { ...params },
  };
}

/**
 * Cell indices containing a world point (may be outside the grid).
 * @param {GridGeometry} grid
 * @param {number} x
 * @param {number} y
 * @returns {[number, number]}
 */
export function worldToCell(grid, x, y) {
  return [
    Math.floor((x - grid.originX) / grid.resolution),
    Math.floor((y - grid.originY) / grid.resolution),
  ];
}

/**
 * @param {GridGeometry} grid
 * @param {number} i
 * @param {number} j
 * @returns {boolean}
 */
export function inGrid(grid, i, j) {
  return i >= 0 && j >= 0 && i < grid.width && j < grid.height;
}

/**
 * Convert log-odds to probability: p = 1 - 1 / (1 + e^l).
 * @param {number} l
 * @returns {number}
 */
export function logOddsToProbability(l) {
  return 1 - 1 / (1 + Math.exp(l));
}

/**
 * Visit every cell on the Bresenham line from (i0, j0) to (i1, j1), both ends
 * included. Stops early if the visitor returns false.
 * @param {number} i0
 * @param {number} j0
 * @param {number} i1
 * @param {number} j1
 * @param {(i: number, j: number) => boolean | void} visit
 */
export function traceLine(i0, j0, i1, j1, visit) {
  const di = Math.abs(i1 - i0);
  const dj = Math.abs(j1 - j0);
  const si = i0 < i1 ? 1 : -1;
  const sj = j0 < j1 ? 1 : -1;
  let err = di - dj;
  let i = i0;
  let j = j0;
  for (;;) {
    if (visit(i, j) === false) return;
    if (i === i1 && j === j1) return;
    const e2 = 2 * err;
    if (e2 > -dj) {
      err -= dj;
      i += si;
    }
    if (e2 < di) {
      err += di;
      j += sj;
    }
  }
}

/**
 * Add `delta` to one cell's log-odds, clamped. Out-of-grid cells are ignored.
 * @param {OccupancyGrid} grid
 * @param {number} i
 * @param {number} j
 * @param {number} delta
 */
function addLogOdds(grid, i, j, delta) {
  if (!inGrid(grid, i, j)) return;
  const k = j * grid.width + i;
  const { min, max } = grid.params;
  grid.logOdds[k] = Math.max(min, Math.min(max, grid.logOdds[k] + delta));
}

/**
 * Integrate a scan taken at `pose` into the grid. Beams without a return are
 * skipped: a missing return is not evidence that the space is empty, because
 * dropouts and dark surfaces look the same as open space.
 * @param {OccupancyGrid} grid
 * @param {Pose} pose
 * @param {Scan} scan
 */
export function integrateScan(grid, pose, scan) {
  const [i0, j0] = worldToCell(grid, pose.x, pose.y);
  const { hit, miss } = grid.params;
  for (let b = 0; b < scan.ranges.length; b++) {
    const r = scan.ranges[b];
    if (Number.isNaN(r)) continue;
    const a = pose.theta + scan.angles[b];
    const [i1, j1] = worldToCell(grid, pose.x + r * Math.cos(a), pose.y + r * Math.sin(a));
    traceLine(i0, j0, i1, j1, (i, j) => {
      if (i === i1 && j === j1) addLogOdds(grid, i, j, hit);
      else addLogOdds(grid, i, j, miss);
    });
  }
}

/**
 * Mask of cells whose log-odds exceed `threshold` (1 = occupied).
 * @param {OccupancyGrid} grid
 * @param {number} [threshold]
 * @returns {Uint8Array}
 */
export function occupiedMask(grid, threshold = 0) {
  const out = new Uint8Array(grid.logOdds.length);
  for (let k = 0; k < out.length; k++) out[k] = grid.logOdds[k] > threshold ? 1 : 0;
  return out;
}
