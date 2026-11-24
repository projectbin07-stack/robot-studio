// File formats for results: the occupancy grid as a ROS map_server style
// PGM + YAML pair, and trajectories as CSV.

import { logOddsToProbability } from "./grid.js";

/** @typedef {import("./grid.js").OccupancyGrid} OccupancyGrid */
/** @typedef {import("./geometry.js").Pose} Pose */

/**
 * Grey level for one cell, using the map_server convention with negate = 0:
 * 254 free, 0 occupied, 205 unknown.
 * @param {number} logOdds
 * @param {number} freeThresh
 * @param {number} occupiedThresh
 * @returns {number}
 */
export function cellToGrey(logOdds, freeThresh, occupiedThresh) {
  const p = logOddsToProbability(logOdds);
  if (p > occupiedThresh) return 0;
  if (p < freeThresh) return 254;
  return 205;
}

/**
 * Binary PGM (P5). Image rows run top to bottom, so the grid's top row (the
 * largest y) is written first, as map_server expects.
 * @param {OccupancyGrid} grid
 * @param {number} [freeThresh]
 * @param {number} [occupiedThresh]
 * @returns {Uint8Array<ArrayBuffer>}
 */
export function gridToPgm(grid, freeThresh = 0.196, occupiedThresh = 0.65) {
  const header = new TextEncoder().encode(`P5\n${grid.width} ${grid.height}\n255\n`);
  const out = new Uint8Array(header.length + grid.width * grid.height);
  out.set(header);
  let k = header.length;
  for (let j = grid.height - 1; j >= 0; j--) {
    for (let i = 0; i < grid.width; i++) {
      out[k++] = cellToGrey(grid.logOdds[j * grid.width + i], freeThresh, occupiedThresh);
    }
  }
  return out;
}

/**
 * map_server YAML describing a PGM written by gridToPgm.
 * @param {OccupancyGrid} grid
 * @param {string} imageName
 * @returns {string}
 */
export function mapYaml(grid, imageName) {
  return [
    `image: ${imageName}`,
    "mode: trinary",
    `resolution: ${grid.resolution}`,
    `origin: [${grid.originX}, ${grid.originY}, 0.0]`,
    "negate: 0",
    "occupied_thresh: 0.65",
    "free_thresh: 0.196",
    "",
  ].join("\n");
}

/**
 * Trajectories side by side, one row per step. Columns that are not given
 * are left out.
 * @param {Record<string, readonly Pose[]>} trails Name to poses; all of equal length.
 * @param {number} dt Seconds per row.
 * @returns {string}
 */
export function trajectoriesCsv(trails, dt) {
  const names = Object.keys(trails);
  const n = names.length === 0 ? 0 : trails[names[0]].length;
  for (const name of names) {
    if (trails[name].length !== n) throw new Error(`trajectory ${name} has a different length`);
  }
  const header = ["t", ...names.flatMap((m) => [`${m}_x`, `${m}_y`, `${m}_theta`])];
  const rows = [header.join(",")];
  for (let i = 0; i < n; i++) {
    const cells = [(i * dt).toFixed(3)];
    for (const name of names) {
      const p = trails[name][i];
      cells.push(p.x.toFixed(4), p.y.toFixed(4), p.theta.toFixed(5));
    }
    rows.push(cells.join(","));
  }
  return rows.join("\n") + "\n";
}
