// Built-in floor plans. They are made up for this simulator and do not
// describe any real building.

import { boundaryWalls } from "./world.js";

/** @typedef {import("./world.js").World} World */
/** @typedef {import("./geometry.js").Segment} Segment */

/**
 * Closed polygon as wall segments.
 * @param {Array<[number, number]>} pts
 * @returns {Segment[]}
 */
function polygon(pts) {
  return pts.map((p, i) => {
    const q = pts[(i + 1) % pts.length];
    return /** @type {Segment} */ ([p[0], p[1], q[0], q[1]]);
  });
}

/**
 * Axis-aligned box as four walls.
 * @param {number} x0
 * @param {number} y0
 * @param {number} x1
 * @param {number} y1
 * @returns {Segment[]}
 */
function box(x0, y0, x1, y1) {
  return polygon([
    [x0, y0],
    [x1, y0],
    [x1, y1],
    [x0, y1],
  ]);
}

/** @returns {World} */
function office() {
  return {
    name: "Office",
    width: 16,
    height: 12,
    walls: [
      ...boundaryWalls(16, 12),
      // Corridor wall with two 1.2 m doorways.
      [0, 5, 5, 5],
      [6.2, 5, 10, 5],
      [11.2, 5, 16, 5],
      // Wall splitting the two lower rooms.
      [8, 0, 8, 5],
      // Upper right room, open towards the corridor below it.
      [11, 7.5, 11, 12],
      // Column, desk and a diagonal cabinet.
      ...box(4, 8.5, 5, 9.5),
      [13, 2, 15, 2],
      [1.5, 1, 3, 2.5],
    ],
    start: { x: 2, y: 7, theta: 0 },
    route: [
      [8.5, 7],
      [13.5, 6.3],
      [13.5, 10],
      [13.5, 6.3],
      [10.6, 6],
      [10.6, 3.5],
      [12, 1.2],
      [10.6, 3.5],
      [10.6, 6],
      [5.6, 6],
      [5.6, 3],
      [3, 3.8],
      [5.6, 3],
      [5.6, 6],
      [2, 8.5],
      [2, 10.5],
      [8, 10.5],
      [7, 7],
      [2, 7],
    ],
  };
}

/** @returns {World} */
function warehouse() {
  return {
    name: "Warehouse aisles",
    width: 20,
    height: 12,
    walls: [
      ...boundaryWalls(20, 12),
      // Three identical shelf rows: a hard case, the aisles look alike.
      ...box(4, 2.5, 16, 3.1),
      ...box(4, 5, 16, 5.6),
      ...box(4, 7.5, 16, 8.1),
      // Two pallets near the right wall.
      ...box(18.9, 3, 19.5, 3.6),
      ...box(18.9, 8.5, 19.5, 9.1),
    ],
    start: { x: 2, y: 1.25, theta: 0 },
    route: [
      [17.5, 1.25],
      [17.5, 4.05],
      [2, 4.05],
      [2, 6.55],
      [17.5, 6.55],
      [17.5, 10],
      [2, 10],
      [2, 1.25],
    ],
  };
}

/** @returns {World} */
function loop() {
  return {
    name: "Corridor loop",
    width: 14,
    height: 14,
    walls: [
      ...boundaryWalls(14, 14),
      // Central block with a notch on its right side.
      ...polygon([
        [3, 3],
        [11, 3],
        [11, 8],
        [10, 8],
        [10, 9],
        [11, 9],
        [11, 11],
        [3, 11],
      ]),
      [7, 0, 7, 0.8],
      [12.5, 14, 14, 12.5],
    ],
    start: { x: 1.5, y: 1.5, theta: 0 },
    route: [
      [12.5, 1.5],
      [12.5, 12.5],
      [1.5, 12.5],
      [1.5, 1.5],
      [12.5, 1.5],
    ],
  };
}

/** Built-in maps by id. Each call returns a fresh copy that can be edited. */
export const BUILTIN_MAPS = Object.freeze({
  office,
  warehouse,
  loop,
});

/** @typedef {keyof typeof BUILTIN_MAPS} BuiltinMapId */

/**
 * @param {string} id
 * @returns {id is BuiltinMapId}
 */
export function isBuiltinMap(id) {
  return Object.prototype.hasOwnProperty.call(BUILTIN_MAPS, id);
}
