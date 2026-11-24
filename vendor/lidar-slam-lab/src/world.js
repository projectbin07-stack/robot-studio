// Floor plans: a rectangle of known size, a list of wall segments and a start
// pose. Stored as JSON so that maps can be saved, shared and loaded.

import { pointSegmentDistance, segmentsIntersect } from "./geometry.js";

/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./geometry.js").Segment} Segment */

/**
 * @typedef {object} World
 * @property {string} name
 * @property {number} width Metres along x.
 * @property {number} height Metres along y.
 * @property {Segment[]} walls
 * @property {Pose} start
 * @property {Array<[number, number]>} route Waypoints for the autopilot (may be empty).
 */

export const FORMAT_VERSION = 1;
export const MAX_SIZE_M = 100;
export const MAX_WALLS = 2000;
export const MAX_ROUTE_POINTS = 500;

/**
 * The four walls along the edge of a width x height rectangle.
 * @param {number} width
 * @param {number} height
 * @returns {Segment[]}
 */
export function boundaryWalls(width, height) {
  return [
    [0, 0, width, 0],
    [width, 0, width, height],
    [width, height, 0, height],
    [0, height, 0, 0],
  ];
}

/**
 * @param {unknown} v
 * @param {string} what
 * @returns {number}
 */
function finite(v, what) {
  if (typeof v !== "number" || !Number.isFinite(v)) throw new Error(`${what} must be a number`);
  return v;
}

/**
 * Parse and validate a map. Accepts a JSON string or an already-parsed value.
 * Error messages name the offending field so they can be shown to the user.
 * @param {string | unknown} input
 * @returns {World}
 */
export function parseWorld(input) {
  /** @type {unknown} */
  let data = input;
  if (typeof input === "string") {
    try {
      data = JSON.parse(input);
    } catch (e) {
      throw new Error(`not valid JSON: ${e instanceof Error ? e.message : String(e)}`);
    }
  }
  if (typeof data !== "object" || data === null || Array.isArray(data)) {
    throw new Error("map must be a JSON object");
  }
  const obj = /** @type {Record<string, unknown>} */ (data);
  if (obj.version !== FORMAT_VERSION) {
    throw new Error(`unsupported map version ${String(obj.version)} (expected ${FORMAT_VERSION})`);
  }
  const width = finite(obj.width, "width");
  const height = finite(obj.height, "height");
  if (width <= 0 || height <= 0 || width > MAX_SIZE_M || height > MAX_SIZE_M) {
    throw new Error(`width and height must be in (0, ${MAX_SIZE_M}] metres`);
  }
  const name = typeof obj.name === "string" ? obj.name.slice(0, 80) : "Untitled";

  if (!Array.isArray(obj.walls)) throw new Error("walls must be an array");
  if (obj.walls.length > MAX_WALLS) throw new Error(`at most ${MAX_WALLS} walls are supported`);
  /** @type {Segment[]} */
  const walls = obj.walls.map((w, i) => {
    if (!Array.isArray(w) || w.length !== 4)
      throw new Error(`walls[${i}] must be [x1, y1, x2, y2]`);
    const seg = /** @type {Segment} */ (w.map((c, k) => finite(c, `walls[${i}][${k}]`)));
    for (let k = 0; k < 4; k++) {
      const limit = k % 2 === 0 ? width : height;
      if (seg[k] < 0 || seg[k] > limit) throw new Error(`walls[${i}] lies outside the map`);
    }
    if (seg[0] === seg[2] && seg[1] === seg[3]) throw new Error(`walls[${i}] has zero length`);
    return seg;
  });

  if (typeof obj.start !== "object" || obj.start === null) throw new Error("start is required");
  const s = /** @type {Record<string, unknown>} */ (obj.start);
  const start = {
    x: finite(s.x, "start.x"),
    y: finite(s.y, "start.y"),
    theta: s.theta === undefined ? 0 : finite(s.theta, "start.theta"),
  };
  if (start.x <= 0 || start.y <= 0 || start.x >= width || start.y >= height) {
    throw new Error("start must lie inside the map");
  }

  /** @type {Array<[number, number]>} */
  let route = [];
  if (obj.route !== undefined) {
    if (!Array.isArray(obj.route)) throw new Error("route must be an array of [x, y]");
    if (obj.route.length > MAX_ROUTE_POINTS) throw new Error("route is too long");
    route = obj.route.map((p, i) => {
      if (!Array.isArray(p) || p.length !== 2) throw new Error(`route[${i}] must be [x, y]`);
      return /** @type {[number, number]} */ ([
        finite(p[0], `route[${i}][0]`),
        finite(p[1], `route[${i}][1]`),
      ]);
    });
  }
  return { name, width, height, walls, start, route };
}

/**
 * @param {World} world
 * @returns {string}
 */
export function serializeWorld(world) {
  const round = (/** @type {number} */ v) => Math.round(v * 1000) / 1000;
  return JSON.stringify(
    {
      version: FORMAT_VERSION,
      name: world.name,
      width: world.width,
      height: world.height,
      start: { x: round(world.start.x), y: round(world.start.y), theta: round(world.start.theta) },
      walls: world.walls.map((w) => w.map(round)),
      route: world.route.map((p) => p.map(round)),
    },
    null,
    1,
  );
}

/**
 * Whether a disc of `radius` moving from (x0, y0) to (x1, y1) would hit a
 * wall: the path crosses a wall, or the end point is within radius of a wall
 * and closer to it than the start (so a robot placed too close to a wall can
 * still back away).
 * @param {readonly Segment[]} walls
 * @param {number} x0
 * @param {number} y0
 * @param {number} x1
 * @param {number} y1
 * @param {number} radius
 * @returns {boolean}
 */
export function collides(walls, x0, y0, x1, y1, radius) {
  /** @type {Segment} */
  const path = [x0, y0, x1, y1];
  for (const w of walls) {
    if (segmentsIntersect(path, w)) return true;
    const d1 = pointSegmentDistance(x1, y1, w);
    if (d1 < radius && d1 < pointSegmentDistance(x0, y0, w)) return true;
  }
  return false;
}
