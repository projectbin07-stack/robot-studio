// Simulation worlds: lidar-slam-lab's floor-plan format (a rectangle, wall
// segments, a start pose and a demo route) plus height: walls share one
// height, and boxes and cylinders each have their own. A 2D LiDAR sees an
// obstacle only if it is taller than the scan plane; everything blocks the
// robot. Bundled worlds are lidar-slam-lab's maps (reused, not copied: they
// come from the vendored maps.js) and Robot Studio's own "lab" with boxes
// and cylinders of different heights.

import { BUILTIN_MAPS } from "../../../vendor/lidar-slam-lab/src/maps.js";
import { boundaryWalls, parseWorld } from "../../../vendor/lidar-slam-lab/src/world.js";

/** @typedef {import("../../../vendor/lidar-slam-lab/src/geometry.js").Segment} Segment */
/** @typedef {import("./se2.js").Pose2} Pose2 */

/**
 * @typedef {object} Box
 * @property {number} x centre, m
 * @property {number} y
 * @property {number} w size along its own x, m
 * @property {number} d size along its own y, m
 * @property {number} yaw rad
 * @property {number} h height, m
 */
/** @typedef {{ x: number, y: number, r: number, h: number }} Cylinder */

/**
 * @typedef {object} SimWorld
 * @property {string} id bundled world id, or "custom"
 * @property {string} name
 * @property {number} width m along x
 * @property {number} height m along y
 * @property {number} wallHeight m, for every wall segment
 * @property {Segment[]} walls
 * @property {Box[]} boxes
 * @property {Cylinder[]} cylinders
 * @property {Pose2} start
 * @property {Array<[number, number]>} route autopilot waypoints
 */

/** Wall height used for lidar-slam-lab's maps (they are 2D). */
export const DEFAULT_WALL_HEIGHT = 2;
/** Cylinders become regular polygons with this many sides. */
export const CYLINDER_SIDES = 24;

/** @returns {SimWorld} */
function lab() {
  return {
    id: "lab",
    name: "Lab (boxes and cylinders of different heights)",
    width: 12,
    height: 8,
    wallHeight: 1.2,
    walls: [...boundaryWalls(12, 8), [6, 0, 6, 3], [6, 5, 6, 8]],
    boxes: [
      { x: 2.5, y: 6.3, w: 1.6, d: 0.8, yaw: 0, h: 0.75 },
      { x: 9.6, y: 1.6, w: 1.0, d: 1.0, yaw: Math.PI / 6, h: 0.5 },
      { x: 3.2, y: 1.3, w: 0.6, d: 0.4, yaw: 0, h: 0.06 },
      { x: 10.4, y: 6.4, w: 1.2, d: 0.6, yaw: -Math.PI / 12, h: 1.0 },
    ],
    cylinders: [
      { x: 4.2, y: 4, r: 0.25, h: 1.2 },
      { x: 8.4, y: 4.3, r: 0.18, h: 0.9 },
      { x: 9.0, y: 6.6, r: 0.12, h: 0.04 },
    ],
    start: { x: 1.2, y: 4, theta: 0 },
    route: [
      [3.2, 2.8],
      [5.2, 3.9],
      [7.2, 4.0],
      [7.4, 2.4],
      [10.8, 3.2],
      [10.8, 5.2],
      [7.6, 6.0],
      [6.8, 4.0],
      [5.0, 4.6],
      [2.4, 5.0],
      [1.2, 4.0],
    ],
  };
}

/**
 * @param {number} w
 * @param {number} h
 * @returns {SimWorld}
 */
function empty(w = 10, h = 10) {
  return {
    id: "empty",
    name: "Empty room",
    width: w,
    height: h,
    wallHeight: DEFAULT_WALL_HEIGHT,
    walls: boundaryWalls(w, h),
    boxes: [],
    cylinders: [],
    start: { x: w / 2, y: h / 2, theta: 0 },
    route: [],
  };
}

/**
 * A lidar-slam-lab map as a world with full-height walls.
 * @param {"office" | "warehouse" | "loop"} id
 * @returns {SimWorld}
 */
function fromLidarSlamLab(id) {
  const m = BUILTIN_MAPS[id]();
  return {
    id,
    name: m.name,
    width: m.width,
    height: m.height,
    wallHeight: DEFAULT_WALL_HEIGHT,
    walls: m.walls.map((w) => /** @type {Segment} */ ([...w])),
    boxes: [],
    cylinders: [],
    start: { ...m.start },
    route: m.route.map((p) => /** @type {[number, number]} */ ([p[0], p[1]])),
  };
}

/** Bundled worlds: id, title and where they come from. */
export const WORLDS = [
  { id: "office", title: "Office (lidar-slam-lab)", make: () => fromLidarSlamLab("office") },
  {
    id: "warehouse",
    title: "Warehouse aisles (lidar-slam-lab)",
    make: () => fromLidarSlamLab("warehouse"),
  },
  { id: "loop", title: "Corridor loop (lidar-slam-lab)", make: () => fromLidarSlamLab("loop") },
  { id: "lab", title: "Lab with boxes and cylinders", make: lab },
  { id: "empty", title: "Empty 10 m room", make: () => empty() },
];

/**
 * A fresh copy of a bundled world, or null.
 * @param {string} id
 * @returns {SimWorld | null}
 */
export function bundledWorld(id) {
  const w = WORLDS.find((x) => x.id === id);
  return w ? w.make() : null;
}

/**
 * Corners of a box in world coordinates, counter-clockwise.
 * @param {Box} b
 * @returns {Array<[number, number]>}
 */
export function boxCorners(b) {
  const c = Math.cos(b.yaw);
  const s = Math.sin(b.yaw);
  return [
    [-1, -1],
    [1, -1],
    [1, 1],
    [-1, 1],
  ].map(([u, v]) => {
    const lx = (u * b.w) / 2;
    const ly = (v * b.d) / 2;
    return /** @type {[number, number]} */ ([b.x + c * lx - s * ly, b.y + s * lx + c * ly]);
  });
}

/**
 * @param {Array<[number, number]>} pts
 * @returns {Segment[]}
 */
const polygon = (pts) =>
  pts.map((p, i) => {
    const q = pts[(i + 1) % pts.length];
    return /** @type {Segment} */ ([p[0], p[1], q[0], q[1]]);
  });

/**
 * Vertices of the polygon a cylinder is modelled as (circumscribed, so
 * the polygon contains the circle).
 * @param {Cylinder} c
 * @returns {Array<[number, number]>}
 */
export function cylinderPolygon(c) {
  const R = c.r / Math.cos(Math.PI / CYLINDER_SIDES);
  /** @type {Array<[number, number]>} */
  const out = [];
  for (let k = 0; k < CYLINDER_SIDES; k++) {
    const a = ((k + 0.5) * 2 * Math.PI) / CYLINDER_SIDES;
    out.push([c.x + R * Math.cos(a), c.y + R * Math.sin(a)]);
  }
  return out;
}

/**
 * Every obstacle as wall segments with their heights.
 * @param {SimWorld} world
 * @returns {{ segments: Segment[], heights: number[] }}
 */
export function obstacles(world) {
  /** @type {Segment[]} */
  const segments = [];
  /** @type {number[]} */
  const heights = [];
  for (const w of world.walls) {
    segments.push(w);
    heights.push(world.wallHeight);
  }
  for (const b of world.boxes)
    for (const s of polygon(boxCorners(b))) {
      segments.push(s);
      heights.push(b.h);
    }
  for (const c of world.cylinders)
    for (const s of polygon(cylinderPolygon(c))) {
      segments.push(s);
      heights.push(c.h);
    }
  return { segments, heights };
}

/**
 * Segments a horizontal scan plane at height z intersects (obstacles taller than z).
 * @param {SimWorld} world
 * @param {number} z
 * @returns {Segment[]}
 */
export function segmentsAbove(world, z) {
  const { segments, heights } = obstacles(world);
  return segments.filter((_, i) => heights[i] > z);
}

/** @param {unknown} v @param {string} what */
function num(v, what) {
  if (typeof v !== "number" || !Number.isFinite(v)) throw new Error(`${what} must be a number`);
  return v;
}

/** @param {number} v */
const r3 = (v) => Math.round(v * 1000) / 1000;

/**
 * Parse and validate a world (JSON text or value). Walls, start and route
 * are checked by lidar-slam-lab's own parseWorld; boxes, cylinders and
 * heights here.
 * @param {string | unknown} input
 * @returns {SimWorld}
 */
export function parseSimWorld(input) {
  /** @type {any} */
  let data = input;
  if (typeof input === "string") {
    try {
      data = JSON.parse(input);
    } catch (e) {
      throw new Error(`not valid JSON: ${e instanceof Error ? e.message : String(e)}`);
    }
  }
  const base = parseWorld(data);
  const wallHeight =
    data.wallHeight === undefined ? DEFAULT_WALL_HEIGHT : num(data.wallHeight, "wallHeight");
  if (!(wallHeight > 0 && wallHeight <= 20)) throw new Error("wallHeight must be in (0, 20] m");
  const inside = (/** @type {number} */ x, /** @type {number} */ y, /** @type {string} */ what) => {
    if (x < 0 || y < 0 || x > base.width || y > base.height)
      throw new Error(`${what} lies outside the map`);
  };
  const list = (/** @type {unknown} */ v, /** @type {string} */ what) => {
    if (v === undefined) return [];
    if (!Array.isArray(v)) throw new Error(`${what} must be an array`);
    if (v.length > 500) throw new Error(`at most 500 ${what} are supported`);
    return v;
  };
  const boxes = list(data.boxes, "boxes").map((b, i) => {
    const w = `boxes[${i}]`;
    const box = {
      x: num(b?.x, `${w}.x`),
      y: num(b?.y, `${w}.y`),
      w: num(b?.w, `${w}.w`),
      d: num(b?.d, `${w}.d`),
      yaw: b?.yaw === undefined ? 0 : num(b.yaw, `${w}.yaw`),
      h: num(b?.h, `${w}.h`),
    };
    if (!(box.w > 0 && box.d > 0 && box.h > 0)) throw new Error(`${w} must have a positive size`);
    for (const [x, y] of boxCorners(box)) inside(x, y, w);
    return box;
  });
  const cylinders = list(data.cylinders, "cylinders").map((c, i) => {
    const w = `cylinders[${i}]`;
    const cyl = {
      x: num(c?.x, `${w}.x`),
      y: num(c?.y, `${w}.y`),
      r: num(c?.r, `${w}.r`),
      h: num(c?.h, `${w}.h`),
    };
    if (!(cyl.r > 0 && cyl.h > 0)) throw new Error(`${w} must have a positive radius and height`);
    inside(cyl.x - cyl.r, cyl.y - cyl.r, w);
    inside(cyl.x + cyl.r, cyl.y + cyl.r, w);
    return cyl;
  });
  const id =
    typeof data.id === "string" && WORLDS.some((x) => x.id === data.id) ? data.id : "custom";
  return {
    id,
    name: base.name,
    width: base.width,
    height: base.height,
    wallHeight,
    walls: base.walls,
    boxes,
    cylinders,
    start: base.start,
    route: base.route,
  };
}

/**
 * JSON text of a world (lidar-slam-lab's map format plus the height fields;
 * lidar-slam-lab itself reads it and ignores what it does not know).
 * @param {SimWorld} w
 * @returns {string}
 */
export function serializeSimWorld(w) {
  return (
    JSON.stringify(
      {
        version: 1,
        id: w.id,
        name: w.name,
        width: w.width,
        height: w.height,
        wallHeight: w.wallHeight,
        start: { x: r3(w.start.x), y: r3(w.start.y), theta: r3(w.start.theta) },
        walls: w.walls.map((s) => s.map(r3)),
        boxes: w.boxes.map((b) => ({
          x: r3(b.x),
          y: r3(b.y),
          w: r3(b.w),
          d: r3(b.d),
          yaw: r3(b.yaw),
          h: r3(b.h),
        })),
        cylinders: w.cylinders.map((c) => ({ x: r3(c.x), y: r3(c.y), r: r3(c.r), h: r3(c.h) })),
        route: w.route.map((p) => p.map(r3)),
      },
      null,
      1,
    ) + "\n"
  );
}

/** @param {number} v */
const n3 = (v) => String(r3(v));

/**
 * Compact text of a world for a URL parameter: a bundled world unchanged is
 * its id; anything else is "~"-separated fields, numbers in metres rounded
 * to millimetres:
 *   name~width~height~wallHeight~sx,sy,stheta~walls~boxes~cylinders~route
 * where walls are "x1,y1,x2,y2" joined by ";", boxes "x,y,w,d,yaw,h",
 * cylinders "x,y,r,h" and route points "x,y".
 * @param {SimWorld} w
 * @returns {string}
 */
export function encodeWorld(w) {
  const b = w.id !== "custom" ? bundledWorld(w.id) : null;
  if (b && serializeSimWorld(b) === serializeSimWorld(w)) return w.id;
  const join = (/** @type {number[][]} */ rows) => rows.map((r) => r.map(n3).join(",")).join(";");
  return [
    w.name.replace(/[~]/g, "-").slice(0, 60),
    n3(w.width),
    n3(w.height),
    n3(w.wallHeight),
    [w.start.x, w.start.y, w.start.theta].map(n3).join(","),
    join(w.walls),
    join(w.boxes.map((x) => [x.x, x.y, x.w, x.d, x.yaw, x.h])),
    join(w.cylinders.map((x) => [x.x, x.y, x.r, x.h])),
    join(w.route),
  ].join("~");
}

/**
 * Inverse of encodeWorld. Throws on malformed text.
 * @param {string} text
 * @returns {SimWorld}
 */
export function decodeWorld(text) {
  const b = bundledWorld(text);
  if (b) return b;
  const f = text.split("~");
  if (f.length !== 9) throw new Error("malformed world parameter");
  const rows = (/** @type {string} */ s, /** @type {number} */ n) =>
    s === ""
      ? []
      : s.split(";").map((r) => {
          const v = r.split(",").map(Number);
          if (v.length !== n || !v.every(Number.isFinite))
            throw new Error("malformed world parameter");
          return v;
        });
  const [sx, sy, st] = rows(f[4], 3)[0] ?? [NaN, NaN, NaN];
  return parseSimWorld({
    version: 1,
    name: f[0],
    width: Number(f[1]),
    height: Number(f[2]),
    wallHeight: Number(f[3]),
    start: { x: sx, y: sy, theta: st },
    walls: rows(f[5], 4),
    boxes: rows(f[6], 6).map(([x, y, w, d, yaw, h]) => ({ x, y, w, d, yaw, h })),
    cylinders: rows(f[7], 4).map(([x, y, r, h]) => ({ x, y, r, h })),
    route: rows(f[8], 2),
  });
}
