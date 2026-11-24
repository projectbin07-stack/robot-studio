// The editable robot document: plain JSON (no Maps, no classes) so it can
// be cloned for undo/redo, compared, and kept in browser storage. It holds
// what a URDF holds (links with inertial, visuals and collisions; joints;
// top-level materials) plus top-level elements Robot Studio does not edit
// (<gazebo>, <ros2_control>, <transmission>, ...), which are carried along
// and written back unchanged except for renamed references.
//
// Sensors are links tagged with a sensor kind; the tag is Robot Studio
// metadata (URDF has no sensor element) and is guessed from the link name
// when a URDF is imported.

/** @typedef {[number, number, number]} Vec3 */
/** @typedef {{ xyz: Vec3, rpy: Vec3 }} Origin */
/**
 * @typedef {{ type: "box", size: Vec3 }
 *   | { type: "cylinder", radius: number, length: number }
 *   | { type: "sphere", radius: number }
 *   | { type: "mesh", filename: string, scale: Vec3 }} Geometry
 */
/** @typedef {{ name: string, rgba: number[] | null }} MaterialRef a visual's material: a name, an inline colour, or both */
/**
 * @typedef {object} ShapeDoc
 * @property {string} name "" when the element has no name attribute
 * @property {Origin} origin
 * @property {Geometry} geometry
 * @property {MaterialRef | null} material visuals only
 */
/** @typedef {import("../../../vendor/urdf-inspector/src/core/urdf.js").InertiaTensor} InertiaTensor */
/**
 * @typedef {object} InertialDoc
 * @property {Origin} origin
 * @property {number | null} mass null when the imported file had none
 * @property {InertiaTensor | null} inertia
 */
/** @typedef {"lidar2d" | "imu" | "camera" | "depth_camera"} SensorKind */
/**
 * @typedef {object} LinkDoc
 * @property {string} name
 * @property {SensorKind | null} sensor
 * @property {InertialDoc | null} inertial
 * @property {ShapeDoc[]} visuals
 * @property {ShapeDoc[]} collisions
 */
/**
 * @typedef {object} LimitDoc
 * @property {number | null} lower
 * @property {number | null} upper
 * @property {number | null} effort
 * @property {number | null} velocity
 */
/** @typedef {"revolute" | "continuous" | "prismatic" | "fixed" | "floating" | "planar"} JointType */
/**
 * @typedef {object} JointDoc
 * @property {string} name
 * @property {JointType} type
 * @property {string} parent
 * @property {string} child
 * @property {Origin} origin
 * @property {Vec3 | null} axis null: not written (URDF default 1 0 0)
 * @property {LimitDoc | null} limit
 * @property {{ damping: number, friction: number } | null} dynamics
 * @property {{ joint: string, multiplier: number, offset: number } | null} mimic
 */
/** @typedef {{ name: string, rgba: number[] | null }} MaterialDoc */
/**
 * A top-level element carried through unchanged.
 * @typedef {object} ExtraElement
 * @property {string} name
 * @property {Record<string, string>} attrs
 * @property {ExtraElement[]} children
 * @property {string} text
 */
/**
 * @typedef {object} TemplateRef
 * @property {string} id
 * @property {Record<string, TemplateValue>} params
 */
/** @typedef {number | boolean | string | Vec3} TemplateValue */
/**
 * @typedef {object} RobotDoc
 * @property {1} format
 * @property {string} name
 * @property {MaterialDoc[]} materials
 * @property {LinkDoc[]} links
 * @property {JointDoc[]} joints kept in the order of their child links (see normalizeOrder)
 * @property {ExtraElement[]} extras
 * @property {TemplateRef | null} template the template and parameters the document was generated from
 * @property {{ entry: string } | null} source the file an imported document came from (mesh paths resolve from it)
 */

export const SENSOR_KINDS = /** @type {const} */ (["lidar2d", "imu", "camera", "depth_camera"]);
export const JOINT_TYPES = /** @type {const} */ ([
  "revolute",
  "continuous",
  "prismatic",
  "fixed",
  "floating",
  "planar",
]);

/** Names Robot Studio accepts for new links and joints (ROS frame-friendly). */
export const NAME_RE = /^[A-Za-z_][A-Za-z0-9_.\-/]{0,127}$/;

/** @returns {Origin} */
export const zeroOrigin = () => ({ xyz: [0, 0, 0], rpy: [0, 0, 0] });

/**
 * @param {Vec3} [xyz]
 * @param {Vec3} [rpy]
 * @returns {Origin}
 */
export const origin = (xyz = [0, 0, 0], rpy = [0, 0, 0]) => ({
  xyz: [xyz[0], xyz[1], xyz[2]],
  rpy: [rpy[0], rpy[1], rpy[2]],
});

/**
 * @param {string} name
 * @returns {LinkDoc}
 */
export const emptyLink = (name) => ({
  name,
  sensor: null,
  inertial: null,
  visuals: [],
  collisions: [],
});

/**
 * @param {string} name
 * @returns {RobotDoc}
 */
export const emptyDoc = (name) => ({
  format: 1,
  name,
  materials: [],
  links: [],
  joints: [],
  extras: [],
  template: null,
  source: null,
});

/**
 * Deep copy of a plain-JSON value.
 * @template T
 * @param {T} v
 * @returns {T}
 */
export const clone = (v) => /** @type {T} */ (JSON.parse(JSON.stringify(v)));

/**
 * Structural equality of plain-JSON values (object key order ignored).
 * @param {unknown} a
 * @param {unknown} b
 * @returns {boolean}
 */
export function deepEqual(a, b) {
  if (a === b) return true;
  if (typeof a !== "object" || typeof b !== "object" || a === null || b === null) return false;
  if (Array.isArray(a) !== Array.isArray(b)) return false;
  if (Array.isArray(a) && Array.isArray(b))
    return a.length === b.length && a.every((x, k) => deepEqual(x, b[k]));
  const ka = Object.keys(a);
  const kb = Object.keys(b);
  if (ka.length !== kb.length) return false;
  const ob = /** @type {Record<string, unknown>} */ (b);
  const oa = /** @type {Record<string, unknown>} */ (a);
  return ka.every((k) => Object.hasOwn(ob, k) && deepEqual(oa[k], ob[k]));
}

/** @param {RobotDoc} doc @param {string} name */
export const findLink = (doc, name) => doc.links.find((l) => l.name === name) ?? null;
/** @param {RobotDoc} doc @param {string} name */
export const findJoint = (doc, name) => doc.joints.find((j) => j.name === name) ?? null;
/** @param {RobotDoc} doc @param {string} link */
export const parentJointOf = (doc, link) => doc.joints.find((j) => j.child === link) ?? null;
/** @param {RobotDoc} doc @param {string} link */
export const childJointsOf = (doc, link) => doc.joints.filter((j) => j.parent === link);

/**
 * Root links (no parent joint), in document order.
 * @param {RobotDoc} doc
 * @returns {string[]}
 */
export function rootLinks(doc) {
  const children = new Set(doc.joints.map((j) => j.child));
  return doc.links.filter((l) => !children.has(l.name)).map((l) => l.name);
}

/**
 * A link and everything below it (depth first, the link first). Guards
 * against cycles in imported documents.
 * @param {RobotDoc} doc
 * @param {string} link
 * @returns {string[]}
 */
export function subtree(doc, link) {
  /** @type {string[]} */
  const out = [];
  const seen = new Set();
  /** @param {string} name */
  const walk = (name) => {
    if (seen.has(name)) return;
    seen.add(name);
    out.push(name);
    for (const j of childJointsOf(doc, name)) walk(j.child);
  };
  walk(link);
  return out;
}

/**
 * A name not yet used by a link or joint: `base`, else `base_2`, `base_3`, ...
 * @param {RobotDoc} doc
 * @param {string} base
 * @param {Iterable<string>} [also] names reserved in addition
 */
export function uniqueName(doc, base, also = []) {
  const used = new Set([
    ...doc.links.map((l) => l.name),
    ...doc.joints.map((j) => j.name),
    ...also,
  ]);
  if (!used.has(base)) return base;
  const stem = base.replace(/_\d+$/, "");
  for (let k = 2; ; k++) if (!used.has(`${stem}_${k}`)) return `${stem}_${k}`;
}

/**
 * Joints ordered by the position of their child link, joints whose child
 * does not exist last (in their previous order). The URDF writer puts each
 * joint right after its child link, so this keeps export -> import -> export
 * stable.
 * @param {RobotDoc} doc
 */
export function normalizeOrder(doc) {
  const index = new Map(doc.links.map((l, k) => [l.name, k]));
  const keyed = doc.joints.map((j, k) => ({ j, k, c: index.get(j.child) ?? Infinity }));
  keyed.sort((a, b) => a.c - b.c || a.k - b.k);
  doc.joints = keyed.map((x) => x.j);
}

/**
 * Sensor kind suggested by a link name (used when importing a URDF).
 * @param {string} name
 * @returns {SensorKind | null}
 */
export function guessSensor(name) {
  const n = name.toLowerCase();
  if (/optical/.test(n)) return null;
  if (/depth|rgbd|realsense|kinect/.test(n)) return "depth_camera";
  if (/lidar|laser|scan|lds|hokuyo|rplidar/.test(n)) return "lidar2d";
  if (/\bimu\b|(^|_)imu(_|$)/.test(n)) return "imu";
  if (/camera|cam(_|$)/.test(n)) return "camera";
  return null;
}
