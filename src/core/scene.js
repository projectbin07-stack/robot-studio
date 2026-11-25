// What the 3D view draws, computed without the DOM or three.js: shapes with
// their colours and local transforms, joint axes and per-link poses as
// column-major 4x4 matrices (the layout three.js Matrix4.fromArray reads).
// Poses come from urdf-inspector's forward kinematics.

import {
  forwardKinematics,
  isMovable,
  jointFrames,
  massProperties,
  sliderRange,
} from "../../vendor/urdf-inspector/src/core/fk.js";
import { fromXyzRpy, normalize, norm } from "../../vendor/urdf-inspector/src/core/math.js";

/** @typedef {import("../../vendor/urdf-inspector/src/core/math.js").Transform} Transform */
/** @typedef {import("../../vendor/urdf-inspector/src/core/urdf.js").Robot} Robot */
/** @typedef {import("../../vendor/urdf-inspector/src/core/urdf.js").Shape} Shape */
/** @typedef {import("../../vendor/urdf-inspector/src/core/tree.js").KinTree} KinTree */

/** Colour used when a visual has no material colour (RViz draws such links grey). */
export const DEFAULT_RGBA = [0.75, 0.76, 0.78, 1];

/**
 * Column-major 4x4 matrix of a transform.
 * @param {Transform} t
 * @returns {number[]}
 */
export function matrix4(t) {
  const R = t.R;
  return [R[0], R[3], R[6], 0, R[1], R[4], R[7], 0, R[2], R[5], R[8], 0, t.p[0], t.p[1], t.p[2], 1];
}

/**
 * @typedef {object} DrawShape
 * @property {string} link
 * @property {"visual" | "collision"} kind
 * @property {number} index
 * @property {import("../../vendor/urdf-inspector/src/core/urdf.js").Geometry} geometry
 * @property {number[]} local shape origin in the link frame (column-major 4x4)
 * @property {number[] | null} rgba URDF colour (inline or named material), null for none
 */

/**
 * Every visual and collision shape with geometry.
 * @param {Robot} robot
 * @returns {DrawShape[]}
 */
export function drawShapes(robot) {
  /** @type {DrawShape[]} */
  const out = [];
  for (const link of robot.links) {
    for (const kind of /** @type {const} */ (["visual", "collision"])) {
      const list = kind === "visual" ? link.visuals : link.collisions;
      list.forEach((s, index) => {
        if (!s.geometry || s.geometry.type === "unknown") return;
        out.push({
          link: link.name,
          kind,
          index,
          geometry: s.geometry,
          local: matrix4(fromXyzRpy(s.origin.xyz, s.origin.rpy)),
          rgba: kind === "visual" ? shapeColor(s, robot) : null,
        });
      });
    }
  }
  return out;
}

/**
 * A visual's colour: its inline colour, else its named top-level material.
 * @param {Shape} s
 * @param {Robot} robot
 * @returns {number[] | null}
 */
export function shapeColor(s, robot) {
  if (!s.material) return null;
  if (s.material.rgba) return s.material.rgba;
  return robot.materials.get(s.material.name)?.rgba ?? null;
}

/**
 * @typedef {object} JointMarker
 * @property {string} name
 * @property {string} type
 * @property {number[]} origin joint frame in the root frame (column-major 4x4)
 * @property {[number, number, number]} axis unit axis in the joint frame
 */

/**
 * Link poses, joint frames and the centre of mass for joint positions q.
 * @param {KinTree} tree
 * @param {Record<string, number>} q
 */
export function posesFor(tree, q) {
  const poses = forwardKinematics(tree, q);
  const frames = jointFrames(tree, q, poses);
  /** @type {Map<string, number[]>} */
  const links = new Map();
  for (const [name, t] of poses) links.set(name, matrix4(t));
  /** @type {JointMarker[]} */
  const joints = [];
  for (const j of tree.joints.values()) {
    const frame = frames.get(j.name);
    if (!frame || !isMovable(j) || norm(j.axis) === 0) continue;
    joints.push({
      name: j.name,
      type: String(j.type),
      origin: matrix4(frame),
      axis: normalize(j.axis),
    });
  }
  const mp = massProperties(tree, poses);
  return { links, joints, com: mp.com, mass: mp.mass };
}

/**
 * Slider definitions for the movable joints, in tree order. Mimic joints
 * follow their source and get no slider of their own.
 * @param {KinTree} tree
 */
export function sliders(tree) {
  /** @type {{ name: string, type: string, min: number, max: number, mimic: string | null, unit: string }[]} */
  const out = [];
  for (const link of tree.order)
    for (const j of tree.childJoints.get(link) ?? []) {
      if (!isMovable(j)) continue;
      const r = sliderRange(j);
      out.push({
        name: j.name,
        type: String(j.type),
        min: r.min,
        max: r.max,
        mimic: j.mimic && tree.joints.has(j.mimic.joint) ? j.mimic.joint : null,
        unit: j.type === "prismatic" ? "m" : "rad",
      });
    }
  return out;
}

/**
 * Axis-aligned bounds of the link origins and shape origins, with a margin
 * for the shapes' size; used to frame the camera and size the grid.
 * @param {Map<string, number[]>} links
 * @param {DrawShape[]} shapes
 * @returns {{ center: [number, number, number], radius: number }}
 */
export function roughBounds(links, shapes) {
  const lo = [Infinity, Infinity, Infinity];
  const hi = [-Infinity, -Infinity, -Infinity];
  /** @param {number} x @param {number} y @param {number} z @param {number} r */
  const add3 = (x, y, z, r) => {
    lo[0] = Math.min(lo[0], x - r);
    lo[1] = Math.min(lo[1], y - r);
    lo[2] = Math.min(lo[2], z - r);
    hi[0] = Math.max(hi[0], x + r);
    hi[1] = Math.max(hi[1], y + r);
    hi[2] = Math.max(hi[2], z + r);
  };
  for (const m of links.values()) add3(m[12], m[13], m[14], 0);
  for (const s of shapes) {
    const m = links.get(s.link);
    if (!m) continue;
    const l = s.local;
    const x = m[0] * l[12] + m[4] * l[13] + m[8] * l[14] + m[12];
    const y = m[1] * l[12] + m[5] * l[13] + m[9] * l[14] + m[13];
    const z = m[2] * l[12] + m[6] * l[13] + m[10] * l[14] + m[14];
    add3(x, y, z, shapeSize(s.geometry));
  }
  if (!Number.isFinite(lo[0])) return { center: [0, 0, 0], radius: 0.5 };
  const center = /** @type {[number, number, number]} */ (
    [0, 1, 2].map((k) => (lo[k] + hi[k]) / 2)
  );
  const radius = Math.max(0.05, Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) / 2);
  return { center, radius };
}

/**
 * Half the largest extent of a primitive; meshes count as 5 cm until loaded.
 * @param {import("../../vendor/urdf-inspector/src/core/urdf.js").Geometry} g
 */
export function shapeSize(g) {
  switch (g.type) {
    case "box":
      return Math.hypot(...g.size) / 2;
    case "cylinder":
      return Math.hypot(g.radius, g.length / 2);
    case "sphere":
      return g.radius;
    default:
      return 0.05;
  }
}
