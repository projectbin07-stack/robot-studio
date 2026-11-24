// Inertial from geometry and mass, with urdf-inspector's solid-body
// formulas (primitiveInertia) and its tensor rotation (rotateTensor), so
// the result is exactly what urdf-inspector's inertia-implausible and
// inertia-axes checks compare against. The centre of mass is put at the
// shape's centroid and the <inertial> frame is not rotated (rpy 0 0 0);
// the tensor is rotated from the shape frame into it.

import { primitiveInertia, rotateTensor } from "../../../vendor/urdf-inspector/src/core/inertia.js";
import { apply, fromXyzRpy } from "../../../vendor/urdf-inspector/src/core/math.js";

/** @typedef {import("./document.js").ShapeDoc} ShapeDoc */
/** @typedef {import("./document.js").InertialDoc} InertialDoc */
/** @typedef {import("./document.js").LinkDoc} LinkDoc */
/** @typedef {import("./document.js").Vec3} Vec3 */

/**
 * Axis-aligned bounds of a mesh in its own file coordinates (before the
 * URDF scale), as measured by the 3D view after loading it.
 * @typedef {{ min: Vec3, max: Vec3 }} MeshBounds
 */

/**
 * The shape whose volume stands for the link: the first collision shape,
 * else the first visual shape (the shape urdf-inspector's checks use when
 * a link has exactly one).
 * @param {LinkDoc} link
 * @returns {ShapeDoc | null}
 */
export function inertiaShape(link) {
  return link.collisions[0] ?? link.visuals[0] ?? null;
}

/**
 * Inertial of a solid shape of mass m. Meshes are treated as a solid box
 * filling their bounding box (bounds required).
 * @param {ShapeDoc} shape
 * @param {number} mass kg, > 0
 * @param {MeshBounds} [bounds] for meshes
 * @returns {InertialDoc}
 */
export function inertialFromShape(shape, mass, bounds) {
  if (!(mass > 0) || !Number.isFinite(mass))
    throw new RangeError(`mass must be positive, got ${mass}`);
  const g = shape.geometry;
  /** @type {import("../../../vendor/urdf-inspector/src/core/urdf.js").Geometry} */
  let solid = g;
  /** @type {Vec3} */
  let centre = [0, 0, 0];
  if (g.type === "mesh") {
    if (!bounds)
      throw new RangeError("a mesh needs its bounding box (load it in the 3D view first)");
    const size = /** @type {Vec3} */ (
      [0, 1, 2].map((k) => (bounds.max[k] - bounds.min[k]) * Math.abs(g.scale[k]))
    );
    centre = /** @type {Vec3} */ (
      [0, 1, 2].map((k) => ((bounds.max[k] + bounds.min[k]) / 2) * g.scale[k])
    );
    solid = { type: "box", size };
  }
  const local = primitiveInertia(solid, mass);
  if (!local) throw new RangeError(`no inertia formula for ${g.type}`);
  const com = apply(fromXyzRpy(shape.origin.xyz, shape.origin.rpy), centre);
  return {
    origin: { xyz: [com[0] + 0, com[1] + 0, com[2] + 0], rpy: [0, 0, 0] },
    mass,
    inertia: rotateTensor(local, shape.origin.rpy, [0, 0, 0]),
  };
}
