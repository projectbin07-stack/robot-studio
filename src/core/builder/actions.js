// Builder actions that the UI offers as one step but that consist of
// several edit operations (a sensor with its optical frame, a new child
// link with an inertia). Each returns the resulting document, ready for a
// single undoable replaceDocument step, or throws EditError.

import { applyOp } from "./ops.js";
import { findLink, origin, uniqueName } from "./document.js";
import { inertialFromShape } from "./inertia.js";
import { SENSORS } from "./templates.js";

/** @typedef {import("./document.js").RobotDoc} RobotDoc */
/** @typedef {import("./document.js").SensorKind} SensorKind */
/** @typedef {import("./document.js").Geometry} Geometry */
/** @typedef {import("./document.js").Vec3} Vec3 */

/**
 * A new child link: a 5 cm box of 0.1 kg (visual, collision, inertia) on a
 * fixed joint 10 cm along the parent's x axis.
 * @param {RobotDoc} doc
 * @param {string} parent
 * @returns {{ doc: RobotDoc, link: string }}
 */
export function addChild(doc, parent) {
  const name = uniqueName(doc, "link_1");
  const jointName = uniqueName(doc, `${name}_joint`, [name]);
  /** @type {Geometry} */
  const geometry = { type: "box", size: [0.05, 0.05, 0.05] };
  const shape = { name: "", origin: origin(), geometry, material: null };
  const next = applyOp(doc, {
    type: "addLink",
    parent,
    link: {
      name,
      sensor: null,
      inertial: inertialFromShape(shape, 0.1),
      visuals: [{ ...shape, material: { name: "", rgba: [0.85, 0.55, 0.2, 1] } }],
      collisions: [shape],
    },
    joint: {
      name: jointName,
      type: "fixed",
      origin: origin([0.1, 0, 0]),
      axis: null,
      limit: null,
      dynamics: null,
      mimic: null,
    },
  });
  return { doc: next, link: name };
}

/**
 * A sensor mount (the templates' sensor link, plus an optical frame for
 * cameras) on a fixed joint under `parent`.
 * @param {RobotDoc} doc
 * @param {SensorKind} kind
 * @param {string} parent
 * @param {Vec3} [xyz]
 * @returns {{ doc: RobotDoc, link: string }}
 */
export function addSensor(doc, kind, parent, xyz = [0, 0, 0.05]) {
  const spec = SENSORS.find((s) => s.kind === kind);
  if (!spec) throw new RangeError(`unknown sensor kind '${kind}'`);
  const name = uniqueName(doc, spec.link);
  const stem = name.replace(/_link(_\d+)?$/, "$1");
  /** @type {Geometry} */
  const geometry =
    spec.shape === "box"
      ? { type: "box", size: [spec.dims[0], spec.dims[1], spec.dims[2]] }
      : { type: "cylinder", radius: spec.dims[0], length: spec.dims[1] };
  const shape = { name: "", origin: origin(), geometry, material: null };
  const fixed = {
    type: /** @type {const} */ ("fixed"),
    axis: null,
    limit: null,
    dynamics: null,
    mimic: null,
  };
  let next = applyOp(doc, {
    type: "addLink",
    parent,
    link: {
      name,
      sensor: kind,
      inertial: inertialFromShape(shape, spec.mass),
      visuals: [{ ...shape, material: { name: "", rgba: [0.08, 0.08, 0.09, 1] } }],
      collisions: [shape],
    },
    joint: { ...fixed, name: uniqueName(doc, `${stem}_joint`, [name]), origin: origin(xyz) },
  });
  if (spec.optical) {
    const optical = uniqueName(next, `${stem}_optical_frame`);
    next = applyOp(next, {
      type: "addLink",
      parent: name,
      link: { name: optical, sensor: null, inertial: null, visuals: [], collisions: [] },
      joint: {
        ...fixed,
        name: uniqueName(next, `${stem}_optical_joint`, [optical]),
        origin: origin([0, 0, 0], [-Math.PI / 2, 0, -Math.PI / 2]),
      },
    });
  }
  return { doc: next, link: name };
}

/**
 * Copy the first visual's geometry and origin as a collision shape.
 * @param {RobotDoc} doc
 * @param {string} link
 */
export function collisionFromVisual(doc, link) {
  const l = findLink(doc, link);
  const v = l?.visuals[0];
  if (!v) throw new RangeError(`link '${link}' has no visual to copy`);
  return applyOp(doc, {
    type: "addShape",
    link,
    kind: "collision",
    shape: { ...v, name: "", material: null },
  });
}
