// Wheel geometry of a document as urdf-inspector measures it, the
// mecanum-kinematics TOML written by urdf-inspector's mecanumToml(), and
// mecanum inverse/forward kinematics computed from the measured wheel
// positions. The kinematics follow mecanum-kinematics' conventions (REP 103
// body frame; FL, FR, RL, RR; a positive wheel speed drives that side
// forward; rollers in an "X" seen from above, so r w_i = vx - w y_i +
// s_i tan(a) (vy + w x_i) with s = -1, +1, +1, -1). They exist so CI can
// load the exported TOML with mecanum-kinematics' own loader and check
// that its numbers and these agree (scripts/mecanum_interop.py).

import { mecanumToml } from "../../../vendor/urdf-inspector/src/core/wheels.js";
import { lateralSign } from "../../../vendor/urdf-inspector/src/core/wheels.js";

/** @typedef {import("../model.js").LoadedModel} LoadedModel */
/** @typedef {import("../../../vendor/urdf-inspector/src/core/wheels.js").WheelBase} WheelBase */

export const WHEEL_ORDER = /** @type {const} */ (["FL", "FR", "RL", "RR"]);
const ROLLER_SIGNS = [-1, 1, 1, -1];

/**
 * @param {LoadedModel} model
 * @returns {WheelBase | null}
 */
export const wheelBase = (model) => model.report?.context?.wheels ?? null;

/**
 * mecanum-kinematics config for a four-wheel model (urdf-inspector's writer).
 * @param {LoadedModel} model
 * @returns {{ ok: true, toml: string } | { ok: false, reason: string }}
 */
export function mecanumConfig(model) {
  const robot = model.report?.robot;
  if (!robot) return { ok: false, reason: "the model could not be read" };
  const base = wheelBase(model);
  if (base?.kind === "differential")
    return {
      ok: false,
      reason:
        "this is a differential base; mecanum-kinematics models four-wheel mecanum bases only",
    };
  return mecanumToml(base, robot.name);
}

/**
 * @typedef {object} MecanumGeometry
 * @property {number} radius
 * @property {{ label: string, joint: string, x: number, y: number, sign: number }[]} wheels
 *   FL, FR, RL, RR: centre relative to the wheel centroid, and the joint
 *   axis sign (+1 when the joint axis is +y, so joint velocity = wheel speed)
 */

/**
 * @param {WheelBase | null} base
 * @returns {MecanumGeometry | null}
 */
export function mecanumGeometry(base) {
  if (!base || base.kind !== "four-wheel" || base.radius === null || base.lx === null) return null;
  const by = new Map(base.wheels.map((w) => [w.label, w]));
  const wheels = WHEEL_ORDER.map((label) => {
    const w = /** @type {import("../../../vendor/urdf-inspector/src/core/wheels.js").Wheel} */ (
      by.get(label)
    );
    return {
      label,
      joint: w.joint,
      x: w.centre[0] - base.centroid[0],
      y: w.centre[1] - base.centroid[1],
      sign: lateralSign(w),
    };
  });
  return { radius: base.radius, wheels };
}

/**
 * Wheel speeds (rad/s, positive = forward) for a body twist.
 * @param {MecanumGeometry} g
 * @param {{ vx: number, vy: number, wz: number }} t
 * @param {number} [rollerAngle] rad
 * @returns {number[]} FL, FR, RL, RR
 */
export function mecanumInverse(g, t, rollerAngle = Math.PI / 4) {
  const tan = Math.tan(rollerAngle);
  return g.wheels.map(
    (w, i) => (t.vx - t.wz * w.y + ROLLER_SIGNS[i] * tan * (t.vy + t.wz * w.x)) / g.radius,
  );
}

/**
 * Least-squares body twist for four wheel speeds (normal equations).
 * @param {MecanumGeometry} g
 * @param {number[]} speeds FL, FR, RL, RR
 * @param {number} [rollerAngle]
 * @returns {{ vx: number, vy: number, wz: number }}
 */
export function mecanumForward(g, speeds, rollerAngle = Math.PI / 4) {
  const tan = Math.tan(rollerAngle);
  const J = g.wheels.map((w, i) => [
    1 / g.radius,
    (ROLLER_SIGNS[i] * tan) / g.radius,
    (-w.y + ROLLER_SIGNS[i] * tan * w.x) / g.radius,
  ]);
  /** @type {number[][]} */
  const A = [0, 1, 2].map((r) =>
    [0, 1, 2].map((c) => J.reduce((s, row) => s + row[r] * row[c], 0)),
  );
  const b = [0, 1, 2].map((r) => J.reduce((s, row, i) => s + row[r] * speeds[i], 0));
  const [vx, vy, wz] = solve3(A, b);
  return { vx, vy, wz };
}

/**
 * Solve a 3x3 linear system by Cramer's rule.
 * @param {number[][]} A
 * @param {number[]} b
 */
function solve3(A, b) {
  /** @param {number[][]} m */
  const det = (m) =>
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  const d = det(A);
  if (Math.abs(d) < 1e-300) throw new RangeError("wheel geometry is singular");
  return [0, 1, 2].map((k) => det(A.map((row, r) => row.map((v, c) => (c === k ? b[r] : v)))) / d);
}
