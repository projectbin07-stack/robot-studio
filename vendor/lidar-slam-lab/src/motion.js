// Robot motion and the wheel-odometry error model.

import { normalizeAngle } from "./geometry.js";

/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./rng.js").Rng} Rng */

/**
 * Relative motion between two poses in the rot1-trans-rot2 decomposition of
 * the odometry motion model (Thrun, Burgard and Fox, Probabilistic Robotics,
 * section 5.4): turn by rot1, drive straight by trans, turn by rot2.
 * @typedef {{ rot1: number, trans: number, rot2: number }} OdometryDelta
 */

/**
 * Odometry error parameters. Each noise term is a standard deviation that grows
 * linearly with the motion (the book uses variances; standard deviations are
 * easier to reason about on a slider).
 * @typedef {object} OdometryNoise
 * @property {number} rotFromRot Std of rotation error per radian turned.
 * @property {number} rotFromTrans Std of rotation error (rad) per metre driven.
 * @property {number} transFromTrans Std of distance error per metre driven.
 * @property {number} transFromRot Std of distance error (m) per radian turned.
 * @property {number} headingBias Systematic heading error in rad per metre driven,
 *   as produced by unequal wheel diameters. Positive drifts counter-clockwise.
 */

/** @type {Readonly<OdometryNoise>} */
export const DEFAULT_ODOMETRY_NOISE = Object.freeze({
  rotFromRot: 0.05,
  rotFromTrans: 0.02,
  transFromTrans: 0.03,
  transFromRot: 0.005,
  headingBias: 0.02,
});

/** Below this distance (m) a motion is treated as a pure rotation. */
const MIN_TRANS = 1e-9;

/**
 * Integrate a unicycle model exactly over dt for constant v (m/s) and w (rad/s).
 * @param {Pose} pose
 * @param {number} v
 * @param {number} w
 * @param {number} dt
 * @returns {Pose}
 */
export function integrateUnicycle(pose, v, w, dt) {
  const th = pose.theta;
  if (Math.abs(w) < 1e-9) {
    return { x: pose.x + v * dt * Math.cos(th), y: pose.y + v * dt * Math.sin(th), theta: th };
  }
  const th2 = th + w * dt;
  const r = v / w;
  return {
    x: pose.x + r * (Math.sin(th2) - Math.sin(th)),
    y: pose.y - r * (Math.cos(th2) - Math.cos(th)),
    theta: normalizeAngle(th2),
  };
}

/**
 * Decompose the motion from `a` to `b` into rot1, trans and rot2.
 * @param {Pose} a
 * @param {Pose} b
 * @returns {OdometryDelta}
 */
export function odometryDelta(a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const trans = Math.hypot(dx, dy);
  if (trans < MIN_TRANS) {
    return { rot1: 0, trans: 0, rot2: normalizeAngle(b.theta - a.theta) };
  }
  const rot1 = normalizeAngle(Math.atan2(dy, dx) - a.theta);
  const rot2 = normalizeAngle(b.theta - a.theta - rot1);
  return { rot1, trans, rot2 };
}

/**
 * Apply a relative motion to a pose (the inverse of odometryDelta).
 * @param {Pose} pose
 * @param {OdometryDelta} d
 * @returns {Pose}
 */
export function applyOdometryDelta(pose, d) {
  const heading = pose.theta + d.rot1;
  return {
    x: pose.x + d.trans * Math.cos(heading),
    y: pose.y + d.trans * Math.sin(heading),
    theta: normalizeAngle(heading + d.rot2),
  };
}

/**
 * What wheel odometry would report for a true motion: the true delta plus
 * zero-mean noise and a systematic heading bias.
 * @param {OdometryDelta} truth
 * @param {OdometryNoise} noise
 * @param {Rng} rng
 * @returns {OdometryDelta}
 */
export function corruptOdometry(truth, noise, rng) {
  const r1 = Math.abs(truth.rot1);
  const r2 = Math.abs(truth.rot2);
  const t = truth.trans;
  const sdRot1 = noise.rotFromRot * r1 + noise.rotFromTrans * t;
  const sdTrans = noise.transFromTrans * t + noise.transFromRot * (r1 + r2);
  const sdRot2 = noise.rotFromRot * r2 + noise.rotFromTrans * t;
  // Always draw three samples so the random sequence does not depend on the motion.
  const n1 = rng.gaussian();
  const n2 = rng.gaussian();
  const n3 = rng.gaussian();
  return {
    rot1: truth.rot1 + sdRot1 * n1,
    trans: Math.max(0, t + sdTrans * n2),
    rot2: truth.rot2 + sdRot2 * n3 + noise.headingBias * t,
  };
}

/**
 * Relative motion between two poses expressed in the first pose's frame:
 * forward dx, leftward dy and turn dtheta. Unlike rot1-trans-rot2 this
 * represents a sideways move (a mecanum strafe) as a sideways move, not as
 * two 90 degree turns around a straight segment, so noise proportional to
 * the turn stays small while strafing.
 * @typedef {{ dx: number, dy: number, dtheta: number }} BodyDelta
 */

/**
 * The relative motion from `a` to `b` in `a`'s frame.
 * @param {Pose} a
 * @param {Pose} b
 * @returns {BodyDelta}
 */
export function bodyDelta(a, b) {
  const ex = b.x - a.x;
  const ey = b.y - a.y;
  const c = Math.cos(a.theta);
  const s = Math.sin(a.theta);
  return { dx: c * ex + s * ey, dy: -s * ex + c * ey, dtheta: normalizeAngle(b.theta - a.theta) };
}

/**
 * The same motion as an odometry delta, in the pose's frame: rot1-trans-rot2
 * and the body delta describe the same end pose.
 * @param {OdometryDelta} d
 * @returns {BodyDelta}
 */
export function bodyFromOdometry(d) {
  return {
    dx: d.trans * Math.cos(d.rot1),
    dy: d.trans * Math.sin(d.rot1),
    dtheta: normalizeAngle(d.rot1 + d.rot2),
  };
}

/**
 * Apply a body-frame motion to a pose (the inverse of bodyDelta).
 * @param {Pose} pose
 * @param {BodyDelta} d
 * @returns {Pose}
 */
export function applyBodyDelta(pose, d) {
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  return {
    x: pose.x + c * d.dx - s * d.dy,
    y: pose.y + s * d.dx + c * d.dy,
    theta: normalizeAngle(pose.theta + d.dtheta),
  };
}

/**
 * Omnidirectional motion noise: the body delta plus zero-mean noise on each
 * axis and the systematic heading bias. With t = |(dx, dy)| and r = |dtheta|,
 * the standard deviations are transFromTrans * |dx| + transFromRot * r forward,
 * transFromTrans * |dy| + transFromRot * r sideways and rotFromRot * r +
 * rotFromTrans * t for the turn; headingBias * t is added to the turn. A
 * pure forward drive therefore gets the same distance noise as in
 * corruptOdometry, and a strafe gets sideways noise instead of rotation noise.
 * Three samples are drawn for every call, like corruptOdometry.
 * @param {BodyDelta} truth
 * @param {OdometryNoise} noise
 * @param {Rng} rng
 * @returns {BodyDelta}
 */
export function corruptBodyDelta(truth, noise, rng) {
  const t = Math.hypot(truth.dx, truth.dy);
  const r = Math.abs(truth.dtheta);
  const sdX = noise.transFromTrans * Math.abs(truth.dx) + noise.transFromRot * r;
  const sdY = noise.transFromTrans * Math.abs(truth.dy) + noise.transFromRot * r;
  const sdTh = noise.rotFromRot * r + noise.rotFromTrans * t;
  const n1 = rng.gaussian();
  const n2 = rng.gaussian();
  const n3 = rng.gaussian();
  return {
    dx: truth.dx + sdX * n1,
    dy: truth.dy + sdY * n2,
    dtheta: truth.dtheta + sdTh * n3 + noise.headingBias * t,
  };
}
