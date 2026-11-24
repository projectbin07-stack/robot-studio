// Planar rigid-body helpers (SE(2)). Poses are { x, y, theta } with theta
// in radians counter-clockwise; the same shape as lidar-slam-lab's Pose so
// both can be passed around without conversion. se2Exp and wrapAngle follow
// encoder-odometry-firmware's odometry.cpp exactly (the firmware integrates
// its wheel odometry with them).

/** @typedef {{ x: number, y: number, theta: number }} Pose2 */

/**
 * Wrap an angle to (-pi, pi] the way the firmware does (fmod based).
 * @param {number} angle
 * @returns {number}
 */
export function wrapAngle(angle) {
  let w = (angle + Math.PI) % (2 * Math.PI);
  if (w <= 0) w += 2 * Math.PI;
  return w - Math.PI;
}

/**
 * Exponential map: the pose reached by moving with a constant body twist
 * (dx, dy, dtheta) for unit time.
 * @param {number} dx
 * @param {number} dy
 * @param {number} dtheta
 * @returns {Pose2}
 */
export function se2Exp(dx, dy, dtheta) {
  let a;
  let b;
  if (Math.abs(dtheta) < 1e-6) {
    a = 1 - (dtheta * dtheta) / 6;
    b = dtheta / 2 - (dtheta * dtheta * dtheta) / 24;
  } else {
    a = Math.sin(dtheta) / dtheta;
    b = (1 - Math.cos(dtheta)) / dtheta;
  }
  return { x: a * dx - b * dy, y: b * dx + a * dy, theta: dtheta };
}

/**
 * p * q: q expressed in p's frame, mapped to p's parent frame. Theta is not
 * wrapped (the firmware keeps the unwrapped angle internally).
 * @param {Pose2} p
 * @param {Pose2} q
 * @returns {Pose2}
 */
export function compose(p, q) {
  const c = Math.cos(p.theta);
  const s = Math.sin(p.theta);
  return { x: p.x + c * q.x - s * q.y, y: p.y + s * q.x + c * q.y, theta: p.theta + q.theta };
}

/**
 * Inverse of a pose.
 * @param {Pose2} p
 * @returns {Pose2}
 */
export function inverse(p) {
  const c = Math.cos(p.theta);
  const s = Math.sin(p.theta);
  return { x: -c * p.x - s * p.y, y: s * p.x - c * p.y, theta: -p.theta };
}

/**
 * Logarithm map: the constant body twist (per unit time) that moves from
 * the identity to p. Inverse of se2Exp.
 * @param {Pose2} p
 * @returns {{ vx: number, vy: number, wz: number }}
 */
export function se2Log(p) {
  const t = p.theta;
  let a;
  let b;
  if (Math.abs(t) < 1e-6) {
    a = 1 - (t * t) / 6;
    b = t / 2 - (t * t * t) / 24;
  } else {
    a = Math.sin(t) / t;
    b = (1 - Math.cos(t)) / t;
  }
  const d = a * a + b * b;
  return { vx: (a * p.x + b * p.y) / d, vy: (-b * p.x + a * p.y) / d, wz: t };
}
