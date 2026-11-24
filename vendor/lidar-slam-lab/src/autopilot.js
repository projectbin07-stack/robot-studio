// Waypoint follower used by the demo route and the benchmark.

import { normalizeAngle } from "./geometry.js";

/** @typedef {import("./geometry.js").Pose} Pose */

/**
 * @typedef {object} AutopilotOptions
 * @property {number} maxSpeed m/s
 * @property {number} maxTurnRate rad/s
 * @property {number} headingGain rad/s per rad of heading error
 * @property {number} arrivalRadius Waypoint counts as reached within this many metres.
 */

/** @type {Readonly<AutopilotOptions>} */
export const DEFAULT_AUTOPILOT = Object.freeze({
  maxSpeed: 0.8,
  maxTurnRate: 1.5,
  headingGain: 2.5,
  arrivalRadius: 0.25,
});

/**
 * Velocity command towards a target: turn proportionally to the heading
 * error, and slow down with the cosine of the error so the robot turns on the
 * spot when the target is beside or behind it.
 * @param {Pose} pose
 * @param {number} tx
 * @param {number} ty
 * @param {AutopilotOptions} [o]
 * @returns {{ v: number, w: number }}
 */
export function steerTo(pose, tx, ty, o = DEFAULT_AUTOPILOT) {
  const err = normalizeAngle(Math.atan2(ty - pose.y, tx - pose.x) - pose.theta);
  const w = Math.max(-o.maxTurnRate, Math.min(o.maxTurnRate, o.headingGain * err));
  const dist = Math.hypot(tx - pose.x, ty - pose.y);
  const v = Math.min(o.maxSpeed, dist) * Math.max(0, Math.cos(err)) ** 2;
  return { v, w };
}

/**
 * Advance along a route.
 * @param {Pose} pose
 * @param {ReadonlyArray<readonly [number, number]>} route
 * @param {number} index Index of the current target waypoint.
 * @param {AutopilotOptions} [o]
 * @returns {{ v: number, w: number, index: number, done: boolean }}
 */
export function followRoute(pose, route, index, o = DEFAULT_AUTOPILOT) {
  let i = index;
  while (
    i < route.length &&
    Math.hypot(route[i][0] - pose.x, route[i][1] - pose.y) < o.arrivalRadius
  ) {
    i++;
  }
  if (i >= route.length) return { v: 0, w: 0, index: route.length, done: true };
  const { v, w } = steerTo(pose, route[i][0], route[i][1], o);
  return { v, w, index: i, done: false };
}
