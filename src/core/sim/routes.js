// Command sources for a run: the world's demo route followed by
// lidar-slam-lab's waypoint autopilot, scripted open-loop manoeuvres, or
// nothing. (Keyboard, gamepad and touch teleop live in the page and give
// the simulator a twist directly.) Like lidar-slam-lab's, the autopilot
// steers with the true pose so runs with different estimators drive the
// same path.

import { followRoute } from "../../../vendor/lidar-slam-lab/src/autopilot.js";

/** @typedef {import("./se2.js").Pose2} Pose2 */
/** @typedef {import("./firmware.js").Twist} Twist */
/** @typedef {import("./robot.js").SimRobot} SimRobot */
/** @typedef {import("./world.js").SimWorld} SimWorld */

export const ROUTES = [
  { id: "demo", title: "World's demo route (autopilot)" },
  { id: "square", title: "Square: strafe (mecanum) or turn (differential)" },
  { id: "spin", title: "Two turns on the spot" },
  { id: "none", title: "Stand still" },
];

/**
 * @typedef {object} Route
 * @property {string} id
 * @property {(pose: Pose2, t: number) => { twist: Twist, done: boolean }} next pose: true pose of
 *   the drive frame (wheel centroid); t: seconds since the start
 * @property {() => string} progress
 */

/**
 * Speed limits the route uses for a robot: well inside what its wheels can do.
 * @param {SimRobot} robot
 */
export function routeSpeeds(robot) {
  const g = robot.geometry;
  const vWheel = g.wheelRadius * robot.maxWheelSpeed;
  const arm = g.drive === "mecanum" ? g.halfWidth + g.halfLength : g.halfWidth;
  return {
    v: Math.min(0.5, 0.6 * vWheel),
    w: Math.min(1.5, (0.6 * vWheel) / arm),
  };
}

/**
 * @param {string} id
 * @param {SimRobot} robot
 * @param {SimWorld} world
 * @returns {Route}
 */
export function createRoute(id, robot, world) {
  const sp = routeSpeeds(robot);
  if (id === "demo") {
    let index = 0;
    let done = world.route.length === 0;
    const opts = { maxSpeed: sp.v, maxTurnRate: sp.w, headingGain: 2.5, arrivalRadius: 0.25 };
    return {
      id,
      next(pose) {
        if (done) return { twist: { vx: 0, vy: 0, wz: 0 }, done };
        const c = followRoute(pose, world.route, index, opts);
        index = c.index;
        done = c.done;
        return { twist: { vx: c.v, vy: 0, wz: c.w }, done };
      },
      progress: () =>
        `waypoint ${Math.min(index + 1, world.route.length)} of ${world.route.length}`,
    };
  }
  /** @type {{ twist: Twist, duration: number, label: string }[]} */
  const segs = [];
  const stop = { twist: { vx: 0, vy: 0, wz: 0 }, duration: 0.5, label: "stop" };
  if (id === "square") {
    const v = Math.min(0.3, sp.v);
    const L = 1.5;
    if (robot.drive === "mecanum") {
      for (const [vx, vy, label] of /** @type {[number, number, string][]} */ ([
        [v, 0, "forward"],
        [0, v, "strafe left"],
        [-v, 0, "backward"],
        [0, -v, "strafe right"],
      ]))
        segs.push({ twist: { vx, vy, wz: 0 }, duration: L / v, label }, stop);
    } else {
      const w = Math.min(0.6, sp.w);
      for (let k = 0; k < 4; k++)
        segs.push(
          { twist: { vx: v, vy: 0, wz: 0 }, duration: L / v, label: "forward" },
          stop,
          { twist: { vx: 0, vy: 0, wz: w }, duration: Math.PI / 2 / w, label: "turn left" },
          stop,
        );
    }
  } else if (id === "spin") {
    const w = Math.min(0.8, sp.w);
    segs.push({ twist: { vx: 0, vy: 0, wz: w }, duration: (4 * Math.PI) / w, label: "turn" });
  } else if (id !== "none")
    throw new RangeError(`unknown route '${id}' (${ROUTES.map((r) => r.id).join(", ")})`);
  const total = segs.reduce((s, x) => s + x.duration, 0);
  let label = id === "none" ? "standing still" : "";
  return {
    id,
    next(_pose, t) {
      let acc = 0;
      for (const s of segs) {
        acc += s.duration;
        if (t < acc - 1e-9) {
          label = s.label;
          return { twist: s.twist, done: false };
        }
      }
      label = id === "none" ? "standing still" : "finished";
      return { twist: { vx: 0, vy: 0, wz: 0 }, done: id !== "none" && t >= total };
    },
    progress: () => label,
  };
}
