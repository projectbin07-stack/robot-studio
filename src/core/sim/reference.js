// lidar-slam-lab's own experiment run through Robot Studio's estimator
// wrapper: the unicycle robot, its rot1-trans-rot2 odometry error model and
// a LiDAR at the robot's origin (an identity mount), with the same seeds
// and random streams as lidar-slam-lab's sim.js. Used to check that the
// wrapper drives the vendored modules exactly as lidar-slam-lab does
// (tests/sim-estimators.test.js against the vendored sim.js, and
// scripts/lsl-crosscheck.mjs against lidar-slam-lab's CLI benchmark).

import { createRng } from "../../../vendor/lidar-slam-lab/src/rng.js";
import { DEFAULT_LIDAR, simulateScan } from "../../../vendor/lidar-slam-lab/src/lidar.js";
import {
  DEFAULT_ODOMETRY_NOISE,
  corruptOdometry,
  integrateUnicycle,
  odometryDelta,
} from "../../../vendor/lidar-slam-lab/src/motion.js";
import { collides } from "../../../vendor/lidar-slam-lab/src/world.js";
import { followRoute } from "../../../vendor/lidar-slam-lab/src/autopilot.js";
import { BUILTIN_MAPS } from "../../../vendor/lidar-slam-lab/src/maps.js";
import { DEFAULT_MATCH } from "../../../vendor/lidar-slam-lab/src/scanmatch.js";
import { createEstimators, errors, firstScan, updateEstimators } from "./estimators.js";

/**
 * @typedef {object} ReferenceResult
 * @property {number} steps
 * @property {number} distance m
 * @property {boolean} finished route finished
 * @property {number} odomRms m
 * @property {number} estRms m (NaN when scan matching is off)
 * @property {number} mclRms m (NaN when MCL is off)
 */

/**
 * Drive a lidar-slam-lab map's demo route (lidar-slam-lab's runRoute) with
 * lidar-slam-lab's default settings.
 * @param {"office" | "warehouse" | "loop"} mapId
 * @param {number} seed
 * @param {{ scanMatching: boolean, particles: number, maxSteps?: number }} opts
 * @returns {ReferenceResult}
 */
export function referenceRun(mapId, seed, opts) {
  const world = BUILTIN_MAPS[mapId]();
  const dt = 0.1;
  const radius = 0.2;
  const lidarRng = createRng(seed);
  const odomRng = createRng(seed ^ 0x9e3779b9);
  const e = createEstimators(
    world,
    world.walls,
    world.start,
    { x: 0, y: 0, theta: 0 },
    {
      seed,
      scanMatching: opts.scanMatching,
      particles: opts.particles,
      mclGlobal: false,
      resolution: 0.05,
      fieldSigma: 0.1,
      fieldOccupiedThreshold: 0.5,
      match: { ...DEFAULT_MATCH },
      clearance: radius,
    },
  );
  let truth = { ...world.start };
  firstScan(e, simulateScan(world.walls, truth, DEFAULT_LIDAR, lidarRng));
  let idx = 0;
  let finished = world.route.length === 0;
  let distance = 0;
  let steps = 0;
  for (let k = 0; k < (opts.maxSteps ?? 3000) && !finished; k++) {
    const cmd = followRoute(truth, world.route, idx);
    idx = cmd.index;
    finished = cmd.done;
    if (finished) break;
    const prev = truth;
    let next = integrateUnicycle(prev, cmd.v, cmd.w, dt);
    if (collides(world.walls, prev.x, prev.y, next.x, next.y, radius))
      next = { x: prev.x, y: prev.y, theta: integrateUnicycle(prev, 0, cmd.w, dt).theta };
    truth = next;
    distance += Math.hypot(next.x - prev.x, next.y - prev.y);
    const measured = corruptOdometry(odometryDelta(prev, next), DEFAULT_ODOMETRY_NOISE, odomRng);
    const scan = simulateScan(world.walls, next, DEFAULT_LIDAR, lidarRng);
    updateEstimators(e, measured, scan, truth);
    steps++;
  }
  const err = errors(e, truth);
  return {
    steps,
    distance,
    finished,
    odomRms: err.odomRms,
    estRms: opts.scanMatching ? err.estRms : NaN,
    mclRms: err.mclRms,
  };
}
