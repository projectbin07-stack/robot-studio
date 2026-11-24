// The estimators, built from the vendored lidar-slam-lab modules: wheel
// odometry (dead reckoning), odometry corrected by scan-to-map matching
// with the occupancy grid built from that estimate, and Monte Carlo
// localisation on the known floor plan.
//
// lidar-slam-lab assumes the LiDAR sits at the robot's origin and the robot
// turns about that origin. Here the estimators track the robot's drive
// frame (the wheel centroid, its centre of rotation), and the LiDAR sits
// wherever the URDF puts it: each scan is re-expressed as seen from the
// drive frame's origin (every return keeps its exact end point; only the
// free-space rays of the map start at the robot's centre instead of at the
// LiDAR). Tracking the LiDAR's own pose instead would make the
// rot1-trans-rot2 odometry model ill-conditioned: a turn on the spot moves
// an offset LiDAR a few millimetres sideways, which that model reads as two
// large rotations and MCL spreads its particles accordingly. With the LiDAR
// at the origin the scan is passed through unchanged and the call sequence
// is exactly lidar-slam-lab's sim.js step(), which the tests check number
// for number.

import { applyOdometryDelta } from "../../../vendor/lidar-slam-lab/src/motion.js";
import {
  createGrid,
  integrateScan,
  occupiedMask,
} from "../../../vendor/lidar-slam-lab/src/grid.js";
import { likelihoodField } from "../../../vendor/lidar-slam-lab/src/distance.js";
import { DEFAULT_MATCH, matchScan } from "../../../vendor/lidar-slam-lab/src/scanmatch.js";
import { positionError, rasterizeWalls } from "../../../vendor/lidar-slam-lab/src/metrics.js";
import {
  createParticleFilter,
  estimate,
  initializeAround,
  initializeUniform,
  predict,
  resampleIfNeeded,
  updateWeights,
} from "../../../vendor/lidar-slam-lab/src/mcl.js";
import { MCL_FIELD_SIGMA } from "../../../vendor/lidar-slam-lab/src/sim.js";
import { createRng } from "../../../vendor/lidar-slam-lab/src/rng.js";
import { normalizeAngle } from "../../../vendor/lidar-slam-lab/src/geometry.js";

/** @typedef {import("./se2.js").Pose2} Pose2 */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/geometry.js").Segment} Segment */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/lidar.js").Scan} Scan */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/grid.js").OccupancyGrid} OccupancyGrid */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/distance.js").LikelihoodField} LikelihoodField */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/scanmatch.js").MatchResult} MatchResult */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/scanmatch.js").MatchOptions} MatchOptions */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/mcl.js").ParticleFilter} ParticleFilter */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/motion.js").OdometryDelta} OdometryDelta */

/**
 * @typedef {object} EstimatorSettings
 * @property {number} seed
 * @property {boolean} scanMatching
 * @property {number} particles MCL particle count, 0 = off
 * @property {boolean} mclGlobal start MCL spread over the whole map
 * @property {number} resolution grid cell size, m
 * @property {number} fieldSigma likelihood-field fall-off for matching, m
 * @property {number} fieldOccupiedThreshold log-odds above which a cell is an obstacle for matching
 * @property {MatchOptions} match
 * @property {number} clearance m, for MCL's free space
 */

/** lidar-slam-lab's defaults (sim.js defaultSettings) for the estimator part. */
export const DEFAULT_ESTIMATORS = Object.freeze({
  scanMatching: true,
  particles: 300,
  mclGlobal: false,
  resolution: 0.05,
  fieldSigma: 0.1,
  fieldOccupiedThreshold: 0.5,
});

/**
 * @typedef {object} Estimators
 * @property {EstimatorSettings} settings
 * @property {Pose2} mount LiDAR scan origin in the tracked frame (planar)
 * @property {Pose2} odom dead reckoning (world frame)
 * @property {Pose2} est odometry + scan matching
 * @property {Pose2 | null} mcl
 * @property {number} mclSpread
 * @property {OccupancyGrid} grid built from the estimate
 * @property {LikelihoodField | null} field
 * @property {ParticleFilter | null} pf
 * @property {MatchResult | null} lastMatch
 * @property {number} scansIntegrated
 * @property {number} matchAttempts
 * @property {number} matchAccepted
 * @property {number} samples error samples (one per scan)
 * @property {number} sumSqOdom
 * @property {number} sumSqEst
 * @property {number} sumSqMcl
 * @property {number} maxOdom
 * @property {number} maxEst
 * @property {number} maxMcl
 */

/**
 * @param {{ width: number, height: number }} area map size (m)
 * @param {readonly Segment[]} knownWalls the floor plan MCL localises on (walls the LiDAR sees)
 * @param {Pose2} start true start pose of the tracked frame (defines the map frame)
 * @param {Pose2} mount LiDAR scan origin in the tracked frame
 * @param {EstimatorSettings} settings
 * @returns {Estimators}
 */
export function createEstimators(area, knownWalls, start, mount, settings) {
  const margin = 2;
  const grid = createGrid(
    area.width + 2 * margin,
    area.height + 2 * margin,
    settings.resolution,
    -margin,
    -margin,
  );
  /** @type {Estimators} */
  const e = {
    settings,
    mount,
    odom: { ...start },
    est: { ...start },
    mcl: null,
    mclSpread: NaN,
    grid,
    field: null,
    pf: null,
    lastMatch: null,
    scansIntegrated: 0,
    matchAttempts: 0,
    matchAccepted: 0,
    samples: 0,
    sumSqOdom: 0,
    sumSqEst: 0,
    sumSqMcl: 0,
    maxOdom: 0,
    maxEst: 0,
    maxMcl: 0,
  };
  if (settings.particles > 0) {
    const known = likelihoodField(rasterizeWalls(grid, knownWalls), grid, MCL_FIELD_SIGMA);
    const space = {
      width: area.width,
      height: area.height,
      walls: knownWalls,
      clearance: settings.clearance,
    };
    const pf = createParticleFilter(
      settings.particles,
      known,
      createRng(settings.seed ^ 0x85ebca6b),
      {},
      space,
    );
    if (settings.mclGlobal) initializeUniform(pf);
    else initializeAround(pf, start, 0.1, 0.05);
    e.pf = pf;
    const est = estimate(pf);
    e.mcl = est.pose;
    e.mclSpread = est.spread;
  }
  return e;
}

/**
 * A scan as seen from the tracked frame's origin: each return's end point
 * is kept exactly; angles and ranges are measured from the origin. A scan
 * from a LiDAR at the origin is returned unchanged.
 * @param {Scan} scan
 * @param {Pose2} mount
 * @returns {Scan}
 */
export function virtualScan(scan, mount) {
  if (mount.x === 0 && mount.y === 0 && mount.theta === 0) return scan;
  const n = scan.ranges.length;
  const angles = new Float64Array(n);
  const ranges = new Float64Array(n);
  const c = Math.cos(mount.theta);
  const s = Math.sin(mount.theta);
  let maxRange = 0;
  for (let i = 0; i < n; i++) {
    const r = scan.ranges[i];
    const a = mount.theta + scan.angles[i];
    if (Number.isNaN(r)) {
      angles[i] = a;
      ranges[i] = NaN;
      continue;
    }
    const lx = r * Math.cos(scan.angles[i]);
    const ly = r * Math.sin(scan.angles[i]);
    const x = mount.x + c * lx - s * ly;
    const y = mount.y + s * lx + c * ly;
    angles[i] = Math.atan2(y, x);
    ranges[i] = Math.hypot(x, y);
    maxRange = Math.max(maxRange, ranges[i]);
  }
  return { angles, ranges, maxRange: Math.max(scan.maxRange, maxRange) };
}

/**
 * Integrate the first scan at the start pose (lidar-slam-lab's initialScan).
 * @param {Estimators} e
 * @param {Scan} scan as the LiDAR took it
 */
export function firstScan(e, scan) {
  integrateScan(e.grid, e.est, virtualScan(scan, e.mount));
  e.scansIntegrated++;
  e.field = null;
}

/**
 * One estimator update with a new scan. `measured` is the tracked frame's
 * motion since the previous scan as odometry reports it.
 * @param {Estimators} e
 * @param {OdometryDelta} measured
 * @param {Scan} raw scan as the LiDAR took it
 * @param {Pose2} truth true pose of the tracked frame, for the error measures only
 */
export function updateEstimators(e, measured, raw, truth) {
  const s = e.settings;
  const scan = virtualScan(raw, e.mount);
  e.odom = applyOdometryDelta(e.odom, measured);
  const guess = applyOdometryDelta(e.est, measured);
  if (e.pf !== null) {
    predict(e.pf, measured);
    updateWeights(e.pf, scan);
    resampleIfNeeded(e.pf);
    const est = estimate(e.pf);
    e.mcl = est.pose;
    e.mclSpread = est.spread;
  }
  if (s.scanMatching && e.scansIntegrated > 0) {
    if (e.field === null) e.field = buildField(e);
    const r = matchScan(e.field, scan, guess, s.match);
    e.lastMatch = r;
    e.matchAttempts++;
    if (r.accepted) e.matchAccepted++;
    e.est = r.pose;
  } else {
    e.lastMatch = null;
    e.est = guess;
  }
  integrateScan(e.grid, e.est, scan);
  e.scansIntegrated++;
  e.field = null;
  record(e, truth);
}

/**
 * Accumulate the error measures against the truth.
 * @param {Estimators} e
 * @param {Pose2} truth
 */
function record(e, truth) {
  const eo = positionError(truth, e.odom);
  const ee = positionError(truth, e.est);
  e.sumSqOdom += eo * eo;
  e.sumSqEst += ee * ee;
  e.maxOdom = Math.max(e.maxOdom, eo);
  e.maxEst = Math.max(e.maxEst, ee);
  if (e.mcl) {
    const em = positionError(truth, e.mcl);
    e.sumSqMcl += em * em;
    e.maxMcl = Math.max(e.maxMcl, em);
  }
  e.samples++;
}

/**
 * @param {Estimators} e
 * @returns {LikelihoodField}
 */
function buildField(e) {
  const mask = occupiedMask(e.grid, e.settings.fieldOccupiedThreshold);
  return likelihoodField(mask, e.grid, e.settings.fieldSigma);
}

/**
 * @typedef {object} EstimatorErrors
 * @property {number} odom current position error of wheel odometry, m
 * @property {number} est of odometry + scan matching
 * @property {number} mcl of MCL (NaN when off)
 * @property {number} odomHeading rad
 * @property {number} estHeading
 * @property {number} mclHeading
 * @property {number} odomRms over every scan so far
 * @property {number} estRms
 * @property {number} mclRms
 * @property {number} acceptRate fraction of scan matches accepted (NaN before the first)
 */

/**
 * Current and RMS errors against a true base pose.
 * @param {Estimators} e
 * @param {Pose2} truth
 * @returns {EstimatorErrors}
 */
export function errors(e, truth) {
  const n = Math.max(1, e.samples);
  const head = (/** @type {Pose2 | null} */ p) =>
    p ? Math.abs(normalizeAngle(truth.theta - p.theta)) : NaN;
  return {
    odom: positionError(truth, e.odom),
    est: positionError(truth, e.est),
    mcl: e.mcl ? positionError(truth, e.mcl) : NaN,
    odomHeading: head(e.odom),
    estHeading: head(e.est),
    mclHeading: head(e.mcl),
    odomRms: Math.sqrt(e.sumSqOdom / n),
    estRms: Math.sqrt(e.sumSqEst / n),
    mclRms: e.pf ? Math.sqrt(e.sumSqMcl / n) : NaN,
    acceptRate: e.matchAttempts === 0 ? NaN : e.matchAccepted / e.matchAttempts,
  };
}

export { DEFAULT_MATCH };
