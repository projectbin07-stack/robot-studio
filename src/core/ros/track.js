// What the views need from data that was not simulated here: a robot's
// odometry, scans, IMU, joints and wheel telemetry arrive (from a replayed
// recording or a live rosbridge connection), the same lidar-slam-lab
// estimators as the simulation run on them (scan matching on the wheel
// odometry, MCL when a floor plan is known), and the result is a ViewFrame
// for the 3D layer plus the plot histories, in the shape SimSession has.
// Error measures need a ground truth, which only a simulated recording has.

import { odometryDelta } from "../../../vendor/lidar-slam-lab/src/motion.js";
import { compose } from "../sim/se2.js";
import {
  DEFAULT_MATCH,
  createEstimators,
  errors,
  firstScan,
  updateEstimators,
} from "../sim/estimators.js";
import { DEFAULT_ESTIMATORS } from "../sim/estimators.js";
import { History, HISTORY_SECONDS } from "../sim/session.js";

/** @typedef {import("../sim/se2.js").Pose2} Pose2 */
/** @typedef {import("../sim/estimators.js").Estimators} Estimators */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/lidar.js").Scan} Scan */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/geometry.js").Segment} Segment */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/grid.js").OccupancyGrid} OccupancyGrid */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/mcl.js").ParticleFilter} ParticleFilter */

/**
 * Everything the 3D layer draws in one frame, whatever the source.
 * @typedef {object} ViewFrame
 * @property {number} t s
 * @property {Pose2} base where the robot's base frame is drawn
 * @property {Record<string, number>} q joint positions for the model
 * @property {Scan | null} scan
 * @property {Pose2 | null} scanPose LiDAR pose of that scan, same frame as `base`
 * @property {number} lidarHeight m above the floor
 * @property {{ truth: Pose2 | null, odom: Pose2 | null, est: Pose2 | null, mcl: Pose2 | null }} trails
 * @property {ParticleFilter | null} particles in the tracked frame
 * @property {Pose2} particleOffset base frame in the particles' tracked frame
 * @property {Float32Array | null} cloud
 * @property {number} cloudPoints
 * @property {OccupancyGrid | null} grid
 * @property {Pose2} mapFrame where the map frame's axes go
 * @property {Pose2 | null} odomFrame where the odom frame's axes go
 */

/**
 * @typedef {object} TrackOptions
 * @property {Pose2} mount LiDAR frame in the tracked (base) frame
 * @property {number} lidarHeight m
 * @property {Pose2 | null} [start] start pose of the tracked frame (default: the first odometry)
 * @property {{ width: number, height: number, x0?: number, y0?: number } | null} [area] map
 *   extent; default 40 m x 40 m around the start
 * @property {readonly Segment[]} [knownWalls] floor plan for MCL (none: MCL off)
 * @property {{ seed?: number, scanMatching?: boolean, particles?: number }} [estimators]
 * @property {boolean} [estimate] run the estimators at all (default true)
 */

const EARLY_SCAN = "scans before the first odometry are shown, not used by the estimators";

export class Track {
  /** @param {TrackOptions} opts */
  constructor(opts) {
    this.opts = opts;
    /** @type {Estimators | null} */
    this.est = null;
    /** @type {Pose2 | null} latest odometry pose of the tracked frame */
    this.odom = null;
    /** @type {Pose2 | null} odometry at the last integrated scan */
    this.odomAtScan = null;
    /** @type {Pose2 | null} */
    this.truth = null;
    /** @type {Scan | null} */
    this.lastScan = null;
    /** @type {Pose2 | null} */
    this.lastScanPose = null;
    /** @type {Record<string, number>} */
    this.q = {};
    this.t = 0;
    this.scans = 0;
    /** @type {string[]} */
    this.notes = [];
    const n = Math.round(HISTORY_SECONDS / 0.05);
    this.history = {
      wheels: new History(8, n), // target then measured, FL FR RL RR
      twist: new History(6, n), // commanded vx vy wz, actual vx vy wz
      imu: new History(6, n),
      errors: new History(3, n), // wheel odometry, scan matching, MCL vs truth
    };
    /** @type {number[]} */
    this.wheelNow = new Array(8).fill(NaN);
    /** @type {number[]} */
    this.twistNow = new Array(6).fill(NaN);
    /** @type {number[]} */
    this.imuNow = new Array(6).fill(NaN);
    this.lastSample = -Infinity;
  }

  /**
   * Time moves on (each input calls this); samples the plots every 50 ms.
   * @param {number} t
   */
  at(t) {
    if (t > this.t) this.t = t;
    if (this.t - this.lastSample >= 0.05 - 1e-9) this.sample();
  }

  sample() {
    this.lastSample = this.t;
    this.history.wheels.push(this.t, this.wheelNow);
    this.history.twist.push(this.t, this.twistNow);
    this.history.imu.push(this.t, this.imuNow);
    const e = this.errors();
    this.history.errors.push(this.t, e ? [e.odom, e.est, e.mcl] : [NaN, NaN, NaN]);
  }

  /**
   * @param {number} t
   * @param {Pose2} pose odometry of the tracked frame
   */
  odometry(t, pose) {
    this.odom = { ...pose };
    this.at(t);
  }

  /**
   * @param {number} t
   * @param {Pose2} pose true pose of the tracked frame (simulated recordings only)
   */
  groundTruth(t, pose) {
    this.truth = { ...pose };
    this.at(t);
  }

  /**
   * A scan as the LiDAR took it (angles in the LiDAR frame, NaN = no return).
   * @param {number} t
   * @param {Scan} scan
   */
  scan(t, scan) {
    this.at(t);
    this.scans++;
    this.lastScan = scan;
    const odom = this.odom;
    if (!odom) {
      if (!this.notes.includes(EARLY_SCAN)) this.notes.push(EARLY_SCAN);
      this.lastScanPose = compose(this.pose() ?? { x: 0, y: 0, theta: 0 }, this.opts.mount);
      return;
    }
    if (this.opts.estimate !== false) {
      if (!this.est) this.createEstimators(odom);
      const e = /** @type {Estimators} */ (this.est);
      if (!this.odomAtScan) firstScan(e, scan);
      else {
        const truth = this.truth ?? { x: NaN, y: NaN, theta: NaN };
        updateEstimators(e, odometryDelta(this.odomAtScan, odom), scan, truth);
      }
    }
    this.odomAtScan = { ...odom };
    this.lastScanPose = compose(/** @type {Pose2} */ (this.pose()), this.opts.mount);
  }

  /** @param {Pose2} odom */
  createEstimators(odom) {
    const start = this.opts.start ?? odom;
    const area = this.opts.area ?? { width: 40, height: 40, x0: start.x - 20, y0: start.y - 20 };
    const walls = this.opts.knownWalls ?? [];
    const s = this.opts.estimators ?? {};
    this.est = createEstimators(area, walls, start, this.opts.mount, {
      ...DEFAULT_ESTIMATORS,
      scanMatching: s.scanMatching ?? true,
      particles: walls.length > 0 ? (s.particles ?? DEFAULT_ESTIMATORS.particles) : 0,
      seed: s.seed ?? 1,
      match: { ...DEFAULT_MATCH },
      clearance: 0.2,
    });
    // The estimators start at `start`; the first scan is integrated there.
    this.est.odom = { ...odom };
    this.est.est = this.opts.start ? { ...start } : { ...odom };
  }

  /**
   * @param {number} t
   * @param {number[]} accel m/s^2
   * @param {number[]} gyro rad/s
   */
  imu(t, accel, gyro) {
    this.imuNow = [...accel, ...gyro];
    this.at(t);
  }

  /**
   * @param {number} t
   * @param {Record<string, number>} positions joint name -> rad
   */
  joints(t, positions) {
    Object.assign(this.q, positions);
    this.at(t);
  }

  /**
   * @param {number} t
   * @param {number[]} target rad/s FL FR RL RR (NaN when unknown)
   * @param {number[]} measured
   */
  wheels(t, target, measured) {
    this.wheelNow = [...target, ...measured];
    this.at(t);
  }

  /**
   * @param {number} t
   * @param {{ vx: number, vy: number, wz: number } | null} cmd
   * @param {{ vx: number, vy: number, wz: number } | null} actual
   */
  twist(t, cmd, actual) {
    if (cmd) this.twistNow.splice(0, 3, cmd.vx, cmd.vy, cmd.wz);
    if (actual) this.twistNow.splice(3, 3, actual.vx, actual.vy, actual.wz);
    this.at(t);
  }

  /** Best pose of the tracked frame: scan matching, else odometry. */
  pose() {
    if (this.est && this.est.scansIntegrated > 0 && this.opts.estimators?.scanMatching !== false)
      return this.est.est;
    return this.odom;
  }

  /** Errors against the truth (null without a truth or estimators). */
  errors() {
    return this.est && this.truth ? errors(this.est, this.truth) : null;
  }

  /**
   * The ViewFrame: the robot drawn at the truth when there is one (simulated
   * recordings), else at the best estimate.
   * @returns {ViewFrame}
   */
  frame() {
    const zero = { x: 0, y: 0, theta: 0 };
    const best = this.pose();
    const base = this.truth ?? best ?? zero;
    const e = this.est;
    const start = this.opts.start ?? (e ? null : this.odom);
    return {
      t: this.t,
      base,
      q: this.q,
      scan: this.lastScan,
      scanPose: this.lastScan ? compose(base, this.opts.mount) : null,
      lidarHeight: this.opts.lidarHeight,
      trails: {
        truth: this.truth,
        odom: this.odom,
        est: e && this.opts.estimators?.scanMatching !== false ? e.est : null,
        mcl: e?.mcl ?? null,
      },
      particles: e?.pf ?? null,
      particleOffset: zero,
      cloud: null,
      cloudPoints: 0,
      grid: e?.grid ?? null,
      mapFrame: this.opts.start ?? start ?? zero,
      odomFrame: null,
    };
  }
}
