// The simulation loop. Fixed step: one step is one control period of the
// wheel controller (100 Hz, encoder-odometry-firmware's control rate), with
// the motors integrated in 10 sub-steps. Per step:
//
//   twist command -> firmware: IK, saturation, PI per wheel -> motor duty
//   -> plant: motor speed, wheel angle, encoder edges
//   -> ground speed = wheel speed x (1 + radius error) x (1 - slip)
//   -> forward kinematics (measured wheel positions) -> body twist
//   -> pose on SE(2), blocked by obstacles (lidar-slam-lab's collision test)
//   -> IMU at its URDF link; every 1/rate s a LiDAR scan from its URDF link
//      and an estimator update (wheel odometry, + scan matching, MCL).
//
// Deterministic: all randomness comes from seeded lidar-slam-lab
// generators, one stream per noise source, and nothing reads the clock.

import { createRng } from "../../../vendor/lidar-slam-lab/src/rng.js";
import { simulateScan } from "../../../vendor/lidar-slam-lab/src/lidar.js";
import { collides } from "../../../vendor/lidar-slam-lab/src/world.js";
import {
  mul3,
  rpyToMatrix,
  transpose3,
  mulVec,
} from "../../../vendor/urdf-inspector/src/core/math.js";
import { mecanumForward } from "../builder/drive.js";
import { DIFFERENTIAL, DriveController, FW_DEFAULTS } from "./firmware.js";
import { Plant } from "./plant.js";
import { compose, inverse, se2Exp, se2Log, wrapAngle } from "./se2.js";
import { odometryDelta } from "../../../vendor/lidar-slam-lab/src/motion.js";
import { obstacles, segmentsAbove } from "./world.js";
import {
  ImuSensor,
  defaultImuModel,
  imuModelFromCalibration,
  imuModelFromGazebo,
  G0,
} from "./imu.js";
import {
  DEFAULT_ESTIMATORS,
  DEFAULT_MATCH,
  createEstimators,
  errors,
  firstScan,
  updateEstimators,
} from "./estimators.js";
import { ticksPerWheelRev } from "./settings.js";
import { depthCloud } from "./depth.js";

/** @typedef {import("./se2.js").Pose2} Pose2 */
/** @typedef {import("./firmware.js").Twist} Twist */
/** @typedef {import("./robot.js").SimRobot} SimRobot */
/** @typedef {import("./robot.js").SimSensor} SimSensor */
/** @typedef {import("./world.js").SimWorld} SimWorld */
/** @typedef {import("./settings.js").SimSettings} SimSettings */
/** @typedef {import("./imu.js").ImuReading} ImuReading */
/** @typedef {import("./imu.js").ImuErrorModel} ImuErrorModel */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/lidar.js").Scan} Scan */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/lidar.js").LidarConfig} LidarConfig */
/** @typedef {import("../../../vendor/lidar-slam-lab/src/geometry.js").Segment} Segment */
/** @typedef {import("./estimators.js").Estimators} Estimators */

export const CONTROL_RATE = FW_DEFAULTS.controlRateHz;
export const DT = 1 / CONTROL_RATE;
export const SUBSTEPS = 10;

/**
 * The LiDAR as simulated: lidar-slam-lab's config plus the rest of a
 * sensor_msgs/LaserScan.
 * @typedef {object} LidarModel
 * @property {SimSensor} sensor
 * @property {LidarConfig} config beams, fov, maxRange, noise, dropout
 * @property {number} minRange
 * @property {number} rateHz
 * @property {number} every scan every this many control periods
 * @property {number} angleMin rad, in the sensor frame
 * @property {number} angleIncrement
 * @property {Pose2} mount scan origin in the base frame (sensor mount turned to the middle of the fan)
 * @property {string} source "settings" or "URDF <gazebo> tag"
 * @property {Segment[]} segments obstacles taller than the scan plane
 */

/**
 * The resolved LiDAR model for a sensor: settings, overridden by a Gazebo
 * <sensor type="ray"> tag on that link when useUrdf is on.
 * @param {SimSensor} sensor
 * @param {SimSettings["lidar"]} s
 * @param {SimWorld} world
 * @returns {LidarModel}
 */
export function lidarModel(sensor, s, world) {
  let beams = s.beams;
  let fov = (s.fovDeg * Math.PI) / 180;
  let center = 0;
  let minRange = s.minRange;
  let maxRange = s.maxRange;
  let noise = s.rangeNoise;
  let rate = s.rateHz;
  let source = "settings";
  const g = sensor.gazebo;
  if (s.useUrdf && g && /ray|lidar/.test(g.type)) {
    source = "URDF <gazebo> tag";
    if (g.samples !== undefined && g.samples >= 2) beams = Math.round(g.samples);
    if (g.minAngle !== undefined && g.maxAngle !== undefined && g.maxAngle > g.minAngle) {
      fov = Math.min(2 * Math.PI, g.maxAngle - g.minAngle);
      center = (g.minAngle + g.maxAngle) / 2;
    }
    if (g.rangeMin !== undefined) minRange = g.rangeMin;
    if (g.rangeMax !== undefined) maxRange = g.rangeMax;
    if (g.rangeNoise !== undefined) noise = g.rangeNoise;
    if (g.updateRate !== undefined && g.updateRate > 0) rate = g.updateRate;
  }
  const full = fov >= 2 * Math.PI - 1e-3;
  if (full) {
    fov = 2 * Math.PI;
    center = 0;
  }
  const every = Math.max(1, Math.round(CONTROL_RATE / rate));
  const inc = full ? fov / beams : fov / (beams - 1);
  return {
    sensor,
    config: { beams, fov, maxRange, rangeNoise: noise, dropout: s.dropout },
    minRange,
    rateHz: CONTROL_RATE / every,
    every,
    angleMin: full ? 0 : center - fov / 2,
    angleIncrement: inc,
    mount: compose(sensor.planar, { x: 0, y: 0, theta: center }),
    source,
    segments: segmentsAbove(world, sensor.height),
  };
}

/**
 * @typedef {object} ImuModel
 * @property {SimSensor} sensor
 * @property {ImuSensor} imu
 * @property {number} every sample every this many control periods
 * @property {import("../../../vendor/urdf-inspector/src/core/math.js").Mat3} R sensor orientation in the base frame
 * @property {string} source
 */

/**
 * @param {SimSensor} sensor
 * @param {SimSettings["imu"]} s
 * @param {number} seed
 * @returns {ImuModel}
 */
function imuModel(sensor, s, seed) {
  const every = Math.max(1, Math.round(CONTROL_RATE / s.rateHz));
  const rate = CONTROL_RATE / every;
  /** @type {ImuErrorModel} */
  let model;
  if (s.calibration !== null && s.calibration !== undefined)
    model = imuModelFromCalibration(s.calibration, rate);
  else if (s.useUrdf && sensor.gazebo?.type === "imu")
    model = imuModelFromGazebo(sensor.gazebo, rate);
  else model = defaultImuModel();
  return {
    sensor,
    imu: new ImuSensor(model, rate, createRng(seed ^ 0x27d4eb2f)),
    every,
    R: sensor.mount.R,
    source: model.source,
  };
}

/**
 * Everything the run records, in the encoder-odometry-firmware's and
 * ROS message layouts. Plain arrays of rows so the CLI and the page can
 * write them out (see record.js).
 * @typedef {object} SimLog
 * @property {{ t: number, truth: Pose2, odom: Pose2, est: Pose2, mcl: Pose2 | null, cmd: Twist, actual: Twist }[]} trajectory
 * @property {{ t: number, ranges: Float64Array }[]} scans
 * @property {{ t: number, accel: number[], gyro: number[] }[]} imu
 * @property {ReturnType<DriveController["wheelState"]>[]} wheelState
 * @property {ReturnType<DriveController["odometry"]>[]} odometry
 * @property {{ t: number, position: number[], velocity: number[] }[]} jointStates
 */

/**
 * @typedef {object} Sim
 * @property {SimRobot} robot
 * @property {SimWorld} world
 * @property {SimSettings} settings
 * @property {DriveController} controller
 * @property {Plant} plant
 * @property {number[]} slotWheel physical wheel for each firmware slot
 * @property {number} ticksPerRev
 * @property {number} steps control periods so far
 * @property {number} t s
 * @property {Pose2} drive true pose of the drive frame (wheel centroid)
 * @property {Pose2} truth true pose of the base frame
 * @property {Pose2} startBase
 * @property {Twist} command last command (body frame of the base, as sent)
 * @property {Twist} actual true body twist of the base frame over the last step
 * @property {number[]} slip current slip fraction per wheel
 * @property {import("../../../vendor/lidar-slam-lab/src/rng.js").Rng} slipRng
 * @property {import("../../../vendor/lidar-slam-lab/src/rng.js").Rng} lidarRng
 * @property {Segment[]} solid every obstacle (for collisions)
 * @property {LidarModel | null} lidar primary LiDAR (the estimators use it)
 * @property {Scan | null} lastScan
 * @property {Pose2 | null} lastScanPose true LiDAR pose of the last scan
 * @property {ImuModel | null} imu
 * @property {ImuReading | null} lastImu
 * @property {Estimators | null} est
 * @property {Pose2} odomAtScan dead-reckoned drive-frame pose at the last scan
 * @property {number} distance m driven (true)
 * @property {number} collisions steps blocked by an obstacle
 * @property {number} saturated steps with the command scaled down
 * @property {number[]} wheelAngle true angle of each physical wheel, rad
 * @property {Pose2[]} imuHist IMU-point positions for finite differences
 * @property {SimLog | null} log
 * @property {Float32Array | null} cloud last depth point cloud (x, y, z in the world)
 * @property {number} cloudPoints
 * @property {SimSensor | null} depth
 * @property {string[]} notes
 * @property {(m: number[]) => Twist} fk true forward kinematics of ground wheel speeds
 */

/**
 * @param {SimRobot} robot
 * @param {SimWorld} world
 * @param {SimSettings} settings
 * @param {{ log?: boolean }} [opts]
 * @returns {Sim}
 */
export function createSim(robot, world, settings, opts = {}) {
  const d = settings.drive;
  const ticks = ticksPerWheelRev(d);
  const controller = new DriveController({
    geometry: robot.geometry,
    ticksPerWheelRev: ticks,
    maxWheelSpeed: d.maxWheelSpeed > 0 ? d.maxWheelSpeed : robot.maxWheelSpeed,
    pid: { ...d.pid },
    velocity: { ...FW_DEFAULTS.velocity },
    watchdogTimeoutMs: FW_DEFAULTS.watchdogTimeoutMs,
    controlRateHz: CONTROL_RATE,
    telemetryDivider: FW_DEFAULTS.telemetryDivider,
  });
  const plant = new Plant(robot.wheels.length, ticks, d.motor, SUBSTEPS);
  const startBase = { ...world.start };
  const drive = compose(startBase, robot.drivePose);
  /** @type {string[]} */
  const notes = [];
  const lidars = robot.sensors.filter((s) => s.kind === "lidar2d" && s.level);
  const lidar = lidars.length > 0 ? lidarModel(lidars[0], settings.lidar, world) : null;
  if (lidars.length > 1)
    notes.push(
      `${lidars.length} LiDARs; ${lidars[0].link} is simulated and used by the estimators`,
    );
  if (!lidar) notes.push("no level link tagged as a 2D LiDAR: no scans, scan matching or MCL");
  const imus = robot.sensors.filter((s) => s.kind === "imu");
  const imu = imus.length > 0 ? imuModel(imus[0], settings.imu, settings.seed) : null;
  if (!imu) notes.push("no link tagged as an IMU: no IMU readings");
  const depthSensor = robot.sensors.find((s) => s.kind === "depth_camera") ?? null;

  /** @type {Sim} */
  const sim = {
    robot,
    world,
    settings,
    controller,
    plant,
    slotWheel: robot.slots,
    ticksPerRev: ticks,
    steps: 0,
    t: 0,
    drive,
    truth: { ...startBase },
    startBase,
    command: { vx: 0, vy: 0, wz: 0 },
    actual: { vx: 0, vy: 0, wz: 0 },
    slip: new Array(robot.wheels.length).fill(0),
    slipRng: createRng(settings.seed ^ 0x9e3779b9),
    lidarRng: createRng(settings.seed),
    solid: obstacles(world).segments,
    lidar,
    lastScan: null,
    lastScanPose: null,
    imu,
    lastImu: null,
    est: null,
    odomAtScan: { ...drive },
    distance: 0,
    collisions: 0,
    saturated: 0,
    wheelAngle: new Array(robot.wheels.length).fill(0),
    imuHist: [],
    log: opts.log
      ? { trajectory: [], scans: [], imu: [], wheelState: [], odometry: [], jointStates: [] }
      : null,
    cloud: null,
    cloudPoints: 0,
    depth: settings.depth.enabled ? depthSensor : null,
    notes,
    fk: forwardKinematics(robot),
  };
  if (lidar) {
    // The estimators track the drive frame; the LiDAR's scan origin in it:
    const mount = compose(inverse(robot.drivePose), lidar.mount);
    sim.est = createEstimators(world, lidar.segments, drive, mount, {
      ...DEFAULT_ESTIMATORS,
      ...settings.estimators,
      seed: settings.seed,
      match: { ...DEFAULT_MATCH },
      clearance: robot.footprint.radius,
    });
    const scan = takeScan(sim);
    firstScan(sim.est, scan);
  }
  sim.imuHist.push(imuPoint(sim), imuPoint(sim));
  return sim;
}

/**
 * True body twist of the drive frame from the ground speed of each
 * physical wheel (rad/s, positive forward): least squares over the
 * measured wheel positions for mecanum, the axle formula for differential.
 * @param {SimRobot} robot
 * @returns {(w: number[]) => Twist}
 */
function forwardKinematics(robot) {
  if (robot.drive === DIFFERENTIAL) {
    const [L, R] = robot.wheels;
    const sep = L.y - R.y;
    return (w) => ({
      vx: 0.5 * (L.radius * w[0] + R.radius * w[1]),
      vy: 0,
      wz: (R.radius * w[1] - L.radius * w[0]) / sep,
    });
  }
  const g = {
    radius: robot.geometry.wheelRadius,
    wheels: robot.wheels.map((w) => ({
      label: w.label,
      joint: w.joint,
      x: w.x,
      y: w.y,
      sign: w.sign,
    })),
  };
  // Forward kinematics is linear: build its 3x4 matrix once.
  const cols = [0, 1, 2, 3].map((k) =>
    mecanumForward(
      g,
      [0, 1, 2, 3].map((i) => (i === k ? 1 : 0)),
    ),
  );
  return (w) => ({
    vx: cols[0].vx * w[0] + cols[1].vx * w[1] + cols[2].vx * w[2] + cols[3].vx * w[3],
    vy: cols[0].vy * w[0] + cols[1].vy * w[1] + cols[2].vy * w[2] + cols[3].vy * w[3],
    wz: cols[0].wz * w[0] + cols[1].wz * w[1] + cols[2].wz * w[2] + cols[3].wz * w[3],
  });
}

/**
 * Snapshots in firmware slot order (FL, FR, RL, RR).
 * @param {Sim} sim
 */
const slotSnapshots = (sim) => {
  const s = sim.plant.snapshots();
  return sim.slotWheel.map((w) => s[w]);
};

/**
 * Duty (or ideal speed) per physical wheel from the slot values: the first
 * slot that drives each wheel.
 * @param {Sim} sim
 * @param {number[]} slotValues
 */
const perWheel = (sim, slotValues) =>
  sim.robot.wheels.map((_, i) => slotValues[sim.slotWheel.indexOf(i)]);

/**
 * Advance one control period with a twist command (base frame, as a
 * cmd_vel would be).
 * @param {Sim} sim
 * @param {Twist} cmd
 */
export function step(sim, cmd) {
  const c = sim.controller;
  const nowUs = sim.plant.nowUs;
  sim.command = { ...cmd };
  c.command(cmd, nowUs);
  if (c.scale < 1) sim.saturated++;
  let input;
  if (sim.settings.drive.ideal) {
    c.step(slotSnapshots(sim), nowUs);
    input = perWheel(sim, c.targets);
  } else input = perWheel(sim, c.step(slotSnapshots(sim), nowUs));
  // Telemetry as the firmware sends it: every telemetry_divider periods,
  // after the control step and before the plant moves on.
  if (sim.steps % c.config.telemetryDivider === 0) recordTelemetry(sim);

  // Slip: a first-order Gauss-Markov process per wheel, updated once per period.
  const d = sim.settings.drive;
  const n = sim.slip.length;
  const a = Math.exp(-DT / d.slipTau);
  const b = Math.sqrt(1 - a * a);
  for (let i = 0; i < n; i++) {
    const z = sim.slipRng.gaussian();
    sim.slip[i] = d.slip ? a * sim.slip[i] + d.slipStd * b * z : 0;
  }
  const radiusScale = sim.robot.wheels.map(
    (w) => 1 + (w.y > 0 ? d.radiusErrorLeft : d.radiusErrorRight),
  );

  const before = { ...sim.drive };
  const prevBase = sim.truth;
  let blocked = false;
  sim.plant.advance(
    input,
    DT,
    (mean, h) => {
      const ground = mean.map((m, i) => m * radiusScale[i] * (1 - sim.slip[i]));
      for (let i = 0; i < n; i++) sim.wheelAngle[i] += mean[i] * h;
      const tw = sim.fk(ground);
      const next = compose(sim.drive, se2Exp(tw.vx * h, tw.vy * h, tw.wz * h));
      const fp = sim.robot.footprint;
      if (collides(sim.solid, sim.drive.x, sim.drive.y, next.x, next.y, fp.radius)) {
        sim.drive = { x: sim.drive.x, y: sim.drive.y, theta: next.theta };
        blocked = true;
      } else sim.drive = next;
    },
    sim.settings.drive.ideal,
  );
  if (blocked) sim.collisions++;
  sim.drive.theta = wrapAngle(sim.drive.theta);
  sim.distance += Math.hypot(sim.drive.x - before.x, sim.drive.y - before.y);
  sim.truth = baseOf(sim, sim.drive);
  // Actual twist of the base frame over the period (comparable with the command).
  const rel = compose(inverse(prevBase), sim.truth);
  const tw = se2Log({ x: rel.x, y: rel.y, theta: wrapAngle(rel.theta) });
  sim.actual = { vx: tw.vx / DT, vy: tw.vy / DT, wz: tw.wz / DT };
  sim.steps++;
  sim.t = sim.steps * DT;

  // IMU
  sim.imuHist.push(imuPoint(sim));
  if (sim.imuHist.length > 3) sim.imuHist.shift();
  if (sim.imu && sim.steps % sim.imu.every === 0) sim.lastImu = readImu(sim);

  // LiDAR, estimators
  if (sim.lidar && sim.est && sim.steps % sim.lidar.every === 0) {
    const scan = takeScan(sim);
    const odomNow = odometryDrive(sim);
    const measured = odometryDelta(sim.odomAtScan, odomNow);
    sim.odomAtScan = odomNow;
    updateEstimators(sim.est, measured, scan, sim.drive);
  }
  if (
    sim.depth &&
    sim.steps % Math.max(1, Math.round(CONTROL_RATE / sim.settings.depth.rateHz)) === 0
  ) {
    const r = depthCloud(sim.world, sim.truth, sim.robot.baseHeight, sim.depth, sim.settings.depth);
    sim.cloud = r.points;
    sim.cloudPoints = r.count;
  }
  if (sim.log) logStep(sim);
}

/**
 * Base frame pose from a drive frame pose.
 * @param {Sim} sim
 * @param {Pose2} drive
 */
function baseOf(sim, drive) {
  const p = compose(drive, inverse(sim.robot.drivePose));
  return { x: p.x, y: p.y, theta: wrapAngle(p.theta) };
}

/**
 * The firmware's wheel odometry as a base pose in the world frame (the
 * firmware starts at zero in the drive frame at the start pose).
 * @param {Sim} sim
 */
export const odometryBase = (sim) => baseOf(sim, odometryDrive(sim));

/**
 * The firmware's wheel odometry as a drive-frame pose in the world frame.
 * @param {Sim} sim
 */
function odometryDrive(sim) {
  const p = compose(compose(sim.startBase, sim.robot.drivePose), sim.controller.pose);
  return { x: p.x, y: p.y, theta: wrapAngle(p.theta) };
}

/**
 * Estimates as base-frame poses (the estimators track the drive frame).
 * @param {Sim} sim
 */
export function estimates(sim) {
  const e = sim.est;
  return {
    odom: odometryBase(sim),
    est: e ? baseOf(sim, e.est) : null,
    mcl: e && e.mcl ? baseOf(sim, e.mcl) : null,
  };
}

/**
 * @param {Sim} sim
 * @returns {Scan}
 */
function takeScan(sim) {
  const l = /** @type {LidarModel} */ (sim.lidar);
  const pose = compose(sim.truth, l.mount);
  const scan = simulateScan(l.segments, pose, l.config, sim.lidarRng);
  for (let i = 0; i < scan.ranges.length; i++)
    if (scan.ranges[i] < l.minRange) scan.ranges[i] = NaN;
  sim.lastScan = scan;
  sim.lastScanPose = pose;
  if (sim.log) sim.log.scans.push({ t: sim.t, ranges: scan.ranges });
  return scan;
}

/**
 * World position (x, y) and heading of the IMU point.
 * @param {Sim} sim
 * @returns {Pose2}
 */
function imuPoint(sim) {
  const m = sim.imu ? sim.imu.sensor.planar : { x: 0, y: 0, theta: 0 };
  return compose(sim.truth, { x: m.x, y: m.y, theta: 0 });
}

/**
 * True specific force and angular rate at the IMU from the last three
 * positions of the IMU point (second differences over one period), then
 * the error model.
 * @param {Sim} sim
 * @returns {ImuReading}
 */
function readImu(sim) {
  const im = /** @type {ImuModel} */ (sim.imu);
  const [p0, p1, p2] =
    sim.imuHist.length === 3 ? sim.imuHist : [sim.imuHist[0], sim.imuHist[0], sim.imuHist[1]];
  const ax = (p2.x - 2 * p1.x + p0.x) / (DT * DT);
  const ay = (p2.y - 2 * p1.y + p0.y) / (DT * DT);
  const wz = wrapAngle(p2.theta - p1.theta) / DT;
  // World -> sensor: R_ws = Rz(theta) R_mount.
  const Rws = mul3(rpyToMatrix([0, 0, sim.truth.theta]), im.R);
  const RT = transpose3(Rws);
  const f = /** @type {[number, number, number]} */ (mulVec(RT, [ax, ay, G0]));
  const w = /** @type {[number, number, number]} */ (mulVec(RT, [0, 0, wz]));
  const r = im.imu.read(f, w);
  if (sim.log) sim.log.imu.push({ t: sim.t, accel: r.accel, gyro: r.gyro });
  return r;
}

/** @param {Sim} sim */
function recordTelemetry(sim) {
  if (!sim.log) return;
  sim.log.wheelState.push(sim.controller.wheelState(sim.plant.nowUs));
  sim.log.odometry.push(sim.controller.odometry(sim.plant.nowUs));
}

/** @param {Sim} sim */
function logStep(sim) {
  const log = /** @type {SimLog} */ (sim.log);
  const e = sim.est;
  log.trajectory.push({
    t: sim.t,
    truth: sim.truth,
    odom: odometryBase(sim),
    est: e ? baseOf(sim, e.est) : sim.truth,
    mcl: e && e.mcl ? baseOf(sim, e.mcl) : null,
    cmd: sim.command,
    actual: sim.actual,
  });
  log.jointStates.push({ t: sim.t, position: jointPositions(sim), velocity: jointVelocities(sim) });
}

/**
 * Wheel joint positions (rad): joint axis sign x wheel angle.
 * @param {Sim} sim
 */
export const jointPositions = (sim) => sim.robot.wheels.map((w, i) => w.sign * sim.wheelAngle[i]);

/** @param {Sim} sim */
export const jointVelocities = (sim) => sim.robot.wheels.map((w, i) => w.sign * sim.plant.speed[i]);

/**
 * Joint positions for the 3D view (wheel joints only).
 * @param {Sim} sim
 * @returns {Record<string, number>}
 */
export function jointMap(sim) {
  /** @type {Record<string, number>} */
  const q = {};
  const pos = jointPositions(sim);
  sim.robot.wheels.forEach((w, i) => (q[w.joint] = pos[i]));
  return q;
}

/**
 * Current errors of the estimators against the truth (null without a LiDAR),
 * measured at the drive frame (wheel centroid; base_link on the templates'
 * mecanum base).
 * @param {Sim} sim
 */
export const simErrors = (sim) => (sim.est ? errors(sim.est, sim.drive) : null);
