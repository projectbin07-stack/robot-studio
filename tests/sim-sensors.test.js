// Simulated sensors against hand-computed values: sensor poses from the
// URDF's forward kinematics, LiDAR ranges to known walls (and obstacles
// below the scan plane), Gazebo <sensor> parameters, IMU gravity, bias,
// rotation rate and centripetal acceleration, imu-calibration-lab's JSON,
// the depth camera, and the estimator wrapper against lidar-slam-lab's own
// loop.

import { test } from "node:test";
import assert from "node:assert/strict";
import { fileURLToPath } from "node:url";
import { generate } from "../src/core/builder/templates.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import { applyOp } from "../src/core/builder/ops.js";
import { importUrdf } from "../src/core/builder/urdf.js";
import { loadModel } from "../src/core/model.js";
import { NodeFs, abs } from "../src/node/nodefs.js";
import { gazeboSensors, simRobot } from "../src/core/sim/robot.js";
import { bundledWorld, parseSimWorld, segmentsAbove } from "../src/core/sim/world.js";
import { normalizeSettings } from "../src/core/sim/settings.js";
import { createSim, lidarModel, step } from "../src/core/sim/sim.js";
import {
  G0,
  ImuSensor,
  defaultImuModel,
  imuModelFromCalibration,
  imuModelFromGazebo,
  invert3,
} from "../src/core/sim/imu.js";
import { createEstimators, virtualScan } from "../src/core/sim/estimators.js";
import { referenceRun } from "../src/core/sim/reference.js";
import { depthCloud } from "../src/core/sim/depth.js";
import { createRng } from "../vendor/lidar-slam-lab/src/rng.js";
import { BUILTIN_MAPS } from "../vendor/lidar-slam-lab/src/maps.js";
import { followRoute } from "../vendor/lidar-slam-lab/src/autopilot.js";
import {
  createSimulation,
  defaultSettings,
  initialScan,
  step as lslStep,
  summarize,
} from "../vendor/lidar-slam-lab/src/sim.js";

const root = fileURLToPath(new URL("..", import.meta.url));

/** @param {import("../src/core/builder/document.js").RobotDoc} doc */
function robotFromDoc(doc) {
  const r = simRobot(modelFromDoc(doc), doc);
  if (!r.ok) throw new Error(r.reason);
  return { robot: r.robot, notes: r.notes };
}

/**
 * A room with walls only, robot in the middle, at a given heading.
 * @param {number} theta
 * @param {object} [extra]
 */
function room(theta = 0, extra = {}) {
  return parseSimWorld({
    version: 1,
    name: "room",
    width: 10,
    height: 10,
    walls: [
      [0, 0, 10, 0],
      [10, 0, 10, 10],
      [10, 10, 0, 10],
      [0, 10, 0, 0],
    ],
    start: { x: 5, y: 5, theta },
    ...extra,
  });
}

const QUIET = {
  lidar: { rangeNoise: 0, dropout: 0, beams: 180 },
  drive: { ideal: true, slip: false, radiusErrorLeft: 0, radiusErrorRight: 0 },
};

test("sensor poses come from the URDF's forward kinematics", () => {
  const doc = generate("mecanum", {});
  const { robot } = robotFromDoc(doc);
  const lidar = robot.sensors.find((s) => s.kind === "lidar2d");
  const imu = robot.sensors.find((s) => s.kind === "imu");
  assert.ok(lidar && imu);
  // Template: lidar_joint at (0.06, 0, 0.07) in base_link, base_link 0.04 m above the floor.
  assert.deepEqual(lidar.planar, { x: 0.06, y: 0, theta: 0 });
  assert.ok(Math.abs(lidar.height - 0.11) < 1e-12);
  assert.ok(lidar.level);
  assert.ok(Math.abs(robot.baseHeight - 0.04) < 1e-12);
  assert.equal(robot.baseFrame, "base_link");
  assert.equal(robot.rootFrame, "base_footprint");
  assert.ok(Math.abs(robot.rootInBase.p[2] + 0.04) < 1e-12);
  // Move and turn the LiDAR mount: the simulated mount follows.
  const moved = applyOp(doc, {
    type: "setOrigin",
    target: { kind: "joint", name: "lidar_joint" },
    origin: { xyz: [0.1, 0.05, 0.2], rpy: [0, 0, 0.4] },
  });
  const m = robotFromDoc(moved).robot.sensors.find((s) => s.kind === "lidar2d");
  assert.ok(m);
  assert.ok(Math.abs(m.planar.x - 0.1) < 1e-12 && Math.abs(m.planar.y - 0.05) < 1e-12);
  assert.ok(Math.abs(m.planar.theta - 0.4) < 1e-12);
  assert.ok(Math.abs(m.height - 0.24) < 1e-12);
  // A tilted LiDAR is reported and not simulated.
  const tilted = applyOp(doc, {
    type: "setOrigin",
    target: { kind: "joint", name: "lidar_joint" },
    origin: { xyz: [0.06, 0, 0.07], rpy: [0.3, 0, 0] },
  });
  const t = robotFromDoc(tilted);
  assert.equal(t.robot.sensors.find((s) => s.kind === "lidar2d")?.level, false);
  assert.ok(t.notes.some((n) => /not level/.test(n)));
  const sim = createSim(t.robot, room(), normalizeSettings({}));
  assert.equal(sim.lidar, null);
  assert.ok(sim.notes.some((n) => /no level link tagged as a 2D LiDAR/.test(n)));
});

test("LiDAR ranges to the walls of a square room match hand-computed distances", () => {
  for (const theta of [0, 0.3]) {
    const { robot } = robotFromDoc(generate("mecanum", {}));
    const sim = createSim(robot, room(theta), normalizeSettings(QUIET));
    const scan = /** @type {NonNullable<typeof sim.lastScan>} */ (sim.lastScan);
    assert.equal(scan.ranges.length, 180);
    // The LiDAR sits 0.06 m ahead of the centre at (5, 5).
    const sx = 5 + 0.06 * Math.cos(theta);
    const sy = 5 + 0.06 * Math.sin(theta);
    for (let i = 0; i < 180; i += 7) {
      const a = theta + scan.angles[i];
      const c = Math.cos(a);
      const s = Math.sin(a);
      const tx = c > 1e-12 ? (10 - sx) / c : c < -1e-12 ? -sx / c : Infinity;
      const ty = s > 1e-12 ? (10 - sy) / s : s < -1e-12 ? -sy / s : Infinity;
      const expect = Math.min(tx, ty);
      assert.ok(
        Math.abs(scan.ranges[i] - expect) < 1e-9,
        `beam ${i}: ${scan.ranges[i]} vs ${expect}`,
      );
    }
    if (theta === 0) {
      assert.ok(Math.abs(scan.ranges[0] - 4.94) < 1e-12);
      assert.ok(Math.abs(scan.ranges[90] - 5.06) < 1e-12);
    }
  }
  // Ranges below the minimum range are no return.
  const { robot } = robotFromDoc(generate("mecanum", {}));
  const near = createSim(
    robot,
    room(),
    normalizeSettings({ ...QUIET, lidar: { ...QUIET.lidar, minRange: 5 } }),
  );
  const r = /** @type {NonNullable<typeof near.lastScan>} */ (near.lastScan).ranges;
  assert.ok(Number.isNaN(r[0]) && r[90] > 5);
});

test("the scan plane sees obstacles taller than the LiDAR and not lower ones", () => {
  const { robot } = robotFromDoc(generate("mecanum", {}));
  const world = room(0, {
    boxes: [{ x: 7, y: 5, w: 0.4, d: 0.4, yaw: 0, h: 0.08 }],
    cylinders: [{ x: 5, y: 7, r: 0.3, h: 1 }],
  });
  assert.equal(segmentsAbove(world, 0.11).length, 4 + 24);
  assert.equal(segmentsAbove(world, 0.05).length, 4 + 4 + 24);
  const sim = createSim(robot, world, normalizeSettings(QUIET));
  const scan = /** @type {NonNullable<typeof sim.lastScan>} */ (sim.lastScan);
  assert.ok(Math.abs(scan.ranges[0] - 4.94) < 1e-12, "the 8 cm box is below the 11 cm scan plane");
  // Beam 45 points at +y: the cylinder's polygon (circumscribed, 24 sides) at about 2 - 0.3.
  assert.ok(scan.ranges[45] > 1.69 && scan.ranges[45] < 1.71, `${scan.ranges[45]}`);
  // The robot still collides with the low box.
  for (let k = 0; k < 800; k++) step(sim, { vx: 0.3, vy: 0, wz: 0 });
  assert.ok(sim.collisions > 0 && sim.truth.x < 6.81);
});

test("Gazebo <sensor> tags set the LiDAR and IMU models (TurtleBot3 Burger)", () => {
  const entry = abs(
    `${root}examples/third_party/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro`,
  );
  const fs = new NodeFs({});
  fs.discoverAround(entry);
  const model = loadModel(fs, entry);
  const doc = importUrdf(model.urdf, { entry }).doc;
  assert.ok(doc);
  const r = simRobot(model, doc);
  assert.ok(r.ok);
  if (!r.ok) return;
  assert.equal(r.robot.drive, "differential");
  assert.equal(r.robot.maxWheelSpeedFrom, "default");
  const scanLink = r.robot.sensors.find((s) => s.kind === "lidar2d");
  assert.equal(scanLink?.link, "base_scan");
  const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld("office"));
  const lm = lidarModel(/** @type {any} */ (scanLink), normalizeSettings({}).lidar, world);
  assert.equal(lm.source, "URDF <gazebo> tag");
  assert.equal(lm.config.beams, 360);
  assert.equal(lm.config.maxRange, 3.5);
  assert.equal(lm.minRange, 0.12);
  assert.equal(lm.config.rangeNoise, 0.01);
  assert.equal(lm.rateHz, 5);
  assert.equal(lm.angleMin, 0);
  const off = lidarModel(
    /** @type {any} */ (scanLink),
    normalizeSettings({ lidar: { useUrdf: false } }).lidar,
    world,
  );
  assert.equal(off.source, "settings");
  assert.equal(off.config.beams, 180);
  const imu = r.robot.sensors.find((s) => s.kind === "imu");
  assert.equal(imu?.gazebo?.gyroNoise, 2e-4);
  assert.equal(imu?.gazebo?.accelBias, 0.1);
  const sim = createSim(r.robot, world, normalizeSettings({}));
  assert.match(sim.imu?.source ?? "", /gazebo/);
  // A partial fan centred off the sensor's x axis.
  const g = gazeboSensors([
    {
      name: "gazebo",
      attrs: { reference: "laser" },
      text: "",
      children: [
        {
          name: "sensor",
          attrs: { type: "gpu_ray", name: "l" },
          text: "",
          children: [
            { name: "update_rate", attrs: {}, text: "20", children: [] },
            {
              name: "ray",
              attrs: {},
              text: "",
              children: [
                {
                  name: "scan",
                  attrs: {},
                  text: "",
                  children: [
                    {
                      name: "horizontal",
                      attrs: {},
                      text: "",
                      children: [
                        { name: "samples", attrs: {}, text: "101", children: [] },
                        { name: "min_angle", attrs: {}, text: "-1", children: [] },
                        { name: "max_angle", attrs: {}, text: "2", children: [] },
                      ],
                    },
                  ],
                },
              ],
            },
          ],
        },
        {
          name: "sensor",
          attrs: { type: "depth", name: "d" },
          text: "",
          children: [
            {
              name: "camera",
              attrs: {},
              text: "",
              children: [{ name: "horizontal_fov", attrs: {}, text: "1.2", children: [] }],
            },
          ],
        },
      ],
    },
  ]);
  const laser = g.get("laser");
  assert.equal(laser?.type, "depth"); // the last sensor of a link wins
  const fan = lidarModel(
    /** @type {any} */ ({
      ...scanLink,
      gazebo: {
        type: "gpu_ray",
        name: "l",
        samples: 101,
        minAngle: -1,
        maxAngle: 2,
        updateRate: 20,
      },
    }),
    normalizeSettings({}).lidar,
    world,
  );
  assert.equal(fan.config.beams, 101);
  assert.ok(Math.abs(fan.config.fov - 3) < 1e-12);
  assert.ok(Math.abs(fan.angleMin + 1) < 1e-12);
  assert.ok(Math.abs(fan.angleIncrement - 0.03) < 1e-12);
  assert.equal(fan.rateHz, 20);
});

/**
 * Mean IMU reading over a run.
 * @param {import("../src/core/sim/sim.js").Sim} sim
 * @param {{ vx: number, vy: number, wz: number }} twist
 * @param {number} steps
 */
function meanImu(sim, twist, steps) {
  const acc = [0, 0, 0, 0, 0, 0];
  for (let k = 0; k < steps; k++) {
    step(sim, twist);
    const r = /** @type {NonNullable<typeof sim.lastImu>} */ (sim.lastImu);
    [...r.accel, ...r.gyro].forEach((v, i) => (acc[i] += v / steps));
  }
  return acc;
}

test("IMU at rest reads gravity plus the calibration's bias (imu-calibration-lab JSON)", () => {
  const { robot } = robotFromDoc(generate("mecanum", {}));
  const calibration = {
    generator: "imu-calibration-lab 0.1.0",
    accel: {
      kind: "accel",
      units: "g",
      bias: [0.03, -0.02, 0.05],
      matrix: [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
      ],
    },
    gyro: {
      kind: "gyro",
      units: "deg/s",
      bias: [1, -2, 0.5],
      scale: [1, 1, 1],
      noiseStd: [0, 0, 0],
    },
    mag: null,
  };
  const sim = createSim(robot, room(), normalizeSettings({ ...QUIET, imu: { calibration } }));
  assert.match(sim.imu?.source ?? "", /imu-calibration-lab/);
  const m = meanImu(sim, { vx: 0, vy: 0, wz: 0 }, 500);
  // Accelerometer noise 400 ug/sqrt(Hz) at 100 Hz: 0.039 m/s^2 per sample, 0.0018 for 500.
  const expect = [0.03 * G0, -0.02 * G0, G0 + 0.05 * G0];
  m.slice(0, 3).forEach((v, i) => assert.ok(Math.abs(v - expect[i]) < 0.008, `accel ${i}: ${v}`));
  // Gyroscope: no noise in this calibration, so the bias exactly.
  [1, -2, 0.5].forEach((b, i) => assert.ok(Math.abs(m[3 + i] - (b * Math.PI) / 180) < 1e-12));
});

test("IMU turning: rate about z, sign flips upside down, centripetal acceleration off-centre", () => {
  const base = generate("mecanum", {});
  const cases = [
    { xyz: [0, 0, 0.02], rpy: [0, 0, Math.PI / 2], gz: 1, az: G0, ax: 0 },
    { xyz: [0, 0, 0.02], rpy: [Math.PI, 0, 0], gz: -1, az: -G0, ax: 0 },
    { xyz: [0.1, 0, 0.02], rpy: [0, 0, 0], gz: 1, az: G0, ax: -0.1 },
  ];
  for (const c of cases) {
    const doc = applyOp(base, {
      type: "setOrigin",
      target: { kind: "joint", name: "imu_joint" },
      origin: { xyz: /** @type {any} */ (c.xyz), rpy: /** @type {any} */ (c.rpy) },
    });
    const { robot } = robotFromDoc(doc);
    const sim = createSim(
      robot,
      room(),
      normalizeSettings({
        ...QUIET,
        imu: { calibration: { kind: "gyro", bias: [0, 0, 0], noiseStd: [0, 0, 0] } },
      }),
    );
    for (let k = 0; k < 50; k++) step(sim, { vx: 0, vy: 0, wz: 1 }); // reach a steady turn
    const m = meanImu(sim, { vx: 0, vy: 0, wz: 1 }, 300);
    assert.ok(Math.abs(m[5] - c.gz) < 1e-9, `gz ${m[5]}`);
    assert.ok(Math.abs(m[2] - c.az) < 0.01, `az ${m[2]}`);
    assert.ok(Math.abs(m[0] - c.ax) < 0.01, `ax ${m[0]} for ${JSON.stringify(c)}`);
  }
});

test("imu-calibration-lab JSON variants and errors", () => {
  const allanGyro = {
    kind: "allan",
    sensor: "gyro",
    axes: [0, 1, 2].map(() => ({ noiseDensity: 0.005, rateRandomWalkFit: { value: 0.0003 } })),
  };
  const allanAccel = {
    kind: "allan",
    sensor: "accel",
    axes: [0, 1, 2].map(() => ({ noiseDensity: 1e-4 })),
  };
  const accel = {
    kind: "accel",
    units: "g",
    bias: [0, 0, 0],
    matrix: [
      [2, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ],
  };
  const m = imuModelFromCalibration(JSON.stringify([allanGyro, allanAccel, accel]));
  assert.ok(Math.abs(m.gyroNoiseDensity[0] - (0.005 * Math.PI) / 180) < 1e-15);
  assert.ok(Math.abs(m.gyroRandomWalk[2] - (0.0003 * Math.PI) / 180) < 1e-15);
  assert.ok(Math.abs(m.accelNoiseDensity[1] - 1e-4 * G0) < 1e-15);
  assert.equal(m.accelMatrixInv[0], 0.5);
  assert.match(m.source, /Allan/);
  assert.throws(() => imuModelFromCalibration("{"), /not valid JSON/);
  assert.throws(() => imuModelFromCalibration({ hello: 1 }), /no accel, gyro or allan/);
  assert.throws(
    () => imuModelFromCalibration({ kind: "accel", units: "mps2", bias: [0, 0, 0] }),
    /expected g/,
  );
  assert.throws(
    () => imuModelFromCalibration({ kind: "gyro", units: "rad/s", bias: [0, 0, 0] }),
    /expected deg\/s/,
  );
  assert.throws(() => imuModelFromCalibration({ kind: "gyro", bias: [0, 0] }), /three numbers/);
  assert.throws(
    () => imuModelFromCalibration({ kind: "accel", bias: [0, 0, 0], matrix: [[1, 0, 0]] }),
    /3x3/,
  );
  assert.throws(
    () =>
      imuModelFromCalibration({
        kind: "accel",
        bias: [0, 0, 0],
        matrix: [
          [1, 0, 0],
          [1, 0, 0],
          [0, 0, 1],
        ],
      }),
    /singular/,
  );
  assert.throws(
    () => imuModelFromCalibration({ kind: "allan", sensor: "mag", axes: [{}, {}, {}] }),
    /expected gyro or accel/,
  );
  assert.throws(
    () => imuModelFromCalibration({ kind: "allan", sensor: "gyro", axes: [] }),
    /three axes/,
  );
  const inv = invert3([2, 1, 0, 0, 1, 0, 0, 3, 4]);
  const back = [0, 1, 2].map((r) =>
    [0, 1, 2].map((c) =>
      [0, 1, 2].reduce((s, k) => s + [2, 1, 0, 0, 1, 0, 0, 3, 4][3 * r + k] * inv[3 * k + c], 0),
    ),
  );
  back.forEach((row, r) =>
    row.forEach((v, c) => assert.ok(Math.abs(v - (r === c ? 1 : 0)) < 1e-12)),
  );
  const gz = imuModelFromGazebo({ gyroNoise: 0.01, accelBias: 0.1 }, 100);
  assert.equal(gz.gyroNoiseDensity[0], 0.001);
  assert.equal(gz.accelBias[2], 0.1);
  // Same seed, same readings; the random walk drifts the gyro bias.
  const model = {
    ...defaultImuModel(),
    gyroRandomWalk: /** @type {[number, number, number]} */ ([0.01, 0.01, 0.01]),
  };
  const a = new ImuSensor(model, 100, createRng(5));
  const b = new ImuSensor(model, 100, createRng(5));
  for (let k = 0; k < 100; k++)
    assert.deepEqual(a.read([0, 0, G0], [0, 0, 0]), b.read([0, 0, G0], [0, 0, 0]));
  assert.ok(a.drift.some((d) => d !== 0));
});

test("the estimator wrapper reproduces lidar-slam-lab's own loop number for number", () => {
  for (const map of /** @type {const} */ (["office", "loop"])) {
    const seed = 3;
    const ours = referenceRun(map, seed, { scanMatching: true, particles: 300, maxSteps: 120 });
    const world = BUILTIN_MAPS[map]();
    const sim = createSimulation(world, { ...defaultSettings(), seed });
    initialScan(sim);
    let idx = 0;
    for (let k = 0; k < 120; k++) {
      const cmd = followRoute(sim.truePose, world.route, idx);
      idx = cmd.index;
      if (cmd.done) break;
      lslStep(sim, cmd.v, cmd.w);
    }
    const theirs = summarize(sim);
    assert.equal(ours.steps, sim.steps);
    assert.equal(ours.distance, theirs.distance);
    assert.equal(ours.odomRms, theirs.odomRms);
    assert.equal(ours.estRms, theirs.estRms);
    assert.equal(ours.mclRms, theirs.mclRms);
  }
  const noSm = referenceRun("office", 1, { scanMatching: false, particles: 0, maxSteps: 10 });
  assert.ok(Number.isNaN(noSm.estRms) && Number.isNaN(noSm.mclRms));
});

test("a scan seen from the drive frame keeps every end point", () => {
  const scan = {
    angles: new Float64Array([0, 1, 2]),
    ranges: new Float64Array([1, NaN, 2]),
    maxRange: 8,
  };
  assert.equal(virtualScan(scan, { x: 0, y: 0, theta: 0 }), scan);
  const mount = { x: 0.1, y: -0.05, theta: 0.3 };
  const v = virtualScan(scan, mount);
  for (const i of [0, 2]) {
    const a = mount.theta + scan.angles[i];
    const x = mount.x + scan.ranges[i] * Math.cos(a);
    const y = mount.y + scan.ranges[i] * Math.sin(a);
    assert.ok(Math.abs(v.ranges[i] * Math.cos(v.angles[i]) - x) < 1e-12);
    assert.ok(Math.abs(v.ranges[i] * Math.sin(v.angles[i]) - y) < 1e-12);
  }
  assert.ok(Number.isNaN(v.ranges[1]));
  const e = createEstimators({ width: 5, height: 5 }, [], { x: 1, y: 1, theta: 0 }, mount, {
    seed: 1,
    scanMatching: true,
    particles: 10,
    mclGlobal: true,
    resolution: 0.1,
    fieldSigma: 0.1,
    fieldOccupiedThreshold: 0.5,
    match: /** @type {any} */ ({}),
    clearance: 0.1,
  });
  assert.ok(e.mcl && e.pf?.count === 10);
});

test("the full simulation's estimators beat wheel odometry on the lab route", () => {
  const { robot } = robotFromDoc(generate("mecanum", {}));
  const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld("lab"));
  const sim = createSim(robot, world, normalizeSettings({ seed: 2 }));
  // Drive a fixed curve: the error measures fill in and the estimators stay close.
  for (let k = 0; k < 1500; k++) step(sim, { vx: 0.25, vy: 0, wz: k < 750 ? 0.12 : -0.12 });
  const e = sim.est;
  assert.ok(e);
  if (!e) return;
  assert.equal(e.samples, 150);
  assert.ok(e.matchAccepted > 100);
  const s = Math.sqrt(e.sumSqEst / e.samples);
  const o = Math.sqrt(e.sumSqOdom / e.samples);
  assert.ok(s < o, `scan matching ${s} vs odometry ${o}`);
  assert.ok(Math.sqrt(e.sumSqMcl / e.samples) < 0.2);
});

test("depth camera: a point cloud on the floor and the obstacles in view", () => {
  const { robot } = robotFromDoc(generate("mecanum", { depth_camera: true }));
  const cam = robot.sensors.find((s) => s.kind === "depth_camera");
  assert.ok(cam);
  if (!cam) return;
  const world = room(0, { boxes: [{ x: 6.5, y: 5, w: 0.4, d: 1, yaw: 0, h: 0.05 }] });
  const s = normalizeSettings({ depth: { enabled: true } }).depth;
  const r = depthCloud(world, { x: 5, y: 5, theta: 0 }, robot.baseHeight, cam, s);
  assert.ok(r.count > 100, `${r.count}`);
  let onBox = 0;
  for (let i = 0; i < r.count; i++) {
    const [x, y, z] = [r.points[3 * i], r.points[3 * i + 1], r.points[3 * i + 2]];
    assert.ok(z > -1e-6 && Math.hypot(x - 5.165, y - 5) <= s.maxRange + 0.2);
    if (Math.abs(x - 6.3) < 1e-3 && z > 1e-3) onBox++;
  }
  assert.ok(onBox > 0, "the 5 cm box below the LiDAR is visible to the depth camera");
  const sim = createSim(
    robot,
    world,
    normalizeSettings({ ...QUIET, depth: { enabled: true, rateHz: 10 } }),
  );
  for (let k = 0; k < 10; k++) step(sim, { vx: 0, vy: 0, wz: 0 });
  assert.ok(sim.cloud && sim.cloudPoints > 0);
});
