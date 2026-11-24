// The simulated drive against analytic results: ideal wheels reproduce
// inverse/forward kinematics exactly (straight line, turn on the spot,
// mecanum strafe, differential arcs), saturation scales like the firmware
// and mecanum-kinematics, encoder counts for a known distance, and the
// firmware mirror's building blocks.

import { test } from "node:test";
import assert from "node:assert/strict";
import { generate } from "../src/core/builder/templates.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import { mecanumGeometry, mecanumInverse, wheelBase } from "../src/core/builder/drive.js";
import { simRobot } from "../src/core/sim/robot.js";
import { bundledWorld } from "../src/core/sim/world.js";
import { normalizeSettings } from "../src/core/sim/settings.js";
import { createSim, step, jointMap, odometryBase, estimates } from "../src/core/sim/sim.js";
import {
  CommandWatchdog,
  DriveController,
  FW_DEFAULTS,
  VelocityEstimator,
  VelocityPid,
  countDelta,
  dutyToWire,
  elapsedUs,
  fwForward,
  fwInverse,
  pyRound,
  saturate,
  slewLimit,
  toInt32,
} from "../src/core/sim/firmware.js";
import { Plant, DEFAULT_MOTOR, steadyState } from "../src/core/sim/plant.js";
import { compose, inverse, se2Exp, wrapAngle } from "../src/core/sim/se2.js";
import { createRoute, routeSpeeds } from "../src/core/sim/routes.js";

/** @param {string} id @param {Record<string, unknown>} [params] */
function robotOf(id, params = {}) {
  const doc = generate(id, params);
  const r = simRobot(modelFromDoc(doc), doc);
  if (!r.ok) throw new Error(r.reason);
  return r.robot;
}

const IDEAL = { drive: { ideal: true, slip: false, radiusErrorLeft: 0, radiusErrorRight: 0 } };
const empty = () =>
  /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld("empty"));

/**
 * @param {import("../src/core/sim/sim.js").Sim} sim
 * @param {{ vx: number, vy: number, wz: number }} twist
 * @param {number} seconds
 */
function drive(sim, twist, seconds) {
  for (let k = 0; k < Math.round(seconds * 100); k++) step(sim, twist);
}

test("ideal wheels: straight line, turn on the spot and strafe follow IK/FK exactly", () => {
  const robot = robotOf("mecanum");
  const g = /** @type {NonNullable<ReturnType<typeof mecanumGeometry>>} */ (
    mecanumGeometry(wheelBase(modelFromDoc(generate("mecanum", {}))))
  );
  for (const [twist, T] of /** @type {[{ vx: number, vy: number, wz: number }, number][]} */ ([
    [{ vx: 0.2, vy: 0, wz: 0 }, 5],
    [{ vx: 0, vy: 0, wz: 0.5 }, 2],
    [{ vx: 0, vy: -0.25, wz: 0 }, 4],
    [{ vx: 0.1, vy: 0.1, wz: 0.3 }, 3],
  ])) {
    const sim = createSim(robot, empty(), normalizeSettings(IDEAL));
    const start = { ...sim.truth };
    drive(sim, twist, T);
    // The commanded wheel speeds are mecanumInverse's (wheel positions measured from the URDF).
    const ik = mecanumInverse(g, twist);
    sim.plant.speed.forEach((w, i) => assert.ok(Math.abs(w - ik[i]) < 1e-12, `wheel ${i}`));
    // Constant twist for T seconds: the pose is the exponential map of twist * T.
    const expect = compose(start, se2Exp(twist.vx * T, twist.vy * T, twist.wz * T));
    assert.ok(Math.abs(sim.truth.x - expect.x) < 1e-9, `x ${sim.truth.x} ${expect.x}`);
    assert.ok(Math.abs(sim.truth.y - expect.y) < 1e-9, `y ${sim.truth.y} ${expect.y}`);
    assert.ok(Math.abs(wrapAngle(sim.truth.theta - expect.theta)) < 1e-9);
    assert.ok(
      Math.abs(sim.actual.vx - twist.vx) < 1e-9 && Math.abs(sim.actual.wz - twist.wz) < 1e-9,
    );
    assert.equal(sim.collisions, 0);
  }
});

test("differential base: arc about the axle, lateral command ignored, base_link offset from the axle", () => {
  const robot = robotOf("diff");
  assert.equal(robot.drive, "differential");
  assert.deepEqual(robot.slots, [0, 1, 0, 1]);
  // The diff template's wheels are 5 cm ahead of base_link.
  assert.ok(Math.abs(robot.drivePose.x - 0.05) < 1e-12);
  const sim = createSim(robot, empty(), normalizeSettings(IDEAL));
  const startDrive = { ...sim.drive };
  drive(sim, { vx: 0, vy: 0.3, wz: 0.8 }, 2);
  // vy is ignored; the axle centre stays put while the base turns.
  assert.ok(Math.hypot(sim.drive.x - startDrive.x, sim.drive.y - startDrive.y) < 1e-12);
  assert.ok(Math.abs(sim.drive.theta - 1.6) < 1e-9);
  // base_link moves on a circle of radius 0.05 m around the axle centre.
  assert.ok(
    Math.abs(Math.hypot(sim.truth.x - sim.drive.x, sim.truth.y - sim.drive.y) - 0.05) < 1e-12,
  );
  const g = robot.geometry;
  const left = (0 - 0.8 * g.halfWidth) / g.wheelRadius;
  assert.ok(Math.abs(sim.plant.speed[0] - left) < 1e-12);
  assert.ok(Math.abs(sim.plant.speed[1] + left) < 1e-12);
  // Joint positions: the wheel joints turn by the wheel angle times their axis sign.
  const q = jointMap(sim);
  assert.ok(Math.abs(q.left_wheel_joint - robot.wheels[0].sign * left * 2) < 1e-9);
});

test("saturation scales every wheel by the same factor, as the firmware and mecanum-kinematics do", () => {
  const robot = robotOf("mecanum");
  assert.equal(robot.maxWheelSpeed, 20);
  assert.equal(robot.maxWheelSpeedFrom, "urdf");
  const sim = createSim(robot, empty(), normalizeSettings(IDEAL));
  const twist = { vx: 0.5, vy: 0.5, wz: 2 };
  step(sim, twist);
  const raw = fwInverse(robot.geometry, twist);
  const peak = Math.max(...raw.map(Math.abs));
  const scale = 20 / peak;
  assert.ok(Math.abs(sim.controller.scale - scale) < 1e-15);
  assert.equal(Math.max(...sim.plant.speed.map(Math.abs)), 20);
  sim.plant.speed.forEach((w, i) => assert.ok(Math.abs(w - raw[i] * scale) < 1e-12));
  assert.ok(Math.abs(sim.controller.active.wz - 2 * scale) < 1e-15);
  assert.equal(sim.saturated, 1);
  assert.deepEqual(saturate([1, -2, 0.5, 0], 4), { wheels: [1, -2, 0.5, 0], scale: 1 });
  assert.deepEqual(saturate([1, -8, 0.5, 0], 4), { wheels: [0.5, -4, 0.25, 0], scale: 0.5 });
  assert.equal(saturate([1, 2, 3, 4], 0).scale, 1);
});

test("encoder counts and firmware odometry for a known distance", () => {
  const robot = robotOf("mecanum");
  const sim = createSim(robot, empty(), normalizeSettings(IDEAL));
  assert.equal(sim.ticksPerRev, 38000); // 500 lines x 4 x 19:1
  drive(sim, { vx: 0.25, vy: 0, wz: 0 }, 4); // 1 m
  const angle = 1 / 0.04; // rad per wheel
  const expected = Math.floor(angle / ((2 * Math.PI) / 38000));
  assert.equal(expected, 151197);
  for (const c of sim.plant.count) assert.ok(Math.abs(c - expected) <= 1, `${c}`);
  // One more period lets the firmware sample the last encoder counts.
  step(sim, { vx: 0, vy: 0, wz: 0 });
  const tick = (0.04 * 2 * Math.PI) / 38000;
  assert.ok(Math.abs(sim.controller.pose.x - 1) < 2 * tick, `${sim.controller.pose.x}`);
  assert.ok(Math.abs(sim.controller.pose.y) < 1e-12);
  const odom = odometryBase(sim);
  assert.ok(Math.abs(odom.x - (sim.startBase.x + 1)) < 2 * tick);
  const ws = sim.controller.wheelState(sim.plant.nowUs);
  assert.equal(ws.count[0], sim.plant.count[0]);
  assert.equal(ws.t_us, 401 * 10000);
  const e = estimates(sim);
  assert.ok(e.est && e.mcl);
});

test("motor + PI model reaches the commanded speed; slip and radius error make odometry drift", () => {
  const robot = robotOf("mecanum");
  const clean = createSim(
    robot,
    empty(),
    normalizeSettings({ drive: { slip: false, radiusErrorLeft: 0, radiusErrorRight: 0 } }),
  );
  drive(clean, { vx: 0.2, vy: 0, wz: 0 }, 3);
  for (const w of clean.plant.speed) assert.ok(Math.abs(w - 5) < 0.01, `${w}`);
  assert.ok(Math.abs(clean.actual.vx - 0.2) < 0.001);
  const cleanOdom = odometryBase(clean);
  assert.ok(Math.hypot(cleanOdom.x - clean.truth.x, cleanOdom.y - clean.truth.y) < 0.003);

  const noisy = createSim(robot, empty(), normalizeSettings({ seed: 4 }));
  drive(noisy, { vx: 0.2, vy: 0, wz: 0 }, 3);
  const noisyOdom = odometryBase(noisy);
  // With the default left/right radius error the robot curves; odometry does not see it.
  assert.ok(Math.abs(noisy.truth.theta) > 0.005, `${noisy.truth.theta}`);
  assert.ok(Math.abs(noisyOdom.theta) < 0.01);
  assert.ok(noisy.slip.some((s) => s !== 0));
});

test("collisions: the robot stops at a wall and its wheels keep turning", () => {
  const robot = robotOf("mecanum");
  const sim = createSim(robot, empty(), normalizeSettings(IDEAL));
  drive(sim, { vx: 0.4, vy: 0, wz: 0 }, 20);
  assert.ok(sim.collisions > 0);
  assert.ok(sim.truth.x < 10 - robot.footprint.radius + 0.01);
  assert.ok(sim.truth.x > 9.5);
  assert.ok(sim.controller.pose.x > 6, "odometry keeps counting the wheel turns");
});

test("routes: the square returns to the start, the spin to the start heading, none stands still", () => {
  const robot = robotOf("mecanum");
  const world = empty();
  for (const id of ["square", "spin", "none"]) {
    const sim = createSim(robot, world, normalizeSettings(IDEAL));
    const route = createRoute(id, robot, world);
    let done = false;
    let n = 0;
    while (!done && n < 4000) {
      const c = route.next(sim.drive, sim.t);
      done = c.done;
      step(sim, c.twist);
      n++;
    }
    assert.ok(
      Math.hypot(sim.truth.x - 5, sim.truth.y - 5) < 1e-6,
      `${id} ${JSON.stringify(sim.truth)}`,
    );
    // Segment ends fall on the 10 ms step grid: a turn ends within one step of its angle.
    assert.ok(Math.abs(wrapAngle(sim.truth.theta)) < 0.01, `${id} ${sim.truth.theta}`);
    assert.match(route.progress(), id === "none" ? /standing/ : /finished/);
    if (id === "square") assert.ok(sim.distance > 5.9);
  }
  const diff = robotOf("diff");
  const sq = createRoute("square", diff, world);
  assert.equal(sq.next({ x: 0, y: 0, theta: 0 }, 0).twist.vy, 0);
  assert.throws(() => createRoute("zigzag", robot, world), /unknown route/);
  const sp = routeSpeeds(robot);
  assert.ok(sp.v > 0 && sp.v <= 0.5 && sp.w > 0 && sp.w <= 1.5);
});

test("demo route: the autopilot follows the world's waypoints", () => {
  const robot = robotOf("mecanum");
  const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld("lab"));
  const sim = createSim(
    robot,
    world,
    normalizeSettings({ estimators: { particles: 0, scanMatching: false } }),
  );
  const route = createRoute("demo", robot, world);
  for (let k = 0; k < 1500; k++) step(sim, route.next(sim.drive, sim.t).twist);
  assert.ok(sim.distance > 3);
  assert.equal(sim.collisions, 0);
  assert.match(route.progress(), /waypoint \d+ of 11/);
  const emptyRoute = createRoute("demo", robot, empty());
  assert.deepEqual(emptyRoute.next(sim.drive, 0), { twist: { vx: 0, vy: 0, wz: 0 }, done: true });
});

test("firmware mirror: helpers, estimator, PID, watchdog and controller", () => {
  assert.equal(countDelta(-2147483648, 2147483647), 1);
  assert.equal(countDelta(5, 7), -2);
  assert.equal(elapsedUs(5, 0xfffffffe), 7);
  assert.equal(toInt32(2 ** 31), -(2 ** 31));
  assert.equal(toInt32(-1), -1);
  assert.deepEqual([0.5, 1.5, 2.5, -0.5, 2.4, 2.6].map(pyRound), [0, 2, 2, 0, 2, 3]);
  assert.deepEqual(
    [0.5, -0.5, 1.00005, -1.00005, 2, NaN].map(dutyToWire),
    [5000, -5000, 10000, -10000, 10000, 0],
  );
  assert.equal(dutyToWire(0.12345), 1235);
  assert.equal(slewLimit(0, 1, 4, 0.01), 0.04);
  assert.equal(slewLimit(0, 1, 0, 0.01), 1);

  const ve = new VelocityEstimator({ lowCps: 200, highCps: 800, zeroTimeoutUs: 250000 });
  assert.equal(ve.update({ count: 0, lastEdgeUs: 0 }, 0), 0);
  // 1000 counts/s, edges exactly timed: fixed and period estimates agree.
  assert.equal(ve.update({ count: 10, lastEdgeUs: 10000 }, 10000), 1000);
  assert.equal(ve.update({ count: 20, lastEdgeUs: 20000 }, 20000), 1000);
  assert.equal(VelocityEstimator.weightFor(500, ve.config), 0.5);
  assert.equal(
    VelocityEstimator.weightFor(100, { lowCps: 200, highCps: 100, zeroTimeoutUs: 1 }),
    0,
  );
  // Slow: one count per 40 ms; the period estimate reads 25 counts/s.
  const slow = new VelocityEstimator({ lowCps: 200, highCps: 800, zeroTimeoutUs: 250000 });
  slow.update({ count: 0, lastEdgeUs: 0 }, 0);
  slow.update({ count: 1, lastEdgeUs: 5000 }, 10000);
  slow.update({ count: 2, lastEdgeUs: 45000 }, 50000);
  assert.equal(slow.periodCps, 25);
  slow.update({ count: 2, lastEdgeUs: 45000 }, 145000); // no edge for 100 ms: bounded by 1/100 ms
  assert.equal(slow.periodCps, 10);
  slow.update({ count: 2, lastEdgeUs: 45000 }, 400000); // past the zero timeout
  assert.equal(slow.periodCps, 0);
  slow.update({ count: 1, lastEdgeUs: 410000 }, 420000);
  slow.update({ count: 1, lastEdgeUs: 415000 }, 430000); // dithering back to the same count
  assert.equal(slow.periodCps, 0);
  slow.reset();
  assert.equal(slow.primed, false);

  const pid = new VelocityPid({ ...FW_DEFAULTS.pid, kd: 0.01, ks: 0.05 });
  const u1 = pid.update(10, 0, 0.01);
  assert.ok(u1 > 0 && u1 <= 0.04 + 1e-12, `${u1}`); // slew limited to 4 duty/s
  assert.equal(pid.limited, true);
  for (let k = 0; k < 500; k++) pid.update(100, 0, 0.01);
  assert.equal(pid.output, 1);
  assert.ok(pid.integral <= 1);
  assert.equal(pid.update(NaN, 0, 0.01), 1);
  assert.equal(pid.feedforward(-1), -0.03 - 0.05);
  assert.equal(pid.feedforward(0), 0);
  const p2 = new VelocityPid({ ...FW_DEFAULTS.pid, kd: 0.01, dCutoffHz: 0 });
  p2.update(1, 0, 0.01);
  p2.update(1, 0.5, 0.01);
  assert.equal(p2.dState, -50);
  pid.reset();
  assert.equal(pid.output, 0);

  const wd = new CommandWatchdog(500000);
  assert.equal(wd.update(0), false);
  wd.feed(0);
  assert.equal(wd.update(400000), true);
  assert.equal(wd.update(600000), false);
  assert.equal(wd.trips, 1);

  const g = {
    wheelRadius: 0.05,
    halfLength: 0.15,
    halfWidth: 0.17,
    rollerAngleDeg: 45,
    drive: /** @type {const} */ ("mecanum"),
  };
  const t = { vx: 0.3, vy: -0.2, wz: 0.7 };
  const back = fwForward(g, fwInverse(g, t));
  assert.ok(
    Math.abs(back.vx - t.vx) < 1e-12 &&
      Math.abs(back.vy - t.vy) < 1e-12 &&
      Math.abs(back.wz - t.wz) < 1e-12,
  );
  const dg = { ...g, drive: /** @type {const} */ ("differential") };
  const dw = fwInverse(dg, t);
  assert.deepEqual([dw[0], dw[2]], [dw[0], dw[0]]);
  const db = fwForward(dg, dw);
  assert.ok(Math.abs(db.vx - 0.3) < 1e-12 && db.vy === 0 && Math.abs(db.wz - 0.7) < 1e-12);

  const c = new DriveController({
    geometry: g,
    ticksPerWheelRev: 38000,
    maxWheelSpeed: 25,
    pid: { ...FW_DEFAULTS.pid },
    velocity: { ...FW_DEFAULTS.velocity },
    watchdogTimeoutMs: 500,
    controlRateHz: 100,
    telemetryDivider: 5,
  });
  assert.equal(c.command({ vx: NaN, vy: 0, wz: 0 }, 0), false);
  assert.equal(c.command({ vx: 0.2, vy: 0, wz: 0 }, 0), true);
  const snaps = [0, 1, 2, 3].map(() => ({ count: 0, lastEdgeUs: 0 }));
  assert.deepEqual(c.step(snaps, 0), [0, 0, 0, 0]);
  assert.ok(c.step(snaps, 10000).every((d) => d > 0));
  // No command for longer than the watchdog timeout: everything stops.
  assert.deepEqual(c.step(snaps, 700000), [0, 0, 0, 0]);
  assert.equal(c.watchdog.trips, 1);
  assert.deepEqual(c.targets, [0, 0, 0, 0]);
  const od = c.odometry(700000);
  assert.equal(od.t_us, 700000);
});

test("plant: steady state, dead band and exact edge times", () => {
  assert.equal(steadyState(DEFAULT_MOTOR, 0.05), 0);
  assert.ok(Math.abs(steadyState(DEFAULT_MOTOR, -0.4) + 0.34 / 0.034) < 1e-12);
  const p = new Plant(1, 1000, DEFAULT_MOTOR, 10);
  /** @type {number[]} */
  const seen = [];
  // Ideal wheel at 2 pi rad/s: one revolution per second, 1000 counts per second.
  p.advance([2.05 * Math.PI], 0.01, (m, h) => seen.push(m[0] * h), true);
  assert.equal(p.count[0], 10);
  assert.equal(p.lastEdgeUs[0], 9756); // 10 / 10.25 of the period, rounded to the microsecond
  assert.equal(seen.length, 10);
  p.advance([-2.05 * Math.PI], 0.0105, () => {}, true);
  assert.equal(p.count[0], -1);
  assert.equal(p.nowUs, 20500);
  assert.equal(p.snapshots()[0].count, -1);
});

test("SE(2) helpers", () => {
  const a = { x: 1, y: 2, theta: 0.7 };
  const b = compose(a, inverse(a));
  assert.ok(Math.abs(b.x) < 1e-15 && Math.abs(b.y) < 1e-15 && b.theta === 0);
  assert.equal(wrapAngle(Math.PI), Math.PI);
  assert.ok(Math.abs(wrapAngle(-Math.PI) - Math.PI) < 1e-15);
  assert.ok(Math.abs(wrapAngle(7) - (7 - 2 * Math.PI)) < 1e-15);
  const small = se2Exp(1, 0, 1e-8);
  assert.ok(Math.abs(small.y - 5e-9) < 1e-15);
});
