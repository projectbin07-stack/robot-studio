#!/usr/bin/env node
// Write the simulator's numbers that CI compares with other projects'
// own code, to tests/corpus/sim/:
//
//   mecanum-wheel-speeds.json  wheel speeds the simulated drive settles at for
//       a set of twists (ideal wheels, and the motor + PI model after 3 s),
//       checked against mecanum-kinematics' inverse_saturated() by
//       scripts/mecanum_interop.py
//   firmware-interop-<template>.json  WHEEL_STATE and ODOMETRY telemetry of the
//       simulated drive for a command schedule, checked against
//       encoder-odometry-firmware's Python firmware model by
//       scripts/firmware_interop.py
//
//   golden-digest.json  SHA-256 (first 16 hex digits) of each recording file
//       of a fixed 2 s run; tests/sim-io.test.js requires the same on every
//       CI platform (determinism across operating systems and Node versions)
//
//   node scripts/sim-fixtures.js           write the files
//   node scripts/sim-fixtures.js --check   fail if any file would change

import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { generate } from "../src/core/builder/templates.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import { simRobot } from "../src/core/sim/robot.js";
import { bundledWorld } from "../src/core/sim/world.js";
import { normalizeSettings } from "../src/core/sim/settings.js";
import { createSim, step } from "../src/core/sim/sim.js";
import { createRoute } from "../src/core/sim/routes.js";
import { recordingFiles } from "../src/core/sim/record.js";
import { createHash } from "node:crypto";

const ROOT = fileURLToPath(new URL("..", import.meta.url));
const DIR = `${ROOT}tests/corpus/sim`;
const check = process.argv.includes("--check");

/** @param {string} id */
function robotOf(id) {
  const doc = generate(id, {});
  const r = simRobot(modelFromDoc(doc), doc);
  if (!r.ok) throw new Error(r.reason);
  return r.robot;
}

/** Twists for the wheel-speed check; the last three saturate the mecanum template's wheels. */
export const SPEED_TWISTS = [
  { vx: 0.3, vy: 0, wz: 0 },
  { vx: 0, vy: 0.3, wz: 0 },
  { vx: 0, vy: 0, wz: 1 },
  { vx: -0.2, vy: 0.1, wz: -0.5 },
  { vx: 0.15, vy: -0.25, wz: 0.8 },
  { vx: 1.0, vy: 0, wz: 0 },
  { vx: 0.5, vy: 0.5, wz: 2 },
  { vx: -0.4, vy: 0.6, wz: -1.5 },
];

/** @param {number} v */
const r12 = (v) => Number(v.toPrecision(12));

function wheelSpeeds() {
  const robot = robotOf("mecanum");
  const world = bundledWorld("empty");
  if (!world) throw new Error("no empty world");
  const cases = SPEED_TWISTS.map((twist) => {
    const ideal = createSim(
      robot,
      world,
      normalizeSettings({
        drive: { ideal: true, slip: false, radiusErrorLeft: 0, radiusErrorRight: 0 },
      }),
    );
    step(ideal, twist);
    const pi = createSim(
      robot,
      world,
      normalizeSettings({ drive: { slip: false, radiusErrorLeft: 0, radiusErrorRight: 0 } }),
    );
    for (let k = 0; k < 300; k++) step(pi, twist);
    return {
      twist,
      scale: r12(ideal.controller.scale),
      ideal_speeds: ideal.plant.speed.map(r12),
      pi_speeds_after_3s: pi.plant.speed.map(r12),
    };
  });
  return {
    note: "Wheel speeds (rad/s, FL FR RL RR, positive drives forward) of Robot Studio's simulated drive for the mecanum template (tests/corpus/builder/mecanum.toml): ideal wheels after one step, and the motor + PI model (encoder-odometry-firmware's controller and plant model, no slip) after 3 s. Simulation output.",
    toml: "mecanum.toml",
    max_wheel_speed: robot.maxWheelSpeed,
    cases,
  };
}

/** Command schedule: [until_s, vx, vy, wz]. */
const SCHEDULE = [
  [1.0, 0.3, 0, 0],
  [2.0, 0, 0.25, 0],
  [3.0, 0, 0, 1.2],
  [4.0, 0.2, -0.1, 0.4],
  [4.6, 0.9, 0, 0],
  [5.5, 0, 0, 0],
  [6.5, -0.15, 0, -0.6],
];

/** @param {string} id */
function firmwareRun(id) {
  const robot = robotOf(id);
  const world = bundledWorld("empty");
  if (!world) throw new Error("no empty world");
  const sim = createSim(
    robot,
    world,
    normalizeSettings({ drive: { slip: false, radiusErrorLeft: 0, radiusErrorRight: 0 } }),
    { log: true },
  );
  const end = SCHEDULE[SCHEDULE.length - 1][0];
  const periods = Math.round(end * 100);
  for (let k = 0; k < periods; k++) {
    const t = k / 100;
    const s = SCHEDULE.find((x) => t < x[0] - 1e-9) ?? SCHEDULE[SCHEDULE.length - 1];
    step(sim, { vx: s[1], vy: s[2], wz: s[3] });
  }
  const log = /** @type {NonNullable<typeof sim.log>} */ (sim.log);
  const g = robot.geometry;
  return {
    note: `WHEEL_STATE and ODOMETRY telemetry (protocol field values, double precision) of Robot Studio's simulated drive for the '${id}' template with the schedule below; scripts/firmware_interop.py replays the schedule through encoder-odometry-firmware's Python firmware model and compares. Simulation output.`,
    config: {
      wheel_radius: g.wheelRadius,
      half_length: g.drive === "mecanum" ? g.halfLength : 0.1,
      half_width: g.halfWidth,
      roller_angle_deg: g.rollerAngleDeg,
      drive: g.drive,
      ticks_per_wheel_rev: sim.ticksPerRev,
      max_wheel_speed: sim.controller.config.maxWheelSpeed,
      control_rate_hz: 100,
      telemetry_divider: 5,
    },
    schedule: SCHEDULE,
    periods,
    wheel_state: log.wheelState.map((m) => ({
      t_us: m.t_us,
      count: m.count,
      velocity: m.velocity.map(r12),
      target: m.target.map(r12),
      duty: m.duty,
    })),
    odometry: log.odometry.map((m) => ({
      t_us: m.t_us,
      pose: [m.x, m.y, m.theta].map(r12),
      twist: [m.vx, m.vy, m.wz].map(r12),
    })),
  };
}

/** The run tests/sim-io.test.js repeats: mecanum template, lab world, demo route, seed 1, 200 steps. */
function goldenDigest() {
  const robot = robotOf("mecanum");
  const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld("lab"));
  const sim = createSim(robot, world, normalizeSettings({ seed: 1 }), { log: true });
  const route = createRoute("demo", robot, world);
  for (let k = 0; k < 200; k++) step(sim, route.next(sim.drive, sim.t).twist);
  const files = recordingFiles(sim, { route: "demo", input: "mecanum", worldId: "lab" });
  return {
    note: "Digest of a fixed simulation run (scripts/sim-fixtures.js); the tests require it on every platform.",
    files: Object.fromEntries(
      Object.entries(files).map(([k, v]) => [
        k,
        createHash("sha256").update(v).digest("hex").slice(0, 16),
      ]),
    ),
  };
}

const files = {
  "golden-digest.json": goldenDigest(),
  "mecanum-wheel-speeds.json": wheelSpeeds(),
  "firmware-interop-mecanum.json": firmwareRun("mecanum"),
  "firmware-interop-diff.json": firmwareRun("diff"),
};

mkdirSync(DIR, { recursive: true });
/** @type {string[]} */
const changed = [];
for (const [name, value] of Object.entries(files)) {
  const text = JSON.stringify(value, null, 1) + "\n";
  const path = `${DIR}/${name}`;
  if (existsSync(path) && readFileSync(path, "utf8") === text) continue;
  changed.push(name);
  if (!check) writeFileSync(path, text);
}
if (check && changed.length > 0) {
  console.error(
    `sim fixtures are out of date: ${changed.join(", ")}\nrun: node scripts/sim-fixtures.js`,
  );
  process.exit(1);
}
console.log(check ? "sim fixtures up to date" : `wrote ${changed.length} file(s)`);
