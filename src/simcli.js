// `robot-studio sim`: drive a robot (a URDF / xacro file or a template id)
// through a world headless and write the recording. Deterministic: the same
// arguments and seed write byte-identical files. The step time it prints
// is measured on the machine it runs on and is not written to the files.

import { parseArgs } from "node:util";
import { loadModel } from "./core/model.js";
import { NodeFs, abs } from "./node/nodefs.js";
import { importUrdf } from "./core/builder/urdf.js";
import { modelFromDoc } from "./core/builder/editor.js";
import { TEMPLATES, generate } from "./core/builder/templates.js";
import { simRobot } from "./core/sim/robot.js";
import { WORLDS, bundledWorld, parseSimWorld } from "./core/sim/world.js";
import { normalizeSettings } from "./core/sim/settings.js";
import { createSim, step, simErrors, DT } from "./core/sim/sim.js";
import { ROUTES, createRoute } from "./core/sim/routes.js";
import { recordingFiles } from "./core/sim/record.js";

/**
 * @typedef {object} SimIo
 * @property {(text: string) => void} out
 * @property {(text: string) => void} err
 * @property {(path: string, data: string | Uint8Array) => void} writeFile
 * @property {(path: string) => boolean} exists
 * @property {(path: string) => string} readText
 * @property {(path: string) => void} mkdir
 * @property {() => number} now milliseconds, for the step time only
 */

export const SIM_HELP = `robot-studio sim <robot.urdf|robot.xacro|template> [options]

Drive the robot through a world with simulated wheels, encoders, IMU and
2D LiDAR, run wheel odometry, scan matching and MCL, and report their
errors against the simulated ground truth. Simulation output only.

  --world W            ${WORLDS.map((w) => w.id).join(", ")} (default office) or a world .json
  --route R            ${ROUTES.map((r) => r.id).join(", ")} (default demo)
  --seconds S          simulated time (default 60, at most 3600)
  --seed N             random seed (default 1)
  --out DIR            write trajectory.csv, scans.jsonl, imu.csv, wheel_state.csv,
                       odometry.csv, joint_states.csv, map.pgm/.yaml, world.json, summary.json
  --imu-cal FILE       imu-calibration-lab JSON export as the IMU error model
  --set key=value      a simulation setting, e.g. lidar.beams=360, drive.slip=false,
                       estimators.particles=0 (repeatable; list in docs/simulation.md)
  --ideal              ideal wheels: commanded wheel speeds reached instantly, no slip
  --json               print summary.json instead of the text report
  --arg name=value     xacro argument (as for check)
`;

class SimUsage extends Error {}

/**
 * @param {string} v
 * @returns {unknown}
 */
function settingValue(v) {
  if (v === "true" || v === "false") return v === "true";
  const n = Number(v);
  if (v.trim() !== "" && Number.isFinite(n)) return n;
  throw new SimUsage(`--set value '${v}' must be a number, true or false`);
}

/**
 * @param {string[]} argv arguments after "sim"
 * @param {SimIo} io
 * @returns {number} exit code
 */
export function simCommand(argv, io) {
  let parsed;
  try {
    parsed = parseArgs({
      args: argv,
      allowPositionals: true,
      options: {
        world: { type: "string", default: "office" },
        route: { type: "string", default: "demo" },
        seconds: { type: "string", default: "60" },
        seed: { type: "string", default: "1" },
        out: { type: "string" },
        "imu-cal": { type: "string" },
        set: { type: "string", multiple: true },
        arg: { type: "string", multiple: true },
        ideal: { type: "boolean", default: false },
        json: { type: "boolean", default: false },
        help: { type: "boolean", short: "h" },
      },
    });
  } catch (e) {
    io.err(`robot-studio sim: ${e instanceof Error ? e.message : String(e)}\n${SIM_HELP}`);
    return 3;
  }
  const { values, positionals } = parsed;
  if (values.help) {
    io.out(SIM_HELP);
    return 0;
  }
  try {
    return run(values, positionals, io);
  } catch (e) {
    if (e instanceof SimUsage || e instanceof RangeError) {
      io.err(`robot-studio sim: ${e.message}\n`);
      return 3;
    }
    throw e;
  }
}

/**
 * @param {any} values
 * @param {string[]} positionals
 * @param {SimIo} io
 */
function run(values, positionals, io) {
  if (positionals.length !== 1)
    throw new SimUsage("give one robot: a .urdf / .xacro file or a template id");
  const input = positionals[0];
  const seconds = Number(values.seconds);
  if (!(seconds > 0 && seconds <= 3600)) throw new SimUsage("--seconds must be in (0, 3600]");
  const seed = Number(values.seed);
  if (!Number.isInteger(seed) || seed < 0 || seed > 2 ** 32 - 1)
    throw new SimUsage("--seed must be an integer >= 0");

  // Robot
  /** @type {Record<string, string>} */
  const args = {};
  for (const a of values.arg ?? []) {
    const m = /^([^=]+)=(.*)$/.exec(a);
    if (!m) throw new SimUsage(`--arg expects name=value, got '${a}'`);
    args[m[1]] = m[2];
  }
  /** @type {import("./core/builder/document.js").RobotDoc} */
  let doc;
  let model;
  if (TEMPLATES.some((t) => t.id === input) && !io.exists(input)) {
    doc = generate(input, {});
    model = modelFromDoc(doc);
  } else {
    if (!io.exists(input)) {
      io.err(`robot-studio sim: cannot read file: ${input}\n`);
      return 3;
    }
    const fs = new NodeFs({});
    const entry = abs(input);
    fs.discoverAround(entry);
    const loaded = loadModel(fs, entry, {
      args,
      env: /** @type {Record<string, string>} */ (process.env),
    });
    if (loaded.error || !loaded.urdf) {
      io.err(`robot-studio sim: ${loaded.error?.message ?? "the model could not be read"}\n`);
      return 2;
    }
    const imp = importUrdf(loaded.urdf, { entry });
    if (!imp.doc) {
      io.err(`robot-studio sim: ${imp.notes.join("; ")}\n`);
      return 2;
    }
    doc = imp.doc;
    model = loaded;
  }
  const r = simRobot(model, doc);
  if (!r.ok) {
    io.err(`robot-studio sim: cannot simulate this robot: ${r.reason}\n`);
    return 2;
  }

  // World and settings
  let world = bundledWorld(values.world);
  if (!world) {
    if (!values.world.endsWith(".json") || !io.exists(values.world))
      throw new SimUsage(
        `unknown world '${values.world}' (${WORLDS.map((w) => w.id).join(", ")} or a .json file)`,
      );
    try {
      world = parseSimWorld(io.readText(values.world));
    } catch (e) {
      throw new SimUsage(`${values.world}: ${e instanceof Error ? e.message : String(e)}`);
    }
  }
  /** @type {Record<string, any>} */
  const given = { seed };
  for (const s of values.set ?? []) {
    const m = /^([A-Za-z.]+)=(.*)$/.exec(s);
    if (!m) throw new SimUsage(`--set expects key=value, got '${s}'`);
    const keys = m[1].split(".");
    let cur = given;
    for (const k of keys.slice(0, -1)) cur = cur[k] ??= {};
    cur[keys[keys.length - 1]] = settingValue(m[2]);
  }
  if (values.ideal) {
    given.drive = {
      ...(given.drive ?? {}),
      ideal: true,
      slip: false,
      radiusErrorLeft: 0,
      radiusErrorRight: 0,
    };
  }
  if (values["imu-cal"]) {
    try {
      given.imu = { ...(given.imu ?? {}), calibration: JSON.parse(io.readText(values["imu-cal"])) };
    } catch (e) {
      throw new SimUsage(`${values["imu-cal"]}: ${e instanceof Error ? e.message : String(e)}`);
    }
  }
  const settings = normalizeSettings(given);

  // Run
  const sim = createSim(r.robot, world, settings, { log: Boolean(values.out) || values.json });
  const route = createRoute(values.route, r.robot, world);
  const steps = Math.round(seconds / DT);
  const t0 = io.now();
  for (let k = 0; k < steps; k++) step(sim, route.next(sim.drive, sim.t).twist);
  const ms = io.now() - t0;
  const meta = { route: values.route, input, worldId: world.id };
  /** @type {Record<string, string | Uint8Array> | null} */
  const files = values.out || values.json ? recordingFiles(sim, meta) : null;
  if (values.out && files) {
    io.mkdir(values.out);
    for (const [name, data] of Object.entries(files)) io.writeFile(`${values.out}/${name}`, data);
  }
  if (values.json && files) {
    io.out(String(files["summary.json"]));
    return 0;
  }
  const e = simErrors(sim);
  const g = r.robot.geometry;
  const fmt = (/** @type {number} */ v, d = 3) => (Number.isFinite(v) ? v.toFixed(d) : "-");
  const lines = [
    "SIMULATION: model output of Robot Studio's drive, sensor and estimator models; not a measurement.",
    `robot     ${r.robot.name}: ${r.robot.drive}, ${r.robot.wheels.length} wheels, r ${fmt(g.wheelRadius)} m, ${g.drive === "mecanum" ? `lx ${fmt(g.halfLength)} m, ` : ""}ly ${fmt(g.halfWidth)} m, max ${fmt(sim.controller.config.maxWheelSpeed, 1)} rad/s`,
    `world     ${world.name}; route ${values.route} (${route.progress()}); ${fmt(sim.t, 1)} s simulated, seed ${seed}`,
    `sensors   ${sim.lidar ? `LiDAR ${sim.lidar.sensor.link}: ${sim.lidar.config.beams} beams, ${fmt(sim.lidar.rateHz, 1)} Hz, ${fmt(sim.lidar.minRange, 2)}-${fmt(sim.lidar.config.maxRange, 1)} m (${sim.lidar.source})` : "no LiDAR"}`,
    `          ${sim.imu ? `IMU ${sim.imu.sensor.link}: ${fmt(100 / sim.imu.every, 0)} Hz, ${sim.imu.source}` : "no IMU"}`,
    `          encoders: ${sim.ticksPerRev} counts per wheel revolution (4x), firmware control loop 100 Hz`,
    `motion    ${fmt(sim.distance, 2)} m driven, ${sim.collisions} blocked step(s), ${sim.saturated} saturated step(s)`,
  ];
  if (e) {
    lines.push(
      "position error   final     RMS       max",
      `wheel odometry   ${fmt(e.odom).padStart(7)} m ${fmt(e.odomRms).padStart(7)} m ${fmt(sim.est?.maxOdom ?? NaN).padStart(7)} m`,
    );
    if (settings.estimators.scanMatching)
      lines.push(
        `+ scan matching  ${fmt(e.est).padStart(7)} m ${fmt(e.estRms).padStart(7)} m ${fmt(sim.est?.maxEst ?? NaN).padStart(7)} m   (${fmt(100 * e.acceptRate, 1)} % of matches accepted)`,
      );
    if (sim.est?.pf)
      lines.push(
        `MCL (${settings.estimators.particles} particles) ${fmt(e.mcl).padStart(5)} m ${fmt(e.mclRms).padStart(7)} m ${fmt(sim.est.maxMcl).padStart(7)} m`,
      );
  }
  for (const n of [...r.notes, ...sim.notes]) lines.push(`note      ${n}`);
  lines.push(
    `step time ${fmt(ms / steps, 3)} ms per ${fmt(DT * 1000, 0)} ms step (${fmt((DT * 1000 * steps) / ms, 1)}x real time; measured on this machine, not written to the files)`,
  );
  if (values.out && files)
    lines.push(`wrote     ${values.out}/ (${Object.keys(files).length} files)`);
  io.out(lines.join("\n") + "\n");
  return 0;
}
