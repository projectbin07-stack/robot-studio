// `robot-studio bag`, `replay` and `compare`: a recording written as a
// ROS 2 bag (MCAP), a recording (simulated, or imported from a bag by the
// Python companion) replayed through the estimators, and a recording's
// command schedule run on the simulator and compared with the recording's
// firmware telemetry.

import { parseArgs } from "node:util";
import { VERSION } from "./core/version.js";
import { RecordingError, parseRecording } from "./core/ros/recording.js";
import { recordingToBag } from "./core/ros/bag.js";
import { ReplaySession } from "./core/ros/replay.js";
import { compareRecordings, simulateSchedule, scheduleOf } from "./core/ros/compare.js";
import { loadRobotInput } from "./simcli.js";

/**
 * @typedef {object} RosIo
 * @property {(text: string) => void} out
 * @property {(text: string) => void} err
 * @property {(path: string, data: string | Uint8Array) => void} writeFile
 * @property {(path: string) => boolean} exists
 * @property {(path: string) => string} readText
 * @property {(path: string) => void} mkdir
 */

/** The text files of a recording directory. */
export const RECORDING_FILES = [
  "summary.json",
  "world.json",
  "trajectory.csv",
  "scans.jsonl",
  "imu.csv",
  "wheel_state.csv",
  "odometry.csv",
  "joint_states.csv",
];

export const BAG_HELP = `robot-studio bag <recording-dir> -o <bag-dir>

Write a recording (robot-studio sim --out, the page's "Download recording",
or an imported bag) as a ROS 2 bag: <bag-dir>/metadata.yaml and
<bag-dir>/<name>_0.mcap (MCAP, CDR, Humble message definitions). The topic
mapping is in docs/ros2.md. A simulated recording stays labelled simulated
(/robot_studio/source and the MCAP metadata say so).
`;

export const REPLAY_HELP = `robot-studio replay <recording-dir> [--json] [--no-scan-matching] [--particles N]

Replay a recording through the studio's estimators (wheel odometry, scan
matching on it, MCL when the recording has a world) and print what they did.
Errors against the truth only for simulated recordings (they have one).
`;

export const COMPARE_HELP = `robot-studio compare <recording-dir> <robot.urdf|robot.xacro|template> [options]

Run the recording's command schedule (its cmd_vel / commanded twist, held
between samples) on the simulator with the same robot, then compare the two
runs through the firmware telemetry fields: ODOMETRY (x, y, theta, vx, vy, wz)
and WHEEL_STATE (wheel speeds; encoder counts when both have them).

  --world W        world for the simulated run (default: the recording's, else empty)
  --set key=value  a simulation setting (as for sim; repeatable)
  --ideal          ideal wheels in the simulated run
  --seed N         seed of the simulated run (default 1)
  --out DIR        also write the simulated run's recording there
  --json           print the comparison as JSON
`;

/**
 * @param {string} dir
 * @param {RosIo} io
 */
export function readRecordingDir(dir, io) {
  /** @type {Record<string, string>} */
  const files = {};
  for (const name of RECORDING_FILES) {
    const p = `${dir}/${name}`;
    if (io.exists(p)) files[name] = io.readText(p);
  }
  return parseRecording(files);
}

/**
 * @param {"bag" | "replay" | "compare"} cmd
 * @param {string[]} argv arguments after the command
 * @param {RosIo} io
 * @returns {number}
 */
export function rosCommand(cmd, argv, io) {
  const help = cmd === "bag" ? BAG_HELP : cmd === "replay" ? REPLAY_HELP : COMPARE_HELP;
  let parsed;
  try {
    parsed = parseArgs({
      args: argv,
      allowPositionals: true,
      options: {
        out: { type: "string", short: "o" },
        name: { type: "string" },
        json: { type: "boolean", default: false },
        "no-scan-matching": { type: "boolean", default: false },
        particles: { type: "string" },
        world: { type: "string" },
        set: { type: "string", multiple: true },
        ideal: { type: "boolean", default: false },
        seed: { type: "string", default: "1" },
        help: { type: "boolean", short: "h" },
      },
    });
  } catch (e) {
    io.err(`robot-studio ${cmd}: ${e instanceof Error ? e.message : String(e)}\n${help}`);
    return 3;
  }
  const { values, positionals } = parsed;
  if (values.help) {
    io.out(help);
    return 0;
  }
  const need = cmd === "compare" ? 2 : 1;
  if (positionals.length !== need) {
    io.err(`robot-studio ${cmd}: expected ${need} argument(s)\n${help}`);
    return 3;
  }
  const dir = positionals[0];
  if (!io.exists(`${dir}/trajectory.csv`) && !io.exists(`${dir}/odometry.csv`)) {
    io.err(`robot-studio ${cmd}: ${dir} is not a recording (no trajectory.csv or odometry.csv)\n`);
    return 3;
  }
  let rec;
  try {
    rec = readRecordingDir(dir, io);
  } catch (e) {
    if (e instanceof RecordingError) {
      io.err(`robot-studio ${cmd}: ${e.message}\n`);
      return 2;
    }
    throw e;
  }
  if (cmd === "bag") {
    if (!values.out) {
      io.err(`robot-studio bag: give the bag directory with -o\n`);
      return 3;
    }
    const name =
      values.name ??
      values.out
        .replace(/[\\/]+$/, "")
        .split(/[\\/]/)
        .pop() ??
      "bag";
    const bag = recordingToBag(rec, { name, library: `robot-studio ${VERSION}` });
    io.mkdir(values.out);
    for (const [path, data] of Object.entries(bag.files))
      io.writeFile(`${values.out}/${path.split("/").pop()}`, data);
    const lines = [
      `${rec.simulated ? "SIMULATED data (labelled so in the bag)" : "recorded data"}: ${bag.messages} messages, ${bag.bytes} bytes of MCAP`,
      ...Object.entries(bag.counts).map(([t, n]) => `  ${t.padEnd(24)} ${String(n).padStart(7)}`),
      `wrote     ${values.out}/metadata.yaml, ${values.out}/${name}_0.mcap`,
    ];
    io.out(lines.join("\n") + "\n");
    return 0;
  }
  if (cmd === "replay") {
    const particles = values.particles === undefined ? undefined : Number(values.particles);
    if (particles !== undefined && !(Number.isInteger(particles) && particles >= 0)) {
      io.err("robot-studio replay: --particles must be an integer >= 0\n");
      return 3;
    }
    const session = new ReplaySession(rec, {
      scanMatching: !values["no-scan-matching"],
      particles,
    });
    session.runToEnd();
    const res = session.results();
    if (values.json) {
      io.out(JSON.stringify(res, null, 2) + "\n");
      return 0;
    }
    io.out(replayReport(res) + "\n");
    return 0;
  }
  // compare
  const sched = scheduleOf(rec);
  if (sched.t.length === 0) {
    io.err("robot-studio compare: the recording has no commanded twist (cmd_vel)\n");
    return 2;
  }
  let run;
  try {
    run = simulateSchedule(positionals[1], sched, rec, {
      world: values.world,
      set: values.set ?? [],
      ideal: values.ideal,
      seed: Number(values.seed),
      readText: io.readText,
      exists: io.exists,
      load: (input) => loadRobotInput(input, {}, io.exists),
    });
  } catch (e) {
    io.err(`robot-studio compare: ${e instanceof Error ? e.message : String(e)}\n`);
    return 2;
  }
  if (values.out) {
    io.mkdir(values.out);
    for (const [n, data] of Object.entries(run.files)) io.writeFile(`${values.out}/${n}`, data);
  }
  const cmp = compareRecordings(rec, parseRecording(run.files));
  if (values.json) {
    io.out(JSON.stringify(cmp, null, 2) + "\n");
    return 0;
  }
  io.out(compareReport(cmp, rec.simulated) + "\n");
  return 0;
}

/** @param {ReturnType<ReplaySession["results"]>} res */
function replayReport(res) {
  const f = (/** @type {number | null | undefined} */ v, d = 3) =>
    typeof v === "number" ? v.toFixed(d) : "-";
  const lines = [
    res.simulated
      ? "REPLAY of SIMULATED data (the recording says it is simulator output)"
      : "REPLAY of recorded data (no ground truth: no error measures)",
    `replayed  ${f(res.seconds, 2)} s, ${res.scans} scans, ${res.scans_integrated} integrated, scan matches accepted ${res.scan_match_accept_rate === null ? "-" : `${f(100 * res.scan_match_accept_rate, 1)} %`}`,
  ];
  for (const [k, p] of Object.entries(res.final))
    if (p) lines.push(`final     ${k.padEnd(14)} x ${f(p.x)} y ${f(p.y)} theta ${f(p.theta)}`);
  if (res.errors_m) {
    lines.push("position error   final     RMS");
    for (const [k, v] of Object.entries(res.errors_m))
      if (v) lines.push(`${k.padEnd(16)} ${f(v.final).padStart(7)} m ${f(v.rms).padStart(7)} m`);
  }
  for (const n of res.notes) lines.push(`note      ${n}`);
  return lines.join("\n");
}

/**
 * @param {ReturnType<typeof compareRecordings>} cmp
 * @param {boolean} simulated
 */
function compareReport(cmp, simulated) {
  const f = (/** @type {number | null} */ v, d = 4) => (v === null ? "-" : v.toFixed(d));
  const lines = [
    `SIMULATION vs ${simulated ? "a SIMULATED recording (not a robot)" : "the recording"}: the recording's command schedule run on the simulator`,
    `compared  ${cmp.samples} ODOMETRY samples over ${f(cmp.seconds, 2)} s; ${cmp.wheelSamples} WHEEL_STATE samples`,
    "field            RMS diff   max diff   final (recording / simulation)",
  ];
  for (const r of cmp.fields)
    lines.push(
      `${r.field.padEnd(16)} ${f(r.rms).padStart(9)} ${f(r.max).padStart(10)}   ${f(r.finalA)} / ${f(r.finalB)}`,
    );
  for (const n of cmp.notes) lines.push(`note      ${n}`);
  return lines.join("\n");
}
