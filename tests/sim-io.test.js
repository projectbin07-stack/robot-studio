// Worlds (format, links, bundled maps), settings, the recording files and
// their determinism, the document's simulation field, and the CLI.

import { test } from "node:test";
import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { existsSync, mkdtempSync, readFileSync, rmSync } from "node:fs";
import { execFileSync } from "node:child_process";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { generate, isTemplateExact } from "../src/core/builder/templates.js";
import { modelFromDoc, Editor } from "../src/core/builder/editor.js";
import { EditError, applyOp, describeOp } from "../src/core/builder/ops.js";
import { simRobot } from "../src/core/sim/robot.js";
import {
  WORLDS,
  boxCorners,
  bundledWorld,
  cylinderPolygon,
  decodeWorld,
  encodeWorld,
  obstacles,
  parseSimWorld,
  serializeSimWorld,
} from "../src/core/sim/world.js";
import {
  normalizeSettings,
  settingsDiff,
  defaultSimSettings,
  ticksPerWheelRev,
} from "../src/core/sim/settings.js";
import { createSim, step } from "../src/core/sim/sim.js";
import { createRoute } from "../src/core/sim/routes.js";
import { recordingFiles, runSummary } from "../src/core/sim/record.js";
import { main } from "../src/cli.js";
import { parseWorld } from "../vendor/lidar-slam-lab/src/world.js";

const root = fileURLToPath(new URL("..", import.meta.url));

/** @param {string} id */
function robotOf(id) {
  const doc = generate(id, {});
  const r = simRobot(modelFromDoc(doc), doc);
  if (!r.ok) throw new Error(r.reason);
  return r.robot;
}

test("bundled worlds: lidar-slam-lab's maps plus the lab, all valid and round-tripping", () => {
  assert.deepEqual(
    WORLDS.map((w) => w.id),
    ["office", "warehouse", "loop", "lab", "empty"],
  );
  for (const w of WORLDS) {
    const world = w.make();
    const text = serializeSimWorld(world);
    const back = parseSimWorld(text);
    assert.equal(serializeSimWorld(back), text, w.id);
    assert.equal(encodeWorld(world), w.id);
    assert.equal(serializeSimWorld(decodeWorld(w.id)), text);
    // lidar-slam-lab itself still reads the file (it ignores the height fields).
    assert.equal(parseWorld(text).walls.length, world.walls.length);
  }
  const office = /** @type {any} */ (bundledWorld("office"));
  assert.equal(office.walls.length, 15);
  assert.equal(office.wallHeight, 2);
  assert.equal(bundledWorld("nope"), null);
});

test("an edited world goes into a link and comes back unchanged", () => {
  const w = /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld("lab"));
  w.boxes.push({ x: 1, y: 1, w: 0.5, d: 0.3, yaw: 0.25, h: 0.4 });
  w.walls.push([2, 2, 3, 3.5]);
  w.name = "My~lab";
  const enc = encodeWorld(w);
  assert.ok(!enc.includes("My~lab"));
  const back = decodeWorld(enc);
  assert.equal(back.id, "custom");
  assert.equal(back.boxes.length, 5);
  assert.deepEqual(back.walls.at(-1), [2, 2, 3, 3.5]);
  assert.equal(encodeWorld(back), enc);
  assert.throws(() => decodeWorld("x~1"), /malformed/);
  assert.throws(() => decodeWorld("n~10~10~2~1,1,0~0,0,a,1~~~"), /malformed/);
});

test("world validation names the bad field", () => {
  const ok = { version: 1, name: "t", width: 4, height: 4, walls: [], start: { x: 1, y: 1 } };
  assert.equal(parseSimWorld(ok).wallHeight, 2);
  assert.throws(() => parseSimWorld("{"), /not valid JSON/);
  assert.throws(() => parseSimWorld({ ...ok, wallHeight: 0 }), /wallHeight/);
  assert.throws(() => parseSimWorld({ ...ok, boxes: {} }), /boxes must be an array/);
  assert.throws(
    () => parseSimWorld({ ...ok, boxes: [{ x: 1, y: 1, w: 0, d: 1, h: 1 }] }),
    /positive size/,
  );
  assert.throws(
    () => parseSimWorld({ ...ok, boxes: [{ x: 3.9, y: 1, w: 1, d: 1, h: 1 }] }),
    /outside the map/,
  );
  assert.throws(
    () => parseSimWorld({ ...ok, boxes: [{ x: "1" }] }),
    /boxes\[0\]\.x must be a number/,
  );
  assert.throws(
    () => parseSimWorld({ ...ok, cylinders: [{ x: 1, y: 1, r: -1, h: 1 }] }),
    /positive radius/,
  );
  assert.throws(
    () => parseSimWorld({ ...ok, cylinders: new Array(501).fill({ x: 1, y: 1, r: 0.1, h: 1 }) }),
    /at most 500/,
  );
  assert.throws(() => parseSimWorld({ ...ok, start: { x: 9, y: 1 } }), /start must lie inside/);
  // Shapes: a box's corners, a cylinder's polygon contains its circle.
  const c = boxCorners({ x: 0, y: 0, w: 2, d: 1, yaw: Math.PI / 2, h: 1 });
  assert.ok(Math.abs(c[0][0] - 0.5) < 1e-12 && Math.abs(c[0][1] + 1) < 1e-12);
  for (const [x, y] of cylinderPolygon({ x: 0, y: 0, r: 1, h: 1 })) assert.ok(Math.hypot(x, y) > 1);
  const o = obstacles(parseSimWorld({ ...ok, cylinders: [{ x: 2, y: 2, r: 0.2, h: 0.3 }] }));
  assert.equal(o.segments.length, 24);
  assert.deepEqual(new Set(o.heights), new Set([0.3]));
});

test("settings: defaults, range checks and the stored difference", () => {
  const d = defaultSimSettings();
  assert.equal(ticksPerWheelRev(d.drive), 38000);
  assert.deepEqual(settingsDiff(normalizeSettings({})), {});
  const s = normalizeSettings({ seed: 7, lidar: { beams: 360 }, drive: { slip: false } });
  assert.deepEqual(settingsDiff(s), { seed: 7, lidar: { beams: 360 }, drive: { slip: false } });
  assert.throws(
    () => normalizeSettings({ lidar: { beams: 3 } }),
    /lidar\.beams must be an integer in \[8, 2000\]/,
  );
  assert.throws(() => normalizeSettings({ lidar: { beams: 100.5 } }), /integer/);
  assert.throws(() => normalizeSettings({ drive: { slip: 1 } }), /true or false/);
  assert.throws(() => normalizeSettings({ drive: { slipStd: "x" } }), /must be a number/);
  assert.throws(() => normalizeSettings({ wheels: 4 }), /unknown simulation setting 'wheels'/);
  assert.throws(() => normalizeSettings({ lidar: 4 }), /lidar must be an object/);
  assert.throws(() => normalizeSettings({ lidar: { minRange: 3, maxRange: 2 } }), /minRange/);
  const cal = normalizeSettings({ imu: { calibration: { kind: "gyro", bias: [0, 0, 0] } } });
  assert.deepEqual(settingsDiff(cal), { imu: { calibration: { kind: "gyro", bias: [0, 0, 0] } } });
});

/**
 * @param {number} seed
 * @param {number} steps
 */
function recordRun(seed, steps) {
  const robot = robotOf("mecanum");
  const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld("lab"));
  const sim = createSim(robot, world, normalizeSettings({ seed }), { log: true });
  const route = createRoute("demo", robot, world);
  for (let k = 0; k < steps; k++) step(sim, route.next(sim.drive, sim.t).twist);
  return { sim, files: recordingFiles(sim, { route: "demo", input: "mecanum", worldId: "lab" }) };
}

/** @param {Record<string, string | Uint8Array>} files */
const digest = (files) =>
  createHash("sha256")
    .update(
      Object.entries(files)
        .map(([k, v]) => `${k}:${createHash("sha256").update(v).digest("hex")}`)
        .join("\n"),
    )
    .digest("hex");

test("recording: the same seed gives byte-identical files, another seed different ones", () => {
  const a = recordRun(3, 300);
  const b = recordRun(3, 300);
  const c = recordRun(4, 300);
  assert.deepEqual(Object.keys(a.files).sort(), [
    "imu.csv",
    "joint_states.csv",
    "map.pgm",
    "map.yaml",
    "odometry.csv",
    "scans.jsonl",
    "summary.json",
    "trajectory.csv",
    "wheel_state.csv",
    "world.json",
  ]);
  assert.equal(digest(a.files), digest(b.files));
  assert.notEqual(digest(a.files), digest(c.files));
  const f = a.files;
  const lines = (/** @type {string} */ k) => String(f[k]).trimEnd().split("\n");
  assert.equal(lines("trajectory.csv").length, 301);
  assert.equal(lines("imu.csv").length, 301);
  assert.equal(lines("scans.jsonl").length, 1 + 31); // header + first scan + one per 0.1 s
  assert.equal(lines("wheel_state.csv").length, 1 + 60); // 20 Hz telemetry
  assert.equal(lines("odometry.csv")[0], "t_s,x,y,theta,vx,vy,wz");
  assert.match(
    lines("wheel_state.csv")[0],
    /^t_us,count_fl,velocity_fl,target_fl,duty_fl,count_fr,/,
  );
  assert.match(lines("joint_states.csv")[0], /^t,pos:front_left_wheel_joint,/);
  const head = JSON.parse(lines("scans.jsonl")[0]).header;
  assert.equal(head.frame_id, "lidar_link");
  assert.equal(head.range_min, 0.12);
  const scan = JSON.parse(lines("scans.jsonl")[5]);
  assert.equal(scan.ranges.length, 180);
  const summary = JSON.parse(String(f["summary.json"]));
  assert.equal(summary.format, "robot-studio-sim-recording/1");
  assert.match(summary.note, /Simulation output/);
  assert.equal(summary.robot.ticks_per_wheel_rev, 38000);
  assert.ok(summary.result.errors_m.wheel_odometry.rms > 0);
  assert.ok(summary.result.map_precision.within_0_1_m > 0.5);
  assert.equal(String(f["map.yaml"]).split("\n")[0], "image: map.pgm");
  // Without logging there is nothing to write.
  const quiet = createSim(robotOf("mecanum"), a.sim.world, normalizeSettings({}));
  assert.throws(
    () => recordingFiles(quiet, { route: "demo", input: "x", worldId: "lab" }),
    /without logging/,
  );
  const s = runSummary(quiet, { route: "none", input: "x", worldId: "lab" });
  assert.equal(s.result.distance_m, 0);
});

test("a run with a fixed seed has a fixed digest on every platform (golden)", () => {
  // If this changes, the simulator's output changed: check why before updating.
  const { files } = recordRun(1, 200);
  const hashes = Object.fromEntries(
    Object.entries(files).map(([k, v]) => [
      k,
      createHash("sha256").update(v).digest("hex").slice(0, 16),
    ]),
  );
  const golden = JSON.parse(
    readFileSync(join(root, "tests/corpus/sim/golden-digest.json"), "utf8"),
  );
  assert.deepEqual(hashes, golden.files);
});

test("the document keeps the simulation settings; edits are checked and undoable", () => {
  const doc = generate("mecanum", {});
  const sim = { world: "lab", route: "square", settings: { seed: 3, lidar: { beams: 360 } } };
  const next = applyOp(doc, { type: "setSimulation", sim });
  assert.deepEqual(next.sim, sim);
  assert.equal(isTemplateExact(next), true, "simulation settings are not robot edits");
  assert.equal(describeOp({ type: "setSimulation", sim }), "simulation settings");
  assert.equal(applyOp(next, { type: "setSimulation", sim: null }).sim, undefined);
  assert.throws(
    () => applyOp(doc, { type: "setSimulation", sim: { ...sim, world: "nope" } }),
    EditError,
  );
  assert.throws(
    () => applyOp(doc, { type: "setSimulation", sim: { ...sim, route: "zigzag" } }),
    /unknown route/,
  );
  assert.throws(
    () => applyOp(doc, { type: "setSimulation", sim: { ...sim, settings: { seed: -1 } } }),
    /seed/,
  );
  assert.throws(
    () => applyOp(doc, { type: "setSimulation", sim: /** @type {any} */ ({ world: 1 }) }),
    /needs a world/,
  );
  const ed = new Editor(doc);
  ed.apply({ type: "setSimulation", sim });
  assert.equal(ed.doc.sim?.route, "square");
  ed.undo();
  assert.equal(ed.doc.sim, undefined);
});

/** @param {string[]} argv */
function cli(argv) {
  let out = "";
  let err = "";
  /** @type {Record<string, string | Uint8Array>} */
  const written = {};
  const code = main(argv, {
    out: (t) => (out += t),
    err: (t) => (err += t),
    writeFile: (p, d) => (written[p] = d),
    exists: (p) => existsSync(p),
    readText: (p) => readFileSync(p, "utf8"),
    mkdir: () => {},
    now: () => 0,
  });
  return { code, out, err, written };
}

test("CLI sim: template or file, text and JSON reports, files, usage errors", () => {
  const t = cli(["sim", "mecanum", "--world", "lab", "--seconds", "3", "--seed", "2"]);
  assert.equal(t.code, 0, t.err);
  assert.match(t.out, /^SIMULATION: model output/);
  assert.match(t.out, /LiDAR lidar_link: 180 beams, 10\.0 Hz/);
  assert.match(t.out, /wheel odometry/);
  assert.match(t.out, /MCL \(300 particles\)/);
  const j = cli([
    "sim",
    "diff",
    "--world",
    "empty",
    "--route",
    "square",
    "--seconds",
    "2",
    "--json",
    "--ideal",
  ]);
  assert.equal(j.code, 0, j.err);
  const s = JSON.parse(j.out);
  assert.equal(s.robot.drive, "differential");
  assert.equal(s.route, "square");
  assert.deepEqual(s.settings_changed.drive, {
    ideal: true,
    slip: false,
    radiusErrorLeft: 0,
    radiusErrorRight: 0,
  });
  const o = cli([
    "sim",
    "mecanum",
    "--seconds",
    "1",
    "--out",
    "run",
    "--set",
    "lidar.beams=90",
    "--set",
    "estimators.particles=0",
  ]);
  assert.equal(o.code, 0, o.err);
  assert.ok(o.written["run/summary.json"]);
  assert.match(o.out, /wrote {5}run\/ \(10 files\)/);
  assert.equal(JSON.parse(String(o.written["run/scans.jsonl"]).split("\n")[1]).ranges.length, 90);
  const f = cli([
    "sim",
    join(root, "examples/third_party/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro"),
    "--seconds",
    "1",
    "--world",
    "loop",
  ]);
  assert.equal(f.code, 0, f.err);
  assert.match(f.out, /LiDAR base_scan: 360 beams, 5\.0 Hz, 0\.12-3\.5 m \(URDF <gazebo> tag\)/);
  assert.match(f.out, /no <limit velocity>/);
  const w = cli([
    "sim",
    "mecanum",
    "--seconds",
    "1",
    "--world",
    join(root, "tests/corpus/sim/world-small.json"),
  ]);
  assert.equal(w.code, 0, w.err);
  assert.equal(cli(["sim", "--help"]).code, 0);
  for (const [args, re] of /** @type {[string[], RegExp][]} */ ([
    [["sim"], /give one robot/],
    [["sim", "mecanum", "--seconds", "0"], /--seconds/],
    [["sim", "mecanum", "--seed", "-1"], /--seed/],
    [["sim", "mecanum", "--world", "mars"], /unknown world 'mars'/],
    [["sim", "mecanum", "--route", "zigzag"], /unknown route/],
    [["sim", "mecanum", "--set", "lidar.beams"], /key=value/],
    [["sim", "mecanum", "--set", "lidar.beams=lots"], /must be a number/],
    [["sim", "mecanum", "--set", "lidar.beams=2"], /lidar\.beams/],
    [["sim", "mecanum", "--imu-cal", join(root, "package.json")], /no accel, gyro or allan/],
    [["sim", "mecanum", "--bogus"], /Unknown option/],
    [["sim", "mecanum", "--arg", "x"], /name=value/],
    [["sim", "nofile.urdf"], /cannot read file/],
  ])) {
    const r = cli(args);
    assert.equal(r.code, 3, `${args.join(" ")}: ${r.err}`);
    assert.match(r.err, re);
  }
  const arm = cli(["sim", "arm"]);
  assert.equal(arm.code, 2);
  assert.match(arm.err, /no drive wheels/);
});

test("CLI sim twice with the same seed writes byte-identical files (real files, separate processes)", () => {
  const tmp = mkdtempSync(join(process.env.TMP ?? tmpdir(), "sim-det-"));
  try {
    const bin = join(root, "bin/robot-studio.js");
    for (const d of ["a", "b"])
      execFileSync(process.execPath, [
        bin,
        "sim",
        "mecanum",
        "--world",
        "office",
        "--seconds",
        "5",
        "--seed",
        "1",
        "--out",
        join(tmp, d),
      ]);
    const names = [
      "trajectory.csv",
      "scans.jsonl",
      "imu.csv",
      "wheel_state.csv",
      "odometry.csv",
      "joint_states.csv",
      "map.pgm",
      "summary.json",
    ];
    for (const n of names)
      assert.ok(readFileSync(join(tmp, "a", n)).equals(readFileSync(join(tmp, "b", n))), n);
  } finally {
    rmSync(tmp, { recursive: true, force: true });
  }
});
