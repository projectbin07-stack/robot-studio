// Simulation vs robot: the command schedule of a recording (its commanded
// twist, from the trajectory or an imported bag's /cmd_vel) is run on the
// simulator with the same robot, and the two runs are compared through the
// firmware telemetry fields the simulator writes: ODOMETRY (x, y, theta,
// vx, vy, wz, the firmware's own pose of the drive frame from where it
// started) and WHEEL_STATE (wheel speeds and encoder counts), or the
// JointState wheel velocities when a recording has no wheel telemetry.

import { simRobot } from "../sim/robot.js";
import { createSim, step, DT } from "../sim/sim.js";
import { bundledWorld, parseSimWorld } from "../sim/world.js";
import { normalizeSettings } from "../sim/settings.js";
import { recordingFiles } from "../sim/record.js";
import { wrapAngle } from "../sim/se2.js";

/** @typedef {import("./recording.js").Recording} Recording */
/** @typedef {import("./recording.js").Table} Table */
/** @typedef {import("../model.js").LoadedModel} LoadedModel */
/** @typedef {import("../builder/document.js").RobotDoc} RobotDoc */
/** @typedef {import("../sim/firmware.js").Twist} Twist */
/** @typedef {import("../sim/world.js").SimWorld} SimWorld */

/**
 * @typedef {object} Schedule
 * @property {number[]} t s, increasing
 * @property {Twist[]} twist command from t on (until the next sample)
 */

/**
 * The commanded twist of a recording, in time order.
 * @param {Recording} rec
 * @returns {Schedule}
 */
export function scheduleOf(rec) {
  const tr = rec.trajectory;
  /** @type {Schedule} */
  const s = { t: [], twist: [] };
  if (!tr || !tr.columns.includes("cmd_vx")) return s;
  for (let i = 0; i < tr.rows; i++) {
    const tw = { vx: tr.col.cmd_vx[i], vy: tr.col.cmd_vy[i], wz: tr.col.cmd_wz[i] };
    if (![tw.vx, tw.vy, tw.wz].every(Number.isFinite)) continue;
    if (s.t.length && tr.col.t[i] < s.t[s.t.length - 1]) continue;
    s.t.push(tr.col.t[i]);
    s.twist.push(tw);
  }
  return s;
}

/**
 * Command for the control period [t0, t0 + DT): the last sample at or
 * before its end. (A simulated recording stamps each command with the end of
 * the period it was applied in, so this reproduces it exactly; for a robot's
 * cmd_vel it applies each message from the period it arrives in.)
 * @param {Schedule} s
 * @param {number} t0
 * @param {{ i: number }} cursor
 */
export function commandAt(s, t0, cursor) {
  const end = t0 + DT + 1e-9;
  while (cursor.i + 1 < s.t.length && s.t[cursor.i + 1] <= end) cursor.i++;
  if (s.t.length === 0 || s.t[cursor.i] > end) return { vx: 0, vy: 0, wz: 0 };
  return s.twist[cursor.i];
}

/**
 * Run a schedule on the simulator.
 * @param {{ model: LoadedModel, doc: RobotDoc | null }} robot
 * @param {Schedule} sched
 * @param {Recording} rec the recording (its world and seed are reused unless overridden)
 * @param {{ world?: SimWorld | null, settings?: Record<string, any>, seed?: number }} [opts]
 * @returns {{ files: Record<string, string | Uint8Array>, seconds: number }}
 */
export function runSchedule(robot, sched, rec, opts = {}) {
  const r = simRobot(robot.model, robot.doc);
  if (!r.ok) throw new Error(`cannot simulate this robot: ${r.reason}`);
  const world = opts.world ?? rec.world ?? /** @type {SimWorld} */ (bundledWorld("empty"));
  const settings = normalizeSettings({ ...(opts.settings ?? {}), seed: opts.seed ?? 1 });
  const sim = createSim(r.robot, world, settings, { log: true });
  const end = sched.t.length ? sched.t[sched.t.length - 1] : 0;
  const steps = Math.round(end / DT);
  const cursor = { i: 0 };
  for (let k = 0; k < steps; k++) step(sim, commandAt(sched, sim.t, cursor));
  const files = recordingFiles(sim, {
    route: "command schedule of a recording",
    input: robot.doc?.name ?? "robot",
    worldId: world.id,
  });
  return { files, seconds: sim.t };
}

/**
 * For the CLI: load the robot and the options, then runSchedule().
 * @param {string} input template id or file
 * @param {Schedule} sched
 * @param {Recording} rec
 * @param {{ world?: string, set: string[], ideal: boolean, seed: number,
 *   readText: (p: string) => string, exists: (p: string) => boolean,
 *   load: (input: string) => { ok: true, model: LoadedModel, doc: RobotDoc } | { ok: false, message: string } }} o
 */
export function simulateSchedule(input, sched, rec, o) {
  const loaded = o.load(input);
  if (!loaded.ok) throw new Error(loaded.message);
  /** @type {SimWorld | null} */
  let world = null;
  if (o.world) {
    world = bundledWorld(o.world);
    if (!world) {
      if (!o.exists(o.world)) throw new Error(`unknown world '${o.world}'`);
      world = parseSimWorld(o.readText(o.world));
    }
  }
  /** @type {Record<string, any>} */
  const given = {};
  for (const kv of o.set) {
    const m = /^([A-Za-z.]+)=(.*)$/.exec(kv);
    if (!m) throw new Error(`--set expects key=value, got '${kv}'`);
    const v = m[2] === "true" ? true : m[2] === "false" ? false : Number(m[2]);
    if (typeof v === "number" && !Number.isFinite(v)) throw new Error(`--set ${kv}: not a number`);
    const keys = m[1].split(".");
    let cur = given;
    for (const k of keys.slice(0, -1)) cur = cur[k] ??= {};
    cur[keys[keys.length - 1]] = v;
  }
  if (o.ideal)
    given.drive = {
      ...(given.drive ?? {}),
      ideal: true,
      slip: false,
      radiusErrorLeft: 0,
      radiusErrorRight: 0,
    };
  if (!Number.isInteger(o.seed) || o.seed < 0) throw new Error("--seed must be an integer >= 0");
  return runSchedule(loaded, sched, rec, { world, settings: given, seed: o.seed });
}

/**
 * Linear interpolation of column c of a table at time t (times in column tc,
 * increasing); NaN outside the table.
 * @param {Table} tb
 * @param {string} tc
 * @param {string} c
 * @param {number} t
 * @param {{ i: number }} cur
 * @param {boolean} [angle]
 */
function interp(tb, tc, c, t, cur, angle = false) {
  const T = tb.col[tc];
  const V = tb.col[c];
  if (!V || tb.rows === 0 || t < T[0] - 1e-9 || t > T[tb.rows - 1] + 1e-9) return NaN;
  while (cur.i + 1 < tb.rows && T[cur.i + 1] <= t) cur.i++;
  if (cur.i + 1 >= tb.rows || Math.abs(T[cur.i] - t) < 1e-9) return V[cur.i];
  const a = (t - T[cur.i]) / (T[cur.i + 1] - T[cur.i]);
  if (angle) return wrapAngle(V[cur.i] + a * wrapAngle(V[cur.i + 1] - V[cur.i]));
  return V[cur.i] + a * (V[cur.i + 1] - V[cur.i]);
}

/**
 * @typedef {object} FieldDiff
 * @property {string} field
 * @property {number | null} rms
 * @property {number | null} max
 * @property {number | null} finalA recording
 * @property {number | null} finalB simulation
 * @property {number} n samples
 */

/**
 * @param {string} field
 * @param {number[]} a
 * @param {number[]} b
 * @param {boolean} angle
 * @returns {FieldDiff}
 */
function diffStats(field, a, b, angle) {
  let ss = 0;
  let mx = 0;
  let n = 0;
  let lastA = NaN;
  let lastB = NaN;
  for (let i = 0; i < a.length; i++) {
    if (!Number.isFinite(a[i]) || !Number.isFinite(b[i])) continue;
    const d = angle ? wrapAngle(b[i] - a[i]) : b[i] - a[i];
    ss += d * d;
    mx = Math.max(mx, Math.abs(d));
    n++;
    lastA = a[i];
    lastB = b[i];
  }
  const r = (/** @type {number} */ v) => (Number.isFinite(v) ? Number(v.toPrecision(6)) : null);
  return {
    field,
    rms: n ? r(Math.sqrt(ss / n)) : null,
    max: n ? r(mx) : null,
    finalA: r(lastA),
    finalB: r(lastB),
    n,
  };
}

/**
 * Compare recording A (the robot, or a bag standing in for it) with B (the
 * simulated run of A's schedule) at A's telemetry times.
 * @param {Recording} a
 * @param {Recording} b
 */
export function compareRecordings(a, b) {
  /** @type {string[]} */
  const notes = [];
  /** @type {FieldDiff[]} */
  const fields = [];
  let samples = 0;
  let seconds = 0;
  if (a.odometry && b.odometry) {
    const ta = a.odometry.col.t_s;
    samples = a.odometry.rows;
    seconds = samples ? ta[samples - 1] - ta[0] : 0;
    for (const [c, angle] of /** @type {[string, boolean][]} */ ([
      ["x", false],
      ["y", false],
      ["theta", true],
      ["vx", false],
      ["vy", false],
      ["wz", false],
    ])) {
      const cur = { i: 0 };
      const va = Array.from(a.odometry.col[c]);
      const vb = Array.from(ta, (t) =>
        interp(/** @type {Table} */ (b.odometry), "t_s", c, t, cur, angle),
      );
      fields.push(diffStats(`odometry.${c}`, va, vb, angle));
    }
  } else notes.push("one of the recordings has no odometry.csv (firmware ODOMETRY)");
  let wheelSamples = 0;
  const wa = a.wheelState;
  const wb = b.wheelState;
  if (wa && wb && wa.columns.includes("velocity_fl") && wb.columns.includes("velocity_fl")) {
    wheelSamples = wa.rows;
    const ta = Array.from(wa.col.t_us, (v) => v / 1e6);
    /** @type {Table} */
    const wbt = { ...wb, col: { ...wb.col, t: Float64Array.from(wb.col.t_us, (v) => v / 1e6) } };
    for (const w of ["fl", "fr", "rl", "rr"])
      for (const kind of ["velocity", "count"]) {
        const c = `${kind}_${w}`;
        const cur = { i: 0 };
        fields.push(
          diffStats(
            `wheel_state.${c}`,
            Array.from(wa.col[c]),
            ta.map((t) => interp(wbt, "t", c, t, cur)),
            false,
          ),
        );
      }
  } else if (a.jointStates && b.jointStates) {
    const ja = a.jointStates;
    const jb = b.jointStates;
    const common = ja.columns.filter((c) => c.startsWith("vel:") && jb.columns.includes(c));
    if (common.length === 0) notes.push("no wheel joints in common: wheel speeds not compared");
    wheelSamples = common.length ? ja.rows : 0;
    for (const c of common) {
      const cur = { i: 0 };
      fields.push(
        diffStats(
          `joint_states.${c.slice(4)}.velocity`,
          Array.from(ja.col[c]),
          Array.from(ja.col.t, (t) => interp(jb, "t", c, t, cur)),
          false,
        ),
      );
    }
  } else notes.push("no wheel telemetry in one of the recordings: wheel speeds not compared");
  return { samples, seconds, wheelSamples, fields, notes };
}
