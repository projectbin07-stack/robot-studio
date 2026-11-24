// Replaying recordings through the estimators (ReplaySession over a Track)
// and the simulation-vs-recording comparison.

import { test } from "node:test";
import assert from "node:assert/strict";
import { generate } from "../src/core/builder/templates.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import { simRobot } from "../src/core/sim/robot.js";
import { bundledWorld } from "../src/core/sim/world.js";
import { normalizeSettings } from "../src/core/sim/settings.js";
import { createSim, step } from "../src/core/sim/sim.js";
import { createRoute } from "../src/core/sim/routes.js";
import { recordingFiles } from "../src/core/sim/record.js";
import { parseRecording } from "../src/core/ros/recording.js";
import { ReplaySession } from "../src/core/ros/replay.js";
import { commandAt, compareRecordings, runSchedule, scheduleOf } from "../src/core/ros/compare.js";

/**
 * A logged simulation run's files.
 * @param {string} template
 * @param {{ seconds?: number, world?: string, route?: string, settings?: Record<string, any> }} [o]
 */
function run(template, o = {}) {
  const doc = generate(template, {});
  const model = modelFromDoc(doc);
  const r = simRobot(model, doc);
  if (!r.ok) throw new Error(r.reason);
  const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (
    bundledWorld(o.world ?? "office")
  );
  const sim = createSim(r.robot, world, normalizeSettings(o.settings ?? {}), { log: true });
  const route = createRoute(o.route ?? "demo", r.robot, world);
  const steps = Math.round((o.seconds ?? 3) / 0.01);
  for (let k = 0; k < steps; k++) step(sim, route.next(sim.drive, sim.t).twist);
  return {
    sim,
    doc,
    model,
    files: recordingFiles(sim, { route: o.route ?? "demo", input: template, worldId: world.id }),
  };
}

test("replay: a simulated recording through the estimators, with errors against its truth", () => {
  const { files, sim } = run("mecanum", { seconds: 12 });
  const rec = parseRecording(files);
  const s = new ReplaySession(rec);
  assert.equal(s.start, 0);
  assert.ok(Math.abs(s.end - 12) < 1e-9);
  s.advance(5);
  assert.ok(!s.done);
  const f = s.frame();
  assert.ok(f.scan && f.grid && f.trails.truth && f.trails.est);
  assert.ok(Math.abs(f.t - 5) < 0.011);
  s.runToEnd();
  assert.ok(s.done);
  const res = s.results();
  assert.equal(res.simulated, true);
  assert.equal(res.scans, rec.scans?.items.length);
  assert.equal(res.scans_integrated, res.scans);
  const e = /** @type {NonNullable<typeof res.errors_m>} */ (res.errors_m);
  // The replay's scan matching corrects the wheel odometry, as the simulation's own did.
  assert.ok(sim.est);
  assert.ok(
    /** @type {number} */ (e.scan_matching.final) < /** @type {number} */ (e.wheel_odometry.final),
  );
  assert.ok(/** @type {number} */ (e.scan_matching.rms) < 0.15, JSON.stringify(e));
  assert.ok(e.mcl && /** @type {number} */ (e.mcl.rms) < 0.15, JSON.stringify(e));
  // History for the plots: wheels from WHEEL_STATE, twist, IMU, errors.
  assert.ok(s.history.wheels.length > 100 && s.history.errors.length > 100);
  // Without scan matching and MCL, the estimate is the odometry.
  const s2 = new ReplaySession(rec, { scanMatching: false, particles: 0 });
  s2.runToEnd();
  const r2 = s2.results();
  assert.deepEqual(r2.final.scan_matching, r2.final.odometry);
  assert.equal(r2.errors_m?.mcl, null);
  // Estimators off: odometry only, no grid.
  const s3 = new ReplaySession(rec, { estimate: false });
  s3.runToEnd();
  assert.equal(s3.frame().grid, null);
  assert.equal(s3.results().errors_m, null);
});

test("replay of data without truth or wheel telemetry (a robot's bag): odometry, joint speeds, no errors", () => {
  const rec = parseRecording({
    "trajectory.csv":
      "t,odom_x,odom_y,odom_theta,cmd_vx,cmd_vy,cmd_wz\n0.1,0,0,0,0.2,0,0\n0.2,0.02,0,0,0.2,0,0\n0.3,0.04,0,0.01,0.2,0,0\n",
    "odometry.csv": "t_s,x,y,theta,vx,vy,wz\n0.1,0,0,0,0.2,0,0\n",
    "joint_states.csv": "t,pos:a,pos:b,vel:a,vel:b\n0.1,0,0,1,2\n0.2,0.1,0.2,1,2\n",
    "scans.jsonl":
      '{"header":{"frame_id":"l","angle_min":-1.5,"angle_increment":0.5,"range_min":0.1,"range_max":4,"scan_time":0.1}}\n' +
      '{"t":0.05,"ranges":[1,1,1,1,1,1,1]}\n{"t":0.15,"ranges":[1,1,1,1,1,null,1]}\n{"t":0.25,"ranges":[1,1,1,1,1,1,1]}\n',
  });
  const s = new ReplaySession(rec, { wheelJoints: ["a", "b"] });
  s.runToEnd();
  const r = s.results();
  assert.equal(r.simulated, false);
  assert.equal(r.errors_m, null);
  assert.equal(r.scans, 3);
  assert.equal(r.scans_integrated, 2); // the first scan came before any odometry
  assert.ok(r.notes.some((n) => /no mount pose/.test(n)));
  assert.ok(r.notes.some((n) => /before the first odometry/.test(n)));
  const f = s.frame();
  assert.deepEqual(f.q, { a: 0.1, b: 0.2 });
  const last = s.history.wheels.at(s.history.wheels.length - 1).values;
  assert.deepEqual(Array.from(last.subarray(4, 6)), [1, 2]);
  // Odometry-only recording (no trajectory).
  const o = new ReplaySession(
    parseRecording({ "odometry.csv": "t_s,x,y,theta,vx,vy,wz\n0.05,0.1,0,0,0.5,0,0\n" }),
  );
  o.runToEnd();
  assert.deepEqual(o.results().final.odometry, { x: 0.1, y: 0, theta: 0 });
});

test("simulation vs recording: the command schedule held per period, and telemetry compared", () => {
  const sched = {
    t: [0.01, 0.02, 0.5],
    twist: [
      { vx: 1, vy: 0, wz: 0 },
      { vx: 2, vy: 0, wz: 0 },
      { vx: 0, vy: 0, wz: 1 },
    ],
  };
  const cur = { i: 0 };
  assert.equal(commandAt(sched, 0, cur).vx, 1); // the period [0, 0.01) ends at the first sample
  assert.equal(commandAt(sched, 0.01, cur).vx, 2);
  assert.equal(commandAt(sched, 0.3, cur).vx, 2);
  assert.equal(commandAt(sched, 0.49, cur).wz, 1);
  assert.deepEqual(commandAt({ t: [], twist: [] }, 0, { i: 0 }), { vx: 0, vy: 0, wz: 0 });
  assert.deepEqual(commandAt({ t: [5], twist: [{ vx: 1, vy: 0, wz: 0 }] }, 0, { i: 0 }), {
    vx: 0,
    vy: 0,
    wz: 0,
  });

  const { files, doc, model } = run("mecanum", { seconds: 6 });
  const rec = parseRecording(files);
  const s = scheduleOf(rec);
  assert.equal(s.t.length, rec.trajectory?.rows);
  // The same robot, world, seed and settings, driven by the recorded (4-decimal) commands.
  const again = runSchedule({ model, doc }, s, rec, { seed: 1 });
  const cmp = compareRecordings(rec, parseRecording(again.files));
  assert.equal(cmp.samples, rec.odometry?.rows);
  const x = /** @type {import("../src/core/ros/compare.js").FieldDiff} */ (
    cmp.fields.find((f) => f.field === "odometry.x")
  );
  assert.ok(x.max !== null && x.max < 1e-3, JSON.stringify(x));
  assert.ok(cmp.fields.some((f) => f.field === "wheel_state.count_fl"));
  // Ideal wheels (no motor lag, no slip): the difference is the drive model's.
  const ideal = runSchedule({ model, doc }, s, rec, {
    seed: 1,
    settings: { drive: { ideal: true, slip: false, radiusErrorLeft: 0, radiusErrorRight: 0 } },
  });
  const c2 = compareRecordings(rec, parseRecording(ideal.files));
  const vx = /** @type {import("../src/core/ros/compare.js").FieldDiff} */ (
    c2.fields.find((f) => f.field === "odometry.vx")
  );
  assert.ok(/** @type {number} */ (vx.max) > 0.01, JSON.stringify(vx));
  // Joint velocities when there is no wheel telemetry; notes when nothing matches.
  const noWs = { ...rec, wheelState: null };
  const c3 = compareRecordings(noWs, parseRecording(again.files));
  assert.ok(c3.fields.some((f) => f.field.startsWith("joint_states.")));
  const c4 = compareRecordings(
    { ...noWs, jointStates: null, odometry: null },
    parseRecording(again.files),
  );
  assert.equal(c4.fields.length, 0);
  assert.equal(c4.notes.length, 2);
  assert.equal(scheduleOf({ ...rec, trajectory: null }).t.length, 0);
  assert.throws(
    () => runSchedule({ model: { ...model, report: null }, doc: null }, s, rec),
    /cannot simulate/,
  );
});
