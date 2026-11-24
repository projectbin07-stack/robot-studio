// The page's simulation session (DOM-free): fixed steps from real time,
// teleop overriding the route, plot histories, the recording cap, and the
// URL state of the simulation.

import { test } from "node:test";
import assert from "node:assert/strict";
import { generate } from "../src/core/builder/templates.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import { History, MAX_ADVANCE, SimSession } from "../src/core/sim/session.js";
import { bundledWorld } from "../src/core/sim/world.js";
import { normalizeSettings } from "../src/core/sim/settings.js";
import { decodeState, encodeState, defaultState } from "../src/ui/state.js";

/** @param {string} id */
function session(id = "mecanum", route = "demo", opts = {}) {
  const doc = generate(id, {});
  const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld("lab"));
  let clock = 0;
  return new SimSession(modelFromDoc(doc), doc, world, route, normalizeSettings({}), {
    clock: () => (clock += 0.5),
    ...opts,
  });
}

test("real time becomes fixed 10 ms steps; a stalled tab cannot run away", () => {
  const s = session();
  assert.equal(s.advance(0.004), 0);
  assert.equal(s.advance(0.007), 1);
  assert.ok(Math.abs(s.accumulator - 0.001) < 1e-12);
  assert.equal(s.advance(10), Math.floor((MAX_ADVANCE + 0.001 + 1e-9) / 0.01));
  assert.ok(s.stepMs > 0);
  assert.ok(s.sim.distance > 0);
  assert.ok(s.history.errors.length > 1);
  assert.ok(s.history.wheels.at(s.history.wheels.length - 1).values[4] > 0);
  const snap = s.snapshot();
  assert.ok(snap.errors && snap.scan && snap.scanPose);
  assert.equal(Object.keys(snap.q).length, 4);
  // The 3D layer's frame: the same shape replays and live robots produce.
  const f = s.frame();
  assert.equal(f.base, s.sim.truth);
  assert.ok(f.grid && f.scan && f.trails.odom && f.trails.est && f.odomFrame);
  assert.equal(f.lidarHeight, s.sim.lidar?.sensor.height);
  assert.deepEqual(f.mapFrame, s.world.start);
  s.reset();
  assert.equal(s.sim.t, 0);
  assert.equal(s.history.twist.length, 1);
});

test("a held teleop command overrides the route; releasing it hands back to the route", () => {
  const s = session("mecanum", "none");
  s.manual = { vx: 0, vy: 0.2, wz: 0 };
  for (let k = 0; k < 20; k++) s.advance(0.05);
  assert.ok(s.sim.truth.y - 4 > 0.1, `${s.sim.truth.y}`);
  assert.ok(Math.abs(s.sim.command.vy - 0.2) < 1e-12);
  s.manual = null;
  s.advance(0.05);
  assert.equal(s.sim.command.vy, 0);
  assert.equal(s.routeDone, false);
  const d = session("diff", "square");
  for (let k = 0; k < 10; k++) d.advance(0.25);
  assert.ok(d.files("diff")["summary.json"]);
});

test("the page keeps the first 10 simulated minutes of the recording", () => {
  const s = session("mecanum", "none");
  s.sim.t = 599.995; // jump ahead: only the recording cap is under test
  s.sim.steps = 59999;
  s.advance(0.02);
  assert.equal(s.sim.log, null);
  const files = s.files("x");
  assert.ok(files["trajectory.csv"]);
  assert.equal(s.files("y"), files);
  const quiet = session("mecanum", "none", { log: false });
  assert.deepEqual(quiet.files("x"), {});
  assert.throws(() => session("arm"), /no drive wheels/);
});

test("history ring: oldest first, fixed capacity", () => {
  const h = new History(2, 3);
  for (let i = 0; i < 5; i++) h.push(i, [i, 10 * i]);
  assert.equal(h.length, 3);
  assert.equal(h.at(0).t, 2);
  assert.deepEqual([...h.at(2).values], [4, 40]);
  h.push(5, [1]);
  assert.ok(Number.isNaN(h.at(2).values[1]));
  h.clear();
  assert.equal(h.length, 0);
});

test("the simulation's world, route and settings go into the link", () => {
  const st = defaultState();
  assert.equal(encodeState(st), "");
  st.sim = true;
  st.world = "lab";
  st.route = "square";
  st.simSettings = { seed: 4, lidar: { beams: 360 }, imu: { calibration: { kind: "gyro" } } };
  const q = encodeState(st);
  assert.match(q, /sim=1/);
  assert.match(q, /w=lab/);
  assert.match(q, /r=square/);
  assert.ok(!q.includes("calibration"), "a calibration file is not put in a link");
  const back = decodeState(q);
  assert.equal(back.sim, true);
  assert.equal(back.world, "lab");
  assert.equal(back.route, "square");
  assert.deepEqual(back.simSettings, { seed: 4, lidar: { beams: 360 } });
  const bad = decodeState("?sim=1&r=Bad!&ss=%7Bnope&w=");
  assert.equal(bad.route, "demo");
  assert.deepEqual(bad.simSettings, {});
  assert.equal(bad.world, "office");
  assert.deepEqual(decodeState("?ss=%5B1%5D").simSettings, {});
  st.sim = false;
  assert.equal(encodeState(st), "", "nothing about the simulation while it is closed");
});
