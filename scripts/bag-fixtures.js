#!/usr/bin/env node
// Writes the bag fixture the Python companion's tests import: a short
// simulated run (mecanum template, office, demo route, seed 1) as its
// recording and as the MCAP bag Robot Studio exports from it. Both are
// simulator output and labelled so. With --check, fails if the committed
// files differ from what the code writes today (CI runs this).

import { mkdirSync, readFileSync, writeFileSync, existsSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { generate } from "../src/core/builder/templates.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import { simRobot } from "../src/core/sim/robot.js";
import { bundledWorld } from "../src/core/sim/world.js";
import { normalizeSettings } from "../src/core/sim/settings.js";
import { createSim, step } from "../src/core/sim/sim.js";
import { createRoute } from "../src/core/sim/routes.js";
import { recordingFiles } from "../src/core/sim/record.js";
import { parseRecording } from "../src/core/ros/recording.js";
import { recordingToBag } from "../src/core/ros/bag.js";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const out = join(root, "python", "tests", "fixtures", "sim_mecanum");
const check = process.argv.includes("--check");

const doc = generate("mecanum", {});
const r = simRobot(modelFromDoc(doc), doc);
if (!r.ok) throw new Error(r.reason);
const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld("office"));
const sim = createSim(r.robot, world, normalizeSettings({ seed: 1 }), { log: true });
const route = createRoute("demo", r.robot, world);
for (let k = 0; k < 300; k++) step(sim, route.next(sim.drive, sim.t).twist);
const files = recordingFiles(sim, { route: "demo", input: "mecanum", worldId: world.id });
/** @type {Record<string, string | Uint8Array>} */
const want = {};
for (const name of [
  "trajectory.csv",
  "scans.jsonl",
  "imu.csv",
  "wheel_state.csv",
  "odometry.csv",
  "joint_states.csv",
  "summary.json",
  "world.json",
])
  want[`run/${name}`] = files[name];
const bag = recordingToBag(parseRecording(files), { name: "bag", library: "robot-studio" });
for (const [path, data] of Object.entries(bag.files)) want[path] = data;

let bad = 0;
for (const [path, data] of Object.entries(want)) {
  const p = join(out, path);
  const bytes = typeof data === "string" ? Buffer.from(data, "utf8") : Buffer.from(data);
  if (check) {
    if (!existsSync(p) || !readFileSync(p).equals(bytes)) {
      console.error(`differs: ${path}`);
      bad++;
    }
  } else {
    mkdirSync(dirname(p), { recursive: true });
    writeFileSync(p, bytes);
  }
}
if (check && bad) {
  console.error(`${bad} file(s) differ; run node scripts/bag-fixtures.js`);
  process.exit(1);
}
console.log(
  `${check ? "checked" : "wrote"} ${Object.keys(want).length} file(s) in python/tests/fixtures/sim_mecanum`,
);
