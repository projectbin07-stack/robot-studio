#!/usr/bin/env node
// Cross-check the estimators against lidar-slam-lab's own benchmark.
//
// 1. Reference configuration: lidar-slam-lab's experiment (unicycle robot,
//    its odometry error model, LiDAR at the origin, its default settings)
//    run through Robot Studio's estimator wrapper (src/core/sim/reference.js)
//    for each map and seed. With --lsl <clone of lidar-slam-lab at v0.1.0>
//    the script also runs that clone's `benchmark` command and requires
//    every number of its table to be identical (3 decimals, as printed).
// 2. Full simulation: the mecanum template driven by Robot Studio's own
//    drive, encoder and sensor models on the same maps, routes and seeds,
//    reported next to (1). The numbers differ because the odometry error
//    comes from a different model (simulated wheel slip and wheel radius
//    error instead of lidar-slam-lab's rot1-trans-rot2 noise), the robot is
//    slower (its wheels' limits), and the LiDAR sits 6 cm ahead of the
//    centre at 10 Hz instead of 0.1 s steps.
//
//   node scripts/lsl-crosscheck.mjs [--seeds 5] [--maps office,warehouse,loop] [--lsl ../lidar-slam-lab]
//                                   [--full-seeds N] [--no-full]
//
// Exit code 1 when the reference numbers differ from lidar-slam-lab's.

import { execFileSync } from "node:child_process";
import { join } from "node:path";
import { parseArgs } from "node:util";
import { referenceRun } from "../src/core/sim/reference.js";
import { generate } from "../src/core/builder/templates.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import { simRobot } from "../src/core/sim/robot.js";
import { bundledWorld } from "../src/core/sim/world.js";
import { normalizeSettings } from "../src/core/sim/settings.js";
import { DT, createSim, simErrors, step } from "../src/core/sim/sim.js";
import { createRoute } from "../src/core/sim/routes.js";

const { values } = parseArgs({
  options: {
    seeds: { type: "string", default: "5" },
    maps: { type: "string", default: "office,warehouse,loop" },
    lsl: { type: "string" },
    "no-full": { type: "boolean", default: false },
    "full-seeds": { type: "string" },
  },
});
const seeds = Number(values.seeds);
const fullSeeds = Number(values["full-seeds"] ?? values.seeds);
const maps = /** @type {("office" | "warehouse" | "loop")[]} */ (values.maps.split(","));
const f3 = (/** @type {number} */ v) => (Number.isNaN(v) ? "-" : v.toFixed(3));

/** @type {Map<string, string[]>} */
const ours = new Map();
console.log("1. Reference configuration through Robot Studio's estimator wrapper");
console.log("RMS position error over the demo route, metres (lidar-slam-lab's table layout)\n");
console.log("map         seed  distance  dead-reckoning  scan-matching  MCL-tracking");
for (const id of maps) {
  const cols = [[], [], []];
  for (let seed = 1; seed <= seeds; seed++) {
    const a = referenceRun(id, seed, { scanMatching: false, particles: 300 });
    const b = referenceRun(id, seed, { scanMatching: true, particles: 0 });
    /** @type {number[][]} */ (cols)[0].push(a.odomRms);
    /** @type {number[][]} */ (cols)[1].push(b.estRms);
    /** @type {number[][]} */ (cols)[2].push(a.mclRms);
    const row = `${id.padEnd(10)} ${String(seed).padStart(5)} ${a.distance.toFixed(1).padStart(8)} m ${f3(a.odomRms).padStart(15)} ${f3(b.estRms).padStart(14)} ${f3(a.mclRms).padStart(13)}`;
    ours.set(`${id} ${seed}`, row.trim().split(/\s+/));
    console.log(row);
  }
  const mean = (/** @type {number[]} */ xs) => xs.reduce((p, q) => p + q, 0) / xs.length;
  const row = `${id.padEnd(10)}  mean ${"".padStart(10)} ${f3(mean(cols[0])).padStart(15)} ${f3(mean(cols[1])).padStart(14)} ${f3(mean(cols[2])).padStart(13)}`;
  ours.set(`${id} mean`, row.trim().split(/\s+/));
  console.log(row);
}

let failed = false;
if (values.lsl) {
  const cli = join(values.lsl, "bin", "lidar-slam-lab.js");
  const out = execFileSync(
    process.execPath,
    [cli, "benchmark", "--maps", maps.join(","), "--seeds", String(seeds)],
    { encoding: "utf8" },
  );
  let compared = 0;
  /** @type {string[]} */
  const diffs = [];
  for (const line of out.split("\n")) {
    const cells = line.trim().split(/\s+/);
    if (cells.length < 5 || !maps.includes(/** @type {any} */ (cells[0]))) continue;
    const mine = ours.get(`${cells[0]} ${cells[1]}`);
    compared++;
    if (!mine || mine.join(" ") !== cells.join(" "))
      diffs.push(`lidar-slam-lab: ${line.trim()}\nours:           ${mine?.join(" ")}`);
  }
  console.log(
    `\nlidar-slam-lab's own CLI (${cli}): ${compared} rows compared, ${diffs.length} differ`,
  );
  for (const d of diffs) console.log(d);
  if (diffs.length > 0 || compared === 0) failed = true;
}

if (!values["no-full"]) {
  console.log(
    "\n2. Full simulation: mecanum template, Robot Studio's drive, encoder and LiDAR models",
  );
  console.log("(simulation output; odometry error from simulated slip and wheel radius error)\n");
  console.log(
    "map         seed  distance  time    wheel-odometry  scan-matching  MCL-tracking   step",
  );
  const doc = generate("mecanum", {});
  const r = simRobot(modelFromDoc(doc), doc);
  if (!r.ok) throw new Error(r.reason);
  for (const id of maps) {
    for (let seed = 1; seed <= fullSeeds; seed++) {
      const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (bundledWorld(id));
      const sim = createSim(r.robot, world, normalizeSettings({ seed }));
      const route = createRoute("demo", r.robot, world);
      const t0 = performance.now();
      let done = false;
      while (!done && sim.t < 600) {
        const c = route.next(sim.drive, sim.t);
        done = c.done;
        step(sim, c.twist);
      }
      const ms = (performance.now() - t0) / sim.steps;
      const e = /** @type {NonNullable<ReturnType<typeof simErrors>>} */ (simErrors(sim));
      console.log(
        `${id.padEnd(10)} ${String(seed).padStart(5)} ${sim.distance.toFixed(1).padStart(8)} m ${sim.t.toFixed(0).padStart(5)} s ${f3(e.odomRms).padStart(15)} ${f3(e.estRms).padStart(14)} ${f3(e.mclRms).padStart(13)} ${ms.toFixed(2).padStart(6)} ms${done ? "" : "  (route not finished)"}`,
      );
    }
  }
  console.log(`\nstep = measured wall time per ${DT * 1000} ms simulation step on this machine`);
}
process.exit(failed ? 1 : 0);
