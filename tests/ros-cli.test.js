// `robot-studio sim --bag`, `bag`, `replay` and `compare` through main(),
// on real files in a temporary directory.

import { test } from "node:test";
import assert from "node:assert/strict";
import { existsSync, mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { main } from "../src/cli.js";
import { bagName } from "../src/simcli.js";
import { readMcap } from "./ros-helpers.js";

/** @param {string[]} argv */
async function cli(argv) {
  let out = "";
  let err = "";
  const code = await main(argv, {
    out: (t) => (out += t),
    err: (t) => (err += t),
    writeFile: (p, d) => writeFileSync(p, d),
    exists: (p) => existsSync(p),
    readText: (p) => readFileSync(p, "utf8"),
    mkdir: (p) => mkdirSync(p, { recursive: true }),
    now: () => 0,
  });
  return { code, out, err };
}

test("sim --bag, bag, replay and compare on real files", async () => {
  const dir = mkdtempSync(join(tmpdir(), "rs-ros-"));
  try {
    const run = join(dir, "run");
    const bag1 = join(dir, "b1");
    let r = await cli([
      "sim",
      "mecanum",
      "--world",
      "lab",
      "--route",
      "square",
      "--seconds",
      "6",
      "--out",
      run,
      "--bag",
      bag1,
    ]);
    assert.equal(r.code, 0, r.err);
    assert.match(r.out, /ROS 2 bag: \d+ messages, \d+ bytes of MCAP, labelled simulated/);
    const m1 = readMcap(readFileSync(join(bag1, "b1_0.mcap")));
    assert.equal(m1.profile, "ros2");
    assert.match(readFileSync(join(bag1, "metadata.yaml"), "utf8"), /- b1_0\.mcap/);

    // bag: the same bag from the recording files.
    const bag2 = join(dir, "b2");
    r = await cli(["bag", run, "-o", bag2, "--name", "b1"]);
    assert.equal(r.code, 0, r.err);
    assert.match(r.out, /^SIMULATED data \(labelled so in the bag\)/);
    assert.match(r.out, /\/scan +\d+/);
    assert.deepEqual(readFileSync(join(bag2, "b1_0.mcap")), readFileSync(join(bag1, "b1_0.mcap")));

    // replay
    r = await cli(["replay", run]);
    assert.equal(r.code, 0, r.err);
    assert.match(r.out, /^REPLAY of SIMULATED data/);
    assert.match(r.out, /scan_matching +\d\.\d{3} m +\d\.\d{3} m/);
    r = await cli(["replay", run, "--json", "--particles", "0", "--no-scan-matching"]);
    const res = JSON.parse(r.out);
    assert.equal(res.errors_m.mcl, null);
    assert.deepEqual(res.final.scan_matching, res.final.odometry);
    assert.equal((await cli(["replay", run, "--particles", "-1"])).code, 3);

    // compare: the run's own commands on the same robot.
    const out = join(dir, "again");
    r = await cli(["compare", run, "mecanum", "--out", out]);
    assert.equal(r.code, 0, r.err);
    assert.match(r.out, /^SIMULATION vs a SIMULATED recording \(not a robot\)/);
    assert.match(r.out, /odometry\.x +\d/);
    assert.ok(existsSync(join(out, "odometry.csv")));
    r = await cli([
      "compare",
      run,
      "mecanum",
      "--json",
      "--ideal",
      "--world",
      "empty",
      "--set",
      "drive.slip=false",
    ]);
    const cmp = JSON.parse(r.out);
    assert.ok(cmp.fields.length >= 6);
    assert.equal((await cli(["compare", run, "nope.urdf"])).code, 2);
    assert.match((await cli(["compare", run, "mecanum", "--set", "x"])).err, /key=value/);
    assert.match((await cli(["compare", run, "mecanum", "--set", "a.b=z"])).err, /not a number/);
    assert.match((await cli(["compare", run, "mecanum", "--seed", "-2"])).err, /--seed/);
    assert.match(
      (await cli(["compare", run, "mecanum", "--world", "atlantis"])).err,
      /unknown world/,
    );
    writeFileSync(join(dir, "w.json"), readFileSync(join(run, "world.json")));
    assert.equal((await cli(["compare", run, "mecanum", "--world", join(dir, "w.json")])).code, 0);

    // Usage and input errors.
    assert.match(
      (await cli(["bag", "--help"])).out,
      /robot-studio bag <recording-dir> -o <bag-dir>/,
    );
    assert.match((await cli(["replay", "--help"])).out, /robot-studio replay/);
    assert.match((await cli(["compare", "-h"])).out, /command schedule/);
    assert.equal((await cli(["bag", run])).code, 3);
    assert.equal((await cli(["bag"])).code, 3);
    assert.equal((await cli(["bag", run, "--bogus"])).code, 3);
    assert.equal((await cli(["replay", dir])).code, 3);
    writeFileSync(join(dir, "trajectory.csv"), "t,odom_x\n1,2\n");
    assert.match((await cli(["replay", dir])).err, /no column odom_y/);
    const noCmd = join(dir, "nocmd");
    mkdirSync(noCmd);
    writeFileSync(join(noCmd, "odometry.csv"), "t_s,x,y,theta,vx,vy,wz\n0.05,0,0,0,0,0,0\n");
    assert.match((await cli(["compare", noCmd, "mecanum"])).err, /no commanded twist/);
    assert.equal(bagName("a/b/c/"), "c");
    assert.equal(bagName("/"), "bag");
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
});
