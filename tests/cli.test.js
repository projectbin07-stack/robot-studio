// The command-line interface, run in-process through main() and once as a
// real process.

import { test } from "node:test";
import assert from "node:assert/strict";
import { execFileSync, spawnSync } from "node:child_process";
import { existsSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { EXIT, main } from "../src/cli.js";

const root = fileURLToPath(new URL("..", import.meta.url));
const MECANUM = join(root, "examples/studio_mecanum/urdf/studio_mecanum.urdf.xacro");
const TB3 = join(
  root,
  "examples/third_party/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro",
);

/** @param {string[]} argv */
function cli(argv) {
  let out = "";
  let err = "";
  /** @type {Record<string, string>} */
  const written = {};
  const code = main(argv, {
    out: (t) => (out += t),
    err: (t) => (err += t),
    writeFile: (p, d) => (written[p] = d),
    exists: (p) => existsSync(p),
  });
  return { code, out, err, written };
}

test("help, version and usage errors", () => {
  assert.match(cli(["--help"]).out, /robot-studio xacro <in\.xacro>/);
  assert.equal(cli(["--help"]).code, EXIT.ok);
  assert.match(cli(["--version"]).out, /^\d+\.\d+\.\d+\n$/);
  assert.equal(cli([]).code, EXIT.usage);
  assert.equal(cli(["frobnicate", "x"]).code, EXIT.usage);
  assert.match(cli(["frobnicate", "x"]).err, /unknown command 'frobnicate'/);
  assert.equal(cli(["xacro"]).code, EXIT.usage);
  assert.equal(cli(["xacro", MECANUM, "extra"]).code, EXIT.usage);
  assert.equal(cli(["xacro", MECANUM, "--nope"]).code, EXIT.usage);
  assert.equal(cli(["xacro", MECANUM, "--arg", "noequals"]).code, EXIT.usage);
  const missing = cli(["check", join(root, "nope.xacro")]);
  assert.equal(missing.code, EXIT.usage);
  assert.match(missing.err, /cannot read file/);
});

test("xacro expands with --arg and name:= arguments and writes -o", () => {
  const r = cli(["xacro", MECANUM, "--arg", "use_lidar=false", "use_camera:=false"]);
  assert.equal(r.code, EXIT.ok, r.err);
  assert.match(r.out, /^<\?xml version="1.0" \?>/);
  assert.doesNotMatch(r.out, /lidar_link/);
  assert.doesNotMatch(r.out, /camera_link/);
  const full = cli(["xacro", MECANUM]);
  assert.match(full.out, /<link name="lidar_link">/);
  const o = cli(["xacro", MECANUM, "-o", "out.urdf"]);
  assert.match(o.written["out.urdf"], /<robot name="studio_mecanum">/);
  assert.match(o.err, /wrote out\.urdf/);
});

test("xacro errors exit 2 with file and line", () => {
  const tmp = mkdtempSync(join(process.env.TMP ?? tmpdir(), "cli-"));
  try {
    const f = join(tmp, "bad.xacro");
    writeFileSync(
      f,
      '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="b">\n  <link name="${nope}"/>\n</robot>\n',
    );
    const r = cli(["xacro", f]);
    assert.equal(r.code, EXIT.errors);
    assert.match(r.err, /bad\.xacro:2: NameError: name 'nope' is not defined/);
    const c = cli(["check", f]);
    assert.equal(c.code, EXIT.errors);
    assert.match(c.out, /ERROR xacro-error/);
    const w = join(tmp, "warn.xacro");
    writeFileSync(
      w,
      '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="w"><xacro:property name="p" value="1" odd="2"/><link name="a"/></robot>',
    );
    const wr = cli(["xacro", w]);
    assert.equal(wr.code, EXIT.ok);
    assert.match(wr.err, /warning: xacro:property: unknown attribute\(s\): odd/);
  } finally {
    rmSync(tmp, { recursive: true, force: true });
  }
});

test("check reports findings with files, honours --strict and --json, finds packages", () => {
  const r = cli(["check", MECANUM]);
  assert.equal(r.code, EXIT.ok, r.out);
  assert.match(r.out, /robot 'studio_mecanum': 11 links, 10 joints/);
  assert.match(r.out, /meshes: 2 of 2 found/);
  assert.match(r.out, /0 error\(s\), 0 warning\(s\), 1 note\(s\)/);
  assert.match(r.out, /INFO  wheel-base-summary  \(.*mecanum_wheel\.xacro:\d+\)/);
  const tb = cli(["check", TB3, "--strict"]);
  assert.equal(tb.code, EXIT.warnings);
  const j = cli(["check", TB3, "--json"]);
  const report = JSON.parse(j.out);
  assert.equal(report.tool, "robot-studio");
  assert.equal(report.xacro, true);
  assert.equal(report.files.length, 3);
  assert.equal(report.summary.links, 7);
  assert.equal(report.meshes.length, 4);
  assert.ok(
    report.findings.every((/** @type {{ file: string }} */ f) => typeof f.file === "string"),
  );
  // Without discovery of its package, $(find) fails: give it explicitly.
  const plain = cli([
    "check",
    join(root, "tests/corpus/features/includes.xacro"),
    "--package",
    `demo_parts=${join(root, "tests/corpus/features/pkgs/demo_parts")}`,
  ]);
  assert.match(plain.out, /mesh-missing/);
  assert.equal(plain.code, EXIT.errors, "the feature fixture is not a complete robot");
});

test("the installed bin runs as a process with the same exit codes", () => {
  const bin = join(root, "bin/robot-studio.js");
  const out = execFileSync(process.execPath, [bin, "xacro", MECANUM, "use_camera:=false"], {
    encoding: "utf8",
  });
  assert.match(out, /<robot name="studio_mecanum">/);
  const bad = spawnSync(process.execPath, [bin, "check", join(root, "missing.urdf")], {
    encoding: "utf8",
  });
  assert.equal(bad.status, EXIT.usage);
  const cmp = spawnSync(process.execPath, [bin, "xacro", TB3], { encoding: "utf8" });
  const golden = readFileSync(join(root, "tests/corpus/golden/turtlebot3-burger.urdf"), "utf8");
  assert.equal(cmp.stdout, golden, "the CLI prints exactly what the official xacro prints");
});
