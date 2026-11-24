#!/usr/bin/env node
// Write the builder's exported files for a fixed set of template cases to
// tests/corpus/builder/: the parametrised xacro, the URDF and (for four-wheel
// bases) the mecanum-kinematics TOML plus reference wheel speeds computed by
// Robot Studio. The xacro files are golden-corpus cases (the official xacro
// expands them in CI; tests compare that with the exported URDF), and the
// TOML files are loaded by mecanum-kinematics in CI
// (scripts/mecanum_interop.py).
//
//   node scripts/builder-fixtures.js           write the files
//   node scripts/builder-fixtures.js --check   fail if any file would change

import { readFileSync, writeFileSync, existsSync, mkdirSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { BUILDER_CASES, builderCaseFiles } from "../tests/builder-cases.js";

const ROOT = fileURLToPath(new URL("..", import.meta.url));
const DIR = `${ROOT}tests/corpus/builder`;
const check = process.argv.includes("--check");

mkdirSync(DIR, { recursive: true });
/** @type {string[]} */
const changed = [];
for (const c of BUILDER_CASES) {
  for (const [name, text] of Object.entries(builderCaseFiles(c))) {
    const path = `${DIR}/${name}`;
    const old = existsSync(path) ? readFileSync(path, "utf8") : null;
    if (old === text) continue;
    changed.push(name);
    if (!check) writeFileSync(path, text);
  }
}
if (check && changed.length > 0) {
  console.error(
    `builder fixtures are out of date: ${changed.join(", ")}\nrun: node scripts/builder-fixtures.js`,
  );
  process.exit(1);
}
console.log(
  check
    ? `builder fixtures up to date (${BUILDER_CASES.length} cases)`
    : `wrote ${changed.length} file(s)`,
);
