#!/usr/bin/env node
// The round trip simulation -> MCAP bag (robot-studio bag) -> recording
// (robot-studio-bag, the Python companion over rosbag-inspect) -> replay:
// compares every stream of the original recording with the imported one,
// value by value, then replays both through the estimators and compares the
// results. Prints the numbers; exits 1 if anything differs by more than the
// precision the recording files are written with.
//
//   node scripts/bag-roundtrip.mjs --original run/ --imported imported/ [--json out.json]

import { parseArgs } from "node:util";
import { existsSync, readFileSync, writeFileSync } from "node:fs";
import { RECORDING_FILES } from "../src/roscli.js";
import { parseRecording } from "../src/core/ros/recording.js";
import { ReplaySession } from "../src/core/ros/replay.js";

const { values } = parseArgs({
  options: {
    original: { type: "string" },
    imported: { type: "string" },
    json: { type: "string" },
  },
});
if (!values.original || !values.imported) {
  console.error(
    "usage: node scripts/bag-roundtrip.mjs --original DIR --imported DIR [--json FILE]",
  );
  process.exit(3);
}

/** @param {string} dir */
function load(dir) {
  /** @type {Record<string, string>} */
  const files = {};
  for (const n of RECORDING_FILES)
    if (existsSync(`${dir}/${n}`)) files[n] = readFileSync(`${dir}/${n}`, "utf8");
  return parseRecording(files);
}
const a = load(values.original);
const b = load(values.imported);

/** Largest error each file's text format allows (half a unit of its last decimal). */
const TOL = 1e-9;
/** @type {{ file: string, column: string, rows: number, maxDiff: number | null, note?: string }[]} */
const rows = [];
let failed = false;

/**
 * @param {string} file
 * @param {import("../src/core/ros/recording.js").Table | null} ta
 * @param {import("../src/core/ros/recording.js").Table | null} tb
 * @param {Record<string, string>} [expectMissing] column -> why it is not in the bag
 */
function compareTable(file, ta, tb, expectMissing = {}) {
  if (!ta || !tb) {
    rows.push({
      file,
      column: "*",
      rows: 0,
      maxDiff: null,
      note: `missing in the ${ta ? "imported" : "original"} recording`,
    });
    failed = true;
    return;
  }
  if (ta.rows !== tb.rows) {
    rows.push({
      file,
      column: "*",
      rows: ta.rows,
      maxDiff: null,
      note: `${ta.rows} rows vs ${tb.rows}`,
    });
    failed = true;
    return;
  }
  for (const c of ta.columns) {
    if (c in expectMissing) {
      rows.push({ file, column: c, rows: ta.rows, maxDiff: null, note: expectMissing[c] });
      continue;
    }
    const va = ta.col[c];
    const vb = tb.col[c];
    if (!vb) {
      rows.push({
        file,
        column: c,
        rows: ta.rows,
        maxDiff: null,
        note: "not in the imported recording",
      });
      failed = true;
      continue;
    }
    let max = 0;
    let nanMismatch = 0;
    for (let i = 0; i < ta.rows; i++) {
      const x = va[i];
      const y = vb[i];
      if (Number.isNaN(x) || Number.isNaN(y)) {
        if (Number.isNaN(x) !== Number.isNaN(y)) nanMismatch++;
        continue;
      }
      max = Math.max(max, Math.abs(x - y));
    }
    const row = { file, column: c, rows: ta.rows, maxDiff: max };
    if (nanMismatch) Object.assign(row, { note: `${nanMismatch} value(s) present in only one` });
    if (nanMismatch || max > TOL) failed = true;
    rows.push(row);
  }
}

const mcl = "MCL runs in the studio, it is not a ROS topic in the bag";
compareTable("trajectory.csv", a.trajectory, b.trajectory, {
  mcl_x: mcl,
  mcl_y: mcl,
  mcl_theta: mcl,
});
compareTable("imu.csv", a.imu, b.imu);
compareTable("odometry.csv", a.odometry, b.odometry);
compareTable("wheel_state.csv", a.wheelState, b.wheelState);
compareTable("joint_states.csv", a.jointStates, b.jointStates);

// Scans: same count, beams, no-return pattern and ranges.
let scanMax = 0;
let scanBeams = 0;
let noReturn = 0;
let patternMismatch = 0;
const sa = a.scans;
const sb = b.scans;
if (!sa || !sb || sa.items.length !== sb.items.length) {
  failed = true;
  rows.push({
    file: "scans.jsonl",
    column: "*",
    rows: sa?.items.length ?? 0,
    maxDiff: null,
    note: "scan count differs",
  });
} else {
  for (let k = 0; k < sa.items.length; k++) {
    const ra = sa.items[k].ranges;
    const rb = sb.items[k].ranges;
    if (ra.length !== rb.length || Math.abs(sa.items[k].t - sb.items[k].t) > TOL) {
      patternMismatch++;
      continue;
    }
    for (let i = 0; i < ra.length; i++) {
      scanBeams++;
      if (Number.isNaN(ra[i]) || Number.isNaN(rb[i])) {
        if (Number.isNaN(ra[i]) !== Number.isNaN(rb[i])) patternMismatch++;
        else noReturn++;
        continue;
      }
      scanMax = Math.max(scanMax, Math.abs(ra[i] - rb[i]));
    }
  }
  if (patternMismatch || scanMax > 5e-5) failed = true;
  rows.push({
    file: "scans.jsonl",
    column: "ranges",
    rows: sa.items.length,
    maxDiff: scanMax,
    note: `${scanBeams} beams, ${noReturn} no-return in both, ${patternMismatch} mismatches`,
  });
  const ha = sa.header;
  const hb = sb.header;
  for (const k of /** @type {const} */ ([
    "angle_min",
    "angle_increment",
    "range_min",
    "range_max",
    "scan_time",
  ])) {
    const d = Math.abs(ha[k] - hb[k]);
    // float32 in the bag
    if (d > 1e-7 * Math.max(1, Math.abs(ha[k]))) failed = true;
    rows.push({ file: "scans.jsonl", column: `header.${k}`, rows: 1, maxDiff: d });
  }
  const ma = ha.mount;
  const mb = hb.mount;
  const md =
    ma && mb
      ? Math.max(
          ...["x", "y", "z", "yaw"].map((k) =>
            Math.abs(/** @type {any} */ (ma)[k] - /** @type {any} */ (mb)[k]),
          ),
        )
      : NaN;
  if (!(md <= 1e-6)) failed = true;
  rows.push({
    file: "scans.jsonl",
    column: "header.mount (from /tf_static)",
    rows: 1,
    maxDiff: Number.isFinite(md) ? md : null,
  });
}

// Replay both through the estimators.
const t0 = performance.now();
const ra = new ReplaySession(a);
ra.runToEnd();
const t1 = performance.now();
const rb = new ReplaySession(b);
rb.runToEnd();
const t2 = performance.now();
const resA = ra.results();
const resB = rb.results();
const same = JSON.stringify({ ...resA, notes: [] }) === JSON.stringify({ ...resB, notes: [] });
if (!same) failed = true;

const fmt = (/** @type {number | null} */ v) =>
  v === null ? "-" : v === 0 ? "0" : v.toExponential(2);
console.log(
  "ROUND TRIP of SIMULATED data: sim recording -> MCAP bag -> robot-studio-bag import -> replay",
);
console.log("file              column                          rows   max |diff|  note");
for (const r of rows)
  console.log(
    `${r.file.padEnd(17)} ${r.column.padEnd(31)} ${String(r.rows).padStart(5)}   ${fmt(r.maxDiff).padStart(9)}  ${r.note ?? ""}`,
  );
console.log(
  `replay    original: ${resA.scans} scans, ${resA.seconds} s; imported: ${resB.scans} scans, ${resB.seconds} s`,
);
console.log(`replay    estimator results identical: ${same ? "yes" : "NO"}`);
if (resA.errors_m)
  console.log(
    `replay    errors vs the simulated truth (both): wheel odometry RMS ${resA.errors_m.wheel_odometry.rms} m, scan matching RMS ${resA.errors_m.scan_matching.rms} m${resA.errors_m.mcl ? `, MCL RMS ${resA.errors_m.mcl.rms} m` : ""}`,
  );
console.log(
  `timing    replay ${(t1 - t0).toFixed(0)} ms (original), ${(t2 - t1).toFixed(0)} ms (imported) on this machine`,
);
console.log(
  failed ? "RESULT    MISMATCH" : "RESULT    every compared value equal within the file precision",
);
if (values.json)
  writeFileSync(
    values.json,
    JSON.stringify(
      { rows, replay: { original: resA, imported: resB, identical: same }, failed },
      null,
      2,
    ),
  );
process.exit(failed ? 1 : 0);
