// Reading a recording (the files `robot-studio sim --out` and "Download
// recording" write, and what the bag importer produces; layout in
// docs/simulation.md) back into numbers: each CSV as named columns, the
// scans with their header, summary.json and world.json. The input is
// untrusted (a file the user opens): sizes are bounded, every number is
// parsed with Number() and checked, nothing is evaluated.

import { parseSimWorld } from "../sim/world.js";

/** @typedef {import("../sim/world.js").SimWorld} SimWorld */

/** Largest recording file read (bytes of text). */
export const MAX_FILE_BYTES = 256 * 1024 * 1024;
/** Most beams accepted in one scan. */
export const MAX_BEAMS = 100000;

/**
 * @typedef {object} Table
 * @property {string[]} columns
 * @property {Record<string, Float64Array>} col
 * @property {number} rows
 */

/**
 * @typedef {object} ScanHeader
 * @property {string} frame_id
 * @property {number} angle_min
 * @property {number} angle_increment
 * @property {number} range_min
 * @property {number} range_max
 * @property {number} scan_time
 * @property {{ x: number, y: number, z: number, yaw: number } | null} mount sensor frame in the base frame
 */

/**
 * @typedef {object} Recording
 * @property {Record<string, any> | null} summary
 * @property {SimWorld | null} world
 * @property {Table | null} trajectory
 * @property {{ header: ScanHeader, items: { t: number, ranges: Float64Array }[] } | null} scans ranges: NaN = no return
 * @property {Table | null} imu
 * @property {Table | null} wheelState
 * @property {Table | null} odometry
 * @property {Table | null} jointStates
 * @property {boolean} simulated the data is simulator output (summary or source says so)
 * @property {string[]} notes
 */

export class RecordingError extends Error {}

const dec = new TextDecoder("utf-8", { fatal: false });

/** @param {string | Uint8Array} v */
function text(v) {
  const n = typeof v === "string" ? v.length : v.byteLength;
  if (n > MAX_FILE_BYTES) throw new RecordingError(`file larger than ${MAX_FILE_BYTES} bytes`);
  return typeof v === "string" ? v : dec.decode(v);
}

/**
 * A number from a CSV cell: "nan" / "" -> NaN; anything else must parse.
 * @param {string} cell
 * @param {string} where
 */
function cellNumber(cell, where) {
  const c = cell.trim();
  if (c === "" || c === "nan" || c === "NaN") return NaN;
  if (c === "inf" || c === "Infinity") return Infinity;
  if (c === "-inf" || c === "-Infinity") return -Infinity;
  const v = Number(c);
  if (Number.isNaN(v)) throw new RecordingError(`${where}: not a number: ${c.slice(0, 40)}`);
  return v;
}

/**
 * Parse a numeric CSV with a header line.
 * @param {string | Uint8Array} data
 * @param {string} name file name for errors
 * @param {string[]} [required] columns that must be present
 * @returns {Table}
 */
export function parseCsv(data, name, required = []) {
  const lines = text(data).split(/\r?\n/);
  while (lines.length && lines[lines.length - 1] === "") lines.pop();
  if (lines.length === 0) throw new RecordingError(`${name}: empty`);
  const columns = lines[0].split(",").map((c) => c.trim());
  if (new Set(columns).size !== columns.length)
    throw new RecordingError(`${name}: repeated column names`);
  for (const r of required)
    if (!columns.includes(r)) throw new RecordingError(`${name}: no column ${r}`);
  const rows = lines.length - 1;
  /** @type {Record<string, Float64Array>} */
  const col = {};
  for (const c of columns) col[c] = new Float64Array(rows);
  for (let i = 0; i < rows; i++) {
    const cells = lines[i + 1].split(",");
    if (cells.length !== columns.length)
      throw new RecordingError(
        `${name} line ${i + 2}: ${cells.length} values for ${columns.length} columns`,
      );
    for (let k = 0; k < columns.length; k++)
      col[columns[k]][i] = cellNumber(cells[k], `${name} line ${i + 2}`);
  }
  return { columns, col, rows };
}

/**
 * @param {unknown} v
 * @param {string} what
 */
function finite(v, what) {
  if (typeof v !== "number" || !Number.isFinite(v))
    throw new RecordingError(`scans.jsonl header: ${what} must be a number`);
  return v;
}

/**
 * scans.jsonl: a header line, then one {"t", "ranges"} object per scan.
 * @param {string | Uint8Array} data
 */
export function parseScans(data) {
  const lines = text(data)
    .split(/\r?\n/)
    .filter((l) => l.trim() !== "");
  if (lines.length === 0) throw new RecordingError("scans.jsonl: empty");
  /** @type {any} */
  let head;
  try {
    head = JSON.parse(lines[0]).header;
  } catch {
    throw new RecordingError("scans.jsonl line 1: not JSON");
  }
  if (!head || typeof head !== "object") throw new RecordingError("scans.jsonl: no header");
  const m = head.mount;
  /** @type {ScanHeader} */
  const header = {
    frame_id: typeof head.frame_id === "string" ? head.frame_id : "laser",
    angle_min: finite(head.angle_min, "angle_min"),
    angle_increment: finite(head.angle_increment, "angle_increment"),
    range_min: finite(head.range_min, "range_min"),
    range_max: finite(head.range_max, "range_max"),
    scan_time: finite(head.scan_time ?? 0, "scan_time"),
    mount:
      m && typeof m === "object"
        ? {
            x: finite(m.x, "mount.x"),
            y: finite(m.y, "mount.y"),
            z: finite(m.z ?? 0, "mount.z"),
            yaw: finite(m.yaw, "mount.yaw"),
          }
        : null,
  };
  /** @type {{ t: number, ranges: Float64Array }[]} */
  const items = [];
  for (let i = 1; i < lines.length; i++) {
    /** @type {any} */
    let o;
    try {
      o = JSON.parse(lines[i]);
    } catch {
      throw new RecordingError(`scans.jsonl line ${i + 1}: not JSON`);
    }
    if (!o || typeof o.t !== "number" || !Array.isArray(o.ranges))
      throw new RecordingError(`scans.jsonl line ${i + 1}: needs "t" and "ranges"`);
    if (o.ranges.length > MAX_BEAMS)
      throw new RecordingError(`scans.jsonl line ${i + 1}: more than ${MAX_BEAMS} beams`);
    const ranges = new Float64Array(o.ranges.length);
    for (let k = 0; k < ranges.length; k++) {
      const v = o.ranges[k];
      // null = no return; so is anything that is not a finite range inside the limits.
      ranges[k] =
        typeof v === "number" &&
        Number.isFinite(v) &&
        v >= header.range_min &&
        v <= header.range_max
          ? v
          : NaN;
    }
    items.push({ t: o.t, ranges });
  }
  return { header, items };
}

/**
 * A recording from its files (names as in the recording; a leading folder
 * such as "simulation/" is ignored).
 * @param {Record<string, string | Uint8Array>} files
 * @returns {Recording}
 */
export function parseRecording(files) {
  /** @type {Record<string, string | Uint8Array>} */
  const f = {};
  for (const [k, v] of Object.entries(files)) f[k.split("/").pop() ?? k] = v;
  /** @type {string[]} */
  const notes = [];
  /** @type {Record<string, any> | null} */
  let summary = null;
  if (f["summary.json"] !== undefined) {
    try {
      const s = JSON.parse(text(f["summary.json"]));
      summary = s && typeof s === "object" && !Array.isArray(s) ? s : null;
    } catch {
      notes.push("summary.json could not be read");
    }
  }
  /** @type {SimWorld | null} */
  let world = null;
  if (f["world.json"] !== undefined) {
    try {
      world = parseSimWorld(text(f["world.json"]));
    } catch (e) {
      notes.push(`world.json: ${e instanceof Error ? e.message : String(e)}`);
    }
  }
  /**
   * @param {string} name
   * @param {string[]} req
   */
  const table = (name, req) => (f[name] === undefined ? null : parseCsv(f[name], name, req));
  const rec = {
    summary,
    world,
    trajectory: table("trajectory.csv", ["t", "odom_x", "odom_y", "odom_theta"]),
    scans: f["scans.jsonl"] === undefined ? null : parseScans(f["scans.jsonl"]),
    imu: table("imu.csv", ["t", "ax", "ay", "az", "gx", "gy", "gz"]),
    wheelState: table("wheel_state.csv", ["t_us"]),
    odometry: table("odometry.csv", ["t_s", "x", "y", "theta", "vx", "vy", "wz"]),
    jointStates: table("joint_states.csv", ["t"]),
    simulated: isSimulated(summary),
    notes,
  };
  if (!rec.trajectory && !rec.odometry)
    throw new RecordingError("not a recording: no trajectory.csv or odometry.csv");
  return rec;
}

/**
 * Whether a summary says the data came from a simulator: Robot Studio's own
 * runs, or an imported bag whose source topic says so.
 * @param {Record<string, any> | null} summary
 */
export function isSimulated(summary) {
  if (!summary) return false;
  if (summary.source && typeof summary.source === "object")
    return summary.source.simulated === true;
  return typeof summary.format === "string" && summary.format.startsWith("robot-studio-sim");
}
