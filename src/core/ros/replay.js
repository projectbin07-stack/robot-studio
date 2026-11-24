// Replaying a recording (a simulated run, or a ROS 2 bag turned into the
// recording format by the Python companion) through the estimators and the
// views: the streams are merged in time order and fed to a Track, which
// runs scan matching on the recorded wheel odometry (and MCL when the
// recording has a world), keeps the plot histories and produces the
// ViewFrame. DOM-free; the page and `robot-studio replay` both use it.

import { Track } from "./track.js";
import { segmentsAbove } from "../sim/world.js";

/** @typedef {import("./recording.js").Recording} Recording */
/** @typedef {import("./recording.js").Table} Table */
/** @typedef {import("../sim/se2.js").Pose2} Pose2 */

const WHEELS = ["fl", "fr", "rl", "rr"];

/**
 * @typedef {object} ReplayEvent
 * @property {number} t
 * @property {number} order tie-break: odometry before the scan at the same time
 * @property {() => void} run
 */

export class ReplaySession {
  /**
   * @param {Recording} rec
   * @param {{ wheelJoints?: string[], estimate?: boolean, scanMatching?: boolean, particles?: number }} [opts]
   *   wheelJoints: joint names in FL FR RL RR order, to plot joint_states when there is no wheel telemetry
   */
  constructor(rec, opts = {}) {
    this.rec = rec;
    const s = rec.summary ?? {};
    const h = rec.scans?.header ?? null;
    const m = h?.mount ?? null;
    const lidarHeight = Number(s.sensors?.lidar?.height) || (m ? m.z : 0.2);
    const world = rec.world;
    const settings = s.settings_changed?.estimators ?? {};
    const mclWanted = (opts.particles ?? settings.particles ?? 300) > 0;
    this.track = new Track({
      mount: m ? { x: m.x, y: m.y, theta: m.yaw } : { x: 0, y: 0, theta: 0 },
      lidarHeight,
      start: world ? { ...world.start } : null,
      area: world ? { width: world.width, height: world.height } : null,
      knownWalls: world && mclWanted ? segmentsAbove(world, lidarHeight) : [],
      estimators: {
        seed: typeof s.seed === "number" ? s.seed : 1,
        scanMatching: opts.scanMatching ?? settings.scanMatching ?? true,
        particles: opts.particles ?? settings.particles,
      },
      estimate: opts.estimate,
    });
    if (!m && rec.scans)
      this.track.notes.push("the scans have no mount pose: the LiDAR is taken to be at the base");
    /** @type {ReplayEvent[]} */
    this.events = buildEvents(rec, this.track, opts.wheelJoints ?? []);
    // A simulated run starts at the world's start pose, where its first scan is taken.
    if (world && this.events.length) {
      const t0 = this.events[0].t;
      const start = { ...world.start };
      this.events.unshift({ t: t0, order: -1, run: () => this.track.odometry(t0, start) });
    }
    this.next = 0;
    this.start = this.events.length ? this.events[0].t : 0;
    this.end = this.events.length ? this.events[this.events.length - 1].t : 0;
    this.time = this.start;
    this.accumulator = 0;
    /** Mean wall time per replayed second (not used for results). */
    this.stepMs = 0;
  }

  get done() {
    return this.next >= this.events.length;
  }

  get history() {
    return this.track.history;
  }

  /**
   * Replay up to `seconds` more of recorded time.
   * @param {number} seconds
   * @returns {number} events run
   */
  advance(seconds) {
    this.time = Math.min(this.end, this.time + Math.max(0, seconds));
    let n = 0;
    while (this.next < this.events.length && this.events[this.next].t <= this.time + 1e-9) {
      this.events[this.next++].run();
      n++;
    }
    return n;
  }

  /** Replay everything that is left. */
  runToEnd() {
    this.time = this.end;
    return this.advance(0);
  }

  frame() {
    return this.track.frame();
  }

  /**
   * What the estimators did over the replay so far.
   */
  results() {
    const e = this.track.est;
    const err = this.track.errors();
    const r = (/** @type {number} */ v, d = 4) =>
      Number.isFinite(v) ? Number(v.toFixed(d)) : null;
    const pose = (/** @type {Pose2 | null} */ p) =>
      p ? { x: r(p.x, 5), y: r(p.y, 5), theta: r(p.theta, 6) } : null;
    return {
      simulated: this.rec.simulated,
      seconds: r(this.time - this.start, 3),
      scans: this.track.scans,
      scans_integrated: e?.scansIntegrated ?? 0,
      scan_match_accept_rate: e && e.matchAttempts ? r(e.matchAccepted / e.matchAttempts) : null,
      final: {
        odometry: pose(this.track.odom),
        scan_matching: pose(e ? e.est : null),
        mcl: pose(e?.mcl ?? null),
        truth: pose(this.track.truth),
      },
      errors_m: err
        ? {
            wheel_odometry: { final: r(err.odom), rms: r(err.odomRms) },
            scan_matching: { final: r(err.est), rms: r(err.estRms) },
            mcl: e?.pf ? { final: r(err.mcl), rms: r(err.mclRms) } : null,
          }
        : null,
      notes: [...this.rec.notes, ...this.track.notes],
    };
  }
}

/**
 * @param {Table} t
 * @param {string} c
 * @param {number} i
 */
const at = (t, c, i) => (t.col[c] ? t.col[c][i] : NaN);

/**
 * The recording's rows as time-ordered events on a track.
 * @param {Recording} rec
 * @param {Track} track
 * @param {string[]} wheelJoints
 * @returns {ReplayEvent[]}
 */
function buildEvents(rec, track, wheelJoints) {
  /** @type {ReplayEvent[]} */
  const ev = [];
  const tr = rec.trajectory;
  if (tr)
    for (let i = 0; i < tr.rows; i++) {
      const t = tr.col.t[i];
      const odom = {
        x: at(tr, "odom_x", i),
        y: at(tr, "odom_y", i),
        theta: at(tr, "odom_theta", i),
      };
      const truth = {
        x: at(tr, "true_x", i),
        y: at(tr, "true_y", i),
        theta: at(tr, "true_theta", i),
      };
      const cmd = { vx: at(tr, "cmd_vx", i), vy: at(tr, "cmd_vy", i), wz: at(tr, "cmd_wz", i) };
      const act = { vx: at(tr, "true_vx", i), vy: at(tr, "true_vy", i), wz: at(tr, "true_wz", i) };
      const ok = (/** @type {Record<string, number>} */ p) =>
        Object.values(p).every(Number.isFinite);
      ev.push({
        t,
        order: 0,
        run: () => {
          if (ok(truth)) track.groundTruth(t, truth);
          if (ok(odom)) track.odometry(t, odom);
          track.twist(t, ok(cmd) ? cmd : null, ok(act) ? act : null);
        },
      });
    }
  else if (rec.odometry) {
    // No trajectory: the firmware's odometry frames are the odometry.
    const od = rec.odometry;
    for (let i = 0; i < od.rows; i++) {
      const t = od.col.t_s[i];
      const p = { x: od.col.x[i], y: od.col.y[i], theta: od.col.theta[i] };
      ev.push({ t, order: 0, run: () => track.odometry(t, p) });
    }
  }
  if (rec.odometry && !hasCols(tr, ["true_vx"])) {
    const od = rec.odometry;
    for (let i = 0; i < od.rows; i++) {
      const t = od.col.t_s[i];
      const v = { vx: od.col.vx[i], vy: od.col.vy[i], wz: od.col.wz[i] };
      ev.push({ t, order: 1, run: () => track.twist(t, null, v) });
    }
  }
  if (rec.scans) {
    const h = rec.scans.header;
    for (const s of rec.scans.items) {
      const n = s.ranges.length;
      const angles = new Float64Array(n);
      for (let k = 0; k < n; k++) angles[k] = h.angle_min + k * h.angle_increment;
      const scan = { angles, ranges: s.ranges, maxRange: h.range_max };
      ev.push({ t: s.t, order: 2, run: () => track.scan(s.t, scan) });
    }
  }
  const im = rec.imu;
  if (im)
    for (let i = 0; i < im.rows; i++) {
      const t = im.col.t[i];
      const a = [im.col.ax[i], im.col.ay[i], im.col.az[i]];
      const g = [im.col.gx[i], im.col.gy[i], im.col.gz[i]];
      ev.push({ t, order: 3, run: () => track.imu(t, a, g) });
    }
  const js = rec.jointStates;
  const ws = rec.wheelState;
  if (js) {
    const names = js.columns.filter((c) => c.startsWith("pos:")).map((c) => c.slice(4));
    for (let i = 0; i < js.rows; i++) {
      const t = js.col.t[i];
      /** @type {Record<string, number>} */
      const q = {};
      for (const n of names) q[n] = js.col[`pos:${n}`][i];
      const vel = wheelJoints.map((n) => at(js, `vel:${n}`, i));
      ev.push({
        t,
        order: 4,
        run: () => {
          track.joints(t, q);
          if (!ws && vel.length)
            track.wheels(t, [NaN, NaN, NaN, NaN], [...vel, NaN, NaN].slice(0, 4));
        },
      });
    }
  }
  if (ws && hasCols(ws, ["velocity_fl", "target_fl"]))
    for (let i = 0; i < ws.rows; i++) {
      const t = ws.col.t_us[i] / 1e6;
      const target = WHEELS.map((w) => at(ws, `target_${w}`, i));
      const vel = WHEELS.map((w) => at(ws, `velocity_${w}`, i));
      ev.push({ t, order: 5, run: () => track.wheels(t, target, vel) });
    }
  ev.sort((a, b) => a.t - b.t || a.order - b.order);
  return ev;
}

/**
 * @param {Table | null} t
 * @param {string[]} cols
 */
const hasCols = (t, cols) => !!t && cols.every((c) => t.columns.includes(c));
