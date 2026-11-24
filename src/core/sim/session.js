// A simulation session for the page: the robot read from the open model,
// real time turned into fixed simulation steps, a manual (teleop) command
// that overrides the route while it is held, step-time measurement, and
// the short histories the live plots draw. DOM-free so it is tested in
// Node; src/ui/simui.js puts it on the page.

import { simRobot } from "./robot.js";
import { createSim, step, simErrors, estimates, jointMap, DT } from "./sim.js";
import { createRoute } from "./routes.js";
import { recordingFiles } from "./record.js";

/** @typedef {import("./sim.js").Sim} Sim */
/** @typedef {import("./firmware.js").Twist} Twist */
/** @typedef {import("./world.js").SimWorld} SimWorld */
/** @typedef {import("./settings.js").SimSettings} SimSettings */
/** @typedef {import("../model.js").LoadedModel} LoadedModel */
/** @typedef {import("../builder/document.js").RobotDoc} RobotDoc */

/** Seconds of history the plots keep. */
export const HISTORY_SECONDS = 20;
/** At most this much simulated time per advance() call (a stalled tab does not run away). */
export const MAX_ADVANCE = 0.25;
/** The page records the first 10 simulated minutes of a run (memory). */
export const MAX_RECORDING = 600;

/**
 * Fixed-size ring of samples: time plus a fixed number of values.
 */
export class History {
  /**
   * @param {number} width values per sample
   * @param {number} capacity samples
   */
  constructor(width, capacity) {
    this.width = width;
    this.capacity = capacity;
    this.t = new Float64Array(capacity);
    this.v = new Float64Array(capacity * width);
    this.start = 0;
    this.length = 0;
  }
  /**
   * @param {number} t
   * @param {ArrayLike<number>} values
   */
  push(t, values) {
    const i = (this.start + this.length) % this.capacity;
    this.t[i] = t;
    for (let k = 0; k < this.width; k++) this.v[i * this.width + k] = values[k] ?? NaN;
    if (this.length < this.capacity) this.length++;
    else this.start = (this.start + 1) % this.capacity;
  }
  /**
   * The n-th oldest sample.
   * @param {number} n
   */
  at(n) {
    const i = (this.start + n) % this.capacity;
    return { t: this.t[i], values: this.v.subarray(i * this.width, (i + 1) * this.width) };
  }
  clear() {
    this.start = 0;
    this.length = 0;
  }
}

export class SimSession {
  /**
   * @param {LoadedModel} model
   * @param {RobotDoc | null} doc
   * @param {SimWorld} world
   * @param {string} routeId
   * @param {SimSettings} settings
   * @param {{ clock?: () => number, log?: boolean }} [opts] clock in ms for the step time
   */
  constructor(model, doc, world, routeId, settings, opts = {}) {
    const r = simRobot(model, doc);
    if (!r.ok) throw new Error(r.reason);
    this.robot = r.robot;
    this.robotNotes = r.notes;
    this.world = world;
    this.routeId = routeId;
    this.settings = settings;
    this.clock = opts.clock ?? (() => 0);
    this.log = opts.log ?? true;
    /** @type {Twist | null} */
    this.manual = null;
    /** Mean wall time per simulation step, ms (exponential average). */
    this.stepMs = 0;
    this.accumulator = 0;
    const n = Math.round(HISTORY_SECONDS / 0.05);
    this.history = {
      wheels: new History(8, n), // target then measured, FL FR RL RR (firmware telemetry)
      twist: new History(6, n), // commanded vx vy wz, actual vx vy wz
      imu: new History(6, n),
      errors: new History(3, n), // wheel odometry, scan matching, MCL
    };
    /** @type {Sim} */
    this.sim = createSim(this.robot, world, settings, { log: this.log });
    this.route = createRoute(routeId, this.robot, world);
    this.routeDone = false;
    this.lastSample = -Infinity;
    /** @type {Record<string, string | Uint8Array> | null} recording frozen at MAX_RECORDING */
    this.frozen = null;
    this.sample();
  }

  /** Start over with the same robot, world and settings. */
  reset() {
    this.sim = createSim(this.robot, this.world, this.settings, { log: this.log });
    this.route = createRoute(this.routeId, this.robot, this.world);
    this.routeDone = false;
    this.accumulator = 0;
    this.lastSample = -Infinity;
    this.frozen = null;
    for (const h of Object.values(this.history)) h.clear();
    this.sample();
  }

  /** The command for the next step: the held teleop command, else the route's. */
  command() {
    const r = this.route.next(this.sim.drive, this.sim.t);
    this.routeDone = r.done;
    return this.manual ?? r.twist;
  }

  /**
   * Run as many fixed steps as `seconds` of simulated time holds.
   * @param {number} seconds
   * @returns {number} steps run
   */
  advance(seconds) {
    this.accumulator = Math.min(this.accumulator + Math.max(0, seconds), MAX_ADVANCE);
    let n = 0;
    const t0 = this.clock();
    while (this.accumulator >= DT - 1e-12) {
      step(this.sim, this.command());
      this.accumulator -= DT;
      n++;
      if (this.sim.t - this.lastSample >= 0.05 - 1e-9) this.sample();
      if (this.sim.log && this.sim.t >= MAX_RECORDING - 1e-9) {
        this.frozen = this.files("robot");
        this.sim.log = null;
      }
    }
    if (n > 0) {
      const ms = (this.clock() - t0) / n;
      this.stepMs = this.stepMs === 0 ? ms : 0.9 * this.stepMs + 0.1 * ms;
    }
    return n;
  }

  /** Record one sample of every plotted quantity. */
  sample() {
    const s = this.sim;
    this.lastSample = s.t;
    const w = s.controller.wheels;
    this.history.wheels.push(s.t, [...w.map((x) => x.target), ...w.map((x) => x.velocity)]);
    const c = s.command;
    const a = s.actual;
    this.history.twist.push(s.t, [c.vx, c.vy, c.wz, a.vx, a.vy, a.wz]);
    const imu = s.lastImu;
    this.history.imu.push(s.t, imu ? [...imu.accel, ...imu.gyro] : [NaN, NaN, NaN, NaN, NaN, NaN]);
    const e = simErrors(s);
    this.history.errors.push(s.t, e ? [e.odom, e.est, e.mcl] : [NaN, NaN, NaN]);
  }

  /** What the view needs this frame. */
  snapshot() {
    const s = this.sim;
    return {
      t: s.t,
      truth: s.truth,
      q: jointMap(s),
      estimates: estimates(s),
      errors: simErrors(s),
      scan: s.lastScan,
      scanPose: s.lastScanPose,
    };
  }

  /**
   * The recording files of the run so far (empty without logging).
   * @param {string} input
   */
  files(input) {
    if (this.frozen) return this.frozen;
    return this.sim.log
      ? recordingFiles(this.sim, { route: this.routeId, input, worldId: this.world.id })
      : {};
  }
}
