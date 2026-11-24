// What the robot's wheel controller computes, mirrored from
// encoder-odometry-firmware (lib/odocore and its pure-Python model
// host/src/odobridge/firmware_model.py at v0.1.0) so the simulated drive
// reports the same quantities, in the same units and fields, as that
// firmware: 4x encoder counts, measured wheel speed from the blended
// fixed-period / period (1/T) estimator, wheel speed targets from inverse
// kinematics with direction-preserving saturation, per-wheel PI(D) with
// feed-forward, anti-windup and slew limit, a command watchdog, and SE(2)
// wheel odometry from encoder increments.
//
// Like the firmware's own Python model this uses double precision where
// the firmware uses float32, so it agrees with the firmware to float
// rounding, not bit for bit. CI runs the Python model on the same commands
// and compares (scripts/firmware_interop.py).

import { compose, se2Exp, wrapAngle } from "./se2.js";

/** @typedef {import("./se2.js").Pose2} Pose2 */
/** @typedef {{ vx: number, vy: number, wz: number }} Twist */
/** @typedef {[number, number, number, number]} Wheels4 FL, FR, RL, RR */

export const MECANUM = "mecanum";
export const DIFFERENTIAL = "differential";

/**
 * The firmware's drive geometry (parameters 11 to 14 and 16).
 * @typedef {object} FwGeometry
 * @property {number} wheelRadius m
 * @property {number} halfLength m, lx (mecanum only)
 * @property {number} halfWidth m, ly, or half the wheel separation (differential)
 * @property {number} rollerAngleDeg
 * @property {"mecanum" | "differential"} drive
 */

/**
 * @typedef {object} PidConfig
 * @property {number} kp duty per rad/s
 * @property {number} ki duty per rad
 * @property {number} kd duty per rad/s^2, on the measurement
 * @property {number} kff feed-forward, duty per rad/s
 * @property {number} ks static-friction feed-forward, duty
 * @property {number} outputLimit duty ceiling
 * @property {number} slewRate duty per second, 0 = off
 * @property {number} dCutoffHz derivative low-pass, 0 = off
 */

/** @typedef {{ lowCps: number, highCps: number, zeroTimeoutUs: number }} VelocityConfig */

/**
 * @typedef {object} DriveConfig
 * @property {FwGeometry} geometry
 * @property {number} ticksPerWheelRev 4x counts per wheel revolution
 * @property {number} maxWheelSpeed rad/s
 * @property {PidConfig} pid
 * @property {VelocityConfig} velocity
 * @property {number} watchdogTimeoutMs
 * @property {number} controlRateHz
 * @property {number} telemetryDivider
 */

/** Firmware parameter defaults (docs/protocol.md, parameter table). */
export const FW_DEFAULTS = Object.freeze({
  pid: Object.freeze({
    kp: 0.02,
    ki: 0.2,
    kd: 0,
    kff: 0.03,
    ks: 0,
    outputLimit: 1,
    slewRate: 4,
    dCutoffHz: 20,
  }),
  velocity: Object.freeze({ lowCps: 200, highCps: 800, zeroTimeoutUs: 250000 }),
  ticksPerWheelRev: 38000,
  maxWheelSpeed: 25,
  watchdogTimeoutMs: 500,
  controlRateHz: 100,
  telemetryDivider: 5,
});

const U32 = 0x100000000;

/** Wrap-safe a - b for counts kept modulo 2^32, as int32. @param {number} a @param {number} b */
export const countDelta = (a, b) => (a - b) | 0;
/** Wrap-safe elapsed time on a 32-bit microsecond clock. @param {number} now @param {number} then */
export const elapsedUs = (now, then) => (now - then) >>> 0;
/** @param {number} v any integer -> int32 */
export const toInt32 = (v) => {
  const m = ((v % U32) + U32) % U32;
  return m >= 0x80000000 ? m - U32 : m;
};

/**
 * Python's round(): to the nearest integer, ties to even. The firmware's
 * Python model rounds edge times with it.
 * @param {number} v
 * @returns {number}
 */
export function pyRound(v) {
  const f = Math.floor(v);
  const d = v - f;
  if (d > 0.5) return f + 1;
  if (d < 0.5) return f;
  return f % 2 === 0 ? f : f + 1;
}

/**
 * Wheel speeds (rad/s, FL FR RL RR) for a body twist, as the firmware computes them.
 * @param {FwGeometry} g
 * @param {Twist} v
 * @returns {Wheels4}
 */
export function fwInverse(g, v) {
  const r = g.wheelRadius;
  if (g.drive === DIFFERENTIAL) {
    const left = (v.vx - v.wz * g.halfWidth) / r;
    const right = (v.vx + v.wz * g.halfWidth) / r;
    return [left, right, left, right];
  }
  const t = Math.tan((g.rollerAngleDeg * Math.PI) / 180);
  const k = g.halfWidth + g.halfLength * t;
  return [
    (v.vx - t * v.vy - k * v.wz) / r,
    (v.vx + t * v.vy + k * v.wz) / r,
    (v.vx + t * v.vy - k * v.wz) / r,
    (v.vx - t * v.vy + k * v.wz) / r,
  ];
}

/**
 * Least-squares body twist (or displacement) from four wheel values.
 * @param {FwGeometry} g
 * @param {ArrayLike<number>} w
 * @returns {Twist}
 */
export function fwForward(g, w) {
  const r = g.wheelRadius;
  if (g.drive === DIFFERENTIAL) {
    const left = 0.5 * (w[0] + w[2]);
    const right = 0.5 * (w[1] + w[3]);
    return { vx: 0.5 * r * (left + right), vy: 0, wz: (r * (right - left)) / (2 * g.halfWidth) };
  }
  const t = Math.tan((g.rollerAngleDeg * Math.PI) / 180);
  const k = g.halfWidth + g.halfLength * t;
  return {
    vx: 0.25 * r * (w[0] + w[1] + w[2] + w[3]),
    vy: ((0.25 * r) / t) * (-w[0] + w[1] + w[2] - w[3]),
    wz: ((0.25 * r) / k) * (-w[0] + w[1] - w[2] + w[3]),
  };
}

/**
 * Uniform, direction-preserving scaling so that no wheel exceeds maxSpeed.
 * @param {ArrayLike<number>} wheels
 * @param {number} maxSpeed
 * @returns {{ wheels: Wheels4, scale: number }}
 */
export function saturate(wheels, maxSpeed) {
  /** @type {Wheels4} */
  const w = [wheels[0], wheels[1], wheels[2], wheels[3]];
  const peak = Math.max(...w.map(Math.abs));
  if (!(maxSpeed > 0) || maxSpeed === Infinity || peak <= maxSpeed) return { wheels: w, scale: 1 };
  const scale = maxSpeed / peak;
  return { wheels: /** @type {Wheels4} */ (w.map((x) => x * scale)), scale };
}

/** @typedef {{ count: number, lastEdgeUs: number }} EncoderSnapshot */

/** Fixed-period / period / blended wheel speed estimator (odocore velocity.h). */
export class VelocityEstimator {
  /** @param {VelocityConfig} config */
  constructor(config) {
    this.config = config;
    this.primed = false;
    this.haveRef = false;
    this.prevCount = 0;
    this.prevUs = 0;
    this.edgeCount = 0;
    this.edgeUs = 0;
    this.fixedCps = 0;
    this.periodCps = 0;
    this.blendedCps = 0;
    this.weight = 0;
  }

  reset() {
    this.primed = false;
    this.haveRef = false;
    this.prevCount = 0;
    this.prevUs = 0;
    this.edgeCount = 0;
    this.edgeUs = 0;
    this.fixedCps = 0;
    this.periodCps = 0;
    this.blendedCps = 0;
    this.weight = 0;
  }

  /**
   * @param {number} fixedCps
   * @param {VelocityConfig} c
   */
  static weightFor(fixedCps, c) {
    const speed = Math.abs(fixedCps);
    if (c.highCps <= c.lowCps) return speed >= c.lowCps ? 1 : 0;
    if (speed <= c.lowCps) return 0;
    if (speed >= c.highCps) return 1;
    return (speed - c.lowCps) / (c.highCps - c.lowCps);
  }

  /**
   * @param {EncoderSnapshot} s
   * @param {number} nowUs
   * @returns {number} counts per second
   */
  update(s, nowUs) {
    if (!this.primed) {
      this.primed = true;
      this.prevCount = s.count;
      this.prevUs = nowUs;
      this.haveRef = false;
      this.edgeCount = s.count;
      this.edgeUs = s.lastEdgeUs;
      return this.blendedCps;
    }
    const dt = elapsedUs(nowUs, this.prevUs);
    const delta = countDelta(s.count, this.prevCount);
    if (dt > 0) this.fixedCps = (delta * 1e6) / dt;
    this.prevCount = s.count;
    this.prevUs = nowUs;
    if (s.count !== this.edgeCount || s.lastEdgeUs !== this.edgeUs) {
      const dc = countDelta(s.count, this.edgeCount);
      const de = elapsedUs(s.lastEdgeUs, this.edgeUs);
      if (dc === 0) this.periodCps = 0;
      else if (this.haveRef && de > 0) this.periodCps = (dc * 1e6) / de;
      this.edgeCount = s.count;
      this.edgeUs = s.lastEdgeUs;
      this.haveRef = true;
    } else if (this.haveRef) {
      const since = elapsedUs(nowUs, this.edgeUs);
      if (since >= this.config.zeroTimeoutUs) {
        this.periodCps = 0;
        this.haveRef = false;
      } else if (since > 0) {
        const bound = 1e6 / since;
        if (Math.abs(this.periodCps) > bound)
          this.periodCps = this.periodCps < 0 || Object.is(this.periodCps, -0) ? -bound : bound;
      }
    }
    this.weight = VelocityEstimator.weightFor(this.fixedCps, this.config);
    this.blendedCps = this.weight * this.fixedCps + (1 - this.weight) * this.periodCps;
    return this.blendedCps;
  }
}

/**
 * @param {number} previous
 * @param {number} requested
 * @param {number} maxRate
 * @param {number} dt
 */
export function slewLimit(previous, requested, maxRate, dt) {
  if (!(maxRate > 0 && dt > 0)) return requested;
  const step = maxRate * dt;
  return Math.min(Math.max(requested, previous - step), previous + step);
}

/** Per-wheel velocity controller (odocore pid.h). */
export class VelocityPid {
  /** @param {PidConfig} config */
  constructor(config) {
    this.config = config;
    this.integral = 0;
    this.output = 0;
    this.limited = false;
    this.dState = 0;
    this.prevMeasured = 0;
    this.havePrev = false;
  }

  reset() {
    this.integral = 0;
    this.output = 0;
    this.limited = false;
    this.dState = 0;
    this.prevMeasured = 0;
    this.havePrev = false;
  }

  /** @param {number} target */
  feedforward(target) {
    let ff = this.config.kff * target;
    if (target > 1e-6) ff += this.config.ks;
    else if (target < -1e-6) ff -= this.config.ks;
    return ff;
  }

  /**
   * @param {number} target rad/s
   * @param {number} measured rad/s
   * @param {number} dt s
   * @returns {number} duty in [-outputLimit, outputLimit]
   */
  update(target, measured, dt) {
    if (!(dt > 0) || !Number.isFinite(dt) || !Number.isFinite(target) || !Number.isFinite(measured))
      return this.output;
    const c = this.config;
    const limit = Math.abs(c.outputLimit);
    const error = target - measured;
    let derivative = 0;
    if (c.kd !== 0 && this.havePrev) {
      const raw = -(measured - this.prevMeasured) / dt;
      if (c.dCutoffHz > 0) {
        const tau = 1 / (2 * Math.PI * c.dCutoffHz);
        this.dState += (dt / (dt + tau)) * (raw - this.dState);
      } else this.dState = raw;
      derivative = c.kd * this.dState;
    }
    this.prevMeasured = measured;
    this.havePrev = true;
    const candidate = this.integral + c.ki * error * dt;
    const unclipped = this.feedforward(target) + c.kp * error + candidate + derivative;
    const saturated = Math.min(Math.max(unclipped, -limit), limit);
    const out = slewLimit(this.output, saturated, c.slewRate, dt);
    this.limited = out !== unclipped;
    const pushing = (unclipped > out && error > 0) || (unclipped < out && error < 0);
    if (!(this.limited && pushing)) this.integral = candidate;
    this.integral = Math.min(Math.max(this.integral, -limit), limit);
    this.output = out;
    return out;
  }
}

/** Command watchdog: targets go to zero when no command arrives for the timeout. */
export class CommandWatchdog {
  /** @param {number} timeoutUs */
  constructor(timeoutUs) {
    this.timeoutUs = timeoutUs;
    this.lastFeed = 0;
    this.expired = true;
    this.trips = 0;
  }
  /** @param {number} nowUs */
  feed(nowUs) {
    this.lastFeed = nowUs;
    this.expired = false;
  }
  /** @param {number} nowUs */
  update(nowUs) {
    if (this.expired) return false;
    if (elapsedUs(nowUs, this.lastFeed) > this.timeoutUs) {
      this.expired = true;
      this.trips++;
      return false;
    }
    return true;
  }
}

/**
 * @typedef {object} WheelStatus
 * @property {number} count int32 4x count
 * @property {number} velocity rad/s measured
 * @property {number} target rad/s set-point
 * @property {number} duty -1..1
 */

/** The firmware's drive controller (odocore drive_controller.h). */
export class DriveController {
  /** @param {DriveConfig} config */
  constructor(config) {
    this.config = config;
    this.estimators = [0, 1, 2, 3].map(() => new VelocityEstimator(config.velocity));
    this.pids = [0, 1, 2, 3].map(() => new VelocityPid(config.pid));
    /** @type {WheelStatus[]} */
    this.wheels = [0, 1, 2, 3].map(() => ({ count: 0, velocity: 0, target: 0, duty: 0 }));
    this.watchdog = new CommandWatchdog(config.watchdogTimeoutMs * 1000);
    /** @type {Pose2} */
    this.pose = { x: 0, y: 0, theta: 0 };
    /** @type {Wheels4} */
    this.targets = [0, 0, 0, 0];
    /** @type {Twist} */
    this.measured = { vx: 0, vy: 0, wz: 0 };
    /** @type {Twist} */
    this.active = { vx: 0, vy: 0, wz: 0 };
    this.scale = 1;
    this.primed = false;
    this.prevUs = 0;
    this.prevCount = [0, 0, 0, 0];
  }

  get radiansPerTick() {
    return (2 * Math.PI) / this.config.ticksPerWheelRev;
  }

  /**
   * A velocity command (VELOCITY_COMMAND): IK, saturation, watchdog feed.
   * @param {Twist} twist
   * @param {number} nowUs
   * @returns {boolean} false for a non-finite command (ignored)
   */
  command(twist, nowUs) {
    if (![twist.vx, twist.vy, twist.wz].every(Number.isFinite)) return false;
    const g = this.config.geometry;
    const sat = saturate(fwInverse(g, twist), this.config.maxWheelSpeed);
    this.scale = sat.scale;
    this.targets = sat.wheels;
    const vy = g.drive === DIFFERENTIAL ? 0 : twist.vy;
    this.active = { vx: twist.vx * sat.scale, vy: vy * sat.scale, wz: twist.wz * sat.scale };
    this.watchdog.feed(nowUs);
    return true;
  }

  stopAll() {
    for (let i = 0; i < 4; i++) {
      this.pids[i].reset();
      this.targets[i] = 0;
      this.wheels[i].target = 0;
      this.wheels[i].duty = 0;
    }
    this.active = { vx: 0, vy: 0, wz: 0 };
    this.scale = 1;
    return [0, 0, 0, 0];
  }

  /**
   * One control period: sample encoders, update odometry, run the loops.
   * @param {EncoderSnapshot[]} snapshots
   * @param {number} nowUs
   * @returns {number[]} duty per wheel
   */
  step(snapshots, nowUs) {
    const radPerTick = this.radiansPerTick;
    if (!this.primed) {
      snapshots.forEach((s, i) => {
        this.estimators[i].update(s, nowUs);
        this.prevCount[i] = s.count;
        this.wheels[i].count = s.count;
        this.wheels[i].duty = 0;
      });
      this.prevUs = nowUs;
      this.primed = true;
      return [0, 0, 0, 0];
    }
    const dt = elapsedUs(nowUs, this.prevUs) * 1e-6;
    this.prevUs = nowUs;
    const speeds = [0, 0, 0, 0];
    const increments = [0, 0, 0, 0];
    snapshots.forEach((s, i) => {
      speeds[i] = this.estimators[i].update(s, nowUs) * radPerTick;
      increments[i] = countDelta(s.count, this.prevCount[i]) * radPerTick;
      this.prevCount[i] = s.count;
      this.wheels[i].count = s.count;
      this.wheels[i].velocity = speeds[i];
    });
    const g = this.config.geometry;
    const d = fwForward(g, increments);
    this.pose = compose(this.pose, se2Exp(d.vx, d.vy, d.wz));
    this.measured = fwForward(g, speeds);
    if (!this.watchdog.update(nowUs)) return this.stopAll();
    const duty = [0, 0, 0, 0];
    for (let i = 0; i < 4; i++) {
      duty[i] = this.pids[i].update(this.targets[i], speeds[i], dt);
      this.wheels[i].target = this.targets[i];
      this.wheels[i].duty = duty[i];
    }
    return duty;
  }

  /**
   * The WHEEL_STATE telemetry message (0x02) as field values.
   * @param {number} nowUs
   */
  wheelState(nowUs) {
    return {
      t_us: nowUs >>> 0,
      count: this.wheels.map((w) => w.count),
      velocity: this.wheels.map((w) => w.velocity),
      target: this.wheels.map((w) => w.target),
      duty: this.wheels.map((w) => dutyToWire(w.duty)),
    };
  }

  /**
   * The ODOMETRY telemetry message (0x03) as field values.
   * @param {number} nowUs
   */
  odometry(nowUs) {
    return {
      t_us: nowUs >>> 0,
      x: this.pose.x,
      y: this.pose.y,
      theta: wrapAngle(this.pose.theta),
      vx: this.measured.vx,
      vy: this.measured.vy,
      wz: this.measured.wz,
    };
  }
}

/**
 * Duty as the protocol's i16: x 10000, clamped, rounded half away from zero
 * (odobridge protocol.duty_to_wire).
 * @param {number} duty
 */
export function dutyToWire(duty) {
  if (!Number.isFinite(duty)) return 0;
  const scaled = Math.max(-10000, Math.min(10000, duty * 10000));
  return scaled >= 0 ? Math.trunc(scaled + 0.5) : -Math.trunc(-scaled + 0.5);
}
