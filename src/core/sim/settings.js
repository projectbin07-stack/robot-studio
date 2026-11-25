// Simulation settings: defaults, validation and the merge with what the
// URDF says (Gazebo <sensor> tags). Every number here is a model parameter
// chosen for the simulation, not a measurement of a real robot; the
// defaults say where they come from.

import { DEFAULT_LIDAR } from "../../../vendor/lidar-slam-lab/src/lidar.js";
import { FW_DEFAULTS } from "./firmware.js";
import { DEFAULT_MOTOR } from "./plant.js";
import { DEFAULT_ESTIMATORS } from "./estimators.js";
import { imuModelFromCalibration } from "./imu.js";

/**
 * @typedef {object} LidarSettings
 * @property {number} beams
 * @property {number} fovDeg
 * @property {number} minRange m
 * @property {number} maxRange m
 * @property {number} rangeNoise std, m
 * @property {number} dropout probability a beam returns nothing
 * @property {number} rateHz
 * @property {boolean} useUrdf take samples, angles, range, noise and rate from a Gazebo <sensor> tag when the URDF has one
 */

/**
 * @typedef {object} ImuSettings
 * @property {number} rateHz
 * @property {unknown} calibration imu-calibration-lab JSON (parsed), or null
 * @property {boolean} useUrdf use Gazebo <imu> noise when the URDF has it and no calibration is given
 */

/**
 * @typedef {object} DriveSettings
 * @property {boolean} ideal wheels reach the commanded speed instantly (no motor, no PI loop)
 * @property {number} encoderPpr encoder lines per motor revolution
 * @property {number} gearRatio motor revolutions per wheel revolution
 * @property {number} maxWheelSpeed rad/s, 0 = from the URDF's wheel <limit velocity>
 * @property {import("./plant.js").MotorModel} motor
 * @property {import("./firmware.js").PidConfig} pid
 * @property {boolean} slip random wheel slip on
 * @property {number} slipStd std of the slip fraction (Gauss-Markov per wheel)
 * @property {number} slipTau s, correlation time of the slip
 * @property {number} radiusErrorLeft fraction: true left wheel radius / nominal - 1
 * @property {number} radiusErrorRight
 */

/**
 * @typedef {object} DepthSettings
 * @property {boolean} enabled
 * @property {number} width columns
 * @property {number} height rows
 * @property {number} hfovDeg
 * @property {number} maxRange m
 * @property {number} rateHz
 */

/**
 * @typedef {object} SimSettings
 * @property {number} seed
 * @property {LidarSettings} lidar
 * @property {ImuSettings} imu
 * @property {DriveSettings} drive
 * @property {{ scanMatching: boolean, particles: number, mclGlobal: boolean, mclOmni: boolean, mclRecovery: boolean }} estimators
 * @property {DepthSettings} depth
 */

/** @returns {SimSettings} */
export function defaultSimSettings() {
  return {
    seed: 1,
    // lidar-slam-lab's LiDAR defaults, plus a minimum range and a rate.
    lidar: {
      beams: DEFAULT_LIDAR.beams,
      fovDeg: 360,
      minRange: 0.12,
      maxRange: DEFAULT_LIDAR.maxRange,
      rangeNoise: DEFAULT_LIDAR.rangeNoise,
      dropout: DEFAULT_LIDAR.dropout,
      rateHz: 10,
      useUrdf: true,
    },
    imu: { rateHz: 100, calibration: null, useUrdf: true },
    drive: {
      ideal: false,
      // 500 lines x 4 (quadrature) x 19:1 = 38000 counts per wheel
      // revolution, encoder-odometry-firmware's default ticks_per_wheel_rev.
      encoderPpr: 500,
      gearRatio: 19,
      maxWheelSpeed: 0,
      motor: { ...DEFAULT_MOTOR },
      pid: { ...FW_DEFAULTS.pid },
      slip: true,
      slipStd: 0.02,
      slipTau: 0.5,
      radiusErrorLeft: 0.003,
      radiusErrorRight: -0.003,
    },
    estimators: {
      scanMatching: DEFAULT_ESTIMATORS.scanMatching,
      particles: DEFAULT_ESTIMATORS.particles,
      mclGlobal: DEFAULT_ESTIMATORS.mclGlobal,
      mclOmni: DEFAULT_ESTIMATORS.mclOmni,
      mclRecovery: DEFAULT_ESTIMATORS.mclRecovery,
    },
    depth: { enabled: true, width: 32, height: 24, hfovDeg: 70, maxRange: 5, rateHz: 2 },
  };
}

/** Allowed range of each numeric setting: [min, max, integer]. */
const LIMITS = /** @type {Record<string, [number, number, boolean]>} */ ({
  seed: [0, 2 ** 32 - 1, true],
  "lidar.beams": [8, 2000, true],
  "lidar.fovDeg": [10, 360, false],
  "lidar.minRange": [0, 5, false],
  "lidar.maxRange": [0.5, 60, false],
  "lidar.rangeNoise": [0, 1, false],
  "lidar.dropout": [0, 0.9, false],
  "lidar.rateHz": [1, 50, false],
  "imu.rateHz": [10, 100, false],
  "drive.encoderPpr": [1, 100000, true],
  "drive.gearRatio": [0.01, 1000, false],
  "drive.maxWheelSpeed": [0, 1000, false],
  "drive.slipStd": [0, 0.5, false],
  "drive.slipTau": [0.01, 100, false],
  "drive.radiusErrorLeft": [-0.2, 0.2, false],
  "drive.radiusErrorRight": [-0.2, 0.2, false],
  "drive.motor.dutyPerRadS": [1e-4, 10, false],
  "drive.motor.deadband": [0, 0.9, false],
  "drive.motor.tau": [0.001, 10, false],
  "drive.pid.kp": [0, 10, false],
  "drive.pid.ki": [0, 100, false],
  "drive.pid.kd": [0, 1, false],
  "drive.pid.kff": [0, 1, false],
  "drive.pid.ks": [0, 1, false],
  "drive.pid.outputLimit": [0, 1, false],
  "drive.pid.slewRate": [0, 1000, false],
  "drive.pid.dCutoffHz": [0, 1000, false],
  "estimators.particles": [0, 5000, true],
  "depth.width": [4, 160, true],
  "depth.height": [3, 120, true],
  "depth.hfovDeg": [10, 170, false],
  "depth.maxRange": [0.3, 20, false],
  "depth.rateHz": [0.2, 30, false],
});

/**
 * Settings with every field present and in range: missing fields take the
 * defaults, out-of-range numbers throw with the field's name.
 * @param {unknown} given partial settings (e.g. from a document or JSON)
 * @returns {SimSettings}
 */
export function normalizeSettings(given) {
  const d = /** @type {any} */ (defaultSimSettings());
  const merge = (/** @type {any} */ base, /** @type {any} */ over, /** @type {string} */ path) => {
    if (over === undefined || over === null) return base;
    if (typeof over !== "object" || Array.isArray(over))
      throw new RangeError(`${path || "settings"} must be an object`);
    for (const k of Object.keys(over)) {
      const key = path ? `${path}.${k}` : k;
      if (!(k in base)) throw new RangeError(`unknown simulation setting '${key}'`);
      const b = base[k];
      const v = over[k];
      if (key === "imu.calibration") base[k] = v ?? null;
      else if (b !== null && typeof b === "object") merge(b, v, key);
      else if (typeof b === "boolean") {
        if (typeof v !== "boolean") throw new RangeError(`${key} must be true or false`);
        base[k] = v;
      } else {
        const lim = LIMITS[key];
        if (typeof v !== "number" || !Number.isFinite(v))
          throw new RangeError(`${key} must be a number`);
        if (lim && (v < lim[0] || v > lim[1] || (lim[2] && !Number.isInteger(v))))
          throw new RangeError(
            `${key} must be ${lim[2] ? "an integer" : "a number"} in [${lim[0]}, ${lim[1]}]`,
          );
        base[k] = v;
      }
    }
    return base;
  };
  const s = merge(d, given, "");
  if (s.imu.calibration !== null) {
    try {
      imuModelFromCalibration(s.imu.calibration);
    } catch (e) {
      throw new RangeError(`imu.calibration: ${e instanceof Error ? e.message : String(e)}`);
    }
  }
  if (s.lidar.minRange >= s.lidar.maxRange)
    throw new RangeError("lidar.minRange must be below lidar.maxRange");
  return /** @type {SimSettings} */ (s);
}

/**
 * Settings that differ from the defaults (what a document or link stores).
 * @param {SimSettings} s
 * @returns {Record<string, unknown>}
 */
export function settingsDiff(s) {
  const d = /** @type {any} */ (defaultSimSettings());
  /** @param {any} a @param {any} b */
  const diff = (a, b) => {
    /** @type {Record<string, unknown>} */
    const out = {};
    for (const k of Object.keys(a)) {
      if (
        a[k] !== null &&
        typeof a[k] === "object" &&
        b[k] !== null &&
        typeof b[k] === "object" &&
        k !== "calibration"
      ) {
        const sub = diff(a[k], b[k]);
        if (Object.keys(sub).length > 0) out[k] = sub;
      } else if (JSON.stringify(a[k]) !== JSON.stringify(b[k])) out[k] = a[k];
    }
    return out;
  };
  return diff(s, d);
}

/** 4x quadrature counts per wheel revolution. @param {DriveSettings} d */
export const ticksPerWheelRev = (d) => 4 * d.encoderPpr * d.gearRatio;
