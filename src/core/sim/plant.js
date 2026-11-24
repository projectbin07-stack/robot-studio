// The physical side of the drive: one first-order DC motor and gearbox per
// wheel, driven by the controller's duty, turning an ideal quadrature
// encoder whose edges are timestamped exactly (by interpolation within a
// sub-step). This is encoder-odometry-firmware's plant model
// (odobridge.firmware_model.MotorModel and SimulatedRobot), integrated the
// same way, with the motion of the robot left to the caller: every
// sub-step reports each wheel's mean speed so the simulator can apply slip
// and move the body.

import { pyRound } from "./firmware.js";

/**
 * First-order DC motor + gearbox, wheel side: steady-state speed for duty
 * u is sign(u) (|u| - deadband) / dutyPerRadS (zero inside the dead band),
 * approached with time constant tau. The defaults are the firmware model's
 * made-up round numbers, not a measured motor.
 * @typedef {{ dutyPerRadS: number, deadband: number, tau: number }} MotorModel
 */

/** @type {Readonly<MotorModel>} */
export const DEFAULT_MOTOR = Object.freeze({ dutyPerRadS: 0.034, deadband: 0.06, tau: 0.08 });

/**
 * @param {MotorModel} m
 * @param {number} duty
 */
export function steadyState(m, duty) {
  const magnitude = Math.abs(duty) - m.deadband;
  if (magnitude <= 0) return 0;
  return (duty < 0 || Object.is(duty, -0) ? -1 : 1) * (magnitude / m.dutyPerRadS);
}

const U32 = 0x100000000;

export class Plant {
  /**
   * @param {number} wheels number of physical wheels
   * @param {number} ticksPerWheelRev
   * @param {MotorModel} motor
   * @param {number} [substeps]
   * @param {number} [startUs]
   */
  constructor(wheels, ticksPerWheelRev, motor, substeps = 10, startUs = 0) {
    this.radPerTick = (2 * Math.PI) / ticksPerWheelRev;
    this.motor = motor;
    this.substeps = substeps;
    this.nowUs = startUs >>> 0;
    this.angle = new Array(wheels).fill(0);
    this.speed = new Array(wheels).fill(0);
    /** Unwrapped counts (the int32 the firmware sees is taken on read). */
    this.count = new Array(wheels).fill(0);
    this.lastEdgeUs = new Array(wheels).fill(this.nowUs);
  }

  /** Encoder snapshots, one per physical wheel. */
  snapshots() {
    return this.count.map((c, i) => ({ count: c | 0, lastEdgeUs: this.lastEdgeUs[i] }));
  }

  /**
   * Advance by dt with the given duty per wheel (motor model) or wheel speed
   * per wheel (ideal wheels: the speed is reached instantly).
   * @param {number[]} input duty (-1..1) or, with ideal, rad/s
   * @param {number} dt s
   * @param {(mean: number[], h: number) => void} move called every sub-step with the
   *   mean wheel speeds over it (rad/s)
   * @param {boolean} [ideal]
   */
  advance(input, dt, move, ideal = false) {
    const h = dt / this.substeps;
    const hUs = h * 1e6;
    const n = this.speed.length;
    const mean = new Array(n).fill(0);
    for (let k = 0; k < this.substeps; k++) {
      const t0 = k * hUs;
      const before = [...this.speed];
      for (let i = 0; i < n; i++) {
        if (ideal) this.speed[i] = input[i];
        else {
          const target = steadyState(this.motor, input[i]);
          const alpha = 1 - Math.exp(-h / this.motor.tau);
          this.speed[i] += alpha * (target - this.speed[i]);
        }
        const old = this.angle[i];
        mean[i] = ideal ? this.speed[i] : 0.5 * (before[i] + this.speed[i]);
        this.angle[i] += mean[i] * h;
        const newCount = Math.floor(this.angle[i] / this.radPerTick);
        if (newCount !== this.count[i]) {
          const boundary =
            newCount > this.count[i]
              ? newCount * this.radPerTick
              : (newCount + 1) * this.radPerTick;
          const span = this.angle[i] - old;
          let frac = span !== 0 ? (boundary - old) / span : 1;
          frac = Math.min(Math.max(frac, 0), 1);
          this.lastEdgeUs[i] = (this.nowUs + pyRound(t0 + frac * hUs)) % U32;
          this.count[i] = newCount;
        }
      }
      move(mean, h);
    }
    this.nowUs = (this.nowUs + pyRound(dt * 1e6)) % U32;
  }
}
