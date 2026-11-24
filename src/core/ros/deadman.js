// The dead-man switch between the teleop and a real robot's /cmd_vel.
//
// Nothing is published until the user has opted in (arm()). While armed, a
// command goes out at a fixed rate only while an input is held and fresh;
// the moment the input is released, the page loses focus or is hidden, the
// connection drops, or the inputs stop arriving for `staleMs` (a frozen
// tab), a zero Twist is sent (three times, so one lost message does not
// leave the robot moving). Hiding the page or losing the connection also
// disarms: driving again needs a new opt-in. Every command is scaled, in
// the direction it points, so that no wheel turns faster than
// `fraction` x the robot's wheel limit (the firmware's own saturation), and
// a fraction above 1 is refused.
//
// The robot itself must still stop on its own when commands stop arriving
// (a cmd_vel timeout in its driver): a browser cannot send a stop over a
// connection that is gone. docs/ros2.md says so.

import { fwInverse, saturate } from "../sim/firmware.js";

/** @typedef {import("../sim/firmware.js").Twist} Twist */
/** @typedef {import("../sim/firmware.js").FwGeometry} FwGeometry */

export const ZERO = Object.freeze({ vx: 0, vy: 0, wz: 0 });

/**
 * @typedef {object} GateOptions
 * @property {FwGeometry} geometry the robot's wheels (firmware convention)
 * @property {number} maxWheelSpeed rad/s, the robot's wheel limit
 * @property {(twist: Twist) => boolean} publish returns false when it could not be sent
 * @property {() => number} now ms
 * @property {number} [rateHz] command rate while held (default 10)
 * @property {number} [staleMs] an input older than this counts as released (default 250)
 * @property {number} [stopRepeats] zero Twists sent on a stop (default 3)
 */

/**
 * @typedef {"disarmed" | "armed" | "driving"} GateState
 */

export class CmdVelGate {
  /** @param {GateOptions} o */
  constructor(o) {
    if (!(o.maxWheelSpeed > 0) || !Number.isFinite(o.maxWheelSpeed))
      throw new Error("the robot's wheel speed limit is unknown: driving is not possible");
    this.o = o;
    this.periodMs = 1000 / (o.rateHz ?? 10);
    this.staleMs = o.staleMs ?? 250;
    this.stopRepeats = o.stopRepeats ?? 3;
    /** @type {GateState} */
    this.state = "disarmed";
    this.fraction = 0.25;
    /** @type {Twist | null} */
    this.input = null;
    this.inputAt = -Infinity;
    this.lastSentAt = -Infinity;
    this.stopsLeft = 0;
    /** @type {Twist} */
    this.last = { ...ZERO };
    this.sent = 0;
    this.stops = 0;
    /** Why the gate last stopped or disarmed. */
    this.reason = "not armed";
    /** @type {Set<(state: GateState, reason: string) => void>} */
    this.listeners = new Set();
  }

  /** @param {(state: GateState, reason: string) => void} fn */
  onChange(fn) {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }

  /**
   * @param {GateState} s
   * @param {string} reason
   */
  set(s, reason) {
    const changed = s !== this.state || reason !== this.reason;
    this.state = s;
    this.reason = reason;
    if (changed) for (const fn of this.listeners) fn(s, reason);
  }

  /**
   * The user's explicit opt-in.
   * @param {{ confirmed: boolean, fraction: number }} o fraction of the wheel limit, (0, 1]
   */
  arm(o) {
    if (!o.confirmed) throw new Error("driving a real robot needs an explicit confirmation");
    if (!(o.fraction > 0 && o.fraction <= 1))
      throw new Error("the speed limit must be more than 0 and at most the robot's wheel limit");
    this.fraction = o.fraction;
    this.input = null;
    this.set("armed", "armed: hold a key or stick to drive");
  }

  /** @param {string} reason */
  disarm(reason) {
    if (this.state === "disarmed") {
      this.set("disarmed", reason);
      return;
    }
    this.stop(reason);
    this.set("disarmed", reason);
  }

  /** Largest wheel speed allowed, rad/s. */
  get wheelLimit() {
    return this.fraction * this.o.maxWheelSpeed;
  }

  /**
   * The command scaled so no wheel exceeds the limit (direction kept).
   * @param {Twist} t
   * @returns {Twist}
   */
  limit(t) {
    if (![t.vx, t.vy, t.wz].every(Number.isFinite)) return { ...ZERO };
    const v = this.o.geometry.drive === "differential" ? { ...t, vy: 0 } : { ...t };
    const { scale } = saturate(fwInverse(this.o.geometry, v), this.wheelLimit);
    return { vx: v.vx * scale, vy: v.vy * scale, wz: v.wz * scale };
  }

  /**
   * The current teleop command, once per animation frame (null: nothing held).
   * Releasing sends the stop at once.
   * @param {Twist | null} t
   */
  update(t) {
    const now = this.o.now();
    const held = t !== null && (t.vx !== 0 || t.vy !== 0 || t.wz !== 0);
    if (this.state === "disarmed") return;
    if (!held) {
      if (this.state === "driving") this.stop("released");
      this.input = null;
      return;
    }
    this.input = this.limit(/** @type {Twist} */ (t));
    this.inputAt = now;
    if (this.state !== "driving") {
      this.set("driving", "driving");
      this.send(this.input, now);
    }
    this.tick();
  }

  /**
   * Run from a timer (and from update()): sends the held command at the
   * rate, a pending stop, or a stop for a stale input.
   */
  tick() {
    const now = this.o.now();
    if (this.state === "driving") {
      if (!this.input || now - this.inputAt > this.staleMs) {
        this.stop(`no input for ${this.staleMs} ms`);
        return;
      }
      if (now - this.lastSentAt >= this.periodMs - 1e-6) this.send(this.input, now);
      return;
    }
    if (this.stopsLeft > 0 && now - this.lastSentAt >= this.periodMs - 1e-6) {
      this.stopsLeft--;
      this.send(ZERO, now);
    }
  }

  /** The page lost focus: stop (key-up events will not arrive). */
  blur() {
    if (this.state === "driving") this.stop("the page lost focus");
    this.input = null;
  }

  /** The page was hidden: stop and disarm. */
  hidden() {
    this.disarm("the page was hidden");
  }

  /**
   * Send a zero Twist now and twice more on the next ticks.
   * @param {string} reason
   */
  stop(reason) {
    const now = this.o.now();
    this.input = null;
    this.stops++;
    this.send(ZERO, now);
    this.stopsLeft = this.stopRepeats - 1;
    if (this.state === "driving") this.set("armed", `stopped: ${reason}`);
    else this.reason = `stopped: ${reason}`;
  }

  /**
   * @param {Twist} t
   * @param {number} now
   */
  send(t, now) {
    this.last = { ...t };
    this.lastSentAt = now;
    if (this.o.publish(this.last)) this.sent++;
  }
}

/**
 * Disarm the gate whenever the connection is not open (lost, reconnecting,
 * closed): after a reconnect the user opts in again.
 * @param {CmdVelGate} gate
 * @param {{ onState: (fn: (state: string, detail: string) => void) => () => void }} client
 */
export function disarmOnDisconnect(gate, client) {
  return client.onState((s, detail) => {
    if (s !== "open") gate.disarm(`connection ${s}${detail ? ` (${detail})` : ""}`);
  });
}
