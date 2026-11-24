// A live robot over rosbridge, in the shape the views expect: it subscribes
// to /joint_states, /tf, /tf_static, /scan, /imu and /odom (and
// /robot_description), checks every message (msgs.js), keeps a small TF
// buffer, and feeds a Track, so the 3D layer, the plots and the HUD show the
// robot's own data, run through the same scan matching as the simulation.
// Nothing here publishes: /cmd_vel goes through the dead-man gate
// (deadman.js), which the page connects separately.

import { Track } from "./track.js";
import * as M from "./msgs.js";
import { compose, inverse } from "../sim/se2.js";

/** @typedef {import("./rosbridge.js").RosbridgeClient} RosbridgeClient */
/** @typedef {import("../sim/robot.js").SimRobot} SimRobot */
/** @typedef {import("../sim/se2.js").Pose2} Pose2 */

/** The topics and types subscribed, and how often rosbridge may send them (ms). */
export const LIVE_TOPICS = Object.freeze({
  joint_states: { topic: "/joint_states", type: "sensor_msgs/msg/JointState", throttle: 50 },
  tf: { topic: "/tf", type: "tf2_msgs/msg/TFMessage", throttle: 0 },
  tf_static: { topic: "/tf_static", type: "tf2_msgs/msg/TFMessage", throttle: 0 },
  scan: { topic: "/scan", type: "sensor_msgs/msg/LaserScan", throttle: 0 },
  imu: { topic: "/imu", type: "sensor_msgs/msg/Imu", throttle: 20 },
  odom: { topic: "/odom", type: "nav_msgs/msg/Odometry", throttle: 0 },
  robot_description: { topic: "/robot_description", type: "std_msgs/msg/String", throttle: 0 },
});

/**
 * @typedef {object} TopicStats
 * @property {number} count accepted messages
 * @property {number} rejected messages that failed the checks
 * @property {number} last ms (clock) of the last accepted one, -Infinity before
 * @property {number[]} times recent arrival times, for the rate
 * @property {string} error last check failure
 */

export class LiveSession {
  /**
   * @param {RosbridgeClient} client
   * @param {{ robot: SimRobot | null, now: () => number, topics?: Partial<Record<keyof typeof LIVE_TOPICS, string>>, estimate?: boolean }} opts
   */
  constructor(client, opts) {
    this.client = client;
    this.robot = opts.robot;
    this.now = opts.now;
    this.t0 = opts.now();
    /** @type {Record<string, TopicStats>} */
    this.topics = {};
    /** @type {Map<string, { parent: string, t: [number, number, number], q: [number, number, number, number] }>} */
    this.staticTf = new Map();
    /** @type {Map<string, { parent: string, pose: Pose2 }>} latest dynamic transform per child */
    this.dynamicTf = new Map();
    /** @type {string | null} */
    this.robotDescription = null;
    /** @type {Set<(urdf: string) => void>} */
    this.descriptionListeners = new Set();
    /** @type {string | null} */
    this.odomChild = null;
    /** @type {string | null} */
    this.scanFrame = null;
    this.mountFrom = "none yet";
    const lidar = opts.robot?.sensors.find((s) => s.kind === "lidar2d") ?? null;
    this.urdfMount = lidar ? lidar.planar : null;
    this.lidarHeight = lidar ? lidar.height : 0.2;
    this.track = new Track({
      mount: this.urdfMount ?? { x: 0, y: 0, theta: 0 },
      lidarHeight: this.lidarHeight,
      estimate: opts.estimate,
    });
    /** @type {(() => void)[]} */
    this.unsubs = [];
    const names = {
      ...Object.fromEntries(Object.entries(LIVE_TOPICS).map(([k, v]) => [k, v.topic])),
      ...(opts.topics ?? {}),
    };
    /**
     * @param {keyof typeof LIVE_TOPICS} key
     * @param {(msg: unknown) => void} fn
     */
    const sub = (key, fn) => {
      const spec = LIVE_TOPICS[key];
      const topic = /** @type {string} */ (names[key]);
      /** @type {TopicStats} */
      const st = (this.topics[topic] = {
        count: 0,
        rejected: 0,
        last: -Infinity,
        times: [],
        error: "",
      });
      this.unsubs.push(
        client.subscribe(
          topic,
          spec.type,
          (msg) => {
            try {
              fn(msg);
            } catch (e) {
              st.rejected++;
              st.error = e instanceof Error ? e.message : String(e);
              return;
            }
            const now = this.now();
            st.count++;
            st.last = now;
            st.times.push(now);
            while (st.times.length > 0 && now - st.times[0] > 2000) st.times.shift();
          },
          spec.throttle ? { throttle_rate: spec.throttle, queue_length: 1 } : { queue_length: 1 },
        ),
      );
    };
    sub("tf_static", (m) => this.onTf(M.tfMessage(m), true));
    sub("tf", (m) => this.onTf(M.tfMessage(m), false));
    sub("odom", (m) => this.onOdom(M.odometry(m)));
    sub("scan", (m) => this.onScan(M.laserScan(m)));
    sub("imu", (m) => {
      const r = M.imu(m);
      this.track.imu(this.time(), r.accel, r.gyro);
    });
    sub("joint_states", (m) => this.onJoints(M.jointState(m)));
    sub("robot_description", (m) => this.setDescription(M.stringMsg(m)));
  }

  /** Seconds since the session started (receive time: robust to a robot clock that is off). */
  time() {
    return (this.now() - this.t0) / 1000;
  }

  /** Stop listening. */
  dispose() {
    for (const u of this.unsubs) u();
    this.unsubs = [];
  }

  /** @param {(urdf: string) => void} fn */
  onDescription(fn) {
    this.descriptionListeners.add(fn);
    if (this.robotDescription !== null) fn(this.robotDescription);
    return () => this.descriptionListeners.delete(fn);
  }

  /** @param {string} text */
  setDescription(text) {
    if (!/<robot[\s>]/.test(text)) throw new M.MessageError("not a URDF (no <robot> element)");
    if (text === this.robotDescription) return;
    this.robotDescription = text;
    for (const fn of this.descriptionListeners) fn(text);
  }

  /**
   * Ask robot_state_publisher for its robot_description parameter through
   * rosapi (ROS 2: "node:parameter"); the value comes back JSON-encoded.
   * @param {string} [node]
   * @param {number} [timeoutMs]
   */
  async requestDescription(node = "/robot_state_publisher", timeoutMs = 4000) {
    const values = /** @type {any} */ (
      await this.client.callService(
        "/rosapi/get_param",
        { name: `${node}:robot_description`, default: "" },
        { timeoutMs },
      )
    );
    const raw = values && typeof values.value === "string" ? values.value : "";
    let text = raw;
    try {
      const v = JSON.parse(raw);
      if (typeof v === "string") text = v;
    } catch {
      // not JSON-encoded: use as it is
    }
    M.stringMsg({ data: text });
    this.setDescription(text);
    return text;
  }

  /**
   * @param {ReturnType<typeof M.tfMessage>} list
   * @param {boolean} isStatic
   */
  onTf(list, isStatic) {
    for (const tr of list) {
      if (isStatic) this.staticTf.set(tr.child, { parent: tr.parent, t: tr.t, q: tr.q });
      else
        this.dynamicTf.set(tr.child, {
          parent: tr.parent,
          pose: { x: tr.t[0], y: tr.t[1], theta: M.yawOf(tr.q) },
        });
    }
    if (isStatic) this.updateMount();
    else if (!this.topics[LIVE_TOPICS.odom.topic]?.count) {
      // No /odom: take odom -> base from /tf.
      const base = this.baseFrame();
      const p = this.lookupPlanar("odom", base);
      if (p) this.track.odometry(this.time(), p);
    }
  }

  /** The base frame: the robot's (URDF) base, else the odometry's child. */
  baseFrame() {
    return this.robot?.baseFrame ?? this.odomChild ?? "base_link";
  }

  /**
   * Planar pose of `child` in `parent` through the TF buffer (static and
   * latest dynamic transforms, any depth), or null when not connected.
   * @param {string} parent
   * @param {string} child
   * @returns {Pose2 | null}
   */
  lookupPlanar(parent, child) {
    /** @param {string} f @returns {[string, Pose2][]} frame chain up to the root, with each pose in the root */
    const chain = (f) => {
      /** @type {[string, Pose2][]} */
      const out = [[f, { x: 0, y: 0, theta: 0 }]];
      let cur = f;
      /** @type {Pose2} */
      let acc = { x: 0, y: 0, theta: 0 };
      for (let i = 0; i < 64; i++) {
        const s = this.staticTf.get(cur);
        const d = this.dynamicTf.get(cur);
        const step = s
          ? { parent: s.parent, pose: { x: s.t[0], y: s.t[1], theta: M.yawOf(s.q) } }
          : d;
        if (!step) break;
        acc = compose(step.pose, acc);
        cur = step.parent;
        out.push([cur, acc]);
      }
      return out;
    };
    const a = chain(parent);
    const b = chain(child);
    for (const [fb, pb] of b) {
      const hit = a.find(([fa]) => fa === fb);
      if (hit) return compose(inverse(hit[1]), pb);
    }
    return null;
  }

  /** 3D mount of the scan frame in the base frame, from /tf_static (z included). */
  updateMount() {
    if (!this.scanFrame) return;
    const base = this.baseFrame();
    const p = this.lookupPlanar(base, this.scanFrame);
    if (!p) return;
    this.track.opts.mount = p;
    this.mountFrom = `/tf_static (${base} -> ${this.scanFrame})`;
    // Height above the floor: walk the static z offsets from the scan frame up to the root.
    let z = 0;
    let cur = this.scanFrame;
    for (let i = 0; i < 64; i++) {
      const s = this.staticTf.get(cur);
      if (!s) break;
      z += s.t[2];
      cur = s.parent;
    }
    if (z > 0) this.track.opts.lidarHeight = this.lidarHeight = z;
  }

  /** @param {ReturnType<typeof M.odometry>} o */
  onOdom(o) {
    this.odomChild = o.child;
    const base = this.baseFrame();
    // Pose of the base frame: the odometry's child frame, moved by the static child -> base offset.
    const off = o.child === base ? { x: 0, y: 0, theta: 0 } : this.lookupPlanar(o.child, base);
    const t = this.time();
    this.track.odometry(t, off ? compose(o.pose, off) : o.pose);
    this.track.twist(t, null, o.twist);
  }

  /** @param {ReturnType<typeof M.laserScan>} s */
  onScan(s) {
    if (s.frame !== this.scanFrame) {
      this.scanFrame = s.frame;
      this.updateMount();
      if (this.mountFrom === "none yet")
        this.mountFrom = this.urdfMount
          ? "the URDF (no /tf_static path)"
          : "none: the LiDAR is drawn at the base";
    }
    const n = s.ranges.length;
    const angles = new Float64Array(n);
    for (let i = 0; i < n; i++) angles[i] = s.angle_min + i * s.angle_increment;
    this.track.scan(this.time(), { angles, ranges: s.ranges, maxRange: s.range_max });
  }

  /** @param {ReturnType<typeof M.jointState>} j */
  onJoints(j) {
    /** @type {Record<string, number>} */
    const q = {};
    j.name.forEach((n, i) => {
      if (Number.isFinite(j.position[i])) q[n] = j.position[i];
    });
    const t = this.time();
    this.track.joints(t, q);
    if (this.robot) {
      const idx = new Map(j.name.map((n, i) => [n, i]));
      const wheels = this.robot.wheels;
      const vel = wheels.map((w) => {
        const i = idx.get(w.joint);
        return i === undefined || j.velocity.length === 0 ? NaN : w.sign * j.velocity[i];
      });
      const four = wheels.length === 2 ? [vel[0], vel[1], NaN, NaN] : vel;
      this.track.wheels(t, [NaN, NaN, NaN, NaN], four);
    }
  }

  /**
   * The command this page is sending (for the twist plot).
   * @param {{ vx: number, vy: number, wz: number }} cmd
   */
  commanded(cmd) {
    this.track.twist(this.time(), cmd, null);
  }

  get history() {
    return this.track.history;
  }

  frame() {
    this.track.at(this.time());
    return this.track.frame();
  }

  /**
   * Per-topic rate (Hz over the last 2 s), age of the last message (s), counts.
   */
  topicSummary() {
    const now = this.now();
    return Object.entries(this.topics).map(([topic, s]) => {
      const recent = s.times.filter((t) => now - t <= 2000);
      return {
        topic,
        count: s.count,
        rejected: s.rejected,
        hz:
          recent.length >= 2
            ? (recent.length - 1) / ((recent[recent.length - 1] - recent[0]) / 1000)
            : 0,
        age: Number.isFinite(s.last) ? (now - s.last) / 1000 : null,
        error: s.error,
      };
    });
  }
}
