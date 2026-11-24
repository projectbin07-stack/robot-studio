// A recording written as a ROS 2 bag: one MCAP file plus the metadata.yaml
// rosbag2 (Humble, bag format version 5) writes next to it, with the topic
// mapping documented in docs/simulation.md and docs/ros2.md:
//
//   /tf_static              tf2_msgs/msg/TFMessage      the URDF's fixed joints
//   /tf                     tf2_msgs/msg/TFMessage      odom -> root frame (wheel odometry), map -> odom (scan matching)
//   /odom                   nav_msgs/msg/Odometry       the firmware's ODOMETRY frames
//   /scan                   sensor_msgs/msg/LaserScan   scans.jsonl, no return -> +inf
//   /imu                    sensor_msgs/msg/Imu         imu.csv, no orientation
//   /joint_states           sensor_msgs/msg/JointState  joint_states.csv
//   /cmd_vel                geometry_msgs/msg/Twist     the commanded twist (trajectory.csv)
//   /firmware/wheel_state   std_msgs/msg/Float64MultiArray  the firmware's WHEEL_STATE fields
//   /ground_truth           nav_msgs/msg/Odometry       simulated truth of the base frame, in "map"
//   /robot_studio/source    std_msgs/msg/String         JSON: where the data came from (simulated)
//
// Frames: "odom" and "map" both sit at the robot's start pose; poses in the
// recording are in the simulated world's frame and are re-expressed. Without
// scans there is no scan-matched estimate and map -> odom is the identity.
// The bag's clock is the recording's time plus BAG_CLOCK_OFFSET: a stamp of
// 0 means "not set" to ROS tools (and "latest" to tf2), so the first
// simulated instant is written at 1 s.

import { compose, inverse } from "../sim/se2.js";
import { encodeCdr } from "./cdr.js";
import { schemaText } from "./msgdefs.js";
import { McapWriter } from "./mcap.js";
import { RECORDING_FORMAT } from "../sim/record.js";
import { serializeSimWorld } from "../sim/world.js";

/** @typedef {import("./recording.js").Recording} Recording */
/** @typedef {import("./recording.js").Table} Table */
/** @typedef {import("../sim/se2.js").Pose2} Pose2 */

/** Durability transient local, as tf_static publishers offer it (Humble's YAML layout). */
export const TF_STATIC_QOS = [
  "- history: 3",
  "  depth: 0",
  "  reliability: 1",
  "  durability: 1",
  "  deadline:",
  "    sec: 2147483647",
  "    nsec: 4294967295",
  "  lifespan:",
  "    sec: 2147483647",
  "    nsec: 4294967295",
  "  liveliness: 1",
  "  liveliness_lease_duration:",
  "    sec: 2147483647",
  "    nsec: 4294967295",
  "  avoid_ros_namespace_conventions: false",
].join("\n");

/** Topics and types the export writes, in channel order. */
export const BAG_TOPICS = Object.freeze([
  ["/robot_studio/source", "std_msgs/msg/String"],
  ["/tf_static", "tf2_msgs/msg/TFMessage"],
  ["/tf", "tf2_msgs/msg/TFMessage"],
  ["/odom", "nav_msgs/msg/Odometry"],
  ["/scan", "sensor_msgs/msg/LaserScan"],
  ["/imu", "sensor_msgs/msg/Imu"],
  ["/joint_states", "sensor_msgs/msg/JointState"],
  ["/cmd_vel", "geometry_msgs/msg/Twist"],
  ["/firmware/wheel_state", "std_msgs/msg/Float64MultiArray"],
  ["/ground_truth", "nav_msgs/msg/Odometry"],
]);

/** Seconds added to the recording's time in the bag. */
export const BAG_CLOCK_OFFSET = 1;

/** @param {number} t seconds of the recording */
export const toNs = (t) => BigInt(Math.round((t + BAG_CLOCK_OFFSET) * 1e9));

/** @param {bigint} ns */
const stamp = (ns) => ({ sec: Number(ns / 1000000000n), nanosec: Number(ns % 1000000000n) });

/** @param {number} yaw */
export const yawQuat = (yaw) => ({ x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) });

/**
 * Quaternion (x, y, z, w) from roll, pitch, yaw (URDF rpy, fixed axes X then Y then Z).
 * @param {number[]} rpy
 */
export function rpyQuat(rpy) {
  const [r, p, y] = rpy;
  const cr = Math.cos(r / 2);
  const sr = Math.sin(r / 2);
  const cp = Math.cos(p / 2);
  const sp = Math.sin(p / 2);
  const cy = Math.cos(y / 2);
  const sy = Math.sin(y / 2);
  return {
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
    w: cr * cp * cy + sr * sp * sy,
  };
}

/**
 * Velocity of a frame at `offset` (in the moving frame) from the moving
 * frame's body twist, in the offset frame's axes.
 * @param {{ vx: number, vy: number, wz: number }} tw
 * @param {Pose2} offset
 */
export function twistAt(tw, offset) {
  const vx = tw.vx - tw.wz * offset.y;
  const vy = tw.vy + tw.wz * offset.x;
  const c = Math.cos(offset.theta);
  const s = Math.sin(offset.theta);
  return { vx: c * vx + s * vy, vy: -s * vx + c * vy, wz: tw.wz };
}

/**
 * @param {any} v
 * @returns {Pose2}
 */
const pose2 = (v) =>
  v && typeof v === "object"
    ? { x: Number(v.x) || 0, y: Number(v.y) || 0, theta: Number(v.theta ?? v.yaw) || 0 }
    : { x: 0, y: 0, theta: 0 };

/**
 * What the export needs to know about the robot and the frames, from the
 * recording's summary and world (defaults for recordings without them).
 * @param {Recording} rec
 */
export function bagFrames(rec) {
  const s = rec.summary ?? {};
  const robot = s.robot ?? {};
  const rootInBase = pose2(robot.root_in_base);
  const baseHeight = Number(robot.base_height) || 0;
  return {
    baseFrame: typeof robot.base_frame === "string" ? robot.base_frame : "base_link",
    rootFrame:
      typeof robot.root_frame === "string"
        ? robot.root_frame
        : typeof robot.base_frame === "string"
          ? robot.base_frame
          : "base_link",
    drivePose: pose2(robot.drive_pose),
    rootInBase,
    rootZ: baseHeight + (Number(robot.root_in_base?.z) || 0),
    baseZ: baseHeight,
    start: rec.world ? { ...rec.world.start } : { x: 0, y: 0, theta: 0 },
    imuFrame: typeof s.sensors?.imu?.frame_id === "string" ? s.sensors.imu.frame_id : "imu_link",
    /** @type {{ parent: string, child: string, xyz: number[], rpy: number[] }[]} */
    statics: Array.isArray(robot.static_transforms) ? robot.static_transforms : [],
  };
}

/**
 * @param {Table} t
 * @param {string} c
 * @param {number} i
 */
const at = (t, c, i) => (t.col[c] ? t.col[c][i] : NaN);

/**
 * Every message of the bag, in time order.
 * @param {Recording} rec
 * @returns {{ topic: string, type: string, t: bigint, msg: unknown }[]}
 */
export function bagMessages(rec) {
  const F = bagFrames(rec);
  /** @type {{ topic: string, type: string, t: bigint, msg: unknown, k: number }[]} */
  const out = [];
  let k = 0;
  /**
   * @param {string} topic
   * @param {bigint} t
   * @param {unknown} msg
   */
  const push = (topic, t, msg) => {
    const type = /** @type {string} */ (BAG_TOPICS.find((x) => x[0] === topic)?.[1]);
    out.push({ topic, type, t, msg, k: k++ });
  };
  const startInv = inverse(F.start);
  const rootOff = F.rootInBase;
  /** Root frame pose in the odom frame from a base pose in the world. @param {Pose2} p */
  const rootInOdom = (p) => compose(compose(startInv, p), rootOff);
  /**
   * @param {string} frame
   * @param {string} child
   * @param {Pose2} p
   * @param {number} z
   * @param {bigint} t
   */
  const tf = (frame, child, p, z, t) => ({
    header: { stamp: stamp(t), frame_id: frame },
    child_frame_id: child,
    transform: { translation: { x: p.x, y: p.y, z }, rotation: yawQuat(p.theta) },
  });

  // Earliest time of any stream, for /tf_static and the source message.
  const firsts = [
    rec.trajectory?.rows ? rec.trajectory.col.t[0] : NaN,
    rec.odometry?.rows ? rec.odometry.col.t_s[0] : NaN,
    rec.scans?.items.length ? rec.scans.items[0].t : NaN,
  ].filter(Number.isFinite);
  const t0 = toNs(firsts.length ? Math.min(...firsts) : 0);
  const source = {
    simulated: rec.simulated,
    note: rec.summary?.note ?? "",
    format: rec.summary?.format ?? RECORDING_FORMAT,
    robot: rec.summary?.robot ?? null,
    world: rec.world
      ? {
          id: rec.summary?.world?.id ?? "custom",
          start: rec.world.start,
          data: JSON.parse(serializeSimWorld(rec.world)),
        }
      : null,
    route: rec.summary?.route ?? null,
    seed: rec.summary?.seed ?? null,
    sensors: rec.summary?.sensors ?? null,
    frames: {
      odom: "the robot's start pose",
      map: "the robot's start pose",
      clock: `recording time + ${BAG_CLOCK_OFFSET} s`,
    },
  };
  push("/robot_studio/source", t0, { data: JSON.stringify(source) });
  if (F.statics.length)
    push("/tf_static", t0, {
      transforms: F.statics.map((j) => ({
        header: { stamp: stamp(t0), frame_id: j.parent },
        child_frame_id: j.child,
        transform: {
          translation: { x: j.xyz[0], y: j.xyz[1], z: j.xyz[2] },
          rotation: rpyQuat(j.rpy),
        },
      })),
    });

  const tr = rec.trajectory;
  if (tr)
    for (let i = 0; i < tr.rows; i++) {
      const t = toNs(tr.col.t[i]);
      const odomW = {
        x: at(tr, "odom_x", i),
        y: at(tr, "odom_y", i),
        theta: at(tr, "odom_theta", i),
      };
      if (![odomW.x, odomW.y, odomW.theta].every(Number.isFinite)) continue;
      const o = rootInOdom(odomW);
      const transforms = [tf("odom", F.rootFrame, o, F.rootZ, t)];
      const estW = { x: at(tr, "est_x", i), y: at(tr, "est_y", i), theta: at(tr, "est_theta", i) };
      // No scans: the recording repeats the truth in est_*, which must not reach TF.
      if (rec.scans && [estW.x, estW.y, estW.theta].every(Number.isFinite)) {
        const e = rootInOdom(estW);
        transforms.push(tf("map", "odom", compose(e, inverse(o)), 0, t));
      } else transforms.push(tf("map", "odom", { x: 0, y: 0, theta: 0 }, 0, t));
      push("/tf", t, { transforms });
      const cmd = [at(tr, "cmd_vx", i), at(tr, "cmd_vy", i), at(tr, "cmd_wz", i)];
      if (cmd.every(Number.isFinite))
        push("/cmd_vel", t, {
          linear: { x: cmd[0], y: cmd[1], z: 0 },
          angular: { x: 0, y: 0, z: cmd[2] },
        });
      const truth = {
        x: at(tr, "true_x", i),
        y: at(tr, "true_y", i),
        theta: at(tr, "true_theta", i),
      };
      if ([truth.x, truth.y, truth.theta].every(Number.isFinite)) {
        const v = { vx: at(tr, "true_vx", i), vy: at(tr, "true_vy", i), wz: at(tr, "true_wz", i) };
        const inMap = compose(startInv, truth);
        push("/ground_truth", t, odometry("map", F.baseFrame, inMap, F.baseZ, v, t));
      }
    }

  const od = rec.odometry;
  if (od)
    for (let i = 0; i < od.rows; i++) {
      const t = toNs(od.col.t_s[i]);
      // The firmware's pose is the drive frame from where it started; the
      // message carries the root frame in the odom frame (the base's start).
      const pd = { x: od.col.x[i], y: od.col.y[i], theta: od.col.theta[i] };
      const base = compose(compose(F.drivePose, pd), inverse(F.drivePose));
      const root = compose(base, rootOff);
      const vd = { vx: od.col.vx[i], vy: od.col.vy[i], wz: od.col.wz[i] };
      const vb = twistAt(vd, inverse(F.drivePose));
      const vr = twistAt(vb, rootOff);
      push("/odom", t, odometry("odom", F.rootFrame, root, F.rootZ, vr, t));
    }

  if (rec.scans) {
    const h = rec.scans.header;
    for (const s of rec.scans.items) {
      const t = toNs(s.t);
      const n = s.ranges.length;
      push("/scan", t, {
        header: { stamp: stamp(t), frame_id: h.frame_id },
        angle_min: h.angle_min,
        angle_max: h.angle_min + (n - 1) * h.angle_increment,
        angle_increment: h.angle_increment,
        time_increment: 0,
        scan_time: h.scan_time,
        range_min: h.range_min,
        range_max: h.range_max,
        ranges: Array.from(s.ranges, (r) => (Number.isNaN(r) ? Infinity : r)),
        intensities: [],
      });
    }
  }
  const im = rec.imu;
  if (im)
    for (let i = 0; i < im.rows; i++) {
      const t = toNs(im.col.t[i]);
      push("/imu", t, {
        header: { stamp: stamp(t), frame_id: F.imuFrame },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
        orientation_covariance: [-1, 0, 0, 0, 0, 0, 0, 0, 0],
        angular_velocity: { x: im.col.gx[i], y: im.col.gy[i], z: im.col.gz[i] },
        angular_velocity_covariance: new Array(9).fill(0),
        linear_acceleration: { x: im.col.ax[i], y: im.col.ay[i], z: im.col.az[i] },
        linear_acceleration_covariance: new Array(9).fill(0),
      });
    }
  const js = rec.jointStates;
  if (js) {
    const names = js.columns.filter((c) => c.startsWith("pos:")).map((c) => c.slice(4));
    for (let i = 0; i < js.rows; i++) {
      const t = toNs(js.col.t[i]);
      push("/joint_states", t, {
        header: { stamp: stamp(t), frame_id: "" },
        name: names,
        position: names.map((n) => at(js, `pos:${n}`, i)),
        velocity: names.map((n) => at(js, `vel:${n}`, i)),
        effort: [],
      });
    }
  }
  const ws = rec.wheelState;
  if (ws) {
    const label = ws.columns.join(",");
    for (let i = 0; i < ws.rows; i++)
      push("/firmware/wheel_state", toNs(ws.col.t_us[i] / 1e6), {
        layout: {
          dim: [{ label, size: ws.columns.length, stride: ws.columns.length }],
          data_offset: 0,
        },
        data: ws.columns.map((c) => ws.col[c][i]),
      });
  }
  out.sort((a, b) => (a.t < b.t ? -1 : a.t > b.t ? 1 : a.k - b.k));
  return out;
}

/**
 * @param {string} frame
 * @param {string} child
 * @param {Pose2} p
 * @param {number} z
 * @param {{ vx: number, vy: number, wz: number }} v
 * @param {bigint} t
 */
function odometry(frame, child, p, z, v, t) {
  return {
    header: { stamp: stamp(t), frame_id: frame },
    child_frame_id: child,
    pose: {
      pose: { position: { x: p.x, y: p.y, z }, orientation: yawQuat(p.theta) },
      covariance: new Array(36).fill(0),
    },
    twist: {
      twist: {
        linear: { x: finiteOr0(v.vx), y: finiteOr0(v.vy), z: 0 },
        angular: { x: 0, y: 0, z: finiteOr0(v.wz) },
      },
      covariance: new Array(36).fill(0),
    },
  };
}

/** @param {number} v */
const finiteOr0 = (v) => (Number.isFinite(v) ? v : 0);

/**
 * The bag: `<name>/<name>_0.mcap` and `<name>/metadata.yaml`.
 * @param {Recording} rec
 * @param {{ name?: string, library?: string }} [opts]
 * @returns {{ files: Record<string, Uint8Array | string>, counts: Record<string, number>, messages: number, bytes: number }}
 */
export function recordingToBag(rec, opts = {}) {
  const name = (opts.name ?? "robot_studio_bag").replace(/[^A-Za-z0-9_.-]/g, "_");
  const msgs = bagMessages(rec);
  const w = new McapWriter({ library: opts.library ?? "robot-studio" });
  /** @type {Map<string, number>} */
  const channel = new Map();
  /** @type {Record<string, number>} */
  const counts = {};
  for (const [topic, type] of BAG_TOPICS) {
    if (!msgs.some((m) => m.topic === topic)) continue;
    const meta = { offered_qos_profiles: topic === "/tf_static" ? TF_STATIC_QOS : "" };
    channel.set(topic, w.addChannel(topic, type, schemaText(type), meta));
    counts[topic] = 0;
  }
  for (const m of msgs) {
    w.addMessage(/** @type {number} */ (channel.get(m.topic)), m.t, encodeCdr(m.type, m.msg));
    counts[m.topic]++;
  }
  w.addMetadata("robot_studio", {
    simulated: String(rec.simulated),
    note: String(rec.summary?.note ?? ""),
  });
  const mcap = w.finish();
  const file = `${name}_0.mcap`;
  const start = msgs.length ? msgs[0].t : 0n;
  const end = msgs.length ? msgs[msgs.length - 1].t : 0n;
  const files = {
    [`${name}/${file}`]: mcap,
    [`${name}/metadata.yaml`]: metadataYaml(file, counts, start, end, msgs.length),
  };
  return { files, counts, messages: msgs.length, bytes: mcap.length };
}

/** @param {string} s */
const yamlString = (s) => (s === "" ? '""' : JSON.stringify(s)); // JSON string escapes are valid YAML double-quoted escapes

/**
 * rosbag2's metadata.yaml (bag format version 5, as Humble writes it).
 * @param {string} file
 * @param {Record<string, number>} counts
 * @param {bigint} start ns
 * @param {bigint} end ns
 * @param {number} total
 */
export function metadataYaml(file, counts, start, end, total) {
  const type = (/** @type {string} */ topic) => BAG_TOPICS.find((x) => x[0] === topic)?.[1] ?? "";
  const lines = [
    "rosbag2_bagfile_information:",
    "  version: 5",
    "  storage_identifier: mcap",
    "  duration:",
    `    nanoseconds: ${end - start}`,
    "  starting_time:",
    `    nanoseconds_since_epoch: ${start}`,
    `  message_count: ${total}`,
    "  topics_with_message_count:",
  ];
  for (const [topic, n] of Object.entries(counts))
    lines.push(
      "    - topic_metadata:",
      `        name: ${topic}`,
      `        type: ${type(topic)}`,
      "        serialization_format: cdr",
      `        offered_qos_profiles: ${yamlString(topic === "/tf_static" ? TF_STATIC_QOS : "")}`,
      `      message_count: ${n}`,
    );
  lines.push(
    '  compression_format: ""',
    '  compression_mode: ""',
    "  relative_file_paths:",
    `    - ${file}`,
    "  files:",
    `    - path: ${file}`,
    "      starting_time:",
    `        nanoseconds_since_epoch: ${start}`,
    "      duration:",
    `        nanoseconds: ${end - start}`,
    `      message_count: ${total}`,
  );
  return lines.join("\n") + "\n";
}
