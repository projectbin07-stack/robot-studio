// The ROS 2 bag writer (CDR, message definitions, MCAP, the topic mapping,
// metadata.yaml) and the recording reader.

import { test } from "node:test";
import assert from "node:assert/strict";
import { generate } from "../src/core/builder/templates.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import { simRobot } from "../src/core/sim/robot.js";
import { bundledWorld } from "../src/core/sim/world.js";
import { normalizeSettings } from "../src/core/sim/settings.js";
import { createSim, step } from "../src/core/sim/sim.js";
import { createRoute } from "../src/core/sim/routes.js";
import { recordingFiles } from "../src/core/sim/record.js";
import { compose, inverse } from "../src/core/sim/se2.js";
import { encodeCdr } from "../src/core/ros/cdr.js";
import { MSG_TEXT, dependencies, msgFields, schemaText } from "../src/core/ros/msgdefs.js";
import { McapWriter } from "../src/core/ros/mcap.js";
import {
  BAG_CLOCK_OFFSET,
  BAG_TOPICS,
  bagMessages,
  recordingToBag,
  rpyQuat,
  twistAt,
  yawQuat,
} from "../src/core/ros/bag.js";
import {
  MAX_BEAMS,
  RecordingError,
  isSimulated,
  parseCsv,
  parseRecording,
  parseScans,
} from "../src/core/ros/recording.js";
import { decodeCdr, readMcap } from "./ros-helpers.js";

/**
 * A logged simulation run's files.
 * @param {string} template
 * @param {{ seconds?: number, world?: string, route?: string, settings?: Record<string, any> }} [o]
 */
function run(template, o = {}) {
  const doc = generate(template, {});
  const model = modelFromDoc(doc);
  const r = simRobot(model, doc);
  if (!r.ok) throw new Error(r.reason);
  const world = /** @type {import("../src/core/sim/world.js").SimWorld} */ (
    bundledWorld(o.world ?? "office")
  );
  const sim = createSim(r.robot, world, normalizeSettings(o.settings ?? {}), { log: true });
  const route = createRoute(o.route ?? "demo", r.robot, world);
  const steps = Math.round((o.seconds ?? 3) / 0.01);
  for (let k = 0; k < steps; k++) step(sim, route.next(sim.drive, sim.t).twist);
  return {
    sim,
    doc,
    model,
    files: recordingFiles(sim, { route: o.route ?? "demo", input: template, worldId: world.id }),
  };
}

const hex = (/** @type {Uint8Array} */ b) => Buffer.from(b).toString("hex");

test("CDR: encapsulation header, alignment from after it, strings with NUL, sequences and fixed arrays", () => {
  // Twist: 6 float64 straight after the 4-byte header.
  const tw = encodeCdr("geometry_msgs/msg/Twist", {
    linear: { x: 1, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: -0.5 },
  });
  assert.equal(tw.length, 4 + 48);
  assert.equal(hex(tw.subarray(0, 12)), "00010000" + "000000000000f03f");
  assert.equal(new DataView(tw.buffer).getFloat64(44, true), -0.5);
  // Header: int32, uint32, then the string's uint32 length (counting the NUL), bytes and NUL.
  const h = encodeCdr("std_msgs/msg/Header", { stamp: { sec: 1, nanosec: 2 }, frame_id: "ab" });
  assert.equal(hex(h), "00010000" + "01000000" + "02000000" + "03000000" + "616200");
  // LaserScan: the float32s after the 19-byte header start at the next multiple of 4.
  const scan = encodeCdr("sensor_msgs/msg/LaserScan", {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "ab" },
    angle_min: 0.5,
    ranges: [1, Infinity],
    intensities: [],
  });
  const d = new DataView(scan.buffer);
  assert.equal(scan[19], 0); // padding
  assert.equal(d.getFloat32(20, true), 0.5);
  assert.equal(d.getUint32(48, true), 2); // ranges count after 7 float32
  assert.equal(d.getFloat32(56, true), Infinity);
  assert.equal(d.getUint32(60, true), 0);
  assert.equal(scan.length, 64);
  // Odometry's covariance is float64[36] with no count; after a string the float64s align to 8.
  const od = encodeCdr("nav_msgs/msg/Odometry", {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "odom" },
    child_frame_id: "b",
    pose: { pose: { orientation: { w: 1 } }, covariance: new Array(36).fill(0) },
    twist: { covariance: new Array(36).fill(0) },
  });
  assert.deepEqual(decodeCdr("nav_msgs/msg/Odometry", od).pose.pose.orientation, {
    x: 0,
    y: 0,
    z: 0,
    w: 1,
  });
  // Every byte the decoder reads back equals what was written, for each type.
  const js = { header: { stamp: { sec: 3, nanosec: 4 }, frame_id: "" }, name: ["a", "bc"] };
  const jsv = { ...js, position: [1.5, -2], velocity: [0.25, 0], effort: [] };
  assert.deepEqual(
    decodeCdr("sensor_msgs/msg/JointState", encodeCdr("sensor_msgs/msg/JointState", jsv)),
    jsv,
  );
  assert.throws(
    () => encodeCdr("nav_msgs/msg/Odometry", { pose: { covariance: [1] } }),
    /float64\[36\] needs 36 values, got 1/,
  );
  assert.throws(
    () => encodeCdr("builtin_interfaces/msg/Time", { sec: 1.5 }),
    /expected an integer/,
  );
  assert.throws(() => encodeCdr("geometry_msgs/msg/Vector3", { x: "1" }), /expected a number/);
  assert.throws(() => encodeCdr("x/msg/Nope", {}), /no definition for x\/msg\/Nope/);
  // Every primitive kind.
  const all = encodeCdr("std_msgs/msg/MultiArrayDimension", { label: "", size: 7, stride: 0 });
  assert.equal(hex(all), "00010000" + "01000000" + "00" + "000000" + "07000000" + "00000000");
});

test("message definitions: Humble field lists, dependencies and the ros2msg bundle rosbag2 writes", () => {
  assert.deepEqual(
    msgFields("sensor_msgs/msg/LaserScan").map((f) => `${f.type}${f.array ? "[]" : ""} ${f.name}`),
    [
      "std_msgs/msg/Header header",
      "float32 angle_min",
      "float32 angle_max",
      "float32 angle_increment",
      "float32 time_increment",
      "float32 scan_time",
      "float32 range_min",
      "float32 range_max",
      "float32[] ranges",
      "float32[] intensities",
    ],
  );
  assert.deepEqual(dependencies("nav_msgs/msg/Odometry"), [
    "std_msgs/msg/Header",
    "builtin_interfaces/msg/Time",
    "geometry_msgs/msg/PoseWithCovariance",
    "geometry_msgs/msg/Pose",
    "geometry_msgs/msg/Point",
    "geometry_msgs/msg/Quaternion",
    "geometry_msgs/msg/TwistWithCovariance",
    "geometry_msgs/msg/Twist",
    "geometry_msgs/msg/Vector3",
  ]);
  const s = schemaText("tf2_msgs/msg/TFMessage");
  assert.ok(
    s.startsWith(
      "geometry_msgs/TransformStamped[] transforms\n" +
        "=".repeat(80) +
        "\nMSG: geometry_msgs/TransformStamped\n",
    ),
  );
  assert.ok(s.includes("\nMSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"));
  assert.equal(msgFields("geometry_msgs/msg/PoseWithCovariance")[1].length, 36);
  for (const t of Object.keys(MSG_TEXT)) assert.ok(msgFields(t).length > 0, t);
  assert.throws(() => schemaText("a/msg/B"), /no definition/);
});

test("MCAP writer: chunks with message indexes, summary, statistics, metadata, and every CRC", () => {
  const w = new McapWriter({ library: "test", chunkSize: 200 });
  const a = w.addChannel("/a", "std_msgs/msg/String", schemaText("std_msgs/msg/String"));
  const b = w.addChannel("/b", "std_msgs/msg/String", schemaText("std_msgs/msg/String"), {
    offered_qos_profiles: "x",
  });
  for (let i = 0; i < 30; i++)
    w.addMessage(
      i % 3 ? a : b,
      BigInt(1000 + i),
      encodeCdr("std_msgs/msg/String", { data: `m${i}` }),
    );
  assert.throws(() => w.addMessage(a, 5n, new Uint8Array(4)), /log-time order/);
  assert.throws(() => w.addMessage(9, 2000n, new Uint8Array(4)), /unknown channel/);
  assert.throws(() => w.addChannel("/c", "std_msgs/msg/String", ""), /before the first message/);
  w.addMetadata("robot_studio", { simulated: "true" });
  const bytes = w.finish();
  assert.throws(() => w.finish(), /already/);
  const m = readMcap(bytes);
  assert.equal(m.profile, "ros2");
  assert.equal(m.schemas.size, 1);
  assert.deepEqual(
    [...m.channels.values()].map((c) => c.topic),
    ["/a", "/b"],
  );
  assert.equal(m.channels.get(b)?.metadata.offered_qos_profiles, "x");
  assert.equal(m.messages.length, 30);
  assert.ok(m.chunks > 1 && m.chunks === m.chunkIndexes, `${m.chunks} chunks`);
  assert.deepEqual(
    m.messages.map((x) => decodeCdr("std_msgs/msg/String", x.data).data).slice(0, 3),
    ["m0", "m1", "m2"],
  );
  // Sequence numbers count per channel.
  assert.deepEqual(
    m.messages
      .filter((x) => x.channel === b)
      .map((x) => x.sequence)
      .slice(0, 3),
    [0, 1, 2],
  );
  assert.deepEqual(
    m.stats && {
      n: m.stats.messageCount,
      s: m.stats.start,
      e: m.stats.end,
      md: m.stats.metadataCount,
    },
    {
      n: 30n,
      s: 1000n,
      e: 1029n,
      md: 1,
    },
  );
  assert.deepEqual(m.metadata, [{ name: "robot_studio", data: { simulated: "true" } }]);
  // An empty file is still valid.
  const empty = readMcap(new McapWriter({ library: "t" }).finish());
  assert.equal(empty.messages.length, 0);
});

test("bag export: the documented topic mapping, decoded back from the MCAP", () => {
  const { files, sim } = run("mecanum", { seconds: 3 });
  const rec = parseRecording(files);
  assert.equal(rec.simulated, true);
  const bag = recordingToBag(rec, { name: "t" });
  assert.deepEqual(Object.keys(bag.files), ["t/t_0.mcap", "t/metadata.yaml"]);
  const m = readMcap(/** @type {Uint8Array} */ (bag.files["t/t_0.mcap"]));
  const topics = [...m.channels.values()].map((c) => c.topic);
  assert.deepEqual(
    topics,
    BAG_TOPICS.map(([t]) => t),
  );
  const byTopic = (/** @type {string} */ t) => {
    const id = [...m.channels].find(([, c]) => c.topic === t)?.[0];
    const type = BAG_TOPICS.find(([x]) => x === t)?.[1] ?? "";
    return m.messages
      .filter((x) => x.channel === id)
      .map((x) => ({ t: x.logTime, msg: decodeCdr(type, x.data) }));
  };
  const offset = BigInt(BAG_CLOCK_OFFSET * 1e9);
  // Source: labelled simulated, with the world.
  const src = JSON.parse(byTopic("/robot_studio/source")[0].msg.data);
  assert.equal(src.simulated, true);
  assert.match(src.note, /Not a measurement of any real robot/);
  assert.deepEqual(src.world.start, sim.world.start);
  // /tf_static: the template's fixed joints as robot_state_publisher sends them.
  const st = byTopic("/tf_static")[0].msg.transforms;
  assert.deepEqual(
    st.map((/** @type {any} */ x) => `${x.header.frame_id}>${x.child_frame_id}`),
    sim.robot.fixedJoints.map((j) => `${j.parent}>${j.child}`),
  );
  const lidarJoint = st.find((/** @type {any} */ x) => x.child_frame_id === sim.lidar?.sensor.link);
  assert.ok(lidarJoint);
  // /scan: every scan, no return -> +inf, angle_max from the beam count, stamps on the bag clock.
  const scans = byTopic("/scan");
  assert.equal(scans.length, rec.scans?.items.length);
  const s0 = scans[5];
  const r0 = /** @type {NonNullable<typeof rec.scans>} */ (rec.scans).items[5];
  assert.equal(s0.t, BigInt(Math.round(r0.t * 1e9)) + offset);
  assert.equal(s0.msg.header.stamp.sec, Number(s0.t / 1000000000n));
  assert.equal(s0.msg.ranges.length, r0.ranges.length);
  for (let i = 0; i < r0.ranges.length; i++)
    if (Number.isNaN(r0.ranges[i])) assert.equal(s0.msg.ranges[i], Infinity);
    else assert.equal(s0.msg.ranges[i], Math.fround(r0.ranges[i]));
  assert.ok(
    Math.abs(
      s0.msg.angle_max - (s0.msg.angle_min + (r0.ranges.length - 1) * s0.msg.angle_increment),
    ) < 1e-6,
  );
  assert.equal(s0.msg.header.frame_id, sim.lidar?.sensor.link);
  // /tf: odom -> root frame at 100 Hz equals the trajectory's wheel odometry, re-expressed.
  const tf = byTopic("/tf");
  const tr = /** @type {NonNullable<typeof rec.trajectory>} */ (rec.trajectory);
  assert.equal(tf.length, tr.rows);
  const k = 150;
  const odomW = { x: tr.col.odom_x[k], y: tr.col.odom_y[k], theta: tr.col.odom_theta[k] };
  const rootIn = { x: 0, y: 0, theta: 0 }; // the templates' root sits under base_link
  const expect = compose(compose(inverse(sim.world.start), odomW), rootIn);
  const o = tf[k].msg.transforms[0];
  assert.equal(o.header.frame_id, "odom");
  assert.equal(o.child_frame_id, sim.robot.rootFrame);
  assert.ok(Math.abs(o.transform.translation.x - expect.x) < 1e-12);
  assert.ok(Math.abs(o.transform.rotation.z - Math.sin(expect.theta / 2)) < 1e-12);
  assert.equal(tf[k].msg.transforms[1].header.frame_id, "map");
  // /odom: the firmware's frames (drive frame = base on this template).
  const od = byTopic("/odom");
  assert.equal(od.length, rec.odometry?.rows);
  assert.equal(od[10].msg.pose.pose.position.x, rec.odometry?.col.x[10]);
  // /cmd_vel, /imu, /joint_states, wheel state, ground truth.
  assert.equal(byTopic("/cmd_vel")[0].msg.linear.x, tr.col.cmd_vx[0]);
  const imu = byTopic("/imu")[3].msg;
  assert.equal(imu.orientation_covariance[0], -1);
  assert.equal(imu.linear_acceleration.z, rec.imu?.col.az[3]);
  const js = byTopic("/joint_states")[0].msg;
  assert.deepEqual(
    js.name,
    sim.robot.wheels.map((w) => w.joint),
  );
  const ws = byTopic("/firmware/wheel_state")[2].msg;
  assert.equal(ws.layout.dim[0].label, rec.wheelState?.columns.join(","));
  assert.equal(ws.data[0], rec.wheelState?.col.t_us[2]);
  const gt = byTopic("/ground_truth")[k].msg;
  assert.equal(gt.header.frame_id, "map");
  const truth = compose(inverse(sim.world.start), {
    x: tr.col.true_x[k],
    y: tr.col.true_y[k],
    theta: tr.col.true_theta[k],
  });
  assert.ok(Math.abs(gt.pose.pose.position.y - truth.y) < 1e-12);
  // metadata.yaml: version 5, per-topic counts that add up, the MCAP file.
  const y = /** @type {string} */ (bag.files["t/metadata.yaml"]);
  assert.match(y, /^rosbag2_bagfile_information:\n {2}version: 5\n {2}storage_identifier: mcap\n/);
  assert.match(y, /relative_file_paths:\n {4}- t_0\.mcap\n/);
  assert.match(y, new RegExp(`message_count: ${bag.messages}\\n`));
  assert.equal(
    Object.values(bag.counts).reduce((a, b) => a + b, 0),
    m.messages.length,
  );
  assert.match(y, /durability: 1/);
  assert.equal(m.metadata[0].data.simulated, "true");
});

test("bag export on the differential template: the odometry is re-expressed from the drive frame to the base", () => {
  const { files, sim } = run("diff", { seconds: 4, route: "square", world: "lab" });
  const dp = sim.robot.drivePose;
  assert.ok(Math.hypot(dp.x, dp.y) > 0.01, "the diff template's wheel centroid is off base_link");
  const rec = parseRecording(files);
  const msgs = bagMessages(rec);
  const od = msgs.filter((x) => x.topic === "/odom");
  const tfs = msgs.filter((x) => x.topic === "/tf");
  // The /odom pose (from the firmware's drive-frame telemetry) and the /tf odom -> base
  // (from the base-frame trajectory) describe the same base. The trajectory's odometry
  // columns hold the pose of the last control step, one period behind the telemetry frame.
  let worst = 0;
  for (const o of od.slice(1)) {
    const t = tfs.find((x) => x.t === o.t + 10000000n);
    if (!t) continue;
    const a = /** @type {any} */ (o.msg).pose.pose.position;
    const b = /** @type {any} */ (t.msg).transforms[0].transform.translation;
    worst = Math.max(worst, Math.hypot(a.x - b.x, a.y - b.y));
  }
  assert.ok(worst < 2e-5, `worst ${worst}`); // float32 telemetry vs 5-decimal trajectory
  // twistAt: a pure rotation seen from an offset point.
  const v = twistAt({ vx: 0, vy: 0, wz: 2 }, { x: 0.1, y: 0, theta: Math.PI / 2 });
  assert.ok(Math.abs(v.vx - 0.2) < 1e-12 && Math.abs(v.vy) < 1e-12);
  const q = rpyQuat([0, 0, Math.PI / 2]);
  assert.ok(Math.abs(q.z - Math.SQRT1_2) < 1e-12 && Math.abs(q.w - Math.SQRT1_2) < 1e-12);
  const q2 = rpyQuat([0.3, -0.2, 0.1]);
  assert.ok(Math.abs(Math.hypot(q2.x, q2.y, q2.z, q2.w) - 1) < 1e-12);
  assert.deepEqual(yawQuat(0), { x: 0, y: 0, z: 0, w: 1 });
});

test("a recording without a LiDAR: no /scan, and map -> odom is the identity (never the truth)", () => {
  const { files } = run("mecanum", { seconds: 1 });
  const { "scans.jsonl": _scans, ...rest } = files;
  const rec = parseRecording(rest);
  assert.equal(rec.scans, null);
  const msgs = bagMessages(rec);
  assert.ok(!msgs.some((x) => x.topic === "/scan"));
  const mo = /** @type {any} */ (msgs.find((x) => x.topic === "/tf")?.msg).transforms[1];
  assert.equal(mo.header.frame_id, "map");
  assert.deepEqual(mo.transform.translation, { x: 0, y: 0, z: 0 });
  // A recording without a summary gets default frame names and is not labelled simulated.
  const bare = parseRecording({ "odometry.csv": "t_s,x,y,theta,vx,vy,wz\n0.05,0.1,0,0,0.5,0,0\n" });
  assert.equal(bare.simulated, false);
  const b = recordingToBag(bare);
  assert.ok(Object.keys(b.files)[0].startsWith("robot_studio_bag/"));
  assert.deepEqual(Object.keys(b.counts), ["/robot_studio/source", "/odom"]);
});

test("recording reader: CSV, scans (out-of-range is no return), limits and errors", () => {
  const t = parseCsv("a,b\n1,nan\n2,inf\n-inf,3\n\n", "x.csv", ["a"]);
  assert.equal(t.rows, 3);
  assert.ok(Number.isNaN(t.col.b[0]));
  assert.equal(t.col.b[1], Infinity);
  assert.equal(t.col.a[2], -Infinity);
  assert.throws(() => parseCsv("a,b\n1\n", "x.csv"), /x\.csv line 2: 1 values for 2 columns/);
  assert.throws(() => parseCsv("a,a\n", "x.csv"), /repeated/);
  assert.throws(() => parseCsv("a\nzz\n", "x.csv"), /not a number: zz/);
  assert.throws(() => parseCsv("a\n", "x.csv", ["b"]), /no column b/);
  assert.throws(() => parseCsv("", "x.csv"), /empty/);
  const head =
    '{"header":{"frame_id":"l","angle_min":0,"angle_increment":0.1,"range_min":0.2,"range_max":5,"scan_time":0.1}}';
  const s = parseScans(`${head}\n{"t":0.1,"ranges":[1,null,0.1,9,"x",3]}\n`);
  assert.equal(s.header.mount, null);
  assert.deepEqual(
    Array.from(s.items[0].ranges).map((v) => (Number.isNaN(v) ? null : v)),
    [1, null, null, null, null, 3],
  );
  assert.throws(() => parseScans(`${head}\n{"t":0.1}\n`), /needs "t" and "ranges"/);
  assert.throws(() => parseScans(`${head}\nnope\n`), /line 2: not JSON/);
  assert.throws(() => parseScans("nope\n"), /line 1: not JSON/);
  assert.throws(() => parseScans('{"x":1}\n'), /no header/);
  assert.throws(
    () => parseScans(head.replace('"angle_min":0', '"angle_min":"0"')),
    /angle_min must be a number/,
  );
  assert.throws(
    () => parseScans(`${head}\n{"t":0,"ranges":[${new Array(MAX_BEAMS + 1).fill(1).join(",")}]}\n`),
    /more than 100000 beams/,
  );
  assert.throws(() => parseScans(""), /empty/);
  assert.throws(() => parseRecording({ "imu.csv": "t,ax,ay,az,gx,gy,gz\n" }), RecordingError);
  const r = parseRecording({
    "simulation/odometry.csv": "t_s,x,y,theta,vx,vy,wz\n",
    "simulation/summary.json": "{not json",
    "simulation/world.json": "{}",
  });
  assert.equal(r.summary, null);
  assert.deepEqual(r.notes.length, 2);
  assert.equal(isSimulated({ source: { simulated: true } }), true);
  assert.equal(
    isSimulated({ source: { simulated: false }, format: "robot-studio-sim-recording/1" }),
    false,
  );
  assert.equal(isSimulated({ format: "robot-studio-sim-recording/1" }), true);
  assert.equal(isSimulated(null), false);
});
