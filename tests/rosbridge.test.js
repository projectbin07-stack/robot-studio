// The rosbridge client, the live session and the dead-man switch, against
// the in-repo fake rosbridge server (real WebSocket connections over
// 127.0.0.1) replaying message fixtures generated from a simulated run.

import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { FakeServer, loadFixture } from "../scripts/fake-rosbridge.mjs";
import { RosbridgeClient, validName } from "../src/core/ros/rosbridge.js";
import { LiveSession } from "../src/core/ros/live.js";
import { CmdVelGate, ZERO, disarmOnDisconnect } from "../src/core/ros/deadman.js";
import { RosJsonError, parseRosJson } from "../src/core/ros/json.js";
import { checkRosbridgeUrl, isLoopback } from "../src/core/ros/url.js";
import * as M from "../src/core/ros/msgs.js";
import { generate } from "../src/core/builder/templates.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import { simRobot } from "../src/core/sim/robot.js";
import { fwInverse } from "../src/core/sim/firmware.js";

const fixtures = fileURLToPath(new URL("./fixtures/rosbridge/", import.meta.url));
const FIXTURE = loadFixture(`${fixtures}sim_mecanum.jsonl`);
const DESCRIPTION = readFileSync(`${fixtures}robot_description.urdf`, "utf8");
const WS = /** @type {any} */ (globalThis).WebSocket;

function mecanum() {
  const doc = generate("mecanum", {});
  const r = simRobot(modelFromDoc(doc), doc);
  if (!r.ok) throw new Error(r.reason);
  return r.robot;
}

/**
 * @param {() => boolean} cond
 * @param {number} [ms]
 * @param {string | (() => string)} [what]
 */
async function waitFor(cond, ms = 4000, what = "condition") {
  const t0 = Date.now();
  while (!cond()) {
    if (Date.now() - t0 > Math.max(ms, 15000))
      throw new Error(`timed out waiting for ${typeof what === "function" ? what() : what}`);
    await new Promise((r) => setTimeout(r, 10));
  }
}

/** @param {Partial<import("../scripts/fake-rosbridge.mjs").FakeOptions>} [o] */
async function server(o = {}) {
  const s = new FakeServer({ fixture: FIXTURE, description: DESCRIPTION, speed: 4, ...o });
  const port = await s.listen();
  return { s, url: `ws://127.0.0.1:${port}/` };
}

test("rosbridge URL: ws/wss only, no credentials, port completed, mixed-content warning", () => {
  const a = checkRosbridgeUrl(" 192.168.1.20 ");
  assert.ok(a.ok && a.url === "ws://192.168.1.20:9090/");
  const b = checkRosbridgeUrl("wss://robot.local:9443/ros", "https:");
  assert.ok(b.ok && b.url === "wss://robot.local:9443/ros" && b.warnings.length === 0);
  const c = checkRosbridgeUrl("ws://192.168.1.20:9090", "https:");
  assert.ok(c.ok && /mixed content/.test(c.warnings[0]));
  const d = checkRosbridgeUrl("ws://localhost:9090", "https:");
  assert.ok(d.ok && /local network access/.test(d.warnings[0]));
  const e = checkRosbridgeUrl("ws://127.0.0.1:9090", "http:");
  assert.ok(e.ok && e.warnings.length === 0);
  for (const [bad, why] of /** @type {[string, RegExp][]} */ ([
    ["", /enter the rosbridge address/],
    ["http://robot:9090", /only ws:\/\/ and wss:\/\//],
    ["javascript:alert(1)", /not a valid address|only ws/],
    ["ws://user:pw@robot:9090", /no user name or password/],
    ["ws://robot:9090/#x", /fragment/],
    ["ws://ro bot", /spaces or control/],
    ["ws://robot:99999", /not a valid address/],
    ["x".repeat(600), /longer than/],
  ])) {
    const r = checkRosbridgeUrl(bad);
    assert.ok(!r.ok && why.test(r.reason), `${bad}: ${JSON.stringify(r)}`);
  }
  assert.ok(isLoopback("127.0.0.1") && isLoopback("[::1]") && !isLoopback("10.0.0.1"));
  assert.ok(validName("/cmd_vel") && validName("scan") && !validName("/a b") && !validName("//x"));
});

test("JSON as rosbridge sends it: bare NaN / Infinity tokens, depth and size limits", () => {
  assert.deepEqual(parseRosJson('{"a":[1,null]}'), { a: [1, null] });
  const v = /** @type {any} */ (
    parseRosJson(
      '{"r":[NaN, Infinity,-Infinity, 1.5e2], "s":"NaN \\" x", "t":true,"f":false,"n":null,"o":{}, "e":[]}',
    )
  );
  assert.ok(Number.isNaN(v.r[0]) && v.r[1] === Infinity && v.r[2] === -Infinity && v.r[3] === 150);
  assert.equal(v.s, 'NaN " x');
  assert.deepEqual([v.t, v.f, v.n, v.o, v.e], [true, false, null, {}, []]);
  for (const bad of [
    '{"a":NaNx}',
    "[1,]",
    '{"a" 1}',
    '{"a":1,,}',
    '["NaN',
    "{1:2}",
    "[Infinity] x",
    '{"a":"\\x"} NaN',
  ])
    assert.throws(() => parseRosJson(bad), RosJsonError, bad);
  assert.throws(() => parseRosJson("{oops}"), /not JSON/);
  assert.throws(() => parseRosJson("[".repeat(70) + "]".repeat(70)), /nested deeper/);
  assert.throws(
    () => parseRosJson("[NaN," + "[".repeat(70) + "]".repeat(70) + "]"),
    /nested deeper/,
  );
  assert.throws(() => parseRosJson("[1]", { maxBytes: 2 }), /larger than 2/);
  // A "__proto__" key is data, not a prototype.
  const p = /** @type {any} */ (parseRosJson('{"__proto__":{"x":1},"v":NaN}'));
  assert.equal(Object.getPrototypeOf(p), Object.prototype);
  assert.equal(/** @type {any} */ ({}).x, undefined);
});

test("message checks: types, lengths, finite numbers; LaserScan no-return values", () => {
  const h = { stamp: { sec: 1, nanosec: 5e8 }, frame_id: "/laser" };
  const s = M.laserScan({
    header: h,
    angle_min: -1,
    angle_increment: 0.5,
    range_min: 0.1,
    range_max: 5,
    ranges: [1, null, NaN, Infinity, 0.05, 9, 5],
  });
  assert.equal(s.frame, "laser");
  assert.equal(s.stamp, 1.5);
  assert.deepEqual(
    Array.from(s.ranges, (v) => (Number.isNaN(v) ? null : v)),
    [1, null, null, null, null, null, 5],
  );
  assert.equal(s.noReturn, 5);
  assert.throws(
    () =>
      M.laserScan({
        header: h,
        angle_min: 0,
        angle_increment: 1,
        range_min: 1,
        range_max: 0.5,
        ranges: [],
      }),
    /out of order/,
  );
  assert.throws(
    () =>
      M.laserScan({
        header: h,
        angle_min: 0,
        angle_increment: 1,
        range_min: 0,
        range_max: 5,
        ranges: ["1"],
      }),
    /not a number/,
  );
  assert.throws(
    () =>
      M.laserScan({
        header: h,
        angle_min: 0,
        angle_increment: 1,
        range_min: 0,
        range_max: 5,
        ranges: new Array(M.LIMITS.beams + 1).fill(1),
      }),
    /more than/,
  );
  assert.throws(() => M.laserScan({ header: { stamp: {}, frame_id: "" } }), /stamp\.sec/);
  assert.throws(() => M.laserScan(null), /expected an object/);
  const js = M.jointState({ header: h, name: ["a", "b"], position: [1, null], velocity: [] });
  assert.deepEqual(js.name, ["a", "b"]);
  assert.ok(Number.isNaN(js.position[1]));
  assert.throws(
    () => M.jointState({ header: h, name: ["a"], position: [1, 2] }),
    /2 values for 1 names/,
  );
  assert.throws(() => M.jointState({ header: h, name: ["x".repeat(300)] }), /longer than/);
  assert.throws(() => M.jointState({ header: h, name: "a" }), /expected an array/);
  const tf = M.tfMessage({
    transforms: [
      {
        header: h,
        child_frame_id: "c",
        transform: { translation: { x: 1, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 2, w: 0 } },
      },
    ],
  });
  assert.deepEqual(tf[0].q, [0, 0, 1, 0]);
  assert.ok(Math.abs(M.yawOf(tf[0].q) - Math.PI) < 1e-12);
  assert.throws(
    () =>
      M.tfMessage({
        transforms: [
          {
            header: h,
            child_frame_id: "c",
            transform: { translation: { x: 0, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 0 } },
          },
        ],
      }),
    /zero quaternion/,
  );
  assert.throws(
    () =>
      M.tfMessage({
        transforms: [
          {
            header: h,
            child_frame_id: "c",
            transform: {
              translation: { x: Infinity, y: 0, z: 0 },
              rotation: { x: 0, y: 0, z: 0, w: 1 },
            },
          },
        ],
      }),
    /finite number/,
  );
  const od = M.odometry({
    header: h,
    child_frame_id: "base_link",
    pose: { pose: { position: { x: 1, y: 2, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } },
    twist: { twist: { linear: { x: 0.5, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0.1 } } },
  });
  assert.deepEqual(
    [od.pose, od.twist],
    [
      { x: 1, y: 2, theta: 0 },
      { vx: 0.5, vy: 0, wz: 0.1 },
    ],
  );
  const im = M.imu({
    header: h,
    linear_acceleration: { x: 0, y: 0, z: 9.8 },
    angular_velocity: { x: 0, y: 0, z: 0 },
  });
  assert.equal(im.accel[2], 9.8);
  assert.equal(M.stringMsg({ data: "<robot/>" }), "<robot/>");
  assert.throws(() => M.stringMsg({ data: 1 }), /expected a string/);
  assert.throws(
    () => M.stringMsg({ data: "x".repeat(M.LIMITS.robotDescription + 1) }),
    /longer than/,
  );
  assert.deepEqual(M.twistMsg({ vx: 1, vy: 2, wz: 3 }), {
    linear: { x: 1, y: 2, z: 0 },
    angular: { x: 0, y: 0, z: 3 },
  });
});

test("live view against the fake server: subscriptions, TF, scans, joints, robot_description", async () => {
  const { s, url } = await server();
  const client = new RosbridgeClient({ url, WebSocket: WS });
  const robot = mecanum();
  const live = new LiveSession(client, { robot, now: () => performance.now() });
  /** @type {string[]} */
  const urdfs = [];
  live.onDescription((u) => urdfs.push(u));
  try {
    client.connect();
    await waitFor(() => client.isOpen, 4000, "open");
    await waitFor(
      () => (live.topics["/scan"]?.count ?? 0) >= 3 && (live.topics["/odom"]?.count ?? 0) >= 3,
      6000,
      "scans",
    );
    const subscribed = s.received
      .filter((r) => r.op === "subscribe")
      .map((r) => r.topic)
      .sort();
    assert.deepEqual(subscribed, [
      "/imu",
      "/joint_states",
      "/odom",
      "/robot_description",
      "/scan",
      "/tf",
      "/tf_static",
    ]);
    const scanSub = s.received.find((r) => r.op === "subscribe" && r.topic === "/scan");
    assert.equal(scanSub?.raw.type, "sensor_msgs/msg/LaserScan");
    const jsSub = s.received.find((r) => r.op === "subscribe" && r.topic === "/joint_states");
    assert.equal(jsSub?.raw.throttle_rate, 50);
    // The LiDAR mount came from /tf_static and equals the URDF's.
    assert.match(live.mountFrom, /^\/tf_static \(base_link -> lidar_link\)/);
    const lidar = robot.sensors.find((x) => x.kind === "lidar2d");
    assert.ok(lidar);
    assert.ok(Math.abs(live.track.opts.mount.x - lidar.planar.x) < 1e-9);
    assert.ok(
      Math.abs(live.lidarHeight - lidar.height) < 1e-9,
      `${live.lidarHeight} vs ${lidar.height}`,
    );
    // Scans with null and with bare Infinity both arrive, with no-return beams as NaN.
    const f = live.frame();
    assert.ok(f.scan && f.scan.ranges.some(Number.isNaN));
    assert.equal(live.topics["/scan"].rejected, 0);
    assert.ok(f.trails.odom && f.trails.est, "odometry and scan matching run on the live data");
    assert.ok(Object.keys(f.q).includes("front_left_wheel_joint"));
    assert.ok(live.history.wheels.length > 0);
    // robot_description, by topic and through rosapi.
    assert.equal(urdfs[0], DESCRIPTION);
    assert.equal(await live.requestDescription(undefined, 20000), DESCRIPTION);
    await assert.rejects(
      client.callService("/no_such_service", {}, { timeoutMs: 20000 }),
      /service call failed: unknown service/,
    );
    await assert.rejects(client.callService("bad name", {}), /not a valid service name/);
    const sum = live.topicSummary();
    const scanRow = sum.find((r) => r.topic === "/scan");
    assert.ok(scanRow && scanRow.hz > 0 && scanRow.age !== null);
    // A message that fails its checks is counted, not shown. (Handed to the client
    // directly: the network path of bad messages is the next test's.)
    client.receive(
      JSON.stringify({
        op: "publish",
        topic: "/scan",
        msg: { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "l" }, ranges: "x" },
      }),
    );
    await waitFor(
      () => live.topics["/scan"].rejected >= 1,
      2000,
      () =>
        `rejected scan (${JSON.stringify({ stats: client.stats, scan: { ...live.topics["/scan"], times: [] } })})`,
    );
    assert.match(live.topics["/scan"].error, /range_min/);
    live.dispose();
    await waitFor(
      () => s.received.some((r) => r.op === "unsubscribe" && r.topic === "/scan"),
      2000,
      "unsubscribe",
    );
  } finally {
    client.close();
    await s.close();
  }
});

test("malformed, oversized, binary and unknown messages are dropped and counted", async () => {
  const { s, url } = await server({ fixture: [] });
  const client = new RosbridgeClient({ url, WebSocket: WS, maxMessageBytes: 10000 });
  /** @type {string[]} */
  const logs = [];
  client.onLog((level, text) => logs.push(`${level}: ${text}`));
  /** @type {unknown[]} */
  const got = [];
  client.subscribe("/x", "std_msgs/msg/String", (m) => got.push(m));
  try {
    client.connect();
    await waitFor(() => client.isOpen);
    await waitFor(() => s.received.some((r) => r.op === "subscribe"));
    s.broadcast("{not json");
    s.broadcast(JSON.stringify({ op: "publish", topic: "/x", msg: { data: "y".repeat(20000) } }));
    s.broadcastBinary(Buffer.from([1, 2, 3]));
    s.broadcast(JSON.stringify({ op: "png", data: "" }));
    s.broadcast("[1,2]");
    s.broadcast(JSON.stringify({ op: "publish", topic: 5, msg: {} }));
    s.broadcast(JSON.stringify({ op: "publish", topic: "/other", msg: {} }));
    s.broadcast(JSON.stringify({ op: "service_response", id: "nobody", values: {} }));
    s.broadcast(JSON.stringify({ op: "status", level: "warning", msg: "something happened" }));
    s.broadcast('{"op":"publish","topic":"/x","msg":{"data":"ok","v":NaN}}');
    // Fragments: reassembled in any order; bad ones dropped.
    const whole = JSON.stringify({ op: "publish", topic: "/x", msg: { data: "fragmented" } });
    const parts = [whole.slice(0, 10), whole.slice(10, 30), whole.slice(30)];
    for (const n of [2, 0, 1])
      s.broadcast(JSON.stringify({ op: "fragment", id: "f1", data: parts[n], num: n, total: 3 }));
    s.broadcast(JSON.stringify({ op: "fragment", id: "f2", data: "x", num: 5, total: 3 }));
    s.broadcast(JSON.stringify({ op: "fragment", id: "f3", data: "{", num: 0, total: 2 }));
    s.broadcast(JSON.stringify({ op: "fragment", id: "f3", data: "x", num: 0, total: 9 }));
    s.broadcast(JSON.stringify({ op: "fragment", id: "f4", data: "{bad", num: 0, total: 1 }));
    s.broadcast(
      JSON.stringify({ op: "fragment", id: "f5", data: "y".repeat(6000), num: 0, total: 3 }),
    );
    s.broadcast(
      JSON.stringify({ op: "fragment", id: "f5", data: "y".repeat(6000), num: 1, total: 3 }),
    );
    await waitFor(() => got.length === 2, 3000, "the two good messages");
    await waitFor(() => client.stats.dropped.oversized >= 2, 3000, "oversized");
    const d = client.stats.dropped;
    assert.deepEqual(
      got.map((m) => /** @type {any} */ (m).data),
      ["ok", "fragmented"],
    );
    assert.ok(Number.isNaN(/** @type {any} */ (got[0]).v));
    assert.equal(d.malformed, 2); // "{not json" and the bad single fragment
    assert.equal(d.binary, 1);
    assert.equal(d.unknownOp, 1);
    assert.equal(d.unsubscribed, 1);
    assert.equal(d.invalid, 5); // the array, topic 5, unknown service id, two bad fragments
    assert.ok(logs.some((l) => l === "warning: rosbridge: something happened"));
    assert.ok(logs.some((l) => /dropped a 2\d{4}-byte message \(limit 10000\)/.test(l)));
    // The client keeps working after all that.
    s.broadcast(JSON.stringify({ op: "publish", topic: "/x", msg: { data: "still fine" } }));
    await waitFor(() => got.length === 3);
    assert.throws(() => client.subscribe("bad topic", "x", () => {}), /not a valid topic/);
    assert.throws(() => client.publish("/nowhere", {}), /advertise \/nowhere before publishing/);
    assert.throws(() => client.advertise("x y", "t"), /not a valid topic/);
  } finally {
    client.close();
    await s.close();
  }
});

test("reconnects after a dropped connection, subscribes and advertises again; driving is disarmed", async () => {
  const { s, url } = await server();
  const client = new RosbridgeClient({ url, WebSocket: WS, reconnectMs: 50, maxReconnectMs: 200 });
  /** @type {string[]} */
  const states = [];
  client.onState((st) => states.push(st));
  /** @type {number} */
  let scans = 0;
  client.subscribe("/scan", "sensor_msgs/msg/LaserScan", () => scans++);
  client.advertise("/cmd_vel", "geometry_msgs/msg/Twist");
  const robot = mecanum();
  let now = 0;
  const gate = new CmdVelGate({
    geometry: robot.geometry,
    maxWheelSpeed: robot.maxWheelSpeed,
    publish: (t) => client.publish("/cmd_vel", M.twistMsg(t)),
    now: () => now,
  });
  disarmOnDisconnect(gate, client);
  try {
    client.connect();
    client.connect(); // a second call while connecting does nothing
    await waitFor(() => client.isOpen && scans > 2, 6000, "first scans");
    gate.arm({ confirmed: true, fraction: 0.5 });
    gate.update({ vx: 0.1, vy: 0, wz: 0 });
    assert.equal(gate.state, "driving");
    s.dropAll();
    await waitFor(
      () => states.includes("reconnecting"),
      3000,
      `reconnecting (${states.join(",")}, ${client.state})`,
    );
    assert.equal(gate.state, "disarmed");
    assert.match(gate.reason, /connection reconnecting/);
    const before = scans;
    await waitFor(() => client.isOpen && scans > before + 2, 6000, "scans after the reconnect");
    assert.equal(s.connections, 2);
    assert.equal(s.received.filter((r) => r.op === "subscribe" && r.topic === "/scan").length, 2);
    assert.equal(
      s.received.filter((r) => r.op === "advertise" && r.topic === "/cmd_vel").length,
      2,
    );
    // The server going away entirely: the client keeps retrying, then gives up on close().
    await s.close();
    await waitFor(() => client.state === "reconnecting", 3000);
    await new Promise((r) => setTimeout(r, 300));
    assert.equal(client.state, "reconnecting");
    client.close();
    assert.equal(client.state, "closed");
    await assert.rejects(client.callService("/x", {}), /not connected/);
    now += 1;
  } finally {
    client.close();
  }
  // A client that is told not to reconnect reports closed.
  const { s: s2, url: url2 } = await server({ fixture: [] });
  const once = new RosbridgeClient({ url: url2, WebSocket: WS, reconnect: false });
  once.connect();
  await waitFor(() => once.isOpen);
  s2.dropAll();
  await waitFor(() => once.state === "closed");
  await s2.close();
  // A constructor that throws (a bad URL in a browser) is retried, not fatal.
  const broken = new RosbridgeClient({
    url: "ws://x",
    WebSocket: /** @type {any} */ (
      class {
        constructor() {
          throw new Error("boom");
        }
      }
    ),
    reconnect: false,
  });
  broken.connect();
  assert.equal(broken.state, "closed");
});

test("dead-man switch timing on a fake clock: opt-in, rate, release, blur, hidden, stale input, speed limit", () => {
  const robot = mecanum();
  let now = 1000;
  /** @type {{ t: number, twist: import("../src/core/sim/firmware.js").Twist }[]} */
  const sent = [];
  let up = true;
  const gate = new CmdVelGate({
    geometry: robot.geometry,
    maxWheelSpeed: robot.maxWheelSpeed,
    publish: (twist) => {
      sent.push({ t: now, twist });
      return up;
    },
    now: () => now,
  });
  /** @type {string[]} */
  const changes = [];
  gate.onChange((st, why) => changes.push(`${st}: ${why}`));
  const drive = { vx: 0.3, vy: 0, wz: 0 };
  // Nothing before the opt-in.
  gate.update(drive);
  gate.tick();
  assert.equal(sent.length, 0);
  assert.throws(() => gate.arm({ confirmed: false, fraction: 0.5 }), /explicit confirmation/);
  assert.throws(
    () => gate.arm({ confirmed: true, fraction: 1.5 }),
    /at most the robot's wheel limit/,
  );
  assert.throws(() => gate.arm({ confirmed: true, fraction: 0 }), /more than 0/);
  gate.arm({ confirmed: true, fraction: 1 });
  // Held: the first command goes out at once, then one per 100 ms.
  gate.update(drive);
  assert.equal(sent.length, 1);
  for (let k = 0; k < 10; k++) {
    now += 20;
    gate.update(drive);
  }
  assert.equal(sent.length, 3); // at 0, 100, 200 ms
  assert.deepEqual(
    sent.map((x) => x.t - 1000),
    [0, 100, 200],
  );
  // Release: a zero Twist in the same call (0 ms), then two more on the next ticks.
  now += 5;
  gate.update(null);
  assert.deepEqual(sent[sent.length - 1], { t: now, twist: ZERO });
  const releasedAt = now;
  for (let k = 0; k < 5; k++) {
    now += 100;
    gate.tick();
  }
  assert.deepEqual(
    sent.slice(-3).map((x) => [x.t - releasedAt, x.twist.vx]),
    [
      [0, 0],
      [100, 0],
      [200, 0],
    ],
  );
  assert.equal(gate.state, "armed");
  // A stale input (the frame loop stopped: a frozen tab) stops at the next tick after 250 ms.
  gate.update(drive);
  const heldAt = now;
  now += 260;
  gate.tick();
  assert.deepEqual(sent[sent.length - 1].twist, ZERO);
  assert.ok(sent[sent.length - 1].t - heldAt <= 260);
  assert.match(gate.reason, /no input for 250 ms/);
  // Blur: stop at once.
  gate.update(drive);
  now += 10;
  gate.blur();
  assert.deepEqual(sent[sent.length - 1], { t: now, twist: ZERO });
  assert.match(gate.reason, /lost focus/);
  // Hidden: stop and disarm; inputs no longer move the robot.
  gate.update(drive);
  gate.hidden();
  assert.equal(gate.state, "disarmed");
  const n = sent.length;
  now += 500;
  gate.update(drive);
  gate.tick();
  assert.equal(
    sent.slice(n).every((x) => x.twist.vx === 0),
    true,
  );
  // Speed limit: wheel speeds never exceed fraction x the wheel limit, direction kept.
  gate.arm({ confirmed: true, fraction: 0.25 });
  gate.update({ vx: 5, vy: 5, wz: 5 });
  const cmd = sent[sent.length - 1].twist;
  const w = fwInverse(robot.geometry, cmd);
  assert.ok(Math.abs(Math.max(...w.map(Math.abs)) - 0.25 * robot.maxWheelSpeed) < 1e-9);
  assert.ok(Math.abs(cmd.vx - cmd.vy) < 1e-12 && Math.abs(cmd.vy - cmd.wz) < 1e-12);
  // A small command is not scaled; NaN is zero; a differential base never strafes.
  assert.deepEqual(gate.limit({ vx: 0.01, vy: 0, wz: 0 }), { vx: 0.01, vy: 0, wz: 0 });
  assert.deepEqual(gate.limit({ vx: NaN, vy: 0, wz: 0 }), ZERO);
  const diff = simRobot(modelFromDoc(generate("diff", {})), generate("diff", {}));
  assert.ok(diff.ok);
  const g2 = new CmdVelGate({
    geometry: diff.robot.geometry,
    maxWheelSpeed: 10,
    publish: () => true,
    now: () => 0,
  });
  assert.equal(g2.limit({ vx: 0, vy: 1, wz: 0 }).vy, 0);
  assert.throws(
    () =>
      new CmdVelGate({
        geometry: robot.geometry,
        maxWheelSpeed: 0,
        publish: () => true,
        now: () => 0,
      }),
    /limit is unknown/,
  );
  // Publishing that fails (connection gone) is not counted as sent.
  up = false;
  const before = gate.sent;
  now += 1000;
  gate.update({ vx: 0.1, vy: 0, wz: 0 });
  assert.equal(gate.sent, before);
  gate.disarm("done");
  gate.disarm("again");
  assert.equal(gate.reason, "again");
  assert.ok(
    changes.some((c) => c.startsWith("driving")) && changes.some((c) => c.startsWith("disarmed")),
  );
});

test("dead-man switch over a real connection: the stop reaches the server right after the release", async () => {
  const { s, url } = await server({ fixture: [] });
  const client = new RosbridgeClient({ url, WebSocket: WS });
  const robot = mecanum();
  client.advertise("/cmd_vel", "geometry_msgs/msg/Twist");
  const gate = new CmdVelGate({
    geometry: robot.geometry,
    maxWheelSpeed: robot.maxWheelSpeed,
    publish: (t) => client.publish("/cmd_vel", M.twistMsg(t)),
    now: () => performance.now(),
  });
  try {
    client.connect();
    await waitFor(() => client.isOpen && s.received.some((r) => r.op === "advertise"));
    assert.equal(s.received.find((r) => r.op === "advertise")?.raw.type, "geometry_msgs/msg/Twist");
    gate.arm({ confirmed: true, fraction: 0.5 });
    const timer = setInterval(() => gate.tick(), 10);
    const t0 = performance.now();
    while (performance.now() - t0 < 450) {
      gate.update({ vx: 0.2, vy: 0, wz: 0 });
      await new Promise((r) => setTimeout(r, 16)); // an animation frame
    }
    const releaseAt = s.now();
    gate.update(null);
    await waitFor(
      () => s.published().filter((p) => /** @type {any} */ (p.msg).linear.x === 0).length >= 3,
      2000,
      "three stops",
    );
    clearInterval(timer);
    const moves = s.published().filter((p) => /** @type {any} */ (p.msg).linear.x > 0);
    const stops = s.published().filter((p) => /** @type {any} */ (p.msg).linear.x === 0);
    // 10 Hz while held for ~450 ms: 5 commands (4 to 6 with timer jitter).
    assert.ok(moves.length >= 4 && moves.length <= 6, `${moves.length} commands`);
    assert.ok(
      stops[0].t - releaseAt < 100,
      `first stop ${stops[0].t - releaseAt} ms after the release`,
    );
    assert.deepEqual(/** @type {any} */ (stops[0].msg), {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    client.unadvertise("/cmd_vel");
    await waitFor(() => s.received.some((r) => r.op === "unadvertise"));
    client.publish = () => false;
  } finally {
    client.close();
    await s.close();
  }
});
