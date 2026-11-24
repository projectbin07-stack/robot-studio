// Incoming ROS messages (rosbridge's JSON form) checked and reduced to what
// the views use. Every message is untrusted: types, array lengths and
// string lengths are checked, numbers must be finite except LaserScan
// ranges, where null (rosbridge's encoding of inf and NaN), non-finite and
// out-of-range values all become NaN, "no return". A message that fails a
// check throws MessageError and is dropped by the caller.

export class MessageError extends Error {}

export const LIMITS = Object.freeze({
  names: 256, // joints per JointState
  transforms: 512, // per TFMessage
  beams: 100000, // per LaserScan
  nameLength: 256, // frame and joint names
  robotDescription: 8 * 1024 * 1024, // characters
});

/** @param {unknown} v @returns {v is Record<string, unknown>} */
const isObj = (v) => v !== null && typeof v === "object" && !Array.isArray(v);

/**
 * @param {unknown} o
 * @param {string} path
 * @returns {Record<string, unknown>}
 */
function obj(o, path) {
  if (!isObj(o)) throw new MessageError(`${path}: expected an object`);
  return o;
}

/**
 * @param {unknown} v
 * @param {string} path
 */
function num(v, path) {
  if (typeof v !== "number" || !Number.isFinite(v))
    throw new MessageError(`${path}: expected a finite number`);
  return v;
}

/**
 * @param {unknown} v
 * @param {string} path
 */
function str(v, path) {
  if (typeof v !== "string") throw new MessageError(`${path}: expected a string`);
  if (v.length > LIMITS.nameLength)
    throw new MessageError(`${path}: longer than ${LIMITS.nameLength}`);
  return v;
}

/**
 * @param {unknown} v
 * @param {string} path
 * @param {number} max
 * @returns {unknown[]}
 */
function arr(v, path, max) {
  if (!Array.isArray(v)) throw new MessageError(`${path}: expected an array`);
  if (v.length > max) throw new MessageError(`${path}: more than ${max} entries`);
  return v;
}

/**
 * @param {unknown} h
 * @param {string} path
 */
function header(h, path) {
  const o = obj(h, path);
  const st = obj(o.stamp, `${path}.stamp`);
  return {
    stamp: num(st.sec, `${path}.stamp.sec`) + num(st.nanosec, `${path}.stamp.nanosec`) * 1e-9,
    frame: str(o.frame_id ?? "", `${path}.frame_id`).replace(/^\//, ""),
  };
}

/**
 * @param {unknown} v
 * @param {string} path
 * @returns {[number, number, number]}
 */
const vec3 = (v, path) => {
  const o = obj(v, path);
  return [num(o.x, `${path}.x`), num(o.y, `${path}.y`), num(o.z, `${path}.z`)];
};

/**
 * @param {unknown} v
 * @param {string} path
 * @returns {[number, number, number, number]}
 */
function quat(v, path) {
  const o = obj(v, path);
  const q = /** @type {[number, number, number, number]} */ ([
    num(o.x, `${path}.x`),
    num(o.y, `${path}.y`),
    num(o.z, `${path}.z`),
    num(o.w, `${path}.w`),
  ]);
  const n = Math.hypot(...q);
  if (!(n > 1e-9)) throw new MessageError(`${path}: not a rotation (zero quaternion)`);
  return /** @type {[number, number, number, number]} */ (q.map((c) => c / n));
}

/** Yaw of a unit quaternion (x, y, z, w). @param {number[]} q */
export const yawOf = (q) =>
  Math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));

/**
 * sensor_msgs/JointState.
 * @param {unknown} m
 */
export function jointState(m) {
  const o = obj(m, "JointState");
  const h = header(o.header, "header");
  const name = arr(o.name, "name", LIMITS.names).map((n, i) => str(n, `name[${i}]`));
  /** @param {string} k */
  const nums = (k) => {
    const a = arr(o[k] ?? [], k, LIMITS.names);
    if (a.length !== 0 && a.length !== name.length)
      throw new MessageError(`${k}: ${a.length} values for ${name.length} names`);
    return a.map((v, i) => (v === null ? NaN : num(v, `${k}[${i}]`)));
  };
  return { stamp: h.stamp, name, position: nums("position"), velocity: nums("velocity") };
}

/**
 * tf2_msgs/TFMessage.
 * @param {unknown} m
 */
export function tfMessage(m) {
  const o = obj(m, "TFMessage");
  return arr(o.transforms, "transforms", LIMITS.transforms).map((t, i) => {
    const p = `transforms[${i}]`;
    const tt = obj(t, p);
    const h = header(tt.header, `${p}.header`);
    const tr = obj(tt.transform, `${p}.transform`);
    return {
      parent: h.frame,
      child: str(tt.child_frame_id, `${p}.child_frame_id`).replace(/^\//, ""),
      stamp: h.stamp,
      t: vec3(tr.translation, `${p}.transform.translation`),
      q: quat(tr.rotation, `${p}.transform.rotation`),
    };
  });
}

/**
 * sensor_msgs/LaserScan; ranges: NaN = no return.
 * @param {unknown} m
 */
export function laserScan(m) {
  const o = obj(m, "LaserScan");
  const h = header(o.header, "header");
  const rmin = num(o.range_min, "range_min");
  const rmax = num(o.range_max, "range_max");
  const inc = num(o.angle_increment, "angle_increment");
  if (!(rmax > rmin && rmin >= 0)) throw new MessageError("range_min / range_max out of order");
  const raw = arr(o.ranges, "ranges", LIMITS.beams);
  const ranges = new Float64Array(raw.length);
  let none = 0;
  for (let i = 0; i < raw.length; i++) {
    const v = raw[i];
    if (v !== null && typeof v !== "number") throw new MessageError(`ranges[${i}]: not a number`);
    const ok = typeof v === "number" && Number.isFinite(v) && v >= rmin && v <= rmax;
    ranges[i] = ok ? /** @type {number} */ (v) : NaN;
    if (!ok) none++;
  }
  return {
    stamp: h.stamp,
    frame: h.frame,
    angle_min: num(o.angle_min, "angle_min"),
    angle_increment: inc,
    range_min: rmin,
    range_max: rmax,
    ranges,
    noReturn: none,
  };
}

/**
 * sensor_msgs/Imu (covariances are not used and may hold NaN / null).
 * @param {unknown} m
 */
export function imu(m) {
  const o = obj(m, "Imu");
  const h = header(o.header, "header");
  return {
    stamp: h.stamp,
    frame: h.frame,
    accel: vec3(o.linear_acceleration, "linear_acceleration"),
    gyro: vec3(o.angular_velocity, "angular_velocity"),
  };
}

/**
 * nav_msgs/Odometry, planar.
 * @param {unknown} m
 */
export function odometry(m) {
  const o = obj(m, "Odometry");
  const h = header(o.header, "header");
  const pose = obj(obj(o.pose, "pose").pose, "pose.pose");
  const tw = obj(obj(o.twist, "twist").twist, "twist.twist");
  const p = vec3(pose.position, "pose.pose.position");
  const q = quat(pose.orientation, "pose.pose.orientation");
  const lin = vec3(tw.linear, "twist.twist.linear");
  const ang = vec3(tw.angular, "twist.twist.angular");
  return {
    stamp: h.stamp,
    frame: h.frame,
    child: str(o.child_frame_id, "child_frame_id").replace(/^\//, ""),
    pose: { x: p[0], y: p[1], theta: yawOf(q) },
    z: p[2],
    twist: { vx: lin[0], vy: lin[1], wz: ang[2] },
  };
}

/**
 * std_msgs/String (robot_description), bounded.
 * @param {unknown} m
 */
export function stringMsg(m) {
  const o = obj(m, "String");
  if (typeof o.data !== "string") throw new MessageError("data: expected a string");
  if (o.data.length > LIMITS.robotDescription)
    throw new MessageError(`data: longer than ${LIMITS.robotDescription} characters`);
  return o.data;
}

/**
 * geometry_msgs/Twist as rosbridge sends it.
 * @param {{ vx: number, vy: number, wz: number }} t
 */
export const twistMsg = (t) => ({
  linear: { x: t.vx, y: t.vy, z: 0 },
  angular: { x: 0, y: 0, z: t.wz },
});
