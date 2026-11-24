// ROS 2 Humble message definitions for the topics Robot Studio writes and
// reads, as `.msg` text (the field lines of the packages' own files, comments
// left out), a parser for that text, and the `ros2msg` schema bundle MCAP
// files carry (the type's text, then each dependency after a line of 80 "="
// and "MSG: pkg/Type", which is how rosbag2's MCAP plugin writes them).

/**
 * @typedef {object} MsgField
 * @property {string} name
 * @property {string} type primitive ("float64", "string", ...) or full name "pkg/msg/Type"
 * @property {"" | "fixed" | "sequence"} array
 * @property {number | null} length fixed size, or a sequence's bound
 */

/** @type {Readonly<Record<string, string>>} */
export const MSG_TEXT = Object.freeze({
  "builtin_interfaces/msg/Time": "int32 sec\nuint32 nanosec\n",
  "std_msgs/msg/Header": "builtin_interfaces/Time stamp\nstring frame_id\n",
  "std_msgs/msg/String": "string data\n",
  "std_msgs/msg/MultiArrayDimension": "string label\nuint32 size\nuint32 stride\n",
  "std_msgs/msg/MultiArrayLayout": "MultiArrayDimension[] dim\nuint32 data_offset\n",
  "std_msgs/msg/Float64MultiArray": "MultiArrayLayout layout\nfloat64[] data\n",
  "geometry_msgs/msg/Vector3": "float64 x\nfloat64 y\nfloat64 z\n",
  "geometry_msgs/msg/Point": "float64 x\nfloat64 y\nfloat64 z\n",
  "geometry_msgs/msg/Quaternion": "float64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n",
  "geometry_msgs/msg/Pose": "Point position\nQuaternion orientation\n",
  "geometry_msgs/msg/PoseWithCovariance": "Pose pose\nfloat64[36] covariance\n",
  "geometry_msgs/msg/Twist": "Vector3 linear\nVector3 angular\n",
  "geometry_msgs/msg/TwistWithCovariance": "Twist twist\nfloat64[36] covariance\n",
  "geometry_msgs/msg/Transform": "Vector3 translation\nQuaternion rotation\n",
  "geometry_msgs/msg/TransformStamped":
    "std_msgs/Header header\nstring child_frame_id\nTransform transform\n",
  "tf2_msgs/msg/TFMessage": "geometry_msgs/TransformStamped[] transforms\n",
  "sensor_msgs/msg/LaserScan":
    "std_msgs/Header header\nfloat32 angle_min\nfloat32 angle_max\nfloat32 angle_increment\n" +
    "float32 time_increment\nfloat32 scan_time\nfloat32 range_min\nfloat32 range_max\n" +
    "float32[] ranges\nfloat32[] intensities\n",
  "sensor_msgs/msg/Imu":
    "std_msgs/Header header\ngeometry_msgs/Quaternion orientation\n" +
    "float64[9] orientation_covariance\ngeometry_msgs/Vector3 angular_velocity\n" +
    "float64[9] angular_velocity_covariance\ngeometry_msgs/Vector3 linear_acceleration\n" +
    "float64[9] linear_acceleration_covariance\n",
  "sensor_msgs/msg/JointState":
    "std_msgs/Header header\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n",
  "nav_msgs/msg/Odometry":
    "std_msgs/Header header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\n" +
    "geometry_msgs/TwistWithCovariance twist\n",
});

export const PRIMITIVES = Object.freeze({
  bool: 1,
  byte: 1,
  char: 1,
  int8: 1,
  uint8: 1,
  int16: 2,
  uint16: 2,
  int32: 4,
  uint32: 4,
  int64: 8,
  uint64: 8,
  float32: 4,
  float64: 8,
});

/** @param {string} t */
export const isPrimitive = (t) => Object.hasOwn(PRIMITIVES, t) || t === "string";

/**
 * Full name of a field's type as written in a definition of package `pkg`.
 * @param {string} token "float64", "Header", "std_msgs/Header", "pkg/msg/Type"
 * @param {string} pkg
 */
function fullType(token, pkg) {
  if (isPrimitive(token)) return token;
  const parts = token.split("/");
  if (parts.length === 1) return token === "Header" ? "std_msgs/msg/Header" : `${pkg}/msg/${token}`;
  if (parts.length === 2) return `${parts[0]}/msg/${parts[1]}`;
  return token;
}

/** @type {Map<string, MsgField[]>} */
const parsed = new Map();

/**
 * The fields of a known type.
 * @param {string} type full name, "pkg/msg/Type"
 * @returns {MsgField[]}
 */
export function msgFields(type) {
  const hit = parsed.get(type);
  if (hit) return hit;
  const text = MSG_TEXT[type];
  if (text === undefined) throw new Error(`no definition for ${type}`);
  const pkg = type.split("/")[0];
  /** @type {MsgField[]} */
  const fields = [];
  for (const raw of text.split("\n")) {
    const line = raw.trim();
    if (!line) continue;
    const [typeToken, name] = line.split(/\s+/);
    const m = /^([A-Za-z0-9_/]+)(?:\[(\d*)\])?$/.exec(typeToken);
    if (!m || !name) throw new Error(`bad field line in ${type}: ${line}`);
    const array = m[2] === undefined ? "" : m[2] === "" ? "sequence" : "fixed";
    fields.push({
      name,
      type: fullType(m[1], pkg),
      array,
      length: array === "fixed" ? Number(m[2]) : null,
    });
  }
  parsed.set(type, fields);
  return fields;
}

/**
 * Every message type a type depends on, depth first, each once.
 * @param {string} type
 * @returns {string[]}
 */
export function dependencies(type) {
  /** @type {string[]} */
  const out = [];
  /** @param {string} t */
  const walk = (t) => {
    for (const f of msgFields(t))
      if (!isPrimitive(f.type) && !out.includes(f.type)) {
        out.push(f.type);
        walk(f.type);
      }
  };
  walk(type);
  return out;
}

/**
 * The `ros2msg` schema text of a type, as rosbag2 stores it in MCAP.
 * @param {string} type
 */
export function schemaText(type) {
  let text = MSG_TEXT[type];
  if (text === undefined) throw new Error(`no definition for ${type}`);
  for (const dep of dependencies(type))
    text += `${"=".repeat(80)}\nMSG: ${dep.replace("/msg/", "/")}\n${MSG_TEXT[dep]}`;
  return text;
}
