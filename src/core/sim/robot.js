// What the simulator needs to know about a robot, read from the model the
// studio has open: the drive (wheels detected and measured by the vendored
// urdf-inspector, mecanum for four wheels, differential for two), where the
// sensors are (the poses of the links tagged as sensors, from forward
// kinematics, never hard-coded offsets), sensor parameters from Gazebo
// <sensor> tags when the URDF has them, the height of the base above the
// floor and a footprint radius for collisions.

import { forwardKinematics } from "../../../vendor/urdf-inspector/src/core/fk.js";
import {
  apply,
  column,
  compose,
  fromXyzRpy,
  invert,
} from "../../../vendor/urdf-inspector/src/core/math.js";
import { lateralSign } from "../../../vendor/urdf-inspector/src/core/wheels.js";
import { guessSensor } from "../builder/document.js";
import { DIFFERENTIAL, FW_DEFAULTS, MECANUM } from "./firmware.js";

/** @typedef {import("../model.js").LoadedModel} LoadedModel */
/** @typedef {import("../builder/document.js").RobotDoc} RobotDoc */
/** @typedef {import("../builder/document.js").ExtraElement} ExtraElement */
/** @typedef {import("../builder/document.js").SensorKind} SensorKind */
/** @typedef {import("../../../vendor/urdf-inspector/src/core/math.js").Transform} Transform */
/** @typedef {import("../../../vendor/urdf-inspector/src/core/math.js").Vec3} Vec3 */
/** @typedef {import("./firmware.js").FwGeometry} FwGeometry */
/** @typedef {import("./se2.js").Pose2} Pose2 */

/**
 * @typedef {object} SimWheel
 * @property {"FL" | "FR" | "RL" | "RR" | "L" | "R"} label
 * @property {string} joint
 * @property {string} link
 * @property {number} x centre relative to the wheel centroid, base axes (m)
 * @property {number} y
 * @property {-1 | 1} sign joint velocity = sign x wheel speed (positive drives forward)
 * @property {number} radius m
 */

/**
 * Gazebo <sensor> parameters found for a link (only the ones given).
 * @typedef {object} GazeboSensor
 * @property {string} type
 * @property {string} name
 * @property {number} [updateRate] Hz
 * @property {number} [samples]
 * @property {number} [minAngle] rad
 * @property {number} [maxAngle] rad
 * @property {number} [rangeMin] m
 * @property {number} [rangeMax] m
 * @property {number} [rangeNoise] std, m
 * @property {number} [gyroNoise] std per sample, rad/s
 * @property {number} [gyroBias] bias mean, rad/s
 * @property {number} [accelNoise] std per sample, m/s^2
 * @property {number} [accelBias] bias mean, m/s^2
 * @property {number} [hfov] rad (depth camera)
 * @property {number} [width] px
 * @property {number} [height] px
 * @property {number} [clipNear] m
 * @property {number} [clipFar] m
 */

/**
 * @typedef {object} SimSensor
 * @property {string} link
 * @property {SensorKind} kind
 * @property {Transform} mount sensor frame in the base frame (from FK)
 * @property {Pose2} planar x, y and yaw of the mount in the base frame
 * @property {number} height sensor origin above the floor (m)
 * @property {boolean} level sensor z axis within 5 degrees of vertical
 * @property {GazeboSensor | null} gazebo
 */

/**
 * @typedef {object} SimRobot
 * @property {string} name
 * @property {"mecanum" | "differential"} drive
 * @property {string} baseFrame frame the planar pose is reported for (base_link when present)
 * @property {string} rootFrame
 * @property {SimWheel[]} wheels FL, FR, RL, RR (mecanum) or L, R (differential)
 * @property {number[]} slots physical wheel index for each firmware slot FL, FR, RL, RR. A
 *   differential base has two wheels; the firmware's differential mode averages FL with RL
 *   (left) and FR with RR (right), so the left wheel fills FL and RL, the right FR and RR.
 * @property {FwGeometry} geometry what the firmware is configured with
 * @property {Pose2} drivePose the wheel centroid frame in the base frame
 * @property {number} maxWheelSpeed rad/s: smallest wheel <limit velocity>, else the firmware default
 * @property {"urdf" | "default"} maxWheelSpeedFrom
 * @property {number} baseHeight base frame origin above the floor (m)
 * @property {Transform} rootInBase
 * @property {{ x: number, y: number, radius: number }} footprint disc around the wheel centroid (drive frame) for collisions
 * @property {SimSensor[]} sensors
 * @property {Map<string, Transform>} zeroPoses link poses in the root frame at q = 0
 */

/**
 * Children of an extra element with a given name.
 * @param {ExtraElement} e
 * @param {string} name
 */
const kids = (e, name) => e.children.filter((c) => c.name === name);

/**
 * Number at a path of element names below e, or undefined.
 * @param {ExtraElement} e
 * @param {string[]} path
 * @returns {number | undefined}
 */
function numAt(e, path) {
  /** @type {ExtraElement | undefined} */
  let cur = e;
  for (const p of path) {
    cur = cur ? kids(cur, p)[0] : undefined;
    if (!cur) return undefined;
  }
  const t = cur ? collectText(cur).trim() : "";
  const v = Number(t);
  return t !== "" && Number.isFinite(v) ? v : undefined;
}

/** @param {ExtraElement} e @returns {string} */
const collectText = (e) => e.text + e.children.map(collectText).join("");

/**
 * IMU noise from an <imu> element in either SDF spelling: per axis
 * (<angular_velocity><x><noise><stddev>, SDF 1.6+) or the older
 * <noise><rate>/<accel> block, which gazebo_ros_imu plugins also use.
 * @param {ExtraElement} imu
 * @param {GazeboSensor} gs
 */
function imuNoise(imu, gs) {
  /** @type {[keyof GazeboSensor, string[]][]} */
  const fields = [
    ["gyroNoise", ["angular_velocity", "x", "noise", "stddev"]],
    ["gyroBias", ["angular_velocity", "x", "noise", "bias_mean"]],
    ["accelNoise", ["linear_acceleration", "x", "noise", "stddev"]],
    ["accelBias", ["linear_acceleration", "x", "noise", "bias_mean"]],
    ["gyroNoise", ["noise", "rate", "stddev"]],
    ["gyroBias", ["noise", "rate", "bias_mean"]],
    ["accelNoise", ["noise", "accel", "stddev"]],
    ["accelBias", ["noise", "accel", "bias_mean"]],
  ];
  for (const [k, p] of fields) {
    const v = numAt(imu, p);
    if (v !== undefined && /** @type {any} */ (gs)[k] === undefined) /** @type {any} */ (gs)[k] = v;
  }
}

/**
 * Gazebo <sensor> tags by link name, from the document's top-level
 * <gazebo reference="..."> elements (Gazebo classic and gz sim spellings),
 * plus IMU noise given to a gazebo_ros IMU plugin for a link (as
 * TurtleBot3's description does).
 * @param {ExtraElement[]} extras
 * @returns {Map<string, GazeboSensor>}
 */
export function gazeboSensors(extras) {
  /** @type {Map<string, GazeboSensor>} */
  const out = new Map();
  for (const g of extras) {
    if (g.name !== "gazebo" || !g.attrs.reference) continue;
    for (const s of kids(g, "sensor")) {
      const type = s.attrs.type ?? "";
      /** @type {GazeboSensor} */
      const gs = { type, name: s.attrs.name ?? "" };
      const rate = numAt(s, ["update_rate"]);
      if (rate !== undefined) gs.updateRate = rate;
      /** @type {[keyof GazeboSensor, string[]][]} */
      let fields = [];
      /** @type {ExtraElement | undefined} */
      let from;
      if (/ray|lidar/.test(type)) {
        from = kids(s, "ray")[0] ?? kids(s, "lidar")[0];
        fields = [
          ["samples", ["scan", "horizontal", "samples"]],
          ["minAngle", ["scan", "horizontal", "min_angle"]],
          ["maxAngle", ["scan", "horizontal", "max_angle"]],
          ["rangeMin", ["range", "min"]],
          ["rangeMax", ["range", "max"]],
          ["rangeNoise", ["noise", "stddev"]],
        ];
      } else if (type === "imu") {
        const imu = kids(s, "imu")[0];
        if (imu) imuNoise(imu, gs);
      } else if (/depth|rgbd|camera/.test(type)) {
        from = kids(s, "camera")[0];
        fields = [
          ["hfov", ["horizontal_fov"]],
          ["width", ["image", "width"]],
          ["height", ["image", "height"]],
          ["clipNear", ["clip", "near"]],
          ["clipFar", ["clip", "far"]],
        ];
      }
      if (from)
        for (const [k, p] of fields) {
          const v = numAt(from, p);
          if (v !== undefined) /** @type {any} */ (gs)[k] = v;
        }
      out.set(g.attrs.reference, gs);
    }
  }
  for (const g of extras) {
    if (g.name !== "gazebo") continue;
    for (const pl of kids(g, "plugin")) {
      if (!/imu/i.test(pl.attrs.filename ?? "")) continue;
      const body = kids(pl, "bodyName")[0] ?? kids(pl, "frameName")[0];
      const imu = kids(pl, "imu")[0];
      const link = body ? collectText(body).trim() : "";
      if (!link || !imu) continue;
      const gs = out.get(link) ?? { type: "imu", name: pl.attrs.name ?? "" };
      if (gs.type !== "imu") continue;
      imuNoise(imu, gs);
      out.set(link, gs);
    }
  }
  return out;
}

/**
 * Planar part of a transform: x, y and the yaw of its x axis.
 * @param {Transform} t
 * @returns {Pose2}
 */
export const planarOf = (t) => ({ x: t.p[0], y: t.p[1], theta: Math.atan2(t.R[3], t.R[0]) });

const COS_5DEG = Math.cos((5 * Math.PI) / 180);

/**
 * Corners of a primitive's bounding box in its own frame, or [] for meshes.
 * @param {import("../../../vendor/urdf-inspector/src/core/urdf.js").Geometry | null} g
 * @returns {Vec3[]}
 */
function corners(g) {
  if (!g) return [];
  /** @type {Vec3} */
  let h;
  if (g.type === "box") h = [g.size[0] / 2, g.size[1] / 2, g.size[2] / 2];
  else if (g.type === "cylinder") h = [g.radius, g.radius, g.length / 2];
  else if (g.type === "sphere") h = [g.radius, g.radius, g.radius];
  else return [];
  /** @type {Vec3[]} */
  const out = [];
  for (const sx of [-1, 1])
    for (const sy of [-1, 1]) for (const sz of [-1, 1]) out.push([sx * h[0], sy * h[1], sz * h[2]]);
  return out;
}

/**
 * Read a robot for simulation. Returns the robot, or the reasons it cannot
 * be driven (no wheels found, wheels not lateral, ...).
 * @param {LoadedModel} model the checked model (urdf-inspector report)
 * @param {RobotDoc | null} doc the editable document (sensor tags, Gazebo extras); null: guess sensors from link names
 * @returns {{ ok: true, robot: SimRobot, notes: string[] } | { ok: false, reason: string }}
 */
export function simRobot(model, doc) {
  const tree = model.report?.context?.tree;
  const base = model.report?.context?.wheels ?? null;
  const robotName = model.report?.robot?.name ?? doc?.name ?? "robot";
  if (!tree || tree.root === null) return { ok: false, reason: "the model could not be read" };
  if (!base)
    return {
      ok: false,
      reason: "no drive wheels found (continuous joints named *wheel* or with a cylinder link)",
    };
  /** @type {string[]} */
  const notes = [];
  /** @type {"mecanum" | "differential"} */
  let drive;
  if (base.kind === "four-wheel" && base.lx !== null && base.ly !== null) drive = MECANUM;
  else if (base.kind === "differential") drive = DIFFERENTIAL;
  else
    return {
      ok: false,
      reason: `found ${base.wheels.length} wheel(s); the simulator drives four-wheel mecanum bases (one wheel per corner) and two-wheel differential bases`,
    };
  if (base.radius === null || !(base.radius > 0))
    return { ok: false, reason: "not every wheel has a cylinder or sphere to measure its radius" };
  const order = drive === MECANUM ? ["FL", "FR", "RL", "RR"] : ["L", "R"];
  /** @type {SimWheel[]} */
  const wheels = [];
  for (const label of order) {
    const w = base.wheels.find((x) => x.label === label);
    if (!w) return { ok: false, reason: `no ${label} wheel` };
    const sign = lateralSign(w);
    if (sign === 0)
      return {
        ok: false,
        reason: `wheel joint ${w.joint} does not turn about the base's y axis (within 5 degrees)`,
      };
    wheels.push({
      label: /** @type {SimWheel["label"]} */ (label),
      joint: w.joint,
      link: w.link,
      x: w.centre[0] - base.centroid[0],
      y: w.centre[1] - base.centroid[1],
      sign,
      radius: /** @type {number} */ (w.radius ?? base.radius),
    });
  }
  const limits = base.wheels.map((w) => w.velocityLimit);
  const fromUrdf = limits.every((v) => v !== null && v > 0);
  const maxWheelSpeed = fromUrdf
    ? Math.min(.../** @type {number[]} */ (limits))
    : FW_DEFAULTS.maxWheelSpeed;
  if (!fromUrdf)
    notes.push(
      `wheel joints have no <limit velocity>; using the firmware default of ${FW_DEFAULTS.maxWheelSpeed} rad/s`,
    );
  /** @type {FwGeometry} */
  const geometry =
    drive === MECANUM
      ? {
          wheelRadius: base.radius,
          halfLength: /** @type {number} */ (base.lx),
          halfWidth: /** @type {number} */ (base.ly),
          rollerAngleDeg: 45,
          drive,
        }
      : {
          wheelRadius: base.radius,
          halfLength: 0,
          halfWidth: /** @type {number} */ (base.ly),
          rollerAngleDeg: 45,
          drive,
        };
  const poses = forwardKinematics(tree, {});
  const baseFrame = base.baseFrame;
  const basePose = /** @type {Transform} */ (poses.get(baseFrame));
  const baseInv = invert(basePose);
  const meanZ = base.wheels.reduce((s, w) => s + w.centre[2], 0) / base.wheels.length;
  const baseHeight = base.radius - meanZ;
  const rootInBase = compose(baseInv, /** @type {Transform} */ (poses.get(tree.root)));

  // Footprint: farthest collision (else visual) primitive corner from the
  // wheel centroid, in the floor plane; wheels themselves included.
  let reach = 0;
  for (const name of tree.order) {
    const link = tree.links.get(name);
    const lp = poses.get(name);
    if (!link || !lp) continue;
    const inBase = compose(baseInv, lp);
    const shapes = link.collisions.length > 0 ? link.collisions : link.visuals;
    for (const s of shapes) {
      const st = compose(inBase, fromXyzRpy(s.origin.xyz, s.origin.rpy));
      for (const c of corners(s.geometry)) {
        const p = apply(st, c);
        reach = Math.max(reach, Math.hypot(p[0] - base.centroid[0], p[1] - base.centroid[1]));
      }
    }
  }
  if (reach === 0) {
    reach = 0.2;
    notes.push("no primitive shapes to size the footprint; using a 0.2 m radius for collisions");
  }

  const gz = gazeboSensors(doc?.extras ?? []);
  /** @type {SimSensor[]} */
  const sensors = [];
  for (const name of tree.order) {
    const kind = doc ? (doc.links.find((l) => l.name === name)?.sensor ?? null) : guessSensor(name);
    if (!kind) continue;
    const mount = compose(baseInv, /** @type {Transform} */ (poses.get(name)));
    const z = column(mount.R, 2);
    sensors.push({
      link: name,
      kind,
      mount,
      planar: planarOf(mount),
      height: baseHeight + mount.p[2],
      level: z[2] >= COS_5DEG,
      gazebo: gz.get(name) ?? null,
    });
  }
  for (const s of sensors)
    if (!s.level && (s.kind === "lidar2d" || s.kind === "imu"))
      notes.push(
        s.kind === "lidar2d"
          ? `LiDAR ${s.link} is not level (z axis more than 5 degrees from vertical); it is not simulated`
          : `IMU ${s.link} is tilted; gravity appears on its x/y axes as it would on the real sensor`,
      );

  return {
    ok: true,
    notes,
    robot: {
      name: robotName,
      drive,
      baseFrame,
      rootFrame: tree.root,
      wheels,
      slots: drive === MECANUM ? [0, 1, 2, 3] : [0, 1, 0, 1],
      geometry,
      drivePose: { x: base.centroid[0], y: base.centroid[1], theta: 0 },
      maxWheelSpeed,
      maxWheelSpeedFrom: fromUrdf ? "urdf" : "default",
      baseHeight,
      rootInBase,
      footprint: { x: 0, y: 0, radius: reach },
      sensors,
      zeroPoses: poses,
    },
  };
}
