// Parametric robot templates: a four-wheel mecanum base, a differential
// base with a caster, and an N-joint serial arm, each with optional sensor
// mounts (2D LiDAR, IMU, camera, depth camera) at poses the user chooses.
//
// Each template has two writers that must agree element for element:
//   generate(params)  -> RobotDoc (what the builder edits and checks)
//   xacro(params)     -> a parametrised xacro file: every dimension, mass
//                        and pose is a xacro:property, every sensor switch
//                        a xacro:arg, wheels and arm segments are macros.
// Expanding the xacro (with Robot Studio's xacro in the tests and the
// official xacro in CI) gives the URDF exported from generate(); see
// tests/builder-templates.test.js and the builder cases of the golden corpus.
// Inertias use urdf-inspector's formulas in both writers, written in the
// same operation order so the numbers agree to the last bit.

import { inertialFromShape } from "./inertia.js";
import { deepEqual, emptyDoc, normalizeOrder, origin } from "./document.js";
import { VERSION } from "../version.js";

/** @typedef {import("./document.js").RobotDoc} RobotDoc */
/** @typedef {import("./document.js").LinkDoc} LinkDoc */
/** @typedef {import("./document.js").JointDoc} JointDoc */
/** @typedef {import("./document.js").ShapeDoc} ShapeDoc */
/** @typedef {import("./document.js").Geometry} Geometry */
/** @typedef {import("./document.js").Vec3} Vec3 */
/** @typedef {import("./document.js").SensorKind} SensorKind */
/** @typedef {import("./document.js").TemplateValue} TemplateValue */

/**
 * @typedef {object} ParamSpec
 * @property {string} key
 * @property {string} label
 * @property {"number" | "int" | "bool" | "vec3" | "text"} kind
 * @property {TemplateValue} def
 * @property {string} group
 * @property {string} [unit]
 * @property {number} [min]
 * @property {number} [max]
 */

/** @typedef {Record<string, any>} Params validated parameter values (numbers, booleans, strings, Vec3) */

/**
 * @typedef {object} Template
 * @property {string} id
 * @property {string} title
 * @property {string} description
 * @property {ParamSpec[]} params
 * @property {(p: Params) => RobotDoc} build
 * @property {(p: Params) => string} xacro
 */

export class TemplateError extends Error {}

// ---------------------------------------------------------------- sensors

/**
 * @typedef {object} SensorSpec
 * @property {SensorKind} kind
 * @property {string} key parameter prefix
 * @property {string} label
 * @property {string} link
 * @property {"box" | "cylinder"} shape
 * @property {number[]} dims box size, or [radius, length]
 * @property {number} mass kg
 * @property {string | null} optical child optical frame link, for cameras
 */

/** @type {SensorSpec[]} */
export const SENSORS = [
  {
    kind: "lidar2d",
    key: "lidar",
    label: "2D LiDAR",
    link: "lidar_link",
    shape: "cylinder",
    dims: [0.035, 0.04],
    mass: 0.17,
    optical: null,
  },
  {
    kind: "imu",
    key: "imu",
    label: "IMU",
    link: "imu_link",
    shape: "box",
    dims: [0.02, 0.02, 0.005],
    mass: 0.01,
    optical: null,
  },
  {
    kind: "camera",
    key: "camera",
    label: "Camera",
    link: "camera_link",
    shape: "box",
    dims: [0.025, 0.09, 0.025],
    mass: 0.06,
    optical: "camera_optical_frame",
  },
  {
    kind: "depth_camera",
    key: "depth_camera",
    label: "Depth camera",
    link: "depth_camera_link",
    shape: "box",
    dims: [0.03, 0.12, 0.03],
    mass: 0.1,
    optical: "depth_camera_optical_frame",
  },
];

/** REP 103 optical frame: z forward, x right, y down. */
const OPTICAL_RPY = /** @type {Vec3} */ ([-Math.PI / 2, 0, -Math.PI / 2]);

/**
 * @param {Record<string, { on: boolean, xyz: Vec3, rpy?: Vec3 }>} defaults per sensor key
 * @returns {ParamSpec[]}
 */
function sensorParams(defaults) {
  return SENSORS.flatMap((s) => {
    const d = defaults[s.key];
    return [
      {
        key: s.key,
        label: `${s.label}`,
        kind: /** @type {const} */ ("bool"),
        def: d.on,
        group: "Sensors",
      },
      {
        key: `${s.key}_xyz`,
        label: `${s.label} position`,
        kind: /** @type {const} */ ("vec3"),
        def: d.xyz,
        unit: "m",
        min: -5,
        max: 5,
        group: "Sensors",
      },
      {
        key: `${s.key}_rpy`,
        label: `${s.label} roll pitch yaw`,
        kind: /** @type {const} */ ("vec3"),
        def: d.rpy ?? [0, 0, 0],
        unit: "rad",
        min: -7,
        max: 7,
        group: "Sensors",
      },
    ];
  });
}

// ---------------------------------------------------------------- shared builders

/** @param {string} name @param {Geometry} geometry @param {Vec3} xyz @param {Vec3} rpy @param {string | null} material */
const shape = (name, geometry, xyz, rpy, material) => ({
  name,
  origin: origin(xyz, rpy),
  geometry,
  material: material ? { name: material, rgba: null } : null,
});

/**
 * A link made of one solid primitive used for visual, collision and inertia.
 * @param {string} name
 * @param {Geometry} geometry
 * @param {Vec3} xyz
 * @param {Vec3} rpy
 * @param {number} mass
 * @param {string} material
 * @param {SensorKind | null} [sensor]
 * @returns {LinkDoc}
 */
function solidLink(name, geometry, xyz, rpy, mass, material, sensor = null) {
  const vis = shape("", geometry, xyz, rpy, material);
  const col = { ...shape("", geometry, xyz, rpy, null) };
  return {
    name,
    sensor,
    inertial: inertialFromShape(col, mass),
    visuals: [vis],
    collisions: [col],
  };
}

/**
 * @param {string} name
 * @param {JointDoc["type"]} type
 * @param {string} parent
 * @param {string} child
 * @param {Vec3} xyz
 * @param {Vec3} [rpy]
 * @param {Partial<JointDoc>} [more]
 * @returns {JointDoc}
 */
const joint = (name, type, parent, child, xyz, rpy = [0, 0, 0], more = {}) => ({
  name,
  type,
  parent,
  child,
  origin: origin(xyz, rpy),
  axis: null,
  limit: null,
  dynamics: null,
  mimic: null,
  ...more,
});

/** @param {RobotDoc} doc @param {string} name @param {number[]} rgba */
const material = (doc, name, rgba) => doc.materials.push({ name, rgba });

/**
 * Add the enabled sensors (and camera optical frames) under their parents.
 * @param {RobotDoc} doc
 * @param {Params} p
 * @param {(s: SensorSpec) => string} parentOf
 */
function addSensors(doc, p, parentOf) {
  for (const s of SENSORS) {
    if (!p[s.key]) continue;
    const g =
      s.shape === "box"
        ? /** @type {Geometry} */ ({ type: "box", size: [s.dims[0], s.dims[1], s.dims[2]] })
        : /** @type {Geometry} */ ({ type: "cylinder", radius: s.dims[0], length: s.dims[1] });
    doc.links.push(solidLink(s.link, g, [0, 0, 0], [0, 0, 0], s.mass, "sensor", s.kind));
    doc.joints.push(
      joint(`${s.key}_joint`, "fixed", parentOf(s), s.link, p[`${s.key}_xyz`], p[`${s.key}_rpy`]),
    );
    if (s.optical) {
      doc.links.push({
        name: s.optical,
        sensor: null,
        inertial: null,
        visuals: [],
        collisions: [],
      });
      doc.joints.push(
        joint(`${s.key}_optical_joint`, "fixed", s.link, s.optical, [0, 0, 0], OPTICAL_RPY),
      );
    }
  }
}

// ---------------------------------------------------------------- xacro text helpers

/** @param {number} v */
const n = (v) => (Object.is(v, -0) ? "0" : String(v));
/** @param {number[]} v */
const ns = (v) => v.map(n).join(" ");

/** Inertia macros shared by the templates; formulas as in urdf-inspector's primitiveInertia. */
const INERTIA_MACROS = `
  <!-- Solid-body inertias (urdf-inspector's formulas); the inertial frame is not rotated. -->
  <xacro:macro name="box_inertial" params="m x y z xyz">
    <inertial>
      <origin xyz="\${xyz}" rpy="0 0 0"/>
      <mass value="\${m}"/>
      <inertia ixx="\${m*(y*y+z*z)/12}" ixy="0" ixz="0" iyy="\${m*(x*x+z*z)/12}" iyz="0" izz="\${m*(x*x+y*y)/12}"/>
    </inertial>
  </xacro:macro>
  <!-- Cylinder along z. -->
  <xacro:macro name="cylinder_inertial" params="m r h xyz">
    <inertial>
      <origin xyz="\${xyz}" rpy="0 0 0"/>
      <mass value="\${m}"/>
      <inertia ixx="\${m*(3*r*r+h*h)/12}" ixy="0" ixz="0" iyy="\${m*(3*r*r+h*h)/12}" iyz="0" izz="\${m*r*r/2}"/>
    </inertial>
  </xacro:macro>
  <!-- Cylinder along y (a wheel: its geometry is rotated by roll pi/2, so is its inertia). -->
  <xacro:macro name="wheel_inertial" params="m r h">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="\${m}"/>
      <inertia ixx="\${m*(3*r*r+h*h)/12}" ixy="0" ixz="0" iyy="\${m*r*r/2}" iyz="0" izz="\${m*(3*r*r+h*h)/12}"/>
    </inertial>
  </xacro:macro>
  <xacro:macro name="sphere_inertial" params="m r xyz">
    <inertial>
      <origin xyz="\${xyz}" rpy="0 0 0"/>
      <mass value="\${m}"/>
      <inertia ixx="\${2/5*m*r*r}" ixy="0" ixz="0" iyy="\${2/5*m*r*r}" iyz="0" izz="\${2/5*m*r*r}"/>
    </inertial>
  </xacro:macro>`;

/**
 * Visual + collision of one primitive (same shape for both).
 * @param {string} geom xacro text of the shape element
 * @param {string} xyz
 * @param {string} rpy
 * @param {string} mat
 * @param {string} indent
 */
const solidShapes = (geom, xyz, rpy, mat, indent) =>
  [
    `<visual>`,
    `  <origin xyz="${xyz}" rpy="${rpy}"/>`,
    `  <geometry>${geom}</geometry>`,
    `  <material name="${mat}"/>`,
    `</visual>`,
    `<collision>`,
    `  <origin xyz="${xyz}" rpy="${rpy}"/>`,
    `  <geometry>${geom}</geometry>`,
    `</collision>`,
  ]
    .map((l) => indent + l)
    .join("\n");

/**
 * xacro for the sensor mounts: one xacro:arg switch and pose properties each.
 * @param {Params} p
 * @param {(s: SensorSpec) => string} parentOf
 */
function sensorsXacro(p, parentOf) {
  const out = [];
  for (const s of SENSORS) {
    const k = s.key;
    const inertial =
      s.shape === "box"
        ? `<xacro:box_inertial m="${n(s.mass)}" x="${n(s.dims[0])}" y="${n(s.dims[1])}" z="${n(s.dims[2])}" xyz="0 0 0"/>`
        : `<xacro:cylinder_inertial m="${n(s.mass)}" r="${n(s.dims[0])}" h="${n(s.dims[1])}" xyz="0 0 0"/>`;
    const geom =
      s.shape === "box"
        ? `<box size="${ns(s.dims)}"/>`
        : `<cylinder radius="${n(s.dims[0])}" length="${n(s.dims[1])}"/>`;
    out.push(`
  <!-- ${s.label} (Robot Studio sensor mount: ${s.kind}) -->
  <xacro:if value="$(arg use_${k})">
    <link name="${s.link}">
      ${inertial}
${solidShapes(geom, "0 0 0", "0 0 0", "sensor", "      ")}
    </link>
    <joint name="${k}_joint" type="fixed">
      <parent link="${parentOf(s)}"/>
      <child link="${s.link}"/>
      <origin xyz="\${${k}_xyz}" rpy="\${${k}_rpy}"/>
    </joint>${
      s.optical
        ? `
    <link name="${s.optical}"/>
    <joint name="${k}_optical_joint" type="fixed">
      <parent link="${s.link}"/>
      <child link="${s.optical}"/>
      <origin xyz="0 0 0" rpy="\${-pi/2} 0 \${-pi/2}"/>
    </joint>`
        : ""
    }
  </xacro:if>`);
  }
  void p;
  return out.join("\n");
}

/**
 * Header, args and properties of a template xacro.
 * @param {Template} t
 * @param {Params} p
 */
function xacroHead(t, p) {
  const args = SENSORS.map(
    (s) => `  <xacro:arg name="use_${s.key}" default="${p[s.key] ? "true" : "false"}"/>`,
  );
  const props = t.params
    .filter((s) => s.kind !== "bool" && s.key !== "name")
    .map((s) => {
      const v = p[s.key];
      return `  <xacro:property name="${s.key}" value="${Array.isArray(v) ? ns(v) : n(v)}"/>`;
    });
  return `<?xml version="1.0"?>
<!-- Generated by Robot Studio ${VERSION} from the '${t.id}' template (${t.title}).
     Parameters are xacro properties below; sensors are switched with arguments, e.g.
       xacro ${p.name}.urdf.xacro use_lidar:=false
     Software-generated: check masses, dimensions and limits against the real robot. -->
<robot name="${p.name}" xmlns:xacro="http://www.ros.org/wiki/xacro">
${args.join("\n")}

${props.join("\n")}
${INERTIA_MACROS}
`;
}

// ---------------------------------------------------------------- mecanum

/** @type {Record<string, { on: boolean, xyz: Vec3, rpy?: Vec3 }>} */
const MOBILE_SENSORS = {
  lidar: { on: true, xyz: [0.06, 0, 0.07] },
  imu: { on: true, xyz: [0, 0, 0.02] },
  camera: { on: false, xyz: [0.1625, 0, 0.065] },
  depth_camera: { on: true, xyz: [0.165, 0, 0.03] },
};

const COLORS = {
  chassis: [0.2, 0.36, 0.62, 1],
  wheel: [0.15, 0.15, 0.17, 1],
  hub: [0.92, 0.42, 0.12, 1],
  caster: [0.6, 0.6, 0.62, 1],
  sensor: [0.08, 0.08, 0.09, 1],
  arm: [0.86, 0.87, 0.89, 1],
  joint: [0.92, 0.42, 0.12, 1],
};

/** Corners in the order FL, FR, RL, RR (mecanum-kinematics' order): name, sign of x, sign of y. */
const CORNERS = /** @type {const} */ ([
  ["front_left", 1, 1],
  ["front_right", 1, -1],
  ["rear_left", -1, 1],
  ["rear_right", -1, -1],
]);

/** @type {ParamSpec[]} */
const BASE_PARAMS = [
  {
    key: "base_length",
    label: "Chassis length (x)",
    kind: "number",
    def: 0.3,
    unit: "m",
    min: 0.02,
    max: 5,
    group: "Chassis",
  },
  {
    key: "base_width",
    label: "Chassis width (y)",
    kind: "number",
    def: 0.2,
    unit: "m",
    min: 0.02,
    max: 5,
    group: "Chassis",
  },
  {
    key: "base_height",
    label: "Chassis height (z)",
    kind: "number",
    def: 0.06,
    unit: "m",
    min: 0.005,
    max: 2,
    group: "Chassis",
  },
  {
    key: "chassis_z",
    label: "Chassis centre above the wheel axles",
    kind: "number",
    def: 0.02,
    unit: "m",
    min: -1,
    max: 2,
    group: "Chassis",
  },
  {
    key: "base_mass",
    label: "Chassis mass",
    kind: "number",
    def: 3,
    unit: "kg",
    min: 0.01,
    max: 1000,
    group: "Chassis",
  },
];

/** @type {Template} */
const MECANUM = {
  id: "mecanum",
  title: "Four-wheel mecanum base",
  description:
    "Box chassis, four mecanum wheels (FL, FR, RL, RR) on continuous joints spinning about +y, base_footprint on the ground. Rollers are drawn in an X seen from above; they are visual only.",
  params: [
    { key: "name", label: "Robot name", kind: "text", def: "mecanum_bot", group: "Robot" },
    ...BASE_PARAMS,
    {
      key: "wheel_radius",
      label: "Wheel radius",
      kind: "number",
      def: 0.04,
      unit: "m",
      min: 0.005,
      max: 1,
      group: "Wheels",
    },
    {
      key: "wheel_width",
      label: "Wheel width",
      kind: "number",
      def: 0.035,
      unit: "m",
      min: 0.002,
      max: 1,
      group: "Wheels",
    },
    {
      key: "wheel_mass",
      label: "Wheel mass",
      kind: "number",
      def: 0.15,
      unit: "kg",
      min: 0.001,
      max: 100,
      group: "Wheels",
    },
    {
      key: "wheelbase",
      label: "Wheelbase (front to rear axle, 2 lx)",
      kind: "number",
      def: 0.2,
      unit: "m",
      min: 0.01,
      max: 10,
      group: "Wheels",
    },
    {
      key: "track",
      label: "Track (left to right wheel centre, 2 ly)",
      kind: "number",
      def: 0.25,
      unit: "m",
      min: 0.01,
      max: 10,
      group: "Wheels",
    },
    {
      key: "wheel_max_speed",
      label: "Wheel speed limit",
      kind: "number",
      def: 20,
      unit: "rad/s",
      min: 0.1,
      max: 1000,
      group: "Wheels",
    },
    {
      key: "wheel_effort",
      label: "Wheel torque limit",
      kind: "number",
      def: 2,
      unit: "N m",
      min: 0.01,
      max: 10000,
      group: "Wheels",
    },
    {
      key: "roller_count",
      label: "Rollers drawn per wheel (0: plain wheel)",
      kind: "int",
      def: 8,
      min: 0,
      max: 16,
      group: "Wheels",
    },
    ...sensorParams(MOBILE_SENSORS),
  ],
  build(p) {
    const doc = mobileBase(p);
    const rr = p.wheel_radius / 5;
    for (const [corner, sx, sy] of CORNERS) {
      const hand = sx * sy;
      const hub = shape(
        "hub",
        { type: "cylinder", radius: p.wheel_radius - 2 * rr, length: p.wheel_width },
        [0, 0, 0],
        [Math.PI / 2, 0, 0],
        "hub",
      );
      /** @type {ShapeDoc[]} */
      const rollers = [];
      for (let i = 0; i < p.roller_count; i++) {
        const theta = (2 * Math.PI * i) / p.roller_count;
        rollers.push(
          shape(
            `roller_${i}`,
            {
              type: "cylinder",
              radius: rr,
              length: ((2 * Math.PI * p.wheel_radius) / p.roller_count) * 1.1,
            },
            [(p.wheel_radius - rr) * Math.cos(theta), 0, (p.wheel_radius - rr) * Math.sin(theta)],
            [(hand * Math.PI) / 4, -theta, 0],
            "wheel",
          ),
        );
      }
      const col = shape(
        "",
        { type: "cylinder", radius: p.wheel_radius, length: p.wheel_width },
        [0, 0, 0],
        [Math.PI / 2, 0, 0],
        null,
      );
      doc.links.push({
        name: `${corner}_wheel`,
        sensor: null,
        inertial: inertialFromShape(col, p.wheel_mass),
        visuals: [hub, ...rollers],
        collisions: [col],
      });
      doc.joints.push(
        joint(
          `${corner}_wheel_joint`,
          "continuous",
          "base_link",
          `${corner}_wheel`,
          [(sx * p.wheelbase) / 2, (sy * p.track) / 2, 0],
          [0, 0, 0],
          {
            axis: [0, 1, 0],
            limit: {
              lower: null,
              upper: null,
              effort: p.wheel_effort,
              velocity: p.wheel_max_speed,
            },
          },
        ),
      );
    }
    addSensors(doc, p, () => "base_link");
    return doc;
  },
  xacro(p) {
    const wheels = CORNERS.map(
      ([c, sx, sy]) =>
        `  <xacro:mecanum_wheel name="${c}" x="\${${sx < 0 ? "-" : ""}wheelbase/2}" y="\${${sy < 0 ? "-" : ""}track/2}" hand="${sx * sy}"/>`,
    ).join("\n");
    return `${xacroHead(this, p)}${mobileBaseXacro()}
  <xacro:property name="roller_radius" value="\${wheel_radius/5}"/>

  <!-- Rollers i .. roller_count-1 of one wheel (visual only); hand is +1 or -1. -->
  <xacro:macro name="rollers" params="i hand">
    <xacro:property name="theta" value="\${2*pi*i/roller_count}"/>
    <visual name="roller_\${i}">
      <origin xyz="\${(wheel_radius-roller_radius)*cos(theta)} 0 \${(wheel_radius-roller_radius)*sin(theta)}" rpy="\${hand*pi/4} \${-theta} 0"/>
      <geometry><cylinder radius="\${roller_radius}" length="\${2*pi*wheel_radius/roller_count*1.1}"/></geometry>
      <material name="wheel"/>
    </visual>
    <xacro:if value="\${i + 1 &lt; roller_count}">
      <xacro:rollers i="\${i + 1}" hand="\${hand}"/>
    </xacro:if>
  </xacro:macro>

  <!-- One mecanum wheel on a continuous joint about +y (positive speed drives forward). -->
  <xacro:macro name="mecanum_wheel" params="name x y hand">
    <link name="\${name}_wheel">
      <xacro:wheel_inertial m="\${wheel_mass}" r="\${wheel_radius}" h="\${wheel_width}"/>
      <visual name="hub">
        <origin xyz="0 0 0" rpy="\${pi/2} 0 0"/>
        <geometry><cylinder radius="\${wheel_radius-2*roller_radius}" length="\${wheel_width}"/></geometry>
        <material name="hub"/>
      </visual>
      <xacro:if value="\${roller_count > 0}">
        <xacro:rollers i="0" hand="\${hand}"/>
      </xacro:if>
      <collision>
        <origin xyz="0 0 0" rpy="\${pi/2} 0 0"/>
        <geometry><cylinder radius="\${wheel_radius}" length="\${wheel_width}"/></geometry>
      </collision>
    </link>
    <joint name="\${name}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="\${name}_wheel"/>
      <origin xyz="\${x} \${y} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="\${wheel_effort}" velocity="\${wheel_max_speed}"/>
    </joint>
  </xacro:macro>

  <!-- Wheels in mecanum-kinematics order: FL, FR, RL, RR. -->
${wheels}
${sensorsXacro(p, () => "base_link")}
</robot>
`;
  },
};

/**
 * base_footprint on the ground, base_link at the wheel axle height, box chassis.
 * @param {Params} p
 */
function mobileBase(p) {
  const doc = emptyDoc(p.name);
  for (const [k, v] of Object.entries(COLORS))
    if (k !== "arm" && k !== "joint") material(doc, k, v);
  doc.links.push({
    name: "base_footprint",
    sensor: null,
    inertial: null,
    visuals: [],
    collisions: [],
  });
  doc.links.push(
    solidLink(
      "base_link",
      { type: "box", size: [p.base_length, p.base_width, p.base_height] },
      [0, 0, p.chassis_z],
      [0, 0, 0],
      p.base_mass,
      "chassis",
    ),
  );
  doc.joints.push(
    joint("base_joint", "fixed", "base_footprint", "base_link", [0, 0, p.wheel_radius]),
  );
  return doc;
}

const MOBILE_MATERIALS = Object.entries(COLORS)
  .filter(([k]) => k !== "arm" && k !== "joint")
  .map(([k, v]) => `  <material name="${k}"><color rgba="${ns(v)}"/></material>`)
  .join("\n");

function mobileBaseXacro() {
  return `
${MOBILE_MATERIALS}

  <link name="base_footprint"/>
  <link name="base_link">
    <xacro:box_inertial m="\${base_mass}" x="\${base_length}" y="\${base_width}" z="\${base_height}" xyz="0 0 \${chassis_z}"/>
${solidShapes(`<box size="\${base_length} \${base_width} \${base_height}"/>`, "0 0 ${chassis_z}", "0 0 0", "chassis", "    ")}
  </link>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 \${wheel_radius}" rpy="0 0 0"/>
  </joint>
`;
}

// ---------------------------------------------------------------- differential

/** @type {Template} */
const DIFF = {
  id: "diff",
  title: "Differential base with caster",
  description:
    "Box chassis, two drive wheels on continuous joints spinning about +y, a spherical caster on a fixed joint touching the ground, base_footprint on the ground.",
  params: [
    { key: "name", label: "Robot name", kind: "text", def: "diff_bot", group: "Robot" },
    ...BASE_PARAMS.map((s) =>
      s.key === "base_length"
        ? { ...s, def: 0.25 }
        : s.key === "base_mass"
          ? { ...s, def: 2.5 }
          : s.key === "chassis_z"
            ? { ...s, def: 0.03 }
            : s,
    ),
    {
      key: "wheel_radius",
      label: "Wheel radius",
      kind: "number",
      def: 0.035,
      unit: "m",
      min: 0.005,
      max: 1,
      group: "Wheels",
    },
    {
      key: "wheel_width",
      label: "Wheel width",
      kind: "number",
      def: 0.025,
      unit: "m",
      min: 0.002,
      max: 1,
      group: "Wheels",
    },
    {
      key: "wheel_mass",
      label: "Wheel mass",
      kind: "number",
      def: 0.1,
      unit: "kg",
      min: 0.001,
      max: 100,
      group: "Wheels",
    },
    {
      key: "track",
      label: "Wheel separation (track)",
      kind: "number",
      def: 0.24,
      unit: "m",
      min: 0.01,
      max: 10,
      group: "Wheels",
    },
    {
      key: "wheel_x",
      label: "Axle position along x",
      kind: "number",
      def: 0.05,
      unit: "m",
      min: -5,
      max: 5,
      group: "Wheels",
    },
    {
      key: "wheel_max_speed",
      label: "Wheel speed limit",
      kind: "number",
      def: 15,
      unit: "rad/s",
      min: 0.1,
      max: 1000,
      group: "Wheels",
    },
    {
      key: "wheel_effort",
      label: "Wheel torque limit",
      kind: "number",
      def: 1,
      unit: "N m",
      min: 0.01,
      max: 10000,
      group: "Wheels",
    },
    {
      key: "caster_radius",
      label: "Caster ball radius",
      kind: "number",
      def: 0.015,
      unit: "m",
      min: 0.002,
      max: 1,
      group: "Caster",
    },
    {
      key: "caster_x",
      label: "Caster position along x",
      kind: "number",
      def: -0.09,
      unit: "m",
      min: -5,
      max: 5,
      group: "Caster",
    },
    {
      key: "caster_mass",
      label: "Caster mass",
      kind: "number",
      def: 0.03,
      unit: "kg",
      min: 0.001,
      max: 100,
      group: "Caster",
    },
    ...sensorParams({
      ...MOBILE_SENSORS,
      camera: { on: false, xyz: [0.1375, 0, 0.065] },
      depth_camera: { on: true, xyz: [0.14, 0, 0.03] },
    }),
  ],
  build(p) {
    const doc = mobileBase(p);
    for (const [side, sy] of /** @type {const} */ ([
      ["left", 1],
      ["right", -1],
    ])) {
      doc.links.push(
        solidLink(
          `${side}_wheel`,
          { type: "cylinder", radius: p.wheel_radius, length: p.wheel_width },
          [0, 0, 0],
          [Math.PI / 2, 0, 0],
          p.wheel_mass,
          "wheel",
        ),
      );
      doc.joints.push(
        joint(
          `${side}_wheel_joint`,
          "continuous",
          "base_link",
          `${side}_wheel`,
          [p.wheel_x, (sy * p.track) / 2, 0],
          [0, 0, 0],
          {
            axis: [0, 1, 0],
            limit: {
              lower: null,
              upper: null,
              effort: p.wheel_effort,
              velocity: p.wheel_max_speed,
            },
          },
        ),
      );
    }
    doc.links.push(
      solidLink(
        "caster_link",
        { type: "sphere", radius: p.caster_radius },
        [0, 0, 0],
        [0, 0, 0],
        p.caster_mass,
        "caster",
      ),
    );
    doc.joints.push(
      joint("caster_joint", "fixed", "base_link", "caster_link", [
        p.caster_x,
        0,
        p.caster_radius - p.wheel_radius,
      ]),
    );
    addSensors(doc, p, () => "base_link");
    return doc;
  },
  xacro(p) {
    return `${xacroHead(this, p)}${mobileBaseXacro()}
  <!-- One drive wheel on a continuous joint about +y (positive speed drives forward). -->
  <xacro:macro name="drive_wheel" params="side y">
    <link name="\${side}_wheel">
      <xacro:wheel_inertial m="\${wheel_mass}" r="\${wheel_radius}" h="\${wheel_width}"/>
${solidShapes(`<cylinder radius="\${wheel_radius}" length="\${wheel_width}"/>`, "0 0 0", "${pi/2} 0 0", "wheel", "      ")}
    </link>
    <joint name="\${side}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="\${side}_wheel"/>
      <origin xyz="\${wheel_x} \${y} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="\${wheel_effort}" velocity="\${wheel_max_speed}"/>
    </joint>
  </xacro:macro>

  <xacro:drive_wheel side="left" y="\${track/2}"/>
  <xacro:drive_wheel side="right" y="\${-track/2}"/>

  <!-- Caster ball touching the ground (base_link is at wheel axle height). -->
  <link name="caster_link">
    <xacro:sphere_inertial m="\${caster_mass}" r="\${caster_radius}" xyz="0 0 0"/>
${solidShapes(`<sphere radius="\${caster_radius}"/>`, "0 0 0", "0 0 0", "caster", "    ")}
  </link>
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_link"/>
    <origin xyz="\${caster_x} 0 \${caster_radius-wheel_radius}" rpy="0 0 0"/>
  </joint>
${sensorsXacro(p, () => "base_link")}
</robot>
`;
  },
};

// ---------------------------------------------------------------- arm

/** Joint axes of a serial arm by joint index, repeating: yaw, pitch, pitch, roll(z), pitch, roll(z). */
const ARM_AXES = /** @type {Vec3[]} */ ([
  [0, 0, 1],
  [0, 1, 0],
  [0, 1, 0],
  [0, 0, 1],
  [0, 1, 0],
  [0, 0, 1],
]);

/** @type {Template} */
const ARM = {
  id: "arm",
  title: "Serial arm (N revolute joints)",
  description:
    "Fixed to the world through a massless root link; a cylinder base, N cylinder segments on revolute joints with axes z, y, y, z, y, z (repeating), and a tool0 flange frame.",
  params: [
    { key: "name", label: "Robot name", kind: "text", def: "arm_6dof", group: "Robot" },
    { key: "dof", label: "Number of joints", kind: "int", def: 6, min: 1, max: 50, group: "Arm" },
    {
      key: "base_radius",
      label: "Base radius",
      kind: "number",
      def: 0.06,
      unit: "m",
      min: 0.005,
      max: 2,
      group: "Arm",
    },
    {
      key: "base_height",
      label: "Base height",
      kind: "number",
      def: 0.05,
      unit: "m",
      min: 0.005,
      max: 2,
      group: "Arm",
    },
    {
      key: "base_mass",
      label: "Base mass",
      kind: "number",
      def: 1,
      unit: "kg",
      min: 0.01,
      max: 1000,
      group: "Arm",
    },
    {
      key: "link_length",
      label: "Segment length",
      kind: "number",
      def: 0.15,
      unit: "m",
      min: 0.01,
      max: 5,
      group: "Arm",
    },
    {
      key: "link_radius",
      label: "Segment radius",
      kind: "number",
      def: 0.03,
      unit: "m",
      min: 0.002,
      max: 1,
      group: "Arm",
    },
    {
      key: "link_mass",
      label: "Segment mass",
      kind: "number",
      def: 0.5,
      unit: "kg",
      min: 0.001,
      max: 1000,
      group: "Arm",
    },
    {
      key: "joint_limit",
      label: "Joint limit (+/-)",
      kind: "number",
      def: 2.9,
      unit: "rad",
      min: 0.01,
      max: 3.14159,
      group: "Arm",
    },
    {
      key: "joint_effort",
      label: "Joint torque limit",
      kind: "number",
      def: 20,
      unit: "N m",
      min: 0.01,
      max: 10000,
      group: "Arm",
    },
    {
      key: "joint_velocity",
      label: "Joint speed limit",
      kind: "number",
      def: 2,
      unit: "rad/s",
      min: 0.01,
      max: 100,
      group: "Arm",
    },
    ...sensorParams({
      lidar: { on: false, xyz: [0.1, 0, 0.02] },
      imu: { on: false, xyz: [0, 0, 0.025] },
      camera: { on: false, xyz: [0.0125, 0, 0.0125], rpy: [0, -Math.PI / 2, 0] },
      depth_camera: { on: false, xyz: [0.015, 0, 0.015], rpy: [0, -Math.PI / 2, 0] },
    }),
  ],
  build(p) {
    const doc = emptyDoc(p.name);
    material(doc, "arm", COLORS.arm);
    material(doc, "joint", COLORS.joint);
    material(doc, "sensor", COLORS.sensor);
    doc.links.push({ name: "world", sensor: null, inertial: null, visuals: [], collisions: [] });
    doc.links.push(
      solidLink(
        "base_link",
        { type: "cylinder", radius: p.base_radius, length: p.base_height },
        [0, 0, p.base_height / 2],
        [0, 0, 0],
        p.base_mass,
        "joint",
      ),
    );
    doc.joints.push(joint("world_joint", "fixed", "world", "base_link", [0, 0, 0]));
    for (let k = 1; k <= p.dof; k++) {
      doc.links.push(
        solidLink(
          `link_${k}`,
          { type: "cylinder", radius: p.link_radius, length: p.link_length },
          [0, 0, p.link_length / 2],
          [0, 0, 0],
          p.link_mass,
          "arm",
        ),
      );
      doc.joints.push(
        joint(
          `joint_${k}`,
          "revolute",
          k === 1 ? "base_link" : `link_${k - 1}`,
          `link_${k}`,
          [0, 0, k === 1 ? p.base_height : p.link_length],
          [0, 0, 0],
          {
            axis: [...ARM_AXES[(k - 1) % ARM_AXES.length]],
            limit: {
              lower: -p.joint_limit,
              upper: p.joint_limit,
              effort: p.joint_effort,
              velocity: p.joint_velocity,
            },
          },
        ),
      );
    }
    doc.links.push({ name: "tool0", sensor: null, inertial: null, visuals: [], collisions: [] });
    doc.joints.push(joint("tool0_joint", "fixed", `link_${p.dof}`, "tool0", [0, 0, p.link_length]));
    addSensors(doc, p, armSensorParent);
    return doc;
  },
  xacro(p) {
    const axes = ARM_AXES.map((a) => `'${ns(a)}'`).join(", ");
    return `${xacroHead(this, p)}
  <material name="arm"><color rgba="${ns(COLORS.arm)}"/></material>
  <material name="joint"><color rgba="${ns(COLORS.joint)}"/></material>
  <material name="sensor"><color rgba="${ns(COLORS.sensor)}"/></material>

  <!-- A massless root fixed to the world (KDL ignores the inertia of a root link). -->
  <link name="world"/>
  <link name="base_link">
    <xacro:cylinder_inertial m="\${base_mass}" r="\${base_radius}" h="\${base_height}" xyz="0 0 \${base_height/2}"/>
${solidShapes(`<cylinder radius="\${base_radius}" length="\${base_height}"/>`, "0 0 ${base_height/2}", "0 0 0", "joint", "    ")}
  </link>
  <joint name="world_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Segment k and its joint, then the next segment until dof. -->
  <xacro:property name="joint_axes" value="\${[${axes}]}"/>
  <xacro:macro name="segment" params="k">
    <link name="link_\${k}">
      <xacro:cylinder_inertial m="\${link_mass}" r="\${link_radius}" h="\${link_length}" xyz="0 0 \${link_length/2}"/>
${solidShapes(`<cylinder radius="\${link_radius}" length="\${link_length}"/>`, "0 0 ${link_length/2}", "0 0 0", "arm", "      ")}
    </link>
    <joint name="joint_\${k}" type="revolute">
      <parent link="\${'base_link' if k == 1 else 'link_' + str(k - 1)}"/>
      <child link="link_\${k}"/>
      <origin xyz="0 0 \${base_height if k == 1 else link_length}" rpy="0 0 0"/>
      <axis xyz="\${joint_axes[(k - 1) % ${ARM_AXES.length}]}"/>
      <limit lower="\${-joint_limit}" upper="\${joint_limit}" effort="\${joint_effort}" velocity="\${joint_velocity}"/>
    </joint>
    <xacro:if value="\${k &lt; dof}">
      <xacro:segment k="\${k + 1}"/>
    </xacro:if>
  </xacro:macro>
  <xacro:segment k="1"/>

  <link name="tool0"/>
  <joint name="tool0_joint" type="fixed">
    <parent link="link_\${dof}"/>
    <child link="tool0"/>
    <origin xyz="0 0 \${link_length}" rpy="0 0 0"/>
  </joint>
${sensorsXacro(p, armSensorParent)}
</robot>
`;
  },
};

/** Cameras go on the flange, LiDAR and IMU on the base. @param {SensorSpec} s */
function armSensorParent(s) {
  return s.kind === "camera" || s.kind === "depth_camera" ? "tool0" : "base_link";
}

// ---------------------------------------------------------------- API

/** @type {Template[]} */
export const TEMPLATES = [MECANUM, DIFF, ARM];

/** @param {string} id */
export function findTemplate(id) {
  const t = TEMPLATES.find((x) => x.id === id);
  if (!t) throw new TemplateError(`no template '${id}' (${TEMPLATES.map((x) => x.id).join(", ")})`);
  return t;
}

/**
 * Default parameters of a template.
 * @param {string} id
 * @returns {Params}
 */
export function defaultParams(id) {
  /** @type {Params} */
  const out = {};
  for (const s of findTemplate(id).params) out[s.key] = Array.isArray(s.def) ? [...s.def] : s.def;
  return out;
}

/**
 * Complete and check parameters: missing ones take their defaults; values
 * must have the right type and lie within the limits. Throws TemplateError
 * listing every problem.
 * @param {string} id
 * @param {Record<string, unknown>} given
 * @returns {Params}
 */
export function validateParams(id, given) {
  const t = findTemplate(id);
  const p = defaultParams(id);
  /** @type {string[]} */
  const errors = [];
  const known = new Set(t.params.map((s) => s.key));
  for (const k of Object.keys(given)) if (!known.has(k)) errors.push(`unknown parameter '${k}'`);
  for (const s of t.params) {
    if (!(s.key in given)) continue;
    const v = given[s.key];
    /** @param {unknown} x */
    const inRange = (x) =>
      typeof x === "number" &&
      Number.isFinite(x) &&
      (s.min === undefined || x >= s.min) &&
      (s.max === undefined || x <= s.max);
    const range = `${s.min ?? "-inf"} .. ${s.max ?? "inf"}${s.unit ? ` ${s.unit}` : ""}`;
    if (s.kind === "bool") {
      if (typeof v !== "boolean") errors.push(`${s.key} must be true or false`);
      else p[s.key] = v;
    } else if (s.kind === "text") {
      if (typeof v !== "string" || !/^[A-Za-z_][A-Za-z0-9_\-]{0,63}$/.test(v))
        errors.push(`${s.key} must be a name (letters, digits, _ and -)`);
      else p[s.key] = v;
    } else if (s.kind === "vec3") {
      if (!Array.isArray(v) || v.length !== 3 || !v.every(inRange))
        errors.push(`${s.key} must be three numbers in ${range}`);
      else p[s.key] = [v[0], v[1], v[2]];
    } else if (!inRange(v) || (s.kind === "int" && !Number.isInteger(v)))
      errors.push(
        `${s.key} must be ${s.kind === "int" ? "a whole number" : "a number"} in ${range}`,
      );
    else p[s.key] = v;
  }
  // Geometric consistency the limits alone do not catch.
  if (id === "mecanum" || id === "diff") {
    if (p.track / 2 - p.wheel_width / 2 < p.base_width / 2 - 1e-9)
      errors.push(
        `the wheels overlap the chassis: track/2 - wheel_width/2 = ${n(p.track / 2 - p.wheel_width / 2)} m is less than base_width/2 = ${n(p.base_width / 2)} m`,
      );
  }
  if (id === "diff" && p.caster_radius > p.wheel_radius)
    errors.push(
      "the caster radius must not exceed the wheel radius (the caster hangs below base_link)",
    );
  if (errors.length > 0) throw new TemplateError(errors.join("; "));
  return p;
}

/**
 * Generate a robot document from a template.
 * @param {string} id
 * @param {Record<string, unknown>} [given] parameters (defaults for missing ones)
 * @returns {RobotDoc}
 */
export function generate(id, given = {}) {
  const t = findTemplate(id);
  const p = validateParams(id, given);
  const doc = t.build(p);
  normalizeOrder(doc);
  doc.template = { id, params: p };
  return doc;
}

/**
 * Whether the document is still exactly what its template parameters
 * generate (no manual edits since), so the parametrised xacro describes it.
 * @param {RobotDoc} doc
 */
export function isTemplateExact(doc) {
  if (!doc.template) return false;
  try {
    const fresh = generate(doc.template.id, doc.template.params);
    // Simulation settings are not part of the robot.
    const mine = { ...doc, source: null };
    delete mine.sim;
    return deepEqual({ ...fresh, source: null }, mine);
  } catch {
    return false;
  }
}

/**
 * The parametrised xacro of a template document, or null when the document
 * was edited after it was generated.
 * @param {RobotDoc} doc
 * @returns {string | null}
 */
export function templateXacro(doc) {
  if (!doc.template || !isTemplateExact(doc)) return null;
  const t = findTemplate(doc.template.id);
  return t.xacro(validateParams(t.id, doc.template.params));
}
