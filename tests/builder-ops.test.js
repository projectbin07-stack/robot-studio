// Edit operations, undo/redo, rename propagation, inertia from geometry,
// and the time an edit takes on a 50-link model.

import { test } from "node:test";
import assert from "node:assert/strict";
import { Editor, HISTORY_LIMIT, modelFromDoc } from "../src/core/builder/editor.js";
import { applyOp, describeOp, EditError, linkPoses } from "../src/core/builder/ops.js";
import { inertialFromShape, inertiaShape } from "../src/core/builder/inertia.js";
import {
  clone,
  deepEqual,
  emptyDoc,
  emptyLink,
  findJoint,
  findLink,
  origin,
  rootLinks,
  subtree,
  uniqueName,
} from "../src/core/builder/document.js";
import { generate } from "../src/core/builder/templates.js";
import { toUrdf } from "../src/core/builder/urdf.js";
import { inspectText } from "../vendor/urdf-inspector/src/core/inspect.js";

/** @typedef {import("../src/core/builder/ops.js").EditOp} EditOp */
/** @typedef {import("../src/core/builder/document.js").RobotDoc} RobotDoc */

const ids = (/** @type {Editor} */ ed) => ed.model.findings.map((f) => f.id);
const errorsAndWarnings = (/** @type {Editor} */ ed) =>
  ed.model.findings.filter((f) => f.severity !== "info").map((f) => `${f.id}: ${f.message}`);

/** @param {string} name @param {import("../src/core/builder/document.js").JointType} [type] */
const jointSpec = (name, type = "fixed") => ({
  name,
  type,
  origin: origin([0.1, 0, 0]),
  axis: type === "fixed" ? null : /** @type {[number, number, number]} */ ([0, 0, 1]),
  limit: type === "revolute" ? { lower: -1, upper: 1, effort: 1, velocity: 1 } : null,
  dynamics: null,
  mimic: null,
});

/** @param {string} name */
const boxLink = (name) => {
  const l = emptyLink(name);
  const s = {
    name: "",
    origin: origin(),
    geometry: /** @type {import("../src/core/builder/document.js").Geometry} */ ({
      type: "box",
      size: [0.1, 0.1, 0.1],
    }),
    material: null,
  };
  l.visuals.push({ ...s, material: { name: "", rgba: [1, 0, 0, 1] } });
  l.collisions.push(s);
  l.inertial = inertialFromShape(s, 1);
  return l;
};

/**
 * Every operation is undoable to the exact previous document, and redo
 * gives the exact edited one.
 * @param {RobotDoc} start
 * @param {EditOp} op
 */
function assertUndoRedo(start, op) {
  const ed = new Editor(start);
  const before = clone(ed.doc);
  ed.apply(op);
  const after = clone(ed.doc);
  assert.ok(!deepEqual(before, after), `${op.type} changed nothing`);
  assert.ok(ed.canUndo && !ed.canRedo);
  assert.equal(ed.undoLabel, describeOp(op));
  assert.ok(ed.undo());
  assert.ok(deepEqual(ed.doc, before), `${op.type}: undo did not restore the document`);
  assert.equal(toUrdf(ed.doc), toUrdf(before));
  assert.equal(ed.redoLabel, describeOp(op));
  assert.ok(ed.redo());
  assert.ok(deepEqual(ed.doc, after), `${op.type}: redo did not reapply`);
  return ed;
}

test("every edit operation applies, undoes and redoes exactly", () => {
  const mec = generate("mecanum");
  const arm = generate("arm", { dof: 3 });
  const meshDoc = clone(arm);
  /** @type {[RobotDoc, EditOp][]} */
  const cases = [
    [mec, { type: "setRobotName", name: "renamed" }],
    [
      mec,
      {
        type: "addLink",
        parent: "base_link",
        link: boxLink("bumper"),
        joint: jointSpec("bumper_joint"),
      },
    ],
    [mec, { type: "removeLink", name: "front_left_wheel" }],
    [mec, { type: "renameLink", from: "base_link", to: "chassis" }],
    [mec, { type: "reparentLink", name: "imu_link", parent: "lidar_link" }],
    [mec, { type: "duplicateLink", name: "depth_camera_link" }],
    [mec, { type: "renameJoint", from: "base_joint", to: "footprint_joint" }],
    [mec, { type: "setJoint", name: "lidar_joint", patch: { type: "revolute" } }],
    [
      mec,
      {
        type: "setOrigin",
        target: { kind: "joint", name: "lidar_joint" },
        origin: origin([0.07, 0, 0.08], [0, 0, 0.5]),
      },
    ],
    [
      mec,
      {
        type: "setOrigin",
        target: { kind: "visual", link: "base_link", index: 0 },
        origin: origin([0, 0, 0.03]),
      },
    ],
    [
      mec,
      {
        type: "setOrigin",
        target: { kind: "collision", link: "base_link", index: 0 },
        origin: origin([0, 0, 0.03]),
      },
    ],
    [
      mec,
      {
        type: "setOrigin",
        target: { kind: "inertial", link: "base_link" },
        origin: origin([0.01, 0, 0.02]),
      },
    ],
    [
      mec,
      {
        type: "setGeometry",
        link: "base_link",
        kind: "visual",
        index: 0,
        geometry: { type: "cylinder", radius: 0.1, length: 0.05 },
      },
    ],
    [
      mec,
      {
        type: "addShape",
        link: "base_link",
        kind: "visual",
        shape: {
          name: "deck",
          origin: origin([0, 0, 0.06]),
          geometry: { type: "sphere", radius: 0.02 },
          material: { name: "chassis", rgba: null },
        },
      },
    ],
    [mec, { type: "removeShape", link: "front_left_wheel", kind: "visual", index: 3 }],
    [
      mec,
      {
        type: "setMaterial",
        link: "base_link",
        index: 0,
        material: { name: "", rgba: [0.1, 0.9, 0.1, 1] },
      },
    ],
    [mec, { type: "setMass", link: "base_link", mass: 4.5 }],
    [mec, { type: "setInertial", link: "base_link", inertial: null }],
    [mec, { type: "computeInertia", link: "base_link", mass: 2 }],
    [mec, { type: "setSensor", link: "imu_link", sensor: null }],
    [mec, { type: "replaceDocument", doc: arm, label: "new arm" }],
    [meshDoc, { type: "setJoint", name: "joint_2", patch: { type: "fixed" } }],
    [
      arm,
      {
        type: "setJoint",
        name: "joint_3",
        patch: {
          mimic: { joint: "joint_2", multiplier: -1, offset: 0.1 },
          dynamics: { damping: 0.1, friction: 0 },
          axis: [1, 0, 0],
          limit: { lower: -2, upper: 2, effort: 5, velocity: 1 },
        },
      },
    ],
  ];
  for (const [doc, op] of cases) assertUndoRedo(doc, op);
});

test("operations reject invalid input and leave the document unchanged", () => {
  const ed = new Editor(generate("mecanum"));
  const before = clone(ed.doc);
  const version = ed.version;
  /** @type {[EditOp, RegExp][]} */
  const bad = [
    [{ type: "setRobotName", name: "bad name" }, /must be letters/],
    [
      { type: "addLink", parent: "nope", link: boxLink("x"), joint: jointSpec("xj") },
      /no link named 'nope'/,
    ],
    [
      { type: "addLink", parent: "base_link", link: boxLink("lidar_link"), joint: jointSpec("xj") },
      /already used/,
    ],
    [
      { type: "addLink", parent: "base_link", link: boxLink("x"), joint: jointSpec("base_joint") },
      /already used/,
    ],
    [
      { type: "addLink", parent: "base_link", link: boxLink("x"), joint: jointSpec("x") },
      /different names/,
    ],
    [
      { type: "addLink", parent: "base_link", link: boxLink("1x"), joint: jointSpec("xj") },
      /not a valid name/,
    ],
    [
      {
        type: "addLink",
        parent: "base_link",
        link: boxLink("x"),
        joint: { ...jointSpec("xj"), type: /** @type {any} */ ("ball") },
      },
      /unknown joint type/,
    ],
    [
      {
        type: "addLink",
        parent: "base_link",
        link: boxLink("x"),
        joint: { ...jointSpec("xj"), origin: /** @type {any} */ ({ xyz: [0, 0], rpy: [0, 0, 0] }) },
      },
      /three finite numbers/,
    ],
    [{ type: "removeLink", name: "base_footprint" }, /root link and cannot be deleted/],
    [{ type: "renameLink", from: "base_link", to: "lidar_link" }, /already used/],
    [{ type: "renameLink", from: "base_link", to: "a b" }, /not a valid name/],
    [
      { type: "reparentLink", name: "base_link", parent: "lidar_link" },
      /below 'base_link'; that would make a loop/,
    ],
    [{ type: "reparentLink", name: "base_footprint", parent: "base_link" }, /root link/],
    [{ type: "duplicateLink", name: "base_footprint" }, /cannot be duplicated/],
    [{ type: "renameJoint", from: "nope", to: "x" }, /no joint named/],
    [{ type: "renameJoint", from: "base_joint", to: "lidar_joint" }, /already used/],
    [
      { type: "setJoint", name: "lidar_joint", patch: { type: /** @type {any} */ ("hinge") } },
      /unknown joint type/,
    ],
    [{ type: "setJoint", name: "lidar_joint", patch: { axis: [0, 0, 0] } }, /not all zero/],
    [
      {
        type: "setJoint",
        name: "lidar_joint",
        patch: { limit: { lower: NaN, upper: 1, effort: 1, velocity: 1 } },
      },
      /limit lower/,
    ],
    [
      { type: "setJoint", name: "lidar_joint", patch: { dynamics: { damping: NaN, friction: 0 } } },
      /damping/,
    ],
    [
      {
        type: "setJoint",
        name: "lidar_joint",
        patch: { mimic: { joint: "lidar_joint", multiplier: 1, offset: 0 } },
      },
      /mimic itself/,
    ],
    [
      {
        type: "setJoint",
        name: "lidar_joint",
        patch: { mimic: { joint: "nope", multiplier: 1, offset: 0 } },
      },
      /no joint named/,
    ],
    [
      {
        type: "setJoint",
        name: "lidar_joint",
        patch: { origin: /** @type {any} */ ({ xyz: [0, 0, Infinity], rpy: [0, 0, 0] }) },
      },
      /finite/,
    ],
    [
      { type: "setOrigin", target: { kind: "inertial", link: "base_footprint" }, origin: origin() },
      /has no inertial/,
    ],
    [
      {
        type: "setOrigin",
        target: { kind: "visual", link: "base_link", index: 5 },
        origin: origin(),
      },
      /has no visual 6/,
    ],
    [
      {
        type: "setGeometry",
        link: "base_link",
        kind: "collision",
        index: 0,
        geometry: { type: "box", size: [1, 0, 1] },
      },
      /positive/,
    ],
    [
      {
        type: "setGeometry",
        link: "base_link",
        kind: "collision",
        index: 0,
        geometry: { type: "cylinder", radius: 1, length: -1 },
      },
      /positive/,
    ],
    [
      {
        type: "setGeometry",
        link: "base_link",
        kind: "collision",
        index: 0,
        geometry: { type: "sphere", radius: 0 },
      },
      /positive/,
    ],
    [
      {
        type: "setGeometry",
        link: "base_link",
        kind: "collision",
        index: 0,
        geometry: { type: "mesh", filename: "", scale: [1, 1, 1] },
      },
      /file name/,
    ],
    [
      {
        type: "setGeometry",
        link: "base_link",
        kind: "collision",
        index: 0,
        geometry: /** @type {any} */ ({ type: "capsule" }),
      },
      /unknown geometry/,
    ],
    [
      {
        type: "setGeometry",
        link: "base_link",
        kind: "collision",
        index: 3,
        geometry: { type: "sphere", radius: 1 },
      },
      /no collision 4/,
    ],
    [{ type: "removeShape", link: "base_link", kind: "collision", index: 1 }, /no collision 2/],
    [
      {
        type: "setMaterial",
        link: "base_link",
        index: 0,
        material: { name: "", rgba: [2, 0, 0, 1] },
      },
      /0 to 1/,
    ],
    [{ type: "setMaterial", link: "base_link", index: 9, material: null }, /no visual 10/],
    [{ type: "setMass", link: "base_link", mass: 0 }, /positive/],
    [{ type: "setMass", link: "base_footprint", mass: 1 }, /no inertia to scale/],
    [
      {
        type: "setInertial",
        link: "base_link",
        inertial: { origin: origin(), mass: NaN, inertia: null },
      },
      /mass must be a number/,
    ],
    [
      {
        type: "setInertial",
        link: "base_link",
        inertial: {
          origin: origin(),
          mass: 1,
          inertia: { ixx: 1, ixy: 0, ixz: 0, iyy: NaN, iyz: 0, izz: 1 },
        },
      },
      /inertia terms/,
    ],
    [{ type: "computeInertia", link: "base_footprint" }, /no geometry/],
    [
      { type: "setSensor", link: "base_link", sensor: /** @type {any} */ ("sonar") },
      /unknown sensor/,
    ],
    [
      { type: "replaceDocument", doc: /** @type {any} */ ({ format: 2 }) },
      /not a Robot Studio document/,
    ],
    [/** @type {any} */ ({ type: "explode" }), /unknown operation/],
  ];
  for (const [op, re] of bad) {
    assert.throws(
      () => ed.apply(op),
      (e) => e instanceof EditError && re.test(e.message),
      `${op.type} ${JSON.stringify(op).slice(0, 80)}`,
    );
  }
  assert.ok(deepEqual(ed.doc, before));
  assert.equal(ed.version, version);
  assert.ok(!ed.canUndo && !ed.undo() && !ed.redo());
  assert.equal(ed.undoLabel, null);
  assert.equal(ed.redoLabel, null);
});

test("renames update every reference: joints, mimic, sensors, findings targets", () => {
  const ed = new Editor(generate("arm", { dof: 3, camera: true }));
  ed.apply({
    type: "setJoint",
    name: "joint_3",
    patch: { mimic: { joint: "joint_2", multiplier: 1, offset: 0 } },
  });
  ed.apply({ type: "renameLink", from: "link_2", to: "upper_arm" });
  ed.apply({ type: "renameJoint", from: "joint_2", to: "shoulder" });
  ed.apply({ type: "renameLink", from: "tool0", to: "flange" });
  const d = ed.doc;
  assert.equal(findJoint(d, "shoulder")?.child, "upper_arm");
  assert.equal(findJoint(d, "joint_3")?.parent, "upper_arm");
  assert.equal(findJoint(d, "joint_3")?.mimic?.joint, "shoulder");
  assert.equal(findJoint(d, "camera_joint")?.parent, "flange");
  assert.equal(findJoint(d, "tool0_joint")?.child, "flange");
  assert.ok(!toUrdf(d).includes("link_2"));
  assert.ok(!toUrdf(d).includes('"joint_2"'));
  assert.deepEqual(errorsAndWarnings(ed), []);
  // Renaming to itself is a no-op that still records a step.
  ed.apply({ type: "renameLink", from: "flange", to: "flange" });
  ed.apply({ type: "renameJoint", from: "shoulder", to: "shoulder" });
  assert.equal(ed.past.length, 6);
});

test("structure edits: add, delete subtree, re-parent keeping the pose, duplicate subtree", () => {
  const ed = new Editor(generate("mecanum"));
  ed.apply({
    type: "addLink",
    parent: "lidar_link",
    link: boxLink("mast"),
    joint: jointSpec("mast_joint"),
  });
  assert.deepEqual(subtree(ed.doc, "lidar_link"), ["lidar_link", "mast"]);
  // The new link follows its parent's subtree in document order.
  const names = ed.doc.links.map((l) => l.name);
  assert.equal(names.indexOf("mast"), names.indexOf("lidar_link") + 1);

  // Re-parent the depth camera under the lidar: its world pose is unchanged.
  const before = linkPoses(ed.doc).get("depth_camera_optical_frame");
  ed.apply({
    type: "setOrigin",
    target: { kind: "joint", name: "lidar_joint" },
    origin: origin([0.06, 0, 0.07], [0.1, -0.2, 0.3]),
  });
  const moved = linkPoses(ed.doc).get("depth_camera_optical_frame");
  ed.apply({ type: "reparentLink", name: "depth_camera_link", parent: "lidar_link" });
  const after = linkPoses(ed.doc).get("depth_camera_optical_frame");
  assert.ok(before && moved && after);
  for (let k = 0; k < 3; k++) assert.ok(Math.abs(after.p[k] - moved.p[k]) < 1e-12);
  for (let k = 0; k < 9; k++) assert.ok(Math.abs(after.R[k] - moved.R[k]) < 1e-12);
  assert.equal(findJoint(ed.doc, "depth_camera_joint")?.parent, "lidar_link");
  // keepPose false keeps the joint origin as written; same parent is a no-op.
  const o = findJoint(ed.doc, "imu_joint")?.origin;
  ed.apply({ type: "reparentLink", name: "imu_link", parent: "lidar_link", keepPose: false });
  assert.deepEqual(findJoint(ed.doc, "imu_joint")?.origin, o);
  ed.apply({ type: "reparentLink", name: "imu_link", parent: "lidar_link" });

  // Duplicate a subtree: new unique names, mimic inside the copy re-pointed.
  ed.apply({ type: "duplicateLink", name: "lidar_link" });
  for (const n of [
    "lidar_link_copy",
    "mast_copy",
    "depth_camera_link_copy",
    "depth_camera_optical_frame_copy",
    "imu_link_copy",
  ])
    assert.ok(findLink(ed.doc, n), n);
  assert.equal(findJoint(ed.doc, "lidar_joint_copy")?.parent, "base_link");
  assert.equal(findJoint(ed.doc, "mast_joint_copy")?.parent, "lidar_link_copy");
  ed.apply({ type: "duplicateLink", name: "lidar_link" });
  assert.ok(findLink(ed.doc, "lidar_link_copy_2"));
  assert.equal(uniqueName(ed.doc, "lidar_link_copy"), "lidar_link_copy_3");
  assert.equal(uniqueName(ed.doc, "fresh"), "fresh");

  // Delete a subtree; mimics of deleted joints are dropped.
  ed.apply({
    type: "setJoint",
    name: "front_left_wheel_joint",
    patch: { mimic: { joint: "mast_joint", multiplier: 1, offset: 0 } },
  });
  ed.apply({ type: "removeLink", name: "lidar_link" });
  for (const n of ["lidar_link", "mast", "depth_camera_link", "imu_link"])
    assert.equal(findLink(ed.doc, n), null, n);
  assert.equal(findJoint(ed.doc, "front_left_wheel_joint")?.mimic, null);
  assert.deepEqual(rootLinks(ed.doc), ["base_footprint"]);

  // An extra root (orphan link) can be deleted; the last root cannot.
  const doc = emptyDoc("two");
  doc.links.push(boxLink("a"), boxLink("b"));
  assert.equal(applyOp(doc, { type: "removeLink", name: "b" }).links.length, 1);
});

test("joint type changes add usable limits and axes, fixed clears them", () => {
  const ed = new Editor(generate("mecanum"));
  ed.apply({ type: "setJoint", name: "lidar_joint", patch: { type: "revolute" } });
  let j = findJoint(ed.doc, "lidar_joint");
  assert.deepEqual(j?.axis, [0, 0, 1]);
  assert.deepEqual(j?.limit, { lower: -Math.PI / 2, upper: Math.PI / 2, effort: 10, velocity: 1 });
  ed.apply({ type: "setJoint", name: "lidar_joint", patch: { type: "prismatic" } });
  j = findJoint(ed.doc, "lidar_joint");
  assert.deepEqual(
    j?.limit,
    { lower: -Math.PI / 2, upper: Math.PI / 2, effort: 10, velocity: 1 },
    "existing limits are kept",
  );
  ed.apply({ type: "setJoint", name: "imu_joint", patch: { type: "prismatic" } });
  assert.deepEqual(findJoint(ed.doc, "imu_joint")?.limit, {
    lower: 0,
    upper: 0.1,
    effort: 10,
    velocity: 0.1,
  });
  ed.apply({
    type: "setJoint",
    name: "imu_joint",
    patch: { type: "continuous", axis: null, limit: null },
  });
  assert.equal(findJoint(ed.doc, "imu_joint")?.axis, null);
  ed.apply({ type: "setJoint", name: "lidar_joint", patch: { type: "fixed" } });
  j = findJoint(ed.doc, "lidar_joint");
  assert.equal(j?.axis, null);
  assert.equal(j?.limit, null);
  ed.apply({ type: "setJoint", name: "lidar_joint", patch: { type: "floating" } });
  assert.equal(findJoint(ed.doc, "lidar_joint")?.axis, null);
  assert.ok(ids(ed).includes("kdl-unsupported-joint"));
});

test("checks re-run after every edit, undo and redo", () => {
  const ed = new Editor(generate("diff"));
  assert.deepEqual(errorsAndWarnings(ed), []);
  ed.apply({ type: "setMass", link: "left_wheel", mass: 30 });
  assert.ok(ids(ed).includes("density-implausible"));
  ed.undo();
  assert.ok(!ids(ed).includes("density-implausible"));
  ed.redo();
  assert.ok(ids(ed).includes("density-implausible"));
  ed.apply({
    type: "setOrigin",
    target: { kind: "joint", name: "left_wheel_joint" },
    origin: origin([0.05, 0.12, 0.01]),
  });
  assert.ok(ids(ed).includes("wheel-layout"));
  ed.apply({ type: "setJoint", name: "right_wheel_joint", patch: { axis: [0, -1, 0] } });
  assert.ok(ids(ed).includes("wheel-direction-mixed"));
  // A new edit clears the redo stack.
  ed.undo();
  assert.ok(ed.canRedo);
  ed.apply({ type: "setRobotName", name: "x" });
  assert.ok(!ed.canRedo);
  // History is bounded.
  for (let k = 0; k < HISTORY_LIMIT + 5; k++) ed.apply({ type: "setRobotName", name: `n${k}` });
  assert.equal(ed.past.length, HISTORY_LIMIT);
});

test("inertia from geometry: urdf-inspector's formulas, rotated with the shape, passing inertia-axes", () => {
  // A cylinder lying along y (roll pi/2) and a box yawed by 30 degrees.
  const doc = emptyDoc("inertia");
  const base = emptyLink("base_link");
  base.collisions.push({
    name: "",
    origin: origin([0.01, 0.02, 0.03], [Math.PI / 2, 0, 0]),
    geometry: { type: "cylinder", radius: 0.05, length: 0.3 },
    material: null,
  });
  doc.links.push(base);
  const ed = new Editor(doc);
  ed.apply({ type: "computeInertia", link: "base_link", mass: 2 });
  const i = findLink(ed.doc, "base_link")?.inertial;
  assert.ok(i?.inertia);
  const side = (2 * (3 * 0.05 * 0.05 + 0.3 * 0.3)) / 12;
  assert.deepEqual(i.origin, origin([0.01, 0.02, 0.03]));
  assert.equal(i.inertia.ixx, side);
  assert.equal(i.inertia.iyy, (2 * 0.05 * 0.05) / 2);
  assert.equal(i.inertia.izz, side);
  assert.equal(i.inertia.iyz, 0);
  assert.ok(!ids(ed).some((x) => x.startsWith("inertia-")), ids(ed).join(","));

  // The same tensor without the rotation is what inertia-axes catches.
  const wrong = clone(ed.doc);
  const t = /** @type {any} */ (wrong.links[0].inertial).inertia;
  [t.iyy, t.izz] = [t.izz, t.iyy];
  assert.ok(inspectText(toUrdf(wrong)).findings.some((f) => f.id === "inertia-axes"));

  ed.apply({
    type: "setGeometry",
    link: "base_link",
    kind: "collision",
    index: 0,
    geometry: { type: "box", size: [0.4, 0.1, 0.05] },
  });
  ed.apply({
    type: "setOrigin",
    target: { kind: "collision", link: "base_link", index: 0 },
    origin: origin([0, 0, 0], [0, 0, Math.PI / 6]),
  });
  ed.apply({ type: "computeInertia", link: "base_link" });
  const b = findLink(ed.doc, "base_link")?.inertial?.inertia;
  assert.ok(b && Math.abs(b.ixy) > 1e-4, "a yawed box has a product of inertia");
  assert.ok(!ids(ed).some((x) => x.startsWith("inertia-")), ids(ed).join(","));

  // Sphere; setMass scales the tensor with the mass.
  ed.apply({
    type: "setGeometry",
    link: "base_link",
    kind: "collision",
    index: 0,
    geometry: { type: "sphere", radius: 0.1 },
  });
  ed.apply({ type: "computeInertia", link: "base_link", mass: 3 });
  assert.equal(findLink(ed.doc, "base_link")?.inertial?.inertia?.ixx, (2 / 5) * 3 * 0.1 * 0.1);
  ed.apply({ type: "setMass", link: "base_link", mass: 6 });
  const s = findLink(ed.doc, "base_link")?.inertial;
  assert.equal(s?.mass, 6);
  assert.ok(Math.abs((s?.inertia?.ixx ?? 0) - 0.024) < 1e-15);
  // setMass on a link without an inertial computes one from its shape.
  ed.apply({ type: "setInertial", link: "base_link", inertial: null });
  ed.apply({ type: "setMass", link: "base_link", mass: 1 });
  assert.ok(Math.abs((findLink(ed.doc, "base_link")?.inertial?.inertia?.izz ?? 0) - 0.004) < 1e-15);

  // Meshes: a solid box filling the scaled bounding box, centred on it.
  ed.apply({
    type: "setGeometry",
    link: "base_link",
    kind: "collision",
    index: 0,
    geometry: { type: "mesh", filename: "package://p/m.stl", scale: [0.001, 0.001, 0.001] },
  });
  assert.throws(() => ed.apply({ type: "computeInertia", link: "base_link" }), /bounding box/);
  ed.apply({
    type: "setOrigin",
    target: { kind: "collision", link: "base_link", index: 0 },
    origin: origin([0, 0, 0.1]),
  });
  ed.apply({
    type: "computeInertia",
    link: "base_link",
    mass: 1,
    bounds: { min: [-100, -50, 0], max: [100, 50, 40] },
  });
  const mi = findLink(ed.doc, "base_link")?.inertial;
  mi?.origin.xyz.forEach((v, k) => assert.ok(Math.abs(v - [0, 0, 0.12][k]) < 1e-15));
  assert.ok(Math.abs((mi?.inertia?.izz ?? 0) - (0.2 * 0.2 + 0.1 * 0.1) / 12) < 1e-15);
  assert.throws(
    () =>
      inertialFromShape(
        /** @type {any} */ (inertiaShape(/** @type {any} */ (findLink(ed.doc, "base_link")))),
        0,
      ),
    /positive/,
  );
  assert.throws(
    () =>
      inertialFromShape(
        {
          name: "",
          origin: origin(),
          geometry: /** @type {any} */ ({ type: "capsule" }),
          material: null,
        },
        1,
      ),
    /no inertia formula/,
  );
  assert.equal(inertiaShape(emptyLink("x")), null);
});

test("every generated robot passes inertia-axes after its wheels and sensors are re-computed", () => {
  for (const [id, params] of /** @type {const} */ ([
    ["mecanum", { camera: true }],
    ["diff", { camera: true }],
    ["arm", { dof: 7, lidar: true, imu: true, camera: true }],
  ])) {
    const ed = new Editor(generate(id, params));
    for (const l of ed.doc.links)
      if (l.inertial) ed.apply({ type: "computeInertia", link: l.name });
    assert.ok(
      !ids(ed).some((x) => x === "inertia-axes" || x === "inertia-implausible"),
      `${id}: ${ids(ed)}`,
    );
    // Recomputing changes nothing: the template already used these formulas.
    assert.ok(deepEqual({ ...ed.doc }, generate(id, params)), id);
  }
});

test("an edit on a 50-link model re-runs every check in under 100 ms", () => {
  const ed = new Editor(generate("arm", { dof: 48 }));
  assert.equal(ed.model.report?.summary.links, 51);
  const times = [];
  for (let k = 0; k < 20; k++) {
    const t0 = performance.now();
    ed.apply({
      type: "setOrigin",
      target: { kind: "joint", name: `joint_${(k % 48) + 1}` },
      origin: origin([0, 0, 0.15 + k * 1e-4]),
    });
    times.push(performance.now() - t0);
  }
  times.sort((a, b) => a - b);
  const median = times[10];
  const worst = times[19];
  console.log(
    `edit + checks on a 51-link model: median ${median.toFixed(1)} ms, worst ${worst.toFixed(1)} ms (20 edits)`,
  );
  assert.ok(median < 100, `median ${median} ms`);
  assert.deepEqual(errorsAndWarnings(ed), []);
});

test("modelFromDoc names a file and reports meshes it cannot resolve", () => {
  const d = generate("arm", { dof: 1 });
  d.links[1].visuals[0].geometry = {
    type: "mesh",
    filename: "package://x/m.stl",
    scale: [1, 1, 1],
  };
  const m = modelFromDoc(d);
  assert.equal(m.entry, "/builder/arm_6dof.urdf");
  assert.equal(m.meshes[0].resolved, null);
  assert.match(m.meshes[0].reason, /builder has no files/);
  assert.equal(modelFromDoc({ ...d, name: "" }).entry, "/builder/robot.urdf");
});

test("one-step actions: add child, add sensor with optical frame, collision from visual", async () => {
  const { addChild, addSensor, collisionFromVisual } =
    await import("../src/core/builder/actions.js");
  const doc = generate("diff", { lidar: false, imu: false, depth_camera: false });
  const c = addChild(doc, "base_link");
  assert.equal(c.link, "link_1");
  assert.equal(findJoint(c.doc, "link_1_joint")?.parent, "base_link");
  assert.equal(addChild(c.doc, "link_1").link, "link_2");
  const cam = addSensor(c.doc, "camera", "link_1");
  assert.equal(cam.link, "camera_link");
  assert.equal(findJoint(cam.doc, "camera_optical_joint")?.child, "camera_optical_frame");
  const cam2 = addSensor(cam.doc, "camera", "base_link", [0.1, 0, 0.1]);
  assert.equal(cam2.link, "camera_link_2");
  assert.ok(findLink(cam2.doc, "camera_2_optical_frame"));
  assert.deepEqual(findJoint(cam2.doc, "camera_2_joint")?.origin.xyz, [0.1, 0, 0.1]);
  const lidar = addSensor(cam2.doc, "lidar2d", "base_link");
  assert.equal(findLink(lidar.doc, "lidar_link")?.sensor, "lidar2d");
  const ed = new Editor(lidar.doc);
  assert.deepEqual(errorsAndWarnings(ed), []);
  assert.throws(() => addSensor(doc, /** @type {any} */ ("sonar"), "base_link"), /unknown sensor/);
  // A visual-only link gets its collision from the visual.
  const bare = clone(doc);
  /** @type {any} */ (findLink(bare, "caster_link")).collisions = [];
  const fixed = collisionFromVisual(bare, "caster_link");
  assert.deepEqual(findLink(fixed, "caster_link")?.collisions[0].geometry, {
    type: "sphere",
    radius: 0.015,
  });
  assert.throws(() => collisionFromVisual(bare, "base_footprint"), /no visual/);
});
