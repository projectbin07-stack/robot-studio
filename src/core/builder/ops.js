// Typed edit operations on a RobotDoc. Each operation is a plain object
// (so it can be logged, replayed and tested) and applyOp() returns a new
// document, leaving the input untouched; the editor keeps the old one for
// undo. Invalid operations throw EditError with a message for the user and
// change nothing.

import {
  childJointsOf,
  clone,
  findJoint,
  findLink,
  JOINT_TYPES,
  NAME_RE,
  normalizeOrder,
  parentJointOf,
  rootLinks,
  SENSOR_KINDS,
  subtree,
  uniqueName,
} from "./document.js";
import { inertialFromShape, inertiaShape } from "./inertia.js";
import {
  compose,
  fromXyzRpy,
  invert,
  matrixToRpy,
} from "../../../vendor/urdf-inspector/src/core/math.js";

/** @typedef {import("./document.js").RobotDoc} RobotDoc */
/** @typedef {import("./document.js").LinkDoc} LinkDoc */
/** @typedef {import("./document.js").JointDoc} JointDoc */
/** @typedef {import("./document.js").ShapeDoc} ShapeDoc */
/** @typedef {import("./document.js").Origin} Origin */
/** @typedef {import("./document.js").Geometry} Geometry */
/** @typedef {import("./document.js").InertialDoc} InertialDoc */
/** @typedef {import("./document.js").SensorKind} SensorKind */
/** @typedef {import("./document.js").MaterialRef} MaterialRef */
/** @typedef {import("./document.js").ExtraElement} ExtraElement */
/** @typedef {import("./document.js").Vec3} Vec3 */
/** @typedef {import("./inertia.js").MeshBounds} MeshBounds */
/** @typedef {import("../../../vendor/urdf-inspector/src/core/math.js").Transform} Transform */

/**
 * @typedef {{ kind: "joint", name: string }
 *   | { kind: "visual" | "collision", link: string, index: number }
 *   | { kind: "inertial", link: string }} OriginTarget
 */
/**
 * @typedef {Partial<Pick<JointDoc, "type" | "origin" | "axis" | "limit" | "dynamics" | "mimic">>} JointPatch
 */
/**
 * @typedef {{ type: "setRobotName", name: string }
 *   | { type: "addLink", parent: string, link: LinkDoc, joint: Omit<JointDoc, "parent" | "child"> }
 *   | { type: "removeLink", name: string }
 *   | { type: "renameLink", from: string, to: string }
 *   | { type: "reparentLink", name: string, parent: string, keepPose?: boolean }
 *   | { type: "duplicateLink", name: string }
 *   | { type: "renameJoint", from: string, to: string }
 *   | { type: "setJoint", name: string, patch: JointPatch }
 *   | { type: "setOrigin", target: OriginTarget, origin: Origin }
 *   | { type: "setGeometry", link: string, kind: "visual" | "collision", index: number, geometry: Geometry }
 *   | { type: "addShape", link: string, kind: "visual" | "collision", shape: ShapeDoc }
 *   | { type: "removeShape", link: string, kind: "visual" | "collision", index: number }
 *   | { type: "setMaterial", link: string, index: number, material: MaterialRef | null }
 *   | { type: "setMass", link: string, mass: number }
 *   | { type: "setInertial", link: string, inertial: InertialDoc | null }
 *   | { type: "computeInertia", link: string, mass?: number, bounds?: MeshBounds }
 *   | { type: "setSensor", link: string, sensor: SensorKind | null }
 *   | { type: "replaceDocument", doc: RobotDoc, label?: string }} EditOp
 */

export class EditError extends Error {}

/** @param {boolean} cond @param {string} msg @returns {asserts cond} */
function check(cond, msg) {
  if (!cond) throw new EditError(msg);
}

/** @param {unknown} v */
const finite = (v) => typeof v === "number" && Number.isFinite(v);
/** @param {unknown} v @returns {v is Vec3} */
const isVec3 = (v) => Array.isArray(v) && v.length === 3 && v.every(finite);
/** @param {Origin} o @param {string} what */
function checkOrigin(o, what) {
  check(
    !!o && isVec3(o.xyz) && isVec3(o.rpy),
    `${what}: xyz and rpy must be three finite numbers each`,
  );
}

/** @param {Geometry} g */
function checkGeometry(g) {
  check(!!g && typeof g === "object", "geometry missing");
  if (g.type === "box")
    check(isVec3(g.size) && g.size.every((v) => v > 0), "box sizes must be positive numbers");
  else if (g.type === "cylinder")
    check(
      finite(g.radius) && finite(g.length) && g.radius > 0 && g.length > 0,
      "cylinder radius and length must be positive",
    );
  else if (g.type === "sphere")
    check(finite(g.radius) && g.radius > 0, "sphere radius must be positive");
  else if (g.type === "mesh")
    check(
      typeof g.filename === "string" && g.filename !== "" && isVec3(g.scale),
      "mesh needs a file name and a scale",
    );
  else check(false, `unknown geometry type '${/** @type {{ type: string }} */ (g).type}'`);
}

/** @param {string} name @param {string} what */
function checkName(name, what) {
  check(
    typeof name === "string" && NAME_RE.test(name),
    `${what} '${name}' is not a valid name (letters, digits, _ . - /; must not start with a digit)`,
  );
}

/** @param {RobotDoc} doc @param {string} name */
function needLink(doc, name) {
  const l = findLink(doc, name);
  check(l !== null, `no link named '${name}'`);
  return l;
}
/** @param {RobotDoc} doc @param {string} name */
function needJoint(doc, name) {
  const j = findJoint(doc, name);
  check(j !== null, `no joint named '${name}'`);
  return j;
}

/** @param {RobotDoc} doc @param {string} name */
const nameTaken = (doc, name) =>
  doc.links.some((l) => l.name === name) || doc.joints.some((j) => j.name === name);

/**
 * Replace attribute values equal to `from` with `to` in carried-through
 * elements, for the attributes that hold link or joint names.
 * @param {ExtraElement[]} extras
 * @param {"link" | "joint"} kind
 * @param {string} from
 * @param {string} to
 */
function renameInExtras(extras, kind, from, to) {
  /** @param {ExtraElement} x */
  const walk = (x) => {
    for (const [k, v] of Object.entries(x.attrs)) {
      if (v !== from) continue;
      const linkAttr =
        k === "reference" || k === "link" || k === "frame" || k === "parent" || k === "child";
      const jointAttr = k === "reference" || k === "joint" || (k === "name" && x.name === "joint");
      if ((kind === "link" && linkAttr) || (kind === "joint" && jointAttr)) x.attrs[k] = to;
    }
    // Plugin parameters often name frames in text: <frame_name>laser</frame_name>.
    if (kind === "link" && x.text === from && /frame|link|body/i.test(x.name)) x.text = to;
    x.children.forEach(walk);
  };
  extras.forEach(walk);
}

/**
 * Root-frame transform of every link at the zero configuration.
 * @param {RobotDoc} doc
 * @returns {Map<string, Transform>}
 */
export function linkPoses(doc) {
  /** @type {Map<string, Transform>} */
  const out = new Map();
  /** @param {string} name @param {Transform} t */
  const walk = (name, t) => {
    if (out.has(name)) return;
    out.set(name, t);
    for (const j of childJointsOf(doc, name))
      walk(j.child, compose(t, fromXyzRpy(j.origin.xyz, j.origin.rpy)));
  };
  for (const r of rootLinks(doc)) walk(r, fromXyzRpy([0, 0, 0], [0, 0, 0]));
  return out;
}

/** @param {number} v */
const tidy = (v) => (Math.abs(v) < 1e-12 ? 0 : v);

/**
 * Apply one operation; returns a new document.
 * @param {RobotDoc} input
 * @param {EditOp} op
 * @returns {RobotDoc}
 */
export function applyOp(input, op) {
  if (op.type === "replaceDocument") {
    check(
      !!op.doc && op.doc.format === 1 && Array.isArray(op.doc.links),
      "not a Robot Studio document",
    );
    const d = clone(op.doc);
    normalizeOrder(d);
    return d;
  }
  const doc = clone(input);
  switch (op.type) {
    case "setRobotName": {
      check(
        typeof op.name === "string" && /^[A-Za-z0-9_.\-]{1,128}$/.test(op.name),
        `robot name '${op.name}' must be letters, digits, _ . -`,
      );
      doc.name = op.name;
      if (doc.template) doc.template.params.name = op.name;
      break;
    }
    case "addLink": {
      needLink(doc, op.parent);
      checkName(op.link.name, "link");
      checkName(op.joint.name, "joint");
      check(op.link.name !== op.joint.name, "the link and its joint need different names");
      check(!nameTaken(doc, op.link.name), `'${op.link.name}' is already used`);
      check(!nameTaken(doc, op.joint.name), `'${op.joint.name}' is already used`);
      check(JOINT_TYPES.includes(op.joint.type), `unknown joint type '${op.joint.type}'`);
      checkOrigin(op.joint.origin, "joint origin");
      for (const s of [...op.link.visuals, ...op.link.collisions]) checkGeometry(s.geometry);
      const link = clone(op.link);
      const parentIndex = doc.links.findIndex((l) => l.name === op.parent);
      // Insert after the parent's subtree so the tree reads in order.
      const after = subtree(doc, op.parent);
      const last = Math.max(
        ...after.map((n) => doc.links.findIndex((l) => l.name === n)),
        parentIndex,
      );
      doc.links.splice(last + 1, 0, link);
      doc.joints.push({ ...clone(op.joint), parent: op.parent, child: link.name });
      break;
    }
    case "removeLink": {
      needLink(doc, op.name);
      const roots = rootLinks(doc);
      check(
        !(roots.includes(op.name) && parentJointOf(doc, op.name) === null && roots.length === 1),
        `'${op.name}' is the root link and cannot be deleted`,
      );
      const gone = new Set(subtree(doc, op.name));
      const goneJoints = new Set(doc.joints.filter((j) => gone.has(j.child)).map((j) => j.name));
      doc.links = doc.links.filter((l) => !gone.has(l.name));
      doc.joints = doc.joints.filter((j) => !goneJoints.has(j.name));
      for (const j of doc.joints) if (j.mimic && goneJoints.has(j.mimic.joint)) j.mimic = null;
      break;
    }
    case "renameLink": {
      const link = needLink(doc, op.from);
      if (op.from === op.to) break;
      checkName(op.to, "link");
      check(!nameTaken(doc, op.to), `'${op.to}' is already used`);
      link.name = op.to;
      for (const j of doc.joints) {
        if (j.parent === op.from) j.parent = op.to;
        if (j.child === op.from) j.child = op.to;
      }
      renameInExtras(doc.extras, "link", op.from, op.to);
      break;
    }
    case "reparentLink": {
      needLink(doc, op.name);
      needLink(doc, op.parent);
      const j = parentJointOf(doc, op.name);
      check(j !== null, `'${op.name}' is the root link; it has no parent to change`);
      check(
        !subtree(doc, op.name).includes(op.parent),
        `'${op.parent}' is below '${op.name}'; that would make a loop`,
      );
      if (j.parent === op.parent) break;
      if (op.keepPose !== false) {
        const poses = linkPoses(doc);
        const p = poses.get(op.parent);
        const c = poses.get(op.name);
        if (p && c) {
          const rel = compose(invert(p), c);
          j.origin = {
            xyz: [tidy(rel.p[0]), tidy(rel.p[1]), tidy(rel.p[2])],
            rpy: /** @type {Vec3} */ (matrixToRpy(rel.R).map(tidy)),
          };
        }
      }
      j.parent = op.parent;
      // Move the subtree after its new parent's subtree.
      const moving = subtree(doc, op.name);
      const moved = doc.links.filter((l) => moving.includes(l.name));
      doc.links = doc.links.filter((l) => !moving.includes(l.name));
      const after = subtree(doc, op.parent);
      const last = Math.max(...after.map((n) => doc.links.findIndex((l) => l.name === n)));
      doc.links.splice(last + 1, 0, ...moved);
      break;
    }
    case "duplicateLink": {
      needLink(doc, op.name);
      const j = parentJointOf(doc, op.name);
      check(j !== null, "the root link cannot be duplicated (it would be a second root)");
      const names = subtree(doc, op.name);
      /** @type {Map<string, string>} */
      const map = new Map();
      /** @type {string[]} */
      const reserved = [];
      for (const n of names) {
        const nn = uniqueName(doc, `${n}_copy`, reserved);
        reserved.push(nn);
        map.set(n, nn);
      }
      const joints = doc.joints.filter((x) => names.includes(x.child));
      /** @type {Map<string, string>} */
      const jmap = new Map();
      for (const x of joints) {
        const nn = uniqueName(doc, `${x.name}_copy`, reserved);
        reserved.push(nn);
        jmap.set(x.name, nn);
      }
      const copies = names.map((n) => ({
        ...clone(/** @type {LinkDoc} */ (findLink(doc, n))),
        name: /** @type {string} */ (map.get(n)),
      }));
      const after = subtree(doc, op.name);
      const last = Math.max(...after.map((n) => doc.links.findIndex((l) => l.name === n)));
      doc.links.splice(last + 1, 0, ...copies);
      for (const x of joints) {
        const c = clone(x);
        c.name = /** @type {string} */ (jmap.get(x.name));
        c.child = /** @type {string} */ (map.get(x.child));
        c.parent = map.get(x.parent) ?? x.parent;
        if (c.mimic && jmap.has(c.mimic.joint))
          c.mimic.joint = /** @type {string} */ (jmap.get(c.mimic.joint));
        doc.joints.push(c);
      }
      break;
    }
    case "renameJoint": {
      const j = needJoint(doc, op.from);
      if (op.from === op.to) break;
      checkName(op.to, "joint");
      check(!nameTaken(doc, op.to), `'${op.to}' is already used`);
      j.name = op.to;
      for (const x of doc.joints) if (x.mimic?.joint === op.from) x.mimic.joint = op.to;
      renameInExtras(doc.extras, "joint", op.from, op.to);
      break;
    }
    case "setJoint": {
      const j = needJoint(doc, op.name);
      const p = op.patch;
      if (p.type !== undefined) {
        check(JOINT_TYPES.includes(p.type), `unknown joint type '${p.type}'`);
        j.type = p.type;
        if (p.type === "fixed") {
          j.axis = null;
          j.limit = null;
          j.mimic = null;
        } else if ((p.type === "revolute" || p.type === "prismatic") && p.limit === undefined) {
          const ang = p.type === "revolute";
          const l = j.limit;
          j.limit = {
            lower: l?.lower ?? (ang ? -Math.PI / 2 : 0),
            upper: l?.upper ?? (ang ? Math.PI / 2 : 0.1),
            effort: l?.effort ?? 10,
            velocity: l?.velocity ?? (ang ? 1 : 0.1),
          };
        }
        if (p.type !== "fixed" && p.type !== "floating" && j.axis === null && p.axis === undefined)
          j.axis = [0, 0, 1];
      }
      if (p.origin !== undefined) {
        checkOrigin(p.origin, "joint origin");
        j.origin = clone(p.origin);
      }
      if (p.axis !== undefined) {
        check(
          p.axis === null || (isVec3(p.axis) && p.axis.some((v) => v !== 0)),
          "axis must be three numbers, not all zero",
        );
        j.axis = p.axis ? [p.axis[0], p.axis[1], p.axis[2]] : null;
      }
      if (p.limit !== undefined) {
        if (p.limit !== null)
          for (const k of /** @type {const} */ (["lower", "upper", "effort", "velocity"])) {
            const v = p.limit[k];
            check(v === null || finite(v), `limit ${k} must be a number`);
          }
        j.limit = p.limit ? { ...p.limit } : null;
      }
      if (p.dynamics !== undefined) {
        check(
          p.dynamics === null || (finite(p.dynamics.damping) && finite(p.dynamics.friction)),
          "damping and friction must be numbers",
        );
        j.dynamics = p.dynamics ? { ...p.dynamics } : null;
      }
      if (p.mimic !== undefined) {
        if (p.mimic) {
          needJoint(doc, p.mimic.joint);
          check(p.mimic.joint !== j.name, "a joint cannot mimic itself");
          check(
            finite(p.mimic.multiplier) && finite(p.mimic.offset),
            "mimic multiplier and offset must be numbers",
          );
        }
        j.mimic = p.mimic ? { ...p.mimic } : null;
      }
      break;
    }
    case "setOrigin": {
      checkOrigin(op.origin, "origin");
      const o = clone(op.origin);
      const t = op.target;
      if (t.kind === "joint") needJoint(doc, t.name).origin = o;
      else if (t.kind === "inertial") {
        const l = needLink(doc, t.link);
        check(l.inertial !== null, `link '${t.link}' has no inertial`);
        l.inertial.origin = o;
      } else {
        const l = needLink(doc, t.link);
        const list = t.kind === "visual" ? l.visuals : l.collisions;
        check(
          t.index >= 0 && t.index < list.length,
          `link '${t.link}' has no ${t.kind} ${t.index + 1}`,
        );
        list[t.index].origin = o;
      }
      break;
    }
    case "setGeometry": {
      const l = needLink(doc, op.link);
      checkGeometry(op.geometry);
      const list = op.kind === "visual" ? l.visuals : l.collisions;
      check(
        op.index >= 0 && op.index < list.length,
        `link '${op.link}' has no ${op.kind} ${op.index + 1}`,
      );
      list[op.index].geometry = clone(op.geometry);
      break;
    }
    case "addShape": {
      const l = needLink(doc, op.link);
      checkGeometry(op.shape.geometry);
      checkOrigin(op.shape.origin, `${op.kind} origin`);
      (op.kind === "visual" ? l.visuals : l.collisions).push({
        ...clone(op.shape),
        material: op.kind === "visual" ? clone(op.shape.material) : null,
      });
      break;
    }
    case "removeShape": {
      const l = needLink(doc, op.link);
      const list = op.kind === "visual" ? l.visuals : l.collisions;
      check(
        op.index >= 0 && op.index < list.length,
        `link '${op.link}' has no ${op.kind} ${op.index + 1}`,
      );
      list.splice(op.index, 1);
      break;
    }
    case "setMaterial": {
      const l = needLink(doc, op.link);
      check(
        op.index >= 0 && op.index < l.visuals.length,
        `link '${op.link}' has no visual ${op.index + 1}`,
      );
      if (op.material?.rgba)
        check(
          op.material.rgba.length === 4 &&
            op.material.rgba.every((v) => finite(v) && v >= 0 && v <= 1),
          "rgba must be four numbers from 0 to 1",
        );
      l.visuals[op.index].material = op.material ? clone(op.material) : null;
      break;
    }
    case "setMass": {
      const l = needLink(doc, op.link);
      check(finite(op.mass) && op.mass > 0, "mass must be a positive number");
      const i = l.inertial;
      if (i && i.inertia && i.mass && i.mass > 0) {
        // Same shape, different mass: the tensor scales with the mass.
        const f = op.mass / i.mass;
        const t = i.inertia;
        i.inertia = {
          ixx: t.ixx * f,
          ixy: t.ixy * f,
          ixz: t.ixz * f,
          iyy: t.iyy * f,
          iyz: t.iyz * f,
          izz: t.izz * f,
        };
        i.mass = op.mass;
      } else {
        const s = inertiaShape(l);
        check(
          s !== null && s.geometry.type !== "mesh",
          `link '${op.link}' has no inertia to scale and no primitive shape to compute one from`,
        );
        l.inertial = inertialFromShape(s, op.mass);
      }
      break;
    }
    case "setInertial": {
      const l = needLink(doc, op.link);
      if (op.inertial) {
        checkOrigin(op.inertial.origin, "inertial origin");
        check(op.inertial.mass === null || finite(op.inertial.mass), "mass must be a number");
        if (op.inertial.inertia)
          check(Object.values(op.inertial.inertia).every(finite), "inertia terms must be numbers");
      }
      l.inertial = op.inertial ? clone(op.inertial) : null;
      break;
    }
    case "computeInertia": {
      const l = needLink(doc, op.link);
      const s = inertiaShape(l);
      check(s !== null, `link '${op.link}' has no geometry to compute an inertia from`);
      const mass = op.mass ?? l.inertial?.mass ?? null;
      check(
        mass !== null && finite(mass) && mass > 0,
        `link '${op.link}' needs a positive mass first`,
      );
      check(
        s.geometry.type !== "mesh" || op.bounds !== undefined,
        "a mesh needs its bounding box: wait until it is shown in the 3D view",
      );
      l.inertial = inertialFromShape(s, mass, op.bounds);
      break;
    }
    case "setSensor": {
      const l = needLink(doc, op.link);
      check(
        op.sensor === null || SENSOR_KINDS.includes(op.sensor),
        `unknown sensor kind '${op.sensor}'`,
      );
      l.sensor = op.sensor;
      break;
    }
    default:
      throw new EditError(`unknown operation '${/** @type {{ type: string }} */ (op).type}'`);
  }
  normalizeOrder(doc);
  return doc;
}

/**
 * A short description of an operation for the undo/redo buttons.
 * @param {EditOp} op
 */
export function describeOp(op) {
  switch (op.type) {
    case "setRobotName":
      return `rename robot to ${op.name}`;
    case "addLink":
      return `add ${op.link.name}`;
    case "removeLink":
      return `delete ${op.name}`;
    case "renameLink":
    case "renameJoint":
      return `rename ${op.from} to ${op.to}`;
    case "reparentLink":
      return `move ${op.name} under ${op.parent}`;
    case "duplicateLink":
      return `duplicate ${op.name}`;
    case "setJoint":
      return `edit ${op.name}`;
    case "setOrigin":
      return `move ${op.target.kind === "joint" ? op.target.name : `${op.target.link} ${op.target.kind}`}`;
    case "setGeometry":
    case "addShape":
    case "removeShape":
      return `${op.type === "addShape" ? "add" : op.type === "removeShape" ? "remove" : "edit"} ${op.kind} of ${op.link}`;
    case "setMaterial":
      return `colour of ${op.link}`;
    case "setMass":
      return `mass of ${op.link}`;
    case "setInertial":
    case "computeInertia":
      return `inertia of ${op.link}`;
    case "setSensor":
      return `sensor of ${op.link}`;
    case "replaceDocument":
      return op.label ?? "new robot";
  }
}

export { childJointsOf };
