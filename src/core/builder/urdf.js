// RobotDoc <-> URDF. The writer builds the same element tree the xacro
// processor produces and prints it with its serializer (xacro's layout),
// so an exported URDF and the expansion of an exported xacro can be
// compared line by line. The reader uses urdf-inspector's URDF reader, so
// a document imports exactly what the checks see.
//
// Element order: top-level materials, then every link followed by the
// joint that has it as child, then the carried-through elements. Inside a
// link: inertial, visuals, collisions; inside a joint: parent, child,
// origin, axis, limit, dynamics, mimic. Origins are always written.

import { parseXml } from "../../../vendor/urdf-inspector/src/core/xml.js";
import { readUrdf } from "../../../vendor/urdf-inspector/src/core/urdf.js";
import { serializeXml } from "../xacro/serialize.js";
import { emptyDoc, guessSensor, JOINT_TYPES, normalizeOrder } from "./document.js";
import { VERSION } from "../version.js";

/** @typedef {import("./document.js").RobotDoc} RobotDoc */
/** @typedef {import("./document.js").Origin} Origin */
/** @typedef {import("./document.js").ShapeDoc} ShapeDoc */
/** @typedef {import("./document.js").Geometry} Geometry */
/** @typedef {import("./document.js").ExtraElement} ExtraElement */
/** @typedef {import("./document.js").JointType} JointType */
/** @typedef {import("../xacro/dom.js").XElement} XElement */
/** @typedef {import("../xacro/dom.js").XNode} XNode */
/** @typedef {import("../../../vendor/urdf-inspector/src/core/xml.js").XmlElement} XmlElement */

/**
 * Number as written to URDF: the shortest text that reads back as the same
 * double (JavaScript's own number printing), without a negative zero.
 * @param {number} v
 */
export const num = (v) => (Object.is(v, -0) ? "0" : String(v));

/** @param {number[]} v */
export const nums = (v) => v.map(num).join(" ");

/**
 * @param {string} name
 * @param {Record<string, string>} [attrs]
 * @param {XNode[]} [children]
 * @returns {XElement}
 */
export const el = (name, attrs = {}, children = []) => ({
  type: "element",
  name,
  attrs,
  children,
  file: "",
  line: 0,
});

/** @param {Origin} o */
const originEl = (o) => el("origin", { xyz: nums(o.xyz), rpy: nums(o.rpy) });

/** @param {Geometry} g */
export function geometryEl(g) {
  /** @type {XElement} */
  let shape;
  if (g.type === "box") shape = el("box", { size: nums(g.size) });
  else if (g.type === "cylinder")
    shape = el("cylinder", { radius: num(g.radius), length: num(g.length) });
  else if (g.type === "sphere") shape = el("sphere", { radius: num(g.radius) });
  else {
    /** @type {Record<string, string>} */
    const attrs = { filename: g.filename };
    if (g.scale.some((s) => s !== 1)) attrs.scale = nums(g.scale);
    shape = el("mesh", attrs);
  }
  return el("geometry", {}, [shape]);
}

/**
 * @param {"visual" | "collision"} kind
 * @param {ShapeDoc} s
 */
function shapeEl(kind, s) {
  /** @type {XElement[]} */
  const kids = [originEl(s.origin), geometryEl(s.geometry)];
  if (kind === "visual" && s.material) {
    const m = el("material", s.material.name ? { name: s.material.name } : {});
    if (s.material.rgba) m.children.push(el("color", { rgba: nums(s.material.rgba) }));
    kids.push(m);
  }
  return el(kind, s.name ? { name: s.name } : {}, kids);
}

/** @param {ExtraElement} x @returns {XElement} */
function extraEl(x) {
  /** @type {XNode[]} */
  const kids = x.text.trim() ? [{ type: "text", value: x.text.trim() }] : [];
  for (const c of x.children) kids.push(extraEl(c));
  return el(x.name, { ...x.attrs }, kids);
}

/**
 * The document as an element tree (the xacro processor's node type).
 * @param {RobotDoc} doc
 * @returns {XElement}
 */
export function docToElement(doc) {
  const robot = el("robot", { name: doc.name });
  for (const m of doc.materials)
    robot.children.push(
      el("material", { name: m.name }, m.rgba ? [el("color", { rgba: nums(m.rgba) })] : []),
    );
  const jointOf = new Map(doc.joints.map((j) => [j.child, j]));
  const written = new Set();
  /** @param {import("./document.js").JointDoc} j */
  const jointEl = (j) => {
    const kids = [
      el("parent", { link: j.parent }),
      el("child", { link: j.child }),
      originEl(j.origin),
    ];
    if (j.axis) kids.push(el("axis", { xyz: nums(j.axis) }));
    if (j.limit) {
      /** @type {Record<string, string>} */
      const a = {};
      for (const k of /** @type {const} */ (["lower", "upper", "effort", "velocity"])) {
        const v = j.limit[k];
        if (v !== null) a[k] = num(v);
      }
      kids.push(el("limit", a));
    }
    if (j.dynamics)
      kids.push(
        el("dynamics", { damping: num(j.dynamics.damping), friction: num(j.dynamics.friction) }),
      );
    if (j.mimic)
      kids.push(
        el("mimic", {
          joint: j.mimic.joint,
          multiplier: num(j.mimic.multiplier),
          offset: num(j.mimic.offset),
        }),
      );
    return el("joint", { name: j.name, type: j.type }, kids);
  };
  for (const link of doc.links) {
    /** @type {XElement[]} */
    const kids = [];
    const i = link.inertial;
    if (i) {
      const ik = [originEl(i.origin)];
      if (i.mass !== null) ik.push(el("mass", { value: num(i.mass) }));
      if (i.inertia) {
        const t = i.inertia;
        ik.push(
          el("inertia", {
            ixx: num(t.ixx),
            ixy: num(t.ixy),
            ixz: num(t.ixz),
            iyy: num(t.iyy),
            iyz: num(t.iyz),
            izz: num(t.izz),
          }),
        );
      }
      kids.push(el("inertial", {}, ik));
    }
    for (const s of link.visuals) kids.push(shapeEl("visual", s));
    for (const s of link.collisions) kids.push(shapeEl("collision", s));
    robot.children.push(el("link", { name: link.name }, kids));
    const j = jointOf.get(link.name);
    if (j) {
      robot.children.push(jointEl(j));
      written.add(j);
    }
  }
  for (const j of doc.joints) if (!written.has(j)) robot.children.push(jointEl(j));
  for (const x of doc.extras) robot.children.push(extraEl(x));
  return robot;
}

/**
 * URDF text of the document.
 * @param {RobotDoc} doc
 * @param {{ comment?: string | null }} [opts] header comment (default: generator and template)
 */
export function toUrdf(doc, opts = {}) {
  const root = docToElement(doc);
  const comment =
    opts.comment !== undefined
      ? opts.comment
      : `Generated by Robot Studio ${VERSION}` +
        (doc.template ? ` from the '${doc.template.id}' template` : "") +
        ". Software-generated: check masses, dimensions and limits against the real robot.";
  if (comment) root.prolog = [{ type: "comment", value: ` ${comment.replace(/--/g, "- -")} ` }];
  return serializeXml(root);
}

/**
 * @typedef {object} ImportResult
 * @property {RobotDoc | null} doc
 * @property {string[]} notes what could not be carried over
 */

/**
 * Import URDF text (already expanded if it was xacro) as an editable document.
 * @param {string} text
 * @param {{ entry?: string }} [opts]
 * @returns {ImportResult}
 */
export function importUrdf(text, opts = {}) {
  /** @type {string[]} */
  const notes = [];
  /** @type {XmlElement} */
  let root;
  try {
    root = parseXml(text);
  } catch (e) {
    return {
      doc: null,
      notes: [`not well-formed XML: ${e instanceof Error ? e.message : String(e)}`],
    };
  }
  const { robot, findings } = readUrdf(root);
  if (!robot) return { doc: null, notes: findings.map((f) => f.message) };
  const doc = emptyDoc(robot.name || "robot");
  doc.source = opts.entry ? { entry: opts.entry } : null;
  for (const [name, m] of robot.materials)
    doc.materials.push({ name, rgba: m.rgba ? [...m.rgba] : null });
  for (const l of robot.links) {
    /** @param {import("../../../vendor/urdf-inspector/src/core/urdf.js").Shape[]} list */
    const shapes = (list) =>
      /** @type {ShapeDoc[]} */ (
        list
          .map((s) => {
            const g = s.geometry;
            if (!g || g.type === "unknown") {
              notes.push(
                `${s.kind} of link '${l.name}' has no box, cylinder, sphere or mesh geometry and was left out`,
              );
              return null;
            }
            return {
              name: s.name,
              origin: { xyz: [...s.origin.xyz], rpy: [...s.origin.rpy] },
              geometry: copyGeometry(g),
              material: s.material
                ? { name: s.material.name, rgba: s.material.rgba ? [...s.material.rgba] : null }
                : null,
            };
          })
          .filter((s) => s !== null)
      );
    doc.links.push({
      name: l.name,
      sensor: guessSensor(l.name),
      inertial: l.inertial
        ? {
            origin: { xyz: [...l.inertial.origin.xyz], rpy: [...l.inertial.origin.rpy] },
            mass: l.inertial.mass,
            inertia: l.inertial.inertia ? { ...l.inertial.inertia } : null,
          }
        : null,
      visuals: shapes(l.visuals),
      collisions: shapes(l.collisions).map((s) => ({ ...s, material: null })),
    });
  }
  for (const j of robot.joints) {
    const type = /** @type {JointType} */ (
      JOINT_TYPES.includes(/** @type {JointType} */ (j.type)) ? j.type : "fixed"
    );
    if (type !== j.type) notes.push(`joint '${j.name}' has type '${j.type}'; imported as fixed`);
    doc.joints.push({
      name: j.name,
      type,
      parent: j.parent,
      child: j.child,
      origin: { xyz: [...j.origin.xyz], rpy: [...j.origin.rpy] },
      axis: j.axisGiven ? [...j.axis] : null,
      limit: j.limit
        ? {
            lower: j.limit.lower,
            upper: j.limit.upper,
            effort: j.limit.effort,
            velocity: j.limit.velocity,
          }
        : null,
      dynamics: j.dynamics ? { ...j.dynamics } : null,
      mimic: j.mimic ? { ...j.mimic } : null,
    });
  }
  for (const c of root.children)
    if (!["link", "joint", "material"].includes(c.name)) doc.extras.push(toExtra(c));
  for (const f of findings) if (f.severity === "error") notes.push(`${f.id}: ${f.message}`);
  normalizeOrder(doc);
  return { doc, notes };
}

/**
 * @param {import("../../../vendor/urdf-inspector/src/core/urdf.js").Geometry} g
 * @returns {Geometry}
 */
function copyGeometry(g) {
  if (g.type === "box") return { type: "box", size: [...g.size] };
  if (g.type === "cylinder") return { type: "cylinder", radius: g.radius, length: g.length };
  if (g.type === "sphere") return { type: "sphere", radius: g.radius };
  if (g.type === "mesh") return { type: "mesh", filename: g.filename, scale: [...g.scale] };
  throw new Error("unknown geometry");
}

/** @param {XmlElement} x @returns {ExtraElement} */
function toExtra(x) {
  return {
    name: x.name,
    attrs: { ...x.attrs },
    children: x.children.map(toExtra),
    text: x.text.trim(),
  };
}
