// Builder panels: the template parameter form and the properties editor
// for the selected link or joint. Pure DOM construction; every change is
// handed to the app as an edit operation (src/core/builder/ops.js), which
// applies it, re-runs the checks and redraws. Values are inserted as text,
// never as HTML.

import {
  JOINT_TYPES,
  SENSOR_KINDS,
  findJoint,
  findLink,
  parentJointOf,
  subtree,
} from "../core/builder/document.js";

/** @typedef {import("../core/builder/document.js").RobotDoc} RobotDoc */
/** @typedef {import("../core/builder/document.js").Vec3} Vec3 */
/** @typedef {import("../core/builder/document.js").Geometry} Geometry */
/** @typedef {import("../core/builder/document.js").ShapeDoc} ShapeDoc */
/** @typedef {import("../core/builder/ops.js").EditOp} EditOp */
/** @typedef {import("../core/builder/templates.js").Template} Template */

/**
 * @param {string} tag
 * @param {Record<string, string>} [attrs]
 * @param {(Node | string)[]} [children]
 */
export function h(tag, attrs = {}, children = []) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") el.className = v;
    else el.setAttribute(k, v);
  }
  for (const c of children) el.append(c);
  return el;
}

/** Text of a number in a field: full precision, no -0. @param {number} v */
const show = (v) => (Object.is(v, -0) ? "0" : String(Number(v.toPrecision(12))));

/**
 * A number input that reports a finite value on change (Enter or leaving
 * the field); invalid text is marked and not reported.
 * @param {number | null} value
 * @param {(v: number) => void} onCommit
 * @param {Record<string, string>} [attrs]
 */
function numberInput(value, onCommit, attrs = {}) {
  const input = /** @type {HTMLInputElement} */ (
    h("input", {
      type: "text",
      inputmode: "decimal",
      spellcheck: "false",
      value: value === null ? "" : show(value),
      ...attrs,
    })
  );
  input.addEventListener("change", () => {
    const v = Number(input.value.trim());
    const ok = input.value.trim() !== "" && Number.isFinite(v);
    input.classList.toggle("invalid", !ok);
    if (ok) onCommit(v);
  });
  return input;
}

/**
 * @param {string} label
 * @param {Node[]} inputs
 * @param {string} [unit]
 */
const row = (label, inputs, unit = "") =>
  h("div", { class: "prop" }, [
    h("span", { class: "prop-label" }, [label]),
    h("span", { class: "prop-inputs" }, inputs),
    h("span", { class: "unit" }, [unit]),
  ]);

/**
 * Three number inputs.
 * @param {string} label
 * @param {Vec3} v
 * @param {(v: Vec3) => void} onCommit
 * @param {string} unit
 * @param {string} key data-key for tests and labels
 */
function vec3Row(label, v, onCommit, unit, key) {
  const cur = /** @type {Vec3} */ ([...v]);
  const inputs = [0, 1, 2].map((k) =>
    numberInput(
      v[k],
      (x) => {
        cur[k] = x;
        onCommit(/** @type {Vec3} */ ([...cur]));
      },
      { "aria-label": `${label} ${"xyz"[k]}`, "data-key": `${key}.${k}` },
    ),
  );
  return row(label, inputs, unit);
}

/**
 * @param {string} label
 * @param {string} value
 * @param {(v: string) => void} onCommit
 * @param {string} key
 */
function textRow(label, value, onCommit, key) {
  const input = /** @type {HTMLInputElement} */ (
    h("input", { type: "text", value, spellcheck: "false", "aria-label": label, "data-key": key })
  );
  input.addEventListener("change", () => onCommit(input.value.trim()));
  return row(label, [input]);
}

/**
 * @param {string} label
 * @param {string} value
 * @param {string[]} options
 * @param {(v: string) => void} onCommit
 * @param {string} key
 */
function selectRow(label, value, options, onCommit, key) {
  const sel = /** @type {HTMLSelectElement} */ (
    h(
      "select",
      { "aria-label": label, "data-key": key },
      options.map((o) => h("option", { value: o }, [o])),
    )
  );
  sel.value = value;
  sel.addEventListener("change", () => onCommit(sel.value));
  return row(label, [sel]);
}

/** @param {string} text @param {() => void} onClick @param {Record<string, string>} [attrs] */
function button(text, onClick, attrs = {}) {
  const b = h("button", { type: "button", ...attrs }, [text]);
  b.addEventListener("click", onClick);
  return b;
}

// ---------------------------------------------------------------- template form

/**
 * Parameter fields of a template, grouped. Sensor parameters are laid out
 * as one switch with position and orientation each.
 * @param {HTMLElement} host
 * @param {Template} t
 * @param {Record<string, any>} params current values
 * @param {(key: string, value: unknown) => void} onChange
 */
export function renderTemplateForm(host, t, params, onChange) {
  /** @type {Map<string, HTMLElement>} */
  const groups = new Map();
  for (const p of t.params) {
    let g = groups.get(p.group);
    if (!g) {
      g = h("fieldset", { class: "params" }, [h("legend", {}, [p.group])]);
      groups.set(p.group, g);
    }
    const v = params[p.key];
    const unit = p.unit ?? "";
    if (p.kind === "bool") {
      const box = /** @type {HTMLInputElement} */ (
        h("input", { type: "checkbox", "data-param": p.key })
      );
      box.checked = Boolean(v);
      box.addEventListener("change", () => onChange(p.key, box.checked));
      g.append(h("label", { class: "toggle param-switch" }, [box, ` ${p.label}`]));
    } else if (p.kind === "vec3") {
      const cur = /** @type {Vec3} */ ([...v]);
      const inputs = [0, 1, 2].map((k) =>
        numberInput(
          cur[k],
          (x) => {
            cur[k] = x;
            onChange(p.key, [...cur]);
          },
          { "data-param": `${p.key}.${k}`, "aria-label": `${p.label} ${"xyz"[k]}` },
        ),
      );
      g.append(row(p.label.replace(/^.*? (position|roll pitch yaw)$/, "$1"), inputs, unit));
    } else if (p.kind === "text") {
      const input = /** @type {HTMLInputElement} */ (
        h("input", { type: "text", value: String(v), "data-param": p.key, spellcheck: "false" })
      );
      input.addEventListener("change", () => onChange(p.key, input.value.trim()));
      g.append(row(p.label, [input]));
    } else {
      g.append(
        row(
          p.label,
          [
            numberInput(v, (x) => onChange(p.key, x), {
              "data-param": p.key,
              "aria-label": p.label,
              title: `${p.min ?? ""} … ${p.max ?? ""}`,
            }),
          ],
          unit,
        ),
      );
    }
  }
  host.replaceChildren(...groups.values());
}

// ---------------------------------------------------------------- properties

/**
 * @typedef {object} PropertyHooks
 * @property {(op: EditOp) => void} apply
 * @property {(link: string) => void} addChild
 * @property {(link: string, kind: import("../core/builder/document.js").SensorKind) => void} addSensor
 * @property {(link: string) => void} collisionFromVisual
 * @property {(link: string, kind: "visual" | "collision", index: number) => import("../core/builder/inertia.js").MeshBounds | undefined} meshBounds
 */

/** @param {Geometry["type"]} type @param {Geometry} old @returns {Geometry} */
function convertGeometry(type, old) {
  const size =
    old.type === "box"
      ? old.size
      : old.type === "cylinder"
        ? [old.radius * 2, old.radius * 2, old.length]
        : old.type === "sphere"
          ? [old.radius * 2, old.radius * 2, old.radius * 2]
          : [0.1, 0.1, 0.1];
  if (type === "box") return { type, size: /** @type {Vec3} */ ([...size]) };
  if (type === "cylinder") return { type, radius: Math.max(size[0], size[1]) / 2, length: size[2] };
  if (type === "sphere") return { type, radius: Math.max(...size) / 2 };
  return {
    type: "mesh",
    filename: old.type === "mesh" ? old.filename : "package://my_robot_description/meshes/part.stl",
    scale: [1, 1, 1],
  };
}

/** @param {number[] | null} rgba */
const toHex = (rgba) =>
  "#" +
  (rgba ?? [0.75, 0.76, 0.78, 1])
    .slice(0, 3)
    .map((c) =>
      Math.round(Math.min(1, Math.max(0, c)) * 255)
        .toString(16)
        .padStart(2, "0"),
    )
    .join("");

/**
 * Editable properties of the selected link or joint (or the robot).
 * @param {HTMLElement} host
 * @param {RobotDoc} doc
 * @param {{ kind: "link" | "joint", name: string } | null} sel
 * @param {PropertyHooks} hooks
 */
export function renderProperties(host, doc, sel, hooks) {
  const { apply } = hooks;
  /** @type {Node[]} */
  const out = [];
  if (!sel) {
    out.push(h("h3", {}, ["Robot"]));
    out.push(
      textRow("Name", doc.name, (v) => apply({ type: "setRobotName", name: v }), "robot.name"),
    );
    out.push(
      h("p", { class: "muted" }, [
        "Select a link or joint in the 3D view or the tree to edit it. Ctrl+Z / Ctrl+Y undo and redo, Delete removes the selected link.",
      ]),
    );
    host.replaceChildren(...out);
    return;
  }
  const link = sel.kind === "link" ? findLink(doc, sel.name) : null;
  const joint =
    sel.kind === "joint" ? findJoint(doc, sel.name) : link ? parentJointOf(doc, link.name) : null;

  if (link) {
    out.push(h("h3", {}, [`Link ${link.name}`]));
    out.push(
      textRow(
        "Name",
        link.name,
        (v) => apply({ type: "renameLink", from: link.name, to: v }),
        "link.name",
      ),
    );
    out.push(
      selectRow(
        "Sensor",
        link.sensor ?? "none",
        ["none", ...SENSOR_KINDS],
        (v) =>
          apply({
            type: "setSensor",
            link: link.name,
            sensor: v === "none" ? null : /** @type {any} */ (v),
          }),
        "link.sensor",
      ),
    );
    const sensorPick = /** @type {HTMLSelectElement} */ (
      h("select", { "aria-label": "Add a sensor", "data-key": "add-sensor" }, [
        h("option", { value: "" }, ["Add sensor…"]),
        ...SENSOR_KINDS.map((k) => h("option", { value: k }, [k.replace("_", " ")])),
      ])
    );
    sensorPick.addEventListener("change", () => {
      if (sensorPick.value) hooks.addSensor(link.name, /** @type {any} */ (sensorPick.value));
    });
    out.push(
      h("div", { class: "row actions" }, [
        button("Add child", () => hooks.addChild(link.name), { "data-action": "add-child" }),
        sensorPick,
        ...(joint
          ? [
              button("Duplicate", () => apply({ type: "duplicateLink", name: link.name }), {
                "data-action": "duplicate",
              }),
            ]
          : []),
        ...(joint
          ? [
              button("Delete", () => apply({ type: "removeLink", name: link.name }), {
                "data-action": "delete",
                class: "danger",
              }),
            ]
          : []),
      ]),
    );
  }

  if (joint) {
    out.push(
      h("h3", {}, [sel.kind === "joint" ? `Joint ${joint.name}` : `Parent joint ${joint.name}`]),
    );
    out.push(
      textRow(
        "Joint name",
        joint.name,
        (v) => apply({ type: "renameJoint", from: joint.name, to: v }),
        "joint.name",
      ),
    );
    out.push(
      selectRow(
        "Type",
        joint.type,
        [...JOINT_TYPES],
        (v) =>
          apply({ type: "setJoint", name: joint.name, patch: { type: /** @type {any} */ (v) } }),
        "joint.type",
      ),
    );
    const below = new Set(subtree(doc, joint.child));
    out.push(
      selectRow(
        "Parent link",
        joint.parent,
        doc.links.map((l) => l.name).filter((n) => !below.has(n)),
        (v) => apply({ type: "reparentLink", name: joint.child, parent: v }),
        "joint.parent",
      ),
    );
    out.push(
      vec3Row(
        "Origin xyz",
        joint.origin.xyz,
        (v) =>
          apply({
            type: "setOrigin",
            target: { kind: "joint", name: joint.name },
            origin: { xyz: v, rpy: joint.origin.rpy },
          }),
        "m",
        "joint.xyz",
      ),
    );
    out.push(
      vec3Row(
        "Origin rpy",
        joint.origin.rpy,
        (v) =>
          apply({
            type: "setOrigin",
            target: { kind: "joint", name: joint.name },
            origin: { xyz: joint.origin.xyz, rpy: v },
          }),
        "rad",
        "joint.rpy",
      ),
    );
    if (joint.type !== "fixed" && joint.type !== "floating")
      out.push(
        vec3Row(
          "Axis",
          joint.axis ?? [1, 0, 0],
          (v) => apply({ type: "setJoint", name: joint.name, patch: { axis: v } }),
          "",
          "joint.axis",
        ),
      );
    if (joint.type === "revolute" || joint.type === "prismatic" || joint.type === "continuous") {
      const lim = joint.limit ?? { lower: null, upper: null, effort: null, velocity: null };
      const unit = joint.type === "prismatic" ? "m" : "rad";
      /** @param {"lower" | "upper" | "effort" | "velocity"} k @param {string} label @param {string} u */
      const limRow = (k, label, u) =>
        row(
          label,
          [
            numberInput(
              lim[k],
              (x) =>
                apply({ type: "setJoint", name: joint.name, patch: { limit: { ...lim, [k]: x } } }),
              { "data-key": `joint.limit.${k}`, "aria-label": label },
            ),
          ],
          u,
        );
      if (joint.type !== "continuous") {
        out.push(limRow("lower", "Lower limit", unit));
        out.push(limRow("upper", "Upper limit", unit));
      }
      out.push(limRow("effort", "Effort limit", joint.type === "prismatic" ? "N" : "N m"));
      out.push(limRow("velocity", "Velocity limit", `${unit}/s`));
    }
  }

  if (link) {
    // Inertial
    out.push(h("h3", {}, ["Inertial"]));
    const inert = link.inertial;
    out.push(
      row(
        "Mass",
        [
          numberInput(
            inert?.mass ?? null,
            (x) => apply({ type: "setMass", link: link.name, mass: x }),
            { "data-key": "inertial.mass", "aria-label": "Mass" },
          ),
        ],
        "kg",
      ),
    );
    if (inert) {
      out.push(
        vec3Row(
          "Centre of mass",
          inert.origin.xyz,
          (v) =>
            apply({
              type: "setOrigin",
              target: { kind: "inertial", link: link.name },
              origin: { xyz: v, rpy: inert.origin.rpy },
            }),
          "m",
          "inertial.xyz",
        ),
      );
      const t = inert.inertia ?? { ixx: 0, ixy: 0, ixz: 0, iyy: 0, iyz: 0, izz: 0 };
      const keys = /** @type {const} */ (["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]);
      out.push(
        row(
          "Inertia",
          keys.map((k) =>
            numberInput(
              inert.inertia ? t[k] : null,
              (x) =>
                apply({
                  type: "setInertial",
                  link: link.name,
                  inertial: { ...inert, inertia: { ...t, [k]: x } },
                }),
              { "aria-label": k, title: k, "data-key": `inertial.${k}` },
            ),
          ),
          "kg m²",
        ),
      );
    }
    out.push(
      h("div", { class: "row actions" }, [
        button(
          "Compute inertia from geometry",
          () => {
            const s = link.collisions[0]
              ? { kind: /** @type {const} */ ("collision"), index: 0 }
              : { kind: /** @type {const} */ ("visual"), index: 0 };
            const bounds = hooks.meshBounds(link.name, s.kind, s.index);
            apply({ type: "computeInertia", link: link.name, ...(bounds ? { bounds } : {}) });
          },
          {
            "data-action": "compute-inertia",
            title:
              "Solid body of the first collision shape (else the first visual), urdf-inspector's formulas, rotated with the shape",
          },
        ),
        ...(inert
          ? [
              button("Remove inertial", () =>
                apply({ type: "setInertial", link: link.name, inertial: null }),
              ),
            ]
          : []),
      ]),
    );

    // Shapes
    for (const kind of /** @type {const} */ (["visual", "collision"])) {
      const list = kind === "visual" ? link.visuals : link.collisions;
      list.forEach((s, index) => out.push(...shapeEditor(link.name, kind, index, s, apply)));
    }
    const add = (/** @type {"visual" | "collision"} */ kind) =>
      apply({
        type: "addShape",
        link: link.name,
        kind,
        shape: {
          name: "",
          origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
          geometry: { type: "box", size: [0.05, 0.05, 0.05] },
          material: kind === "visual" ? { name: "", rgba: [0.85, 0.55, 0.2, 1] } : null,
        },
      });
    out.push(
      h("div", { class: "row actions" }, [
        button("Add visual", () => add("visual")),
        button("Add collision", () => add("collision")),
        ...(link.visuals.length > 0 && link.collisions.length === 0
          ? [button("Collision from visual", () => hooks.collisionFromVisual(link.name))]
          : []),
      ]),
    );
  }
  host.replaceChildren(...out);
}

/**
 * @param {string} link
 * @param {"visual" | "collision"} kind
 * @param {number} index
 * @param {ShapeDoc} s
 * @param {(op: EditOp) => void} apply
 * @returns {Node[]}
 */
function shapeEditor(link, kind, index, s, apply) {
  const g = s.geometry;
  const key = `${kind}.${index}`;
  /** @param {Geometry} geometry */
  const setG = (geometry) => apply({ type: "setGeometry", link, kind, index, geometry });
  const out = [h("h4", {}, [`${kind} ${index + 1}${s.name ? ` (${s.name})` : ""}`])];
  out.push(
    selectRow(
      "Shape",
      g.type,
      ["box", "cylinder", "sphere", "mesh"],
      (v) => setG(convertGeometry(/** @type {any} */ (v), g)),
      `${key}.type`,
    ),
  );
  if (g.type === "box")
    out.push(vec3Row("Size", g.size, (v) => setG({ type: "box", size: v }), "m", `${key}.size`));
  else if (g.type === "cylinder") {
    out.push(
      row(
        "Radius",
        [
          numberInput(g.radius, (x) => setG({ ...g, radius: x }), {
            "data-key": `${key}.radius`,
            "aria-label": "Radius",
          }),
        ],
        "m",
      ),
    );
    out.push(
      row(
        "Length",
        [
          numberInput(g.length, (x) => setG({ ...g, length: x }), {
            "data-key": `${key}.length`,
            "aria-label": "Length",
          }),
        ],
        "m",
      ),
    );
  } else if (g.type === "sphere")
    out.push(
      row(
        "Radius",
        [
          numberInput(g.radius, (x) => setG({ ...g, radius: x }), {
            "data-key": `${key}.radius`,
            "aria-label": "Radius",
          }),
        ],
        "m",
      ),
    );
  else {
    out.push(
      textRow("Mesh file", g.filename, (v) => setG({ ...g, filename: v }), `${key}.filename`),
    );
    out.push(vec3Row("Scale", g.scale, (v) => setG({ ...g, scale: v }), "", `${key}.scale`));
  }
  out.push(
    vec3Row(
      "Origin xyz",
      s.origin.xyz,
      (v) =>
        apply({
          type: "setOrigin",
          target: { kind, link, index },
          origin: { xyz: v, rpy: s.origin.rpy },
        }),
      "m",
      `${key}.xyz`,
    ),
  );
  out.push(
    vec3Row(
      "Origin rpy",
      s.origin.rpy,
      (v) =>
        apply({
          type: "setOrigin",
          target: { kind, link, index },
          origin: { xyz: s.origin.xyz, rpy: v },
        }),
      "rad",
      `${key}.rpy`,
    ),
  );
  /** @type {Node[]} */
  const tail = [];
  if (kind === "visual") {
    const color = /** @type {HTMLInputElement} */ (
      h("input", {
        type: "color",
        value: toHex(s.material?.rgba ?? null),
        "aria-label": "Colour",
        "data-key": `${key}.color`,
      })
    );
    color.addEventListener("change", () => {
      const v = color.value;
      const rgba = [1, 3, 5].map((k) => Number((parseInt(v.slice(k, k + 2), 16) / 255).toFixed(3)));
      apply({
        type: "setMaterial",
        link,
        index,
        material: { name: "", rgba: [...rgba, s.material?.rgba?.[3] ?? 1] },
      });
    });
    tail.push(color);
    if (s.material?.name && !s.material.rgba)
      tail.push(h("span", { class: "muted" }, [` material ${s.material.name}`]));
  }
  tail.push(
    button("Remove", () => apply({ type: "removeShape", link, kind, index }), { class: "danger" }),
  );
  out.push(h("div", { class: "row actions" }, tail));
  return out;
}
