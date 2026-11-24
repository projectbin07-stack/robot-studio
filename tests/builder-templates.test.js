// Templates: every generated robot is clean, parameters are validated, the
// exported xacro expands (with Robot Studio's xacro and, through the golden
// corpus, the official one) to the exported URDF, and the committed
// fixtures match what the builder writes today.

import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import {
  defaultParams,
  findTemplate,
  generate,
  isTemplateExact,
  TEMPLATES,
  TemplateError,
  templateXacro,
  validateParams,
} from "../src/core/builder/templates.js";
import { Editor, modelFromDoc } from "../src/core/builder/editor.js";
import { toUrdf } from "../src/core/builder/urdf.js";
import { compareXml } from "../src/core/canonical.js";
import { expandXacro, serializeXml } from "../src/core/xacro/index.js";
import { VirtualFs } from "../src/core/vfs.js";
import { BUILDER_CASES, builderCaseFiles } from "./builder-cases.js";
import { loadCases } from "./golden.js";

const ROOT = fileURLToPath(new URL("..", import.meta.url));

/** Parameter sets each template is checked with. */
const VARIANTS = {
  mecanum: [
    {},
    { lidar: false, imu: false, camera: false, depth_camera: false },
    { camera: true, roller_count: 0 },
    {
      roller_count: 16,
      wheel_radius: 0.1,
      wheelbase: 0.5,
      track: 0.6,
      base_width: 0.4,
      base_length: 0.7,
      base_mass: 20,
    },
  ],
  diff: [{}, { camera: true, lidar: false }, { wheel_x: 0, caster_x: 0.1, caster_radius: 0.03 }],
  arm: [
    {},
    { dof: 1 },
    { dof: 2, camera: true },
    { dof: 7, depth_camera: true, imu: true, lidar: true },
    { dof: 50 },
  ],
};

/**
 * @param {string} text
 * @param {Record<string, string>} [args]
 */
function expand(text, args = {}) {
  const fs = new VirtualFs();
  fs.addText("/t/robot.urdf.xacro", text);
  return serializeXml(expandXacro("/t/robot.urdf.xacro", fs, { args }).root);
}

test("every template variant has no errors or warnings and its inertias pass the checks", () => {
  for (const t of TEMPLATES)
    for (const params of VARIANTS[/** @type {keyof typeof VARIANTS} */ (t.id)]) {
      const doc = generate(t.id, params);
      const m = modelFromDoc(doc);
      const bad = m.findings.filter((f) => f.severity !== "info");
      assert.deepEqual(
        bad.map((f) => `${f.id}: ${f.message}`),
        [],
        `${t.id} ${JSON.stringify(params)}`,
      );
      assert.ok(!m.findings.some((f) => f.id.startsWith("inertia")));
      assert.equal(doc.template?.id, t.id);
      assert.ok(isTemplateExact(doc));
    }
});

test("templates have the expected structure", () => {
  const mec = modelFromDoc(generate("mecanum"));
  assert.equal(mec.report?.context?.wheels?.kind, "four-wheel");
  assert.deepEqual(
    mec.report?.context?.wheels?.wheels.map((w) => w.label),
    ["FL", "FR", "RL", "RR"],
  );
  assert.equal(mec.report?.summary.root, "base_footprint");
  const diff = modelFromDoc(generate("diff"));
  assert.equal(diff.report?.context?.wheels?.kind, "differential");
  assert.equal(diff.report?.context?.wheels?.ly, 0.12);
  const arm = modelFromDoc(generate("arm", { dof: 6, camera: true }));
  assert.equal(arm.report?.summary.jointTypes.revolute, 6);
  assert.equal(arm.report?.summary.root, "world");
  const doc = generate("arm", { dof: 6, camera: true });
  assert.equal(doc.joints.find((j) => j.name === "camera_joint")?.parent, "tool0");
  assert.deepEqual(
    doc.joints.filter((j) => j.type === "revolute").map((j) => j.axis?.join("")),
    ["001", "010", "010", "001", "010", "001"],
  );
  assert.deepEqual(
    doc.links.filter((l) => l.sensor).map((l) => `${l.name}:${l.sensor}`),
    ["camera_link:camera"],
  );
  // Sensor poses are the user's.
  const placed = generate("mecanum", { lidar_xyz: [0.1, 0.02, 0.2], lidar_rpy: [0, 0, 1.5] });
  assert.deepEqual(placed.joints.find((j) => j.name === "lidar_joint")?.origin, {
    xyz: [0.1, 0.02, 0.2],
    rpy: [0, 0, 1.5],
  });
  assert.equal(findTemplate("diff").title, "Differential base with caster");
  assert.throws(() => findTemplate("hexapod"), TemplateError);
});

test("parameters are validated with a message for every problem", () => {
  assert.equal(defaultParams("arm").dof, 6);
  assert.deepEqual(validateParams("mecanum", {}), defaultParams("mecanum"));
  /** @type {[string, Record<string, unknown>, RegExp][]} */
  const bad = [
    ["mecanum", { wheel_radius: -1 }, /wheel_radius must be a number in 0.005 .. 1 m/],
    ["mecanum", { wheel_radius: "big" }, /wheel_radius must be a number/],
    ["mecanum", { roller_count: 2.5 }, /whole number/],
    ["mecanum", { lidar: "yes" }, /lidar must be true or false/],
    ["mecanum", { name: "two words" }, /name must be a name/],
    ["mecanum", { lidar_xyz: [0, 0] }, /lidar_xyz must be three numbers/],
    ["mecanum", { track: 0.1 }, /wheels overlap the chassis/],
    ["mecanum", { colour: "red" }, /unknown parameter 'colour'/],
    ["diff", { caster_radius: 0.05 }, /caster radius must not exceed/],
    ["arm", { dof: 0 }, /dof must be a whole number in 1 .. 50/],
  ];
  for (const [id, p, re] of bad)
    assert.throws(
      () => generate(id, p),
      (e) => e instanceof TemplateError && re.test(e.message),
      `${id} ${JSON.stringify(p)}`,
    );
  assert.throws(
    () => generate("mecanum", { wheel_radius: -1, track: "x" }),
    /wheel_radius.*; track/,
  );
});

test("the exported xacro expands, with Robot Studio's xacro, to the exported URDF (all variants and sensor switches)", () => {
  for (const t of TEMPLATES)
    for (const params of VARIANTS[/** @type {keyof typeof VARIANTS} */ (t.id)]) {
      const doc = generate(t.id, params);
      const x = /** @type {string} */ (templateXacro(doc));
      const cmp = compareXml(toUrdf(doc), expand(x));
      assert.ok(
        cmp.equal,
        `${t.id} ${JSON.stringify(params)}: ${JSON.stringify(cmp.firstDifference)}`,
      );
      // Switching a sensor with its xacro argument equals regenerating without it.
      const p = /** @type {Record<string, unknown>} */ (doc.template?.params);
      for (const key of ["lidar", "camera"]) {
        const flipped = generate(t.id, { ...p, [key]: !p[key] });
        const c2 = compareXml(toUrdf(flipped), expand(x, { [`use_${key}`]: String(!p[key]) }));
        assert.ok(c2.equal, `${t.id} use_${key}: ${JSON.stringify(c2.firstDifference)}`);
      }
    }
});

test("template parameters stay editable: regenerating is undoable, manual edits drop the parametrised xacro", () => {
  const ed = new Editor(generate("mecanum"));
  const p = /** @type {Record<string, unknown>} */ (ed.doc.template?.params);
  ed.apply({
    type: "replaceDocument",
    doc: generate("mecanum", { ...p, wheel_radius: 0.05 }),
    label: "wheel_radius = 0.05",
  });
  assert.equal(ed.doc.template?.params.wheel_radius, 0.05);
  assert.ok(isTemplateExact(ed.doc));
  assert.equal(ed.undoLabel, "wheel_radius = 0.05");
  ed.undo();
  assert.equal(ed.doc.template?.params.wheel_radius, 0.04);
  ed.apply({ type: "setRobotName", name: "renamed_bot" });
  assert.equal(ed.doc.template?.params.name, "renamed_bot");
  assert.ok(isTemplateExact(ed.doc), "renaming the robot keeps the template parameters in step");
  ed.apply({ type: "setMass", link: "base_link", mass: 3.5 });
  assert.equal(isTemplateExact(ed.doc), false);
  assert.equal(templateXacro(ed.doc), null);
  assert.equal(isTemplateExact({ ...ed.doc, template: null }), false);
  assert.equal(isTemplateExact({ ...ed.doc, template: { id: "nope", params: {} } }), false);
});

test("committed builder fixtures are what the builder writes, and the official xacro's expansion of them equals the exported URDF", () => {
  const cases = loadCases();
  for (const c of BUILDER_CASES) {
    const files = builderCaseFiles(c);
    for (const [name, text] of Object.entries(files))
      assert.equal(
        readFileSync(`${ROOT}tests/corpus/builder/${name}`, "utf8"),
        text,
        `${name} is stale: run node scripts/builder-fixtures.js`,
      );
    const golden = cases.find(
      (g) => g.entry === `tests/corpus/builder/${c.id}.urdf.xacro` && !g.args,
    );
    assert.ok(golden, `no golden case for ${c.id}`);
    const official = readFileSync(`${ROOT}tests/corpus/golden/${golden.id}.urdf`, "utf8");
    const cmp = compareXml(files[`${c.id}.urdf`], official);
    assert.ok(cmp.equal, `${c.id}: ${JSON.stringify(cmp.firstDifference)}`);
  }
  // The argument cases match regenerated documents too.
  for (const g of cases.filter((x) => x.id.startsWith("builder-") && x.args)) {
    const c = /** @type {import("./builder-cases.js").BuilderCase} */ (
      BUILDER_CASES.find((b) => g.entry.endsWith(`/${b.id}.urdf.xacro`))
    );
    /** @type {Record<string, unknown>} */
    const params = { ...c.params };
    for (const [k, v] of Object.entries(g.args ?? {}))
      params[k.replace(/^use_/, "")] = v === "true";
    const official = readFileSync(`${ROOT}tests/corpus/golden/${g.id}.urdf`, "utf8");
    const cmp = compareXml(toUrdf(generate(c.template, params)), official);
    assert.ok(cmp.equal, `${g.id}: ${JSON.stringify(cmp.firstDifference)}`);
  }
});
