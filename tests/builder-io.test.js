// Import and export of builder documents: URDF round trips on the bundled
// examples and the whole golden corpus, carried-through elements, the xacro
// export of edited documents, the ROS 2 package zip, the zip writer, the
// mecanum-kinematics TOML and kinematics, and the autosave format.

import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync, readdirSync } from "node:fs";
import { join as pjoin } from "node:path";
import { fileURLToPath } from "node:url";
import { EXAMPLES, exampleFs } from "../src/core/examples.js";
import { loadModel } from "../src/core/model.js";
import { VirtualFs, addZip } from "../src/core/vfs.js";
import { listZip, readZipEntry } from "../src/core/zip.js";
import { writeZip } from "../src/core/zipwrite.js";
import { compareXml } from "../src/core/canonical.js";
import { expandXacro, serializeXml } from "../src/core/xacro/index.js";
import { inspectText } from "../vendor/urdf-inspector/src/core/inspect.js";
import { parseXml } from "../vendor/urdf-inspector/src/core/xml.js";
import { readUrdf } from "../vendor/urdf-inspector/src/core/urdf.js";
import { importUrdf, toUrdf } from "../src/core/builder/urdf.js";
import { Editor, modelFromDoc } from "../src/core/builder/editor.js";
import { generate } from "../src/core/builder/templates.js";
import { buildPackage, exportXacro, packageZip } from "../src/core/builder/export.js";
import { deepEqual, guessSensor } from "../src/core/builder/document.js";
import {
  mecanumConfig,
  mecanumForward,
  mecanumGeometry,
  mecanumInverse,
  wheelBase,
} from "../src/core/builder/drive.js";
import { AUTOSAVE_KEY, decodeAutosave, encodeAutosave } from "../src/core/builder/storage.js";

const root = fileURLToPath(new URL("..", import.meta.url));
const read = {
  /** @param {string} url */
  text: async (url) => readFileSync(pjoin(root, url), "utf8"),
  /** @param {string} url */
  bytes: async (url) => new Uint8Array(readFileSync(pjoin(root, url))),
};

/**
 * What a URDF means, as urdf-inspector reads it: everything the builder
 * edits, with defaults filled in, without element paths and line numbers.
 * @param {string} text
 */
function meaning(text) {
  const { robot } = readUrdf(parseXml(text));
  assert.ok(robot);
  /** @param {import("../vendor/urdf-inspector/src/core/urdf.js").Shape} s */
  const shape = (s) => ({
    name: s.name,
    origin: s.origin,
    geometry: s.geometry,
    material: s.kind === "visual" ? s.material : null,
  });
  // JSON drops the sign of zero (the writer prints -0 as 0).
  return JSON.parse(
    JSON.stringify({
      name: robot.name,
      materials: [...robot.materials].map(([n, m]) => [n, m.rgba]),
      links: robot.links.map((l) => ({
        name: l.name,
        inertial: l.inertial
          ? { origin: l.inertial.origin, mass: l.inertial.mass, inertia: l.inertial.inertia }
          : null,
        visuals: l.visuals.map(shape),
        collisions: l.collisions.map(shape),
      })),
      joints: robot.joints
        .map((j) => ({
          name: j.name,
          type: j.type,
          parent: j.parent,
          child: j.child,
          origin: j.origin,
          axis: j.axis,
          limit: j.limit ? { ...j.limit, path: "" } : null,
          dynamics: j.dynamics,
          mimic: j.mimic,
        }))
        .sort((a, b) => (a.name < b.name ? -1 : 1)),
      other: [
        ...robot.gazebo.map((g) => g.reference),
        ...robot.transmissions.map((t) => t.name),
        ...robot.ros2Control.map((r) => r.name),
      ].sort(),
    }),
  );
}

/** Findings that do not depend on element order or line numbers. @param {string} text */
const findingKeys = (text) =>
  inspectText(text)
    .findings.map((f) => `${f.severity} ${f.id} ${f.target.kind}:${f.target.name} ${f.message}`)
    .sort();

/**
 * open -> import -> export URDF -> open: same model, same findings; and
 * export -> import -> export is stable.
 * @param {string} urdf
 * @param {string} label
 */
function assertRoundTrip(urdf, label) {
  const first = importUrdf(urdf);
  assert.ok(first.doc, `${label}: ${first.notes.join("; ")}`);
  assert.deepEqual(first.notes, [], label);
  const exported = toUrdf(first.doc);
  assert.deepEqual(
    meaning(exported),
    meaning(urdf),
    `${label}: exported URDF means something else`,
  );
  assert.deepEqual(findingKeys(exported), findingKeys(urdf), `${label}: findings changed`);
  const second = importUrdf(exported);
  assert.ok(second.doc);
  assert.ok(deepEqual(second.doc, first.doc), `${label}: re-imported document differs`);
  assert.equal(toUrdf(second.doc), exported, `${label}: second export differs`);
}

test("round trip on every bundled example (xacro expanded, then imported and exported)", async () => {
  for (const ex of EXAMPLES) {
    const fs = await exampleFs(ex, read);
    const entries = [ex.entry];
    if (ex.id === "turtlebot3")
      entries.push(ex.entry.replace("burger", "waffle"), ex.entry.replace("burger", "waffle_pi"));
    for (const entry of entries) {
      const m = loadModel(fs, entry);
      assert.equal(m.error, null, entry);
      assertRoundTrip(m.urdf, entry);
      // As an editor document: same findings as the opened model, meshes still resolve.
      const imp = importUrdf(m.urdf, { entry });
      const ed = new Editor(/** @type {any} */ (imp.doc), fs);
      assert.equal(ed.model.meshes.length, m.meshes.length, entry);
      assert.deepEqual(
        ed.model.meshes.map((x) => x.resolved),
        m.meshes.map((x) => x.resolved),
        entry,
      );
      assert.deepEqual(
        ed.model.findings.map((f) => f.id).sort(),
        m.findings
          .filter((f) => f.id !== "xacro-warning")
          .map((f) => f.id)
          .sort(),
        entry,
      );
    }
  }
});

test("round trip on every expanded URDF of the golden corpus (third-party, features, builder)", () => {
  const dir = pjoin(root, "tests/corpus/golden");
  let n = 0;
  for (const f of readdirSync(dir).filter((x) => x.endsWith(".urdf"))) {
    const text = readFileSync(pjoin(dir, f), "utf8");
    const { robot } = readUrdf(parseXml(text));
    if (!robot || robot.links.length === 0) continue; // feature fixtures without links
    const imp = importUrdf(text);
    if (imp.notes.length > 0) {
      // Only fixtures that exercise unusual input may lose something; say what.
      assert.ok(/^feature-|^unsupported-/.test(f), `${f}: ${imp.notes.join("; ")}`);
      continue;
    }
    assertRoundTrip(text, f);
    n++;
  }
  assert.ok(n >= 15, `only ${n} corpus models round-tripped`);
});

test("import keeps what it cannot edit and says what it dropped", () => {
  const urdf = `<robot name="r">
  <link name="base_link"><visual><geometry><capsule radius="1" length="2"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry><material name="x"/></collision></link>
  <link name="laser"/>
  <joint name="j" type="weird"><parent link="base_link"/><child link="laser"/></joint>
  <gazebo reference="laser"><sensor type="ray"><frame_name>laser</frame_name></sensor></gazebo>
  <transmission name="t"><joint name="j"/></transmission>
</robot>`;
  const { doc, notes } = importUrdf(urdf, { entry: "/x/r.urdf" });
  assert.ok(doc);
  assert.equal(doc.source?.entry, "/x/r.urdf");
  assert.equal(doc.links[0].visuals.length, 0);
  assert.equal(doc.links[0].collisions[0].material, null);
  assert.equal(doc.joints[0].type, "fixed");
  assert.ok(notes.some((x) => /capsule|no box/.test(x)));
  assert.ok(notes.some((x) => /type 'weird'/.test(x)));
  assert.equal(doc.links[1].sensor, "lidar2d");
  assert.deepEqual(
    doc.extras.map((x) => x.name),
    ["gazebo", "transmission"],
  );
  const out = toUrdf(doc);
  assert.match(out, /<gazebo reference="laser">/);
  assert.match(out, /<frame_name>laser<\/frame_name>/);
  // Renames reach the carried-through elements.
  const ed = new Editor(doc);
  ed.apply({ type: "renameLink", from: "laser", to: "lidar_link" });
  ed.apply({ type: "renameJoint", from: "j", to: "lidar_joint" });
  const renamed = toUrdf(ed.doc);
  assert.match(renamed, /<gazebo reference="lidar_link">/);
  assert.match(renamed, /<frame_name>lidar_link<\/frame_name>/);
  assert.match(renamed, /<transmission name="t">\s*<joint name="lidar_joint"\/>/);
  assert.equal(importUrdf("<robot").doc, null);
  assert.equal(importUrdf("<sdf/>").doc, null);
  assert.deepEqual(
    [
      "base_scan",
      "imu_link",
      "camera_rgb_frame",
      "camera_rgb_optical_frame",
      "realsense_link",
      "wheel",
    ].map(guessSensor),
    ["lidar2d", "imu", "camera", null, "depth_camera", null],
  );
});

test("xacro export: parametrised for template documents, plain after manual edits, both expand to the URDF", () => {
  const doc = generate("mecanum");
  const x = exportXacro(doc);
  assert.equal(x.parametrised, true);
  assert.match(x.text, /<xacro:property name="wheel_radius" value="0.04"\/>/);
  assert.match(x.text, /<xacro:macro name="mecanum_wheel"/);
  const ed = new Editor(doc);
  ed.apply({ type: "setMass", link: "base_link", mass: 4 });
  const plain = exportXacro(ed.doc);
  assert.equal(plain.parametrised, false);
  assert.match(plain.text, /edited after it was generated/);
  const own = exportXacro(importUrdf(toUrdf(ed.doc)).doc ?? doc);
  assert.match(own.text, /not made from a template/);
  for (const [text, d] of /** @type {const} */ ([
    [x.text, doc],
    [plain.text, ed.doc],
  ])) {
    const fs = new VirtualFs();
    fs.addText("/p/r.urdf.xacro", text);
    const r = expandXacro("/p/r.urdf.xacro", fs, {});
    const cmp = compareXml(toUrdf(d), serializeXml(r.root));
    assert.ok(cmp.equal && cmp.maxUlps === 0, JSON.stringify(cmp.firstDifference));
  }
});

test("ROS 2 package: files, a readable zip, meshes copied and re-pointed", async () => {
  const p = await buildPackage(generate("mecanum"));
  assert.equal(p.name, "mecanum_bot_description");
  assert.deepEqual(
    p.files.map((f) => f.path),
    [
      "mecanum_bot_description/CMakeLists.txt",
      "mecanum_bot_description/README.md",
      "mecanum_bot_description/config/mecanum_kinematics.toml",
      "mecanum_bot_description/launch/display.launch.py",
      "mecanum_bot_description/package.xml",
      "mecanum_bot_description/rviz/display.rviz",
      "mecanum_bot_description/urdf/mecanum_bot.urdf",
      "mecanum_bot_description/urdf/mecanum_bot.urdf.xacro",
    ],
  );
  const text = (/** @type {string} */ s) => String(p.files.find((f) => f.path.endsWith(s))?.data);
  assert.match(text("CMakeLists.txt"), /install\(DIRECTORY config launch rviz urdf\n/);
  assert.match(text("package.xml"), /<name>mecanum_bot_description<\/name>/);
  assert.match(text("package.xml"), /<exec_depend>robot_state_publisher<\/exec_depend>/);
  assert.match(text("display.launch.py"), /joint_state_publisher_gui/);
  assert.match(text("display.rviz"), /Fixed Frame: base_footprint/);
  // diff base: no TOML, no xacro after an edit
  const ed = new Editor(generate("diff"));
  ed.apply({ type: "renameLink", from: "caster_link", to: "caster" });
  const d = await buildPackage(ed.doc, {
    packageName: "my_bot",
    maintainer: "A <B>",
    email: "a@b.c",
    license: "Apache-2.0",
  });
  assert.ok(!d.files.some((f) => /\.toml$|\.xacro$/.test(f.path)));
  assert.ok(d.notes.some((x) => /no xacro/.test(x)));
  assert.match(String(d.files.find((f) => f.path.endsWith("package.xml"))?.data), /A &lt;B&gt;/);
  await assert.rejects(
    buildPackage(ed.doc, { packageName: "Bad-Name" }),
    /not a valid ROS package name/,
  );

  // An imported model with meshes: copied into meshes/, URDF re-pointed, and
  // the zip opens in Robot Studio with every mesh found.
  const ex = /** @type {import("../src/core/examples.js").Example} */ (
    EXAMPLES.find((e) => e.id === "mecanum")
  );
  const fs = await exampleFs(ex, read);
  const m = loadModel(fs, ex.entry);
  const doc = /** @type {import("../src/core/builder/document.js").RobotDoc} */ (
    importUrdf(m.urdf, { entry: ex.entry }).doc
  );
  const z = await packageZip(doc, { meshFs: fs });
  assert.deepEqual(z.notes, [
    "no xacro file: the robot is not exactly a template's output (see the xacro export)",
  ]);
  const names = listZip(z.zip).map((e) => e.name);
  assert.ok(names.includes("studio_mecanum_description/meshes/lidar.stl"), names.join(" "));
  assert.ok(names.includes("studio_mecanum_description/meshes/camera.obj"));
  const vfs = new VirtualFs();
  await addZip(vfs, z.zip);
  const reopened = loadModel(vfs, "/studio_mecanum_description/urdf/studio_mecanum.urdf");
  assert.equal(reopened.meshes.length, 2);
  assert.deepEqual(
    reopened.meshes.filter((x) => !x.resolved),
    [],
  );
  assert.match(reopened.urdf, /package:\/\/studio_mecanum_description\/meshes\/lidar.stl/);
  const stl = await vfs.readBinary("/studio_mecanum_description/meshes/lidar.stl");
  assert.deepEqual(stl, await fs.readBinary("/studio_mecanum_description/meshes/lidar.stl"));
  // Without the files, the mesh references are kept and reported.
  const noFiles = await buildPackage(doc);
  assert.equal(noFiles.notes.filter((x) => /was not copied/.test(x)).length, 2);
});

test("zip writer: entries read back byte for byte, deterministic output", async () => {
  const big = new Uint8Array(70000).map((_, i) => (i * 7) & 255);
  const files = [
    { path: "a/b.txt", data: "hello\n" },
    { path: "a/ü.bin", data: big },
    { path: "empty", data: "" },
  ];
  const z = writeZip(files);
  assert.deepEqual(writeZip(files), z);
  const entries = listZip(z);
  assert.deepEqual(
    entries.map((e) => [e.name, e.method, e.size]),
    [
      ["a/b.txt", 0, 6],
      ["a/ü.bin", 0, 70000],
      ["empty", 0, 0],
    ],
  );
  assert.equal(new TextDecoder().decode(await readZipEntry(z, entries[0])), "hello\n");
  assert.deepEqual(await readZipEntry(z, entries[1]), big);
});

test("mecanum TOML from urdf-inspector and kinematics from the measured wheels", () => {
  const model = modelFromDoc(
    generate("mecanum", { wheel_radius: 0.05, wheelbase: 0.3, track: 0.34 }),
  );
  const toml = mecanumConfig(model);
  assert.ok(toml.ok);
  assert.match(toml.toml, /wheel_radius = 0.05 /);
  assert.match(toml.toml, /half_length = 0.15 /);
  assert.match(toml.toml, /half_width = 0.17 /);
  assert.match(toml.toml, /max_wheel_speed = 20 /);
  const g = mecanumGeometry(wheelBase(model));
  assert.ok(g);
  assert.deepEqual(
    g.wheels.map((w) => [w.label, w.joint, w.sign]),
    [
      ["FL", "front_left_wheel_joint", 1],
      ["FR", "front_right_wheel_joint", 1],
      ["RL", "rear_left_wheel_joint", 1],
      ["RR", "rear_right_wheel_joint", 1],
    ],
  );
  // Hand-derived: r w = vx -/+ vy -/+ (lx + ly) wz for FL, FR, RL, RR.
  const k = 0.15 + 0.17;
  const t = { vx: 0.2, vy: -0.1, wz: 0.5 };
  const w = mecanumInverse(g, t);
  const want = [
    t.vx - t.vy - k * t.wz,
    t.vx + t.vy + k * t.wz,
    t.vx + t.vy - k * t.wz,
    t.vx - t.vy + k * t.wz,
  ].map((v) => v / 0.05);
  w.forEach((v, i) => assert.ok(Math.abs(v - want[i]) < 1e-12, `${i}: ${v} vs ${want[i]}`));
  const back = mecanumForward(g, w);
  assert.ok(
    Math.abs(back.vx - t.vx) < 1e-12 &&
      Math.abs(back.vy - t.vy) < 1e-12 &&
      Math.abs(back.wz - t.wz) < 1e-12,
  );
  // Pure forward motion: all wheels the same speed.
  assert.deepEqual(mecanumInverse(g, { vx: 0.5, vy: 0, wz: 0 }), [10, 10, 10, 10]);
  const diff = mecanumConfig(modelFromDoc(generate("diff")));
  assert.equal(diff.ok, false);
  assert.match(diff.ok ? "" : diff.reason, /differential base/);
  assert.equal(mecanumConfig(modelFromDoc(generate("arm"))).ok, false);
  assert.equal(mecanumGeometry(wheelBase(modelFromDoc(generate("diff")))), null);
  assert.throws(
    () =>
      mecanumForward(
        {
          radius: 1,
          wheels: [0, 1, 2, 3].map(() => ({ label: "", joint: "", x: 0, y: 0, sign: 1 })),
        },
        [1, 1, 1, 1],
      ),
    /singular/,
  );
  const broken = modelFromDoc({ ...generate("arm"), links: [] });
  assert.equal(mecanumConfig({ ...broken, report: null }).ok, false);
});

test("autosave format: round trip, and anything else is rejected", () => {
  const doc = generate("arm", { dof: 3 });
  const text = encodeAutosave(doc, new Date("2026-01-02T03:04:05Z"));
  const back = decodeAutosave(text);
  assert.ok(back);
  assert.equal(back.savedAt, "2026-01-02T03:04:05.000Z");
  assert.ok(deepEqual(back.doc, doc));
  assert.equal(AUTOSAVE_KEY, "robot-studio:builder:v1");
  for (const bad of [
    null,
    "",
    "{",
    "[]",
    '{"app":"x"}',
    JSON.stringify({
      app: "robot-studio",
      format: 1,
      savedAt: "t",
      doc: { format: 1, name: "r", links: [{}], joints: [], materials: [], extras: [] },
    }),
  ])
    assert.equal(decodeAutosave(bad), null, String(bad));
});
