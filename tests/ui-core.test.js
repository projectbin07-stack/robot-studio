// DOM-free parts of the browser app: URL state and what the 3D view draws.

import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { decodeState, defaultState, encodeState } from "../src/ui/state.js";
import {
  DEFAULT_RGBA,
  drawShapes,
  matrix4,
  posesFor,
  roughBounds,
  shapeColor,
  shapeSize,
  sliders,
} from "../src/core/scene.js";
import { exampleFs, findExample } from "../src/core/examples.js";
import { loadModel } from "../src/core/model.js";
import { memFs } from "./helpers.js";

const root = fileURLToPath(new URL("..", import.meta.url));
const read = {
  /** @param {string} u */
  text: async (u) => readFileSync(join(root, u), "utf8"),
  /** @param {string} u */
  bytes: async (u) => new Uint8Array(readFileSync(join(root, u))),
};

test("URL state round-trips and rejects junk", () => {
  assert.equal(encodeState(defaultState()), "");
  const st = defaultState();
  st.source = "rrbot";
  st.selected = { kind: "joint", name: "joint1" };
  st.geometry = "both";
  st.frames = true;
  st.axes = true;
  st.com = true;
  st.q = { joint2: -0.5, joint1: 0.25, zero: 0 };
  st.args = { prefix: "arm_" };
  st.theme = "dark";
  const qs = encodeState(st);
  assert.equal(
    qs,
    "s=rrbot&sel=j%3Ajoint1&g=both&fr=1&ja=1&com=1&q=joint1%3A0.25%2Cjoint2%3A-0.5&a=prefix%3Aarm_&theme=dark",
  );
  const back = decodeState(`?${qs}`);
  assert.deepEqual(back, { ...st, q: { joint1: 0.25, joint2: -0.5 } });
  const files = defaultState();
  files.source = "files";
  files.q = { a: 1 };
  files.args = { x: "1" };
  assert.equal(encodeState(files), "s=files", "local models keep no joint values or args in links");
  assert.equal(decodeState("s=files").source, "files");
  const junk = decodeState(
    "s=evil&sel=x:y&g=wire&q=a:NaN,:1,b:1e9,ok:2&a=bad name:1,x:<script>,y:fine&theme=pink&fr=2",
  );
  assert.deepEqual(junk, { ...defaultState(), q: { ok: 2 }, args: { y: "fine" } });
  assert.equal(decodeState("sel=l:base_link").selected?.kind, "link");
});

test("draw list, colours, poses and sliders for the mecanum example", async () => {
  const ex = /** @type {import("../src/core/examples.js").Example} */ (findExample("mecanum"));
  const m = loadModel(await exampleFs(ex, read), ex.entry);
  const robot = /** @type {import("../vendor/urdf-inspector/src/core/urdf.js").Robot} */ (
    m.report?.robot
  );
  const tree = /** @type {import("../vendor/urdf-inspector/src/core/tree.js").KinTree} */ (
    m.report?.context?.tree
  );
  const shapes = drawShapes(robot);
  assert.equal(shapes.filter((s) => s.kind === "visual").length, 2 + 4 * 10 + 1 + 1 + 1);
  assert.equal(shapes.filter((s) => s.kind === "collision").length, 1 + 4 + 3);
  const hub = shapes.find(
    (s) => s.link === "front_left_wheel_link" && s.index === 0 && s.kind === "visual",
  );
  assert.deepEqual(hub?.rgba, [0.92, 0.42, 0.12, 1]);
  assert.equal(shapes.find((s) => s.kind === "collision")?.rgba, null);
  const p0 = posesFor(tree, {});
  assert.deepEqual(p0.links.get("base_link")?.slice(12, 15), [0, 0, 0.04]);
  assert.ok(Math.abs(p0.mass - 3.054) < 1e-9);
  assert.equal(p0.joints.length, 4);
  const wheel = p0.joints.find((j) => j.name === "front_left_wheel_joint");
  assert.deepEqual(wheel?.axis, [0, 1, 0]);
  assert.deepEqual(
    wheel?.origin.slice(12, 15).map((v) => Number(v.toFixed(4))),
    [0.1, 0.1225, 0.04],
  );
  const turned = posesFor(tree, { front_left_wheel_joint: Math.PI / 2 }).links.get(
    "front_left_wheel_link",
  );
  // A quarter turn about y maps the wheel's x axis onto -z.
  assert.ok(Math.abs((turned?.[2] ?? 0) + 1) < 1e-12);
  const sl = sliders(tree);
  assert.deepEqual(
    sl.map((s) => [s.name, s.min, s.max, s.unit]),
    [
      ["front_left_wheel_joint", -Math.PI, Math.PI, "rad"],
      ["front_right_wheel_joint", -Math.PI, Math.PI, "rad"],
      ["rear_left_wheel_joint", -Math.PI, Math.PI, "rad"],
      ["rear_right_wheel_joint", -Math.PI, Math.PI, "rad"],
    ],
  );
  const b = roughBounds(p0.links, shapes);
  assert.ok(b.radius > 0.15 && b.radius < 0.4, String(b.radius));
});

test("limits, mimic joints, named materials and helpers", () => {
  const fs = memFs({
    "/a.urdf": `<robot name="arm"><material name="red"><color rgba="1 0 0 1"/></material>
      <link name="base"><visual><geometry><box size="0.1 0.2 0.3"/></geometry><material name="red"/></visual>
        <visual><geometry><sphere radius="0.1"/></geometry><material name="undefined_one"/></visual>
        <visual><geometry><mesh filename="m.stl"/></geometry></visual><visual><geometry><capsule radius="1" length="1"/></geometry></visual></link>
      <link name="a"/><link name="b"/><link name="c"/>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="a"/><axis xyz="0 0 1"/><limit lower="-0.1" upper="0.2" effort="1" velocity="1"/></joint>
      <joint name="copy" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="1" effort="1" velocity="1"/><mimic joint="slide" multiplier="2"/></joint>
      <joint name="noaxis" type="revolute"><parent link="b"/><child link="c"/><axis xyz="0 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
    </robot>`,
  });
  const m = loadModel(fs, "/a.urdf");
  const robot = /** @type {import("../vendor/urdf-inspector/src/core/urdf.js").Robot} */ (
    m.report?.robot
  );
  const tree = /** @type {import("../vendor/urdf-inspector/src/core/tree.js").KinTree} */ (
    m.report?.context?.tree
  );
  const shapes = drawShapes(robot);
  assert.equal(shapes.length, 3, "unknown geometry is skipped");
  assert.deepEqual(shapes[0].rgba, [1, 0, 0, 1]);
  assert.equal(shapes[1].rgba, null);
  assert.equal(shapeColor(robot.links[0].visuals[2], robot), null);
  assert.deepEqual(
    sliders(tree).map((s) => [s.name, s.min, s.max, s.mimic, s.unit]),
    [
      ["slide", -0.1, 0.2, null, "m"],
      ["copy", -1, 1, "slide", "rad"],
      ["noaxis", 0, 1, null, "rad"],
    ],
  );
  assert.equal(posesFor(tree, {}).joints.length, 2, "a joint without an axis has no marker");
  assert.equal(posesFor(tree, {}).com, null);
  assert.equal(shapeSize({ type: "box", size: [3, 4, 0] }), 2.5);
  assert.equal(shapeSize({ type: "cylinder", radius: 3, length: 8 }), 5);
  assert.equal(shapeSize({ type: "sphere", radius: 2 }), 2);
  assert.equal(shapeSize({ type: "mesh", filename: "x", scale: [1, 1, 1] }), 0.05);
  assert.deepEqual(roughBounds(new Map(), []), { center: [0, 0, 0], radius: 0.5 });
  assert.deepEqual(
    matrix4({ R: [1, 2, 3, 4, 5, 6, 7, 8, 9], p: [10, 11, 12] }),
    [1, 4, 7, 0, 2, 5, 8, 0, 3, 6, 9, 0, 10, 11, 12, 1],
  );
  assert.equal(DEFAULT_RGBA.length, 4);
});
