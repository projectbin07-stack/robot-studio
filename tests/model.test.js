// Loading models: bundled examples, the virtual file system, package and
// mesh resolution, findings with source files, and the path helpers.

import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync, readdirSync, statSync } from "node:fs";
import { join as pjoin, relative, sep } from "node:path";
import { fileURLToPath } from "node:url";
import { EXAMPLES, exampleFs, findExample } from "../src/core/examples.js";
import { STUDIO_CHECKS, loadModel } from "../src/core/model.js";
import { VirtualFs, packageName } from "../src/core/vfs.js";
import {
  basename,
  dirname,
  extname,
  globToRegExp,
  isAbsolute,
  join,
  normalize,
} from "../src/core/paths.js";
import { compareXml, canonicalLines } from "../src/core/canonical.js";
import { doc, memFs } from "./helpers.js";

const root = fileURLToPath(new URL("..", import.meta.url));
const read = {
  /** @param {string} url */
  text: async (url) => readFileSync(pjoin(root, url), "utf8"),
  /** @param {string} url */
  bytes: async (url) => new Uint8Array(readFileSync(pjoin(root, url))),
};

test("every bundled example loads without errors and finds all its meshes", async () => {
  /** @type {Record<string, [number, number]>} links and meshes */
  const expected = { mecanum: [11, 2], turtlebot3: [7, 4], r2d2: [16, 8], rrbot: [5, 0] };
  for (const ex of EXAMPLES) {
    const fs = await exampleFs(ex, read);
    const m = loadModel(fs, ex.entry);
    assert.equal(m.error, null, `${ex.id}: ${m.error?.message}`);
    assert.equal(m.findings.filter((f) => f.severity === "error").length, 0, ex.id);
    assert.equal(m.report?.summary.links, expected[ex.id][0], ex.id);
    assert.equal(m.meshes.length, expected[ex.id][1], ex.id);
    assert.deepEqual(
      m.meshes.filter((x) => !x.resolved),
      [],
      ex.id,
    );
    for (const mesh of m.meshes) {
      const bytes = await fs.readBinary(/** @type {string} */ (mesh.resolved));
      assert.ok(bytes && bytes.length > 100, mesh.filename);
    }
  }
  assert.equal(findExample("nope"), null);
  assert.equal(findExample("mecanum")?.id, "mecanum");
});

test("the studio_mecanum example passes every check with notes only", async () => {
  const ex = /** @type {import("../src/core/examples.js").Example} */ (findExample("mecanum"));
  const m = loadModel(await exampleFs(ex, read), ex.entry);
  assert.deepEqual(
    m.findings.filter((f) => f.severity !== "info").map((f) => f.id),
    [],
  );
  assert.deepEqual(m.args, { use_lidar: "true", use_camera: "true", wheel_radius: "0.04" });
  const fl = m.findings.find((f) => f.id === "wheel-base-summary");
  assert.equal(fl?.file, "/studio_mecanum_description/urdf/mecanum_wheel.xacro");
  const bare = loadModel(await exampleFs(ex, read), ex.entry, {
    args: { use_lidar: "false", use_camera: "false" },
  });
  assert.equal(bare.report?.summary.links, 7);
});

test("every file under examples/ belongs to an example and every listed file exists", () => {
  /** @type {string[]} */
  const onDisk = [];
  /** @param {string} d */
  const walk = (d) => {
    for (const e of readdirSync(d)) {
      const p = pjoin(d, e);
      if (statSync(p).isDirectory()) walk(p);
      else onDisk.push(relative(root, p).split(sep).join("/"));
    }
  };
  walk(pjoin(root, "examples"));
  const listed = new Set(
    EXAMPLES.flatMap((e) => e.sources.flatMap((s) => s.files.map((f) => `${s.dir}/${f}`))),
  );
  for (const f of listed) assert.ok(onDisk.includes(f), `${f} is listed but missing`);
  for (const f of onDisk) assert.ok(listed.has(f), `${f} is on disk but not in an example`);
});

test("findings name the file their element came from; xacro problems become findings", () => {
  const fs = memFs({
    "/r/package.xml": "<package><name>r</name></package>",
    "/r/main.xacro": doc(`<xacro:include filename="parts.xacro"/><link name="a"/><xacro:part/>
  <joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint><xacro:property name="x" value="1" odd="1"/>`),
    "/r/parts.xacro": doc(
      `<xacro:macro name="part"><link name="b"><visual><geometry><mesh filename="package://r/m.stl"/></geometry></visual><visual><geometry><mesh filename="package://other/m.stl"/></geometry></visual></link></xacro:macro>`,
    ),
  });
  const m = loadModel(fs, "/r/main.xacro");
  const missing = m.findings.filter((f) => f.id === "mesh-missing");
  assert.equal(missing.length, 2);
  assert.equal(missing[0].file, "/r/parts.xacro");
  assert.match(missing[0].message, /\/r\/m\.stl is not among the loaded files/);
  assert.match(missing[1].message, /package 'other' is not among the loaded files/);
  const warn = m.findings.find((f) => f.id === "xacro-warning");
  assert.match(warn?.message ?? "", /unknown attribute\(s\): odd/);
  assert.equal(warn?.file, "/r/main.xacro");
  assert.ok(Object.keys(STUDIO_CHECKS).includes("mesh-missing"));
  assert.match(m.urdf, /^<\?xml version="1.0" \?>\n<robot name="t">/);
  assert.equal(m.xacro, true);
});

test("plain URDF is read without xacro; broken input is reported, not thrown", () => {
  const plain = loadModel(
    memFs({ "/a.urdf": '<robot name="p"><link name="only"/></robot>' }),
    "/a.urdf",
  );
  assert.equal(plain.xacro, false);
  assert.equal(plain.report?.summary.links, 1);
  const badXml = loadModel(memFs({ "/a.urdf": "<robot><link></robot>" }), "/a.urdf");
  assert.equal(badXml.findings[0].id, "xml-syntax");
  assert.equal(badXml.findings[0].line, 1);
  const badXacro = loadModel(memFs({ "/a.xacro": doc(`<x v="\${nope}"/>`) }), "/a.xacro");
  assert.equal(badXacro.findings[0].id, "xacro-error");
  assert.equal(badXacro.findings[0].line, 3);
  assert.match(badXacro.findings[0].message, /name 'nope' is not defined/);
  const none = loadModel(memFs({}), "/missing.urdf");
  assert.equal(none.findings[0].id, "xacro-error");
  const notRobot = loadModel(memFs({ "/a.urdf": "<sdf/>" }), "/a.urdf");
  assert.equal(notRobot.findings[0].id, "not-urdf");
  assert.equal(notRobot.meshes.length, 0);
  const fallback = memFs({
    "/d/package.xml": "<package><name>d</name></package>",
    "/d/description/urdf/r.xacro": doc(
      `<xacro:include filename="$(find d)/urdf/x.xacro"/><link name="a"/>`,
    ),
    "/d/description/urdf/x.xacro": doc(""),
  });
  const fm = loadModel(fallback, "/d/description/urdf/r.xacro");
  assert.equal(fm.findings.find((f) => f.id === "file-fallback")?.severity, "info");
  const noResolver = loadModel(
    {
      readText: () =>
        doc(`<link name="a"><visual><geometry><mesh filename="m.stl"/></geometry></visual></link>`),
      findPackage: () => null,
    },
    "/x.xacro",
  );
  assert.equal(noResolver.meshes[0].resolved, null);
  assert.equal(noResolver.findings.filter((f) => f.id === "mesh-missing").length, 0);
});

test("the virtual file system: packages, URIs, candidates", async () => {
  const fs = new VirtualFs();
  fs.addText("/ws/a/package.xml", "<package><!-- <name>no</name> --><name>alpha</name></package>");
  fs.addText("/ws/a/urdf/robot.urdf.xacro", '<robot name="r"/>');
  fs.addText("/ws/a/urdf/robot.gazebo.xacro", "<robot/>");
  fs.addText("/ws/a/urdf/plain.urdf", '<robot name="p"/>');
  fs.addLazy("/ws/a/meshes/m.stl", async () => new Uint8Array([1, 2, 3]), 3);
  fs.addText("/ws/a/meshes/rel.txt", "hello");
  fs.addText("/loose/thing.xacro", "<robot/>");
  assert.equal(fs.findPackage("alpha"), "/ws/a");
  assert.equal(fs.findPackage("loose"), "/loose");
  assert.equal(fs.findPackage("ws"), "/ws");
  assert.equal(fs.findPackage("none"), null);
  assert.equal(packageName("<package/>"), null);
  assert.deepEqual(fs.resolveUri("package://alpha/meshes/m.stl", "/ws/a/urdf/robot.urdf.xacro"), {
    path: "/ws/a/meshes/m.stl",
    reason: "",
  });
  assert.deepEqual(fs.resolveUri("../meshes/m.stl", "/ws/a/urdf/robot.urdf.xacro"), {
    path: "/ws/a/meshes/m.stl",
    reason: "",
  });
  assert.equal(fs.resolveUri("file:///ws/a/meshes/m.stl", "/x").path, "/ws/a/meshes/m.stl");
  assert.match(
    fs.resolveUri("file:///home/me/m.stl", "/x").reason,
    /absolute path on another computer/,
  );
  assert.match(
    fs.resolveUri("https://example.com/m.stl", "/x").reason,
    /https:\/\/ URIs are not fetched/,
  );
  assert.match(
    fs.resolveUri("package://alpha/nope.stl", "/x").reason,
    /not among the loaded files/,
  );
  assert.match(
    fs.resolveUri("nope.stl", "/ws/a/urdf/r.xacro").reason,
    /not among the loaded files/,
  );
  assert.deepEqual(await fs.readBinary("/ws/a/meshes/m.stl"), new Uint8Array([1, 2, 3]));
  assert.equal(
    new TextDecoder().decode(
      /** @type {Uint8Array} */ (await fs.readBinary("/ws/a/meshes/rel.txt")),
    ),
    "hello",
  );
  assert.equal(await fs.readBinary("/nope"), null);
  assert.equal(fs.readText("/nope"), null);
  assert.equal(fs.has("/ws/a/meshes/m.stl"), true);
  assert.equal(fs.candidates()[0], "/ws/a/urdf/robot.urdf.xacro");
  assert.equal(fs.candidates().at(-1), "/ws/a/urdf/robot.gazebo.xacro");
});

test("path helpers", () => {
  assert.equal(normalize("/a/./b/../c//d"), "/a/c/d");
  assert.equal(normalize("a/../../b"), "../b");
  assert.equal(normalize("/../a"), "/a");
  assert.equal(normalize("C:\\x\\y\\..\\z"), "C:/x/z");
  assert.equal(normalize(""), ".");
  assert.equal(join("/a/b", "../c"), "/a/c");
  assert.equal(join("/a", "/abs"), "/abs");
  assert.equal(join("D:/w", "x"), "D:/w/x");
  assert.equal(join("", ""), ".");
  assert.equal(dirname("/a/b.xacro"), "/a");
  assert.equal(dirname("/a"), "/");
  assert.equal(dirname("b"), ".");
  assert.equal(dirname("C:/a"), "C:/");
  assert.equal(basename("/a/b.urdf"), "b.urdf");
  assert.equal(extname("/a/b.URDF"), "urdf");
  assert.equal(extname("/a/LICENSE"), "");
  assert.equal(isAbsolute("C:/x"), true);
  assert.equal(isAbsolute("x/y"), false);
  const g = globToRegExp("/a/*.xacro");
  assert.ok(g.test("/a/b.xacro"));
  assert.ok(!g.test("/a/sub/b.xacro"));
  assert.ok(globToRegExp("/a/[!b]?.x").test("/a/cd.x"));
  assert.ok(globToRegExp("/a/[x.y").test("/a/[x.y"));
});

test("canonical XML comparison ignores comments, xmlns and attribute order only", () => {
  const a = `<?xml version="1.0"?><!-- c --><robot xmlns:xacro="x" name="r"><link b="2" a="1"> t </link></robot>`;
  const b = `<robot name="r"><link a="1" b="2">t</link></robot>`;
  assert.equal(compareXml(a, b).equal, true);
  assert.equal(compareXml(a, b).byteIdentical, false);
  assert.equal(compareXml(b, b).byteIdentical, true);
  const c = compareXml(b, `<robot name="r"><link a="1" b="3">t</link></robot>`);
  assert.equal(c.equal, false);
  assert.equal(c.firstDifference?.line, 2);
  assert.equal(compareXml(b, `<robot name="r"/>`).firstDifference?.actual, "(end)");
  assert.deepEqual(canonicalLines(b), ['<robot name="r">', '  <link a="1" b="2"> "t"']);
});

test("sourceOf maps element paths to the file that defined them", async () => {
  const ex = /** @type {import("../src/core/examples.js").Example} */ (findExample("mecanum"));
  const m = loadModel(await exampleFs(ex, read), ex.entry);
  assert.equal(
    m.sourceOf("/robot/link[@name='lidar_link']"),
    "/studio_mecanum_description/urdf/sensors.xacro",
  );
  assert.equal(m.sourceOf("/robot/link[@name='base_link']/visual[1]"), ex.entry);
  assert.equal(m.sourceOf("/robot/gazebo"), ex.entry);
  assert.equal(loadModel(memFs({}), "/x.urdf").sourceOf("/robot"), "/x.urdf");
});

test("the golden comparison tolerates last-bit float differences and reports them", async () => {
  const { ulps, ULP_TOLERANCE } = await import("../src/core/canonical.js");
  const a = `<robot name="r"><o xyz="-0.027712812921102028 0 1"/></robot>`;
  const b = `<robot name="r"><o xyz="-0.02771281292110203 0 1"/></robot>`;
  const c = compareXml(a, b);
  assert.equal(c.equal, true);
  assert.equal(c.exact, false);
  assert.equal(c.maxUlps, 1);
  assert.equal(compareXml(a, `<robot name="r"><o xyz="-0.0277128 0 1"/></robot>`).equal, false);
  assert.equal(
    compareXml(a, `<robot name="r"><o xyz="-0.027712812921102028 0 2"/></robot>`).equal,
    false,
  );
  assert.equal(
    compareXml(a, `<robot name="r"><o xyz="-0.027712812921102028 0 1 x"/></robot>`).equal,
    false,
  );
  assert.equal(ulps(1, 1), 0);
  assert.equal(ulps(1, -1), Infinity);
  assert.equal(ulps(NaN, 1), Infinity);
  assert.ok(ULP_TOLERANCE >= 1);
});
