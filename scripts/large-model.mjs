#!/usr/bin/env node
// Generate the performance-budget model: a ROS description package with
// N links (default 200) and binary STL meshes of about M megabytes in total
// (default 50), as a stored .zip. The meshes are distorted spheres, each
// different (seeded), so nothing can be shared between links. The output is
// a test input, not committed (it is about 50 MB):
//
//   node scripts/large-model.mjs [--links 200] [--mb 50] [--seed 1] -o large_robot.zip
//
// Then open it in the page (the CI browser job uploads it and times it):
//   node scripts/browser-smoke.mjs --upload large_robot.zip --expect-links 200 \
//     --expect-summary "meshes 200/200 shown" --max-open-ms 10000

import { writeFileSync } from "node:fs";
import { parseArgs } from "node:util";
import { writeZip } from "../src/core/zipwrite.js";

const { values } = parseArgs({
  options: {
    links: { type: "string", default: "200" },
    mb: { type: "string", default: "50" },
    seed: { type: "string", default: "1" },
    out: { type: "string", short: "o", default: "large_robot.zip" },
  },
});
const links = Number(values.links);
const megabytes = Number(values.mb);
if (!Number.isInteger(links) || links < 2 || links > 5000) throw new Error("--links: 2 to 5000");
if (!(megabytes > 0 && megabytes <= 500)) throw new Error("--mb: (0, 500]");

/** Small deterministic generator (mulberry32). @param {number} a */
function rng(a) {
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const rand = rng(Number(values.seed));

// Triangles per mesh for the size target: a binary STL is 84 + 50 n bytes.
const perMesh = (megabytes * 1e6) / links;
const n = Math.max(12, Math.floor((perMesh - 84) / 50));
// A UV sphere with s stacks and s slices has 2 s (s - 1) triangles; at least n.
const s = Math.max(3, Math.ceil((1 + Math.sqrt(1 + 2 * n)) / 2));

/**
 * A distorted UV sphere as binary STL.
 * @param {number} radius m
 * @param {number} bumps relative radius variation
 * @returns {Uint8Array}
 */
function sphereStl(radius, bumps) {
  /** @type {number[][]} */
  const verts = [];
  const phase = rand() * 6.283;
  for (let i = 0; i <= s; i++) {
    const th = (Math.PI * i) / s;
    for (let j = 0; j < s; j++) {
      const ph = (2 * Math.PI * j) / s;
      const r = radius * (1 + bumps * Math.sin(3 * th + phase) * Math.cos(2 * ph));
      verts.push([
        r * Math.sin(th) * Math.cos(ph),
        r * Math.sin(th) * Math.sin(ph),
        r * Math.cos(th),
      ]);
    }
  }
  /** @type {number[][][]} */
  const tris = [];
  const at = (/** @type {number} */ i, /** @type {number} */ j) => verts[i * s + (j % s)];
  for (let i = 0; i < s; i++)
    for (let j = 0; j < s; j++) {
      if (i > 0) tris.push([at(i, j), at(i + 1, j), at(i, j + 1)]);
      if (i < s - 1) tris.push([at(i, j + 1), at(i + 1, j), at(i + 1, j + 1)]);
    }
  const buf = new Uint8Array(84 + 50 * tris.length);
  const dv = new DataView(buf.buffer);
  dv.setUint32(80, tris.length, true);
  let o = 84;
  for (const t of tris) {
    const [a, b, c] = t;
    const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    const nx = u[1] * v[2] - u[2] * v[1];
    const ny = u[2] * v[0] - u[0] * v[2];
    const nz = u[0] * v[1] - u[1] * v[0];
    const len = Math.hypot(nx, ny, nz) || 1;
    for (const x of [nx / len, ny / len, nz / len, ...a, ...b, ...c]) {
      dv.setFloat32(o, x, true);
      o += 4;
    }
    o += 2; // attribute byte count
  }
  return buf;
}

// Kinematic structure: a base with four arms of revolute joints (a tree,
// not one long chain), so the tree panel and the FK have depth and breadth.
const arms = 4;
const pkg = "large_robot";
/** @type {{ path: string, data: Uint8Array | string }[]} */
const files = [];
const urdf = [`<?xml version="1.0"?>`, `<robot name="${pkg}">`];
const inertial = (/** @type {number} */ m, /** @type {number} */ r) => {
  const i = (2 / 5) * m * r * r;
  return `<inertial><mass value="${m}"/><inertia ixx="${i.toPrecision(6)}" ixy="0" ixz="0" iyy="${i.toPrecision(6)}" iyz="0" izz="${i.toPrecision(6)}"/></inertial>`;
};
let meshBytes = 0;
for (let k = 0; k < links; k++) {
  const name = k === 0 ? "base_link" : `link_${String(k).padStart(3, "0")}`;
  const radius = k === 0 ? 0.12 : 0.03 + 0.02 * rand();
  const stl = sphereStl(radius, 0.15);
  meshBytes += stl.length;
  files.push({ path: `${pkg}/meshes/${name}.stl`, data: stl });
  urdf.push(
    `  <link name="${name}">`,
    `    <visual><geometry><mesh filename="package://${pkg}/meshes/${name}.stl"/></geometry></visual>`,
    `    <collision><geometry><sphere radius="${radius.toFixed(4)}"/></geometry></collision>`,
    `    ${inertial(k === 0 ? 5 : 0.2, radius)}`,
    `  </link>`,
  );
  if (k === 0) continue;
  const arm = (k - 1) % arms;
  const parentIndex = k <= arms ? 0 : k - arms;
  const parent = parentIndex === 0 ? "base_link" : `link_${String(parentIndex).padStart(3, "0")}`;
  const yaw = (2 * Math.PI * arm) / arms;
  const xyz = k <= arms ? [0.15 * Math.cos(yaw), 0.15 * Math.sin(yaw), 0] : [0.06, 0, 0];
  urdf.push(
    `  <joint name="joint_${String(k).padStart(3, "0")}" type="revolute">`,
    `    <parent link="${parent}"/><child link="${name}"/>`,
    `    <origin xyz="${xyz.map((v) => v.toFixed(4)).join(" ")}" rpy="0 0 ${k <= arms ? yaw.toFixed(4) : "0"}"/>`,
    `    <axis xyz="0 ${k % 2 ? 1 : 0} ${k % 2 ? 0 : 1}"/>`,
    `    <limit lower="-0.5" upper="0.5" effort="1" velocity="1"/>`,
    `  </joint>`,
  );
}
urdf.push("</robot>", "");
files.push({ path: `${pkg}/urdf/${pkg}.urdf`, data: urdf.join("\n") });
files.push({
  path: `${pkg}/package.xml`,
  data: `<?xml version="1.0"?>\n<package format="3">\n  <name>${pkg}</name>\n  <version>0.0.1</version>\n  <description>Generated performance-budget model (Robot Studio scripts/large-model.mjs)</description>\n  <maintainer email="none@example.com">none</maintainer>\n  <license>MIT</license>\n</package>\n`,
});
const zip = writeZip(files);
writeFileSync(/** @type {string} */ (values.out), zip);
console.log(
  `${values.out}: ${links} links, ${links} STL meshes of ${2 * s * (s - 1)} triangles, ` +
    `${(meshBytes / 1e6).toFixed(1)} MB of meshes, zip ${(zip.length / 1e6).toFixed(1)} MB`,
);
