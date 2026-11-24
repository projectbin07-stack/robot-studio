// Helpers for scripts/vendor.mjs: what is vendored from where, hashing,
// a minimal tar reader for npm tarballs, and the offline manifest check
// used by the tests. Vendored files are never edited; this module only
// copies and compares bytes.

import { createHash } from "node:crypto";
import { existsSync, readdirSync, readFileSync, statSync } from "node:fs";
import { join, relative, sep } from "node:path";
import { gunzipSync } from "node:zlib";

/**
 * @typedef {object} GitSource
 * @property {"git"} kind
 * @property {string} name
 * @property {string} repo clone URL
 * @property {string} tag
 * @property {string} dest directory under the repository root
 * @property {string} license SPDX identifier of the upstream licence
 * @property {string[]} files paths inside the upstream repository; LICENSE is always added
 */
/**
 * @typedef {object} NpmSource
 * @property {"npm"} kind
 * @property {string} name
 * @property {string} package
 * @property {string} version
 * @property {string} dest
 * @property {string} license
 * @property {string[]} files paths inside the package tarball (without the "package/" prefix)
 */
/** @typedef {GitSource | NpmSource} Source */

/**
 * @typedef {object} ManifestEntry
 * @property {string} name
 * @property {"git" | "npm"} kind
 * @property {string} [repo]
 * @property {string} [tag]
 * @property {string} [commit] commit SHA the tag pointed to when vendored
 * @property {string} [package]
 * @property {string} [version]
 * @property {string} [integrity] npm registry integrity of the tarball
 * @property {string} license
 * @property {string} dest
 * @property {{ path: string, sha256: string }[]} files
 */
/** @typedef {{ comment: string, sources: ManifestEntry[] }} Manifest */

const URDF_CORE = [
  "xml.js",
  "urdf.js",
  "findings.js",
  "fk.js",
  "inertia.js",
  "math.js",
  "tree.js",
  "wheels.js",
  "inspect.js",
  "report.js",
  "version.js",
  "checks/context.js",
  "checks/frames.js",
  "checks/geometry.js",
  "checks/inertia.js",
  "checks/integration.js",
  "checks/joints.js",
  "checks/wheels.js",
].map((f) => `src/core/${f}`);

// lidar-slam-lab: the DOM-free simulation modules. sim.js (its own loop) is
// vendored only so the tests can run lidar-slam-lab's loop next to Robot
// Studio's estimator wrapper and compare them; cli.js and app.js are not used.
const LIDAR_SLAM = [
  "rng.js",
  "geometry.js",
  "world.js",
  "maps.js",
  "lidar.js",
  "motion.js",
  "grid.js",
  "distance.js",
  "scanmatch.js",
  "mcl.js",
  "metrics.js",
  "autopilot.js",
  "export.js",
  "sim.js",
].map((f) => `src/${f}`);

/** @type {Source[]} */
export const SOURCES = [
  {
    kind: "git",
    name: "urdf-inspector",
    repo: "https://github.com/projectbin07-stack/urdf-inspector.git",
    tag: "v0.1.1",
    dest: "vendor/urdf-inspector",
    license: "MIT",
    files: ["LICENSE", ...URDF_CORE],
  },
  {
    kind: "git",
    name: "lidar-slam-lab",
    repo: "https://github.com/projectbin07-stack/lidar-slam-lab.git",
    tag: "v0.1.0",
    dest: "vendor/lidar-slam-lab",
    license: "MIT",
    files: ["LICENSE", ...LIDAR_SLAM],
  },
  {
    kind: "npm",
    name: "three",
    package: "three",
    version: "0.187.0",
    dest: "vendor/three",
    license: "MIT",
    files: [
      "LICENSE",
      "build/three.module.js",
      "build/three.core.js",
      "examples/jsm/controls/OrbitControls.js",
      "examples/jsm/loaders/STLLoader.js",
      "examples/jsm/loaders/OBJLoader.js",
      "examples/jsm/loaders/ColladaLoader.js",
      "examples/jsm/loaders/TGALoader.js",
      "examples/jsm/loaders/collada/ColladaParser.js",
      "examples/jsm/loaders/collada/ColladaComposer.js",
    ],
  },
];

/** @param {Uint8Array | string} data */
export const sha256 = (data) => createHash("sha256").update(data).digest("hex");

/**
 * Read the regular files of a (gzipped) tar archive. Supports the ustar
 * format npm writes, including the 155-byte name prefix.
 * @param {Uint8Array} tgz
 * @returns {Map<string, Uint8Array>}
 */
export function readTarGz(tgz) {
  const tar = gunzipSync(tgz);
  /** @type {Map<string, Uint8Array>} */
  const out = new Map();
  const text = (/** @type {number} */ a, /** @type {number} */ n) => {
    const raw = tar.subarray(a, a + n);
    const z = raw.indexOf(0);
    return new TextDecoder().decode(z >= 0 ? raw.subarray(0, z) : raw);
  };
  let off = 0;
  while (off + 512 <= tar.length) {
    if (tar[off] === 0) break; // end-of-archive blocks
    const name = text(off, 100);
    const size = parseInt(text(off + 124, 12).trim() || "0", 8);
    const type = String.fromCharCode(tar[off + 156] || 48);
    const prefix =
      tar.subarray(off + 257, off + 262).toString() === "ustar" ? text(off + 345, 155) : "";
    const full = prefix ? `${prefix}/${name}` : name;
    if (type === "0" || type === "\0") out.set(full, tar.subarray(off + 512, off + 512 + size));
    off += 512 + Math.ceil(size / 512) * 512;
  }
  return out;
}

/**
 * All files below a directory, as forward-slash paths relative to it.
 * @param {string} dir
 * @returns {string[]}
 */
export function listFiles(dir) {
  /** @type {string[]} */
  const out = [];
  /** @param {string} d */
  const walk = (d) => {
    for (const e of readdirSync(d)) {
      const p = join(d, e);
      if (statSync(p).isDirectory()) walk(p);
      else out.push(relative(dir, p).split(sep).join("/"));
    }
  };
  if (existsSync(dir)) walk(dir);
  return out.sort();
}

/**
 * Offline check: every file listed in the manifest exists under `root`
 * with the recorded hash, and the vendor directories hold nothing else.
 * @param {Manifest} manifest
 * @param {string} root repository root
 * @returns {string[]} problems, empty when the vendored tree matches
 */
export function verifyAgainstManifest(manifest, root) {
  /** @type {string[]} */
  const problems = [];
  for (const src of manifest.sources) {
    const listed = new Set(src.files.map((f) => f.path));
    for (const f of src.files) {
      const p = join(root, src.dest, f.path);
      if (!existsSync(p)) {
        problems.push(`${src.dest}/${f.path}: missing`);
        continue;
      }
      const h = sha256(readFileSync(p));
      if (h !== f.sha256)
        problems.push(`${src.dest}/${f.path}: sha256 ${h} != manifest ${f.sha256}`);
    }
    for (const f of listFiles(join(root, src.dest)))
      if (!listed.has(f)) problems.push(`${src.dest}/${f}: not in the manifest`);
  }
  return problems;
}

/**
 * Compare upstream file contents with the manifest and the vendored copy.
 * @param {ManifestEntry} entry
 * @param {Map<string, Uint8Array>} upstream path -> bytes as published
 * @param {string} root
 * @returns {string[]}
 */
export function compareWithUpstream(entry, upstream, root) {
  /** @type {string[]} */
  const problems = [];
  for (const f of entry.files) {
    const up = upstream.get(f.path);
    if (!up) {
      problems.push(`${entry.name}: ${f.path} does not exist upstream`);
      continue;
    }
    const h = sha256(up);
    if (h !== f.sha256)
      problems.push(`${entry.name}: ${f.path} upstream sha256 ${h} != manifest ${f.sha256}`);
    const local = join(root, entry.dest, f.path);
    if (existsSync(local) && sha256(readFileSync(local)) !== h)
      problems.push(`${entry.name}: ${entry.dest}/${f.path} differs from upstream`);
  }
  return problems;
}
