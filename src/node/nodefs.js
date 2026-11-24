// The xacro file interface on the real file system, for the CLI and the
// tests. Packages for $(find) come from --package name=dir options and from
// package.xml files found in the input file's package and its siblings
// (the usual colcon workspace layout src/<pkg>/package.xml).

import { existsSync, readFileSync, readdirSync, statSync } from "node:fs";
import { resolve } from "node:path";
import { packageName } from "../core/vfs.js";
import { dirname, join, normalize, toSlashes } from "../core/paths.js";

/** @param {string} p */
const isFile = (p) => {
  try {
    return statSync(p).isFile();
  } catch {
    return false;
  }
};

/** @param {string} p */
const isDir = (p) => {
  try {
    return statSync(p).isDirectory();
  } catch {
    return false;
  }
};

/**
 * Package folders below `dir`, at most `depth` levels down.
 * @param {string} dir
 * @param {number} depth
 * @param {Map<string, string>} out
 */
export function scanPackages(dir, depth, out) {
  const xml = join(dir, "package.xml");
  if (isFile(xml)) {
    const name = packageName(readFileSync(xml, "utf8"));
    if (name && !out.has(name)) out.set(name, dir);
  }
  if (depth <= 0) return;
  let entries = [];
  try {
    entries = readdirSync(dir);
  } catch {
    return;
  }
  for (const e of entries) {
    if (e.startsWith(".") || e === "node_modules") continue;
    const p = join(dir, e);
    if (isDir(p)) scanPackages(p, depth - 1, out);
  }
}

export class NodeFs {
  /**
   * @param {Record<string, string>} [packages] explicit package name -> folder
   */
  constructor(packages = {}) {
    /** @type {Map<string, string>} */
    this.packageDirs = new Map(
      Object.entries(packages).map(([k, v]) => [k, normalize(toSlashes(resolve(v)))]),
    );
    /** @type {{ path: string, message: string }[]} */
    this.notes = [];
    /** @type {Set<string>} folders of the files read so far, searched by glob includes */
    this.roots = new Set();
  }

  /**
   * Discover packages around an input file: its own package (the nearest
   * folder above it with a package.xml) and the packages next to it.
   * @param {string} file
   */
  discoverAround(file) {
    let dir = dirname(abs(file));
    for (let k = 0; k < 6; k++) {
      if (isFile(join(dir, "package.xml"))) {
        scanPackages(dirname(dir), 3, this.packageDirs);
        return;
      }
      const up = dirname(dir);
      if (up === dir) break;
      dir = up;
    }
    scanPackages(dirname(abs(file)), 2, this.packageDirs);
  }

  /** @param {string} name */
  findPackage(name) {
    return this.packageDirs.get(name) ?? null;
  }

  /** @param {string} path @returns {string | null} */
  locate(path) {
    const p = normalize(path);
    if (isFile(p)) return p;
    for (const dir of this.packageDirs.values()) {
      if (!p.startsWith(`${dir}/`)) continue;
      const rel = p.slice(dir.length + 1);
      /** @type {string[]} */
      const hits = [];
      for (const sub of safeList(dir)) {
        const cand = join(dir, sub, rel);
        if (isDir(join(dir, sub)) && isFile(cand)) hits.push(cand);
      }
      if (hits.length === 1) {
        this.notes.push({
          path: p,
          message: `${p} does not exist; using ${hits[0]} (package installs that folder into its share directory)`,
        });
        return hits[0];
      }
    }
    return null;
  }

  /** @param {string} path */
  readText(path) {
    const p = this.locate(path);
    if (p === null) return null;
    this.roots.add(dirname(p));
    return readFileSync(p, "utf8");
  }

  /**
   * Bytes of a file (meshes copied into an exported package).
   * @param {string} path
   * @returns {Promise<Uint8Array | null>}
   */
  async readBinary(path) {
    const p = this.locate(path);
    return p === null ? null : new Uint8Array(readFileSync(p));
  }

  /**
   * Resolve a mesh reference (package://, file:// or relative to the loaded file).
   * @param {string} uri
   * @param {string} baseFile
   * @returns {{ path: string | null, reason: string }}
   */
  resolveUri(uri, baseFile) {
    const pkg = /^package:\/\/([^/]+)\/(.*)$/.exec(uri);
    /** @type {string} */
    let want;
    if (pkg) {
      const dir = this.findPackage(pkg[1]);
      if (dir === null)
        return {
          path: null,
          reason: `package '${pkg[1]}' not found (use --package ${pkg[1]}=DIR)`,
        };
      want = join(dir, pkg[2]);
    } else if (uri.startsWith("file://")) want = uri.slice(7);
    else if (/^[A-Za-z][A-Za-z0-9+.-]*:\/\//.test(uri))
      return { path: null, reason: `${uri.split(":")[0]}:// URIs are not fetched` };
    else want = join(dirname(baseFile), uri);
    const path = this.locate(want);
    return path ? { path, reason: "" } : { path: null, reason: `${want} does not exist` };
  }

  listFiles() {
    /** @type {string[]} */
    const out = [];
    /** @param {string} d @param {number} depth */
    const walk = (d, depth) => {
      for (const e of safeList(d)) {
        const p = join(d, e);
        if (isDir(p)) {
          if (depth > 0 && !e.startsWith(".")) walk(p, depth - 1);
        } else out.push(p);
      }
    };
    for (const dir of this.roots) walk(dir, 1);
    for (const dir of new Set(this.packageDirs.values())) walk(dir, 6);
    return [...new Set(out)];
  }
}

/** @param {string} d */
function safeList(d) {
  try {
    return existsSync(d) ? readdirSync(d) : [];
  } catch {
    return [];
  }
}

/** Absolute forward-slash path. @param {string} p */
export const abs = (p) => normalize(toSlashes(resolve(p)));
