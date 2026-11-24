// Virtual file system for a robot description loaded in the browser (a
// dropped folder, a chosen set of files or a zip) or bundled as an example.
// Text files are held in memory so xacro can include them synchronously;
// meshes are read on demand. ROS packages are found by their package.xml.
//
// Resolution rules (documented in docs/xacro.md):
// - $(find pkg) and package://pkg/... map to the folder holding the
//   package.xml whose <name> is pkg; with no package.xml, a top-level folder
//   named pkg is used.
// - When a file is not found inside a package but exactly one file exists
//   at the same relative path one folder deeper (ament_cmake packages that
//   install(DIRECTORY description/...) into share/), that file is used and
//   a note is recorded.
// - Relative mesh paths are resolved against the folder of the file that
//   was loaded.

import { basename, dirname, extname, join, normalize } from "./paths.js";
import { listZip, readZipEntry } from "./zip.js";

/**
 * @typedef {object} VfsEntry
 * @property {string | null} text contents for text files, null for binary or not yet read
 * @property {(() => Promise<Uint8Array>) | null} load reads the bytes on demand
 * @property {number} size bytes, -1 when unknown
 */

/** Extensions read as text up front. */
export const TEXT_EXTENSIONS = [
  "xacro",
  "urdf",
  "xml",
  "gazebo",
  "trans",
  "sdf",
  "yaml",
  "yml",
  "launch",
  "txt",
];

/** @param {string} path */
export const isTextFile = (path) => {
  const ext = extname(path);
  return TEXT_EXTENSIONS.includes(ext) || basename(path) === "package.xml";
};

/**
 * Package name from package.xml text (format 1, 2 or 3).
 * @param {string} xml
 * @returns {string | null}
 */
export function packageName(xml) {
  const m = /<name>\s*([^<\s]+)\s*<\/name>/.exec(xml.replace(/<!--[\s\S]*?-->/g, ""));
  return m ? m[1] : null;
}

/**
 * @typedef {object} VfsNote
 * @property {string} path
 * @property {string} message
 */

export class VirtualFs {
  constructor() {
    /** @type {Map<string, VfsEntry>} */
    this.entries = new Map();
    /** @type {Map<string, string>} explicit or discovered package -> folder */
    this.packageDirs = new Map();
    /** @type {VfsNote[]} */
    this.notes = [];
  }

  /**
   * @param {string} path
   * @param {string} text
   */
  addText(path, text) {
    this.entries.set(normalize(path), { text, load: null, size: text.length });
  }

  /**
   * @param {string} path
   * @param {() => Promise<Uint8Array>} load
   * @param {number} [size]
   */
  addLazy(path, load, size = -1) {
    this.entries.set(normalize(path), { text: null, load, size });
  }

  /** @param {string} name @param {string} dir */
  setPackage(name, dir) {
    this.packageDirs.set(name, normalize(dir));
  }

  /** @returns {string[]} */
  listFiles() {
    return [...this.entries.keys()].sort();
  }

  /** @param {string} path */
  has(path) {
    return this.entries.has(normalize(path));
  }

  /** Packages: explicit ones first, then every package.xml, then top-level folders. */
  packages() {
    /** @type {Map<string, string>} */
    const out = new Map(this.packageDirs);
    for (const [path, e] of this.entries) {
      if (basename(path) !== "package.xml" || e.text === null) continue;
      const name = packageName(e.text);
      if (name && !out.has(name)) out.set(name, dirname(path));
    }
    for (const path of this.entries.keys()) {
      const top = /^\/([^/]+)\//.exec(path)?.[1];
      if (top && !out.has(top)) out.set(top, `/${top}`);
    }
    return out;
  }

  /** @param {string} name */
  findPackage(name) {
    return this.packages().get(name) ?? null;
  }

  /**
   * Find a file, applying the install-layout fallback described above.
   * @param {string} path
   * @returns {string | null} the path that exists
   */
  locate(path) {
    const p = normalize(path);
    if (this.entries.has(p)) return p;
    for (const dir of this.packages().values()) {
      if (!p.startsWith(`${dir}/`)) continue;
      const rel = p.slice(dir.length + 1);
      const hits = this.listFiles().filter(
        (f) =>
          f.startsWith(`${dir}/`) &&
          f.endsWith(`/${rel}`) &&
          f.slice(dir.length + 1).split("/").length === rel.split("/").length + 1,
      );
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

  /** @param {string} path @returns {string | null} */
  readText(path) {
    const p = this.locate(path);
    if (p === null) return null;
    return /** @type {VfsEntry} */ (this.entries.get(p)).text;
  }

  /**
   * @param {string} path
   * @returns {Promise<Uint8Array | null>}
   */
  async readBinary(path) {
    const p = this.locate(path);
    if (p === null) return null;
    const e = /** @type {VfsEntry} */ (this.entries.get(p));
    if (e.load) return e.load();
    return new TextEncoder().encode(e.text ?? "");
  }

  /**
   * Resolve a mesh or texture reference from a URDF.
   * @param {string} uri package://pkg/path, file:///abs/path or a relative path
   * @param {string} baseFile the URDF/xacro file that was loaded
   * @returns {{ path: string | null, reason: string }}
   */
  resolveUri(uri, baseFile) {
    const pkg = /^package:\/\/([^/]+)\/(.*)$/.exec(uri);
    if (pkg) {
      const dir = this.findPackage(pkg[1]);
      if (dir === null)
        return { path: null, reason: `package '${pkg[1]}' is not among the loaded files` };
      const path = this.locate(join(dir, pkg[2]));
      return path
        ? { path, reason: "" }
        : { path: null, reason: `${join(dir, pkg[2])} is not among the loaded files` };
    }
    const file = /^file:\/\/(.*)$/.exec(uri);
    if (file) {
      const path = this.locate(file[1]);
      if (path) return { path, reason: "" };
      return { path: null, reason: `${file[1]} is an absolute path on another computer` };
    }
    if (/^[A-Za-z][A-Za-z0-9+.-]*:\/\//.test(uri))
      return { path: null, reason: `${uri.split(":")[0]}:// URIs are not fetched` };
    const path = this.locate(join(dirname(baseFile), uri));
    return path
      ? { path, reason: "" }
      : { path: null, reason: `${join(dirname(baseFile), uri)} is not among the loaded files` };
  }

  /**
   * The robot description files that could be opened, best first: files
   * named *.urdf.xacro, *.xacro or *.urdf, excluding files that are only
   * included by others is not possible without expanding them, so files in
   * a "urdf" folder whose root element is <robot name=...> rank first.
   * @returns {string[]}
   */
  candidates() {
    const score = (/** @type {string} */ p) => {
      const t = this.entries.get(p)?.text ?? "";
      let s = 0;
      if (/\.urdf\.xacro$/i.test(p)) s += 4;
      else if (/\.xacro$/i.test(p)) s += 2;
      else if (/\.urdf$/i.test(p)) s += 3;
      if (/<robot\b[^>]*\bname\s*=/.test(t)) s += 3;
      if (/\/urdf\//.test(p)) s += 1;
      if (/gazebo|materials|common|ros2_control|transmission|macro/i.test(basename(p))) s -= 3;
      return s;
    };
    return this.listFiles()
      .filter((p) => /\.(urdf|xacro)$/i.test(p))
      .sort((a, b) => score(b) - score(a) || (a < b ? -1 : 1));
  }
}

/**
 * Add every file of a zip archive under `mount` (text files are read now,
 * the others when first needed).
 * @param {VirtualFs} fs
 * @param {Uint8Array} bytes
 * @param {string} [mount]
 * @returns {Promise<number>} number of files added
 */
export async function addZip(fs, bytes, mount = "/") {
  const entries = listZip(bytes).filter(
    (e) => !e.directory && !/(^|\/)(__MACOSX|\.git)\//.test(e.name),
  );
  for (const e of entries) {
    const path = join(mount, e.name);
    if (isTextFile(path)) fs.addText(path, new TextDecoder().decode(await readZipEntry(bytes, e)));
    else fs.addLazy(path, () => readZipEntry(bytes, e), e.size);
  }
  return entries.length;
}
