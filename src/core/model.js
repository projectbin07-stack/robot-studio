// Load a robot model from a virtual file system: expand xacro when the
// file needs it, run the vendored urdf-inspector checks on the result,
// resolve mesh references, and attach the source file to every finding.
// DOM-free; used by the browser app and the CLI.

import { inspectXml } from "../../vendor/urdf-inspector/src/core/inspect.js";
import { sortFindings } from "../../vendor/urdf-inspector/src/core/findings.js";
import { XmlSyntaxError, parseDocument } from "./xacro/dom.js";
import { XacroError, expandXacro } from "./xacro/process.js";
import { looksLikeXacro } from "./xacro/index.js";
import { serializeXml, toXmlElement } from "./xacro/serialize.js";
import { normalize } from "./paths.js";

/** @typedef {import("../../vendor/urdf-inspector/src/core/findings.js").Finding} Finding */
/** @typedef {import("../../vendor/urdf-inspector/src/core/inspect.js").Report} Report */
/** @typedef {import("../../vendor/urdf-inspector/src/core/xml.js").XmlElement} XmlElement */
/** @typedef {import("./xacro/process.js").XacroFs} XacroFs */

/** Findings Robot Studio adds to urdf-inspector's catalogue. */
export const STUDIO_CHECKS = {
  "xacro-error": {
    group: "xacro",
    severity: "error",
    title: "xacro expansion failed",
    why: "The official xacro would stop at the same place (see docs/xacro.md); nothing after this point can be checked.",
  },
  "xacro-warning": {
    group: "xacro",
    severity: "warning",
    title: "xacro warning or message",
    why: "Deprecated syntax, unknown attributes or xacro.warning() output, reported as xacro reports it.",
  },
  "file-fallback": {
    group: "files",
    severity: "info",
    title: "File found one folder deeper",
    why: "ament_cmake packages often install a source folder (e.g. description/) into share/<pkg>; $(find) then points to the share layout.",
  },
};

/**
 * @typedef {Finding & { file: string }} StudioFinding
 */

/**
 * @typedef {object} MeshRef
 * @property {string} link
 * @property {"visual" | "collision"} kind
 * @property {number} index 0-based index among the link's visuals or collisions
 * @property {string} filename as written in the URDF
 * @property {string | null} resolved path in the file system, null when missing
 * @property {string} reason why it is missing
 */

/**
 * @typedef {object} LoadedModel
 * @property {string} entry
 * @property {boolean} xacro whether the file was expanded
 * @property {string} urdf expanded URDF text ("" when expansion failed)
 * @property {Report | null} report urdf-inspector's report (null when expansion failed)
 * @property {StudioFinding[]} findings sorted by severity
 * @property {MeshRef[]} meshes
 * @property {Record<string, string>} args every xacro:arg with its value
 * @property {string[]} files files read
 * @property {XacroError | null} error
 * @property {(path: string) => string} sourceOf file that defined the element at an element path
 */

/**
 * @typedef {object} ModelFs
 * @property {(path: string) => string | null} readText
 * @property {(name: string) => string | null} findPackage
 * @property {() => string[]} [listFiles]
 * @property {(uri: string, base: string) => { path: string | null, reason: string }} [resolveUri]
 * @property {{ path: string, message: string }[]} [notes]
 */

const NO_TARGET = /** @type {Finding["target"]} */ ({ kind: "element", name: "" });

/**
 * @param {string} id
 * @param {"error" | "warning" | "info"} severity
 * @param {string} message
 * @param {string} file
 * @param {number} line
 * @param {string} [path]
 * @param {Finding["target"]} [target]
 * @returns {StudioFinding}
 */
const studioFinding = (id, severity, message, file, line, path = "/robot", target = NO_TARGET) => ({
  id,
  severity,
  message,
  path,
  target,
  line,
  file,
});

/**
 * @param {ModelFs} fs
 * @param {string} entry
 * @param {{ args?: Record<string, string>, env?: Record<string, string> }} [opts]
 * @returns {LoadedModel}
 */
export function loadModel(fs, entry, opts = {}) {
  const path = normalize(entry);
  /** @type {LoadedModel} */
  const model = {
    entry: path,
    xacro: false,
    urdf: "",
    report: null,
    findings: [],
    meshes: [],
    args: {},
    files: [path],
    error: null,
    sourceOf: () => path,
  };
  const text = fs.readText(path);
  if (text === null) {
    model.error = new XacroError(`No such file or directory: ${path}`, path, 0);
    model.findings = [studioFinding("xacro-error", "error", model.error.detail, path, 0)];
    return model;
  }
  /** @type {import("./xacro/dom.js").XElement} */
  let root;
  /** @type {StudioFinding[]} */
  const extra = [];
  const notesBefore = fs.notes?.length ?? 0;
  try {
    if (looksLikeXacro(text)) {
      model.xacro = true;
      const r = expandXacro(path, fs, { args: opts.args ?? {}, env: opts.env ?? {} });
      root = r.root;
      model.args = r.args;
      model.files = r.files;
      for (const m of r.messages)
        extra.push(
          studioFinding(
            "xacro-warning",
            m.level === "warning" ? "warning" : "info",
            m.text,
            m.file,
            m.line,
          ),
        );
    } else root = parseDocument(text, path);
  } catch (e) {
    if (e instanceof XmlSyntaxError) {
      model.error = new XacroError(`XML is not well formed: ${e.detail}`, e.file, e.line);
      model.findings = [studioFinding("xml-syntax", "error", model.error.detail, e.file, e.line)];
      return model;
    }
    if (e instanceof XacroError) {
      model.error = e;
      model.findings = [
        studioFinding(
          "xacro-error",
          "error",
          e.message.replace(/^[^\n]*?:\d+: /, ""),
          e.file,
          e.line,
        ),
      ];
      return model;
    }
    throw e;
  }
  for (const n of (fs.notes ?? []).slice(notesBefore))
    extra.push(studioFinding("file-fallback", "info", n.message, n.path, 0));

  model.urdf = serializeXml(root);
  // Every element carries the file it came from (XmlElement.source), so
  // urdf-inspector puts the file on each finding; missing meshes are looked
  // up through the file system with its meshExists option (mesh-missing).
  const xml = toXmlElement(root);
  const report = inspectXml(
    xml,
    fs.resolveUri
      ? {
          meshExists: (f) => {
            const r = /** @type {NonNullable<ModelFs["resolveUri"]>} */ (fs.resolveUri).call(
              fs,
              f,
              path,
            );
            return r.path ? true : r.reason;
          },
        }
      : {},
  );
  model.report = report;
  const fileOf = elementFiles(xml, path);
  model.sourceOf = fileOf;
  /** @type {StudioFinding[]} */
  const findings = report.findings.map((f) => ({ ...f, file: f.source ?? path }));

  if (report.robot) {
    for (const link of report.robot.links) {
      for (const kind of /** @type {const} */ (["visual", "collision"])) {
        const shapes = kind === "visual" ? link.visuals : link.collisions;
        shapes.forEach((s, index) => {
          if (s.geometry?.type !== "mesh" || s.geometry.filename === "") return;
          const res = fs.resolveUri
            ? fs.resolveUri(s.geometry.filename, path)
            : { path: null, reason: "mesh files are not read here" };
          model.meshes.push({
            link: link.name,
            kind,
            index,
            filename: s.geometry.filename,
            resolved: res.path,
            reason: res.reason,
          });
        });
      }
    }
  }
  model.findings = /** @type {StudioFinding[]} */ (sortFindings([...findings, ...extra]));
  return model;
}

/**
 * Map an element path to the file its top-level element came from (for the
 * details panel; findings get theirs from urdf-inspector).
 * @param {XmlElement} root
 * @param {string} fallback
 * @returns {(path: string) => string}
 */
function elementFiles(root, fallback) {
  /** @type {Map<string, string>} */
  const byKey = new Map();
  for (const c of root.children)
    if (c.attrs.name !== undefined) {
      const key = `${c.name}|${c.attrs.name}`;
      if (!byKey.has(key)) byKey.set(key, c.source ?? fallback);
    }
  return (path) => {
    const m = /^\/robot\/([^[/]+)\[@(?:name|reference)='((?:[^']|&apos;)*)'\]/.exec(path);
    if (!m) return root.source ?? fallback;
    return byKey.get(`${m[1]}|${m[2].replace(/&apos;/g, "'")}`) ?? root.source ?? fallback;
  };
}
