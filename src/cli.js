// Command-line interface: expand xacro like the official tool, and check
// the result with the vendored urdf-inspector checks. Runs the same
// src/core code as the browser app, on the real file system.

import { parseArgs } from "node:util";
import { summaryLines } from "../vendor/urdf-inspector/src/core/report.js";
import { VERSION } from "./core/version.js";
import { loadModel } from "./core/model.js";
import { XacroError, expandXacro, serializeXml } from "./core/xacro/index.js";
import { NodeFs, abs } from "./node/nodefs.js";

/**
 * @typedef {object} CliIo
 * @property {(text: string) => void} out
 * @property {(text: string) => void} err
 * @property {(path: string, data: string) => void} writeFile
 * @property {(path: string) => boolean} exists
 */

export const EXIT = { ok: 0, warnings: 1, errors: 2, usage: 3 };

const HELP = `robot-studio ${VERSION}: open any robot model (URDF or xacro)

Usage:
  robot-studio xacro <in.xacro> [--arg name=value ...] [name:=value ...] [-o out.urdf]
  robot-studio check <robot.urdf|robot.xacro> [--arg name=value ...] [--json] [--strict]
  robot-studio --help | --version

xacro     expand a xacro file the way the official xacro does and print the
          URDF (or write it with -o). Arguments are given as --arg name=value
          or, as with xacro, name:=value.
check     expand (if needed) and run urdf-inspector's checks plus the mesh
          references; findings are sorted by severity with file:line

Options for both:
  --package name=dir   folder of a ROS package for $(find name) and
                       package://name/ (repeatable). Packages next to the
                       input file's package are found automatically from
                       their package.xml.
  --json               (check) machine-readable report
  --strict             (check) exit 1 when there are warnings and no errors

Exit codes: 0 no errors (and no warnings with --strict), 1 warnings with
--strict, 2 xacro failed or the model has errors, 3 usage or file error.
Nothing is uploaded anywhere and no network connection is made.
`;

class UsageError extends Error {}

/**
 * @param {string[]} argv
 * @param {CliIo} io
 * @returns {number}
 */
export function main(argv, io) {
  try {
    return run(argv, io);
  } catch (e) {
    if (
      e instanceof UsageError ||
      (e instanceof Error && "code" in e && String(e.code).startsWith("ERR_PARSE_ARGS"))
    ) {
      io.err(`robot-studio: ${e.message}\nRun 'robot-studio --help' for usage.\n`);
      return EXIT.usage;
    }
    throw e;
  }
}

/**
 * @param {string[] | undefined} specs
 * @param {string} what
 * @returns {Record<string, string>}
 */
function pairs(specs, what) {
  /** @type {Record<string, string>} */
  const out = {};
  for (const s of specs ?? []) {
    const m = /^([^=]+)=(.*)$/.exec(s);
    if (!m) throw new UsageError(`${what} expects name=value, got '${s}'`);
    out[m[1]] = m[2];
  }
  return out;
}

/**
 * @param {string[]} argv
 * @param {CliIo} io
 */
function run(argv, io) {
  // xacro-style name:=value arguments
  /** @type {Record<string, string>} */
  const remaps = {};
  const rest = [];
  for (const a of argv) {
    const m = /^([^-][^:=]*):=(.*)$/.exec(a);
    if (m) remaps[m[1]] = m[2];
    else rest.push(a);
  }
  const { values, positionals } = parseArgs({
    args: rest,
    allowPositionals: true,
    options: {
      help: { type: "boolean", short: "h" },
      version: { type: "boolean", short: "v" },
      json: { type: "boolean" },
      strict: { type: "boolean" },
      arg: { type: "string", short: "a", multiple: true },
      package: { type: "string", short: "p", multiple: true },
      output: { type: "string", short: "o" },
    },
  });
  if (values.help) {
    io.out(HELP);
    return EXIT.ok;
  }
  if (values.version) {
    io.out(`${VERSION}\n`);
    return EXIT.ok;
  }
  const [cmd, file, ...extra] = positionals;
  if (cmd === undefined) {
    io.out(HELP);
    return EXIT.usage;
  }
  if (cmd !== "xacro" && cmd !== "check") throw new UsageError(`unknown command '${cmd}'`);
  if (file === undefined) throw new UsageError(`${cmd} needs an input file`);
  if (extra.length > 0) throw new UsageError(`unexpected argument '${extra[0]}'`);
  if (!io.exists(file)) {
    io.err(`robot-studio: cannot read file: ${file}\n`);
    return EXIT.usage;
  }
  const args = { ...pairs(values.arg, "--arg"), ...remaps };
  const fs = new NodeFs(pairs(values.package, "--package"));
  const entry = abs(file);
  fs.discoverAround(entry);

  const cwd = abs(".");
  /** @param {string} f */
  const rel = (f) => (f.startsWith(`${cwd}/`) ? f.slice(cwd.length + 1) : f);

  if (cmd === "xacro") {
    try {
      const r = expandXacro(entry, fs, { args, cwd });
      for (const m of r.messages) io.err(`${m.level}: ${m.text} (${m.file}:${m.line})\n`);
      const xml = serializeXml(r.root);
      if (values.output) {
        io.writeFile(values.output, xml);
        io.err(`wrote ${values.output}\n`);
      } else io.out(xml);
      return EXIT.ok;
    } catch (e) {
      if (e instanceof XacroError) {
        io.err(`robot-studio: xacro failed: ${e.message}\n`);
        return EXIT.errors;
      }
      throw e;
    }
  }

  const model = loadModel(fs, entry, { args });
  for (const f of model.findings) f.file = rel(f.file);
  model.files = model.files.map(rel);
  const s = model.report?.summary;
  const errors = model.findings.filter((f) => f.severity === "error").length;
  const warnings = model.findings.filter((f) => f.severity === "warning").length;
  const infos = model.findings.length - errors - warnings;
  if (values.json) {
    io.out(
      JSON.stringify(
        {
          tool: "robot-studio",
          version: VERSION,
          file,
          xacro: model.xacro,
          files: model.files,
          args: model.args,
          summary: s
            ? { ...s, errors, warnings, infos, totalMass: Number(s.totalMass.toPrecision(6)) }
            : { errors, warnings, infos },
          findings: model.findings,
          meshes: model.meshes,
        },
        null,
        2,
      ) + "\n",
    );
  } else {
    const lines = [`${file}${model.xacro ? ` (xacro, ${model.files.length} file(s))` : ""}`];
    if (model.report) lines.push(...summaryLines(model.report).map((l) => `  ${l}`));
    if (model.meshes.length > 0)
      lines.push(
        `  meshes: ${model.meshes.filter((m) => m.resolved).length} of ${model.meshes.length} found`,
      );
    lines.push(`  ${errors} error(s), ${warnings} warning(s), ${infos} note(s)`, "");
    const tag = { error: "ERROR", warning: "WARN ", info: "INFO " };
    for (const f of model.findings) {
      lines.push(`${tag[f.severity]} ${f.id}  (${f.file}${f.line ? `:${f.line}` : ""})`);
      lines.push(`      ${f.message.replace(/\n/g, "\n      ")}`);
      if (f.path !== "/robot") lines.push(`      at ${f.path}`);
    }
    io.out(lines.join("\n") + "\n");
  }
  if (errors > 0) return EXIT.errors;
  if (values.strict && warnings > 0) return EXIT.warnings;
  return EXIT.ok;
}
