// Command-line interface: expand xacro like the official tool, and check
// the result with the vendored urdf-inspector checks. Runs the same
// src/core code as the browser app, on the real file system.

import { parseArgs } from "node:util";
import { summaryLines } from "../vendor/urdf-inspector/src/core/report.js";
import { VERSION } from "./core/version.js";
import { loadModel } from "./core/model.js";
import { XacroError, expandXacro, serializeXml } from "./core/xacro/index.js";
import { NodeFs, abs } from "./node/nodefs.js";
import { TEMPLATES, TemplateError, findTemplate, generate } from "./core/builder/templates.js";
import { importUrdf, toUrdf } from "./core/builder/urdf.js";
import { exportXacro, packageZip } from "./core/builder/export.js";
import { mecanumConfig } from "./core/builder/drive.js";
import { modelFromDoc } from "./core/builder/editor.js";
import { simCommand } from "./simcli.js";

/**
 * @typedef {object} CliIo
 * @property {(text: string) => void} out
 * @property {(text: string) => void} err
 * @property {(path: string, data: string | Uint8Array) => void} writeFile
 * @property {(path: string) => boolean} exists
 * @property {(path: string) => string} [readText] (sim)
 * @property {(path: string) => void} [mkdir] (sim --out)
 * @property {() => number} [now] milliseconds (sim, for the step time it prints)
 */

export const EXIT = { ok: 0, warnings: 1, errors: 2, usage: 3 };

const HELP = `robot-studio ${VERSION}: open any robot model (URDF or xacro)

Usage:
  robot-studio xacro <in.xacro> [--arg name=value ...] [name:=value ...] [-o out.urdf]
  robot-studio check <robot.urdf|robot.xacro> [--arg name=value ...] [--json] [--strict]
  robot-studio new <template> [--set name=value ...] [--format F] [-o out]
  robot-studio export <robot.urdf|robot.xacro> [--arg name=value ...] [--format F] [-o out]
  robot-studio templates
  robot-studio sim <robot.urdf|robot.xacro|template> [--world W] [--route R]
                   [--seconds S] [--seed N] [--out DIR] [--imu-cal FILE] [--set k=v]
  robot-studio --help | --version

xacro     expand a xacro file the way the official xacro does and print the
          URDF (or write it with -o). Arguments are given as --arg name=value
          or, as with xacro, name:=value.
check     expand (if needed) and run urdf-inspector's checks plus the mesh
          references; findings are sorted by severity with file:line

new       generate a robot from a template (mecanum, diff, arm); parameters
          with --set name=value (numbers, true/false, x,y,z for positions)
export    open a model (expanding xacro) and export it as the builder does
templates list the templates and their parameters with defaults
sim       drive the robot in a simulated world (wheels, encoders, IMU, LiDAR,
          wheel odometry, scan matching, MCL) and write the recording;
          'robot-studio sim --help' for the options. Simulation output only.

  --format F           (new, export) urdf (default), xacro, toml (four-wheel
                       mecanum bases: mecanum-kinematics config) or package
                       (ROS 2 description package as a .zip; needs -o)

Options for xacro, check and export:
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
 * Run the CLI. Returns the exit code, or a promise of it for commands that
 * read files asynchronously (package export).
 * @param {string[]} argv
 * @param {CliIo} io
 * @returns {number | Promise<number>}
 */
export function main(argv, io) {
  /** @param {unknown} e */
  const fail = (e) => {
    if (
      e instanceof UsageError ||
      (e instanceof Error && "code" in e && String(e.code).startsWith("ERR_PARSE_ARGS"))
    ) {
      io.err(`robot-studio: ${e.message}\nRun 'robot-studio --help' for usage.\n`);
      return EXIT.usage;
    }
    if (e instanceof TemplateError || e instanceof RangeError) {
      io.err(`robot-studio: ${e.message}\n`);
      return EXIT.usage;
    }
    throw e;
  };
  if (argv[0] === "sim")
    return simCommand(argv.slice(1), {
      ...io,
      readText: io.readText ?? (() => ""),
      mkdir: io.mkdir ?? (() => {}),
      now: io.now ?? (() => 0),
    });
  try {
    const r = run(argv, io);
    return typeof r === "number" ? r : r.catch(fail);
  } catch (e) {
    return fail(e);
  }
}

/**
 * A --set value: number, boolean, x,y,z vector or text.
 * @param {string} v
 * @returns {unknown}
 */
function parseValue(v) {
  if (v === "true" || v === "false") return v === "true";
  if (v.includes(",")) return v.split(",").map((x) => (x.trim() === "" ? NaN : Number(x)));
  if (v.trim() !== "" && Number.isFinite(Number(v))) return Number(v);
  return v;
}

/**
 * Write one export of a builder document.
 * @param {import("./core/builder/document.js").RobotDoc} doc
 * @param {string} format
 * @param {string | undefined} output
 * @param {CliIo} io
 * @param {NodeFs | null} meshFs
 * @returns {number | Promise<number>}
 */
function writeExport(doc, format, output, io, meshFs) {
  /** @param {string | Uint8Array} data */
  const emit = (data) => {
    if (output) {
      io.writeFile(output, data);
      io.err(`wrote ${output}\n`);
    } else io.out(String(data));
    return EXIT.ok;
  };
  if (format === "urdf") return emit(toUrdf(doc));
  if (format === "xacro") {
    const x = exportXacro(doc);
    if (!x.parametrised) io.err("note: not parametrised (the robot is not a template's output)\n");
    return emit(x.text);
  }
  if (format === "toml") {
    const t = mecanumConfig(modelFromDoc(doc, meshFs));
    if (!t.ok) {
      io.err(`robot-studio: no mecanum-kinematics config: ${t.reason}\n`);
      return EXIT.errors;
    }
    return emit(t.toml);
  }
  if (format === "package") {
    if (!output) throw new UsageError("--format package writes a .zip; give -o file.zip");
    return packageZip(doc, { meshFs }).then((p) => {
      for (const n of p.notes) io.err(`note: ${n}\n`);
      io.err(`package ${p.name}\n`);
      return emit(p.zip);
    });
  }
  throw new UsageError(`unknown format '${format}' (urdf, xacro, toml or package)`);
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
      set: { type: "string", short: "s", multiple: true },
      format: { type: "string", short: "f", default: "urdf" },
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
  if (cmd === "templates") {
    for (const t of TEMPLATES) {
      io.out(`${t.id}: ${t.title}\n  ${t.description}\n`);
      for (const p of t.params) {
        const def = Array.isArray(p.def) ? p.def.join(",") : String(p.def);
        io.out(
          `  ${p.key.padEnd(20)} ${def.padEnd(16)} ${p.label}${p.unit ? ` (${p.unit})` : ""}\n`,
        );
      }
    }
    return EXIT.ok;
  }
  if (cmd === "new") {
    if (file === undefined)
      throw new UsageError(`new needs a template: ${TEMPLATES.map((t) => t.id).join(", ")}`);
    if (extra.length > 0) throw new UsageError(`unexpected argument '${extra[0]}'`);
    findTemplate(file);
    /** @type {Record<string, unknown>} */
    const params = {};
    for (const [k, v] of Object.entries(pairs(values.set, "--set"))) params[k] = parseValue(v);
    return writeExport(generate(file, params), String(values.format), values.output, io, null);
  }
  if (cmd !== "xacro" && cmd !== "check" && cmd !== "export")
    throw new UsageError(`unknown command '${cmd}'`);
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

  if (cmd === "export") {
    const model = loadModel(fs, entry, {
      args,
      env: /** @type {Record<string, string>} */ (process.env),
    });
    if (model.error) {
      io.err(`robot-studio: ${model.error.message}\n`);
      return EXIT.errors;
    }
    const imp = importUrdf(model.urdf, { entry });
    for (const n of imp.notes) io.err(`note: ${n}\n`);
    if (!imp.doc) return EXIT.errors;
    return writeExport(imp.doc, String(values.format), values.output, io, fs);
  }

  if (cmd === "xacro") {
    try {
      const env = /** @type {Record<string, string>} */ (process.env);
      const r = expandXacro(entry, fs, { args, cwd, env });
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

  const model = loadModel(fs, entry, {
    args,
    env: /** @type {Record<string, string>} */ (process.env),
  });
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
