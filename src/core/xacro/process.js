// xacro processing: properties, args, macros, blocks, conditionals,
// includes, xacro:element / xacro:attribute and ${} / $() evaluation, in
// document order ("in-order" processing, the only mode of xacro 2.x).
//
// Written from the xacro documentation (https://github.com/ros/xacro/wiki,
// http://wiki.ros.org/xacro) and checked against the behaviour of the
// official implementation (xacro 2.1.1 on PyPI, xacro/__init__.py and
// substitution_args.py): the golden tests in tests/corpus compare this
// module's output with the official tool's output for every input.

import { dirname, globToRegExp, isAbsolute, join, normalize } from "../paths.js";
import { cloneNode, elementChildren, parseDocument, XmlSyntaxError } from "./dom.js";
import {
  NOT_FOUND,
  DotDict,
  PyModule,
  callValue,
  evalExpression,
  iterate,
  makeBuiltins,
} from "./pyeval.js";
import { PyDict, PyError, PyFunction, pyStr, truthy } from "./pyvalue.js";
import { NameSpace, Table } from "./table.js";

/** @typedef {import("./dom.js").XElement} XElement */
/** @typedef {import("./dom.js").XNode} XNode */
/** @typedef {import("./pyvalue.js").PyValue} PyValue */

/**
 * Files the processor can read. Paths use forward slashes.
 * @typedef {object} XacroFs
 * @property {(path: string) => string | null} readText null when the file does not exist
 * @property {(name: string) => string | null} findPackage directory of a package, null if unknown
 * @property {() => string[]} [listFiles] every file path, for glob includes
 */

/**
 * @typedef {object} XacroOptions
 * @property {Record<string, string>} [args] values for xacro:arg, like name:=value on the command line
 * @property {Record<string, string>} [env] environment for $(env) and $(optenv); empty by default
 * @property {string} [cwd] what $(cwd) returns
 */

/**
 * @typedef {object} XacroMessage
 * @property {"warning" | "message"} level
 * @property {string} text
 * @property {string} file
 * @property {number} line
 */

/**
 * @typedef {object} XacroResult
 * @property {XElement} root the expanded document element
 * @property {XacroMessage[]} messages warnings and xacro.message() output
 * @property {string[]} files every file read, entry first
 * @property {Record<string, string>} args final argument values (defaults included)
 */

/** @typedef {{ name: string, file: string, line: number }} Frame */

export class XacroError extends Error {
  /**
   * @param {string} message
   * @param {string} file
   * @param {number} line
   * @param {string[]} trace "when instantiating macro ..." / "included from ..." lines
   */
  constructor(message, file, line, trace = []) {
    super(`${file}${line ? `:${line}` : ""}: ${message}${trace.map((t) => `\n  ${t}`).join("")}`);
    this.detail = message;
    this.file = file;
    this.line = line;
    this.trace = trace;
  }
}

/**
 * @typedef {object} Macro
 * @property {XElement} body the xacro:macro element
 * @property {string} name
 * @property {string[]} params in order, block params with their * or ** prefix
 * @property {Map<string, [string | null, string | null]>} defaults param -> [forward name, default text]
 */

const DEFAULT_VALUE = String.raw`\$\{.*?\}|\$\(.*?\)|(?:'.*?'|".*?"|[^\s'"]+)+|`;
const RE_MACRO_ARG = new RegExp(
  String.raw`^\s*([^\s:=]+?)\s*:?=\s*(\^\|?)?(` + DEFAULT_VALUE + String.raw`)(?:\s+|$)(.*)`,
);

/**
 * Parse the first parameter of a macro's params attribute, as xacro's
 * parse_macro_arg: "name", "name:=default", "name:=^", "name:=^|default".
 * @param {string} s
 * @returns {[string, [string | null, string | null] | null, string]}
 */
export function parseMacroArg(s) {
  const m = RE_MACRO_ARG.exec(s);
  if (m) {
    const [, param, forward, dflt, rest] = m;
    return [param, [forward ? param : null, dflt ? dflt : null], rest];
  }
  const parts = s.trimStart().split(/\s+/);
  const first = parts[0];
  const rest = s.trimStart().slice(first.length).trimStart();
  return [first, null, rest];
}

const PY_KEYWORDS = new Set(
  "False None True and as assert async await break class continue def del elif else except finally for from global if import in is lambda nonlocal not or pass raise return try while with yield".split(
    " ",
  ),
);

/** @param {PyValue} name */
const isValidName = (name) =>
  typeof name === "string" && /^[A-Za-z_][A-Za-z0-9_]*$/.test(name) && !PY_KEYWORDS.has(name);

const LEXER = /** @type {const} */ ([
  ["DOLLAR_DOLLAR_BRACE", /^\$\$+(\{|\()/],
  ["EXPR", /^\$\{[^}]*\}/],
  ["EXTENSION", /^\$\([^)]*\)/],
  ["TEXT", /^(?:[^$]+|\$[^{($]+|\$$)/],
]);

/**
 * Python's convert_value(value, "auto") used by $(eval).
 * @param {string} value
 * @returns {PyValue}
 */
function convertAuto(value) {
  if (value.includes(".")) {
    if (/^\s*[+-]?(\d+\.?\d*(e[+-]?\d+)?|\.\d+(e[+-]?\d+)?)\s*$/i.test(value)) return Number(value);
  } else if (/^\s*[+-]?\d+\s*$/.test(value)) return BigInt(value.trim());
  const l = value.toLowerCase();
  if (l === "true" || l === "false") return l === "true";
  return value;
}

class Processor {
  /**
   * @param {XacroFs} fs
   * @param {XacroOptions} opts
   */
  constructor(fs, opts) {
    this.fs = fs;
    /** @type {Record<string, string>} */
    this.args = { ...(opts.args ?? {}) };
    this.env = opts.env ?? {};
    this.cwd = opts.cwd ?? "/";
    /** @type {XacroMessage[]} */
    this.messages = [];
    /** @type {string[]} */
    this.files = [];
    /** @type {string[]} */
    this.fileStack = [];
    /** @type {Frame[]} */
    this.macroStack = [];
    /** @type {XElement | null} element being processed, for error locations */
    this.at = null;
    const hooks = {
      /** @param {string} text @param {Table} table */
      evalText: (text, table) => this.evalText(text, table),
      /** @param {string} msg */
      warn: (msg) => this.warn(msg),
    };
    const { builtins } = makeBuiltins(hooks);
    this.addXacroModule(builtins);
    this.globals = builtins;
    this.hooks = hooks;
  }

  /** @param {Map<string, PyValue>} b */
  addXacroModule(b) {
    /** @type {Map<string, PyValue>} */
    const x = new Map();
    /** @param {"warning" | "message"} level */
    const say = (level) =>
      new PyFunction(level, (args) => {
        this.emit(level, args.map((a) => pyStr(a)).join(" "));
        return "";
      });
    x.set("message", say("message"));
    x.set("warning", say("warning"));
    x.set("error", say("warning"));
    x.set("print_location", new PyFunction("print_location", () => ""));
    x.set(
      "fatal",
      new PyFunction("fatal", (args) => {
        throw new PyError("XacroError", args.map((a) => pyStr(a)).join(" "));
      }),
    );
    x.set(
      "arg",
      new PyFunction("arg", (args) => {
        const name = pyStr(args[0] ?? null);
        if (!(name in this.args)) throw new PyError("KeyError", `'${name}'`);
        return this.args[name];
      }),
    );
    x.set(
      "tokenize",
      new PyFunction("tokenize", (args, kw) => {
        const s = pyStr(args[0] ?? "");
        const sep = pyStr(args[1] ?? kw.get("sep") ?? ",; ");
        const skip = truthy(args[2] ?? kw.get("skip_empty") ?? true);
        const cls = [...sep].map((c) => c.replace(/[\]\\^-]/g, "\\$&")).join("");
        const parts = s.split(new RegExp(`[${cls}]`));
        return skip ? parts.filter((p) => p !== "") : parts;
      }),
    );
    const absFilename = new PyFunction("abs_filename", (args) =>
      this.absPath(pyStr(args[0] ?? "")),
    );
    const dotify = new PyFunction("dotify", (args) => {
      const src = args[0];
      if (!(src instanceof PyDict)) throw new PyError("TypeError", "dotify() expects a dict");
      const d = new DotDict();
      for (const k of src.keys()) d.set(k, src.get(k) ?? null);
      return d;
    });
    const loadYaml = new PyFunction("load_yaml", () => {
      throw new PyError(
        "NotImplementedError",
        "xacro.load_yaml() is not supported by Robot Studio yet (no YAML reader); see docs/xacro.md",
      );
    });
    x.set("abs_filename", absFilename);
    x.set("dotify", dotify);
    x.set("load_yaml", loadYaml);
    b.set("xacro", new PyModule("xacro", x));
    /** @param {string} name @param {PyFunction} f */
    const deprecated = (name, f) =>
      new PyFunction(name, (args, kw) => {
        this.warn(`Using ${name}() directly is deprecated. Use xacro.${name}() instead.`);
        return callValue(f, args, kw);
      });
    b.set("load_yaml", deprecated("load_yaml", loadYaml));
    b.set("abs_filename", deprecated("abs_filename", absFilename));
    b.set("dotify", deprecated("dotify", dotify));
  }

  /** @param {string} text */
  warn(text) {
    this.emit("warning", text);
  }

  /** @param {"warning" | "message"} level @param {string} text */
  emit(level, text) {
    this.messages.push({
      level,
      text,
      file: this.at?.file ?? this.currentFile(),
      line: this.at?.line ?? 0,
    });
  }

  currentFile() {
    return this.fileStack[this.fileStack.length - 1] ?? "";
  }

  /** @param {string} spec */
  absPath(spec) {
    return normalize(isAbsolute(spec) ? spec : join(dirname(this.currentFile()), spec));
  }

  /**
   * An error at the element being processed, with the macro and include trace.
   * @param {string} message
   * @param {XElement | null} [el]
   */
  error(message, el = this.at) {
    /** @type {string[]} */
    const trace = [];
    for (const m of [...this.macroStack].reverse())
      trace.push(`when instantiating macro: ${m.name} (${m.file}:${m.line})`);
    const files = [...this.fileStack].reverse();
    for (const f of files.slice(1)) trace.push(`included from: ${f}`);
    return new XacroError(message, el?.file ?? this.currentFile(), el?.line ?? 0, trace);
  }

  /**
   * Evaluate ${...} and $(...) in a string. A string that is exactly one
   * expression keeps the expression's type; otherwise the parts are joined
   * as text.
   * @param {string} text
   * @param {Table} symbols
   * @returns {PyValue}
   */
  evalText(text, symbols) {
    /** @type {PyValue[]} */
    const results = [];
    let s = text;
    while (s.length > 0) {
      let matched = false;
      for (const [kind, re] of LEXER) {
        const m = re.exec(s);
        if (!m) continue;
        const tok = m[0];
        s = s.slice(tok.length);
        matched = true;
        if (kind === "EXPR") results.push(this.evalExpr(tok.slice(2, -1), symbols));
        else if (kind === "EXTENSION")
          results.push(this.evalExtension(`$(${pyStr(this.evalText(tok.slice(2, -1), symbols))})`));
        else if (kind === "TEXT") results.push(tok);
        else results.push(tok.slice(1));
        break;
      }
      if (!matched) throw this.error(`invalid expression: ${s}`);
    }
    if (results.length === 1) return results[0];
    return results.map((r) => pyStr(r)).join("");
  }

  /**
   * @param {string} src
   * @param {Table} symbols
   */
  evalExpr(src, symbols) {
    const inner = this.evalText(src, symbols);
    try {
      return evalExpression(pyStr(inner), symbols);
    } catch (e) {
      if (e instanceof PyError) {
        const msg = e.pyType === "XacroError" ? e.detail : e.message;
        throw this.error(`${msg}\nwhen evaluating expression '${src}'`);
      }
      throw e;
    }
  }

  /**
   * $(arg x), $(find pkg), $(env V), $(optenv V default), $(dirname),
   * $(cwd) and $(eval expr), as substitution_args.resolve_args.
   * @param {string} s the full "$(...)" text
   * @returns {string}
   */
  evalExtension(s) {
    if (s === "$(cwd)") return this.cwd;
    if (s.startsWith("$(eval ") && s.endsWith(")")) return this.evalSubstitution(s.slice(7, -1));
    let resolved = this.resolveCommands(s, ["env", "optenv", "dirname", "arg", "find"]);
    resolved = this.resolveCommands(resolved, ["find"]);
    return resolved;
  }

  /**
   * @param {string} argStr
   * @param {string[]} commands which commands to apply in this pass
   */
  resolveCommands(argStr, commands) {
    let resolved = argStr;
    for (const a of collectArgs(argStr, (m) => this.error(m))) {
      const parts = a.split(" ").filter((x) => x !== "");
      const [command, ...args] = parts;
      const valid = ["find", "env", "optenv", "dirname", "arg"];
      if (!valid.includes(command))
        throw this.error(
          `Unknown substitution command [${a}]. Valid commands are ['find', 'env', 'optenv', 'dirname', 'arg']`,
        );
      if (!commands.includes(command)) continue;
      const value = this.substitute(command, args, a);
      resolved = resolved.split(`$(${a})`).join(value);
    }
    return resolved;
  }

  /**
   * @param {string} command
   * @param {string[]} args
   * @param {string} whole
   * @returns {string}
   */
  substitute(command, args, whole) {
    switch (command) {
      case "arg": {
        if (args.length === 0)
          throw this.error(`$(arg var) must specify a variable name [${whole}]`);
        if (args.length > 1) throw this.error(`$(arg var) may only specify one arg [${whole}]`);
        if (!(args[0] in this.args)) throw this.error(`Undefined substitution argument ${args[0]}`);
        return this.args[args[0]];
      }
      case "find": {
        if (args.length !== 1)
          throw this.error(`$(find pkg) accepts exactly one argument [${whole}]`);
        const dir = this.fs.findPackage(args[0]);
        if (dir === null)
          throw this.error(
            `package '${args[0]}' not found (add its folder, with its package.xml, to the files you load)`,
          );
        return dir;
      }
      case "env": {
        if (args.length !== 1)
          throw this.error(`$(env var) command only accepts one argument [${whole}]`);
        const v = this.env[args[0]];
        if (v === undefined) throw this.error(`environment variable ${args[0]} is not set`);
        return v;
      }
      case "optenv": {
        if (args.length === 0)
          throw this.error(`$(optenv var) must specify an environment variable [${whole}]`);
        return this.env[args[0]] ?? args.slice(1).join(" ");
      }
      default:
        // dirname
        return dirname(this.currentFile());
    }
  }

  /** @param {string} expr */
  evalSubstitution(expr) {
    if (expr.includes("__"))
      throw this.error("$(eval ...) may not contain double underscore expressions");
    /** @type {Map<string, PyValue>} */
    const functions = new Map();
    functions.set(
      "arg",
      new PyFunction("arg", (args) => {
        const name = pyStr(args[0] ?? null);
        if (!(name in this.args))
          throw new PyError("XacroError", `Undefined substitution argument ${name}`);
        return convertAuto(this.args[name]);
      }),
    );
    functions.set("dirname", new PyFunction("dirname", () => dirname(this.currentFile())));
    for (const [k, v] of [
      ["true", true],
      ["false", false],
      ["True", true],
      ["False", false],
    ])
      functions.set(/** @type {string} */ (k), /** @type {boolean} */ (v));
    functions.set(
      "env",
      new PyFunction("env", (args) => {
        const v = this.env[pyStr(args[0] ?? null)];
        if (v === undefined)
          throw new PyError(
            "XacroError",
            `environment variable ${pyStr(args[0] ?? null)} is not set`,
          );
        return v;
      }),
    );
    functions.set(
      "optenv",
      new PyFunction("optenv", (args) => this.env[pyStr(args[0] ?? null)] ?? pyStr(args[1] ?? "")),
    );
    functions.set(
      "find",
      new PyFunction("find", (args) => {
        const dir = this.fs.findPackage(pyStr(args[0] ?? null));
        if (dir === null)
          throw new PyError("XacroError", `package '${pyStr(args[0] ?? null)}' not found`);
        return dir;
      }),
    );
    const math = /** @type {PyModule} */ (this.globals.get("math"));
    const python = /** @type {PyModule} */ (this.globals.get("python"));
    const scope = {
      /** @param {string} name */
      lookup: (name) => {
        const f = functions.get(name) ?? math.members.get(name);
        if (f !== undefined) return f;
        if (name in this.args) return convertAuto(this.args[name]);
        const b = python.members.get(name);
        return b === undefined ? NOT_FOUND : b;
      },
    };
    try {
      return pyStr(evalExpression(expr, scope));
    } catch (e) {
      if (e instanceof PyError)
        throw this.error(
          `${e.pyType === "XacroError" ? e.detail : e.message}\nwhen evaluating $(eval ${expr})`,
        );
      throw e;
    }
  }

  /**
   * Required and optional attributes; warns about unknown ones.
   * @param {XElement} el
   * @param {string[]} required
   * @param {string[]} optional
   * @returns {(string | null)[]}
   */
  checkAttrs(el, required, optional) {
    /** @type {(string | null)[]} */
    const out = [];
    for (const r of required) {
      const v = el.attrs[r];
      if (v === undefined) throw this.error(`${el.name}: missing attribute '${r}'`, el);
      out.push(v);
    }
    for (const o of optional) out.push(el.attrs[o] ?? null);
    const allowed = [...required, ...optional];
    const extra = Object.keys(el.attrs).filter(
      (a) => !allowed.includes(a) && !a.startsWith("xmlns:"),
    );
    if (extra.length > 0) this.warn(`${el.name}: unknown attribute(s): ${extra.join(", ")}`);
    return out;
  }

  /**
   * xacro's get_boolean_value.
   * @param {PyValue} value
   * @param {string | null} condition
   */
  boolean(value, condition) {
    if (typeof value === "string") {
      if (value === "true" || value === "True") return true;
      if (value === "false" || value === "False") return false;
      const t = value.trim().replace(/(?<=\d)_(?=\d)/g, "");
      if (/^[+-]?\d+$/.test(t)) return BigInt(t) !== 0n;
    } else return truthy(value);
    throw this.error(
      `Xacro conditional "${condition}" evaluated to "${pyStr(value)}", which is not a boolean expression.`,
    );
  }

  /**
   * Replace parent.children[i] with `nodes`; returns the index after them.
   * @param {XElement} parent
   * @param {number} i
   * @param {XNode[]} nodes
   */
  replace(parent, i, nodes) {
    parent.children.splice(i, 1, ...nodes);
    return i + nodes.length;
  }

  /**
   * Remove the comments directly before children[i] (xacro drops the
   * comments that document a directive). Returns how many were removed.
   * @param {XElement} parent
   * @param {number} i
   */
  removePreviousComments(parent, i) {
    let k = i - 1;
    let removed = 0;
    while (k >= 0) {
      const n = parent.children[k];
      if (
        n.type === "text" &&
        n.value.length > 0 &&
        /^\s+$/.test(n.value) &&
        n.value.split("\n").length <= 2
      )
        k--;
      const p = k >= 0 ? parent.children[k] : null;
      if (p && p.type === "comment") {
        parent.children.splice(k, 1);
        removed++;
        k--;
      } else break;
    }
    return removed;
  }

  /**
   * Copy xmlns:* declarations to `target`.
   * @param {XElement} target
   * @param {Record<string, string>} attrs
   */
  importNamespaces(target, attrs) {
    for (const [k, v] of Object.entries(attrs)) {
      if (!k.startsWith("xmlns:")) continue;
      const old = target.attrs[k];
      if (old !== undefined && old !== v)
        this.warn(`inconsistent namespace redefinitions for ${k}`);
      else target.attrs[k] = v;
    }
  }

  /**
   * Evaluate an element in place: attributes, then children in order.
   * @param {XElement} node
   * @param {Table} macros
   * @param {Table} symbols
   */
  evalAll(node, macros, symbols) {
    this.at = node;
    for (const name of Object.keys(node.attrs)) {
      if (name.startsWith("xacro:")) delete node.attrs[name];
      else node.attrs[name] = pyStr(this.evalText(node.attrs[name], symbols));
    }
    delete node.attrs["xmlns:xacro"];
    let evalComments = false;
    let i = 0;
    while (i < node.children.length) {
      const c = node.children[i];
      if (c.type === "element") {
        evalComments = false;
        this.at = c;
        i = this.processElement(node, i, c, macros, symbols);
        this.at = node;
      } else if (c.type === "text") {
        this.at = node;
        c.value = pyStr(this.evalText(c.value, symbols));
        if (c.value.trim()) evalComments = false;
        i++;
      } else if (c.type === "cdata") {
        i++;
      } else if (c.value.includes("xacro:eval-comments")) {
        evalComments = !c.value.includes("xacro:eval-comments:off");
        node.children.splice(i, 1);
      } else {
        if (evalComments) c.value = pyStr(this.evalText(c.value, symbols));
        i++;
      }
    }
  }

  /**
   * @param {XElement} parent
   * @param {number} i index of el in parent.children
   * @param {XElement} el
   * @param {Table} macros
   * @param {Table} symbols
   * @returns {number} index of the next node to process
   */
  processElement(parent, i, el, macros, symbols) {
    switch (el.name) {
      case "xacro:insert_block": {
        const [name] = /** @type {string[]} */ (this.checkAttrs(el, ["name"], []));
        let block;
        let contentOnly;
        if (symbols.has(`**${name}`)) {
          block = symbols.lookup(`**${name}`);
          contentOnly = true;
        } else if (symbols.has(`*${name}`)) {
          block = symbols.lookup(`*${name}`);
          contentOnly = false;
        } else throw this.error(`Undefined block "${name}"`);
        if (
          block === null ||
          typeof block !== "object" ||
          !("type" in block) ||
          block.type !== "element"
        )
          throw this.error(`"${name}" is not a block`);
        const copy = cloneNode(block);
        this.evalAll(copy, macros, symbols);
        return this.replace(parent, i, contentOnly ? copy.children : [copy]);
      }
      case "xacro:include":
        return this.processInclude(parent, i, el, macros, symbols);
      case "xacro:property": {
        i -= this.removePreviousComments(parent, i);
        parent.children.splice(i, 1);
        this.grabProperty(el, symbols);
        return i;
      }
      case "xacro:macro": {
        i -= this.removePreviousComments(parent, i);
        parent.children.splice(i, 1);
        this.grabMacro(el, macros);
        return i;
      }
      case "xacro:arg": {
        const [name, dflt] = /** @type {string[]} */ (this.checkAttrs(el, ["name", "default"], []));
        if (!(name in this.args)) this.args[name] = pyStr(this.evalText(dflt, symbols));
        i -= this.removePreviousComments(parent, i);
        parent.children.splice(i, 1);
        return i;
      }
      case "xacro:element": {
        const raw = el.attrs["xacro:name"];
        if (raw === undefined) throw this.error("xacro:element: missing attribute 'xacro:name'");
        const name = pyStr(this.evalText(raw, symbols));
        if (!name) throw this.error("xacro:element: empty name");
        delete el.attrs["xacro:name"];
        el.name = name;
        return i; // process again under its new name
      }
      case "xacro:attribute": {
        const raw = /** @type {string[]} */ (this.checkAttrs(el, ["name", "value"], []));
        const [name, value] = raw.map((a) => pyStr(this.evalText(a, symbols)));
        if (!name) throw this.error("xacro:attribute: empty name");
        parent.attrs[name] = value;
        parent.children.splice(i, 1);
        return i;
      }
      case "xacro:if":
      case "xacro:unless": {
        i -= this.removePreviousComments(parent, i);
        const [cond] = /** @type {string[]} */ (this.checkAttrs(el, ["value"], []));
        let keep = this.boolean(this.evalText(cond, symbols), cond);
        if (el.name === "xacro:unless") keep = !keep;
        if (!keep) {
          parent.children.splice(i, 1);
          return i;
        }
        this.evalAll(el, macros, symbols);
        return this.replace(parent, i, el.children);
      }
    }
    const next = this.macroCall(parent, i, el, macros, symbols);
    if (next !== null) return next;
    this.evalAll(el, macros, symbols);
    return i + 1;
  }

  /**
   * @param {XElement} el
   * @param {Table} table
   */
  grabProperty(el, table) {
    const [rawName, value0, dflt, remove, scope, lazy] = this.checkAttrs(
      el,
      ["name"],
      ["value", "default", "remove", "scope", "lazy_eval"],
    );
    const name = this.evalText(/** @type {string} */ (rawName), table);
    if (!isValidName(name))
      throw this.error(`Property names must be valid python identifiers: ${pyStr(name)}`);
    const key = /** @type {string} */ (name);
    if (key.startsWith("__"))
      throw this.error(`Property names must not start with double underscore:${key}`);
    const removeFlag = this.boolean(this.evalText(remove ?? "false", table), remove);
    if ([value0 !== null, dflt !== null, removeFlag].filter(Boolean).length > 1)
      throw this.error(
        `Property attributes default, value, and remove are mutually exclusive: ${key}`,
      );
    if (removeFlag && table.has(key)) {
      table.remove(key);
      return;
    }
    /** @type {PyValue} */
    let value = value0;
    if (dflt !== null) {
      if (scope !== null)
        this.warn(`${key}: default property value can only be defined on local scope`);
      if (table.has(key)) return;
      value = dflt;
    }
    let target = key;
    if (value === null) {
      target = `**${key}`;
      value = el;
    }
    let lazyEval = this.boolean(this.evalText(lazy ?? "true", table), lazy);
    let targetTable = table;
    if (scope === "global") {
      targetTable = table.top();
      lazyEval = false;
    } else if (scope === "parent") {
      if (!table.parent) throw this.error(`${key}: no parent scope at global scope`);
      let t = table.parent;
      lazyEval = false;
      if (!(table instanceof NameSpace)) while (t instanceof NameSpace && t.parent) t = t.parent;
      targetTable = t;
    }
    if (!lazyEval && typeof value === "string") value = this.evalText(value, table);
    targetTable.set(target, value, lazyEval);
  }

  /**
   * @param {XElement} el
   * @param {Table} macros
   */
  grabMacro(el, macros) {
    const [rawName, rawParams] = this.checkAttrs(el, ["name"], ["params"]);
    let name = /** @type {string} */ (rawName);
    if (name === "call") throw this.error("Invalid use of macro name 'call'");
    if (name.includes("."))
      throw this.error(`macro names must not contain '.' (reserved for namespaces): ${name}`);
    if (name.startsWith("xacro:")) {
      this.warn(`macro names must not contain prefix 'xacro:': ${name}`);
      name = name.slice(6);
    }
    /** @type {Macro} */
    const macro = { body: el, name, params: [], defaults: new Map() };
    let params = rawParams ?? "";
    while (params.trim() !== "") {
      const [param, value, rest] = parseMacroArg(params);
      macro.params.push(param);
      if (value !== null) macro.defaults.set(param, value);
      params = rest;
    }
    macros.set(name, /** @type {PyValue} */ (/** @type {unknown} */ (macro)), false);
  }

  /**
   * @param {string} fullname
   * @param {Table} macros
   * @param {Table} symbols
   * @returns {[Table, Table, Macro] | null}
   */
  resolveMacro(fullname, macros, symbols) {
    const direct = macros.lookup(fullname);
    if (direct !== NOT_FOUND)
      return [macros, symbols, /** @type {Macro} */ (/** @type {unknown} */ (direct))];
    const parts = fullname.split(".");
    const name = /** @type {string} */ (parts.pop());
    if (parts.length === 0) return null;
    let m = macros;
    let s = symbols;
    for (const ns of parts) {
      const nm = m.lookup(ns);
      const nsym = s.lookup(ns);
      if (!(nm instanceof Table) || !(nsym instanceof Table)) return null;
      m = nm;
      s = nsym;
    }
    const found = m.lookup(name);
    return found === NOT_FOUND
      ? null
      : [m, s, /** @type {Macro} */ (/** @type {unknown} */ (found))];
  }

  /**
   * Expand a macro call in place; null when el is not a macro call.
   * @param {XElement} parent
   * @param {number} i
   * @param {XElement} node
   * @param {Table} macros
   * @param {Table} symbols
   * @returns {number | null}
   */
  macroCall(parent, i, node, macros, symbols) {
    if (node.name === "xacro:call") {
      const raw = node.attrs.macro;
      if (raw === undefined) throw this.error("xacro:call: missing attribute 'macro'");
      if (!raw) throw this.error("xacro:call is missing the 'macro' attribute");
      const name = pyStr(this.evalText(raw, symbols));
      delete node.attrs.macro;
      node.name = `xacro:${name}`;
    } else if (!node.name.startsWith("xacro:")) return null;

    const fullname = node.name.slice(6);
    const resolved = this.resolveMacro(fullname, macros, symbols);
    if (!resolved) throw this.error(`unknown macro name: ${node.name}`);
    const [scopedMacros, scopedSymbols, m] = resolved;
    const body = cloneNode(m.body);
    this.macroStack.push({ name: fullname, file: node.file, line: node.line });

    const ss = new Table(scopedSymbols, this.globals, this.hooks);
    const sm = new Table(scopedMacros, scopedMacros.globals, this.hooks);
    const params = [...m.params];
    for (const [name, value] of Object.entries(node.attrs)) {
      const k = params.indexOf(name);
      if (k < 0) throw this.error(`Invalid parameter "${name}" of macro ${fullname}`, node);
      params.splice(k, 1);
      ss.set(name, this.evalText(value, symbols), false);
      node.attrs[name] = "";
    }
    this.evalAll(node, macros, symbols);
    this.at = node;

    const blocks = elementChildren(node);
    let b = 0;
    for (const param of [...params]) {
      if (param[0] !== "*") continue;
      if (b >= blocks.length) throw this.error(`Not enough blocks for macro ${fullname}`, node);
      params.splice(params.indexOf(param), 1);
      ss.set(param, blocks[b++], true);
    }
    if (b < blocks.length)
      throw this.error(`Unused block "${blocks[b].name}" in call of macro ${fullname}`, node);

    for (const param of [...params]) {
      if (param[0] === "*") continue;
      const d = m.defaults.get(param);
      if (!d) continue;
      const [forward, dflt] = d;
      ss.set(param, this.defaultArg(forward, dflt, symbols), false);
      params.splice(params.indexOf(param), 1);
    }
    if (params.length > 0)
      throw this.error(
        `Undefined parameters [${params.join(",")}] in call of macro ${fullname}`,
        node,
      );

    this.evalAll(body, sm, ss);
    this.at = node;
    i -= this.removePreviousComments(parent, i);
    this.importNamespaces(parent, body.attrs);
    const next = this.replace(parent, i, body.children);
    this.macroStack.pop();
    return next;
  }

  /**
   * @param {string | null} forward
   * @param {string | null} dflt
   * @param {Table} symbols the caller's scope
   */
  defaultArg(forward, dflt, symbols) {
    if (forward === null) return this.evalText(/** @type {string} */ (dflt), symbols);
    const v = symbols.lookup(forward);
    if (v !== NOT_FOUND) return v;
    if (dflt !== null) return this.evalText(dflt, symbols);
    throw this.error(`Undefined property to forward: ${forward}`);
  }

  /**
   * @param {XElement} parent
   * @param {number} i
   * @param {XElement} el
   * @param {Table} macros
   * @param {Table} symbols
   */
  processInclude(parent, i, el, macros, symbols) {
    const [spec, nsSpec, optional] = this.checkAttrs(el, ["filename"], ["ns", "optional"]);
    let nsMacros = macros;
    let nsSymbols = symbols;
    if (nsSpec) {
      const ns = pyStr(this.evalText(nsSpec, symbols));
      nsMacros = new NameSpace(null, new Map(), this.hooks);
      nsSymbols = new NameSpace(symbols, this.globals, this.hooks);
      macros.set(ns, /** @type {PyValue} */ (/** @type {unknown} */ (nsMacros)), false);
      symbols.set(ns, /** @type {PyValue} */ (/** @type {unknown} */ (nsSymbols)), false);
    }
    const isOptional = optional === null ? false : this.boolean(optional, optional);
    if (elementChildren(el).length > 0)
      this.warn("Child elements of a <xacro:include> tag are ignored");

    const filename = this.absPath(pyStr(this.evalText(/** @type {string} */ (spec), symbols)));
    let files = [filename];
    if (/[*[?]/.test(filename)) {
      const re = globToRegExp(filename);
      files = (this.fs.listFiles?.() ?? []).filter((f) => re.test(f)).sort();
      if (files.length === 0)
        this.warn(`Include tag's filename spec "${filename}" matched no files.`);
    }
    /** @type {XNode[]} */
    const included = [];
    for (const f of files) {
      const text = this.fs.readText(f);
      if (text === null) {
        if (isOptional) continue;
        throw this.error(`No such file or directory: ${f}`, el);
      }
      this.files.push(f);
      this.fileStack.push(f);
      let root;
      try {
        root = parseDocument(text, f);
      } catch (e) {
        if (e instanceof XmlSyntaxError) throw this.error(`XML parsing error: ${e.message}`, el);
        throw e;
      }
      this.evalAll(root, nsMacros, nsSymbols);
      included.push(...root.children);
      this.importNamespaces(parent, root.attrs);
      this.fileStack.pop();
      this.at = el;
    }
    i -= this.removePreviousComments(parent, i);
    return this.replace(parent, i, included);
  }

  /** @param {string} entry */
  run(entry) {
    const text = this.fs.readText(entry);
    if (text === null) throw new XacroError(`No such file or directory: ${entry}`, entry, 0);
    this.files.push(entry);
    this.fileStack.push(entry);
    let root;
    try {
      root = parseDocument(text, entry);
    } catch (e) {
      if (e instanceof XmlSyntaxError)
        throw new XacroError(`XML parsing error: ${e.detail}`, e.file, e.line);
      throw e;
    }
    const macros = new Table(null, new Map(), this.hooks);
    const symbols = new Table(null, this.globals, this.hooks);
    const target = root.attrs["xacro:targetNamespace"];
    if (target) {
      delete root.attrs["xacro:targetNamespace"];
      root.attrs.xmlns = target;
    }
    this.evalAll(root, macros, symbols);
    /** @type {XacroResult} */
    const result = { root, messages: this.messages, files: this.files, args: this.args };
    return result;
  }
}

/**
 * substitution_args._collect_args: the $(...) groups in a string.
 * @param {string} s
 * @param {(msg: string) => Error} fail
 */
function collectArgs(s, fail) {
  /** @type {string[]} */
  const args = [];
  let buf = "";
  let state = 0; // 0 out, 1 dollar, 2 left paren, 3 in
  for (const c of s) {
    if (c === "$") {
      if (state === 0) state = 1;
      else if (state !== 1)
        throw fail(`Dollar signs "$" cannot be inside of substitution args [${s}]`);
    } else if (c === "(") {
      if (state === 1) state = 2;
      else if (state !== 0) throw fail(`Invalid left parenthesis "(" in substitution args [${s}]`);
    } else if (c === ")") {
      if (state === 3) {
        args.push(buf);
        buf = "";
      }
      state = 0;
    } else if (state === 1) state = 0;
    else if (state === 2) state = 3;
    if (state === 3) buf += c;
  }
  return args;
}

/**
 * Expand a xacro document.
 * @param {string} entry path of the top-level file in `fs`
 * @param {XacroFs} fs
 * @param {XacroOptions} [opts]
 * @returns {XacroResult}
 * @throws {XacroError} with file:line and the macro/include trace
 */
export function expandXacro(entry, fs, opts = {}) {
  return new Processor(fs, opts).run(normalize(entry));
}

export { iterate };
