// Evaluator for the AST from pyexpr.js, plus the symbols xacro makes
// available to expressions. xacro 2.x (xacro/__init__.py,
// create_global_symbols) exposes: list dict map len str float int bool True
// False min max round directly; sorted and range directly with a
// deprecation warning; python.* with more builtins; every public name of
// Python's math module both directly and as math.*; and xacro.* helpers.
// This module implements the subset listed in docs/xacro.md; anything else
// raises a NameError or a TypeError naming what is missing.

import { parseExpression } from "./pyexpr.js";
import {
  PyDict,
  PyError,
  PyFunction,
  PyRange,
  PyTuple,
  binary,
  fixedHalfEven,
  floatRepr,
  formatFloat,
  isNumber,
  pyCompare,
  pyEq,
  pyRepr,
  pyStr,
  toFloat,
  truthy,
  typeName,
  unary,
} from "./pyvalue.js";

/** @typedef {import("./pyvalue.js").PyValue} PyValue */
/** @typedef {import("./pyexpr.js").Node} Node */

export const NOT_FOUND = Symbol("not found");

/**
 * Where names are looked up.
 * @typedef {{ lookup(name: string): PyValue | typeof NOT_FOUND }} Scope
 */

/** A module-like object whose attributes are a fixed map. */
export class PyModule {
  /**
   * @param {string} name
   * @param {Map<string, PyValue>} members
   */
  constructor(name, members) {
    this.pyName = name;
    this.members = members;
  }
  /** @param {string} name @returns {PyValue} */
  getAttr(name) {
    const v = this.members.get(name);
    if (v === undefined)
      throw new PyError("AttributeError", `module '${this.pyName}' has no attribute '${name}'`);
    return v;
  }
}

/** A dict whose keys are also attributes, as returned by xacro.dotify(). */
export class DotDict extends PyDict {}

/** @param {PyValue} v @returns {PyValue[]} */
export function iterate(v) {
  if (Array.isArray(v)) return [...v];
  if (v instanceof PyTuple) return [...v.items];
  if (typeof v === "string") return [...v];
  if (v instanceof PyDict) return v.keys();
  throw new PyError("TypeError", `'${typeName(v)}' object is not iterable`);
}

/** Python float(str) parsing. @param {string} s */
function parseFloatStrict(s) {
  const t = s.trim();
  if (/^[+-]?(\d+\.?\d*(e[+-]?\d+)?|\.\d+(e[+-]?\d+)?)$/i.test(t)) return Number(t);
  const m = /^([+-]?)(inf|infinity|nan)$/i.exec(t);
  if (m) return m[2].toLowerCase() === "nan" ? NaN : m[1] === "-" ? -Infinity : Infinity;
  throw new PyError("ValueError", `could not convert string to float: ${pyRepr(s)}`);
}

/** Python int(str, base). @param {string} s @param {number} base */
function parseIntStrict(s, base) {
  const t = s.trim().replace(/(?<=\w)_(?=\w)/g, "");
  const ok =
    base === 10
      ? /^[+-]?\d+$/.test(t)
      : base === 16
        ? /^[+-]?(0[xX])?[0-9a-fA-F]+$/.test(t)
        : base === 8
          ? /^[+-]?(0[oO])?[0-7]+$/.test(t)
          : base === 2
            ? /^[+-]?(0[bB])?[01]+$/.test(t)
            : false;
  if (!ok)
    throw new PyError("ValueError", `invalid literal for int() with base ${base}: ${pyRepr(s)}`);
  const neg = t.startsWith("-");
  const body = t.replace(/^[+-]/, "").replace(/^0[xXoObB]/, "");
  const prefix = base === 16 ? "0x" : base === 8 ? "0o" : base === 2 ? "0b" : "";
  const v = BigInt(prefix + body);
  return neg ? -v : v;
}

/** @param {PyValue} v @param {string} fn */
function asFloat(v, fn) {
  if (!isNumber(v))
    throw new PyError("TypeError", `${fn}() argument must be a real number, not '${typeName(v)}'`);
  return toFloat(v);
}

/** @param {PyValue} v @param {string} fn */
function asInt(v, fn) {
  if (typeof v === "boolean") return v ? 1n : 0n;
  if (typeof v === "bigint") return v;
  throw new PyError("TypeError", `${fn}() argument must be an integer, not '${typeName(v)}'`);
}

/** @param {number} x */
function floatToInt(x) {
  if (Number.isNaN(x)) throw new PyError("ValueError", "cannot convert float NaN to integer");
  if (!Number.isFinite(x))
    throw new PyError("OverflowError", "cannot convert float infinity to integer");
  return BigInt(Math.trunc(x));
}

/**
 * @param {string} name
 * @param {number} min
 * @param {number} max
 * @param {(...a: PyValue[]) => PyValue} f
 * @returns {PyFunction}
 */
function fn(name, min, max, f) {
  return new PyFunction(name, (args, kwargs) => {
    if (kwargs.size > 0) throw new PyError("TypeError", `${name}() takes no keyword arguments`);
    if (args.length < min || args.length > max)
      throw new PyError(
        "TypeError",
        `${name}() takes ${min === max ? min : `${min} to ${max}`} argument(s) (${args.length} given)`,
      );
    return f(...args);
  });
}

/** @param {string} name @param {(x: number) => number} f */
const mathFloat = (name, f) =>
  fn(name, 1, 1, (x) => {
    const r = f(asFloat(x, name));
    if (Number.isNaN(r) && !Number.isNaN(asFloat(x, name)))
      throw new PyError("ValueError", "math domain error");
    return r;
  });

/** @param {string} name @param {(x: number) => number} f */
const mathToInt = (name, f) =>
  fn(name, 1, 1, (x) => (typeof x === "bigint" ? x : floatToInt(f(asFloat(x, name)))));

/** @param {PyValue[]} args @param {string} name */
function minMax(args, name) {
  const items = args.length === 1 ? iterate(args[0]) : args;
  if (items.length === 0) throw new PyError("ValueError", `${name}() iterable argument is empty`);
  let best = items[0];
  for (const x of items.slice(1)) {
    const c = pyCompare(x, best, name === "min" ? "<" : ">");
    if (name === "min" ? c < 0 : c > 0) best = x;
  }
  return best;
}

/**
 * Python round().
 * @param {PyValue} x
 * @param {PyValue} [nd]
 * @returns {PyValue}
 */
function pyRound(x, nd) {
  if (!isNumber(x))
    throw new PyError("TypeError", `type ${typeName(x)} doesn't define __round__ method`);
  if (nd === undefined || nd === null) {
    if (typeof x !== "number") return asInt(x, "round");
    if (!Number.isFinite(x)) floatToInt(x);
    const f = Math.floor(x);
    const diff = x - f;
    const r = diff > 0.5 || (diff === 0.5 && f % 2 !== 0) ? f + 1 : f;
    return BigInt(r);
  }
  const n = Number(asInt(nd, "round"));
  if (typeof x !== "number") {
    const v = asInt(x, "round");
    if (n >= 0) return v;
    const m = 10n ** BigInt(-n);
    const q = v / m;
    const r = v - q * m;
    const half = m / 2n;
    const absR = r < 0n ? -r : r;
    let qq = q;
    if (absR > half || (absR === half && q % 2n !== 0n)) qq += v < 0n ? -1n : 1n;
    return qq * m;
  }
  if (!Number.isFinite(x)) return x;
  if (n >= 0) return n > 30 ? x : Number(fixedHalfEven(x, n));
  const m = 10 ** -n;
  return Number(fixedHalfEven(x / m, 0)) * m;
}

/** @param {PyValue[]} args */
function pyRange(args) {
  const ints = args.map((a) => asInt(a, "range"));
  const [start, stop, step] =
    ints.length === 1 ? [0n, ints[0], 1n] : [ints[0], ints[1], ints[2] ?? 1n];
  if (step === 0n) throw new PyError("ValueError", "range() arg 3 must not be zero");
  /** @type {PyValue[]} */
  const out = [];
  for (let v = start; step > 0n ? v < stop : v > stop; v += step) {
    out.push(v);
    if (out.length > 1_000_000) throw new PyError("MemoryError", "range too large");
  }
  return new PyRange(start, stop, step, out);
}

/**
 * Call a callable value.
 * @param {PyValue} f
 * @param {PyValue[]} args
 * @param {Map<string, PyValue>} [kwargs]
 */
export function callValue(f, args, kwargs = new Map()) {
  if (!(f instanceof PyFunction))
    throw new PyError("TypeError", `'${typeName(f)}' object is not callable`);
  return f.fn(args, kwargs);
}

/**
 * The builtins and the math module as xacro exposes them.
 * @param {{ warn: (msg: string) => void }} hooks
 * @returns {{ builtins: Map<string, PyValue>, math: PyModule, python: PyModule }}
 */
export function makeBuiltins(hooks) {
  /** @type {Map<string, PyValue>} */
  const b = new Map();
  b.set("True", true);
  b.set("False", false);
  b.set(
    "list",
    fn("list", 0, 1, (x) => (x === undefined ? [] : iterate(x))),
  );
  b.set(
    "dict",
    new PyFunction("dict", (args, kwargs) => {
      const d = new PyDict();
      if (args.length > 1) throw new PyError("TypeError", "dict expected at most 1 argument");
      if (args.length === 1) {
        if (args[0] instanceof PyDict)
          for (const k of args[0].keys()) d.set(k, args[0].get(k) ?? null);
        else
          for (const pair of iterate(args[0])) {
            const kv = iterate(pair);
            if (kv.length !== 2)
              throw new PyError(
                "ValueError",
                "dictionary update sequence element has wrong length",
              );
            d.set(kv[0], kv[1]);
          }
      }
      for (const [k, v] of kwargs) d.set(k, v);
      return d;
    }),
  );
  b.set(
    "map",
    new PyFunction("map", (args) => {
      if (args.length < 2)
        throw new PyError("TypeError", "map() must have at least two arguments.");
      const lists = args.slice(1).map(iterate);
      const n = Math.min(...lists.map((l) => l.length));
      return Array.from({ length: n }, (_, k) =>
        callValue(
          args[0],
          lists.map((l) => l[k]),
        ),
      );
    }),
  );
  b.set(
    "len",
    fn("len", 1, 1, (x) => {
      if (typeof x === "string") return BigInt([...x].length);
      if (x instanceof PyDict) return BigInt(x.map.size);
      return BigInt(iterate(x).length);
    }),
  );
  b.set(
    "str",
    fn("str", 0, 1, (x) => (x === undefined ? "" : pyStr(x))),
  );
  b.set(
    "float",
    fn("float", 0, 1, (x) => {
      if (x === undefined) return 0;
      if (typeof x === "string") return parseFloatStrict(x);
      return asFloat(x, "float");
    }),
  );
  b.set(
    "int",
    fn("int", 0, 2, (x, base) => {
      if (x === undefined) return 0n;
      if (typeof x === "string")
        return parseIntStrict(x, base === undefined ? 10 : Number(asInt(base, "int")));
      if (base !== undefined)
        throw new PyError("TypeError", "int() can't convert non-string with explicit base");
      if (typeof x === "number") return floatToInt(x);
      return asInt(x, "int");
    }),
  );
  b.set(
    "bool",
    fn("bool", 0, 1, (x) => (x === undefined ? false : truthy(x))),
  );
  b.set("min", new PyFunction("min", (args) => minMax(args, "min")));
  b.set("max", new PyFunction("max", (args) => minMax(args, "max")));
  b.set("round", fn("round", 1, 2, pyRound));

  const sorted = fn("sorted", 1, 1, (x) => iterate(x).sort((p, q) => pyCompare(p, q, "<") || 0));
  const range = new PyFunction("range", (args) => {
    if (args.length < 1 || args.length > 3)
      throw new PyError("TypeError", `range expected 1 to 3 arguments, got ${args.length}`);
    return pyRange(args);
  });

  /** @type {Map<string, PyValue>} */
  const py = new Map(b);
  py.set("sorted", sorted);
  py.set("range", range);
  py.set(
    "abs",
    fn("abs", 1, 1, (x) => {
      if (!isNumber(x))
        throw new PyError("TypeError", `bad operand type for abs(): '${typeName(x)}'`);
      const v = typeof x === "boolean" ? (x ? 1n : 0n) : x;
      return typeof v === "bigint" ? (v < 0n ? -v : v) : Math.abs(v);
    }),
  );
  py.set(
    "all",
    fn("all", 1, 1, (x) => iterate(x).every(truthy)),
  );
  py.set(
    "any",
    fn("any", 1, 1, (x) => iterate(x).some(truthy)),
  );
  py.set(
    "sum",
    fn("sum", 1, 2, (x, start) =>
      iterate(x).reduce((acc, v) => binary("+", acc, v), start === undefined ? 0n : start),
    ),
  );
  py.set(
    "tuple",
    fn("tuple", 0, 1, (x) => new PyTuple(x === undefined ? [] : iterate(x))),
  );
  py.set(
    "reversed",
    fn("reversed", 1, 1, (x) => iterate(x).reverse()),
  );
  py.set(
    "zip",
    new PyFunction("zip", (args) => {
      const lists = args.map(iterate);
      const n = lists.length === 0 ? 0 : Math.min(...lists.map((l) => l.length));
      return Array.from({ length: n }, (_, k) => new PyTuple(lists.map((l) => l[k])));
    }),
  );
  py.set(
    "enumerate",
    fn("enumerate", 1, 2, (x, start) => {
      const s = start === undefined ? 0n : asInt(start, "enumerate");
      return iterate(x).map((v, k) => new PyTuple([s + BigInt(k), v]));
    }),
  );
  py.set(
    "ord",
    fn("ord", 1, 1, (x) => {
      if (typeof x !== "string" || [...x].length !== 1)
        throw new PyError("TypeError", "ord() expected a character");
      return BigInt(/** @type {number} */ (x.codePointAt(0)));
    }),
  );
  py.set(
    "repr",
    fn("repr", 1, 1, (x) => pyRepr(x)),
  );
  py.set(
    "divmod",
    fn("divmod", 2, 2, (x, y) => new PyTuple([binary("//", x, y), binary("%", x, y)])),
  );
  const python = new PyModule("python", py);

  /** @type {Map<string, PyValue>} */
  const m = new Map();
  m.set("pi", Math.PI);
  m.set("e", Math.E);
  m.set("tau", 2 * Math.PI);
  m.set("inf", Infinity);
  m.set("nan", NaN);
  /** @type {[string, (x: number) => number][]} */
  const unaryFloat = [
    ["sin", Math.sin],
    ["cos", Math.cos],
    ["tan", Math.tan],
    ["asin", Math.asin],
    ["acos", Math.acos],
    ["atan", Math.atan],
    ["sinh", Math.sinh],
    ["cosh", Math.cosh],
    ["tanh", Math.tanh],
    ["asinh", Math.asinh],
    ["acosh", Math.acosh],
    ["atanh", Math.atanh],
    ["sqrt", Math.sqrt],
    ["cbrt", Math.cbrt],
    ["exp", Math.exp],
    ["expm1", Math.expm1],
    ["log10", Math.log10],
    ["log2", Math.log2],
    ["log1p", Math.log1p],
    ["fabs", Math.abs],
    ["degrees", (x) => x * (180 / Math.PI)],
    ["radians", (x) => x * (Math.PI / 180)],
  ];
  for (const [name, f] of unaryFloat) m.set(name, mathFloat(name, f));
  m.set("floor", mathToInt("floor", Math.floor));
  m.set("ceil", mathToInt("ceil", Math.ceil));
  m.set("trunc", mathToInt("trunc", Math.trunc));
  m.set(
    "log",
    fn("log", 1, 2, (x, base) => {
      const v = asFloat(x, "log");
      if (v <= 0) throw new PyError("ValueError", "math domain error");
      return base === undefined ? Math.log(v) : Math.log(v) / Math.log(asFloat(base, "log"));
    }),
  );
  m.set(
    "pow",
    fn("pow", 2, 2, (x, y) => {
      const a = asFloat(x, "pow");
      const c = asFloat(y, "pow");
      if (a < 0 && !Number.isInteger(c)) throw new PyError("ValueError", "math domain error");
      return a ** c;
    }),
  );
  m.set(
    "atan2",
    fn("atan2", 2, 2, (y, x) => Math.atan2(asFloat(y, "atan2"), asFloat(x, "atan2"))),
  );
  m.set(
    "hypot",
    new PyFunction("hypot", (args) => Math.hypot(...args.map((a) => asFloat(a, "hypot")))),
  );
  m.set(
    "fmod",
    fn("fmod", 2, 2, (x, y) => {
      const d = asFloat(y, "fmod");
      if (d === 0) throw new PyError("ValueError", "math domain error");
      return asFloat(x, "fmod") % d;
    }),
  );
  m.set(
    "copysign",
    fn("copysign", 2, 2, (x, y) => {
      const a = Math.abs(asFloat(x, "copysign"));
      const s = asFloat(y, "copysign");
      return s < 0 || Object.is(s, -0) ? -a : a;
    }),
  );
  m.set(
    "isnan",
    fn("isnan", 1, 1, (x) => Number.isNaN(asFloat(x, "isnan"))),
  );
  m.set(
    "isinf",
    fn("isinf", 1, 1, (x) => Math.abs(asFloat(x, "isinf")) === Infinity),
  );
  m.set(
    "isfinite",
    fn("isfinite", 1, 1, (x) => Number.isFinite(asFloat(x, "isfinite"))),
  );
  m.set(
    "isclose",
    new PyFunction("isclose", (args, kw) => {
      const [a, c] = args.map((v) => asFloat(v, "isclose"));
      const rel = kw.has("rel_tol")
        ? asFloat(/** @type {PyValue} */ (kw.get("rel_tol")), "isclose")
        : 1e-9;
      const abs = kw.has("abs_tol")
        ? asFloat(/** @type {PyValue} */ (kw.get("abs_tol")), "isclose")
        : 0;
      return a === c || Math.abs(a - c) <= Math.max(rel * Math.max(Math.abs(a), Math.abs(c)), abs);
    }),
  );
  m.set(
    "factorial",
    fn("factorial", 1, 1, (x) => {
      const n = asInt(x, "factorial");
      if (n < 0n) throw new PyError("ValueError", "factorial() not defined for negative values");
      let r = 1n;
      for (let k = 2n; k <= n; k++) r *= k;
      return r;
    }),
  );
  m.set(
    "gcd",
    new PyFunction("gcd", (args) => {
      let g = 0n;
      for (const a of args) {
        let x = asInt(a, "gcd");
        if (x < 0n) x = -x;
        let y = g;
        while (y) [x, y] = [y, x % y];
        g = x;
      }
      return g;
    }),
  );
  const math = new PyModule("math", m);
  for (const [k, v] of m) b.set(k, v);
  b.set("math", math);
  b.set("python", python);
  /**
   * @param {string} name
   * @param {PyFunction} f
   */
  const deprecated = (name, f) =>
    new PyFunction(name, (args, kw) => {
      hooks.warn(`Using ${name}() directly is deprecated. Use python.${name}() instead.`);
      return f.fn(args, kw);
    });
  b.set("sorted", deprecated("sorted", sorted));
  b.set("range", deprecated("range", range));
  return { builtins: b, math, python };
}

/**
 * Methods of str, list and dict values that xacro files use.
 * @param {PyValue} obj
 * @param {string} name
 * @returns {PyValue}
 */
function method(obj, name) {
  if (typeof obj === "string") {
    /** @param {PyValue} v */
    const str = (v) => {
      if (typeof v !== "string") throw new PyError("TypeError", `must be str, not ${typeName(v)}`);
      return v;
    };
    /** @type {Record<string, PyFunction>} */
    const methods = {
      upper: fn("upper", 0, 0, () => obj.toUpperCase()),
      lower: fn("lower", 0, 0, () => obj.toLowerCase()),
      strip: fn("strip", 0, 1, (c) => stripChars(obj, c, true, true)),
      lstrip: fn("lstrip", 0, 1, (c) => stripChars(obj, c, true, false)),
      rstrip: fn("rstrip", 0, 1, (c) => stripChars(obj, c, false, true)),
      split: fn("split", 0, 2, (sep, max) => {
        const limit = max === undefined || max === null ? -1 : Number(asInt(max, "split"));
        if (sep === undefined || sep === null) {
          const parts = obj
            .trim()
            .split(/\s+/)
            .filter((x) => x !== "");
          if (limit < 0 || parts.length <= limit + 1) return parts;
          const head = parts.slice(0, limit);
          let rest = obj.trimStart();
          for (const h of head) rest = rest.slice(rest.indexOf(h) + h.length).trimStart();
          return [...head, rest];
        }
        const s = str(sep);
        if (s === "") throw new PyError("ValueError", "empty separator");
        const parts = obj.split(s);
        if (limit < 0 || parts.length <= limit + 1) return parts;
        return [...parts.slice(0, limit), parts.slice(limit).join(s)];
      }),
      join: fn("join", 1, 1, (it) => iterate(it).map(str).join(obj)),
      replace: fn("replace", 2, 2, (a, c) => obj.split(str(a)).join(str(c))),
      startswith: fn("startswith", 1, 1, (p) => obj.startsWith(str(p))),
      endswith: fn("endswith", 1, 1, (p) => obj.endsWith(str(p))),
      find: fn("find", 1, 1, (p) => BigInt(obj.indexOf(str(p)))),
      count: fn("count", 1, 1, (p) => BigInt(obj.split(str(p)).length - 1)),
      format: new PyFunction("format", (args, kw) => strFormat(obj, args, kw)),
    };
    const f = methods[name];
    if (f) return f;
  } else if (Array.isArray(obj)) {
    if (name === "index")
      return fn("index", 1, 1, (x) => {
        const k = obj.findIndex((v) => pyEq(v, x));
        if (k < 0) throw new PyError("ValueError", `${pyRepr(x)} is not in list`);
        return BigInt(k);
      });
    if (name === "count")
      return fn("count", 1, 1, (x) => BigInt(obj.filter((v) => pyEq(v, x)).length));
  } else if (obj instanceof PyDict) {
    if (name === "get")
      return fn("get", 1, 2, (k, d) =>
        obj.has(k) ? (obj.get(k) ?? null) : d === undefined ? null : d,
      );
    if (name === "keys") return fn("keys", 0, 0, () => obj.keys());
    if (name === "values") return fn("values", 0, 0, () => obj.values());
    if (name === "items")
      return fn("items", 0, 0, () => obj.keys().map((k) => new PyTuple([k, obj.get(k) ?? null])));
  }
  throw new PyError("AttributeError", `'${typeName(obj)}' object has no attribute '${name}'`);
}

/** @param {string} s @param {PyValue} chars @param {boolean} left @param {boolean} right */
function stripChars(s, chars, left, right) {
  const set = chars === undefined || chars === null ? null : String(chars);
  const strip = (/** @type {string} */ c) => (set === null ? /\s/.test(c) : set.includes(c));
  let a = 0;
  let z = s.length;
  if (left) while (a < z && strip(s[a])) a++;
  if (right) while (z > a && strip(s[z - 1])) z--;
  return s.slice(a, z);
}

/**
 * str.format with positional, numbered and named fields and the format
 * specs "", "d", ".Nf", ".Ne", ".Ng", "g".
 * @param {string} f
 * @param {PyValue[]} args
 * @param {Map<string, PyValue>} kw
 */
function strFormat(f, args, kw) {
  let auto = 0;
  return f.replace(/\{\{|\}\}|\{([^{}:!]*)(?::([^{}]*))?\}/g, (m, field, spec) => {
    if (m === "{{") return "{";
    if (m === "}}") return "}";
    /** @type {PyValue | undefined} */
    let v;
    if (field === "") v = args[auto++];
    else if (/^\d+$/.test(field)) v = args[Number(field)];
    else v = kw.get(field);
    if (v === undefined)
      throw new PyError("IndexError", `Replacement index ${field || auto - 1} out of range`);
    if (!spec) return pyStr(v);
    const sm = /^\.(\d+)([fegFEG%]?)$|^([dg])$/.exec(spec);
    if (!sm) throw new PyError("ValueError", `format spec '${spec}' is not supported`);
    if (sm[3] === "d") {
      if (typeof v !== "bigint" && typeof v !== "boolean")
        throw new PyError(
          "ValueError",
          `Unknown format code 'd' for object of type '${typeName(v)}'`,
        );
      return asInt(v, "format").toString();
    }
    if (!isNumber(v)) throw new PyError("ValueError", `format spec '${spec}' needs a number`);
    if (sm[3] === "g") return formatFloat(toFloat(v), 6, "g");
    const type = (sm[2] || "g").toLowerCase();
    if (type === "%") return `${fixedHalfEven(toFloat(v) * 100, Number(sm[1]))}%`;
    return formatFloat(toFloat(v), Number(sm[1]), type);
  });
}

/** @param {string} name */
function checkName(name) {
  if (name.startsWith("__")) throw new PyError("XacroError", `Use of invalid name(s): ${name}`);
}

/**
 * Evaluate an expression AST.
 * @param {Node} node
 * @param {Scope} scope
 * @returns {PyValue}
 */
export function evaluate(node, scope) {
  switch (node.k) {
    case "const":
      return node.v;
    case "name": {
      checkName(node.id);
      const v = scope.lookup(node.id);
      if (v === NOT_FOUND) throw new PyError("NameError", `name '${node.id}' is not defined`);
      return v;
    }
    case "tuple":
      return new PyTuple(node.items.map((x) => evaluate(x, scope)));
    case "list":
      return node.items.map((x) => evaluate(x, scope));
    case "dict": {
      const d = new PyDict();
      node.keys.forEach((k, i) => d.set(evaluate(k, scope), evaluate(node.values[i], scope)));
      return d;
    }
    case "attr": {
      checkName(node.name);
      if (node.name.startsWith("_"))
        throw new PyError(
          "AttributeError",
          `access to private attribute '${node.name}' is not allowed`,
        );
      const obj = evaluate(node.obj, scope);
      if (obj instanceof DotDict && obj.has(node.name)) return obj.get(node.name) ?? null;
      if (obj instanceof PyModule || (obj !== null && typeof obj === "object" && "getAttr" in obj))
        return /** @type {import("./pyvalue.js").PyObject} */ (obj).getAttr(node.name);
      return method(obj, node.name);
    }
    case "sub": {
      const obj = evaluate(node.obj, scope);
      const idx = evaluate(node.index, scope);
      return subscript(obj, idx);
    }
    case "slice": {
      const obj = evaluate(node.obj, scope);
      /** @param {Node | null} n */
      const opt = (n) => {
        if (n === null) return null;
        const v = evaluate(n, scope);
        return v === null ? null : Number(asInt(v, "slice"));
      };
      return slice(obj, opt(node.lo), opt(node.hi), opt(node.step));
    }
    case "call": {
      const f = evaluate(node.fn, scope);
      const args = node.args.map((a) => evaluate(a, scope));
      /** @type {Map<string, PyValue>} */
      const kwargs = new Map();
      for (const [k, v] of node.kwargs) kwargs.set(k, evaluate(v, scope));
      return callValue(f, args, kwargs);
    }
    case "unary":
      return unary(node.op, evaluate(node.x, scope));
    case "not":
      return !truthy(evaluate(node.x, scope));
    case "bin":
      return binary(node.op, evaluate(node.a, scope), evaluate(node.b, scope));
    case "bool": {
      let v = evaluate(node.items[0], scope);
      for (const x of node.items.slice(1)) {
        if (node.op === "and" ? !truthy(v) : truthy(v)) return v;
        v = evaluate(x, scope);
      }
      return v;
    }
    case "if":
      return truthy(evaluate(node.test, scope))
        ? evaluate(node.body, scope)
        : evaluate(node.orelse, scope);
    case "cmp": {
      let left = evaluate(node.first, scope);
      for (const [op, n] of node.rest) {
        const right = evaluate(n, scope);
        if (!compare(op, left, right)) return false;
        left = right;
      }
      return true;
    }
  }
}

/**
 * @param {string} op
 * @param {PyValue} a
 * @param {PyValue} b
 */
function compare(op, a, b) {
  switch (op) {
    case "==":
      return pyEq(a, b);
    case "!=":
      return !pyEq(a, b);
    case "in":
    case "not in": {
      let r;
      if (typeof b === "string") {
        if (typeof a !== "string")
          throw new PyError(
            "TypeError",
            `'in <string>' requires string as left operand, not ${typeName(a)}`,
          );
        r = b.includes(a);
      } else if (b instanceof PyDict) r = b.has(a);
      else r = iterate(b).some((x) => pyEq(x, a));
      return op === "in" ? r : !r;
    }
    case "is":
    case "is not": {
      const same = a === b || (typeof a === "boolean" && a === b);
      return op === "is" ? same : !same;
    }
  }
  const c = pyCompare(a, b, op);
  if (Number.isNaN(c)) return false;
  return op === "<" ? c < 0 : op === ">" ? c > 0 : op === "<=" ? c <= 0 : c >= 0;
}

/** @param {PyValue} obj @param {PyValue} idx @returns {PyValue} */
function subscript(obj, idx) {
  if (obj instanceof PyDict) {
    const v = obj.get(idx);
    if (v === undefined) throw new PyError("KeyError", pyRepr(idx));
    return v;
  }
  const seq = Array.isArray(obj)
    ? obj
    : obj instanceof PyTuple
      ? obj.items
      : typeof obj === "string"
        ? [...obj]
        : null;
  if (!seq) throw new PyError("TypeError", `'${typeName(obj)}' object is not subscriptable`);
  if (typeof idx !== "bigint" && typeof idx !== "boolean")
    throw new PyError(
      "TypeError",
      `${typeName(obj)} indices must be integers, not ${typeName(idx)}`,
    );
  let k = Number(asInt(idx, "index"));
  if (k < 0) k += seq.length;
  if (k < 0 || k >= seq.length)
    throw new PyError("IndexError", `${typeName(obj)} index out of range`);
  return seq[k];
}

/**
 * @param {PyValue} obj
 * @param {number | null} lo
 * @param {number | null} hi
 * @param {number | null} step
 * @returns {PyValue}
 */
function slice(obj, lo, hi, step) {
  const seq = Array.isArray(obj)
    ? obj
    : obj instanceof PyTuple
      ? obj.items
      : typeof obj === "string"
        ? [...obj]
        : null;
  if (!seq) throw new PyError("TypeError", `'${typeName(obj)}' object is not subscriptable`);
  const n = seq.length;
  const st = step ?? 1;
  if (st === 0) throw new PyError("ValueError", "slice step cannot be zero");
  /** @param {number | null} v @param {number} dflt */
  const norm = (v, dflt) => {
    if (v === null) return dflt;
    if (v < 0) v += n;
    return st > 0 ? Math.min(Math.max(v, 0), n) : Math.min(Math.max(v, -1), n - 1);
  };
  const a = norm(lo, st > 0 ? 0 : n - 1);
  const z = norm(hi, st > 0 ? n : -1);
  /** @type {PyValue[]} */
  const out = [];
  for (let k = a; st > 0 ? k < z : k > z; k += st) out.push(seq[k]);
  if (typeof obj === "string") return out.join("");
  return obj instanceof PyTuple ? new PyTuple(out) : out;
}

/**
 * Parse and evaluate an expression string.
 * @param {string} src
 * @param {Scope} scope
 */
export const evalExpression = (src, scope) => evaluate(parseExpression(src.trim()), scope);

export { floatRepr };
