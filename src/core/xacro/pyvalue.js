// Python values as used by xacro expressions, with Python's semantics for
// the operations xacro files rely on. Python's int is arbitrary precision
// and distinct from float ("${6/3}" prints "2.0", "${2*3}" prints "6"), so
// ints are BigInts and floats are numbers. str() of a float follows
// Python's repr (shortest round-trip digits, "1e-05", "2.0").
//
// Mapping: int -> bigint, float -> number, bool -> boolean, str -> string,
// None -> null, list -> Array, tuple -> PyTuple, dict -> PyDict,
// callables -> PyFunction, namespaces -> objects with a `getAttr` method.

export class PyError extends Error {
  /**
   * @param {string} type Python exception class name, e.g. "TypeError"
   * @param {string} message
   */
  constructor(type, message) {
    super(`${type}: ${message}`);
    this.pyType = type;
    this.detail = message;
  }
}

export class PyTuple {
  /** @param {PyValue[]} items */
  constructor(items) {
    this.items = items;
  }
}

/** range(): iterates like a tuple, prints as "range(0, 3)". */
export class PyRange extends PyTuple {
  /**
   * @param {bigint} start
   * @param {bigint} stop
   * @param {bigint} step
   * @param {PyValue[]} items
   */
  constructor(start, stop, step, items) {
    super(items);
    this.start = start;
    this.stop = stop;
    this.step = step;
  }
}

/** A dict with Python key semantics for str, int, float and bool keys. */
export class PyDict {
  constructor() {
    /** @type {Map<string, [PyValue, PyValue]>} */
    this.map = new Map();
  }
  /**
   * @param {PyValue} k
   * @returns {string}
   */
  static keyOf(k) {
    if (typeof k === "string") return `s${k}`;
    if (typeof k === "bigint" || typeof k === "boolean") return `n${BigInt(k)}`;
    if (typeof k === "number") return Number.isInteger(k) ? `n${BigInt(k)}` : `f${k}`;
    if (k === null) return "None";
    if (k instanceof PyTuple) return `t(${k.items.map((x) => PyDict.keyOf(x)).join(",")})`;
    throw new PyError("TypeError", `unhashable type: '${typeName(k)}'`);
  }
  /** @param {PyValue} k @param {PyValue} v */
  set(k, v) {
    this.map.set(PyDict.keyOf(k), [k, v]);
  }
  /** @param {PyValue} k */
  has(k) {
    return this.map.has(PyDict.keyOf(k));
  }
  /** @param {PyValue} k @returns {PyValue | undefined} */
  get(k) {
    return this.map.get(PyDict.keyOf(k))?.[1];
  }
  keys() {
    return [...this.map.values()].map(([k]) => k);
  }
  values() {
    return [...this.map.values()].map(([, v]) => v);
  }
}

export class PyFunction {
  /**
   * @param {string} name
   * @param {(args: PyValue[], kwargs: Map<string, PyValue>) => PyValue} fn
   */
  constructor(name, fn) {
    this.name = name;
    this.fn = fn;
  }
}

/**
 * Anything with attributes (a module-like namespace, a dotified dict).
 * @typedef {{ getAttr(name: string): PyValue, pyName: string }} PyObject
 */

/**
 * @typedef {bigint | number | boolean | string | null | PyValue[] | PyTuple | PyDict
 *   | PyFunction | PyObject | import("./dom.js").XElement} PyValue
 */

/** @param {PyValue} v */
export function typeName(v) {
  if (v === null) return "NoneType";
  switch (typeof v) {
    case "bigint":
      return "int";
    case "number":
      return "float";
    case "boolean":
      return "bool";
    case "string":
      return "str";
  }
  if (Array.isArray(v)) return "list";
  if (v instanceof PyRange) return "range";
  if (v instanceof PyTuple) return "tuple";
  if (v instanceof PyDict) return "dict";
  if (v instanceof PyFunction) return "builtin_function_or_method";
  if ("getAttr" in v) return v.pyName;
  return "Element";
}

/** Shortest repr of a float, as Python prints it. @param {number} x */
export function floatRepr(x) {
  if (Number.isNaN(x)) return "nan";
  if (x === Infinity) return "inf";
  if (x === -Infinity) return "-inf";
  if (x === 0) return Object.is(x, -0) ? "-0.0" : "0.0";
  const [mant, expS] = x.toExponential().split("e");
  const exp = Number(expS);
  const neg = mant.startsWith("-");
  const digits = mant.replace("-", "").replace(".", "");
  let s;
  if (exp < -4 || exp >= 16) {
    const m = digits.length > 1 ? `${digits[0]}.${digits.slice(1)}` : digits;
    s = `${m}e${exp < 0 ? "-" : "+"}${String(Math.abs(exp)).padStart(2, "0")}`;
  } else if (exp >= 0) {
    s =
      digits.length <= exp + 1
        ? `${digits}${"0".repeat(exp + 1 - digits.length)}.0`
        : `${digits.slice(0, exp + 1)}.${digits.slice(exp + 1)}`;
  } else s = `0.${"0".repeat(-exp - 1)}${digits}`;
  return neg ? `-${s}` : s;
}

/** Python repr() of a str. @param {string} s */
export function strRepr(s) {
  const q = s.includes("'") && !s.includes('"') ? '"' : "'";
  let out = q;
  for (const ch of s) {
    const c = /** @type {number} */ (ch.codePointAt(0));
    if (ch === "\\") out += "\\\\";
    else if (ch === q) out += `\\${q}`;
    else if (ch === "\n") out += "\\n";
    else if (ch === "\r") out += "\\r";
    else if (ch === "\t") out += "\\t";
    else if (c < 0x20 || c === 0x7f) out += `\\x${c.toString(16).padStart(2, "0")}`;
    else out += ch;
  }
  return out + q;
}

/** Python repr(). @param {PyValue} v @returns {string} */
export function pyRepr(v) {
  return typeof v === "string" ? strRepr(v) : pyStr(v, pyRepr);
}

/**
 * Python str().
 * @param {PyValue} v
 * @param {(v: PyValue) => string} [inner] how list items are shown (repr)
 * @returns {string}
 */
export function pyStr(v, inner = pyRepr) {
  if (v === null) return "None";
  switch (typeof v) {
    case "string":
      return v;
    case "bigint":
      return v.toString();
    case "number":
      return floatRepr(v);
    case "boolean":
      return v ? "True" : "False";
  }
  if (Array.isArray(v)) return `[${v.map(inner).join(", ")}]`;
  if (v instanceof PyRange)
    return `range(${v.start}, ${v.stop}${v.step === 1n ? "" : `, ${v.step}`})`;
  if (v instanceof PyTuple)
    return v.items.length === 1 ? `(${inner(v.items[0])},)` : `(${v.items.map(inner).join(", ")})`;
  if (v instanceof PyDict)
    return `{${[...v.map.values()].map(([k, x]) => `${inner(k)}: ${inner(x)}`).join(", ")}}`;
  if (v instanceof PyFunction) return `<built-in function ${v.name}>`;
  if ("getAttr" in v) return `<${v.pyName}>`;
  return `<DOM Element: ${v.name}>`;
}

/** Python truthiness. @param {PyValue} v */
export function truthy(v) {
  if (v === null) return false;
  switch (typeof v) {
    case "boolean":
      return v;
    case "bigint":
      return v !== 0n;
    case "number":
      return v !== 0;
    case "string":
      return v.length > 0;
  }
  if (Array.isArray(v)) return v.length > 0;
  if (v instanceof PyTuple) return v.items.length > 0;
  if (v instanceof PyDict) return v.map.size > 0;
  return true;
}

/** @param {PyValue} v @returns {v is bigint | number | boolean} */
export const isNumber = (v) =>
  typeof v === "bigint" || typeof v === "number" || typeof v === "boolean";

/** bool and int as bigint, float unchanged. @param {bigint | number | boolean} v */
const num = (v) => (typeof v === "boolean" ? (v ? 1n : 0n) : v);

/** @param {bigint | number | boolean} v */
export const toFloat = (v) => Number(num(v));

/** Python ==. @param {PyValue} a @param {PyValue} b @returns {boolean} */
export function pyEq(a, b) {
  if (isNumber(a) && isNumber(b)) {
    const x = num(a);
    const y = num(b);
    if (typeof x === "bigint" && typeof y === "bigint") return x === y;
    return Number(x) === Number(y);
  }
  if (typeof a === "string" || typeof b === "string") return a === b;
  if (a === null || b === null) return a === b;
  const la = Array.isArray(a) ? a : null;
  const lb = Array.isArray(b) ? b : null;
  if (la && lb) return la.length === lb.length && la.every((x, k) => pyEq(x, lb[k]));
  if (a instanceof PyTuple && b instanceof PyTuple)
    return a.items.length === b.items.length && a.items.every((x, k) => pyEq(x, b.items[k]));
  if (a instanceof PyDict && b instanceof PyDict)
    return (
      a.map.size === b.map.size &&
      [...a.map.entries()].every(([k, [, v]]) => b.map.has(k) && pyEq(v, b.map.get(k)?.[1] ?? null))
    );
  return a === b;
}

/**
 * Python ordering: -1, 0 or 1; TypeError for unorderable types.
 * @param {PyValue} a
 * @param {PyValue} b
 * @param {string} op for the error message
 * @returns {number}
 */
export function pyCompare(a, b, op) {
  if (isNumber(a) && isNumber(b)) {
    const x = num(a);
    const y = num(b);
    if (typeof x === "bigint" && typeof y === "bigint") return x < y ? -1 : x > y ? 1 : 0;
    const fx = Number(x);
    const fy = Number(y);
    if (Number.isNaN(fx) || Number.isNaN(fy)) return NaN;
    return fx < fy ? -1 : fx > fy ? 1 : 0;
  }
  if (typeof a === "string" && typeof b === "string") return a < b ? -1 : a > b ? 1 : 0;
  const seqA = Array.isArray(a) ? a : a instanceof PyTuple ? a.items : null;
  const seqB = Array.isArray(b) ? b : b instanceof PyTuple ? b.items : null;
  if (seqA && seqB && Array.isArray(a) === Array.isArray(b)) {
    for (let k = 0; k < Math.min(seqA.length, seqB.length); k++)
      if (!pyEq(seqA[k], seqB[k])) return pyCompare(seqA[k], seqB[k], op);
    return Math.sign(seqA.length - seqB.length);
  }
  throw new PyError(
    "TypeError",
    `'${op}' not supported between instances of '${typeName(a)}' and '${typeName(b)}'`,
  );
}

/** @param {string} op @param {PyValue} a @param {PyValue} b @returns {never} */
const unsupported = (op, a, b) => {
  throw new PyError(
    "TypeError",
    `unsupported operand type(s) for ${op}: '${typeName(a)}' and '${typeName(b)}'`,
  );
};

/** Python float floor division and modulo (Objects/floatobject.c). @param {number} a @param {number} b */
function floatDivmod(a, b) {
  if (b === 0) throw new PyError("ZeroDivisionError", "division by zero");
  let mod = a % b;
  let div = (a - mod) / b;
  if (mod) {
    if (b < 0 !== mod < 0) {
      mod += b;
      div -= 1;
    }
  } else mod = b < 0 ? -0 : 0;
  let floordiv;
  if (div) {
    floordiv = Math.floor(div);
    if (div - floordiv > 0.5) floordiv += 1;
  } else floordiv = a / b < 0 ? -0 : 0;
  return { div: floordiv, mod };
}

/** @param {bigint} a @param {bigint} b */
function intDivmod(a, b) {
  if (b === 0n) throw new PyError("ZeroDivisionError", "division by zero");
  let q = a / b;
  let r = a % b;
  if (r !== 0n && r < 0n !== b < 0n) {
    q -= 1n;
    r += b;
  }
  return { div: q, mod: r };
}

/**
 * Binary arithmetic with Python semantics.
 * @param {string} op one of + - * / // % ** << >> & | ^
 * @param {PyValue} a
 * @param {PyValue} b
 * @returns {PyValue}
 */
export function binary(op, a, b) {
  if (isNumber(a) && isNumber(b)) {
    const x = num(a);
    const y = num(b);
    if (typeof x === "bigint" && typeof y === "bigint") {
      switch (op) {
        case "+":
          return x + y;
        case "-":
          return x - y;
        case "*":
          return x * y;
        case "/":
          if (y === 0n) throw new PyError("ZeroDivisionError", "division by zero");
          return Number(x) / Number(y);
        case "//":
          return intDivmod(x, y).div;
        case "%":
          return intDivmod(x, y).mod;
        case "**":
          if (y >= 0n) return x ** y;
          if (x === 0n) throw new PyError("ZeroDivisionError", "zero to a negative power");
          return Number(x) ** Number(y);
        case "<<":
          if (y < 0n) throw new PyError("ValueError", "negative shift count");
          return x << y;
        case ">>":
          if (y < 0n) throw new PyError("ValueError", "negative shift count");
          return x >> y;
        case "&":
          return x & y;
        case "|":
          return x | y;
        case "^":
          return x ^ y;
      }
    }
    const fx = Number(x);
    const fy = Number(y);
    switch (op) {
      case "+":
        return fx + fy;
      case "-":
        return fx - fy;
      case "*":
        return fx * fy;
      case "/":
        if (fy === 0) throw new PyError("ZeroDivisionError", "division by zero");
        return fx / fy;
      case "//":
        return floatDivmod(fx, fy).div;
      case "%":
        return floatDivmod(fx, fy).mod;
      case "**": {
        if (fx === 0 && fy < 0) throw new PyError("ZeroDivisionError", "zero to a negative power");
        if (fx < 0 && !Number.isInteger(fy))
          throw new PyError("ValueError", "complex results are not supported");
        const r = fx ** fy;
        if (!Number.isFinite(r) && Number.isFinite(fx) && Number.isFinite(fy))
          throw new PyError("OverflowError", "(34, 'Numerical result out of range')");
        return r;
      }
    }
    return unsupported(op, a, b);
  }
  if (op === "+") {
    if (typeof a === "string" && typeof b === "string") return a + b;
    if (Array.isArray(a) && Array.isArray(b)) return [...a, ...b];
    if (a instanceof PyTuple && b instanceof PyTuple) return new PyTuple([...a.items, ...b.items]);
  }
  if (op === "*") {
    const rep = (/** @type {PyValue} */ s, /** @type {PyValue} */ k) => {
      if (typeof k !== "bigint" && typeof k !== "boolean") return undefined;
      const times = Math.max(0, Number(num(k)));
      if (typeof s === "string") return s.repeat(times);
      if (Array.isArray(s)) return Array.from({ length: times }, () => s).flat(1);
      if (s instanceof PyTuple)
        return new PyTuple(Array.from({ length: times }, () => s.items).flat(1));
      return undefined;
    };
    const r = rep(a, b) ?? rep(b, a);
    if (r !== undefined) return r;
  }
  if (op === "%" && typeof a === "string") return percentFormat(a, b);
  return unsupported(op, a, b);
}

/**
 * Unary operators.
 * @param {string} op "-", "+" or "~"
 * @param {PyValue} a
 * @returns {PyValue}
 */
export function unary(op, a) {
  if (!isNumber(a))
    throw new PyError("TypeError", `bad operand type for unary ${op}: '${typeName(a)}'`);
  const x = num(a);
  if (op === "-") return typeof x === "bigint" ? -x : -x;
  if (op === "+") return x;
  if (typeof x !== "bigint")
    throw new PyError("TypeError", `bad operand type for unary ~: '${typeName(a)}'`);
  return ~x;
}

/**
 * Round the exact decimal value of a double to `nd` fractional digits,
 * ties to even, as Python's round(x, n) and format(x, ".nf") do.
 * @param {number} x
 * @param {number} nd 0..20
 * @returns {string}
 */
export function fixedHalfEven(x, nd) {
  if (!Number.isFinite(x)) return floatRepr(x);
  const neg = x < 0 || Object.is(x, -0);
  // toFixed(100) prints the exact binary value for any double that matters here.
  const exact = Math.abs(x).toFixed(100);
  const [ip, fp] = exact.split(".");
  const keep = fp.slice(0, nd);
  const rest = fp.slice(nd);
  let digits = ip + keep;
  const first = rest[0] ?? "0";
  const tail = rest.slice(1);
  const last = Number(digits[digits.length - 1]);
  const up = first > "5" || (first === "5" && (/[1-9]/.test(tail) || last % 2 === 1));
  if (up) {
    let k = digits.length - 1;
    const arr = digits.split("");
    while (k >= 0) {
      if (arr[k] === "9") {
        arr[k] = "0";
        k--;
      } else {
        arr[k] = String(Number(arr[k]) + 1);
        break;
      }
    }
    digits = (k < 0 ? "1" : "") + arr.join("");
  }
  const intLen = digits.length - nd;
  const s = nd > 0 ? `${digits.slice(0, intLen)}.${digits.slice(intLen)}` : digits;
  const clean = s.replace(/^0+(?=\d)/, "");
  return neg ? `-${clean}` : clean;
}

/**
 * Old-style "%" string formatting: %s %r %d %i %f %e %g with an optional
 * precision, and %%.
 * @param {string} fmt
 * @param {PyValue} arg
 */
function percentFormat(fmt, arg) {
  const args = arg instanceof PyTuple ? arg.items : [arg];
  let k = 0;
  const out = fmt.replace(/%(?:\.(\d+))?([srdifeg%])/g, (_m, prec, conv) => {
    if (conv === "%") return "%";
    if (k >= args.length) throw new PyError("TypeError", "not enough arguments for format string");
    const v = args[k++];
    if (conv === "s") return pyStr(v);
    if (conv === "r") return pyRepr(v);
    if (!isNumber(v))
      throw new PyError(
        "TypeError",
        `%${conv} format: a real number is required, not ${typeName(v)}`,
      );
    if (conv === "d" || conv === "i")
      return (typeof num(v) === "bigint" ? num(v) : BigInt(Math.trunc(Number(num(v))))).toString();
    const p = prec === undefined ? 6 : Number(prec);
    return formatFloat(toFloat(v), p, conv);
  });
  if (k < args.length)
    throw new PyError("TypeError", "not all arguments converted during string formatting");
  return out;
}

/**
 * Format with a precision and a type letter f, e or g.
 * @param {number} x
 * @param {number} p
 * @param {string} type
 */
export function formatFloat(x, p, type) {
  if (type === "f") return fixedHalfEven(x, p);
  if (type === "e") {
    const s = x.toExponential(p);
    return s.replace(/e([+-])(\d)$/, "e$10$2");
  }
  // g: Python's general format
  const P = p === 0 ? 1 : p;
  if (x === 0) return "0";
  const exp = Math.floor(Math.log10(Math.abs(Number(x.toExponential(P - 1)))));
  let s;
  if (exp < -4 || exp >= P) {
    s = x.toExponential(P - 1).replace(/\.?0+e/, "e");
    s = s.replace(/e([+-])(\d)$/, "e$10$2");
  } else {
    s = fixedHalfEven(x, Math.max(0, P - 1 - exp));
    if (s.includes(".")) s = s.replace(/\.?0+$/, "");
  }
  return s;
}
