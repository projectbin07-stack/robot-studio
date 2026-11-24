// JSON as rosbridge sends it. rosbridge_server serializes with Python's
// json module, which writes NaN, Infinity and -Infinity as bare tokens
// (a LaserScan with no-return beams, an IMU without covariance); those are
// not JSON and JSON.parse rejects them. Standard JSON goes through
// JSON.parse; text that fails is parsed again by a strict JSON reader that
// also accepts exactly those three tokens. Nothing is evaluated, nesting
// depth and size are bounded.

export class RosJsonError extends Error {}

/** Deepest nesting accepted. */
export const MAX_DEPTH = 64;

/**
 * @param {string} text
 * @param {{ maxBytes?: number }} [opts] maxBytes bounds the text length (UTF-16 code units)
 * @returns {unknown}
 */
export function parseRosJson(text, opts = {}) {
  const max = opts.maxBytes ?? Infinity;
  if (text.length > max) throw new RosJsonError(`message larger than ${max} bytes`);
  try {
    const v = JSON.parse(text);
    if (depthOf(v) > MAX_DEPTH) throw new RosJsonError(`nested deeper than ${MAX_DEPTH}`);
    return v;
  } catch (e) {
    if (e instanceof RosJsonError) throw e;
    if (!/NaN|Infinity/.test(text))
      throw new RosJsonError(`not JSON: ${/** @type {Error} */ (e).message}`);
  }
  return new Reader(text).document();
}

/**
 * @param {unknown} v
 * @returns {number}
 */
function depthOf(v) {
  // Iterative, so a hostile document cannot overflow the stack.
  let max = 0;
  /** @type {[unknown, number][]} */
  const stack = [[v, 1]];
  while (stack.length) {
    const [x, d] = /** @type {[unknown, number]} */ (stack.pop());
    if (x === null || typeof x !== "object") continue;
    if (d > max) max = d;
    if (max > MAX_DEPTH) return max;
    for (const c of Array.isArray(x) ? x : Object.values(x)) stack.push([c, d + 1]);
  }
  return max;
}

const NUMBER = /-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+-]?\d+)?/y;

class Reader {
  /** @param {string} s */
  constructor(s) {
    this.s = s;
    this.i = 0;
    this.depth = 0;
  }
  /** @param {string} what */
  fail(what) {
    return new RosJsonError(`not JSON at ${this.i}: ${what}`);
  }
  ws() {
    const s = this.s;
    while (this.i < s.length) {
      const c = s.charCodeAt(this.i);
      if (c === 32 || c === 9 || c === 10 || c === 13) this.i++;
      else break;
    }
  }
  document() {
    const v = this.value();
    this.ws();
    if (this.i !== this.s.length) throw this.fail("trailing characters");
    return v;
  }
  /** @returns {unknown} */
  value() {
    this.ws();
    const c = this.s[this.i];
    if (c === "{") return this.object();
    if (c === "[") return this.array();
    if (c === '"') return this.string();
    for (const [word, v] of /** @type {[string, unknown][]} */ ([
      ["true", true],
      ["false", false],
      ["null", null],
      ["NaN", NaN],
      ["Infinity", Infinity],
      ["-Infinity", -Infinity],
    ]))
      if (this.s.startsWith(word, this.i)) {
        this.i += word.length;
        return v;
      }
    NUMBER.lastIndex = this.i;
    const m = NUMBER.exec(this.s);
    if (!m) throw this.fail("unexpected character");
    this.i += m[0].length;
    return Number(m[0]);
  }
  enter() {
    if (++this.depth > MAX_DEPTH) throw this.fail(`nested deeper than ${MAX_DEPTH}`);
  }
  object() {
    this.enter();
    this.i++;
    /** @type {Record<string, unknown>} */
    const o = Object.create(null);
    this.ws();
    if (this.s[this.i] === "}") {
      this.i++;
      this.depth--;
      return { ...o };
    }
    for (;;) {
      this.ws();
      if (this.s[this.i] !== '"') throw this.fail("expected a key");
      const k = this.string();
      this.ws();
      if (this.s[this.i++] !== ":") throw this.fail("expected ':'");
      o[k] = this.value();
      this.ws();
      const c = this.s[this.i++];
      if (c === "}") break;
      if (c !== ",") throw this.fail("expected ',' or '}'");
    }
    this.depth--;
    return { ...o };
  }
  array() {
    this.enter();
    this.i++;
    /** @type {unknown[]} */
    const a = [];
    this.ws();
    if (this.s[this.i] === "]") {
      this.i++;
      this.depth--;
      return a;
    }
    for (;;) {
      a.push(this.value());
      this.ws();
      const c = this.s[this.i++];
      if (c === "]") break;
      if (c !== ",") throw this.fail("expected ',' or ']'");
    }
    this.depth--;
    return a;
  }
  string() {
    // Find the closing quote, then let JSON.parse decode the escapes of just this string.
    const start = this.i;
    let j = start + 1;
    const s = this.s;
    while (j < s.length) {
      const c = s[j];
      if (c === "\\") j += 2;
      else if (c === '"') break;
      else j++;
    }
    if (j >= s.length) throw this.fail("unterminated string");
    this.i = j + 1;
    try {
      return /** @type {string} */ (JSON.parse(s.slice(start, j + 1)));
    } catch {
      throw this.fail("bad string");
    }
  }
}
