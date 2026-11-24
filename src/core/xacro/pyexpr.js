// Tokenizer and parser for the subset of Python expressions that xacro
// files use inside ${...}. The result is a plain AST evaluated by
// pyeval.js; nothing is ever passed to eval() or Function().
//
// Grammar (precedence from low to high, after the Python reference,
// https://docs.python.org/3/reference/expressions.html):
//   exprlist   := test ("," test)* [","]                 tuple when a comma is present
//   test       := or_test ["if" or_test "else" test]
//   or_test    := and_test ("or" and_test)*
//   and_test   := not_test ("and" not_test)*
//   not_test   := "not" not_test | comparison
//   comparison := bitor (compop bitor)*                   < > == != <= >= in, not in, is, is not
//   bitor      := bitxor ("|" bitxor)* ; bitxor := bitand ("^" bitand)* ; bitand := shift ("&" shift)*
//   shift      := arith (("<<" | ">>") arith)*
//   arith      := term (("+" | "-") term)*
//   term       := factor (("*" | "/" | "//" | "%") factor)*
//   factor     := ("+" | "-" | "~") factor | power
//   power      := primary ["**" factor]
//   primary    := atom (call | subscript | "." NAME)*
//   atom       := NAME | NUMBER | STRING+ | None | True | False
//               | "(" [exprlist] ")" | "[" [test ("," test)* [","]] "]" | "{" [dict items] "}"
// Everything else (lambda, comprehensions, walrus, starred arguments,
// f-strings, bytes, complex numbers, ellipsis, statements) is rejected with
// a SyntaxError naming the construct.

import { PyError } from "./pyvalue.js";

/**
 * @typedef {{ t: "name" | "num" | "str" | "op" | "kw" | "end", v: string, pos: number, value?: import("./pyvalue.js").PyValue }} Token
 */

/**
 * @typedef {{ k: "const", v: import("./pyvalue.js").PyValue }
 *   | { k: "name", id: string }
 *   | { k: "tuple", items: Node[] }
 *   | { k: "list", items: Node[] }
 *   | { k: "dict", keys: Node[], values: Node[] }
 *   | { k: "attr", obj: Node, name: string }
 *   | { k: "sub", obj: Node, index: Node }
 *   | { k: "slice", obj: Node, lo: Node | null, hi: Node | null, step: Node | null }
 *   | { k: "call", fn: Node, args: Node[], kwargs: [string, Node][] }
 *   | { k: "unary", op: string, x: Node }
 *   | { k: "not", x: Node }
 *   | { k: "bin", op: string, a: Node, b: Node }
 *   | { k: "cmp", first: Node, rest: [string, Node][] }
 *   | { k: "bool", op: "and" | "or", items: Node[] }
 *   | { k: "if", test: Node, body: Node, orelse: Node }} Node
 */

const KEYWORDS = new Set([
  "and",
  "or",
  "not",
  "in",
  "is",
  "if",
  "else",
  "None",
  "True",
  "False",
  // Python keywords that are valid Python but not part of this subset:
  "lambda",
  "for",
  "async",
  "await",
  "yield",
  "import",
  "from",
  "def",
  "class",
  "return",
  "del",
  "global",
  "nonlocal",
  "pass",
  "raise",
  "try",
  "except",
  "finally",
  "while",
  "with",
  "as",
  "assert",
  "break",
  "continue",
  "elif",
]);

const OPERATORS = [
  "**",
  "//",
  "<<",
  ">>",
  "<=",
  ">=",
  "==",
  "!=",
  ":=",
  "->",
  "+",
  "-",
  "*",
  "/",
  "%",
  "@",
  "&",
  "|",
  "^",
  "~",
  "<",
  ">",
  "(",
  ")",
  "[",
  "]",
  "{",
  "}",
  ",",
  ":",
  ".",
  ";",
  "=",
];

/**
 * @param {string} msg
 * @returns {never}
 */
function syntax(msg) {
  throw new PyError("SyntaxError", msg);
}

/**
 * Decode the body of a non-raw string literal.
 * @param {string} body
 */
function unescape(body) {
  return body.replace(
    /\\(x[0-9A-Fa-f]{2}|u[0-9A-Fa-f]{4}|U[0-9A-Fa-f]{8}|[0-7]{1,3}|N\{[^}]*\}|.|\n)/g,
    (m, e) => {
      /** @type {Record<string, string>} */
      const simple = {
        "\\": "\\",
        "'": "'",
        '"': '"',
        n: "\n",
        t: "\t",
        r: "\r",
        a: "\x07",
        b: "\b",
        f: "\f",
        v: "\v",
        "\n": "",
      };
      if (e in simple) return simple[e];
      if (e[0] === "x" || e[0] === "u" || e[0] === "U")
        return String.fromCodePoint(parseInt(e.slice(1), 16));
      if (/^[0-7]/.test(e)) return String.fromCodePoint(parseInt(e, 8));
      if (e[0] === "N") syntax("\\N{...} escapes are not supported");
      return m;
    },
  );
}

/**
 * @param {string} src
 * @returns {Token[]}
 */
export function tokenize(src) {
  /** @type {Token[]} */
  const out = [];
  let i = 0;
  while (i < src.length) {
    const c = src[i];
    if (/\s/.test(c)) {
      i++;
      continue;
    }
    if (c === "#") syntax("comments are not allowed in expressions");
    if (c === "\\") syntax("line continuation is not allowed in expressions");
    const strStart = /^([rRuUbBfF]{0,2})(['"])/.exec(src.slice(i, i + 3));
    if (strStart && (strStart[1] === "" || /^[A-Za-z]+$/.test(strStart[1]))) {
      const prefix = strStart[1].toLowerCase();
      if (prefix.includes("f")) syntax("f-strings are not supported");
      if (prefix.includes("b")) syntax("bytes literals are not supported");
      if (prefix.length === 2 && prefix !== "ur" && prefix !== "ru")
        syntax(`bad string prefix ${prefix}`);
      const q = strStart[2];
      const start = i;
      i += strStart[0].length;
      if (src.startsWith(q + q, i)) syntax("triple-quoted strings are not supported");
      let body = "";
      for (;;) {
        if (i >= src.length || src[i] === "\n") syntax("unterminated string literal");
        if (src[i] === "\\") {
          body += src.slice(i, i + 2);
          i += 2;
          continue;
        }
        if (src[i] === q) break;
        body += src[i++];
      }
      i++;
      const raw = prefix.includes("r");
      out.push({
        t: "str",
        v: src.slice(start, i),
        pos: start,
        value: raw ? body : unescape(body),
      });
      continue;
    }
    const numM =
      /^(0[xX](?:_?[0-9A-Fa-f])+|0[oO](?:_?[0-7])+|0[bB](?:_?[01])+|(?:\d(?:_?\d)*\.(?:\d(?:_?\d)*)?|\.\d(?:_?\d)*|\d(?:_?\d)*)(?:[eE][+-]?\d(?:_?\d)*)?)([jJ]?)/.exec(
        src.slice(i),
      );
    if (numM && /[0-9.]/.test(c) && !(c === "." && !/\d/.test(src[i + 1] ?? ""))) {
      if (numM[2]) syntax("complex numbers are not supported");
      const text = numM[1].replace(/_/g, "");
      if (/[A-Za-z0-9_]/.test(src[i + numM[0].length] ?? ""))
        syntax(`invalid number '${src.slice(i).split(/\s/)[0]}'`);
      /** @type {import("./pyvalue.js").PyValue} */
      let value;
      if (/^0[xXoObB]/.test(text)) value = BigInt(text);
      else if (/^\d+$/.test(text)) {
        if (/^0+[1-9]/.test(text))
          syntax("leading zeros in decimal integer literals are not permitted");
        value = BigInt(text);
      } else value = Number(text);
      out.push({ t: "num", v: numM[1], pos: i, value });
      i += numM[0].length;
      continue;
    }
    const nameM = /^[A-Za-z_][A-Za-z0-9_]*/.exec(src.slice(i));
    if (nameM) {
      const w = nameM[0];
      out.push({ t: KEYWORDS.has(w) ? "kw" : "name", v: w, pos: i });
      i += w.length;
      continue;
    }
    if (src.startsWith("...", i)) syntax("'...' is not supported");
    const op = OPERATORS.find((o) => src.startsWith(o, i));
    if (op === undefined) throw new PyError("SyntaxError", `unexpected character '${c}'`);
    out.push({ t: "op", v: op, pos: i });
    i += op.length;
  }
  out.push({ t: "end", v: "", pos: src.length });
  return out;
}

/** Constructs named when rejected. @type {Record<string, string>} */
const REJECTED = {
  lambda: "lambda expressions",
  for: "comprehensions and generator expressions",
  async: "async comprehensions",
  await: "await",
  yield: "yield",
  ":=": "assignment expressions (:=)",
  "@": "the @ operator",
  ";": "statements",
  "=": "assignment",
};

/**
 * Parse an expression.
 * @param {string} src
 * @returns {Node}
 */
export function parseExpression(src) {
  const toks = tokenize(src);
  let p = 0;
  const peek = () => toks[p];
  /** @param {string} v */
  const isOp = (v) => toks[p].t === "op" && toks[p].v === v;
  /** @param {string} v */
  const isKw = (v) => toks[p].t === "kw" && toks[p].v === v;
  /** @param {Token} t @returns {never} */
  const unexpected = (t) => {
    const why = REJECTED[t.v];
    if (why && (t.t === "kw" || t.t === "op"))
      syntax(`${why} are not supported in xacro expressions`);
    if (t.t === "kw" && !["and", "or", "not", "in", "is", "if", "else"].includes(t.v))
      syntax(`'${t.v}' is not supported in xacro expressions`);
    throw new PyError(
      "SyntaxError",
      t.t === "end" ? "unexpected end of expression" : `unexpected '${t.v}' at offset ${t.pos}`,
    );
  };
  /** @param {string} v */
  const expectOp = (v) => {
    if (!isOp(v)) unexpected(peek());
    p++;
  };

  /** @returns {Node} */
  const exprlist = () => {
    const first = test();
    if (!isOp(",")) return first;
    const items = [first];
    while (isOp(",")) {
      p++;
      if (atEndOfList()) break;
      items.push(test());
    }
    return { k: "tuple", items };
  };
  const atEndOfList = () => peek().t === "end" || isOp(")") || isOp("]") || isOp("}");

  /** @returns {Node} */
  const test = () => {
    if (isKw("lambda")) unexpected(peek());
    const body = orTest();
    if (isKw("if")) {
      p++;
      const cond = orTest();
      if (!isKw("else")) syntax("expected 'else' in conditional expression");
      p++;
      const orelse = test();
      return { k: "if", test: cond, body, orelse };
    }
    return body;
  };
  /** @returns {Node} */
  const orTest = () => {
    const items = [andTest()];
    while (isKw("or")) {
      p++;
      items.push(andTest());
    }
    return items.length === 1 ? items[0] : { k: "bool", op: "or", items };
  };
  /** @returns {Node} */
  const andTest = () => {
    const items = [notTest()];
    while (isKw("and")) {
      p++;
      items.push(notTest());
    }
    return items.length === 1 ? items[0] : { k: "bool", op: "and", items };
  };
  /** @returns {Node} */
  const notTest = () => {
    if (isKw("not")) {
      p++;
      return { k: "not", x: notTest() };
    }
    return comparison();
  };
  /** @returns {Node} */
  const comparison = () => {
    const first = bitor();
    /** @type {[string, Node][]} */
    const rest = [];
    for (;;) {
      const t = peek();
      let op = null;
      if (t.t === "op" && ["<", ">", "==", "!=", "<=", ">="].includes(t.v)) {
        op = t.v;
        p++;
      } else if (isKw("in")) {
        op = "in";
        p++;
      } else if (isKw("not") && toks[p + 1].t === "kw" && toks[p + 1].v === "in") {
        op = "not in";
        p += 2;
      } else if (isKw("is")) {
        p++;
        if (isKw("not")) {
          p++;
          op = "is not";
        } else op = "is";
      }
      if (op === null) break;
      rest.push([op, bitor()]);
    }
    return rest.length === 0 ? first : { k: "cmp", first, rest };
  };
  /**
   * @param {string[]} ops
   * @param {() => Node} next
   * @returns {() => Node}
   */
  const leftAssoc = (ops, next) => () => {
    let a = next();
    while (peek().t === "op" && ops.includes(peek().v)) {
      const op = toks[p++].v;
      a = { k: "bin", op, a, b: next() };
    }
    if (isOp("@")) unexpected(peek());
    return a;
  };
  /** @returns {Node} */
  const factor = () => {
    if (isOp("-") || isOp("+") || isOp("~")) {
      const op = toks[p++].v;
      return { k: "unary", op, x: factor() };
    }
    return power();
  };
  const term = leftAssoc(["*", "/", "//", "%"], factor);
  const arith = leftAssoc(["+", "-"], term);
  const shift = leftAssoc(["<<", ">>"], arith);
  const bitand = leftAssoc(["&"], shift);
  const bitxor = leftAssoc(["^"], bitand);
  const bitor = leftAssoc(["|"], bitxor);

  /** @returns {Node} */
  const power = () => {
    const base = primary();
    if (isOp("**")) {
      p++;
      return { k: "bin", op: "**", a: base, b: factor() };
    }
    return base;
  };
  /** @returns {Node} */
  const primary = () => {
    let x = atom();
    for (;;) {
      if (isOp("(")) {
        p++;
        /** @type {Node[]} */
        const args = [];
        /** @type {[string, Node][]} */
        const kwargs = [];
        while (!isOp(")")) {
          if (isOp("*") || isOp("**"))
            syntax("starred arguments are not supported in xacro expressions");
          if (peek().t === "name" && toks[p + 1].t === "op" && toks[p + 1].v === "=") {
            const name = toks[p].v;
            p += 2;
            kwargs.push([name, test()]);
          } else {
            if (kwargs.length > 0) syntax("positional argument follows keyword argument");
            args.push(test());
            if (isKw("for")) unexpected(peek());
          }
          if (!isOp(",")) break;
          p++;
        }
        expectOp(")");
        x = { k: "call", fn: x, args, kwargs };
      } else if (isOp("[")) {
        p++;
        /** @type {(Node | null)[]} */
        const parts = [null];
        let colons = 0;
        while (!isOp("]")) {
          if (isOp(":")) {
            p++;
            colons++;
            if (colons > 2) unexpected(toks[p - 1]);
            parts.push(null);
            continue;
          }
          if (parts[parts.length - 1] !== null) unexpected(peek());
          parts[parts.length - 1] = colons === 0 ? exprlist() : test();
        }
        expectOp("]");
        if (colons === 0) {
          if (parts[0] === null) syntax("empty subscript");
          x = { k: "sub", obj: x, index: /** @type {Node} */ (parts[0]) };
        } else
          x = { k: "slice", obj: x, lo: parts[0], hi: parts[1] ?? null, step: parts[2] ?? null };
      } else if (isOp(".")) {
        p++;
        const t = peek();
        if (t.t !== "name") unexpected(t);
        p++;
        x = { k: "attr", obj: x, name: t.v };
      } else return x;
    }
  };
  /** @returns {Node} */
  const atom = () => {
    const t = peek();
    if (t.t === "num") {
      p++;
      return { k: "const", v: /** @type {import("./pyvalue.js").PyValue} */ (t.value) };
    }
    if (t.t === "str") {
      let s = "";
      while (peek().t === "str") s += /** @type {string} */ (toks[p++].value);
      return { k: "const", v: s };
    }
    if (t.t === "name") {
      p++;
      return { k: "name", id: t.v };
    }
    if (t.t === "kw") {
      if (t.v === "None" || t.v === "True" || t.v === "False") {
        p++;
        return { k: "const", v: t.v === "None" ? null : t.v === "True" };
      }
      return unexpected(t);
    }
    if (isOp("(")) {
      p++;
      if (isOp(")")) {
        p++;
        return { k: "tuple", items: [] };
      }
      const first = test();
      if (isKw("for")) unexpected(peek());
      if (isOp(")")) {
        p++;
        return first;
      }
      /** @type {Node[]} */
      const items = [first];
      while (isOp(",")) {
        p++;
        if (isOp(")")) break;
        items.push(test());
      }
      expectOp(")");
      return { k: "tuple", items };
    }
    if (isOp("[")) {
      p++;
      /** @type {Node[]} */
      const items = [];
      while (!isOp("]")) {
        items.push(test());
        if (isKw("for")) unexpected(peek());
        if (!isOp(",")) break;
        p++;
      }
      expectOp("]");
      return { k: "list", items };
    }
    if (isOp("{")) {
      p++;
      /** @type {Node[]} */
      const keys = [];
      /** @type {Node[]} */
      const values = [];
      while (!isOp("}")) {
        if (isOp("**")) syntax("dict unpacking is not supported in xacro expressions");
        keys.push(test());
        if (!isOp(":")) syntax("set literals are not supported in xacro expressions");
        p++;
        values.push(test());
        if (isKw("for")) unexpected(peek());
        if (!isOp(",")) break;
        p++;
      }
      expectOp("}");
      return { k: "dict", keys, values };
    }
    return unexpected(t);
  };

  if (toks[0].t === "end") syntax("empty expression");
  const tree = exprlist();
  if (peek().t !== "end") unexpected(peek());
  return tree;
}
