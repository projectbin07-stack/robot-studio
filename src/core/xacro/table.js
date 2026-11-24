// Scoped symbol tables with xacro's lazy evaluation (xacro/__init__.py,
// class Table): a property's value text is stored as written and evaluated
// on first use, in the scope it was defined in; literals ("0.1", "3",
// "true") become numbers or booleans when stored. Macro calls and
// namespaced includes open child scopes; lookups walk up to the globals.

import { PyError } from "./pyvalue.js";
import { NOT_FOUND } from "./pyeval.js";

/** @typedef {import("./pyvalue.js").PyValue} PyValue */

/**
 * Python int()/float()/bool conversion of a property string, as
 * Table._eval_literal does. Strings in single quotes lose the quotes;
 * strings containing "_" are kept as text.
 * @param {PyValue} value
 * @returns {PyValue}
 */
export function evalLiteral(value) {
  if (typeof value !== "string") return value;
  if (value.length >= 2 && value[0] === "'" && value[value.length - 1] === "'")
    return value.slice(1, -1);
  if (value.includes("_")) return value;
  const t = value.trim();
  if (/^[+-]?\d+$/.test(t)) return BigInt(t);
  if (/^[+-]?(\d+\.?\d*(e[+-]?\d+)?|\.\d+(e[+-]?\d+)?)$/i.test(t)) return Number(t);
  const special = /^([+-]?)(inf|infinity|nan)$/i.exec(t);
  if (special)
    return special[2].toLowerCase() === "nan" ? NaN : special[1] === "-" ? -Infinity : Infinity;
  if (value === "true" || value === "True") return true;
  if (value === "false" || value === "False") return false;
  return value;
}

/**
 * @typedef {object} TableHooks
 * @property {(text: string, table: Table) => PyValue} evalText evaluate ${} and $() in a string
 * @property {(msg: string) => void} warn
 */

export class Table {
  /**
   * @param {Table | null} parent
   * @param {Map<string, PyValue>} globals
   * @param {TableHooks} hooks
   */
  constructor(parent, globals, hooks) {
    this.parent = parent;
    this.globals = globals;
    this.hooks = hooks;
    /** @type {Map<string, PyValue>} */
    this.store = new Map();
    /** @type {Set<string>} */
    this.unevaluated = new Set();
    /** @type {string[]} */
    this.recursive = [];
  }

  /**
   * @param {string} key
   * @returns {PyValue | typeof NOT_FOUND}
   */
  lookup(key) {
    if (this.store.has(key)) return this.resolve(key);
    if (this.parent) return this.parent.lookup(key);
    const g = this.globals.get(key);
    return g === undefined ? NOT_FOUND : g;
  }

  /** @param {string} key */
  resolve(key) {
    if (this.unevaluated.has(key)) {
      if (this.recursive.includes(key))
        throw new PyError(
          "XacroError",
          `circular variable definition: ${[...this.recursive, key].join(" -> ")}\n` +
            'Consider disabling lazy evaluation via lazy_eval="false"',
        );
      this.recursive.push(key);
      try {
        const raw = /** @type {string} */ (this.store.get(key));
        this.store.set(key, evalLiteral(this.hooks.evalText(raw, this)));
        this.unevaluated.delete(key);
      } finally {
        this.recursive.pop();
      }
    }
    return /** @type {PyValue} */ (this.store.get(key));
  }

  /** True when the key is defined anywhere up the chain (nothing is evaluated). @param {string} key */
  has(key) {
    /** @type {Table | null} */
    let t = this;
    while (t) {
      if (t.store.has(key)) return true;
      t = t.parent;
    }
    return this.globals.has(key);
  }

  /**
   * Store a value in this scope.
   * @param {string} key
   * @param {PyValue} value
   * @param {boolean} unevaluated keep a string for lazy evaluation
   */
  set(key, value, unevaluated) {
    if (this.globals.has(key)) this.hooks.warn(`redefining global symbol: ${key}`);
    const v = evalLiteral(value);
    this.store.set(key, v);
    if (unevaluated && typeof v === "string") this.unevaluated.add(key);
    else this.unevaluated.delete(key);
  }

  /** Remove a key from this scope and every parent scope. @param {string} key */
  remove(key) {
    /** @type {Table | null} */
    let t = this;
    while (t) {
      t.store.delete(key);
      t.unevaluated.delete(key);
      t = t.parent;
    }
    if (this.globals.has(key)) this.hooks.warn(`Cannot remove global symbol: ${key}`);
  }

  /** The document-level scope. */
  top() {
    /** @type {Table} */
    let t = this;
    while (t.parent) t = t.parent;
    return t;
  }
}

/** A namespace created by <xacro:include ns="...">: also reachable as ns.name. */
export class NameSpace extends Table {
  pyName = "NameSpace";
  /** @param {string} name @returns {PyValue} */
  getAttr(name) {
    const v = this.lookup(name);
    if (v === NOT_FOUND) throw new PyError("NameError", `name '${name}' is not defined`);
    return v;
  }
}
