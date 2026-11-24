// Shared test helpers.

import { VirtualFs } from "../src/core/vfs.js";
import { evalExpression } from "../src/core/xacro/pyeval.js";
import { makeBuiltins } from "../src/core/xacro/pyeval.js";
import { NOT_FOUND } from "../src/core/xacro/pyeval.js";
import { pyStr } from "../src/core/xacro/pyvalue.js";
import { expandXacro, serializeXml } from "../src/core/xacro/index.js";

/**
 * In-memory file system from a map of path -> text.
 * @param {Record<string, string>} files
 */
export function memFs(files) {
  const fs = new VirtualFs();
  for (const [p, t] of Object.entries(files)) fs.addText(p, t);
  return fs;
}

const { builtins } = makeBuiltins({ warn: () => {} });

/**
 * Evaluate an expression with xacro's globals plus `vars`, and return
 * Python's str() of the result.
 * @param {string} src
 * @param {Record<string, import("../src/core/xacro/pyvalue.js").PyValue>} [vars]
 */
export function py(src, vars = {}) {
  const scope = {
    /** @param {string} n */
    lookup: (n) => (n in vars ? vars[n] : (builtins.get(n) ?? NOT_FOUND)),
  };
  return pyStr(evalExpression(src, scope));
}

/** @param {string} body */
export const doc = (body) =>
  `<?xml version="1.0"?>\n<robot name="t" xmlns:xacro="http://www.ros.org/wiki/xacro">\n${body}\n</robot>\n`;

/**
 * Expand a single in-memory document (plus optional extra files) and
 * return the serialised output without the XML declaration.
 * @param {string} body
 * @param {{ args?: Record<string, string>, env?: Record<string, string>, files?: Record<string, string> }} [opts]
 */
export function expand(body, opts = {}) {
  const fs = memFs({ "/pkg/main.xacro": doc(body), ...(opts.files ?? {}) });
  const r = expandXacro("/pkg/main.xacro", fs, { args: opts.args, env: opts.env });
  return serializeXml(r.root).replace(/^<\?xml[^>]*>\n/, "");
}
