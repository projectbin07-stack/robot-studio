// Runs every case of tests/corpus/cases.json through Robot Studio's xacro
// and compares the result with the golden output of the official xacro.
// Shared by tests/xacro-golden.test.js and scripts/golden-report.js.

import { existsSync, readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { compareXml } from "../src/core/canonical.js";
import { expandXacro, serializeXml, XacroError } from "../src/core/xacro/index.js";
import { abs, NodeFs } from "../src/node/nodefs.js";

export const ROOT = abs(fileURLToPath(new URL("..", import.meta.url)));
const GOLDEN = `${ROOT}/tests/corpus/golden`;

/**
 * @typedef {object} Case
 * @property {string} id
 * @property {string} entry path relative to the repository root
 * @property {Record<string, string>} [packages]
 * @property {Record<string, string>} [args]
 * @property {string} [unsupported] reason, when the official xacro accepts the input and Robot Studio deliberately rejects it
 */

/** @returns {Case[]} */
export const loadCases = () => JSON.parse(readFileSync(`${ROOT}/tests/corpus/cases.json`, "utf8"));

/**
 * @typedef {object} CaseResult
 * @property {Case} case
 * @property {"urdf" | "error"} expected
 * @property {"urdf" | "error"} actual
 * @property {boolean} pass
 * @property {boolean} byteIdentical
 * @property {number} maxUlps largest float difference in units in the last place
 * @property {number} elements
 * @property {string} detail
 */

/**
 * @param {Case} c
 * @returns {CaseResult}
 */
export function runCase(c) {
  const packages = Object.fromEntries(
    Object.entries(c.packages ?? {}).map(([k, v]) => [k, `${ROOT}/${v}`]),
  );
  const urdfPath = `${GOLDEN}/${c.id}.urdf`;
  const expected = existsSync(urdfPath) ? "urdf" : "error";
  const golden = readFileSync(
    expected === "urdf" ? urdfPath : `${GOLDEN}/${c.id}.error.txt`,
    "utf8",
  );
  /** @type {"urdf" | "error"} */
  let actual;
  let out = "";
  try {
    const r = expandXacro(`${ROOT}/${c.entry}`, new NodeFs(packages), { args: c.args ?? {} });
    out = serializeXml(r.root).split(ROOT).join("@ROOT@");
    actual = "urdf";
  } catch (e) {
    if (!(e instanceof XacroError)) throw e;
    out = e.message;
    actual = "error";
  }
  /** @type {CaseResult} */
  const res = {
    case: c,
    expected,
    actual,
    pass: false,
    byteIdentical: false,
    maxUlps: 0,
    elements: 0,
    detail: "",
  };
  if (c.unsupported) {
    res.pass = actual === "error" && out.includes(c.unsupported);
    res.detail = res.pass
      ? `rejected as documented: ${c.unsupported}`
      : `expected rejection: ${out}`;
    return res;
  }
  if (expected === "error") {
    res.pass = actual === "error";
    res.detail = res.pass
      ? `both fail; official: ${golden.trim().split("\n")[0]}`
      : "official xacro fails, Robot Studio does not";
    return res;
  }
  if (actual === "error") {
    res.detail = `Robot Studio fails: ${out}`;
    return res;
  }
  const cmp = compareXml(golden, out);
  res.pass = cmp.equal;
  res.byteIdentical = cmp.byteIdentical;
  res.maxUlps = cmp.maxUlps;
  res.elements = cmp.lines;
  res.detail = cmp.firstDifference
    ? `element ${cmp.firstDifference.line}:\n  official: ${cmp.firstDifference.expected}\n  ours:     ${cmp.firstDifference.actual}`
    : "";
  return res;
}
