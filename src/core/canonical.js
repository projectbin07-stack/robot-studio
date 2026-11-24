// Canonical form of an XML document for comparing Robot Studio's xacro
// output with the official xacro's: comments, the XML declaration,
// xmlns declarations and attribute order are ignored, character data is
// trimmed, whitespace-only text between elements is dropped. Everything
// else (element names and order, attribute names and exact values, text)
// must match.

import { parseXml } from "../../vendor/urdf-inspector/src/core/xml.js";

/** @typedef {import("../../vendor/urdf-inspector/src/core/xml.js").XmlElement} XmlElement */

/**
 * One line per element: depth, name, sorted attributes, trimmed text.
 * @param {string} xml
 * @returns {string[]}
 */
export function canonicalLines(xml) {
  /** @type {string[]} */
  const out = [];
  /** @param {XmlElement} el @param {number} depth */
  const walk = (el, depth) => {
    const attrs = Object.keys(el.attrs)
      .filter((k) => k !== "xmlns" && !k.startsWith("xmlns:"))
      .sort()
      .map((k) => ` ${k}=${JSON.stringify(el.attrs[k])}`)
      .join("");
    const text = el.text.trim();
    out.push(`${"  ".repeat(depth)}<${el.name}${attrs}>${text ? ` ${JSON.stringify(text)}` : ""}`);
    for (const c of el.children) walk(c, depth + 1);
  };
  walk(parseXml(xml), 0);
  return out;
}

/**
 * @typedef {object} Comparison
 * @property {boolean} equal canonical forms match (numbers may differ in the last bits, see maxUlps)
 * @property {boolean} exact canonical forms are identical
 * @property {boolean} byteIdentical the texts are identical
 * @property {number} maxUlps largest difference between two numbers, in units in the last place
 * @property {number} lines number of canonical lines (elements)
 * @property {{ line: number, expected: string, actual: string } | null} firstDifference
 */

/** Differences up to this many units in the last place count as float rounding. */
export const ULP_TOLERANCE = 4;

const NUMBER = /-?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?/gi;

/** Distance between two doubles in units in the last place. @param {number} a @param {number} b */
export function ulps(a, b) {
  if (a === b) return 0;
  if (!Number.isFinite(a) || !Number.isFinite(b) || Math.sign(a) !== Math.sign(b)) return Infinity;
  const buf = new DataView(new ArrayBuffer(16));
  buf.setFloat64(0, Math.abs(a));
  buf.setFloat64(8, Math.abs(b));
  const d = buf.getBigUint64(0) - buf.getBigUint64(8);
  return Number(d < 0n ? -d : d);
}

/**
 * Compare two canonical lines allowing numbers to differ by a few ulps.
 * Returns the largest ulp distance, or Infinity when anything else differs.
 * @param {string} e
 * @param {string} a
 */
function lineUlps(e, a) {
  if (e.replace(NUMBER, "#") !== a.replace(NUMBER, "#")) return Infinity;
  const ne = e.match(NUMBER) ?? [];
  const na = a.match(NUMBER) ?? [];
  let worst = 0;
  for (let k = 0; k < ne.length; k++) worst = Math.max(worst, ulps(Number(ne[k]), Number(na[k])));
  return worst;
}

/**
 * Compare the official xacro output with Robot Studio's. Element structure,
 * names, attributes and text must match exactly, except that a number may
 * differ by up to ULP_TOLERANCE units in the last place: the C library
 * behind Python's math.sin/cos and the JavaScript engine's may round the
 * last bit differently, and that is reported in maxUlps, not hidden.
 * @param {string} expected
 * @param {string} actual
 * @returns {Comparison}
 */
export function compareXml(expected, actual) {
  const e = canonicalLines(expected);
  const a = canonicalLines(actual);
  const n = Math.max(e.length, a.length);
  let maxUlps = 0;
  for (let k = 0; k < n; k++) {
    if (e[k] === a[k]) continue;
    const u = e[k] === undefined || a[k] === undefined ? Infinity : lineUlps(e[k], a[k]);
    if (u > ULP_TOLERANCE)
      return {
        equal: false,
        exact: false,
        byteIdentical: false,
        maxUlps: u,
        lines: e.length,
        firstDifference: { line: k + 1, expected: e[k] ?? "(end)", actual: a[k] ?? "(end)" },
      };
    maxUlps = Math.max(maxUlps, u);
  }
  return {
    equal: true,
    exact: maxUlps === 0,
    byteIdentical: expected === actual,
    maxUlps,
    lines: e.length,
    firstDifference: null,
  };
}
