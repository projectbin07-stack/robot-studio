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
 * @property {boolean} equal canonical forms are identical
 * @property {boolean} byteIdentical the texts are identical
 * @property {number} lines number of canonical lines (elements)
 * @property {{ line: number, expected: string, actual: string } | null} firstDifference
 */

/**
 * @param {string} expected
 * @param {string} actual
 * @returns {Comparison}
 */
export function compareXml(expected, actual) {
  const e = canonicalLines(expected);
  const a = canonicalLines(actual);
  const n = Math.max(e.length, a.length);
  for (let k = 0; k < n; k++) {
    if (e[k] !== a[k])
      return {
        equal: false,
        byteIdentical: false,
        lines: e.length,
        firstDifference: { line: k + 1, expected: e[k] ?? "(end)", actual: a[k] ?? "(end)" },
      };
  }
  return {
    equal: true,
    byteIdentical: expected === actual,
    lines: e.length,
    firstDifference: null,
  };
}
