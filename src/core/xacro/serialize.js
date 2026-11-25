// Output of an expanded document: XML text in the layout xacro writes
// (minidom toprettyxml with xacro's fixed_writexml: two-space indent,
// attributes sorted by name, whitespace-only text dropped between
// elements, a lone text child kept inline), and conversion to the element
// tree the vendored urdf-inspector reads.

/** @typedef {import("./dom.js").XElement} XElement */
/** @typedef {import("./dom.js").XNode} XNode */
/** @typedef {import("../../../vendor/urdf-inspector/src/core/xml.js").XmlElement} XmlElement */

/** @param {string} s */
const escText = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
/** @param {string} s */
const escAttr = (s) =>
  escText(s)
    .replace(/"/g, "&quot;")
    .replace(/\r/g, "&#13;")
    .replace(/\n/g, "&#10;")
    .replace(/\t/g, "&#9;");

/**
 * @param {XNode} node
 * @param {string} indent
 * @param {string[]} out
 */
function write(node, indent, out) {
  if (node.type === "text") {
    out.push(`${indent}${escText(node.value)}\n`);
    return;
  }
  if (node.type === "cdata") {
    out.push(`<![CDATA[${node.value}]]>`);
    return;
  }
  if (node.type === "comment") {
    out.push(`${indent}<!--${node.value}-->\n`);
    return;
  }
  let s = `${indent}<${node.name}`;
  for (const k of Object.keys(node.attrs).sort()) s += ` ${k}="${escAttr(node.attrs[k])}"`;
  const kids = node.children;
  if (kids.length === 0) {
    out.push(`${s}/>\n`);
    return;
  }
  if (kids.length === 1 && kids[0].type === "text") {
    out.push(`${s}>${escText(kids[0].value)}</${node.name}>\n`);
    return;
  }
  out.push(`${s}>\n`);
  for (const c of kids) {
    if (c.type === "text" && c.value.trim() === "") continue;
    write(c, `${indent}  `, out);
  }
  out.push(`${indent}</${node.name}>\n`);
}

/**
 * Serialise a document element as xacro does.
 * @param {XElement} root
 * @returns {string}
 */
export function serializeXml(root) {
  /** @type {string[]} */
  const out = ['<?xml version="1.0" ?>\n'];
  for (const c of root.prolog ?? []) write(c, "", out);
  write(root, "", out);
  return out.join("");
}

/**
 * Convert to urdf-inspector's XmlElement tree (text concatenated, comments
 * dropped). Every element's `source` is the file it came from.
 * @param {XElement} root
 * @returns {XmlElement}
 */
export function toXmlElement(root) {
  /** @param {XElement} el @returns {XmlElement} */
  const conv = (el) => {
    /** @type {XmlElement[]} */
    const children = [];
    let text = "";
    for (const c of el.children) {
      if (c.type === "element") children.push(conv(c));
      else if (c.type === "text" || c.type === "cdata") text += c.value;
    }
    return {
      name: el.name,
      attrs: { ...el.attrs },
      children,
      text,
      line: el.line,
      source: el.file,
    };
  };
  return conv(root);
}
