// Public entry point of the xacro implementation.

export { expandXacro, parseMacroArg, XacroError } from "./process.js";
export { serializeXml, toXmlElement } from "./serialize.js";
export { parseDocument, XmlSyntaxError } from "./dom.js";
export { evalExpression } from "./pyeval.js";

/**
 * Whether a document needs xacro at all: a xacro namespace, a xacro:
 * element or attribute, or a ${...} / $(...) substitution.
 * @param {string} text
 */
export const looksLikeXacro = (text) => /xmlns:xacro\s*=|<xacro:|\$\{|\$\(/.test(text);
