// A small XML document model for xacro processing. Unlike the vendored
// urdf-inspector reader (vendor/urdf-inspector/src/core/xml.js), which
// merges character data per element and drops comments, xacro needs the
// children in document order with text and comments kept as separate
// nodes: blocks, conditionals and macro bodies move mixed content around,
// and comments survive into the output. See docs/decisions.md.
//
// Follows the XML 1.0 rules that matter for robot descriptions: line-end
// normalisation, attribute-value whitespace normalisation, the five
// predefined entities and character references, comments, CDATA and
// processing instructions. DTD internal subsets are rejected.

/**
 * @typedef {object} XText
 * @property {"text"} type
 * @property {string} value
 */
/**
 * CDATA sections stay separate: xacro does not evaluate them.
 * @typedef {object} XCData
 * @property {"cdata"} type
 * @property {string} value
 */
/**
 * @typedef {object} XComment
 * @property {"comment"} type
 * @property {string} value
 */
/**
 * @typedef {object} XElement
 * @property {"element"} type
 * @property {string} name tag name including any prefix
 * @property {Record<string, string>} attrs in document order
 * @property {XNode[]} children
 * @property {string} file source file the element was read from
 * @property {number} line 1-based line of the start tag
 * @property {XComment[]} [prolog] comments before the document element (root only)
 */
/** @typedef {XElement | XText | XCData | XComment} XNode */

export class XmlSyntaxError extends Error {
  /**
   * @param {string} message
   * @param {string} file
   * @param {number} line
   * @param {number} col
   */
  constructor(message, file, line, col) {
    super(`${file}:${line}:${col}: ${message}`);
    this.file = file;
    this.line = line;
    this.col = col;
    this.detail = message;
  }
}

const NAME_START = /[A-Za-z_:À-￿]/;
const NAME_CHAR = /[A-Za-z0-9_:.\-·À-￿]/;
/** @type {Record<string, string>} */
const ENTITIES = { lt: "<", gt: ">", amp: "&", quot: '"', apos: "'" };

/**
 * Parse a document and return its root element.
 * @param {string} input
 * @param {string} [file] name used in error messages and stored on elements
 * @returns {XElement}
 */
export function parseDocument(input, file = "<string>") {
  const src = input.replace(/\r\n?/g, "\n");
  let i = src.charCodeAt(0) === 0xfeff ? 1 : 0;
  let line = 1;
  let lineStart = 0;
  const n = src.length;

  /** @param {string} msg @returns {never} */
  const fail = (msg) => {
    throw new XmlSyntaxError(msg, file, line, i - lineStart + 1);
  };
  /** @param {number} to */
  const advance = (to) => {
    for (; i < to; i++)
      if (src.charCodeAt(i) === 10) {
        line++;
        lineStart = i + 1;
      }
  };
  /** @param {string} s */
  const at = (s) => src.startsWith(s, i);
  /** @param {string} end @param {string} what */
  const until = (end, what) => {
    const j = src.indexOf(end, i);
    if (j < 0) fail(`unterminated ${what}`);
    const body = src.slice(i, j);
    advance(j + end.length);
    return body;
  };
  const skipSpace = () => {
    while (i < n && /\s/.test(src[i])) advance(i + 1);
  };
  const readName = () => {
    if (i >= n || !NAME_START.test(src[i])) fail("expected a name");
    const s = i;
    while (i < n && NAME_CHAR.test(src[i])) i++;
    return src.slice(s, i);
  };
  /** @param {string} raw */
  const decode = (raw) =>
    raw.replace(/&(#x[0-9A-Fa-f]+|#[0-9]+|[A-Za-z]+);|&/g, (m, ent) => {
      if (ent === undefined) fail("bare '&' (write &amp;)");
      if (ent[0] === "#") {
        const code = ent[1] === "x" ? parseInt(ent.slice(2), 16) : parseInt(ent.slice(1), 10);
        if (!Number.isFinite(code) || code > 0x10ffff) fail(`bad character reference ${m}`);
        return String.fromCodePoint(code);
      }
      const v = ENTITIES[ent];
      if (v === undefined) fail(`unknown entity ${m}`);
      return v;
    });

  /** @type {XComment[]} */
  const prologComments = [];
  /** @param {boolean} prolog */
  const skipMisc = (prolog) => {
    for (;;) {
      skipSpace();
      if (at("<!--")) {
        advance(i + 4);
        const c = until("-->", "comment");
        if (prolog) prologComments.push({ type: "comment", value: c });
      } else if (at("<?")) until("?>", "processing instruction");
      else if (prolog && at("<!DOCTYPE")) {
        const gt = src.indexOf(">", i);
        const br = src.indexOf("[", i);
        if (br >= 0 && (gt < 0 || br < gt))
          fail("DOCTYPE with an internal subset is not supported");
        until(">", "DOCTYPE");
      } else return;
    }
  };

  /** @returns {XElement} */
  const readElement = () => {
    const elLine = line;
    advance(i + 1);
    const name = readName();
    /** @type {Record<string, string>} */
    const attrs = {};
    /** @type {XElement} */
    const el = { type: "element", name, attrs, children: [], file, line: elLine };
    for (;;) {
      const before = i;
      skipSpace();
      if (at("/>")) {
        advance(i + 2);
        return el;
      }
      if (at(">")) {
        advance(i + 1);
        break;
      }
      if (i === before) fail(`expected whitespace, '>' or '/>' in <${name}>`);
      const key = readName();
      skipSpace();
      if (src[i] !== "=") fail(`attribute ${key} has no value`);
      advance(i + 1);
      skipSpace();
      const q = src[i];
      if (q !== '"' && q !== "'") fail(`attribute ${key} value must be quoted`);
      const end = src.indexOf(q, i + 1);
      if (end < 0) fail(`unterminated value of attribute ${key}`);
      const raw = src.slice(i + 1, end);
      if (raw.includes("<")) fail(`'<' inside the value of attribute ${key}`);
      if (Object.hasOwn(attrs, key)) fail(`duplicate attribute ${key} on <${name}>`);
      attrs[key] = decode(raw.replace(/[\t\n]/g, " "));
      advance(end + 1);
    }
    const kids = el.children;
    /** @param {string} t */
    const pushText = (t) => {
      const last = kids[kids.length - 1];
      if (last && last.type === "text") last.value += t;
      else kids.push({ type: "text", value: t });
    };
    for (;;) {
      if (i >= n) fail(`<${name}> (line ${elLine}) is not closed`);
      if (at("</")) {
        advance(i + 2);
        const close = readName();
        if (close !== name) fail(`</${close}> does not match <${name}> from line ${elLine}`);
        skipSpace();
        if (src[i] !== ">") fail(`expected '>' after </${close}`);
        advance(i + 1);
        return el;
      }
      if (at("<!--")) {
        advance(i + 4);
        kids.push({ type: "comment", value: until("-->", "comment") });
      } else if (at("<![CDATA[")) {
        advance(i + 9);
        kids.push({ type: "cdata", value: until("]]>", "CDATA section") });
      } else if (at("<?")) until("?>", "processing instruction");
      else if (src[i] === "<") kids.push(readElement());
      else {
        let end = src.indexOf("<", i);
        if (end < 0) end = n;
        const raw = src.slice(i, end);
        advance(end);
        pushText(decode(raw));
      }
    }
  };

  skipMisc(true);
  if (i >= n || src[i] !== "<") fail("no root element");
  const root = readElement();
  if (prologComments.length > 0) root.prolog = prologComments;
  skipMisc(false);
  if (i < n) fail("content after the root element");
  return root;
}

/**
 * Deep copy of a node (macro bodies and blocks are cloned before use).
 * @template {XNode} T
 * @param {T} node
 * @returns {T}
 */
export function cloneNode(node) {
  if (node.type !== "element") return /** @type {T} */ ({ ...node });
  return /** @type {T} */ ({
    ...node,
    attrs: { ...node.attrs },
    children: node.children.map((c) => cloneNode(c)),
  });
}

/**
 * Element children only.
 * @param {XElement} el
 * @returns {XElement[]}
 */
export const elementChildren = (el) =>
  /** @type {XElement[]} */ (el.children.filter((c) => c.type === "element"));
