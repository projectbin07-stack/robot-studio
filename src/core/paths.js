// Forward-slash path helpers shared by the browser (virtual file system of a
// dropped folder or zip) and the CLI (real paths, converted to forward
// slashes). A path is absolute when it starts with "/" or a drive letter.

/** @param {string} p */
export const toSlashes = (p) => p.replace(/\\/g, "/");

/** @param {string} p */
export const isAbsolute = (p) => p.startsWith("/") || /^[A-Za-z]:\//.test(p);

/**
 * Collapse "." and ".." segments and repeated slashes. A ".." above the
 * root of an absolute path is dropped; above a relative path it is kept.
 * @param {string} path
 */
export function normalize(path) {
  const p = toSlashes(path);
  const drive = /^[A-Za-z]:/.exec(p)?.[0] ?? "";
  const rest = p.slice(drive.length);
  const abs = rest.startsWith("/");
  /** @type {string[]} */
  const out = [];
  for (const seg of rest.split("/")) {
    if (seg === "" || seg === ".") continue;
    if (seg === "..") {
      if (out.length > 0 && out[out.length - 1] !== "..") out.pop();
      else if (!abs) out.push("..");
    } else out.push(seg);
  }
  const body = out.join("/");
  if (abs) return `${drive}/${body}`;
  return drive + (body || ".");
}

/** @param {...string} parts */
export function join(...parts) {
  let acc = "";
  for (const raw of parts) {
    const part = toSlashes(raw);
    if (part === "") continue;
    acc = acc === "" || isAbsolute(part) ? part : `${acc.replace(/\/+$/, "")}/${part}`;
  }
  return normalize(acc || ".");
}

/** @param {string} path */
export function dirname(path) {
  const p = normalize(path);
  const k = p.lastIndexOf("/");
  if (k < 0) return ".";
  if (k === 0) return "/";
  if (/^[A-Za-z]:$/.test(p.slice(0, k))) return p.slice(0, k + 1);
  return p.slice(0, k);
}

/** @param {string} path */
export const basename = (path) => {
  const p = normalize(path);
  return p.slice(p.lastIndexOf("/") + 1);
};

/** Lower-case extension without the dot, "" when there is none. @param {string} path */
export const extname = (path) => /\.([A-Za-z0-9]+)$/.exec(basename(path))?.[1]?.toLowerCase() ?? "";

/**
 * Translate a glob with * and ? (no "/" crossing) and [...] classes into a
 * regular expression matching whole paths.
 * @param {string} pattern
 */
export function globToRegExp(pattern) {
  let re = "";
  for (let i = 0; i < pattern.length; i++) {
    const c = pattern[i];
    if (c === "*") re += "[^/]*";
    else if (c === "?") re += "[^/]";
    else if (c === "[") {
      const end = pattern.indexOf("]", i + 1);
      if (end < 0) re += "\\[";
      else {
        const body = pattern
          .slice(i + 1, end)
          .replace(/^!/, "^")
          .replace(/\\/g, "\\\\");
        re += `[${body}]`;
        i = end;
      }
    } else re += c.replace(/[.+^${}()|\\]/g, "\\$&");
  }
  return new RegExp(`^${re}$`);
}
