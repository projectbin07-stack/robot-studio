// Autosave format for the working document. The browser keeps it in
// localStorage (src/ui wraps every access in try/catch); this module only
// turns a document into text and back, rejecting anything that is not a
// document this version can edit.

/** @typedef {import("./document.js").RobotDoc} RobotDoc */

export const AUTOSAVE_KEY = "robot-studio:builder:v1";

/**
 * @param {RobotDoc} doc
 * @param {Date} [now]
 */
export function encodeAutosave(doc, now = new Date()) {
  return JSON.stringify({ app: "robot-studio", format: 1, savedAt: now.toISOString(), doc });
}

/**
 * @param {string | null} text
 * @returns {{ doc: RobotDoc, savedAt: string } | null}
 */
export function decodeAutosave(text) {
  if (!text) return null;
  /** @type {any} */
  let v;
  try {
    v = JSON.parse(text);
  } catch {
    return null;
  }
  if (!v || v.app !== "robot-studio" || v.format !== 1 || typeof v.savedAt !== "string")
    return null;
  const d = v.doc;
  const ok =
    d &&
    d.format === 1 &&
    typeof d.name === "string" &&
    Array.isArray(d.links) &&
    Array.isArray(d.joints) &&
    Array.isArray(d.materials) &&
    Array.isArray(d.extras) &&
    d.links.every(
      (/** @type {any} */ l) =>
        l && typeof l.name === "string" && Array.isArray(l.visuals) && Array.isArray(l.collisions),
    ) &&
    d.joints.every(
      (/** @type {any} */ j) =>
        j &&
        typeof j.name === "string" &&
        typeof j.parent === "string" &&
        typeof j.child === "string" &&
        j.origin,
    );
  return ok ? { doc: /** @type {RobotDoc} */ (d), savedAt: v.savedAt } : null;
}
