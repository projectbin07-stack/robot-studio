// URL parameters <-> page state. DOM-free so it is tested under Node. Only
// values that differ from the defaults are written. A model loaded from
// local files is never put in the URL: a link made with one says so and
// asks for the files again.

import { EXAMPLES } from "../core/examples.js";

/**
 * @typedef {object} PageState
 * @property {string} source example id, or "files" for a dropped model
 * @property {{ kind: "link" | "joint", name: string } | null} selected
 * @property {"visual" | "collision" | "both"} geometry
 * @property {boolean} frames link frame axes
 * @property {boolean} axes joint axes
 * @property {boolean} com centre-of-mass marker
 * @property {Record<string, number>} q joint positions (examples only)
 * @property {Record<string, string>} args xacro argument overrides (examples only)
 * @property {"auto" | "light" | "dark"} theme
 */

/** @returns {PageState} */
export function defaultState() {
  return {
    source: "mecanum",
    selected: null,
    geometry: "visual",
    frames: false,
    axes: false,
    com: false,
    q: {},
    args: {},
    theme: "auto",
  };
}

const THEMES = ["auto", "light", "dark"];
const GEOMETRY = ["visual", "collision", "both"];
const NAME = /^[A-Za-z0-9_.\-/:]{1,128}$/;
const ARG_VALUE = /^[A-Za-z0-9_.\-/+ ]{0,128}$/;

/** @param {number} v */
const num = (v) => String(Number(v.toPrecision(6)));

/** @param {[string, unknown][]} entries */
const sorted = (entries) => entries.sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0));

/**
 * @param {PageState} st
 * @returns {string} query string without "?"
 */
export function encodeState(st) {
  const d = defaultState();
  const p = new URLSearchParams();
  const isExample = EXAMPLES.some((e) => e.id === st.source);
  if (st.source !== d.source) p.set("s", st.source);
  if (st.selected) p.set("sel", `${st.selected.kind === "link" ? "l" : "j"}:${st.selected.name}`);
  if (st.geometry !== d.geometry) p.set("g", st.geometry);
  if (st.frames !== d.frames) p.set("fr", st.frames ? "1" : "0");
  if (st.axes !== d.axes) p.set("ja", st.axes ? "1" : "0");
  if (st.com !== d.com) p.set("com", st.com ? "1" : "0");
  if (isExample) {
    const q = sorted(Object.entries(st.q).filter(([, v]) => v !== 0 && Number.isFinite(v)));
    if (q.length > 0)
      p.set("q", q.map(([n, v]) => `${n}:${num(/** @type {number} */ (v))}`).join(","));
    const a = sorted(Object.entries(st.args));
    if (a.length > 0) p.set("a", a.map(([n, v]) => `${n}:${v}`).join(","));
  }
  if (st.theme !== d.theme) p.set("theme", st.theme);
  return p.toString();
}

/**
 * Parse a query string; unknown or malformed values fall back to defaults.
 * @param {string} search with or without "?"
 * @returns {PageState}
 */
export function decodeState(search) {
  const st = defaultState();
  const p = new URLSearchParams(search);
  const s = p.get("s");
  if (s !== null && (EXAMPLES.some((e) => e.id === s) || s === "files")) st.source = s;
  const sel = p.get("sel");
  const m = sel ? /^([lj]):(.+)$/.exec(sel) : null;
  if (m && NAME.test(m[2])) st.selected = { kind: m[1] === "l" ? "link" : "joint", name: m[2] };
  const g = p.get("g");
  if (g && GEOMETRY.includes(g)) st.geometry = /** @type {PageState["geometry"]} */ (g);
  if (p.get("fr") === "1") st.frames = true;
  if (p.get("ja") === "1") st.axes = true;
  if (p.get("com") === "1") st.com = true;
  for (const part of (p.get("q") ?? "").split(",")) {
    const k = part.lastIndexOf(":");
    if (k <= 0) continue;
    const name = part.slice(0, k);
    const v = Number(part.slice(k + 1));
    if (NAME.test(name) && Number.isFinite(v) && Math.abs(v) < 1e4) st.q[name] = v;
  }
  for (const part of (p.get("a") ?? "").split(",")) {
    const k = part.indexOf(":");
    if (k <= 0) continue;
    const name = part.slice(0, k);
    const v = part.slice(k + 1);
    if (/^[A-Za-z_][A-Za-z0-9_]{0,63}$/.test(name) && ARG_VALUE.test(v)) st.args[name] = v;
  }
  const theme = p.get("theme");
  if (theme && THEMES.includes(theme)) st.theme = /** @type {PageState["theme"]} */ (theme);
  return st;
}
