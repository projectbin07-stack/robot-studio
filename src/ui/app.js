// Browser app: wires the file inputs, examples, panels and the 3D view to
// the DOM-free core. Model names and values are inserted as text, never as
// HTML.

import { EXAMPLES, exampleFs, findExample } from "../core/examples.js";
import { loadModel } from "../core/model.js";
import { sliders } from "../core/scene.js";
import { CHECKS } from "../../vendor/urdf-inspector/src/core/findings.js";
import { STUDIO_CHECKS } from "../core/model.js";
import { decodeState, encodeState } from "./state.js";
import { fsFromDrop, fsFromFileList } from "./files.js";
import { RobotView } from "./view3d.js";
import { VERSION } from "../core/version.js";

/** @typedef {import("../core/model.js").LoadedModel} LoadedModel */
/** @typedef {import("../core/vfs.js").VirtualFs} VirtualFs */
/** @typedef {import("../core/model.js").StudioFinding} StudioFinding */

/** @param {string} id */
const $ = (id) => /** @type {HTMLElement} */ (document.getElementById(id));

/**
 * @param {string} tag
 * @param {Record<string, string>} [attrs]
 * @param {(Node | string)[]} [children]
 */
function h(tag, attrs = {}, children = []) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") el.className = v;
    else el.setAttribute(k, v);
  }
  for (const c of children) el.append(c);
  return el;
}

/** @param {number} v @param {number} [d] */
const fx = (v, d = 4) => (Math.abs(v) < 0.5 * 10 ** -d ? 0 : v).toFixed(d);
/** @param {number} v */
const g6 = (v) => String(Number(v.toPrecision(6)));

const state = decodeState(location.search);
/** @type {VirtualFs | null} */
let fs = null;
/** @type {LoadedModel | null} */
let model = null;
let entry = "";
/** @type {StudioFinding[]} */
let meshProblems = [];
let sourceLabel = "";

const view = new RobotView($("view"), (link) => select(link ? { kind: "link", name: link } : null));

// ---------------------------------------------------------------- theme
const media = window.matchMedia("(prefers-color-scheme: dark)");
function applyTheme() {
  const dark = state.theme === "dark" || (state.theme === "auto" && media.matches);
  document.documentElement.dataset.theme = dark ? "dark" : "light";
  $("btn-theme").textContent = `Theme: ${state.theme}`;
  view.setTheme(dark);
}
media.addEventListener("change", applyTheme);
$("btn-theme").addEventListener("click", () => {
  state.theme = state.theme === "auto" ? "light" : state.theme === "light" ? "dark" : "auto";
  applyTheme();
  saveUrl();
});

// ---------------------------------------------------------------- URL
let urlTimer = 0;
function saveUrl() {
  clearTimeout(urlTimer);
  urlTimer = window.setTimeout(() => {
    const qs = encodeState(state);
    history.replaceState(null, "", qs ? `?${qs}` : location.pathname);
  }, 150);
}
$("btn-link").addEventListener("click", async () => {
  const url = `${location.origin}${location.pathname}?${encodeState(state)}`;
  try {
    await navigator.clipboard.writeText(url);
    $("btn-link").textContent = "Copied";
  } catch {
    $("btn-link").textContent = "Copy failed";
  }
  setTimeout(() => ($("btn-link").textContent = "Copy link"), 1500);
});

// ---------------------------------------------------------------- inputs
const exampleSel = /** @type {HTMLSelectElement} */ ($("example"));
exampleSel.append(h("option", { value: "" }, ["(choose an example)"]));
for (const ex of EXAMPLES) exampleSel.append(h("option", { value: ex.id }, [ex.title]));
exampleSel.addEventListener("change", () => {
  if (!exampleSel.value) return;
  state.source = exampleSel.value;
  state.q = {};
  state.args = {};
  state.selected = null;
  void openExample(exampleSel.value);
});

/** @param {VirtualFs} newFs @param {string} label */
function openFiles(newFs, label) {
  const cands = newFs.candidates();
  if (cands.length === 0) {
    setStatus(`no .urdf or .xacro file in ${label}`);
    return;
  }
  fs = newFs;
  state.source = "files";
  state.q = {};
  state.args = {};
  state.selected = null;
  exampleSel.value = "";
  $("example-note").textContent = "";
  $("attribution").textContent = "";
  const sel = /** @type {HTMLSelectElement} */ ($("entry"));
  sel.replaceChildren(...cands.map((c) => h("option", { value: c }, [c])));
  $("entry-field").hidden = cands.length < 2;
  entry = cands[0];
  sourceLabel = `files (${entry.split("/").pop()})`;
  openModel();
}
$("entry").addEventListener("change", () => {
  entry = /** @type {HTMLSelectElement} */ ($("entry")).value;
  sourceLabel = `files (${entry.split("/").pop()})`;
  state.q = {};
  state.selected = null;
  openModel();
});
$("file").addEventListener("change", async (e) => {
  const input = /** @type {HTMLInputElement} */ (e.target);
  if (input.files?.length) openFiles(await fsFromFileList(input.files), "the chosen files");
  input.value = "";
});
$("folder").addEventListener("change", async (e) => {
  const input = /** @type {HTMLInputElement} */ (e.target);
  if (input.files?.length) openFiles(await fsFromFileList(input.files), "the folder");
  input.value = "";
});
let dragDepth = 0;
window.addEventListener("dragenter", (e) => {
  e.preventDefault();
  dragDepth++;
  document.body.classList.add("is-dragging");
});
window.addEventListener("dragleave", () => {
  dragDepth = Math.max(0, dragDepth - 1);
  if (dragDepth === 0) document.body.classList.remove("is-dragging");
});
window.addEventListener("dragover", (e) => e.preventDefault());
window.addEventListener("drop", async (e) => {
  e.preventDefault();
  dragDepth = 0;
  document.body.classList.remove("is-dragging");
  if (!e.dataTransfer) return;
  setStatus("reading the dropped files");
  try {
    openFiles(await fsFromDrop(e.dataTransfer), "the dropped files");
  } catch (err) {
    setStatus(
      `could not read the dropped files: ${err instanceof Error ? err.message : String(err)}`,
    );
  }
});

$("args").addEventListener("submit", (e) => {
  e.preventDefault();
  /** @type {Record<string, string>} */
  const args = {};
  for (const input of /** @type {NodeListOf<HTMLInputElement>} */ (
    $("args-fields").querySelectorAll("input")
  )) {
    const name = /** @type {string} */ (input.dataset.arg);
    if (input.value !== input.dataset.default) args[name] = input.value;
  }
  state.args = args;
  state.q = {};
  openModel();
});

// ---------------------------------------------------------------- view options
for (const b of document.querySelectorAll("[data-geometry]"))
  b.addEventListener("click", () => {
    state.geometry = /** @type {import("./state.js").PageState["geometry"]} */ (
      /** @type {HTMLElement} */ (b).dataset.geometry
    );
    syncOptions();
  });
for (const [id, key] of /** @type {const} */ ([
  ["opt-frames", "frames"],
  ["opt-axes", "axes"],
  ["opt-com", "com"],
]))
  $(id).addEventListener("change", (e) => {
    state[key] = /** @type {HTMLInputElement} */ (e.target).checked;
    syncOptions();
  });
$("btn-fit").addEventListener("click", () => {
  if (model)
    void view
      .show(model, /** @type {VirtualFs} */ (fs), state.q)
      .then(() => select(state.selected));
});
function syncOptions() {
  for (const b of document.querySelectorAll("[data-geometry]"))
    b.setAttribute(
      "aria-pressed",
      String(/** @type {HTMLElement} */ (b).dataset.geometry === state.geometry),
    );
  /** @type {HTMLInputElement} */ ($("opt-frames")).checked = state.frames;
  /** @type {HTMLInputElement} */ ($("opt-axes")).checked = state.axes;
  /** @type {HTMLInputElement} */ ($("opt-com")).checked = state.com;
  view.applyOptions({
    geometry: state.geometry,
    frames: state.frames,
    axes: state.axes,
    com: state.com,
  });
  saveUrl();
}

$("btn-download").addEventListener("click", () => {
  if (!model?.urdf) return;
  const a = h("a", {
    href: URL.createObjectURL(new Blob([model.urdf], { type: "application/xml" })),
    download: `${model.report?.robot?.name || "robot"}.urdf`,
  });
  a.click();
});

// ---------------------------------------------------------------- loading
/** @param {string} text */
function setStatus(text) {
  $("status").textContent = text;
}

/** @param {string} id */
async function openExample(id) {
  const ex = findExample(id);
  if (!ex) return;
  exampleSel.value = id;
  $("example-note").textContent = ex.note;
  $("attribution").textContent = ex.attribution ? `Source: ${ex.attribution}` : "";
  $("entry-field").hidden = true;
  setStatus(`loading example ${id}`);
  document.body.dataset.state = "loading";
  try {
    fs = await exampleFs(ex, {
      text: async (url) => {
        const r = await fetch(url);
        if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
        return r.text();
      },
      bytes: async (url) => {
        const r = await fetch(url);
        if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
        return new Uint8Array(await r.arrayBuffer());
      },
    });
  } catch (err) {
    setStatus(`could not load example ${id}: ${err instanceof Error ? err.message : String(err)}`);
    document.body.dataset.state = "error";
    return;
  }
  entry = ex.entry;
  sourceLabel = `example ${id}`;
  openModel();
}

function openModel() {
  if (!fs) return;
  const thisFs = fs;
  document.body.dataset.state = "loading";
  setStatus(`expanding ${entry}`);
  model = loadModel(thisFs, entry, { args: state.args });
  meshProblems = [];
  const m = model;
  renderArgs(m);
  renderTree(m);
  renderJoints(m);
  renderFindings();
  renderSummary();
  $("urdf").textContent = m.urdf || "(no output: xacro failed, see Findings)";
  void view.show(m, thisFs, state.q).then((reports) => {
    if (model !== m) return;
    meshProblems = reports.map((r) => ({
      id: r.severity === "info" ? "mesh-texture-missing" : "mesh-unreadable",
      severity: r.severity ?? "warning",
      message: `mesh '${r.filename}' of link '${r.link}': ${r.message}`,
      path: `/robot/link[@name='${r.link}']`,
      target: { kind: /** @type {const} */ ("link"), name: r.link },
      line: 0,
      file: entry,
    }));
    renderFindings();
    renderSummary();
    syncOptions();
    select(state.selected);
    setStatus(`ready: ${sourceLabel}`);
    document.body.dataset.state = m.error ? "error" : "ready";
    document.body.dataset.links = String(m.report?.summary.links ?? 0);
  });
}

function allFindings() {
  if (!model) return [];
  return [...model.findings, ...meshProblems];
}

function renderSummary() {
  if (!model) return;
  const f = allFindings();
  const e = f.filter((x) => x.severity === "error").length;
  const w = f.filter((x) => x.severity === "warning").length;
  const i = f.length - e - w;
  const s = model.report?.summary;
  const parts = [];
  if (s && model.report?.robot)
    parts.push(`robot '${s.name}': ${s.links} links, ${s.joints} joints, root ${s.root ?? "none"}`);
  else parts.push("no robot model");
  parts.push(`${e} error(s), ${w} warning(s), ${i} note(s)`);
  if (model.meshes.length > 0)
    parts.push(
      `meshes ${model.meshes.length - model.meshes.filter((x) => !x.resolved).length - meshProblems.filter((p) => p.id === "mesh-unreadable").length}/${model.meshes.length} shown`,
    );
  $("summary").textContent = parts.join(" · ");
  $("finding-count").textContent = f.length ? `(${f.length})` : "";
}

/** @param {LoadedModel} m */
function renderArgs(m) {
  const names = Object.keys(m.args);
  const form = /** @type {HTMLFormElement} */ ($("args"));
  form.hidden = names.length === 0;
  const fields = names.map((n) => {
    const input = /** @type {HTMLInputElement} */ (
      h("input", { type: "text", value: m.args[n], spellcheck: "false" })
    );
    input.dataset.arg = n;
    input.dataset.default = state.args[n] === undefined ? m.args[n] : "\u0000";
    return h("label", { class: "field arg" }, [h("span", {}, [n]), input]);
  });
  $("args-fields").replaceChildren(...fields);
}

/** @param {LoadedModel} m */
function renderTree(m) {
  const tree = m.report?.context?.tree;
  const host = $("tree");
  if (!tree || tree.root === null) {
    host.replaceChildren(h("p", { class: "muted" }, ["No kinematic tree."]));
    return;
  }
  /** @param {string} link @returns {HTMLElement} */
  const node = (link) => {
    const li = h("li", {}, [
      h("button", { type: "button", class: "t-link", "data-link": link }, [link]),
    ]);
    const kids = tree.childJoints.get(link) ?? [];
    if (kids.length) {
      const ul = h("ul");
      for (const j of kids) {
        const jl = h("li", {}, [
          h("button", { type: "button", class: `t-joint ${j.type}`, "data-joint": j.name }, [
            `${j.name}`,
            h("span", { class: "jt" }, [String(j.type)]),
          ]),
        ]);
        const sub = h("ul", {}, [node(j.child)]);
        jl.append(sub);
        ul.append(jl);
      }
      li.append(ul);
    }
    return li;
  };
  const ul = h("ul", { class: "tree-root" }, [node(tree.root)]);
  const orphans = tree.roots.filter((r) => r !== tree.root);
  host.replaceChildren(
    ul,
    ...(orphans.length
      ? [h("p", { class: "muted" }, [`Not connected: ${orphans.join(", ")}`])]
      : []),
  );
  host.querySelectorAll("button").forEach((b) =>
    b.addEventListener("click", () => {
      const el = /** @type {HTMLElement} */ (b);
      if (el.dataset.link) select({ kind: "link", name: el.dataset.link });
      else select({ kind: "joint", name: /** @type {string} */ (el.dataset.joint) });
    }),
  );
}

/** @param {LoadedModel} m */
function renderJoints(m) {
  const tree = m.report?.context?.tree;
  const host = $("joints");
  const list = tree ? sliders(tree) : [];
  $("joint-count").textContent = list.length ? `(${list.length})` : "";
  if (!tree || list.length === 0) {
    host.replaceChildren(h("p", { class: "muted" }, ["No movable joints."]));
    return;
  }
  const rows = list.map((s) => {
    const v = state.q[s.name] ?? 0;
    const input = /** @type {HTMLInputElement} */ (
      h("input", {
        type: "range",
        min: String(s.min),
        max: String(s.max),
        step: String((s.max - s.min) / 400),
        value: String(v),
        "aria-label": s.name,
      })
    );
    const out = h("output", {}, [`${fx(v, 3)} ${s.unit}`]);
    if (s.mimic) {
      input.disabled = true;
      input.title = `mimics ${s.mimic}`;
    }
    input.addEventListener("input", () => {
      const val = Number(input.value);
      if (val === 0) delete state.q[s.name];
      else state.q[s.name] = val;
      const p = view.setPose(state.q);
      out.textContent = `${fx(val, 3)} ${s.unit}`;
      if (p) updateMimics(list, rows);
      if (state.selected) renderDetails(state.selected);
      saveUrl();
    });
    const label = h("button", { type: "button", class: "j-name" }, [s.name]);
    label.addEventListener("click", () => select({ kind: "joint", name: s.name }));
    const row = h("div", { class: "j-row" }, [
      label,
      input,
      out,
      h("span", { class: "j-range" }, [
        `${fx(s.min, 2)} … ${fx(s.max, 2)}${s.mimic ? ` · mimics ${s.mimic}` : ""}`,
      ]),
    ]);
    return { row, input, out, s };
  });
  const reset = h("button", { type: "button" }, ["Reset all to 0"]);
  reset.addEventListener("click", () => {
    state.q = {};
    for (const r of rows) {
      r.input.value = "0";
      r.out.textContent = `0.000 ${r.s.unit}`;
    }
    view.setPose(state.q);
    saveUrl();
  });
  host.replaceChildren(...rows.map((r) => r.row), reset);
  updateMimics(list, rows);
}

/**
 * Mimic sliders show the value forward kinematics used.
 * @param {ReturnType<typeof sliders>} list
 * @param {{ input: HTMLInputElement, out: HTMLElement, s: ReturnType<typeof sliders>[number] }[]} rows
 */
function updateMimics(list, rows) {
  const tree = model?.report?.context?.tree;
  if (!tree) return;
  for (const r of rows) {
    if (!r.s.mimic) continue;
    const j = tree.joints.get(r.s.name);
    if (!j?.mimic) continue;
    const v = j.mimic.multiplier * (state.q[j.mimic.joint] ?? 0) + j.mimic.offset;
    r.input.value = String(v);
    r.out.textContent = `${fx(v, 3)} ${r.s.unit}`;
  }
  void list;
}

function renderFindings() {
  const list = allFindings();
  const host = $("findings");
  if (list.length === 0) {
    host.replaceChildren(h("li", { class: "muted" }, ["No findings."]));
    return;
  }
  host.replaceChildren(
    ...list.map((f) => {
      const info = /** @type {Record<string, { title: string, why: string }>} */ ({
        ...CHECKS,
        ...STUDIO_CHECKS,
        "mesh-unreadable": {
          title: "Mesh could not be shown",
          why: "The file was found but could not be read as STL, OBJ or COLLADA.",
        },
        "mesh-texture-missing": {
          title: "Mesh texture missing",
          why: "The COLLADA file names an image that is not among the loaded files; RViz would also draw it without that texture.",
        },
      })[f.id];
      const loc = `${f.file.split("/").slice(-2).join("/")}${f.line ? `:${f.line}` : ""}`;
      const btn = h("button", { type: "button", class: "f-head" }, [
        h("span", { class: `sev ${f.severity}` }, [f.severity]),
        h("code", {}, [f.id]),
        h("span", { class: "loc", title: f.file }, [loc]),
      ]);
      const li = h("li", { class: `finding ${f.severity}` }, [btn, h("p", {}, [f.message])]);
      if (info) li.append(h("p", { class: "why" }, [info.why]));
      btn.addEventListener("click", () => {
        if (f.target.kind === "link" || f.target.kind === "joint")
          select({ kind: f.target.kind, name: f.target.name });
      });
      return li;
    }),
  );
}

// ---------------------------------------------------------------- selection
/** @param {{ kind: "link" | "joint", name: string } | null} sel */
function select(sel) {
  const tree = model?.report?.context?.tree;
  if (sel && tree && !(sel.kind === "link" ? tree.links.has(sel.name) : tree.joints.has(sel.name)))
    sel = null;
  state.selected = sel;
  for (const b of $("tree").querySelectorAll("button")) {
    const el = /** @type {HTMLElement} */ (b);
    const on =
      sel !== null &&
      (sel.kind === "link" ? el.dataset.link === sel.name : el.dataset.joint === sel.name);
    el.classList.toggle("selected", on);
    if (on) el.scrollIntoView({ block: "nearest" });
  }
  const link =
    sel === null
      ? null
      : sel.kind === "link"
        ? sel.name
        : (tree?.joints.get(sel.name)?.child ?? null);
  view.highlight(link);
  renderDetails(sel);
  saveUrl();
}

/** @param {{ kind: "link" | "joint", name: string } | null} sel */
function renderDetails(sel) {
  const host = $("details");
  const tree = model?.report?.context?.tree;
  if (!sel || !tree || !model) {
    host.replaceChildren(h("p", { class: "muted" }, ["Click a part or a tree entry."]));
    return;
  }
  const poses = view.setPose(state.q);
  /** @type {[string, string][]} */
  const rows = [];
  const fileOf = (/** @type {string} */ path) => model?.sourceOf(path);
  if (sel.kind === "link") {
    const l = /** @type {import("../../vendor/urdf-inspector/src/core/urdf.js").Link} */ (
      tree.links.get(sel.name)
    );
    const pj = tree.parentJoint.get(l.name);
    rows.push(["Parent joint", pj ? pj.name : "(root)"]);
    rows.push([
      "Child joints",
      (tree.childJoints.get(l.name) ?? []).map((j) => j.name).join(", ") || "none",
    ]);
    rows.push(["Source line", `${l.line}${fileOf(l.path) ? ` (${fileOf(l.path)})` : ""}`]);
    const m = poses?.links.get(l.name);
    if (m) rows.push(["Position in root", `${fx(m[12])}, ${fx(m[13])}, ${fx(m[14])} m`]);
    if (l.inertial) {
      rows.push(["Mass", l.inertial.mass === null ? "missing" : `${g6(l.inertial.mass)} kg`]);
      rows.push([
        "Inertial origin",
        `xyz ${l.inertial.origin.xyz.map((v) => fx(v)).join(" ")} · rpy ${l.inertial.origin.rpy.map((v) => fx(v)).join(" ")}`,
      ]);
      const t = l.inertial.inertia;
      if (t)
        rows.push([
          "Inertia (kg m²)",
          `ixx ${g6(t.ixx)} iyy ${g6(t.iyy)} izz ${g6(t.izz)} ixy ${g6(t.ixy)} ixz ${g6(t.ixz)} iyz ${g6(t.iyz)}`,
        ]);
    } else rows.push(["Inertial", "none"]);
    for (const kind of /** @type {const} */ (["visual", "collision"])) {
      const shapes = kind === "visual" ? l.visuals : l.collisions;
      shapes.forEach((s, i) => {
        const g = s.geometry;
        let d = "no geometry";
        if (g?.type === "box") d = `box ${g.size.map(g6).join(" × ")} m`;
        else if (g?.type === "cylinder")
          d = `cylinder r ${g6(g.radius)} m, length ${g6(g.length)} m`;
        else if (g?.type === "sphere") d = `sphere r ${g6(g.radius)} m`;
        else if (g?.type === "mesh") {
          const ref = model?.meshes.find(
            (x) => x.link === l.name && x.kind === kind && x.index === i,
          );
          d = `mesh ${g.filename}${g.scale.some((v) => v !== 1) ? ` × ${g.scale.map(g6).join(" ")}` : ""} — ${ref?.resolved ? "found" : `missing (${ref?.reason ?? "not resolved"})`}`;
        }
        const prev = rows[rows.length - 1];
        const m = prev ? /^(\w+) (\d+)(?:–(\d+))?$/.exec(prev[0]) : null;
        if (prev && m && m[1] === kind && prev[1] === d) prev[0] = `${kind} ${m[2]}–${i + 1}`;
        else rows.push([`${kind} ${i + 1}`, d]);
      });
    }
  } else {
    const j = /** @type {import("../../vendor/urdf-inspector/src/core/urdf.js").Joint} */ (
      tree.joints.get(sel.name)
    );
    rows.push(["Type", String(j.type)]);
    rows.push(["Parent → child", `${j.parent} → ${j.child}`]);
    rows.push([
      "Origin",
      `xyz ${j.origin.xyz.map((v) => fx(v)).join(" ")} · rpy ${j.origin.rpy.map((v) => fx(v)).join(" ")}`,
    ]);
    if (j.type !== "fixed") rows.push(["Axis", j.axis.map((v) => fx(v, 3)).join(" ")]);
    if (j.limit)
      rows.push([
        "Limits",
        `lower ${j.limit.lower ?? "-"} upper ${j.limit.upper ?? "-"} effort ${j.limit.effort ?? "-"} velocity ${j.limit.velocity ?? "-"}`,
      ]);
    if (j.mimic)
      rows.push(["Mimic", `${j.mimic.joint} × ${j.mimic.multiplier} + ${j.mimic.offset}`]);
    if (["revolute", "continuous", "prismatic"].includes(String(j.type)))
      rows.push([
        "Position",
        `${fx(state.q[j.name] ?? 0, 4)} ${j.type === "prismatic" ? "m" : "rad"}`,
      ]);
    rows.push(["Source line", `${j.line}${fileOf(j.path) ? ` (${fileOf(j.path)})` : ""}`]);
  }
  const related = allFindings().filter(
    (f) => f.target.kind === sel.kind && f.target.name === sel.name,
  );
  host.replaceChildren(
    h("h3", {}, [`${sel.kind === "link" ? "Link" : "Joint"} ${sel.name}`]),
    h(
      "dl",
      {},
      rows.flatMap(([k, v]) => [h("dt", {}, [k]), h("dd", {}, [v])]),
    ),
    ...(related.length
      ? [
          h("p", { class: "muted" }, [
            `${related.length} finding(s): ${related.map((f) => f.id).join(", ")}`,
          ]),
        ]
      : []),
  );
}

// ---------------------------------------------------------------- start
document.title = `Robot Studio ${VERSION}`;
applyTheme();
syncOptions();
if (state.source === "files") {
  setStatus(
    "This link was made with local files, which are never put in a link. Drop them again, or pick an example.",
  );
  document.body.dataset.state = "ready";
  document.body.dataset.links = "0";
} else void openExample(state.source);
