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
import { Editor } from "../core/builder/editor.js";
import { EditError } from "../core/builder/ops.js";
import { importUrdf, toUrdf } from "../core/builder/urdf.js";
import {
  TEMPLATES,
  TemplateError,
  defaultParams,
  findTemplate,
  generate,
  validateParams,
} from "../core/builder/templates.js";
import { PACKAGE_NAME_RE, exportXacro, packageZip } from "../core/builder/export.js";
import { mecanumConfig } from "../core/builder/drive.js";
import { AUTOSAVE_KEY, decodeAutosave, encodeAutosave } from "../core/builder/storage.js";
import { addChild, addSensor, collisionFromVisual } from "../core/builder/actions.js";
import { rootLinks } from "../core/builder/document.js";
import { VirtualFs as EmptyFs } from "../core/vfs.js";
import { renderProperties, renderTemplateForm } from "./builder.js";
import { initSim } from "./simui.js";
import { initRos } from "./rosui.js";
import { deepEqual } from "../core/builder/document.js";

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
/** The builder session, null while only viewing a model. @type {Editor | null} */
let editor = null;
/** Files an imported model came from (for its meshes), null for template robots. @type {VirtualFs | null} */
let editMeshFs = null;
const NO_FILES = new EmptyFs();
/** The file system the 3D view reads meshes from. */
const viewFs = () => (editor ? (editMeshFs ?? NO_FILES) : /** @type {VirtualFs} */ (fs));

const view = new RobotView($("view"), (link) => select(link ? { kind: "link", name: link } : null));
/** The ROS 2 panel (set up at the end of this module). @type {ReturnType<typeof initRos> | null} */
let rosUi = null;
/** The simulation panel (set up at the end of this module). @type {ReturnType<typeof initSim> | null} */
let simUi = null;
/** The opened model as an editable document, for the simulator (sensor tags, Gazebo tags). */
/** @type {{ model: LoadedModel, doc: import("../core/builder/document.js").RobotDoc | null } | null} */
let viewerDoc = null;

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
  if (model) void view.show(model, viewFs(), state.q).then(() => select(state.selected));
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
  editor = null;
  editMeshFs = null;
  updateEditUi();
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
    meshProblems = meshFindings(reports, entry);
    renderFindings();
    renderSummary();
    syncOptions();
    select(state.selected);
    setStatus(`ready: ${sourceLabel}`);
    document.body.dataset.state = m.error ? "error" : "ready";
    document.body.dataset.links = String(m.report?.summary.links ?? 0);
    simUi?.robotChanged();
    simUi?.openFromLink();
    rosUi?.robotChanged();
  });
}

/**
 * Findings for meshes the view could not draw.
 * @param {import("./view3d.js").MeshReport[]} reports
 * @param {string} file
 * @returns {StudioFinding[]}
 */
function meshFindings(reports, file) {
  return reports.map((r) => ({
    id: r.severity === "info" ? "mesh-texture-missing" : "mesh-unreadable",
    severity: r.severity ?? "warning",
    message: `mesh '${r.filename}' of link '${r.link}': ${r.message}`,
    path: `/robot/link[@name='${r.link}']`,
    target: { kind: /** @type {const} */ ("link"), name: r.link },
    line: 0,
    file,
  }));
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
  if (editor) {
    renderProperties(host, editor.doc, sel, {
      apply: edit,
      addChild: (link) => runAction(() => addChild(editor_().doc, link), "add child"),
      addSensor: (link, kind) =>
        runAction(() => addSensor(editor_().doc, kind, link), `add ${kind.replace("_", " ")}`),
      collisionFromVisual: (link) =>
        runAction(
          () => ({ doc: collisionFromVisual(editor_().doc, link), link }),
          "collision from visual",
        ),
      meshBounds: (link, kind, index) => view.meshBounds.get(`${link}|${kind}|${index}`),
    });
    return;
  }
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

// ---------------------------------------------------------------- builder

const editor_ = () => /** @type {Editor} */ (editor);
const templateSel = /** @type {HTMLSelectElement} */ ($("template"));
for (const t of TEMPLATES) templateSel.append(h("option", { value: t.id }, [t.title]));
/** Parameters typed into the form for templates not being edited live. @type {Record<string, Record<string, any>>} */
const formParams = {};
let renderedForm = "";

/** @param {unknown} v */
const fmtValue = (v) => (Array.isArray(v) ? v.join(" ") : String(v));

/** @param {string} text */
function flash(text) {
  $("edit-status").textContent = text;
  setStatus(text);
}

/** The template form: live (regenerating the robot) while editing that template's robot. */
function renderTemplateUi(force = false) {
  const t = findTemplate(templateSel.value);
  const live = editor?.doc.template?.id === t.id;
  const params = live
    ? /** @type {Record<string, any>} */ (editor_().doc.template?.params)
    : (formParams[t.id] ??= defaultParams(t.id));
  const key = JSON.stringify([t.id, live, params]);
  $("template-note").textContent = t.description;
  $("template-live").hidden = !live;
  if (!force && key === renderedForm) return;
  renderedForm = key;
  renderTemplateForm($("template-fields"), t, params, (k, value) => {
    const next = { ...params, [k]: value };
    try {
      if (live) {
        const doc = generate(t.id, next);
        const sim = editor_().doc.sim;
        if (sim) doc.sim = sim; // a parameter change keeps the simulation settings
        $("template-error").textContent = "";
        edit({ type: "replaceDocument", doc, label: `${k} = ${fmtValue(value)}` });
      } else {
        validateParams(t.id, next);
        formParams[t.id] = next;
        $("template-error").textContent = "";
      }
    } catch (e) {
      if (!(e instanceof TemplateError)) throw e;
      $("template-error").textContent = e.message;
      renderTemplateUi(true);
    }
  });
}
templateSel.addEventListener("change", () => {
  $("template-error").textContent = "";
  renderTemplateUi();
});

$("template-form").addEventListener("submit", (e) => {
  e.preventDefault();
  const t = findTemplate(templateSel.value);
  const live = editor?.doc.template?.id === t.id;
  let doc;
  try {
    doc = generate(t.id, live ? editor_().doc.template?.params : (formParams[t.id] ?? {}));
  } catch (err) {
    if (!(err instanceof TemplateError)) throw err;
    $("template-error").textContent = err.message;
    return;
  }
  state.selected = null;
  if (editor && !editMeshFs) edit({ type: "replaceDocument", doc, label: `new ${t.id}` }, true);
  else startEditing(doc, null, `new robot from the ${t.id} template`);
});

$("btn-edit-model").addEventListener("click", () => {
  if (!model?.urdf || !fs) return;
  const imp = importUrdf(model.urdf, { entry: model.entry });
  if (!imp.doc) {
    setStatus(`cannot edit this model: ${imp.notes.join("; ")}`);
    return;
  }
  startEditing(
    imp.doc,
    fs,
    `editing ${sourceLabel}${model.xacro ? " (the expanded URDF: xacro structure is not kept)" : ""}${imp.notes.length ? `; ${imp.notes.join("; ")}` : ""}`,
  );
});

/**
 * Start a new builder session (fresh history).
 * @param {import("../core/builder/document.js").RobotDoc} doc
 * @param {VirtualFs | null} meshFs
 * @param {string} message
 */
function startEditing(doc, meshFs, message) {
  editor = new Editor(doc, meshFs);
  editMeshFs = meshFs;
  state.source = "builder";
  state.q = {};
  state.args = {};
  exampleSel.value = "";
  $("example-note").textContent = "";
  $("attribution").textContent = "";
  $("entry-field").hidden = true;
  sourceLabel = "builder";
  flash(message);
  void showEditor(true);
}

/**
 * Apply an edit; on a rejected edit nothing changes and the reason is shown.
 * @param {import("../core/builder/ops.js").EditOp} op
 * @param {boolean} [frame] re-frame the camera afterwards
 */
function edit(op, frame = false) {
  if (!editor) return;
  const t0 = performance.now();
  try {
    editor.apply(op);
  } catch (e) {
    if (!(e instanceof EditError || e instanceof RangeError || e instanceof TemplateError)) throw e;
    flash(e.message);
    renderDetails(state.selected);
    return;
  }
  const sel = state.selected;
  if (sel && (op.type === "renameLink" || op.type === "renameJoint") && sel.name === op.from)
    state.selected = { kind: sel.kind, name: op.to };
  $("edit-status").textContent = `${editor.undoLabel ?? ""}`;
  void showEditor(frame, t0);
}

/**
 * A multi-operation action as one undoable step; selects the new link.
 * @param {() => { doc: import("../core/builder/document.js").RobotDoc, link: string }} make
 * @param {string} label
 */
function runAction(make, label) {
  try {
    const r = make();
    state.selected = { kind: "link", name: r.link };
    edit({ type: "replaceDocument", doc: r.doc, label });
  } catch (e) {
    if (!(e instanceof EditError || e instanceof RangeError)) throw e;
    flash(e.message);
  }
}

function undo() {
  if (editor?.undo()) {
    $("edit-status").textContent = `undone: ${editor.redoLabel}`;
    void showEditor(false);
  }
}
function redo() {
  if (editor?.redo()) {
    $("edit-status").textContent = `redone: ${editor.undoLabel}`;
    void showEditor(false);
  }
}

/** The link the toolbar acts on: the selected link, or the child of the selected joint. */
function selectedLink() {
  const sel = state.selected;
  if (!editor || !sel) return null;
  if (sel.kind === "link") return sel.name;
  return editor.doc.joints.find((j) => j.name === sel.name)?.child ?? null;
}

$("btn-undo").addEventListener("click", undo);
$("btn-redo").addEventListener("click", redo);
$("btn-add-child").addEventListener("click", () => {
  if (!editor) return;
  const parent = selectedLink() ?? rootLinks(editor.doc)[0];
  if (parent) runAction(() => addChild(editor_().doc, parent), "add child");
});
$("btn-duplicate").addEventListener("click", () => {
  const l = selectedLink();
  if (l) edit({ type: "duplicateLink", name: l });
});
function deleteSelected() {
  const l = selectedLink();
  if (!l) return;
  edit({ type: "removeLink", name: l });
}
$("btn-delete").addEventListener("click", deleteSelected);

window.addEventListener("keydown", (e) => {
  if (!editor) return;
  const t = /** @type {HTMLElement | null} */ (e.target);
  if (t && (t.tagName === "INPUT" || t.tagName === "SELECT" || t.tagName === "TEXTAREA")) return;
  const mod = e.ctrlKey || e.metaKey;
  const k = e.key.toLowerCase();
  if (mod && k === "z" && !e.shiftKey) {
    e.preventDefault();
    undo();
  } else if (mod && (k === "y" || (k === "z" && e.shiftKey))) {
    e.preventDefault();
    redo();
  } else if (!mod && (e.key === "Delete" || e.key === "Backspace")) {
    e.preventDefault();
    deleteSelected();
  }
});

/**
 * Show the editor's current model.
 * @param {boolean} frame re-frame the camera (new robot)
 * @param {number} [t0] when the edit started, for the timing shown to tests
 */
async function showEditor(frame, t0 = performance.now()) {
  if (!editor) return;
  const m = editor.model;
  model = m;
  meshProblems = [];
  document.body.dataset.state = "loading";
  renderArgs(m);
  renderTree(m);
  renderJoints(m);
  renderFindings();
  renderSummary();
  $("urdf").textContent = m.urdf;
  updateEditUi();
  autosave();
  const reports = await view.show(m, viewFs(), state.q, { keepCamera: !frame });
  if (model !== m) return;
  meshProblems = meshFindings(reports, m.entry);
  renderFindings();
  renderSummary();
  syncOptions();
  select(state.selected);
  document.body.dataset.state = "ready";
  document.body.dataset.links = String(m.report?.summary.links ?? 0);
  document.body.dataset.editMs = (performance.now() - t0).toFixed(1);
  document.body.dataset.version = String(editor.version);
  simUi?.robotChanged();
  simUi?.openFromLink();
  rosUi?.robotChanged();
}

let packageNameEdited = false;
$("package-name").addEventListener("input", () => (packageNameEdited = true));

function updateEditUi() {
  const on = editor !== null;
  document.body.classList.toggle("editing", on);
  $("edit-bar").hidden = !on;
  $("p-export").hidden = !on;
  $("btn-edit-model").hidden = on;
  $("details-title").textContent = on ? "Properties" : "Details";
  $("btn-restore").hidden = on || readAutosave() === null;
  if (!on) {
    const saved = readAutosave();
    if (saved) $("btn-restore").textContent = `Restore autosaved robot '${saved.doc.name}'`;
    renderTemplateUi();
    return;
  }
  const ed = editor_();
  const undoBtn = /** @type {HTMLButtonElement} */ ($("btn-undo"));
  const redoBtn = /** @type {HTMLButtonElement} */ ($("btn-redo"));
  undoBtn.disabled = !ed.canUndo;
  redoBtn.disabled = !ed.canRedo;
  undoBtn.title = ed.undoLabel ? `Undo ${ed.undoLabel} (Ctrl+Z)` : "Nothing to undo";
  redoBtn.title = ed.redoLabel ? `Redo ${ed.redoLabel} (Ctrl+Y)` : "Nothing to redo";
  const toml = mecanumConfig(ed.model);
  const tomlBtn = /** @type {HTMLButtonElement} */ ($("btn-export-toml"));
  tomlBtn.disabled = !toml.ok;
  tomlBtn.title = toml.ok ? "mecanum-kinematics config measured from the wheels" : toml.reason;
  const x = exportXacro(ed.doc);
  $("btn-export-xacro").title = x.parametrised
    ? "Parametrised xacro (template parameters as properties and arguments)"
    : "Not parametrised: the robot is not exactly a template's output";
  const pkg = /** @type {HTMLInputElement} */ ($("package-name"));
  if (!packageNameEdited)
    pkg.value = `${
      ed.doc.name
        .toLowerCase()
        .replace(/[^a-z0-9_]/g, "_")
        .replace(/^[^a-z]+/, "") || "robot"
    }_description`;
  if (ed.doc.template && templateSel.value !== ed.doc.template.id)
    templateSel.value = ed.doc.template.id;
  renderTemplateUi();
}

// ---------------------------------------------------------------- exports

/**
 * @param {string} name
 * @param {string | Uint8Array} data
 * @param {string} type
 */
function download(name, data, type) {
  const url = URL.createObjectURL(new Blob([/** @type {BlobPart} */ (data)], { type }));
  const a = h("a", { href: url, download: name });
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 10000);
  const size = typeof data === "string" ? new TextEncoder().encode(data).length : data.length;
  $("export-note").textContent = `Exported ${name} (${size} bytes).`;
  document.body.dataset.lastExport = name;
}
$("btn-export-urdf").addEventListener("click", () => {
  if (editor) download(`${editor.doc.name}.urdf`, toUrdf(editor.doc), "application/xml");
});
$("btn-export-xacro").addEventListener("click", () => {
  if (!editor) return;
  const x = exportXacro(editor.doc);
  download(`${editor.doc.name}.urdf.xacro`, x.text, "application/xml");
  if (!x.parametrised)
    $("export-note").textContent +=
      " Not parametrised: the robot is not exactly a template's output.";
});
$("btn-export-toml").addEventListener("click", () => {
  if (!editor) return;
  const t = mecanumConfig(editor.model);
  if (t.ok) download(`${editor.doc.name}.toml`, t.toml, "application/toml");
  else $("export-note").textContent = t.reason;
});
$("btn-export-package").addEventListener("click", async () => {
  if (!editor) return;
  const name = /** @type {HTMLInputElement} */ ($("package-name")).value.trim();
  if (!PACKAGE_NAME_RE.test(name)) {
    $("export-note").textContent =
      `'${name}' is not a ROS package name (lower case letters, digits and _, starting with a letter).`;
    return;
  }
  const p = await packageZip(editor.doc, { packageName: name, meshFs: editMeshFs });
  download(`${p.name}.zip`, p.zip, "application/zip");
  if (p.notes.length) $("export-note").textContent += ` ${p.notes.join(" ")}`;
});

// ---------------------------------------------------------------- autosave (this browser only)

function autosave() {
  if (!editor) return;
  try {
    localStorage.setItem(AUTOSAVE_KEY, encodeAutosave(editor.doc));
  } catch {
    // Storage may be full, disabled or blocked; editing works without it.
  }
}

function readAutosave() {
  try {
    return decodeAutosave(localStorage.getItem(AUTOSAVE_KEY));
  } catch {
    return null;
  }
}

function restoreAutosave() {
  const saved = readAutosave();
  if (!saved) return false;
  startEditing(
    saved.doc,
    null,
    `restored '${saved.doc.name}' autosaved in this browser at ${new Date(saved.savedAt).toLocaleString()}${saved.doc.source ? " (mesh files are not stored; open the package again to see them)" : ""}`,
  );
  return true;
}
$("btn-restore").addEventListener("click", restoreAutosave);

// ---------------------------------------------------------------- simulation
simUi = initSim({
  view,
  state,
  saveUrl,
  robot() {
    if (editor)
      return { model: editor.model, doc: editor.doc, label: `builder robot '${editor.doc.name}'` };
    if (!model?.urdf) return null;
    if (viewerDoc?.model !== model) {
      const imp = importUrdf(model.urdf, { entry: model.entry });
      viewerDoc = { model, doc: imp.doc };
    }
    return { model, doc: viewerDoc.doc, label: sourceLabel };
  },
  docSim: () => editor?.doc.sim ?? null,
  storeDocSim(sim) {
    // Kept in the document (autosaved, undoable) without redrawing the robot.
    if (!editor || deepEqual(editor.doc.sim ?? null, sim)) return;
    editor.apply({ type: "setSimulation", sim });
    $("edit-status").textContent = "simulation settings";
    updateEditUi();
    autosave();
  },
});

// ---------------------------------------------------------------- ROS 2
rosUi = initRos({
  view,
  state,
  saveUrl,
  robot: () => simUi?.hostRobot() ?? null,
  openUrdf(text, label) {
    const f = new EmptyFs();
    f.addText("robot_description/robot.urdf", text);
    openFiles(f, label);
  },
  sim: /** @type {ReturnType<typeof initSim>} */ (simUi),
});

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
} else if (state.source === "builder") {
  if (!restoreAutosave()) {
    state.source = "mecanum";
    void openExample(state.source);
  }
} else void openExample(state.source);
updateEditUi();
