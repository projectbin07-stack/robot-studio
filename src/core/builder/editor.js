// The editing session: the current document, undo/redo history and the
// checks. Every applied operation re-runs urdf-inspector's checks on the
// document's URDF through the same loadModel() the viewer uses, so the
// builder shows exactly the findings an opened file would get.

import { loadModel } from "../model.js";
import { applyOp, describeOp } from "./ops.js";
import { toUrdf } from "./urdf.js";

/** @typedef {import("./document.js").RobotDoc} RobotDoc */
/** @typedef {import("./ops.js").EditOp} EditOp */
/** @typedef {import("../model.js").LoadedModel} LoadedModel */
/** @typedef {import("../model.js").ModelFs} ModelFs */

/** Undo steps kept; older ones are dropped. */
export const HISTORY_LIMIT = 200;

/**
 * Check a document as if its URDF had been opened. Mesh references
 * resolve through `meshFs` (the files an imported model came from), from
 * the imported file's folder.
 * @param {RobotDoc} doc
 * @param {ModelFs | null} [meshFs]
 * @returns {LoadedModel}
 */
export function modelFromDoc(doc, meshFs = null) {
  const entry = doc.source?.entry ?? `/builder/${doc.name || "robot"}.urdf`;
  const urdf = toUrdf(doc);
  /** @type {ModelFs} */
  const fs = {
    readText: (p) => (p === entry ? urdf : (meshFs?.readText(p) ?? null)),
    findPackage: (n) => meshFs?.findPackage(n) ?? null,
    notes: [],
  };
  const resolve = meshFs?.resolveUri;
  if (resolve) fs.resolveUri = (uri, base) => resolve.call(meshFs, uri, base);
  else
    fs.resolveUri = () => ({
      path: null,
      reason: "the builder has no files for it (meshes come from an opened package)",
    });
  return loadModel(fs, entry);
}

export class Editor {
  /**
   * @param {RobotDoc} doc
   * @param {ModelFs | null} [meshFs]
   */
  constructor(doc, meshFs = null) {
    /** @type {RobotDoc} */
    this.doc = applyOp(doc, { type: "replaceDocument", doc });
    this.meshFs = meshFs;
    /** @type {{ doc: RobotDoc, label: string }[]} */
    this.past = [];
    /** @type {{ doc: RobotDoc, label: string }[]} */
    this.future = [];
    /** Increases with every change (edit, undo, redo). */
    this.version = 0;
    /** @type {LoadedModel} */
    this.model = modelFromDoc(this.doc, meshFs);
  }

  /**
   * Apply an operation; on EditError nothing changes and the error propagates.
   * @param {EditOp} op
   * @returns {LoadedModel}
   */
  apply(op) {
    const next = applyOp(this.doc, op);
    this.past.push({ doc: this.doc, label: describeOp(op) });
    if (this.past.length > HISTORY_LIMIT) this.past.shift();
    this.future = [];
    return this.set(next);
  }

  get canUndo() {
    return this.past.length > 0;
  }
  get canRedo() {
    return this.future.length > 0;
  }
  /** Label of the step undo would revert, or null. */
  get undoLabel() {
    return this.past.at(-1)?.label ?? null;
  }
  get redoLabel() {
    return this.future.at(-1)?.label ?? null;
  }

  /** @returns {boolean} whether anything was undone */
  undo() {
    const step = this.past.pop();
    if (!step) return false;
    this.future.push({ doc: this.doc, label: step.label });
    this.set(step.doc);
    return true;
  }

  /** @returns {boolean} whether anything was redone */
  redo() {
    const step = this.future.pop();
    if (!step) return false;
    this.past.push({ doc: this.doc, label: step.label });
    this.set(step.doc);
    return true;
  }

  /** @param {RobotDoc} doc */
  set(doc) {
    this.doc = doc;
    this.version++;
    this.model = modelFromDoc(doc, this.meshFs);
    return this.model;
  }
}
