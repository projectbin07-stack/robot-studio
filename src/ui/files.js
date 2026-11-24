// Turn what the user drops or picks into a virtual file system: single
// files, whole folders (DataTransfer entries or <input webkitdirectory>)
// and .zip archives. Text files are read immediately; meshes and images
// when the 3D view needs them. Nothing leaves the browser.

import { VirtualFs, addZip, isTextFile } from "../core/vfs.js";

/** Files larger than this are not read as text up front (they are still listed). */
const TEXT_LIMIT = 8 * 1024 * 1024;

/**
 * @param {VirtualFs} fs
 * @param {File} file
 * @param {string} path
 */
async function addFile(fs, file, path) {
  if (/\.zip$/i.test(file.name)) {
    await addZip(fs, new Uint8Array(await file.arrayBuffer()), "/");
    return;
  }
  if (isTextFile(path) && file.size <= TEXT_LIMIT) fs.addText(path, await file.text());
  else fs.addLazy(path, async () => new Uint8Array(await file.arrayBuffer()), file.size);
}

/**
 * From an <input type="file"> (with or without webkitdirectory).
 * @param {FileList} list
 * @returns {Promise<VirtualFs>}
 */
export async function fsFromFileList(list) {
  const fs = new VirtualFs();
  for (const f of Array.from(list)) {
    const rel = f.webkitRelativePath || f.name;
    await addFile(fs, f, `/${rel}`);
  }
  return fs;
}

/**
 * From a drop event: folders are walked recursively.
 * @param {DataTransfer} dt
 * @returns {Promise<VirtualFs>}
 */
export async function fsFromDrop(dt) {
  const fs = new VirtualFs();
  const entries = Array.from(dt.items)
    .map((it) => (typeof it.webkitGetAsEntry === "function" ? it.webkitGetAsEntry() : null))
    .filter((e) => e !== null);
  if (entries.length === 0) {
    for (const f of Array.from(dt.files)) await addFile(fs, f, `/${f.name}`);
    return fs;
  }
  for (const e of entries) await walk(fs, /** @type {FileSystemEntry} */ (e));
  return fs;
}

/**
 * @param {VirtualFs} fs
 * @param {FileSystemEntry} entry
 */
async function walk(fs, entry) {
  if (entry.isFile) {
    const file = await new Promise((resolve, reject) =>
      /** @type {FileSystemFileEntry} */ (entry).file(resolve, reject),
    );
    await addFile(fs, /** @type {File} */ (file), entry.fullPath);
    return;
  }
  if (/(^|\/)(\.git|node_modules|build|install|log)$/.test(entry.fullPath)) return;
  const reader = /** @type {FileSystemDirectoryEntry} */ (entry).createReader();
  for (;;) {
    /** @type {FileSystemEntry[]} */
    const batch = await new Promise((resolve, reject) => reader.readEntries(resolve, reject));
    if (batch.length === 0) break;
    for (const e of batch) await walk(fs, e);
  }
}
