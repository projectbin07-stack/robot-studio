// Mesh files from the virtual file system as three.js objects: STL, OBJ
// and COLLADA (DAE, with textures from the same file system). COLLADA
// <up_axis> is ignored and <unit> applied, as RViz does (rviz_rendering's
// assimp_loader.cpp sets AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION); see
// docs/decisions.md.

import * as THREE from "three";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { dirname, extname, join } from "../core/paths.js";

/** @typedef {import("../core/vfs.js").VirtualFs} VirtualFs */

/**
 * @typedef {object} LoadedMesh
 * @property {THREE.Object3D} object
 * @property {boolean} ownMaterials true when the file brought its own materials (DAE)
 * @property {string[]} missing textures the file refers to that are not among the loaded files
 */

/** A 1x1 flat-normal pixel (128, 128, 255) served for textures that are missing, so nothing is fetched from the network. */
const NEUTRAL_PNG =
  "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAIAAACQd1PeAAAADElEQVR4nGNoaPgPAAODAgAApfuJAAAAAElFTkSuQmCC";

export const MESH_EXTENSIONS = ["stl", "obj", "dae"];

/**
 * Load one mesh file. Rejects with a readable message when the file cannot
 * be read or parsed.
 * @param {VirtualFs} fs
 * @param {string} path resolved path in the file system
 * @returns {Promise<LoadedMesh>}
 */
export async function loadMesh(fs, path) {
  const ext = extname(path);
  if (!MESH_EXTENSIONS.includes(ext))
    throw new Error(`.${ext} meshes are not supported (STL, OBJ and DAE are)`);
  const bytes = await fs.readBinary(path);
  if (!bytes) throw new Error(`${path} could not be read`);
  if (ext === "stl") {
    const geom = new STLLoader().parse(bytes.slice().buffer);
    geom.computeVertexNormals();
    return { object: new THREE.Mesh(geom), ownMaterials: false, missing: [] };
  }
  const text = new TextDecoder().decode(bytes);
  if (ext === "obj") {
    const group = new OBJLoader().parse(text);
    let meshes = 0;
    group.traverse((o) => {
      if (o instanceof THREE.Mesh) meshes++;
    });
    if (meshes === 0) throw new Error(`${path} contains no faces`);
    return { object: group, ownMaterials: false, missing: [] };
  }
  // COLLADA: serve its textures from the file system through blob URLs.
  const base = dirname(path);
  /** @type {Map<string, string>} */
  const blobs = new Map();
  /** @type {string[]} */
  const missing = [];
  for (const m of text.matchAll(/<init_from>\s*([^<\s]+)\s*<\/init_from>/g)) {
    const rel = decodeURIComponent(m[1].replace(/^file:\/\//, ""));
    if (!/\.(png|jpe?g|gif|bmp|webp|tga)$/i.test(rel)) continue;
    const img = await fs.readBinary(join(base, rel));
    if (img) blobs.set(rel, URL.createObjectURL(new Blob([/** @type {BlobPart} */ (img)])));
    else if (!missing.includes(rel)) missing.push(rel);
  }
  const manager = new THREE.LoadingManager();
  manager.setURLModifier((url) => {
    const rel = url.startsWith("vfs:") ? url.slice(4) : url;
    return blobs.get(rel) ?? blobs.get(decodeURIComponent(rel)) ?? NEUTRAL_PNG;
  });
  const collada = new ColladaLoader(manager).parse(text, "vfs:");
  if (!collada || !collada.scene)
    throw new Error(`${path} is not a COLLADA file three.js can read`);
  collada.scene.rotation.set(0, 0, 0);
  return { object: collada.scene, ownMaterials: true, missing };
}
