// The three.js view: robot shapes and meshes posed by forward kinematics,
// grid, lights, link frames, joint axes, centre of mass, visual/collision
// switch and click selection. World frame is the model's root frame with
// Z up (REP 103). Renders on demand.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { DEFAULT_RGBA, drawShapes, posesFor, roughBounds } from "../core/scene.js";
import { loadMesh } from "./meshes.js";

/** @typedef {import("../core/vfs.js").VirtualFs} VirtualFs */
/** @typedef {import("../core/model.js").LoadedModel} LoadedModel */
/** @typedef {import("../core/scene.js").DrawShape} DrawShape */

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

/**
 * @typedef {object} ViewOptions
 * @property {"visual" | "collision" | "both"} geometry
 * @property {boolean} frames
 * @property {boolean} axes
 * @property {boolean} com
 */

/**
 * @typedef {object} MeshReport
 * @property {string} link
 * @property {string} filename
 * @property {string} message
 * @property {"warning" | "info"} [severity]
 */

const SELECT_COLOR = new THREE.Color(0xff8a1f);

export class RobotView {
  /**
   * @param {HTMLElement} host
   * @param {(link: string | null) => void} onPick
   */
  constructor(host, onPick) {
    this.host = host;
    this.onPick = onPick;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    host.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.005, 200);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(1.2, -1.4, 0.9);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.addEventListener("change", () => this.render());

    this.hemi = new THREE.HemisphereLight(0xffffff, 0x444a55, 1.6);
    this.hemi.position.set(0, 0, 1);
    this.key = new THREE.DirectionalLight(0xffffff, 2.2);
    this.key.castShadow = true;
    this.key.shadow.mapSize.set(2048, 2048);
    this.key.shadow.bias = -0.0005;
    this.key.shadow.normalBias = 0.01;
    this.fill = new THREE.DirectionalLight(0xdde6ff, 0.7);
    this.scene.add(this.hemi, this.key, this.key.target, this.fill);

    this.ground = new THREE.Mesh(
      new THREE.PlaneGeometry(1, 1),
      new THREE.ShadowMaterial({ opacity: 0.22 }),
    );
    this.ground.receiveShadow = true;
    this.scene.add(this.ground);
    /** @type {THREE.GridHelper | null} */
    this.grid = null;
    this.worldAxes = new THREE.AxesHelper(0.1);
    this.scene.add(this.worldAxes);

    this.robot = new THREE.Group();
    this.overlay = new THREE.Group();
    this.scene.add(this.robot, this.overlay);
    /** @type {Map<string, THREE.Group>} */
    this.linkGroups = new Map();
    /** @type {THREE.Group[]} */
    this.frameHelpers = [];
    /** @type {THREE.Group[]} */
    this.jointHelpers = [];
    this.comMarker = new THREE.Group();
    this.overlay.add(this.comMarker);
    /** @type {string | null} */
    this.selected = null;
    this.size = 0.5;
    /** @type {ViewOptions} */
    this.opts = { geometry: "visual", frames: false, axes: false, com: false };
    this.dark = false;
    /** @type {import("../../vendor/urdf-inspector/src/core/tree.js").KinTree | null} */
    this.tree = null;
    this.generation = 0;
    /** Parsed meshes per file system and path, reused when the builder redraws. */
    /** @type {WeakMap<VirtualFs, Map<string, Promise<import("./meshes.js").LoadedMesh>>>} */
    this.meshCache = new WeakMap();
    /** Floor extent while a simulation world is shown: { x0, y0, x1, y1 } in metres. */
    /** @type {{ x0: number, y0: number, x1: number, y1: number } | null} */
    this.floor = null;
    /** While true, render() does nothing (the simulation draws once per frame). */
    this.holdRender = false;
    /** Bounding boxes of loaded meshes in their file coordinates, by "link|kind|index". */
    /** @type {Map<string, { min: [number, number, number], max: [number, number, number] }>} */
    this.meshBounds = new Map();

    const ro = new ResizeObserver(() => this.resize());
    ro.observe(host);
    this.installPicking();
    this.resize();
  }

  resize() {
    const w = Math.max(1, this.host.clientWidth);
    const h = Math.max(1, this.host.clientHeight);
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.render();
  }

  render() {
    if (this.holdRender) return;
    this.renderer.render(this.scene, this.camera);
  }

  /** @param {boolean} dark */
  setTheme(dark) {
    this.dark = dark;
    this.scene.background = new THREE.Color(dark ? 0x15181d : 0xf3f5f8);
    this.hemi.groundColor = new THREE.Color(dark ? 0x30343c : 0x8a909a);
    /** @type {THREE.ShadowMaterial} */ (this.ground.material).opacity = dark ? 0.35 : 0.2;
    this.rebuildGrid();
    this.render();
  }

  rebuildGrid() {
    if (this.grid) this.scene.remove(this.grid);
    if (this.floor) {
      // A 0.5 m grid over the simulated world.
      const f = this.floor;
      const extent = Math.ceil(Math.max(f.x1 - f.x0, f.y1 - f.y0) + 2);
      this.grid = new THREE.GridHelper(
        extent,
        extent * 2,
        this.dark ? 0x59606b : 0xa9b0ba,
        this.dark ? 0x2b3038 : 0xdde1e6,
      );
      this.grid.rotation.x = Math.PI / 2;
      this.grid.position.set((f.x0 + f.x1) / 2, (f.y0 + f.y1) / 2, -0.0005);
      this.scene.add(this.grid);
      this.ground.scale.set(extent, extent, 1);
      this.ground.position.set((f.x0 + f.x1) / 2, (f.y0 + f.y1) / 2, 0);
      return;
    }
    const step = 10 ** Math.floor(Math.log10(this.size / 2));
    const divisions = Math.min(200, Math.max(10, Math.ceil((this.size * 6) / step)));
    const extent = divisions * step;
    this.grid = new THREE.GridHelper(
      extent,
      divisions,
      this.dark ? 0x59606b : 0xa9b0ba,
      this.dark ? 0x2b3038 : 0xdde1e6,
    );
    this.grid.rotation.x = Math.PI / 2;
    this.grid.position.z = -0.0005;
    this.scene.add(this.grid);
    this.ground.scale.set(extent, extent, 1);
  }

  /**
   * Show a model. Meshes load in the background; the returned promise
   * resolves with problems found while loading them.
   * @param {LoadedModel} model
   * @param {VirtualFs} fs
   * @param {Record<string, number>} q
   * @param {{ keepCamera?: boolean }} [opts] keepCamera: redraw without moving the camera (builder edits)
   * @returns {Promise<MeshReport[]>}
   */
  async show(model, fs, q, opts = {}) {
    const gen = ++this.generation;
    this.clear();
    this.meshBounds.clear();
    const robot = model.report?.robot;
    const tree = model.report?.context?.tree ?? null;
    this.tree = tree;
    if (!robot || !tree) {
      this.render();
      return [];
    }
    for (const name of tree.order) {
      const g = new THREE.Group();
      g.name = name;
      g.matrixAutoUpdate = false;
      g.userData.link = name;
      this.linkGroups.set(name, g);
      this.robot.add(g);
    }
    const shapes = drawShapes(robot);
    /** @type {Promise<MeshReport | null>[]} */
    const pending = [];
    for (const s of shapes) {
      const g = this.linkGroups.get(s.link);
      if (!g) continue;
      const holder = new THREE.Group();
      holder.matrixAutoUpdate = false;
      holder.matrix.fromArray(s.local);
      holder.userData = { link: s.link, kind: s.kind };
      g.add(holder);
      const mat = this.material(s);
      if (s.geometry.type === "mesh") {
        const ref = model.meshes.find(
          (m) => m.link === s.link && m.kind === s.kind && m.index === s.index,
        );
        const geom = s.geometry;
        const placeholder = this.placeholder();
        holder.add(placeholder);
        if (!ref?.resolved) continue;
        const path = ref.resolved;
        let cache = this.meshCache.get(fs);
        if (!cache) this.meshCache.set(fs, (cache = new Map()));
        let loading = cache.get(path);
        if (!loading) cache.set(path, (loading = loadMesh(fs, path)));
        pending.push(
          loading.then(
            ({ object: shared, ownMaterials, missing }) => {
              if (gen !== this.generation) return null;
              holder.remove(placeholder);
              const object = shared.clone();
              const box = new THREE.Box3().setFromObject(object);
              if (!box.isEmpty())
                this.meshBounds.set(`${s.link}|${s.kind}|${s.index}`, {
                  min: [box.min.x, box.min.y, box.min.z],
                  max: [box.max.x, box.max.y, box.max.z],
                });
              object.scale.multiply(new THREE.Vector3(...geom.scale));
              object.traverse((o) => {
                if (!(o instanceof THREE.Mesh)) return;
                o.castShadow = s.kind === "visual";
                o.receiveShadow = s.kind === "visual";
                if (s.kind === "collision" || !ownMaterials || s.rgba) o.material = mat;
                else
                  o.material = Array.isArray(o.material)
                    ? o.material.map((m) => m.clone())
                    : o.material.clone();
              });
              holder.add(object);
              this.applyOptions();
              if (missing.length === 0) return null;
              return {
                link: s.link,
                filename: geom.filename,
                message: `texture(s) ${missing.join(", ")} not among the loaded files; drawn without them`,
                severity: "info",
              };
            },
            (err) => ({
              link: s.link,
              filename: geom.filename,
              message: err instanceof Error ? err.message : String(err),
            }),
          ),
        );
        continue;
      }
      const mesh = new THREE.Mesh(primitive(s.geometry), mat);
      mesh.castShadow = s.kind === "visual";
      mesh.receiveShadow = s.kind === "visual";
      holder.add(mesh);
    }
    const poses = posesFor(tree, q);
    const b = roughBounds(poses.links, shapes);
    this.size = b.radius;
    this.buildHelpers(tree);
    this.setPose(q);
    if (!opts.keepCamera) {
      this.frame(b.center, b.radius);
      this.rebuildGrid();
    }
    this.applyOptions();
    const reports = await Promise.all(pending);
    if (gen !== this.generation) return [];
    if (!opts.keepCamera) this.fit();
    else this.render();
    return /** @type {MeshReport[]} */ (reports.filter((r) => r !== null));
  }

  clear() {
    for (const child of [...this.robot.children]) this.robot.remove(child);
    for (const h of [...this.frameHelpers, ...this.jointHelpers]) this.overlay.remove(h);
    this.linkGroups.clear();
    this.frameHelpers = [];
    this.jointHelpers = [];
    this.comMarker.clear();
    this.selected = null;
  }

  /** @param {DrawShape} s */
  material(s) {
    if (s.kind === "collision")
      return new THREE.MeshStandardMaterial({
        color: 0x3fa7ff,
        transparent: true,
        opacity: 0.45,
        roughness: 0.8,
        depthWrite: false,
      });
    const [r, g, b, a] = s.rgba ?? DEFAULT_RGBA;
    return new THREE.MeshStandardMaterial({
      color: new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace),
      transparent: a < 1,
      opacity: a,
      roughness: 0.55,
      metalness: 0.15,
    });
  }

  placeholder() {
    const m = new THREE.Mesh(
      new THREE.IcosahedronGeometry(Math.max(0.01, this.size * 0.04), 0),
      new THREE.MeshBasicMaterial({ color: 0xe0308a, wireframe: true }),
    );
    m.userData.placeholder = true;
    return m;
  }

  /** @param {import("../../vendor/urdf-inspector/src/core/tree.js").KinTree} tree */
  buildHelpers(tree) {
    const len = Math.max(0.02, this.size * 0.18);
    for (const name of tree.order) {
      const g = new THREE.Group();
      g.matrixAutoUpdate = false;
      g.userData.link = name;
      g.add(new THREE.AxesHelper(len));
      this.frameHelpers.push(g);
      this.overlay.add(g);
    }
    const ring = new THREE.MeshBasicMaterial({ color: 0xffc23d, depthTest: false });
    const markers = posesFor(tree, {}).joints;
    for (const m of markers) {
      const g = new THREE.Group();
      g.matrixAutoUpdate = false;
      g.userData.joint = m.name;
      const dir = new THREE.Vector3(...m.axis);
      const arrow = new THREE.ArrowHelper(
        dir,
        new THREE.Vector3(),
        len * 1.3,
        0xffb020,
        len * 0.3,
        len * 0.15,
      );
      g.add(arrow);
      if (m.type !== "prismatic") {
        const torus = new THREE.Mesh(new THREE.TorusGeometry(len * 0.35, len * 0.025, 6, 32), ring);
        torus.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir);
        g.add(torus);
      }
      g.renderOrder = 10;
      this.jointHelpers.push(g);
      this.overlay.add(g);
    }
    const r = Math.max(0.006, this.size * 0.035);
    const ball = new THREE.Mesh(
      new THREE.SphereGeometry(r, 24, 16),
      new THREE.MeshBasicMaterial({
        color: 0x9b5cff,
        depthTest: false,
        transparent: true,
        opacity: 0.9,
      }),
    );
    const cross = new THREE.AxesHelper(r * 2.2);
    ball.renderOrder = 11;
    this.comMarker.add(ball, cross);
  }

  /**
   * Pose every link for joint positions q.
   * @param {Record<string, number>} q
   */
  setPose(q) {
    if (!this.tree) return;
    const p = posesFor(this.tree, q);
    for (const [name, m] of p.links) this.linkGroups.get(name)?.matrix.fromArray(m);
    for (const g of this.frameHelpers) {
      const m = p.links.get(g.userData.link);
      if (m) g.matrix.fromArray(m);
    }
    for (const g of this.jointHelpers) {
      const m = p.joints.find((j) => j.name === g.userData.joint);
      if (m) g.matrix.fromArray(m.origin);
    }
    if (p.com) this.comMarker.position.set(p.com[0], p.com[1], p.com[2]);
    this.comMarker.visible = this.opts.com && p.com !== null;
    this.render();
    return p;
  }

  /**
   * Place the model's root frame in the world (simulation), or back at the
   * origin with null. `m` is a column-major 4x4 matrix.
   * @param {number[] | null} m
   */
  setBase(m) {
    for (const g of [this.robot, this.overlay]) {
      g.matrixAutoUpdate = false;
      if (m) g.matrix.fromArray(m);
      else g.matrix.identity();
      g.matrixWorldNeedsUpdate = true;
    }
  }

  /**
   * Show a floor and grid for a simulated world (null: back to the model's own grid).
   * @param {{ x0: number, y0: number, x1: number, y1: number } | null} f
   */
  setFloor(f) {
    this.floor = f;
    this.rebuildGrid();
    this.render();
  }

  /**
   * Keep the shadow-casting light over a moving robot.
   * @param {number} x
   * @param {number} y
   */
  followLight(x, y) {
    const r = Math.max(this.size, 0.5);
    const s = Math.max(r * 3, 0.5);
    this.key.position.set(x + s * 0.6, y - s * 0.4, s * 1.4);
    this.key.target.position.set(x, y, 0);
    const cam = this.key.shadow.camera;
    cam.left = cam.bottom = -r * 2;
    cam.right = cam.top = r * 2;
    cam.far = s * 4;
    cam.updateProjectionMatrix();
  }

  /** Frame the camera on everything drawn (meshes included). */
  fit() {
    const box = new THREE.Box3().setFromObject(this.robot);
    if (box.isEmpty()) return this.render();
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    this.size = Math.max(0.05, sphere.radius);
    this.frame([sphere.center.x, sphere.center.y, sphere.center.z], this.size * 1.3);
    this.rebuildGrid();
    this.render();
  }

  /**
   * @param {[number, number, number]} center
   * @param {number} radius
   */
  frame(center, radius) {
    const c = new THREE.Vector3(...center);
    const dist = (radius / Math.sin((this.camera.fov * Math.PI) / 360)) * 0.9;
    const dir = new THREE.Vector3(1.0, -1.25, 0.8).normalize();
    this.camera.position.copy(c).addScaledVector(dir, dist);
    this.camera.near = Math.max(0.001, dist / 200);
    this.camera.far = dist * 50;
    this.camera.updateProjectionMatrix();
    this.controls.target.copy(c);
    this.controls.update();
    const s = Math.max(radius * 3, 0.5);
    this.key.position.set(c.x + s * 0.6, c.y - s * 0.4, c.z + s * 1.4);
    this.key.target.position.copy(c);
    const cam = this.key.shadow.camera;
    cam.left = cam.bottom = -radius * 1.6;
    cam.right = cam.top = radius * 1.6;
    cam.near = 0.01;
    cam.far = s * 4;
    cam.updateProjectionMatrix();
    this.fill.position.set(c.x - s, c.y + s * 0.8, c.z + s * 0.5);
    this.worldAxes.scale.setScalar(Math.max(0.05, radius * 0.25) / 0.1);
    this.ground.position.set(c.x, c.y, 0);
    this.render();
  }

  /** @param {Partial<ViewOptions>} [opts] */
  applyOptions(opts) {
    if (opts) this.opts = { ...this.opts, ...opts };
    const g = this.opts.geometry;
    this.robot.traverse((o) => {
      if (o.userData.kind === "visual") o.visible = g !== "collision";
      if (o.userData.kind === "collision") o.visible = g !== "visual";
    });
    for (const h of this.frameHelpers) h.visible = this.opts.frames;
    for (const h of this.jointHelpers) h.visible = this.opts.axes;
    this.comMarker.visible = this.opts.com && this.comMarker.children.length > 0;
    this.highlight(this.selected);
    this.render();
  }

  /**
   * Highlight a link (emissive tint on its visual and collision materials).
   * @param {string | null} link
   */
  highlight(link) {
    this.selected = link;
    for (const [name, g] of this.linkGroups)
      g.traverse((o) => {
        if (!(o instanceof THREE.Mesh) || o.userData.placeholder) return;
        const mats = Array.isArray(o.material) ? o.material : [o.material];
        for (const m of mats)
          if (
            m instanceof THREE.MeshStandardMaterial ||
            m instanceof THREE.MeshPhongMaterial ||
            m instanceof THREE.MeshLambertMaterial
          )
            m.emissive =
              name === link ? SELECT_COLOR.clone().multiplyScalar(0.55) : new THREE.Color(0, 0, 0);
      });
    for (const h of this.frameHelpers) h.visible = this.opts.frames || h.userData.link === link;
    this.render();
  }

  installPicking() {
    const el = this.renderer.domElement;
    let down = { x: 0, y: 0 };
    el.addEventListener("pointerdown", (e) => (down = { x: e.clientX, y: e.clientY }));
    el.addEventListener("pointerup", (e) => {
      if (Math.hypot(e.clientX - down.x, e.clientY - down.y) > 4) return;
      const rect = el.getBoundingClientRect();
      const ndc = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1,
      );
      const ray = new THREE.Raycaster();
      ray.setFromCamera(ndc, this.camera);
      const hits = ray
        .intersectObjects(this.robot.children, true)
        .filter((h) => isVisible(h.object));
      let link = null;
      for (let o = /** @type {THREE.Object3D | null} */ (hits[0]?.object ?? null); o; o = o.parent)
        if (o.userData.link && this.linkGroups.has(o.userData.link)) {
          link = o.userData.link;
          break;
        }
      this.onPick(link);
    });
  }
}

/** @param {THREE.Object3D} o */
function isVisible(o) {
  for (let p = /** @type {THREE.Object3D | null} */ (o); p; p = p.parent)
    if (!p.visible) return false;
  return true;
}

/**
 * three.js geometry of a URDF primitive (URDF cylinders lie along z).
 * @param {import("../../vendor/urdf-inspector/src/core/urdf.js").Geometry} g
 * @returns {THREE.BufferGeometry}
 */
function primitive(g) {
  switch (g.type) {
    case "box":
      return new THREE.BoxGeometry(...g.size);
    case "cylinder":
      return new THREE.CylinderGeometry(g.radius, g.radius, g.length, 40).rotateX(Math.PI / 2);
    case "sphere":
      return new THREE.SphereGeometry(g.radius, 32, 20);
    default:
      return new THREE.BufferGeometry();
  }
}
