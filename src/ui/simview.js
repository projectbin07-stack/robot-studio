// The simulation drawn into the 3D view: the world (walls, boxes and
// cylinders at their heights), LiDAR rays and returns from the LiDAR's
// URDF frame, the occupancy map built from the estimate, trajectories of
// the truth and of each estimate, MCL particles, the depth point cloud and
// the map / odom frames. The robot itself is the view's model, placed by
// RobotView.setBase and turned by the wheel joint positions.

import * as THREE from "three";
import { compose } from "../core/sim/se2.js";
import { matrix4 } from "../core/scene.js";
import { compose as compose3, fromXyzRpy } from "../../vendor/urdf-inspector/src/core/math.js";

/** @typedef {import("../core/sim/world.js").SimWorld} SimWorld */
/** @typedef {import("../core/sim/session.js").SimSession} SimSession */
/** @typedef {import("../core/sim/se2.js").Pose2} Pose2 */
/** @typedef {import("./view3d.js").RobotView} RobotView */
/** @typedef {import("../core/ros/track.js").ViewFrame} ViewFrame */

/** Trajectory colours: truth, wheel odometry, + scan matching, MCL. */
export const TRAIL_COLORS = { truth: 0x2e9d5b, odom: 0xd8453b, est: 0x2a78d6, mcl: 0x9b5cff };
const MAX_TRAIL = 6000;

/**
 * @typedef {object} LayerOptions
 * @property {boolean} rays
 * @property {boolean} map
 * @property {boolean} trails
 * @property {boolean} particles
 * @property {boolean} cloud
 * @property {boolean} tf
 */

export class SimLayer {
  /** @param {RobotView} view */
  constructor(view) {
    this.view = view;
    this.group = new THREE.Group();
    this.group.name = "simulation";
    view.scene.add(this.group);
    this.worldGroup = new THREE.Group();
    this.group.add(this.worldGroup);

    // LiDAR rays and returns
    this.rayGeom = new THREE.BufferGeometry();
    this.rays = new THREE.LineSegments(
      this.rayGeom,
      new THREE.LineBasicMaterial({ color: 0xff3d3d, transparent: true, opacity: 0.12 }),
    );
    this.hitGeom = new THREE.BufferGeometry();
    this.hits = new THREE.Points(
      this.hitGeom,
      new THREE.PointsMaterial({ color: 0xe8202a, size: 0.06, sizeAttenuation: true }),
    );
    // Trails
    /** @type {Record<string, { line: THREE.Line, pos: Float32Array, n: number }>} */
    this.trails = {};
    for (const [k, c] of Object.entries(TRAIL_COLORS)) {
      const pos = new Float32Array(MAX_TRAIL * 3);
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
      g.setDrawRange(0, 0);
      const line = new THREE.Line(g, new THREE.LineBasicMaterial({ color: c }));
      line.frustumCulled = false;
      this.trails[k] = { line, pos, n: 0 };
      this.group.add(line);
    }
    // Particles and depth cloud
    this.particleGeom = new THREE.BufferGeometry();
    this.particles = new THREE.Points(
      this.particleGeom,
      new THREE.PointsMaterial({ color: TRAIL_COLORS.mcl, size: 0.04 }),
    );
    this.cloudGeom = new THREE.BufferGeometry();
    this.cloud = new THREE.Points(
      this.cloudGeom,
      new THREE.PointsMaterial({ color: 0x00a3a3, size: 0.035 }),
    );
    for (const o of [this.rays, this.hits, this.particles, this.cloud]) o.frustumCulled = false;
    this.group.add(this.rays, this.hits, this.particles, this.cloud);
    // Occupancy map
    /** @type {THREE.Mesh | null} */
    this.mapMesh = null;
    /** @type {THREE.DataTexture | null} */
    this.mapTex = null;
    this.mapAt = -1;
    // Frames: map (start pose) and odom
    this.mapAxes = new THREE.AxesHelper(0.5);
    this.odomAxes = new THREE.AxesHelper(0.35);
    this.group.add(this.mapAxes, this.odomAxes);
    /** @type {LayerOptions} */
    this.opts = { rays: true, map: true, trails: true, particles: false, cloud: true, tf: false };
    this.lastTrailT = -1;
    /** @type {unknown} */
    this.lastScanDrawn = null;
    this.group.visible = false;
  }

  /** @param {boolean} on */
  setVisible(on) {
    this.group.visible = on;
    // No shadow pass while simulating: it costs most of a software-rendered
    // frame and the world's walls would need a large shadow map anyway.
    this.view.key.castShadow = !on;
    if (!on) {
      this.view.setBase(null);
      this.view.setFloor(null);
      // The camera and the key light followed the robot through the world;
      // bring both back to the model at the origin (the view was empty).
      this.view.followLight(0, 0);
      this.view.fit();
    }
  }

  /**
   * Build the world's meshes.
   * @param {SimWorld} world
   * @param {number} lidarHeight obstacles lower than this are tinted: the LiDAR cannot see them
   */
  setWorld(world, lidarHeight) {
    for (const c of [...this.worldGroup.children]) {
      this.worldGroup.remove(c);
      if (c instanceof THREE.Mesh) c.geometry.dispose();
    }
    const dark = this.view.dark;
    const wallMat = new THREE.MeshStandardMaterial({
      color: dark ? 0x7d8590 : 0xb9c0ca,
      roughness: 0.9,
      transparent: true,
      opacity: 0.88,
    });
    const tall = new THREE.MeshStandardMaterial({ color: 0x8b7355, roughness: 0.8 });
    const low = new THREE.MeshStandardMaterial({ color: 0xe8a33d, roughness: 0.8 });
    const T = 0.05;
    for (const [x1, y1, x2, y2] of world.walls) {
      const len = Math.hypot(x2 - x1, y2 - y1);
      const m = new THREE.Mesh(new THREE.BoxGeometry(len + T, T, world.wallHeight), wallMat);
      m.position.set((x1 + x2) / 2, (y1 + y2) / 2, world.wallHeight / 2);
      m.rotation.z = Math.atan2(y2 - y1, x2 - x1);
      m.castShadow = m.receiveShadow = true;
      this.worldGroup.add(m);
    }
    for (const b of world.boxes) {
      const m = new THREE.Mesh(
        new THREE.BoxGeometry(b.w, b.d, b.h),
        b.h > lidarHeight ? tall : low,
      );
      m.position.set(b.x, b.y, b.h / 2);
      m.rotation.z = b.yaw;
      m.castShadow = m.receiveShadow = true;
      this.worldGroup.add(m);
    }
    for (const c of world.cylinders) {
      const m = new THREE.Mesh(
        new THREE.CylinderGeometry(c.r, c.r, c.h, 32).rotateX(Math.PI / 2),
        c.h > lidarHeight ? tall : low,
      );
      m.position.set(c.x, c.y, c.h / 2);
      m.castShadow = m.receiveShadow = true;
      this.worldGroup.add(m);
    }
    this.view.setFloor({ x0: 0, y0: 0, x1: world.width, y1: world.height });
    this.mapAxes.position.set(world.start.x, world.start.y, 0.002);
    this.mapAxes.rotation.z = world.start.theta;
  }

  /**
   * No world (a live robot or a recording without one): only a floor of
   * `half` metres around the origin.
   * @param {number} half
   */
  clearWorld(half) {
    for (const c of [...this.worldGroup.children]) {
      this.worldGroup.remove(c);
      if (c instanceof THREE.Mesh) c.geometry.dispose();
    }
    this.view.setFloor({ x0: -half, y0: -half, x1: half, y1: half });
  }

  /** Forget trails and the map (after a reset). */
  clear() {
    for (const t of Object.values(this.trails)) {
      t.n = 0;
      t.line.geometry.setDrawRange(0, 0);
    }
    this.lastTrailT = -1;
    this.mapAt = -1;
  }

  /** @param {Partial<LayerOptions>} o */
  setOptions(o) {
    this.opts = { ...this.opts, ...o };
  }

  /**
   * Bring everything up to date with the simulation session.
   * @param {SimSession} session
   */
  update(session) {
    this.draw(session.frame(), session.sim.robot);
  }

  /**
   * Draw one frame from any source (simulation, replay, live robot).
   * @param {ViewFrame} f
   * @param {{ baseHeight: number, rootInBase: import("../../vendor/urdf-inspector/src/core/math.js").Transform }} robot
   */
  draw(f, robot) {
    const o = this.opts;
    const z = robot.baseHeight;
    // Robot: world pose of the root frame = base pose (lifted to its height) * root-in-base.
    const base = f.base;
    const T = compose3(fromXyzRpy([base.x, base.y, z], [0, 0, base.theta]), robot.rootInBase);
    this.view.setBase(matrix4(T));
    this.view.followLight(base.x, base.y);

    // LiDAR
    const scan = f.scan;
    const sp = f.scanPose;
    this.rays.visible = o.rays && !!scan;
    this.hits.visible = o.rays && !!scan;
    if (scan && sp && scan !== this.lastScanDrawn) {
      this.lastScanDrawn = scan;
      const h = f.lidarHeight;
      const n = scan.ranges.length;
      const rays = new Float32Array(n * 6);
      const hits = new Float32Array(n * 3);
      let k = 0;
      let m = 0;
      for (let i = 0; i < n; i++) {
        const r = scan.ranges[i];
        const a = sp.theta + scan.angles[i];
        const reach = Number.isNaN(r) ? 0.3 : r;
        const x = sp.x + reach * Math.cos(a);
        const y = sp.y + reach * Math.sin(a);
        rays.set([sp.x, sp.y, h, x, y, h], k);
        k += 6;
        if (!Number.isNaN(r)) {
          hits.set([x, y, h], m);
          m += 3;
        }
      }
      this.rayGeom.setAttribute("position", new THREE.BufferAttribute(rays, 3));
      this.hitGeom.setAttribute("position", new THREE.BufferAttribute(hits.subarray(0, m), 3));
    }

    // Trails every 0.1 s
    for (const t of Object.values(this.trails)) t.line.visible = o.trails;
    if (f.t - this.lastTrailT >= 0.1 - 1e-9 || f.t < this.lastTrailT) {
      this.lastTrailT = f.t;
      for (const [k, p] of Object.entries(f.trails)) if (p) this.pushTrail(this.trails[k], p);
    }

    // Particles
    const pf = f.particles;
    this.particles.visible = o.particles && !!pf;
    if (pf && o.particles) {
      const pts = new Float32Array(pf.count * 3);
      for (let i = 0; i < pf.count; i++) {
        const p = compose({ x: pf.x[i], y: pf.y[i], theta: pf.theta[i] }, f.particleOffset);
        pts.set([p.x, p.y, 0.01], i * 3);
      }
      this.particleGeom.setAttribute("position", new THREE.BufferAttribute(pts, 3));
    }

    // Depth cloud
    this.cloud.visible = o.cloud && !!f.cloud;
    if (f.cloud && o.cloud)
      this.cloudGeom.setAttribute(
        "position",
        new THREE.BufferAttribute(f.cloud.subarray(0, f.cloudPoints * 3), 3),
      );

    // Map, twice a second
    if (o.map && f.grid && (this.mapAt < 0 || Math.abs(f.t - this.mapAt) >= 0.5)) {
      this.mapAt = f.t;
      this.drawMap(f.grid);
    }
    if (this.mapMesh) this.mapMesh.visible = o.map && !!f.grid;

    // Frames: map at the start pose, odom where wheel odometry puts it.
    this.mapAxes.visible = o.tf;
    this.mapAxes.position.set(f.mapFrame.x, f.mapFrame.y, 0.002);
    this.mapAxes.rotation.z = f.mapFrame.theta;
    this.odomAxes.visible = o.tf && !!f.odomFrame;
    if (f.odomFrame) {
      this.odomAxes.position.set(f.odomFrame.x, f.odomFrame.y, 0.004);
      this.odomAxes.rotation.z = f.odomFrame.theta;
    }
    this.view.setPose(f.q);
  }

  /**
   * @param {{ line: THREE.Line, pos: Float32Array, n: number }} t
   * @param {Pose2} p
   */
  pushTrail(t, p) {
    if (t.n >= MAX_TRAIL) {
      t.pos.copyWithin(0, 3 * (MAX_TRAIL / 2));
      t.n = MAX_TRAIL / 2;
    }
    t.pos.set([p.x, p.y, 0.006], t.n * 3);
    t.n++;
    t.line.geometry.setDrawRange(0, t.n);
    t.line.geometry.attributes.position.needsUpdate = true;
  }

  /** @param {import("../../vendor/lidar-slam-lab/src/grid.js").OccupancyGrid} grid */
  drawMap(grid) {
    const { width, height } = grid;
    if (!this.mapTex || this.mapTex.image.width !== width || this.mapTex.image.height !== height) {
      const data = new Uint8Array(width * height * 4);
      this.mapTex = new THREE.DataTexture(data, width, height, THREE.RGBAFormat);
      this.mapTex.magFilter = THREE.NearestFilter;
      if (this.mapMesh) this.group.remove(this.mapMesh);
      this.mapMesh = new THREE.Mesh(
        new THREE.PlaneGeometry(width * grid.resolution, height * grid.resolution),
        new THREE.MeshBasicMaterial({ map: this.mapTex, transparent: true, depthWrite: false }),
      );
      this.mapMesh.position.set(
        grid.originX + (width * grid.resolution) / 2,
        grid.originY + (height * grid.resolution) / 2,
        0.003,
      );
      this.mapMesh.renderOrder = 1;
      this.group.add(this.mapMesh);
    }
    const data = /** @type {Uint8Array} */ (this.mapTex.image.data);
    const dark = this.view.dark;
    for (let k = 0; k < grid.logOdds.length; k++) {
      const l = grid.logOdds[k];
      const i = k * 4;
      if (l > 0.5) {
        data[i] = dark ? 240 : 20;
        data[i + 1] = dark ? 240 : 24;
        data[i + 2] = dark ? 245 : 32;
        data[i + 3] = 230;
      } else if (l < -0.5) {
        data[i] = 120;
        data[i + 1] = 190;
        data[i + 2] = 255;
        data[i + 3] = 60;
      } else data[i + 3] = 0;
    }
    this.mapTex.needsUpdate = true;
  }
}
