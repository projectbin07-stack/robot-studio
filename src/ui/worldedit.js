// Top-down editor for the simulation world on a canvas: draw walls, boxes
// and cylinders, erase, move the start pose, place route waypoints. Mouse,
// pen and touch (pointer events). Every change hands a new world to
// onChange; the world itself is plain data (src/core/sim/world.js).

/** @typedef {import("../core/sim/world.js").SimWorld} SimWorld */
/** @typedef {import("../core/sim/se2.js").Pose2} Pose2 */
/** @typedef {"wall" | "box" | "cylinder" | "erase" | "start" | "route"} Tool */

import { boxCorners } from "../core/sim/world.js";
import { pointSegmentDistance } from "../../vendor/lidar-slam-lab/src/geometry.js";

const SNAP = 0.05;
/** @param {number} v */
const snap = (v) => Math.round(v / SNAP) * SNAP;

export class WorldEditor {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {(w: SimWorld, what: string) => void} onChange
   */
  constructor(canvas, onChange) {
    this.canvas = canvas;
    this.onChange = onChange;
    /** @type {SimWorld | null} */
    this.world = null;
    /** @type {Tool} */
    this.tool = "wall";
    this.boxHeight = 0.5;
    this.cylHeight = 1;
    /** @type {Pose2 | null} */
    this.robot = null;
    this.robotRadius = 0.2;
    /** @type {{ x: number, y: number } | null} */
    this.down = null;
    /** @type {{ x: number, y: number } | null} */
    this.cur = null;
    canvas.addEventListener("pointerdown", (e) => this.pointer(e, "down"));
    canvas.addEventListener("pointermove", (e) => this.pointer(e, "move"));
    canvas.addEventListener("pointerup", (e) => this.pointer(e, "up"));
    canvas.addEventListener("pointercancel", () => {
      this.down = null;
      this.draw();
    });
  }

  /** @param {SimWorld} w */
  setWorld(w) {
    this.world = w;
    this.draw();
  }

  /** @param {Pose2 | null} p @param {number} r */
  setRobot(p, r) {
    this.robot = p;
    this.robotRadius = r;
    this.draw();
  }

  /** Scale and offset that fit the world in the canvas. */
  frame() {
    const w = /** @type {SimWorld} */ (this.world);
    const cw = this.canvas.clientWidth;
    const ch = this.canvas.clientHeight;
    const m = 8;
    const s = Math.min((cw - 2 * m) / w.width, (ch - 2 * m) / w.height);
    return { s, ox: (cw - s * w.width) / 2, oy: (ch + s * w.height) / 2 };
  }

  /**
   * @param {PointerEvent} e
   * @returns {{ x: number, y: number }}
   */
  toWorld(e) {
    const r = this.canvas.getBoundingClientRect();
    const f = this.frame();
    const w = /** @type {SimWorld} */ (this.world);
    const x = Math.min(w.width, Math.max(0, (e.clientX - r.left - f.ox) / f.s));
    const y = Math.min(w.height, Math.max(0, (f.oy - (e.clientY - r.top)) / f.s));
    return { x, y };
  }

  /**
   * @param {PointerEvent} e
   * @param {"down" | "move" | "up"} kind
   */
  pointer(e, kind) {
    if (!this.world) return;
    const p = this.toWorld(e);
    if (kind === "down") {
      this.canvas.setPointerCapture(e.pointerId);
      this.down = p;
      this.cur = p;
      e.preventDefault();
      return;
    }
    if (kind === "move") {
      if (this.down) {
        this.cur = p;
        this.draw();
      }
      return;
    }
    const a = this.down;
    this.down = null;
    if (!a) return;
    this.apply(a, p);
    this.draw();
  }

  /**
   * Apply the current tool to a drag from a to b (a click: a == b).
   * @param {{ x: number, y: number }} a
   * @param {{ x: number, y: number }} b
   */
  apply(a, b) {
    const w = /** @type {SimWorld} */ (structuredClone(this.world));
    const drag = Math.hypot(b.x - a.x, b.y - a.y);
    let what = "";
    if (this.tool === "wall" && drag >= 0.1) {
      w.walls.push([snap(a.x), snap(a.y), snap(b.x), snap(b.y)]);
      what = "wall";
    } else if (this.tool === "box" && Math.abs(b.x - a.x) >= 0.05 && Math.abs(b.y - a.y) >= 0.05) {
      const x0 = snap(Math.min(a.x, b.x));
      const x1 = snap(Math.max(a.x, b.x));
      const y0 = snap(Math.min(a.y, b.y));
      const y1 = snap(Math.max(a.y, b.y));
      if (x1 > x0 && y1 > y0) {
        w.boxes.push({
          x: (x0 + x1) / 2,
          y: (y0 + y1) / 2,
          w: x1 - x0,
          d: y1 - y0,
          yaw: 0,
          h: this.boxHeight,
        });
        what = "box";
      }
    } else if (this.tool === "cylinder") {
      const r = Math.max(0.05, snap(drag) || 0.15);
      const c = { x: snap(a.x), y: snap(a.y), r, h: this.cylHeight };
      if (c.x - r >= 0 && c.y - r >= 0 && c.x + r <= w.width && c.y + r <= w.height) {
        w.cylinders.push(c);
        what = "cylinder";
      }
    } else if (this.tool === "erase") {
      what = erase(w, b.x, b.y) ? "erase" : "";
    } else if (this.tool === "start") {
      const theta = drag >= 0.1 ? Math.atan2(b.y - a.y, b.x - a.x) : w.start.theta;
      if (a.x > 0 && a.y > 0 && a.x < w.width && a.y < w.height) {
        w.start = { x: snap(a.x), y: snap(a.y), theta };
        what = "start";
      }
    } else if (this.tool === "route") {
      w.route.push([snap(b.x), snap(b.y)]);
      what = "route";
    }
    if (!what) return;
    w.id = "custom";
    this.world = w;
    this.onChange(w, what);
  }

  draw() {
    const c = this.canvas;
    const w = this.world;
    if (!w) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const cw = c.clientWidth;
    const ch = c.clientHeight;
    if (cw === 0 || ch === 0) return;
    if (c.width !== Math.round(cw * dpr)) c.width = Math.round(cw * dpr);
    if (c.height !== Math.round(ch * dpr)) c.height = Math.round(ch * dpr);
    const g = /** @type {CanvasRenderingContext2D} */ (c.getContext("2d"));
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const css = getComputedStyle(c);
    const text = css.getPropertyValue("--text").trim() || "#222";
    const muted = css.getPropertyValue("--muted").trim() || "#888";
    const accent = css.getPropertyValue("--accent").trim() || "#2a78d6";
    const accent2 = css.getPropertyValue("--accent-2").trim() || "#eb6a1f";
    g.clearRect(0, 0, cw, ch);
    const f = this.frame();
    const X = (/** @type {number} */ x) => f.ox + x * f.s;
    const Y = (/** @type {number} */ y) => f.oy - y * f.s;
    // 1 m grid
    g.strokeStyle = css.getPropertyValue("--line").trim() || "#ddd";
    g.lineWidth = 1;
    g.beginPath();
    for (let x = 0; x <= w.width + 1e-9; x += 1) {
      g.moveTo(X(x), Y(0));
      g.lineTo(X(x), Y(w.height));
    }
    for (let y = 0; y <= w.height + 1e-9; y += 1) {
      g.moveTo(X(0), Y(y));
      g.lineTo(X(w.width), Y(y));
    }
    g.stroke();
    // Obstacles
    g.fillStyle = "rgba(139,115,85,0.55)";
    g.strokeStyle = text;
    g.lineWidth = 1;
    for (const b of w.boxes) {
      const pts = boxCorners(b);
      g.beginPath();
      pts.forEach(([x, y], i) => (i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y))));
      g.closePath();
      g.fill();
      g.stroke();
    }
    for (const cy of w.cylinders) {
      g.beginPath();
      g.arc(X(cy.x), Y(cy.y), cy.r * f.s, 0, 2 * Math.PI);
      g.fill();
      g.stroke();
    }
    g.lineWidth = 2.5;
    g.lineCap = "round";
    g.beginPath();
    for (const [x1, y1, x2, y2] of w.walls) {
      g.moveTo(X(x1), Y(y1));
      g.lineTo(X(x2), Y(y2));
    }
    g.stroke();
    // Route
    if (w.route.length > 0) {
      g.strokeStyle = accent;
      g.lineWidth = 1.2;
      g.setLineDash([4, 3]);
      g.beginPath();
      g.moveTo(X(w.start.x), Y(w.start.y));
      for (const [x, y] of w.route) g.lineTo(X(x), Y(y));
      g.stroke();
      g.setLineDash([]);
      g.fillStyle = accent;
      for (const [x, y] of w.route) g.fillRect(X(x) - 2, Y(y) - 2, 4, 4);
    }
    // Start pose
    arrow(g, X(w.start.x), Y(w.start.y), w.start.theta, 12, muted);
    // Robot
    if (this.robot) {
      g.strokeStyle = accent2;
      g.lineWidth = 1.5;
      g.beginPath();
      g.arc(X(this.robot.x), Y(this.robot.y), Math.max(3, this.robotRadius * f.s), 0, 2 * Math.PI);
      g.stroke();
      arrow(g, X(this.robot.x), Y(this.robot.y), this.robot.theta, 10, accent2);
    }
    // Drag preview
    const a = this.down;
    const b = this.cur;
    if (a && b) {
      g.strokeStyle = accent2;
      g.lineWidth = 1.5;
      g.setLineDash([3, 3]);
      g.beginPath();
      if (this.tool === "box")
        g.rect(
          X(Math.min(a.x, b.x)),
          Y(Math.max(a.y, b.y)),
          Math.abs(b.x - a.x) * f.s,
          Math.abs(b.y - a.y) * f.s,
        );
      else if (this.tool === "cylinder")
        g.arc(X(a.x), Y(a.y), Math.hypot(b.x - a.x, b.y - a.y) * f.s, 0, 2 * Math.PI);
      else {
        g.moveTo(X(a.x), Y(a.y));
        g.lineTo(X(b.x), Y(b.y));
      }
      g.stroke();
      g.setLineDash([]);
    }
  }
}

/**
 * Remove the obstacle or waypoint nearest to (x, y) within 0.3 m.
 * @param {SimWorld} w
 * @param {number} x
 * @param {number} y
 * @returns {boolean}
 */
export function erase(w, x, y) {
  /** @type {[number, () => void] | null} */
  let best = null;
  const consider = (/** @type {number} */ d, /** @type {() => void} */ rm) => {
    if (d <= 0.3 && (!best || d < best[0])) best = [d, rm];
  };
  w.walls.forEach((s, i) => consider(pointSegmentDistance(x, y, s), () => w.walls.splice(i, 1)));
  w.boxes.forEach((b, i) => {
    const inside =
      Math.abs((x - b.x) * Math.cos(b.yaw) + (y - b.y) * Math.sin(b.yaw)) <= b.w / 2 &&
      Math.abs(-(x - b.x) * Math.sin(b.yaw) + (y - b.y) * Math.cos(b.yaw)) <= b.d / 2;
    consider(inside ? 0 : 1, () => w.boxes.splice(i, 1));
  });
  w.cylinders.forEach((c, i) =>
    consider(Math.max(0, Math.hypot(x - c.x, y - c.y) - c.r), () => w.cylinders.splice(i, 1)),
  );
  w.route.forEach((p, i) => consider(Math.hypot(x - p[0], y - p[1]), () => w.route.splice(i, 1)));
  if (!best) return false;
  /** @type {[number, () => void]} */ (best)[1]();
  return true;
}

/**
 * @param {CanvasRenderingContext2D} g
 * @param {number} x
 * @param {number} y
 * @param {number} theta
 * @param {number} len
 * @param {string} color
 */
function arrow(g, x, y, theta, len, color) {
  const dx = Math.cos(theta);
  const dy = -Math.sin(theta);
  g.strokeStyle = color;
  g.fillStyle = color;
  g.lineWidth = 2;
  g.beginPath();
  g.moveTo(x, y);
  g.lineTo(x + dx * len, y + dy * len);
  g.stroke();
  g.beginPath();
  g.arc(x, y, 2.5, 0, 2 * Math.PI);
  g.fill();
}
