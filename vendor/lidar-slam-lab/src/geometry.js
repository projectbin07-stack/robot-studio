// 2D geometry helpers. World frame: metres, x to the right, y up, angles in
// radians counter-clockwise from +x.

/**
 * A wall segment as [x1, y1, x2, y2] in metres.
 * @typedef {[number, number, number, number]} Segment
 */

/**
 * A planar pose. `theta` is the heading in radians.
 * @typedef {{ x: number, y: number, theta: number }} Pose
 */

const PARALLEL_EPS = 1e-12;

/**
 * Wrap an angle to the interval (-pi, pi].
 * @param {number} a
 * @returns {number}
 */
export function normalizeAngle(a) {
  let r = a % (2 * Math.PI);
  if (r <= -Math.PI) r += 2 * Math.PI;
  else if (r > Math.PI) r -= 2 * Math.PI;
  return r;
}

/**
 * Distance along a ray to a segment, or Infinity if the ray misses it.
 * The ray is o + t * d with t >= 0; d need not be unit length, in which case
 * the result is in multiples of |d|.
 * @param {number} ox
 * @param {number} oy
 * @param {number} dx
 * @param {number} dy
 * @param {Segment} seg
 * @returns {number}
 */
export function raySegmentDistance(ox, oy, dx, dy, seg) {
  const ex = seg[2] - seg[0];
  const ey = seg[3] - seg[1];
  const denom = dx * ey - dy * ex;
  if (Math.abs(denom) < PARALLEL_EPS) return Infinity;
  const ax = seg[0] - ox;
  const ay = seg[1] - oy;
  const t = (ax * ey - ay * ex) / denom;
  const u = (ax * dy - ay * dx) / denom;
  if (t < 0 || u < 0 || u > 1) return Infinity;
  return t;
}

/**
 * Distance from (ox, oy) along `angle` to the nearest wall, or Infinity if
 * nothing is hit within `maxRange`.
 * @param {readonly Segment[]} walls
 * @param {number} ox
 * @param {number} oy
 * @param {number} angle
 * @param {number} maxRange
 * @returns {number}
 */
export function castRay(walls, ox, oy, angle, maxRange) {
  const dx = Math.cos(angle);
  const dy = Math.sin(angle);
  let best = Infinity;
  for (const w of walls) {
    const t = raySegmentDistance(ox, oy, dx, dy, w);
    if (t < best) best = t;
  }
  return best <= maxRange ? best : Infinity;
}

/**
 * Shortest distance from a point to a segment.
 * @param {number} px
 * @param {number} py
 * @param {Segment} seg
 * @returns {number}
 */
export function pointSegmentDistance(px, py, seg) {
  const [x1, y1, x2, y2] = seg;
  const ex = x2 - x1;
  const ey = y2 - y1;
  const len2 = ex * ex + ey * ey;
  let u = len2 === 0 ? 0 : ((px - x1) * ex + (py - y1) * ey) / len2;
  u = Math.max(0, Math.min(1, u));
  return Math.hypot(px - (x1 + u * ex), py - (y1 + u * ey));
}

/**
 * True if the closed segments p and q intersect (touching counts).
 * @param {Segment} p
 * @param {Segment} q
 * @returns {boolean}
 */
export function segmentsIntersect(p, q) {
  const o1 = orientation(p[0], p[1], p[2], p[3], q[0], q[1]);
  const o2 = orientation(p[0], p[1], p[2], p[3], q[2], q[3]);
  const o3 = orientation(q[0], q[1], q[2], q[3], p[0], p[1]);
  const o4 = orientation(q[0], q[1], q[2], q[3], p[2], p[3]);
  if (o1 !== o2 && o3 !== o4) return true;
  if (o1 === 0 && onSegment(p, q[0], q[1])) return true;
  if (o2 === 0 && onSegment(p, q[2], q[3])) return true;
  if (o3 === 0 && onSegment(q, p[0], p[1])) return true;
  if (o4 === 0 && onSegment(q, p[2], p[3])) return true;
  return false;
}

/**
 * @param {number} ax
 * @param {number} ay
 * @param {number} bx
 * @param {number} by
 * @param {number} cx
 * @param {number} cy
 * @returns {-1 | 0 | 1}
 */
function orientation(ax, ay, bx, by, cx, cy) {
  const v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  if (Math.abs(v) < 1e-12) return 0;
  return v > 0 ? 1 : -1;
}

/**
 * Whether a point known to be collinear with seg lies within its bounding box.
 * @param {Segment} seg
 * @param {number} x
 * @param {number} y
 * @returns {boolean}
 */
function onSegment(seg, x, y) {
  return (
    x >= Math.min(seg[0], seg[2]) - 1e-12 &&
    x <= Math.max(seg[0], seg[2]) + 1e-12 &&
    y >= Math.min(seg[1], seg[3]) - 1e-12 &&
    y <= Math.max(seg[1], seg[3]) + 1e-12
  );
}

/**
 * Index of the wall nearest to a point, or -1 if none is within maxDist.
 * @param {readonly Segment[]} walls
 * @param {number} x
 * @param {number} y
 * @param {number} maxDist
 * @returns {number}
 */
export function nearestWallIndex(walls, x, y, maxDist) {
  let best = -1;
  let bestDist = maxDist;
  walls.forEach((w, i) => {
    const d = pointSegmentDistance(x, y, w);
    if (d <= bestDist) {
      best = i;
      bestDist = d;
    }
  });
  return best;
}

/**
 * Transform a point from a pose's local frame into the world frame.
 * @param {Pose} pose
 * @param {number} lx
 * @param {number} ly
 * @returns {[number, number]}
 */
export function localToWorld(pose, lx, ly) {
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  return [pose.x + c * lx - s * ly, pose.y + s * lx + c * ly];
}
