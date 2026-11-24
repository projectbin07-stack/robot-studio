// Scan-to-map matching on a likelihood field: an exhaustive (correlative)
// search over a small window around the odometry guess, followed by local
// refinement with a shrinking step. See Olson, "Real-Time Correlative Scan
// Matching" (ICRA 2009) for the idea; this is a simplified single-resolution
// version without the probabilistic covariance estimate.

import { normalizeAngle } from "./geometry.js";
import { sampleField } from "./distance.js";

/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./lidar.js").Scan} Scan */
/** @typedef {import("./distance.js").LikelihoodField} LikelihoodField */

/**
 * @typedef {object} MatchOptions
 * @property {number} linearWindow Search +/- this many metres in x and y.
 * @property {number} linearStep Coarse search step in metres.
 * @property {number} angularWindow Search +/- this many radians.
 * @property {number} angularStep Coarse search step in radians.
 * @property {number} refineIterations Number of step halvings in refinement.
 * @property {number} priorWeight Penalty for moving away from the guess, as a
 *   fraction of the score per (window)^2. Breaks ties in featureless corridors.
 * @property {number} minBeams Fewer valid beams than this: keep the guess.
 * @property {number} minScore Best mean score below this: keep the guess.
 * @property {number} maxBeams Use at most this many beams (evenly subsampled).
 */

/** @type {Readonly<MatchOptions>} */
export const DEFAULT_MATCH = Object.freeze({
  linearWindow: 0.2,
  linearStep: 0.05,
  angularWindow: 0.1,
  angularStep: 0.02,
  refineIterations: 6,
  priorWeight: 0.05,
  minBeams: 20,
  minScore: 0.25,
  maxBeams: 120,
});

/**
 * @typedef {object} MatchResult
 * @property {Pose} pose Corrected pose (equal to the guess if not accepted).
 * @property {number} score Mean field value of the scan at the returned pose.
 * @property {number} guessScore Mean field value at the guess.
 * @property {boolean} accepted Whether the correction was applied.
 * @property {string} reason Why it was or was not applied.
 * @property {number} evaluations Number of candidate poses scored.
 */

/**
 * Valid beams of a scan as robot-frame points, subsampled to at most maxBeams.
 * @param {Scan} scan
 * @param {number} maxBeams
 * @returns {{ r: Float64Array, a: Float64Array }}
 */
function validBeams(scan, maxBeams) {
  /** @type {number[]} */
  const idx = [];
  for (let i = 0; i < scan.ranges.length; i++) if (!Number.isNaN(scan.ranges[i])) idx.push(i);
  const stride = Math.max(1, idx.length / maxBeams);
  const count = Math.min(idx.length, Math.floor(idx.length / stride));
  const r = new Float64Array(count);
  const a = new Float64Array(count);
  for (let k = 0; k < count; k++) {
    const i = idx[Math.floor(k * stride)];
    r[k] = scan.ranges[i];
    a[k] = scan.angles[i];
  }
  return { r, a };
}

/**
 * Mean likelihood-field value of the scan endpoints placed at `pose`.
 * @param {LikelihoodField} field
 * @param {Scan} scan
 * @param {Pose} pose
 * @returns {number} 0 if the scan has no valid beams.
 */
export function scoreScan(field, scan, pose) {
  const { r, a } = validBeams(scan, Infinity);
  return scorePoints(field, r, a, pose.x, pose.y, pose.theta);
}

/**
 * @param {LikelihoodField} field
 * @param {Float64Array} r
 * @param {Float64Array} a
 * @param {number} x
 * @param {number} y
 * @param {number} theta
 * @returns {number}
 */
function scorePoints(field, r, a, x, y, theta) {
  if (r.length === 0) return 0;
  let sum = 0;
  for (let k = 0; k < r.length; k++) {
    const ang = theta + a[k];
    sum += sampleField(field, x + r[k] * Math.cos(ang), y + r[k] * Math.sin(ang));
  }
  return sum / r.length;
}

/**
 * Find the pose near `guess` at which the scan best fits the map.
 * @param {LikelihoodField} field
 * @param {Scan} scan
 * @param {Pose} guess
 * @param {Partial<MatchOptions>} [options]
 * @returns {MatchResult}
 */
export function matchScan(field, scan, guess, options = {}) {
  const o = { ...DEFAULT_MATCH, ...options };
  const { r, a } = validBeams(scan, o.maxBeams);
  const guessScore = scorePoints(field, r, a, guess.x, guess.y, guess.theta);
  /**
   * @param {string} reason
   * @returns {MatchResult}
   */
  const keepGuess = (reason) => ({
    pose: { ...guess },
    score: guessScore,
    guessScore,
    accepted: false,
    reason,
    evaluations: 1,
  });
  if (r.length < o.minBeams) return keepGuess("too few valid beams");
  if (field.occupiedCells === 0) return keepGuess("map is empty");

  const lw2 = o.linearWindow * o.linearWindow;
  const aw2 = o.angularWindow * o.angularWindow;
  /**
   * Score with the prior penalty, for offsets from the guess.
   * @param {number} dx
   * @param {number} dy
   * @param {number} dth
   */
  const objective = (dx, dy, dth) =>
    scorePoints(field, r, a, guess.x + dx, guess.y + dy, guess.theta + dth) -
    o.priorWeight * ((dx * dx + dy * dy) / lw2 + (dth * dth) / aw2);

  let evaluations = 0;
  let best = { dx: 0, dy: 0, dth: 0, value: -Infinity };
  const nl = Math.round(o.linearWindow / o.linearStep);
  const na = Math.round(o.angularWindow / o.angularStep);
  for (let ia = -na; ia <= na; ia++) {
    const dth = ia * o.angularStep;
    for (let ix = -nl; ix <= nl; ix++) {
      for (let iy = -nl; iy <= nl; iy++) {
        const dx = ix * o.linearStep;
        const dy = iy * o.linearStep;
        const value = objective(dx, dy, dth);
        evaluations++;
        if (value > best.value) best = { dx, dy, dth, value };
      }
    }
  }

  // Local refinement: try +/- step on each axis, halve the step when stuck.
  let ls = o.linearStep / 2;
  let as = o.angularStep / 2;
  for (let it = 0; it < o.refineIterations; it++) {
    let improved = true;
    while (improved) {
      improved = false;
      const candidates = [
        [ls, 0, 0],
        [-ls, 0, 0],
        [0, ls, 0],
        [0, -ls, 0],
        [0, 0, as],
        [0, 0, -as],
      ];
      for (const [cx, cy, ca] of candidates) {
        const dx = best.dx + cx;
        const dy = best.dy + cy;
        const dth = best.dth + ca;
        if (Math.abs(dx) > o.linearWindow || Math.abs(dy) > o.linearWindow) continue;
        if (Math.abs(dth) > o.angularWindow) continue;
        const value = objective(dx, dy, dth);
        evaluations++;
        if (value > best.value + 1e-12) {
          best = { dx, dy, dth, value };
          improved = true;
        }
      }
    }
    ls /= 2;
    as /= 2;
  }

  const pose = {
    x: guess.x + best.dx,
    y: guess.y + best.dy,
    theta: normalizeAngle(guess.theta + best.dth),
  };
  const score = scorePoints(field, r, a, pose.x, pose.y, pose.theta);
  if (score < o.minScore) {
    return { ...keepGuess("best score below threshold"), evaluations };
  }
  return { pose, score, guessScore, accepted: true, reason: "matched", evaluations };
}
