// Monte Carlo localisation (particle filter) on a known map, following the
// structure of Probabilistic Robotics, chapter 8: sample the odometry motion
// model, weight each particle with the likelihood-field measurement model,
// and resample with the low-variance sampler when the weights degenerate.
//
// Unlike the scan-matching estimator, which builds its own map, MCL is given
// the true floor plan. It answers "where am I on this map?", including from a
// completely unknown start (global localisation).

import { normalizeAngle, pointSegmentDistance } from "./geometry.js";
import { applyBodyDelta, bodyFromOdometry, corruptBodyDelta, corruptOdometry } from "./motion.js";
import { sampleField } from "./distance.js";

/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./geometry.js").Segment} Segment */
/** @typedef {import("./rng.js").Rng} Rng */
/** @typedef {import("./lidar.js").Scan} Scan */
/** @typedef {import("./motion.js").OdometryDelta} OdometryDelta */
/** @typedef {import("./motion.js").OdometryNoise} OdometryNoise */
/** @typedef {import("./distance.js").LikelihoodField} LikelihoodField */

/**
 * @typedef {object} MclOptions
 * @property {number} beams Beams used per measurement update (evenly subsampled).
 * @property {number} zHit Weight of the likelihood-field term.
 * @property {number} zRandom Weight of the uniform "random measurement" term;
 *   keeps one bad beam from zeroing a particle.
 * @property {number} resampleThreshold Resample when the effective sample size
 *   falls below this fraction of the particle count.
 * @property {OdometryNoise} motionNoise Noise added when propagating particles.
 *   The filter does not know the robot's systematic heading bias.
 * @property {MotionModel} motionModel How a particle's motion is sampled:
 *   "odometry" (rot1-trans-rot2, Probabilistic Robotics 5.4; right for robots
 *   that drive forward and turn) or "omni" (forward, sideways and turn noise in
 *   the body frame; for holonomic bases such as mecanum, where a strafe would
 *   otherwise be read as two 90 degree turns).
 * @property {number} alphaSlow Smoothing of the long-term average fit (augmented MCL).
 * @property {number} alphaFast Smoothing of the short-term average fit.
 * @property {number} injectionDeadband Ignore fit drops smaller than this
 *   fraction; the fit ratio fluctuates by a few per cent in normal tracking.
 * @property {number} maxInjection Upper bound on the fraction of particles
 *   replaced by random ones in one resampling step.
 */

/** @typedef {"odometry" | "omni"} MotionModel */

/** @type {Readonly<MclOptions>} */
export const DEFAULT_MCL = Object.freeze({
  beams: 36,
  zHit: 0.9,
  zRandom: 0.1,
  resampleThreshold: 0.5,
  motionNoise: Object.freeze({
    rotFromRot: 0.1,
    rotFromTrans: 0.05,
    transFromTrans: 0.06,
    transFromRot: 0.01,
    headingBias: 0,
  }),
  motionModel: "odometry",
  alphaSlow: 0.01,
  alphaFast: 0.2,
  injectionDeadband: 0.05,
  maxInjection: 0.1,
});

/**
 * Where particles may be: the map rectangle, at least `clearance` from walls.
 * @typedef {{ width: number, height: number, walls: readonly Segment[], clearance: number }} FreeSpace
 */

/**
 * @typedef {object} ParticleFilter
 * @property {number} count
 * @property {Float64Array} x
 * @property {Float64Array} y
 * @property {Float64Array} theta
 * @property {Float64Array} weight Normalised, sums to 1.
 * @property {LikelihoodField} field Built from the known map.
 * @property {MclOptions} options
 * @property {Rng} rng
 * @property {number} resamples Number of resampling steps so far.
 * @property {FreeSpace | null} space Needed for global initialisation and for
 *   injecting random particles; without it neither happens.
 * @property {number} wSlow Long-term average per-beam fit (0 until the first update).
 * @property {number} wFast Short-term average per-beam fit.
 * @property {number} injected Particles replaced by random ones so far.
 */

/**
 * @param {number} count
 * @param {LikelihoodField} field
 * @param {Rng} rng
 * @param {Partial<MclOptions>} [options]
 * @param {FreeSpace | null} [space]
 * @returns {ParticleFilter}
 */
export function createParticleFilter(count, field, rng, options = {}, space = null) {
  if (!Number.isInteger(count) || count < 1)
    throw new RangeError("count must be a positive integer");
  return {
    count,
    x: new Float64Array(count),
    y: new Float64Array(count),
    theta: new Float64Array(count),
    weight: new Float64Array(count).fill(1 / count),
    field,
    options: { ...DEFAULT_MCL, ...options },
    rng,
    resamples: 0,
    space,
    wSlow: 0,
    wFast: 0,
    injected: 0,
  };
}

/**
 * Spread the particles as a Gaussian around a known pose (pose tracking).
 * @param {ParticleFilter} pf
 * @param {Pose} pose
 * @param {number} sigmaXY metres
 * @param {number} sigmaTheta radians
 */
export function initializeAround(pf, pose, sigmaXY, sigmaTheta) {
  for (let i = 0; i < pf.count; i++) {
    pf.x[i] = pose.x + sigmaXY * pf.rng.gaussian();
    pf.y[i] = pose.y + sigmaXY * pf.rng.gaussian();
    pf.theta[i] = normalizeAngle(pose.theta + sigmaTheta * pf.rng.gaussian());
  }
  pf.weight.fill(1 / pf.count);
}

/**
 * A random pose in free space (uniform position and heading). Gives up after
 * 1000 rejected positions and returns the last one, so a map with no free
 * space cannot hang the filter.
 * @param {Rng} rng
 * @param {FreeSpace} space
 * @returns {Pose}
 */
export function randomFreePose(rng, space) {
  let x = 0;
  let y = 0;
  for (let t = 0; t < 1000; t++) {
    x = rng.next() * space.width;
    y = rng.next() * space.height;
    if (space.walls.every((w) => pointSegmentDistance(x, y, w) >= space.clearance)) break;
  }
  return { x, y, theta: normalizeAngle(rng.next() * 2 * Math.PI) };
}

/**
 * Spread the particles uniformly over free space (global localisation).
 * @param {ParticleFilter} pf
 */
export function initializeUniform(pf) {
  if (pf.space === null) throw new Error("global initialisation needs the free space");
  for (let i = 0; i < pf.count; i++) {
    const p = randomFreePose(pf.rng, pf.space);
    pf.x[i] = p.x;
    pf.y[i] = p.y;
    pf.theta[i] = p.theta;
  }
  pf.weight.fill(1 / pf.count);
}

/**
 * Motion update: move every particle by the measured odometry, each with its
 * own noise sample.
 * @param {ParticleFilter} pf
 * @param {OdometryDelta} measured
 */
export function predict(pf, measured) {
  if (pf.options.motionModel === "omni") {
    const body = bodyFromOdometry(measured);
    for (let i = 0; i < pf.count; i++) {
      const d = corruptBodyDelta(body, pf.options.motionNoise, pf.rng);
      const p = applyBodyDelta({ x: pf.x[i], y: pf.y[i], theta: pf.theta[i] }, d);
      pf.x[i] = p.x;
      pf.y[i] = p.y;
      pf.theta[i] = p.theta;
    }
    return;
  }
  if (pf.options.motionModel !== "odometry")
    throw new RangeError(`unknown motion model "${pf.options.motionModel}"`);
  for (let i = 0; i < pf.count; i++) {
    const d = corruptOdometry(measured, pf.options.motionNoise, pf.rng);
    const heading = pf.theta[i] + d.rot1;
    pf.x[i] += d.trans * Math.cos(heading);
    pf.y[i] += d.trans * Math.sin(heading);
    pf.theta[i] = normalizeAngle(heading + d.rot2);
  }
}

/**
 * Measurement update with the likelihood-field model:
 * p(z | x) = prod_k (zHit * field(endpoint_k) + zRandom).
 * Done in log space and renormalised, so long scans do not underflow.
 * @param {ParticleFilter} pf
 * @param {Scan} scan
 * @returns {number} Number of beams used (0 means the weights were not changed).
 */
export function updateWeights(pf, scan) {
  /** @type {number[]} */
  const idx = [];
  for (let i = 0; i < scan.ranges.length; i++) if (!Number.isNaN(scan.ranges[i])) idx.push(i);
  if (idx.length === 0) return 0;
  const stride = Math.max(1, idx.length / pf.options.beams);
  /** @type {number[]} */
  const used = [];
  for (let k = 0; k * stride < idx.length; k++) used.push(idx[Math.floor(k * stride)]);
  const { zHit, zRandom } = pf.options;
  const origin = scan.origin;
  const space = pf.space;
  const logW = new Float64Array(pf.count);
  let maxLog = -Infinity;
  let avgFit = 0;
  for (let p = 0; p < pf.count; p++) {
    const px = pf.x[p];
    const py = pf.y[p];
    if (space !== null && (px < 0 || py < 0 || px > space.width || py > space.height)) {
      logW[p] = -Infinity; // a robot cannot be outside the building
      continue;
    }
    let ll = 0;
    let sx = px;
    let sy = py;
    let sth = pf.theta[p];
    if (origin !== undefined) {
      const c = Math.cos(sth);
      const s = Math.sin(sth);
      sx = px + c * origin.x - s * origin.y;
      sy = py + s * origin.x + c * origin.y;
      sth += origin.theta;
    }
    for (const b of used) {
      const a = sth + scan.angles[b];
      const r = scan.ranges[b];
      const v = sampleField(pf.field, sx + r * Math.cos(a), sy + r * Math.sin(a));
      ll += Math.log(zHit * v + zRandom);
    }
    // Per-beam geometric mean likelihood: a fit measure in [zRandom, zHit + zRandom].
    avgFit += pf.weight[p] * Math.exp(ll / used.length);
    const lw = Math.log(pf.weight[p]) + ll;
    logW[p] = lw;
    if (lw > maxLog) maxLog = lw;
  }
  if (pf.wSlow === 0) {
    pf.wSlow = avgFit;
    pf.wFast = avgFit;
  } else {
    pf.wSlow += pf.options.alphaSlow * (avgFit - pf.wSlow);
    pf.wFast += pf.options.alphaFast * (avgFit - pf.wFast);
  }
  if (maxLog === -Infinity) {
    // Every particle is impossible: start over.
    if (space !== null) initializeUniform(pf);
    else pf.weight.fill(1 / pf.count);
    return used.length;
  }
  let sum = 0;
  for (let p = 0; p < pf.count; p++) {
    const w = Math.exp(logW[p] - maxLog);
    pf.weight[p] = w;
    sum += w;
  }
  for (let p = 0; p < pf.count; p++) pf.weight[p] /= sum;
  return used.length;
}

/**
 * Fraction of particles to replace with random ones (augmented MCL,
 * Probabilistic Robotics table 8.3, plus a dead band): positive when the recent fit is worse
 * than the long-term fit, which happens after the robot is kidnapped or the
 * filter has converged on the wrong place.
 * @param {ParticleFilter} pf
 * @returns {number}
 */
export function injectionRate(pf) {
  if (pf.space === null || pf.wSlow <= 0) return 0;
  const drop = 1 - pf.wFast / pf.wSlow - pf.options.injectionDeadband;
  return Math.min(pf.options.maxInjection, Math.max(0, drop));
}

/**
 * Effective sample size 1 / sum(w^2): equals the particle count for uniform
 * weights and 1 when one particle holds all the weight.
 * @param {Float64Array} weights Normalised weights.
 * @returns {number}
 */
export function effectiveSampleSize(weights) {
  let s = 0;
  for (const w of weights) s += w * w;
  return 1 / s;
}

/**
 * Low-variance (systematic) resampling: one random offset r in [0, 1/N), then
 * N equally spaced pointers r + m/N into the cumulative weights.
 * @param {Float64Array} weights Normalised weights.
 * @param {number} r Offset in [0, 1/N).
 * @returns {Int32Array} Index of the source particle for each new particle.
 */
export function lowVarianceIndices(weights, r) {
  const n = weights.length;
  const out = new Int32Array(n);
  let c = weights[0];
  let i = 0;
  for (let m = 0; m < n; m++) {
    const u = r + m / n;
    while (u > c && i < n - 1) {
      i++;
      c += weights[i];
    }
    out[m] = i;
  }
  return out;
}

/**
 * Resample if the effective sample size is below the threshold or random
 * particles are due to be injected.
 * @param {ParticleFilter} pf
 * @returns {boolean} Whether resampling happened.
 */
export function resampleIfNeeded(pf) {
  const inject = injectionRate(pf);
  const degenerate = effectiveSampleSize(pf.weight) < pf.options.resampleThreshold * pf.count;
  if (!degenerate && inject <= 0) return false;
  const idx = lowVarianceIndices(pf.weight, pf.rng.next() / pf.count);
  const x = Float64Array.from(idx, (i) => pf.x[i]);
  const y = Float64Array.from(idx, (i) => pf.y[i]);
  const th = Float64Array.from(idx, (i) => pf.theta[i]);
  if (inject > 0 && pf.space !== null) {
    for (let m = 0; m < pf.count; m++) {
      if (pf.rng.next() >= inject) continue;
      const p = randomFreePose(pf.rng, pf.space);
      x[m] = p.x;
      y[m] = p.y;
      th[m] = p.theta;
      pf.injected++;
    }
  }
  pf.x.set(x);
  pf.y.set(y);
  pf.theta.set(th);
  pf.weight.fill(1 / pf.count);
  pf.resamples++;
  return true;
}

/**
 * @typedef {object} MclEstimate
 * @property {Pose} pose Weighted mean; the heading is a circular mean.
 * @property {number} spread Weighted RMS distance of the particles from the mean (m).
 */

/**
 * @param {ParticleFilter} pf
 * @returns {MclEstimate}
 */
export function estimate(pf) {
  let mx = 0;
  let my = 0;
  let sc = 0;
  let ss = 0;
  for (let i = 0; i < pf.count; i++) {
    const w = pf.weight[i];
    mx += w * pf.x[i];
    my += w * pf.y[i];
    sc += w * Math.cos(pf.theta[i]);
    ss += w * Math.sin(pf.theta[i]);
  }
  let v = 0;
  for (let i = 0; i < pf.count; i++) {
    v += pf.weight[i] * ((pf.x[i] - mx) ** 2 + (pf.y[i] - my) ** 2);
  }
  return { pose: { x: mx, y: my, theta: Math.atan2(ss, sc) }, spread: Math.sqrt(v) };
}
