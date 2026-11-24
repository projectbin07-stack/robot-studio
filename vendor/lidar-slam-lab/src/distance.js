// Exact Euclidean distance transform (Felzenszwalb and Huttenlocher, "Distance
// Transforms of Sampled Functions", 2012) and the likelihood field built on it.

const INF = 1e20;

/**
 * Squared distance transform of a 1D sampled function, in place into `d`.
 * @param {Float64Array} f Input samples (0 at sites, INF elsewhere).
 * @param {number} n
 * @param {Float64Array} d Output.
 * @param {Int32Array} v Scratch, length >= n.
 * @param {Float64Array} z Scratch, length >= n + 1.
 */
function dt1d(f, n, d, v, z) {
  let k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = INF;
  for (let q = 1; q < n; q++) {
    // Intersection of the parabola rooted at q with the rightmost kept one.
    // z[0] is -INF, so this loop always stops at k = 0 at the latest.
    let p = v[k];
    let s = (f[q] + q * q - (f[p] + p * p)) / (2 * q - 2 * p);
    while (s <= z[k]) {
      k--;
      p = v[k];
      s = (f[q] + q * q - (f[p] + p * p)) / (2 * q - 2 * p);
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = INF;
  }
  k = 0;
  for (let q = 0; q < n; q++) {
    while (z[k + 1] < q) k++;
    const p = v[k];
    d[q] = (q - p) * (q - p) + f[p];
  }
}

/**
 * Euclidean distance, in cells, from every cell to the nearest set cell of
 * `mask`. If no cell is set, every distance is Infinity.
 * @param {Uint8Array} mask Row-major, 1 = site.
 * @param {number} width
 * @param {number} height
 * @returns {Float64Array}
 */
export function distanceTransform(mask, width, height) {
  const n = Math.max(width, height);
  const f = new Float64Array(n);
  const d = new Float64Array(n);
  const v = new Int32Array(n);
  const z = new Float64Array(n + 1);
  const sq = new Float64Array(width * height);
  let any = false;
  for (let k = 0; k < mask.length; k++) {
    sq[k] = mask[k] ? 0 : INF;
    if (mask[k]) any = true;
  }
  if (!any) return new Float64Array(width * height).fill(Infinity);
  // Columns, then rows.
  for (let i = 0; i < width; i++) {
    for (let j = 0; j < height; j++) f[j] = sq[j * width + i];
    dt1d(f, height, d, v, z);
    for (let j = 0; j < height; j++) sq[j * width + i] = d[j];
  }
  for (let j = 0; j < height; j++) {
    const row = j * width;
    for (let i = 0; i < width; i++) f[i] = sq[row + i];
    dt1d(f, width, d, v, z);
    for (let i = 0; i < width; i++) sq[row + i] = Math.sqrt(d[i]);
  }
  return sq;
}

/**
 * @typedef {object} LikelihoodField
 * @property {number} resolution
 * @property {number} width
 * @property {number} height
 * @property {number} originX
 * @property {number} originY
 * @property {Float32Array} values Score in [0, 1]: exp(-d^2 / (2 sigma^2)).
 * @property {number} occupiedCells Number of cells treated as obstacles.
 */

/**
 * Build a likelihood field from an occupancy mask. A scan endpoint landing
 * exactly on an obstacle cell scores 1; the score falls off as a Gaussian of
 * the distance to the nearest obstacle.
 * @param {Uint8Array} mask
 * @param {{ resolution: number, width: number, height: number, originX: number, originY: number }} geom
 * @param {number} sigma Standard deviation of the fall-off, in metres.
 * @returns {LikelihoodField}
 */
export function likelihoodField(mask, geom, sigma) {
  const dist = distanceTransform(mask, geom.width, geom.height);
  const values = new Float32Array(dist.length);
  const k = (geom.resolution * geom.resolution) / (2 * sigma * sigma);
  let occupiedCells = 0;
  for (let c = 0; c < dist.length; c++) {
    values[c] = Math.exp(-dist[c] * dist[c] * k);
    if (mask[c]) occupiedCells++;
  }
  return {
    resolution: geom.resolution,
    width: geom.width,
    height: geom.height,
    originX: geom.originX,
    originY: geom.originY,
    values,
    occupiedCells,
  };
}

/**
 * Bilinearly interpolated field value at a world point; 0 outside the field.
 * Sample points sit at cell centres.
 * @param {LikelihoodField} field
 * @param {number} x
 * @param {number} y
 * @returns {number}
 */
export function sampleField(field, x, y) {
  const fx = (x - field.originX) / field.resolution - 0.5;
  const fy = (y - field.originY) / field.resolution - 0.5;
  const i = Math.floor(fx);
  const j = Math.floor(fy);
  if (i < 0 || j < 0 || i + 1 >= field.width || j + 1 >= field.height) return 0;
  const tx = fx - i;
  const ty = fy - j;
  const w = field.width;
  const v = field.values;
  const a = v[j * w + i];
  const b = v[j * w + i + 1];
  const c = v[(j + 1) * w + i];
  const d = v[(j + 1) * w + i + 1];
  return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
}
