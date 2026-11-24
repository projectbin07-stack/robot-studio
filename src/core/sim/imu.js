// Simulated IMU at a URDF link: specific force (acceleration minus gravity)
// and angular rate of that link, in the link's own frame, corrupted by an
// error model. The error model is either the defaults below, the values
// from a Gazebo <sensor type="imu"> tag on the link, or the calibration
// that imu-calibration-lab exports (JSON): its accelerometer calibration is
// a_cal = M (a_raw - b), so the simulated raw reading is a_raw = M^-1 a + b;
// its gyroscope calibration is w_cal = (w_raw - bias) / scale, so
// w_raw = scale w + bias; white noise from the gyroscope's static noise
// standard deviation or from an Allan analysis' noise density, and rate
// random walk from the Allan K term.

/** @typedef {import("../../../vendor/lidar-slam-lab/src/rng.js").Rng} Rng */
/** @typedef {[number, number, number]} V3 */

export const G0 = 9.80665;
const DEG = Math.PI / 180;

/**
 * Error model in SI units, per axis (x, y, z of the IMU frame).
 * @typedef {object} ImuErrorModel
 * @property {V3} accelBias m/s^2
 * @property {number[]} accelMatrixInv 3x3 row-major, raw = Minv * true + bias
 * @property {V3} accelNoiseDensity m/s^2/sqrt(Hz)
 * @property {V3} gyroBias rad/s
 * @property {V3} gyroScale
 * @property {V3} gyroNoiseDensity rad/s/sqrt(Hz)
 * @property {V3} gyroRandomWalk rad/s/sqrt(s) (bias random walk, Allan K)
 * @property {string} source where the numbers came from
 */

/** @returns {number[]} */
const I3 = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];
/** @param {number} v @returns {V3} */
const v3 = (v) => [v, v, v];

/**
 * Defaults: zero bias and unit scale, white noise at the typical noise
 * densities in InvenSense's MPU-6000/6050 product specification (gyroscope
 * 0.005 deg/s/sqrt(Hz), accelerometer 400 ug/sqrt(Hz)). Not a measurement
 * of any particular sensor.
 * @returns {ImuErrorModel}
 */
export function defaultImuModel() {
  return {
    accelBias: v3(0),
    accelMatrixInv: I3(),
    accelNoiseDensity: v3(400e-6 * G0),
    gyroBias: v3(0),
    gyroScale: v3(1),
    gyroNoiseDensity: v3(0.005 * DEG),
    gyroRandomWalk: v3(0),
    source: "defaults (MPU-6050 datasheet typical noise density, no bias)",
  };
}

/** @param {number[]} m 3x3 row-major @returns {number[]} */
export function invert3(m) {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h;
  const B = -(d * i - f * g);
  const C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (!(Math.abs(det) > 1e-12)) throw new Error("accelerometer matrix is singular");
  return [
    A / det,
    -(b * i - c * h) / det,
    (b * f - c * e) / det,
    B / det,
    (a * i - c * g) / det,
    -(a * f - c * d) / det,
    C / det,
    -(a * h - b * g) / det,
    (a * e - b * d) / det,
  ];
}

/** @param {unknown} v @param {string} what @returns {V3} */
function vec3(v, what) {
  if (
    !Array.isArray(v) ||
    v.length !== 3 ||
    !v.every((x) => typeof x === "number" && Number.isFinite(x))
  )
    throw new Error(`${what} must be an array of three numbers`);
  return /** @type {V3} */ ([v[0], v[1], v[2]]);
}

/**
 * Error model from imu-calibration-lab's JSON: the page's combined export
 * ({ generator, accel, gyro, mag }), a single CLI result ({ kind: "accel" |
 * "gyro" | "allan" }), or an array of those. Fields not given keep the
 * defaults. Units are converted from the export's (g, deg/s,
 * g/sqrt(Hz), deg/s/sqrt(Hz)) to SI.
 * @param {string | unknown} input
 * @param {number} [sampleRate] Hz, to turn a per-sample noise std into a density
 * @returns {ImuErrorModel}
 */
export function imuModelFromCalibration(input, sampleRate = 100) {
  /** @type {any} */
  let data = input;
  if (typeof input === "string") {
    try {
      data = JSON.parse(input);
    } catch (e) {
      throw new Error(`not valid JSON: ${e instanceof Error ? e.message : String(e)}`);
    }
  }
  /** @type {any[]} */
  const parts = [];
  const add = (/** @type {any} */ v) => {
    if (Array.isArray(v)) v.forEach(add);
    else if (v && typeof v === "object") {
      if (typeof v.kind === "string") parts.push(v);
      else for (const k of ["accel", "gyro", "allan"]) if (v[k]) add(v[k]);
    }
  };
  add(data);
  if (parts.length === 0)
    throw new Error(
      "no accel, gyro or allan result found (expected imu-calibration-lab's JSON export)",
    );
  const m = defaultImuModel();
  /** @type {string[]} */
  const used = [];
  for (const p of parts) {
    if (p.kind === "accel") {
      if (p.units !== undefined && p.units !== "g")
        throw new Error(`accel units '${p.units}' (expected g)`);
      m.accelBias = /** @type {V3} */ (vec3(p.bias, "accel.bias").map((b) => b * G0));
      if (p.matrix !== undefined) {
        if (!Array.isArray(p.matrix) || p.matrix.length !== 3)
          throw new Error("accel.matrix must be 3x3");
        m.accelMatrixInv = invert3(
          p.matrix.flatMap((/** @type {unknown} */ r, /** @type {number} */ i) =>
            vec3(r, `accel.matrix[${i}]`),
          ),
        );
      }
      used.push("accelerometer bias and matrix");
    } else if (p.kind === "gyro") {
      if (p.units !== undefined && p.units !== "deg/s")
        throw new Error(`gyro units '${p.units}' (expected deg/s)`);
      m.gyroBias = /** @type {V3} */ (vec3(p.meanBias ?? p.bias, "gyro.bias").map((b) => b * DEG));
      if (p.scale !== undefined) m.gyroScale = vec3(p.scale, "gyro.scale");
      if (p.noiseStd !== undefined)
        m.gyroNoiseDensity = /** @type {V3} */ (
          vec3(p.noiseStd, "gyro.noiseStd").map((s) => (s * DEG) / Math.sqrt(sampleRate))
        );
      used.push("gyroscope bias, scale and noise");
    } else if (p.kind === "allan") {
      if (!Array.isArray(p.axes) || p.axes.length !== 3)
        throw new Error("allan.axes must have three axes");
      const dens = p.axes.map((/** @type {any} */ a) => a.noiseDensity);
      if (p.sensor === "gyro") {
        if (dens.every((/** @type {unknown} */ d) => typeof d === "number"))
          m.gyroNoiseDensity = /** @type {V3} */ (dens.map((/** @type {number} */ d) => d * DEG));
        const k = p.axes.map((/** @type {any} */ a) => a.rateRandomWalkFit?.value);
        if (k.every((/** @type {unknown} */ v) => typeof v === "number"))
          m.gyroRandomWalk = /** @type {V3} */ (k.map((/** @type {number} */ v) => v * DEG));
        used.push("gyroscope noise density and rate random walk (Allan)");
      } else if (p.sensor === "accel") {
        if (dens.every((/** @type {unknown} */ d) => typeof d === "number"))
          m.accelNoiseDensity = /** @type {V3} */ (dens.map((/** @type {number} */ d) => d * G0));
        used.push("accelerometer noise density (Allan)");
      } else throw new Error(`allan.sensor '${p.sensor}' (expected gyro or accel)`);
    }
  }
  m.source = `imu-calibration-lab JSON: ${used.join("; ")}`;
  return m;
}

/**
 * Error model from Gazebo <imu> noise parameters (per-sample std and bias mean).
 * @param {{ gyroNoise?: number, gyroBias?: number, accelNoise?: number, accelBias?: number }} g
 * @param {number} rate Hz
 * @returns {ImuErrorModel}
 */
export function imuModelFromGazebo(g, rate) {
  const m = defaultImuModel();
  const s = Math.sqrt(rate);
  if (g.gyroNoise !== undefined) m.gyroNoiseDensity = v3(g.gyroNoise / s);
  if (g.gyroBias !== undefined) m.gyroBias = v3(g.gyroBias);
  if (g.accelNoise !== undefined) m.accelNoiseDensity = v3(g.accelNoise / s);
  if (g.accelBias !== undefined) m.accelBias = v3(g.accelBias);
  m.source = 'URDF <gazebo> <sensor type="imu"> noise';
  return m;
}

/**
 * One IMU reading in the sensor frame.
 * @typedef {{ accel: V3, gyro: V3 }} ImuReading
 */

export class ImuSensor {
  /**
   * @param {ImuErrorModel} model
   * @param {number} rate Hz (sets the per-sample noise from the density)
   * @param {Rng} rng
   */
  constructor(model, rate, rng) {
    this.model = model;
    this.rate = rate;
    this.rng = rng;
    /** Bias drift from the random walk, rad/s. @type {V3} */
    this.drift = [0, 0, 0];
  }

  /**
   * Corrupt the true specific force (m/s^2) and angular rate (rad/s), both
   * in the sensor frame. Draws exactly 9 Gaussian samples per call so the
   * random sequence does not depend on the model.
   * @param {V3} f
   * @param {V3} w
   * @returns {ImuReading}
   */
  read(f, w) {
    const m = this.model;
    const sq = Math.sqrt(this.rate);
    const dt = 1 / this.rate;
    const M = m.accelMatrixInv;
    /** @type {V3} */
    const accel = [0, 0, 0];
    /** @type {V3} */
    const gyro = [0, 0, 0];
    for (let i = 0; i < 3; i++)
      accel[i] =
        M[3 * i] * f[0] +
        M[3 * i + 1] * f[1] +
        M[3 * i + 2] * f[2] +
        m.accelBias[i] +
        m.accelNoiseDensity[i] * sq * this.rng.gaussian();
    for (let i = 0; i < 3; i++) {
      const n = this.rng.gaussian();
      const rw = this.rng.gaussian();
      this.drift[i] += m.gyroRandomWalk[i] * Math.sqrt(dt) * rw;
      gyro[i] =
        m.gyroScale[i] * w[i] + m.gyroBias[i] + this.drift[i] + m.gyroNoiseDensity[i] * sq * n;
    }
    return { accel, gyro };
  }
}
