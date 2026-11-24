// Live time plots of the simulation on small canvases: one plot per
// quantity group, the last 20 s, a shared time axis, labelled series.
// Colours come from CSS custom properties so both themes read well.

/** @typedef {import("../core/sim/session.js").History} History */

/**
 * @typedef {object} Series
 * @property {string} label
 * @property {number} index value index in the history sample
 * @property {string} color CSS colour or var(--name)
 * @property {boolean} [dashed]
 */

export class Plot {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {string} title
   * @param {string} unit
   * @param {Series[]} series
   * @param {{ min?: number, max?: number }} [range] fixed limits; otherwise from the data
   */
  constructor(canvas, title, unit, series, range = {}) {
    this.canvas = canvas;
    this.title = title;
    this.unit = unit;
    this.series = series;
    this.range = range;
  }

  /**
   * @param {History} h
   * @param {number} now simulated time, s
   * @param {number} span seconds shown
   */
  draw(h, now, span) {
    const c = this.canvas;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = c.clientWidth;
    const ht = c.clientHeight;
    if (w === 0 || ht === 0) return;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(ht * dpr)) {
      c.width = Math.round(w * dpr);
      c.height = Math.round(ht * dpr);
    }
    const g = /** @type {CanvasRenderingContext2D} */ (c.getContext("2d"));
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const css = getComputedStyle(c);
    const fg = css.getPropertyValue("--text").trim() || "#222";
    const muted = css.getPropertyValue("--muted").trim() || "#888";
    const line = css.getPropertyValue("--line").trim() || "#ddd";
    g.clearRect(0, 0, w, ht);
    const L = 38;
    const R = 6;
    const T = 16;
    const B = 14;
    const t0 = now - span;
    let lo = Infinity;
    let hi = -Infinity;
    for (let n = 0; n < h.length; n++) {
      const s = h.at(n);
      if (s.t < t0) continue;
      for (const se of this.series) {
        const v = s.values[se.index];
        if (Number.isFinite(v)) {
          lo = Math.min(lo, v);
          hi = Math.max(hi, v);
        }
      }
    }
    if (this.range.min !== undefined) lo = Math.min(lo, this.range.min);
    if (this.range.max !== undefined) hi = Math.max(hi, this.range.max);
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
      lo = -1;
      hi = 1;
    }
    if (hi - lo < 1e-6) {
      hi += 0.5;
      lo -= 0.5;
    }
    const pad = (hi - lo) * 0.08;
    lo -= pad;
    hi += pad;
    const x = (/** @type {number} */ t) => L + ((t - t0) / span) * (w - L - R);
    const y = (/** @type {number} */ v) => T + (1 - (v - lo) / (hi - lo)) * (ht - T - B);
    g.font = "11px system-ui, sans-serif";
    g.fillStyle = fg;
    g.fillText(`${this.title} (${this.unit})`, L, 11);
    // Axes and zero line
    g.strokeStyle = line;
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(L, T);
    g.lineTo(L, ht - B);
    g.lineTo(w - R, ht - B);
    if (lo < 0 && hi > 0) {
      g.moveTo(L, y(0));
      g.lineTo(w - R, y(0));
    }
    g.stroke();
    g.fillStyle = muted;
    g.textAlign = "right";
    g.fillText(fmt(hi - pad), L - 3, T + 8);
    g.fillText(fmt(lo + pad), L - 3, ht - B);
    g.textAlign = "left";
    g.fillText(`${Math.max(0, t0).toFixed(0)} s`, L, ht - 2);
    g.textAlign = "right";
    g.fillText(`${now.toFixed(1)} s`, w - R, ht - 2);
    g.textAlign = "left";
    // Series
    let lx = L + 6 + g.measureText(`${this.title} (${this.unit})`).width + 8;
    for (const se of this.series) {
      const col = se.color.startsWith("--") ? css.getPropertyValue(se.color).trim() : se.color;
      g.strokeStyle = col;
      g.lineWidth = 1.4;
      g.setLineDash(se.dashed ? [4, 3] : []);
      g.beginPath();
      let started = false;
      for (let n = 0; n < h.length; n++) {
        const s = h.at(n);
        if (s.t < t0) continue;
        const v = s.values[se.index];
        if (!Number.isFinite(v)) {
          started = false;
          continue;
        }
        if (started) g.lineTo(x(s.t), y(v));
        else g.moveTo(x(s.t), y(v));
        started = true;
      }
      g.stroke();
      g.setLineDash([]);
      if (se.label && lx < w - 30) {
        g.fillStyle = col;
        g.fillRect(lx, 5, 8, 3);
        g.fillStyle = fg;
        g.fillText(se.label, lx + 10, 11);
        lx += 16 + g.measureText(se.label).width;
      }
    }
  }
}

/** @param {number} v */
function fmt(v) {
  const a = Math.abs(v);
  return a >= 100 ? v.toFixed(0) : a >= 1 ? v.toFixed(1) : v.toFixed(2);
}
