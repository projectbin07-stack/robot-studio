// Template cases whose exported files are committed under
// tests/corpus/builder/ (written by scripts/builder-fixtures.js) and
// checked against the official xacro and mecanum-kinematics in CI.

import { generate } from "../src/core/builder/templates.js";
import { toUrdf } from "../src/core/builder/urdf.js";
import { exportXacro } from "../src/core/builder/export.js";
import { modelFromDoc } from "../src/core/builder/editor.js";
import {
  mecanumConfig,
  mecanumForward,
  mecanumGeometry,
  mecanumInverse,
  wheelBase,
} from "../src/core/builder/drive.js";

/**
 * @typedef {object} BuilderCase
 * @property {string} id file stem
 * @property {string} template
 * @property {Record<string, unknown>} params
 */

/** @type {BuilderCase[]} */
export const BUILDER_CASES = [
  { id: "mecanum", template: "mecanum", params: {} },
  {
    id: "mecanum-variant",
    template: "mecanum",
    params: {
      name: "mecanum_large",
      wheel_radius: 0.0485,
      wheel_width: 0.04,
      wheelbase: 0.31,
      track: 0.335,
      base_length: 0.42,
      base_width: 0.26,
      roller_count: 6,
      lidar: false,
      camera: true,
      camera_rpy: [0, 0.2, 0],
    },
  },
  { id: "diff", template: "diff", params: {} },
  { id: "arm-2dof", template: "arm", params: { name: "arm_2dof", dof: 2, camera: true } },
  { id: "arm-6dof", template: "arm", params: {} },
];

/** Body twists the interop check runs through inverse and forward kinematics. */
export const TWISTS = [
  { vx: 0.3, vy: 0, wz: 0 },
  { vx: 0, vy: 0.25, wz: 0 },
  { vx: 0, vy: 0, wz: 1.2 },
  { vx: 0.2, vy: -0.15, wz: 0.7 },
  { vx: -0.12, vy: 0.31, wz: -0.45 },
];

/**
 * Exported files of one case, by file name.
 * @param {BuilderCase} c
 * @returns {Record<string, string>}
 */
export function builderCaseFiles(c) {
  const doc = generate(c.template, c.params);
  /** @type {Record<string, string>} */
  const out = {
    [`${c.id}.urdf.xacro`]: exportXacro(doc).text,
    [`${c.id}.urdf`]: toUrdf(doc),
  };
  const model = modelFromDoc(doc);
  const toml = mecanumConfig(model);
  const g = mecanumGeometry(wheelBase(model));
  if (toml.ok && g) {
    out[`${c.id}.toml`] = toml.toml;
    const p = /** @type {Record<string, number>} */ (doc.template?.params);
    const reference = {
      note: "Robot Studio's numbers for scripts/mecanum_interop.py: template parameters, wheel positions measured from the URDF by urdf-inspector, and wheel speeds (rad/s, FL FR RL RR, positive drives forward) from Robot Studio's mecanum kinematics.",
      template: {
        wheel_radius: p.wheel_radius,
        half_length: p.wheelbase / 2,
        half_width: p.track / 2,
      },
      wheels: g.wheels,
      cases: TWISTS.map((t) => {
        const speeds = mecanumInverse(g, t);
        return { twist: t, speeds, forward: mecanumForward(g, speeds) };
      }),
    };
    out[`${c.id}.reference.json`] = JSON.stringify(reference, null, 2) + "\n";
  }
  return out;
}
