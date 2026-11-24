// A run's recording as files: what the CLI writes with --out and the page
// offers as a download. Text formats chosen so milestone 4 can turn them
// into a ROS 2 bag without guessing (docs/simulation.md has the topic
// mapping): one file per stream, every row stamped with simulated time.
// Numbers are written with fixed precision, so the same seed gives
// byte-identical files.

import { gridToPgm, mapYaml } from "../../../vendor/lidar-slam-lab/src/export.js";
import { mapPrecision } from "../../../vendor/lidar-slam-lab/src/metrics.js";
import { CONTROL_RATE, DT, SUBSTEPS, simErrors } from "./sim.js";
import { settingsDiff } from "./settings.js";
import { serializeSimWorld } from "./world.js";
import { planarOf } from "./robot.js";

/** @typedef {import("./sim.js").Sim} Sim */
/** @typedef {import("./se2.js").Pose2} Pose2 */

export const RECORDING_FORMAT = "robot-studio-sim-recording/1";

/** @param {number} v @param {number} d */
const f = (v, d) => (Number.isFinite(v) ? (Object.is(v, -0) ? 0 : v).toFixed(d) : "nan");
/** @param {number} v @param {number} d */
const fx = (v, d) => {
  const s = f(v, d);
  return /^-0\.?0*$/.test(s) ? s.slice(1) : s;
};
/** Value as it crosses the wire (float32) and odobridge logs it. @param {number} v */
const f32 = (v) => fx(Math.fround(v), 6);
/** @param {Pose2 | null} p */
const pose = (p) => (p ? [fx(p.x, 5), fx(p.y, 5), fx(p.theta, 6)] : ["nan", "nan", "nan"]);

/**
 * Summary of a run (what summary.json holds, without file hashes).
 * @param {Sim} sim
 * @param {{ route: string, input: string, worldId: string }} meta
 */
export function runSummary(sim, meta) {
  const e = simErrors(sim);
  const r = (/** @type {number} */ v, d = 4) => (Number.isFinite(v) ? Number(v.toFixed(d)) : null);
  const planar = (/** @type {Pose2} */ p) => ({ x: r(p.x, 6), y: r(p.y, 6), theta: r(p.theta, 9) });
  const est = sim.est;
  const map = est && sim.lidar ? mapPrecision(est.grid, sim.lidar.segments, 0.1) : null;
  return {
    format: RECORDING_FORMAT,
    note: "Simulation output of Robot Studio's models (made-up noise and motor parameters unless stated). Not a measurement of any real robot.",
    input: meta.input,
    robot: {
      name: sim.robot.name,
      drive: sim.robot.drive,
      base_frame: sim.robot.baseFrame,
      root_frame: sim.robot.rootFrame,
      wheels: sim.robot.wheels.map((w) => ({ label: w.label, joint: w.joint, axis_sign: w.sign })),
      wheel_radius: r(sim.robot.geometry.wheelRadius, 6),
      half_length: r(sim.robot.geometry.halfLength, 6),
      half_width: r(sim.robot.geometry.halfWidth, 6),
      max_wheel_speed: r(sim.controller.config.maxWheelSpeed, 6),
      ticks_per_wheel_rev: sim.ticksPerRev,
      footprint_radius: r(sim.robot.footprint.radius, 4),
      base_height: r(sim.robot.baseHeight, 6),
      drive_pose: planar(sim.robot.drivePose),
      root_in_base: {
        ...planar(planarOf(sim.robot.rootInBase)),
        z: r(sim.robot.rootInBase.p[2], 6),
      },
      static_transforms: sim.robot.fixedJoints.map((j) => ({
        parent: j.parent,
        child: j.child,
        xyz: j.xyz.map((v) => r(v, 9)),
        rpy: j.rpy.map((v) => r(v, 9)),
      })),
    },
    world: { id: meta.worldId, name: sim.world.name },
    route: meta.route,
    seed: sim.settings.seed,
    settings_changed: settingsDiff(sim.settings),
    timing: {
      control_rate_hz: CONTROL_RATE,
      substeps: SUBSTEPS,
      steps: sim.steps,
      seconds: r(sim.t, 3),
    },
    sensors: {
      lidar: sim.lidar
        ? {
            frame_id: sim.lidar.sensor.link,
            source: sim.lidar.source,
            rate_hz: r(sim.lidar.rateHz, 4),
            beams: sim.lidar.config.beams,
            range_min: sim.lidar.minRange,
            range_max: sim.lidar.config.maxRange,
            range_noise: sim.lidar.config.rangeNoise,
            dropout: sim.lidar.config.dropout,
            height: r(sim.lidar.sensor.height, 4),
          }
        : null,
      imu: sim.imu
        ? {
            frame_id: sim.imu.sensor.link,
            rate_hz: r(CONTROL_RATE / sim.imu.every, 4),
            error_model: sim.imu.source,
          }
        : null,
      encoders: { ticks_per_wheel_rev: sim.ticksPerRev, decoding: "4x quadrature" },
    },
    result: {
      distance_m: r(sim.distance, 3),
      collisions: sim.collisions,
      saturated_steps: sim.saturated,
      final_pose: { x: r(sim.truth.x), y: r(sim.truth.y), theta: r(sim.truth.theta) },
      errors_m: e
        ? {
            wheel_odometry: {
              final: r(e.odom),
              rms: r(e.odomRms),
              max: r(sim.est?.maxOdom ?? NaN),
            },
            scan_matching: sim.settings.estimators.scanMatching
              ? { final: r(e.est), rms: r(e.estRms), max: r(sim.est?.maxEst ?? NaN) }
              : null,
            mcl: sim.est?.pf
              ? { final: r(e.mcl), rms: r(e.mclRms), max: r(sim.est?.maxMcl ?? NaN) }
              : null,
          }
        : null,
      scan_match_accept_rate: e ? r(e.acceptRate) : null,
      map_precision: map
        ? { occupied_cells: map.occupiedCells, within_0_1_m: r(map.precision) }
        : null,
    },
    notes: sim.notes,
  };
}

/**
 * Every file of a recorded run (the sim must have been created with log: true).
 * @param {Sim} sim
 * @param {{ route: string, input: string, worldId: string }} meta
 * @returns {Record<string, string | Uint8Array>}
 */
export function recordingFiles(sim, meta) {
  const log = sim.log;
  if (!log) throw new Error("the simulation was created without logging");
  /** @type {Record<string, string | Uint8Array>} */
  const files = {};
  const t = (/** @type {number} */ v) => fx(v, 2);

  const traj = [
    "t,true_x,true_y,true_theta,odom_x,odom_y,odom_theta,est_x,est_y,est_theta,mcl_x,mcl_y,mcl_theta,cmd_vx,cmd_vy,cmd_wz,true_vx,true_vy,true_wz",
  ];
  for (const r of log.trajectory)
    traj.push(
      [
        t(r.t),
        ...pose(r.truth),
        ...pose(r.odom),
        ...pose(r.est),
        ...pose(r.mcl),
        fx(r.cmd.vx, 4),
        fx(r.cmd.vy, 4),
        fx(r.cmd.wz, 4),
        fx(r.actual.vx, 4),
        fx(r.actual.vy, 4),
        fx(r.actual.wz, 4),
      ].join(","),
    );
  files["trajectory.csv"] = traj.join("\n") + "\n";

  if (sim.lidar) {
    const l = sim.lidar;
    const head = {
      frame_id: l.sensor.link,
      angle_min: Number(l.angleMin.toFixed(6)),
      angle_increment: Number(l.angleIncrement.toFixed(8)),
      range_min: l.minRange,
      range_max: l.config.maxRange,
      scan_time: Number((1 / l.rateHz).toFixed(6)),
      // The sensor frame in the base frame (x, y, yaw; z above the base frame origin).
      mount: {
        x: Number(l.sensor.planar.x.toFixed(6)),
        y: Number(l.sensor.planar.y.toFixed(6)),
        z: Number(l.sensor.mount.p[2].toFixed(6)),
        yaw: Number(l.sensor.planar.theta.toFixed(9)),
      },
    };
    files["scans.jsonl"] =
      JSON.stringify({ header: head, note: "ranges in metres, null = no return" }) +
      "\n" +
      log.scans
        .map(
          (s) =>
            `{"t":${t(s.t)},"ranges":[${Array.from(s.ranges, (v) => (Number.isNaN(v) ? "null" : fx(v, 4))).join(",")}]}`,
        )
        .join("\n") +
      "\n";
  }
  if (sim.imu) {
    const rows = ["t,ax,ay,az,gx,gy,gz"];
    for (const r of log.imu)
      rows.push(
        [t(r.t), ...r.accel.map((v) => fx(v, 5)), ...r.gyro.map((v) => fx(v, 6))].join(","),
      );
    files["imu.csv"] = rows.join("\n") + "\n";
  }
  const labels = ["fl", "fr", "rl", "rr"];
  const ws = [
    "t_us," + labels.map((w) => `count_${w},velocity_${w},target_${w},duty_${w}`).join(","),
  ];
  for (const m of log.wheelState)
    ws.push(
      [
        String(m.t_us),
        ...labels.flatMap((_, i) => [
          String(m.count[i]),
          f32(m.velocity[i]),
          f32(m.target[i]),
          String(m.duty[i]),
        ]),
      ].join(","),
    );
  files["wheel_state.csv"] = ws.join("\n") + "\n";
  // Same columns and formatting as odobridge's --csv odometry log.
  const od = ["t_s,x,y,theta,vx,vy,wz"];
  for (const m of log.odometry)
    od.push([fx(m.t_us / 1e6, 6), ...[m.x, m.y, m.theta, m.vx, m.vy, m.wz].map(f32)].join(","));
  files["odometry.csv"] = od.join("\n") + "\n";

  const names = sim.robot.wheels.map((w) => w.joint);
  const js = [
    "t," + names.map((n) => `pos:${n}`).join(",") + "," + names.map((n) => `vel:${n}`).join(","),
  ];
  for (const r of log.jointStates)
    js.push(
      [t(r.t), ...r.position.map((v) => fx(v, 5)), ...r.velocity.map((v) => fx(v, 4))].join(","),
    );
  files["joint_states.csv"] = js.join("\n") + "\n";

  if (sim.est) {
    files["map.pgm"] = gridToPgm(sim.est.grid);
    files["map.yaml"] = mapYaml(sim.est.grid, "map.pgm");
  }
  files["world.json"] = serializeSimWorld(sim.world);
  files["summary.json"] = JSON.stringify(runSummary(sim, meta), null, 2) + "\n";
  return files;
}

export { DT };
