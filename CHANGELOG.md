# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [1.0.0] - 2026-11-25

Milestone 5, "Release polish": documentation, accessibility, a
performance budget and the upstream fixes. Validation: software tests;
simulation (labelled simulated); hardware none: no robot has been connected
and no robot bag has been read (milestone 4's criterion 7 is still open).

### Added

- Documentation pages for every feature (`docs/README.md`: getting
  started, model, builder, simulation, ROS 2, command line, keyboard and
  accessibility, xacro), with screenshots taken from the live site by a new
  `screenshots` job in the Pages workflow after each deploy.
- First run: a **Start here** panel over the 3D view on a plain visit
  (open or build a robot, simulate it, connect to ROS 2, one button each)
  and a four-step guided example that does each step in the page; **Start
  here**, **Docs** and **Source** in the header.
- `robot-studio studio [--ros URL] [--port N] [--no-open]`: serves the page
  on 127.0.0.1 only and opens it with the rosbridge address filled in; with
  `npx github:projectbin07-stack/robot-studio studio` no clone is needed. A
  page served from the computer reaches `ws://` on the local network without
  mixed-content blocking or a permission prompt (tested with Chrome 157 and
  Edge 158).
- The ROS 2 panel says what the browser will do with `ws://` from the https
  site (local network permission for IP addresses and `.local`, mixed
  content for other names) and when local network access was denied.
- Accessibility: the kinematic tree is a WAI-ARIA tree (one Tab stop, arrow
  keys, Home/End, Enter); visible focus on every control; the folder and
  world-file inputs are reachable with the keyboard; the template form keeps
  the focus after it re-renders. axe-core 4.14.0 (vendored, MPL-2.0, CI only)
  runs after every browser scenario, and keyboard-only scenarios run in both
  themes. On the development machine axe-core found 2 to 3 rules violated on
  3 to 16 elements per page state in 0.4.0 and none now
  ([docs/accessibility.md](docs/accessibility.md)).
- Performance budget: `scripts/large-model.mjs` generates a 200-link
  package with 51.0 MB of STL meshes; the CI browser job opens it and fails
  above 10 s. Measured: 2.8 s in CI (headless Chrome 157, SwiftShader),
  2.4 s on the development machine (headless Edge 158).
- Settings `estimators.mclOmni` (lidar-slam-lab's omnidirectional MCL motion
  model, on mecanum bases, default on) and `estimators.mclRecovery`
  (random-particle recovery, default off).
- Browser smoke test options `--axe`, `--keyboard`, `--tour`, `--shots`,
  `--max-open-ms`, `--grant-local-network`, `--theme`, `--close-welcome`;
  the fake rosbridge server's `--host`; `scripts/loopback-buffer.mjs`.

### Changed

- Vendored urdf-inspector v0.2.0 (was v0.1.1): its `meshExists` option
  reports `mesh-missing`, findings get their file from `XmlElement.source`,
  and the joint axes use `jointFrames`; the studio's own versions are gone.
- Vendored lidar-slam-lab v0.2.1 (was v0.1.0): scans carry the LiDAR mount
  as their origin (the studio's re-expressed scans are gone), the minimum
  range is lidar-slam-lab's, and the MCL estimate is taken before
  resampling.
- MCL tracks without random-particle recovery by default. MCL RMS over
  seeds 1 to 8 (development machine): TurtleBot3 in the corridor loop
  0.129 to 7.438 m before, 0.063 to 0.168 m now (seed 1 over 20 s: 5.455 m
  before, 0.120 m now); the mecanum template's square in the lab 0.231 to
  0.237 m before, 0.017 to 0.030 m now; the differential template's square
  0.131 to 0.155 m before, 0.027 to 0.075 m now; a replay of a differential
  run 0.200 m before, 0.021 m now. Simulation output, not measurements.
- rosbag-inspect v0.3.0 (was v0.2.0; per-topic Hz for MCAP, ROS 1 bags
  named, schema-v4 definitions); CI checks the rates on the exported bags.
- odobridge v0.2.0 (was v0.1.0) in the firmware cross-check: the same model
  with a declared, documented API.
- The golden digest and the bag and rosbridge fixtures were regenerated
  (estimates changed; the drive, sensors and telemetry did not).
- Contrast in both themes: the light theme's accent, warning and
  "simulated" badge colours are darker; text on the accent is dark in the
  dark theme.

### Fixed

- Closing the simulation or a replay left the 3D view empty: the camera
  stayed where the robot had driven. The view is framed on the model again.
- The template form moved the keyboard focus out of it after a parameter
  edit.

## [0.4.0] - 2026-11-25

Milestone 4, "ROS 2". Validation: software tests; simulation (bags,
replays and comparisons made from simulated runs are labelled simulated);
hardware none: no robot has been connected and no robot bag has been read.

### Added

- ROS 2 panel ([docs/ros2.md](docs/ros2.md)): a live robot over rosbridge
  with a rosbridge v2 client written from the protocol (subscribe,
  advertise, publish, call_service, status, fragments, reconnect);
  `/joint_states`, `/tf`, `/tf_static`, `/scan`, `/imu`, `/odom` and
  `robot_description` shown with the simulation's 3D layers, plots and HUD
  under a LIVE badge, the LiDAR at its `/tf_static` mount, scan matching on
  the robot's odometry, and a per-topic rate / age / rejected table. The
  address is checked and a link (`?ros=`) only pre-fills it.
- Driving a real robot: `/cmd_vel` from the keyboard, gamepad and touch
  teleop through a dead-man gate (explicit per-arming opt-in, 10 Hz while
  held, zero on release, blur, stale input, hide and disconnect, the last
  two disarming; speed capped at 10 to 100 % of the URDF wheel limit;
  "DRIVING A REAL ROBOT" banner).
- ROS 2 bag export: `robot-studio sim --bag`, `robot-studio bag`, and
  "Download ROS 2 bag" in the Simulate panel write rosbag2 directories
  (MCAP + `metadata.yaml`, CDR, Humble definitions) with `/tf_static`,
  `/tf`, `/odom`, `/scan`, `/imu`, `/joint_states`, `/cmd_vel`,
  `/firmware/wheel_state`, `/ground_truth` and `/robot_studio/source`
  (simulated: true).
- `robot-studio-bag` (`python/`): turns a ROS 2 bag into the recording
  format using rosbag-inspect v0.2.0 as a library; LaserScan arrays kept,
  inf / NaN / out-of-range as no return, LiDAR mount from `/tf_static`.
- Replay of recordings through the estimators and views (`robot-studio
replay`, "Replay a recording"), labelled Replay / SIMULATED data.
- Simulation vs recording (`robot-studio compare`, "Compare with the
  simulator"): a recording's `/cmd_vel` schedule on the simulator, compared
  through the firmware's ODOMETRY and WHEEL_STATE fields.
- Tests against an in-repo fake rosbridge server (RFC 6455 on
  `node:http`) with message fixtures from a simulated run; CI jobs for the
  Python companion, the bag round trip value by value, and ROS 2 Humble
  reading the exported bag (`ros2 bag info`, `rosbag2_py` + `rclpy`); the
  browser job drives the live view against the fake server and checks the
  Twists it received.

### Changed

- The recording's scans header carries the LiDAR mount, and `summary.json`
  the URDF's fixed joints, base height, drive pose and root frame offset
  (the golden digest changed for those two files only).
- The 3D simulation layer draws a view frame (`SimSession.frame()`), shared
  by simulation, replay and live robot.
- Version 0.4.0.

### Fixed

- A plot with no data labelled its axis "NaN".

## [0.3.0] - 2026-11-24

Milestone 3, "simulation". Validation: software tests; simulation results
are model output, labelled simulated everywhere; no hardware results.

### Added

- Simulation of the open or built robot (`src/core/sim/`, details in
  [docs/simulation.md](docs/simulation.md)): mecanum (four wheels) or
  differential (two) detected from the URDF by the vendored urdf-inspector;
  fixed 10 ms steps; deterministic for a seed.
- Drive mirrored from encoder-odometry-firmware v0.1.0: inverse kinematics,
  direction-preserving saturation, blended velocity estimator, per-wheel
  PI with feed-forward, anti-windup and slew limit, watchdog, SE(2) wheel
  odometry; its Python model's motor and exact-edge encoder plant; optional
  wheel slip and wheel radius error; ideal-wheel mode.
- Sensors at the URDF's sensor links (poses from forward kinematics): 2D
  LiDAR with lidar-slam-lab's ray caster against obstacles taller than the
  scan plane, minimum range and rate; IMU with gravity, rotation rate and
  centripetal acceleration and an error model from imu-calibration-lab's
  JSON export, Gazebo noise or defaults; 4x encoders; a depth camera point
  cloud. Gazebo `<sensor>` tags set LiDAR and IMU parameters.
- Estimators through the vendored lidar-slam-lab v0.1.0 modules: wheel
  odometry, odometry + scan matching with the occupancy grid built from the
  estimate, MCL; live and RMS errors against the truth.
- Worlds with heights (walls, boxes, cylinders): lidar-slam-lab's office,
  warehouse and loop, a lab and an empty room; a top-down world editor;
  worlds, routes and settings kept in the link and in the robot document
  (undoable `setSimulation` operation).
- Simulate panel: run, pause, reset, speed, routes (demo autopilot, square,
  spin), keyboard / gamepad / touch teleop; 3D layers for LiDAR rays and
  returns, the map, trajectories, MCL particles, the depth cloud and
  map/odom frames; following camera; HUD with errors, frame rate and
  measured step time; live plots of wheel speeds, twist, IMU and errors;
  recording download.
- CLI `robot-studio sim <robot|template>` with `--world`, `--route`,
  `--seconds`, `--seed`, `--out`, `--imu-cal`, `--set`, `--ideal`, `--json`:
  trajectory, LiDAR scans, IMU, the firmware's WHEEL_STATE and ODOMETRY
  telemetry, joint states, map and summary; byte-identical for a seed.
- Recording format with its planned mapping to ROS 2 topics (`/scan`,
  `/imu`, `/odom`, `/joint_states`, `/tf`, `/tf_static`).
- Tests against analytic results (IK/FK, strafe, saturation, encoder counts,
  IMU at rest and turning, LiDAR ranges, sensor poses), the estimator
  wrapper against lidar-slam-lab's own loop, determinism and a golden
  digest on every CI platform.
- CI: simulated wheel speeds against mecanum-kinematics'
  `inverse_saturated()`; every telemetry frame against
  encoder-odometry-firmware's Python model (new job); the estimators against
  lidar-slam-lab's CLI benchmark (new job, 18 of 18 rows identical); CLI
  determinism; a browser scenario that drives the mecanum and differential
  templates and checks movement, errors and plots.

### Changed

- lidar-slam-lab v0.1.0 is vendored next to urdf-inspector, with the same
  hash manifest and CI check.
- The browser smoke script waits for a new template robot correctly (it
  used to wait until its timeout), brings its tab to the front, and can use
  the GPU in a visible window (`--headed --gpu`). On Windows it now ends every
  browser process started with its profile directory; before, Edge's
  renderer and software-rendering GPU processes outlived each run.

## [0.2.0] - 2026-11-24

Milestone 2, "builder". Validation: software tests only; no simulation or
hardware results.

### Added

- Visual builder in the page: robots from templates (four-wheel mecanum
  base, differential base with caster, N-joint serial arm) with optional
  2D LiDAR, IMU, camera and depth camera mounts at user-chosen poses;
  any opened URDF/xacro/package can be imported as an editable robot.
- Properties panel for the selected link or joint (numeric fields): rename
  with reference propagation, re-parent keeping the world pose, add child,
  add sensor, duplicate and delete subtrees, joint type, axis and limits,
  origins, geometry, colour, mass and inertia.
- Inertia from geometry with urdf-inspector's `primitiveInertia` and
  `rotateTensor` (box, cylinder, sphere, mesh bounding box).
- Undo/redo of every edit (toolbar, Ctrl+Z, Ctrl+Y / Ctrl+Shift+Z), Delete
  key, checks re-run after every edit, autosave in local storage
  (`?s=builder` restores it).
- Exports: URDF; parametrised xacro for template robots (properties,
  sensor arguments, macros); mecanum-kinematics TOML through
  urdf-inspector's `mecanumToml`; ROS 2 description package zip
  (package.xml, CMakeLists.txt, urdf/, meshes/, launch/display.launch.py,
  rviz/display.rviz).
- CLI: `robot-studio new`, `robot-studio export`, `robot-studio templates`.
- Tests: every edit operation with undo/redo, rename propagation,
  templates (no errors or warnings), inertia, import/export round trips on
  the bundled examples and the golden corpus, package zip, autosave, a
  51-link edit timing.
- CI: the official xacro expands the exported template xacro (7 new golden
  cases); mecanum-kinematics v0.1.0 loads the exported TOML and its IK/FK
  are compared with Robot Studio's; exported packages are built with colcon
  and checked with check_urdf, ROS xacro, `ros2 launch --show-args` and
  robot_state_publisher in `ros:humble-ros-base`; the browser job creates a
  mecanum robot through the UI, edits a parameter, undoes it with Ctrl+Z,
  exports and reloads.

### Changed

- The 3D view keeps the camera when an edit redraws the model and reuses
  parsed meshes.
- `actions/upload-artifact` 6 -> 7 (Dependabot #1).

## [0.1.0] - 2026-11-24

Milestone 1, "open any robot model". Validation: software tests only; no
simulation or hardware results.

### Added

- xacro expansion in JavaScript (browser and Node): properties (lazy, scoped,
  blocks, default/remove/scope/lazy_eval), args and `$(arg)`, macros with
  defaults, `^`/`^|` inheritance, `*`/`**` block parameters, `xacro:if` /
  `xacro:unless`, includes with `$(find)`, namespaces, globs and `optional`,
  `xacro:element`, `xacro:attribute`, `xacro:call`, `$(eval)`, `$(env)`,
  `$(optenv)`, `$(cwd)`, `$(dirname)`, comment handling and xacro's output
  layout. Errors with file:line and macro/include trace.
- A safe evaluator for the Python expression subset used in `${}` with
  Python's int/float semantics and number formatting.
- Golden corpus of 38 cases compared with the official xacro 2.1.1 in CI: 14
  expansions byte-identical (including TurtleBot3 Burger/Waffle/Waffle Pi,
  the ROS URDF tutorial and ros2_control_demos RRBot), 2 equal except for
  1-ulp differences from `sin`/`cos`, 21 errors matched, 1 documented gap
  (lambda).
- Loading of a URDF or xacro file, a whole description folder or a zip
  (stored/deflate reader with CRC-32), with `package://` and relative mesh
  resolution and an install-layout fallback.
- three.js 3D view: STL, OBJ and COLLADA meshes (textures, unit scale, up
  axis as RViz), visual/collision switch, link frames, joint axes, centre of
  mass, click selection, joint sliders driving urdf-inspector's forward
  kinematics, shadows, light and dark themes, phone layout.
- urdf-inspector v0.1.1 checks on the expanded model with findings linked
  to the source file, the tree and the 3D selection; Robot Studio's own
  `mesh-missing`, `xacro-error`, `xacro-warning` and `file-fallback` findings.
- Kinematic tree, details panel, xacro argument form, expanded URDF
  download, URL state.
- Bundled examples: studio_mecanum (our own xacro model with LiDAR, IMU and
  depth camera), TurtleBot3 Burger, the URDF tutorial R2D2 and RRBot, with
  attribution.
- CLI: `robot-studio xacro` and `robot-studio check` with urdf-inspector's
  exit codes.
- Vendoring of urdf-inspector v0.1.1 and three.js 0.187.0 with a SHA-256
  manifest and a CI job that re-fetches and compares them.
- CI on Ubuntu and Windows with Node 22 and 24, xacro golden job, vendor
  integrity job, headless Chrome smoke test with WebGL, GitHub Pages
  deployment.
