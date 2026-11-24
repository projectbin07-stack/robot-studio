# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

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
