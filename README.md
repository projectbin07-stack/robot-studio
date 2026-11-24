# Robot Studio

[![CI](https://github.com/projectbin07-stack/robot-studio/actions/workflows/ci.yml/badge.svg)](https://github.com/projectbin07-stack/robot-studio/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Open, build and simulate a robot model in the browser. Open a URDF or
xacro file, a whole ROS description package or a zip: Robot Studio expands
the xacro itself, checks the model, and shows it in 3D with its meshes and
joint sliders. Or build one: start from a mecanum, differential or arm
template, edit links and joints, and export URDF, xacro, a
mecanum-kinematics config or a ROS 2 description package. Then drive it in a
simulated world: wheels, encoders, IMU and 2D LiDAR at the robot's own URDF
frames, with wheel odometry, scan matching and MCL compared against the
simulated truth. No ROS install, no upload.

**Live:** <https://projectbin07-stack.github.io/robot-studio/>

> **Validation status (v0.3.0):** software tests: yes (Node tests on
> Ubuntu and Windows, xacro compared with the official `xacro`, builder
> exports checked with the official xacro, mecanum-kinematics and colcon).
> Simulation: yes, labelled as model output everywhere; the simulator is
> cross-checked in CI against mecanum-kinematics, encoder-odometry-firmware's
> own firmware model and lidar-slam-lab's own benchmark, which is agreement
> between models, not validation. Hardware: none; nothing has been run
> against a real robot. See [Validation status](#validation-status).

![Simulation in the light theme: the mecanum template driving lidar-slam-lab's office map on its demo route, with LiDAR returns, trajectories of the truth and of each estimate, the error HUD and live plots](docs/img/sim-mecanum-office-light.png)

## What it is and where it is going

Robot Studio is the flagship of this account's robotics tools: one browser
app that ties together the separate repositories instead of duplicating
them. It is built in five milestones ([ROADMAP](docs/ROADMAP.md)):

1. **Model (0.1):** open any model; xacro expansion; 3D view with
   meshes; live checks from [urdf-inspector](https://github.com/projectbin07-stack/urdf-inspector).
2. **Builder (0.2):** create and edit links and joints,
   templates (mecanum, diff drive, arm) with sensor mounts, export URDF,
   xacro, a [mecanum-kinematics](https://github.com/projectbin07-stack/mecanum-kinematics)
   config and a ROS 2 package.
3. **Simulation (0.3, this release):** drive the robot with
   [encoder-odometry-firmware](https://github.com/projectbin07-stack/encoder-odometry-firmware)'s
   wheel control, LiDAR, scan matching and MCL from
   [lidar-slam-lab](https://github.com/projectbin07-stack/lidar-slam-lab), an
   IMU with [imu-calibration-lab](https://github.com/projectbin07-stack/imu-calibration-lab)'s
   calibration as its error model, TF and plots, always labelled simulated.
4. **ROS 2:** a rosbridge client for a live robot view and `cmd_vel`, and
   bag replay through [rosbag-inspect](https://github.com/projectbin07-stack/rosbag-inspect).
5. **Release polish:** docs, performance budget, accessibility, upstream
   fixes.

## Simulation (new in 0.3)

Press **Simulate this robot** in the Simulate panel (or open a link with
`?sim=1`). Everything is model output and labelled "simulated".

- **The robot you opened or built**: mecanum (four wheels) or differential
  (two) detected from the URDF by urdf-inspector; the LiDAR, IMU and depth
  camera sit at the poses forward kinematics gives their links; LiDAR range,
  beams, noise and rate and IMU noise come from Gazebo `<sensor>` tags when
  the URDF has them (TurtleBot3's do).
- **Drive**: twist → inverse kinematics → saturation → per-wheel PI loop →
  motor → encoders → slip → forward kinematics → pose, mirrored from
  encoder-odometry-firmware so the simulated robot reports the firmware's
  own telemetry fields; fixed 10 ms steps, deterministic for a seed.
- **Sensors**: 2D LiDAR with lidar-slam-lab's ray caster against obstacles
  taller than its scan plane; IMU with gravity, rotation rate, noise and
  bias, taking imu-calibration-lab's JSON export as its error model; 4x
  encoders; a depth camera point cloud.
- **Estimators** from lidar-slam-lab: wheel odometry, odometry + scan
  matching with the occupancy map it builds, MCL; live and RMS errors
  against the truth.
- **Worlds with heights**: lidar-slam-lab's office, warehouse and loop, a lab
  with boxes and cylinders from 4 cm to 1.2 m, an empty room; edit walls,
  boxes, cylinders, start and route in a top-down editor; the world is kept
  in the link and in the robot document.
- **View**: the robot driving in 3D with its wheels turning, LiDAR rays and
  returns, the map, trajectories of the truth and each estimate, particles,
  the depth cloud, map/odom frames; live plots of wheel speeds, commanded and
  actual twist, IMU and errors; a HUD with errors, frame rate and the
  measured step time.
- **Drive it**: the world's demo route (autopilot), a square (strafing on a
  mecanum base), a spin, or by hand with the keyboard (W/A/S/D, Q/E strafe),
  a gamepad or on-screen sticks.
- **Record**: download the run (trajectory, LiDAR scans, IMU, the firmware's
  WHEEL_STATE and ODOMETRY telemetry, joint states, map, summary), laid out
  for a ROS 2 bag export in milestone 4.

```console
$ node bin/robot-studio.js sim mecanum --world office --route demo --seconds 60 --seed 1 --out run/
SIMULATION: model output of Robot Studio's drive, sensor and estimator models; not a measurement.
robot     mecanum_bot: mecanum, 4 wheels, r 0.040 m, lx 0.100 m, ly 0.125 m, max 20.0 rad/s
world     Office; route demo (waypoint 8 of 19); 60.0 s simulated, seed 1
sensors   LiDAR lidar_link: 180 beams, 10.0 Hz, 0.12-8.0 m (settings)
          IMU imu_link: 100 Hz, defaults (MPU-6050 datasheet typical noise density, no bias)
          encoders: 38000 counts per wheel revolution (4x), firmware control loop 100 Hz
motion    25.99 m driven, 0 blocked step(s), 0 saturated step(s)
position error   final     RMS       max
wheel odometry     1.350 m   0.746 m   1.350 m
+ scan matching    0.021 m   0.059 m   0.096 m   (100.0 % of matches accepted)
MCL (300 particles) 0.034 m   0.035 m   0.046 m
step time 1.200 ms per 10 ms step (8.3x real time; measured on this machine, not written to the files)
wrote     run/ (10 files)
```

(Step time from a GitHub Actions Ubuntu runner; the other lines are the
same on every platform CI runs on. Running the same command twice writes
byte-identical files.)

How each model works, every default and where it comes from, the recording
format with its ROS 2 topic mapping, the cross-check results and measured
performance: [docs/simulation.md](docs/simulation.md).

| Lab world with boxes and cylinders, dark theme (orange: lower than the LiDAR)                                                                                | Phone width                                                    |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------- |
| ![Mecanum template in the lab world: a tall box and cylinders, a low cylinder in orange, LiDAR returns, the map and trajectories](docs/img/sim-lab-dark.png) | ![The simulation at phone width](docs/img/sim-phone-light.png) |

## Builder (new in 0.2)

- **Templates:** four-wheel mecanum base, differential base with caster,
  serial arm with 1 to 50 revolute joints. Dimensions, wheel radius and
  width, wheelbase, track, masses and limits are parameters; LiDAR, IMU,
  camera and depth camera mounts are switched on and placed by you. Every
  template robot has no errors or warnings from urdf-inspector's checks.
- **Edit anything:** select a link or joint in the 3D view or the tree and
  edit it in the Properties panel (numeric fields): name (references
  follow), joint type, parent (re-parent keeps the world pose), origin,
  axis, limits, geometry, colour, mass, inertia. Add child, add sensor,
  duplicate and delete subtrees. Undo/redo with Ctrl+Z / Ctrl+Y; Delete
  removes the selected link. Checks and the 3D view update after every edit.
- **Edit an opened model:** any URDF, xacro, package folder or zip becomes
  an editable robot (its expansion; meshes still resolve).
- **Inertia from geometry** with urdf-inspector's formulas, rotated with
  the shape (so it passes the `inertia-axes` check).
- **Exports:** URDF; xacro with the template parameters as
  `xacro:property` and sensor switches as `xacro:arg`; a
  mecanum-kinematics TOML for mecanum bases; a ROS 2 description package as
  a zip (package.xml, CMakeLists.txt, urdf/, meshes/, launch file with
  robot_state_publisher, joint_state_publisher_gui and RViz).
- **Autosave** of the working robot in this browser's local storage
  (`?s=builder` restores it); never uploaded.

Details, formats and how each export is verified: [docs/builder.md](docs/builder.md).

| Arm template, dark theme                                                   | Imported R2D2 tutorial robot being edited                                         |
| -------------------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| ![Six-joint arm template in the dark theme](docs/img/builder-arm-dark.png) | ![URDF tutorial robot imported into the builder](docs/img/builder-r2d2-light.png) |

## Viewer features (0.1)

- **xacro in the browser and the CLI**, written from the xacro docs and
  checked against the official implementation: properties (lazy, scoped,
  blocks), args, macros with defaults, `^` / `^|` inheritance and `*` / `**`
  block parameters, conditionals, includes with `$(find)`, namespaces and
  globs, `xacro:element` / `xacro:attribute` / `xacro:call`, `$(eval)`, and
  `${}` expressions evaluated by a **safe Python-subset evaluator** (no
  `eval`) with Python's int/float semantics. Errors name file:line and the
  macro and include trace. Full list and the honest unsupported list:
  [docs/xacro.md](docs/xacro.md).
- **Open anything:** drop a file, a folder or a `.zip` (built-in zip reader),
  or use the folder picker. `package://` and relative mesh paths resolve
  inside what you dropped; missing meshes show a magenta placeholder and a
  finding.
- **3D view** (three.js): STL, OBJ and COLLADA meshes with colours and
  textures, orbit/pan/zoom, shadows, grid, visual / collision / both, link
  frames, joint axes, centre of mass, click to select, Z up (REP 103).
- **Joint sliders** that respect limits (continuous joints ±π, mimic joints
  follow) and drive urdf-inspector's forward kinematics live.
- **Live checks:** urdf-inspector's 72 checks run on the expanded model;
  each finding names the file and line it came from and selects its link or
  joint. Robot Studio adds `mesh-missing` and xacro warnings.
- Kinematic tree, details panel (pose, inertia, geometry, mesh status),
  xacro argument form, download of the expanded URDF, shareable links,
  light and dark themes, phone layout.
- **Examples:** studio_mecanum (written for this project, illustrative),
  TurtleBot3 Burger (ROBOTIS, Apache-2.0), the ROS URDF tutorial robot
  (BSD-3-Clause) and ros2_control_demos RRBot (Apache-2.0), unmodified and
  attributed ([sources](tests/corpus/SOURCES.md)).
- **CLI:** `robot-studio xacro` and `robot-studio check`.

| Dark theme, R2D2 tutorial robot with COLLADA gripper meshes                           | Phone width                                               |
| ------------------------------------------------------------------------------------- | --------------------------------------------------------- |
| ![URDF tutorial robot in the dark theme with a selected link](docs/img/r2d2-dark.png) | ![studio_mecanum at phone width](docs/img/phone-dark.png) |

![TurtleBot3 Burger from its unmodified xacro description, with findings](docs/img/turtlebot3-light.png)

## Quick start

In the browser: open the [live page](https://projectbin07-stack.github.io/robot-studio/)
and drop your description package folder (or a zip of it) onto it. The
best candidate file (`*.urdf.xacro` with a `<robot name=...>`) opens; pick
another one in "File to open".

Locally:

```bash
git clone https://github.com/projectbin07-stack/robot-studio.git
cd robot-studio
npm ci                 # dev tools only
npm run serve          # http://127.0.0.1:8000/
```

CLI (Node 22 or newer; `sim` is shown [above](#simulation-new-in-03)), viewer:

```console
$ node bin/robot-studio.js check examples/studio_mecanum/urdf/studio_mecanum.urdf.xacro
examples/studio_mecanum/urdf/studio_mecanum.urdf.xacro (xacro, 4 file(s))
  robot 'studio_mecanum': 11 links, 10 joints (4 continuous, 6 fixed), root base_footprint
  mass 3.054 kg, centre of mass (0.0065, 0.0000, 0.0592) m in base_footprint
  meshes: 2 of 2 found
  0 error(s), 0 warning(s), 1 note(s)

INFO  wheel-base-summary  (examples/studio_mecanum/urdf/mecanum_wheel.xacro:50)
      4 drive wheel(s) in base_link (FL=front_left_wheel_joint, FR=front_right_wheel_joint, RL=rear_left_wheel_joint, RR=rear_right_wheel_joint), radius 0.04 m, lx 0.1 m, ly 0.1225 m (export-mecanum writes a mecanum-kinematics config)
      at /robot/joint[@name='front_left_wheel_joint']

$ node bin/robot-studio.js xacro my_robot.urdf.xacro use_camera:=false -o my_robot.urdf
```

```
robot-studio xacro <in.xacro> [--arg name=value ...] [name:=value ...] [-o out.urdf]
robot-studio check <robot.urdf|robot.xacro> [--arg name=value ...] [--json] [--strict]
robot-studio sim <robot.urdf|robot.xacro|template> [--world W] [--route R] [--seconds S]
                 [--seed N] [--out DIR] [--imu-cal FILE] [--set key=value] [--ideal] [--json]
  --package name=dir   folder of a ROS package for $(find) and package:// (repeatable);
                       packages next to the input file's package are found automatically
Exit codes: 0 no errors (and no warnings with --strict), 1 warnings with --strict,
            2 xacro failed or the model has errors, 3 usage or file error.
```

Builder from the command line:

```console
$ node bin/robot-studio.js templates
$ node bin/robot-studio.js new mecanum --set wheel_radius=0.05 --set camera=true -o bot.urdf
$ node bin/robot-studio.js new mecanum -f toml
$ node bin/robot-studio.js new arm --set dof=6 -f xacro -o arm.urdf.xacro
$ node bin/robot-studio.js export my_robot.urdf.xacro -f package -o my_robot_description.zip
```

`new` and `export` take `--format urdf|xacro|toml|package` (package needs
`-o file.zip`). `robot-studio new mecanum -f toml` prints:

```toml
# Generated by urdf-inspector from the URDF geometry. Check every value
# against the real robot before using it for odometry.
name = "mecanum_bot"
description = "Measured from URDF 'mecanum_bot' (frame base_link)"

wheel_radius = 0.04      # m, mean of the wheel cylinders
half_length = 0.1       # m, lx: wheel centroid to wheel centre along x
half_width = 0.125        # m, ly: wheel centroid to wheel centre along y
roller_angle_deg = 45.0   # not in the URDF; 45 for ordinary mecanum wheels
# ticks_per_rev and gear_ratio are not in the URDF; add them for encoder odometry.
max_wheel_speed = 20   # rad/s, smallest wheel <limit velocity>
wheel_order = ["FL", "FR", "RL", "RR"]
...
```

`robot-studio xacro` prints the same text the official `xacro` prints (the
CI test compares the CLI's TurtleBot3 output with the official one byte for
byte), minus xacro's "autogenerated" banner.

## Architecture

```mermaid
flowchart LR
  subgraph input[Input]
    F[file / folder / zip]
    E[bundled examples]
  end
  subgraph core["src/core (DOM-free, tested in Node)"]
    V[vfs.js + zip.js<br/>packages, package://]
    X[xacro/<br/>dom, parser, evaluator,<br/>tables, processor]
    M[model.js<br/>findings with files, meshes]
    S[scene.js<br/>shapes, poses, sliders]
    B[builder/<br/>document, edit ops, undo,<br/>templates, exports]
    SIM[sim/<br/>firmware mirror, plant, sensors,<br/>worlds, estimators, recording]
  end
  subgraph vendor["vendor/ (pinned, hash-checked)"]
    UI[urdf-inspector v0.1.1<br/>URDF reader, 72 checks,<br/>FK, inertia, wheels]
    LS[lidar-slam-lab v0.1.0<br/>ray casting, grid, scan matching,<br/>MCL, maps, autopilot]
    T[three.js 0.187.0<br/>+ STL/OBJ/Collada loaders]
  end
  subgraph ui["src/ui (browser)"]
    A[app.js panels<br/>builder.js properties]
    R[view3d.js + meshes.js]
    SU[simui.js, simview.js,<br/>plots.js, worldedit.js]
  end
  CLI[bin/robot-studio.js]
  F --> V
  E --> V
  V --> X --> M
  M --> UI
  S --> UI
  M --> A
  S --> R --> T
  A --> R
  V --> CLI --> M
  B --> M
  B --> UI
  A --> B
  B --> SIM
  SIM --> UI
  SIM --> LS
  SU --> SIM
  SU --> R
  CLI --> SIM
  MK[mecanum-kinematics<br/>CI interop] -.-> SIM
  FW[encoder-odometry-firmware<br/>mirrored, CI interop] -.-> SIM
  IC[imu-calibration-lab<br/>JSON import] -.-> SIM
  RB[rosbag-inspect<br/>milestone 4] -.-> ui
```

Reuse is by vendoring at pinned tags: `scripts/vendor.mjs` copies the needed
files of urdf-inspector (tag `v0.1.1`), lidar-slam-lab (tag `v0.1.0`) and
three.js (`0.187.0`) into `vendor/` with a SHA-256 manifest, and a CI job
re-fetches all three and fails on any difference. Nothing in `vendor/` is
edited; changes that Robot Studio needs from them are listed in
[docs/upstream-requests.md](docs/upstream-requests.md). The Python projects
(mecanum-kinematics, encoder-odometry-firmware's host model) cannot run in
the page, so their conventions are mirrored in JavaScript and CI installs
them from their public tags and compares the numbers. The reasons for this
and the other design choices (three.js, the separate XML reader for xacro,
the expression evaluator, COLLADA axes) are in
[docs/decisions.md](docs/decisions.md). For the module map and how to run
everything, see [docs/HANDOFF.md](docs/HANDOFF.md).

## Validation status

Kept as three separate tiers in every release:

| Tier               | Status in 0.3.0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Software tests** | Yes. 162 Node tests (xacro, expression evaluator, zip, packages, vendor manifest, CLI, examples, UI state, builder operations, templates, inertia, import/export, autosave; simulation: drive against analytic IK/FK, strafe, saturation and encoder counts, IMU, LiDAR ranges, sensor poses from FK, Gazebo tags, worlds, settings, recording, determinism and a golden digest, the page's session); coverage of `src/core` 99.6 % lines / 95.9 % branches with thresholds enforced. Golden job: the official `xacro` 2.1.1 regenerates 45 outputs; Robot Studio matches 21 of the 23 successful expansions byte for byte and the other 2 except for a 1-ulp difference in 4 numbers each, and fails on all 21 inputs the official tool rejects. Builder exports: the official xacro expands each exported xacro to the exported URDF; mecanum-kinematics v0.1.0 loads the exported TOML and agrees to about 2e-15; exported ROS 2 packages build with colcon in `ros:humble-ros-base` and pass `check_urdf`, ROS's `xacro`, `ros2 launch --show-args` and robot_state_publisher. Headless Chrome job: every example, a zip upload, phone width, the builder scenario, and the simulation driven through the UI (route, teleop key, world editor, a setting, recording download) with no console errors. |
| **Simulation**     | Yes, and labelled as model output in every panel, plot, export and CLI report; the parameters are made-up defaults or yours. Cross-checks of the simulator against other projects' own code in CI: simulated wheel speeds equal mecanum-kinematics' `inverse_saturated()` (ideal wheels to 4.9e-12 rad/s, motor + PI model after 3 s to 6.1e-4 rad/s); every WHEEL_STATE and ODOMETRY frame of a 6.5 s command schedule equals encoder-odometry-firmware's Python firmware model (counts exact, speeds to 4.9e-11 rad/s); the estimators run through Robot Studio's wrapper reproduce lidar-slam-lab's CLI benchmark exactly (18 of 18 rows). Results of full runs (for example 0.035 m RMS for MCL and 0.059 m for scan matching on the office route) are in [docs/simulation.md](docs/simulation.md). These are comparisons between models, not validation.                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| **Hardware**       | None. Nothing in this release has been run against a real robot or a bag recorded from one. No exported package and no simulated parameter has been checked on a robot.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

## How it compares

Written from the tools' own documentation and source; details and links in
[docs/prior-art.md](docs/prior-art.md).

- **RViz2** is the reference viewer inside a ROS 2 system; it needs ROS,
  takes xacro through a launch file and gets joint sliders from
  `joint_state_publisher_gui`. Robot Studio is for when no ROS machine is at
  hand, and adds the URDF checks next to the model.
- **Foxglove** loads URDF from a URL, file (desktop), parameter or topic;
  its docs do not describe xacro expansion; it has been closed source since 2024.
- **urdf-viz** (Rust) expands xacro by calling the external `xacro`
  command, so xacro input needs ROS or the Python package.
- **Phobos** is a Blender add-on for modelling and exporting URDF/SDF, a
  different job (editing in Blender).
- **ros3djs** shows the robot from a running ROS system through rosbridge.
  **urdf-loaders** (three.js) has no xacro support; the companion
  **xacro-parser** lists `^|` defaults, `xacro:call` and include namespaces
  as not implemented, all of which Robot Studio supports and checks against
  the official tool.
- Online viewers (the urdf-loaders demo, Robot Viewer) show URDF in the
  browser; Robot Viewer lists xacro support, whose extent we did not test.

## Testing

```bash
npm run check                    # format, typecheck, all tests, coverage thresholds
node scripts/golden-report.js    # golden comparison table
python scripts/xacro_golden.py --check   # with xacro installed: goldens still match the official tool
node scripts/vendor.mjs --verify # vendored files match upstream (network)
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --expect-links 11 --screenshot shot.png
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --builder   # builder scenario
node scripts/builder-fixtures.js --check  # exported template files are current
python scripts/mecanum_interop.py        # with mecanum-kinematics installed (also checks the simulator)
node scripts/sim-fixtures.js --check      # simulator fixtures and the golden digest are current
python scripts/firmware_interop.py       # with odobridge (encoder-odometry-firmware host) installed
node scripts/lsl-crosscheck.mjs --lsl ../lidar-slam-lab   # estimators vs lidar-slam-lab's benchmark
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --template mecanum --sim --sim-interact
```

## Known limitations

- `xacro.load_yaml()` is not supported yet; Python comprehensions, `lambda`
  and f-strings in `${}` are rejected (full list in
  [docs/xacro.md](docs/xacro.md#not-supported-honest-list)).
- Meshes: STL, OBJ (no `.mtl` materials) and COLLADA only.
- Floating and planar joints have no sliders.
- A link to a model you dropped cannot reopen it (files are never put in
  URLs); links to examples restore joint positions, arguments and the
  selection.
- The checks are urdf-inspector's; a clean report is not proof that the
  model is right.
- Builder: numeric fields only (no transform gizmo); mimic and dynamics are
  not editable in the panel; a xacro file is imported as its expansion
  (macros and properties are not kept); the xacro export is parametrised
  only while the robot is exactly a template's output; mecanum-kinematics
  export only for four-wheel bases (no differential model there); template
  dimensions and masses are illustrative defaults, not a real robot's.
- Autosave lives in one browser's local storage; mesh files are not stored,
  so an imported robot shows placeholders after a reload until its files
  are opened again.
- Simulation: simulation only (the hardware tier is empty); 2D motion on a
  flat floor without dynamics; slip is a random process, not friction;
  mecanum rollers are not modelled one by one; one LiDAR, no motion
  distortion; the depth camera is geometry only and not recorded;
  lidar-slam-lab's MCL has no heading-bias term and reads a strafe as two
  rotations (with TurtleBot3's short-range LiDAR it lost track in the corridor
  loop in one documented run); scan matching has no loop closure. Headless
  browsers render on the CPU (10 to 14 fps), so the CI browser job checks
  results, not frame rate. Full list in [docs/simulation.md](docs/simulation.md#limitations).

## License

MIT, see [LICENSE](LICENSE). Vendored code keeps its own licence
(`vendor/three/LICENSE`, `vendor/urdf-inspector/LICENSE`,
`vendor/lidar-slam-lab/LICENSE`), and the
third-party example descriptions keep theirs (Apache-2.0 for TurtleBot3 and
ros2_control_demos, BSD-3-Clause for urdf_tutorial; files in
`examples/third_party/*/LICENSE`).
