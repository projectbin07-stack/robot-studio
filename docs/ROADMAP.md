# Roadmap

Robot Studio grows in five milestones (from the flagship plan). Every
milestone keeps the three validation tiers separate: **software tests**
(unit/integration tests and CI), **simulation** (results produced by the
studio's own models, always labelled simulated) and **hardware** (only what
was run against a real robot or a bag recorded from one).

## Milestone 1: Model (v0.1.0, done)

Open any robot model: xacro expansion checked against the official xacro,
URDF/xacro/folder/zip loading, three.js view with STL/OBJ/DAE meshes, joint
sliders, frames, joint axes, centre of mass, live urdf-inspector checks,
tree and details panels, URL state, themes, phone layout, CLI. Validation:
software tests only. See the [README](../README.md) and
[HANDOFF.md](HANDOFF.md).

Carried over (known gaps): `xacro.load_yaml`, comprehensions and lambda in
`${}`; see [xacro.md](xacro.md#not-supported-honest-list).

## Milestone 2: Builder (v0.2.0, done)

Create and edit a robot in the browser instead of only viewing one. Details
in [builder.md](builder.md). Validation: software tests only.

Acceptance criteria and where they stand:

1. **Met.** Add, rename (references follow), delete, duplicate and
   re-parent links and joints from the tree, the 3D view and the toolbar;
   edit origin, axis, limits, inertial and geometry in the Properties panel
   (numeric fields). Every edit re-runs the checks: on a 51-link arm an edit
   plus all checks takes a median of 10 ms on the development machine and
   16 to 19 ms on the CI runners (20 edits in Node; the test prints it). In
   the browser, edit-to-redraw on the 10-link mecanum template took 36 ms in
   headless Edge and 32 ms in headless Chrome in CI (both SwiftShader); a
   50-link model was not timed in the browser.
2. **Met.** Templates: four-wheel mecanum base, differential base with
   caster, N-joint arm (1 to 50 joints; 2 and 6 tested among others), with
   optional LiDAR, IMU, camera and depth camera mounts placed by the user.
   Every tested variant has no errors and no warnings (test).
3. **Met.** Inertia from geometry and mass for box, cylinder, sphere and a
   mesh's bounding box with urdf-inspector's `primitiveInertia`, rotated with
   the shape by its `rotateTensor`; generated robots pass `inertia-*`
   (test, including a negative control that `inertia-axes` catches).
4. **Met.** URDF and xacro exports; the template xacro has properties,
   sensor arguments and wheel/roller/segment macros; the official xacro
   expands it in the golden CI job to the exported URDF (7 cases).
5. **Met for mecanum bases.** TOML via urdf-inspector's `mecanumToml`; CI
   installs mecanum-kinematics from tag `v0.1.0`, loads it with its own
   loader, compares radius, lx, ly and runs its IK/FK against Robot
   Studio's numbers (agreement to about 2e-15). **Not met for the
   differential template:** mecanum-kinematics has no differential model.
6. **Met.** Undo/redo for every edit (toolbar and Ctrl+Z / Ctrl+Y); the
   working robot survives a reload (local storage, labelled as local to
   the browser).

Also delivered: import of any opened model as an editable robot (round trip
tested on the bundled examples and the golden corpus), ROS 2 description
package zip built with colcon in CI (`ros:humble-ros-base`), CLI `new`,
`export`, `templates`.

Not done (moved on): a transform gizmo (numeric fields only); editing
mimic and dynamics in the UI (the document and operations support them);
importing xacro as xacro (the expansion is imported).

## Milestone 3: Simulation

Drive the model and simulate its sensors, labelled as simulation
throughout. Builds on the builder: a template robot (or an imported one)
is the thing that is driven.

Acceptance criteria:

1. Kinematic drive from the wheel geometry urdf-inspector measures
   (`measureWheels`, already used by `src/core/builder/drive.js`): mecanum
   IK/FK there is cross-checked against mecanum-kinematics in CI; add a
   differential model with the same conventions. Keyboard / on-screen
   joystick produce a body twist, wheel speeds within the joints' velocity
   limits (saturated as mecanum-kinematics does), and the robot moving on
   the grid; wheel joints turn in the view. IK/FK agree to 1e-9 (test).
2. 2D LiDAR from lidar-slam-lab's ray caster (`lidar.js`, `geometry.js`,
   `world.js`, vendored like urdf-inspector) mounted at every link tagged
   `lidar2d` (template mounts or tagged on import); range, resolution and
   noise as builder parameters; a simple world editor (walls, boxes); scans
   drawn in 3D; a test checks ranges against hand-computed values for a
   square room.
3. IMU at links tagged `imu` with noise and bias; optional calibration
   constants from imu-calibration-lab's JSON; encoders with ticks per
   revolution and quantisation on the wheel joints; seeded randomness so
   runs are reproducible (test).
4. TF tree view and time plots (wheel speeds, IMU, pose); CSV export.
5. Every screen and export says "simulated"; nothing is presented as
   measured.

## Milestone 4: ROS 2

Connect to a real robot or replay a recorded one.

Acceptance criteria:

1. rosbridge v2 client (JSON over WebSocket) written from the protocol
   specification: subscribe `/joint_states`, `/tf`, `/tf_static`, `/scan`,
   `/imu`, `/odom`; publish `/cmd_vel` with a dead-man switch and a speed
   limit; read `robot_description` from the parameter or topic.
2. Tested against a fake rosbridge server in CI (recorded message
   fixtures), including reconnects and malformed messages.
3. Bag replay companion: a small Python tool using rosbag-inspect that
   converts a ROS 2 bag into a timeline file the studio plays back with the
   same views as a live connection (test on a bag fixture).
4. Hardware tier: the owner's MentorPi (ROS 2 Humble, mecanum, LiDAR, IMU,
   depth camera, encoders) connected once over rosbridge, or a bag recorded
   from it, with the date, commit and what was observed recorded in the
   README. Until that happens the hardware tier stays empty.

## Milestone 5: Release polish

1. Documentation site pages for each feature with screenshots taken from
   the live site in CI.
2. Accessibility pass (keyboard operation of the tree, sliders and view
   toolbar; contrast in both themes) with an automated check in CI.
3. Performance budget: a 200-link, 50 MB-mesh model opens in under 10 s in
   the CI browser job (measured, reported).
4. Fixes found on the way pushed upstream to urdf-inspector, lidar-slam-lab
   and the other reused repositories (see
   [upstream-requests.md](upstream-requests.md)), and the vendored pins
   moved to those releases.
