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

## Milestone 2: Builder

Create and edit a robot in the browser instead of only viewing one.

Acceptance criteria:

1. Add, rename, delete and re-parent links and joints from the tree; edit
   origin, axis, limits, inertial and geometry in the details panel; every
   edit re-runs the checks and updates the view within 100 ms for a
   50-link model (measured in a test, reported in CI output).
2. Templates: four-wheel mecanum base, differential drive with caster,
   two- and six-axis arm. Each template, exported, passes the urdf-inspector
   checks with no errors or warnings (test).
3. Inertia from geometry and mass (box, cylinder, sphere, mesh bounding
   box), using urdf-inspector's `primitiveInertia`; the result passes the
   `inertia-*` checks (test).
4. Export URDF; export xacro with properties and a wheel macro for the
   templates; the exported xacro, expanded by the official xacro in the
   golden CI job, equals the exported URDF (golden test).
5. Export a mecanum-kinematics TOML config with urdf-inspector's
   `mecanumToml`; CI loads it with the mecanum-kinematics repository's
   parser at a pinned tag and compares wheel radius, `lx`, `ly` (test).
6. Undo/redo for every edit; edits survive a page reload (local storage,
   clearly labelled as local to the browser).

## Milestone 3: Simulation

Drive the model and simulate its sensors, labelled as simulation
throughout.

Acceptance criteria:

1. Kinematic drive: mecanum and differential models from the wheel
   geometry detected by urdf-inspector (`measureWheels`); keyboard / on-
   screen joystick commands produce body twist and wheel speeds; inverse
   and forward kinematics agree to 1e-9 in tests, and match
   mecanum-kinematics' reference values for the same geometry (test against
   its published examples).
2. 2D LiDAR from lidar-slam-lab's ray caster (`lidar.js`, `geometry.js`,
   `world.js`, vendored like urdf-inspector) mounted at the URDF's laser
   frame; scans drawn in 3D; a test checks ranges against hand-computed
   values for a square room.
3. IMU model with noise and bias; optional calibration constants imported
   from imu-calibration-lab's JSON; encoders with ticks per revolution and
   quantisation; seeded randomness so runs are reproducible (test).
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
