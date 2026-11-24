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

## Milestone 3: Simulation (v0.3.0, done)

Drive the robot that is open in the studio, simulate its sensors at its
URDF frames, run the estimators and compare them with the simulated truth,
labelled as simulation throughout. Details in
[simulation.md](simulation.md). Validation: software tests and simulation
(model output); hardware none.

Acceptance criteria (the five planned at the end of milestone 2, then the
milestone 3 brief) and where they stand:

1. **Met.** Kinematic drive from the wheels urdf-inspector measures:
   mecanum (four wheels) and differential (two), detected automatically;
   command twist → firmware-mirrored IK, saturation (scaled as
   mecanum-kinematics and the firmware do) and per-wheel PI loop → motor
   model → slip option → forward kinematics → SE(2) pose; deterministic for
   a seed, fixed 10 ms step. Wheel joints turn in the view. With ideal
   wheels IK/FK agree with the analytic pose to 1e-9 m and the wheel speeds
   with `mecanumInverse` to 1e-12 (tests); keyboard, gamepad and touch
   sticks give the twist.
2. **Met.** 2D LiDAR from lidar-slam-lab's ray caster (vendored v0.1.0) at
   the link tagged `lidar2d`, its pose from FK; range, beams, noise, dropout
   and rate as settings or from a URDF Gazebo `<sensor>` tag; a world editor
   (walls, boxes, cylinders, start, route) with heights; rays and returns in
   3D; a test checks ranges against hand-computed distances in a square
   room. Only the first level LiDAR is simulated.
3. **Met.** IMU at the link tagged `imu` (gravity, rotation rate,
   centripetal acceleration, noise, bias) with imu-calibration-lab's JSON
   as the error model; encoders with lines, gear ratio, 4x decoding and
   quantisation on the wheel joints, in the firmware's telemetry fields;
   seeded randomness with byte-identical recordings for a seed (tests, and
   a golden digest on four CI platforms).
4. **Met.** TF: link frames plus `map` and `odom` frames in the view (TF
   toggle); live plots of wheel speeds (measured and target), commanded and
   actual twist, IMU and estimator errors; CSV export (the recording, below).
   A separate TF tree panel was not built: the kinematic tree panel shows
   the URDF part of it.
5. **Met.** Every panel, plot, HUD, export, CLI report and summary file
   says "simulated" / "simulation output".
6. **Met.** Worlds: editable floor plan with walls, boxes and cylinders at
   their heights, lidar-slam-lab's office, warehouse and loop maps plus a lab
   and an empty room bundled, saved in the link and in the robot document.
7. **Met.** Estimators through the vendored lidar-slam-lab modules: wheel
   odometry (the firmware's), odometry + scan matching with the occupancy
   grid built from that estimate, MCL; live current and RMS error against
   the truth; the map drawn in 3D and exported as PGM/YAML.
8. **Met.** Headless CLI `robot-studio sim robot.urdf --world office --route
demo --seconds 60 --seed 1 --out run/` writing trajectory CSV, LiDAR
   scans, IMU, encoder telemetry in encoder-odometry-firmware's field layout,
   odometry in odobridge's CSV layout, joint states, the map and a summary
   JSON; byte-identical for the same seed (test and CI).
9. **Met.** Recording format documented with its mapping to `/scan`,
   `/imu`, `/odom`, `/joint_states`, `/tf` and `/tf_static` for the bag export
   in milestone 4.
10. **Met (lower priority).** Depth camera point cloud at the link tagged
    `depth_camera`, against the floor and obstacles at their heights;
    geometry only, not recorded.
11. **Met.** Cross-checks in CI: simulated wheel speeds equal
    mecanum-kinematics' `inverse_saturated()` (ideal wheels 4.9e-12 rad/s,
    PI model after 3 s 6.1e-4 rad/s); every telemetry frame equals
    encoder-odometry-firmware's Python model (counts exact, speeds 4.9e-11
    rad/s); the estimators reproduce lidar-slam-lab's CLI benchmark exactly
    (18 of 18 rows).
12. **Met.** Performance measured: 0.6 to 1.2 ms per 10 ms step on the CI
    runners and on the development machine (CLI); in the page 60 fps (the
    display's rate) in a visible Edge window with the GPU on the development
    machine, 12.7 to 14 fps in headless Edge and 9.7 to 14.3 fps in headless
    Chrome on CI, both rendering with SwiftShader (full table in
    [simulation.md](simulation.md#performance)).

Not done (moved on): a TF tree panel separate from the kinematic tree;
more than one LiDAR; noise and recording for the depth camera; an
omnidirectional motion model for MCL (upstream request).

## Milestone 4: ROS 2

Connect to a real robot or replay a recorded one, with the same views the
simulation has. Milestone 3's recording format is the bridge: a bag
becomes the same streams, and a simulated run becomes a bag.

Acceptance criteria:

1. **rosbridge live view**: a rosbridge v2 client (JSON over WebSocket)
   written from the protocol specification; subscribe `/joint_states`,
   `/tf`, `/tf_static`, `/scan`, `/imu`, `/odom`; read `robot_description`
   from the parameter or topic; show them with the simulation's 3D layers,
   plots and HUD (labelled live, not simulated).
2. **cmd_vel**: publish `geometry_msgs/Twist` on `/cmd_vel` from the same
   keyboard, gamepad and touch teleop, with a dead-man switch (commands stop
   when the key, stick or page focus is released), a speed limit no higher
   than the robot's wheel limit, and a visible "driving a real robot" state.
3. **Tested against a fake rosbridge server** in CI (recorded message
   fixtures), including reconnects, malformed messages and the dead-man
   switch.
4. **Bag export**: write a simulated run as a ROS 2 bag (MCAP) with the
   topic mapping in [simulation.md](simulation.md#recording-format-and-the-ros-2-bag-mapping),
   checked by reading it back with rosbag-inspect and with `ros2 bag info`
   in `ros:humble-ros-base` in CI.
5. **Bag import**: a small Python companion using rosbag-inspect converts a
   ROS 2 bag into the recording format, and the studio replays it through
   the same estimators and views. This closes the rosbag-inspect →
   LiDAR adapter gap from the ecosystem audit (findings #20 and #27:
   rosbag-inspect's `export` drops LaserScan arrays and writes inf as text;
   lidar-slam-lab takes no scan input and no mount pose): the adapter keeps
   the arrays and turns inf, NaN and out-of-range values into "no return".
   Test on a bag fixture.
6. **Simulation vs robot**: the same command schedule on the simulator and
   (from a bag) on the robot, compared through the firmware's telemetry
   fields the simulator already writes.
7. **Hardware tier**: the owner's MentorPi (ROS 2 Humble, mecanum, LiDAR,
   IMU, depth camera, encoders) connected once over rosbridge, or a bag
   recorded from it, with the date, commit and what was observed recorded in
   the README. Until that happens the hardware tier stays empty.

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
