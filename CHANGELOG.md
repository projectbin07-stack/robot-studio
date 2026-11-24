# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

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
