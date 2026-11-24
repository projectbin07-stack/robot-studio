# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

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
- Golden corpus of 38 cases compared with the official xacro 2.1.1 in CI: 16
  expansions byte-identical (including TurtleBot3 Burger/Waffle/Waffle Pi,
  the ROS URDF tutorial and ros2_control_demos RRBot), 21 errors matched, 1
  documented gap (lambda).
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
