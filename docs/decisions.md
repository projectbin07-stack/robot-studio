# Design decisions

Short records of the choices that shape the code, with the reason and what
would change them.

## 1. Static browser app, ES modules, no build step

Same conventions as urdf-inspector: `index.html` loads `src/ui/app.js`
directly; types are JSDoc checked by `tsc --checkJs --strict`; Prettier;
`node:test` with coverage thresholds on `src/core/`. GitHub Pages serves
the repository files as they are. Everything that computes something lives
in `src/core/` (DOM-free, tested under Node); `src/ui/` only wires the DOM
and three.js.

## 2. Reuse by vendoring at pinned versions

urdf-inspector's URDF reader, 71+ checks, forward kinematics, inertia and
wheel code are used, not rewritten. They are copied by
`scripts/vendor.mjs` from the tag recorded in `scripts/vendor-lib.js`
(currently `v0.1.1`, commit in `vendor/manifest.json`) into
`vendor/urdf-inspector/src/core/`, with a SHA-256 per file. The CI job
`vendor` re-clones the tag and fails if any byte differs or the tag moved;
`tests/vendor.test.js` checks the manifest offline. Vendored files are
never edited: changes go to urdf-inspector first
([upstream-requests.md](upstream-requests.md)), then the pin moves.

Why not an npm dependency or a git submodule: urdf-inspector is not
published to npm and has no `exports` map; a submodule would not be served
by GitHub Pages without a build step and does not pin individual files.
Vendoring keeps the deployed site self-contained and the provenance
checkable.

lidar-slam-lab's DOM-free simulation modules (`geometry.js`, `lidar.js`,
`world.js`, `maps.js`, `motion.js`, `grid.js`, `distance.js`, `scanmatch.js`,
`mcl.js`, `metrics.js`, `autopilot.js`, `export.js`, `sim.js`, `rng.js`) are
the third `SOURCES` entry, vendored at **v0.1.0** to
`vendor/lidar-slam-lab/src/` with the same tooling and the same CI check
(milestone 3).

## 3. three.js is the one third-party runtime library

The 3D view must show real meshes (STL, OBJ and COLLADA with textures),
with lighting, shadows and picking good enough to compare with RViz.
urdf-inspector's hand-written canvas projection draws only primitives in
wireframe/flat shading; writing a mesh renderer, three mesh parsers
(COLLADA alone is a large format) and texture handling would be a project
of its own and less reliable than three.js, which is MIT-licensed, widely
used and has maintained loaders for exactly these formats.

Only what is used is vendored: `build/three.module.js`, `three.core.js`
and the addons `OrbitControls`, `STLLoader`, `OBJLoader`, `ColladaLoader`
(plus the `collada/` parser and composer and `TGALoader`, which it
imports), with three.js's `LICENSE`. Release `0.187.0`, fetched by
`npm pack` (registry integrity recorded in the manifest), resolved in the
page through an import map. Nothing is loaded from a CDN. `three` and
`@types/three` are also dev dependencies at the same version so `tsc`
type-checks `src/ui/` against them; Dependabot ignores both because a bump
means re-vendoring.

## 4. Our own XML document model for xacro

urdf-inspector's `xml.js` concatenates an element's character data and
drops comments. xacro needs children in document order with text, CDATA
and comments as separate nodes: blocks and conditionals move mixed content,
CDATA must not be evaluated (minidom keeps it as a separate node type),
comments survive into the output and `xacro:eval-comments` evaluates them.
`src/core/xacro/dom.js` is that reader (with line numbers and the XML rules
xacro's expat parser applies: line-end and attribute-value normalisation).
After expansion the tree is converted to urdf-inspector's `XmlElement`
shape (`toXmlElement`) so the vendored URDF reader and checks run
unchanged. An option for ordered mixed content in urdf-inspector's reader
is listed as an upstream request.

## 5. Expressions: a Python subset with Python's number semantics

xacro evaluates `${}` with Python's `eval`. A browser tool cannot, and must
not use JavaScript `eval`/`Function` on untrusted files either. The
evaluator (`pyexpr.js` parser, `pyeval.js` evaluator, `pyvalue.js` value
model) implements the expression grammar robot descriptions use, with
Python's semantics where they differ from JavaScript: ints are BigInts and
floats are numbers (so `6/3` prints `2.0` and `2**100` is exact), floor
division and modulo follow Python's signs, `round` is half-to-even on the
exact binary value, `str()` of a float is Python's shortest repr. Names
come only from xacro's documented symbol table; attribute access to names
starting with `_` and anything outside the grammar is rejected.

## 6. COLLADA up axis ignored, unit applied (as RViz)

three.js's ColladaLoader rotates `Z_UP` files by -90° about X (to its Y-up
convention) and scales by `<unit meter>`. RViz ignores the up axis
(`rviz_rendering/src/rviz_rendering/mesh_loader_helpers/assimp_loader.cpp`
sets `AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION`, with the comment
"ASSIMP wants to change the orientation of the axis, but that's wrong for
rviz") and keeps the unit scale. Robot Studio resets the loader's rotation
and keeps its scale, so meshes look as they do in RViz. The world is Z-up
(REP 103): `Object3D.DEFAULT_UP` and the camera's up vector are +Z and the
grid lies in the XY plane.

## 7. Files stay in the browser

Dropped files, folders and zips become an in-memory virtual file system
(`src/core/vfs.js`). Text files (`.xacro`, `.urdf`, `.xml`, `package.xml`,
`.yaml`, ...) are read up front because xacro includes are processed
synchronously, like the reference implementation; meshes and images are
read when the view needs them. Missing textures are replaced by a built-in
1×1 pixel instead of letting three.js request a URL, so the page never
makes network requests for model content. A link to a dropped model cannot
reproduce it and says so.

## 8. Zip reading with DecompressionStream

The zip reader (`src/core/zip.js`) parses the central directory and
inflates deflate entries with the platform's `DecompressionStream
("deflate-raw")`, available in current browsers and Node 22+. Stored and
deflate cover what zip tools write by default; ZIP64, encryption and other
methods are rejected with a message. It is tested against a zip written by
Python's `zipfile` and archives built in the test.

## 9. Golden tests against the official xacro, with `$(find)` substituted

The goldens are produced by the unmodified `xacro` 2.1.1 from PyPI. ROS 2's
xacro resolves `$(find)` with `ament_index_python`, which is not on PyPI;
the generator replaces that one function with the package map of each test
case. Everything else (parsing, evaluation, macro expansion, output) is the
official code.

## 10. The builder edits a plain-JSON document; undo keeps whole snapshots

The builder's robot (`src/core/builder/document.js`) is plain JSON: links,
joints, materials and the top-level elements it does not edit. Every edit
is a typed operation that returns a new document; the editor keeps the
previous documents for undo (200 steps). A 50-link document is a few tens
of kilobytes, so whole snapshots are cheaper to get right than inverse
operations, and the same value goes to local storage. Checks run on the
document's URDF through `loadModel()`, the same path as an opened file, so
the builder never has its own idea of what is valid.

## 11. Templates have two writers that are tested against each other

Each template generates the document (what is edited and checked) and a
parametrised xacro file (what a ROS user keeps). Both are written in the
same element order, and the inertias in the same operation order as
urdf-inspector's formulas; the tests expand the xacro with Robot Studio's
xacro, and the golden CI job with the official one, and require the result
to equal the exported URDF. The parametrised xacro is offered only while
the robot is exactly what its parameters generate; after manual edits the
xacro export is the URDF in a xacro file, labelled as not parametrised,
because a parameter set that no longer describes the robot would mislead.

## 12. Sensors are tagged links, not a new element

URDF has no sensor element and Gazebo's `<sensor>` tags are simulator
specific. A sensor in the builder is a link with a `sensor` tag in Robot
Studio's document (LiDAR, IMU, camera, depth camera; cameras get a REP 103
optical frame). The tag is not written to URDF; on import it is guessed
from link names (`laser`, `scan`, `imu`, `camera`, `depth`, ...).
Milestone 3 simulates sensors at tagged links.

## 13. Mecanum kinematics are cross-checked, not reused at run time

mecanum-kinematics is Python and the page is static, so it cannot run in
the browser. The TOML export reuses urdf-inspector's `mecanumToml`, and
`src/core/builder/drive.js` implements mecanum inverse/forward kinematics
with mecanum-kinematics' conventions from the wheel positions
urdf-inspector measures. CI installs mecanum-kinematics from its public tag,
loads the exported TOML with its own loader and compares both directions
with Robot Studio's numbers, so a convention mismatch (wheel order, roller
signs, lx/ly) fails the build.

## 14. The simulated drive mirrors the firmware instead of a generic motor model

The robot this studio is for runs encoder-odometry-firmware-style wheel
control, and milestone 4 compares the simulation with that robot. So the
simulator does not invent its own controller: `src/core/sim/firmware.js`
follows `lib/odocore` and the firmware's own Python model class by class
(IK and saturation, blended velocity estimator, PI with feed-forward,
anti-windup and slew limit, watchdog, SE(2) odometry), and the plant is the
firmware model's first-order motor with exact encoder edge times. The
recording carries the firmware's WHEEL_STATE and ODOMETRY fields. CI replays
the same command schedule through the Python model from the firmware's
public tag and compares every frame (`scripts/firmware_interop.py`), so a
change on either side shows up. The simulation step is therefore the
firmware's control period (10 ms, 10 motor sub-steps).

## 15. The estimators track the drive frame and see re-expressed scans

lidar-slam-lab's estimators assume the LiDAR at the robot's origin. Tracking
the LiDAR's pose made MCL scatter on turns (an offset LiDAR moves sideways a
few millimetres per scan, which rot1-trans-rot2 reads as two large
rotations). Tracking the drive frame (wheel centroid, the centre of
rotation) and re-expressing each scan from its origin keeps every end point
exact and lets the vendored code run unchanged; a test checks the wrapper
against lidar-slam-lab's own loop number for number, and CI against its CLI
benchmark. The proper fix, a sensor pose in lidar-slam-lab, is upstream
request 1.

## 16. Worlds are lidar-slam-lab maps plus heights

The world format extends lidar-slam-lab's map JSON with a wall height,
boxes and cylinders, so lidar-slam-lab still reads every world and its
bundled maps are reused from the vendored `maps.js`. Height is what makes it
more than 2D: the LiDAR sees only what is taller than its URDF height, the
robot collides with everything, the depth camera sees the floor and low
obstacles. Cylinders are 24-gons for the ray caster (circumscribed, so they
contain the circle).

## 17. Determinism is tested, not assumed

All randomness comes from lidar-slam-lab's seeded mulberry32 generator, one
stream per noise source (LiDAR, slip, IMU, MCL), and nothing reads the
clock; numbers in the recording are written with fixed precision. Tests
compare two runs byte for byte, two CLI processes, and a golden SHA-256 of
a fixed run on every CI platform (Ubuntu and Windows, Node 22 and 24).
The step time the CLI prints is measured and therefore not written to the
files.

## 18. Simulation settings live in the document and the link, not the URDF

Like sensor tags (decision 12), the world, route and settings are Robot
Studio metadata: `RobotDoc.sim`, changed by an undoable `setSimulation`
operation and autosaved, and the link's `sim`, `w`, `r`, `ss` parameters.
They are not part of the robot: a template robot with simulation settings is
still "exactly the template" for the parametrised xacro export.

## 19. Our own rosbridge client, and our own WebSocket server for its tests

roslibjs is the usual rosbridge client, but it is a bundled library with
its own event emitter and conversion helpers, and the studio needs only the
v2 JSON operations it uses, with every incoming message treated as
untrusted (size before parsing, no evaluation, per-type checks and limits)
and the bare `NaN` / `Infinity` tokens Python's json writes accepted.
`src/core/ros/rosbridge.js` is written from the protocol document, takes the
WebSocket constructor and timers as parameters and so runs unchanged in
Node 22+ (global `WebSocket`) against the fake server. The fake server is
also our own (RFC 6455 on `node:http`) because it must misbehave on
purpose (malformed, oversized, binary frames, dropped connections) and the
repository has no runtime or test dependencies besides the type checker
and formatter. What would change this: a need for rosbridge's BSON / CBOR
encodings or action support.

## 20. The dead-man switch is a separate gate between teleop and /cmd_vel

The simulation's teleop already produces a body twist; driving a real robot
adds a state machine (`CmdVelGate`) that publishes nothing before an
explicit, per-arming confirmation, re-sends the held command at 10 Hz (so a
robot-side timeout can be short), sends a zero Twist three times on release,
blur, stale input, hide or disconnect, disarms on hide and disconnect, and
scales commands with the firmware's own saturation so no wheel exceeds the
chosen fraction (at most 1) of the URDF's wheel limit. It is DOM-free and
tested on a fake clock; the page only feeds it inputs and focus events. It
cannot stop a robot over a lost connection, so the docs require a
robot-side cmd_vel timeout.

## 21. Bags: an MCAP writer here, reading through rosbag-inspect

Writing a bag in the browser and in the CLI needs CDR, the message
definitions and MCAP in JavaScript, so `src/core/ros` has a small writer
for exactly the ten types used (uncompressed chunks with indexes, so
rosbag2's MCAP plugin reads it like its own). Reading bags is
rosbag-inspect's job: the Python companion uses it as a library pinned to a
release tag instead of porting its SQLite3, MCAP, LZ4/zstd and CDR readers
to JavaScript, and the one gap it had (finding 20: arrays dropped by
`export`) was fixed there and released as v0.2.0. Correctness of the
writer is checked by rosbag-inspect and by ROS 2 Humble's own `rosbag2_py`

- `rclpy` deserializing every message in CI, not only by our tests.

## 22. Replays and live robots feed a Track, and the 3D layer draws a ViewFrame

`SimLayer` used to read the simulation directly. Replays and live robots
have the same views but different sources, so every source now produces a
`ViewFrame` (base pose, joints, scan and its pose, trails, grid, particles,
frames) and `SimLayer.draw()` takes it. `Track` holds what replays and live
robots share (estimators on incoming odometry and scans, plot histories);
`SimSession.frame()` gives the same shape for the simulation. The bag clock
starts at 1 s (a stamp of 0 means "not set" to ROS tools) and the importer
subtracts the bag's start, so recordings keep starting at 0.
