# Handoff

State of Robot Studio after milestone 5 (v1.0.0), for whoever works on it
next. Read [decisions.md](decisions.md) first, then the feature pages listed
in [docs/README.md](README.md).

## Status

- Milestone 1 scope is implemented: xacro expansion, model loading
  (file / folder / zip / examples), three.js view, live checks, panels,
  URL state, themes, phone layout, CLI.
- Milestone 2 scope is implemented: editable robot document with typed
  operations and undo/redo, templates (mecanum, differential, arm) with
  sensor mounts, properties panel, inertia from geometry, URDF / xacro /
  mecanum-kinematics TOML / ROS 2 package exports, autosave, CLI `new`,
  `export`, `templates`. Status of each acceptance criterion in
  [ROADMAP.md](ROADMAP.md#milestone-2-builder-v020-done).
- Milestone 3 scope is implemented: simulation of the open robot (mecanum
  or differential, detected from the URDF) with the drive mirrored from
  encoder-odometry-firmware, LiDAR / IMU / encoders / depth point cloud at
  the URDF's sensor frames, wheel odometry, scan matching and MCL from
  lidar-slam-lab, worlds with heights and an editor, live 3D layers, plots,
  HUD and teleop, a deterministic CLI recording. Status of each criterion in
  [ROADMAP.md](ROADMAP.md#milestone-3-simulation-v030-done).
- Milestone 4 scope is implemented: rosbridge live view and `/cmd_vel`
  behind a dead-man switch, ROS 2 bag (MCAP) export, `robot-studio-bag`
  (Python, over rosbag-inspect, now v0.3.0) for bag import, replay through the
  estimators, simulation-vs-recording comparison. Criterion 7 (hardware)
  is not met. Status of each criterion in
  [ROADMAP.md](ROADMAP.md#milestone-4-ros-2-v040-criteria-1-6-met-7-not-met).
- Milestone 5 scope is implemented: documentation pages with screenshots
  from the live site, keyboard operation and axe-core in CI, the 200-link
  performance budget, upstream fixes released and the pins moved, the
  first-run panel and guided example, `robot-studio studio`. Status in
  [ROADMAP.md](ROADMAP.md#milestone-5-release-polish-v100-done).
- Validation: software tests; simulation (model output, labelled
  simulated, including bags, replays and comparisons made from it);
  hardware none.
- Vendored: urdf-inspector **v0.2.0**, lidar-slam-lab **v0.2.1** (commits in
  `vendor/manifest.json`), three.js **0.187.0**, axe-core **4.14.0** (browser
  tests only, not published). Pinned elsewhere: rosbag-inspect **v0.3.0**
  (`python/pyproject.toml`), odobridge **v0.2.0**, mecanum-kinematics
  **v0.1.0** (CI install lines).

## Module map

```
index.html, styles.css         page and styles (import map for three.js)
src/core/                      DOM-free, tested in Node, coverage-gated
  xacro/dom.js                 XML reader with ordered mixed content, CDATA, comments, lines
  xacro/pyexpr.js              tokenizer + parser for the Python expression subset
  xacro/pyvalue.js             Python values (BigInt ints), repr/str, arithmetic, formatting
  xacro/pyeval.js              evaluator, builtins, math, str/list/dict methods
  xacro/table.js               scoped symbol tables with lazy evaluation
  xacro/process.js             the xacro processor (directives, macros, includes, $() args)
  xacro/serialize.js           xacro-identical output; conversion to urdf-inspector's XmlElement
  xacro/index.js               public API
  vfs.js                       virtual file system, packages, package:// resolution, addZip
  zip.js                       zip reader (stored + deflate, CRC-32)
  paths.js                     forward-slash path helpers, glob
  model.js                     loadModel(): xacro -> urdf-inspector report -> findings with files, meshes
  scene.js                     what the 3D view draws: shapes, colours, poses, sliders, bounds
  examples.js                  bundled example catalogue and exampleFs()
  canonical.js                 canonical XML comparison for the golden tests
  zipwrite.js                  stored zip writer (package export)
  builder/document.js          RobotDoc (plain JSON), tree helpers, unique names, sensor guess
  builder/ops.js               typed edit operations (EditOp), validation, rename propagation
  builder/editor.js            Editor: history, undo/redo, modelFromDoc() -> loadModel()
  builder/inertia.js           inertial from a shape (urdf-inspector primitiveInertia + rotateTensor)
  builder/urdf.js              RobotDoc <-> URDF (the writer uses the xacro serializer)
  builder/templates.js         templates: params, document generator, xacro writer, validation
  builder/actions.js           add child / add sensor / collision from visual as one step
  builder/drive.js             mecanum TOML (urdf-inspector) and IK/FK from measured wheels
  builder/export.js            xacro export, ROS 2 package files and zip
  builder/storage.js           autosave format
  sim/se2.js                   SE(2) exp/log/compose (the firmware's odometry maths)
  sim/firmware.js              encoder-odometry-firmware mirrored: IK, saturation, velocity estimator, PI, watchdog, odometry
  sim/plant.js                 motor + gearbox per wheel, exact encoder edges (the firmware's Python plant)
  sim/robot.js                 simRobot(): drive kind, wheel geometry, sensor mounts from FK, Gazebo tags, footprint
  sim/world.js                 worlds with heights, bundled worlds, JSON and link encodings
  sim/imu.js                   IMU error model, imu-calibration-lab JSON and Gazebo noise import
  sim/depth.js                 depth camera point cloud
  sim/estimators.js            wheel odometry + scan matching + MCL over lidar-slam-lab, scans with their mount as origin
  sim/reference.js             lidar-slam-lab's own experiment through the wrapper (cross-check)
  sim/sim.js                   the fixed-step loop: createSim(), step()
  sim/routes.js                demo (autopilot), square, spin, none
  sim/settings.js              defaults, limits, normalizeSettings(), settingsDiff()
  sim/record.js                recording files and summary.json
  sim/session.js               the page's real-time session: fixed steps, teleop, plot histories, frame()
  ros/json.js                  JSON with rosbridge's bare NaN / Infinity tokens, depth and size limits
  ros/url.js                   rosbridge address check (ws/wss only, mixed-content warning)
  ros/msgs.js                  checks for incoming JointState, TFMessage, LaserScan, Imu, Odometry, String
  ros/rosbridge.js             rosbridge v2 client: ops, fragments, reconnect, dropped-message counters
  ros/live.js                  LiveSession: subscriptions, TF buffer, LiDAR mount, feeds a Track
  ros/deadman.js               CmdVelGate (opt-in, rate, stop rules, speed limit), disarmOnDisconnect
  ros/track.js                 Track (estimators + histories for replays and live robots), ViewFrame type
  ros/msgdefs.js, cdr.js       Humble .msg definitions, ros2msg bundles, XCDR1 serializer
  ros/mcap.js                  MCAP writer (chunks, indexes, summary, CRCs)
  ros/bag.js                   recording -> bag messages (topic mapping) -> MCAP + metadata.yaml
  ros/recording.js             recording files -> numbers (bounded, checked)
  ros/replay.js                ReplaySession over a recording
  ros/compare.js               command schedule, runSchedule(), compareRecordings()
  treenav.js                   arrow-key movement in the kinematic tree (WAI-ARIA tree view)
src/ui/                        browser only (not coverage-gated)
  app.js                       DOM wiring, panels, selection, URL updates, builder session
  builder.js                   template form and properties panel (DOM only)
  view3d.js                    three.js scene, lights, helpers, picking
  meshes.js                    STL/OBJ/DAE loading from the VFS, textures via blob URLs
  files.js                     drop / file input / folder input -> VirtualFs
  state.js                     URL <-> page state (DOM-free, tested)
  simui.js                     Simulate panel, HUD, plots, teleop, frame loop
  simview.js                   the 3D layer: draws a ViewFrame (world, LiDAR, map, trails, particles, cloud, frames)
  rosui.js                     ROS 2 panel: live robot, driving, replay, comparison
  plots.js                     canvas time plots
  worldedit.js                 top-down world editor
  welcome.js                   first-run "Start here" panel and the four-step guided example
src/node/nodefs.js             real-file-system XacroFs for the CLI and tests
src/node/serve.js              127.0.0.1-only static server (studio command, scripts/serve.js)
src/studiocli.js               the `studio` command (one-command local launch)
src/cli.js, bin/robot-studio.js  CLI
src/simcli.js                  the `sim` command (and --bag)
src/roscli.js                  the `bag`, `replay` and `compare` commands
python/                        robot-studio-bag: ROS 2 bag -> recording (uses rosbag-inspect v0.3.0)
vendor/                        vendored code + manifest.json (never edit)
examples/studio_mecanum/       our illustrative model (4 xacro files, STL + OBJ)
examples/third_party/          TurtleBot3, urdf_tutorial, ros2_control_demos (unmodified, with licences)
tests/                         node:test suites; tests/corpus = golden corpus
tests/corpus/builder/          exported template files (xacro, URDF, TOML, reference speeds)
tests/builder-cases.js         which template cases are exported there
tests/corpus/sim/              simulator fixtures for CI cross-checks, golden digest, a test world
scripts/                       vendor.mjs, xacro_golden.py, browser-smoke.mjs, serve.js, generators,
                               builder-fixtures.js, mecanum_interop.py, ros2_package_check.sh,
                               sim-fixtures.js, firmware_interop.py, lsl-crosscheck.mjs,
                               fake-rosbridge.mjs, rosbridge-fixtures.js, bag-fixtures.js,
                               bag-roundtrip.mjs, check_scan_export.py, ros2_bag_check.sh, ros2_bag_verify.py,
                               large-model.mjs (performance budget input), loopback-buffer.mjs (diagnostic)
```

## How to run everything

```bash
npm ci
npm run check                      # format, typecheck, tests with coverage
npm run serve                      # http://127.0.0.1:8000/ (PORT=... to change)
node bin/robot-studio.js check examples/studio_mecanum/urdf/studio_mecanum.urdf.xacro

# goldens (needs Python; keep the venv out of git)
python -m venv .venv && .venv/Scripts/python -m pip install -r scripts/requirements-golden.txt   # bin/ on Linux
.venv/Scripts/python scripts/xacro_golden.py          # regenerate after adding a case
.venv/Scripts/python scripts/xacro_golden.py --check  # CI runs this on Linux; on Windows the 2 studio_mecanum goldens differ in 4 last digits (C library sin)
node scripts/golden-report.js                          # table for docs/xacro.md

# vendored code
node scripts/vendor.mjs            # re-vendor after changing a pin in scripts/vendor-lib.js
node scripts/vendor.mjs --verify   # what CI runs (needs network)

# browser smoke test / screenshots (Chrome or Edge)
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --expect-links 11 --screenshot shot.png
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --builder   # template, edit, Ctrl+Z, export, reload
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --template arm --select-link link_3 --screenshot arm.png

# builder exports
node scripts/builder-fixtures.js           # rewrite tests/corpus/builder after changing a template or writer
node scripts/builder-fixtures.js --check   # what CI runs
# then regenerate the goldens on Linux (roller positions use sin/cos, last bit differs on Windows)
PYTHONPATH=../mecanum-kinematics/src python scripts/mecanum_interop.py   # CI installs mecanum-kinematics v0.1.0
node bin/robot-studio.js new mecanum -f package -o pkg.zip   # CI builds such zips with colcon in ros:humble-ros-base

# simulation
node bin/robot-studio.js sim mecanum --world office --route demo --seconds 60 --seed 1 --out run/
node scripts/sim-fixtures.js [--check]       # fixtures for the cross-checks and the golden digest
PYTHONPATH=../encoder-odometry-firmware/host/src python scripts/firmware_interop.py   # CI installs odobridge v0.2.0
node scripts/lsl-crosscheck.mjs --lsl ../lidar-slam-lab [--seeds 5] [--full-seeds 1]  # about 15 min locally
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --template mecanum --sim --sim-world lab --screenshot sim.png
node scripts/browser-smoke.mjs ... --headed --gpu   # visible window with the GPU, for frame rates

# accessibility, first run, performance budget (all in the CI browser job)
node scripts/browser-smoke.mjs --url "http://127.0.0.1:8000/?theme=dark" --keyboard --axe --axe-report axe.json
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --tour --shots first-run
node scripts/large-model.mjs -o large.zip && node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ \
  --upload large.zip --expect-links 200 --expect-summary "meshes 200/200 shown" --max-open-ms 10000 --timeout 180000

# the page from this computer, for a robot on the LAN (also: npx github:projectbin07-stack/robot-studio studio)
node bin/robot-studio.js studio --ros 192.168.1.20
# from the live site with the local network permission granted (what "Allow" does), against the fake server:
node scripts/browser-smoke.mjs --url https://projectbin07-stack.github.io/robot-studio/ --template mecanum \
  --ros ws://127.0.0.1:9090 --grant-local-network

# ROS 2
node bin/robot-studio.js sim mecanum --out run --bag bag    # or: node bin/robot-studio.js bag run -o bag
node scripts/bag-fixtures.js [--check]        # python/tests/fixtures/sim_mecanum (bag + recording)
node scripts/rosbridge-fixtures.js [--check]  # tests/fixtures/rosbridge (fake server messages)
python -m venv ../.venv-rsbag && ../.venv-rsbag/Scripts/pip install -e "python[dev]"   # pulls rosbag-inspect v0.3.0
../.venv-rsbag/Scripts/robot-studio-bag bag imported
node scripts/bag-roundtrip.mjs --original run --imported imported
node bin/robot-studio.js replay imported && node bin/robot-studio.js compare imported mecanum
node scripts/fake-rosbridge.mjs --port 9090 --fixture tests/fixtures/rosbridge/sim_mecanum.jsonl \
  --description tests/fixtures/rosbridge/robot_description.urdf --log rb.jsonl &
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --template mecanum --ros ws://127.0.0.1:9090 --ros-log rb.jsonl
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --template mecanum --replay rec.zip
# ROS 2 Humble check (needs Docker; CI job ros2-bag):
docker run --rm -v "$PWD/out:/data:ro" -v "$PWD/scripts:/scripts:ro" ros:humble-ros-base bash /scripts/ros2_bag_check.sh
```

On this development machine: work only on D:, set
`TMP=/d/github-engineering-operations/.tmp`, `npm_config_cache` and
`PIP_CACHE_DIR` there, and use `PORT=8765` for the dev server if 8000 is
taken.

## Adding things

- A xacro feature: add a fixture to `tests/corpus/features/`, a case to
  `tests/corpus/cases.json`, run the golden generator, make the test pass.
- An example: put files under `examples/`, list them in
  `src/core/examples.js` (a test fails for unlisted or missing files), add
  the expected link count to `tests/model.test.js` and a line to the CI
  browser job.
- A new upstream module or file: add it to `SOURCES` in
  `scripts/vendor-lib.js` (lidar-slam-lab's files are the `LIDAR_SLAM` list),
  run `node scripts/vendor.mjs`, import from `vendor/<name>/...`.
- A world: add it to `WORLDS` in `src/core/sim/world.js` (the tests
  round-trip every bundled world).

## Next steps

- Hardware (milestone 4, criterion 7) is still open and needs the owner:
  the steps are in [ros2.md](ros2.md#for-the-robots-owner-mentorpi). When a
  MentorPi bag exists, run it through `robot-studio-bag`, `replay` and
  `compare` and record the result in the README's hardware row; do not
  fill that row from simulated data.
- The ROS 2 panel's elements (`#ros-url`, `#ros-connect`, `#ros-drive-confirm`,
  `#ros-arm`, `#ros-replay-file`, `#ros-compare`) and `document.body.dataset`
  values (`ros`, `rosScans`, `rosDrive`, `rosSent`, `replayT`, `replayDone`,
  `compareRows`, `readyAt`, `welcome`, `tourStep`) are what the browser
  smoke scenarios use; keep them or update `scripts/browser-smoke.mjs`.
- Screenshots in `docs/img/live/` come from the Pages workflow's
  `screenshots` job: download the `docs-screenshots` artifact of a deploy,
  look at each picture, then copy it. Do not commit ones you have not seen.
- A screen reader pass has not been done ([accessibility.md](accessibility.md)).
- New UI must keep axe-core at zero violations (the browser job fails
  otherwise) and keep every control reachable with Tab.

## How ROS 2 hangs together

- Every source produces a `ViewFrame` (`ros/track.js`): `SimSession.frame()`
  for the simulation, `Track.frame()` for replays (`ReplaySession`) and live
  robots (`LiveSession`). `SimLayer.draw(frame, robot)` draws any of them;
  `simui.js` and `rosui.js` each run their own frame loop and only one is
  active (opening one closes the other).
- Replays track the recording's base frame (the simulation tracks the wheel
  centroid); see the MCL note in [ros2.md](ros2.md#limitations).
- The bag clock is recording time + 1 s (`BAG_CLOCK_OFFSET`); the importer
  subtracts the bag's start, so recordings start at 0 again.
- `robot-studio-bag` never parses storage formats; anything a bag reader
  lacks belongs in rosbag-inspect (fix there, release, move the pin).
- The dead-man gate is the only path to `/cmd_vel`; the page never
  publishes anything else.
- `tests/rosbridge.test.js` runs real WebSocket connections on 127.0.0.1;
  under heavy parallel load the waits allow 15 s. In 0.4.0, on Linux CI
  runners only, a single message the fake server sent while its fixture
  replay was streaming (a test scan, then a service reply) was not seen by
  the client within 15-20 s. The explanation found in milestone 5: the fake
  server drops replayed data for a slow client only once Node's own write
  queue for it passes 64 KiB, and everything below that sits in the
  operating system's socket buffers, ahead of any later message.
  `scripts/loopback-buffer.mjs` (run in every CI test job) measures how much
  a loopback connection with a paused reader holds before that point:
  2688 KiB on the `ubuntu-latest` runners (about 30 s of the fixture replay, which runs at about 90 KiB/s) against 128 KiB on the `windows-latest` runners (about 1.4 s) and 240 KiB on the development machine (Windows 11). When the test process falls behind the replay's real-time
  rate under parallel load, a Linux connection can queue many seconds of
  replay ahead of the reply; a Windows one cannot. The mechanism is
  measured; that the test process was falling behind on those runs is
  inferred, not observed. The live-view test keeps stopping the replay and
  letting the client drain before its service calls. A live view on a real
  robot over a slow link could see the same delay; rosbridge's own
  `queue_length` / throttling on the robot side is the place to bound it.

## How the simulation hangs together

- The robot is the open model plus its `RobotDoc` (builder) or the model
  imported as a document (viewer): `simRobot(model, doc)`. Sensor tags and
  Gazebo extras come from the document; geometry from urdf-inspector's
  report. Simulation settings live in `doc.sim` (setSimulation operation)
  and in the link (`sim`, `w`, `r`, `ss`).
- The 3D view places the model's root frame with `view.setBase(matrix)` and
  turns the wheels with `view.setPose(q)`; `view.holdRender` lets the frame
  loop draw once per animation frame.
- Determinism: one seeded stream per noise source. Adding a random draw to
  an existing stream changes every later number and the golden digest in
  `tests/corpus/sim/golden-digest.json`; regenerate it with
  `node scripts/sim-fixtures.js` only after checking why it changed.
- Changing the drive or firmware mirror: regenerate the fixtures and run
  `scripts/firmware_interop.py` and `scripts/mecanum_interop.py`.

## Known issues

- ROS 2: see [ros2.md](ros2.md#limitations) (no hardware validation, fixed
  topic names, one LiDAR, no MCL or robot `map` frame in the live view).

- Simulation: see [simulation.md](simulation.md#limitations) (2D motion only,
  one LiDAR, MCL has no heading-drift model and does not recover once lost
  with recovery off, scan matching has no loop closure).

- From the https live site the browser may not reach `ws://` on the robot:
  Chrome and Edge ask for local network access (IP addresses and `.local`),
  Firefox allows only 127.0.0.1, Safari none; `robot-studio studio` avoids
  this ([ros2.md](ros2.md#reaching-a-robot-from-the-live-site)).
- No screen reader has been used on the page.
- `xacro.load_yaml` is not implemented (no YAML reader); descriptions that
  need it stop with a clear error.
- Python comprehensions, lambda, f-strings and a few builtins are not
  supported in `${}` (list in [xacro.md](xacro.md)).
- Mesh formats: STL, OBJ (without `.mtl` materials) and DAE only; glTF and
  others show a placeholder and a finding.
- Floating and planar joints have no sliders (urdf-inspector's FK shows
  them at their origin).
- The camera is framed on rough bounds first and re-framed on the real
  bounding box once meshes have loaded; the helper sizes (axes, centre of
  mass marker) use the rough size.
- Large models: every mesh is loaded at once; no level of detail (the
  200-link, 51 MB budget model opens in 2.8 s in CI).
- Builder: no transform gizmo (numeric fields only); mimic and dynamics
  have no fields in the Properties panel (the operations support them);
  importing a xacro file imports its expansion (macros and properties are
  not kept); after a reload an imported robot's meshes are placeholders
  until its files are opened again; the differential template has no
  mecanum-kinematics export (that library models mecanum bases only).
- urdf-inspector requests are listed in
  [upstream-requests.md](upstream-requests.md).
