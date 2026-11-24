# Upstream requests

## urdf-inspector

Robot Studio vendors urdf-inspector's `src/core` unchanged (see
[decisions.md](decisions.md#2-reuse-by-vendoring-at-pinned-versions)).
These are changes that would let it reuse more and duplicate less. None of
them was made in urdf-inspector during this milestone.

| #   | Request                                                                                                                     | Why Robot Studio needs it                                                                                                     | Current workaround                                                                        |
| --- | --------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| 1   | `xml.js`: optional ordered mixed content (text, CDATA and comment nodes kept in order), alongside the current merged `text` | xacro moves mixed content and must not evaluate CDATA; Robot Studio carries its own reader (`src/core/xacro/dom.js`) for this | Own reader, converted to `XmlElement` after expansion                                     |
| 2   | An `exports` map in `package.json` (also raised in urdf-inspector's audit, #35)                                             | A stable public module list makes it clear which files are API and safe to vendor                                             | Vendoring by file path, pinned per tag                                                    |
| 3   | `Finding` with an optional `source` (file) next to `line`                                                                   | After xacro expansion an element's line belongs to one of several files                                                       | `src/core/model.js` maps each finding's element path to the file of its top-level element |
| 4   | `report.js`: let `checkText` take a function that formats a finding's location                                              | The CLI wants `file:line` instead of `(line N)`                                                                               | The CLI formats findings itself and reuses only `summaryLines`                            |
| 5   | `fk.js`: export joint frames in the root frame (parent pose × joint origin) for given positions                             | The 3D view draws joint axes at the joint frame                                                                               | `src/core/scene.js` composes it with the exported `compose` and `fromXyzRpy`              |
| 6   | Mesh checks: accept an optional "does this mesh exist" callback                                                             | Missing meshes are the most common problem with dropped packages                                                              | Robot Studio adds its own `mesh-missing` finding                                          |

Pin history: v0.1.0 at the start of milestone 1, moved to **v0.1.1**
(adds the `inertia-axes` check and `symEigen`, `principalAxisMismatch`;
nothing removed) before the v0.1.0 release of Robot Studio.

## lidar-slam-lab

Robot Studio vendors lidar-slam-lab's `src/` simulation modules unchanged
at **v0.1.0** (milestone 3). Changes that would remove workarounds; none was
made in lidar-slam-lab during this milestone.

| #   | Request                                                                                                                   | Why Robot Studio needs it                                                                                        | Current workaround                                                                                                                                                             |
| --- | ------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 1   | A sensor pose (LiDAR mount offset and yaw) for `simulateScan`, `integrateScan`, `matchScan` and MCL (ecosystem audit #27) | The LiDAR sits where the URDF puts it, not at the robot's centre                                                 | Rays are cast from the mount pose; scans are re-expressed from the drive frame's origin for the estimators (`virtualScan`), so map free-space rays start at the robot's centre |
| 2   | A minimum range in `LidarConfig` (audit #27)                                                                              | Real 2D LiDARs report nothing closer than about 0.1 m; Gazebo tags give it                                       | Returns below the minimum are set to NaN after `simulateScan`                                                                                                                  |
| 3   | Obstacle heights in the map format (or a height filter in `castRay`)                                                      | A 2D LiDAR sees only obstacles taller than its scan plane                                                        | Robot Studio's world format adds `wallHeight`, boxes and cylinders; the segments above the LiDAR's height are passed to `simulateScan`                                         |
| 4   | An asymmetric fan (`angleMin`, `angleMax`) in `beamAngles`                                                                | Gazebo and LaserScan give min and max angles                                                                     | The scan pose is turned to the middle of the fan                                                                                                                               |
| 5   | A heading-bias term (or inflated rotation noise per metre) in MCL's motion model, and an omnidirectional motion model     | Wheel radius errors drift heading systematically; a mecanum strafe reads as two 90° rotations in rot1-trans-rot2 | None; documented in [simulation.md](simulation.md#limitations)                                                                                                                 |
| 6   | An `exports` map / documented public API                                                                                  | Makes it clear which files are safe to vendor                                                                    | Vendored by file path at a pinned tag                                                                                                                                          |

## The other reused projects

| Project                   | Request                                                                                  | Why                                                                  |
| ------------------------- | ---------------------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| imu-calibration-lab       | A CLI option that writes the page's combined `{ accel, gyro, mag }` JSON                 | The simulator reads both, but one file from the CLI would be simpler |
| encoder-odometry-firmware | Publish odobridge's firmware model as a documented API (it is used by Robot Studio's CI) | CI installs it from the tag's `host/` directory                      |
| rosbag-inspect            | Keep LaserScan arrays in `export` and write inf/NaN as numbers (ecosystem audit #20)     | Milestone 4's bag import feeds recorded scans to the same estimators |
