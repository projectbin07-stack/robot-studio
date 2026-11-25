# robot-studio-bag

Turns a ROS 2 bag into [Robot Studio](https://github.com/projectbin07-stack/robot-studio)'s
recording format, so the studio can replay it through its estimators and views
(`robot-studio replay`, or the page's ROS 2 panel → Replay).

Bags are read with [rosbag-inspect](https://github.com/projectbin07-stack/rosbag-inspect)
(pinned to its `v0.3.0` tag): SQLite3 and MCAP storage, CDR decoding, message definitions from
the bag or rosbag-inspect's built-in table. This package only maps decoded messages to the
recording's files.

```bash
pip install "robot-studio-bag @ git+https://github.com/projectbin07-stack/robot-studio@v0.4.0#subdirectory=python"
robot-studio-bag my_bag/ my_recording/
robot-studio replay my_recording/          # from the robot-studio repository (Node 22+)
```

What it writes, from which topic (all names can be changed with `--<role>-topic`):

| File               | From                                     | Notes                                                                                    |
| ------------------ | ---------------------------------------- | ---------------------------------------------------------------------------------------- |
| `trajectory.csv`   | `/tf` (odom → base), `/cmd_vel`          | `est_*` from `map` → `odom` if present; `true_*` only from a simulator's `/ground_truth` |
| `scans.jsonl`      | `/scan` (`sensor_msgs/msg/LaserScan`)    | every beam kept; inf, -inf, NaN and values outside `[range_min, range_max]` → `null`     |
|                    | `/tf_static`                             | the LiDAR's mount in the base frame (x, y, z, yaw) in the header                         |
| `imu.csv`          | `/imu`                                   | linear acceleration and angular velocity, IMU frame                                      |
| `odometry.csv`     | `/odom`                                  | the base frame's planar odometry in the firmware's ODOMETRY layout                       |
| `joint_states.csv` | `/joint_states`                          | joints of the first message                                                              |
| `wheel_state.csv`  | `/firmware/wheel_state` (Robot Studio's) | only in bags Robot Studio exported                                                       |
| `summary.json`     | `/robot_studio/source`                   | `source.simulated` is true only when the bag says it came from a simulator               |

Times are seconds from the bag's start, from header stamps (receive time when a stamp is 0).

Development: `pip install -e ".[dev]"`, then `ruff check .`, `ruff format --check .`, `mypy`,
`pytest`. MIT licence (see the repository's `LICENSE`).
