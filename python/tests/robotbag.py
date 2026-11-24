"""A small robot-like bag for the tests, written with rosbag-inspect's CDR encoder.

SQLite3 storage (rosbag2's default on Humble, schema version 3 tables), no
``/robot_studio/source``, the layout a robot's ``ros2 bag record`` gives. The LiDAR is turned
180 degrees about z and sits 0.1 m behind ``base_link``, which is 0.05 m above
``base_footprint``; its scans hold +inf, -inf, NaN and values outside
``[range_min, range_max]``. Everything in it is hand-made test data, not a recording.
"""

from __future__ import annotations

import math
import sqlite3
from pathlib import Path
from typing import Any

from rosbag_inspect.cdr import encode
from rosbag_inspect.msgdef import Registry

REG = Registry()
T0 = 1_700_000_000_000_000_000
LIDAR_YAW = math.pi
LIDAR_XYZ = (-0.1, 0.02, 0.12)
BASE_Z = 0.05
RANGES = [1.0, math.inf, -math.inf, math.nan, 0.01, 30.0, 2.5, 12.0]
RANGE_MIN = 0.05
RANGE_MAX = 12.0


def stamp(ns: int) -> dict[str, int]:
    return {"sec": ns // 1_000_000_000, "nanosec": ns % 1_000_000_000}


def quat(yaw: float) -> dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def tf(parent: str, child: str, ns: int, xyz: tuple[float, float, float], yaw: float) -> Any:
    return {
        "header": {"stamp": stamp(ns), "frame_id": parent},
        "child_frame_id": child,
        "transform": {
            "translation": {"x": xyz[0], "y": xyz[1], "z": xyz[2]},
            "rotation": quat(yaw),
        },
    }


def pose_at(t: float) -> tuple[float, float, float]:
    """The robot drives a quarter circle: 0.2 m/s, 0.1 rad/s."""
    v, w = 0.2, 0.1
    th = w * t
    return (v / w * math.sin(th), v / w * (1 - math.cos(th)), th)


def messages() -> list[tuple[str, str, int, bytes]]:
    out: list[tuple[str, str, int, bytes]] = []

    def add(topic: str, type_name: str, ns: int, msg: Any) -> None:
        out.append((topic, type_name, ns, encode(REG, type_name, msg)))

    tfm = "tf2_msgs/msg/TFMessage"
    add(
        "/tf_static",
        tfm,
        T0,
        {
            "transforms": [
                tf("base_footprint", "base_link", T0, (0.0, 0.0, BASE_Z), 0.0),
                tf("base_link", "laser", T0, LIDAR_XYZ, LIDAR_YAW),
            ]
        },
    )
    for k in range(1, 201):  # 2 s at 100 Hz
        ns = T0 + k * 10_000_000
        t = k * 0.01
        x, y, th = pose_at(t)
        add("/tf", tfm, ns, {"transforms": [tf("odom", "base_footprint", ns, (x, y, 0.0), th)]})
        if k % 5 == 0:
            add(
                "/odom",
                "nav_msgs/msg/Odometry",
                ns,
                {
                    "header": {"stamp": stamp(ns), "frame_id": "odom"},
                    "child_frame_id": "base_footprint",
                    "pose": {
                        "pose": {
                            "position": {"x": x, "y": y, "z": 0.0},
                            "orientation": quat(th),
                        },
                        "covariance": [0.0] * 36,
                    },
                    "twist": {
                        "twist": {
                            "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
                            "angular": {"x": 0.0, "y": 0.0, "z": 0.1},
                        },
                        "covariance": [0.0] * 36,
                    },
                },
            )
        if k % 10 == 0:
            add(
                "/cmd_vel",
                "geometry_msgs/msg/Twist",
                ns - 3_000_000,
                {
                    "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.1},
                },
            )
            add(
                "/scan",
                "sensor_msgs/msg/LaserScan",
                ns + 1_000_000,  # received 1 ms after its stamp
                {
                    "header": {"stamp": stamp(ns), "frame_id": "laser"},
                    "angle_min": -math.pi / 2,
                    "angle_max": math.pi / 2,
                    "angle_increment": math.pi / (len(RANGES) - 1),
                    "time_increment": 0.0,
                    "scan_time": 0.1,
                    "range_min": RANGE_MIN,
                    "range_max": RANGE_MAX,
                    "ranges": RANGES,
                    "intensities": [],
                },
            )
        add(
            "/imu",
            "sensor_msgs/msg/Imu",
            ns,
            {
                "header": {"stamp": stamp(ns), "frame_id": "imu_link"},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "orientation_covariance": [-1.0] + [0.0] * 8,
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.1},
                "angular_velocity_covariance": [0.0] * 9,
                "linear_acceleration": {"x": 0.0, "y": 0.02, "z": 9.81},
                "linear_acceleration_covariance": [0.0] * 9,
            },
        )
        add(
            "/joint_states",
            "sensor_msgs/msg/JointState",
            ns,
            {
                "header": {"stamp": stamp(ns), "frame_id": ""},
                "name": ["wheel_left_joint", "wheel_right_joint"],
                "position": [t * 4.0, t * 6.0],
                "velocity": [4.0, 6.0],
                "effort": [],
            },
        )
    out.sort(key=lambda m: m[2])
    return out


def write_bag(directory: Path) -> Path:
    """Write the bag as rosbag2's SQLite3 storage plus metadata.yaml."""
    directory.mkdir(parents=True, exist_ok=True)
    db = directory / "robot_0.db3"
    msgs = messages()
    con = sqlite3.connect(db)
    con.executescript(
        "CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);"
        "INSERT INTO schema VALUES (3, 'humble');"
        "CREATE TABLE metadata(id INTEGER PRIMARY KEY, metadata_version INTEGER NOT NULL,"
        " metadata TEXT NOT NULL);"
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,"
        " serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);"
        "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,"
        " timestamp INTEGER NOT NULL, data BLOB NOT NULL);"
    )
    ids: dict[str, int] = {}
    for topic, type_name, _, _ in msgs:
        if topic not in ids:
            ids[topic] = len(ids) + 1
            con.execute(
                "INSERT INTO topics VALUES (?, ?, ?, 'cdr', '')", (ids[topic], topic, type_name)
            )
    con.executemany(
        "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)",
        [(ids[t], ns, data) for t, _, ns, data in msgs],
    )
    con.commit()
    con.close()
    counts: dict[str, tuple[str, int]] = {}
    for topic, type_name, _, _ in msgs:
        counts[topic] = (type_name, counts.get(topic, (type_name, 0))[1] + 1)
    start = msgs[0][2]
    end = msgs[-1][2]
    lines = [
        "rosbag2_bagfile_information:",
        "  version: 5",
        "  storage_identifier: sqlite3",
        "  duration:",
        f"    nanoseconds: {end - start}",
        "  starting_time:",
        f"    nanoseconds_since_epoch: {start}",
        f"  message_count: {len(msgs)}",
        "  topics_with_message_count:",
    ]
    for topic, (type_name, n) in counts.items():
        lines += [
            "    - topic_metadata:",
            f"        name: {topic}",
            f"        type: {type_name}",
            "        serialization_format: cdr",
            '        offered_qos_profiles: ""',
            f"      message_count: {n}",
        ]
    lines += [
        '  compression_format: ""',
        '  compression_mode: ""',
        "  relative_file_paths:",
        "    - robot_0.db3",
    ]
    (directory / "metadata.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return directory
