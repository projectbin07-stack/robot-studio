"""Read a bag Robot Studio exported with ROS 2's own tools and compare it with its recording.

Runs inside ros:humble-ros-base with ros-humble-rosbag2-storage-mcap installed (the CI job
ros2-bag): rosbag2_py opens the bag with the MCAP storage plugin, every message is
deserialized with rclpy into the installed Humble message classes (so a wrong field, type or
alignment in Robot Studio's CDR would fail or give wrong values), and the values are compared
with the recording's CSV / JSONL files.

    python3 ros2_bag_verify.py BAG_DIR RECORDING_DIR
"""

from __future__ import annotations

import csv
import json
import math
import struct
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CLOCK_OFFSET_NS = 1_000_000_000  # docs/ros2.md: bag time = recording time + 1 s


def f32(v: float) -> float:
    return struct.unpack("<f", struct.pack("<f", v))[0]


def rows(path: Path) -> list[dict[str, float]]:
    with path.open(encoding="utf-8") as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def main() -> int:
    bag_dir, rec_dir = Path(sys.argv[1]), Path(sys.argv[2])
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs: dict[str, list[tuple[int, object]]] = {t: [] for t in types}
    while reader.has_next():
        topic, data, t = reader.read_next()
        msgs[topic].append((t, deserialize_message(data, get_message(types[topic]))))
    failures: list[str] = []

    def check(ok: bool, what: str) -> None:
        print(("ok    " if ok else "FAIL  ") + what)
        if not ok:
            failures.append(what)

    for topic, items in sorted(msgs.items()):
        print(f"      {topic:24} {len(items):6} x {types[topic]}")

    traj = rows(rec_dir / "trajectory.csv")
    odo = rows(rec_dir / "odometry.csv")
    imu = rows(rec_dir / "imu.csv")
    js = rows(rec_dir / "joint_states.csv")
    ws = rows(rec_dir / "wheel_state.csv")
    lines = (rec_dir / "scans.jsonl").read_text(encoding="utf-8").splitlines()
    head = json.loads(lines[0])["header"]
    scans = [json.loads(x) for x in lines[1:]]
    world = json.loads((rec_dir / "world.json").read_text(encoding="utf-8"))
    start = world["start"]

    check(len(msgs["/tf"]) == len(traj), f"/tf: one message per trajectory row ({len(traj)})")
    check(len(msgs["/odom"]) == len(odo), f"/odom: one message per ODOMETRY frame ({len(odo)})")
    check(len(msgs["/scan"]) == len(scans), f"/scan: every scan ({len(scans)})")
    check(len(msgs["/imu"]) == len(imu), f"/imu: every sample ({len(imu)})")
    check(len(msgs["/joint_states"]) == len(js), f"/joint_states ({len(js)})")
    check(len(msgs["/firmware/wheel_state"]) == len(ws), f"/firmware/wheel_state ({len(ws)})")
    check(len(msgs["/tf_static"]) == 1, "/tf_static: one message")

    # Scans: float32 ranges, null -> +inf, stamps on the bag clock.
    worst = 0.0
    inf_ok = True
    stamp_ok = True
    for (t, m), s in zip(msgs["/scan"], scans):
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        stamp_ok &= stamp == t == round(s["t"] * 1e9) + CLOCK_OFFSET_NS
        for a, b in zip(m.ranges, s["ranges"]):
            if b is None:
                inf_ok &= a == math.inf
            else:
                worst = max(worst, abs(a - f32(b)))
    check(worst == 0.0, f"/scan ranges equal the recording as float32 (max diff {worst})")
    check(inf_ok, "/scan: every no-return beam is +inf")
    check(stamp_ok, "/scan: header stamp = log time = recording time + 1 s")
    m0 = msgs["/scan"][0][1]
    check(m0.header.frame_id == head["frame_id"], f"/scan frame_id {m0.header.frame_id}")
    check(abs(m0.angle_increment - f32(head["angle_increment"])) == 0, "/scan angle_increment")
    check(m0.range_min == f32(head["range_min"]) and m0.range_max == f32(head["range_max"]),
          "/scan range_min / range_max")

    # /odom against the firmware's ODOMETRY frames (drive frame = base on the mecanum template).
    worst = max(
        max(abs(m.pose.pose.position.x - r["x"]), abs(m.pose.pose.position.y - r["y"]),
            abs(m.twist.twist.linear.x - r["vx"]), abs(m.twist.twist.angular.z - r["wz"]))
        for (_, m), r in zip(msgs["/odom"], odo)
    )
    check(worst < 1e-12, f"/odom pose and twist equal odometry.csv (max diff {worst:.1e})")

    # /tf odom -> base: the trajectory's wheel odometry from the start pose.
    c, s_ = math.cos(start["theta"]), math.sin(start["theta"])
    worst = 0.0
    for (_, m), r in zip(msgs["/tf"], traj):
        tr = m.transforms[0].transform.translation
        dx, dy = r["odom_x"] - start["x"], r["odom_y"] - start["y"]
        worst = max(worst, abs(tr.x - (c * dx + s_ * dy)), abs(tr.y - (-s_ * dx + c * dy)))
    check(worst < 1e-9, f"/tf odom -> base equals the trajectory's odometry (max diff {worst:.1e})")
    check(msgs["/tf"][0][1].transforms[1].header.frame_id == "map", "/tf carries map -> odom")

    worst = max(
        max(abs(m.linear_acceleration.z - r["az"]), abs(m.angular_velocity.z - r["gz"]))
        for (_, m), r in zip(msgs["/imu"], imu)
    )
    check(worst == 0.0, f"/imu equals imu.csv (max diff {worst})")
    check(all(m.orientation_covariance[0] == -1.0 for _, m in msgs["/imu"]),
          "/imu: orientation marked as not provided")
    names = [k[4:] for k in js[0] if k.startswith("pos:")]
    worst = max(
        max(abs(p - r[f"pos:{n}"]) for p, n in zip(m.position, names))
        for (_, m), r in zip(msgs["/joint_states"], js)
    )
    check(list(msgs["/joint_states"][0][1].name) == names, "/joint_states names")
    check(worst == 0.0, f"/joint_states positions equal joint_states.csv (max diff {worst})")
    cols = list(ws[0])
    worst = max(
        max(abs(a - r[k]) for a, k in zip(m.data, cols)) for (_, m), r in zip(msgs["/firmware/wheel_state"], ws)
    )
    check(msgs["/firmware/wheel_state"][0][1].layout.dim[0].label == ",".join(cols),
          "/firmware/wheel_state layout label = the CSV header")
    check(worst == 0.0, f"/firmware/wheel_state equals wheel_state.csv (max diff {worst})")
    worst = max(abs(m.linear.x - r["cmd_vx"]) for (_, m), r in zip(msgs["/cmd_vel"], traj))
    check(worst == 0.0, "/cmd_vel equals the commanded twist")
    src = json.loads(msgs["/robot_studio/source"][0][1].data)
    check(src["simulated"] is True, "/robot_studio/source says simulated")
    print(f"{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
