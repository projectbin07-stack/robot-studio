"""Turn a ROS 2 bag into Robot Studio's recording format.

Bags are read with rosbag-inspect (SQLite3 and MCAP, CDR decoding, message definitions);
nothing here parses storage formats. The recording format is the one ``robot-studio sim
--out`` writes (``docs/simulation.md`` in Robot Studio): one file per stream, every row stamped
with seconds from the start of the bag.

- ``trajectory.csv``: one row per odometry transform (``odom`` -> the odometry's child frame
  on ``/tf``, else one row per ``/odom`` message). ``odom_*`` is the base frame's pose;
  ``est_*`` comes from ``map`` -> ``odom`` when the bag has it; ``true_*`` only from a
  simulator's ``/ground_truth``; ``cmd_*`` is the last ``/cmd_vel`` (held).
- ``scans.jsonl``: every beam is kept. A range that is not finite (inf, -inf, NaN) or lies
  outside ``[range_min, range_max]`` is written as ``null`` ("no return"). The header carries
  the LiDAR's mount in the base frame, looked up in ``/tf_static``.
- ``imu.csv``, ``joint_states.csv``, ``odometry.csv`` (the base frame's planar odometry in the
  firmware's layout) and, when the bag has ``/firmware/wheel_state``, ``wheel_state.csv``.
- ``summary.json``: where the data came from. A bag Robot Studio exported from a simulation
  carries ``/robot_studio/source``; its ``simulated: true`` is kept, so the studio labels the
  replay as simulated, never as a robot.
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rosbag_inspect.analysis import stamp_ns
from rosbag_inspect.bag import Bag, open_bag
from rosbag_inspect.cdr import decode

from . import __version__
from .fmt import fixed, float32, short32
from .frames import Pose2, StaticTree, Transform, twist_at, wrap

FORMAT = "robot-studio-bag-import/1"

TYPES = {
    "scan": "sensor_msgs/msg/LaserScan",
    "imu": "sensor_msgs/msg/Imu",
    "odom": "nav_msgs/msg/Odometry",
    "joint_states": "sensor_msgs/msg/JointState",
    "cmd_vel": "geometry_msgs/msg/Twist",
    "tf": "tf2_msgs/msg/TFMessage",
    "tf_static": "tf2_msgs/msg/TFMessage",
    "ground_truth": "nav_msgs/msg/Odometry",
    "wheel_state": "std_msgs/msg/Float64MultiArray",
    "source": "std_msgs/msg/String",
}

TRAJECTORY_COLUMNS = (
    "t,true_x,true_y,true_theta,odom_x,odom_y,odom_theta,est_x,est_y,est_theta,"
    "mcl_x,mcl_y,mcl_theta,cmd_vx,cmd_vy,cmd_wz,true_vx,true_vy,true_wz"
)


class BagImportError(ValueError):
    """The bag cannot be turned into a recording."""


@dataclass
class Topics:
    """Topic names to read (any of them may be missing from the bag)."""

    scan: str = "/scan"
    imu: str = "/imu"
    odom: str = "/odom"
    joint_states: str = "/joint_states"
    cmd_vel: str = "/cmd_vel"
    tf: str = "/tf"
    tf_static: str = "/tf_static"
    ground_truth: str = "/ground_truth"
    wheel_state: str = "/firmware/wheel_state"
    source: str = "/robot_studio/source"


@dataclass
class Options:
    topics: Topics = field(default_factory=Topics)
    base_frame: str | None = None
    odom_frame: str = "odom"
    map_frame: str = "map"


@dataclass
class Result:
    files: dict[str, str]
    counts: dict[str, int]
    notes: list[str]
    simulated: bool


def _yaw(q: Mapping[str, Any]) -> float:
    return Transform((0.0, 0.0, 0.0), (q["x"], q["y"], q["z"], q["w"])).yaw


def _pose_of(odom_msg: Mapping[str, Any]) -> Pose2:
    p = odom_msg["pose"]["pose"]
    return Pose2(float(p["position"]["x"]), float(p["position"]["y"]), _yaw(p["orientation"]))


def _clean(frame: str) -> str:
    return frame.lstrip("/")


class _Importer:
    def __init__(self, bag: Bag, options: Options) -> None:
        self.bag = bag
        self.o = options
        self.notes: list[str] = []
        self.t0 = bag.start_ns or 0
        self.static = StaticTree()
        self.source: dict[str, Any] | None = None

    # -- reading --------------------------------------------------------------------------------
    def _wanted(self) -> dict[str, str]:
        """Topic name -> role, for the topics present and decodable with the expected type."""
        out: dict[str, str] = {}
        for role, name in vars(self.o.topics).items():
            info = self.bag.topics.get(name)
            if info is None:
                continue
            if info.type != TYPES[role]:
                self.notes.append(f"{name} is {info.type or 'untyped'}, not {TYPES[role]}: skipped")
                continue
            if not self.bag.decodable(name):
                self.notes.append(f"{name}: no message definition, cannot decode: skipped")
                continue
            out[name] = role
        return out

    def _t(self, ns: int) -> float:
        return (ns - self.t0) / 1e9

    def _stamp_t(self, msg: Mapping[str, Any], log_ns: int) -> float:
        """Header stamp as seconds from the bag's start; the receive time when it is unset."""
        ns = stamp_ns(msg["header"])
        return self._t(ns if ns > 0 else log_ns)

    def read(self) -> dict[str, list[tuple[float, dict[str, Any]]]]:
        wanted = self._wanted()
        streams: dict[str, list[tuple[float, dict[str, Any]]]] = {r: [] for r in TYPES}
        reg = self.bag.registry
        for raw in self.bag.messages(list(wanted)):
            role = wanted[raw.topic]
            msg = decode(reg, TYPES[role], raw.data)
            if role in ("cmd_vel", "source", "wheel_state", "tf", "tf_static"):
                t = self._t(raw.log_time)
            else:
                t = self._stamp_t(msg, raw.log_time)
            streams[role].append((t, msg))
        for _, m in streams["tf_static"]:
            for tr in m["transforms"]:
                self.static.add(
                    tr["header"]["frame_id"],
                    tr["child_frame_id"],
                    Transform.from_msg(tr["transform"]),
                )
        if streams["source"]:
            try:
                src = json.loads(streams["source"][0][1]["data"])
                if isinstance(src, dict):
                    self.source = src
            except json.JSONDecodeError:
                self.notes.append(f"{self.o.topics.source} is not JSON: ignored")
        return streams

    # -- frames ---------------------------------------------------------------------------------
    def robot(self) -> dict[str, Any]:
        r = (self.source or {}).get("robot")
        return r if isinstance(r, dict) else {}

    def start(self) -> Pose2:
        """The odom frame in the world of a simulated recording (identity for a robot)."""
        world = (self.source or {}).get("world")
        return Pose2.from_any(world.get("start")) if isinstance(world, dict) else Pose2()

    # -- conversion -----------------------------------------------------------------------------
    def convert(self) -> Result:
        s = self.read()
        files: dict[str, str] = {}
        counts = {k: len(v) for k, v in s.items()}
        robot = self.robot()
        odom_child = next((_clean(m["child_frame_id"]) for _, m in s["odom"]), None)
        tf_child = next(
            (
                _clean(tr["child_frame_id"])
                for _, m in s["tf"]
                for tr in m["transforms"]
                if _clean(tr["header"]["frame_id"]) == self.o.odom_frame
            ),
            None,
        )
        child = odom_child or tf_child or "base_link"
        base = self.o.base_frame or robot.get("base_frame") or child
        child_to_base = self.static.lookup(child, base)
        if child_to_base is None:
            if base != child:
                self.notes.append(
                    f"no /tf_static path from {child} to {base}: the odometry frame {child} is "
                    "used as the base frame"
                )
            base = child
            child_to_base = Transform()
        cb = child_to_base.planar()  # base frame in the odometry's child frame
        drive = Pose2.from_any(robot.get("drive_pose"))
        start = self.start()

        def base_world(child_in_odom: Pose2) -> Pose2:
            return start.compose(child_in_odom.compose(cb))

        files["trajectory.csv"] = self.trajectory(s, child, base_world)
        if s["odom"]:
            files["odometry.csv"] = self.odometry(s["odom"], cb, drive)
        if s["scan"]:
            files["scans.jsonl"] = self.scans(s["scan"], base)
        if s["imu"]:
            files["imu.csv"] = self.imu(s["imu"])
        if s["joint_states"]:
            files["joint_states.csv"] = self.joint_states(s["joint_states"])
        if s["wheel_state"]:
            files["wheel_state.csv"] = self.wheel_state(s["wheel_state"])
        world = (self.source or {}).get("world")
        if isinstance(world, dict) and isinstance(world.get("data"), dict):
            files["world.json"] = json.dumps(world["data"], indent=2) + "\n"
        simulated = bool((self.source or {}).get("simulated", False))
        summary = {
            "format": FORMAT,
            "note": (
                "Imported from a ROS 2 bag produced by a simulation (the bag says so): "
                "simulator output, not a measurement of a robot."
                if simulated
                else "Imported from a ROS 2 bag."
            ),
            "source": {
                "bag": Path(self.bag.path).name,
                "storage": self.bag.storage,
                "simulated": simulated,
                "importer": f"robot-studio-bag {__version__}",
                "robot_studio_source": self.source,
            },
            "robot": {**robot, "base_frame": base, "odometry_frame": child},
            "world": {"id": world.get("id")} if isinstance(world, dict) else None,
            "seed": (self.source or {}).get("seed"),
            "settings_changed": {},
            "sensors": self.sensors(s, base),
            "counts": {k: v for k, v in counts.items() if v},
            "notes": self.notes,
        }
        files["summary.json"] = json.dumps(summary, indent=2) + "\n"
        if not s["tf"] and not s["odom"]:
            raise BagImportError(
                f"the bag has no odometry: neither {self.o.topics.tf} ({self.o.odom_frame} -> "
                f"base) nor {self.o.topics.odom}"
            )
        return Result(files, counts, self.notes, simulated)

    def trajectory(
        self,
        s: dict[str, list[tuple[float, dict[str, Any]]]],
        child: str,
        base_world: Any,
    ) -> str:
        """Rows at the odometry transforms (or /odom messages) with estimate, truth and command."""
        rows: list[tuple[float, Pose2, Pose2 | None]] = []
        map_odom: Pose2 | None = None
        for t, m in s["tf"]:
            odom_pose: Pose2 | None = None
            for tr in m["transforms"]:
                parent = _clean(tr["header"]["frame_id"])
                ch = _clean(tr["child_frame_id"])
                if parent == self.o.map_frame and ch == self.o.odom_frame:
                    map_odom = Transform.from_msg(tr["transform"]).planar()
                elif parent == self.o.odom_frame and ch == child:
                    odom_pose = Transform.from_msg(tr["transform"]).planar()
            if odom_pose is not None:
                est = None if map_odom is None else map_odom.compose(odom_pose)
                rows.append((t, odom_pose, est))
        if not rows:
            rows = [(t, _pose_of(m), None) for t, m in s["odom"]]
        truth = s["ground_truth"]
        cmds = s["cmd_vel"]
        start = self.start()
        out = [TRAJECTORY_COLUMNS]
        ti = ci = -1
        for t, odom_pose, est in rows:
            while ti + 1 < len(truth) and truth[ti + 1][0] <= t + 1e-9:
                ti += 1
            while ci + 1 < len(cmds) and cmds[ci + 1][0] <= t + 1e-9:
                ci += 1
            cells = [fixed(t, 6)]
            if ti >= 0 and abs(truth[ti][0] - t) < 0.5:
                g = truth[ti][1]
                tp = start.compose(_pose_of(g))
                tw = g["twist"]["twist"]
                true_pose = _pose_cells(tp)
                true_v = [
                    fixed(tw["linear"]["x"], 4),
                    fixed(tw["linear"]["y"], 4),
                    fixed(tw["angular"]["z"], 4),
                ]
            else:
                true_pose = ["nan"] * 3
                true_v = ["nan"] * 3
            cells += true_pose
            cells += _pose_cells(base_world(odom_pose))
            cells += _pose_cells(None if est is None else base_world(est))
            cells += ["nan"] * 3
            if ci >= 0:
                c = cmds[ci][1]
                cells += [
                    fixed(c["linear"]["x"], 4),
                    fixed(c["linear"]["y"], 4),
                    fixed(c["angular"]["z"], 4),
                ]
            else:
                cells += ["nan"] * 3
            cells += true_v
            out.append(",".join(cells))
        return "\n".join(out) + "\n"

    def odometry(self, odom: list[tuple[float, dict[str, Any]]], cb: Pose2, drive: Pose2) -> str:
        """The firmware's ODOMETRY layout: the drive frame's pose from where it started."""
        out = ["t_s,x,y,theta,vx,vy,wz"]
        dinv = drive.inverse()
        for t, m in odom:
            child_pose = _pose_of(m)
            base = child_pose.compose(cb)
            pd = dinv.compose(base).compose(drive)
            tw = m["twist"]["twist"]
            vb = twist_at(tw["linear"]["x"], tw["linear"]["y"], tw["angular"]["z"], cb)
            vd = twist_at(*vb, drive)
            vals = [pd.x, pd.y, wrap(pd.theta), *vd]
            out.append(",".join([fixed(t, 6), *(fixed(float32(v), 6) for v in vals)]))
        return "\n".join(out) + "\n"

    def scans(self, scans: list[tuple[float, dict[str, Any]]], base: str) -> str:
        first = scans[0][1]
        frame = _clean(first["header"]["frame_id"])
        mount = self.static.lookup(base, frame)
        mount_json: dict[str, float] | None = None
        if mount is None:
            self.notes.append(
                f"no /tf_static path from {base} to the LiDAR frame {frame}: no mount pose "
                "(the studio then takes the LiDAR to be at the base)"
            )
        else:
            mount_json = {
                "x": round(mount.t[0], 6),
                "y": round(mount.t[1], 6),
                "z": round(mount.t[2], 6),
                "yaw": round(mount.yaw, 9),
            }
            tilt = abs(mount.rotate((0.0, 0.0, 1.0))[2])
            if tilt < math.cos(math.radians(5)):
                self.notes.append(f"the LiDAR frame {frame} is tilted more than 5 degrees")
        rmin = float32(first["range_min"])
        rmax = float32(first["range_max"])
        head = {
            "frame_id": frame,
            "angle_min": float(short32(first["angle_min"])),
            "angle_increment": float(short32(first["angle_increment"])),
            "range_min": float(short32(rmin)),
            "range_max": float(short32(rmax)),
            "scan_time": float(short32(first["scan_time"])),
            "mount": mount_json,
        }
        lines = [
            json.dumps(
                {"header": head, "note": "ranges in metres, null = no return"},
                separators=(",", ":"),
            )
        ]
        kinds = {"inf": 0, "-inf": 0, "nan": 0, "below range_min": 0, "above range_max": 0}
        changed = 0
        for t, m in scans:
            if (
                len(m["ranges"]) != len(first["ranges"])
                or float32(m["angle_min"]) != float32(first["angle_min"])
                or float32(m["angle_increment"]) != float32(first["angle_increment"])
            ):
                changed += 1
                continue
            cells = []
            for r in m["ranges"]:
                if math.isnan(r):
                    kinds["nan"] += 1
                elif r == math.inf:
                    kinds["inf"] += 1
                elif r == -math.inf:
                    kinds["-inf"] += 1
                elif r < rmin:
                    kinds["below range_min"] += 1
                elif r > rmax:
                    kinds["above range_max"] += 1
                else:
                    cells.append(short32(r))
                    continue
                cells.append("null")
            lines.append(f'{{"t":{fixed(t, 6)},"ranges":[{",".join(cells)}]}}')
        if changed:
            self.notes.append(
                f"{changed} scan(s) with a different beam count or angles than the first: skipped"
            )
        none = {k: v for k, v in kinds.items() if v}
        if none:
            self.notes.append(
                "beams written as no return (null): "
                + ", ".join(f"{v} {k}" for k, v in none.items())
            )
        return "\n".join(lines) + "\n"

    def imu(self, imu: list[tuple[float, dict[str, Any]]]) -> str:
        out = ["t,ax,ay,az,gx,gy,gz"]
        for t, m in imu:
            a = m["linear_acceleration"]
            g = m["angular_velocity"]
            out.append(
                ",".join(
                    [
                        fixed(t, 6),
                        *(fixed(a[k], 6) for k in "xyz"),
                        *(fixed(g[k], 7) for k in "xyz"),
                    ]
                )
            )
        return "\n".join(out) + "\n"

    def joint_states(self, js: list[tuple[float, dict[str, Any]]]) -> str:
        names = list(js[0][1]["name"])
        out = [
            "t," + ",".join(f"pos:{n}" for n in names) + "," + ",".join(f"vel:{n}" for n in names)
        ]
        other = 0
        for t, m in js:
            index = {n: i for i, n in enumerate(m["name"])}
            if any(n not in index for n in names):
                other += 1
                continue
            pos = m["position"]
            vel = m["velocity"]

            def val(arr: list[float], n: str, index: dict[str, int] = index) -> str:
                i = index[n]
                return fixed(arr[i], 6) if i < len(arr) else "nan"

            out.append(
                ",".join(
                    [fixed(t, 6), *(val(pos, n) for n in names), *(val(vel, n) for n in names)]
                )
            )
        if other:
            self.notes.append(f"{other} joint_states message(s) without the first one's joints")
        return "\n".join(out) + "\n"

    def wheel_state(self, ws: list[tuple[float, dict[str, Any]]]) -> str:
        """Robot Studio's /firmware/wheel_state: the CSV header is the layout's label."""
        label = ws[0][1]["layout"]["dim"][0]["label"] if ws[0][1]["layout"]["dim"] else ""
        cols = label.split(",")
        out = [label]
        for _, m in ws:
            data = m["data"]
            if len(data) != len(cols):
                continue
            cells = []
            for c, v in zip(cols, data, strict=True):
                if c.startswith(("velocity_", "target_")):
                    cells.append(fixed(float32(v), 6))
                else:
                    cells.append(str(round(v)))
            out.append(",".join(cells))
        return "\n".join(out) + "\n"

    def sensors(
        self, s: dict[str, list[tuple[float, dict[str, Any]]]], base: str
    ) -> dict[str, Any]:
        out: dict[str, Any] = {}
        if s["scan"]:
            frame = _clean(s["scan"][0][1]["header"]["frame_id"])
            mount = self.static.lookup(base, frame)
            src = (self.source or {}).get("sensors")
            height = None
            if isinstance(src, dict) and isinstance(src.get("lidar"), dict):
                height = src["lidar"].get("height")
            out["lidar"] = {
                "frame_id": frame,
                "height": height,
                "mount_from": "/tf_static" if mount is not None else None,
            }
        if s["imu"]:
            out["imu"] = {"frame_id": _clean(s["imu"][0][1]["header"]["frame_id"])}
        return out


def _pose_cells(p: Pose2 | None) -> list[str]:
    if p is None:
        return ["nan"] * 3
    return [fixed(p.x, 5), fixed(p.y, 5), fixed(wrap(p.theta), 6)]


def convert_bag(path: str | Path, options: Options | None = None) -> Result:
    """Read a bag (directory, ``.db3`` or ``.mcap``) and return the recording's files."""
    with open_bag(path) as bag:
        return _Importer(bag, options or Options()).convert()


def write_recording(result: Result, out_dir: str | Path) -> list[Path]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    written = []
    for name, text in result.files.items():
        p = out / name
        p.write_text(text, encoding="utf-8", newline="\n")
        written.append(p)
    return written


def topics_in(result: Result) -> Iterable[str]:
    return (k for k, v in result.counts.items() if v)
