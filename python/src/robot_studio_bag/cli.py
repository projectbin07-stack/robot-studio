"""``robot-studio-bag``: convert a ROS 2 bag into a Robot Studio recording directory."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence

from rosbag_inspect.bag import BagError
from rosbag_inspect.cdr import CdrError
from rosbag_inspect.msgdef import MsgDefError

from . import __version__
from .convert import BagImportError, Options, Topics, convert_bag, write_recording


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="robot-studio-bag",
        description=(
            "Convert a ROS 2 bag (directory, .db3 or .mcap; read with rosbag-inspect) into "
            "Robot Studio's recording format, for 'robot-studio replay' and the studio's "
            "Replay panel. LaserScan beams are all kept; inf, NaN and out-of-range values "
            "become 'no return'; the LiDAR mount comes from /tf_static."
        ),
    )
    p.add_argument("--version", action="version", version=f"robot-studio-bag {__version__}")
    p.add_argument("bag", help="bag directory, .db3 or .mcap file")
    p.add_argument("out", help="recording directory to write")
    d = Topics()
    for name in vars(d):
        p.add_argument(
            f"--{name.replace('_', '-')}-topic",
            dest=name,
            default=getattr(d, name),
            metavar="TOPIC",
            help=f"(default {getattr(d, name)})",
        )
    p.add_argument("--base-frame", help="base frame (default: the odometry's child frame)")
    p.add_argument("--odom-frame", default="odom", help="odometry frame on /tf (default odom)")
    p.add_argument("--map-frame", default="map", help="map frame on /tf (default map)")
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    topics = Topics(**{k: getattr(args, k) for k in vars(Topics())})
    opts = Options(topics, args.base_frame, args.odom_frame, args.map_frame)
    try:
        result = convert_bag(args.bag, opts)
    except (BagError, BagImportError, CdrError, MsgDefError, OSError) as exc:
        sys.stderr.write(f"robot-studio-bag: error: {exc}\n")
        return 2
    written = write_recording(result, args.out)
    label = "SIMULATED data (the bag says so)" if result.simulated else "recorded data"
    used = ", ".join(f"{k} {v}" for k, v in result.counts.items() if v)
    sys.stdout.write(f"{label}: {used}\n")
    for n in result.notes:
        sys.stdout.write(f"note: {n}\n")
    sys.stdout.write(f"wrote {len(written)} file(s) to {args.out}\n")
    return 0
