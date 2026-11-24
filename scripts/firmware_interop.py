"""Compare Robot Studio's simulated drive with encoder-odometry-firmware's own model.

tests/corpus/sim/firmware-interop-<template>.json (written by
scripts/sim-fixtures.js) holds the WHEEL_STATE and ODOMETRY telemetry Robot
Studio's simulator produced for a command schedule: its JavaScript mirror of
the firmware's drive controller (IK, saturation, blended velocity estimator,
PI with feed-forward, anti-windup and slew limit, SE(2) wheel odometry) and
the firmware model's motor and encoder plant, with slip and wheel radius
errors off.

This script replays the same schedule through the firmware project's
Python model (odobridge.firmware_model: DriveController and
SimulatedRobot, which mirror lib/odocore class by class) and compares every
telemetry frame: encoder counts, measured and target wheel speeds, duty and
the odometry pose and twist. CI installs odobridge from the firmware
repository's public tag (see .github/workflows/ci.yml).

Both run in double precision, so the expected difference is rounding in the
last bits (Python's libm exp vs V8's), not model differences. Exit code 0
when counts agree to 1 count, speeds to 1e-6 rad/s, duty to 1 wire unit and
poses to 1e-6 m / rad.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from odobridge.firmware_model import DriveConfig, DriveController, SimulatedRobot, wrap_angle
from odobridge.kinematics import Geometry, Twist
from odobridge.protocol import duty_to_wire

ROOT = Path(__file__).resolve().parent.parent
DIR = ROOT / "tests" / "corpus" / "sim"


def replay(ref: dict) -> tuple[list[dict], list[dict]]:
    c = ref["config"]
    geometry = Geometry(
        wheel_radius=c["wheel_radius"],
        half_length=c["half_length"],
        half_width=c["half_width"],
        roller_angle_deg=c["roller_angle_deg"],
        drive=c["drive"],
    )
    config = DriveConfig(
        geometry=geometry,
        ticks_per_wheel_rev=c["ticks_per_wheel_rev"],
        max_wheel_speed=c["max_wheel_speed"],
        control_rate_hz=c["control_rate_hz"],
        telemetry_divider=c["telemetry_divider"],
    )
    controller = DriveController(config)
    robot = SimulatedRobot(geometry, config.ticks_per_wheel_rev)
    period = 1.0 / config.control_rate_hz
    schedule = ref["schedule"]
    wheel_state: list[dict] = []
    odometry: list[dict] = []
    for k in range(ref["periods"]):
        t = k / config.control_rate_hz
        s = next((x for x in schedule if t < x[0] - 1e-9), schedule[-1])
        controller.command(Twist(s[1], s[2], s[3]), robot.now_us)
        duty = controller.step(robot.snapshots(), robot.now_us)
        if k % config.telemetry_divider == 0:
            w = controller.wheels
            wheel_state.append(
                {
                    "t_us": robot.now_us,
                    "count": [x.count for x in w],
                    "velocity": [x.velocity for x in w],
                    "target": [x.target for x in w],
                    "duty": [duty_to_wire(x.duty) for x in w],
                }
            )
            m = controller.measured
            p = controller.pose
            odometry.append(
                {
                    "t_us": robot.now_us,
                    "pose": [p.x, p.y, wrap_angle(p.theta)],
                    "twist": [m.vx, m.vy, m.wz],
                }
            )
        robot.advance(duty, period)
    return wheel_state, odometry


def compare(name: str, ref: dict) -> list[str]:
    ws, od = replay(ref)
    problems: list[str] = []
    if len(ws) != len(ref["wheel_state"]) or len(od) != len(ref["odometry"]):
        return [f"{name}: frame counts differ"]
    worst = {"count": 0, "velocity": 0.0, "target": 0.0, "duty": 0, "pose": 0.0, "twist": 0.0}
    for theirs, ours in zip(ws, ref["wheel_state"], strict=True):
        if theirs["t_us"] != ours["t_us"]:
            problems.append(f"t_us {theirs['t_us']} != {ours['t_us']}")
        for key in ("count", "velocity", "target", "duty"):
            for a, b in zip(theirs[key], ours[key], strict=True):
                worst[key] = max(worst[key], abs(a - b))
    for theirs, ours in zip(od, ref["odometry"], strict=True):
        for key in ("pose", "twist"):
            for a, b in zip(theirs[key], ours[key], strict=True):
                worst[key] = max(worst[key], abs(a - b))
    limits = {"count": 1, "velocity": 1e-6, "target": 1e-6, "duty": 1, "pose": 1e-6, "twist": 1e-6}
    for key, lim in limits.items():
        if worst[key] > lim:
            problems.append(f"{key} differs by up to {worst[key]:.3g} (limit {lim})")
    final = od[-1]["pose"]
    print(
        f"{name}: {len(ws)} WHEEL_STATE + {len(od)} ODOMETRY frames over "
        f"{ref['periods'] / 100:.1f} s; max |diff| counts {worst['count']}, "
        f"velocity {worst['velocity']:.2e} rad/s, target {worst['target']:.2e} rad/s, "
        f"duty {worst['duty']}, pose {worst['pose']:.2e}, twist {worst['twist']:.2e}; "
        f"final odometry x={final[0]:+.4f} y={final[1]:+.4f} theta={final[2]:+.4f}: "
        f"{'OK' if not problems else 'FAIL'}"
    )
    return problems


def main() -> int:
    files = sorted(DIR.glob("firmware-interop-*.json"))
    if not files:
        print("no firmware-interop fixtures in", DIR)
        return 1
    failed = False
    for f in files:
        for p in compare(f.name, json.loads(f.read_text(encoding="utf-8"))):
            failed = True
            print(f"  FAIL {p}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
