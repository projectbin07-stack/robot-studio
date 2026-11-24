"""Check Robot Studio's mecanum-kinematics exports with mecanum-kinematics itself.

For every tests/corpus/builder/<case>.toml written by the builder
(scripts/builder-fixtures.js), this script

1. loads the TOML with mecanum-kinematics' own loader (RobotConfig.from_toml),
2. compares wheel_radius, half_length (lx) and half_width (ly) with the
   template parameters Robot Studio generated the robot from,
3. runs mecanum-kinematics' inverse kinematics for the body twists in
   <case>.reference.json and compares the wheel speeds with the ones Robot
   Studio computed from the wheel positions it measured in the URDF,
4. runs mecanum-kinematics' forward kinematics on Robot Studio's wheel
   speeds and checks that it returns the original twist (round trip).

It then checks the simulator: tests/corpus/sim/mecanum-wheel-speeds.json holds
the wheel speeds Robot Studio's simulated drive settles at for a set of body
twists (scripts/sim-fixtures.js), some of them beyond the wheels' speed limit.
mecanum-kinematics' inverse_saturated() must give the ideal-wheel speeds to
1e-9 (including the saturation scale), and the motor + PI model's speeds
after 3 s must be within 0.01 rad/s of them.

CI installs mecanum-kinematics from its public tag (see .github/workflows/ci.yml).
Exit code 0 when every comparison agrees.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import mecanum_kinematics as mk
from mecanum_kinematics import MecanumKinematics, RobotConfig, Twist

ROOT = Path(__file__).resolve().parent.parent
DIR = ROOT / "tests" / "corpus" / "builder"
TOL = 1e-9


def check_case(toml: Path) -> list[str]:
    ref = json.loads(toml.with_suffix(".reference.json").read_text(encoding="utf-8"))
    cfg = RobotConfig.from_toml(toml)
    problems: list[str] = []
    for key in ("wheel_radius", "half_length", "half_width"):
        ours = float(ref["template"][key])
        theirs = float(getattr(cfg, key))
        if abs(ours - theirs) > TOL:
            problems.append(f"{key}: TOML gives {theirs}, template parameter is {ours}")
    if list(cfg.wheel_order) != ["FL", "FR", "RL", "RR"]:
        problems.append(f"wheel_order is {cfg.wheel_order}")
    model = MecanumKinematics(cfg)
    worst_ik = 0.0
    worst_fk = 0.0
    for case in ref["cases"]:
        t = case["twist"]
        twist = Twist(t["vx"], t["vy"], t["wz"])
        theirs_speeds = model.inverse(twist)
        for ours, theirs in zip(case["speeds"], theirs_speeds, strict=True):
            worst_ik = max(worst_ik, abs(ours - float(theirs)))
        back = model.forward(case["speeds"]).twist
        for a, b in ((back.vx, t["vx"]), (back.vy, t["vy"]), (back.wz, t["wz"])):
            worst_fk = max(worst_fk, abs(a - b))
        ours_fk = case["forward"]
        for a, b in ((back.vx, ours_fk["vx"]), (back.vy, ours_fk["vy"]), (back.wz, ours_fk["wz"])):
            worst_fk = max(worst_fk, abs(a - b))
    if worst_ik > TOL:
        problems.append(f"inverse kinematics differ by up to {worst_ik:.3g} rad/s")
    if worst_fk > TOL:
        problems.append(f"forward kinematics differ by up to {worst_fk:.3g}")
    print(
        f"{toml.name}: r={cfg.wheel_radius} lx={cfg.half_length} ly={cfg.half_width}, "
        f"{len(ref['cases'])} twists, max |IK diff| {worst_ik:.2e} rad/s, "
        f"max |FK diff| {worst_fk:.2e}: {'OK' if not problems else 'FAIL'}"
    )
    return problems


SIM = ROOT / "tests" / "corpus" / "sim" / "mecanum-wheel-speeds.json"
PI_TOL = 0.01


def check_sim() -> list[str]:
    ref = json.loads(SIM.read_text(encoding="utf-8"))
    cfg = RobotConfig.from_toml(DIR / ref["toml"])
    problems: list[str] = []
    if abs(float(cfg.max_wheel_speed) - float(ref["max_wheel_speed"])) > TOL:
        problems.append(
            f"max_wheel_speed: TOML {cfg.max_wheel_speed}, simulator {ref['max_wheel_speed']}"
        )
    model = MecanumKinematics(cfg)
    worst_ideal = 0.0
    worst_pi = 0.0
    worst_scale = 0.0
    saturated = 0
    for case in ref["cases"]:
        t = case["twist"]
        cmd = model.inverse_saturated(Twist(t["vx"], t["vy"], t["wz"]))
        saturated += int(cmd.saturated)
        worst_scale = max(worst_scale, abs(float(cmd.scale) - float(case["scale"])))
        for theirs, ideal, pi in zip(
            cmd.speeds, case["ideal_speeds"], case["pi_speeds_after_3s"], strict=True
        ):
            worst_ideal = max(worst_ideal, abs(float(theirs) - ideal))
            worst_pi = max(worst_pi, abs(float(theirs) - pi))
    if worst_ideal > TOL or worst_scale > TOL:
        problems.append(
            f"simulated ideal wheel speeds differ by up to {worst_ideal:.3g} rad/s "
            f"(saturation scale by {worst_scale:.3g})"
        )
    if worst_pi > PI_TOL:
        problems.append(f"PI-model wheel speeds after 3 s differ by up to {worst_pi:.3g} rad/s")
    print(
        f"{SIM.name}: {len(ref['cases'])} twists ({saturated} saturated) vs "
        f"inverse_saturated(): ideal wheels max |diff| {worst_ideal:.2e} rad/s, "
        f"scale {worst_scale:.2e}; motor + PI model after 3 s max |diff| {worst_pi:.2e} rad/s: "
        f"{'OK' if not problems else 'FAIL'}"
    )
    return problems


def main() -> int:
    files = sorted(DIR.glob("*.toml"))
    if not files:
        print("no TOML files in", DIR)
        return 1
    print(f"mecanum-kinematics {mk.__version__}")
    failed = False
    for f in files:
        for p in check_case(f):
            failed = True
            print(f"  FAIL {p}")
    for p in check_sim():
        failed = True
        print(f"  FAIL {p}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
