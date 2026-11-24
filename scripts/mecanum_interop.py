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

CI installs mecanum-kinematics from its public tag (see .github/workflows/ci.yml).
Exit code 0 when every comparison agrees to 1e-9.
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
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
