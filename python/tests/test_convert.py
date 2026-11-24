"""The importer on two bags: Robot Studio's MCAP export of a simulated run (round trip to the
recording it came from) and a hand-made robot-like SQLite3 bag (edge cases)."""

from __future__ import annotations

import csv
import io
import json
import math
from pathlib import Path

import pytest

from robot_studio_bag.cli import main
from robot_studio_bag.convert import BagImportError, Options, Topics, convert_bag
from robotbag import BASE_Z, LIDAR_XYZ, LIDAR_YAW, pose_at, write_bag

FIXTURE = Path(__file__).parent / "fixtures" / "sim_mecanum"


def table(text: str) -> list[dict[str, float]]:
    return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(io.StringIO(text))]


def scans(text: str) -> tuple[dict[str, object], list[dict[str, object]]]:
    lines = text.splitlines()
    return json.loads(lines[0])["header"], [json.loads(x) for x in lines[1:]]


def same(a: float, b: float) -> bool:
    return (math.isnan(a) and math.isnan(b)) or a == b


# -- Robot Studio's own export of a simulated run ---------------------------------------------


def test_simulated_bag_round_trips_to_its_recording() -> None:
    result = convert_bag(FIXTURE / "bag")
    assert result.simulated is True
    run = {p.name: p.read_text(encoding="utf-8") for p in (FIXTURE / "run").iterdir()}
    f = result.files
    for name in ("imu.csv", "odometry.csv", "wheel_state.csv", "joint_states.csv"):
        a, b = table(run[name]), table(f[name])
        assert len(a) == len(b), name
        assert a[0].keys() == b[0].keys(), name
        pairs = enumerate(zip(a, b, strict=True))
        bad = [(i, k) for i, (ra, rb) in pairs for k in ra if not same(ra[k], rb[k])]
        assert bad == [], (name, bad[:5])
    a, b = table(run["trajectory.csv"]), table(f["trajectory.csv"])
    assert len(a) == len(b) == 300
    for ra, rb in zip(a, b, strict=True):
        for k in ra:
            if k.startswith("mcl_"):
                assert math.isnan(rb[k])  # MCL is the studio's, not in the bag
            else:
                assert same(ra[k], rb[k]), (k, ra, rb)
    ha, sa = scans(run["scans.jsonl"])
    hb, sb = scans(f["scans.jsonl"])
    assert ha == hb  # float32 in the bag, written back in its shortest form
    assert [s["ranges"] for s in sa] == [s["ranges"] for s in sb]
    assert [s["t"] for s in sa] == [s["t"] for s in sb]
    summary = json.loads(f["summary.json"])
    assert summary["source"]["simulated"] is True
    assert "simulation" in summary["note"]
    assert summary["robot"]["base_frame"] == "base_link"
    assert summary["robot"]["odometry_frame"] == "base_footprint"
    assert json.loads(f["world.json"]) == json.loads(run["world.json"])
    assert (
        summary["sensors"]["lidar"]["height"]
        == json.loads(run["summary.json"])["sensors"]["lidar"]["height"]
    )


# -- a robot-like bag ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def robot_bag(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return write_bag(tmp_path_factory.mktemp("robot") / "robot")


def test_laserscan_keeps_every_beam_and_marks_no_returns(robot_bag: Path) -> None:
    r = convert_bag(robot_bag)
    assert r.simulated is False
    head, items = scans(r.files["scans.jsonl"])
    assert len(items) == 20
    # +inf, -inf, NaN, below range_min and above range_max are no return; range_max itself is kept.
    assert items[0]["ranges"] == [1, None, None, None, None, None, 2.5, 12]
    assert items[0]["t"] == pytest.approx(0.1)  # header stamp, not the receive time
    assert head["range_min"] == 0.05 and head["range_max"] == 12
    assert head["angle_min"] == pytest.approx(-math.pi / 2, abs=1e-7)
    assert any(
        "20 inf, 20 -inf, 20 nan, 20 below range_min, 20 above range_max" in n for n in r.notes
    )


def test_lidar_mount_comes_from_tf_static(robot_bag: Path) -> None:
    head, _ = scans(convert_bag(robot_bag).files["scans.jsonl"])
    mount = head["mount"]
    assert isinstance(mount, dict)
    # In the odometry's frame (base_footprint): base_link's 0.05 m plus the LiDAR's offset.
    assert mount["x"] == pytest.approx(LIDAR_XYZ[0])
    assert mount["y"] == pytest.approx(LIDAR_XYZ[1])
    assert mount["z"] == pytest.approx(LIDAR_XYZ[2] + BASE_Z)
    assert abs(mount["yaw"]) == pytest.approx(LIDAR_YAW)
    # With base_link as the base frame, z is the joint's own.
    head2, _ = scans(convert_bag(robot_bag, Options(base_frame="base_link")).files["scans.jsonl"])
    assert isinstance(head2["mount"], dict)
    assert head2["mount"]["z"] == pytest.approx(LIDAR_XYZ[2])


def test_odometry_trajectory_and_other_streams(robot_bag: Path) -> None:
    f = convert_bag(robot_bag).files
    tr = table(f["trajectory.csv"])
    assert len(tr) == 200
    row = tr[99]
    assert row["t"] == pytest.approx(1.0)
    x, y, th = pose_at(1.0)
    assert (row["odom_x"], row["odom_y"], row["odom_theta"]) == pytest.approx((x, y, th), abs=6e-6)
    assert math.isnan(row["est_x"]) and math.isnan(row["true_x"])  # no map frame, no truth
    assert math.isnan(tr[4]["cmd_vx"])  # before the first /cmd_vel
    assert tr[9]["cmd_vx"] == 0.2 and tr[9]["cmd_wz"] == 0.1  # held from t = 0.097
    od = table(f["odometry.csv"])
    assert len(od) == 40
    assert od[19]["t_s"] == pytest.approx(1.0)
    assert od[19]["x"] == pytest.approx(x, abs=1e-6)
    assert (od[19]["vx"], od[19]["wz"]) == pytest.approx((0.2, 0.1))
    imu = table(f["imu.csv"])
    assert len(imu) == 200 and imu[0]["az"] == 9.81 and imu[0]["gz"] == 0.1
    js = table(f["joint_states.csv"])
    assert list(js[0]) == [
        "t",
        "pos:wheel_left_joint",
        "pos:wheel_right_joint",
        "vel:wheel_left_joint",
        "vel:wheel_right_joint",
    ]
    assert js[49]["pos:wheel_right_joint"] == pytest.approx(3.0)
    assert "wheel_state.csv" not in f and "world.json" not in f
    s = json.loads(f["summary.json"])
    assert s["source"]["storage"] == "sqlite3"
    assert s["source"]["robot_studio_source"] is None
    assert s["sensors"] == {
        "lidar": {"frame_id": "laser", "height": None, "mount_from": "/tf_static"},
        "imu": {"frame_id": "imu_link"},
    }


def test_odom_only_and_missing_frames(robot_bag: Path) -> None:
    # Without /tf the trajectory comes from /odom; the LiDAR mount is then unknown.
    no_tf = Topics(tf="/none", tf_static="/none")
    r = convert_bag(robot_bag, Options(topics=no_tf))
    assert len(table(r.files["trajectory.csv"])) == 40
    head, _ = scans(r.files["scans.jsonl"])
    assert head["mount"] is None
    assert any(
        "no /tf_static path from base_footprint to the LiDAR frame laser" in n for n in r.notes
    )
    r2 = convert_bag(robot_bag, Options(base_frame="nowhere"))
    assert any("used as the base frame" in n for n in r2.notes)
    with pytest.raises(BagImportError, match="no odometry"):
        convert_bag(robot_bag, Options(topics=Topics(tf="/none", odom="/none")))
    # A topic of the wrong type is skipped with a note.
    r3 = convert_bag(robot_bag, Options(topics=Topics(imu="/joint_states")))
    assert "imu.csv" not in r3.files
    assert any(
        "/joint_states is sensor_msgs/msg/JointState, not sensor_msgs/msg/Imu" in n
        for n in r3.notes
    )


def test_cli(robot_bag: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "rec"
    assert main([str(robot_bag), str(out)]) == 0
    text = capsys.readouterr().out
    assert text.startswith("recorded data: scan 20, imu 200, odom 40")
    assert (out / "scans.jsonl").is_file() and (out / "summary.json").is_file()
    assert main([str(FIXTURE / "bag"), str(tmp_path / "sim")]) == 0
    assert capsys.readouterr().out.startswith("SIMULATED data (the bag says so)")
    assert main([str(tmp_path / "missing"), str(tmp_path / "x")]) == 2
    assert "no such file" in capsys.readouterr().err
    with pytest.raises(SystemExit):
        main(["--version"])
