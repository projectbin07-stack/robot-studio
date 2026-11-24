"""Transforms and number formatting, against hand-computed values."""

from __future__ import annotations

import math

import pytest

from robot_studio_bag.fmt import fixed, float32, short32
from robot_studio_bag.frames import Pose2, StaticTree, Transform, twist_at, wrap


def yawed(x: float, y: float, z: float, yaw: float) -> Transform:
    return Transform((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


def test_transform_compose_inverse_and_yaw() -> None:
    a = yawed(1, 0, 0, math.pi / 2)
    b = yawed(1, 0, 0.5, 0)
    c = a.compose(b)  # b's origin seen from a: 1 m along a's x, which points along +y
    assert c.t == pytest.approx((1, 1, 0.5))
    assert c.yaw == pytest.approx(math.pi / 2)
    i = c.compose(c.inverse())
    assert i.t == pytest.approx((0, 0, 0), abs=1e-12)
    assert i.q == pytest.approx((0, 0, 0, 1), abs=1e-12)
    # A roll of 90 degrees takes z to -y.
    roll = Transform((0, 0, 0), (math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)))
    assert roll.rotate((0, 0, 1)) == pytest.approx((0, -1, 0), abs=1e-12)
    msg = {"translation": {"x": 1, "y": 2, "z": 3}, "rotation": {"x": 0, "y": 0, "z": 0, "w": 2}}
    assert Transform.from_msg(msg).q == (0, 0, 0, 1)  # normalised
    with pytest.raises(ValueError, match="quaternion"):
        Transform.from_msg({**msg, "rotation": {"x": 0, "y": 0, "z": 0, "w": 0}})


def test_pose2_and_twist() -> None:
    p = Pose2(1, 2, math.pi / 2)
    q = p.compose(Pose2(1, 0, math.pi))
    assert (q.x, q.y) == pytest.approx((1, 3))
    assert q.theta == pytest.approx(-math.pi / 2)
    r = p.compose(p.inverse())
    assert (r.x, r.y, r.theta) == pytest.approx((0, 0, 0), abs=1e-12)
    assert wrap(3 * math.pi) == pytest.approx(math.pi)
    assert wrap(-3 * math.pi) == pytest.approx(math.pi)
    assert wrap(-math.pi) == pytest.approx(math.pi)
    # Turning at 2 rad/s, a point 0.1 m ahead moves sideways at 0.2 m/s.
    assert twist_at(0, 0, 2, Pose2(0.1, 0, 0)) == pytest.approx((0, 0.2, 2))
    assert Pose2.from_any({"x": 1, "yaw": 0.5}) == Pose2(1, 0, 0.5)
    assert Pose2.from_any({"x": "a", "theta": math.nan}) == Pose2()
    assert Pose2.from_any(None) == Pose2()


def test_static_tree_lookup() -> None:
    tree = StaticTree()
    tree.add("base_footprint", "base_link", yawed(0, 0, 0.05, 0))
    tree.add("/base_link", "laser", yawed(0.1, 0, 0.1, math.pi))
    tree.add("base_link", "imu", yawed(0, 0.02, 0, 0))
    laser = tree.lookup("base_footprint", "laser")
    assert laser is not None
    assert laser.t == pytest.approx((0.1, 0, 0.15))
    imu_in_laser = tree.lookup("laser", "imu")
    assert imu_in_laser is not None
    assert imu_in_laser.t == pytest.approx((0.1, -0.02, -0.1))
    assert tree.lookup("laser", "laser") == Transform()
    assert tree.lookup("base_link", "camera") is None
    tree.add("a", "b", Transform())
    tree.add("b", "a", Transform())
    with pytest.raises(ValueError, match="cycle"):
        tree.lookup("a", "x")


def test_number_formatting_matches_the_javascript_writer() -> None:
    assert fixed(0.0390625, 6) == "0.039063"  # a tie: toFixed rounds away from zero
    assert fixed(-0.0390625, 6) == "-0.039063"
    assert fixed(2.5, 0) == "3"
    assert fixed(-0.0000001, 6) == "0.000000"  # no negative zero
    assert fixed(math.nan, 3) == "nan" and fixed(math.inf, 3) == "nan"
    assert float32(0.1) == pytest.approx(0.1, rel=1e-7) and float32(0.1) != 0.1
    assert float32(1e40) == math.inf and float32(math.nan) != float32(math.nan)
    assert short32(float32(0.12)) == "0.12"
    assert short32(float32(2.4567)) == "2.4567"
    assert short32(float32(1e-8)) == "0.00000001"
    assert short32(math.inf) == "nan"
