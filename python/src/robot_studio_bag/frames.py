"""Rigid transforms for the importer: 3D (TF) and planar (the recording's x, y, theta)."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Transform:
    """A rigid transform: translation ``t`` and unit quaternion ``q`` = (x, y, z, w)."""

    t: tuple[float, float, float] = (0.0, 0.0, 0.0)
    q: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    @staticmethod
    def from_msg(transform: Mapping[str, Any]) -> Transform:
        """From a ``geometry_msgs/Transform`` as rosbag-inspect decodes it."""
        tr = transform["translation"]
        rot = transform["rotation"]
        q = (float(rot["x"]), float(rot["y"]), float(rot["z"]), float(rot["w"]))
        n = math.sqrt(sum(c * c for c in q))
        if not (n > 0 and math.isfinite(n)):
            raise ValueError("transform rotation is not a valid quaternion")
        return Transform(
            (float(tr["x"]), float(tr["y"]), float(tr["z"])),
            (q[0] / n, q[1] / n, q[2] / n, q[3] / n),
        )

    def rotate(self, v: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z, w = self.q
        # v' = v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
        cx = y * v[2] - z * v[1]
        cy = z * v[0] - x * v[2]
        cz = x * v[1] - y * v[0]
        ccx = y * cz - z * cy
        ccy = z * cx - x * cz
        ccz = x * cy - y * cx
        return (
            v[0] + 2 * (w * cx + ccx),
            v[1] + 2 * (w * cy + ccy),
            v[2] + 2 * (w * cz + ccz),
        )

    def compose(self, other: Transform) -> Transform:
        """self * other: ``other`` expressed in the frame of ``self``."""
        x1, y1, z1, w1 = self.q
        x2, y2, z2, w2 = other.q
        q = (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )
        r = self.rotate(other.t)
        return Transform((self.t[0] + r[0], self.t[1] + r[1], self.t[2] + r[2]), q)

    def inverse(self) -> Transform:
        x, y, z, w = self.q
        inv = Transform((0.0, 0.0, 0.0), (-x, -y, -z, w))
        r = inv.rotate(self.t)
        return Transform((-r[0], -r[1], -r[2]), inv.q)

    @property
    def yaw(self) -> float:
        x, y, z, w = self.q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def planar(self) -> Pose2:
        return Pose2(self.t[0], self.t[1], self.yaw)


@dataclass(frozen=True)
class Pose2:
    """A planar pose: x, y (m) and heading theta (rad)."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def compose(self, other: Pose2) -> Pose2:
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return Pose2(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            wrap(self.theta + other.theta),
        )

    def inverse(self) -> Pose2:
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return Pose2(-c * self.x - s * self.y, s * self.x - c * self.y, wrap(-self.theta))

    @staticmethod
    def from_any(value: object) -> Pose2:
        """From ``{"x", "y", "theta" | "yaw"}``; missing or bad values are 0."""
        if not isinstance(value, Mapping):
            return Pose2()

        def num(key: str) -> float:
            v = value.get(key)
            return float(v) if isinstance(v, (int, float)) and math.isfinite(v) else 0.0

        return Pose2(num("x"), num("y"), num("theta") if "theta" in value else num("yaw"))


def wrap(a: float) -> float:
    """Angle in (-pi, pi]."""
    a = math.fmod(a, 2 * math.pi)
    if a <= -math.pi:
        a += 2 * math.pi
    elif a > math.pi:
        a -= 2 * math.pi
    return a


def twist_at(vx: float, vy: float, wz: float, offset: Pose2) -> tuple[float, float, float]:
    """Body twist of a frame at ``offset`` (in the moving frame), in that frame's axes."""
    ux = vx - wz * offset.y
    uy = vy + wz * offset.x
    c = math.cos(offset.theta)
    s = math.sin(offset.theta)
    return (c * ux + s * uy, -s * ux + c * uy, wz)


class StaticTree:
    """Fixed transforms parent -> child (from /tf_static), looked up between any two frames."""

    def __init__(self) -> None:
        self.parent: dict[str, tuple[str, Transform]] = {}

    def add(self, parent: str, child: str, transform: Transform) -> None:
        self.parent[child.lstrip("/")] = (parent.lstrip("/"), transform)

    def _to_root(self, frame: str) -> tuple[str, Transform]:
        """The root of ``frame``'s tree and ``frame``'s pose in it."""
        pose = Transform()
        seen: set[str] = set()
        while frame in self.parent:
            if frame in seen:
                raise ValueError(f"/tf_static has a cycle through {frame}")
            seen.add(frame)
            parent, tr = self.parent[frame]
            pose = tr.compose(pose)
            frame = parent
        return frame, pose

    def lookup(self, target: str, source: str) -> Transform | None:
        """Pose of frame ``source`` in frame ``target``, or None if they are not connected."""
        target = target.lstrip("/")
        source = source.lstrip("/")
        if target == source:
            return Transform()
        rt, pt = self._to_root(target)
        rs, ps = self._to_root(source)
        if rt != rs:
            return None
        return pt.inverse().compose(ps)
