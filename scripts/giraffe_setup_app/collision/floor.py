"""Floor / mount collision helpers."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class FloorPlane:
    """z = floor_z plane in base frame (z up). Contact if point.z <= floor_z."""

    floor_z: float = -0.02
    margin: float = 0.005

    def penetrates(self, z: float) -> bool:
        return z <= (self.floor_z + self.margin)


@dataclass
class MountBox:
    """AABB under the base for clamp / robot mount structure."""

    x_min: float = -0.08
    x_max: float = 0.08
    y_min: float = -0.08
    y_max: float = 0.08
    z_min: float = -0.15
    z_max: float = 0.0

    def contains(self, x: float, y: float, z: float) -> bool:
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )
