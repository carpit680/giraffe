"""Precomputed joint-space collision workspace (lightweight sphere FK)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np

from giraffe_control.follower_config import JOINT_NAMES, find_repo_root

from .acm import load_disabled_pairs
from .floor import FloorPlane, MountBox
from ..autocal.urdf_limits import load_urdf_joint_limits_rad

# Ordered arm links for sphere centers along a simplified serial chain.
LINK_SPHERES = [
    ("base", 0.04),
    ("shoulder_pan", 0.04),
    ("shoulder_lift", 0.045),
    ("elbow", 0.04),
    ("wrist_1", 0.035),
    ("wrist_2", 0.035),
    ("gripper", 0.03),
]

# Approximate segment lengths between spheres (m).
SEGMENT_LENS = [0.04, 0.05, 0.12, 0.12, 0.05, 0.06]


@dataclass
class CollisionWorkspace:
    floor: FloorPlane = field(default_factory=FloorPlane)
    mount: MountBox = field(default_factory=MountBox)
    samples: np.ndarray | None = None  # (N, 6) free configs
    disabled_pairs: set[tuple[str, str]] = field(default_factory=set)
    joint_limits: dict[str, tuple[float, float]] = field(default_factory=dict)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            samples=self.samples if self.samples is not None else np.zeros((0, 6)),
            floor_z=np.array([self.floor.floor_z]),
            mount=np.array(
                [
                    self.mount.x_min,
                    self.mount.x_max,
                    self.mount.y_min,
                    self.mount.y_max,
                    self.mount.z_min,
                    self.mount.z_max,
                ]
            ),
        )

    @classmethod
    def load(cls, path: Path) -> CollisionWorkspace:
        data = np.load(path, allow_pickle=False)
        ws = cls()
        ws.samples = data["samples"]
        ws.floor = FloorPlane(floor_z=float(data["floor_z"][0]))
        m = data["mount"]
        ws.mount = MountBox(*[float(x) for x in m])
        ws.disabled_pairs = load_disabled_pairs()
        ws.joint_limits = load_urdf_joint_limits_rad()
        return ws


def _fk_spheres(q: dict[str, float]) -> list[tuple[str, float, float, float, float]]:
    """Return list of (name, x, y, z, radius) for coarse collision."""
    yaw = q.get("shoulder_pan_actuator_shoulder_pan_joint", 0.0)
    lift = q.get("shoulder_lift_actuator_shoulder_lift_joint", 0.0)
    elbow = q.get("elbow_actuator_elbow_joint", 0.0)
    w1 = q.get("wrist_1_actuator_wrist_1_joint", 0.0)
    # ignore wrist_2 / gripper for chain pose; still place tip spheres

    pts: list[tuple[str, float, float, float, float]] = []
    x = y = 0.0
    z = 0.0
    pts.append(("base", x, y, z, LINK_SPHERES[0][1]))

    # pan rotates the following chain in XY
    pitch = 0.0
    names = [n for n, _ in LINK_SPHERES[1:]]
    pitches = [lift, elbow, w1, 0.0, 0.0]
    for i, name in enumerate(names):
        pitch += pitches[i] if i < len(pitches) else 0.0
        length = SEGMENT_LENS[i] if i < len(SEGMENT_LENS) else 0.05
        x += length * math.cos(pitch) * math.cos(yaw)
        y += length * math.cos(pitch) * math.sin(yaw)
        z += length * math.sin(pitch)
        r = LINK_SPHERES[i + 1][1]
        pts.append((name, x, y, z, r))
    return pts


def _spheres_collide(
    a: tuple[str, float, float, float, float],
    b: tuple[str, float, float, float, float],
    disabled: set[tuple[str, str]],
) -> bool:
    na, xa, ya, za, ra = a
    nb, xb, yb, zb, rb = b
    key = tuple(sorted((na, nb)))
    if key in disabled:
        return False
    # skip adjacent links in chain
    names = [n for n, _ in LINK_SPHERES]
    if na in names and nb in names:
        ia, ib = names.index(na), names.index(nb)
        if abs(ia - ib) <= 1:
            return False
    dx, dy, dz = xa - xb, ya - yb, za - zb
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    return dist < (ra + rb)


def is_collision_free(
    q: dict[str, float],
    floor: FloorPlane,
    mount: MountBox,
    disabled: set[tuple[str, str]],
) -> bool:
    spheres = _fk_spheres(q)
    # floor / mount
    for name, x, y, z, r in spheres:
        if name == "base":
            continue
        if floor.penetrates(z - r):
            return False
        if mount.contains(x, y, z):
            # gripper/arm in mount volume is collision
            if name != "base":
                return False
    # self
    for i in range(len(spheres)):
        for j in range(i + 1, len(spheres)):
            if _spheres_collide(spheres[i], spheres[j], disabled):
                return False
    return True


def build_workspace(
    floor_z: float = -0.02,
    samples_per_joint: int = 5,
    repo_root: Path | None = None,
) -> CollisionWorkspace:
    limits = load_urdf_joint_limits_rad()
    disabled = load_disabled_pairs()
    floor = FloorPlane(floor_z=floor_z)
    mount = MountBox(z_max=min(0.0, floor_z + 0.01))

    grids = []
    for name in JOINT_NAMES:
        lo, hi = limits[name]
        grids.append(np.linspace(lo, hi, samples_per_joint))

    free: list[list[float]] = []
    # Cartesian product can explode — use coarse grid 5^6=15625 ok
    from itertools import product

    for vals in product(*grids):
        q = {n: float(v) for n, v in zip(JOINT_NAMES, vals)}
        if is_collision_free(q, floor, mount, disabled):
            free.append([q[n] for n in JOINT_NAMES])

    ws = CollisionWorkspace(
        floor=floor,
        mount=mount,
        samples=np.array(free, dtype=float) if free else np.zeros((0, 6)),
        disabled_pairs=disabled,
        joint_limits=limits,
    )
    return ws


def default_workspace_path(repo_root: Path | None = None) -> Path:
    root = repo_root or find_repo_root()
    if root is None:
        raise FileNotFoundError("repo root not found")
    return root / "config" / "collision_workspace.npz"


def ensure_workspace(
    floor_z: float = -0.02,
    repo_root: Path | None = None,
    rebuild: bool = False,
) -> CollisionWorkspace:
    path = default_workspace_path(repo_root)
    if path.is_file() and not rebuild:
        ws = CollisionWorkspace.load(path)
        # refresh floor if changed significantly
        if abs(ws.floor.floor_z - floor_z) > 1e-3:
            ws = build_workspace(floor_z=floor_z, repo_root=repo_root)
            ws.save(path)
        return ws
    ws = build_workspace(floor_z=floor_z, repo_root=repo_root)
    ws.save(path)
    return ws


def query_free(ws: CollisionWorkspace, q: dict[str, float]) -> bool:
    return is_collision_free(q, ws.floor, ws.mount, ws.disabled_pairs)


def nearest_safe(ws: CollisionWorkspace, q: dict[str, float]) -> dict[str, float] | None:
    if ws.samples is None or len(ws.samples) == 0:
        return None
    vec = np.array([q[n] for n in JOINT_NAMES], dtype=float)
    d = np.linalg.norm(ws.samples - vec, axis=1)
    idx = int(np.argmin(d))
    best = ws.samples[idx]
    return {n: float(best[i]) for i, n in enumerate(JOINT_NAMES)}
