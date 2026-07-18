"""Parse URDF joint limit priors for Giraffe."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from giraffe_control.follower_config import JOINT_NAMES, find_repo_root

# Hardcoded fallbacks matching giraffe.xacro.urdf if file missing.
_FALLBACK_LIMITS_RAD: dict[str, tuple[float, float]] = {
    "shoulder_pan_actuator_shoulder_pan_joint": (-1.570796, 1.570796),
    "shoulder_lift_actuator_shoulder_lift_joint": (-0.244346, 3.054326),
    "elbow_actuator_elbow_joint": (-0.436332, 3.054326),
    "wrist_1_actuator_wrist_1_joint": (-1.570796, 1.570796),
    "wrist_2_actuator_wrist_2_joint": (-3.124139, 3.124139),
    "finger_actuator_gripper_joint": (0.0, 1.570796),
}


def default_urdf_path(repo_root: Path | None = None) -> Path:
    root = repo_root or find_repo_root()
    if root is None:
        raise FileNotFoundError("repo root not found")
    return (
        root
        / "giraffe_ws"
        / "src"
        / "giraffe_description"
        / "urdf"
        / "giraffe.xacro.urdf"
    )


def load_urdf_joint_limits_rad(urdf_path: Path | None = None) -> dict[str, tuple[float, float]]:
    path = urdf_path or default_urdf_path()
    if not path.is_file():
        return dict(_FALLBACK_LIMITS_RAD)

    tree = ET.parse(path)
    root = tree.getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in JOINT_NAMES:
            continue
        lim = joint.find("limit")
        if lim is None:
            continue
        lower = float(lim.get("lower", "0"))
        upper = float(lim.get("upper", "0"))
        limits[name] = (lower, upper)
    for name in JOINT_NAMES:
        limits.setdefault(name, _FALLBACK_LIMITS_RAD[name])
    return limits


def rad_to_steps_delta(radians: float, resolution: int = 4096) -> int:
    return int(abs(radians) / (2 * 3.141592653589793) * resolution)
