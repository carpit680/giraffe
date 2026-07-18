"""Load/save Giraffe follower arm config (port, servo IDs, reverse, offset, ranges)."""

from __future__ import annotations

import os
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml

JOINT_NAMES = [
    "shoulder_pan_actuator_shoulder_pan_joint",
    "shoulder_lift_actuator_shoulder_lift_joint",
    "elbow_actuator_elbow_joint",
    "wrist_1_actuator_wrist_1_joint",
    "wrist_2_actuator_wrist_2_joint",
    "finger_actuator_gripper_joint",
]

JOINT_LABELS = {
    "shoulder_pan_actuator_shoulder_pan_joint": "shoulder pan (base rotation)",
    "shoulder_lift_actuator_shoulder_lift_joint": "shoulder lift",
    "elbow_actuator_elbow_joint": "elbow",
    "wrist_1_actuator_wrist_1_joint": "wrist 1",
    "wrist_2_actuator_wrist_2_joint": "wrist 2",
    "finger_actuator_gripper_joint": "gripper finger",
}

JOINT_POSITIVE_MOTION = {
    "shoulder_pan_actuator_shoulder_pan_joint": (
        "Yaw the whole arm counter-clockwise when viewed from above (top-down)."
    ),
    "shoulder_lift_actuator_shoulder_lift_joint": (
        "Lift the upper arm upward (raise the shoulder / open the arm upward)."
    ),
    "elbow_actuator_elbow_joint": (
        "Flex the elbow — fold the forearm toward the upper arm."
    ),
    "wrist_1_actuator_wrist_1_joint": (
        "Pitch the wrist — tilt the gripper as if nodding (typical: tip toward the floor "
        "when the arm is roughly outstretched)."
    ),
    "wrist_2_actuator_wrist_2_joint": (
        "Roll the wrist — rotate the gripper counter-clockwise when looking along the "
        "forearm toward the gripper."
    ),
    "finger_actuator_gripper_joint": (
        "Close the gripper fingers (pinch inward)."
    ),
}

DEFAULT_REVERSES = {
    "shoulder_pan_actuator_shoulder_pan_joint": True,
    "shoulder_lift_actuator_shoulder_lift_joint": False,
    "elbow_actuator_elbow_joint": True,
    "wrist_1_actuator_wrist_1_joint": True,
    "wrist_2_actuator_wrist_2_joint": True,
    "finger_actuator_gripper_joint": False,
}

DEFAULT_BAUDRATE = 1_000_000
DEFAULT_MODEL = "sts3215"
WRIST_2_JOINT = "wrist_2_actuator_wrist_2_joint"


@dataclass
class MotorConfig:
    id: int
    model: str = DEFAULT_MODEL
    reverse: bool = False
    offset: float = 0.0
    range_min_steps: int | None = None
    range_max_steps: int | None = None
    homing_offset_steps: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "model": self.model,
            "reverse": self.reverse,
            "offset": self.offset,
            "range_min_steps": self.range_min_steps,
            "range_max_steps": self.range_max_steps,
            "homing_offset_steps": self.homing_offset_steps,
        }


@dataclass
class CalibrationMeta:
    floor_z: float | None = None
    mount: str = "auto"
    method: str | None = None
    urdf_prior: bool = True
    eeprom_written: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class FollowerConfig:
    port: str = "/dev/ttyACM0"
    baudrate: int = DEFAULT_BAUDRATE
    motors: dict[str, MotorConfig] = field(default_factory=dict)
    calibration: CalibrationMeta = field(default_factory=CalibrationMeta)

    def motors_for_bus(self) -> dict[str, tuple[int, str]]:
        return {name: (m.id, m.model) for name, m in self.motors.items()}

    def to_dict(self) -> dict[str, Any]:
        return {
            "port": self.port,
            "baudrate": self.baudrate,
            "motors": {name: m.to_dict() for name, m in self.motors.items()},
            "calibration": self.calibration.to_dict(),
        }


def find_repo_root(start: Path | None = None) -> Path | None:
    """Walk upward looking for config/follower.example.yaml."""
    starts: list[Path] = []
    if start is not None:
        starts.append(start)
    starts.append(Path(__file__).resolve())
    env_root = os.environ.get("GIRAFFE_ROOT")
    if env_root:
        starts.insert(0, Path(env_root))
    starts.append(Path.cwd())

    seen: set[Path] = set()
    for origin in starts:
        for candidate in [origin, *origin.parents]:
            if candidate in seen:
                continue
            seen.add(candidate)
            if (candidate / "config" / "follower.example.yaml").is_file():
                return candidate
            if (candidate / "giraffe_ws").is_dir() and (
                candidate / "config" / "follower.example.yaml"
            ).is_file():
                return candidate
    return None


def example_config_path(repo_root: Path | None = None) -> Path:
    root = repo_root or find_repo_root()
    if root is None:
        raise FileNotFoundError(
            "Could not find Giraffe repo root (looked for config/follower.example.yaml). "
            "Set GIRAFFE_ROOT or run from the repository."
        )
    return root / "config" / "follower.example.yaml"


def default_config_path(repo_root: Path | None = None) -> Path:
    root = repo_root or find_repo_root()
    if root is None:
        raise FileNotFoundError(
            "Could not find Giraffe repo root (looked for config/follower.example.yaml). "
            "Set GIRAFFE_ROOT or run from the repository."
        )
    return root / "config" / "follower.yaml"


def resolve_config_path(path: str | Path | None = None) -> Path:
    """Prefer explicit path, then follower.yaml, then the example template."""
    if path:
        return Path(path).expanduser().resolve()

    env_path = os.environ.get("GIRAFFE_FOLLOWER_CONFIG")
    if env_path:
        return Path(env_path).expanduser().resolve()

    root = find_repo_root()
    if root is not None:
        real = root / "config" / "follower.yaml"
        if real.is_file():
            return real
        example = root / "config" / "follower.example.yaml"
        if example.is_file():
            return example

    cwd_real = Path.cwd() / "config" / "follower.yaml"
    if cwd_real.is_file():
        return cwd_real.resolve()

    raise FileNotFoundError(
        "No follower config found. Run: python3 scripts/giraffe_setup.py "
        "or copy config/follower.example.yaml to config/follower.yaml"
    )


def _opt_int(value: Any) -> int | None:
    if value is None:
        return None
    return int(value)


def _parse_motor(name: str, raw: dict[str, Any]) -> MotorConfig:
    if "id" not in raw:
        raise ValueError(f"Motor '{name}' missing required field 'id'")
    return MotorConfig(
        id=int(raw["id"]),
        model=str(raw.get("model", DEFAULT_MODEL)),
        reverse=bool(raw.get("reverse", DEFAULT_REVERSES.get(name, False))),
        offset=float(raw.get("offset", 0.0)),
        range_min_steps=_opt_int(raw.get("range_min_steps")),
        range_max_steps=_opt_int(raw.get("range_max_steps")),
        homing_offset_steps=_opt_int(raw.get("homing_offset_steps")),
    )


def _parse_calibration(raw: dict[str, Any] | None) -> CalibrationMeta:
    raw = raw or {}
    floor = raw.get("floor_z")
    return CalibrationMeta(
        floor_z=float(floor) if floor is not None else None,
        mount=str(raw.get("mount", "auto")),
        method=raw.get("method"),
        urdf_prior=bool(raw.get("urdf_prior", True)),
        eeprom_written=bool(raw.get("eeprom_written", False)),
    )


def load_follower_config(path: str | Path | None = None) -> tuple[FollowerConfig, Path]:
    config_path = resolve_config_path(path)
    with open(config_path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    motors_raw = data.get("motors") or {}
    motors: dict[str, MotorConfig] = {}
    for name in JOINT_NAMES:
        if name not in motors_raw:
            raise ValueError(f"Config {config_path} missing motor '{name}'")
        motors[name] = _parse_motor(name, motors_raw[name])

    cfg = FollowerConfig(
        port=str(data.get("port", "/dev/ttyACM0")),
        baudrate=int(data.get("baudrate", DEFAULT_BAUDRATE)),
        motors=motors,
        calibration=_parse_calibration(data.get("calibration")),
    )
    return cfg, config_path


def save_follower_config(config: FollowerConfig, path: str | Path | None = None) -> Path:
    if path is None:
        path = default_config_path()
    else:
        path = Path(path)
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)

    header = (
        "# Generated by scripts/giraffe_setup.py — arm-specific, do not commit.\n"
        "# Re-run the setup wizard anytime to regenerate this file.\n"
    )
    with open(path, "w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(config.to_dict(), f, default_flow_style=False, sort_keys=False)
    return path


def make_default_config(
    port: str = "/dev/ttyACM0",
    ids: dict[str, int] | None = None,
) -> FollowerConfig:
    """Build a config with default reverses and zero offsets."""
    motors: dict[str, MotorConfig] = {}
    for i, name in enumerate(JOINT_NAMES, start=1):
        motor_id = ids[name] if ids and name in ids else i
        motors[name] = MotorConfig(
            id=motor_id,
            model=DEFAULT_MODEL,
            reverse=DEFAULT_REVERSES.get(name, False),
            offset=0.0,
        )
    return FollowerConfig(port=port, baudrate=DEFAULT_BAUDRATE, motors=motors)
