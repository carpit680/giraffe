"""Shared wizard state for the Giraffe setup TUI."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from giraffe_control.follower_config import (
    DEFAULT_BAUDRATE,
    DEFAULT_REVERSES,
    JOINT_NAMES,
    FollowerConfig,
    MotorConfig,
    default_config_path,
    load_follower_config,
)

from . import bus as servo_bus
from .autocal.engine import AutoCalibState

STEPS = [
    "welcome",
    "env_check",
    "port_select",
    "scan",
    "safety",
    "map_joints",
    "review_map",
    "reverses",
    "auto_calibrate",
    "wrist2_ground",
    "calib_confirm",
    "smoke_test",
    "done",
]

STEP_TITLES = {
    "welcome": "Welcome",
    "edit_hub": "Edit config",
    "env_check": "Environment",
    "port_select": "Serial port",
    "scan": "Scan servos",
    "safety": "Safety",
    "map_joints": "Map joints",
    "review_map": "Review mapping",
    "reverses": "Reverse flags",
    "calibrate": "Zero calibration (legacy)",
    "auto_calibrate": "Limit sweeps",
    "wrist2_ground": "Wrist_2 + ground",
    "calib_confirm": "Confirm / EEPROM",
    "write_confirm": "Write config",
    "smoke_test": "Smoke test",
    "done": "Done",
}


@dataclass
class SetupState:
    repo_root: Path
    port: str | None = None
    baudrate: int = DEFAULT_BAUDRATE
    port_handler: Any | None = None
    packet: Any | None = None
    found_ids: list[int] = field(default_factory=list)
    mapping: dict[str, int] = field(default_factory=dict)
    reverses: dict[str, bool] = field(default_factory=lambda: dict(DEFAULT_REVERSES))
    offsets: dict[str, float] = field(
        default_factory=lambda: {name: 0.0 for name in JOINT_NAMES}
    )
    safety_confirmed: bool = False
    torque_off_accepted: bool = False
    torque_disabled: bool = False
    calibrate_torque_accepted: bool = False
    calibrate_torque_disabled: bool = False
    config_path: Path | None = None
    edit_mode: bool = False
    auto_calib: AutoCalibState | None = None
    # MapJoints: which servo ID is currently being assigned; None = pick next
    active_map_id: int | None = None
    redo_servo_id: int | None = None  # force remapping this ID
    smoke_index: int = 0
    message: str = ""

    def existing_config_path(self) -> Path:
        return default_config_path(self.repo_root)

    def has_existing_config(self) -> bool:
        return self.existing_config_path().is_file()

    def load_from_disk(self, path: Path | None = None) -> Path:
        """Load follower.yaml into this state (does not open the serial bus)."""
        cfg, resolved = load_follower_config(path or self.existing_config_path())
        if cfg.port != "auto":
            self.port = cfg.port
        self.baudrate = cfg.baudrate
        self.mapping = {name: m.id for name, m in cfg.motors.items()}
        self.reverses = {name: m.reverse for name, m in cfg.motors.items()}
        self.offsets = {name: m.offset for name, m in cfg.motors.items()}
        self.found_ids = sorted(set(self.mapping.values()))
        self.config_path = resolved
        self.edit_mode = True
        self.safety_confirmed = True  # already set up once
        self.message = f"Loaded {resolved}"
        return resolved

    def close_bus(self) -> None:
        servo_bus.close_bus(self.port_handler)
        self.port_handler = None
        self.packet = None

    def reset(self) -> None:
        self.close_bus()
        self.port = None
        self.found_ids = []
        self.mapping = {}
        self.reverses = dict(DEFAULT_REVERSES)
        self.offsets = {name: 0.0 for name in JOINT_NAMES}
        self.safety_confirmed = False
        self.torque_off_accepted = False
        self.torque_disabled = False
        self.calibrate_torque_accepted = False
        self.calibrate_torque_disabled = False
        self.config_path = None
        self.edit_mode = False
        self.auto_calib = None
        self.active_map_id = None
        self.redo_servo_id = None
        self.smoke_index = 0
        self.message = ""

    @property
    def mapped_count(self) -> int:
        return len(self.mapping)

    def unmapped_ids(self) -> list[int]:
        used = set(self.mapping.values())
        return [i for i in self.found_ids if i not in used]

    def remaining_joints(self) -> list[str]:
        return [n for n in JOINT_NAMES if n not in self.mapping]

    def unmap_joint(self, joint: str) -> int | None:
        servo_id = self.mapping.pop(joint, None)
        return servo_id

    def unmap_all(self) -> None:
        self.mapping.clear()
        self.active_map_id = None
        self.redo_servo_id = None

    def build_config(self) -> FollowerConfig:
        if not self.port:
            raise RuntimeError("No port selected")
        motors = {
            name: MotorConfig(
                id=self.mapping[name],
                reverse=self.reverses[name],
                offset=self.offsets.get(name, 0.0),
            )
            for name in JOINT_NAMES
        }
        return FollowerConfig(
            port=self.port, baudrate=self.baudrate, motors=motors
        )

    def status_text(self) -> str:
        port = self.port or "—"
        ids = len(self.found_ids)
        mapped = self.mapped_count
        return f"port {port}  ·  IDs {ids}  ·  mapped {mapped}/6"
