"""Live joint telemetry snapshots for auto-calibration UI."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from giraffe_control.follower_config import JOINT_LABELS, JOINT_NAMES

from .. import bus as servo_bus

ProgressCb = Callable[["SweepProgress"], None]


@dataclass
class JointTelemetry:
    name: str
    label: str
    servo_id: int
    pos: int | None = None
    speed: int | None = None
    load: int | None = None
    current: int | None = None
    moving: int | None = None
    active: bool = False


@dataclass
class SweepProgress:
    message: str
    active_joint: str | None = None
    joints: list[JointTelemetry] = field(default_factory=list)
    end_reason: str | None = None  # stall|prior|collision|abort|timeout
    aborted: bool = False


def empty_table(mapping: dict[str, int]) -> list[JointTelemetry]:
    rows: list[JointTelemetry] = []
    for name in JOINT_NAMES:
        if name not in mapping:
            continue
        rows.append(
            JointTelemetry(
                name=name,
                label=JOINT_LABELS.get(name, name),
                servo_id=mapping[name],
            )
        )
    return rows


def sample_all(
    packet: Any | None,
    mapping: dict[str, int],
    *,
    active_joint: str | None = None,
    dry_run: bool = False,
) -> list[JointTelemetry]:
    rows = empty_table(mapping)
    if dry_run or packet is None:
        for row in rows:
            row.pos = 2048
            row.speed = 0
            row.load = 0
            row.current = 0
            row.moving = 0
            row.active = row.name == active_joint
        return rows

    for row in rows:
        row.pos = servo_bus.read_pos(packet, row.servo_id)
        row.speed = servo_bus.read_speed(packet, row.servo_id)
        row.load = servo_bus.read_load(packet, row.servo_id)
        row.current = servo_bus.read_current(packet, row.servo_id)
        row.moving = servo_bus.read_moving(packet, row.servo_id)
        row.active = row.name == active_joint
    return rows


def emit(
    on_progress: ProgressCb | None,
    message: str,
    packet: Any | None,
    mapping: dict[str, int],
    *,
    active_joint: str | None = None,
    dry_run: bool = False,
    end_reason: str | None = None,
    aborted: bool = False,
) -> None:
    if on_progress is None:
        return
    on_progress(
        SweepProgress(
            message=message,
            active_joint=active_joint,
            joints=sample_all(
                packet, mapping, active_joint=active_joint, dry_run=dry_run
            ),
            end_reason=end_reason,
            aborted=aborted,
        )
    )
