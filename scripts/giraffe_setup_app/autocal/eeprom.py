"""Read/write Feetech EEPROM angle limits and homing offset."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from giraffe_control.follower_config import JOINT_NAMES

from .. import bus as servo_bus
from .sweep import JointRangeResult, calib_mode


@dataclass
class EepromWriteResult:
    name: str
    servo_id: int
    ok: bool
    detail: str


def homing_offset_from_center(center_steps: int) -> int:
    """Map measured center toward Feetech half-turn convention (~2048)."""
    # Store center as offset such that present≈actual-homing; use center itself
    # as a practical homing value when firmware treats OFS as zero shift.
    return int(center_steps) & 0xFFFF


def write_joint_eeprom(
    packet: Any,
    name: str,
    servo_id: int,
    result: JointRangeResult,
) -> EepromWriteResult:
    if calib_mode() == "dry_run":
        return EepromWriteResult(name, servo_id, True, "dry_run skip")

    homing = homing_offset_from_center(result.center_steps)
    ok_lim = servo_bus.write_angle_limits(
        packet, servo_id, result.range_min_steps, result.range_max_steps
    )
    ok_ofs = servo_bus.write_homing_offset(packet, servo_id, homing)
    if ok_lim and ok_ofs:
        # verify
        mn, mx = servo_bus.read_angle_limits(packet, servo_id)
        detail = f"EEPROM min={mn} max={mx} homing={homing}"
        return EepromWriteResult(name, servo_id, True, detail)
    return EepromWriteResult(
        name, servo_id, False, f"limits_ok={ok_lim} offset_ok={ok_ofs}"
    )


def write_all_eeprom(
    packet: Any | None,
    mapping: dict[str, int],
    ranges: dict[str, JointRangeResult],
) -> list[EepromWriteResult]:
    out: list[EepromWriteResult] = []
    if packet is None and calib_mode() != "dry_run":
        for name in JOINT_NAMES:
            if name in ranges:
                out.append(
                    EepromWriteResult(name, mapping.get(name, -1), False, "no bus")
                )
        return out

    for name in JOINT_NAMES:
        if name not in ranges or name not in mapping:
            continue
        res = write_joint_eeprom(packet, name, mapping[name], ranges[name])
        out.append(res)
        if not res.ok and calib_mode() != "dry_run":
            break
    return out
