"""Orchestrate auto-calibration results into follower config fields."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any

from giraffe_control.follower_config import (
    WRIST_2_JOINT,
    CalibrationMeta,
    FollowerConfig,
    JOINT_NAMES,
    MotorConfig,
)

from ..collision.workspace import CollisionWorkspace
from .ground import probe_ground_height
from .sweep import JointRangeResult, SweepAbort, sweep_all_except_wrist2
from .telemetry import ProgressCb
from .wrist2 import probe_wrist2_ends, record_wrist2_center


@dataclass
class AutoCalibState:
    ranges: dict[str, JointRangeResult] = field(default_factory=dict)
    wrist2_center: int | None = None
    floor_z: float | None = None
    phase: str = "idle"  # idle|limits|wrist2_center|wrist2_probe|ground|done
    message: str = ""
    wrist2_torque_accepted: bool = False
    wrist2_torque_disabled: bool = False
    sweeping: bool = False
    last_warnings: list[str] = field(default_factory=list)


def run_limit_sweeps(
    packet: Any | None,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    calib: AutoCalibState,
    *,
    workspace: CollisionWorkspace | None = None,
    abort: threading.Event | None = None,
    on_progress: ProgressCb | None = None,
) -> AutoCalibState:
    calib.phase = "limits"
    calib.sweeping = True
    calib.last_warnings = []
    try:
        results = sweep_all_except_wrist2(
            packet,
            mapping,
            reverses,
            workspace=workspace,
            abort=abort,
            on_progress=on_progress,
        )
        calib.ranges.update(results)
        for r in results.values():
            calib.last_warnings.extend(r.warnings)
        calib.message = f"Swept {len(results)} joints (wrist_2 next)"
    except SweepAbort:
        calib.message = "Limit sweep stopped by operator"
        raise
    finally:
        calib.sweeping = False
    return calib


def apply_wrist2_center(
    packet: Any | None,
    mapping: dict[str, int],
    calib: AutoCalibState,
) -> AutoCalibState:
    calib.phase = "wrist2_center"
    sid = mapping[WRIST_2_JOINT]
    calib.wrist2_center = record_wrist2_center(packet, sid)
    calib.message = f"Wrist_2 center @ {calib.wrist2_center}"
    return calib


def run_wrist2_probe(
    packet: Any | None,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    calib: AutoCalibState,
    *,
    workspace: CollisionWorkspace | None = None,
    abort: threading.Event | None = None,
    on_progress: ProgressCb | None = None,
) -> AutoCalibState:
    if calib.wrist2_center is None:
        raise RuntimeError("wrist_2 center not recorded")
    calib.phase = "wrist2_probe"
    calib.sweeping = True
    sid = mapping[WRIST_2_JOINT]
    try:
        result = probe_wrist2_ends(
            packet,
            sid,
            calib.wrist2_center,
            mapping,
            reverses,
            workspace=workspace,
            abort=abort,
            on_progress=on_progress,
        )
        calib.ranges[WRIST_2_JOINT] = result
        calib.last_warnings.extend(result.warnings)
        calib.message = "Wrist_2 soft limits measured"
    except SweepAbort:
        calib.message = "Wrist_2 probe stopped by operator"
        raise
    finally:
        calib.sweeping = False
    return calib


def run_ground_probe(
    packet: Any | None,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    calib: AutoCalibState,
    *,
    abort: threading.Event | None = None,
    on_progress: ProgressCb | None = None,
) -> AutoCalibState:
    calib.phase = "ground"
    calib.sweeping = True
    offsets = {n: r.offset_rad for n, r in calib.ranges.items()}
    for n in JOINT_NAMES:
        offsets.setdefault(n, 0.0)
    try:
        calib.floor_z = probe_ground_height(
            packet,
            mapping,
            reverses,
            offsets,
            abort=abort,
            on_progress=on_progress,
        )
        calib.phase = "done"
        calib.message = f"Ground floor_z={calib.floor_z:.4f}"
    except SweepAbort:
        calib.message = "Ground probe stopped by operator"
        raise
    finally:
        calib.sweeping = False
    return calib


def build_follower_config(
    port: str,
    baudrate: int,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    calib: AutoCalibState,
    models: dict[str, str] | None = None,
) -> FollowerConfig:
    motors: dict[str, MotorConfig] = {}
    for name in JOINT_NAMES:
        r = calib.ranges.get(name)
        motors[name] = MotorConfig(
            id=mapping[name],
            model=(models or {}).get(name, "sts3215"),
            reverse=reverses.get(name, False),
            offset=r.offset_rad if r else 0.0,
            range_min_steps=r.range_min_steps if r else None,
            range_max_steps=r.range_max_steps if r else None,
            homing_offset_steps=r.center_steps if r else None,
        )
    meta = CalibrationMeta(
        floor_z=calib.floor_z,
        mount="auto",
        method="auto_v1",
        urdf_prior=True,
        eeprom_written=False,
    )
    return FollowerConfig(
        port=port, baudrate=baudrate, motors=motors, calibration=meta
    )
