"""Wrist_2 center (user) + soft-end probe for wire length."""

from __future__ import annotations

import threading
import time
from typing import Any

from giraffe_control.follower_config import WRIST_2_JOINT

from .. import bus as servo_bus
from ..collision.workspace import CollisionWorkspace
from .stall import StallDetector
from .sweep import (
    CALIB_ACC,
    CALIB_SPEED,
    CALIB_TORQUE_LIMIT,
    MARGIN_STEPS,
    JointRangeResult,
    SweepAbort,
    _check_abort,
    _sweep_direction,
    calib_mode,
    dry_run_range_for_joint,
    mid_offset_rad,
)
from .telemetry import ProgressCb, emit
from .urdf_limits import load_urdf_joint_limits_rad, rad_to_steps_delta

WRIST2_CENTER_ART = r"""
  WRIST 2 — WIRE CENTER

  Rotate the gripper roll by hand to the middle of the free
  cable travel (slack equal both ways):

           CW wire limit <--- [ gripper ] ---> CCW wire limit
                                   ^
                               CENTER HERE

  Keep other joints still. Support the arm — it is limp.
"""


def record_wrist2_center(packet: Any | None, servo_id: int) -> int:
    if calib_mode() == "dry_run" or packet is None:
        return 2048
    pos = servo_bus.read_pos(packet, servo_id)
    if pos is None:
        raise RuntimeError("Could not read wrist_2 position")
    return pos


def probe_wrist2_ends(
    packet: Any | None,
    servo_id: int,
    center_steps: int,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    *,
    workspace: CollisionWorkspace | None = None,
    abort: threading.Event | None = None,
    on_progress: ProgressCb | None = None,
) -> JointRangeResult:
    priors = load_urdf_joint_limits_rad()
    name = WRIST_2_JOINT
    if calib_mode() == "dry_run" or packet is None:
        emit(
            on_progress,
            "dry_run wrist_2 probe",
            packet,
            mapping,
            active_joint=name,
            dry_run=True,
        )
        r = dry_run_range_for_joint(name, priors, mid_steps=center_steps)
        r.center_steps = center_steps
        r.offset_rad = mid_offset_rad(r.range_min_steps, r.range_max_steps)
        return r

    lo_r, hi_r = priors[name]
    span = rad_to_steps_delta((hi_r - lo_r) / 2.0)
    prior_hi = servo_bus.clamp_pos(center_steps + span)
    prior_lo = servo_bus.clamp_pos(center_steps - span)

    start_steps: dict[str, int] = {}
    for n, sid in mapping.items():
        pos = servo_bus.read_pos(packet, sid)
        start_steps[n] = pos if pos is not None else 2048
    start_steps[name] = center_steps

    servo_bus.set_torque(packet, servo_id, True)
    servo_bus.set_torque_limit(packet, servo_id, CALIB_TORQUE_LIMIT)
    servo_bus.set_acceleration(packet, servo_id, CALIB_ACC)

    det = StallDetector(load_threshold=150, hold_cycles=10)
    emit(
        on_progress,
        "Wrist_2 probing + direction (wire)…",
        packet,
        mapping,
        active_joint=name,
    )
    try:
        end_hi, hi_reason = _sweep_direction(
            packet,
            servo_id,
            name,
            +1,
            prior_hi,
            det,
            mapping=mapping,
            reverses=reverses,
            start_steps=start_steps,
            priors=priors,
            workspace=workspace,
            abort=abort,
            on_progress=on_progress,
        )
        _check_abort(abort)
        servo_bus.write_pos(
            packet, servo_id, center_steps, speed=CALIB_SPEED, acc=CALIB_ACC
        )
        time.sleep(0.8)
        _check_abort(abort)
        emit(
            on_progress,
            "Wrist_2 probing − direction (wire)…",
            packet,
            mapping,
            active_joint=name,
        )
        end_lo, lo_reason = _sweep_direction(
            packet,
            servo_id,
            name,
            -1,
            prior_lo,
            det,
            mapping=mapping,
            reverses=reverses,
            start_steps=start_steps,
            priors=priors,
            workspace=workspace,
            abort=abort,
            on_progress=on_progress,
        )
    except SweepAbort:
        emit(
            on_progress,
            "Wrist_2 probe aborted",
            packet,
            mapping,
            active_joint=name,
            aborted=True,
            end_reason="abort",
        )
        raise

    mn = min(end_lo, end_hi) + MARGIN_STEPS
    mx = max(end_lo, end_hi) - MARGIN_STEPS
    mn = servo_bus.clamp_pos(min(mn, center_steps - 10))
    mx = servo_bus.clamp_pos(max(mx, center_steps + 10))

    servo_bus.write_pos(packet, servo_id, center_steps, speed=CALIB_SPEED, acc=CALIB_ACC)
    warnings: list[str] = []
    if hi_reason == "collision" or lo_reason == "collision":
        warnings.append("wrist_2: collision gated one soft end")
    return JointRangeResult(
        name=name,
        range_min_steps=mn,
        range_max_steps=mx,
        center_steps=center_steps,
        offset_rad=mid_offset_rad(mn, mx),
        prior_lower_rad=lo_r,
        prior_upper_rad=hi_r,
        dry_run=False,
        hi_reason=hi_reason,
        lo_reason=lo_reason,
        warnings=warnings,
    )
