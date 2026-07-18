"""Per-joint low-torque limit sweeps with URDF priors + collision gating."""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from giraffe_control.follower_config import JOINT_NAMES, WRIST_2_JOINT

from .. import bus as servo_bus
from ..collision.workspace import CollisionWorkspace, query_free
from .stall import StallDetector
from .telemetry import ProgressCb, emit
from .urdf_limits import load_urdf_joint_limits_rad, rad_to_steps_delta

# Sweep order: tip-first (LeRobot-style), wrist_2 handled separately
SWEEP_ORDER = [
    "finger_actuator_gripper_joint",
    "wrist_1_actuator_wrist_1_joint",
    "elbow_actuator_elbow_joint",
    "shoulder_lift_actuator_shoulder_lift_joint",
    "shoulder_pan_actuator_shoulder_pan_joint",
]

MARGIN_STEPS = 70  # ~6°
CALIB_SPEED = 80
CALIB_ACC = 20
CALIB_TORQUE_LIMIT = 200
STEP_CHUNK = 40
POLL_DT = 0.05
MAX_SWEEP_S = 25.0
# Stall within this fraction of prior travel counts as a true endstop
PRIOR_NEAR_FRAC = 0.55


def calib_mode() -> str:
    return os.environ.get("GIRAFFE_CALIB_MODE", "hw").strip().lower()


@dataclass
class JointRangeResult:
    name: str
    range_min_steps: int
    range_max_steps: int
    center_steps: int
    offset_rad: float
    prior_lower_rad: float
    prior_upper_rad: float
    dry_run: bool = False
    hi_reason: str = "stall"  # stall|prior|collision|abort|timeout
    lo_reason: str = "stall"
    warnings: list[str] = field(default_factory=list)


class SweepAbort(Exception):
    """Raised when the operator presses Stop."""


def mid_offset_rad(min_steps: int, max_steps: int, resolution: int = 4096) -> float:
    mid = 0.5 * (min_steps + max_steps)
    return float(mid / resolution * 2 * 3.141592653589793)


def _apply_soft_caps(packet: Any, servo_id: int) -> None:
    servo_bus.set_torque(packet, servo_id, True)
    servo_bus.set_torque_limit(packet, servo_id, CALIB_TORQUE_LIMIT)
    servo_bus.set_acceleration(packet, servo_id, CALIB_ACC)


def _check_abort(abort: threading.Event | None) -> None:
    if abort is not None and abort.is_set():
        raise SweepAbort("stopped by operator")


def steps_to_joint_rad(steps: int, offset_rad: float, reverse: bool) -> float:
    pos_rad = (steps / 4096.0) * 2 * math.pi
    radians = -pos_rad + offset_rad
    if reverse:
        radians = -radians
    return float(radians)


def estimate_q_from_starts(
    packet: Any,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    start_steps: dict[str, int],
    priors: dict[str, tuple[float, float]],
    *,
    override_name: str | None = None,
    override_steps: int | None = None,
) -> dict[str, float]:
    """Map current encoder poses into URDF joint space assuming start ≈ mid-prior."""
    q: dict[str, float] = {}
    for name in JOINT_NAMES:
        if name not in mapping:
            lo, hi = priors.get(name, (-1.0, 1.0))
            q[name] = 0.5 * (lo + hi)
            continue
        if name == override_name and override_steps is not None:
            pos = override_steps
        else:
            pos = servo_bus.read_pos(packet, mapping[name])
            if pos is None:
                pos = start_steps.get(name, 2048)
        start = start_steps.get(name, pos)
        lo, hi = priors[name]
        mid = 0.5 * (lo + hi)
        delta = pos - start
        # Match giraffe_driver joint-space sign for positive command
        if reverses.get(name, False):
            delta_rad = (delta / 4096.0) * 2 * math.pi
        else:
            delta_rad = -(delta / 4096.0) * 2 * math.pi
        q[name] = mid + delta_rad
    return q


def _classify_end(
    start: int,
    end: int,
    prior_bound: int,
    *,
    stalled: bool,
    collided: bool,
    aborted: bool,
    timed_out: bool,
) -> str:
    if aborted:
        return "abort"
    if collided:
        return "collision"
    if timed_out:
        return "timeout"
    travel = abs(prior_bound - start)
    moved = abs(end - start)
    near_prior = travel <= 2 or moved >= PRIOR_NEAR_FRAC * max(travel, 1)
    if near_prior and not stalled:
        return "prior"
    if near_prior:
        return "stall"
    if stalled:
        # Early stall in free space → treat as mechanical stop before URDF prior
        return "stall"
    return "prior"


def _sweep_direction(
    packet: Any,
    servo_id: int,
    joint_name: str,
    direction: int,
    prior_bound_steps: int,
    detector: StallDetector,
    *,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    start_steps: dict[str, int],
    priors: dict[str, tuple[float, float]],
    workspace: CollisionWorkspace | None,
    abort: threading.Event | None,
    on_progress: ProgressCb | None,
) -> tuple[int, str]:
    """Move until stall, prior, collision, abort, or timeout. Returns (pos, reason)."""
    start = servo_bus.read_pos(packet, servo_id)
    if start is None:
        raise RuntimeError(f"cannot read ID {servo_id}")

    detector.reset()
    t0 = time.time()
    pos = start
    last_free = start
    while time.time() - t0 < MAX_SWEEP_S:
        _check_abort(abort)
        target = servo_bus.clamp_pos(pos + direction * STEP_CHUNK)
        if direction > 0 and target >= prior_bound_steps:
            target = prior_bound_steps
        if direction < 0 and target <= prior_bound_steps:
            target = prior_bound_steps

        if workspace is not None:
            q = estimate_q_from_starts(
                packet,
                mapping,
                reverses,
                start_steps,
                priors,
                override_name=joint_name,
                override_steps=target,
            )
            if not query_free(workspace, q):
                emit(
                    on_progress,
                    f"{joint_name}: collision ahead — stopping this direction",
                    packet,
                    mapping,
                    active_joint=joint_name,
                    end_reason="collision",
                )
                return last_free, "collision"

        servo_bus.write_pos(packet, servo_id, target, speed=CALIB_SPEED, acc=CALIB_ACC)
        time.sleep(POLL_DT)
        _check_abort(abort)

        pos_now = servo_bus.read_pos(packet, servo_id)
        load = servo_bus.read_load(packet, servo_id) or 0
        current = servo_bus.read_current(packet, servo_id)
        speed = servo_bus.read_speed(packet, servo_id)
        if pos_now is None:
            continue
        pos = pos_now
        last_free = pos
        emit(
            on_progress,
            f"Moving {joint_name}  pos={pos}  vel={speed}  load={load}  cur={current}",
            packet,
            mapping,
            active_joint=joint_name,
        )
        if detector.update(pos, load, current):
            reason = _classify_end(
                start,
                pos,
                prior_bound_steps,
                stalled=True,
                collided=False,
                aborted=False,
                timed_out=False,
            )
            return pos, reason
        if direction > 0 and pos >= prior_bound_steps - 2:
            return pos, "prior"
        if direction < 0 and pos <= prior_bound_steps + 2:
            return pos, "prior"
    return pos, "timeout"


def dry_run_range_for_joint(
    name: str, priors: dict[str, tuple[float, float]], mid_steps: int = 2048
) -> JointRangeResult:
    lo_r, hi_r = priors[name]
    half = rad_to_steps_delta((hi_r - lo_r) / 2.0)
    mn = servo_bus.clamp_pos(mid_steps - half)
    mx = servo_bus.clamp_pos(mid_steps + half)
    if mx - mn < 100:
        mx = servo_bus.clamp_pos(mn + 200)
    return JointRangeResult(
        name=name,
        range_min_steps=mn + MARGIN_STEPS,
        range_max_steps=mx - MARGIN_STEPS,
        center_steps=(mn + mx) // 2,
        offset_rad=mid_offset_rad(mn + MARGIN_STEPS, mx - MARGIN_STEPS),
        prior_lower_rad=lo_r,
        prior_upper_rad=hi_r,
        dry_run=True,
        hi_reason="prior",
        lo_reason="prior",
    )


def park_unfold(
    packet: Any,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    *,
    abort: threading.Event | None = None,
    on_progress: ProgressCb | None = None,
) -> None:
    """LeRobot-inspired raise: lift / elbow / wrist_1 toward a clear mid pose."""
    # Positive joint-space deltas (rad) — same sign convention as giraffe_driver
    targets = [
        ("shoulder_lift_actuator_shoulder_lift_joint", 0.45),
        ("elbow_actuator_elbow_joint", 0.35),
        ("wrist_1_actuator_wrist_1_joint", 0.20),
    ]
    emit(on_progress, "Unfolding to a clear pose…", packet, mapping)
    for name, delta_rad in targets:
        _check_abort(abort)
        if name not in mapping:
            continue
        sid = mapping[name]
        _apply_soft_caps(packet, sid)
        start = servo_bus.read_pos(packet, sid)
        if start is None:
            continue
        step_delta = rad_to_steps_delta(delta_rad)
        signed = step_delta if reverses.get(name, False) else -step_delta
        target = servo_bus.clamp_pos(start + signed)
        emit(
            on_progress,
            f"Unfold {name} → {target}",
            packet,
            mapping,
            active_joint=name,
        )
        servo_bus.write_pos(packet, sid, target, speed=CALIB_SPEED, acc=CALIB_ACC)
        time.sleep(0.9)


def sweep_joint(
    packet: Any | None,
    name: str,
    servo_id: int,
    reverse: bool,
    priors: dict[str, tuple[float, float]],
    *,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    start_steps: dict[str, int],
    workspace: CollisionWorkspace | None = None,
    abort: threading.Event | None = None,
    on_progress: ProgressCb | None = None,
) -> JointRangeResult:
    mode = calib_mode()
    lo_r, hi_r = priors[name]
    span = rad_to_steps_delta(hi_r - lo_r)
    warnings: list[str] = []

    if mode == "dry_run" or packet is None:
        emit(
            on_progress,
            f"dry_run sweep {name}",
            packet,
            mapping,
            active_joint=name,
            dry_run=True,
        )
        time.sleep(0.15)
        return dry_run_range_for_joint(name, priors)

    start = servo_bus.read_pos(packet, servo_id)
    if start is None:
        raise RuntimeError(f"Failed to read {name}")

    prior_hi = servo_bus.clamp_pos(start + span)
    prior_lo = servo_bus.clamp_pos(start - span)

    _apply_soft_caps(packet, servo_id)
    det = StallDetector()

    emit(
        on_progress,
        f"Sweeping {name} toward + prior…",
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
        time.sleep(0.3)
        servo_bus.write_pos(packet, servo_id, start, speed=CALIB_SPEED, acc=CALIB_ACC)
        time.sleep(0.8)
        _check_abort(abort)

        emit(
            on_progress,
            f"Sweeping {name} toward − prior…",
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
        pos = servo_bus.read_pos(packet, servo_id) or start
        emit(
            on_progress,
            f"Stopped while sweeping {name}",
            packet,
            mapping,
            active_joint=name,
            end_reason="abort",
            aborted=True,
        )
        raise

    if hi_reason == "collision":
        warnings.append(f"{name} +: stopped by self-collision (not counted as hard stop)")
    if lo_reason == "collision":
        warnings.append(f"{name} −: stopped by self-collision (not counted as hard stop)")

    # If a side hit collision only, fall back toward prior span around start
    if hi_reason == "collision" and lo_reason == "collision":
        end_hi = prior_hi
        end_lo = prior_lo
        warnings.append(f"{name}: both sides collision-blocked — using URDF prior span")
    elif hi_reason == "collision":
        end_hi = servo_bus.clamp_pos(start + max(abs(start - end_lo), span // 4))
    elif lo_reason == "collision":
        end_lo = servo_bus.clamp_pos(start - max(abs(end_hi - start), span // 4))

    mn = min(end_lo, end_hi) + MARGIN_STEPS
    mx = max(end_lo, end_hi) - MARGIN_STEPS
    if mx <= mn + 20:
        mn = servo_bus.clamp_pos(start - span // 2 + MARGIN_STEPS)
        mx = servo_bus.clamp_pos(start + span // 2 - MARGIN_STEPS)
        warnings.append(f"{name}: narrow measured range — fell back to URDF prior")

    mn = servo_bus.clamp_pos(mn)
    mx = servo_bus.clamp_pos(mx)
    center = (mn + mx) // 2
    servo_bus.write_pos(packet, servo_id, center, speed=CALIB_SPEED, acc=CALIB_ACC)
    time.sleep(0.5)

    emit(
        on_progress,
        f"Done {name}: [{mn},{mx}] hi={hi_reason} lo={lo_reason}",
        packet,
        mapping,
        active_joint=name,
        end_reason=hi_reason,
    )

    return JointRangeResult(
        name=name,
        range_min_steps=mn,
        range_max_steps=mx,
        center_steps=center,
        offset_rad=mid_offset_rad(mn, mx),
        prior_lower_rad=lo_r,
        prior_upper_rad=hi_r,
        dry_run=False,
        hi_reason=hi_reason,
        lo_reason=lo_reason,
        warnings=warnings,
    )


def sweep_all_except_wrist2(
    packet: Any | None,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    *,
    workspace: CollisionWorkspace | None = None,
    abort: threading.Event | None = None,
    on_progress: ProgressCb | None = None,
) -> dict[str, JointRangeResult]:
    priors = load_urdf_joint_limits_rad()
    results: dict[str, JointRangeResult] = {}
    mode = calib_mode()

    start_steps: dict[str, int] = {}
    if packet is not None and mode != "dry_run":
        for name, sid in mapping.items():
            pos = servo_bus.read_pos(packet, sid)
            if pos is not None:
                start_steps[name] = pos
            else:
                start_steps[name] = 2048
        try:
            park_unfold(
                packet, mapping, reverses, abort=abort, on_progress=on_progress
            )
            # Refresh starts after unfold so collision estimate stays consistent
            for name, sid in mapping.items():
                pos = servo_bus.read_pos(packet, sid)
                if pos is not None:
                    start_steps[name] = pos
        except SweepAbort:
            emit(
                on_progress,
                "Sweep aborted during unfold",
                packet,
                mapping,
                aborted=True,
                end_reason="abort",
            )
            raise

    for name in SWEEP_ORDER:
        _check_abort(abort)
        if name not in mapping:
            continue
        results[name] = sweep_joint(
            packet,
            name,
            mapping[name],
            reverses.get(name, False),
            priors,
            mapping=mapping,
            reverses=reverses,
            start_steps=start_steps,
            workspace=workspace,
            abort=abort,
            on_progress=on_progress,
        )
    emit(
        on_progress,
        f"Swept {len(results)} joints (wrist_2 next)",
        packet,
        mapping,
        dry_run=(mode == "dry_run" or packet is None),
    )
    return results
