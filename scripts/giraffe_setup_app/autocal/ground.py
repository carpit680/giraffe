"""Estimate floor_z in base frame by reaching downward."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from .. import bus as servo_bus
from .stall import StallDetector
from .sweep import (
    CALIB_ACC,
    CALIB_SPEED,
    CALIB_TORQUE_LIMIT,
    SweepAbort,
    _check_abort,
    calib_mode,
    steps_to_joint_rad,
)
from .telemetry import ProgressCb, emit

# Approximate link lengths (m) for a simple planar FK in the lift–elbow plane.
L_SHOULDER = 0.05
L_UPPER = 0.12
L_FOREARM = 0.12
L_WRIST = 0.06
TIP_RADIUS = 0.02


def approximate_ee_z(
    shoulder_lift_rad: float,
    elbow_rad: float,
    wrist1_rad: float = 0.0,
) -> float:
    """Crude sagittal FK: z up from base. Angles are joint-space radians."""
    a1 = shoulder_lift_rad
    a2 = elbow_rad
    a3 = wrist1_rad
    z = (
        L_SHOULDER
        + L_UPPER * math.sin(a1)
        + L_FOREARM * math.sin(a1 + a2)
        + L_WRIST * math.sin(a1 + a2 + a3)
    )
    return float(z)


def probe_ground_height(
    packet: Any | None,
    mapping: dict[str, int],
    reverses: dict[str, bool],
    offsets: dict[str, float],
    *,
    abort: threading.Event | None = None,
    on_progress: ProgressCb | None = None,
) -> float:
    """Lower shoulder_lift / elbow until stall; return estimated floor_z."""
    if calib_mode() == "dry_run" or packet is None:
        emit(
            on_progress,
            "dry_run ground → floor_z=-0.02",
            packet,
            mapping,
            dry_run=True,
        )
        return -0.02

    lift = "shoulder_lift_actuator_shoulder_lift_joint"
    elbow = "elbow_actuator_elbow_joint"
    if lift not in mapping or elbow not in mapping:
        return -0.02

    lift_id = mapping[lift]
    elbow_id = mapping[elbow]
    for sid in (lift_id, elbow_id):
        servo_bus.set_torque(packet, sid, True)
        servo_bus.set_torque_limit(packet, sid, CALIB_TORQUE_LIMIT)
        servo_bus.set_acceleration(packet, sid, CALIB_ACC)

    det = StallDetector(load_threshold=160, hold_cycles=10, pos_epsilon=2)
    t0 = time.time()
    floor_z = -0.02

    while time.time() - t0 < 30.0:
        _check_abort(abort)
        for sid in (lift_id, elbow_id):
            pos = servo_bus.read_pos(packet, sid)
            if pos is None:
                continue
            target = servo_bus.clamp_pos(pos - 25)
            servo_bus.write_pos(packet, sid, target, speed=60, acc=CALIB_ACC)
        time.sleep(0.08)

        pos_l = servo_bus.read_pos(packet, lift_id) or 0
        load_l = servo_bus.read_load(packet, lift_id) or 0
        emit(
            on_progress,
            f"Ground probe  lift={pos_l}  load={load_l}",
            packet,
            mapping,
            active_joint=lift,
        )
        if det.update(pos_l, load_l):
            q_lift = steps_to_joint_rad(
                pos_l, offsets.get(lift, 0.0), reverses.get(lift, False)
            )
            pos_e = servo_bus.read_pos(packet, elbow_id) or 0
            q_elb = steps_to_joint_rad(
                pos_e, offsets.get(elbow, 0.0), reverses.get(elbow, False)
            )
            floor_z = approximate_ee_z(q_lift, q_elb) - TIP_RADIUS
            break

    emit(
        on_progress,
        f"floor_z ≈ {floor_z:.4f} m (base frame)",
        packet,
        mapping,
        active_joint=lift,
    )
    return float(floor_z)
