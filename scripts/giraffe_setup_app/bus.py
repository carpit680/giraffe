"""Servo bus helpers for the Giraffe setup TUI. No EEPROM writes."""

from __future__ import annotations

import time
from typing import Any

import serial.tools.list_ports

from giraffe_control.follower_config import DEFAULT_BAUDRATE

try:
    from STservo_sdk import COMM_SUCCESS, PortHandler, sts
    from STservo_sdk.sts import (
        STS_TORQUE_ENABLE,
        STS_MIN_ANGLE_LIMIT_L,
        STS_MAX_ANGLE_LIMIT_L,
        STS_OFS_L,
        STS_PRESENT_LOAD_L,
        STS_PRESENT_SPEED_L,
        STS_PRESENT_CURRENT_L,
        STS_ACC,
    )
except ImportError:
    sts = None
    PortHandler = None
    COMM_SUCCESS = 0
    STS_TORQUE_ENABLE = 40
    STS_MIN_ANGLE_LIMIT_L = 9
    STS_MAX_ANGLE_LIMIT_L = 11
    STS_OFS_L = 31
    STS_PRESENT_LOAD_L = 60
    STS_PRESENT_SPEED_L = 58
    STS_PRESENT_CURRENT_L = 69
    STS_ACC = 41

# SRAM Torque_Limit (feetech table addr 48)
STS_TORQUE_LIMIT = 48

NUDGE_STEPS = 80  # ~7 degrees at 4096 steps / 360
NUDGE_SPEED = 500
NUDGE_ACC = 30
SMOKE_STEPS = 60
# ~10° positive joint-space test for reverse checking
REVERSE_TEST_RAD = 0.175
SCAN_FAST = range(1, 21)
SCAN_WIDE = range(1, 253)


def list_serial_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


def open_bus(port: str, baudrate: int = DEFAULT_BAUDRATE) -> tuple[Any, Any]:
    if PortHandler is None or sts is None:
        raise RuntimeError("STservo_sdk is not installed")
    port_handler = PortHandler(port)
    if not port_handler.openPort():
        raise RuntimeError(f"Failed to open port {port}")
    if not port_handler.setBaudRate(baudrate):
        port_handler.closePort()
        raise RuntimeError(f"Failed to set baudrate {baudrate} on {port}")
    packet = sts(port_handler)
    return port_handler, packet


def close_bus(port_handler: Any | None) -> None:
    if port_handler is not None:
        try:
            port_handler.closePort()
        except Exception:
            pass


def ping_id(packet: Any, servo_id: int) -> bool:
    _, result, _ = packet.ping(servo_id)
    return result == COMM_SUCCESS


def scan_ids(packet: Any, on_found=None, wide: bool = False) -> list[int]:
    """Scan for servo IDs. Optional on_found(servo_id) callback for UI progress."""
    found: list[int] = []

    def _scan(id_range) -> None:
        for servo_id in id_range:
            if ping_id(packet, servo_id):
                found.append(servo_id)
                if on_found:
                    on_found(servo_id)

    if wide:
        _scan(SCAN_WIDE)
    else:
        _scan(SCAN_FAST)
        if not found:
            _scan(SCAN_WIDE)

    return sorted(set(found))


def read_pos(packet: Any, servo_id: int) -> int | None:
    pos, result, _ = packet.ReadPos(servo_id)
    if result != COMM_SUCCESS:
        return None
    return int(pos)


def write_pos(
    packet: Any,
    servo_id: int,
    position: int,
    speed: int = NUDGE_SPEED,
    acc: int = NUDGE_ACC,
) -> bool:
    result, _ = packet.WritePosEx(servo_id, int(position), speed, acc)
    return result == COMM_SUCCESS


def set_torque(packet: Any, servo_id: int, enable: bool) -> bool:
    """Enable (1) or disable (0) motor torque. Does not touch EEPROM."""
    value = 1 if enable else 0
    result, _ = packet.write1ByteTxRx(servo_id, STS_TORQUE_ENABLE, value)
    return result == COMM_SUCCESS


def disable_torque_all(
    packet: Any, servo_ids: list[int]
) -> tuple[list[int], list[int]]:
    """Force torque off on each ID. Returns (ok_ids, failed_ids)."""
    ok: list[int] = []
    failed: list[int] = []
    for servo_id in servo_ids:
        if set_torque(packet, servo_id, False):
            ok.append(servo_id)
        else:
            failed.append(servo_id)
    return ok, failed


def enable_torque_all(
    packet: Any, servo_ids: list[int]
) -> tuple[list[int], list[int]]:
    ok: list[int] = []
    failed: list[int] = []
    for servo_id in servo_ids:
        if set_torque(packet, servo_id, True):
            ok.append(servo_id)
        else:
            failed.append(servo_id)
    return ok, failed


def clamp_pos(pos: int) -> int:
    return max(0, min(4095, int(pos)))


def radians_to_steps(radians: float, resolution: int = 4096) -> int:
    degrees = radians * (180.0 / 3.141592653589793)
    return int(degrees / 360.0 * resolution)


def nudge_joint_positive(
    packet: Any,
    servo_id: int,
    reverse: bool,
    delta_rad: float = REVERSE_TEST_RAD,
    hold_s: float = 1.0,
) -> tuple[bool, str]:
    """Nudge in the *positive joint-space* direction used by giraffe_driver.

    Driver mapping for command q:
      q' = -q if reverse else q
      steps ∝ -q'  (+ offset)
    So a positive joint command (+delta) changes motor steps by:
      (+steps) if reverse else (-steps)
    """
    set_torque(packet, servo_id, True)
    start = read_pos(packet, servo_id)
    if start is None:
        return False, f"Failed to read position for ID {servo_id}"

    step_delta = radians_to_steps(delta_rad)
    signed = step_delta if reverse else -step_delta
    target = clamp_pos(start + signed)
    if target == start:
        # Near end of travel — try the other way within clamp for visibility
        target = clamp_pos(start - signed)
        if target == start:
            return False, f"ID {servo_id} cannot move further (at travel limit?)"

    if not write_pos(packet, servo_id, target):
        write_pos(packet, servo_id, start)
        return False, f"Test move write failed for ID {servo_id}"
    time.sleep(hold_s)
    write_pos(packet, servo_id, start)
    time.sleep(0.4)
    return (
        True,
        f"Positive joint nudge on ID {servo_id} "
        f"({'reverse' if reverse else 'normal'}; steps {start}→{target}→restored)",
    )


def nudge_servo(
    packet: Any,
    servo_id: int,
    steps: int = NUDGE_STEPS,
    hold_s: float = 0.8,
) -> tuple[bool, str]:
    """Enable torque, move briefly, then restore. Returns (ok, message)."""
    set_torque(packet, servo_id, True)
    start = read_pos(packet, servo_id)
    if start is None:
        return False, f"Failed to read position for ID {servo_id}"
    target = clamp_pos(start + steps)
    if not write_pos(packet, servo_id, target):
        write_pos(packet, servo_id, start)
        return False, f"Nudge write failed for ID {servo_id}"
    time.sleep(hold_s)
    write_pos(packet, servo_id, start)
    time.sleep(0.4)
    return True, f"Nudged ID {servo_id} ({start} → {target} → restored)"


def capture_offsets(packet: Any, mapping: dict[str, int]) -> dict[str, float]:
    resolution = 4096
    offsets: dict[str, float] = {}
    for name, servo_id in mapping.items():
        pos = read_pos(packet, servo_id)
        if pos is None:
            offsets[name] = 0.0
            continue
        offsets[name] = float((pos / resolution) * 2 * 3.141592653589793)
    return offsets


def _decode_load(raw: int) -> int:
    """Feetech load is often magnitude with direction bit; return absolute load 0–1000-ish."""
    raw = int(raw) & 0xFFFF
    direction = (raw >> 10) & 0x1
    magnitude = raw & 0x3FF
    return int(magnitude)


def read_load(packet: Any, servo_id: int) -> int | None:
    raw, result, _ = packet.read2ByteTxRx(servo_id, STS_PRESENT_LOAD_L)
    if result != COMM_SUCCESS:
        return None
    return _decode_load(raw)


def read_current(packet: Any, servo_id: int) -> int | None:
    raw, result, _ = packet.read2ByteTxRx(servo_id, STS_PRESENT_CURRENT_L)
    if result != COMM_SUCCESS:
        return None
    return int(raw) & 0xFFFF


def read_speed(packet: Any, servo_id: int) -> int | None:
    """Present speed (signed steps/s-ish). Prefer SDK ReadSpeed when available."""
    if hasattr(packet, "ReadSpeed"):
        speed, result, _ = packet.ReadSpeed(servo_id)
        if result != COMM_SUCCESS:
            return None
        return int(speed)
    raw, result, _ = packet.read2ByteTxRx(servo_id, STS_PRESENT_SPEED_L)
    if result != COMM_SUCCESS:
        return None
    if hasattr(packet, "sts_tohost"):
        return int(packet.sts_tohost(raw, 15))
    # Fallback: treat as signed 16-bit
    raw = int(raw) & 0xFFFF
    return raw - 0x10000 if raw & 0x8000 else raw


def read_moving(packet: Any, servo_id: int) -> int | None:
    if hasattr(packet, "ReadMoving"):
        moving, result, _ = packet.ReadMoving(servo_id)
        if result != COMM_SUCCESS:
            return None
        return int(moving)
    return None


def set_torque_limit(packet: Any, servo_id: int, limit: int) -> bool:
    """SRAM torque limit 0–1000 typical. Does not touch EEPROM."""
    limit = max(0, min(1000, int(limit)))
    result, _ = packet.write2ByteTxRx(servo_id, STS_TORQUE_LIMIT, limit)
    return result == COMM_SUCCESS


def set_acceleration(packet: Any, servo_id: int, acc: int) -> bool:
    result, _ = packet.write1ByteTxRx(servo_id, STS_ACC, max(0, min(254, int(acc))))
    return result == COMM_SUCCESS


def read_angle_limits(packet: Any, servo_id: int) -> tuple[int | None, int | None]:
    mn, r1, _ = packet.read2ByteTxRx(servo_id, STS_MIN_ANGLE_LIMIT_L)
    mx, r2, _ = packet.read2ByteTxRx(servo_id, STS_MAX_ANGLE_LIMIT_L)
    if r1 != COMM_SUCCESS or r2 != COMM_SUCCESS:
        return None, None
    return int(mn), int(mx)


def read_homing_offset(packet: Any, servo_id: int) -> int | None:
    raw, result, _ = packet.read2ByteTxRx(servo_id, STS_OFS_L)
    if result != COMM_SUCCESS:
        return None
    # Signed 12-bit-ish offset depending on firmware; treat as uint16 host decode
    return int(packet.sts_tohost(raw, 11)) if hasattr(packet, "sts_tohost") else int(raw)


def write_angle_limits(packet: Any, servo_id: int, min_steps: int, max_steps: int) -> bool:
    packet.unLockEprom(servo_id)
    r1, _ = packet.write2ByteTxRx(servo_id, STS_MIN_ANGLE_LIMIT_L, int(min_steps))
    r2, _ = packet.write2ByteTxRx(servo_id, STS_MAX_ANGLE_LIMIT_L, int(max_steps))
    packet.LockEprom(servo_id)
    return r1 == COMM_SUCCESS and r2 == COMM_SUCCESS


def write_homing_offset(packet: Any, servo_id: int, offset_steps: int) -> bool:
    """Write EEPROM position offset (STS OFS)."""
    packet.unLockEprom(servo_id)
    # Store as unsigned 16-bit compatible with SDK write
    value = int(offset_steps) & 0xFFFF
    result, _ = packet.write2ByteTxRx(servo_id, STS_OFS_L, value)
    packet.LockEprom(servo_id)
    return result == COMM_SUCCESS
