"""Stall detection for low-torque limit sweeps."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class StallDetector:
    """Detect mechanical/wire stall from position plateau + load."""

    pos_epsilon: int = 3
    hold_cycles: int = 8
    load_threshold: int = 180
    current_threshold: int | None = None
    _last_pos: int | None = None
    _stable: int = 0
    history: list[tuple[int, int]] = field(default_factory=list)

    def reset(self) -> None:
        self._last_pos = None
        self._stable = 0
        self.history.clear()

    def update(self, position: int, load: int, current: int | None = None) -> bool:
        self.history.append((position, load))
        if self._last_pos is None:
            self._last_pos = position
            self._stable = 0
            return False

        if abs(position - self._last_pos) <= self.pos_epsilon:
            self._stable += 1
        else:
            self._stable = 0
        self._last_pos = position

        load_ok = load >= self.load_threshold
        current_ok = True
        if self.current_threshold is not None and current is not None:
            current_ok = current >= self.current_threshold

        return self._stable >= self.hold_cycles and load_ok and current_ok
