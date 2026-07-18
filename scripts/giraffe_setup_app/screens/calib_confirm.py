"""Review auto-calibration results; write YAML + optional EEPROM."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Button, Checkbox, DataTable, Static

from giraffe_control.follower_config import (
    JOINT_LABELS,
    JOINT_NAMES,
    default_config_path,
    save_follower_config,
)

from ..autocal.eeprom import write_all_eeprom
from ..autocal.engine import build_follower_config
from ..autocal.sweep import calib_mode
from .base import WizardScreen


class CalibConfirmScreen(WizardScreen):
    step_id = "calib_confirm"

    def compose_body(self) -> ComposeResult:
        yield Static("Confirm calibration → config + EEPROM", classes="title")
        yield Static(
            "Review measured soft limits and zeros. "
            "EEPROM write updates Feetech Min/Max angle + Offset after you accept.",
            classes="subtitle",
        )
        yield DataTable(id="calib-table", cursor_type="row")
        floor = (
            self.state.auto_calib.floor_z
            if getattr(self.state, "auto_calib", None)
            else None
        )
        warns = []
        cal = getattr(self.state, "auto_calib", None)
        if cal is not None:
            warns = list(getattr(cal, "last_warnings", None) or [])
        meta = (
            f"floor_z: {floor if floor is not None else '—'} m   ·   mode: {calib_mode()}"
        )
        if warns:
            meta += "\nWarnings: " + "; ".join(warns)
        yield Static(meta, id="calib-meta", classes="hint")
        yield Checkbox(
            "I reviewed the table and accept writing software config + Feetech EEPROM",
            id="eeprom-accept",
            value=False,
        )
        yield Static("", id="calib-write-status", classes="hint")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        if self.state.edit_mode:
            yield Button("Edit menu", id="btn-edit-hub", variant="default")
        yield Button("Write YAML + EEPROM", id="btn-write", variant="warning")
        yield Button("Next →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        super().on_mount()
        table = self.query_one("#calib-table", DataTable)
        table.clear(columns=True)
        table.add_columns("Joint", "min", "max", "center", "offset", "+end", "−end")
        cal = getattr(self.state, "auto_calib", None)
        if cal is None:
            return
        for name in JOINT_NAMES:
            r = cal.ranges.get(name)
            if r is None:
                table.add_row(JOINT_LABELS[name], "—", "—", "—", "—", "—", "—")
            else:
                table.add_row(
                    JOINT_LABELS[name],
                    str(r.range_min_steps),
                    str(r.range_max_steps),
                    str(r.center_steps),
                    f"{r.offset_rad:.4f}",
                    getattr(r, "hi_reason", "—"),
                    getattr(r, "lo_reason", "—"),
                )

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-write":
            self._write()

    def _write(self) -> bool:
        try:
            accepted = self.query_one("#eeprom-accept", Checkbox).value
        except Exception:
            accepted = False
        if not accepted:
            self.notify_error("Accept the EEPROM / config write checkbox first.")
            return False
        cal = self.state.auto_calib
        if cal is None or len(cal.ranges) < 6:
            self.notify_error("Incomplete calibration data.")
            return False
        if not self.state.port:
            self.notify_error("No port in state.")
            return False

        cfg = build_follower_config(
            self.state.port,
            self.state.baudrate,
            self.state.mapping,
            self.state.reverses,
            cal,
        )
        eeprom_results = write_all_eeprom(
            self.state.packet, self.state.mapping, cal.ranges
        )
        failed = [r for r in eeprom_results if not r.ok]
        if failed and calib_mode() != "dry_run":
            detail = "; ".join(f"{r.name}:{r.detail}" for r in failed)
            self.notify_error(f"EEPROM write aborted: {detail}")
            self.query_one("#calib-write-status", Static).update(
                f"EEPROM failed — YAML not marked written.\n{detail}"
            )
            return False

        cfg.calibration.eeprom_written = True
        path = save_follower_config(cfg, default_config_path(self.state.repo_root))
        self.state.config_path = path
        for name, motor in cfg.motors.items():
            self.state.offsets[name] = motor.offset
        self.query_one("#calib-write-status", Static).update(
            f"Wrote {path}\nEEPROM: "
            + ", ".join(f"{r.name}={'ok' if r.ok else 'fail'}" for r in eeprom_results)
        )
        self.notify_info(f"Saved {path}")
        return True

    def validate_next(self) -> bool:
        if self.state.config_path is None:
            return self._write()
        return True

    def next_step_id(self) -> str:
        return "write_confirm"

    def prev_step_id(self) -> str:
        return "wrist2_ground"
