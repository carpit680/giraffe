"""Auto-calibrate limit sweeps with live telemetry and Stop."""

from __future__ import annotations

import threading

from textual import work
from textual.app import ComposeResult
from textual.widgets import Button, DataTable, Static

from giraffe_control.follower_config import JOINT_LABELS, JOINT_NAMES

from .. import bus as servo_bus
from ..autocal.engine import run_limit_sweeps
from ..autocal.sweep import SweepAbort, calib_mode
from ..autocal.telemetry import SweepProgress, empty_table
from ..collision.workspace import ensure_workspace
from .base import WizardScreen


class AutoCalibrateScreen(WizardScreen):
    """Step 1 of auto-cal: tip-first limit sweeps (wrist_2 is the next screen)."""

    step_id = "auto_calibrate"

    def __init__(self, state) -> None:
        super().__init__(state)
        if not hasattr(state, "auto_calib") or state.auto_calib is None:
            from ..autocal.engine import AutoCalibState

            state.auto_calib = AutoCalibState()
        self._abort: threading.Event | None = None
        self._row_keys: dict[str, object] = {}

    def compose_body(self) -> ComposeResult:
        mode = calib_mode()
        yield Static("Limit sweeps", classes="title")
        yield Static(
            f"Mode: [bold]{mode}[/bold]  (GIRAFFE_CALIB_MODE=dry_run|hw)\n"
            "Low-torque tip-first sweeps with URDF priors, stall detection, and "
            "collision gating. Wrist_2 wire centering is the next screen.",
            classes="subtitle",
        )
        yield Static(
            "Active joint: —",
            id="active-joint",
            classes="hint",
        )
        yield Static("", id="auto-status", classes="hint")
        yield DataTable(id="telem-table", cursor_type="row", zebra_stripes=True)

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        if self.state.edit_mode:
            yield Button("Edit menu", id="btn-edit-hub", variant="default")
        yield Button("Start sweep", id="btn-sweep", variant="primary")
        yield Button("Stop", id="btn-stop", variant="error", disabled=True)
        yield Button("Next →", id="btn-next", variant="success")

    def on_mount(self) -> None:
        super().on_mount()
        table = self.query_one("#telem-table", DataTable)
        table.clear(columns=True)
        table.add_columns(
            ("Joint", "Joint"),
            ("ID", "ID"),
            ("pos", "pos"),
            ("vel", "vel"),
            ("load", "load"),
            ("cur", "cur"),
            ("mov", "mov"),
        )
        self._row_keys.clear()
        for row in empty_table(self.state.mapping):
            key = table.add_row(
                row.label,
                str(row.servo_id),
                "—",
                "—",
                "—",
                "—",
                "—",
                key=row.name,
            )
            self._row_keys[row.name] = key
        self._set_status(
            self.state.auto_calib.message
            or "Build/load collision workspace, then Start sweep."
        )
        try:
            ensure_workspace(
                floor_z=self.state.auto_calib.floor_z or -0.02,
                repo_root=self.state.repo_root,
            )
            self.notify_info("Collision workspace ready")
        except Exception as exc:
            self.notify_error(f"Collision workspace: {exc}")

    def _set_status(self, msg: str) -> None:
        self.query_one("#auto-status", Static).update(msg)

    def _set_active(self, name: str | None) -> None:
        label = JOINT_LABELS.get(name, name) if name else "—"
        self.query_one("#active-joint", Static).update(f"Active joint: [bold]{label}[/bold]")

    def _set_running(self, running: bool) -> None:
        self.query_one("#btn-sweep", Button).disabled = running
        self.query_one("#btn-stop", Button).disabled = not running
        self.query_one("#btn-next", Button).disabled = running
        try:
            self.query_one("#btn-back", Button).disabled = running
        except Exception:
            pass

    def _apply_progress(self, progress: SweepProgress) -> None:
        try:
            self._set_status(progress.message)
            self._set_active(progress.active_joint)
            table = self.query_one("#telem-table", DataTable)
            for joint in progress.joints:
                key = self._row_keys.get(joint.name)
                if key is None:
                    continue
                marker = "▶ " if joint.active else ""
                table.update_cell(key, "Joint", f"{marker}{joint.label}")
                table.update_cell(
                    key, "pos", "—" if joint.pos is None else str(joint.pos)
                )
                table.update_cell(
                    key, "vel", "—" if joint.speed is None else str(joint.speed)
                )
                table.update_cell(
                    key, "load", "—" if joint.load is None else str(joint.load)
                )
                table.update_cell(
                    key, "cur", "—" if joint.current is None else str(joint.current)
                )
                table.update_cell(
                    key, "mov", "—" if joint.moving is None else str(joint.moving)
                )
        except Exception as exc:
            self._set_status(f"{progress.message} (telem UI: {exc})")

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-sweep":
            self._start_sweep()
        elif button_id == "btn-stop":
            self._stop_sweep()

    def _start_sweep(self) -> None:
        if self.state.mapped_count < 6:
            self.notify_error("Map all joints first.")
            return
        if self.state.auto_calib.sweeping:
            return
        self._abort = threading.Event()
        self._set_running(True)
        self._set_status("Starting sweeps…")
        self._sweep_worker()

    def _stop_sweep(self) -> None:
        if self._abort is not None:
            self._abort.set()
            self._set_status("Stop requested — waiting for motion to halt…")
            self.notify_info("Stop requested")

    @work(exclusive=True, thread=True)
    def _sweep_worker(self) -> None:
        abort = self._abort

        def on_progress(progress: SweepProgress) -> None:
            self.app.call_from_thread(self._apply_progress, progress)

        try:
            if self.state.packet and calib_mode() != "dry_run":
                servo_bus.enable_torque_all(
                    self.state.packet, list(self.state.mapping.values())
                )
            ws = None
            try:
                ws = ensure_workspace(
                    floor_z=self.state.auto_calib.floor_z or -0.02,
                    repo_root=self.state.repo_root,
                )
            except Exception:
                ws = None

            self.state.auto_calib = run_limit_sweeps(
                self.state.packet,
                self.state.mapping,
                self.state.reverses,
                self.state.auto_calib,
                workspace=ws,
                abort=abort,
                on_progress=on_progress,
            )
            msg = self.state.auto_calib.message
            warns = self.state.auto_calib.last_warnings
            self.app.call_from_thread(self._on_sweep_done, msg, warns, None)
        except SweepAbort:
            self.app.call_from_thread(
                self._on_sweep_done,
                "Limit sweep stopped by operator",
                [],
                None,
            )
        except Exception as exc:
            self.app.call_from_thread(self._on_sweep_done, None, [], str(exc))

    def _on_sweep_done(
        self, message: str | None, warnings: list[str], error: str | None
    ) -> None:
        self._set_running(False)
        self.state.auto_calib.sweeping = False
        if error:
            self.notify_error(error)
            self._set_status(error)
            return
        if message:
            self._set_status(message)
            self.notify_info(message)
        for w in warnings:
            self.notify_info(w)

    def validate_next(self) -> bool:
        # Need the 5 non-wrist2 joints swept before wrist screen
        needed = [n for n in JOINT_NAMES if n != "wrist_2_actuator_wrist_2_joint"]
        missing = [n for n in needed if n not in self.state.auto_calib.ranges]
        if missing:
            self.notify_error(
                "Run Start sweep and finish all joints before wrist_2 centering."
            )
            return False
        if self.state.auto_calib.sweeping:
            self.notify_error("Wait for the sweep to finish (or press Stop).")
            return False
        return True

    def next_step_id(self) -> str:
        return "wrist2_ground"

    def prev_step_id(self) -> str:
        return "reverses"
