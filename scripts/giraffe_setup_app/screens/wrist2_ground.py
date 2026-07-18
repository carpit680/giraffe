"""Wrist_2 centering + soft-end probe + ground height (after limit sweeps)."""

from __future__ import annotations

import threading

from textual import work
from textual.app import ComposeResult
from textual.widgets import Button, Checkbox, DataTable, Static

from giraffe_control.follower_config import WRIST_2_JOINT

from .. import bus as servo_bus
from ..autocal.engine import (
    AutoCalibState,
    apply_wrist2_center,
    run_ground_probe,
    run_wrist2_probe,
)
from ..autocal.sweep import SweepAbort, calib_mode
from ..autocal.telemetry import SweepProgress, empty_table
from ..collision.workspace import ensure_workspace
from .base import WizardScreen


class Wrist2GroundScreen(WizardScreen):
    step_id = "wrist2_ground"

    def __init__(self, state) -> None:
        super().__init__(state)
        if not hasattr(state, "auto_calib") or state.auto_calib is None:
            state.auto_calib = AutoCalibState()
        self._abort: threading.Event | None = None
        self._row_keys: dict[str, object] = {}

    def compose_body(self) -> ComposeResult:
        mode = calib_mode()
        yield Static("Wrist_2 + ground height", classes="title")
        yield Static(
            f"Mode: [bold]{mode}[/bold]\n"
            "Center wrist_2 by hand (wire mid-travel), probe soft ends, then "
            "estimate floor_z with a careful reach-down.",
            classes="subtitle",
        )
        yield Static("", id="wg-status", classes="hint")
        yield DataTable(id="wg-telem", cursor_type="row", zebra_stripes=True)

        yield Static(
            "Before rotating wrist_2 by hand we DISABLE SERVO TORQUE on all joints. "
            "The arm will go limp and may DROP — support it with your hands.",
            classes="danger-box",
        )
        yield Checkbox(
            "I understand torque will be disabled and the arm may drop — I will support it",
            id="wrist2-torque-accept",
            value=False,
        )
        yield Static(
            "[warn]Torque still ON.[/warn] Accept the warning, then disable torque "
            "before rotating wrist_2.",
            id="wrist2-torque-status",
            classes="hint",
        )
        yield Static(
            "Rotate gripper roll to mid wire travel (equal slack both ways).",
            id="wrist2-art",
            classes="hint",
        )
        yield Checkbox(
            "Wrist_2 is centered (mid wire travel) — ready to record",
            id="wrist2-ready",
            value=False,
        )

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        if self.state.edit_mode:
            yield Button("Edit menu", id="btn-edit-hub", variant="default")
        yield Button(
            "1a) Disable torque",
            id="btn-wrist-torque-off",
            variant="warning",
        )
        yield Button(
            "1b) Record wrist_2 center",
            id="btn-wrist-center",
            variant="default",
        )
        yield Button("2) Probe wrist_2 ends", id="btn-wrist-probe", variant="primary")
        yield Button("3) Probe ground", id="btn-ground", variant="primary")
        yield Button("Stop", id="btn-stop", variant="error", disabled=True)
        yield Button("Next →", id="btn-next", variant="success")

    def on_mount(self) -> None:
        super().on_mount()
        self.state.auto_calib.wrist2_torque_disabled = False
        self._refresh_wrist2_torque_status()
        table = self.query_one("#wg-telem", DataTable)
        table.clear(columns=True)
        table.add_columns(
            ("Joint", "Joint"),
            ("ID", "ID"),
            ("pos", "pos"),
            ("vel", "vel"),
            ("load", "load"),
            ("cur", "cur"),
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
                key=row.name,
            )
            self._row_keys[row.name] = key
        self._set_status(
            self.state.auto_calib.message
            or "Accept torque warning → disable → center wrist_2 → probe → ground."
        )

    def on_checkbox_changed(self, event: Checkbox.Changed) -> None:
        if event.checkbox.id == "wrist2-torque-accept":
            self.state.auto_calib.wrist2_torque_accepted = event.value

    def _set_status(self, msg: str) -> None:
        self.query_one("#wg-status", Static).update(msg)

    def _set_running(self, running: bool) -> None:
        for bid in (
            "btn-wrist-torque-off",
            "btn-wrist-center",
            "btn-wrist-probe",
            "btn-ground",
            "btn-next",
            "btn-back",
        ):
            try:
                self.query_one(f"#{bid}", Button).disabled = running
            except Exception:
                pass
        self.query_one("#btn-stop", Button).disabled = not running

    def _apply_progress(self, progress: SweepProgress) -> None:
        try:
            self._set_status(progress.message)
            table = self.query_one("#wg-telem", DataTable)
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
        except Exception as exc:
            self._set_status(f"{progress.message} (telem UI: {exc})")

    def _refresh_wrist2_torque_status(self) -> None:
        status = self.query_one("#wrist2-torque-status", Static)
        art = self.query_one("#wrist2-art", Static)
        if self.state.auto_calib.wrist2_torque_disabled or calib_mode() == "dry_run":
            status.update(
                "[ok]Torque OFF.[/ok] Support the arm, rotate wrist_2 to mid-wire center, "
                "then check “ready to record” and press 1b."
            )
            art.update(
                "Torque is OFF — arm is limp. Center mid wire travel, then record."
            )
        else:
            status.update(
                "[warn]Torque still ON.[/warn] Accept the drop warning, then press "
                "“1a) Disable torque” before rotating wrist_2 by hand."
            )
            art.update("Do not rotate yet — disable torque first.")

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-wrist-torque-off":
            self._disable_torque_for_wrist2()
        elif button_id == "btn-wrist-center":
            self._run_wrist_center()
        elif button_id == "btn-wrist-probe":
            self._start_wrist_probe()
        elif button_id == "btn-ground":
            self._start_ground()
        elif button_id == "btn-stop":
            if self._abort is not None:
                self._abort.set()
                self._set_status("Stop requested…")
                self.notify_info("Stop requested")

    def _disable_torque_for_wrist2(self) -> None:
        try:
            accepted = self.query_one("#wrist2-torque-accept", Checkbox).value
        except Exception:
            accepted = self.state.auto_calib.wrist2_torque_accepted
        self.state.auto_calib.wrist2_torque_accepted = accepted

        if not accepted and calib_mode() != "dry_run":
            self.notify_error(
                "Accept the torque-off / drop warning before disabling servos."
            )
            return

        if calib_mode() == "dry_run":
            self.state.auto_calib.wrist2_torque_disabled = True
            self._refresh_wrist2_torque_status()
            self.notify_info("dry_run: treating torque as OFF")
            return

        if self.state.packet is None:
            self.notify_error("Bus not open — go Back to Port / Scan.")
            return

        ids = list(self.state.mapping.values())
        ok, failed = servo_bus.disable_torque_all(self.state.packet, ids)
        if not ok:
            self.state.auto_calib.wrist2_torque_disabled = False
            self.notify_error(f"Failed to disable torque: {failed}")
            self._refresh_wrist2_torque_status()
            return

        self.state.auto_calib.wrist2_torque_disabled = True
        if failed:
            self.notify_error(
                f"Torque off for {ok}; failed for {failed}. Support the arm and retry."
            )
        else:
            self.notify_info(
                f"Torque disabled on {len(ok)} servo(s). Support the arm, then center wrist_2."
            )
        self._refresh_wrist2_torque_status()

    def _run_wrist_center(self) -> None:
        if (
            not self.state.auto_calib.wrist2_torque_disabled
            and calib_mode() != "dry_run"
        ):
            self.notify_error(
                "Disable torque first (1a) so you can rotate wrist_2 by hand safely."
            )
            return
        try:
            ready = self.query_one("#wrist2-ready", Checkbox).value
        except Exception:
            ready = False
        if not ready and calib_mode() != "dry_run":
            self.notify_error("Center wrist_2 by hand, then check the ready box.")
            return

        if self.state.packet and calib_mode() != "dry_run":
            servo_bus.disable_torque_all(
                self.state.packet, list(self.state.mapping.values())
            )

        try:
            self.state.auto_calib = apply_wrist2_center(
                self.state.packet, self.state.mapping, self.state.auto_calib
            )
            self._set_status(self.state.auto_calib.message)
            self.notify_info(self.state.auto_calib.message)
        except Exception as exc:
            self.notify_error(str(exc))

    def _start_wrist_probe(self) -> None:
        if self.state.auto_calib.wrist2_center is None:
            self.notify_error("Record wrist_2 center (1b) first.")
            return
        if self.state.auto_calib.sweeping:
            return
        self._abort = threading.Event()
        self._set_running(True)
        self._wrist_probe_worker()

    def _start_ground(self) -> None:
        if WRIST_2_JOINT not in self.state.auto_calib.ranges:
            self.notify_error("Probe wrist_2 ends (step 2) first.")
            return
        if self.state.auto_calib.sweeping:
            return
        self._abort = threading.Event()
        self._set_running(True)
        self._ground_worker()

    @work(exclusive=True, thread=True)
    def _wrist_probe_worker(self) -> None:
        abort = self._abort

        def on_progress(progress: SweepProgress) -> None:
            self.app.call_from_thread(self._apply_progress, progress)

        try:
            ws = None
            try:
                ws = ensure_workspace(
                    floor_z=self.state.auto_calib.floor_z or -0.02,
                    repo_root=self.state.repo_root,
                )
            except Exception:
                ws = None
            self.state.auto_calib = run_wrist2_probe(
                self.state.packet,
                self.state.mapping,
                self.state.reverses,
                self.state.auto_calib,
                workspace=ws,
                abort=abort,
                on_progress=on_progress,
            )
            self.state.auto_calib.wrist2_torque_disabled = False
            msg = self.state.auto_calib.message
            self.app.call_from_thread(
                lambda: self._on_motion_done(msg, None, refresh_torque=True)
            )
        except SweepAbort:
            self.app.call_from_thread(
                lambda: self._on_motion_done(
                    "Wrist_2 probe stopped", None, refresh_torque=True
                )
            )
        except Exception as exc:
            err = str(exc)
            self.app.call_from_thread(
                lambda: self._on_motion_done(None, err, refresh_torque=True)
            )

    @work(exclusive=True, thread=True)
    def _ground_worker(self) -> None:
        abort = self._abort

        def on_progress(progress: SweepProgress) -> None:
            self.app.call_from_thread(self._apply_progress, progress)

        try:
            self.state.auto_calib = run_ground_probe(
                self.state.packet,
                self.state.mapping,
                self.state.reverses,
                self.state.auto_calib,
                abort=abort,
                on_progress=on_progress,
            )
            if self.state.auto_calib.floor_z is not None:
                ensure_workspace(
                    floor_z=self.state.auto_calib.floor_z,
                    repo_root=self.state.repo_root,
                    rebuild=True,
                )
            self.app.call_from_thread(
                self._on_motion_done, self.state.auto_calib.message, None
            )
        except SweepAbort:
            self.app.call_from_thread(
                self._on_motion_done, "Ground probe stopped", None
            )
        except Exception as exc:
            self.app.call_from_thread(self._on_motion_done, None, str(exc))

    def _on_motion_done(
        self,
        message: str | None,
        error: str | None,
        *,
        refresh_torque: bool = False,
    ) -> None:
        self._set_running(False)
        self.state.auto_calib.sweeping = False
        if refresh_torque:
            self._refresh_wrist2_torque_status()
        if error:
            self.notify_error(error)
            self._set_status(error)
            return
        if message:
            self._set_status(message)
            self.notify_info(message)

    def validate_next(self) -> bool:
        cal = self.state.auto_calib
        if WRIST_2_JOINT not in cal.ranges:
            self.notify_error("Complete wrist_2 probe first.")
            return False
        if cal.floor_z is None:
            self.notify_error("Run ground probe (step 3).")
            return False
        if cal.sweeping:
            self.notify_error("Wait for motion to finish (or press Stop).")
            return False
        for name, r in cal.ranges.items():
            self.state.offsets[name] = r.offset_rad
        return True

    def next_step_id(self) -> str:
        return "calib_confirm"

    def prev_step_id(self) -> str:
        return "auto_calibrate"
