"""Optional zero / home-pose calibration with torque-off + ASCII guidance."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Button, Checkbox, Static

from giraffe_control.follower_config import JOINT_LABELS, JOINT_NAMES

from .. import bus as servo_bus
from ..poses import HOME_POSE_ART, HOME_POSE_CHECKLIST
from .base import WizardScreen


class CalibrateScreen(WizardScreen):
    step_id = "calibrate"

    def compose_body(self) -> ComposeResult:
        yield Static("Zero calibration (optional)", classes="title")
        yield Static(
            "Before posing the arm we DISABLE SERVO TORQUE again. "
            "The arm will go limp and may DROP — support it with your hands.",
            classes="danger-box",
        )
        yield Checkbox(
            "I understand torque will be disabled and the arm may drop — I will support it",
            value=self.state.calibrate_torque_accepted,
            id="cal-torque-accept",
        )
        yield Static("", id="cal-torque-status", classes="hint")
        yield Static("Home / zero pose guidance", classes="title")
        yield Static(HOME_POSE_ART.strip("\n"), id="pose-art", classes="pose-art")
        checklist = "\n".join(f"  • {line}" for line in HOME_POSE_CHECKLIST)
        yield Static(checklist, id="pose-checklist", classes="subtitle")
        yield Checkbox(
            "Arm is in the home / zero pose shown above (after torque-off)",
            value=False,
            id="cal-pose-ready",
        )
        yield Static("Offsets not captured yet.", id="cal-status")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button(
            "Disable torque to pose",
            id="btn-torque-off",
            variant="warning",
        )
        yield Button("Capture offsets", id="btn-capture", variant="primary")
        yield Button("Skip (zeros)", id="btn-skip", variant="default")
        yield Button("Next →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        super().on_mount()
        # Reverse / smoke tests may have re-enabled torque — require a fresh disable.
        self.state.calibrate_torque_disabled = False
        self._refresh_torque_status()
        if any(v != 0.0 for v in self.state.offsets.values()):
            self._show_offsets()

    def on_checkbox_changed(self, event: Checkbox.Changed) -> None:
        if event.checkbox.id == "cal-torque-accept":
            self.state.calibrate_torque_accepted = event.value

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-torque-off":
            self._disable_torque()
        elif button_id == "btn-capture":
            self._capture()
        elif button_id == "btn-skip":
            self.state.offsets = {name: 0.0 for name in JOINT_NAMES}
            self.query_one("#cal-status", Static).update(
                "Using zero offsets for all joints (skipped pose capture)."
            )
            self.notify_info("Offsets cleared to 0.0")

    def _refresh_torque_status(self) -> None:
        status = self.query_one("#cal-torque-status", Static)
        if self.state.calibrate_torque_disabled:
            status.update(
                "[ok]Torque OFF.[/ok] Support the arm, match the home pose diagram, "
                "check the pose box, then Capture."
            )
        else:
            status.update(
                "[warn]Torque may still be ON.[/warn] Accept the warning, then "
                "“Disable torque to pose” before moving the arm by hand."
            )

    def _disable_torque(self) -> None:
        try:
            accepted = self.query_one("#cal-torque-accept", Checkbox).value
        except Exception:
            accepted = self.state.calibrate_torque_accepted
        self.state.calibrate_torque_accepted = accepted

        if not accepted:
            self.notify_error(
                "Accept the torque-off / drop warning before disabling servos."
            )
            return
        if self.state.packet is None:
            self.notify_error("Bus not open.")
            return

        ids = list(self.state.mapping.values()) or list(self.state.found_ids)
        if not ids:
            self.notify_error("No mapped servos to disable.")
            return

        ok, failed = servo_bus.disable_torque_all(self.state.packet, ids)
        if not ok:
            self.state.calibrate_torque_disabled = False
            self.notify_error(f"Failed to disable torque: {failed}")
            self._refresh_torque_status()
            return

        self.state.calibrate_torque_disabled = True
        self.state.torque_disabled = True
        if failed:
            self.notify_error(f"Torque off for {ok}; failed for {failed}.")
        else:
            self.notify_info(
                f"Torque disabled on {len(ok)} servo(s). Pose the arm using the diagram."
            )
        self._refresh_torque_status()

    def _capture(self) -> None:
        if not self.state.calibrate_torque_accepted:
            self.notify_error("Accept the torque-off warning first.")
            return
        if not self.state.calibrate_torque_disabled:
            self.notify_error("Disable torque before posing and capturing.")
            return
        try:
            posed = self.query_one("#cal-pose-ready", Checkbox).value
        except Exception:
            posed = False
        if not posed:
            self.notify_error(
                "Confirm the arm is in the home / zero pose shown in the diagram."
            )
            return
        if self.state.packet is None:
            self.notify_error("Bus not open.")
            return
        self.state.offsets = servo_bus.capture_offsets(
            self.state.packet, self.state.mapping
        )
        self._show_offsets()
        self.notify_info("Offsets captured from current home pose")

    def _show_offsets(self) -> None:
        lines = [
            f"  {JOINT_LABELS[n]}: {self.state.offsets[n]:.4f} rad"
            for n in JOINT_NAMES
        ]
        self.query_one("#cal-status", Static).update(
            "Captured offsets:\n" + "\n".join(lines)
        )
