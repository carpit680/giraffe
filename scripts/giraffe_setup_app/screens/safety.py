"""Mandatory safety screen: torque-off + move joints off limits."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Button, Checkbox, Markdown, Static

from .. import bus as servo_bus
from .base import WizardScreen


class SafetyScreen(WizardScreen):
    step_id = "safety"

    def compose_body(self) -> ComposeResult:
        yield Static("Safety before moving servos", classes="title")
        yield Static(
            "This step will DISABLE MOTOR TORQUE on all discovered servos "
            "(software torque-off). The arm will go limp and may DROP under "
            "gravity — support it with your hands before continuing.",
            classes="danger-box",
        )
        yield Markdown(
            "Why:\n\n"
            "- If servos are holding position, you **cannot** safely move joints by hand.\n"
            "- Torque-off lets you reposition every joint **away from hard stops** "
            "before mapping nudges (~7°).\n"
            "- A joint against a limit during a nudge can strip gears or break parts.\n\n"
            "After torque is off:\n\n"
            "1. Support the arm so it cannot slam down.\n"
            "2. Move **every** joint to mid-travel (away from mechanical limits).\n"
            "3. Keep the area clear; leave 12V power connected (USB + bus stay up).\n"
        )
        yield Checkbox(
            "I understand torque will be disabled and the arm may drop — I will support it",
            value=self.state.torque_off_accepted,
            id="torque-accept",
        )
        yield Static(
            "Torque still enabled — accept the warning, then disable torque."
            if not self.state.torque_disabled
            else "Servo torque is OFF. You can move the joints by hand.",
            id="torque-status",
            classes="hint",
        )
        yield Checkbox(
            "I have moved all joints away from their limits and the area is clear",
            value=self.state.safety_confirmed,
            id="safety-check",
        )

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button(
            "Disable all servo torque",
            id="btn-torque-off",
            variant="warning",
        )
        yield Button("I understand — Next →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        super().on_mount()
        self._refresh_torque_status()

    def on_checkbox_changed(self, event: Checkbox.Changed) -> None:
        if event.checkbox.id == "safety-check":
            self.state.safety_confirmed = event.value
        elif event.checkbox.id == "torque-accept":
            self.state.torque_off_accepted = event.value

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-torque-off":
            self._disable_torque()

    def _refresh_torque_status(self) -> None:
        status = self.query_one("#torque-status", Static)
        if self.state.torque_disabled:
            ids = ", ".join(str(i) for i in self.state.found_ids) or "—"
            status.update(
                f"[ok]Torque OFF[/ok] for IDs: {ids}. "
                "Move every joint to mid-travel, then confirm below."
            )
        else:
            status.update(
                "[warn]Torque still ON.[/warn] Accept the warning, then press "
                "“Disable all servo torque”."
            )

    def _disable_torque(self) -> None:
        try:
            accepted = self.query_one("#torque-accept", Checkbox).value
        except Exception:
            accepted = self.state.torque_off_accepted
        self.state.torque_off_accepted = accepted

        if not accepted:
            self.notify_error(
                "Accept the torque-off / drop warning before disabling servos."
            )
            return
        if self.state.packet is None:
            self.notify_error("Bus not open — go Back to Port / Scan.")
            return
        if not self.state.found_ids:
            self.notify_error("No servo IDs from scan — go Back and re-scan.")
            return

        ok, failed = servo_bus.disable_torque_all(
            self.state.packet, self.state.found_ids
        )
        if failed and not ok:
            self.state.torque_disabled = False
            self.notify_error(
                f"Failed to disable torque on all servos: {failed}"
            )
            self._refresh_torque_status()
            return

        self.state.torque_disabled = True
        if failed:
            self.notify_error(
                f"Torque off for {ok}; failed for {failed}. Support the arm and retry."
            )
        else:
            self.notify_info(
                f"Torque disabled on {len(ok)} servo(s). Support the arm and move joints."
            )
        self._refresh_torque_status()

    def validate_next(self) -> bool:
        try:
            torque_ok = self.query_one("#torque-accept", Checkbox).value
            moved_ok = self.query_one("#safety-check", Checkbox).value
        except Exception:
            torque_ok = self.state.torque_off_accepted
            moved_ok = self.state.safety_confirmed

        self.state.torque_off_accepted = torque_ok
        self.state.safety_confirmed = moved_ok

        if not torque_ok:
            self.notify_error("Accept the torque-off / drop warning first.")
            return False
        if not self.state.torque_disabled:
            self.notify_error(
                "Disable all servo torque before continuing (so you can move joints by hand)."
            )
            return False
        if not moved_ok:
            self.notify_error(
                "Confirm that joints are away from limits before continuing."
            )
            return False
        return True
