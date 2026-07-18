"""Per-joint smoke test with redo."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.binding import Binding
from textual.widgets import Button, Static

from giraffe_control.follower_config import JOINT_LABELS, JOINT_NAMES

from .. import bus as servo_bus
from .base import WizardScreen


class SmokeTestScreen(WizardScreen):
    step_id = "smoke_test"

    BINDINGS = WizardScreen.BINDINGS + [
        Binding("n", "nudge_current", "Nudge", show=True),
        Binding("y", "mark_ok", "OK", show=True),
        Binding("d", "redo_joint", "Redo map", show=True),
    ]

    def compose_body(self) -> ComposeResult:
        yield Static("Smoke test", classes="title")
        yield Static(
            "Nudge each joint and confirm the correct link moves. "
            "If wrong, Redo map for that joint.",
            classes="subtitle",
        )
        yield Static("", id="smoke-status")
        yield Static("", id="smoke-hint", classes="hint")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button("Nudge", id="btn-nudge", variant="primary")
        yield Button("Looks good", id="btn-ok", variant="success")
        yield Button("Redo this joint", id="btn-redo", variant="warning")
        yield Button("Skip test →", id="btn-next", variant="default")

    def on_mount(self) -> None:
        super().on_mount()
        if self.state.smoke_index >= len(JOINT_NAMES):
            self.state.smoke_index = 0
        self._update_labels()

    def _current_joint(self) -> str | None:
        if self.state.smoke_index >= len(JOINT_NAMES):
            return None
        return JOINT_NAMES[self.state.smoke_index]

    def _update_labels(self) -> None:
        joint = self._current_joint()
        status = self.query_one("#smoke-status", Static)
        hint = self.query_one("#smoke-hint", Static)
        if joint is None:
            status.update("All joints checked. Next to finish.")
            hint.update("")
            return
        sid = self.state.mapping.get(joint)
        status.update(
            f"Joint {self.state.smoke_index + 1}/6: "
            f"[bold]{JOINT_LABELS[joint]}[/bold]  (servo ID {sid})"
        )
        hint.update("Nudge · Looks good · Redo this joint · or Skip test")

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-nudge":
            self.action_nudge_current()
        elif button_id == "btn-ok":
            self.action_mark_ok()
        elif button_id == "btn-redo":
            self.action_redo_joint()

    def action_nudge_current(self) -> None:
        joint = self._current_joint()
        if joint is None or self.state.packet is None:
            return
        sid = self.state.mapping[joint]
        ok, msg = servo_bus.nudge_servo(
            self.state.packet, sid, steps=servo_bus.SMOKE_STEPS, hold_s=0.7
        )
        if ok:
            self.notify_info(msg)
        else:
            self.notify_error(msg)

    def action_mark_ok(self) -> None:
        if self._current_joint() is None:
            self.app.goto_step("done")
            return
        self.state.smoke_index += 1
        if self.state.smoke_index >= len(JOINT_NAMES):
            self.notify_info("Smoke test complete")
            self.app.goto_step("done")
            return
        self._update_labels()

    def action_redo_joint(self) -> None:
        joint = self._current_joint()
        if joint is None:
            return
        sid = self.state.unmap_joint(joint)
        self.state.redo_servo_id = sid
        self.app.goto_step("map_joints")

    def validate_next(self) -> bool:
        return True

    def next_step_id(self) -> str:
        return "done"
