"""Review and redo joint mapping."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.binding import Binding
from textual.widgets import Button, DataTable, Static

from giraffe_control.follower_config import JOINT_LABELS, JOINT_NAMES

from .base import ConfirmModal, WizardScreen


class ReviewMapScreen(WizardScreen):
    step_id = "review_map"

    BINDINGS = WizardScreen.BINDINGS + [
        Binding("d", "redo_one", "Redo joint", show=True),
        Binding("a", "redo_all", "Redo all", show=True),
    ]

    def compose_body(self) -> ComposeResult:
        yield Static("Review joint ↔ servo mapping", classes="title")
        yield Static(
            "Select a row and Redo joint to remapping that servo. "
            "You can also Redo all and map again.",
            classes="subtitle",
        )
        yield DataTable(id="map-table", cursor_type="row")
        yield Static("", id="review-hint", classes="hint")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button("Redo selected", id="btn-redo", variant="warning")
        yield Button("Redo all", id="btn-redo-all", variant="default")
        yield Button("Next →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        super().on_mount()
        table = self.query_one("#map-table", DataTable)
        table.clear(columns=True)
        table.add_columns("Joint", "Servo ID", "Status")
        for name in JOINT_NAMES:
            sid = self.state.mapping.get(name)
            status = "ok" if sid is not None else "MISSING"
            table.add_row(
                JOINT_LABELS[name],
                str(sid) if sid is not None else "—",
                status,
                key=name,
            )
        hint = self.query_one("#review-hint", Static)
        if self.state.mapped_count < 6:
            hint.update(
                f"Only {self.state.mapped_count}/6 mapped. "
                "Go Back to Map joints or Redo to finish."
            )
        else:
            hint.update("All 6 joints mapped. Next continues to reverse flags.")
        self.refresh_status()

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-redo":
            self.action_redo_one()
        elif button_id == "btn-redo-all":
            self.action_redo_all()

    def _selected_joint(self) -> str | None:
        table = self.query_one("#map-table", DataTable)
        row = table.cursor_row
        if row is None or row < 0 or row >= len(JOINT_NAMES):
            return None
        return JOINT_NAMES[row]

    def action_redo_one(self) -> None:
        joint = self._selected_joint()
        if joint is None:
            self.notify_error("Select a joint row first.")
            return
        sid = self.state.unmap_joint(joint)
        if sid is None:
            # Not mapped yet — go map any unmapped
            self.state.redo_servo_id = None
        else:
            self.state.redo_servo_id = sid
        self.app.goto_step("map_joints")

    def action_redo_all(self) -> None:
        def _done(confirmed: bool | None) -> None:
            if confirmed:
                self.state.unmap_all()
                self.app.goto_step("map_joints")

        self.app.push_screen(
            ConfirmModal("Clear all joint mappings and start over?", title="Redo all"),
            _done,
        )

    def validate_next(self) -> bool:
        if self.state.mapped_count < 6:
            self.notify_error("Map all 6 joints before continuing.")
            return False
        # Ensure IDs are unique
        ids = list(self.state.mapping.values())
        if len(ids) != len(set(ids)):
            self.notify_error("Duplicate servo IDs in mapping.")
            return False
        return True

    def prev_step_id(self) -> str:
        return "map_joints"
