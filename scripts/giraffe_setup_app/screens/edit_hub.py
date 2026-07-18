"""Hub for editing an existing follower.yaml."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Button, Label, ListItem, ListView, Static

from giraffe_control.follower_config import JOINT_LABELS

from .base import WizardScreen

EDIT_ACTIONS = [
    ("env_check", "Environment check"),
    ("port_select", "Serial port / open bus"),
    ("scan", "Re-scan servo IDs"),
    ("safety", "Safety / torque-off"),
    ("map_joints", "Re-map joints"),
    ("review_map", "Review joint mapping"),
    ("reverses", "Reverse flags (test moves)"),
    ("auto_calibrate", "Limit sweeps (auto-cal)"),
    ("wrist2_ground", "Wrist_2 center + ground"),
    ("calib_confirm", "Confirm / write EEPROM"),
    ("calibrate", "Legacy manual zero pose"),
    ("write_confirm", "Review & write config"),
    ("smoke_test", "Smoke test"),
    ("done", "Done / next steps"),
]


class EditHubScreen(WizardScreen):
    step_id = "edit_hub"

    def compose_body(self) -> ComposeResult:
        path = self.state.config_path or self.state.existing_config_path()
        yield Static("Edit existing follower config", classes="title")
        yield Static(
            f"Loaded: {path}\n"
            f"Port: {self.state.port or '—'}  ·  "
            f"IDs: {self.state.found_ids}  ·  "
            f"Mapped: {self.state.mapped_count}/6",
            classes="subtitle",
        )
        summary_lines = []
        for name, sid in self.state.mapping.items():
            rev = "REV" if self.state.reverses.get(name) else "ok"
            off = self.state.offsets.get(name, 0.0)
            summary_lines.append(
                f"  {JOINT_LABELS[name]}: id={sid}  [{rev}]  offset={off:.3f}"
            )
        yield Static(
            "Current mapping:\n" + ("\n".join(summary_lines) or "  (empty)"),
            id="edit-summary",
        )
        yield Static(
            "Select a section to edit. Bus opens when you visit Serial port. "
            "Remember to Write config when finished.",
            classes="hint",
        )
        items = [
            ListItem(Label(label), id=f"act-{step}") for step, label in EDIT_ACTIONS
        ]
        yield ListView(*items, id="edit-actions")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Welcome", id="btn-back", variant="default")
        yield Button("Open selected →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        super().on_mount()
        self.query_one("#edit-actions", ListView).index = 0

    def _selected_step(self) -> str | None:
        lv = self.query_one("#edit-actions", ListView)
        item = lv.highlighted_child
        if item is None or item.id is None or not item.id.startswith("act-"):
            return None
        return item.id[len("act-") :]

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        self.action_next()

    def validate_next(self) -> bool:
        step = self._selected_step()
        if not step:
            self.notify_error("Select a section.")
            return False
        return True

    def next_step_id(self) -> str:
        return self._selected_step() or "edit_hub"

    def prev_step_id(self) -> str:
        return "welcome"
