"""Map servo IDs to joints via nudge."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.binding import Binding
from textual.widgets import Button, Label, ListItem, ListView, Static

from giraffe_control.follower_config import JOINT_LABELS

from .. import bus as servo_bus
from .base import WizardScreen


class MapJointsScreen(WizardScreen):
    step_id = "map_joints"

    BINDINGS = WizardScreen.BINDINGS + [
        Binding("n", "nudge", "Nudge", show=True),
        Binding("space", "nudge", "Nudge", show=False),
    ]

    def __init__(self, state) -> None:
        super().__init__(state)
        self._phase = "pick_id"  # pick_id | assign_joint
        self._current_id: int | None = None
        self._nudged = False

    def compose_body(self) -> ComposeResult:
        yield Static("Map servos → joints", classes="title")
        yield Static(
            "Joints should already be away from limits (previous step). "
            "Each nudge moves ~7°, then restores. Watch which joint moves.",
            classes="warning-box",
        )
        yield Static("", id="map-status")
        yield Static("Select a servo ID, then Nudge:", id="map-help")
        yield ListView(id="map-list")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button("Nudge selected", id="btn-nudge", variant="primary")
        yield Button("Skip ID", id="btn-skip", variant="default")
        yield Button("Review / Next →", id="btn-next", variant="default")

    def on_mount(self) -> None:
        super().on_mount()
        if self.state.redo_servo_id is not None:
            self._current_id = self.state.redo_servo_id
            self.state.redo_servo_id = None
            self._phase = "pick_id"
            self._nudged = False
            # Remove any joint still mapped to this id
            for j, i in list(self.state.mapping.items()):
                if i == self._current_id:
                    del self.state.mapping[j]
        self._refresh_list()

    def _refresh_list(self) -> None:
        lv = self.query_one("#map-list", ListView)
        lv.clear()
        status = self.query_one("#map-status", Static)
        help_w = self.query_one("#map-help", Static)

        if self._phase == "pick_id":
            ids = self.state.unmapped_ids()
            if self._current_id is not None and self._current_id not in ids:
                # already mapped
                self._current_id = None
            if self._current_id is not None:
                ids = [self._current_id] + [i for i in ids if i != self._current_id]
            status.update(
                f"Mapped {self.state.mapped_count}/6. "
                f"Unmapped IDs: {ids or 'none'}"
            )
            help_w.update(
                "↑↓ select an ID · Enter or Nudge to move it · then pick the joint"
            )
            for sid in ids:
                lv.append(ListItem(Label(f"Servo ID {sid}"), id=f"id-{sid}"))
            if not ids:
                help_w.update("All found IDs are mapped (or none left). Go to Review.")
        else:
            joints = self.state.remaining_joints()
            status.update(
                f"Which joint moved for ID {self._current_id}? "
                f"(mapped {self.state.mapped_count}/6)"
            )
            help_w.update("↑↓ select joint · Enter to assign")
            for name in joints:
                lv.append(
                    ListItem(
                        Label(JOINT_LABELS[name]),
                        id=f"joint-{name}",
                    )
                )
        if lv.children:
            lv.index = 0
        self.refresh_status()

    def _selected_id(self) -> int | None:
        lv = self.query_one("#map-list", ListView)
        if lv.index is None or not lv.children:
            return self._current_id
        item = lv.highlighted_child
        if item is None or item.id is None:
            return None
        if item.id.startswith("id-"):
            return int(item.id.split("-", 1)[1])
        return self._current_id

    def _selected_joint(self) -> str | None:
        lv = self.query_one("#map-list", ListView)
        item = lv.highlighted_child
        if item is None or item.id is None:
            return None
        if item.id.startswith("joint-"):
            return item.id[len("joint-") :]
        return None

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-nudge":
            self.action_nudge()
        elif button_id == "btn-skip":
            self._skip_id()

    def action_nudge(self) -> None:
        if self.state.packet is None:
            self.notify_error("Bus not open.")
            return
        if self._phase == "assign_joint":
            # Re-nudge current id
            sid = self._current_id
        else:
            sid = self._selected_id()
        if sid is None:
            self.notify_error("Select a servo ID first.")
            return
        self._current_id = sid
        ok, msg = servo_bus.nudge_servo(self.state.packet, sid)
        if not ok:
            self.notify_error(msg)
            return
        self._nudged = True
        self._phase = "assign_joint"
        self.notify_info(msg)
        self._refresh_list()

    def _skip_id(self) -> None:
        if self._phase == "assign_joint":
            self._phase = "pick_id"
            self._current_id = None
            self._nudged = False
            self._refresh_list()
            return
        sid = self._selected_id()
        if sid is None:
            return
        # Remove from consideration by temporarily not listing — mark as skipped by
        # leaving unmapped; user can still map later. Just advance highlight.
        self.notify_info(f"Skipped ID {sid} for now")

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        if self._phase == "assign_joint":
            joint = self._selected_joint()
            if joint and self._current_id is not None:
                self.state.mapping[joint] = self._current_id
                self.notify_info(
                    f"Mapped ID {self._current_id} → {JOINT_LABELS[joint]}"
                )
                self._current_id = None
                self._nudged = False
                self._phase = "pick_id"
                self._refresh_list()
        elif self._phase == "pick_id":
            # Enter on ID starts nudge
            self.action_nudge()

    def validate_next(self) -> bool:
        if self.state.mapped_count < 6:
            # Allow going to review anyway to redo / see status
            pass
        return True

    def next_step_id(self) -> str:
        return "review_map"
