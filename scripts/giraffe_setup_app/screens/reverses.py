"""Interactive reverse-flag checking with directional test moves."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import Button, Label, ListItem, ListView, Static

from giraffe_control.follower_config import (
    DEFAULT_REVERSES,
    JOINT_LABELS,
    JOINT_NAMES,
    JOINT_POSITIVE_MOTION,
)

from .. import bus as servo_bus
from .base import WizardScreen


class ReversesScreen(WizardScreen):
    step_id = "reverses"

    BINDINGS = WizardScreen.BINDINGS + [
        Binding("t", "test_move", "Test move", show=True),
        Binding("f", "flip", "Flip reverse", show=True),
        Binding("y", "mark_ok", "Looks correct", show=True),
    ]

    def __init__(self, state) -> None:
        super().__init__(state)
        self._confirmed: set[str] = set()

    def compose_body(self) -> ComposeResult:
        yield Static("Check reverse flags", classes="title")
        yield Static(
            "For each joint, read the expected motion, press Test move, then "
            "Flip reverse if it moved the wrong way. Mark Looks correct when it matches.",
            classes="subtitle",
        )
        with Horizontal(id="rev-layout"):
            items = [
                ListItem(
                    Label(self._list_label(name)),
                    id=f"j-{name}",
                )
                for name in JOINT_NAMES
            ]
            yield ListView(*items, id="joint-list")
            with Vertical(id="rev-detail"):
                yield Static("", id="rev-joint-title", classes="title")
                yield Static("", id="rev-expected", classes="warning-box")
                yield Static("", id="rev-state", classes="hint")
                yield Static(
                    "Clear space around the arm. Test move is ~10° then returns.",
                    classes="hint",
                )
                with Horizontal(id="rev-actions"):
                    yield Button("Test move", id="btn-test", variant="primary")
                    yield Button("Flip reverse", id="btn-flip", variant="warning")
                    yield Button("Looks correct", id="btn-ok", variant="success")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button("Reset defaults", id="btn-defaults", variant="default")
        yield Button("Next →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        super().on_mount()
        lv = self.query_one("#joint-list", ListView)
        lv.index = 0
        self._refresh_detail()

    def _list_label(self, name: str) -> str:
        rev = "REV" if self.state.reverses.get(name, False) else "ok "
        conf = "✓" if name in self._confirmed else "·"
        return f"{conf} [{rev}] {JOINT_LABELS[name]}"

    def _selected_joint(self) -> str | None:
        lv = self.query_one("#joint-list", ListView)
        item = lv.highlighted_child
        if item is None or item.id is None or not item.id.startswith("j-"):
            return None
        return item.id[len("j-") :]

    def _refresh_list_labels(self) -> None:
        lv = self.query_one("#joint-list", ListView)
        idx = lv.index
        for i, name in enumerate(JOINT_NAMES):
            try:
                item = lv.children[i]
                label = item.query_one(Label)
                label.update(self._list_label(name))
            except Exception:
                pass
        if idx is not None:
            lv.index = idx

    def _refresh_detail(self) -> None:
        name = self._selected_joint()
        title = self.query_one("#rev-joint-title", Static)
        expected = self.query_one("#rev-expected", Static)
        state = self.query_one("#rev-state", Static)
        if name is None:
            title.update("Select a joint")
            expected.update("")
            state.update("")
            return
        sid = self.state.mapping.get(name, "?")
        rev = self.state.reverses.get(name, False)
        conf = "confirmed" if name in self._confirmed else "not confirmed yet"
        title.update(f"{JOINT_LABELS[name]}  (servo ID {sid})")
        expected.update(
            "Expected when you Test move (positive joint command):\n\n"
            + JOINT_POSITIVE_MOTION.get(name, "(no description)")
        )
        state.update(
            f"Reverse flag: [bold]{'ON' if rev else 'OFF'}[/bold]  ·  {conf}\n"
            "If the arm moves the opposite of the description → Flip reverse, then Test again."
        )

    def on_list_view_highlighted(self, event: ListView.Highlighted) -> None:
        self._refresh_detail()

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-test":
            self.action_test_move()
        elif button_id == "btn-flip":
            self.action_flip()
        elif button_id == "btn-ok":
            self.action_mark_ok()
        elif button_id == "btn-defaults":
            self.state.reverses = dict(DEFAULT_REVERSES)
            self._confirmed.clear()
            self._refresh_list_labels()
            self._refresh_detail()
            self.notify_info("Restored default reverse flags")

    def action_test_move(self) -> None:
        name = self._selected_joint()
        if name is None:
            self.notify_error("Select a joint first.")
            return
        if self.state.packet is None:
            self.notify_error("Bus not open.")
            return
        sid = self.state.mapping.get(name)
        if sid is None:
            self.notify_error("No servo mapped for this joint.")
            return
        reverse = self.state.reverses.get(name, False)
        ok, msg = servo_bus.nudge_joint_positive(
            self.state.packet, sid, reverse=reverse
        )
        if ok:
            self.notify_info(msg)
            self._confirmed.discard(name)
            self._refresh_list_labels()
            self._refresh_detail()
        else:
            self.notify_error(msg)

    def action_flip(self) -> None:
        name = self._selected_joint()
        if name is None:
            self.notify_error("Select a joint first.")
            return
        self.state.reverses[name] = not self.state.reverses.get(name, False)
        self._confirmed.discard(name)
        self._refresh_list_labels()
        self._refresh_detail()
        rev = self.state.reverses[name]
        self.notify_info(
            f"{JOINT_LABELS[name]}: reverse now {'ON' if rev else 'OFF'} — Test move again"
        )

    def action_mark_ok(self) -> None:
        name = self._selected_joint()
        if name is None:
            return
        self._confirmed.add(name)
        self._refresh_list_labels()
        self._refresh_detail()
        self.notify_info(f"{JOINT_LABELS[name]} marked correct")
        # Advance highlight to next unconfirmed joint
        lv = self.query_one("#joint-list", ListView)
        for i, jn in enumerate(JOINT_NAMES):
            if jn not in self._confirmed:
                lv.index = i
                self._refresh_detail()
                return

    def validate_next(self) -> bool:
        # Allow continue even if not all confirmed — soft nudge
        missing = [JOINT_LABELS[n] for n in JOINT_NAMES if n not in self._confirmed]
        if missing:
            self.notify_info(
                "Tip: unconfirmed joints: " + ", ".join(missing)
                + " (you can still continue)"
            )
        return True
