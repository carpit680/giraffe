"""Shared screen chrome for wizard steps."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, Vertical
from textual.screen import ModalScreen, Screen
from textual.widgets import Button, Footer, Header, Label, Static

from ..state import STEP_TITLES, STEPS, SetupState


class ConfirmModal(ModalScreen[bool]):
    """Yes/No confirmation dialog."""

    BINDINGS = [
        Binding("escape", "no", "Cancel", show=True),
        Binding("y", "yes", "Yes", show=True),
        Binding("n", "no", "No", show=True),
    ]

    def __init__(self, message: str, *, title: str = "Confirm") -> None:
        super().__init__()
        self.message = message
        self.title_text = title

    def compose(self) -> ComposeResult:
        with Vertical(id="modal"):
            yield Label(self.title_text, classes="modal-title")
            yield Static(self.message, id="modal-body")
            with Horizontal(classes="modal-actions"):
                yield Button("Yes", variant="primary", id="yes")
                yield Button("No / Cancel", variant="default", id="no")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss(event.button.id == "yes")

    def action_yes(self) -> None:
        self.dismiss(True)

    def action_no(self) -> None:
        self.dismiss(False)


class WizardScreen(Screen):
    """Base wizard screen with header status and nav helpers."""

    BINDINGS = [
        Binding("b", "back", "Back", show=True),
        Binding("backspace", "back", "Back", show=False),
        Binding("r", "restart", "Restart", show=True),
        Binding("q", "quit_wizard", "Quit", show=True),
    ]

    step_id: str = "welcome"
    can_go_back: bool = True

    def __init__(self, state: SetupState) -> None:
        super().__init__()
        self.state = state

    @property
    def step_index(self) -> int:
        if self.step_id not in STEPS:
            return 0
        return STEPS.index(self.step_id)

    @property
    def step_label(self) -> str:
        title = STEP_TITLES.get(self.step_id, self.step_id)
        if self.step_id not in STEPS:
            mode = "edit" if self.state.edit_mode else "setup"
            return f"{mode}  ·  {title}"
        prefix = "edit" if self.state.edit_mode else f"{self.step_index + 1}/{len(STEPS)}"
        return f"{prefix}  {title}"

    def compose(self) -> ComposeResult:
        yield Header(show_clock=False)
        with Container(id="body"):
            yield Static(self.step_label, id="step-banner")
            yield Static(self.state.status_text(), id="status-line")
            yield from self.compose_body()
            with Horizontal(id="nav-bar"):
                yield from self.compose_nav()
        yield Footer()

    def compose_body(self) -> ComposeResult:
        yield Static("")

    def compose_nav(self) -> ComposeResult:
        if self.can_go_back and self.step_id != "welcome":
            yield Button("← Back", id="btn-back", variant="default")
        if self.state.edit_mode and self.step_id not in ("welcome", "edit_hub"):
            yield Button("Edit menu", id="btn-edit-hub", variant="default")
        yield Button("Next →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        self.query_one("#status-line", Static).update(self.state.status_text())

    def refresh_status(self) -> None:
        try:
            self.query_one("#status-line", Static).update(self.state.status_text())
        except Exception:
            pass

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "btn-back":
            self.action_back()
        elif event.button.id == "btn-next":
            self.action_next()
        elif event.button.id == "btn-edit-hub":
            self.app.goto_step("edit_hub")
        else:
            self.handle_button(event.button.id)

    def handle_button(self, button_id: str | None) -> None:
        pass

    def action_next(self) -> None:
        if self.validate_next():
            self.app.goto_step(self.next_step_id())

    def action_back(self) -> None:
        if not self.can_go_back or self.step_id == "welcome":
            return
        if self.state.edit_mode and self.step_id == "edit_hub":
            self.app.goto_step("welcome")
            return
        if self.state.edit_mode and self.step_id == "review_map":
            self.app.goto_step("map_joints")
            return
        if self.state.edit_mode and self.step_id not in ("welcome",):
            self.app.goto_step("edit_hub")
            return
        self.app.goto_step(self.prev_step_id())

    def action_restart(self) -> None:
        def _done(confirmed: bool | None) -> None:
            if confirmed:
                self.app.restart_wizard()

        self.app.push_screen(
            ConfirmModal(
                "Restart setup from the beginning?\nPort will close and all progress will be cleared.",
                title="Restart",
            ),
            _done,
        )

    def action_quit_wizard(self) -> None:
        try:
            past_scan = self.step_id in STEPS and self.step_index > STEPS.index("scan")
        except ValueError:
            past_scan = self.state.edit_mode

        def _done(confirmed: bool | None) -> None:
            if confirmed or not past_scan:
                self.app.exit_wizard()

        if past_scan:
            self.app.push_screen(
                ConfirmModal("Quit setup? Unsaved progress will be lost.", title="Quit"),
                _done,
            )
        else:
            self.app.exit_wizard()

    def validate_next(self) -> bool:
        return True

    def next_step_id(self) -> str:
        idx = self.step_index
        return STEPS[min(idx + 1, len(STEPS) - 1)]

    def prev_step_id(self) -> str:
        idx = self.step_index
        return STEPS[max(idx - 1, 0)]

    def notify_error(self, message: str) -> None:
        self.notify(message, severity="error")

    def notify_info(self, message: str) -> None:
        self.notify(message, severity="information")
