"""Welcome screen — fresh setup or edit existing config."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Button, Label, ListItem, ListView, Markdown, Static

from .base import ConfirmModal, WizardScreen


class WelcomeScreen(WizardScreen):
    step_id = "welcome"
    can_go_back = False

    def compose_body(self) -> ComposeResult:
        yield Static("Giraffe follower bringup", classes="title")
        yield Markdown(
            "Configure an **assembled** follower arm without disassembly "
            "and without resetting servo EEPROM IDs.\n\n"
            "- Discover live servo IDs on the bus\n"
            "- Map each ID to a joint with a small nudge\n"
            "- Save `config/follower.yaml` for the ROS driver\n\n"
            "**Keys:** `↑` `↓` move · `Enter` select/next · `b` back · "
            "`r` restart · `q` quit"
        )
        if self.state.has_existing_config():
            path = self.state.existing_config_path()
            yield Static(
                f"Found existing config:\n{path}\n\n"
                "Choose Edit to change port, mapping, reverses, or calibration, "
                "or Restart to wipe in-memory state and run the full wizard again "
                "(writing later will overwrite the file).",
                classes="warning-box",
            )
            yield ListView(
                ListItem(
                    Label("Edit existing config"),
                    id="opt-edit",
                ),
                ListItem(
                    Label("Restart setup from scratch"),
                    id="opt-fresh",
                ),
                id="welcome-options",
            )
        else:
            yield Static(
                "No config/follower.yaml yet — continue to create one.",
                classes="hint",
            )

    def compose_nav(self) -> ComposeResult:
        if self.state.has_existing_config():
            yield Button("Continue →", id="btn-next", variant="primary")
        else:
            yield Button("Start →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        super().on_mount()
        if self.state.has_existing_config():
            try:
                self.query_one("#welcome-options", ListView).index = 0
            except Exception:
                pass

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        self.action_next()

    def _selected_option(self) -> str:
        if not self.state.has_existing_config():
            return "fresh"
        try:
            lv = self.query_one("#welcome-options", ListView)
            item = lv.highlighted_child
            if item is not None and item.id == "opt-edit":
                return "edit"
            if item is not None and item.id == "opt-fresh":
                return "fresh"
        except Exception:
            pass
        return "edit"

    def validate_next(self) -> bool:
        return True

    def action_next(self) -> None:
        choice = self._selected_option()
        if choice == "edit":
            try:
                path = self.state.load_from_disk()
            except Exception as exc:
                self.notify_error(f"Failed to load config: {exc}")
                return
            self.notify_info(f"Loaded {path}")
            self.app.goto_step("edit_hub")
            return

        # Fresh start
        if self.state.has_existing_config() and (
            self.state.edit_mode or self.state.mapping
        ):

            def _done(confirmed: bool | None) -> None:
                if confirmed:
                    self.state.reset()
                    self.app.goto_step("env_check")

            self.app.push_screen(
                ConfirmModal(
                    "Start from scratch?\n"
                    "This clears the current session. "
                    "config/follower.yaml on disk is kept until you Write again "
                    "(then it will be overwritten).",
                    title="Restart from scratch",
                ),
                _done,
            )
            return

        self.state.reset()
        self.state.edit_mode = False
        self.app.goto_step("env_check")
