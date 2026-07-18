"""Preview and write config/follower.yaml."""

from __future__ import annotations

import yaml
from textual.app import ComposeResult
from textual.widgets import Button, Static

from giraffe_control.follower_config import default_config_path, save_follower_config

from .base import WizardScreen


class WriteConfirmScreen(WizardScreen):
    step_id = "write_confirm"

    def compose_body(self) -> ComposeResult:
        yield Static("Write config/follower.yaml", classes="title")
        yield Static(
            "Preview below. Enter / Write saves the arm-specific config (gitignored).",
            classes="subtitle",
        )
        try:
            preview = yaml.safe_dump(
                self.state.build_config().to_dict(),
                default_flow_style=False,
                sort_keys=False,
            )
        except Exception as exc:
            preview = f"Error building config: {exc}"
        yield Static(preview, id="yaml-preview")
        yield Static("", id="write-status", classes="hint")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button("Write file", id="btn-write", variant="primary")
        yield Button("Next →", id="btn-next", variant="default")

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-write":
            self._write()

    def _write(self) -> bool:
        try:
            path = save_follower_config(
                self.state.build_config(),
                default_config_path(self.state.repo_root),
            )
        except Exception as exc:
            self.notify_error(str(exc))
            return False
        self.state.config_path = path
        self.query_one("#write-status", Static).update(f"Wrote {path}")
        self.notify_info(f"Wrote {path}")
        return True

    def validate_next(self) -> bool:
        if self.state.config_path is None:
            return self._write()
        return True
