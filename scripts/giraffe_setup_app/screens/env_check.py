"""Environment check screen."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Button, Static

from ..env_checks import INSTALL_HINT, hard_deps_ok, run_env_checks
from .base import WizardScreen


class EnvCheckScreen(WizardScreen):
    step_id = "env_check"

    def compose_body(self) -> ComposeResult:
        yield Static("Dependency checklist", classes="title")
        self._items = run_env_checks(self.state.repo_root)
        lines = []
        for item in self._items:
            mark = "OK" if item.ok else "FAIL"
            style = "ok" if item.ok else ("warn" if not item.required else "fail")
            req = "" if item.required else " (optional)"
            lines.append(f"[{style}]{mark}[/{style}]  {item.name}{req}  —  {item.detail}")
        yield Static("\n".join(lines), id="env-list")
        if not hard_deps_ok(self._items):
            yield Static(
                "Install from the repo root:\n" + INSTALL_HINT,
                classes="danger-box",
            )
        else:
            yield Static("Hard dependencies look good.", classes="ok")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button("Re-check", id="btn-recheck", variant="default")
        yield Button("Next →", id="btn-next", variant="primary")

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-recheck":
            self.app.goto_step("env_check")

    def validate_next(self) -> bool:
        items = run_env_checks(self.state.repo_root)
        if not hard_deps_ok(items):
            self.notify_error("Fix required dependencies before continuing.")
            return False
        return True
