"""Done screen."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Button, Markdown, Static

from .base import WizardScreen


class DoneScreen(WizardScreen):
    step_id = "done"
    can_go_back = True

    def compose_body(self) -> ComposeResult:
        path = self.state.config_path or (self.state.repo_root / "config" / "follower.yaml")
        yield Static("Setup complete", classes="title")
        yield Markdown(
            f"Config saved to `{path}`.\n\n"
            "Optional ROS control:\n\n"
            "```bash\n"
            "cd giraffe_ws && colcon build --symlink-install\n"
            "source install/local_setup.bash\n"
            "ros2 launch giraffe_control giraffe_control_launch.py\n"
            "```\n\n"
            "Do **not** disassemble or reset servo IDs unless a scan finds collisions "
            "or fewer than 6 servos. Advanced EEPROM tools: `scripts/st_configurator.py`.\n\n"
            "Press **Restart** (`r`) to run the wizard again, or **Quit** (`q`)."
        )

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button("Restart", id="btn-restart", variant="default")
        yield Button("Quit", id="btn-quit", variant="primary")

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-restart":
            self.action_restart()
        elif button_id == "btn-quit":
            self.app.exit_wizard(0)

    def action_next(self) -> None:
        self.app.exit_wizard(0)
