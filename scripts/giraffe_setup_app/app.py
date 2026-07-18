"""Giraffe follower setup Textual application."""

from __future__ import annotations

from pathlib import Path

from textual.app import App
from textual.binding import Binding

from .state import STEPS, SetupState
from .screens.welcome import WelcomeScreen
from .screens.edit_hub import EditHubScreen
from .screens.env_check import EnvCheckScreen
from .screens.port_select import PortSelectScreen
from .screens.scan import ScanScreen
from .screens.safety import SafetyScreen
from .screens.map_joints import MapJointsScreen
from .screens.review_map import ReviewMapScreen
from .screens.reverses import ReversesScreen
from .screens.calibrate import CalibrateScreen
from .screens.auto_calibrate import AutoCalibrateScreen
from .screens.wrist2_ground import Wrist2GroundScreen
from .screens.calib_confirm import CalibConfirmScreen
from .screens.write_confirm import WriteConfirmScreen
from .screens.smoke_test import SmokeTestScreen
from .screens.done import DoneScreen

SCREEN_MAP = {
    "welcome": WelcomeScreen,
    "edit_hub": EditHubScreen,
    "env_check": EnvCheckScreen,
    "port_select": PortSelectScreen,
    "scan": ScanScreen,
    "safety": SafetyScreen,
    "map_joints": MapJointsScreen,
    "review_map": ReviewMapScreen,
    "reverses": ReversesScreen,
    "calibrate": CalibrateScreen,
    "auto_calibrate": AutoCalibrateScreen,
    "wrist2_ground": Wrist2GroundScreen,
    "calib_confirm": CalibConfirmScreen,
    "write_confirm": WriteConfirmScreen,
    "smoke_test": SmokeTestScreen,
    "done": DoneScreen,
}

CSS = """
Screen {
    background: #0f1419;
}

#body {
    padding: 1 2;
    height: 1fr;
}

#step-banner {
    color: #7dd3fc;
    text-style: bold;
    margin-bottom: 0;
}

#status-line {
    color: #94a3b8;
    margin-bottom: 1;
}

#nav-bar {
    dock: bottom;
    height: 3;
    align: left middle;
    padding-top: 1;
}

#nav-bar Button {
    margin-right: 1;
}

.title {
    text-style: bold;
    color: #e2e8f0;
    margin-bottom: 1;
}

.subtitle {
    color: #94a3b8;
    margin-bottom: 1;
}

.hint {
    color: #64748b;
    margin-top: 1;
}

.warning-box {
    background: #422006;
    color: #fde68a;
    padding: 1 2;
    margin: 1 0;
    border: solid #b45309;
}

.danger-box {
    background: #450a0a;
    color: #fecaca;
    padding: 1 2;
    margin: 1 0;
    border: solid #dc2626;
}

.pose-art {
    background: #0c1222;
    color: #a5f3fc;
    padding: 1 2;
    margin: 1 0;
    border: solid #155e75;
    height: auto;
}

.ok {
    color: #86efac;
}

.fail {
    color: #fca5a5;
}

.warn {
    color: #fde68a;
}

ListView {
    height: 1fr;
    border: solid #334155;
    background: #1e293b;
}

#rev-layout {
    height: 1fr;
}

#rev-layout #joint-list {
    width: 36;
    margin-right: 1;
}

#rev-detail {
    width: 1fr;
    height: 1fr;
}

#rev-actions {
    height: 3;
    margin-top: 1;
}

#rev-actions Button {
    margin-right: 1;
}

ListItem {
    padding: 0 1;
}

ListItem.--highlight {
    background: #0369a1;
}

Button {
    min-width: 12;
}

#modal {
    width: 60;
    height: auto;
    padding: 1 2;
    background: #1e293b;
    border: thick #38bdf8;
    align: center middle;
}

.modal-title {
    text-style: bold;
    color: #7dd3fc;
    margin-bottom: 1;
}

.modal-actions {
    margin-top: 1;
    height: 3;
}

.modal-actions Button {
    margin-right: 1;
}

#telem-table, #wg-telem {
    height: 8;
    max-height: 9;
    margin: 0 0 1 0;
}

DataTable {
    height: auto;
    max-height: 14;
    margin: 1 0;
}

Checkbox {
    margin: 1 0;
}
"""


class GiraffeSetupApp(App[int]):
    """Keyboard-driven follower bringup / reconfigure wizard."""

    TITLE = "Giraffe Follower Setup"
    CSS = CSS
    BINDINGS = [
        Binding("ctrl+c", "quit_wizard", "Quit", show=False),
    ]

    def __init__(self, repo_root: Path) -> None:
        super().__init__()
        self.repo_root = repo_root
        self.state = SetupState(repo_root=repo_root)
        self._current_step = "welcome"

    def on_mount(self) -> None:
        self._current_step = "welcome"
        self.push_screen(WelcomeScreen(self.state))

    def goto_step(self, step_id: str) -> None:
        if step_id not in SCREEN_MAP:
            raise KeyError(step_id)
        self._current_step = step_id
        screen = SCREEN_MAP[step_id](self.state)
        # First real screen is pushed onto Textual's default screen; later steps replace it.
        if len(self.screen_stack) <= 1:
            self.push_screen(screen)
        else:
            self.switch_screen(screen)

    def restart_wizard(self) -> None:
        self.state.reset()
        self.goto_step("welcome")
        self.notify("Wizard restarted", severity="information")

    def exit_wizard(self, code: int = 0) -> None:
        self.state.close_bus()
        self.exit(code)

    def action_quit_wizard(self) -> None:
        # Delegate to current screen if it has confirm logic
        screen = self.screen
        if hasattr(screen, "action_quit_wizard"):
            screen.action_quit_wizard()
        else:
            self.exit_wizard()
