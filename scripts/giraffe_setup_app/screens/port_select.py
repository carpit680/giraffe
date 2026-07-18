"""Serial port selection screen."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Button, Label, ListItem, ListView, Static

from .. import bus as servo_bus
from .base import WizardScreen


class PortSelectScreen(WizardScreen):
    step_id = "port_select"

    def __init__(self, state) -> None:
        super().__init__(state)
        self._ports = servo_bus.list_serial_ports()
        self._detect_phase = 0  # 0 idle, 1 connected snapshot taken
        self._before: set[str] = set()

    def compose_body(self) -> ComposeResult:
        yield Static("Select Waveshare servo driver port", classes="title")
        yield Static(
            "Power the arm (12V) and plug in USB. Pick a port or use plug/unplug detect.",
            classes="subtitle",
        )
        items = [ListItem(Label(p), id=f"port-{i}") for i, p in enumerate(self._ports)]
        if not items:
            yield Static("No serial ports found.", classes="warning-box")
        else:
            yield ListView(*items, id="port-list")
        yield Static("", id="port-msg", classes="hint")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button("Refresh", id="btn-refresh", variant="default")
        yield Button("Plug/unplug detect", id="btn-detect", variant="default")
        yield Button("Open & Next →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        super().on_mount()
        if self._ports and self.state.port in self._ports:
            idx = self._ports.index(self.state.port)
            lv = self.query_one("#port-list", ListView)
            lv.index = idx
        elif self._ports:
            self.query_one("#port-list", ListView).index = 0

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-refresh":
            self._ports = servo_bus.list_serial_ports()
            self.app.goto_step("port_select")
        elif button_id == "btn-detect":
            self._run_detect_step()

    def _run_detect_step(self) -> None:
        msg = self.query_one("#port-msg", Static)
        if self._detect_phase == 0:
            self._before = set(servo_bus.list_serial_ports())
            self._detect_phase = 1
            msg.update(
                f"Ports now: {sorted(self._before) or '(none)'}. "
                "Disconnect the Waveshare driver, then press Plug/unplug detect again."
            )
            return

        after = set(servo_bus.list_serial_ports())
        gone = list(self._before - after)
        self._detect_phase = 0
        if len(gone) == 1:
            self.state.port = gone[0]
            msg.update(
                f"Detected {gone[0]}. Reconnect the driver, then Open & Next."
            )
            self.refresh_status()
            self.notify_info(f"Detected port {gone[0]}")
        elif len(gone) > 1:
            msg.update(f"Multiple ports disappeared: {gone}. Pick manually.")
        else:
            msg.update("No port change detected. Pick manually.")

    def _selected_port(self) -> str | None:
        if self.state.port and self._detect_phase == 0:
            # Prefer explicit detect result if still set and user didn't change list
            pass
        try:
            lv = self.query_one("#port-list", ListView)
        except Exception:
            return self.state.port
        if lv.index is None or not self._ports:
            return self.state.port
        if 0 <= lv.index < len(self._ports):
            return self._ports[lv.index]
        return self.state.port

    def validate_next(self) -> bool:
        port = self._selected_port()
        if not port:
            self.notify_error("Select a serial port first.")
            return False

        # Reopen if changing ports
        if self.state.port_handler and self.state.port != port:
            self.state.close_bus()

        if self.state.packet is None or self.state.port != port:
            try:
                handler, packet = servo_bus.open_bus(port, self.state.baudrate)
            except Exception as exc:
                self.notify_error(f"Could not open {port}: {exc}")
                return False
            self.state.port = port
            self.state.port_handler = handler
            self.state.packet = packet
            self.notify_info(f"Opened {port}")
        self.refresh_status()
        return True
