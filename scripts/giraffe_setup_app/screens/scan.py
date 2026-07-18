"""Servo ID scan screen."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Button, Static

from .. import bus as servo_bus
from .base import WizardScreen


class ScanScreen(WizardScreen):
    step_id = "scan"

    def compose_body(self) -> ComposeResult:
        yield Static("Scan servo IDs (read-only)", classes="title")
        yield Static(
            "No EEPROM writes. Scanning the bus for live Feetech/Waveshare servos.",
            classes="subtitle",
        )
        yield Static("Press Scan to begin.", id="scan-status")
        yield Static("", id="scan-results")

    def compose_nav(self) -> ComposeResult:
        yield Button("← Back", id="btn-back", variant="default")
        yield Button("Scan", id="btn-scan", variant="primary")
        yield Button("Wide rescan (1–252)", id="btn-wide", variant="default")
        yield Button("Next →", id="btn-next", variant="primary")

    def on_mount(self) -> None:
        super().on_mount()
        if self.state.found_ids:
            self._show_results(self.state.found_ids)
        elif self.state.packet is not None:
            self._do_scan(wide=False)

    def handle_button(self, button_id: str | None) -> None:
        if button_id == "btn-scan":
            self._do_scan(wide=False)
        elif button_id == "btn-wide":
            self._do_scan(wide=True)

    def _do_scan(self, wide: bool) -> None:
        if self.state.packet is None:
            self.notify_error("Open a serial port first (go Back).")
            return
        status = self.query_one("#scan-status", Static)
        status.update("Scanning…")
        found: list[int] = []

        def on_found(sid: int) -> None:
            found.append(sid)
            status.update(f"Found ID {sid}…")

        try:
            ids = servo_bus.scan_ids(self.state.packet, on_found=on_found, wide=wide)
        except Exception as exc:
            self.notify_error(str(exc))
            status.update("Scan failed.")
            return
        self.state.found_ids = ids
        # Clear mapping entries that no longer exist
        self.state.mapping = {
            j: i for j, i in self.state.mapping.items() if i in ids
        }
        self._show_results(ids)
        self.refresh_status()

    def _show_results(self, ids: list[int]) -> None:
        status = self.query_one("#scan-status", Static)
        results = self.query_one("#scan-results", Static)
        status.update(f"Found {len(ids)} servo(s).")
        if not ids:
            results.update("No servos responded. Check 12V power, daisy-chain, and USB.")
            return
        lines = [f"  ID {i}" for i in ids]
        note = ""
        if len(ids) < 6:
            note = (
                "\n\n[warn]Expected 6 servos. Fix wiring/power before continuing.[/warn]"
            )
        elif len(ids) > 6:
            note = (
                "\n\n[warn]More than 6 IDs found. You will map only 6 joints next.[/warn]"
            )
        else:
            note = "\n\n[ok]Looks good — 6 servos on the bus.[/ok]"
        results.update("\n".join(lines) + note)

    def validate_next(self) -> bool:
        if len(self.state.found_ids) < 6:
            self.notify_error("Need at least 6 unique servo IDs before continuing.")
            return False
        return True
