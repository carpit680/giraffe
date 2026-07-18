#!/usr/bin/env python3
"""Giraffe follower setup — keyboard-driven Textual TUI.

Maps servo IDs to joints (no ID EEPROM resets for normal use), runs
auto-calibration (limit sweeps, wrist_2 wire center, ground height),
optionally writes Feetech range/offset EEPROM, and saves config/follower.yaml.

  GIRAFFE_CALIB_MODE=dry_run python3 scripts/giraffe_setup.py   # no motion
  python3 scripts/giraffe_setup.py                              # hardware

See README.md → Hardware Setup Instructions.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent

sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "giraffe_ws" / "src" / "giraffe_control"))
sys.path.insert(0, str(SCRIPTS_DIR))


def main() -> int:
    try:
        from giraffe_setup_app.app import GiraffeSetupApp
    except ImportError as exc:
        print(
            "Failed to import the setup TUI. Install dependencies:\n"
            "  python3 -m venv giraffe_env && source giraffe_env/bin/activate\n"
            "  pip install -r requirements.txt && pip install .\n"
            f"\nDetails: {exc}",
            file=sys.stderr,
        )
        return 1

    app = GiraffeSetupApp(repo_root=REPO_ROOT)
    result = app.run()
    return int(result) if result is not None else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nAborted.", file=sys.stderr)
        raise SystemExit(130)
