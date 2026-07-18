"""Environment checks for the setup wizard."""

from __future__ import annotations

import getpass
import grp
import os
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass
class CheckItem:
    name: str
    ok: bool
    detail: str
    required: bool = True


def run_env_checks(repo_root: Path) -> list[CheckItem]:
    items: list[CheckItem] = []

    py_ok = sys.version_info >= (3, 8)
    items.append(
        CheckItem(
            "Python 3.8+",
            py_ok,
            sys.version.split()[0],
            required=True,
        )
    )

    for label, mod in (("pyserial", "serial"), ("PyYAML", "yaml"), ("textual", "textual")):
        try:
            __import__(mod)
            items.append(CheckItem(label, True, "import ok", required=True))
        except ImportError:
            items.append(CheckItem(label, False, "missing — pip install -r requirements.txt", required=True))

    try:
        from STservo_sdk import PortHandler  # noqa: F401

        items.append(CheckItem("STservo_sdk", True, "import ok", required=True))
    except ImportError:
        items.append(
            CheckItem(
                "STservo_sdk",
                False,
                "missing — pip install . from repo root",
                required=True,
            )
        )

    try:
        import scservo_sdk  # noqa: F401

        items.append(CheckItem("feetech-servo-sdk", True, "import ok", required=False))
    except ImportError:
        items.append(
            CheckItem(
                "feetech-servo-sdk",
                False,
                "optional for wizard; needed for ROS giraffe_driver",
                required=False,
            )
        )

    dialout_ok = True
    dialout_detail = "ok"
    try:
        dialout = grp.getgrnam("dialout")
        username = getpass.getuser()
        if os.getuid() == 0:
            dialout_detail = "running as root"
        elif dialout.gr_gid in os.getgroups() or username in dialout.gr_mem:
            dialout_detail = f"{username} in dialout"
        else:
            dialout_ok = False
            dialout_detail = "not in dialout — sudo usermod -aG dialout $USER && newgrp dialout"
    except KeyError:
        dialout_ok = False
        dialout_detail = "no dialout group on this system"
    except OSError as exc:
        dialout_ok = False
        dialout_detail = str(exc)

    items.append(CheckItem("Serial permissions (dialout)", dialout_ok, dialout_detail, required=False))

    example = repo_root / "config" / "follower.example.yaml"
    items.append(
        CheckItem(
            "Repo config template",
            example.is_file(),
            str(repo_root),
            required=True,
        )
    )
    return items


def hard_deps_ok(items: list[CheckItem]) -> bool:
    return all(i.ok for i in items if i.required)


INSTALL_HINT = (
    "python3 -m venv giraffe_env && source giraffe_env/bin/activate\n"
    "pip install -r requirements.txt && pip install ."
)
