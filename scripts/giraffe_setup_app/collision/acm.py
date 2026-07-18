"""Load MoveIt SRDF disabled-collision pairs."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from giraffe_control.follower_config import find_repo_root


def default_srdf_path(repo_root: Path | None = None) -> Path:
    root = repo_root or find_repo_root()
    if root is None:
        raise FileNotFoundError("repo root not found")
    return (
        root
        / "giraffe_ws"
        / "src"
        / "giraffe_moveit_config"
        / "config"
        / "giraffe.srdf"
    )


def load_disabled_pairs(srdf_path: Path | None = None) -> set[tuple[str, str]]:
    path = srdf_path or default_srdf_path()
    pairs: set[tuple[str, str]] = set()
    if not path.is_file():
        return pairs
    root = ET.parse(path).getroot()
    for el in root.findall("disable_collisions"):
        a = el.get("link1")
        b = el.get("link2")
        if a and b:
            pairs.add(tuple(sorted((a, b))))
    return pairs
