import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _resolve_follower_config():
    env = os.environ.get("GIRAFFE_FOLLOWER_CONFIG")
    if env and Path(env).is_file():
        return str(Path(env).resolve())

    root_env = os.environ.get("GIRAFFE_ROOT")
    candidates = []
    if root_env:
        candidates.append(Path(root_env) / "config" / "follower.yaml")
        candidates.append(Path(root_env) / "config" / "follower.example.yaml")

    # launch file is at .../giraffe_ws/src/giraffe_control/launch/ (source)
    # or .../share/giraffe_control/launch/ (install)
    here = Path(__file__).resolve().parent
    for parent in here.parents:
        candidates.append(parent / "config" / "follower.yaml")
        candidates.append(parent / "config" / "follower.example.yaml")

    candidates.append(Path.cwd() / "config" / "follower.yaml")
    candidates.append(Path.cwd() / "config" / "follower.example.yaml")

    for path in candidates:
        if path.is_file():
            return str(path.resolve())
    return None


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("giraffe_control"),
        "config",
        "giraffe_control_params.yaml",
    )

    parameters = [config]
    follower_config = _resolve_follower_config()
    if follower_config:
        parameters.append({"follower_config": follower_config})

    return LaunchDescription(
        [
            Node(
                package="giraffe_control",
                executable="giraffe_driver",
                name="giraffe_driver",
                output="screen",
                parameters=parameters,
            ),
        ]
    )
