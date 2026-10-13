from __future__ import annotations

import configparser
from collections.abc import Callable
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[2]

_ROS_SKIP_REASON = "ROS2 not discoverable, source install/setup.bash to enable"


def _ros_available() -> bool:
    try:
        import rclpy  # noqa: F401
    except ImportError:
        return False
    return True


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    if _ros_available():
        return
    skip = pytest.mark.skip(reason=_ROS_SKIP_REASON)
    for item in items:
        path = str(item.path)
        if "/tests/ros/" in path or "/tests/integration/" in path:
            item.add_marker(skip)


@pytest.fixture(scope="session", autouse=True)
def rclpy_context():
    try:
        import rclpy
        import rclpy.node
    except ImportError:
        yield None
        return
    rclpy.init()
    node = rclpy.node.Node("pytest_host")
    try:
        yield rclpy
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _submodules() -> dict[str, list[Path]]:
    cfg = configparser.ConfigParser()
    cfg.read(_REPO / ".gitmodules")
    out: dict[str, list[Path]] = {}
    for section in cfg.sections():
        path = cfg[section]["path"]
        names = cfg[section].get("robot", "").split() + cfg[section].get("component", "").split()
        if not names and path.startswith("arena_robots/robots/"):
            names = [path.split("/")[2]]
        for name in names:
            out.setdefault(name, []).append(_REPO / path)
    return out


@pytest.fixture(scope="session")
def require_installed() -> Callable[[str], None]:
    """Skips the test unless every submodule tagged for the robot or component is checked out."""
    submodules = _submodules()

    def require(name: str) -> None:
        if any(not path.is_dir() or not any(path.iterdir()) for path in submodules.get(name, [])):
            pytest.skip(f"{name} is not installed, run `arena feature robots add {name}`")

    return require
