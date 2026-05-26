"""Domain fixtures for arena_robots ROS-gated tests."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
import yaml


@pytest.fixture()
def temp_robot_dir(tmp_path: Path):
    """Return a factory that writes a minimal robot directory tree.

    Usage::

        def test_foo(temp_robot_dir):
            robot_dir = temp_robot_dir(
                model_params={"base_frame": "base_link"},
                control={"cmd_vel": "/cmd_vel"},
                mobile_cap={"odom_frame": "odom"},
            )
    """

    def _factory(
        *,
        model_params: dict[str, Any] | None = None,
        control: dict[str, Any] | None = None,
        mobile_cap: dict[str, Any] | None = None,
        extra_caps: dict[str, dict[str, Any]] | None = None,
        name: str = "test_robot",
    ) -> Path:
        robot_dir = tmp_path / name
        robot_dir.mkdir(parents=True, exist_ok=True)

        if model_params is not None:
            (robot_dir / "model_params.yaml").write_text(yaml.dump(model_params))

        if control is not None:
            (robot_dir / "control.yaml").write_text(yaml.dump(control))

        if mobile_cap is not None or extra_caps is not None:
            caps_dir = robot_dir / "caps"
            caps_dir.mkdir(exist_ok=True)
            if mobile_cap is not None:
                (caps_dir / "mobile.yaml").write_text(yaml.dump(mobile_cap))
            for cap_name, cap_data in (extra_caps or {}).items():
                (caps_dir / f"{cap_name}.yaml").write_text(yaml.dump(cap_data))

        return robot_dir

    return _factory


@pytest.fixture()
def stub_node():
    """Minimal Node-like SimpleNamespace for tests that only read one or two attrs."""
    node = SimpleNamespace()
    node.get_logger = lambda: SimpleNamespace(
        info=lambda msg: None,
        warn=lambda msg: None,
        error=lambda msg: None,
        debug=lambda msg: None,
    )
    node.create_publisher = MagicMock()
    node.create_timer = MagicMock()
    node.destroy_timer = MagicMock()
    return node
