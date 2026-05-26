"""Tests for arena_robots.bringup.mobile.external - ExternalBringup."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import pytest
import yaml


def _make_mobile_spec(cfg: dict) -> object:
    from arena_robots.caps import MobileSpec

    return MobileSpec(path=Path("/tmp/mobile.yaml"), raw={"external": cfg})


def _make_robot(external_cfg: dict, name: str = "ext_robot") -> object:
    mobile_spec = _make_mobile_spec(external_cfg)
    mock_caps = MagicMock()
    mock_caps.mobile = mobile_spec
    mock_robot = MagicMock()
    mock_robot.name = name
    mock_robot.caps = mock_caps
    return mock_robot


class TestExternalBringupAttributes:
    def test_kind_value(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        assert ExternalBringup.kind == "external"


class TestExternalBringupCfg:
    def test_cfg_returns_external_sub_block(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        robot = _make_robot({"launch_file": "/path/to/launch.py"})
        b = ExternalBringup(robot=robot, namespace="/ns")
        assert b._cfg == {"launch_file": "/path/to/launch.py"}

    def test_launch_file_present(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        robot = _make_robot({"launch_file": "/my/launch.py"})
        b = ExternalBringup(robot=robot, namespace="/ns")
        assert b.launch_file == "/my/launch.py"

    def test_launch_file_missing_raises_key_error(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        robot = _make_robot({})
        b = ExternalBringup(robot=robot, namespace="/ns")
        with pytest.raises(KeyError):
            _ = b.launch_file

    def test_requires_from_cfg(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        robot = _make_robot({"launch_file": "x.py", "requires": ["mobile", "arm"]})
        b = ExternalBringup(robot=robot, namespace="/ns")
        assert b.requires == frozenset({"mobile", "arm"})

    def test_requires_default_mobile(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        robot = _make_robot({"launch_file": "x.py"})
        b = ExternalBringup(robot=robot, namespace="/ns")
        assert b.requires == frozenset({"mobile"})

    def test_extra_empty_by_default(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        robot = _make_robot({"launch_file": "x.py"})
        b = ExternalBringup(robot=robot, namespace="/ns")
        assert b.extra == {}

    def test_extra_dict_returned(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        robot = _make_robot({"launch_file": "x.py", "extra": {"speed": 1.5}})
        b = ExternalBringup(robot=robot, namespace="/ns")
        assert b.extra == {"speed": 1.5}


class TestExternalBringupTopics:
    def test_goal_topic_contains_namespace(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        robot = _make_robot({"launch_file": "x.py"})
        b = ExternalBringup(robot=robot, namespace="/myns")
        assert "goal_pose" in b.goal_topic
        assert "myns" in b.goal_topic

    def test_cmd_vel_topic_contains_namespace(self):
        from arena_robots.bringup.mobile.external import ExternalBringup

        robot = _make_robot({"launch_file": "x.py"})
        b = ExternalBringup(robot=robot, namespace="/myns")
        assert "cmd_vel" in b.cmd_vel_topic
        assert "myns" in b.cmd_vel_topic


class TestExternalBringupLaunchActions:
    def _make(self, extra_cfg: dict | None = None) -> object:
        from arena_robots.bringup.mobile.external import ExternalBringup

        cfg = {"launch_file": "/launch/ext.py"}
        if extra_cfg:
            cfg.update(extra_cfg)
        robot = _make_robot(cfg)
        return ExternalBringup(robot=robot, namespace="/ns")

    def test_returns_list(self):
        b = self._make()
        actions = b._launch_actions()
        assert isinstance(actions, list)
        assert len(actions) == 1

    def test_use_sim_time_false(self):
        b = self._make()
        actions = b._launch_actions(use_sim_time=False)
        args = dict(actions[0].launch_arguments)
        assert args["use_sim_time"] == "false"

    def test_use_sim_time_true(self):
        b = self._make()
        actions = b._launch_actions(use_sim_time=True)
        args = dict(actions[0].launch_arguments)
        assert args["use_sim_time"] == "true"

    def test_extra_args_included_as_strings(self):
        b = self._make(extra_cfg={"extra": {"speed": 2}})
        actions = b._launch_actions()
        args = dict(actions[0].launch_arguments)
        assert args.get("speed") == "2"

    def test_namespace_in_args(self):
        b = self._make()
        actions = b._launch_actions()
        args = dict(actions[0].launch_arguments)
        assert "ns" in args["namespace"]

    def test_extra_kwargs_ignored(self):
        b = self._make()
        actions = b._launch_actions(unknown="ignored")
        assert isinstance(actions, list)
