"""Tests for arena_robots.bringup.mobile.nav2 - Nav2Bringup."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest


def _make_mock_robot(name: str = "test_robot") -> object:
    mock_robot = MagicMock()
    mock_robot.name = name
    return mock_robot


class TestNav2BringupAttributes:
    def test_kind_value(self):
        from arena_robots.bringup.mobile.nav2 import Nav2Bringup

        assert Nav2Bringup.kind == "nav2"

    def test_requires_mobile(self):
        from arena_robots.bringup.mobile.nav2 import Nav2Bringup

        assert "mobile" in Nav2Bringup._bringup_meta.requires

    def test_requires_frozenset(self):
        from arena_robots.bringup.mobile.nav2 import Nav2Bringup

        assert isinstance(Nav2Bringup._bringup_meta.requires, frozenset)


class TestNav2BringupProperties:
    def setup_method(self):
        robot = _make_mock_robot("myrobot")
        from arena_robots.bringup.mobile.nav2 import Nav2Bringup

        self.b = Nav2Bringup(robot=robot, namespace="/robot1")

    def test_native_action_name_contains_namespace(self):
        ep = self.b.native_action_name
        assert "navigate_to_pose" in ep
        assert "robot1" in ep

    def test_bt_node_name_contains_namespace(self):
        ep = self.b.bt_node_name
        assert "bt_navigator" in ep
        assert "robot1" in ep


class TestNav2LaunchActions:
    def _make_bringup(self, namespace: str = "/robot1"):
        robot = _make_mock_robot("myrobot")
        from arena_robots.bringup.mobile.nav2 import Nav2Bringup

        return Nav2Bringup(robot=robot, namespace=namespace)

    def test_returns_list(self):
        b = self._make_bringup()
        actions = b._launch_actions()
        assert isinstance(actions, list)
        assert len(actions) == 1

    def test_use_sim_time_true_lowercase(self):
        b = self._make_bringup()
        actions = b._launch_actions(use_sim_time=True)
        action = actions[0]
        args = dict(action.launch_arguments)
        assert args["use_sim_time"] == "true"

    def test_use_sim_time_false_lowercase(self):
        b = self._make_bringup()
        actions = b._launch_actions(use_sim_time=False)
        action = actions[0]
        args = dict(action.launch_arguments)
        assert args["use_sim_time"] == "false"

    def test_planner_arg_forwarded(self):
        b = self._make_bringup()
        actions = b._launch_actions(global_planner="NavFn")
        args = dict(actions[0].launch_arguments)
        assert args["global_planner"] == "NavFn"

    def test_local_planner_arg_forwarded(self):
        b = self._make_bringup()
        actions = b._launch_actions(local_planner="TEB")
        args = dict(actions[0].launch_arguments)
        assert args["local_planner"] == "TEB"

    def test_inter_planner_arg_forwarded(self):
        b = self._make_bringup()
        actions = b._launch_actions(inter_planner="smac")
        args = dict(actions[0].launch_arguments)
        assert args["inter_planner"] == "smac"

    def test_default_planners(self):
        b = self._make_bringup()
        actions = b._launch_actions()
        args = dict(actions[0].launch_arguments)
        assert args["global_planner"] == "navfn"
        assert args["local_planner"] == "regulated_pure_pursuit"
        assert args["inter_planner"] == "default"

    def test_extra_kwargs_ignored(self):
        b = self._make_bringup()
        actions = b._launch_actions(unknown_kwarg="ignored")
        assert isinstance(actions, list)

    def test_robot_name_forwarded(self):
        b = self._make_bringup()
        actions = b._launch_actions()
        args = dict(actions[0].launch_arguments)
        assert args["robot"] == "myrobot"

    def test_namespace_forwarded(self):
        b = self._make_bringup()
        actions = b._launch_actions()
        args = dict(actions[0].launch_arguments)
        assert "robot1" in args["namespace"]
