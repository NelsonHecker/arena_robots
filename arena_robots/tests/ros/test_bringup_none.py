"""Tests for arena_robots.bringup.mobile.none - NoneBringup."""

from __future__ import annotations

from unittest.mock import MagicMock


def _make_mock_robot(name: str = "test_robot") -> object:
    mock_robot = MagicMock()
    mock_robot.name = name
    return mock_robot


class TestNoneBringupAttributes:
    def test_kind_value(self):
        from arena_robots.bringup.mobile.none import NoneBringup

        assert NoneBringup.kind == "none"

    def test_requires_contains_mobile(self):
        from arena_robots.bringup.mobile.none import NoneBringup

        assert "mobile" in NoneBringup._bringup_meta.requires

    def test_requires_frozenset(self):
        from arena_robots.bringup.mobile.none import NoneBringup

        assert isinstance(NoneBringup._bringup_meta.requires, frozenset)


class TestNoneBringupProperties:
    def _make(self, namespace: str = "/robot1") -> object:
        from arena_robots.bringup.mobile.none import NoneBringup

        return NoneBringup(robot=_make_mock_robot(), namespace=namespace)

    def test_goal_topic_contains_goal_pose(self):
        b = self._make()
        assert "goal_pose" in b.goal_topic

    def test_goal_topic_contains_namespace(self):
        b = self._make("/ns1")
        assert "ns1" in b.goal_topic


class TestNoneBringupLaunchActions:
    def _make(self) -> object:
        from arena_robots.bringup.mobile.none import NoneBringup

        return NoneBringup(robot=_make_mock_robot(), namespace="/robot1")

    def test_returns_list(self):
        b = self._make()
        actions = b._launch_actions()
        assert isinstance(actions, list)
        assert len(actions) == 1

    def test_use_sim_time_accepted(self):
        b = self._make()
        actions = b._launch_actions(use_sim_time=False)
        assert isinstance(actions, list)

    def test_extra_kwargs_ignored(self):
        b = self._make()
        actions = b._launch_actions(extra_kwarg="ignored")
        assert isinstance(actions, list)
