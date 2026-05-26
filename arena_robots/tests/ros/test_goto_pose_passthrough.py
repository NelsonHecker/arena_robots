"""Tests for arena_robots.task_server_handlers.goto_pose._passthrough."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace
from unittest.mock import MagicMock, patch


def _make_goal_handle(target_pose=None):
    if target_pose is None:
        try:
            from geometry_msgs.msg import PoseStamped

            target_pose = PoseStamped()
        except ImportError:
            target_pose = MagicMock()
    goal = MagicMock()
    goal.target = target_pose
    handle = MagicMock()
    handle.request = goal
    handle.succeed = MagicMock()
    return handle


def _make_bringup(goal_topic: str = "/robot1/goal_pose") -> object:
    b = MagicMock()
    b.goal_topic = goal_topic
    return b


def _make_node():
    publisher = MagicMock()
    node = MagicMock()
    node.create_publisher.return_value = publisher
    return node, publisher


class TestPassthroughHandlerInit:
    def test_creates_publisher(self):
        from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerNone

        node, publisher = _make_node()
        bringup = _make_bringup("/ns/goal_pose")
        handler = GotoPoseHandlerNone(bringup, tf_buffer=None, node=node)
        node.create_publisher.assert_called_once()
        call_args = node.create_publisher.call_args
        assert "/ns/goal_pose" in str(call_args) or call_args[0][1] == "/ns/goal_pose"


class TestPassthroughHandlerExecute:
    def _make_handler(self, goal_topic: str = "/ns/goal_pose"):
        from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerNone

        node, publisher = _make_node()
        bringup = _make_bringup(goal_topic)
        handler = GotoPoseHandlerNone(bringup, tf_buffer=None, node=node)
        return handler, publisher

    def test_publishes_target(self):
        handler, publisher = self._make_handler()
        goal_handle = _make_goal_handle()
        asyncio.run(handler.execute(goal_handle))
        publisher.publish.assert_called_once_with(goal_handle.request.target)

    def test_calls_succeed(self):
        handler, publisher = self._make_handler()
        goal_handle = _make_goal_handle()
        asyncio.run(handler.execute(goal_handle))
        goal_handle.succeed.assert_called_once()

    def test_result_status_succeeded(self):
        from arena_robots_msgs.action import GotoPose

        handler, publisher = self._make_handler()
        goal_handle = _make_goal_handle()
        result = asyncio.run(handler.execute(goal_handle))
        assert result.status == GotoPose.Result.STATUS_SUCCEEDED

    def test_result_final_pose_is_target(self):
        try:
            from geometry_msgs.msg import PoseStamped

            target = PoseStamped()
        except ImportError:
            target = MagicMock()

        from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerNone

        node, _ = _make_node()
        bringup = _make_bringup()
        handler = GotoPoseHandlerNone(bringup, tf_buffer=None, node=node)
        goal = MagicMock()
        goal.target = target
        goal_handle = MagicMock()
        goal_handle.request = goal
        result = asyncio.run(handler.execute(goal_handle))
        assert result.final_pose is target


class TestPassthroughHandlerExternal:
    def test_external_variant_also_works(self):
        from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerExternal

        node, publisher = _make_node()
        bringup = _make_bringup("/ext/goal_pose")
        handler = GotoPoseHandlerExternal(bringup, tf_buffer=None, node=node)
        goal_handle = _make_goal_handle()
        result = asyncio.run(handler.execute(goal_handle))
        publisher.publish.assert_called_once()
        goal_handle.succeed.assert_called_once()
