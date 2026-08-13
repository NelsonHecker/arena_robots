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
    node.get_namespace.return_value = "/env_0/robot1"
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


class TestGoalWindowHandlerExternal:
    def _make_handler(self, transform):
        from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerExternal

        node, publisher = _make_node()
        bringup = _make_bringup("/ext/goal_pose")
        bringup.frame = "env_0/"
        bringup.robot.model_params.base_frame = "base_link"
        tf_buffer = MagicMock()
        tf_buffer.lookup_transform.return_value = transform
        handler = GotoPoseHandlerExternal(bringup, tf_buffer=tf_buffer, node=node)
        return handler, publisher

    def _make_transform(self, x: float = 0.0, y: float = 0.0):
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.w = 1.0
        return t

    def _make_goal_handle(self, *, pose_tolerance: float = 0.5):
        goal_handle = _make_goal_handle()
        goal_handle.is_active = True
        goal_handle.is_cancel_requested = False
        goal_handle.request.pose_tolerance = pose_tolerance
        goal_handle.request.yaw_tolerance = 0.0
        return goal_handle

    def test_publishes_target(self):
        handler, publisher = self._make_handler(self._make_transform())
        goal_handle = self._make_goal_handle()
        asyncio.run(handler.execute(goal_handle))
        publisher.publish.assert_called_once_with(goal_handle.request.target)

    def test_succeeds_once_within_tolerance(self):
        from arena_robots_msgs.action import GotoPose

        handler, _ = self._make_handler(self._make_transform(0.1, 0.0))
        goal_handle = self._make_goal_handle()
        result = asyncio.run(handler.execute(goal_handle))
        goal_handle.succeed.assert_called_once()
        assert result.status == GotoPose.Result.STATUS_SUCCEEDED

    def test_final_pose_is_measured_pose(self):
        handler, _ = self._make_handler(self._make_transform(0.1, 0.0))
        goal_handle = self._make_goal_handle()
        result = asyncio.run(handler.execute(goal_handle))
        assert result.final_pose.pose.position.x == 0.1

    def test_cancel_requested_returns_canceled(self):
        from arena_robots_msgs.action import GotoPose

        handler, _ = self._make_handler(self._make_transform())
        goal_handle = self._make_goal_handle()
        goal_handle.is_cancel_requested = True
        result = asyncio.run(handler.execute(goal_handle))
        goal_handle.canceled.assert_called_once()
        goal_handle.succeed.assert_not_called()
        assert result.status == GotoPose.Result.STATUS_CANCELED
