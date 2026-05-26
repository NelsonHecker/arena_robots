"""Tests for arena_robots.task_server_handlers.goto_pose.nav2."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

import pytest


def _make_bringup(native_action_name: str = "/robot1/navigate_to_pose"):
    b = MagicMock()
    b.native_action_name = native_action_name
    return b


def _make_node():
    node = MagicMock()
    node.get_logger.return_value = MagicMock(info=lambda m: None, warn=lambda m: None)
    return node


class TestTranslateNav2Status:
    def test_succeeded(self):
        from action_msgs.msg import GoalStatus

        from arena_robots.task_server_handlers.goto_pose.nav2 import _translate_nav2_status
        from arena_robots_msgs.action import GotoPose

        status, reason = _translate_nav2_status(GoalStatus.STATUS_SUCCEEDED)
        assert status == GotoPose.Result.STATUS_SUCCEEDED
        assert reason == ""

    def test_canceled(self):
        from action_msgs.msg import GoalStatus

        from arena_robots.task_server_handlers.goto_pose.nav2 import _translate_nav2_status
        from arena_robots_msgs.action import GotoPose

        status, reason = _translate_nav2_status(GoalStatus.STATUS_CANCELED)
        assert status == GotoPose.Result.STATUS_CANCELED
        assert reason

    def test_aborted_on_unknown(self):
        from arena_robots.task_server_handlers.goto_pose.nav2 import _translate_nav2_status
        from arena_robots_msgs.action import GotoPose

        status, reason = _translate_nav2_status(99)
        assert status == GotoPose.Result.STATUS_ABORTED
        assert reason

    def test_aborted_on_executing(self):
        from action_msgs.msg import GoalStatus

        from arena_robots.task_server_handlers.goto_pose.nav2 import _translate_nav2_status
        from arena_robots_msgs.action import GotoPose

        status, reason = _translate_nav2_status(GoalStatus.STATUS_EXECUTING)
        assert status == GotoPose.Result.STATUS_ABORTED
        assert reason


class TestGotoPoseHandlerNav2Init:
    def test_creates_action_client(self):
        from arena_robots.task_server_handlers.goto_pose.nav2 import GotoPoseHandlerNav2

        bringup = _make_bringup()
        node = _make_node()
        with patch("arena_robots.task_server_handlers.goto_pose.nav2.ActionClient") as MockAC:
            handler = GotoPoseHandlerNav2(bringup, tf_buffer=None, node=node)
            MockAC.assert_called_once()


class TestGotoPoseHandlerNav2Execute:
    def _make_handler(self, node=None):
        from arena_robots.task_server_handlers.goto_pose.nav2 import GotoPoseHandlerNav2

        bringup = _make_bringup()
        if node is None:
            node = _make_node()
        with patch("arena_robots.task_server_handlers.goto_pose.nav2.ActionClient"):
            handler = GotoPoseHandlerNav2(bringup, tf_buffer=None, node=node)
        return handler

    def _make_goal_handle(self, *, is_active=True, is_cancel_requested=False):
        try:
            from geometry_msgs.msg import PoseStamped

            target = PoseStamped()
        except ImportError:
            target = MagicMock()
        goal = MagicMock()
        goal.target = target
        gh = MagicMock()
        gh.request = goal
        gh.is_active = is_active
        gh.is_cancel_requested = is_cancel_requested
        gh.publish_feedback = MagicMock()
        gh.succeed = MagicMock()
        gh.canceled = MagicMock()
        return gh

    def test_not_active_during_server_wait_returns_canceled(self):
        from arena_robots_msgs.action import GotoPose

        handler = self._make_handler()
        handler._native_client = MagicMock()
        handler._native_client.server_is_ready.return_value = False

        goal_handle = self._make_goal_handle(is_active=False)
        result = asyncio.run(handler.execute(goal_handle))
        assert result.status == GotoPose.Result.STATUS_CANCELED

    def test_cancel_requested_returns_canceled(self):
        from arena_robots_msgs.action import GotoPose

        handler = self._make_handler()
        handler._native_client = MagicMock()
        handler._native_client.server_is_ready.return_value = True

        goal_handle = self._make_goal_handle(is_cancel_requested=True)
        result = asyncio.run(handler.execute(goal_handle))
        assert result.status == GotoPose.Result.STATUS_CANCELED
        goal_handle.canceled.assert_called_once()

    def test_not_active_after_server_ready_returns_canceled(self):
        from arena_robots_msgs.action import GotoPose

        handler = self._make_handler()
        handler._native_client = MagicMock()
        server_ready_calls = [True]

        def server_is_ready():
            return True

        handler._native_client.server_is_ready.side_effect = server_is_ready

        call_count = 0

        def is_active_prop():
            nonlocal call_count
            call_count += 1
            return call_count <= 1

        goal_handle = self._make_goal_handle()
        type(goal_handle).is_active = property(lambda self: not call_count > 0)

        goal_handle2 = MagicMock()
        goal_handle2.is_active = False
        goal_handle2.is_cancel_requested = False
        goal_handle2.request = MagicMock()
        from geometry_msgs.msg import PoseStamped

        goal_handle2.request.target = PoseStamped()
        goal_handle2.publish_feedback = MagicMock()

        result = asyncio.run(handler.execute(goal_handle2))
        assert result.status == GotoPose.Result.STATUS_CANCELED

    def test_nav2_goal_accepted_and_succeeded(self):
        from action_msgs.msg import GoalStatus

        from arena_robots_msgs.action import GotoPose

        handler = self._make_handler()
        handler._native_client = MagicMock()
        handler._native_client.server_is_ready.return_value = True

        nav2_result = MagicMock()
        from geometry_msgs.msg import PoseStamped

        nav2_result.final_pose = PoseStamped()
        wrapped = MagicMock()
        wrapped.status = GoalStatus.STATUS_SUCCEEDED
        wrapped.result = nav2_result

        nav2_gh = MagicMock()
        nav2_gh.accepted = True
        nav2_gh.get_result_async = AsyncMock(return_value=wrapped)

        send_future = asyncio.get_event_loop().create_future() if False else AsyncMock(return_value=nav2_gh)
        handler._native_client.send_goal_async = send_future

        goal_handle = self._make_goal_handle(is_active=True)
        result = asyncio.run(handler.execute(goal_handle))
        assert result.status == GotoPose.Result.STATUS_SUCCEEDED
        goal_handle.succeed.assert_called_once()

    def test_nav2_goal_rejected_retries(self):
        from action_msgs.msg import GoalStatus

        from arena_robots_msgs.action import GotoPose

        handler = self._make_handler()
        handler._native_client = MagicMock()
        handler._native_client.server_is_ready.return_value = True

        nav2_result = MagicMock()
        from geometry_msgs.msg import PoseStamped

        nav2_result.final_pose = PoseStamped()
        wrapped = MagicMock()
        wrapped.status = GoalStatus.STATUS_SUCCEEDED
        wrapped.result = nav2_result

        accepted_gh = MagicMock()
        accepted_gh.accepted = True
        accepted_gh.get_result_async = AsyncMock(return_value=wrapped)

        rejected_gh = MagicMock()
        rejected_gh.accepted = False

        call_count = [0]

        async def mock_send_goal(goal, **kwargs):
            call_count[0] += 1
            if call_count[0] == 1:
                return rejected_gh
            return accepted_gh

        handler._native_client.send_goal_async = mock_send_goal

        destroyed = []

        async def mock_sleep(node, t, **kw):
            pass

        with patch("arena_robots.task_server_handlers.goto_pose.nav2._executor_sleep", mock_sleep):
            goal_handle = self._make_goal_handle(is_active=True)
            result = asyncio.run(handler.execute(goal_handle))
        assert call_count[0] == 2
        assert result.status == GotoPose.Result.STATUS_SUCCEEDED

    def test_feedback_translation(self):
        from action_msgs.msg import GoalStatus

        from arena_robots_msgs.action import GotoPose
        from geometry_msgs.msg import PoseStamped

        handler = self._make_handler()
        handler._native_client = MagicMock()
        handler._native_client.server_is_ready.return_value = True

        nav2_result = MagicMock()
        nav2_result.final_pose = PoseStamped()
        wrapped = MagicMock()
        wrapped.status = GoalStatus.STATUS_SUCCEEDED
        wrapped.result = nav2_result

        feedback_callbacks = []

        async def mock_send_goal(goal, feedback_callback=None, **kwargs):
            if feedback_callback:
                feedback_callbacks.append(feedback_callback)
                fb_inner = MagicMock()
                fb_inner.current_pose = PoseStamped()
                fb_inner.distance_remaining = 1.5
                eta = MagicMock()
                eta.sec = 3
                eta.nanosec = 0
                fb_inner.estimated_time_remaining = eta
                fb_msg = MagicMock()
                fb_msg.feedback = fb_inner
                feedback_callback(fb_msg)
            nav2_gh = MagicMock()
            nav2_gh.accepted = True
            nav2_gh.get_result_async = AsyncMock(return_value=wrapped)
            return nav2_gh

        handler._native_client.send_goal_async = mock_send_goal

        goal_handle = self._make_goal_handle()
        asyncio.run(handler.execute(goal_handle))
        goal_handle.publish_feedback.assert_called()
        fb_arg = goal_handle.publish_feedback.call_args[0][0]
        assert isinstance(fb_arg, GotoPose.Feedback)
        assert fb_arg.distance_remaining == pytest.approx(1.5)

    def test_final_pose_fallback_when_none(self):
        from action_msgs.msg import GoalStatus

        from arena_robots_msgs.action import GotoPose
        from geometry_msgs.msg import PoseStamped

        handler = self._make_handler()
        handler._native_client = MagicMock()
        handler._native_client.server_is_ready.return_value = True

        wrapped = MagicMock()
        wrapped.status = GoalStatus.STATUS_SUCCEEDED
        wrapped.result = None

        nav2_gh = MagicMock()
        nav2_gh.accepted = True
        nav2_gh.get_result_async = AsyncMock(return_value=wrapped)
        handler._native_client.send_goal_async = AsyncMock(return_value=nav2_gh)

        goal_handle = self._make_goal_handle()
        result = asyncio.run(handler.execute(goal_handle))
        assert isinstance(result.final_pose, PoseStamped)
