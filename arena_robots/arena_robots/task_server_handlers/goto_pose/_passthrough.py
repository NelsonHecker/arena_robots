from __future__ import annotations

from typing import TYPE_CHECKING

from arena_robots_msgs.action import GotoPose
from geometry_msgs.msg import PoseStamped

if TYPE_CHECKING:
    from arena_robots.bringup.mobile.drl import DrlBringup
    from arena_robots.bringup.mobile.external import ExternalBringup
    from arena_robots.bringup.mobile.none import NoneBringup
    from arena_robots.bringup.mobile.rosnav_rl import RosnavRlBringup


class _PassthroughHandler:
    """Publishes the goal to a topic and immediately succeeds.

    Shared body of the ``none``, ``external``, ``rosnav_rl``, and ``drl`` bringups, all
    treat the arena ``GotoPose`` action as a fire-and-forget goal-pose publish."""

    def __init__(self, bringup: NoneBringup | ExternalBringup | RosnavRlBringup | DrlBringup, *, tf_buffer: object, node: object) -> None:
        self._pub = node.create_publisher(PoseStamped, bringup.goal_topic, 1)

    async def execute(self, goal_handle: object) -> GotoPose.Result:
        arena_goal: GotoPose.Goal = goal_handle.request
        self._pub.publish(arena_goal.target)
        goal_handle.succeed()
        result = GotoPose.Result()
        result.status = GotoPose.Result.STATUS_SUCCEEDED
        result.reason = ""
        result.final_pose = arena_goal.target
        return result


class GotoPoseHandlerNone(_PassthroughHandler):
    pass


class GotoPoseHandlerExternal(_PassthroughHandler):
    pass


class GotoPoseHandlerRosnavRl(_PassthroughHandler):
    pass


class GotoPoseHandlerDrl(_PassthroughHandler):
    pass
