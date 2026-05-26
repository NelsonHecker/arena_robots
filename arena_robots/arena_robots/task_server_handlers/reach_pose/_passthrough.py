from __future__ import annotations

from typing import TYPE_CHECKING

from arena_robots_msgs.action import ReachPose

if TYPE_CHECKING:
    from arena_robots.bringup.arm.none import NoneArmBringup


class ReachPoseHandlerNone:
    """Immediately succeeds without motion planning. Used by the arm/none bringup."""

    def __init__(self, bringup: NoneArmBringup, *, tf_buffer: object, node: object) -> None:
        pass

    async def execute(self, goal_handle: object) -> ReachPose.Result:
        goal_handle.succeed()
        result = ReachPose.Result()
        result.status = ReachPose.Result.STATUS_SUCCEEDED
        return result
