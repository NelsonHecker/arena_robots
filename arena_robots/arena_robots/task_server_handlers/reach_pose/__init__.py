"""Type alias for ``TaskKind.REACH_POSE`` handler implementations.

Handler registration lives on the Bringup subclass (``task_handlers`` ClassVar)
in ``arena_robots.bringup.arm.<kind>``; this module only exposes the protocol
alias used by handler implementations.
"""

from __future__ import annotations

from arena_robots_msgs.action import ReachPose

from arena_robots.task_server_handlers import TaskHandler

ReachPoseHandler = TaskHandler[ReachPose.Goal, ReachPose.Feedback, ReachPose.Result]
