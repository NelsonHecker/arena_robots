"""Type alias for ``TaskKind.GOTO_POSE`` handler implementations.

Handler registration lives on the Bringup subclass (``task_handlers`` ClassVar)
in ``arena_robots.bringup.mobile.<kind>``; this module only exposes the protocol
alias used by handler implementations.
"""

from __future__ import annotations

from arena_robots_msgs.action import GotoPose

from arena_robots.task_server_handlers import TaskHandler

GotoPoseHandler = TaskHandler[GotoPose.Goal, GotoPose.Feedback, GotoPose.Result]
