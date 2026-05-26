from __future__ import annotations

from typing import ClassVar

from launch import Action

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.task_kinds import TaskKind


def _load_reach_pose_none() -> type:
    from arena_robots.task_server_handlers.reach_pose._passthrough import ReachPoseHandlerNone

    return ReachPoseHandlerNone


@BringupMeta.attach(requires={"arm"}, cap="arm")
class NoneArmBringup(Bringup):
    kind = "none"
    task_handlers: ClassVar[dict] = {TaskKind.REACH_POSE: _load_reach_pose_none}

    @property
    def goal_topic(self) -> str:
        return self.namespace("reach_pose_goal")

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        **_: object,
    ) -> list[Action]:
        return []
