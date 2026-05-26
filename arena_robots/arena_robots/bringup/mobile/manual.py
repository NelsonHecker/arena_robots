from __future__ import annotations

from typing import ClassVar

import launch_ros.actions
from launch import Action

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.task_kinds import TaskKind


def _load_goto_pose_manual() -> type:
    from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerNone

    return GotoPoseHandlerNone


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class ManualBringup(Bringup):
    kind = "manual"
    task_handlers: ClassVar[dict] = {TaskKind.GOTO_POSE: _load_goto_pose_manual}

    @property
    def goal_topic(self) -> str:
        return self.namespace("goal_pose")

    @property
    def cmd_vel_topic(self) -> str:
        return self.namespace("cmd_vel")

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        **_: object,
    ) -> list[Action]:
        return [
            launch_ros.actions.Node(
                package="rqt_robot_steering",
                executable="rqt_robot_steering",
                name="rqt_robot_steering",
                output="screen",
                remappings=[("/cmd_vel", str(self.cmd_vel_topic))],
            )
        ]
