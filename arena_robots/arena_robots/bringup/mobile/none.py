from __future__ import annotations

from typing import ClassVar

from launch import Action
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.task_kinds import TaskKind


def _load_goto_pose_none() -> type:
    from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerNone

    return GotoPoseHandlerNone


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class NoneBringup(Bringup):
    kind = "none"
    task_handlers: ClassVar[dict] = {TaskKind.GOTO_POSE: _load_goto_pose_none}

    @property
    def goal_topic(self) -> str:
        return self.namespace("goal_pose")

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        **_: object,
    ) -> list[Action]:
        launch_file = PathJoinSubstitution(
            [
                FindPackageShare("arena_robots"),
                "launch",
                "adapters",
                "mobile",
                "none.launch.py",
            ]
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
            )
        ]
