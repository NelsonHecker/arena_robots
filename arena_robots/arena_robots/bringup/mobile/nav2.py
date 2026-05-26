from __future__ import annotations

from typing import ClassVar

from launch import Action
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.task_kinds import TaskKind


def _load_goto_pose_nav2() -> type:
    from arena_robots.task_server_handlers.goto_pose.nav2 import GotoPoseHandlerNav2

    return GotoPoseHandlerNav2


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class Nav2Bringup(Bringup):
    kind = "nav2"
    task_handlers: ClassVar[dict] = {TaskKind.GOTO_POSE: _load_goto_pose_nav2}

    @property
    def native_action_name(self) -> str:
        return self.namespace("navigate_to_pose")

    @property
    def bt_node_name(self) -> str:
        return self.namespace("bt_navigator")

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        global_planner: str = "navfn",
        local_planner: str = "regulated_pure_pursuit",
        inter_planner: str = "navigate_w_replanning_time",
        train_mode: bool = False,
        task_generator_node: str = "",
        **_: object,
    ) -> list[Action]:
        launch_file = PathJoinSubstitution(
            [
                FindPackageShare("arena_robots"),
                "launch",
                "adapters",
                "mobile",
                "nav2.launch.py",
            ]
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "robot": self.robot.name,
                    "namespace": self.namespace,
                    "use_sim_time": str(use_sim_time).lower(),
                    "frame": frame,
                    "global_planner": global_planner,
                    "local_planner": local_planner,
                    "inter_planner": inter_planner,
                    "train_mode": str(train_mode).lower(),
                    "task_generator_node": task_generator_node,
                }.items(),
            )
        ]
