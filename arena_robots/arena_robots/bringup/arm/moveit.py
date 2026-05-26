from __future__ import annotations

import json
from typing import ClassVar

from launch import Action
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.task_kinds import TaskKind


def _load_reach_pose_moveit() -> type:
    from arena_robots.task_server_handlers.reach_pose.moveit import ReachPoseHandlerMoveIt

    return ReachPoseHandlerMoveIt


def _load_play_gesture_moveit() -> type:
    from arena_robots.task_server_handlers.play_gesture.moveit import PlayGestureHandlerMoveIt

    return PlayGestureHandlerMoveIt


@BringupMeta.attach(requires={"arm"}, cap="arm")
class MoveItArmBringup(Bringup):
    kind = "moveit"
    task_handlers: ClassVar[dict] = {
        TaskKind.REACH_POSE: _load_reach_pose_moveit,
        TaskKind.PLAY_GESTURE: _load_play_gesture_moveit,
    }

    def _launch_actions(self, *, use_sim_time: bool = True, frame: str = "", **launch_args: object) -> list[Action]:
        arms = self.robot.caps.arm
        if arms is None:
            raise ValueError(f"{self.robot.name}: arm cap required but absent")
        if len(arms) != 1:
            raise NotImplementedError("Multi-arm robots not yet supported")
        (arm,) = arms.values()
        mv = arm.raw.get("moveit") or {}
        if not mv.get("package"):
            raise ValueError(f"{arm.path}: arm '{arm.name}' missing moveit.package")
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("arena_robots"),
                            "launch",
                            "adapters",
                            "arm",
                            "moveit.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "robot": self.robot.name,
                    "namespace": str(self.namespace),
                    "frame": frame,
                    "use_sim_time": str(use_sim_time).lower(),
                    "arm_controller": arm.controller,
                    "arm_joints_json": json.dumps(list(arm.chain)),
                }.items(),
            ),
        ]
