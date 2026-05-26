from __future__ import annotations

from typing import ClassVar

from launch import Action
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.task_kinds import TaskKind


def _load_goto_pose_external() -> type:
    from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerExternal

    return GotoPoseHandlerExternal


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class ExternalBringup(Bringup):
    kind = "external"
    task_handlers: ClassVar[dict] = {TaskKind.GOTO_POSE: _load_goto_pose_external}

    @property
    def _cfg(self) -> dict:
        return self.robot.caps.mobile.sub("external")

    @property
    def requires(self) -> frozenset[str]:
        return frozenset(self._cfg.get("requires", ["mobile"]))

    @property
    def goal_topic(self) -> str:
        return self.namespace("goal_pose")

    @property
    def cmd_vel_topic(self) -> str:
        return self.namespace("cmd_vel")

    @property
    def launch_file(self) -> str:
        return self._cfg["launch_file"]

    @property
    def extra(self) -> dict:
        return self._cfg.get("extra", {})

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        **_: object,
    ) -> list[Action]:
        args: dict[str, str] = {
            "goal_topic": self.goal_topic,
            "cmd_vel_topic": self.cmd_vel_topic,
            "namespace": self.namespace,
            "use_sim_time": str(use_sim_time).lower(),
            "frame": frame,
            **{k: str(v) for k, v in self.extra.items()},
        }
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(self.launch_file),
                launch_arguments=args.items(),
            )
        ]
