from __future__ import annotations

from typing import ClassVar

from launch import Action
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.task_kinds import TaskKind


def _load_goto_pose_drl() -> type:
    from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerDrl

    return GotoPoseHandlerDrl


# caps/mobile.yaml sub-block consumed by this bringup (key: "drl"):
#
#   drl:
#     planner: str            # required, planner name, e.g. drlvo
#     observations: dict      # optional, default {} — free-form obs pipeline overrides
#       aliases: dict[str, str]
#     rate: float             # optional, default 10.0 — per-tick obs rate in Hz
#     obs_policy: str         # optional, default "lossless" — "lossless" | "latest_only"
#     goal_topic: str         # optional, default ~/goal_pose
#     cmd_vel_topic: str      # optional, default ~/cmd_vel
#     global_planner: str     # optional, default "nav2/navfn" — "<family>/<kind>" or "none"
#     namespace: str          # optional, default ""
#     use_sim_time: bool      # optional, default true
#
# The DrlAdapter (task_generator adapters/mobile/drl.py) owns subprocess lifecycle;
# this bringup spawns an optional global planner stack (nav2 planner_server-only by default).


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class DrlBringup(Bringup):
    kind = "drl"
    task_handlers: ClassVar[dict] = {TaskKind.GOTO_POSE: _load_goto_pose_drl}

    @property
    def _cfg(self) -> dict:
        return self.robot.caps.mobile.sub("drl")

    @property
    def planner(self) -> str:
        v = self._cfg.get("planner")
        if not v:
            raise ValueError(f"drl bringup for '{self.robot.name}' missing required 'planner': set caps/mobile.yaml 'drl.planner' or pass mobile.planner:=<name>")
        return str(v)

    @property
    def observations(self) -> dict:
        v = self._cfg.get("observations", {})
        if not isinstance(v, dict):
            raise ValueError(f"drl bringup for '{self.robot.name}': 'drl.observations' must be a mapping")
        return v

    @property
    def rate(self) -> float:
        return float(self._cfg.get("rate", 10.0))

    @property
    def obs_policy(self) -> str:
        v = str(self._cfg.get("obs_policy", "lossless"))
        if v not in {"lossless", "latest_only"}:
            raise ValueError(f"drl bringup for '{self.robot.name}': 'drl.obs_policy' must be 'lossless' or 'latest_only'; got {v!r}")
        return v

    @property
    def goal_topic(self) -> str:
        v = self._cfg.get("goal_topic")
        return str(v) if v else self.namespace("goal_pose")

    @property
    def cmd_vel_topic(self) -> str:
        v = self._cfg.get("cmd_vel_topic")
        return str(v) if v else self.namespace("cmd_vel")

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        global_planner: str = "nav2/navfn",
        task_generator_node: str = "",
        **_: object,
    ) -> list[Action]:
        from arena_planners.resolver import (  # noqa: PLC0415
            ResolverError,
            resolve_global_planner,
            split_global_planner,
        )

        try:
            parsed = split_global_planner(global_planner)
        except ValueError as exc:
            raise RuntimeError(f"drl bringup for '{self.robot.name}': invalid mobile.global_planner: {exc}") from exc

        if parsed is None:
            return []

        family, kind = parsed
        try:
            launch_path, _metadata = resolve_global_planner(family)
        except ResolverError as exc:
            raise RuntimeError(f"drl bringup for '{self.robot.name}': {exc}") from exc

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_path)),
                launch_arguments={
                    "robot": self.robot.name,
                    "namespace": self.namespace,
                    "use_sim_time": str(use_sim_time).lower(),
                    "frame": frame,
                    "kind": kind,
                    "task_generator_node": task_generator_node,
                }.items(),
            )
        ]
