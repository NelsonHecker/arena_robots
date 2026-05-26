from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Iterable
from typing import TYPE_CHECKING, ClassVar

import attrs

if TYPE_CHECKING:
    from arena_robots.Robot import RobotView
    from arena_robots.task_server_handlers import TaskHandler

from arena_rclpy_mixins.shared import Namespace
from launch import Action

from arena_robots.task_kinds import TaskKind


class AdapterCapMismatch(RuntimeError):
    pass


def _to_frozenset_str(v: str | Iterable[str]) -> frozenset[str]:
    if isinstance(v, str):
        return frozenset({v})
    return frozenset(v)


@attrs.frozen
class BringupMeta:
    """Static metadata attached to a Bringup class."""

    requires: frozenset[str] = attrs.field(converter=_to_frozenset_str)
    cap: str = ""

    @classmethod
    def attach(cls, **kwargs: object) -> Callable[[type], type]:
        meta = cls(**kwargs)

        def wrap(target: type) -> type:
            target._bringup_meta = meta
            return target

        return wrap


class Bringup(ABC):
    kind: ClassVar[str]
    task_handlers: ClassVar[dict[TaskKind, Callable[[], type[TaskHandler]]]] = {}

    def __init__(self, robot: RobotView, namespace: str, *, frame: str = "") -> None:
        self.robot = robot
        self.namespace = Namespace(namespace)
        self.frame = frame

    @property
    def cap(self) -> str:
        return self._bringup_meta.cap

    @abstractmethod
    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        **launch_args: object,
    ) -> list[Action]: ...

    @property
    def requires(self) -> frozenset[str]:
        return self._bringup_meta.requires

    @property
    def accepts_task_kinds(self) -> frozenset[TaskKind]:
        return frozenset(self.task_handlers.keys())


def check_caps(bringup: Bringup) -> None:
    available = bringup.robot.caps.available
    missing = bringup.requires - available
    if missing:
        raise AdapterCapMismatch(f"Bringup {bringup.kind!r} requires caps {sorted(missing)} but robot {bringup.robot.name!r} only advertises {sorted(available)}")


from arena_rclpy_mixins.registry import ClassRegistry

BRINGUPS: dict[str, ClassRegistry[str, type[Bringup]]] = {
    "mobile": ClassRegistry(),
    "arm": ClassRegistry(),
}

from . import arm, mobile  # noqa: F401, E402
