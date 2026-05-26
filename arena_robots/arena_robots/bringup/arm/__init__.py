from arena_robots.bringup import BRINGUPS, Bringup


@BRINGUPS["arm"].register("none")
def _load_none() -> type[Bringup]:
    from .none import NoneArmBringup

    return NoneArmBringup


@BRINGUPS["arm"].register("moveit")
def _load_moveit() -> type[Bringup]:
    from .moveit import MoveItArmBringup

    return MoveItArmBringup
