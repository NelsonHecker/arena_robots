"""Single source of truth for task kinds and their public endpoint names."""

import enum

from arena_rclpy_mixins.shared import Namespace


class TaskKind(enum.Enum):
    GOTO_POSE = "goto_pose"
    REACH_POSE = "reach_pose"
    PLAY_GESTURE = "play_gesture"


PUBLIC_SUFFIX: dict[TaskKind, str] = {
    TaskKind.GOTO_POSE: "goto_pose",
    TaskKind.REACH_POSE: "reach_pose",
    TaskKind.PLAY_GESTURE: "play_gesture",
}

TASK_KIND_CAP: dict[TaskKind, str] = {
    TaskKind.GOTO_POSE: "mobile",
    TaskKind.REACH_POSE: "arm",
    TaskKind.PLAY_GESTURE: "arm",
}


def cap_for_task_kind(tk: TaskKind) -> str:
    return TASK_KIND_CAP[tk]


def action_type(tk: TaskKind) -> type:
    if tk is TaskKind.GOTO_POSE:
        from arena_robots_msgs.action import GotoPose

        return GotoPose
    if tk is TaskKind.REACH_POSE:
        from arena_robots_msgs.action import ReachPose

        return ReachPose
    if tk is TaskKind.PLAY_GESTURE:
        from arena_robots_msgs.action import PlayGesture

        return PlayGesture
    raise KeyError(tk)


def endpoint(namespace: str, tk: TaskKind) -> str:
    return Namespace(namespace)(PUBLIC_SUFFIX[tk])
