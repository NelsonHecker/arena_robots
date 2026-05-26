"""ROS node that exposes per-TaskKind action servers for a robot's configured bringups."""

import threading
from typing import cast

import rclpy
import tf2_ros
from arena_rclpy_mixins.spin import spin_node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.parameter import Parameter

from arena_robots.bringup import BRINGUPS, check_caps
from arena_robots.Robot import RobotIdentifier
from arena_robots.task_kinds import TaskKind, action_type, endpoint


class TaskServerNode(Node):
    def __init__(self) -> None:
        super().__init__("task_server")

        robot_name = self.declare_parameter("robot_name", "").value
        bringup_caps = cast(list[str], self.declare_parameter("bringup_caps", Parameter.Type.STRING_ARRAY).value)
        bringup_kinds = cast(list[str], self.declare_parameter("bringup_kinds", Parameter.Type.STRING_ARRAY).value)
        frame = cast(str, self.declare_parameter("frame", "").value)

        if not robot_name:
            raise RuntimeError("Parameter 'robot_name' is required")
        if len(bringup_caps) != len(bringup_kinds):
            raise RuntimeError(f"Parameter length mismatch: bringup_caps={len(bringup_caps)} bringup_kinds={len(bringup_kinds)}")
        if not bringup_caps:
            self.get_logger().warning(f"no bringups configured for robot {robot_name!r}; task_server idle")

        namespace = self.get_namespace()
        robot = RobotIdentifier(robot_name).resolve_sync()

        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer, self)

        # Single-goal-per-TaskKind: a new accepted goal preempts the previous.
        # The preempted handler sees ``goal_handle.is_active == False`` and
        # bails without retrying, so the newest goal is the only one nav2 sees.
        self._current_handles: dict[TaskKind, ServerGoalHandle] = {}
        self._handle_lock = threading.Lock()

        self._bringups: list[object] = []
        self._servers: list[ActionServer] = []

        for cap, kind in zip(bringup_caps, bringup_kinds, strict=True):
            try:
                bringup_cls = BRINGUPS[cap].get(kind)
            except KeyError:
                self.get_logger().error(f"no bringup registered for ({cap!r}, {kind!r}); skipping")
                continue

            try:
                bringup = bringup_cls(robot, namespace, frame=frame)
                check_caps(bringup)
            except Exception as exc:
                self.get_logger().error(f"bringup ({cap!r}, {kind!r}) init failed: {exc}; skipping")
                continue

            self._bringups.append(bringup)

            for tk in bringup.accepts_task_kinds:
                loader = bringup.task_handlers.get(tk)
                if loader is None:
                    self.get_logger().warning(f"no handler for ({tk!r}, {kind!r}); skipping endpoint")
                    continue
                handler_cls = loader()
                try:
                    handler = handler_cls(bringup, tf_buffer=self._tf_buffer, node=self)
                    server = ActionServer(
                        self,
                        action_type(tk),
                        endpoint(namespace, tk),
                        execute_callback=handler.execute,
                        handle_accepted_callback=self._make_handle_accepted(tk),
                    )
                except Exception as exc:
                    self.get_logger().error(f"handler ({tk!r}, {kind!r}) init failed: {exc}; skipping endpoint")
                    continue
                self._servers.append(server)

    def _make_handle_accepted(self, tk: TaskKind) -> object:
        def _handle_accepted(goal_handle: ServerGoalHandle) -> None:
            with self._handle_lock:
                prev = self._current_handles.get(tk)
                self._current_handles[tk] = goal_handle
            if prev is not None and prev.is_active:
                prev.abort()
            goal_handle.execute()

        return _handle_accepted


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    spin_node(TaskServerNode())


if __name__ == "__main__":
    main()
