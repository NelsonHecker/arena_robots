"""Drive `planner_server` from `<ns>/goal_pose`.

`planner_server` only publishes `/plan` as a side effect of the
`compute_path_to_pose` action. When we run nav2 in `planner_only` mode there is
no `bt_navigator` calling it, so this node bridges PoseStamped goals to the
action.
"""

from __future__ import annotations

import math

import rclpy
from arena_rclpy_mixins.spin import spin_node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

_DEDUPE_POS_TOL: float = 0.05
_DEDUPE_YAW_TOL: float = 0.05


def _yaw_from_quat(q: object) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalToPlanBridge(Node):
    def __init__(self) -> None:
        super().__init__("goal_to_plan_bridge")

        goal_topic = self.declare_parameter("goal_topic", "goal_pose").value
        action_name = self.declare_parameter("action_name", "compute_path_to_pose").value
        self._planner_id = self.declare_parameter("planner_id", "").value

        self._last_goal: PoseStamped | None = None
        self._client: ActionClient = ActionClient(self, ComputePathToPose, action_name)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, qos)

    def _on_goal(self, msg: PoseStamped) -> None:
        if self._last_goal is not None and self._same_goal(self._last_goal, msg):
            return
        self._last_goal = msg
        if not self._client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning("compute_path_to_pose action server not available; goal queued, will retry on next message")
            return
        goal = ComputePathToPose.Goal()
        goal.goal = msg
        goal.use_start = False
        if self._planner_id:
            goal.planner_id = self._planner_id
        self._client.send_goal_async(goal)

    @staticmethod
    def _same_goal(a: PoseStamped, b: PoseStamped) -> bool:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        if math.hypot(dx, dy) > _DEDUPE_POS_TOL:
            return False
        return abs(_yaw_from_quat(a.pose.orientation) - _yaw_from_quat(b.pose.orientation)) <= _DEDUPE_YAW_TOL


def main() -> None:
    rclpy.init()
    node = GoalToPlanBridge()
    try:
        spin_node(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
