"""Drive `planner_server` from `<ns>/goal_pose`.

`planner_server` only publishes `/plan` as a side effect of the
`compute_path_to_pose` action. When we run nav2 in `planner_only` mode there is
no `bt_navigator` calling it, so this node bridges PoseStamped goals to the
action and re-issues them so `/plan` tracks the robot as it moves.
"""

from __future__ import annotations

import rclpy
from arena_rclpy_mixins.spin import spin_node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class GoalToPlanBridge(Node):
    def __init__(self) -> None:
        super().__init__("goal_to_plan_bridge")

        goal_topic = self.declare_parameter("goal_topic", "goal_pose").value
        action_name = self.declare_parameter("action_name", "compute_path_to_pose").value
        replan_period = float(self.declare_parameter("replan_period", 1.0).value)
        self._planner_id = self.declare_parameter("planner_id", "").value

        self._goal: PoseStamped | None = None
        self._in_flight: bool = False
        self._client: ActionClient = ActionClient(self, ComputePathToPose, action_name)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, qos)
        self.create_timer(replan_period, self._replan)

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg

    def _replan(self) -> None:
        if self._goal is None or self._in_flight:
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            return
        goal = ComputePathToPose.Goal()
        goal.goal = self._goal
        goal.use_start = False
        if self._planner_id:
            goal.planner_id = self._planner_id
        self._in_flight = True
        self._client.send_goal_async(goal).add_done_callback(self._on_sent)

    def _on_sent(self, _future: object) -> None:
        self._in_flight = False


def main() -> None:
    rclpy.init()
    node = GoalToPlanBridge()
    try:
        spin_node(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
