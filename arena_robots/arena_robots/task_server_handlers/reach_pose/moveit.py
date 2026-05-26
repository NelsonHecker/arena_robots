from __future__ import annotations

from typing import TYPE_CHECKING

from action_msgs.msg import GoalStatus
from arena_robots_msgs.action import ReachPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

from arena_robots.task_server_handlers import _executor_sleep

if TYPE_CHECKING:
    from arena_robots.bringup.arm.moveit import MoveItArmBringup

_DEFAULT_POSITION_TOLERANCE = 0.01
_DEFAULT_ORIENTATION_TOLERANCE = 0.1
_DEFAULT_PLANNING_TIME = 5.0


def _translate_moveit_status(code: int) -> tuple[int, str]:
    if code == MoveItErrorCodes.SUCCESS:
        return ReachPose.Result.STATUS_SUCCEEDED, ""
    if code == MoveItErrorCodes.NO_IK_SOLUTION:
        return ReachPose.Result.STATUS_NO_IK, "MoveIt: no IK solution"
    if code == MoveItErrorCodes.PLANNING_FAILED:
        return ReachPose.Result.STATUS_NO_PLAN, "MoveIt: planning failed"
    if code == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return ReachPose.Result.STATUS_NO_PLAN, "MoveIt: motion plan invalid"
    if code == MoveItErrorCodes.TIMED_OUT:
        return ReachPose.Result.STATUS_TIMEOUT, "MoveIt: timed out"
    return ReachPose.Result.STATUS_ABORTED, f"MoveIt: error code {code}"


class ReachPoseHandlerMoveIt:
    def __init__(self, bringup: MoveItArmBringup, *, tf_buffer: object, node: object) -> None:
        self._bringup = bringup
        self._tf_buffer = tf_buffer
        self._node = node
        action_name = bringup.namespace("move_action")
        self._native_client = ActionClient(node, MoveGroup, str(action_name))
        self._tf_prefix = bringup.frame

    async def execute(self, goal_handle: object) -> ReachPose.Result:
        arena_goal: ReachPose.Goal = goal_handle.request
        result = ReachPose.Result()

        arms = self._bringup.robot.caps.arm
        if arms is None:
            raise ValueError(f"{self._bringup.robot.name}: arm cap required but absent")
        if len(arms) != 1:
            result.status = ReachPose.Result.STATUS_ABORTED
            result.reason = f"expected exactly 1 arm cap, got {len(arms)}"
            return result
        (arm,) = arms.values()

        mg_goal = self._build_goal(arena_goal, arm)
        if mg_goal is None:
            goal_handle.abort()
            result.status = ReachPose.Result.STATUS_UNKNOWN_NAMED
            result.reason = f"named pose {arena_goal.named_target!r} not in caps/arm.yaml.named_poses"
            return result

        while not self._native_client.server_is_ready():
            if not goal_handle.is_active:
                result.status = ReachPose.Result.STATUS_CANCELED
                result.reason = "goal canceled by client"
                return result
            await _executor_sleep(self._node, 0.1, wall=True)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.status = ReachPose.Result.STATUS_CANCELED
            result.reason = "goal canceled by client"
            return result

        send_future = self._native_client.send_goal_async(mg_goal)
        native_handle = await send_future

        if not native_handle.accepted:
            goal_handle.abort()
            result.status = ReachPose.Result.STATUS_ABORTED
            result.reason = "MoveIt MotionPlanRequest aborted"
            return result

        wrapped = await native_handle.get_result_async()
        if not goal_handle.is_active:
            result.status = ReachPose.Result.STATUS_CANCELED
            result.reason = "goal canceled by client"
            return result

        if wrapped.result is not None:
            arena_status, arena_reason = _translate_moveit_status(wrapped.result.error_code.val)
        else:
            arena_status, arena_reason = ReachPose.Result.STATUS_ABORTED, "MoveIt MotionPlanRequest aborted"
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            arena_status = ReachPose.Result.STATUS_CANCELED
            arena_reason = "goal canceled by client"

        if arena_status == ReachPose.Result.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif arena_status == ReachPose.Result.STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        result.status = arena_status
        result.reason = arena_reason
        return result

    def _build_goal(self, arena_goal: ReachPose.Goal, arm: object) -> MoveGroup.Goal | None:
        constraints = Constraints()

        if arena_goal.named_target:
            joints = arm.named_poses.get(arena_goal.named_target)
            if joints is None:
                return None
            for joint_name, position in joints.items():
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = float(position)
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
        else:
            pos_tol = arena_goal.position_tolerance or _DEFAULT_POSITION_TOLERANCE
            ori_tol = arena_goal.orientation_tolerance or _DEFAULT_ORIENTATION_TOLERANCE

            pc = PositionConstraint()
            pc.header = arena_goal.target.header
            pc.link_name = self._tf_prefix + arm.tip_link
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = [max(float(pos_tol), 0.001)]
            pc.constraint_region.primitives.append(prim)
            pc.constraint_region.primitive_poses.append(arena_goal.target.pose)
            pc.weight = 1.0
            constraints.position_constraints.append(pc)

            oc = OrientationConstraint()
            oc.header = arena_goal.target.header
            oc.link_name = self._tf_prefix + arm.tip_link
            oc.orientation = arena_goal.target.pose.orientation
            oc.absolute_x_axis_tolerance = float(ori_tol)
            oc.absolute_y_axis_tolerance = float(ori_tol)
            oc.absolute_z_axis_tolerance = float(ori_tol)
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)

        mg_goal = MoveGroup.Goal()
        mg_goal.request = MotionPlanRequest()
        mg_goal.request.group_name = arm.planning_group or arm.name
        mg_goal.request.goal_constraints.append(constraints)
        mg_goal.request.allowed_planning_time = float(arena_goal.planning_time or _DEFAULT_PLANNING_TIME)
        mg_goal.request.num_planning_attempts = 5
        mg_goal.request.max_velocity_scaling_factor = 1.0
        mg_goal.request.max_acceleration_scaling_factor = 1.0
        # is_diff=true means "use the current robot state as start; ignore the (empty) joint_state I'm sending".
        # Without this, MoveIt warns every goal that it's ignoring our supplied start state.
        mg_goal.request.start_state.is_diff = True
        ws = arm.workspace
        if ws is not None and ws.get("type") == "box":
            mg_goal.request.workspace_parameters.header.frame_id = self._tf_prefix + str(ws.get("frame", "base_link"))
            mg_goal.request.workspace_parameters.min_corner.x = float(ws["min"][0])
            mg_goal.request.workspace_parameters.min_corner.y = float(ws["min"][1])
            mg_goal.request.workspace_parameters.min_corner.z = float(ws["min"][2])
            mg_goal.request.workspace_parameters.max_corner.x = float(ws["max"][0])
            mg_goal.request.workspace_parameters.max_corner.y = float(ws["max"][1])
            mg_goal.request.workspace_parameters.max_corner.z = float(ws["max"][2])
        mg_goal.planning_options.plan_only = False
        return mg_goal
