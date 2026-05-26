from __future__ import annotations

from typing import TYPE_CHECKING

import builtin_interfaces.msg
from action_msgs.msg import GoalStatus
from arena_robots_msgs.action import PlayGesture
from arena_simulation_setup.tree.Gesture import GestureIdentifier, GestureSpec
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arena_robots.task_server_handlers import _executor_sleep

if TYPE_CHECKING:
    from arena_robots.bringup.arm.moveit import MoveItArmBringup


def _translate_fjt_status(error_code: int) -> tuple[int, str]:
    if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
        return PlayGesture.Result.STATUS_SUCCEEDED, ""
    if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
        return PlayGesture.Result.STATUS_ABORTED, "FJT: PATH_TOLERANCE_VIOLATED"
    if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
        return PlayGesture.Result.STATUS_ABORTED, "FJT: GOAL_TOLERANCE_VIOLATED"
    if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
        return PlayGesture.Result.STATUS_ABORTED, "FJT: INVALID_GOAL"
    if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
        return PlayGesture.Result.STATUS_ABORTED, "FJT: INVALID_JOINTS"
    if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
        return PlayGesture.Result.STATUS_ABORTED, "FJT: OLD_HEADER_TIMESTAMP"
    return PlayGesture.Result.STATUS_ABORTED, f"FJT: error code {error_code}"


def _build_movegroup_goal(arm: object, joints: dict[str, float]) -> MoveGroup.Goal:
    constraints = Constraints()
    for joint_name, position in joints.items():
        jc = JointConstraint()
        jc.joint_name = joint_name
        jc.position = float(position)
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)

    mg = MoveGroup.Goal()
    mg.request = MotionPlanRequest()
    mg.request.group_name = arm.planning_group or arm.name
    mg.request.goal_constraints.append(constraints)
    mg.request.allowed_planning_time = 5.0
    mg.request.num_planning_attempts = 5
    mg.request.max_velocity_scaling_factor = 1.0
    mg.request.max_acceleration_scaling_factor = 1.0
    mg.request.start_state.is_diff = True
    mg.planning_options.plan_only = False
    return mg


def _translate_moveit_status(code: int) -> str:
    if code == MoveItErrorCodes.SUCCESS:
        return ""
    if code == MoveItErrorCodes.NO_IK_SOLUTION:
        return "MoveIt: no IK solution"
    if code == MoveItErrorCodes.PLANNING_FAILED:
        return "MoveIt: planning failed"
    if code == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return "MoveIt: motion plan invalid"
    if code == MoveItErrorCodes.TIMED_OUT:
        return "MoveIt: timed out"
    return f"MoveIt: error code {code}"


def _build_trajectory(gesture: GestureSpec, arm: object) -> JointTrajectory:
    chain: list[str] = arm.chain
    named_poses: dict[str, dict[str, float]] = arm.named_poses

    traj = JointTrajectory()
    traj.joint_names = chain

    base_t = gesture.keyframes[0].t
    for kf in gesture.keyframes[1:]:
        pose_joints = named_poses[kf.pose]
        pt = JointTrajectoryPoint()
        pt.positions = [pose_joints.get(j, 0.0) for j in chain]
        pt.velocities = [0.0] * len(chain)
        t_total = kf.t - base_t
        sec = int(t_total)
        pt.time_from_start = builtin_interfaces.msg.Duration(
            sec=sec,
            nanosec=int((t_total - sec) * 1e9),
        )
        traj.points.append(pt)

    return traj


class PlayGestureHandlerMoveIt:
    def __init__(self, bringup: MoveItArmBringup, *, tf_buffer: object, node: object) -> None:
        del tf_buffer
        self._bringup = bringup
        self._node = node
        self._mg_action = str(bringup.namespace("move_action"))
        self._mg_client = ActionClient(node, MoveGroup, self._mg_action)

    async def execute(self, goal_handle: object) -> PlayGesture.Result:
        arena_goal: PlayGesture.Goal = goal_handle.request
        result = PlayGesture.Result()

        arms = self._bringup.robot.caps.arm
        if arms is None:
            raise ValueError(f"{self._bringup.robot.name}: arm cap required but absent")
        if len(arms) != 1:
            result.status = PlayGesture.Result.STATUS_ABORTED
            result.reason = f"expected exactly 1 arm cap, got {len(arms)}"
            goal_handle.abort()
            return result
        (arm,) = arms.values()

        gesture: GestureSpec | None = arm.gestures.get(arena_goal.gesture)
        if gesture is None:
            try:
                gesture = GestureIdentifier(arena_goal.gesture).resolve_sync()
            except FileNotFoundError:
                goal_handle.abort()
                result.status = PlayGesture.Result.STATUS_UNKNOWN_GESTURE
                result.reason = f"gesture {arena_goal.gesture!r} not found in arm overrides or shared library"
                return result

        missing = gesture.required_poses() - set(arm.named_poses.keys())
        if missing:
            goal_handle.abort()
            result.status = PlayGesture.Result.STATUS_UNSUPPORTED
            result.reason = f"arm '{arm.name}' missing named poses: {sorted(missing)}"
            return result

        first_kf_joints = arm.named_poses[gesture.keyframes[0].pose]
        mg_goal = _build_movegroup_goal(arm, first_kf_joints)

        while not self._mg_client.server_is_ready():
            if not goal_handle.is_active:
                result.status = PlayGesture.Result.STATUS_CANCELED
                result.reason = "goal canceled by client"
                return result
            await _executor_sleep(self._node, 0.1, wall=True)

        mg_handle = await self._mg_client.send_goal_async(mg_goal)
        if not mg_handle.accepted:
            goal_handle.abort()
            result.status = PlayGesture.Result.STATUS_ABORTED
            result.reason = "MoveIt transition goal rejected"
            return result

        mg_wrapped = await mg_handle.get_result_async()
        if not goal_handle.is_active:
            result.status = PlayGesture.Result.STATUS_CANCELED
            result.reason = "goal canceled by client"
            return result
        if mg_wrapped.status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
            result.status = PlayGesture.Result.STATUS_CANCELED
            result.reason = "goal canceled by client"
            return result
        if mg_wrapped.result is None or mg_wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = mg_wrapped.result.error_code.val if mg_wrapped.result is not None else -1
            goal_handle.abort()
            result.status = PlayGesture.Result.STATUS_ABORTED
            result.reason = f"transition to first keyframe failed: {_translate_moveit_status(code)}"
            return result

        if len(gesture.keyframes) <= 1:
            goal_handle.succeed()
            result.status = PlayGesture.Result.STATUS_SUCCEEDED
            return result

        traj = _build_trajectory(gesture, arm)

        fjt_action = str(self._bringup.namespace(arm.controller, "follow_joint_trajectory"))
        fjt_client: ActionClient = ActionClient(self._node, FollowJointTrajectory, fjt_action)

        while not fjt_client.server_is_ready():
            if not goal_handle.is_active:
                result.status = PlayGesture.Result.STATUS_CANCELED
                result.reason = "goal canceled by client"
                return result
            await _executor_sleep(self._node, 0.1, wall=True)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.status = PlayGesture.Result.STATUS_CANCELED
            result.reason = "goal canceled by client"
            return result

        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.trajectory = traj

        fjt_handle = await fjt_client.send_goal_async(fjt_goal)
        if not fjt_handle.accepted:
            goal_handle.abort()
            result.status = PlayGesture.Result.STATUS_ABORTED
            result.reason = "FollowJointTrajectory goal rejected"
            return result

        fjt_wrapped = await fjt_handle.get_result_async()
        if not goal_handle.is_active:
            result.status = PlayGesture.Result.STATUS_CANCELED
            result.reason = "goal canceled by client"
            return result
        if fjt_wrapped.status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
            result.status = PlayGesture.Result.STATUS_CANCELED
            result.reason = "goal canceled by client"
            return result

        if fjt_wrapped.result is not None:
            arena_status, arena_reason = _translate_fjt_status(fjt_wrapped.result.error_code)
        else:
            arena_status = PlayGesture.Result.STATUS_ABORTED
            arena_reason = "FollowJointTrajectory aborted with no result"

        if arena_status == PlayGesture.Result.STATUS_SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result.status = arena_status
        result.reason = arena_reason
        return result
