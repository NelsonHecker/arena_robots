"""MoveIt arm adapter launch."""

import json

from arena_robots.moveit_factory import build_moveit_params
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace

from arena_bringup.substitutions import LaunchArgument


def generate_launch_description():
    ld_items = []
    LaunchArgument.auto_append(ld_items)

    robot = LaunchArgument("robot")
    namespace = LaunchArgument("namespace")
    frame = LaunchArgument("frame")
    use_sim_time = LaunchArgument("use_sim_time")
    arm_controller = LaunchArgument("arm_controller")
    arm_joints_json = LaunchArgument("arm_joints_json")

    def launch_setup(context, *args, **kwargs):
        ns = namespace.substitution.perform(context)
        robot_name = robot.substitution.perform(context)
        sim = use_sim_time.substitution.perform(context).lower() == "true"
        controller_name = arm_controller.substitution.perform(context)
        joints = json.loads(arm_joints_json.substitution.perform(context))

        tf_prefix = frame.substitution.perform(context)

        moveit_params = build_moveit_params(robot_name, tf_prefix=tf_prefix)
        if moveit_params is None:
            raise ValueError(f"{robot_name}: missing 'arm' cap or moveit.package; cannot launch moveit adapter")

        controllers_params = {
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_simple_controller_manager": {
                "controller_names": [controller_name],
                controller_name: {
                    "action_ns": "follow_joint_trajectory",
                    "type": "FollowJointTrajectory",
                    "default": True,
                    "joints": list(joints),
                },
            },
            "trajectory_execution": {
                "allowed_execution_duration_scaling": 1.2,
                "allowed_goal_duration_margin": 0.5,
                "allowed_start_tolerance": 0.1,
            },
        }

        move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_params,
                controllers_params,
                {"use_sim_time": sim},
            ],
        )

        return [PushRosNamespace(ns), move_group]

    return LaunchDescription([*ld_items, OpaqueFunction(function=launch_setup)])
