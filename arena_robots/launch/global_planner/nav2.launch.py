"""nav2 family handler for `mobile.global_planner:=nav2/<kind>`.

Bootstraps a planner_server-only nav2 stack and a goal-pose -> compute_path_to_pose bridge.
"""

from arena_bringup.substitutions import LaunchArgument
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld_items = []
    LaunchArgument.auto_append(ld_items)

    robot = LaunchArgument('robot')
    task_generator_node = LaunchArgument('task_generator_node', default_value='')
    namespace = LaunchArgument('namespace')
    frame = LaunchArgument('frame')
    use_sim_time = LaunchArgument('use_sim_time')
    kind = LaunchArgument('kind')

    nav2_launch = PathJoinSubstitution([
        FindPackageShare('arena_robots'),
        'launch', 'adapters', 'mobile', 'nav2.launch.py',
    ])

    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'robot': robot.substitution,
            'namespace': namespace.substitution,
            'use_sim_time': use_sim_time.substitution,
            'frame': frame.substitution,
            'global_planner': kind.substitution,
            'local_planner': 'dwb',
            'inter_planner': 'empty',
            'train_mode': 'true',
            'planner_only': 'true',
            'task_generator_node': task_generator_node.substitution,
        }.items(),
    )

    bridge_node = Node(
        package='arena_robots',
        executable='goal_to_plan_bridge',
        name='goal_to_plan_bridge',
        namespace=namespace.substitution,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time.substitution}],
    )

    return LaunchDescription([*ld_items, nav2_include, bridge_node])
