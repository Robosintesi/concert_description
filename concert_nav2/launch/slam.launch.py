"""
    SLAM Toolbox (async, lifecycle) for online mapping.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('concert_nav2')
    default_params = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_toolbox),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    activate = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_toolbox),
        transition_id=Transition.TRANSITION_ACTIVATE,
    ))

    # Configure as soon as the node is up; activate once configured.
    on_inactive = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_toolbox,
        start_state='configuring',
        goal_state='inactive',
        entities=[activate],
    ))

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_toolbox,
        on_inactive,
        configure,
    ])
