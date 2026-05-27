"""
    Nav2 localization
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('concert_nav2')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    map_file_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map .yaml. If empty, map_server starts unconfigured.')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time, 'yaml_filename': map_file},
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        map_file_arg,
        autostart_arg,
        map_server,
        amcl,
        lifecycle_manager,
    ])
