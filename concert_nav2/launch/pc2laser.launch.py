"""
    Convert a Velodyne PointCloud2 to LaserScan
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('concert_nav2')
    params_file = os.path.join(pkg, 'config', 'pointcloud_to_laserscan.yaml')

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/VLP16_lidar_front/points',
        description='Input PointCloud2 topic to convert',
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Output LaserScan topic',
    )

    node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_topic')),
            ('scan', LaunchConfiguration('scan_topic')),
        ],
    )

    return LaunchDescription([cloud_topic_arg, scan_topic_arg, node])
