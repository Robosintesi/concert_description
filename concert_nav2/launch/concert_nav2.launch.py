"""
    ToDo: Nav2 Simulation BringUp
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    concert_nav2_pkg = get_package_share_directory('concert_nav2')
    concert_gazebo_pkg = get_package_share_directory('concert_gazebo')
    concert_odom_pkg = get_package_share_directory('concert_odometry_ros2')

    # ---- args ----
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='slam', choices=['slam', 'localization'],
        description='Run SLAM Toolbox (online mapping) or localization (map+AMCL)')
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Map .yaml absolute path (only used when mode:=localization)')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution(
            [FindPackageShare('concert_gazebo'), 'world', 'small_warehouse.sdf']),
        description='Absolute path to the Gazebo SDF world file')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    xbot2_gui_arg = DeclareLaunchArgument('xbot2_gui', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    xbot2_gui = LaunchConfiguration('xbot2_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Gazebo + CONCERT (xbot2 + omnisteering plugin live in xbot2 config) ----
    gazebo_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(concert_gazebo_pkg, 'launch', 'modular.launch.xml')),
        launch_arguments={
            'gazebo': 'true',
            'xbot2': 'true',
            'xbot2_gui': xbot2_gui,
            'velodyne': 'true',
            'use_gpu_ray': 'false',
            'gui': gui,
            'use_sim_time': use_sim_time,
            'world_file': world,
            'rviz': 'false',
        }.items(),
    )

    # ---- Wheel odometry (publishes odom -> base_link) ----
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(concert_odom_pkg, 'launch', 'concert_odometry.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_ground_truth': 'false',
            'gui': 'false',
        }.items(),
    )

    # ---- PointCloud2 -> LaserScan ----
    pc2laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(concert_nav2_pkg, 'launch', 'pc2laser.launch.py')),
        launch_arguments={
            'cloud_topic': '/VLP16_lidar_front/points',
            'scan_topic': '/scan',
        }.items(),
    )

    # ---- Map provider: SLAM Toolbox or map_server+AMCL ----
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(concert_nav2_pkg, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=LaunchConfigurationEquals('mode', 'slam'),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(concert_nav2_pkg, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'localization'),
    )

    # ---- Nav2 core ----
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(concert_nav2_pkg, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': '/omnisteering/cmd_vel',
        }.items(),
    )

    # ---- RViz ----
    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='nav2_rviz',
        output='log',
        arguments=['-d', os.path.join(concert_nav2_pkg, 'rviz', 'concert_nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        mode_arg,
        map_arg,
        world_arg,
        gui_arg,
        rviz_arg,
        xbot2_gui_arg,
        use_sim_time_arg,
        gazebo_launch,
        odometry_launch,
        pc2laser_launch,
        GroupAction([slam_launch, localization_launch]),
        nav2_launch,
        rviz_node,
    ])
