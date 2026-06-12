import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('concert_nav2')
    cfg = os.path.join(pkg, 'config', 'tracking')

    detector_yaml = os.path.join(cfg, 'aruco_detector.yaml')
    landmarks_yaml = os.path.join(cfg, 'aruco_landmarks.yaml')
    ekf_yaml = os.path.join(cfg, 'ekf.yaml')

    map_yaml = os.path.join(pkg, 'maps', 'cantamessa', 'ground_floor', 'map.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def detector(cam):
        return Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name=f'aruco_tracker_{cam}',
            namespace=f'/D435i_camera_{cam}',
            output='screen',
            parameters=[
                detector_yaml,
                {
                    'use_sim_time': use_sim_time,
                    'cam_base_topic': f'/D435i_camera_{cam}/color/image_raw',
                },
            ],
            remappings=[
                (f'/D435i_camera_{cam}/color/camera_info',
                 f'/D435i_camera_{cam}/camera_info'),
            ],
        )

    aruco_localization = Node(
        package='concert_nav2',
        executable='aruco_localization',
        name='aruco_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'landmarks_file': landmarks_yaml,
            'detection_topics': [
                '/D435i_camera_front/aruco_detections',
                '/D435i_camera_back/aruco_detections',
            ],
            'marker_yaw_offset': 1.5708,
            'max_range': 8.0,
            'max_view_angle_deg': 65.0,
            'gate_enabled': True,
            'gate_xy': 0.50,
            'gate_yaw': 0.35,
            'gate_growth_per_sec': 0.20,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        detector('front'),
        detector('back'),
        aruco_localization,
        ekf,
        map_server,
        lifecycle_manager,
    ])
