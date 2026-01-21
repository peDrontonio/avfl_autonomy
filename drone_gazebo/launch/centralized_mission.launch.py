"""
Centralized Mission Launch File (Modular)

Launches:
1. aruco_detector_new.py (Sensors - 4 Marker Lock)
2. centralized_mission.py (Control - Dynamic Distance)

Usage:
    ros2 launch drone_gazebo centralized_mission.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    search_velocity_arg = DeclareLaunchArgument(
        'search_velocity', default_value='1.0', description='Search velocity (m/s)'
    )
    centering_velocity_arg = DeclareLaunchArgument(
        'centering_velocity', default_value='0.3', description='Centering velocity (m/s)'
    )
    fly_velocity_arg = DeclareLaunchArgument(
        'fly_velocity', default_value='1.0', description='Fly through velocity (m/s)'
    )
    search_distance_arg = DeclareLaunchArgument(
        'search_distance', default_value='3.0', description='Search distance (m)'
    )
    # Thresholds in Meters
    centering_threshold_x_arg = DeclareLaunchArgument(
        'centering_threshold_x', default_value='0.1', description='X Threshold (m)'
    )
    centering_threshold_y_arg = DeclareLaunchArgument(
        'centering_threshold_y', default_value='0.1', description='Y Threshold (m)'
    )
    # This argument sets the fallback distance if sensors fail
    fly_through_distance_arg = DeclareLaunchArgument(
        'fly_through_distance', default_value='5.0', description='Default Fly distance (fallback) (m)'
    )
    
    # 1. Detector Node (The "Eyes")
    detector_node = Node(
        package='drone_gazebo',
        executable='aruco_detector_new.py',
        name='drone_tracker',
        output='screen',
        emulate_tty=True
    )

    # 2. Mission Node (The "Brain")
    mission_node = Node(
        package='drone_gazebo',
        executable='centralized_mission.py',
        name='centralized_mission_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'search_velocity': LaunchConfiguration('search_velocity'),
            'centering_velocity': LaunchConfiguration('centering_velocity'),
            'fly_velocity': LaunchConfiguration('fly_velocity'),
            'search_distance': LaunchConfiguration('search_distance'),
            'centering_threshold_x': LaunchConfiguration('centering_threshold_x'),
            'centering_threshold_y': LaunchConfiguration('centering_threshold_y'),
            # CORRECTED MAPPING HERE:
            'default_fly_distance': LaunchConfiguration('fly_through_distance'),
        }]
    )
    
    return LaunchDescription([
        LogInfo(msg='=== Starting 4-Marker Centralized Mission ==='),
        search_velocity_arg,
        centering_velocity_arg,
        fly_velocity_arg,
        search_distance_arg,
        centering_threshold_x_arg,
        centering_threshold_y_arg,
        fly_through_distance_arg,
        detector_node,
        mission_node,
    ])