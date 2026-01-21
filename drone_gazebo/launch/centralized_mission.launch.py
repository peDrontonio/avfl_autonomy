from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='=== Starting HUD Mission with Adaptive Search ==='),
        
        DeclareLaunchArgument('search_velocity', default_value='1.2'),
        DeclareLaunchArgument('search_distance', default_value='4.0'), # Bigger search area
        DeclareLaunchArgument('advance_distance', default_value='1.5'), # Move forward more
        DeclareLaunchArgument('fly_through_distance', default_value='5.0'),

        Node(
            package='drone_gazebo',
            executable='aruco_detector_new.py',
            name='drone_tracker',
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='drone_gazebo',
            executable='centralized_mission.py',
            name='centralized_mission',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'search_velocity': LaunchConfiguration('search_velocity'),
                'search_distance': LaunchConfiguration('search_distance'),
                'advance_distance': LaunchConfiguration('advance_distance'),
                'centering_velocity': 0.3,
                'fly_velocity': 1.5,
                'centering_threshold_x': 0.1,
                'centering_threshold_y': 0.1,
                'default_fly_distance': LaunchConfiguration('fly_through_distance'),
            }]
        )
    ])