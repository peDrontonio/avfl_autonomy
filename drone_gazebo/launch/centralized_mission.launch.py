from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='=== Starting Mission: Pattern Search (0 -> +6 -> 0 -> -6 -> 0) ==='),
        
        # Search Distance: 6.0 meters
        DeclareLaunchArgument('search_distance', default_value='6.0'), 
        DeclareLaunchArgument('advance_distance', default_value='0.5'), 
        
        DeclareLaunchArgument('search_velocity', default_value='1.0'),
        DeclareLaunchArgument('fly_through_distance', default_value='5.0'),
        
        # New parameters: start_direction ('left' or 'right') and pass_velocity
        DeclareLaunchArgument('start_direction', default_value='right',
                             description='Initial search direction: left or right'),
        DeclareLaunchArgument('pass_velocity', default_value='1.5',
                             description='Velocity when flying through the arc'),

        Node(
            package='drone_gazebo',
            executable='aruco.py',
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
                'centering_velocity': 0.15,  # Slower for stability
                'fly_velocity': LaunchConfiguration('pass_velocity'),
                'centering_threshold_x': 0.1,
                'centering_threshold_y': 0.1,
                'partial_centering_threshold': 0.8,  # Very tolerant - less trust in pixels
                'default_fly_distance': LaunchConfiguration('fly_through_distance'),
                'start_direction': LaunchConfiguration('start_direction'),
            }]
        )
    ])