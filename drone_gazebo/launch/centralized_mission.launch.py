"""
Centralized Mission Launch File

This launch file starts the centralized_mission node for ArUco-based arch traversal.

Usage:
    # First, start the simulation:
    ros2 launch drone_gazebo simple_world.launch.py

    # Then, via terminal, arm and takeoff:
    ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
    ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
    ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 2.0}"

    # Wait for takeoff to complete, then launch the mission with default parameters:
    ros2 launch drone_gazebo centralized_mission.launch.py

    # Or launch with custom parameters:
    ros2 launch drone_gazebo centralized_mission.launch.py search_velocity:=1.5 search_distance:=4.0

Available Parameters:
    - search_velocity: Velocity for searching ArUco markers (default: 1.0 m/s)
    - centering_velocity: Velocity for centering on arch (default: 0.3 m/s)
    - fly_velocity: Velocity for flying through arch (default: 1.0 m/s)
    - search_distance: Distance to search before reversing direction (default: 3.0 m)
    - centering_threshold_x: Pixel threshold for X centering (default: 20.0 px)
    - centering_threshold_y: Pixel threshold for Y centering (default: 20.0 px)
    - fly_through_distance: Distance to fly through after centering (default: 5.0 m)

Examples:
    # Faster search with longer distance:
    ros2 launch drone_gazebo centralized_mission.launch.py search_velocity:=1.5 search_distance:=5.0
    
    # Slower, more precise centering:
    ros2 launch drone_gazebo centralized_mission.launch.py centering_velocity:=0.2 centering_threshold_x:=10.0
    
    # Quick pass through:
    ros2 launch drone_gazebo centralized_mission.launch.py fly_velocity:=2.0 fly_through_distance:=3.0

The drone will:
1. Search for ArUco markers by moving right/left
2. After 2 complete searches without finding markers, advance 0.5m forward and retry
3. Center on the arch using the detected markers
4. Fly through the centered arch
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    search_velocity_arg = DeclareLaunchArgument(
        'search_velocity',
        default_value='1.0',
        description='Velocity for searching ArUco markers (m/s)'
    )
    
    centering_velocity_arg = DeclareLaunchArgument(
        'centering_velocity',
        default_value='0.3',
        description='Velocity for centering on arch (m/s)'
    )
    
    fly_velocity_arg = DeclareLaunchArgument(
        'fly_velocity',
        default_value='1.0',
        description='Velocity for flying through arch (m/s)'
    )
    
    search_distance_arg = DeclareLaunchArgument(
        'search_distance',
        default_value='3.0',
        description='Distance to search before reversing direction (m)'
    )
    
    centering_threshold_x_arg = DeclareLaunchArgument(
        'centering_threshold_x',
        default_value='20.0',
        description='Pixel threshold for X centering'
    )
    
    centering_threshold_y_arg = DeclareLaunchArgument(
        'centering_threshold_y',
        default_value='20.0',
        description='Pixel threshold for Y centering'
    )
    
    fly_through_distance_arg = DeclareLaunchArgument(
        'fly_through_distance',
        default_value='5.0',
        description='Distance to fly through after centering (m)'
    )
    
    # Mission node
    centralized_mission_node = Node(
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
            'fly_through_distance': LaunchConfiguration('fly_through_distance'),
        }]
    )
    
    return LaunchDescription([
        LogInfo(msg='=== Starting Centralized Mission ==='),
        LogInfo(msg='Make sure the drone has already taken off before running this mission!'),
        search_velocity_arg,
        centering_velocity_arg,
        fly_velocity_arg,
        search_distance_arg,
        centering_threshold_x_arg,
        centering_threshold_y_arg,
        fly_through_distance_arg,
        centralized_mission_node,
    ])
