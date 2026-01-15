"""
Usage:
    # First, start the simulation:
    ros2 launch drone_gazebo simple_world.launch.py

    # Then, via terminal, arm and takeoff:
    ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
    ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
    ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 10.0}"

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    square_side_arg = DeclareLaunchArgument(
        'square_side',
        default_value='5.0',
        description='Size of the square side in meters'
    )
    
    velocity_arg = DeclareLaunchArgument(
        'velocity',
        default_value='1.0',
        description='Movement velocity in m/s'
    )
    
    # Mission node
    simple_mission_node = Node(
        package='drone_gazebo',
        executable='simple_mission.py',
        name='simple_mission_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'square_side': LaunchConfiguration('square_side'),
            'velocity': LaunchConfiguration('velocity'),
        }]
    )
    
    return LaunchDescription([
        LogInfo(msg=' Starting Simple Mission'),
        square_side_arg,
        velocity_arg,
        simple_mission_node,
    ])
