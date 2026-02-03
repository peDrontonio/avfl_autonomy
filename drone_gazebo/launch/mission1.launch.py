#!/usr/bin/env python3
"""
Mission 1 Launch File - Gate Passing with ArUco Detection

This launch file starts everything needed for Mission 1:
1. Gazebo simulation with simple_world (gate and ArUco markers)
2. Drone navigation services (navigate, get_telemetry, set_yaw_rate)
3. ArUco detector node
4. Master mission node (optional, can be started separately)

Usage:
    ros2 launch drone_gazebo mission1.launch.py
    
Optional arguments:
    rviz:=false          # Disable RViz
    auto_start:=true     # Auto-start the mission (default: false)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for Mission 1."""
    
    # Declare arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start the mission (call /start_mission1 service)'
    )
    
    # 1. Launch simple_world (Gazebo + Drone)
    simple_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('drone_gazebo'),
                'launch',
                'simple_world.launch.py'
            ])
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz')
        }.items()
    )
    
    # 2. Launch drone navigation services
    drone_services = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('drone_navigate'),
                'launch',
                'drone_services.launch.py'
            ])
        )
    )

    
    return LaunchDescription([
        rviz_arg,
        auto_start_arg,
        simple_world,
        drone_services,
        # mission_starter,  # Uncomment if you create the mission_starter node
    ])
