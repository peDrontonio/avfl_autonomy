#!/usr/bin/env python3
"""
Mission 1 Launch File - ArUco Gate Passing

This launch file starts the mission-specific nodes:
1. ArUco detector node
2. Master mission1 controller node

Note: Launch Gazebo and drone navigation services separately:
    ros2 launch drone_gazebo simple_world.launch.py
    ros2 launch drone_navigate drone_services.launch.py

Usage:
    ros2 launch drone_gazebo mission1.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Mission 1."""

    takeoff_alt_arg = DeclareLaunchArgument(
        'takeoff_alt',
        default_value='10.0',
        description='Takeoff altitude in meters above ground'
    )

    # ArUco Detector Node
    aruco_detector = Node(
        package='drone_navigate',
        executable='aruco_node',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'camera_topic': '/camera_frente',
        }]
    )

    # Master Mission1 Controller Node
    master_mission1 = Node(
        package='drone_navigate',
        executable='master_mission1',
        name='master_mission1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'takeoff_alt': LaunchConfiguration('takeoff_alt'),
        }]
    )

    return LaunchDescription([
        takeoff_alt_arg,
        aruco_detector,
        master_mission1,
    ])
