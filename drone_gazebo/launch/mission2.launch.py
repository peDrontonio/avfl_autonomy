#!/usr/bin/env python3
"""
Mission 2 Launch File - Mobile Base Landing

This launch file starts the mission-specific nodes:
1. YOLO base detector node
2. Master mission2 controller node

Note: Launch Gazebo and drone navigation services separately:
    ros2 launch drone_gazebo simple_world.launch.py
    ros2 launch drone_navigate drone_services.launch.py

Usage:
    ros2 launch drone_gazebo mission2.launch.py
    ros2 launch drone_gazebo mission2.launch.py base_lat:=-35.363257 base_lon:=149.165400 takeoff_alt:=10.0 search_alt:=8.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Mission 2."""

    base_lat_arg = DeclareLaunchArgument(
        'base_lat',
        default_value='0.0',
        description='Base GPS latitude'
    )

    base_lon_arg = DeclareLaunchArgument(
        'base_lon',
        default_value='0.0',
        description='Base GPS longitude'
    )

    takeoff_alt_arg = DeclareLaunchArgument(
        'takeoff_alt',
        default_value='10.0',
        description='Takeoff altitude in meters above ground'
    )

    search_alt_arg = DeclareLaunchArgument(
        'search_alt',
        default_value='8.0',
        description='Search altitude in meters for the search pattern'
    )

    # YOLO Base Detector Node
    yolo_detector = Node(
        package='drone_gazebo',
        executable='yolo_base_detector',
        name='yolo_base_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    # Master Mission2 Controller Node
    master_mission2 = Node(
        package='drone_gazebo',
        executable='master_mission2',
        name='master_mission2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_lat': LaunchConfiguration('base_lat'),
            'base_lon': LaunchConfiguration('base_lon'),
            'takeoff_alt': LaunchConfiguration('takeoff_alt'),
            'search_alt': LaunchConfiguration('search_alt'),
        }]
    )

    return LaunchDescription([
        base_lat_arg,
        base_lon_arg,
        takeoff_alt_arg,
        search_alt_arg,
        yolo_detector,
        master_mission2,
    ])
