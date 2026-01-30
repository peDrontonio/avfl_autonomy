#!/usr/bin/env python3
"""
AVFL Launch - Real Drone Configuration
========================================

This launch file starts all required components for real-world drone operation:
- Navigation services (navigate, telemetry, yaw control, GPS navigation)
- ArUco marker detector (for real camera feed)

Usage:
    ros2 launch drone_navigate avfl.launch.py
"""

from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='=========================================='),
        LogInfo(msg='  AVFL - Real Drone System Launch'),
        LogInfo(msg='  Starting navigation services and ArUco detector...'),
        LogInfo(msg='=========================================='),
        
        # ====================================
        # NAVIGATION SERVICES
        # ====================================
        # Include the drone services launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('drone_navigate'),
                    'launch',
                    'drone_services.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'false'
            }.items()
        ),
        
        # ====================================
        # ARUCO DETECTOR
        # ====================================
        Node(
            package='drone_navigate',
            executable='aruco',
            name='aruco_detector',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        LogInfo(msg='=========================================='),
        LogInfo(msg='  All systems launched successfully!'),
        LogInfo(msg='  Ready for real drone operation.'),
        LogInfo(msg='=========================================='),
    ])
