#!/usr/bin/env python3
"""
AVFL Complete Launch - Real Drone with DDS Agent
=================================================

This launch file starts ALL required components for real-world drone operation:
- MicroXRCE DDS Agent (for ArduPilot communication)
- Navigation services (navigate, telemetry, yaw control, GPS navigation)
- ArUco marker detector (for real camera feed)

Usage:
    ros2 launch drone_navigate avfl_with_dds.launch.py serial_port:=/dev/ttyAMA0 baud_rate:=115200
"""

from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS1',
        description='Serial port for micro_ros_agent communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Baud rate for serial communication'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        
        LogInfo(msg='=========================================='),
        LogInfo(msg='  AVFL - Complete Real Drone System'),
        LogInfo(msg='  Starting DDS Agent + Navigation + ArUco'),
        LogInfo(msg='=========================================='),
        
        # ====================================
        # MICRO ROS DDS AGENT - SERIAL MODE
        # ====================================
        LogInfo(msg='Starting Micro-ROS DDS Agent (Serial)...'),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'serial', '--dev', LaunchConfiguration('serial_port'),
                '-b', LaunchConfiguration('baud_rate'),
                '-v6'
            ],
            name='micro_ros_agent',
            output='screen',
            emulate_tty=True
        ),
        
        # ====================================
        # NAVIGATION SERVICES
        # ====================================
        LogInfo(msg='Starting drone navigation services...'),
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
        LogInfo(msg='Starting ArUco detector...'),
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