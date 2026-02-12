#!/usr/bin/env python3
"""
Launch file for all drone navigation services

This launch file starts:
- Navigate service (position-based navigation)
- Telemetry service (drone state information)
- Set yaw rate service (rotation control)
- Navigate global service (GPS-based navigation)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        LogInfo(msg='Starting drone navigation services...'),
        
        # NED to ENU Converter - Must start before other navigation nodes
        Node(
            package='drone_navigate',
            executable='ned_to_enu_node',
            name='ned_to_enu_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            emulate_tty=True,
        ),

        # Telemetry Node - Provides /get_telemetry service
        Node(
            package='drone_navigate',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            emulate_tty=True,
        ),
        
        # Navigate Service - Provides /navigate service
        Node(
            package='drone_navigate',
            executable='navigate_service',
            name='navigate_service',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            emulate_tty=True,
        ),
        
        # Set Yaw Rate Service - Provides /set_yaw_rate service
        Node(
            package='drone_navigate',
            executable='set_yaw_rate_service',
            name='set_yaw_rate_service',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            emulate_tty=True,
        ),
        
        # Navigate Global Service - Provides /navigate_global service (GPS)
        Node(
            package='drone_navigate',
            executable='navigate_global_service',
            name='navigate_global_service',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            emulate_tty=True,
        ),
        
        LogInfo(msg='All drone navigation services started successfully!'),
    ])
