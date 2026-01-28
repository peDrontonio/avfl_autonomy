#!/usr/bin/env python3
"""
Navigate Service Node for ArduPilot + ROS 2 Humble
Provides position-based navigation using direct topic subscriptions.

This node provides a 'navigate' service that moves the drone to a target position
using position control rather than direct velocity commands, providing more
accurate and reliable navigation.

NOTE: This version subscribes directly to ArduPilot topics instead of using
the get_telemetry service to avoid callback deadlocks.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
from drone_navigate.srv import Navigate

import math
import time
import threading


COPTER_MODE_GUIDED = 4



def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple:
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class NavigateServiceNode(Node):
    "ROS 2 Node providing a navigate service for ArduPilot drones."

def