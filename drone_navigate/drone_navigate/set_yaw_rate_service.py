#!/usr/bin/env python3
"""
Set Yaw Rate Service Node for ArduPilot + ROS 2 Humble
Provides continuous yaw rate control.

This node provides a 'set_yaw_rate' service that continuously rotates the drone
at a specified rate while maintaining current position or velocity mode.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from ardupilot_msgs.srv import ModeSwitch
from drone_navigate.srv import SetYawRate

import time


COPTER_MODE_GUIDED = 4


class SetYawRateServiceNode(Node):
    """
    ROS 2 Node that provides set_yaw_rate service for ArduPilot drones.
    
    Continuously publishes velocity commands with yaw rate control.
    The drone will rotate at the specified rate while hovering or
    maintaining current velocity.
    
    Publishes to:
        - /ap/cmd_vel: Velocity commands with yaw rate
    """

    def __init__(self):
        super().__init__('set_yaw_rate_service_node')

        # --- Parameters ---
        self.declare_parameter('control_rate', 20.0)
        self.control_rate = self.get_parameter('control_rate').value

        # QoS for ArduPilot topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- State Variables ---
        self.current_pose = None
        self.yaw_rate = 0.0
        self.is_active = False

        # --- Subscribers (using ENU-converted topic) ---
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered/enu',
            self.pose_callback,
            qos
        )

        # --- Publishers ---
        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)

        # --- Service Clients ---
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')

        # --- Service Server ---
        self.set_yaw_rate_srv = self.create_service(
            SetYawRate,
            'avfl/set_yaw_rate',
            self.set_yaw_rate_callback
        )

        # --- Control Timer ---
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        self.get_logger().info("Set Yaw Rate Service Node initialized.")
        self.get_logger().info("Service 'avfl/set_yaw_rate' is ready.")

    def pose_callback(self, msg: PoseStamped):
        """Handle local pose updates."""
        self.current_pose = msg

    def set_yaw_rate_callback(self, request, response):
        """Service callback for set_yaw_rate requests."""
        self.get_logger().info(f"Set Yaw Rate request: {request.yaw_rate} rad/s")

        if self.current_pose is None:
            response.success = False
            response.message = "No pose data available"
            return response

        # Set the yaw rate
        self.yaw_rate = request.yaw_rate
        self.is_active = (abs(self.yaw_rate) > 0.001)  # Active if rate is non-zero

        if not self.is_active:
            # Stop rotation
            self.publish_zero_velocity()
            response.success = True
            response.message = "Yaw rate stopped"
        else:
            response.success = True
            response.message = f"Yaw rate set to {self.yaw_rate:.3f} rad/s"

        return response

    def control_loop(self):
        """Control loop for yaw rate."""
        if not self.is_active:
            return

        # Publish velocity command with yaw rate
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Hover in place (zero linear velocity)
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        
        # Apply yaw rate
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.yaw_rate
        
        self.vel_pub.publish(msg)

    def publish_zero_velocity(self):
        """Stop all motion."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        
        # Send multiple times to ensure it's received
        for _ in range(3):
            self.vel_pub.publish(msg)
            time.sleep(0.02)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = SetYawRateServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
