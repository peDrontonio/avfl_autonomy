#!/usr/bin/env python3
"""
Navigate Service Node for ArduPilot + ROS 2 Humble
Provides position-based navigation using the get_telemetry service.

This node provides a 'navigate' service that moves the drone to a target position
using position control rather than direct velocity commands, providing more
accurate and reliable navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
from drone_navigate.srv import Navigate, GetTelemetry

import math
import time


COPTER_MODE_GUIDED = 4


class NavigateServiceNode(Node):
    """
    ROS 2 Node that provides navigate service for ArduPilot drones.
    
    Uses get_telemetry service to obtain current position and orientation,
    then navigates to target position using proportional control.
    
    Supports:
        - 'map' frame: Navigate to absolute position in map coordinates
        - 'body' frame: Navigate relative to current drone position/orientation
    
    Services:
        - avfl/navigate: Navigate to target position
        
    Requires:
        - avfl/get_telemetry: Telemetry service for position data
        - /ap/arm_motors: ArduPilot arm service
        - /ap/mode_switch: ArduPilot mode switch service
    """

    def __init__(self):
        super().__init__('navigate_service_node')

        # Callback group for concurrent service handling
        self.callback_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameter('position_tolerance', 0.15)  # meters
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('max_velocity', 1.0)  # m/s
        self.declare_parameter('kp_xy', 1.0)  # Proportional gain for XY
        self.declare_parameter('kp_z', 1.0)  # Proportional gain for Z
        
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.kp_xy = self.get_parameter('kp_xy').value
        self.kp_z = self.get_parameter('kp_z').value

        # --- State Variables ---
        self.target_position = None  # {'x': float, 'y': float, 'z': float}
        self.nav_speed = 0.5
        self.is_navigating = False
        self.navigation_success = False

        # --- Publishers ---
        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)

        # --- Service Clients ---
        self.telemetry_client = self.create_client(
            GetTelemetry, 
            'avfl/get_telemetry',
            callback_group=self.callback_group
        )
        self.arm_client = self.create_client(
            ArmMotors, 
            '/ap/arm_motors',
            callback_group=self.callback_group
        )
        self.mode_client = self.create_client(
            ModeSwitch, 
            '/ap/mode_switch',
            callback_group=self.callback_group
        )

        # --- Service Server ---
        self.navigate_srv = self.create_service(
            Navigate,
            'avfl/navigate',
            self.navigate_callback,
            callback_group=self.callback_group
        )

        # --- Control Timer ---
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, 
            self.control_loop
        )

        self.get_logger().info("Navigate Service Node initialized.")
        self.get_logger().info("Service 'avfl/navigate' is ready.")
        self.get_logger().info("Waiting for telemetry service...")

    def get_telemetry(self, frame_id: str = 'map') -> dict:
        """
        Call the get_telemetry service to get current drone state.
        
        Args:
            frame_id: Reference frame for coordinates ('map' or 'body')
            
        Returns:
            dict with telemetry data or None if service fails
        """
        if not self.telemetry_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Telemetry service not available!")
            return None

        request = GetTelemetry.Request()
        request.frame_id = frame_id

        future = self.telemetry_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            try:
                result = future.result()
                return {
                    'frame_id': result.frame_id,
                    'x': result.x,
                    'y': result.y,
                    'z': result.z,
                    'lat': result.lat,
                    'lon': result.lon,
                    'alt': result.alt,
                    'vx': result.vx,
                    'vy': result.vy,
                    'vz': result.vz,
                    'roll': result.roll,
                    'pitch': result.pitch,
                    'yaw': result.yaw
                }
            except Exception as e:
                self.get_logger().error(f"Telemetry service call failed: {e}")
                return None
        else:
            self.get_logger().error("Telemetry service call timed out")
            return None

    def call_service_sync(self, client, request, timeout: float = 5.0) -> bool:
        """
        Synchronous service call helper.
        
        Args:
            client: ROS 2 service client
            request: Service request
            timeout: Timeout in seconds
            
        Returns:
            True if successful, False otherwise
        """
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {client.srv_name} not available")
            return False

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.done():
            try:
                result = future.result()
                self.get_logger().info(f"Service call succeeded: {result}")
                return True
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return False
        else:
            self.get_logger().error("Service call timed out")
            return False

    def auto_arm_sequence(self) -> tuple:
        """
        Execute auto-arming sequence: switch to GUIDED mode and arm motors.
        
        Returns:
            tuple: (success: bool, message: str)
        """
        self.get_logger().info("Initiating auto-arm sequence...")

        # 1. Switch to GUIDED mode
        mode_req = ModeSwitch.Request()
        mode_req.mode = COPTER_MODE_GUIDED
        if not self.call_service_sync(self.mode_client, mode_req):
            return False, "Failed to switch to GUIDED mode"

        self.get_logger().info("Switched to GUIDED mode.")

        # 2. Arm motors
        arm_req = ArmMotors.Request()
        arm_req.arm = True
        if not self.call_service_sync(self.arm_client, arm_req):
            return False, "Failed to ARM motors"

        self.get_logger().info("Motors armed successfully.")

        # 3. Wait for spin-up
        self.get_logger().info("Waiting for motor spin-up...")
        time.sleep(1.0)

        return True, "Auto-arm sequence completed"

    def calculate_target_position(self, request, telemetry: dict) -> dict:
        """
        Calculate target position based on frame_id.
        
        Args:
            request: Navigate.Request with x, y, z, frame_id
            telemetry: Current telemetry data in 'map' frame
            
        Returns:
            dict with target position in map frame {'x', 'y', 'z'}
        """
        if request.frame_id == 'body':
            # Body frame: coordinates relative to current position and yaw
            yaw = telemetry['yaw']
            
            # Rotate body-frame offsets to map frame
            dx_map = request.x * math.cos(yaw) - request.y * math.sin(yaw)
            dy_map = request.x * math.sin(yaw) + request.y * math.cos(yaw)
            
            target_x = telemetry['x'] + dx_map
            target_y = telemetry['y'] + dy_map
            target_z = telemetry['z'] + request.z
            
            self.get_logger().info(
                f"Body frame navigation: offset ({request.x}, {request.y}, {request.z}) "
                f"-> map target ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
            )
        else:
            # Map frame (or any other): interpret as absolute coordinates
            # If relative movement is desired in map frame, add to current position
            target_x = telemetry['x'] + request.x
            target_y = telemetry['y'] + request.y
            target_z = telemetry['z'] + request.z
            
            self.get_logger().info(
                f"Map frame navigation: current ({telemetry['x']:.2f}, {telemetry['y']:.2f}, {telemetry['z']:.2f}) "
                f"+ offset ({request.x}, {request.y}, {request.z}) "
                f"-> target ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
            )

        return {'x': target_x, 'y': target_y, 'z': target_z}

    def navigate_callback(self, request, response):
        """
        Service callback for navigate requests.
        
        Args:
            request: Navigate.Request with target position
            response: Navigate.Response
            
        Returns:
            Response with success status and message
        """
        self.get_logger().info(
            f"Navigate request: x={request.x}, y={request.y}, z={request.z}, "
            f"speed={request.speed}, frame={request.frame_id}, auto_arm={request.auto_arm}"
        )

        # --- Get current telemetry ---
        telemetry = self.get_telemetry('map')
        if telemetry is None:
            response.success = False
            response.message = "Failed to get telemetry data"
            return response

        # Check for valid position data
        if math.isnan(telemetry['x']) or math.isnan(telemetry['y']) or math.isnan(telemetry['z']):
            response.success = False
            response.message = "No valid position data available (EKF not ready?)"
            return response

        # --- Auto-arm sequence if requested ---
        if request.auto_arm:
            success, message = self.auto_arm_sequence()
            if not success:
                response.success = False
                response.message = message
                return response

        # --- Calculate target position ---
        self.target_position = self.calculate_target_position(request, telemetry)
        
        # --- Set navigation speed ---
        self.nav_speed = request.speed if request.speed > 0 else 0.5
        self.nav_speed = min(self.nav_speed, self.max_velocity)

        # --- Start navigation ---
        self.is_navigating = True
        self.navigation_success = False

        response.success = True
        response.message = (
            f"Navigation started to ({self.target_position['x']:.2f}, "
            f"{self.target_position['y']:.2f}, {self.target_position['z']:.2f}) "
            f"at {self.nav_speed} m/s"
        )
        
        return response

    def control_loop(self):
        """
        Control loop that runs at fixed rate to navigate towards target.
        Uses proportional control based on position error from telemetry.
        """
        if not self.is_navigating or self.target_position is None:
            return

        # Get current telemetry
        telemetry = self.get_telemetry('map')
        if telemetry is None:
            self.get_logger().warn("Lost telemetry during navigation!", throttle_duration_sec=1.0)
            return

        if math.isnan(telemetry['x']):
            self.get_logger().warn("Invalid telemetry data!", throttle_duration_sec=1.0)
            return

        # Calculate position error
        err_x = self.target_position['x'] - telemetry['x']
        err_y = self.target_position['y'] - telemetry['y']
        err_z = self.target_position['z'] - telemetry['z']
        
        distance = math.sqrt(err_x**2 + err_y**2 + err_z**2)
        distance_xy = math.sqrt(err_x**2 + err_y**2)

        # Log progress
        self.get_logger().info(
            f"Nav: dist={distance:.2f}m, "
            f"current=({telemetry['x']:.2f}, {telemetry['y']:.2f}, {telemetry['z']:.2f}), "
            f"target=({self.target_position['x']:.2f}, {self.target_position['y']:.2f}, {self.target_position['z']:.2f})",
            throttle_duration_sec=1.0
        )

        # Check if target reached
        if distance < self.position_tolerance:
            self.get_logger().info(
                f"Target reached! Final position: "
                f"({telemetry['x']:.2f}, {telemetry['y']:.2f}, {telemetry['z']:.2f})"
            )
            self.stop_drone()
            self.target_position = None
            self.is_navigating = False
            self.navigation_success = True
            return

        # Calculate velocity commands (proportional control)
        vel_x = self.kp_xy * err_x
        vel_y = self.kp_xy * err_y
        vel_z = self.kp_z * err_z

        # Limit XY speed
        speed_xy = math.sqrt(vel_x**2 + vel_y**2)
        if speed_xy > self.nav_speed:
            scale = self.nav_speed / speed_xy
            vel_x *= scale
            vel_y *= scale

        # Limit Z speed (typically slower for safety)
        max_vz = self.nav_speed * 0.5  # Half of XY speed for vertical
        if abs(vel_z) > max_vz:
            vel_z = math.copysign(max_vz, vel_z)

        # Publish velocity command
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = vel_x
        msg.twist.linear.y = vel_y
        msg.twist.linear.z = vel_z
        self.vel_pub.publish(msg)

    def stop_drone(self):
        """Send zero velocity command to stop the drone."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # All velocities are 0 by default
        self.vel_pub.publish(msg)
        self.get_logger().info("Drone stopped (zero velocity sent).")


def main(args=None):
    """Entry point for the navigate service node."""
    rclpy.init(args=args)

    node = NavigateServiceNode()

    # Use MultiThreadedExecutor for concurrent service handling
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info("Navigate service node spinning...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down navigate service node...")
    finally:
        # Stop drone before shutting down
        if node.is_navigating:
            node.stop_drone()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
