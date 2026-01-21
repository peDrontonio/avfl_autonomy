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
    """
    ROS 2 Node that provides navigate service for ArduPilot drones.
    
    Subscribes directly to ArduPilot topics for telemetry data,
    then navigates to target position using proportional control.
    
    Supports:
        - 'map' frame: Navigate to position relative to current in map coordinates
        - 'body' frame: Navigate relative to current drone position/orientation
    """

    def __init__(self):
        super().__init__('navigate_service_node')

        # Callback groups
        self.sub_callback_group = ReentrantCallbackGroup()
        self.srv_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # --- Parameters ---
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('kp_xy', 1.0)
        self.declare_parameter('kp_z', 1.0)
        
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.kp_xy = self.get_parameter('kp_xy').value
        self.kp_z = self.get_parameter('kp_z').value

        # QoS for ArduPilot topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- State Variables ---
        self.current_pose = None
        self.current_geopose = None
        self.current_yaw = 0.0
        self.pose_received = False
        
        self.target_position = None
        self.nav_speed = 0.5
        self.is_navigating = False
        
        # Thread lock for state variables
        self.state_lock = threading.Lock()

        # --- Subscribers ---
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos,
            callback_group=self.sub_callback_group
        )

        self.geopose_sub = self.create_subscription(
            GeoPoseStamped,
            '/ap/geopose/filtered',
            self.geopose_callback,
            qos,
            callback_group=self.sub_callback_group
        )

        # --- Publishers ---
        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)

        # --- Service Clients ---
        self.arm_client = self.create_client(
            ArmMotors, 
            '/ap/arm_motors',
            callback_group=self.srv_callback_group
        )
        self.mode_client = self.create_client(
            ModeSwitch, 
            '/ap/mode_switch',
            callback_group=self.srv_callback_group
        )

        # --- Service Server ---
        self.navigate_srv = self.create_service(
            Navigate,
            'avfl/navigate',
            self.navigate_callback,
            callback_group=self.srv_callback_group
        )

        # --- Control Timer ---
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, 
            self.control_loop,
            callback_group=self.timer_callback_group
        )

        self.get_logger().info("Navigate Service Node initialized.")
        self.get_logger().info("Service 'avfl/navigate' is ready.")
        self.get_logger().info("Waiting for pose data from /ap/pose/filtered...")

    def pose_callback(self, msg: PoseStamped):
        """Handle local pose updates from ArduPilot EKF."""
        with self.state_lock:
            self.current_pose = msg
            q = msg.pose.orientation
            _, _, self.current_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
            
            if not self.pose_received:
                self.get_logger().info("Pose data received. Service ready!")
                self.pose_received = True

    def geopose_callback(self, msg: GeoPoseStamped):
        """Handle global geopose updates."""
        with self.state_lock:
            self.current_geopose = msg

    def get_current_position(self) -> dict:
        """Get current position safely with lock."""
        with self.state_lock:
            if self.current_pose is None:
                return None
            return {
                'x': self.current_pose.pose.position.x,
                'y': self.current_pose.pose.position.y,
                'z': self.current_pose.pose.position.z,
                'yaw': self.current_yaw
            }

    def call_service_with_timeout(self, client, request, timeout: float = 5.0):
        """
        Call service with timeout using polling to avoid deadlock.
        """
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {client.srv_name} not available")
            return None

        future = client.call_async(request)
        
        # Wait for result with timeout (polling, not spinning)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error("Service call timed out")
                return None
            time.sleep(0.05)
        
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

    def auto_arm_sequence(self) -> tuple:
        """Execute auto-arming sequence."""
        self.get_logger().info("Initiating auto-arm sequence...")

        # 1. Switch to GUIDED mode
        mode_req = ModeSwitch.Request()
        mode_req.mode = COPTER_MODE_GUIDED
        result = self.call_service_with_timeout(self.mode_client, mode_req)
        if result is None:
            return False, "Failed to switch to GUIDED mode"
        self.get_logger().info(f"Mode switch result: {result}")

        # 2. Arm motors
        arm_req = ArmMotors.Request()
        arm_req.arm = True
        result = self.call_service_with_timeout(self.arm_client, arm_req)
        if result is None:
            return False, "Failed to ARM motors"
        self.get_logger().info(f"Arm result: {result}")

        # 3. Wait for spin-up
        self.get_logger().info("Waiting for motor spin-up...")
        time.sleep(1.0)

        return True, "Auto-arm sequence completed"

    def calculate_target_position(self, request, current_pos: dict) -> dict:
        """Calculate target position based on frame_id."""
        if request.frame_id == 'body':
            # Body frame: rotate offsets by current yaw
            yaw = current_pos['yaw']
            dx_map = request.x * math.cos(yaw) - request.y * math.sin(yaw)
            dy_map = request.x * math.sin(yaw) + request.y * math.cos(yaw)
            
            target = {
                'x': current_pos['x'] + dx_map,
                'y': current_pos['y'] + dy_map,
                'z': current_pos['z'] + request.z
            }
            
            self.get_logger().info(
                f"Body frame: offset ({request.x}, {request.y}, {request.z}) "
                f"-> target ({target['x']:.2f}, {target['y']:.2f}, {target['z']:.2f})"
            )
        else:
            # Map frame: relative to current position
            target = {
                'x': current_pos['x'] + request.x,
                'y': current_pos['y'] + request.y,
                'z': current_pos['z'] + request.z
            }
            
            self.get_logger().info(
                f"Map frame: current ({current_pos['x']:.2f}, {current_pos['y']:.2f}, {current_pos['z']:.2f}) "
                f"+ ({request.x}, {request.y}, {request.z}) "
                f"-> target ({target['x']:.2f}, {target['y']:.2f}, {target['z']:.2f})"
            )

        return target

    def navigate_callback(self, request, response):
        """Service callback for navigate requests."""
        self.get_logger().info(
            f"Navigate request: x={request.x}, y={request.y}, z={request.z}, "
            f"speed={request.speed}, frame={request.frame_id}, auto_arm={request.auto_arm}"
        )

        # Get current position
        current_pos = self.get_current_position()
        if current_pos is None:
            response.success = False
            response.message = "No position data available (EKF not ready?)"
            return response

        # Auto-arm if requested
        if request.auto_arm:
            success, message = self.auto_arm_sequence()
            if not success:
                response.success = False
                response.message = message
                return response

        # Calculate target
        with self.state_lock:
            self.target_position = self.calculate_target_position(request, current_pos)
            self.nav_speed = min(request.speed if request.speed > 0 else 0.5, self.max_velocity)
            self.is_navigating = True

        response.success = True
        response.message = (
            f"Navigation started to ({self.target_position['x']:.2f}, "
            f"{self.target_position['y']:.2f}, {self.target_position['z']:.2f}) "
            f"at {self.nav_speed} m/s"
        )
        
        return response

    def control_loop(self):
        """Control loop for position-based navigation."""
        with self.state_lock:
            if not self.is_navigating or self.target_position is None or self.current_pose is None:
                return
            
            # Get current state
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y
            curr_z = self.current_pose.pose.position.z
            target = self.target_position.copy()
            speed = self.nav_speed

        # Calculate error
        err_x = target['x'] - curr_x
        err_y = target['y'] - curr_y
        err_z = target['z'] - curr_z
        distance = math.sqrt(err_x**2 + err_y**2 + err_z**2)

        # Log progress
        self.get_logger().info(
            f"Nav: dist={distance:.2f}m, "
            f"pos=({curr_x:.2f}, {curr_y:.2f}, {curr_z:.2f}), "
            f"target=({target['x']:.2f}, {target['y']:.2f}, {target['z']:.2f})",
            throttle_duration_sec=1.0
        )

        # Check if reached
        if distance < self.position_tolerance:
            self.get_logger().info(f"Target reached! Final pos: ({curr_x:.2f}, {curr_y:.2f}, {curr_z:.2f})")
            self.stop_drone()
            with self.state_lock:
                self.target_position = None
                self.is_navigating = False
            return

        # Calculate velocity (proportional control)
        vel_x = self.kp_xy * err_x
        vel_y = self.kp_xy * err_y
        vel_z = self.kp_z * err_z

        # Limit XY speed
        speed_xy = math.sqrt(vel_x**2 + vel_y**2)
        if speed_xy > speed:
            scale = speed / speed_xy
            vel_x *= scale
            vel_y *= scale

        # Limit Z speed
        max_vz = speed * 0.5
        if abs(vel_z) > max_vz:
            vel_z = math.copysign(max_vz, vel_z)

        # Publish velocity
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vel_x
        msg.twist.linear.y = vel_y
        msg.twist.linear.z = vel_z
        self.vel_pub.publish(msg)

    def stop_drone(self):
        """Send zero velocity command."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.vel_pub.publish(msg)
        self.get_logger().info("Drone stopped.")


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)

    node = NavigateServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        node.get_logger().info("Navigate service node spinning...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        if node.is_navigating:
            node.stop_drone()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
