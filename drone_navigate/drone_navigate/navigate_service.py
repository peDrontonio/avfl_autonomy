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

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3, TransformStamped
from geographic_msgs.msg import GeoPoseStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
from ardupilot_msgs.msg import GlobalPosition
from drone_navigate.srv import Navigate

import tf2_ros
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
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.005)  # ~0.29 degrees
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('max_yaw_rate', 0.5)  # rad/s, ~28 deg/s
        self.declare_parameter('kp_xy', 1.0)
        self.declare_parameter('kp_z', 1.0)
        self.declare_parameter('kp_yaw', 1.5)
        
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.kp_xy = self.get_parameter('kp_xy').value
        self.kp_z = self.get_parameter('kp_z').value
        self.kp_yaw = self.get_parameter('kp_yaw').value

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
        self.target_yaw = None  # Target yaw in radians (None = maintain current)
        self.target_yaw_rate = None  # Max yaw rate (None = use default)
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
        # Use BEST_EFFORT QoS to match ArduPilot's subscriber QoS
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', cmd_vel_qos)
        self.gps_pose_pub = self.create_publisher(GlobalPosition, '/ap/cmd_gps_pose', 10)

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

        # --- TF Broadcaster for navigate_target frame ---
        # Static TF is time-independent, avoids sim_time vs wall_clock mismatch
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

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
            # Body frame: relative offset rotated by current yaw
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
            # Map frame: absolute position in map coordinates (Clover behavior)
            target = {
                'x': request.x,
                'y': request.y,
                'z': request.z
            }
            
            self.get_logger().info(
                f"Map frame: absolute target ({target['x']:.2f}, {target['y']:.2f}, {target['z']:.2f})"
            )

        return target

    def navigate_callback(self, request, response):
        """Service callback for navigate requests."""
        self.get_logger().info(
            f"Navigate request: x={request.x}, y={request.y}, z={request.z}, "
            f"yaw={request.yaw}, speed={request.speed}, frame={request.frame_id}, auto_arm={request.auto_arm}"
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
            
            # Set target yaw: only if both yaw AND yaw_rate are specified
            if not math.isnan(request.yaw) and request.yaw_rate > 0:
                if request.frame_id == 'body':
                    # Body frame: yaw is relative to current yaw
                    self.target_yaw = current_pos['yaw'] + request.yaw
                else:
                    # Map frame: yaw is absolute
                    self.target_yaw = request.yaw
                self.target_yaw_rate = request.yaw_rate
            else:
                # No rotation - maintain current yaw
                self.target_yaw = None
                self.target_yaw_rate = None
            
            self.nav_speed = min(request.speed if request.speed > 0 else 0.5, self.max_velocity)
            self.is_navigating = True

        # Broadcast navigate_target static TF so telemetry queries work
        self.broadcast_navigate_target(self.target_position)

        response.success = True
        response.message = (
            f"Navigation started to ({self.target_position['x']:.2f}, "
            f"{self.target_position['y']:.2f}, {self.target_position['z']:.2f}) "
            f"at {self.nav_speed} m/s"
        )
        
        return response

    def broadcast_navigate_target(self, target):
        """Broadcast navigate_target TF frame at the current target position."""
        t = TransformStamped()
        # Static TF: stamp=0 means valid at all times (avoids sim_time mismatch)
        t.header.frame_id = 'odom'
        t.child_frame_id = 'navigate_target'
        t.transform.translation.x = target['x']
        t.transform.translation.y = target['y']
        t.transform.translation.z = target['z']
        t.transform.rotation.w = 1.0  # Identity rotation
        self.tf_broadcaster.sendTransform(t)

    def control_loop(self):
        """Control loop for position-based navigation."""
        with self.state_lock:
            if not self.is_navigating or self.target_position is None or self.current_pose is None:
                return
            
            # Get current state
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y
            curr_z = self.current_pose.pose.position.z
            curr_yaw = self.current_yaw
            target = self.target_position.copy()
            target_yaw = self.target_yaw
            speed = self.nav_speed
            geopose = self.current_geopose

        # Calculate error in MAP frame
        err_x_map = target['x'] - curr_x
        err_y_map = target['y'] - curr_y
        err_z = target['z'] - curr_z
        distance = math.sqrt(err_x_map**2 + err_y_map**2 + err_z**2)
        
        # Calculate yaw error if target yaw is specified
        yaw_error = 0.0
        yaw_rate_cmd = 0.0
        if target_yaw is not None:
            # Normalize yaw error to [-pi, pi]
            yaw_error = target_yaw - curr_yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            
            # Apply proportional control with yaw rate limiting
            yaw_rate_cmd = self.kp_yaw * yaw_error
            if self.target_yaw_rate is not None:  # Apply rate limit
                yaw_rate_cmd = math.copysign(
                    min(abs(yaw_rate_cmd), self.target_yaw_rate),
                    yaw_rate_cmd
                )

        # Log progress
        yaw_str = f"target_yaw={math.degrees(target_yaw):.1f}°, err={math.degrees(yaw_error):.1f}°" if target_yaw is not None else "no_yaw_control"
        self.get_logger().info(
            f"Nav: dist={distance:.2f}m, "
            f"pos=({curr_x:.2f}, {curr_y:.2f}, {curr_z:.2f}), "
            f"yaw={math.degrees(curr_yaw):.1f}°, {yaw_str}",
            throttle_duration_sec=1.0
        )

        # Check if both position AND yaw are reached
        position_reached = distance < self.position_tolerance
        yaw_reached = target_yaw is None or abs(yaw_error) < self.yaw_tolerance
        
        if position_reached and yaw_reached:
            self.get_logger().info(
                f"Target reached! Pos: ({curr_x:.2f}, {curr_y:.2f}, {curr_z:.2f}), "
                f"Yaw: {math.degrees(curr_yaw):.1f}°"
            )
            
            # Stop immediately by publishing zero velocity
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = "map"
            stop_msg.twist.linear.x = 0.0
            stop_msg.twist.linear.y = 0.0
            stop_msg.twist.linear.z = 0.0
            self.vel_pub.publish(stop_msg)
            
            # Then set flags to stop the control loop
            with self.state_lock:
                self.target_position = None
                self.target_yaw = None
                self.is_navigating = False
            
            return

        # Publish velocity commands only if position not reached
        if not position_reached:
            # Calculate velocity in MAP frame (proportional control)
            vel_x_map = self.kp_xy * err_x_map
            vel_y_map = self.kp_xy * err_y_map
            vel_z = self.kp_z * err_z

            # Limit XY speed in map frame
            speed_xy = math.sqrt(vel_x_map**2 + vel_y_map**2)
            if speed_xy > speed:
                scale = speed / speed_xy
                vel_x_map *= scale
                vel_y_map *= scale

            # Limit Z speed
            max_vz = speed * 0.5
            if abs(vel_z) > max_vz:
                vel_z = math.copysign(max_vz, vel_z)

            # Publish velocity in MAP frame (ENU)
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.twist.linear.x = vel_x_map  # East velocity
            msg.twist.linear.y = vel_y_map  # North velocity
            msg.twist.linear.z = vel_z      # Up velocity
            msg.twist.angular.z = yaw_rate_cmd  # Yaw rate
            self.vel_pub.publish(msg)
        else:
            # Position reached, send zero velocity (but keep yaw control active)
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = "map"
            stop_msg.twist.linear.x = 0.0
            stop_msg.twist.linear.y = 0.0
            stop_msg.twist.linear.z = 0.0
            stop_msg.twist.angular.z = yaw_rate_cmd  # Keep rotating to target yaw
            self.vel_pub.publish(stop_msg)
        
        # Publish yaw command if target yaw is specified
        if target_yaw is not None and geopose is not None:
            yaw_msg = GlobalPosition()
            yaw_msg.header.stamp = self.get_clock().now().to_msg()
            yaw_msg.header.frame_id = "map"
            yaw_msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_REL_ALT
            
            # Ignore position and velocity, only control yaw
            yaw_msg.type_mask = (
                GlobalPosition.IGNORE_LATITUDE |
                GlobalPosition.IGNORE_LONGITUDE |
                GlobalPosition.IGNORE_ALTITUDE |
                GlobalPosition.IGNORE_VX |
                GlobalPosition.IGNORE_VY |
                GlobalPosition.IGNORE_VZ |
                GlobalPosition.IGNORE_AFX |
                GlobalPosition.IGNORE_AFY |
                GlobalPosition.IGNORE_AFZ |
                GlobalPosition.IGNORE_YAW_RATE
            )
            
            # Use current GPS position
            yaw_msg.latitude = geopose.pose.position.latitude
            yaw_msg.longitude = geopose.pose.position.longitude
            yaw_msg.altitude = geopose.pose.position.altitude
            yaw_msg.yaw = target_yaw
            
            self.gps_pose_pub.publish(yaw_msg)


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
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
