#!/usr/bin/env python3
"""
Navigate Global Service Node for ArduPilot + ROS 2 Humble
Provides GPS-based navigation using lat/lon coordinates.

This node provides a 'navigate_global' service that moves the drone to a target
GPS position using the ArduPilot GlobalPosition interface for precise
GPS-based navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from ardupilot_msgs.msg import GlobalPosition
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
from drone_navigate.srv import NavigateGlobal

import math
import time
import threading


COPTER_MODE_GUIDED = 4

# GlobalPosition frame constants
FRAME_GLOBAL_INT = 5
FRAME_GLOBAL_REL_ALT = 6
FRAME_GLOBAL_TERRAIN_ALT = 11

# GlobalPosition type_mask constants
IGNORE_VX = 8
IGNORE_VY = 16
IGNORE_VZ = 32
IGNORE_AFX = 64
IGNORE_AFY = 128
IGNORE_AFZ = 256
IGNORE_YAW = 1024
IGNORE_YAW_RATE = 2048


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


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great-circle distance between two points on Earth.
    
    Args:
        lat1, lon1: First point coordinates in degrees
        lat2, lon2: Second point coordinates in degrees
        
    Returns:
        Distance in meters
    """
    R = 6371000  # Earth's radius in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c


class NavigateGlobalServiceNode(Node):
    """
    ROS 2 Node that provides GPS-based navigation service for ArduPilot drones.
    
    Uses the ArduPilot GlobalPosition message to send GPS waypoints directly,
    providing more precise navigation than velocity-based control.
    
    Subscribes to:
        - /ap/geopose/filtered: Current GPS position
        - /ap/pose/filtered: Local pose for altitude reference
        
    Publishes to:
        - /ap/cmd_gps_pose: GPS waypoint commands
    """

    def __init__(self):
        super().__init__('navigate_global_service_node')

        # Callback groups
        self.sub_callback_group = ReentrantCallbackGroup()
        self.srv_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # --- Parameters ---
        self.declare_parameter('position_tolerance', 1.0)  # meters (GPS accuracy)
        self.declare_parameter('altitude_tolerance', 0.5)  # meters
        self.declare_parameter('control_rate', 5.0)  # Hz (slower for GPS)
        self.declare_parameter('default_speed', 2.0)  # m/s
        
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.altitude_tolerance = self.get_parameter('altitude_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.default_speed = self.get_parameter('default_speed').value

        # QoS for ArduPilot topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- State Variables ---
        self.current_geopose = None
        self.current_pose = None
        self.current_yaw = 0.0
        self.geopose_received = False
        self.pose_received = False
        
        self.target_lat = None
        self.target_lon = None
        self.target_alt = None
        self.target_yaw = None
        self.nav_speed = 2.0
        self.is_navigating = False
        
        # Thread lock for state variables
        self.state_lock = threading.Lock()

        # --- Subscribers ---
        self.geopose_sub = self.create_subscription(
            GeoPoseStamped,
            '/ap/geopose/filtered',
            self.geopose_callback,
            qos,
            callback_group=self.sub_callback_group
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos,
            callback_group=self.sub_callback_group
        )

        # --- Publishers ---
        self.global_pos_pub = self.create_publisher(
            GlobalPosition, 
            '/ap/cmd_gps_pose', 
            10
        )

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
        self.navigate_global_srv = self.create_service(
            NavigateGlobal,
            'avfl/navigate_global',
            self.navigate_global_callback,
            callback_group=self.srv_callback_group
        )

        # --- Control Timer ---
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, 
            self.control_loop,
            callback_group=self.timer_callback_group
        )

        self.get_logger().info("Navigate Global Service Node initialized.")
        self.get_logger().info("Service 'avfl/navigate_global' is ready.")
        self.get_logger().info("Waiting for GPS data from /ap/geopose/filtered...")

    def geopose_callback(self, msg: GeoPoseStamped):
        """Handle global geopose updates."""
        with self.state_lock:
            self.current_geopose = msg
            
            if not self.geopose_received:
                self.get_logger().info(
                    f"GPS data received: lat={msg.pose.position.latitude:.6f}, "
                    f"lon={msg.pose.position.longitude:.6f}, "
                    f"alt={msg.pose.position.altitude:.2f}m"
                )
                self.geopose_received = True

    def pose_callback(self, msg: PoseStamped):
        """Handle local pose updates."""
        with self.state_lock:
            self.current_pose = msg
            q = msg.pose.orientation
            _, _, self.current_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
            
            if not self.pose_received:
                self.get_logger().info("Local pose data received.")
                self.pose_received = True

    def get_current_gps(self) -> dict:
        """Get current GPS position safely with lock."""
        with self.state_lock:
            if self.current_geopose is None:
                return None
            return {
                'lat': self.current_geopose.pose.position.latitude,
                'lon': self.current_geopose.pose.position.longitude,
                'alt': self.current_geopose.pose.position.altitude,
                'yaw': self.current_yaw
            }

    def call_service_with_timeout(self, client, request, timeout: float = 5.0):
        """Call service with timeout using polling to avoid deadlock."""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {client.srv_name} not available")
            return None

        future = client.call_async(request)
        
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

    def navigate_global_callback(self, request, response):
        """Service callback for navigate_global requests."""
        self.get_logger().info(
            f"Navigate Global request: lat={request.lat:.6f}, lon={request.lon:.6f}, "
            f"z={request.z}, yaw={request.yaw}, speed={request.speed}, "
            f"frame={request.frame_id}, auto_arm={request.auto_arm}"
        )

        # Get current GPS position
        current_gps = self.get_current_gps()
        if current_gps is None:
            response.success = False
            response.message = "No GPS data available"
            return response

        # Auto-arm if requested
        if request.auto_arm:
            success, message = self.auto_arm_sequence()
            if not success:
                response.success = False
                response.message = message
                return response

        # Calculate target altitude
        if request.frame_id == 'rel' or request.frame_id == 'relative':
            # Relative altitude: add z to current altitude
            target_alt = current_gps['alt'] + request.z
        else:
            # Absolute altitude or default
            if request.z > 0:
                target_alt = current_gps['alt'] + request.z  # Treat as relative by default
            else:
                target_alt = current_gps['alt']  # Keep current altitude

        # Set target
        with self.state_lock:
            self.target_lat = request.lat
            self.target_lon = request.lon
            self.target_alt = target_alt
            self.target_yaw = request.yaw if request.yaw != 0 else None
            self.nav_speed = request.speed if request.speed > 0 else self.default_speed
            self.is_navigating = True

        # Calculate distance for info
        distance_m = haversine_distance(
            current_gps['lat'], current_gps['lon'],
            request.lat, request.lon
        )

        response.success = True
        response.message = (
            f"Navigation started to GPS ({request.lat:.6f}, {request.lon:.6f}), "
            f"alt={target_alt:.1f}m, distance={distance_m:.1f}m"
        )
        
        return response

    def send_global_position(self):
        """Send GlobalPosition command to ArduPilot."""
        if self.target_lat is None or self.target_lon is None:
            return

        msg = GlobalPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Use FRAME_GLOBAL_REL_ALT for altitude relative to home
        msg.coordinate_frame = FRAME_GLOBAL_REL_ALT
        
        # Set position
        msg.latitude = self.target_lat
        msg.longitude = self.target_lon
        msg.altitude = self.target_alt
        
        # Set velocity based on desired speed (optional, for smoother motion)
        # We ignore velocity and let ArduPilot handle the path planning
        msg.velocity.linear.x = 0.0
        msg.velocity.linear.y = 0.0
        msg.velocity.linear.z = 0.0
        
        # Ignore acceleration
        msg.acceleration_or_force.linear.x = 0.0
        msg.acceleration_or_force.linear.y = 0.0
        msg.acceleration_or_force.linear.z = 0.0
        
        # Set yaw if specified
        if self.target_yaw is not None:
            msg.yaw = self.target_yaw
            msg.type_mask = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE
        else:
            msg.yaw = 0.0
            msg.type_mask = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW | IGNORE_YAW_RATE
        
        self.global_pos_pub.publish(msg)

    def control_loop(self):
        """Control loop for GPS-based navigation."""
        with self.state_lock:
            if not self.is_navigating or self.target_lat is None or self.current_geopose is None:
                return
            
            # Get current state
            curr_lat = self.current_geopose.pose.position.latitude
            curr_lon = self.current_geopose.pose.position.longitude
            curr_alt = self.current_geopose.pose.position.altitude
            
            target_lat = self.target_lat
            target_lon = self.target_lon
            target_alt = self.target_alt

        # Calculate horizontal distance
        horiz_distance = haversine_distance(curr_lat, curr_lon, target_lat, target_lon)
        
        # Calculate vertical distance
        vert_distance = abs(target_alt - curr_alt)
        
        # Total 3D distance (approximate)
        total_distance = math.sqrt(horiz_distance**2 + vert_distance**2)

        # Log progress
        self.get_logger().info(
            f"NavGlobal: dist={horiz_distance:.1f}m (horiz), {vert_distance:.1f}m (vert), "
            f"current=({curr_lat:.6f}, {curr_lon:.6f}, {curr_alt:.1f}m), "
            f"target=({target_lat:.6f}, {target_lon:.6f}, {target_alt:.1f}m)",
            throttle_duration_sec=2.0
        )

        # Check if reached (using GPS tolerance)
        if horiz_distance < self.position_tolerance and vert_distance < self.altitude_tolerance:
            self.get_logger().info(
                f"Target reached! Final GPS: ({curr_lat:.6f}, {curr_lon:.6f}, {curr_alt:.1f}m)"
            )
            with self.state_lock:
                self.target_lat = None
                self.target_lon = None
                self.target_alt = None
                self.target_yaw = None
                self.is_navigating = False
            return

        # Send position command continuously
        self.send_global_position()


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)

    node = NavigateGlobalServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        node.get_logger().info("Navigate Global service node spinning...")
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
