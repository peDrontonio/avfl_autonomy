#!/usr/bin/env python3
"""
Telemetry Node for ArduPilot + ROS 2 Humble
Provides get_telemetry service compatible with Clover's interface.

This node listens to the TF tree and ArduPilot topics to provide
position, orientation, and velocity data relative to any requested frame.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState
from ardupilot_msgs.msg import GlobalPosition
from mavros_msgs.msg import State
from std_msgs.msg import String as RosString
from drone_navigate.srv import GetTelemetry

import math


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        x, y, z, w: Quaternion components
        
    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TelemetryNode(Node):
    """
    ROS 2 Node that provides telemetry service for ArduPilot drones.
    
    Subscribes to:
        - /ap/pose/filtered: Local position from EKF
        - /ap/geopose/filtered: Global GPS position
        - /ap/twist/filtered or velocity topics: Velocity data
        
    Provides:
        - avfl/get_telemetry: Service to get drone state relative to any TF frame
    """

    def __init__(self):
        super().__init__('telemetry_node')

        # Callback group for concurrent service handling
        self.callback_group = ReentrantCallbackGroup()

        # QoS for ArduPilot topics (BEST_EFFORT, VOLATILE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- TF2 Buffer and Listener ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- State Variables ---
        self.current_pose = None          # PoseStamped from /ap/pose/filtered
        self.current_geopose = None       # GeoPoseStamped from /ap/geopose/filtered
        self.current_velocity = None      # TwistStamped from velocity topic
        self.current_battery = None       # BatteryState from /ap/battery
        self.current_state = None         # State from /ap/state
        
        # Track data availability
        self.pose_received = False
        self.geopose_received = False
        self.velocity_received = False

        # --- Subscribers (in callback group for concurrency) ---
        # Local pose (EKF filtered position in local frame)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos,
            callback_group=self.callback_group
        )

        # Global geopose (GPS coordinates)
        self.geopose_sub = self.create_subscription(
            GeoPoseStamped,
            '/ap/geopose/filtered',
            self.geopose_callback,
            qos,
            callback_group=self.callback_group
        )

        # Velocity (try different topic names ArduPilot might use)
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/ap/twist/filtered',
            self.velocity_callback,
            qos,
            callback_group=self.callback_group
        )

        # Battery state
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/ap/battery',
            self.battery_callback,
            qos,
            callback_group=self.callback_group
        )

        # FCU state (connected, armed, mode)
        self.state_sub = self.create_subscription(
            State,
            '/ap/state',
            self.state_callback,
            qos,
            callback_group=self.callback_group
        )

        # --- Service Server ---
        self.telemetry_srv = self.create_service(
            GetTelemetry,
            'avfl/get_telemetry',
            self.get_telemetry_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info("Telemetry Node initialized. Waiting for data...")
        self.get_logger().info("Service 'avfl/get_telemetry' is ready.")

    def pose_callback(self, msg: PoseStamped):
        """Handle local pose updates from ArduPilot EKF."""
        self.current_pose = msg
        if not self.pose_received:
            self.get_logger().info("Local pose data received.")
            self.pose_received = True

    def geopose_callback(self, msg: GeoPoseStamped):
        """Handle global geopose updates (GPS coordinates)."""
        self.current_geopose = msg
        if not self.geopose_received:
            self.get_logger().info("Geopose (GPS) data received.")
            self.geopose_received = True

    def velocity_callback(self, msg: TwistStamped):
        """Handle velocity updates."""
        self.current_velocity = msg
        if not self.velocity_received:
            self.get_logger().info("Velocity data received.")
            self.velocity_received = True

    def battery_callback(self, msg: BatteryState):
        """Handle battery state updates."""
        self.current_battery = msg

    def state_callback(self, msg: State):
        """Handle FCU state updates."""
        self.current_state = msg

    def get_telemetry_callback(self, request, response):
        """
        Service callback to get telemetry data.
        
        The service calculates the drone's position relative to the requested frame_id
        using the TF tree. If no frame_id is specified, it returns data in the 'map' frame.
        
        Args:
            request: GetTelemetry.Request with frame_id
            response: GetTelemetry.Response with position, orientation, velocity
            
        Returns:
            Populated response with telemetry data
        """
        # Default frame if not specified
        target_frame = request.frame_id if request.frame_id else 'map'
        response.frame_id = target_frame

        # --- Get State (connected, armed, mode) ---
        if self.current_state is not None:
            response.connected = self.current_state.connected
            response.armed = self.current_state.armed
            response.mode = self.current_state.mode
        else:
            response.connected = False
            response.armed = False
            response.mode = "UNKNOWN"

        # --- Get Position and Orientation ---
        if self.current_pose is not None:
            try:
                # Try to transform pose to the requested frame
                # The drone's frame is typically 'base_link' or we use the pose directly
                source_frame = self.current_pose.header.frame_id or 'map'
                
                if target_frame == source_frame or target_frame == 'map':
                    # Direct use of current pose
                    response.x = self.current_pose.pose.position.x
                    response.y = self.current_pose.pose.position.y
                    response.z = self.current_pose.pose.position.z

                    # Convert quaternion to Euler
                    q = self.current_pose.pose.orientation
                    roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
                    response.roll = roll
                    response.pitch = pitch
                    response.yaw = yaw

                elif target_frame == 'body':
                    # Body frame: position is always (0, 0, 0) relative to itself
                    response.x = 0.0
                    response.y = 0.0
                    response.z = 0.0
                    
                    # Orientation in body frame is still the absolute orientation
                    q = self.current_pose.pose.orientation
                    roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
                    response.roll = roll
                    response.pitch = pitch
                    response.yaw = yaw

                else:
                    # Try to use TF to transform to the requested frame
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            target_frame,
                            'base_link',
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.5)
                        )
                        
                        response.x = transform.transform.translation.x
                        response.y = transform.transform.translation.y
                        response.z = transform.transform.translation.z

                        q = transform.transform.rotation
                        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
                        response.roll = roll
                        response.pitch = pitch
                        response.yaw = yaw

                    except TransformException as e:
                        self.get_logger().warn(
                            f"TF transform failed for frame '{target_frame}': {e}. "
                            f"Returning data in 'map' frame."
                        )
                        # Fallback to map frame data
                        response.frame_id = 'map'
                        response.x = self.current_pose.pose.position.x
                        response.y = self.current_pose.pose.position.y
                        response.z = self.current_pose.pose.position.z

                        q = self.current_pose.pose.orientation
                        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
                        response.roll = roll
                        response.pitch = pitch
                        response.yaw = yaw

            except Exception as e:
                self.get_logger().error(f"Error processing pose: {e}")
                response.x = float('nan')
                response.y = float('nan')
                response.z = float('nan')
                response.roll = float('nan')
                response.pitch = float('nan')
                response.yaw = float('nan')
        else:
            self.get_logger().warn("No pose data available yet.")
            response.x = float('nan')
            response.y = float('nan')
            response.z = float('nan')
            response.roll = float('nan')
            response.pitch = float('nan')
            response.yaw = float('nan')

        # --- Get GPS Coordinates ---
        if self.current_geopose is not None:
            response.lat = self.current_geopose.pose.position.latitude
            response.lon = self.current_geopose.pose.position.longitude
            response.alt = self.current_geopose.pose.position.altitude
        else:
            self.get_logger().warn("No GPS data available yet.")
            response.lat = float('nan')
            response.lon = float('nan')
            response.alt = float('nan')

        # --- Get Velocity ---
        if self.current_velocity is not None:
            # Velocity is typically in body frame or map frame depending on source
            if target_frame == 'body' and self.current_pose is not None:
                # Transform velocity to body frame if needed
                # For simplicity, assume velocity is already in the correct frame
                # or transform based on yaw
                q = self.current_pose.pose.orientation
                _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
                
                vx_map = self.current_velocity.twist.linear.x
                vy_map = self.current_velocity.twist.linear.y
                
                # Rotate velocity from map to body frame
                cos_yaw = math.cos(-yaw)
                sin_yaw = math.sin(-yaw)
                response.vx = vx_map * cos_yaw - vy_map * sin_yaw
                response.vy = vx_map * sin_yaw + vy_map * cos_yaw
                response.vz = self.current_velocity.twist.linear.z
            else:
                # Return velocity in map frame
                response.vx = self.current_velocity.twist.linear.x
                response.vy = self.current_velocity.twist.linear.y
                response.vz = self.current_velocity.twist.linear.z
            
            # Angular rates (body frame)
            response.roll_rate = self.current_velocity.twist.angular.x
            response.pitch_rate = self.current_velocity.twist.angular.y
            response.yaw_rate = self.current_velocity.twist.angular.z
        else:
            # If no velocity topic, try to estimate from pose changes (not implemented)
            # For now, return zeros or NaN
            self.get_logger().warn("No velocity data available.", throttle_duration_sec=5.0)
            response.vx = 0.0
            response.vy = 0.0
            response.vz = 0.0
            response.roll_rate = 0.0
            response.pitch_rate = 0.0
            response.yaw_rate = 0.0

        # --- Get Battery Data ---
        if self.current_battery is not None:
            response.voltage = self.current_battery.voltage
            if len(self.current_battery.cell_voltage) > 0:
                response.cell_voltage = self.current_battery.cell_voltage[0]
            else:
                response.cell_voltage = float('nan')
        else:
            response.voltage = float('nan')
            response.cell_voltage = float('nan')

        self.get_logger().debug(
            f"Telemetry response: frame={response.frame_id}, "
            f"pos=({response.x:.2f}, {response.y:.2f}, {response.z:.2f}), "
            f"rpy=({math.degrees(response.roll):.1f}°, {math.degrees(response.pitch):.1f}°, {math.degrees(response.yaw):.1f}°)"
        )

        return response


def main(args=None):
    """Entry point for the telemetry node."""
    rclpy.init(args=args)
    
    node = TelemetryNode()
    
    # Use MultiThreadedExecutor to handle concurrent service calls
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("Telemetry node spinning...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down telemetry node...")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()