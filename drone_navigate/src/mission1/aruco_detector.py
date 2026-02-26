#!/usr/bin/env python3
"""
ArUco Detector Node for Mission 1 - Gate Passing

This node detects ArUco markers from the camera feed and publishes:
- Gate center error (for centering control)
- Number of markers detected (for FSM state transitions)
- Detection status flags

The gate is assumed to have 4 ArUco markers (IDs 0-3) at its corners.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int32MultiArray, Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time


# ==========================================
# CONFIGURATION
# ==========================================
# Marker configuration
GATE_MARKER_IDS = [0, 1, 2, 3]  # IDs of the 4 gate corner markers
DEFAULT_MARKER_SIZE = 0.18  # Marker size in meters

# Camera calibration (640x480) - adjust for your camera
FOCAL_LENGTH = 588.0
CX, CY = 320, 240  # Principal point (image center)
CAMERA_MATRIX = np.array([
    [FOCAL_LENGTH, 0, CX],
    [0, FOCAL_LENGTH, CY],
    [0, 0, 1]
], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1))

# Detection thresholds
CENTERING_THRESHOLD_X = 0.08  # meters - how close to center in X
CENTERING_THRESHOLD_Y = 0.08  # meters - how close to center in Y
MIN_DISTANCE_THRESHOLD = 0.5  # meters - minimum safe distance to gate


# ==========================================
# MATH HELPERS
# ==========================================
def get_marker_corners(size):
    """Get 3D object points for a marker of given size."""
    s = size / 2.0
    return np.array([
        [-s, -s, 0],
        [+s, -s, 0],
        [+s, +s, 0],
        [-s, +s, 0]
    ], dtype=np.float32)


def rotation_matrix_to_euler(R):
    """Convert rotation matrix to Euler angles (roll, pitch, yaw)."""
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    if sy < 1e-6:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


# ==========================================
# ARUCO DETECTOR NODE
# ==========================================
class ArucoDetectorNode(Node):
    """
    ROS 2 Node that detects ArUco markers and publishes gate centering information.
    
    Publishers:
        /aruco/gate_error (Vector3): Error from gate center (x, y, z in camera frame)
        /aruco/marker_count (Int32): Number of gate markers currently visible
        /aruco/is_centered (Bool): True if drone is centered on gate
        /aruco/is_gate_visible (Bool): True if any gate markers are visible
        /aruco/all_markers_visible (Bool): True if all 4 markers are visible
        /aruco/distance_to_gate (Float32): Estimated distance to gate center
        /target_position (Float32): Horizontal error for compatibility with tools_mission1
        
    Subscribers:
        /camera/image (Image): Camera feed from simulation/real camera
    """

    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # Declare parameters
        self.declare_parameter('marker_size', DEFAULT_MARKER_SIZE)
        self.declare_parameter('camera_topic', '/camera_frente')
        self.declare_parameter('centering_threshold_x', CENTERING_THRESHOLD_X)
        self.declare_parameter('centering_threshold_y', CENTERING_THRESHOLD_Y)
        self.declare_parameter('debug_visualization', True)
        
        self.marker_size = self.get_parameter('marker_size').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.threshold_x = self.get_parameter('centering_threshold_x').value
        self.threshold_y = self.get_parameter('centering_threshold_y').value
        self.debug_viz = self.get_parameter('debug_visualization').value
        
        # ArUco detector setup
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.obj_points = get_marker_corners(self.marker_size)
        
        # Publishers
        self.gate_error_pub = self.create_publisher(Vector3, '/aruco/gate_error', 10)
        self.marker_count_pub = self.create_publisher(Int32, '/aruco/marker_count', 10)
        self.marker_ids_pub = self.create_publisher(Int32, '/aruco/detected_marker_ids', 10)
        self.is_centered_pub = self.create_publisher(Bool, '/aruco/is_centered', 10)
        self.gate_visible_pub = self.create_publisher(Bool, '/aruco/is_gate_visible', 10)
        self.all_markers_pub = self.create_publisher(Bool, '/aruco/all_markers_visible', 10)
        self.distance_pub = self.create_publisher(Float32, '/aruco/distance_to_gate', 10)
        # Compatibility with existing tools_mission1.py
        self.target_position_pub = self.create_publisher(Float32, '/target_position', 10)
        
        # Debug image publisher
        if self.debug_viz:
            self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 
            self.camera_topic, 
            self.image_callback, 
            10
        )
        
        # State variables
        self.last_detection_time = 0.0
        self.last_log_time = self.get_clock().now()
        self.detection_timeout = 0.5  # seconds
        
        self.get_logger().info(f"ArUco Detector initialized")
        self.get_logger().info(f"  Camera topic: {self.camera_topic}")
        self.get_logger().info(f"  Marker size: {self.marker_size}m")
        self.get_logger().info(f"  Gate marker IDs: {GATE_MARKER_IDS}")

    def image_callback(self, msg):
        """Process incoming camera image and detect ArUco markers."""
        current_time = self.get_clock().now()
        should_log = (current_time - self.last_log_time).nanoseconds > 1e9  # Log every 1s
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        # Filter for gate markers only
        gate_markers = []
        gate_corners = []
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in GATE_MARKER_IDS:
                    gate_markers.append(marker_id)
                    gate_corners.append(corners[i])
        
        num_markers = len(gate_markers)
        
        # Publish marker count
        count_msg = Int32()
        count_msg.data = num_markers
        self.marker_count_pub.publish(count_msg)
        
        # Publish detected marker IDs as bit flags
        ids_msg = Int32()
        ids_bits = 0
        for mid in gate_markers:
            ids_bits |= (1 << int(mid))
        ids_msg.data = int(ids_bits)
        self.marker_ids_pub.publish(ids_msg)
        
        # Publish gate visibility
        gate_visible_msg = Bool()
        gate_visible_msg.data = num_markers > 0
        self.gate_visible_pub.publish(gate_visible_msg)
        
        # Publish all markers visible flag
        all_visible_msg = Bool()
        all_visible_msg.data = num_markers == 4
        self.all_markers_pub.publish(all_visible_msg)
        
        if num_markers == 0:
            # No markers - publish NaN error and not centered
            error_msg = Vector3()
            error_msg.x = float('nan')
            error_msg.y = float('nan')
            error_msg.z = float('nan')
            self.gate_error_pub.publish(error_msg)
            
            centered_msg = Bool()
            centered_msg.data = False
            self.is_centered_pub.publish(centered_msg)
            
            # Publish NaN to target_position for compatibility
            target_msg = Float32()
            target_msg.data = float('nan')
            self.target_position_pub.publish(target_msg)
            
            if should_log:
                self.get_logger().warn("No gate markers detected")
                self.last_log_time = current_time
            return
        
        # Process detected markers
        self.last_detection_time = time.time()
        
        # Calculate pose for each marker
        marker_poses = []  # List of (id, tvec, rvec, center_2d)
        
        for i, marker_id in enumerate(gate_markers):
            img_pts = gate_corners[i][0]
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, img_pts, CAMERA_MATRIX, DIST_COEFFS,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if success:
                center_x = np.mean(img_pts[:, 0])
                center_y = np.mean(img_pts[:, 1])
                marker_poses.append({
                    'id': marker_id,
                    'tvec': tvec.flatten(),
                    'rvec': rvec,
                    'center_2d': (center_x, center_y)
                })
        
        if not marker_poses:
            return
        
        # Calculate gate center error based on number of markers
        error_x, error_y, error_z = self._calculate_gate_error(marker_poses, num_markers)
        
        # Publish gate error
        error_msg = Vector3()
        error_msg.x = error_x
        error_msg.y = error_y
        error_msg.z = error_z
        self.gate_error_pub.publish(error_msg)
        
        # Publish distance
        distance_msg = Float32()
        distance_msg.data = error_z
        self.distance_pub.publish(distance_msg)
        
        # Publish target_position (horizontal error) for compatibility
        # Positive = gate is to the right, Negative = gate is to the left
        target_msg = Float32()
        target_msg.data = error_x * 100  # Convert to pixels-like value for compatibility
        self.target_position_pub.publish(target_msg)
        
        # Check if centered
        is_centered = (abs(error_x) < self.threshold_x and 
                       abs(error_y) < self.threshold_y and
                       num_markers >= 3)  # Need at least 3 markers for reliable centering
        
        centered_msg = Bool()
        centered_msg.data = is_centered
        self.is_centered_pub.publish(centered_msg)
        
        # Debug visualization
        if self.debug_viz:
            debug_frame = self._draw_debug_visualization(
                frame, gate_corners, gate_markers, 
                error_x, error_y, error_z, is_centered, num_markers
            )
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Debug image publish error: {e}")
        
        # Logging
        if should_log:
            self.get_logger().info("=" * 50)
            self.get_logger().info(f"Gate markers: {num_markers}/4 detected")
            self.get_logger().info(f"Marker IDs: {gate_markers}")
            self.get_logger().info(f"Gate error: X={error_x:.3f}m, Y={error_y:.3f}m, Z={error_z:.3f}m")
            self.get_logger().info(f"Centered: {is_centered}")
            self.get_logger().info("=" * 50)
            self.last_log_time = current_time

    def _calculate_gate_error(self, marker_poses, num_markers):
        """
        Calculate error from gate center based on visible markers.
        
        Returns:
            (error_x, error_y, error_z) in camera frame (meters)
            - error_x > 0: gate is to the right
            - error_y > 0: gate is below
            - error_z: distance to gate
        """
        if num_markers == 1:
            # Single marker: only Z (distance) is reliable
            t = marker_poses[0]['tvec']
            return 0.0, 0.0, float(t[2])
        
        elif num_markers == 2:
            # Two markers: determine if horizontal or vertical pair
            c1 = marker_poses[0]['center_2d']
            c2 = marker_poses[1]['center_2d']
            t1 = marker_poses[0]['tvec']
            t2 = marker_poses[1]['tvec']
            
            dx_pixel = abs(c1[0] - c2[0])
            dy_pixel = abs(c1[1] - c2[1])
            
            avg_z = (t1[2] + t2[2]) / 2.0
            
            if dx_pixel > dy_pixel:
                # Horizontal pair - X is reliable
                avg_x = (t1[0] + t2[0]) / 2.0
                return float(avg_x), 0.0, float(avg_z)
            else:
                # Vertical pair - Y is reliable
                avg_y = (t1[1] + t2[1]) / 2.0
                return 0.0, float(avg_y), float(avg_z)
        
        else:
            # 3 or 4 markers: calculate centroid
            sum_x = sum(p['tvec'][0] for p in marker_poses)
            sum_y = sum(p['tvec'][1] for p in marker_poses)
            sum_z = sum(p['tvec'][2] for p in marker_poses)
            
            avg_x = sum_x / num_markers
            avg_y = sum_y / num_markers
            avg_z = sum_z / num_markers
            
            return float(avg_x), float(avg_y), float(avg_z)

    def _draw_debug_visualization(self, frame, corners, marker_ids, 
                                   error_x, error_y, error_z, is_centered, num_markers):
        """Draw debug visualization on frame."""
        debug_frame = frame.copy()
        
        # Draw detected markers
        if corners:
            cv2.aruco.drawDetectedMarkers(debug_frame, corners)
        
        # Draw crosshair at image center
        h, w = debug_frame.shape[:2]
        cx, cy = w // 2, h // 2
        color = (0, 255, 0) if is_centered else (0, 0, 255)
        cv2.line(debug_frame, (cx - 30, cy), (cx + 30, cy), color, 2)
        cv2.line(debug_frame, (cx, cy - 30), (cx, cy + 30), color, 2)
        
        # Draw gate center (if markers visible)
        if num_markers > 0 and not math.isnan(error_x):
            # Convert error back to pixel coordinates (approximate)
            gate_px = int(cx + error_x * FOCAL_LENGTH / error_z) if error_z > 0 else cx
            gate_py = int(cy + error_y * FOCAL_LENGTH / error_z) if error_z > 0 else cy
            cv2.circle(debug_frame, (gate_px, gate_py), 10, (255, 0, 0), 2)
            cv2.line(debug_frame, (cx, cy), (gate_px, gate_py), (255, 255, 0), 2)
        
        # Status text
        status_color = (0, 255, 0) if is_centered else (0, 165, 255)
        status_text = "CENTERED" if is_centered else "ALIGNING"
        cv2.putText(debug_frame, f"Status: {status_text}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(debug_frame, f"Markers: {num_markers}/4", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_frame, f"Error X: {error_x:.3f}m", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug_frame, f"Error Y: {error_y:.3f}m", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug_frame, f"Distance: {error_z:.2f}m", (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return debug_frame


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArUco detector...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
