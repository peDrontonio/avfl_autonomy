#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import String  # For receiving mission state
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

# ==========================================
# 1. CONFIGURATION
# ==========================================
DEFAULT_MARKER_SIZE = 0.25
REQUIRED_MARKERS = 4 

# Camera Calibration (640x480)
FOCAL_LENGTH = 588.0
CX, CY = 320, 240
CAMERA_MATRIX = np.array([[FOCAL_LENGTH, 0, CX], [0, FOCAL_LENGTH, CY], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1)) 

# ==========================================
# 2. MATH HELPERS
# ==========================================
def get_marker_corners(size):
    s = size / 2.0
    return np.array([
        [-s, -s, 0], [+s, -s, 0], 
        [+s, +s, 0], [-s, +s, 0]
    ], dtype=np.float32)

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    if sy < 1e-6:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    else:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    return x, y, z

# ==========================================
# 3. ROS2 NODE CLASS
# ==========================================

class DroneTrackerNode(Node):
    def __init__(self):
        super().__init__('drone_tracker')
        
        self.pose_pub_ = self.create_publisher(PoseStamped, 'drone_pose', 10)
        self.error_pub_ = self.create_publisher(Vector3, 'aruco_error', 10)
        
        # Subscribe to Mission State for HUD
        self.state_sub = self.create_subscription(String, '/mission_state', self.state_callback, 10)
        self.current_mission_state = "IDLE"

        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        
        self.bridge = CvBridge()
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Tuning for better detection at distance
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 2
        self.aruco_params.minMarkerPerimeterRate = 0.01 
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.standard_obj_points = get_marker_corners(DEFAULT_MARKER_SIZE)
        
        self.last_log_time = self.get_clock().now()
        self.get_logger().info(f"Drone Tracker HUD Ready. Waiting for images...")

    def state_callback(self, msg):
        self.current_mission_state = msg.data

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray) 
        
        corners, ids, _ = self.detector.detectMarkers(gray)

        # Draw HUD (Heads Up Display)
        # Background box for text
        cv2.rectangle(frame, (10, 10), (350, 60), (0, 0, 0), -1)
        cv2.putText(frame, f"MODE: {self.current_mission_state}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        if ids is not None and len(ids) > 0:
            
            detected_ids_list = ids.flatten().tolist()
            detected_ids_list.sort()

            accumulated_tvec = np.array([0.0, 0.0, 0.0])
            count = 0
            ref_rvec = None 

            for i in range(len(ids)):
                img_pts = corners[i][0]
                success, rvec, tvec = cv2.solvePnP(
                    self.standard_obj_points, img_pts, CAMERA_MATRIX, DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE
                )
                if success:
                    accumulated_tvec += tvec.flatten()
                    count += 1
                    if ref_rvec is None: ref_rvec = rvec

            # --- HUD: Marker Count ---
            color = (0, 255, 0) if count >= REQUIRED_MARKERS else (0, 0, 255)
            cv2.putText(frame, f"Markers: {count}/{REQUIRED_MARKERS}", (20, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Draw all markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # --- STRICT CHECK ---
            if count < REQUIRED_MARKERS:
                cv2.imshow("Drone HUD", frame)
                cv2.waitKey(1)
                return 

            # --- Found 4 Markers ---
            avg_x = accumulated_tvec[0] / count
            avg_y = accumulated_tvec[1] / count
            avg_z = accumulated_tvec[2] / count
            
            # Publish Error
            error_msg = Vector3()
            error_msg.x = float(avg_x)
            error_msg.y = float(avg_y)
            error_msg.z = float(avg_z)
            self.error_pub_.publish(error_msg)
            
            # Draw Axis
            if ref_rvec is not None:
                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, ref_rvec, np.array([avg_x, avg_y, avg_z]), 0.1)
                # Show Distance on HUD
                cv2.putText(frame, f"Dist: {avg_z:.2f}m", (20, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        else:
            cv2.putText(frame, "SCANNING...", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        # Show the window (Keep it open!)
        cv2.imshow("Drone HUD", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DroneTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()