#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

# ==========================================
# 1. CONFIGURATION
# ==========================================
DEFAULT_MARKER_SIZE = 0.36

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
        
        # Publishes the Pose of the markers relative to the drone
        self.pose_pub_ = self.create_publisher(PoseStamped, 'drone_pose', 10)
        
        # Publishes the RAW error (Vector from Camera -> Center of Markers)
        self.error_pub_ = self.create_publisher(Vector3, 'aruco_error', 10)
        
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        
        self.bridge = CvBridge()
        
        # --- ARUCO SETUP ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Tuning for distance (Sensitivity)
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 2
        self.aruco_params.minMarkerPerimeterRate = 0.01 
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.standard_obj_points = get_marker_corners(DEFAULT_MARKER_SIZE)
        
        self.last_log_time = self.get_clock().now()
        self.get_logger().info("Drone Tracker Node STARTED (Centroid Mode + ID Log).")

    def image_callback(self, msg):
        current_time = self.get_clock().now()
        should_log = (current_time - self.last_log_time).nanoseconds > 1e9

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray) 
        
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            
            # --- 1. Get IDs for Logging (Visual only, not used for math) ---
            detected_ids_list = ids.flatten().tolist()
            detected_ids_list.sort()

            # --- 2. Math Processing (Agnostic to IDs) ---
            accumulated_tvec = np.array([0.0, 0.0, 0.0])
            count = 0
            ref_rvec = None 

            for i in range(len(ids)):
                img_pts = corners[i][0]
                
                success, rvec, tvec = cv2.solvePnP(
                    self.standard_obj_points, 
                    img_pts, 
                    CAMERA_MATRIX, 
                    DIST_COEFFS, 
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                
                if success:
                    accumulated_tvec += tvec.flatten()
                    count += 1
                    if ref_rvec is None: ref_rvec = rvec

            if count == 0: return

            # Calculate Average (Centroid)
            avg_x = accumulated_tvec[0] / count
            avg_y = accumulated_tvec[1] / count
            avg_z = accumulated_tvec[2] / count
            
            # --- 3. Publish /aruco_error (For PID Control) ---
            error_msg = Vector3()
            error_msg.x = float(avg_x)
            error_msg.y = float(avg_y)
            error_msg.z = float(avg_z)
            self.error_pub_.publish(error_msg)

            # --- 4. Publish /drone_pose (Global Pose Estimation) ---
            R, _ = cv2.Rodrigues(ref_rvec)
            R_inv = np.transpose(R)
            t_vec_avg = np.array([avg_x, avg_y, avg_z])
            
            cam_world_pos = -np.dot(R_inv, t_vec_avg)
            
            roll, pitch, yaw = rotationMatrixToEulerAngles(R_inv)
            q = euler_to_quaternion(roll, pitch, yaw)

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            
            msg.pose.position.x = float(cam_world_pos[0])
            msg.pose.position.y = float(cam_world_pos[1])
            msg.pose.position.z = float(cam_world_pos[2])
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            self.pose_pub_.publish(msg)

            # --- Logging ---
            if should_log:
                self.get_logger().info("========================================")
                self.get_logger().info(f" COUNT: {count} markers detected")
                self.get_logger().info(f" IDs:   {detected_ids_list}")  # <--- SHOWS IDs HERE
                self.get_logger().info(f" ERROR: X={avg_x:.2f} | Y={avg_y:.2f} | Z={avg_z:.2f}")
                self.get_logger().info("========================================")
                self.last_log_time = current_time

            # Visual Debug 
            cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, ref_rvec, np.array([avg_x, avg_y, avg_z]), 0.1)
            cv2.imshow("Centroid Tracker", frame)
            cv2.waitKey(1)

        else:
            if should_log:
                self.get_logger().warn("NO MARKERS VISIBLE")
                self.last_log_time = current_time

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