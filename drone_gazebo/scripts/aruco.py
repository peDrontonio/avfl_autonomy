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
DEFAULT_MARKER_SIZE = 0.18

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
        
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.standard_obj_points = get_marker_corners(DEFAULT_MARKER_SIZE)
        
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info("Drone Tracker Node STARTED (With Status Flags).")

    def image_callback(self, msg):
        current_time = self.get_clock().now()
        # Logar apenas a cada 1 segundo (1e9 nanosegundos)
        should_log = (current_time - self.last_log_time).nanoseconds > 1e9

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            
            detected_tvecs = []
            detected_centers_2d = [] 
            ref_rvec = None 
            
            # --- PROCESSAMENTO ---
            for i in range(len(ids)):
                img_pts = corners[i][0]
                success, rvec, tvec = cv2.solvePnP(
                    self.standard_obj_points, img_pts, CAMERA_MATRIX, DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE
                )
                
                if success:
                    detected_tvecs.append(tvec.flatten()) 
                    c_x = np.mean(img_pts[:, 0])
                    c_y = np.mean(img_pts[:, 1])
                    detected_centers_2d.append((c_x, c_y))
                    if ref_rvec is None: ref_rvec = rvec

            count = len(detected_tvecs)
            if count == 0: return

            # --- LÓGICA E FLAGS ---
            final_x, final_y, final_z = 0.0, 0.0, 0.0
            log_mode_str = ""
            detected_ids_list = ids.flatten().tolist()

            # CASO 1: Apenas 1 Aruco
            if count == 1:
                t = detected_tvecs[0]
                final_x = 0.0  
                final_y = 0.0  
                final_z = float(t[2])
                log_mode_str = "SINGLE MARKER (Z Only)"

            # CASO 2: 2 Arucos
            elif count == 2:
                t1, t2 = detected_tvecs[0], detected_tvecs[1]
                c1, c2 = detected_centers_2d[0], detected_centers_2d[1]
                
                dx_pixel = abs(c1[0] - c2[0])
                dy_pixel = abs(c1[1] - c2[1])
                
                if dx_pixel > dy_pixel:
                    # Horizontais
                    final_x = float((t1[0] + t2[0]) / 2.0)
                    final_y = 0.0 
                    final_z = float((t1[2] + t2[2]) / 2.0)
                    log_mode_str = "DUAL HORIZONTAL (Avg X, Z | Y Locked)"
                else:
                    # Verticais
                    final_x = 0.0
                    final_y = float((t1[1] + t2[1]) / 2.0)
                    final_z = float((t1[2] + t2[2]) / 2.0)
                    log_mode_str = "DUAL VERTICAL (Avg Y, Z | X Locked)"

            # CASO 3: 3+ Arucos
            else:
                sum_x = sum(t[0] for t in detected_tvecs)
                sum_y = sum(t[1] for t in detected_tvecs)
                sum_z = sum(t[2] for t in detected_tvecs)
                
                final_x = float(sum_x / count)
                final_y = float(sum_y / count)
                final_z = float(sum_z / count)
                log_mode_str = f"MULTI MARKER ({count}) (Full Centroid)"

            # --- PUBLICAÇÃO ---
            R, _ = cv2.Rodrigues(ref_rvec)
            R_inv = np.transpose(R) 
            roll, pitch, yaw = rotationMatrixToEulerAngles(R_inv)
            q = euler_to_quaternion(roll, pitch, yaw)

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_visual_center"
            msg.pose.position.x = final_x
            msg.pose.position.y = final_y
            msg.pose.position.z = final_z
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            self.pose_pub_.publish(msg)

            error_msg = Vector3()
            error_msg.x = final_x
            error_msg.y = final_y
            error_msg.z = final_z
            self.error_pub_.publish(error_msg)

            # --- LOGGING / FLAGS ---
            if should_log:
                self.get_logger().info("========================================")
                self.get_logger().info(f" VISUAL: {count} ArUcos detected")
                self.get_logger().info(f" IDs:    {detected_ids_list}")
                self.get_logger().info(f" MODE:   {log_mode_str}")
                self.get_logger().info(f" PUB /aruco_error -> X: {final_x:.3f} | Y: {final_y:.3f} | Z: {final_z:.3f}")
                self.get_logger().info("========================================")
                self.last_log_time = current_time

        else:
            # Log de "Nada encontrado"
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