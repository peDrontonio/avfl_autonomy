#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import sys

# ==========================================
# 1. CONFIGURATION
# ==========================================
MARKER_SIZE = 0.18      
GAP_X = 1.5 - MARKER_SIZE            
GAP_Y = 1.5 - MARKER_SIZE            

# Camera Calibration
FOCAL_LENGTH = 1100
CX, CY = 640, 360
CAMERA_MATRIX = np.array([[FOCAL_LENGTH, 0, CX], [0, FOCAL_LENGTH, CY], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1)) 

dx = (GAP_X / 2) + (MARKER_SIZE / 2)
dy = (GAP_Y / 2) + (MARKER_SIZE / 2)

# ==========================================
# 2. MATH HELPERS
# ==========================================
def get_marker_corners(center_x, center_y, size):
    s = size / 2
    return np.array([
        [center_x - s, center_y - s, 0], 
        [center_x + s, center_y - s, 0], 
        [center_x + s, center_y + s, 0], 
        [center_x - s, center_y + s, 0]
    ], dtype=np.float32)

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles to Quaternion (x, y, z, w)
    ROS uses Radians for this input.
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    # Returns Radians for ROS compatibility
    return x, y, z

# ==========================================
# 3. ROS2 NODE CLASS
# ==========================================

class DroneTrackerNode(Node):
    def __init__(self):
        super().__init__('drone_tracker')
        
        # Publisher
        # Topic: /drone_pose
        # Type: PoseStamped (includes Header + Position + Orientation)
        self.publisher_ = self.create_publisher(PoseStamped, 'drone_pose', 10)
        
        # Subscriber
        # Topic: /camera (from Gazebo simulation)
        # Type: Image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
        
        # CvBridge for converting ROS Image to OpenCV
        self.bridge = CvBridge()
        
        # Aruco Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Map Setup
        self.obj_points_map = {
            2: get_marker_corners(-dx, -dy, MARKER_SIZE),
            3: get_marker_corners(+dx, -dy, MARKER_SIZE),
            1: get_marker_corners(+dx, +dy, MARKER_SIZE),
            0: get_marker_corners(-dx, +dy, MARKER_SIZE)
        }
        
        self.get_logger().info("Drone Tracker Node Started. Subscribing to /camera, Publishing to /drone_pose")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            all_img_pts = []
            all_obj_pts = []
            
            for i in range(len(ids)):
                current_id = ids[i][0]
                if current_id in self.obj_points_map:
                    all_img_pts.extend(corners[i][0])
                    all_obj_pts.extend(self.obj_points_map[current_id])

            if len(all_obj_pts) >= 4:
                img_pts_np = np.array(all_img_pts, dtype=np.float32)
                obj_pts_np = np.array(all_obj_pts, dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(obj_pts_np, img_pts_np, CAMERA_MATRIX, DIST_COEFFS)

                if success:
                    # 1. Calc Position
                    R, _ = cv2.Rodrigues(rvec)
                    R_inv = np.transpose(R)
                    cam_pos_world = -np.dot(R_inv, tvec)

                    # 2. Calc Rotation (Radians)
                    roll, pitch, yaw = rotationMatrixToEulerAngles(R_inv)
                    
                    # 3. Create ROS Message
                    msg = PoseStamped()
                    
                    # Header
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_optical_frame" 
                    
                    # Position
                    msg.pose.position.x = float(cam_pos_world[0])
                    msg.pose.position.y = float(cam_pos_world[1])
                    msg.pose.position.z = float(cam_pos_world[2])
                    
                    # Orientation (Convert Euler to Quaternion)
                    q = euler_to_quaternion(roll, pitch, yaw)
                    msg.pose.orientation.x = q[0]
                    msg.pose.orientation.y = q[1]
                    msg.pose.orientation.z = q[2]
                    msg.pose.orientation.w = q[3]

                    # Publish
                    self.publisher_.publish(msg)
                    
                    # Optional: Visual Debugging (imshow in ROS node is okay for testing, bad for production headless)
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, 0.1)
                    cv2.imshow("ROS2 Drone Tracker", frame)
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
