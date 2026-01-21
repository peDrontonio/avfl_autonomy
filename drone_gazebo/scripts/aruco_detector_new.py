#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

DEFAULT_MARKER_SIZE = 0.25
REQUIRED_MARKERS = 4 

FOCAL_LENGTH = 588.0
CX, CY = 320, 240
CAMERA_MATRIX = np.array([[FOCAL_LENGTH, 0, CX], [0, FOCAL_LENGTH, CY], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1)) 

def get_marker_corners(size):
    s = size / 2.0
    return np.array([
        [-s, -s, 0], [+s, -s, 0], 
        [+s, +s, 0], [-s, +s, 0]
    ], dtype=np.float32)

class DroneTrackerNode(Node):
    def __init__(self):
        super().__init__('drone_tracker')
        self.pose_pub_ = self.create_publisher(PoseStamped, 'drone_pose', 10)
        self.error_pub_ = self.create_publisher(Vector3, 'aruco_error', 10)
        self.state_sub = self.create_subscription(String, '/mission_state', self.state_callback, 10)
        self.current_mission_state = "IDLE"
        self.subscription = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 2
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.standard_obj_points = get_marker_corners(DEFAULT_MARKER_SIZE)
        self.get_logger().info("Detector Ready.")

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

        # HUD
        cv2.rectangle(frame, (10, 10), (350, 60), (0, 0, 0), -1)
        cv2.putText(frame, f"MODE: {self.current_mission_state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        if ids is not None and len(ids) > 0:
            accumulated_tvec = np.array([0.0, 0.0, 0.0])
            count = 0
            ref_rvec = None 
            for i in range(len(ids)):
                img_pts = corners[i][0]
                success, rvec, tvec = cv2.solvePnP(self.standard_obj_points, img_pts, CAMERA_MATRIX, DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE)
                if success:
                    accumulated_tvec += tvec.flatten()
                    count += 1
                    if ref_rvec is None: ref_rvec = rvec

            color = (0, 255, 0) if count >= REQUIRED_MARKERS else (0, 0, 255)
            cv2.putText(frame, f"Markers: {count}/{REQUIRED_MARKERS}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if count < REQUIRED_MARKERS:
                cv2.imshow("Drone HUD", frame)
                cv2.waitKey(1)
                return 

            avg_x = accumulated_tvec[0] / count
            avg_y = accumulated_tvec[1] / count
            avg_z = accumulated_tvec[2] / count
            
            error_msg = Vector3()
            error_msg.x = float(avg_x)
            error_msg.y = float(avg_y)
            error_msg.z = float(avg_z)
            self.error_pub_.publish(error_msg)
            
            if ref_rvec is not None:
                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, ref_rvec, np.array([avg_x, avg_y, avg_z]), 0.1)
                cv2.putText(frame, f"Dist: {avg_z:.2f}m", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        else:
            cv2.putText(frame, "SCANNING...", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

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