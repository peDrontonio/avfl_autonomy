#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        self.bridge = CvBridge()
        self.cap = None
        self.pub = None
        self.map1 = None
        self.map2 = None
        self.newcameramatrix = None
        self.roi = None

        self.camera_active = False
        self.camera_id = None
        self.other_cap = None
        self.other_pub = None

        # Camera 2 calibration parameters
        self.K = np.array([
            [867.5579087775083, 0.0, 696.5006675198273],
            [0.0, 868.4321850250884, 398.84707775998703],
            [0.0, 0.0, 1.0]
        ])
        self.D = np.array([0.08204167161670779, -0.12296502983205114,
                      0.0038487695384758764, 0.0069080838317992265, 0.0])
        
        # Create service to switch cameras
        self.srv = self.create_service(SetBool, '/start_camera', self.switch_camera_service)
        self.get_logger().info("Service /start_camera available. True = camera 1, False = camera 2")
        
        # Create timer for camera publishing
        self.timer = self.create_timer(1.0/20.0, self.timer_callback)  # 20 Hz

    def close_other_camera(self):
        """Close previously opened camera"""
        if self.other_cap is not None:
            self.get_logger().info("Closing previously opened camera")
            self.other_cap.release()
            self.other_cap = None
        if self.other_pub is not None:
            self.other_pub = None

    def switch_camera_service(self, request, response):
        """Service callback to switch between cameras"""
        cam_id = 1 if request.data else 2  # True = camera 1, False = camera 2

        if self.camera_active and self.camera_id == cam_id:
            response.success = False
            response.message = f"Camera {self.camera_id} is already active"
            return response

        # Close the other camera if open
        self.close_other_camera()

        if cam_id == 1:
            self.get_logger().info("Initializing Camera 1 (front)")
            self.pub = self.create_publisher(Image, '/camera_frente', 3)
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            if not self.cap.isOpened():
                response.success = False
                response.message = "Could not open Camera 1"
                return response

        elif cam_id == 2:
            self.get_logger().info("Initializing Camera 2 (bottom)")
            self.pub = self.create_publisher(Image, '/camera_baixo', 3)

            self.cap = cv2.VideoCapture(2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            if not self.cap.isOpened():
                response.success = False
                response.message = "Could not open Camera 2"
                return response

        # Store references to close on next activation
        self.other_cap = self.cap
        self.other_pub = self.pub
        self.camera_active = True
        self.camera_id = cam_id

        response.success = True
        response.message = f"Camera {self.camera_id} activated"
        return response

    def timer_callback(self):
        """Timer callback to publish camera frames"""
        if self.camera_active and self.cap is not None and self.pub is not None:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture image")
                return

            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = f"camera_{self.camera_id}"
                self.pub.publish(img_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Image conversion error: {e}")
    
    def cleanup(self):
        """Cleanup resources on shutdown"""
        if self.cap is not None:
            self.cap.release()
        if self.other_cap is not None:
            self.other_cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.cleanup()
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
