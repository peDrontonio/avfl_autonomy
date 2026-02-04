#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drone_navigate.msg import BaseDetection
from ultralytics import YOLO
import torch
import os

class YOLOBaseDetector(Node):
    def __init__(self):
        super().__init__('yolo_base_detector')
        
        # Load YOLO model
        model_path = os.path.join(os.path.dirname(__file__), 'yolo/modelo_fumaca.pt')
        if not os.path.exists(model_path):
            self.get_logger().error(f"YOLO model not found at {model_path}")
            raise FileNotFoundError(f"YOLO model not found at {model_path}")

        self.model = YOLO(model_path)
        self.conf = 0.7
        
        self.bridge = CvBridge()
        self.detection_pub = self.create_publisher(BaseDetection, '/yolo/base_detection', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/base_detection/image', 10)
        
        # Subscribe to bottom camera topic for downward-facing detection
        self.create_subscription(Image, '/bottom_camera', self.camera_callback, 10)
        
        
        # Consecutive detection tracking
        self.consecutive_detections = 0
        self.required_consecutive_detections = 20
        
        # Image dimensions for center calculation
        self.image_width = 640
        self.image_height = 480
        
        # Detection history for smoothing
        self.detection_history = []
        self.history_size = 3
        
        self.get_logger().info("YOLO Base Detector node initialized")

    def camera_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = frame.shape[:2]
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Run YOLO detection
        with torch.no_grad():
            results = self.model(frame, conf = self.conf, verbose = False)
            
        detections = results[0].boxes.data.cpu().numpy()
        
        # Create detection message
        detection_msg = BaseDetection()
        detection_msg.header = msg.header
        detection_msg.detected = False
        
        # Copy image for visualization
        output_frame = frame.copy()

        if len(detections) > 0:
            best_detection = None
            max_confidence = 0
            
            for detection in detections:
                xmin, ymin, xmax, ymax, confidence, class_id = detection
                if class_id == 0 and confidence > max_confidence:  # Class 0 = base móvel
                    max_confidence = confidence
                    best_detection = detection
            
            if best_detection is not None:
                xmin, ymin, xmax, ymax, confidence, class_id = best_detection
                
                # Calculate center
                x_center = int((xmin + xmax) / 2)
                y_center = int((ymin + ymax) / 2)
                bbox_width = int(xmax - xmin)
                bbox_height = int(ymax - ymin)
                
                # Update detection message
                detection_msg.detected = True
                detection_msg.confidence = float(confidence)
                detection_msg.x_center = x_center
                detection_msg.y_center = y_center
                detection_msg.bbox_width = bbox_width
                detection_msg.bbox_height = bbox_height
                
                # Track consecutive detections
                self.consecutive_detections += 1
                self.detection_history.append((x_center, y_center, bbox_width, bbox_height))
                
                if len(self.detection_history) > self.history_size:
                    self.detection_history.pop(0)
                
                # === Desenha bounding box e centro ===
                cv2.rectangle(output_frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                cv2.circle(output_frame, (x_center, y_center), 4, (0, 0, 255), -1)
                label = f"Base: {confidence:.2f}"
                cv2.putText(output_frame, label, (int(xmin), int(ymin) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                self.consecutive_detections = 0
                self.detection_history = []
                self.get_logger().warn("Base not detected in current frame", throttle_duration_sec=5.0)
        else:
            self.consecutive_detections = 0
            self.detection_history = []
            self.get_logger().warn("No objects detected in current frame", throttle_duration_sec=5.0)

        # === Publica imagem anotada ===
        try:
            image_msg = self.bridge.cv2_to_imgmsg(output_frame, "bgr8")
            image_msg.header = msg.header
            self.image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing annotated image: {e}")

        # Publica dados de detecção
        self.detection_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        detector = YOLOBaseDetector()
        rclpy.spin(detector)
    except Exception as e:
        print(f"Error in YOLO Base Detector: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()