#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drone_navigate.msg import BaseDetection


class ColorBaseDetector(Node):
    def __init__(self):
        super().__init__('color_base_detector')

        self.bridge = CvBridge()

        # HSV color filter parameters
        self.lower_hsv = np.array([96, 8, 88])
        self.upper_hsv = np.array([126, 113, 255])

        # Minimum contour area to consider a valid detection (pixels²)
        self.min_contour_area = 500

        # Publishers
        self.detection_pub = self.create_publisher(BaseDetection, '/color/base_detection', 10)
        self.image_pub = self.create_publisher(Image, '/color/base_detection/image', 10)

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

        self.get_logger().info("Color Base Detector node initialized")
        self.get_logger().info(f"  Lower HSV: {self.lower_hsv}")
        self.get_logger().info(f"  Upper HSV: {self.upper_hsv}")

    def camera_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = frame.shape[:2]
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Convert to HSV and apply color filter
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, self.lower_hsv, self.upper_hsv)

        # Morphological operations to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create detection message
        detection_msg = BaseDetection()
        detection_msg.header = msg.header
        detection_msg.detected = False

        # Copy image for visualization
        output_frame = frame.copy()

        # Draw center crosshair
        center_x = self.image_width // 2
        center_y = self.image_height // 2
        crosshair_size = 20
        cv2.line(output_frame, (center_x - crosshair_size, center_y),
                 (center_x + crosshair_size, center_y), (255, 0, 0), 2)
        cv2.line(output_frame, (center_x, center_y - crosshair_size),
                 (center_x, center_y + crosshair_size), (255, 0, 0), 2)
        cv2.circle(output_frame, (center_x, center_y), 5, (255, 0, 0), 2)

        # Filter contours by minimum area and find the largest one
        valid_contours = [c for c in contours if cv2.contourArea(c) >= self.min_contour_area]

        if valid_contours:
            # Pick the largest contour as the base
            best_contour = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(best_contour)

            # Bounding rect
            bx, by, bw, bh = cv2.boundingRect(best_contour)

            # Calculate centroid using moments (more accurate for irregular shapes)
            M = cv2.moments(best_contour)
            if M["m00"] != 0:
                x_center = int(M["m10"] / M["m00"])
                y_center = int(M["m01"] / M["m00"])
            else:
                # Fallback to bounding box center
                x_center = bx + bw // 2
                y_center = by + bh // 2
            
            # Log detection info
            self.get_logger().info(f"Color Base Detected! Area: {area:.0f}, Centroid: ({x_center}, {y_center})", throttle_duration_sec=3.0)

            # Confidence based on area ratio (larger area -> higher confidence)
            max_area = self.image_width * self.image_height * 0.5  # 50% of image as max reference
            confidence = min(float(area) / max_area, 1.0)

            # Update detection message
            detection_msg.detected = True
            detection_msg.confidence = float(confidence)
            detection_msg.x_center = x_center
            detection_msg.y_center = y_center
            detection_msg.bbox_width = bw
            detection_msg.bbox_height = bh

            # Track consecutive detections
            self.consecutive_detections += 1
            self.detection_history.append((x_center, y_center, bw, bh))

            if len(self.detection_history) > self.history_size:
                self.detection_history.pop(0)

            # === Draw bounding box and center ===
            cv2.rectangle(output_frame, (bx, by), (bx + bw, by + bh), (0, 255, 0), 3)
            cv2.circle(output_frame, (x_center, y_center), 6, (0, 0, 255), -1)

            # Draw line from detection center to image center
            cv2.line(output_frame, (x_center, y_center), (center_x, center_y), (255, 255, 0), 2)

            # Draw all valid contours
            cv2.drawContours(output_frame, valid_contours, -1, (0, 255, 255), 1)

            # Labels
            label = f"Base (color): {confidence:.2f} | Area: {area:.0f}"
            cv2.putText(output_frame, label, (bx, by - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Offset from center
            offset_x = x_center - center_x
            offset_y = y_center - center_y
            offset_text = f"Offset: X={offset_x:+d} Y={offset_y:+d}"
            cv2.putText(output_frame, offset_text, (bx, by - 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # Status text at top
            status_text = f"DETECTED | Consecutive: {self.consecutive_detections}"
            cv2.putText(output_frame, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            self.consecutive_detections = 0
            self.detection_history = []
            # Status text at top
            status_text = "NO BASE DETECTED (color filter)"
            cv2.putText(output_frame, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.get_logger().warn("Base not detected by color filter", throttle_duration_sec=5.0)

        # Draw the mask as a small overlay in the corner for debugging
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_color, (160, 120))
        output_frame[0:120, self.image_width - 160:self.image_width] = mask_small

        # === Publish annotated image ===
        try:
            image_msg = self.bridge.cv2_to_imgmsg(output_frame, "bgr8")
            image_msg.header = msg.header
            self.image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing annotated image: {e}")

        # Publish detection data
        self.detection_pub.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        detector = ColorBaseDetector()
        rclpy.spin(detector)
    except Exception as e:
        print(f"Error in Color Base Detector: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
