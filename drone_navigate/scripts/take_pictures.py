#!/usr/bin/env python3
"""
Standalone camera capture script.

Subscribes to the same ROS 2 camera topic as the ArUco detector and saves
frames to disk on demand (press ENTER) or at a fixed interval.

Usage:
    python3 take_pictures.py
    python3 take_pictures.py --topic /camera/image --save-dir ~/Pictures/captures
    python3 take_pictures.py --auto --interval 2.0
"""

import argparse
import os
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraCapture(Node):
    def __init__(self, topic: str, save_dir: str, auto: bool, interval: float):
        super().__init__('camera_capture')
        self.save_dir = os.path.expanduser(save_dir)
        self.auto = auto
        self.interval = interval
        self.bridge = CvBridge()
        self.current_frame = None
        self._lock = threading.Lock()
        self._last_auto = time.time()
        self._count = 0

        os.makedirs(self.save_dir, exist_ok=True)

        self.sub = self.create_subscription(Image, topic, self._cb, 10)
        self.get_logger().info(f"Subscribed to: {topic}")
        self.get_logger().info(f"Saving to:     {self.save_dir}")
        if auto:
            self.get_logger().info(f"Auto-capture every {interval}s  (Ctrl-C to stop)")
        else:
            self.get_logger().info("Press ENTER to capture a frame  (Ctrl-C to stop)")

    def _cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        with self._lock:
            self.current_frame = frame.copy()

        if self.auto:
            now = time.time()
            if now - self._last_auto >= self.interval:
                self._save()
                self._last_auto = now

    def capture(self):
        """Save the latest frame. Called from the ENTER-listener thread."""
        self._save()

    def _save(self):
        with self._lock:
            frame = self.current_frame

        if frame is None:
            print("[!] No frame received yet.", flush=True)
            return

        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        self._count += 1
        path = os.path.join(self.save_dir, f"capture_{ts}.jpg")
        cv2.imwrite(path, frame)
        print(f"[{self._count:04d}] Saved: {path}", flush=True)


def input_listener(node: CameraCapture, stop_event: threading.Event):
    """Background thread: wait for ENTER key presses."""
    while not stop_event.is_set():
        try:
            input()          # blocks until ENTER
        except EOFError:
            break
        if not stop_event.is_set():
            node.capture()


def main():
    parser = argparse.ArgumentParser(description='ROS 2 camera frame capture tool')
    parser.add_argument('--topic',    default='/camera_frente',
                        help='Camera image topic (default: /camera/image)')
    parser.add_argument('--save-dir', default='~/avfl_ws/src/drone_navigate/captures',
                        help='Directory to save images (default: ~/Pictures/captures)')
    parser.add_argument('--auto',     action='store_true',
                        help='Auto-capture at fixed interval instead of ENTER prompt')
    parser.add_argument('--interval', type=float, default=2.0,
                        help='Seconds between auto-captures (default: 2.0)')
    args = parser.parse_args()

    rclpy.init()
    node = CameraCapture(args.topic, args.save_dir, args.auto, args.interval)

    stop_event = threading.Event()

    if not args.auto:
        t = threading.Thread(target=input_listener, args=(node, stop_event), daemon=True)
        t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\nDone. {node._count} image(s) saved to {node.save_dir}")


if __name__ == '__main__':
    main()
