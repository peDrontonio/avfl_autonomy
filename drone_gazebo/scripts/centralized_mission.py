#!/usr/bin/env python3
"""
Centralized Mission Node - ArUco-based arch traversal.

This mission makes the drone:
1. Search for ArUco markers by moving right/left
2. Center on the arch using ArUco markers
3. Fly through the centered arch

Author: AVFL Autonomy Brazilians Team
Date: January 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from enum import Enum


class MissionState(Enum):
    IDLE = 0
    SEARCHING_RIGHT = 1
    SEARCHING_LEFT = 2
    ADVANCING = 3
    CENTERING_X = 4
    CENTERING_Y = 5
    FLY_THROUGH = 6
    COMPLETED = 7


class CentralizedMissionNode(Node):
    def __init__(self):
        super().__init__('centralized_mission_node')
        
        # ==========================================
        # PARAMETERS
        # ==========================================
        self.declare_parameter('search_velocity', 1.0)  # m/s for searching
        self.declare_parameter('centering_velocity', 0.3)  # m/s for centering (slower)
        self.declare_parameter('fly_velocity', 1.0)  # m/s for flying through
        self.declare_parameter('search_distance', 3.0)  # meters to search before turning
        self.declare_parameter('centering_threshold_x', 20.0)  # pixels tolerance for X centering
        self.declare_parameter('centering_threshold_y', 20.0)  # pixels tolerance for Y centering
        self.declare_parameter('fly_through_distance', 5.0)  # meters to fly after centering
        
        self.search_velocity = self.get_parameter('search_velocity').value
        self.centering_velocity = self.get_parameter('centering_velocity').value
        self.fly_velocity = self.get_parameter('fly_velocity').value
        self.search_distance = self.get_parameter('search_distance').value
        self.centering_threshold_x = self.get_parameter('centering_threshold_x').value
        self.centering_threshold_y = self.get_parameter('centering_threshold_y').value
        self.fly_through_distance = self.get_parameter('fly_through_distance').value
        
        # ==========================================
        # ARUCO CONFIGURATION
        # ==========================================
        # ArUco marker layout on the arch:
        #   ArUco 0 (top-front)    ArUco 1 (top-back)
        #   ArUco 3 (bottom-front) ArUco 2 (bottom-back)
        # 
        # From drone's perspective (looking at arch):
        # - Left markers: 0, 3 (front side of arch)
        # - Right markers: 1, 2 (back side of arch)
        # - Top markers: 0, 1
        # - Bottom markers: 2, 3
        
        # IMPORTANT: Markers were generated with DICT_6X6_250
        # Using new OpenCV 4.7+ API with ArucoDetector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Optimize detection parameters for better performance
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_params.minCornerDistanceRate = 0.05
        self.aruco_params.minDistanceToBorder = 3
        self.aruco_params.minMarkerDistanceRate = 0.05
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.1
        
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Marker IDs for centering
        self.left_markers = [0, 3]   # Markers on the left side
        self.right_markers = [1, 2]  # Markers on the right side
        self.top_markers = [0, 1]    # Markers on top
        self.bottom_markers = [2, 3] # Markers on bottom
        
        # ==========================================
        # STATE VARIABLES
        # ==========================================
        self.state = MissionState.IDLE
        self.prev_state = MissionState.IDLE
        self.move_start_time = None
        self.search_direction = 1  # 1 = right (Y-), -1 = left (Y+)
        self.distance_traveled = 0.0
        self.search_count = 0  # Count number of searches
        self.advance_distance = 0.5  # Distance to advance forward (meters)
        
        # Detected ArUco markers (updated by camera callback)
        self.detected_markers = {}  # {id: {'corners': [...], 'center': (cx, cy)}}
        self.markers_lock = False
        
        # Logging control
        self.last_marker_count = -1
        self.log_counter = 0
        self.LOG_INTERVAL = 10  # Log every N iterations
        
        # Image dimensions (will be set from camera)
        self.image_width = 1280
        self.image_height = 720
        self.image_center_x = self.image_width / 2
        self.image_center_y = self.image_height / 2
        
        # ==========================================
        # QOS PROFILE
        # ==========================================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ==========================================
        # PUBLISHERS & SUBSCRIBERS
        # ==========================================
        self.vel_pub = self.create_publisher(
            TwistStamped, 
            '/ap/cmd_vel', 
            qos_profile
        )
        
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.camera_callback,
            10
        )
        
        # Current velocity command
        self.current_vel_cmd = TwistStamped()
        
        # ==========================================
        # TIMERS
        # ==========================================
        # Main mission timer (10 Hz)
        self.mission_timer = self.create_timer(0.1, self.mission_loop)
        
        # Velocity command timer (20 Hz)
        self.vel_timer = self.create_timer(0.05, self.velocity_command_loop)
        
        # ==========================================
        # STARTUP
        # ==========================================
        self.get_logger().info('=== Centralized Mission Node Started ===')
        self.get_logger().info(f'Search velocity: {self.search_velocity}m/s')
        self.get_logger().info(f'Centering velocity: {self.centering_velocity}m/s')
        self.get_logger().info(f'Search distance: {self.search_distance}m')
        self.get_logger().info(f'Centering threshold X: {self.centering_threshold_x}px')
        self.get_logger().info(f'Centering threshold Y: {self.centering_threshold_y}px')
        
        # Start mission after small delay
        self.create_timer(2.0, self.start_mission)
        self.mission_started = False

    # ==========================================
    # CAMERA CALLBACK
    # ==========================================
    def camera_callback(self, msg):
        """Process camera images to detect ArUco markers."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Update image dimensions
            self.image_height, self.image_width = frame.shape[:2]
            self.image_center_x = self.image_width / 2
            self.image_center_y = self.image_height / 2
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Apply image preprocessing for better detection
            # Equalize histogram to improve contrast
            gray = cv2.equalizeHist(gray)
            
            # Optional: Apply slight Gaussian blur to reduce noise
            gray = cv2.GaussianBlur(gray, (3, 3), 0)
            
            # Detect ArUco markers using new OpenCV 4.7+ API
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            
            # Update detected markers
            self.detected_markers = {}
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    marker_corners = corners[i][0]
                    center_x = np.mean(marker_corners[:, 0])
                    center_y = np.mean(marker_corners[:, 1])
                    self.detected_markers[marker_id] = {
                        'corners': marker_corners,
                        'center': (center_x, center_y)
                    }
            
            # Log marker detection changes
            current_count = len(self.detected_markers)
            if current_count != self.last_marker_count:
                if current_count == 0:
                    self.get_logger().info('No ArUco markers detected')
                elif current_count == 1:
                    marker_ids = list(self.detected_markers.keys())
                    self.get_logger().info(f'1 ArUco marker detected (ID: {marker_ids[0]})')
                else:
                    marker_ids = list(self.detected_markers.keys())
                    self.get_logger().info(f'{current_count} ArUco markers detected (IDs: {marker_ids})')
                self.last_marker_count = current_count
            
            # Optional: Display debug visualization
            self.display_debug_image(frame, corners, ids)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def display_debug_image(self, frame, corners, ids):
        """Display debug visualization with detected markers and centering info."""
        display_frame = frame.copy()
        
        # Draw detected markers
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
        
        # Draw center crosshair
        cv2.line(display_frame, 
                 (int(self.image_center_x), 0), 
                 (int(self.image_center_x), self.image_height), 
                 (0, 255, 0), 1)
        cv2.line(display_frame, 
                 (0, int(self.image_center_y)), 
                 (self.image_width, int(self.image_center_y)), 
                 (0, 255, 0), 1)
        
        # Draw arch center if markers detected
        arch_center = self.calculate_arch_center()
        if arch_center:
            cv2.circle(display_frame, (int(arch_center[0]), int(arch_center[1])), 10, (0, 0, 255), -1)
            cv2.line(display_frame, 
                     (int(self.image_center_x), int(self.image_center_y)),
                     (int(arch_center[0]), int(arch_center[1])),
                     (255, 0, 0), 2)
        
        # Display state
        state_text = f"State: {self.state.name}"
        cv2.putText(display_frame, state_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Display detected markers
        markers_text = f"Markers: {list(self.detected_markers.keys())}"
        cv2.putText(display_frame, markers_text, (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.imshow('Centralized Mission', display_frame)
        cv2.waitKey(1)

    # ==========================================
    # ARCH CENTER CALCULATION
    # ==========================================
    def calculate_arch_center(self):
        """Calculate the center of the arch based on detected markers."""
        if not self.detected_markers:
            return None
        
        # Get all detected marker centers
        centers = [data['center'] for data in self.detected_markers.values()]
        
        if len(centers) >= 2:
            # Calculate average of all marker centers
            avg_x = np.mean([c[0] for c in centers])
            avg_y = np.mean([c[1] for c in centers])
            return (avg_x, avg_y)
        elif len(centers) == 1:
            # Use single marker center
            return centers[0]
        
        return None

    def calculate_x_error(self):
        """
        Calculate X error for centering.
        Positive error means drone should move right (Y-)
        Negative error means drone should move left (Y+)
        """
        arch_center = self.calculate_arch_center()
        if arch_center:
            # Error is how far the arch center is from image center
            # In image: X increases to the right
            # If arch is to the right of center, we need to move right (Y- in drone frame)
            error_x = arch_center[0] - self.image_center_x
            return error_x
        return None

    def calculate_y_error(self):
        """
        Calculate Y error for centering (vertical).
        Positive error means drone should move down (Z-)
        Negative error means drone should move up (Z+)
        """
        arch_center = self.calculate_arch_center()
        if arch_center:
            # Error is how far the arch center is from image center
            # In image: Y increases downward
            # If arch is below center, we need to move down (Z-)
            error_y = arch_center[1] - self.image_center_y
            return error_y
        return None

    # ==========================================
    # VELOCITY CONTROL
    # ==========================================
    def start_mission(self):
        """Start the mission."""
        if not self.mission_started:
            self.mission_started = True
            self.state = MissionState.SEARCHING_RIGHT
            self.move_start_time = time.time()
            self.get_logger().info('Starting centralized mission!')
            self.get_logger().info('Searching for ArUco markers (moving right)...')

    def velocity_command_loop(self):
        """Loop to send velocity commands continuously."""
        active_states = [
            MissionState.SEARCHING_RIGHT, 
            MissionState.SEARCHING_LEFT,
            MissionState.ADVANCING,
            MissionState.CENTERING_X,
            MissionState.CENTERING_Y,
            MissionState.FLY_THROUGH
        ]
        if self.state in active_states:
            self.current_vel_cmd.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(self.current_vel_cmd)

    def set_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """Set the current velocity command."""
        self.current_vel_cmd = TwistStamped()
        self.current_vel_cmd.header.stamp = self.get_clock().now().to_msg()
        self.current_vel_cmd.header.frame_id = 'base_link'
        self.current_vel_cmd.twist.linear.x = float(vx)
        self.current_vel_cmd.twist.linear.y = float(vy)
        self.current_vel_cmd.twist.linear.z = float(vz)
        self.current_vel_cmd.twist.angular.z = float(yaw_rate)

    def stop_movement(self):
        """Stop drone movement."""
        self.set_velocity(0.0, 0.0, 0.0, 0.0)
        for _ in range(5):
            self.current_vel_cmd.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(self.current_vel_cmd)
            time.sleep(0.05)

    # ==========================================
    # MISSION STATE MACHINE
    # ==========================================
    def log_state_change(self, new_state_msg):
        """Log state changes with descriptive messages."""
        self.get_logger().info(f'>>> STATE: {new_state_msg}')
    
    def mission_loop(self):
        """Main mission state machine loop."""
        
        # Log state changes
        if self.state != self.prev_state:
            state_messages = {
                MissionState.IDLE: 'IDLE - Waiting',
                MissionState.SEARCHING_RIGHT: 'SEARCHING - Moving right to find ArUcos',
                MissionState.SEARCHING_LEFT: 'SEARCHING - Moving left to find ArUcos',
                MissionState.ADVANCING: 'ADVANCING - Moving forward to get closer to ArUcos',
                MissionState.CENTERING_X: 'CENTERING X - Aligning horizontally with gate',
                MissionState.CENTERING_Y: 'CENTERING Y - Aligning vertically with gate',
                MissionState.FLY_THROUGH: 'FLY THROUGH - Passing through the gate!',
                MissionState.COMPLETED: 'COMPLETED - Mission finished!'
            }
            self.log_state_change(state_messages.get(self.state, str(self.state)))
            self.prev_state = self.state
        
        if self.state == MissionState.IDLE:
            pass
            
        elif self.state == MissionState.SEARCHING_LEFT:
            self.do_search(direction='left')
            
        elif self.state == MissionState.SEARCHING_RIGHT:
            self.do_search(direction='right')
            
        elif self.state == MissionState.ADVANCING:
            self.do_advancing()
            
        elif self.state == MissionState.CENTERING_X:
            self.do_centering_x()
            
        elif self.state == MissionState.CENTERING_Y:
            self.do_centering_y()
            
        elif self.state == MissionState.FLY_THROUGH:
            self.do_fly_through()
            
        elif self.state == MissionState.COMPLETED:
            self.get_logger().info('=== Mission completed successfully! ===')
            self.get_logger().info('Drone has passed through the arch.')
            self.state = MissionState.IDLE

    def do_search(self, direction='right'):
        """Search for ArUco markers by moving laterally."""
        
        # Check if we found at least 1 marker - can start centering with 1+
        if len(self.detected_markers) >= 1:
            marker_ids = list(self.detected_markers.keys())
            self.get_logger().info(f'>>> GATE FOUND! {len(self.detected_markers)} ArUco marker(s) detected (IDs: {marker_ids})')
            self.get_logger().info('>>> Starting gate alignment...')
            self.stop_movement()
            time.sleep(0.5)
            self.state = MissionState.CENTERING_X
            self.move_start_time = None
            return
        
        # Initialize movement
        if self.move_start_time is None:
            self.move_start_time = time.time()
            self.distance_traveled = 0.0
        
        # Calculate distance traveled
        elapsed = time.time() - self.move_start_time
        self.distance_traveled = elapsed * self.search_velocity
        
        # Log progress periodically
        self.log_counter += 1
        if self.log_counter >= self.LOG_INTERVAL:
            self.log_counter = 0
            self.get_logger().info(f'Searching {direction}: {self.distance_traveled:.1f}m / {self.search_distance}m')
        
        # Set velocity based on direction
        # In drone frame (NED): Y+ is right, Y- is left
        if direction == 'right':
            self.set_velocity(0.0, self.search_velocity, 0.0)
        else:
            self.set_velocity(0.0, -self.search_velocity, 0.0)
        
        # Check if we've traveled the search distance
        if self.distance_traveled >= self.search_distance:
            self.stop_movement()
            self.search_count += 1
            
            # After 2 complete searches (right + left), advance forward
            if self.search_count >= 2:
                self.get_logger().info(f'>>> Searched twice ({self.search_distance}m each direction) without finding gate.')
                self.get_logger().info(f'>>> Advancing {self.advance_distance}m forward to get closer...')
                time.sleep(0.5)
                self.state = MissionState.ADVANCING
                self.move_start_time = None
                self.search_count = 0  # Reset counter
            else:
                self.get_logger().info(f'>>> Searched {self.search_distance}m without finding gate, reversing direction... (search {self.search_count}/2)')
                time.sleep(0.5)
                
                # Switch direction
                if direction == 'right':
                    self.state = MissionState.SEARCHING_LEFT
                else:
                    self.state = MissionState.SEARCHING_RIGHT
                
                self.move_start_time = None

    def do_advancing(self):
        """Advance forward to get closer to markers."""
        
        # Check if we found at least 1 marker during advance
        if len(self.detected_markers) >= 1:
            marker_ids = list(self.detected_markers.keys())
            self.get_logger().info(f'>>> GATE FOUND during advance! {len(self.detected_markers)} ArUco marker(s) detected (IDs: {marker_ids})')
            self.get_logger().info('>>> Starting gate alignment...')
            self.stop_movement()
            time.sleep(0.5)
            self.state = MissionState.CENTERING_X
            self.move_start_time = None
            return
        
        # Initialize movement
        if self.move_start_time is None:
            self.move_start_time = time.time()
            self.distance_traveled = 0.0
        
        # Calculate distance traveled
        elapsed = time.time() - self.move_start_time
        self.distance_traveled = elapsed * self.centering_velocity
        
        # Log progress periodically
        self.log_counter += 1
        if self.log_counter >= self.LOG_INTERVAL:
            self.log_counter = 0
            self.get_logger().info(f'Advancing: {self.distance_traveled:.2f}m / {self.advance_distance}m')
        
        # Move forward (X+ in drone frame)
        self.set_velocity(self.centering_velocity, 0.0, 0.0)
        
        # Check if we've traveled the advance distance
        if self.distance_traveled >= self.advance_distance:
            self.stop_movement()
            self.get_logger().info(f'>>> Advanced {self.advance_distance}m forward. Resuming search...')
            time.sleep(0.5)
            
            # Resume searching from the right
            self.state = MissionState.SEARCHING_RIGHT
            self.move_start_time = None

    def do_centering_x(self):
        """Center the drone horizontally (X axis in image = Y axis in drone frame)."""
        
        error_x = self.calculate_x_error()
        
        if error_x is None:
            # Lost markers, go back to searching
            self.get_logger().warn('>>> Lost ArUco markers during X centering!')
            self.state = MissionState.SEARCHING_RIGHT
            self.move_start_time = None
            return
        
        # Log progress periodically
        self.log_counter += 1
        if self.log_counter >= self.LOG_INTERVAL:
            self.log_counter = 0
            direction = "right" if error_x > 0 else "left"
            self.get_logger().info(f'Centering X: error={error_x:.1f}px, moving {direction}')
        
        # Check if centered
        if abs(error_x) <= self.centering_threshold_x:
            self.stop_movement()
            self.get_logger().info('>>> Centered in X! Moving to Y centering...')
            time.sleep(0.5)
            self.state = MissionState.CENTERING_Y
            return
        
        # Calculate centering velocity (proportional control)
        # error_x > 0 means arch is to the right, move right (Y+)
        # error_x < 0 means arch is to the left, move left (Y-)
        # Use proportional gain with saturation
        kp = 0.005  # Proportional gain
        vy = np.clip(kp * error_x, -self.centering_velocity, self.centering_velocity)
        
        self.set_velocity(0.0, vy, 0.0)

    def do_centering_y(self):
        """Center the drone vertically (Y axis in image = Z axis in drone frame)."""
        
        error_y = self.calculate_y_error()
        
        if error_y is None:
            # Lost markers, go back to X centering
            self.get_logger().warn('>>> Lost ArUco markers during Y centering!')
            self.state = MissionState.CENTERING_X
            return
        
        # Log progress periodically
        self.log_counter += 1
        if self.log_counter >= self.LOG_INTERVAL:
            self.log_counter = 0
            direction = "down" if error_y > 0 else "up"
            self.get_logger().info(f'Centering Y: error={error_y:.1f}px, moving {direction}')
        
        # Check if centered
        if abs(error_y) <= self.centering_threshold_y:
            self.stop_movement()
            self.get_logger().info('>>> Centered in Y! Gate aligned - preparing to fly through...')
            time.sleep(0.5)
            self.state = MissionState.FLY_THROUGH
            self.move_start_time = None
            return
        
        # Also check X centering and correct if needed
        error_x = self.calculate_x_error()
        vy = 0.0
        if error_x and abs(error_x) > self.centering_threshold_x:
            kp_x = 0.003
            vy = np.clip(kp_x * error_x, -self.centering_velocity, self.centering_velocity)
        
        # Calculate centering velocity (proportional control)
        # error_y > 0 means arch is below center, move down (Z-)
        # error_y < 0 means arch is above center, move up (Z+)
        kp = 0.005  # Proportional gain
        vz = -np.clip(kp * error_y, -self.centering_velocity, self.centering_velocity)
        
        self.set_velocity(0.0, vy, vz)

    def do_fly_through(self):
        """Fly straight through the arch while maintaining centering."""
        
        # Initialize movement
        if self.move_start_time is None:
            self.move_start_time = time.time()
            self.get_logger().info(f'>>> PASSING THROUGH THE GATE! Distance: {self.fly_through_distance}m')
        
        # Calculate distance traveled
        elapsed = time.time() - self.move_start_time
        distance = elapsed * self.fly_velocity
        
        # Check if we've flown through
        if distance >= self.fly_through_distance:
            self.stop_movement()
            self.get_logger().info('>>> GATE PASSED SUCCESSFULLY!')
            self.state = MissionState.COMPLETED
            return
        
        # Try to maintain centering while flying forward
        vx = self.fly_velocity
        vy = 0.0
        vz = 0.0
        
        # Correct drift if markers are still visible
        error_x = self.calculate_x_error()
        error_y = self.calculate_y_error()
        
        if error_x and abs(error_x) > self.centering_threshold_x:
            kp = 0.002
            vy = np.clip(kp * error_x, -self.centering_velocity/2, self.centering_velocity/2)
        
        if error_y and abs(error_y) > self.centering_threshold_y:
            kp = 0.002
            vz = -np.clip(kp * error_y, -self.centering_velocity/2, self.centering_velocity/2)
        
        self.set_velocity(vx, vy, vz)
        
        # Log progress periodically
        self.log_counter += 1
        if self.log_counter >= self.LOG_INTERVAL:
            self.log_counter = 0
            progress_pct = (distance / self.fly_through_distance) * 100
            self.get_logger().info(f'Flying through gate: {distance:.1f}m / {self.fly_through_distance}m ({progress_pct:.0f}%)')


def main(args=None):
    rclpy.init(args=args)
    
    node = CentralizedMissionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user')
        node.stop_movement()
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
