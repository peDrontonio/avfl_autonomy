#!/usr/bin/env python3
"""
Centralized Mission Node
Features:
- "Flower" Search Pattern: Center -> Right -> Center -> Left -> Center -> Advance
- 4-Marker Lock Requirement
- HUD State Publishing
- Navigate Service for precise movements
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import String, Int32
from ardupilot_msgs.srv import ModeSwitch
from drone_navigate.srv import Navigate
import numpy as np
import math
import time
from enum import Enum

class MissionState(Enum):
    IDLE = 0
    # --- The New Pattern ---
    SEARCHING_RIGHT = 1       # 0 -> +6m
    RETURNING_FROM_RIGHT = 2  # +6m -> 0
    SEARCHING_LEFT = 3        # 0 -> -6m
    RETURNING_FROM_LEFT = 4   # -6m -> 0
    ADVANCING = 5             # Move Forward
    # -----------------------
    CENTERING_X = 6
    CENTERING_Y = 7
    FLY_THROUGH = 8
    LANDING = 9
    COMPLETED = 10
    # --- New States for Partial Detection ---
    PARTIAL_CENTERING = 11    # Center on partial markers to find the rest
    ADJUSTING_FOR_MARKERS = 12  # Adjust Y or Z based on layout

class CentralizedMissionNode(Node):
    def __init__(self):
        super().__init__('centralized_mission_node')
        
        # Callback group for service calls
        self.callback_group = ReentrantCallbackGroup()
        
        # ==========================================
        # PARAMETERS
        # ==========================================
        self.declare_parameter('search_velocity', 1.0) 
        self.declare_parameter('centering_velocity', 0.15)  # Slower for stability
        self.declare_parameter('fly_velocity', 1.0)
        
        # Distance from Center to the Side (e.g., 6 meters)
        self.declare_parameter('search_distance', 6.0) 
        self.declare_parameter('advance_distance', 0.5) 
        
        self.declare_parameter('centering_threshold_x', 0.1) 
        self.declare_parameter('centering_threshold_y', 0.1)
        self.declare_parameter('partial_centering_threshold', 0.8)  # Very tolerant for partial (less trust in pixels)
        self.declare_parameter('default_fly_distance', 5.0)
        
        # New parameter: start_direction ('left' or 'right')
        self.declare_parameter('start_direction', 'right')
        
        self.search_velocity = self.get_parameter('search_velocity').value
        self.centering_velocity = self.get_parameter('centering_velocity').value
        self.fly_velocity = self.get_parameter('fly_velocity').value
        self.search_distance = self.get_parameter('search_distance').value
        self.advance_distance = self.get_parameter('advance_distance').value
        self.centering_threshold_x = self.get_parameter('centering_threshold_x').value
        self.centering_threshold_y = self.get_parameter('centering_threshold_y').value
        self.partial_centering_threshold = self.get_parameter('partial_centering_threshold').value
        self.default_fly_distance = self.get_parameter('default_fly_distance').value
        self.start_direction = self.get_parameter('start_direction').value
        
        # ==========================================
        # STATE VARIABLES
        # ==========================================
        self.state = MissionState.IDLE
        self.prev_state = MissionState.IDLE
        self.move_start_time = None
        self.calculated_fly_distance = 0.0 
        
        self.latest_error = Vector3()
        self.last_detection_time = 0.0
        self.DETECTION_TIMEOUT = 0.5 
        
        # Partial detection variables
        self.latest_partial_error = Vector3()
        self.last_partial_detection_time = 0.0
        self.marker_count = 0
        self.last_marker_count_time = 0.0
        self.marker_layout = ""  # 'horizontal' or 'vertical'
        self.last_layout_time = 0.0
        self.adjustment_direction = None  # Track which way we're adjusting
        self.adjustment_start_time = None
        self.MAX_ADJUSTMENT_TIME = 8.0  # More time for Z adjustment to work
        
        # Partial centering retry counter - after 3 failures, try Z adjustment
        self.partial_centering_attempts = 0
        self.MAX_PARTIAL_CENTERING_ATTEMPTS = 3
        self.z_adjustment_tried = False  # Track if we already tried Z adjustment
        
        # Navigation state
        self.navigation_in_progress = False
        self.navigation_future = None
        
        # ==========================================
        # COMMUNICATIONS
        # ==========================================
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', qos_profile)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        
        self.mode_switch_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        
        # Navigate service client
        self.navigate_client = self.create_client(
            Navigate, 
            '/avfl/navigate',
            callback_group=self.callback_group
        )
        
        self.error_sub = self.create_subscription(Vector3, '/aruco_error', self.error_callback, 10)
        self.partial_error_sub = self.create_subscription(Vector3, '/aruco_partial_error', self.partial_error_callback, 10)
        self.marker_count_sub = self.create_subscription(Int32, '/aruco_marker_count', self.marker_count_callback, 10)
        self.marker_layout_sub = self.create_subscription(String, '/aruco_marker_layout', self.marker_layout_callback, 10)
        
        self.current_vel_cmd = TwistStamped()
        
        self.create_timer(0.1, self.mission_loop)
        self.create_timer(0.05, self.velocity_command_loop)
        
        self.create_timer(2.0, self.start_mission)
        self.mission_started = False
        
        # Wait for navigate service
        self.get_logger().info('Waiting for navigate service...')
        if self.navigate_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Navigate service available!')
        else:
            self.get_logger().warn('Navigate service not available, will use velocity control')
        
        self.get_logger().info(f'=== Pattern Mission Ready (start: {self.start_direction}, dist: {self.search_distance}m) ===')

    def error_callback(self, msg):
        self.latest_error = msg
        self.last_detection_time = time.time()

    def partial_error_callback(self, msg):
        self.latest_partial_error = msg
        self.last_partial_detection_time = time.time()

    def marker_count_callback(self, msg):
        self.marker_count = msg.data
        self.last_marker_count_time = time.time()

    def marker_layout_callback(self, msg):
        self.marker_layout = msg.data
        self.last_layout_time = time.time()

    def is_target_visible(self):
        if self.last_detection_time == 0.0: return False
        return (time.time() - self.last_detection_time) < self.DETECTION_TIMEOUT

    def is_partial_target_visible(self):
        """Returns True if we have 2-3 markers visible (partial detection)"""
        if self.last_partial_detection_time == 0.0: return False
        return (time.time() - self.last_partial_detection_time) < self.DETECTION_TIMEOUT

    def get_marker_count(self):
        """Returns current marker count if recent, else 0"""
        if (time.time() - self.last_marker_count_time) < self.DETECTION_TIMEOUT:
            return self.marker_count
        return 0

    def get_marker_layout(self):
        """Returns marker layout ('horizontal' or 'vertical') if recent"""
        if (time.time() - self.last_layout_time) < self.DETECTION_TIMEOUT:
            return self.marker_layout
        return ""

    def start_mission(self):
        if not self.mission_started:
            self.mission_started = True
            # Use start_direction parameter to determine initial search direction
            if self.start_direction.lower() == 'left':
                self.state = MissionState.SEARCHING_LEFT
                self.get_logger().info('Mission Started: Searching Left...')
            else:
                self.state = MissionState.SEARCHING_RIGHT
                self.get_logger().info('Mission Started: Searching Right...')
            self.move_start_time = time.time()

    def velocity_command_loop(self):
        if self.state not in [MissionState.IDLE, MissionState.COMPLETED, MissionState.LANDING]:
            self.current_vel_cmd.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(self.current_vel_cmd)

    def set_velocity(self, vx, vy, vz):
        self.current_vel_cmd = TwistStamped()
        self.current_vel_cmd.header.frame_id = 'base_link'
        self.current_vel_cmd.twist.linear.x = float(vx)
        self.current_vel_cmd.twist.linear.y = float(vy)
        self.current_vel_cmd.twist.linear.z = float(vz)

    def stop_movement(self):
        self.set_velocity(0.0, 0.0, 0.0)
        for _ in range(3):
            self.current_vel_cmd.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(self.current_vel_cmd)
            time.sleep(0.02)

    # ==========================================
    # NAVIGATE SERVICE HELPERS
    # ==========================================
    def navigate_to(self, x, y, z, speed=None, frame='body', blocking=False):
        """
        Navigate to a position using the navigate service.
        
        Args:
            x, y, z: Target position
            speed: Navigation speed (uses search_velocity if None)
            frame: 'body' or 'map'
            blocking: If True, wait for navigation to complete
        
        Returns:
            True if navigation started/completed successfully
        """
        if not self.navigate_client.service_is_ready():
            self.get_logger().warn('Navigate service not available')
            return False
        
        req = Navigate.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float('nan')  # Maintain current heading
        req.yaw_rate = float('nan')
        req.speed = float(speed if speed is not None else self.search_velocity)
        req.frame_id = frame
        req.auto_arm = False
        
        self.get_logger().info(f'Navigate: ({x:.2f}, {y:.2f}, {z:.2f}) @ {req.speed:.1f}m/s [{frame}]')
        
        if blocking:
            try:
                future = self.navigate_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                if future.result() is not None:
                    result = future.result()
                    return result.success
                return False
            except Exception as e:
                self.get_logger().error(f'Navigation error: {e}')
                return False
        else:
            self.navigation_future = self.navigate_client.call_async(req)
            self.navigation_in_progress = True
            return True

    def navigate_until_condition(self, x, y, z, speed=None, frame='body', 
                                  condition_func=None, timeout=30.0, 
                                  check_interval=0.1, max_retries=3):
        """
        Navigate to a position but interrupt and return if a condition becomes true.
        Similar to navigateInterrupted from Clover package.
        
        Args:
            x, y, z: Target position (relative in body frame)
            speed: Navigation speed
            frame: 'body' or 'map'
            condition_func: Callable that returns True when we should stop navigation
            timeout: Maximum time to wait for condition
            check_interval: How often to check the condition (seconds)
            max_retries: Maximum navigation retries if target overshot
        
        Returns:
            'condition_met' - if condition_func returned True
            'reached' - if reached target position
            'timeout' - if timed out
            'error' - if navigation failed
        """
        if not self.navigate_client.service_is_ready():
            self.get_logger().warn('Navigate service not available')
            return 'error'
        
        if condition_func is None:
            condition_func = lambda: False
        
        speed = speed if speed is not None else self.search_velocity
        start_time = time.time()
        retry_count = 0
        current_y = y
        current_speed = speed
        
        while retry_count < max_retries:
            # Start navigation
            req = Navigate.Request()
            req.x = float(x)
            req.y = float(current_y)
            req.z = float(z)
            req.yaw = float('nan')
            req.yaw_rate = float('nan')
            req.speed = float(current_speed)
            req.frame_id = frame
            req.auto_arm = False
            
            self.get_logger().info(f'NavigateUntil: ({x:.2f}, {current_y:.2f}, {z:.2f}) @ {current_speed:.1f}m/s [retry {retry_count}]')
            
            try:
                future = self.navigate_client.call_async(req)
                
                # Wait for service call to complete (just the call, not the movement)
                while not future.done():
                    if time.time() - start_time > timeout:
                        self.get_logger().warn('Navigate call timeout')
                        return 'timeout'
                    time.sleep(0.05)
                
                result = future.result()
                if result is None or not result.success:
                    self.get_logger().error(f'Navigation failed: {result.message if result else "No response"}')
                    return 'error'
                
                # Navigation started, now monitor condition while drone moves
                self.get_logger().info('Navigation started, monitoring condition...')
                
                nav_start_time = time.time()
                last_error_x = None
                
                while True:
                    # Check timeout
                    if time.time() - start_time > timeout:
                        self.get_logger().warn('Total timeout reached')
                        self.stop_movement()
                        return 'timeout'
                    
                    # Check condition
                    if condition_func():
                        self.get_logger().info('Condition met! Stopping navigation.')
                        self.stop_movement()
                        return 'condition_met'
                    
                    # Get current error for overshot detection
                    current_error_x = self.latest_partial_error.x if self.is_partial_target_visible() else None
                    
                    # Check if we overshot (error changed sign significantly)
                    if current_error_x is not None and last_error_x is not None:
                        # If error changed sign and we're moving in Y
                        if current_y != 0 and (current_error_x * last_error_x < 0):
                            self.get_logger().info(f'Overshot detected! Error went from {last_error_x:.2f} to {current_error_x:.2f}')
                            self.stop_movement()
                            # Reverse direction and reduce speed
                            current_y = -current_y * 0.5
                            current_speed *= 0.8
                            retry_count += 1
                            time.sleep(0.2)  # Small pause before retry
                            break
                    
                    last_error_x = current_error_x
                    
                    # Check if navigation completed (rough time estimate)
                    estimated_travel_time = abs(current_y) / current_speed if current_speed > 0 else 0
                    if time.time() - nav_start_time > estimated_travel_time + 1.0:
                        self.get_logger().info('Navigation likely complete (time-based)')
                        return 'reached'
                    
                    time.sleep(check_interval)
                    
            except Exception as e:
                self.get_logger().error(f'Navigation exception: {e}')
                return 'error'
        
        self.get_logger().warn(f'Max retries ({max_retries}) reached')
        return 'timeout'

    def is_navigation_complete(self):
        """Check if ongoing navigation is complete"""
        if not self.navigation_in_progress or self.navigation_future is None:
            return True
        
        if self.navigation_future.done():
            self.navigation_in_progress = False
            try:
                result = self.navigation_future.result()
                return result.success if result else False
            except Exception:
                return False
        return False

    def cancel_navigation(self):
        """Cancel ongoing navigation by stopping movement"""
        self.navigation_in_progress = False
        self.navigation_future = None
        self.stop_movement()

    # ==========================================
    # MISSION LOOP
    # ==========================================
    def mission_loop(self):
        # Update HUD
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

        if self.state != self.prev_state:
            self.get_logger().info(f'>>> TRANSITION: {self.state.name}')
            self.prev_state = self.state
        
        # --- THE PATTERN LOGIC ---
        if self.state == MissionState.SEARCHING_RIGHT:
            # Go 0 -> +6m (Right)
            self.do_lateral_move(direction='right', distance=self.search_distance, next_state=MissionState.RETURNING_FROM_RIGHT)
            
        elif self.state == MissionState.RETURNING_FROM_RIGHT:
            # Go +6m -> 0 (Left)
            self.do_lateral_move(direction='left', distance=self.search_distance, next_state=MissionState.SEARCHING_LEFT)
            
        elif self.state == MissionState.SEARCHING_LEFT:
            # Go 0 -> -6m (Left)
            self.do_lateral_move(direction='left', distance=self.search_distance, next_state=MissionState.RETURNING_FROM_LEFT)
            
        elif self.state == MissionState.RETURNING_FROM_LEFT:
            # Go -6m -> 0 (Right)
            self.do_lateral_move(direction='right', distance=self.search_distance, next_state=MissionState.ADVANCING)
            
        elif self.state == MissionState.ADVANCING:
            self.do_advancing(next_state=self._get_initial_search_state())
            
        # --- CENTERING & FLYING ---
        elif self.state == MissionState.CENTERING_X:
            self.do_centering_x()
        elif self.state == MissionState.CENTERING_Y:
            self.do_centering_y()
        elif self.state == MissionState.FLY_THROUGH:
            self.do_fly_through()
        elif self.state == MissionState.LANDING:
            self.do_landing()
        # --- PARTIAL DETECTION STATES ---
        elif self.state == MissionState.PARTIAL_CENTERING:
            self.do_partial_centering()
        elif self.state == MissionState.ADJUSTING_FOR_MARKERS:
            self.do_adjusting_for_markers()

    def _get_initial_search_state(self):
        """Return the initial search state based on start_direction parameter"""
        if self.start_direction.lower() == 'left':
            return MissionState.SEARCHING_LEFT
        return MissionState.SEARCHING_RIGHT

    def do_lateral_move(self, direction, distance, next_state):
        # Full detection (4 markers) - go to centering
        if self.is_target_visible():
            self.get_logger().info(f'Target Found while {self.state.name}! Centering...')
            self.stop_movement()
            self.state = MissionState.CENTERING_X
            self.move_start_time = None
            return
        
        # Partial detection (2-3 markers) - try to center on them to find the rest
        if self.is_partial_target_visible():
            marker_count = self.get_marker_count()
            layout = self.get_marker_layout()
            self.get_logger().info(f'Partial Detection ({marker_count} markers, layout: {layout}) while {self.state.name}! Adjusting...')
            self.stop_movement()
            self.state = MissionState.PARTIAL_CENTERING
            self.move_start_time = None
            return
        
        if self.move_start_time is None:
            self.move_start_time = time.time()
        
        elapsed = time.time() - self.move_start_time
        dist_traveled = elapsed * self.search_velocity
        
        # Log progress
        if int(elapsed * 10) % 20 == 0: 
             self.get_logger().info(f'{self.state.name}: {dist_traveled:.1f}/{distance:.1f}m')

        vy = self.search_velocity if direction == 'right' else -self.search_velocity
        self.set_velocity(0.0, vy, 0.0)
        
        if dist_traveled >= distance:
            self.stop_movement()
            self.state = next_state
            self.move_start_time = None

    def do_advancing(self, next_state):
        if self.is_target_visible():
            self.stop_movement()
            self.state = MissionState.CENTERING_X
            return
        
        # Also check for partial detection
        if self.is_partial_target_visible():
            self.stop_movement()
            self.state = MissionState.PARTIAL_CENTERING
            return
            
        if self.move_start_time is None:
            self.move_start_time = time.time()
            self.get_logger().info('Advancing Forward...')
            
        elapsed = time.time() - self.move_start_time
        dist = elapsed * self.search_velocity 
        
        self.set_velocity(self.search_velocity, 0.0, 0.0)
        
        if dist >= self.advance_distance:
            self.stop_movement()
            self.state = next_state
            self.move_start_time = None

    def do_centering_x(self):
        if not self.is_target_visible():
            self.get_logger().warn('Lost Lock. Returning to Search Pattern...')
            # If lost, we default back to initial search direction
            self.state = self._get_initial_search_state()
            return
        
        error_x = self.latest_error.x 
        if abs(error_x) <= self.centering_threshold_x:
            self.stop_movement()
            self.state = MissionState.CENTERING_Y
            return
        
        vy = np.clip(0.8 * error_x, -self.centering_velocity, self.centering_velocity)
        self.set_velocity(0.0, vy, 0.0)

    def do_centering_y(self):
        if not self.is_target_visible():
            self.state = MissionState.CENTERING_X
            return
            
        error_y = self.latest_error.y
        if abs(error_y) <= self.centering_threshold_y:
            self.stop_movement()
            
            measured = self.latest_error.z
            if measured > 0.1:
                self.calculated_fly_distance = measured * 1.5
            else:
                self.calculated_fly_distance = self.default_fly_distance
                
            self.get_logger().info(f'Locked. Fly Dist: {self.calculated_fly_distance:.2f}m')
            self.state = MissionState.FLY_THROUGH
            self.move_start_time = None
            return

        vy = np.clip(0.5 * self.latest_error.x, -0.1, 0.1)
        vz = -np.clip(0.8 * error_y, -self.centering_velocity, self.centering_velocity)
        self.set_velocity(0.0, vy, vz)

    def do_fly_through(self):
        if self.move_start_time is None:
            self.move_start_time = time.time()
            
        elapsed = time.time() - self.move_start_time
        dist = elapsed * self.fly_velocity
        
        if dist >= self.calculated_fly_distance:
            self.stop_movement()
            self.state = MissionState.LANDING
            return
            
        self.set_velocity(self.fly_velocity, 0.0, 0.0)

    def do_landing(self):
        self.stop_movement()
        req = ModeSwitch.Request()
        req.mode = 9 
        self.mode_switch_client.call_async(req)
        self.state = MissionState.COMPLETED

    # ==========================================
    # PARTIAL DETECTION HANDLERS
    # ==========================================
    def do_partial_centering(self):
        """
        Center on partial markers (2-3) in X axis using navigate_until_condition,
        then adjust Y or Z based on layout to find the remaining markers.
        Uses the interruptible navigation pattern.
        After MAX_PARTIAL_CENTERING_ATTEMPTS failures, forces Z adjustment.
        """
        # If we now see all 4 markers, go to normal centering
        if self.is_target_visible():
            self.get_logger().info('Full detection achieved! Going to normal centering.')
            self.cancel_navigation()
            self.state = MissionState.CENTERING_X
            self.adjustment_start_time = None
            self.partial_centering_attempts = 0  # Reset counter
            self.z_adjustment_tried = False
            return
        
        # If we lost the partial detection, go back to searching
        if not self.is_partial_target_visible():
            self.get_logger().warn('Lost partial detection. Returning to search...')
            self.cancel_navigation()
            self.state = self._get_initial_search_state()
            self.adjustment_start_time = None
            self.partial_centering_attempts = 0  # Reset counter
            self.z_adjustment_tried = False
            return
        
        # Get current error
        error_x = self.latest_partial_error.x
        
        # If already centered enough in X, IMMEDIATELY go to Z adjustment (UP)
        if abs(error_x) <= self.partial_centering_threshold:
            self.stop_movement()
            self.partial_centering_attempts += 1
            self.get_logger().info(f'Partial X centered (attempt {self.partial_centering_attempts}). IMMEDIATELY adjusting Z (UP)!')
            self.state = MissionState.ADJUSTING_FOR_MARKERS
            self.adjustment_start_time = time.time()
            # ALWAYS adjust UP immediately after X centering
            self.adjustment_direction = 'up'
            self.get_logger().info('Forcing immediate Z adjustment: UP')
            return
        
        # Use navigate_until_condition to center while checking for full detection
        def check_full_detection():
            return self.is_target_visible()
        
        def check_centered():
            if self.is_partial_target_visible():
                return abs(self.latest_partial_error.x) <= self.partial_centering_threshold
            return False
        
        def combined_condition():
            return check_full_detection() or check_centered()
        
        # Navigate towards the markers (error_x is the offset)
        self.get_logger().info(f'Partial centering: error_x = {error_x:.2f}m')
        
        if self.navigate_client.service_is_ready():
            result = self.navigate_until_condition(
                x=0.0, 
                y=error_x,  # Move towards the partial markers
                z=0.0,
                speed=self.centering_velocity,
                frame='body',
                condition_func=combined_condition,
                timeout=10.0,
                max_retries=3
            )
            
            if result == 'condition_met':
                if self.is_target_visible():
                    self.get_logger().info('Full detection during centering! Going to CENTERING_X.')
                    self.state = MissionState.CENTERING_X
                else:
                    self.get_logger().info('Centered on partial. Going to adjustment phase.')
                    self.state = MissionState.ADJUSTING_FOR_MARKERS
                    self.adjustment_start_time = time.time()
                    self.adjustment_direction = None
            elif result == 'reached':
                self.get_logger().info('Navigation reached target.')
                # Loop will re-evaluate
            else:
                self.get_logger().warn(f'Navigation result: {result}. Retrying...')
        else:
            # Fallback to velocity control - slow and gentle
            vy = np.clip(0.4 * error_x, -self.centering_velocity, self.centering_velocity)
            self.set_velocity(0.0, vy, 0.0)

    def do_adjusting_for_markers(self):
        """
        Adjust Y or Z based on marker layout to find remaining markers using navigate_until_condition.
        - If layout is 'horizontal' (same Y): markers are side by side, adjust Y to find more
        - If layout is 'vertical' (same X): markers are stacked, adjust Z to find more
        - If forced Z adjustment after multiple failures, prioritize Z movement
        """
        # If we now see all 4 markers, success!
        if self.is_target_visible():
            self.get_logger().info('Found all markers! Going to centering.')
            self.cancel_navigation()
            self.state = MissionState.CENTERING_X
            self.adjustment_start_time = None
            self.adjustment_direction = None
            self.partial_centering_attempts = 0  # Reset counter
            self.z_adjustment_tried = False
            return
        
        # Check timeout
        if self.adjustment_start_time is not None:
            elapsed = time.time() - self.adjustment_start_time
            if elapsed > self.MAX_ADJUSTMENT_TIME:
                self.get_logger().warn(f'Adjustment timeout after {elapsed:.1f}s.')
                self.cancel_navigation()
                
                # If Z adjustment was tried and failed, return to search
                if self.z_adjustment_tried:
                    self.get_logger().warn('Z adjustment also failed. Returning to search.')
                    self.state = self._get_initial_search_state()
                    self.partial_centering_attempts = 0
                    self.z_adjustment_tried = False
                else:
                    # Go back to partial centering to increment counter
                    self.get_logger().info('Going back to partial centering...')
                    self.state = MissionState.PARTIAL_CENTERING
                
                self.adjustment_start_time = None
                self.adjustment_direction = None
                return
        
        # If we lost partial detection, go back to partial centering (not search)
        if not self.is_partial_target_visible():
            self.get_logger().warn('Lost partial detection during adjustment. Going back to partial centering...')
            self.cancel_navigation()
            self.state = MissionState.PARTIAL_CENTERING
            self.adjustment_start_time = None
            self.adjustment_direction = None
            return
        
        layout = self.get_marker_layout()
        
        # Determine adjustment direction based on layout
        # If direction was forced (from z_adjustment_tried), don't override it
        if self.adjustment_direction is None:
            if layout == "horizontal":
                # Markers are at same height (horizontal row) - need to adjust Y to find markers on the side
                if self.latest_partial_error.x > 0:
                    self.adjustment_direction = 'left'
                else:
                    self.adjustment_direction = 'right'
                self.get_logger().info(f'Horizontal layout: adjusting Y ({self.adjustment_direction})')
            elif layout == "vertical":
                # Markers are stacked vertically - ALWAYS try UP first
                self.adjustment_direction = 'up'
                self.get_logger().info(f'Vertical layout: adjusting Z (ALWAYS UP first)')
            else:
                # Unknown layout - if we've tried many times, force Z adjustment UP
                if self.partial_centering_attempts >= 2:
                    # Likely altitude issue, ALWAYS try UP first
                    self.adjustment_direction = 'up'
                    self.get_logger().info(f'Unknown layout + multiple attempts: forcing Z UP')
                else:
                    # Try Y first
                    self.adjustment_direction = 'left' if self.latest_partial_error.x > 0 else 'right'
                    self.get_logger().info(f'Unknown layout: trying Y first ({self.adjustment_direction})')
        
        # Condition to stop: found all 4 markers
        def check_full_detection():
            return self.is_target_visible()
        
        # Step distance and speed - LARGER steps for Z adjustment
        if self.adjustment_direction in ['up', 'down']:
            step_distance = 0.6  # Larger steps for Z to have more effect
        else:
            step_distance = 0.3  # Smaller steps for Y
        adjustment_speed = self.centering_velocity * 0.5  # Even slower for stability
        
        # Determine movement based on direction
        if self.adjustment_direction == 'left':
            target_y, target_z = -step_distance, 0.0
        elif self.adjustment_direction == 'right':
            target_y, target_z = step_distance, 0.0
        elif self.adjustment_direction == 'up':
            target_y, target_z = 0.0, step_distance
        elif self.adjustment_direction == 'down':
            target_y, target_z = 0.0, -step_distance
        else:
            target_y, target_z = 0.0, 0.0
        
        if self.navigate_client.service_is_ready():
            result = self.navigate_until_condition(
                x=0.0,
                y=target_y,
                z=target_z,
                speed=adjustment_speed,
                frame='body',
                condition_func=check_full_detection,
                timeout=5.0,
                max_retries=1
            )
            
            if result == 'condition_met':
                self.get_logger().info('Found all markers during adjustment!')
                self.state = MissionState.CENTERING_X
                self.adjustment_start_time = None
                self.adjustment_direction = None
            elif result == 'reached':
                self.get_logger().info('Adjustment step complete, checking markers...')
                # Will loop and check again
            else:
                self.get_logger().warn(f'Adjustment result: {result}')
                # Try opposite direction on next iteration
                if self.adjustment_direction in ['left', 'right']:
                    self.adjustment_direction = 'right' if self.adjustment_direction == 'left' else 'left'
                elif self.adjustment_direction in ['up', 'down']:
                    self.adjustment_direction = 'down' if self.adjustment_direction == 'up' else 'up'
        else:
            # Fallback to velocity control - very slow for stability
            adjustment_vel = self.centering_velocity * 0.3
            
            if self.adjustment_direction == 'left':
                self.set_velocity(0.0, -adjustment_vel, 0.0)
            elif self.adjustment_direction == 'right':
                self.set_velocity(0.0, adjustment_vel, 0.0)
            elif self.adjustment_direction == 'up':
                self.set_velocity(0.0, 0.0, adjustment_vel)
            elif self.adjustment_direction == 'down':
                self.set_velocity(0.0, 0.0, -adjustment_vel)

def main(args=None):
    rclpy.init(args=args)
    node = CentralizedMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_movement()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()