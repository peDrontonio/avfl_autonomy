#!/usr/bin/python3
import math
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from states import Searching
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Int32, Float32, Bool
from geometry_msgs.msg import Vector3
from drone_navigate.srv import Navigate, GetTelemetry, SetYawRate
from std_srvs.srv import Trigger, SetBool
from ardupilot_msgs.srv import ModeSwitch
import colorful as cf

# ArduPilot flight modes
COPTER_MODE_LAND = 9
COPTER_MODE_GUIDED = 4

class Tools(Node):
    def __init__(self) -> None:
        super().__init__('tools_mission1_node')
        self.fsm = Searching()
        self.start_mission = False
        self.mission_running = False
        self.distance_error = 0.0
        self.max_attempts = 3

        # ArUco detection state
        self.gate_error = Vector3()  # Error from gate center (x, y, z)
        self.marker_count = 0
        self.detected_marker_ids = 0  # Bit flags for detected markers
        self.is_centered = False
        self.is_gate_visible = False
        self.all_markers_visible = False
        self.distance_to_gate = 0.0
        self.last_detection_time = 0.0
        self.detection_timeout = 0.5  # seconds
        
        # Centering parameters
        self.centering_speed = 0.15  # m/s for centering movements
        self.centering_gain_x = 0.5  # Proportional gain for X correction
        self.centering_gain_y = 0.5  # Proportional gain for Y correction
        self.centering_threshold = 0.08  # meters - considered centered if error < this

        # Optional: Try to activate camera service if available
        self.activate_camera = self.create_client(SetBool, '/start_camera')
        if self.activate_camera.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Camera service available, activating camera...')
            req = SetBool.Request()
            req.data = True
            future = self.activate_camera.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done():
                self.get_logger().info('Camera activated successfully')
            else:
                self.get_logger().warn('Camera activation timeout, continuing anyway...')
        else:
            self.get_logger().info('Camera service not available (optional), assuming camera is already running')


    def setSubscribers(self):
        # Legacy target position subscription
        self.create_subscription(Float32, '/target_position', self.arch_dx, 10)
        
        # ArUco detection subscriptions
        self.create_subscription(Vector3, '/aruco/gate_error', self.gate_error_callback, 10)
        self.create_subscription(Int32, '/aruco/marker_count', self.marker_count_callback, 10)
        self.create_subscription(Int32, '/aruco/detected_marker_ids', self.marker_ids_callback, 10)
        self.create_subscription(Bool, '/aruco/is_centered', self.is_centered_callback, 10)
        self.create_subscription(Bool, '/aruco/is_gate_visible', self.gate_visible_callback, 10)
        self.create_subscription(Bool, '/aruco/all_markers_visible', self.all_markers_callback, 10)
        self.create_subscription(Float32, '/aruco/distance_to_gate', self.distance_callback, 10)

    def setPublishers(self):
        None

    def setClients(self):
        self.navigate = self.create_client(Navigate, 'avfl/navigate')
        self.set_yaw_rate = self.create_client(SetYawRate, 'avfl/set_yaw_rate')
        self.get_telemetry = self.create_client(GetTelemetry, 'avfl/get_telemetry')
        self.mode_switch = self.create_client(ModeSwitch, '/ap/mode_switch')        
        

    def setServer(self):
        self.create_service(Trigger, '/start_mission1', self.start_mission_callback)
        
    def start_mission_callback(self, request, response):
        self.mission_running = True
        
        # Mission loop
        while self.mission_running and rclpy.ok():
            self.update()
            time.sleep(0.1)
            
        response.success = True
        response.message = "Mission 1 completed!"
        return response

    def arch_dx(self, data):
        self.distance_error = data.data

    # ==========================================
    # ARUCO DETECTION CALLBACKS
    # ==========================================
    def gate_error_callback(self, msg):
        """Callback for gate center error from ArUco detector."""
        self.gate_error = msg
        self.last_detection_time = time.time()
    
    def marker_count_callback(self, msg):
        """Callback for number of markers detected."""
        old_count = self.marker_count
        self.marker_count = msg.data
        self.last_detection_time = time.time()  # Update detection time!
        
        # Log when marker count changes
        if self.marker_count != old_count:
            self.get_logger().info(f"*** MARKER COUNT CHANGED: {old_count} -> {self.marker_count} ***")
    
    def marker_ids_callback(self, msg):
        """Callback for detected marker IDs (as bit flags)."""
        self.detected_marker_ids = msg.data
        self.last_detection_time = time.time()  # Update detection time!
    
    def is_centered_callback(self, msg):
        """Callback for centered status."""
        self.is_centered = msg.data
    
    def gate_visible_callback(self, msg):
        """Callback for gate visibility status."""
        self.is_gate_visible = msg.data
    
    def all_markers_callback(self, msg):
        """Callback for all markers visible status."""
        self.all_markers_visible = msg.data
    
    def distance_callback(self, msg):
        """Callback for distance to gate."""
        self.distance_to_gate = msg.data

    def is_detection_recent(self):
        """Check if ArUco detection data is recent."""
        return (time.time() - self.last_detection_time) < self.detection_timeout

    # ==========================================
    # GATE DETECTION HELPERS
    # ==========================================
    def check_markers_detected(self):
        """Check if any gate markers are detected (for FSM transitions)."""
        return self.is_gate_visible and self.is_detection_recent()
    
    def check_all_markers_visible(self):
        """Check if all 4 gate markers are visible."""
        return self.all_markers_visible and self.is_detection_recent()
    
    def check_markers_lost(self):
        """Check if we lost all markers."""
        return not self.is_gate_visible or not self.is_detection_recent()
    
    def check_centered(self):
        """Check if drone is centered on gate."""
        return self.is_centered and self.is_detection_recent()
    
    def get_detected_marker_ids(self):
        """Get set of detected marker IDs from bit flags."""
        ids = set()
        for i in range(4):  # Markers 0-3
            if self.detected_marker_ids & (1 << i):
                ids.add(i)
        return ids

    # ==========================================
    # CENTERING ALGORITHMS
    # ==========================================
    def centerOnGate(self, timeout=30.0):
        """
        Center the drone on the gate using ArUco marker feedback.
        
        This method continuously adjusts the drone position to center on the gate.
        Uses proportional control based on the gate error from ArUco detection.
        
        Args:
            timeout: Maximum time to attempt centering (seconds)
            
        Returns:
            True if centered successfully, False if timeout or lost markers
        """
        self.get_logger().info("Starting gate centering...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            # Check if we still see markers
            if not self.is_detection_recent():
                self.get_logger().warn("Lost ArUco detection during centering")
                return False
            
            if self.marker_count < 2:
                self.get_logger().warn(f"Only {self.marker_count} markers visible, need at least 2")
                time.sleep(0.2)
                continue
            
            # Check if already centered
            if self.is_centered:
                self.get_logger().info("Gate centering complete!")
                return True
            
            # Calculate correction based on gate error
            # gate_error.x > 0 means gate is to the right -> move right (positive Y in body frame)
            # gate_error.y > 0 means gate is below -> move down (negative Z in body frame)
            error_x = self.gate_error.x
            error_y = self.gate_error.y
            
            # Skip if error is NaN
            if math.isnan(error_x) or math.isnan(error_y):
                time.sleep(0.1)
                continue
            
            # Calculate corrections with proportional gain
            # Note: In body frame, Y is left/right, Z is up/down
            correction_y = error_x * self.centering_gain_x  # Lateral correction
            correction_z = -error_y * self.centering_gain_y  # Vertical correction
            
            # Limit correction magnitude
            max_correction = 0.3  # meters
            correction_y = max(-max_correction, min(max_correction, correction_y))
            correction_z = max(-max_correction, min(max_correction, correction_z))
            
            self.get_logger().info(
                f"Centering: error=({error_x:.3f}, {error_y:.3f}), "
                f"correction=({correction_y:.3f}, {correction_z:.3f})"
            )
            
            # Execute small correction movement
            if abs(correction_y) > 0.02 or abs(correction_z) > 0.02:
                self.navigateWait(
                    x=0, 
                    y=correction_y, 
                    z=correction_z,
                    speed=self.centering_speed, 
                    frame_id='body', 
                    tolerance=0.05,
                    auto_arm=False
                )
            
            time.sleep(0.2)  # Small delay between corrections
        
        self.get_logger().warn("Centering timeout")
        return False

    def partialCenterOnGate(self, timeout=15.0, threshold=0.15):
        """
        Center the drone on partial markers (2-3 visible) in X axis only.
        
        This is less accurate than full centering but helps position the drone
        to see the remaining markers.
        
        Args:
            timeout: Maximum time to attempt centering (seconds)
            threshold: Error threshold to consider centered (meters)
            
        Returns:
            True if centered on partial markers, False if timeout or lost markers
        """
        self.get_logger().info("Starting partial centering on visible markers...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            # Spin to get latest data
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # Check if we now see all 4 markers
            if self.marker_count >= 4:
                self.get_logger().info("All 4 markers visible during partial centering!")
                return True
            
            # Check if we lost markers (< 2)
            if self.marker_count < 2:
                self.get_logger().warn("Lost markers during partial centering")
                return False
            
            # Get error (X axis only for partial centering)
            error_x = self.gate_error.x
            
            # Skip if error is NaN
            if math.isnan(error_x):
                time.sleep(0.1)
                continue
            
            # Check if centered enough in X
            if abs(error_x) <= threshold:
                self.get_logger().info(f"Partial centering complete! Error X: {error_x:.3f}")
                # Stop movement
                self.navigateWait(x=0, y=0, z=0, frame_id='body', auto_arm=False, tolerance=0.05)
                return True
            
            # Calculate correction (X axis only)
            correction_y = error_x * self.centering_gain_x
            
            # Limit correction magnitude
            max_correction = 0.3
            correction_y = max(-max_correction, min(max_correction, correction_y))
            
            self.get_logger().info(f"Partial centering: error_x={error_x:.3f}, correction_y={correction_y:.3f}")
            
            # Execute correction
            if abs(correction_y) > 0.02:
                self.navigateWait(
                    x=0, 
                    y=correction_y, 
                    z=0,
                    speed=self.centering_speed, 
                    frame_id='body', 
                    tolerance=0.05,
                    auto_arm=False
                )
            
            time.sleep(0.2)
        
        self.get_logger().warn("Partial centering timeout")
        return False

    def searchForGate(self, search_distance=5.0, search_speed=0.3):
        """
        Search for the gate by moving laterally.
        Uses navigateInterruptedByMarkers to stop immediately when 2+ markers are found.
        
        Args:
            search_distance: Distance to search in each direction (meters)
            search_speed: Speed of search movement (m/s)
            
        Returns:
            True if gate found, False if search completed without finding
        """
        self.get_logger().info(f"Searching for gate (distance={search_distance}m)...")
        
        # First check if we already see at least 2 markers before moving
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Initial marker count: {self.marker_count}")
        if self.marker_count >= 2:
            self.get_logger().info(f"At least 2 markers already visible ({self.marker_count})! Stopping search.")
            self._stop_drone()
            return True
        
        # Search pattern: right, then left, then back to center
        directions = [
            ('right', search_distance),
            ('left', -search_distance * 2),  # Go left from current position
            ('center', search_distance),  # Return to center
        ]
        
        for direction_name, distance in directions:
            self.get_logger().info(f"Searching {direction_name}...")
            
            # Check before starting movement
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.marker_count >= 2:
                self.get_logger().info(f"At least 2 markers detected ({self.marker_count})! Stopping search.")
                self._stop_drone()
                return True
            
            # Use navigateInterruptedByMarkers - it will stop when 2+ markers detected
            result = self.navigateInterruptedByMarkers(
                x=0.0, 
                y=distance, 
                z=0.0,
                speed=search_speed,
                frame_id='body',
                min_markers=2
            )
            
            if result == 'markers_found':
                self.get_logger().info(f"===== GATE FOUND during search ({direction_name})! Markers: {self.marker_count} =====")
                return True
            elif result == 'reached':
                self.get_logger().info(f"Reached end of {direction_name} search, continuing...")
            else:
                self.get_logger().warn(f"Navigation issue during {direction_name}: {result}")
        
        self.get_logger().warn("Search completed without finding gate")
        return False

    def navigateInterruptedByMarkers(self, x=0, y=0, z=0, speed=0.3, frame_id='body', min_markers=2, timeout=30.0):
        """
        Navigate but interrupt immediately when enough markers are detected.
        This uses spin_once frequently to process ArUco callbacks during navigation.
        
        Args:
            x, y, z: Target position
            speed: Navigation speed
            frame_id: Reference frame
            min_markers: Minimum number of markers to trigger interruption
            timeout: Maximum time for navigation
            
        Returns:
            'markers_found' - if min_markers detected
            'reached' - if target position reached
            'timeout' - if timed out
            'error' - if navigation failed
        """
        try:
            # Wait for navigate service
            if not self.navigate.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Navigate service not available")
                return 'error'
            
            req = Navigate.Request()
            req.x = float(x)
            req.y = float(y)
            req.z = float(z)
            req.yaw = float('nan')
            req.yaw_rate = 0.0
            req.speed = float(speed)
            req.frame_id = frame_id
            req.auto_arm = False
            
            self.get_logger().info(f"NavigateInterruptedByMarkers: y={y:.1f}m, speed={speed}")
            
            future = self.navigate.call_async(req)
            
            # Wait for service call to complete while checking markers
            start_time = time.time()
            while not future.done():
                # CRITICAL: Spin to process ArUco callbacks
                rclpy.spin_once(self, timeout_sec=0.02)
                
                # Check for markers
                if self.marker_count >= min_markers:
                    self.get_logger().info(f"*** {min_markers}+ MARKERS DETECTED during nav call! Count: {self.marker_count} ***")
                    self._stop_drone()
                    return 'markers_found'
                
                if time.time() - start_time > 5.0:
                    self.get_logger().warn("Navigate service call timeout")
                    return 'timeout'
                
                time.sleep(0.01)
            
            res = future.result()
            if not res or not res.success:
                self.get_logger().error(f"Navigation failed: {res.message if res else 'No response'}")
                return 'error'
            
            self.get_logger().info("Navigation command accepted, monitoring for markers...")
            
            # Wait for telemetry service
            if not self.get_telemetry.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn("GetTelemetry service not available, using time-based monitoring")
                # Fallback: time-based monitoring
                estimated_time = abs(y) / speed if speed > 0 else 10.0
                monitor_start = time.time()
                while (time.time() - monitor_start) < (estimated_time + 2.0):
                    rclpy.spin_once(self, timeout_sec=0.02)
                    if self.marker_count >= min_markers:
                        self.get_logger().info(f"*** MARKERS FOUND! Count: {self.marker_count} ***")
                        self._stop_drone()
                        return 'markers_found'
                    time.sleep(0.02)
                return 'reached'
            
            # Monitor navigation with interruption for markers
            nav_start = time.time()
            while (time.time() - nav_start) < timeout:
                # CRITICAL: Spin frequently to get ArUco updates
                rclpy.spin_once(self, timeout_sec=0.02)
                
                # Check for markers - this is the interrupt condition
                if self.marker_count >= min_markers:
                    self.get_logger().info(f"*** MARKERS FOUND during movement! Count: {self.marker_count} ***")
                    self._stop_drone()
                    return 'markers_found'
                
                # Check if we've reached target
                telem_req = GetTelemetry.Request()
                telem_req.frame_id = 'navigate_target'
                telem_future = self.get_telemetry.call_async(telem_req)
                
                # Wait for telemetry with frequent spinning
                telem_start = time.time()
                while not telem_future.done():
                    rclpy.spin_once(self, timeout_sec=0.02)
                    
                    # Keep checking markers while waiting for telemetry!
                    if self.marker_count >= min_markers:
                        self.get_logger().info(f"*** MARKERS FOUND while getting telemetry! Count: {self.marker_count} ***")
                        self._stop_drone()
                        return 'markers_found'
                    
                    if time.time() - telem_start > 2.0:
                        break
                    time.sleep(0.01)
                
                if telem_future.done():
                    telem = telem_future.result()
                    if telem:
                        distance = math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)
                        if distance < 0.2:  # Reached target
                            self.get_logger().info("Target position reached")
                            return 'reached'
                
                time.sleep(0.02)
            
            self.get_logger().warn("Navigation timeout")
            return 'timeout'
            
        except Exception as e:
            self.get_logger().error(f"navigateInterruptedByMarkers error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return 'error'

    def _stop_drone(self):
        """Emergency stop - send stop command multiple times to ensure it takes effect."""
        self.get_logger().info("STOPPING DRONE...")
        
        # Send stop command via navigate service
        if self.navigate.wait_for_service(timeout_sec=1.0):
            req = Navigate.Request()
            req.x = 0.0
            req.y = 0.0
            req.z = 0.0
            req.yaw = float('nan')
            req.yaw_rate = 0.0
            req.speed = 0.5
            req.frame_id = 'body'
            req.auto_arm = False
            
            future = self.navigate.call_async(req)
            
            # Wait briefly for the stop command
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.05)
            
            self.get_logger().info("Stop command sent")
        
        time.sleep(0.3)  # Brief pause to stabilize

    def alignToGate(self, timeout=20.0):
        """
        Align with the gate to bring all markers into view.
        
        Strategy based on visible marker IDs:
        Gate layout:
            2 --- 3
            |     |
            0 --- 1
        
        When 2 markers detected:
        - {0, 1} (bottom pair) -> Move UP to see top markers
        - {2, 3} (top pair) -> Move DOWN to see bottom markers
        - {1, 3} (right pair) -> Move LEFT to see left markers
        - {0, 2} (left pair) -> Move RIGHT to see right markers
        
        Args:
            timeout: Maximum time for alignment (seconds)
            
        Returns:
            True if all markers visible, False otherwise
        """
        self.get_logger().info("Aligning to see all gate markers...")
        start_time = time.time()
        alignment_distance = 0.5  # meters to move in each direction
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            # Check if we already see all markers
            if self.check_all_markers_visible():
                self.get_logger().info("All 4 markers now visible!")
                # Stop completely before transitioning
                self.navigateWait(x=0, y=0, z=0, frame_id='body', auto_arm=False, tolerance=0.05)
                return True
            
            # Check if we lost all markers
            if self.check_markers_lost():
                self.get_logger().warn("Lost all markers during alignment")
                return False
            
            # Get detected marker IDs
            detected_ids = self.get_detected_marker_ids()
            
            if self.marker_count == 2 and len(detected_ids) == 2:
                self.get_logger().info(f"2 markers detected: {detected_ids}")
                
                # First, center laterally on the 2 visible markers
                error_x = self.gate_error.x
                if not math.isnan(error_x) and abs(error_x) > 0.2:
                    correction_y = error_x * 0.4
                    correction_y = max(-0.3, min(0.3, correction_y))
                    self.get_logger().info(f"Centering on visible markers: Y={correction_y:.2f}m")
                    self.navigateWait(x=0, y=correction_y, z=0, speed=0.2, 
                                    frame_id='body', tolerance=0.1, auto_arm=False)
                    time.sleep(0.5)
                    continue
                
                # Determine movement based on which pair is visible
                if detected_ids == {0, 1}:
                    # Bottom pair -> Move UP
                    self.get_logger().info("Bottom markers {0, 1} detected -> Moving UP")
                    self.navigateWait(x=0, y=0, z=alignment_distance, speed=0.3,
                                    frame_id='body', tolerance=0.1, auto_arm=False)
                elif detected_ids == {2, 3}:
                    # Top pair -> Move DOWN
                    self.get_logger().info("Top markers {2, 3} detected -> Moving DOWN")
                    self.navigateWait(x=0, y=0, z=-alignment_distance, speed=0.3,
                                    frame_id='body', tolerance=0.1, auto_arm=False)
                elif detected_ids == {1, 3}:
                    # Right pair -> Move LEFT
                    self.get_logger().info("Right markers {1, 3} detected -> Moving LEFT")
                    self.navigateWait(x=0, y=-alignment_distance, z=0, speed=0.3,
                                    frame_id='body', tolerance=0.1, auto_arm=False)
                elif detected_ids == {0, 2}:
                    # Left pair -> Move RIGHT
                    self.get_logger().info("Left markers {0, 2} detected -> Moving RIGHT")
                    self.navigateWait(x=0, y=alignment_distance, z=0, speed=0.3,
                                    frame_id='body', tolerance=0.1, auto_arm=False)
                else:
                    # Diagonal or unknown pair, use error-based movement
                    self.get_logger().info(f"Diagonal/unknown pair {detected_ids} -> Error-based alignment")
                    error_y = self.gate_error.y
                    if not math.isnan(error_y) and abs(error_y) > 0.15:
                        correction_z = -error_y * 0.5
                        correction_z = max(-alignment_distance, min(alignment_distance, correction_z))
                        self.navigateWait(x=0, y=0, z=correction_z, speed=0.3,
                                        frame_id='body', tolerance=0.1, auto_arm=False)
                
                time.sleep(1.0)  # Give time for detection after movement
                continue
            
            # With 1 or 3 markers, adjust position based on error
            error_x = self.gate_error.x
            
            if math.isnan(error_x):
                time.sleep(0.1)
                continue
            
            # General alignment correction
            if abs(error_x) > 0.1:
                correction_y = error_x * 0.5
                correction_y = max(-0.5, min(0.5, correction_y))
                
                self.get_logger().info(f"Alignment correction: Y={correction_y:.2f}m")
                
                self.navigateWait(
                    x=0, y=correction_y, z=0,
                    speed=0.2, frame_id='body',
                    tolerance=0.1, auto_arm=False
                )
            
            time.sleep(0.3)
        
        self.get_logger().warn("Alignment timeout")
        return False

    def advanceThroughGate(self, advance_distance=3.0, advance_speed=0.3):
        """
        Advance through the gate once centered.
        
        Args:
            advance_distance: Distance to fly forward (meters)
            advance_speed: Speed of forward movement (m/s)
            
        Returns:
            True if successfully advanced, False otherwise
        """
        self.get_logger().info(f"Advancing through gate ({advance_distance}m)...")
        
        # Simple forward navigation
        result = self.navigateWait(
            x=advance_distance, y=0, z=0,
            speed=advance_speed, frame_id='body',
            tolerance=0.2, auto_arm=False
        )
        
        if result and result.success:
            self.get_logger().info("Successfully passed through gate!")
            return True
        else:
            self.get_logger().error("Failed to advance through gate")
            return False

    def landDrone(self):
        """Switch to LAND mode to land the drone."""
        self.get_logger().info("Initiating landing...")
        
        if not self.mode_switch.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Mode switch service not available")
            return False
        
        land_req = ModeSwitch.Request()
        land_req.mode = COPTER_MODE_LAND
        
        future = self.mode_switch.call_async(land_req)
        
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 10.0:
            time.sleep(0.1)
        
        if future.done():
            self.get_logger().info("Switched to LAND mode")
            return True
        
        self.get_logger().error("Failed to switch to LAND mode")
        return False
        
    def navigateWait(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=True, z_participation = 1):
        '''Navigate without interruption, wait until target is reached '''
        try:
            # Wait for navigate service with longer timeout
            self.get_logger().info("Waiting for navigate service...")
            if not self.navigate.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("Navigate service not available after 10s")
                return None
            
            req = Navigate.Request()
            req.x = float(x)
            req.y = float(y)
            req.z = float(z)
            req.yaw = float(yaw)
            req.yaw_rate = 0.0
            req.speed = float(speed)
            req.frame_id = frame_id
            req.auto_arm = auto_arm
            
            self.get_logger().info(f"Calling navigate: x={x}, y={y}, z={z}, frame={frame_id}")
            
            # Call navigate service
            future = self.navigate.call_async(req)
            
            # Wait for response with timeout
            timeout = 15.0
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout:
                    self.get_logger().error("Navigate service call timeout")
                    return None
                time.sleep(0.1)
            
            res = future.result()
            
            if not res.success:
                self.get_logger().error(f"Navigation failed: {res.message}")
                return res
            self.get_logger().info("Navigation command accepted")
            
            # Wait for telemetry service
            if not self.get_telemetry.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("GetTelemetry service not available")
                return res
            
            # Monitor navigation progress
            while True:
                telem_req = GetTelemetry.Request()
                telem_req.frame_id = 'navigate_target'
                telem_future = self.get_telemetry.call_async(telem_req)
                
                # Wait for telemetry response
                start_time = time.time()
                while not telem_future.done():
                    if time.time() - start_time > 5.0:
                        self.get_logger().warn("Telemetry timeout")
                        break
                    time.sleep(0.1)
                
                if not telem_future.done():
                    continue
                    
                telem = telem_future.result()
                
                distance = math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)
                if distance < tolerance:
                    self.get_logger().info("Target reached")
                    return res
                    
                time.sleep(0.2)  # Check every 200ms
                
        except Exception as e:
            self.get_logger().error(f"Error calling navigation service: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None
            
    def navigateInterrupted(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, direction = ''):
        '''Navigate with interruption capability based on target detection'''
        attempt = 0 
        while(attempt < self.max_attempts):
            try:
                # Wait for navigate service with longer timeout
                self.get_logger().info("Waiting for navigate service...")
                if not self.navigate.wait_for_service(timeout_sec=10.0):
                    self.get_logger().error("Navigate service not available after 10s")
                    return None
                    
                req = Navigate.Request()
                req.x = float(x)
                req.y = float(y)
                req.z = float(z)
                req.yaw = float(yaw)
                req.yaw_rate = 0.0
                req.speed = float(speed)
                req.frame_id = frame_id
                req.auto_arm = auto_arm
                
                future = self.navigate.call_async(req)
                
                # Wait for response
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > 10.0:
                        self.get_logger().error("Navigate service call timeout")
                        return None
                    time.sleep(0.1)
                    
                res = future.result()
                
                if not res.success:
                    self.get_logger().error(f"Navigation failed: {res.message}")
                    return res
                self.get_logger().info("Navigation started")
                
                # Wait for telemetry service
                if not self.get_telemetry.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error("GetTelemetry service not available")
                    return res
                
                # Monitor navigation with interruption capability
                while True:
                    telem_req = GetTelemetry.Request()
                    telem_req.frame_id = 'navigate_target'
                    telem_future = self.get_telemetry.call_async(telem_req)
                    
                    # Wait for telemetry
                    start_time = time.time()
                    while not telem_future.done():
                        if time.time() - start_time > 5.0:
                            self.get_logger().warn("Telemetry timeout")
                            break
                        time.sleep(0.1)
                    
                    if not telem_future.done():
                        continue
                        
                    telem = telem_future.result()
                    
                    distance = math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)
                    self.get_logger().info(f"Distance to target: {distance:.2f} m")
                    
                    if (-10 <= self.distance_error <= 10) or (distance < tolerance):
                        self.get_logger().info("Target reached")
                        res = self.navigateWait(y=0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                        return res
                        
                    if not math.isnan(self.distance_error) and self.distance_error * y > 0:
                        y *= -1
                        speed *= 0.8
                        auto_arm = False
                        self.navigateWait(y=0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                        self.get_logger().info("Target overshot. Trying again.")
                        attempt += 1
                        break
                        
                    time.sleep(0.2)  # Check every 200ms
                    
            except Exception as e:
                self.get_logger().error(f"Error calling navigation service: {e}")
                
        self.get_logger().info(f"All {self.max_attempts} attempts exhausted. Landing...")
        time.sleep(1)
        
        # Use mode_switch to switch to LAND mode
        if self.mode_switch.wait_for_service(timeout_sec=5.0):
            land_req = ModeSwitch.Request()
            land_req.mode = COPTER_MODE_LAND
            land_future = self.mode_switch.call_async(land_req)
            start_time = time.time()
            while not land_future.done() and (time.time() - start_time < 10.0):
                time.sleep(0.1)
            if land_future.done():
                self.get_logger().info("Switched to LAND mode")
        
        return None
        

    def navigateWaitTest(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=True, z_participation = 1):
        '''Test navigation without interruption, wait until target is reached'''
        try:
            self.get_logger().info("Navigation started")
            
            telem_req = GetTelemetry.Request()
            telem_req.frame_id = 'map'
            initial_future = self.get_telemetry.call_async(telem_req)
            rclpy.spin_until_future_complete(self, initial_future)
            initial = initial_future.result()
            
            if frame_id == "body":
                initial.x += x
                initial.y += y
                initial.z += z
            if frame_id == "map":
                initial.x = x
                initial.y = y
                initial.z = z
            while rclpy.ok():
                telem_req = GetTelemetry.Request()
                telem_req.frame_id = 'map'
                diff_future = self.get_telemetry.call_async(telem_req)
                rclpy.spin_until_future_complete(self, diff_future)
                diff = diff_future.result()
                
                diff.x, diff.y, diff.z = (diff.x - initial.x), (diff.y - initial.y), (diff.z - initial.z)
                distance = math.sqrt(diff.x ** 2 + diff.y ** 2 + diff.z ** 2)
                self.get_logger().info(f"Distance to target: {distance:.2f} m")
                # self.get_logger().info(f"X: {diff.x}\nY: {diff.y}\n Z: {diff.z}\n")
                if distance < tolerance:
                    self.get_logger().info("Target reached")
                    return (0, 'Success')
        except Exception as e:
            self.get_logger().error(f"Error calling navigation service: {e}")
    
    def navigateInterruptedTest(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, direction = ''):
        '''Test navigation with interruption capability based on target detection'''
        attempt = 0 
        while(attempt < self.max_attempts):
            try:
                self.get_logger().info("Navigation started")
                
                telem_req = GetTelemetry.Request()
                telem_req.frame_id = 'map'
                initial_future = self.get_telemetry.call_async(telem_req)
                rclpy.spin_until_future_complete(self, initial_future)
                initial = initial_future.result()
                
                if frame_id == "body":
                    initial.x += x
                    initial.y += y
                    initial.z += z
                if frame_id == "map":
                    initial.x = x
                    initial.y = y
                    initial.z = z
                while rclpy.ok():
                    telem_req = GetTelemetry.Request()
                    telem_req.frame_id = 'map'
                    diff_future = self.get_telemetry.call_async(telem_req)
                    rclpy.spin_until_future_complete(self, diff_future)
                    diff = diff_future.result()
                    
                    diff.x, diff.y, diff.z = (initial.x - diff.x), (initial.y - diff.y), (initial.z - diff.z)
                    distance = math.sqrt(diff.x ** 2 + diff.y ** 2 + diff.z ** 2)
                    self.get_logger().info(f"Distance to target: {distance:.2f} m")
                    # self.get_logger().info(f"X: {diff.x}\nY: {diff.y}\n Z: {diff.z}\n")
                    if (-10 <= self.distance_error <= 10) or (distance < tolerance):
                        self.get_logger().info("Target reached")
                        res = self.navigateWaitTest(y=0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                        return res
                    if not math.isnan(self.distance_error) and self.distance_error * y > 0:
                        y *= -1
                        speed *= 0.8
                        auto_arm = False
                        self.navigateWaitTest(y=0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                        self.get_logger().info("Target overshot. Trying again.")
                        attempt += 1
                        break
            except Exception as e:
                self.get_logger().error(f"Error calling navigation service: {e}")
        self.get_logger().info(f"All {self.max_attempts} attempts exhausted. Landing...")
        time.sleep(1)
        # land_future = self.land.call_async(Trigger.Request())
        # rclpy.spin_until_future_complete(self, land_future)
        # rclpy.shutdown()