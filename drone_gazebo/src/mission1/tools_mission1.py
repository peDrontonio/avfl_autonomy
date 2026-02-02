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
        self.marker_count = msg.data
    
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

    def searchForGate(self, search_distance=5.0, search_speed=0.3):
        """
        Search for the gate by moving laterally.
        
        Args:
            search_distance: Distance to search in each direction (meters)
            search_speed: Speed of search movement (m/s)
            
        Returns:
            True if gate found, False if search completed without finding
        """
        self.get_logger().info(f"Searching for gate (distance={search_distance}m)...")
        
        # Search pattern: right, then left
        directions = [
            ('right', search_distance),
            ('left', -search_distance * 2),  # Go left from current position
            ('center', search_distance),  # Return to center
        ]
        
        for direction_name, distance in directions:
            self.get_logger().info(f"Searching {direction_name}...")
            
            # Start navigation
            if not self.navigate.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Navigate service not available")
                return False
            
            req = Navigate.Request()
            req.x = 0.0
            req.y = float(distance)
            req.z = 0.0
            req.yaw = float('nan')
            req.yaw_rate = 0.0
            req.speed = search_speed
            req.frame_id = 'body'
            req.auto_arm = False
            
            future = self.navigate.call_async(req)
            
            # Wait for service call
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                time.sleep(0.1)
            
            if not future.done():
                continue
            
            # Monitor for gate detection during movement
            movement_start = time.time()
            estimated_time = abs(distance) / search_speed
            
            while (time.time() - movement_start) < (estimated_time + 2.0):
                # Check if we found the gate
                if self.check_markers_detected():
                    self.get_logger().info(f"Gate found during search ({direction_name})!")
                    # Stop navigation
                    self.navigateWait(x=0, y=0, z=0, frame_id='body', auto_arm=False)
                    return True
                
                time.sleep(0.1)
        
        self.get_logger().warn("Search completed without finding gate")
        return False

    def alignToGate(self, timeout=20.0):
        """
        Align with the gate to bring all markers into view.
        
        When only 1-3 markers are visible, move to try to see all 4.
        Uses the visible markers to infer which direction to move.
        
        Args:
            timeout: Maximum time for alignment (seconds)
            
        Returns:
            True if all markers visible, False otherwise
        """
        self.get_logger().info("Aligning to see all gate markers...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            # Check if we already see all markers
            if self.check_all_markers_visible():
                self.get_logger().info("All 4 markers now visible!")
                return True
            
            # Check if we lost all markers
            if self.check_markers_lost():
                self.get_logger().warn("Lost all markers during alignment")
                return False
            
            # Determine movement direction based on gate error
            error_x = self.gate_error.x
            
            if math.isnan(error_x):
                time.sleep(0.1)
                continue
            
            # If error_x is large, we need to move to center the partial view
            # This should bring more markers into view
            if abs(error_x) > 0.1:
                correction_y = error_x * 0.5  # Move towards the markers
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