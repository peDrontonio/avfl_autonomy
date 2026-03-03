#!/usr/bin/python3
import math
import os
import sys
import time

# add mission2 package to path when running as installed executable
script_dir = os.path.dirname(os.path.abspath(__file__))
mission2_dir = os.path.join(script_dir, 'mission2')
if os.path.exists(mission2_dir) and mission2_dir not in sys.path:
    sys.path.insert(0, mission2_dir)

import numpy as np
import rclpy
from rclpy.node import Node
from states import Search_Base, GoingToBase, Takeoff
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Int32, Bool
from drone_navigate.srv import Navigate, GetTelemetry, SetYawRate, NavigateGlobal
from std_srvs.srv import Trigger, SetBool
from ardupilot_msgs.srv import ModeSwitch, ArmMotors, Takeoff as TakeoffSrv
from drone_navigate.msg import BaseDetection
import colorful as cf
from collections import deque
from rclpy.callback_groups import ReentrantCallbackGroup

class Tools(Node):
    def __init__(self) -> None:
        super().__init__('tools_mission2_node')

        # callback groups for multithreadedexecutor
        self.service_cb_group = ReentrantCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        self.fsm = Takeoff()
        self.mission_running = False
        self.global_nav_active = False
        self.nav_active = False
        
        # safety: search attempt counters for rtl
        self.search_attempts = 0
        self.scan_attempts = 0
        self.max_search_attempts = 3  # maximum attempts before rtl
        
        # yolo detection parameters
        self.base_detected = False
        self.detection_confidence = 0.0
        self.x_center = 0
        self.y_center = 0
        self.bbox_width = 0
        self.bbox_height = 0
        self.last_base_coordinates = None
        
        # consecutive detection tracking
        self.consecutive_detections = 0
        self.required_consecutive_detections = 3
        
        # image parameters
        self.image_width = 640
        self.image_height = 480
        
        # camera focal lengths
        self.fx = 867.5579087775083
        self.fy = 868.4321850250884 

        # camera service client
        self.activate_camera = self.create_client(SetBool, '/start_camera')
        if self.activate_camera.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Camera service available, trying to activate...')
            req = SetBool.Request()
            req.data = False
            future = self.activate_camera.call_async(req)
        else:
            self.get_logger().info('Camera service not available, continuing anyway')

        # base gps coordinates (lat, lon, altitude)
        self.declare_parameter('base_lat', 0.0)
        self.declare_parameter('base_lon', 0.0)
        self.declare_parameter('takeoff_alt', 10.0)  # meters above ground
        self.declare_parameter('search_alt', 10.0)  # altitude for search pattern
        
        self.base_lat = self.get_parameter('base_lat').get_parameter_value().double_value
        self.base_lon = self.get_parameter('base_lon').get_parameter_value().double_value
        self.takeoff_alt = self.get_parameter('takeoff_alt').get_parameter_value().double_value
        self.search_alt = self.get_parameter('search_alt').get_parameter_value().double_value
        
        self.get_logger().info(f"Base coordinates - Lat: {self.base_lat}, Lon: {self.base_lon}, Alt: {self.takeoff_alt}m")
        self.get_logger().info(f"Search altitude: {self.search_alt}m")
        
    def setSubscribers(self):
        self.create_subscription(BaseDetection, '/yolo/base_detection', self.base_detection_callback, 10)
        self.create_subscription(Bool, 'avfl/global_nav_active', self.global_nav_active_callback, 10)
        self.create_subscription(Bool, 'avfl/nav_active', self.nav_active_callback, 10)
    def setPublishers(self):
        pass
        
    def setClients(self):
        self.navigate = self.create_client(Navigate, 'avfl/navigate', callback_group=self.client_cb_group)
        self.navigate_global = self.create_client(NavigateGlobal, 'avfl/navigate_global', callback_group=self.client_cb_group)
        self.get_telemetry = self.create_client(GetTelemetry, 'avfl/get_telemetry', callback_group=self.client_cb_group)
        self.land_client = self.create_client(ModeSwitch, '/ap/mode_switch', callback_group=self.client_cb_group)
        self.mode_switch_client = self.create_client(ModeSwitch, '/ap/mode_switch', callback_group=self.client_cb_group)
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors', callback_group=self.client_cb_group)
        self.takeoff_client = self.create_client(TakeoffSrv, '/ap/experimental/takeoff', callback_group=self.client_cb_group)        
        

    def setServer(self):
        self.create_service(Trigger, '/start_mission2', self.start_mission_callback, callback_group=self.service_cb_group)

    def wait_for_future(self, future, timeout_sec=10.0):
        """Wait for a future to complete without blocking the executor."""
        start = time.time()
        while not future.done() and (time.time() - start) < timeout_sec:
            time.sleep(0.05)
        return future.done()

    def start_mission_callback(self, request, response):
        self.mission_running = True
        
        # initialize drone position
        self.get_logger().info("Initializing Mission 2 - Mobile Base Landing")
        
        # mission loop
        while self.mission_running and rclpy.ok():
            self.update()
            time.sleep(0.1)
            
        response.success = True
        response.message = "Mission 2 completed!"
        return response

    def base_detection_callback(self, msg):
        # callback for yolo base detection messages
        self.base_detected = msg.detected
        if msg.detected:
            self.detection_confidence = msg.confidence
            self.x_center = msg.x_center
            self.y_center = msg.y_center
            self.bbox_width = msg.bbox_width
            self.bbox_height = msg.bbox_height
            
            # track consecutive detections
            self.consecutive_detections += 1
        else:
            self.consecutive_detections = 0

    def global_nav_active_callback(self, msg):
        # track whether global navigation is currently active
        self.global_nav_active = msg.data

    def nav_active_callback(self, msg):
        # track whether local navigation is currently active
        self.nav_active = msg.data

    def is_base_centered(self, tolerance = 30):
        # check if the detected base is centered in the camera frame
        if not self.base_detected:
            return False
            
        image_center_x = self.image_width / 2
        image_center_y = self.image_height / 2
        
        offset_x = abs(self.x_center - image_center_x)
        offset_y = abs(self.y_center - image_center_y)
        # print("DEBUG - Offsets: ")
        # print(offset_x, offset_y)
        return offset_x <= tolerance and offset_y <= tolerance 
    
    def is_base_in_x_Axis(self, tolerance = 20):
        if not self.base_detected:
            return False
            
        image_center_x = self.image_width / 2
        
        offset_x = abs(self.x_center - image_center_x)
        # print("DEBUG - Offsets: ")
        # print(offset_x, offset_y)
        return offset_x <= tolerance 

    def is_base_in_y_Axis(self, tolerance = 20):
        if not self.base_detected:
            return False
            
        image_center_y = self.image_height / 2
        
        offset_y = abs(self.y_center - image_center_y)
        # print("DEBUG - Offsets: ")
        # print(offset_x, offset_y)
        return offset_y <= tolerance 

    def return_base(self):
        # activate rtl (return to base) mode - safety measure
        self.get_logger().warn("⚠" * 30)
        self.get_logger().warn("SAFETY: Activating Return Base!")
        self.get_logger().warn("⚠" * 30)
        req = ModeSwitch.Request()
        req.mode = 6  # rtl mode
        future = self.land_client.call_async(req)
        if self.wait_for_future(future, timeout_sec=5.0):
            result = future.result()
            if result.status:
                self.get_logger().info("RTL mode activated successfully")
                return True
            else:
                self.get_logger().error("Failed to activate RTL mode")
                return False
        else:
            self.get_logger().error("RTL service call timeout")
            return False

    def land(self):
        # land the drone by switching to land mode
        req = ModeSwitch.Request()
        req.mode = 9  # land mode
        future = self.land_client.call_async(req)
        if self.wait_for_future(future, timeout_sec=5.0):
            result = future.result()
            if result.status:
                self.get_logger().info("Landing mode activated")
            else:
                self.get_logger().error("Failed to activate landing mode")
        else:
            self.get_logger().error("Landing service call timeout")
    
    def wait_for_landing(self, timeout=120.0):
        # wait until the drone has actually landed based only on altitude stability
        self.get_logger().info("Waiting for landing confirmation (altitude stability)...")
        
        start_time = time.time()
        
        # control variables
        stable_start_time = None
        reference_altitude = 0.0
        
        # calibration parameters
        landing_region = 1.0      # height (m) to start monitoring
        stability_margin = 0.10   # allowed margin (m) of variation relative to reference
        required_time = 5.0       # time (s) it must stay within the margin
        
        while rclpy.ok():
            # safety timeout for the function
            if time.time() - start_time > timeout:
                self.get_logger().warn(f"Landing timeout ({timeout}s)")
                return False
            
            # call telemetry service
            telem_req = GetTelemetry.Request()
            telem_req.frame_id = 'map'
            telem_future = self.get_telemetry.call_async(telem_req)
            
            if self.wait_for_future(telem_future, timeout_sec=1.0):
                telem = telem_future.result()
                current_z = telem.z
                
                # if altitude is greater than 1.0m, drone is still flying high
                if current_z > landing_region:
                    if stable_start_time is not None:
                        self.get_logger().info(f"Altitude increased ({current_z:.2f}m). Resetting timer.")
                        stable_start_time = None
                    
                    self.get_logger().info(f"Descending... Alt: {current_z:.2f}m", throttle_duration_sec=2.0)
                
                # entered landing region (< 1.0m)
                else:
                    if stable_start_time is None:
                        # starts counting time now and locks current altitude as reference.
                        stable_start_time = time.time()
                        reference_altitude = current_z
                        self.get_logger().info(f"Landing zone reached ({current_z:.2f}m). Starting stability verification...")
                    
                    else:
                        # compare current altitude with locked reference (not previous)
                        diff = abs(current_z - reference_altitude)
                        
                        if diff > stability_margin:
                            # if difference is large, it means drone is still descending. updating reference and resetting timer.
                            self.get_logger().info(f"Variation detected ({diff:.3f}m). Still moving...")
                            reference_altitude = current_z
                            stable_start_time = time.time()
                        
                        elif (time.time() - stable_start_time) > required_time:
                            # if 5 seconds passed and altitude never left the 0.1m margin from reference
                            self.get_logger().info(f"Landing confirmed! Altitude stable at {reference_altitude:.2f}m for {required_time}s.")
                            return True
            
            time.sleep(0.2)
        
        return False

    def takeoffWait(self, z, auto_arm=True):
        # arm motors and takeoff to specified altitude
        self.get_logger().info(f"Starting takeoff sequence to {z}m altitude...")
                # step 0: switch to guided mode
        self.get_logger().info("Switching to GUIDED mode...")
        mode_req = ModeSwitch.Request()
        mode_req.mode = 4  # guided mode
        mode_future = self.mode_switch_client.call_async(mode_req)
        
        if self.wait_for_future(mode_future, timeout_sec=5.0):
            mode_result = mode_future.result()
            if mode_result.status:
                self.get_logger().info("GUIDED mode activated")
            else:
                self.get_logger().error("Failed to switch to GUIDED mode")
                return False
        else:
            self.get_logger().error("Mode switch service call timeout")
            return False
                # step 1: arm the motors
        if auto_arm:
            self.get_logger().info("Arming motors...")
            arm_req = ArmMotors.Request()
            arm_req.arm = True
            arm_future = self.arm_client.call_async(arm_req)
            
            if self.wait_for_future(arm_future, timeout_sec=5.0):
                arm_result = arm_future.result()
                if arm_result.result:
                    self.get_logger().info("Motors armed successfully!")
                else:
                    self.get_logger().error("Failed to arm motors")
                    return False
            else:
                self.get_logger().error("Arm motors service call timeout")
                return False
        
        # step 2: takeoff to specified altitude
        self.get_logger().info(f"Taking off to {z}m...")
        takeoff_req = TakeoffSrv.Request()
        takeoff_req.alt = float(z)
        takeoff_future = self.takeoff_client.call_async(takeoff_req)
        
        if self.wait_for_future(takeoff_future, timeout_sec=5.0):
            takeoff_result = takeoff_future.result()
            # note: ardupilot takeoff service might not have a 'result' field. just check if service call completed.
            self.get_logger().info("Takeoff command sent, waiting for altitude...")
            
            # wait until we reach the target altitude
            start_time = time.time()
            timeout = 120.0  # 30 seconds timeout
            
            while rclpy.ok() and (time.time() - start_time) < timeout:
                telem_req = GetTelemetry.Request()
                telem_req.frame_id = 'map'
                telem_future = self.get_telemetry.call_async(telem_req)
                
                if self.wait_for_future(telem_future, timeout_sec=1.0):
                    telem = telem_future.result()
                    current_alt = telem.z
                    
                    self.get_logger().info(f"Current altitude: {current_alt:.2f}m / Target: {z:.2f}m", 
                                         throttle_duration_sec=1.0)
                    
                    # check if we've reached target altitude (within tolerance)
                    if abs(current_alt - z) < 0.5:
                        self.get_logger().info(f"Target altitude reached: {current_alt:.2f}m")
                        return True
                
                time.sleep(0.2)
            
            self.get_logger().error("Timeout waiting for target altitude")
            return False
        else:
            self.get_logger().error("Takeoff service call timeout")
            return False

    def navigateGlobalWait(self, lat, lon, z, yaw=float('nan'), speed=0.5, auto_arm=True):
        # navigate to gps coordinates and wait until the global nav service signals arrival
        try:
            # wait for service to be available
            if not self.navigate_global.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Navigate global service not available")
                return None
            
            req = NavigateGlobal.Request()
            req.lat = float(lat)
            req.lon = float(lon)
            req.z = float(z)
            req.yaw = float(yaw)
            req.yaw_rate = 0.0
            req.speed = float(speed)
            req.frame_id = 'map'
            req.auto_arm = auto_arm
            
            self.get_logger().info(f"Navigating to GPS: Lat={lat:.6f}, Lon={lon:.6f}, Alt={z}m")
            
            future = self.navigate_global.call_async(req)
            
            if not self.wait_for_future(future, timeout_sec=30.0):
                self.get_logger().error("Global navigation service call timeout after 30 seconds")
                return None
            
            res = future.result()
            if not res.success:
                self.get_logger().error(f"Failed to start global navigation: {res.message}")
                return res
            
            self.get_logger().info("Global navigation started, waiting for arrival...")
            
            # safety timeout
            nav_start = time.time()
            nav_timeout = 120.0
            
            # wait until the service itself signals navigation is done
            while rclpy.ok():
                if time.time() - nav_start > nav_timeout:
                    self.get_logger().warn(f"navigateGlobalWait timeout ({nav_timeout}s)")
                    return res
                
                if not self.global_nav_active:
                    self.get_logger().info("Target GPS location reached!")
                    return res
                
                time.sleep(0.2)

        except Exception as e:
            self.get_logger().error(f"Error during global navigation: {e}")
            return None

    def navigateWait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.2, frame_id='map', tolerance=0.1, auto_arm=True, z_participation = 1):
        # navigate and wait until the navigate service reports target reached via nav_active topic.
        try:
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
            
            if not self.wait_for_future(future, timeout_sec=10.0):
                self.get_logger().error("Navigation timeout")
                return None
            
            res = future.result()
            if not res.success:
                self.get_logger().error("Navigation failed")
                return res
            self.get_logger().info("Navigation started")
            
            # safety timeout
            nav_distance = math.sqrt(float(x)**2 + float(y)**2 + float(z)**2)
            nav_timeout = max(10.0, (nav_distance / max(speed, 0.1)) * 30.0 + 5.0)
            nav_start = time.time()
            
            # wait until the navigate service signals completion via nav_active topic
            while rclpy.ok():
                if time.time() - nav_start > nav_timeout:
                    self.get_logger().warn(f"navigateWait timeout ({nav_timeout:.1f}s)")
                    return res
                
                if not self.nav_active:
                    self.get_logger().info("Target reached (navigate service confirmed)")
                    return res
                
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Error calling navigation service: {e}")

    def navigateInterrupted(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, center_tolerance = 30):
        # navigate without interruption, wait until target is reached
        max_attempts = 2
        for i in range(max_attempts):  # limit the number of attempts
            try:
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
                
                if not self.wait_for_future(future, timeout_sec=10.0):
                    self.get_logger().error("Navigation timeout")
                    return "failed"
                
                res = future.result()
                if not res.success:
                    self.get_logger().error("Navigation failed")
                    return "failed"
                self.get_logger().info("Navigation started")
                while rclpy.ok():
                    # update last known base coordinates if detected
                    if self.consecutive_detections > self.required_consecutive_detections:
                        map_req = GetTelemetry.Request()
                        map_req.frame_id = 'map'
                        map_future = self.get_telemetry.call_async(map_req)
                        if self.wait_for_future(map_future, timeout_sec=1.0):
                            self.last_base_coordinates = map_future.result()
                            self.last_base_coordinates.x = self.last_base_coordinates.x - (self.y_center - self.image_height//2)/self.fy * self.last_base_coordinates.z
                            self.last_base_coordinates.y = self.last_base_coordinates.y - (self.x_center - self.image_width//2)/self.fx * self.last_base_coordinates.z
                    
                    # check if base is centered (interrupt navigation)
                    if self.is_base_centered(center_tolerance):
                        self.get_logger().info("Base found and centered. Target reached.")
                        self.navigateWait(x=0, y = 0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                        return "success"
                    
                    # check if navigate service reached target (base not found in this direction)
                    if not self.nav_active:
                        self.get_logger().info("Base not found in this direction. Trying again.")
                        y = -2 * y # invert direction and double the distance
                        auto_arm = False
                        break
                    
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Error calling navigation service: {e}")
                
        self.get_logger().info(f"Base was not centralized correctly. Searching again...")
        return "failed"

    def navigateCentralize(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, center_tolerance = 10):
        '''Navigate without interruption, wait until target is reached '''
        try:
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
            
            if not self.wait_for_future(future, timeout_sec=10.0):
                self.get_logger().error("Navigation timeout")
                return "failed"
            
            res = future.result()
            if not res.success:
                self.get_logger().error("Navigation failed")
                return "failed"
            self.get_logger().info("Navigation started")
            while rclpy.ok():
                # update last known base coordinates if detected
                if self.consecutive_detections > self.required_consecutive_detections:
                    map_req = GetTelemetry.Request()
                    map_req.frame_id = 'map'
                    map_future = self.get_telemetry.call_async(map_req)
                    if self.wait_for_future(map_future, timeout_sec=1.0):
                        self.last_base_coordinates = map_future.result()
                        self.last_base_coordinates.x = self.last_base_coordinates.x - (self.y_center - self.image_height//2)/self.fy * self.last_base_coordinates.z
                        self.last_base_coordinates.y = self.last_base_coordinates.y - (self.x_center - self.image_width//2)/self.fx * self.last_base_coordinates.z
                
                # check if base is centered on x axis (success)
                if self.is_base_in_x_Axis(center_tolerance):
                    self.get_logger().info("Base found and centered. Target reached.")
                    self.navigateWait(x=0, y = 0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                    return "success"
                
                # check if navigate service reached target (movement limit reached)
                if not self.nav_active:
                    self.get_logger().info("Movement limit reached.")
                    return "failure"
                
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Error calling navigation service: {e}")
                
        self.get_logger().info(f"Base was not centralized correctly. Searching again...")
        return "failed"
        



    def navigateWaitTeste(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=True, z_participation = 1):
        '''Navigate without interruption, wait until target is reached '''
        try:
            self.get_logger().info("Navigation started")
            telem_req = GetTelemetry.Request()
            telem_req.frame_id = 'map'
            init_future = self.get_telemetry.call_async(telem_req)
            if not self.wait_for_future(init_future, timeout_sec=2.0):
                self.get_logger().error("Telemetry timeout")
                return None
            initial = init_future.result()
            if frame_id == "body":
                initial.x += x
                initial.y += y
                initial.z += z
            if frame_id == "map":
                initial.x = x
                initial.y = y
                initial.z = z
            while rclpy.ok():
                telem_req2 = GetTelemetry.Request()
                telem_req2.frame_id = 'map'
                diff_future = self.get_telemetry.call_async(telem_req2)
                if not self.wait_for_future(diff_future, timeout_sec=1.0):
                    continue
                difference = diff_future.result()
                difference.x, difference.y, difference.z = (initial.x - difference.x), (initial.y - difference.y), (initial.z - difference.z)
                distance = math.sqrt(difference.x ** 2 + difference.y ** 2 + (difference.z * z_participation) ** 2)
                # self.get_logger().info(f"Distance to target: {distance:.2f} m")
                # self.get_logger().info(f"X: {diferenca.x}\nY: {diferenca.y}\n Z: {diferenca.z}\n")
                if distance < tolerance:
                    self.get_logger().info("Target reached")
                    return (0, 'Sucesso')
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Error calling navigation service: {e}")


    def navigateInterruptedTeste(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, center_tolerance = 30):
        # navigate without interruption, wait until target is reached
        max_attempts = 2
        for i in range(max_attempts):  # limit the number of attempts
            try:
                self.get_logger().info("Navigation started")
                telem_req = GetTelemetry.Request()
                telem_req.frame_id = 'map'
                init_future = self.get_telemetry.call_async(telem_req)
                if not self.wait_for_future(init_future, timeout_sec=2.0):
                    self.get_logger().error("Telemetry timeout")
                    return "failed"
                initial = init_future.result()
                if frame_id == "body":
                    initial.x += x
                    initial.y += y
                    initial.z += z
                if frame_id == "map":
                    initial.x = x
                    initial.y = y
                    initial.z = z
                while rclpy.ok():
                    telem_req2 = GetTelemetry.Request()
                    telem_req2.frame_id = 'map'
                    diff_future = self.get_telemetry.call_async(telem_req2)
                    if not self.wait_for_future(diff_future, timeout_sec=1.0):
                        continue
                    difference = diff_future.result()
                    difference.x, difference.y, difference.z = (initial.x - difference.x), (initial.y - difference.y), (initial.z - difference.z)
                    distance = math.sqrt(difference.x ** 2 + difference.y ** 2 + (difference.z * z_participation) ** 2)
                    if self.consecutive_detections > self.required_consecutive_detections:
                        map_req = GetTelemetry.Request()
                        map_req.frame_id = 'map'
                        map_future = self.get_telemetry.call_async(map_req)
                        if self.wait_for_future(map_future, timeout_sec=1.0):
                            self.last_base_coordinates = map_future.result()
                            self.last_base_coordinates.x = self.last_base_coordinates.x - (self.y_center - self.image_height//2)/self.fy * self.last_base_coordinates.z
                            self.last_base_coordinates.y = self.last_base_coordinates.y - (self.x_center - self.image_width//2)/self.fx * self.last_base_coordinates.z
                    # self.get_logger().info(f"Distance to target: {distance:.2f} m")
                    # self.get_logger().info(f"X: {diferenca.x}\nY: {diferenca.y}\n Z: {diferenca.z}\n")
                    if (distance < tolerance):
                        self.get_logger().info("Base not found in this direction. Trying again.")
                        y = -2 * y # invert direction and double the distance
                        auto_arm = False
                        break
                    if self.is_base_centered(center_tolerance):
                        self.get_logger().info("Base found and centered. Target reached.")
                        self.navigateWait(x=0, y = 0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                        return "success"
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Error calling navigation service: {e}")
                
        self.get_logger().info(f"Base was not centralized correctly. Searching again...")
        return "failed"

    def navigateCentralizeTeste(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=False, z_participation = 1, center_tolerance = 10):
        '''Navigate without interruption, wait until target is reached '''
        try:
            self.get_logger().info("Navigation started")
            telem_req = GetTelemetry.Request()
            telem_req.frame_id = 'map'
            init_future = self.get_telemetry.call_async(telem_req)
            if not self.wait_for_future(init_future, timeout_sec=2.0):
                self.get_logger().error("Telemetry timeout")
                return "failed"
            initial = init_future.result()
            if frame_id == "body":
                initial.x += x
                initial.y += y
                initial.z += z
            if frame_id == "map":
                initial.x = x
                initial.y = y
                initial.z = z
            while rclpy.ok():
                telem_req2 = GetTelemetry.Request()
                telem_req2.frame_id = 'map'
                diff_future = self.get_telemetry.call_async(telem_req2)
                if not self.wait_for_future(diff_future, timeout_sec=1.0):
                    continue
                difference = diff_future.result()
                difference.x, difference.y, difference.z = (initial.x - difference.x), (initial.y - difference.y), (initial.z - difference.z)
                distance = math.sqrt(difference.x ** 2 + difference.y ** 2 + (difference.z * z_participation) ** 2)
                if self.consecutive_detections > self.required_consecutive_detections:
                    map_req = GetTelemetry.Request()
                    map_req.frame_id = 'map'
                    map_future = self.get_telemetry.call_async(map_req)
                    if self.wait_for_future(map_future, timeout_sec=1.0):
                        self.last_base_coordinates = map_future.result()
                        self.last_base_coordinates.x = self.last_base_coordinates.x - (self.y_center - self.image_height//2)/self.fy * self.last_base_coordinates.z
                        self.last_base_coordinates.y = self.last_base_coordinates.y - (self.x_center - self.image_width//2)/self.fx * self.last_base_coordinates.z
                self.get_logger().info(f"Distância até o alvo: {distance:.2f} m")
                if (distance < tolerance):
                    self.get_logger().info("Movement limit reached.")
                    return "failure"
                if self.is_base_in_x_Axis(center_tolerance):
                    self.get_logger().info("Base found and centered. Target reached.")
                    self.navigateWait(x=0, y = 0, yaw=yaw, speed=speed, frame_id='body', auto_arm=False)
                    return "success"
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Error calling navigation service: {e}")
                
        self.get_logger().info(f"Base was not centralized correctly. Searching again...")
        return "failed"