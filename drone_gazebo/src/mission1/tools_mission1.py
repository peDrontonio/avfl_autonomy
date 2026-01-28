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
from std_msgs.msg import Int32, Float32
from drone_navigate.srv import Navigate, GetTelemetry, SetYawRate
from std_srvs.srv import Trigger, SetBool
import colorful as cf

class Tools(Node):
    def __init__(self) -> None:
        super().__init__('tools_mission1_node')
        self.fsm = Searching()
        self.start_mission = False
        self.mission_running = False
        self.distance_error = 0.0
        self.max_attempts = 3

        # Wait for camera service
        self.activate_camera = self.create_client(SetBool, '/start_camera')
        while not self.activate_camera.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /start_camera service...')
        
        req = SetBool.Request()
        req.data = True
        future = self.activate_camera.call_async(req)
        rclpy.spin_until_future_complete(self, future)


    def setSubscribers(self):
        self.create_subscription(Float32, '/target_position', self.arch_dx, 10)

    def setPublishers(self):
        None

    def setClients(self):
        self.navigate = self.create_client(Navigate, '/navigate')
        self.land = self.create_client(Trigger, '/land')
        self.set_yaw_rate = self.create_client(SetYawRate, '/set_yaw_rate')
        self.get_telemetry = self.create_client(GetTelemetry, '/get_telemetry')        
        

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
        
        if self.land.wait_for_service(timeout_sec=5.0):
            land_future = self.land.call_async(Trigger.Request())
            start_time = time.time()
            while not land_future.done() and (time.time() - start_time < 10.0):
                time.sleep(0.1)
        
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