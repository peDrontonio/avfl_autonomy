#!/usr/bin/python3
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from drone_navigate.srv import GetTelemetry, Navigate
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Pose2D
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from tools_mission1 import Tools
import colorful as cf
from states import *

class MasterMission1(Tools):
    def __init__(self) -> None:
        super().__init__()
        self.setSubscribers()
        self.setPublishers()
        self.setClients()
        self.setServer()
        
    def update(self):
        """
        State machine logic for Mission 1.
        """
        
        if self.fsm == 'Searching':
            print(cf.blue("State - Searching"))
            print(cf.yellow("Searching for the gate"))
            self.navigateInterrupted(y=-5, frame_id='body', speed=0.2, auto_arm=False)
            time.sleep(2)
            print(cf.yellow("Searching complete"))
            # Add event and transition to next state
            self.fsm.add('markers_detected')
            self.fsm.updateEvent()
            
        elif self.fsm == 'Aligning':
            print(cf.blue("State - Aligning"))
            print(cf.yellow("Aligning with markers"))
            time.sleep(1)
            
            print(cf.yellow("Aligning complete"))
            # Add event and transition to next state
            self.fsm.add('all_markers_visible')
            self.fsm.updateEvent()

        elif self.fsm == 'Centering':
            print(cf.blue("State - Centering"))
            print(cf.yellow("Centering on gate"))
            time.sleep(1)
            print(cf.yellow("Centering complete"))
            # Add event and transition to next state
            self.fsm.add('centered')
            self.fsm.updateEvent()
        
        elif self.fsm == 'Advancing':
            print(cf.blue("State - Advancing"))
            print(cf.yellow("Moving forward through gate"))
            
            # Wait for navigate service
            self.get_logger().info("Waiting for navigate service...")
            if not self.navigate.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("Navigate service not available after 10s")
            else:
                # Navigate forward
                req = Navigate.Request()
                req.x = 2.0
                req.y = 0.0
                req.z = 0.0
                req.yaw = 0.0
                req.yaw_rate = 0.0
                req.speed = 0.2
                req.frame_id = 'body'
                req.auto_arm = False
                
                self.get_logger().info("Calling navigate service...")
                future = self.navigate.call_async(req)
                
                # Wait for response with timeout
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > 15.0:
                        self.get_logger().error("Navigate timeout")
                        break
                    time.sleep(0.1)
                
                if future.done():
                    res = future.result()
                    self.get_logger().info(f"Navigate result: {res.success}, {res.message}")
            
            time.sleep(2)
            print(cf.yellow("Advancing complete"))
            # Add event and transition to landing
            self.fsm.add('crossed')
            self.fsm.updateEvent()
            
        elif self.fsm == 'Landing':
            print(cf.blue("State - Landing"))
            
            # Wait for land service
            self.get_logger().info("Waiting for land service...")
            if not self.land.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("Land service not available after 10s")
            else:
                # Land
                self.get_logger().info("Calling land service...")
                land_future = self.land.call_async(Trigger.Request())
                
                # Wait for response with timeout
                start_time = time.time()
                while not land_future.done():
                    if time.time() - start_time > 15.0:
                        self.get_logger().error("Land service timeout")
                        break
                    time.sleep(0.1)
                
                if land_future.done():
                    res = land_future.result()
                    self.get_logger().info(f"Land result: {res.success}, {res.message}")
                    
            print(cf.yellow("Landing complete"))
            time.sleep(1)
            # Add event to finish FSM
            self.fsm.add('landed')
            self.fsm.updateEvent()
            self.mission_running = False




def main(args=None): 
    rclpy.init(args=args)
    mestre = MasterMission1()
    try:
        rclpy.spin(mestre)
    except KeyboardInterrupt:
        pass
    finally:
        mestre.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 