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
from rclpy.executors import MultiThreadedExecutor
from drone_navigate.srv import GetTelemetry
from ardupilot_msgs.srv import ModeSwitch
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Pose2D
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from tools_mission2 import Tools
import colorful as cf
from states import *

class MasterMission2(Tools):
    def __init__(self) -> None:
        super().__init__()
        self.setSubscribers()
        self.setPublishers()
        self.setClients()
        self.setServer()
        
    def update(self):
        # mission 2 state machine logic. fsm: goingtobase -> search_base -> scan_for_base -> landing
        if self.fsm == 'Takeoff':
            self.current_state = 'Takeoff'
            self.get_logger().info("=" * 60)
            self.get_logger().info("STATE - Takeoff")
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"Taking off to mission altitude: {self.takeoff_alt}m...")
            
            if self.takeoffWait(z=self.takeoff_alt, auto_arm=True):
                self.get_logger().info("Takeoff successful!")
                self.current_state = ''
                self.fsm.add('takeoff_complete')
            else:
                self.get_logger().error("Takeoff failed! Ending mission.")
                self.mission_running = False
                return

        if self.fsm == 'GoingToBase':
            self.current_state = 'GoingToBase'
            self.get_logger().info("=" * 60)
            self.get_logger().info("STATE - GoingToBase")
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"Navigating to base area: Lat={self.base_lat}, Lon={self.base_lon}")

            # navigate to base gps coordinates
            result = self.navigateGlobalWait(
                lat=self.base_lat,
                lon=self.base_lon,
                z=self.takeoff_alt,
                speed=0.5,
                auto_arm=True
            )
            
            if result and result.success:
                self.get_logger().info("Reached base area!")
                self.fsm.add('reached_base_area')
                self.current_state = ''
            else:
                self.get_logger().error("Failed to reach base area")
                self.current_state = ''
            
        # search_base state
        if self.fsm == 'Search_Base':
            self.current_state = 'Search_Base'
            self.search_attempts += 1
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"STATE - Search_Base (attempt {self.search_attempts}/{self.max_search_attempts})")
            self.get_logger().info("=" * 60)
            
            # safety: check if max attempts exceeded
            if self.search_attempts > self.max_search_attempts:
                self.get_logger().warn(f"Search_Base: {self.max_search_attempts} attempts exhausted! Activating RTL...")
                self.current_state = ''
                self.fsm.add('rtl')
                self.fsm.updateEvent()
                return
            
            self.get_logger().info("Searching for mobile base with camera...")

            # move to search altitude before starting lateral search
            self.get_logger().info(f"Moving to search altitude: {self.search_alt}m...")
            telem_req = GetTelemetry.Request()
            telem_req.frame_id = 'map'
            telem_future = self.get_telemetry.call_async(telem_req)
            if self.wait_for_future(telem_future, timeout_sec=2.0):
                telem = telem_future.result()
                self.navigateWait(x= telem.x, y=telem.y, z=self.search_alt,
                                  speed=2.0, frame_id='map', tolerance=0.1, auto_arm=False)
                self.get_logger().info(f"Search altitude reached: {self.search_alt}m")

            # start search pattern - move laterally while looking for base
            self.get_logger().info("Starting lateral search pattern...")
            resposta = self.navigateInterrupted(y=-0.5, speed=0.3, frame_id='body', 
                                               tolerance=0.2, center_tolerance=40, auto_arm=False)
            
            if resposta == "failed":
                self.get_logger().warn("Base not found in initial search direction")
                self.current_state = ''
                self.fsm.add('base_lost') 
            elif resposta == "success":
                self.get_logger().info("Base detected and centered!")
                self.current_state = ''
                self.fsm.add('base_found')

        if self.fsm == 'Scan_for_Base':
            self.current_state = 'Scan_for_Base'
            self.scan_attempts += 1
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"STATE - Scan_for_Base (attempt {self.scan_attempts}/{self.max_search_attempts})")
            self.get_logger().info("=" * 60)
            
            # safety: check if max attempts exceeded
            if self.scan_attempts > self.max_search_attempts:
                self.get_logger().warn(f"Scan_for_Base: {self.max_search_attempts} attempts exhausted! Activating RTL...")
                self.current_state = ''
                self.fsm.add('rtl')
                self.fsm.updateEvent()
                return
            
            self.get_logger().info("Base lost, attempting to reacquire...")
            
            if self.last_base_coordinates is None:
                self.get_logger().error("Base never detected. Cannot recover. Activating Return Base...")
                self.return_base()
                self.current_state = ''
                self.fsm.add('rtl')
                self.fsm.updateEvent()
                return
            
            # return to last known base position
            self.get_logger().info(f"Returning to last known position: x={self.last_base_coordinates.x:.2f}, y={self.last_base_coordinates.y:.2f}")
            
            telem_req = GetTelemetry.Request()
            telem_req.frame_id = 'map'
            telem_future = self.get_telemetry.call_async(telem_req)
            
            if self.wait_for_future(telem_future, timeout_sec=1.0):
                telem = telem_future.result()
                self.navigateWait(x=self.last_base_coordinates.x, y=self.last_base_coordinates.y, 
                                 z=telem.z, speed=0.3, frame_id='map')
            
            time.sleep(1)
            
            # wait for base detection and try to center
            while rclpy.ok():
                if self.consecutive_detections > self.required_consecutive_detections:
                    if self.is_base_centered():
                        self.get_logger().info("Base reacquired and centered!")
                        self.current_state = ''
                        self.fsm.add('centered')
                        break
                    else:
                        # fine-tune position to center on base
                        self.get_logger().info("Fine-tuning position to center on base...")
                        telem_req2 = GetTelemetry.Request()
                        telem_req2.frame_id = 'map'
                        telem_future2 = self.get_telemetry.call_async(telem_req2)
                        
                        if self.wait_for_future(telem_future2, timeout_sec=1.0):
                            telem_atual = telem_future2.result()
                            distancia_y = -(self.x_center - self.image_width//2)/self.fx * telem_atual.z
                            distancia_x = -(self.y_center - self.image_height//2)/self.fy * telem_atual.z
                            resultado = self.navigateCentralize(x=distancia_x, y=distancia_y, 
                                                                     speed=0.15, frame_id='body')
                            if resultado == "success":
                                self.current_state = ''
                                self.fsm.add('centered')
                                break

                time.sleep(0.1)


        elif self.fsm == 'Descend_and_Centralize':
            self.current_state = 'Descend_and_Centralize'
            low_alt = self.search_alt * 0.4
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"STATE - Descend_and_Centralize")
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"Descending to {low_alt:.1f}m for precision centralization...")
            
            # descend to low altitude
            telem_req = GetTelemetry.Request()
            telem_req.frame_id = 'map'
            telem_future = self.get_telemetry.call_async(telem_req)
            
            if self.wait_for_future(telem_future, timeout_sec=2.0):
                telem = telem_future.result()
                self.navigateWait(x=telem.x, y=telem.y, z=low_alt,
                                  speed=1.0, frame_id='map', tolerance=0.2, auto_arm=False)
                self.get_logger().info(f"Low altitude reached: {low_alt:.1f}m")
            else:
                self.get_logger().error("Telemetry timeout during descent")
                self.current_state = ''
                self.fsm.add('base_lost')
                self.fsm.updateEvent()
                return
            
            time.sleep(0.5)
            
            # centralize once at low altitude
            self.get_logger().info("Centralizing on base at low altitude...")
            telem_req2 = GetTelemetry.Request()
            telem_req2.frame_id = 'map'
            telem_future2 = self.get_telemetry.call_async(telem_req2)
            
            if self.wait_for_future(telem_future2, timeout_sec=1.0):
                telem_atual = telem_future2.result()
                distancia_y = -(self.x_center - self.image_width//2)/self.fx * telem_atual.z
                distancia_x = -(self.y_center - self.image_height//2)/self.fy * telem_atual.z
                resultado = self.navigateCentralize(x=distancia_x, y=distancia_y,
                                                   speed=0.1, frame_id='body',
                                                   center_tolerance=15)
                if resultado == "success":
                    self.get_logger().info("Base centered at low altitude! Ready for landing.")
                    self.current_state = ''
                    self.fsm.add('centered_low')
                else:
                    self.get_logger().warn("Failed to centralize at low altitude, going back to scan")
                    self.current_state = ''
                    self.fsm.add('base_lost')
            else:
                self.get_logger().error("Telemetry timeout during centralization")
                self.current_state = ''
                self.fsm.add('base_lost')

        elif self.fsm == 'Landing':
            self.current_state = 'Landing'
            self.get_logger().info("=" * 60)
            self.get_logger().info("STATE - Landing")
            self.get_logger().info("=" * 60)
            self.get_logger().info("Waiting for base alignment to execute landing...")
            
            try:
                while rclpy.ok():
                    if self.is_base_in_y_Axis(tolerance=15):
                        self.get_logger().info("Base aligned! Executing landing sequence...")
                        self.land()
                        
                        # wait for landing to actually complete
                        if self.wait_for_landing(timeout=300.0):
                            self.get_logger().info("★" * 30)
                            self.get_logger().info("Landing complete! Mission successful!")
                            self.get_logger().info("★" * 30)
                            self.current_state = ''
                            self.fsm.add('finished')
                            self.mission_running = False
                        else:
                            self.get_logger().error("Landing timeout or failed")
                            self.mission_running = False
                        break
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Error during landing: {e}")
        
        # returnbase state (safety)
        elif self.fsm == 'ReturnBase':
            self.current_state = 'ReturnBase'
            self.get_logger().warn("=" * 60)
            self.get_logger().warn("STATE - ReturnBase (SAFETY)")
            self.get_logger().warn("=" * 60)
            self.get_logger().warn(f"Base not found after multiple attempts.")
            self.get_logger().warn(f"(Search: {self.search_attempts}, Scan: {self.scan_attempts})")
            self.get_logger().warn("Activating Return Base for drone safety!")
            
            self.return_base()
            
            self.current_state = ''
            self.fsm.add('rtl_activated')
            self.mission_running = False
            
        # update internal fsm state
        if self.fsm != self.current_state:
            self.fsm.updateEvent()


def main(): 
    rclpy.init()
    mestre = MasterMission2()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(mestre)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        mestre.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()