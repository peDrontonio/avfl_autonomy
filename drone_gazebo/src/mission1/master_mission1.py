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
from ardupilot_msgs.srv import ModeSwitch
from tools_mission1 import Tools
import colorful as cf
from states import *

# ArduPilot flight modes
COPTER_MODE_LAND = 9

class MasterMission1(Tools):
    def __init__(self) -> None:
        super().__init__()
        self.setSubscribers()
        self.setPublishers()
        self.setClients()
        self.setServer()
        
        # Mission parameters
        self.search_distance = 6.0  # meters to search in each direction
        self.search_speed = 0.3     # m/s for search movement
        self.advance_distance = 3.0  # meters to fly through gate
        self.advance_speed = 0.3    # m/s for advancing
        
    def update(self):
        """
        State machine logic for Mission 1 - Gate Passing with ArUco Detection.
        
        States:
        - Searching: Look for the gate by moving laterally
        - Aligning: Partial markers visible, move to see all 4
        - Centering: All markers visible, center on gate
        - Advancing: Fly through the gate
        - Landing: Land the drone
        """
        
        if self.fsm == 'Searching':
            print(cf.blue("=" * 50))
            print(cf.blue("State - SEARCHING"))
            print(cf.blue("=" * 50))
            print(cf.yellow("Looking for gate markers..."))
            
            # Check if we already see markers
            if self.check_markers_detected():
                print(cf.green("Markers detected! Transitioning to Aligning..."))
                self.fsm.add('markers_detected')
                self.fsm.updateEvent()
                return
            
            # Search for the gate
            found = self.searchForGate(
                search_distance=self.search_distance,
                search_speed=self.search_speed
            )
            
            if found or self.check_markers_detected():
                print(cf.green("Gate found!"))
                self.fsm.add('markers_detected')
                self.fsm.updateEvent()
            else:
                print(cf.red("Gate not found, continuing search..."))
                time.sleep(1)
            
        elif self.fsm == 'Aligning':
            print(cf.blue("=" * 50))
            print(cf.blue("State - ALIGNING"))
            print(cf.blue("=" * 50))
            print(cf.yellow(f"Markers visible: {self.marker_count}/4"))
            
            # Check if all markers are already visible
            if self.check_all_markers_visible():
                print(cf.green("All markers visible! Transitioning to Centering..."))
                self.fsm.add('all_markers_visible')
                self.fsm.updateEvent()
                return
            
            # Check if we lost all markers
            if self.check_markers_lost():
                print(cf.red("Lost all markers! Returning to Searching..."))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
                return
            
            # Try to align to see all markers
            aligned = self.alignToGate(timeout=20.0)
            
            if aligned or self.check_all_markers_visible():
                print(cf.green("Alignment complete!"))
                self.fsm.add('all_markers_visible')
                self.fsm.updateEvent()
            elif self.check_markers_lost():
                print(cf.red("Lost markers during alignment"))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()

        elif self.fsm == 'Centering':
            print(cf.blue("=" * 50))
            print(cf.blue("State - CENTERING"))
            print(cf.blue("=" * 50))
            print(cf.yellow(f"Gate error: X={self.gate_error.x:.3f}, Y={self.gate_error.y:.3f}"))
            
            # Check if we lost markers
            if self.marker_count < 3:
                if self.check_markers_lost():
                    print(cf.red("Lost all markers! Returning to Searching..."))
                    self.fsm.add('all_markers_lost')
                    self.fsm.updateEvent()
                    return
                else:
                    print(cf.orange("Lost some markers, returning to Aligning..."))
                    self.fsm.add('markers_lost')
                    self.fsm.updateEvent()
                    return
            
            # Check if already centered
            if self.check_centered():
                print(cf.green("Centered! Transitioning to Advancing..."))
                self.fsm.add('centered')
                self.fsm.updateEvent()
                return
            
            # Center on the gate
            centered = self.centerOnGate(timeout=30.0)
            
            if centered or self.check_centered():
                print(cf.green("Centering complete!"))
                self.fsm.add('centered')
                self.fsm.updateEvent()
            elif self.check_markers_lost():
                print(cf.red("Lost markers during centering"))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
        
        elif self.fsm == 'Advancing':
            print(cf.blue("=" * 50))
            print(cf.blue("State - ADVANCING"))
            print(cf.blue("=" * 50))
            print(cf.yellow("Flying through the gate..."))
            
            # Check if still centered before advancing
            if not self.check_centered() and self.marker_count >= 3:
                print(cf.orange("Lost centering, re-centering..."))
                self.fsm.add('lost_alignment')
                self.fsm.updateEvent()
                return
            
            # Advance through the gate
            success = self.advanceThroughGate(
                advance_distance=self.advance_distance,
                advance_speed=self.advance_speed
            )
            
            print(cf.green("Gate crossed!"))
            self.fsm.add('crossed')
            self.fsm.updateEvent()
            
        elif self.fsm == 'Landing':
            print(cf.blue("=" * 50))
            print(cf.blue("State - LANDING"))
            print(cf.blue("=" * 50))
            print(cf.yellow("Landing the drone..."))
            
            # Land the drone
            self.landDrone()
            
            print(cf.green("Landing complete!"))
            time.sleep(2)
            
            # Finish FSM
            self.fsm.add('landed')
            self.fsm.updateEvent()
            self.mission_running = False
            print(cf.green("=" * 50))
            print(cf.green("MISSION 1 COMPLETED!"))
            print(cf.green("=" * 50))




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