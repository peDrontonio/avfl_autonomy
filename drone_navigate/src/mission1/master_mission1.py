#!/usr/bin/python3
"""
Mission 1: ArUco Gate Passing with Intelligent Alignment

Gate Layout (ArUco markers 0-3):
    2 --- 3
    |     |
    0 --- 1

FSM States and Logic:
1. SEARCHING: No markers visible
   - Lateral search pattern
   - Transition: markers detected -> ALIGNING

2. ALIGNING: 1-3 markers visible
   - Intelligent alignment based on detected marker pairs:
     * {0, 1} (bottom) -> Move UP
     * {2, 3} (top) -> Move DOWN
     * {1, 3} (right) -> Move LEFT
     * {0, 2} (left) -> Move RIGHT
   - Transition: all 4 markers visible -> CENTERING

3. CENTERING: All 4 markers visible
   - Stop completely
   - Center on gate using ArUco error feedback
   - Transition: centered -> ADVANCING

4. ADVANCING: Centered on gate
   - Fly through gate (advance_distance meters)
   - Move forward 1 additional meter
   - Transition: completed -> LANDING

5. LANDING: Mission complete
   - Switch to LAND mode
"""
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
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
        self.search_speed = 0.2     # m/s for search movement
        self.advance_distance = 5.0  # meters to fly through gate
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
            
            # Brief wait to ensure latest ArUco data is processed
            time.sleep(0.2)
            
            # Check if we already see all 4 markers
            print(cf.yellow(f"Current marker count: {self.marker_count}"))
            if self.marker_count >= 4:
                print(cf.green(f"All 4 markers detected! Stopping and transitioning to Centering..."))
                # Stop the drone
                self.navigateWait(x=0, y=0, z=0, frame_id='body', auto_arm=False, tolerance=0.1)
                self.fsm.add('all_markers_visible')
                self.fsm.updateEvent()
                time.sleep(2)
                return
            
            # Check if we see 2-3 markers (partial detection)
            if self.marker_count >= 2:
                print(cf.green(f"Partial detection ({self.marker_count} markers)! Stopping and transitioning to PartialCentering..."))
                # Stop the drone
                self.navigateWait(x=0, y=0, z=0, frame_id='body', auto_arm=False, tolerance=0.1)
                self.fsm.add('partial_detected')
                self.fsm.updateEvent()
                time.sleep(2)
                return
            
            # Search for the gate
            found = self.searchForGate(
                search_distance=self.search_distance,
                search_speed=self.search_speed
            )
            
            # Brief wait for latest data after search
            time.sleep(0.1)
            
            if self.marker_count >= 4:
                print(cf.green(f"All markers found! Markers: {self.marker_count}"))
                self.fsm.add('all_markers_visible')
                self.fsm.updateEvent()
                time.sleep(2)
            elif found or self.marker_count >= 2:
                print(cf.green(f"Partial detection! Markers: {self.marker_count}"))
                self.fsm.add('partial_detected')
                self.fsm.updateEvent()
                time.sleep(2)
            else:
                print(cf.red(f"Gate not found (need 2+ markers, have {self.marker_count}), continuing search..."))
                time.sleep(1)
        
        elif self.fsm == 'PartialCentering':
            print(cf.blue("=" * 50))
            print(cf.blue("State - PARTIAL CENTERING"))
            print(cf.blue("=" * 50))
            print(cf.yellow(f"Markers visible: {self.marker_count}/4"))
            print(cf.yellow(f"Detected IDs: {self.get_detected_marker_ids()}"))
            
            # Callbacks processed by MultiThreadedExecutor automatically
            
            # Check if all 4 markers are now visible
            if self.marker_count >= 4:
                print(cf.green("All 4 markers visible! Transitioning to Centering..."))
                self.navigateWait(x=0, y=0, z=0, frame_id='body', auto_arm=False, tolerance=0.1)
                self.fsm.add('all_markers_visible')
                self.fsm.updateEvent()
                time.sleep(2)
                return
            
            # Check if we lost markers (< 2)
            if self.marker_count < 2:
                print(cf.red("Lost markers! Returning to Searching..."))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
                time.sleep(2)
                return
            
            # Center on the partial markers (X axis only)
            centered = self.partialCenterOnGate(timeout=15.0)
            
            # Check again after centering
            time.sleep(0.1)
            
            if self.marker_count >= 4:
                print(cf.green("All markers visible after partial centering!"))
                self.fsm.add('all_markers_visible')
                self.fsm.updateEvent()
                time.sleep(2)
            elif centered:
                print(cf.green("Partial centering complete! Transitioning to Aligning..."))
                self.fsm.add('partial_centered')
                self.fsm.updateEvent()
                time.sleep(2)
            elif self.marker_count < 2:
                print(cf.red("Lost markers during partial centering"))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
                time.sleep(2)
            
        elif self.fsm == 'Aligning':
            print(cf.blue("=" * 50))
            print(cf.blue("State - ALIGNING"))
            print(cf.blue("=" * 50))
            print(cf.yellow(f"Markers visible: {self.marker_count}/4"))
            print(cf.yellow(f"Detected IDs: {self.get_detected_marker_ids()}"))
            
            # Callbacks processed by MultiThreadedExecutor automatically
            
            # Check if all markers are already visible
            if self.marker_count >= 4:
                print(cf.green("All markers visible! Transitioning to Centering..."))
                self.navigateWait(x=0, y=0, z=0, frame_id='body', auto_arm=False, tolerance=0.1)
                self.fsm.add('all_markers_visible')
                self.fsm.updateEvent()
                time.sleep(2)
                return
            
            # Check if we lost all markers (< 2)
            if self.marker_count < 2:
                print(cf.red("Lost all markers! Returning to Searching..."))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
                time.sleep(2)
                return
            
            # Try to align to see all markers (adjust based on detected marker pair)
            aligned = self.alignToGate(timeout=20.0)
            
            # Brief wait for latest data after alignment
            time.sleep(0.1)
            
            if self.marker_count >= 4:
                print(cf.green("Alignment complete! All markers visible!"))
                self.fsm.add('all_markers_visible')
                self.fsm.updateEvent()
                time.sleep(2)
            elif self.marker_count < 2:
                print(cf.red("Lost markers during alignment"))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
                time.sleep(2)
            else:
                # Still have 2-3 markers, go back to partial centering
                print(cf.orange("Still have partial detection, returning to PartialCentering..."))
                self.fsm.add('partial_lost')
                self.fsm.updateEvent()
                time.sleep(2)

        elif self.fsm == 'Centering':
            print(cf.blue("=" * 50))
            print(cf.blue("State - CENTERING"))
            print(cf.blue("=" * 50))
            print(cf.yellow(f"Gate error: X={self.gate_error.x:.3f}, Y={self.gate_error.y:.3f}"))
            print(cf.yellow(f"Markers visible: {self.marker_count}/4"))
            
            # Callbacks processed by MultiThreadedExecutor automatically
            
            # Stop completely before starting centering
            print(cf.yellow("Stopping before centering..."))
            self.navigateWait(x=0, y=0, z=0, frame_id='body', auto_arm=False, tolerance=0.05)
            time.sleep(0.5)
            
            # VERIFY: Must have all 4 markers visible to stay in Centering
            if self.marker_count < 2:
                print(cf.red("Lost all markers! Returning to Searching..."))
                self.fsm.add('all_markers_lost')
                self.fsm.updateEvent()
                time.sleep(2)
                return
            elif self.marker_count < 4:
                print(cf.orange(f"Lost some markers ({self.marker_count}/4), returning to PartialCentering..."))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
                time.sleep(2)
                return
            
            # Center on the gate (both X and Y)
            centered = self.centerOnGate(timeout=30.0)
            
            # Brief wait for latest data after centering
            time.sleep(0.1)
            
            # VERIFY BOTH CONDITIONS: 4 markers visible AND centered
            if self.marker_count >= 4 and (centered or self.check_centered()):
                print(cf.green("=" * 50))
                print(cf.green("VERIFIED: 4 markers visible AND centered!"))
                print(cf.green(f"  Markers: {self.marker_count}/4"))
                print(cf.green(f"  Error: X={self.gate_error.x:.3f}, Y={self.gate_error.y:.3f}"))
                print(cf.green("  Ready to advance through gate!"))
                print(cf.green("=" * 50))
                self.fsm.add('centered')
                self.fsm.updateEvent()
                time.sleep(2)
            elif self.marker_count < 2:
                print(cf.red("Lost all markers during centering"))
                self.fsm.add('all_markers_lost')
                self.fsm.updateEvent()
                time.sleep(2)
            elif self.marker_count < 4:
                print(cf.orange(f"Lost some markers during centering ({self.marker_count}/4)"))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
                time.sleep(2)
            else:
                # Still have 4 markers but not centered yet - keep trying
                print(cf.yellow("Still centering... (4 markers visible but not centered yet)"))
        
        elif self.fsm == 'Advancing':
            print(cf.blue("=" * 50))
            print(cf.blue("State - ADVANCING"))
            print(cf.blue("=" * 50))
            print(cf.yellow("Flying through the gate..."))
            
            # Callbacks processed by MultiThreadedExecutor automatically
            
            # VERIFY before advancing: should still see markers and be centered
            if self.marker_count >= 4 and not self.check_centered():
                print(cf.orange("Lost centering (still have 4 markers), re-centering..."))
                self.fsm.add('lost_alignment')
                self.fsm.updateEvent()
                time.sleep(2)
                return
            
            # Advance through the gate
            success = self.advanceThroughGate(
                advance_distance=self.advance_distance,
                advance_speed=self.advance_speed
            )
            
            if success:
                print(cf.green("Gate crossed! Moving forward 1 meter..."))
                # Move forward an additional 1 meter after crossing
                self.navigateWait(
                    x=1.0, y=0, z=0,
                    speed=0.3, frame_id='body',
                    tolerance=0.2, auto_arm=False
                )
                print(cf.green("Completed post-gate movement!"))
            
            print(cf.green("Ready to land!"))
            self.fsm.add('crossed')
            self.fsm.updateEvent()
            time.sleep(2)
            
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