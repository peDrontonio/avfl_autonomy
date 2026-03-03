#!/usr/bin/python3
"""
Mission 1: ArUco Gate Passing — Simplified Flow

Gate Layout (ArUco markers 0-3):
    2 --- 3
    |     |
    0 --- 1

FSM States:
1. TAKEOFF      – Arm and take off to search altitude.
2. SEARCHING    – Lateral search until ≥ 2 markers found.
3. ALIGNING     – Continuous movement in the direction dictated by the
                  visible marker pair until all 4 markers are visible.
4. CENTERING    – Phase A: center with HIGH tolerance, log distance.
                  Phase B: approach until 3 m from gate.
                  Phase C: center with LOW tolerance.
5. ADVANCING    – Fly through the gate.
6. LANDING      – Land the drone.
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
        self.search_distance = 6.0    # metres to search in each direction
        self.search_speed = 0.2       # m/s for lateral search
        self.advance_distance = 6.0   # metres to fly through gate
        self.advance_speed = 0.3      # m/s for advancing
        self.approach_target = 3.0    # stop approaching at this distance (m)
        self.high_tolerance = 0.20    # loose centering (m)
        self.low_tolerance = 0.08     # tight centering (m)

        self.declare_parameter('takeoff_alt', 10.0)
        self.takeoff_alt = self.get_parameter('takeoff_alt').get_parameter_value().double_value
        self.get_logger().info(f"Takeoff altitude: {self.takeoff_alt}m")

        self.fsm = Takeoff()
        
    def update(self):
        """Main state-machine loop (called repeatedly from the mission service)."""

        # ─────────────────────── TAKEOFF ───────────────────────
        if self.fsm == 'Takeoff':
            print(cf.blue("=" * 50))
            print(cf.blue("State — TAKEOFF"))
            print(cf.blue("=" * 50))
            self.get_logger().info(f"Taking off to {self.takeoff_alt}m...")

            if self.takeoffWait(z=self.takeoff_alt, auto_arm=True):
                self.get_logger().info("Takeoff complete!")
                self.fsm.add('takeoff_complete')
                self.fsm.updateEvent()
            else:
                self.get_logger().error("Takeoff failed! Ending mission.")
                self.mission_running = False
            return

        # ─────────────────────── SEARCHING ───────────────────────
        if self.fsm == 'Searching':
            print(cf.blue("=" * 50))
            print(cf.blue("State — SEARCHING"))
            print(cf.blue("=" * 50))
            print(cf.yellow(f"Current marker count: {self.marker_count}"))

            time.sleep(0.2)

            # Already have ≥ 2 markers?
            if self.marker_count >= 2:
                print(cf.green(f"{self.marker_count} markers detected, skipping search."))
                self._stop_drone()
                self.fsm.add('markers_found')
                self.fsm.updateEvent()
                return

            # Lateral search
            found = self.searchForGate(
                search_distance=self.search_distance,
                search_speed=self.search_speed
            )
            time.sleep(0.1)

            if found or self.marker_count >= 2:
                print(cf.green(f"Gate found! Markers: {self.marker_count}"))
                self.fsm.add('markers_found')
                self.fsm.updateEvent()
            else:
                print(cf.red(f"Gate not found ({self.marker_count} markers), retrying..."))
                time.sleep(1)
            return

        # ─────────────────────── ALIGNING ───────────────────────
        if self.fsm == 'Aligning':
            print(cf.blue("=" * 50))
            print(cf.blue("State — ALIGNING (continuous)"))
            print(cf.blue("=" * 50))
            print(cf.yellow(f"Markers: {self.marker_count}/4  IDs: {self.get_detected_marker_ids()}"))

            # Already all visible?
            if self.check_all_markers_visible():
                print(cf.green("All 4 markers already visible!"))
                self._stop_drone()
                self.fsm.add('all_visible')
                self.fsm.updateEvent()
                return

            if self.marker_count < 2:
                print(cf.red("Lost markers, back to Searching"))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
                return

            # Determine direction from the visible marker pair
            detected_ids = self.get_detected_marker_ids()
            y_move = 0.0
            z_move = 0.0
            align_dist = 3.0

            if detected_ids == {0, 1}:    # Bottom pair → move UP
                z_move = align_dist
            elif detected_ids == {2, 3}:  # Top pair → move DOWN
                z_move = -align_dist
            elif detected_ids == {1, 3}:  # Right pair → move LEFT (y+)
                y_move = align_dist
            elif detected_ids == {0, 2}:  # Left pair → move RIGHT (y-)
                y_move = -align_dist
            else:
                # 3 markers or diagonal: use gate_error
                ex = self.gate_error.x
                ey = self.gate_error.y
                if not math.isnan(ex):
                    y_move = -ex * 2.0
                if not math.isnan(ey):
                    z_move = -ey * 2.0
                y_move = max(-align_dist, min(align_dist, y_move))
                z_move = max(-align_dist, min(align_dist, z_move))

            print(cf.yellow(f"Moving y={y_move:.2f}, z={z_move:.2f} until 4 markers visible"))

            result = self.navigateInterruptedByMarkers(
                x=0.0, y=y_move, z=z_move,
                speed=0.3, frame_id='body',
                min_markers=4, timeout=20.0
            )

            time.sleep(0.1)

            if result == 'markers_found' or self.marker_count >= 4:
                print(cf.green("All 4 markers visible!"))
                self.fsm.add('all_visible')
                self.fsm.updateEvent()
            elif self.marker_count < 2:
                print(cf.red("Lost markers during alignment"))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
            else:
                # Still 2-3 markers, loop back and re-evaluate direction
                print(cf.orange(f"Still {self.marker_count} markers, re-evaluating..."))
            return

        # ─────────────────────── CENTERING ───────────────────────
        if self.fsm == 'Centering':
            print(cf.blue("=" * 50))
            print(cf.blue("State — CENTERING"))
            print(cf.blue("=" * 50))
            print(cf.yellow(f"Gate error: X={self.gate_error.x:.3f}  Y={self.gate_error.y:.3f}"))
            print(cf.yellow(f"Markers: {self.marker_count}/4  Distance: {self.distance_to_gate:.1f}m"))

            # Safety: still see markers?
            if self.marker_count < 2:
                print(cf.red("Lost markers! Back to Searching."))
                self.fsm.add('markers_lost')
                self.fsm.updateEvent()
                return

            # ── Phase A: Loose centering ──
            print(cf.yellow(f"Phase A — Centering with HIGH tolerance ({self.high_tolerance}m)..."))
            self._stop_drone()
            time.sleep(0.3)

            centered_loose = self.centerOnGate(
                timeout=30.0,
                acceptance_radius=self.high_tolerance
            )
            if not centered_loose:
                if self.marker_count < 2:
                    self.fsm.add('markers_lost')
                    self.fsm.updateEvent()
                    return
                print(cf.orange("Loose centering incomplete, retrying state..."))
                time.sleep(0.5)
                return

            print(cf.green(f"Loosely centered!  Distance to gate: {self.distance_to_gate:.1f}m"))

            # ── Phase B: Approach to 3 m ──
            if self.distance_to_gate > self.approach_target:
                forward = self.distance_to_gate - self.approach_target
                print(cf.yellow(f"Phase B — Approaching {forward:.1f}m forward (dist={self.distance_to_gate:.1f}m → {self.approach_target}m)"))
                self.navigateWait(
                    x=forward, y=0, z=0,
                    speed=0.3, frame_id='body',
                    tolerance=0.3, auto_arm=False
                )
                time.sleep(0.3)
                print(cf.green(f"Distance to gate now: {self.distance_to_gate:.1f}m"))
            else:
                print(cf.green(f"Already within {self.approach_target}m ({self.distance_to_gate:.1f}m)"))

            # ── Phase C: Tight centering ──
            print(cf.yellow(f"Phase C — Centering with LOW tolerance ({self.low_tolerance}m)..."))
            self._stop_drone()
            time.sleep(0.3)

            centered_tight = self.centerOnGate(
                timeout=30.0,
                acceptance_radius=self.low_tolerance
            )
            if not centered_tight:
                if self.marker_count < 2:
                    self.fsm.add('markers_lost')
                    self.fsm.updateEvent()
                    return
                print(cf.orange("Tight centering incomplete, retrying state..."))
                time.sleep(0.5)
                return

            # ── Post-centering vertical offset ──
            if self.centering_y_offset != 0.0:
                print(cf.yellow(f"Applying vertical offset: {self.centering_y_offset:+.2f}m"))
                self.navigateWait(
                    x=0, y=0, z=self.centering_y_offset,
                    speed=self.centering_speed, frame_id='body',
                    tolerance=0.05, auto_arm=False
                )

            print(cf.green("=" * 50))
            print(cf.green("CENTERED & CLOSE — ready to pass through gate!"))
            print(cf.green(f"  Error: X={self.gate_error.x:.3f}  Y={self.gate_error.y:.3f}"))
            print(cf.green(f"  Distance: {self.distance_to_gate:.1f}m"))
            print(cf.green("=" * 50))
            self.fsm.add('ready_to_pass')
            self.fsm.updateEvent()
            return

        # ─────────────────────── ADVANCING ───────────────────────
        if self.fsm == 'Advancing':
            print(cf.blue("=" * 50))
            print(cf.blue("State — ADVANCING"))
            print(cf.blue("=" * 50))
            print(cf.yellow("Flying through the gate..."))

            success = self.advanceThroughGate(
                advance_distance=self.advance_distance,
                advance_speed=self.advance_speed
            )

            print(cf.green("Ready to land!"))
            self.fsm.add('crossed')
            self.fsm.updateEvent()
            return

        # ─────────────────────── LANDING ───────────────────────
        if self.fsm == 'Landing':
            print(cf.blue("=" * 50))
            print(cf.blue("State — LANDING"))
            print(cf.blue("=" * 50))

            self.landDrone()

            print(cf.green("Landing complete!"))
            time.sleep(2)

            self.fsm.add('landed')
            self.fsm.updateEvent()
            self.mission_running = False
            print(cf.green("=" * 50))
            print(cf.green("MISSION 1 COMPLETED!"))
            print(cf.green("=" * 50))
            return




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