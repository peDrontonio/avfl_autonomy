#!/usr/bin/env python3
"""
Centralized Mission Node
Features:
- "Flower" Search Pattern: Center -> Right -> Center -> Left -> Center -> Advance
- 4-Marker Lock Requirement
- HUD State Publishing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import String
from ardupilot_msgs.srv import ModeSwitch
import numpy as np
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

class CentralizedMissionNode(Node):
    def __init__(self):
        super().__init__('centralized_mission_node')
        
        # ==========================================
        # PARAMETERS
        # ==========================================
        self.declare_parameter('search_velocity', 1.0) 
        self.declare_parameter('centering_velocity', 0.3)
        self.declare_parameter('fly_velocity', 1.0)
        
        # Distance from Center to the Side (e.g., 6 meters)
        self.declare_parameter('search_distance', 6.0) 
        self.declare_parameter('advance_distance', 0.5) 
        
        self.declare_parameter('centering_threshold_x', 0.1) 
        self.declare_parameter('centering_threshold_y', 0.1)
        self.declare_parameter('default_fly_distance', 5.0)
        
        self.search_velocity = self.get_parameter('search_velocity').value
        self.centering_velocity = self.get_parameter('centering_velocity').value
        self.fly_velocity = self.get_parameter('fly_velocity').value
        self.search_distance = self.get_parameter('search_distance').value
        self.advance_distance = self.get_parameter('advance_distance').value
        self.centering_threshold_x = self.get_parameter('centering_threshold_x').value
        self.centering_threshold_y = self.get_parameter('centering_threshold_y').value
        self.default_fly_distance = self.get_parameter('default_fly_distance').value
        
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
        
        # ==========================================
        # COMMUNICATIONS
        # ==========================================
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', qos_profile)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        
        self.mode_switch_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.error_sub = self.create_subscription(Vector3, '/aruco_error', self.error_callback, 10)
        
        self.current_vel_cmd = TwistStamped()
        
        self.create_timer(0.1, self.mission_loop)
        self.create_timer(0.05, self.velocity_command_loop)
        
        self.create_timer(2.0, self.start_mission)
        self.mission_started = False
        self.get_logger().info(f'=== Pattern Mission Ready (0->{self.search_distance}m->0) ===')

    def error_callback(self, msg):
        self.latest_error = msg
        self.last_detection_time = time.time()

    def is_target_visible(self):
        if self.last_detection_time == 0.0: return False
        return (time.time() - self.last_detection_time) < self.DETECTION_TIMEOUT

    def start_mission(self):
        if not self.mission_started:
            self.mission_started = True
            self.state = MissionState.SEARCHING_RIGHT
            self.move_start_time = time.time()
            self.get_logger().info('Mission Started: Searching Right...')

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
            self.do_advancing(next_state=MissionState.SEARCHING_RIGHT)
            
        # --- CENTERING & FLYING ---
        elif self.state == MissionState.CENTERING_X:
            self.do_centering_x()
        elif self.state == MissionState.CENTERING_Y:
            self.do_centering_y()
        elif self.state == MissionState.FLY_THROUGH:
            self.do_fly_through()
        elif self.state == MissionState.LANDING:
            self.do_landing()

    def do_lateral_move(self, direction, distance, next_state):
        # Even when returning to center, if we see the gate, WE STOP!
        if self.is_target_visible():
            self.get_logger().info(f'Target Found while {self.state.name}! Centering...')
            self.stop_movement()
            self.state = MissionState.CENTERING_X
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
            # If lost, we default back to searching right to reset the loop
            self.state = MissionState.SEARCHING_RIGHT 
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