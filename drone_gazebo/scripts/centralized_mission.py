#!/usr/bin/env python3
"""
Centralized Mission Node - ArUco-based arch traversal.
Separated Control Logic: Listens to /aruco_error from an external detector node.
Feature: Requires detector to find 4 markers.
Feature: Flies 50% further than measured distance.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped, Vector3
from ardupilot_msgs.srv import ModeSwitch
import numpy as np
import time
from enum import Enum


class MissionState(Enum):
    IDLE = 0
    SEARCHING_RIGHT = 1
    SEARCHING_LEFT = 2
    ADVANCING = 3
    CENTERING_X = 4
    CENTERING_Y = 5
    FLY_THROUGH = 6
    CROSSING = 7
    LANDING = 8
    COMPLETED = 9


class CentralizedMissionNode(Node):
    def __init__(self):
        super().__init__('centralized_mission_node')
        
        # ==========================================
        # PARAMETERS
        # ==========================================
        self.declare_parameter('search_velocity', 1.0)
        self.declare_parameter('centering_velocity', 0.3)
        self.declare_parameter('fly_velocity', 1.0)
        self.declare_parameter('search_distance', 3.0)
        self.declare_parameter('centering_threshold_x', 0.1) 
        self.declare_parameter('centering_threshold_y', 0.1)
        self.declare_parameter('default_fly_distance', 5.0) # Fallback
        
        self.search_velocity = self.get_parameter('search_velocity').value
        self.centering_velocity = self.get_parameter('centering_velocity').value
        self.fly_velocity = self.get_parameter('fly_velocity').value
        self.search_distance = self.get_parameter('search_distance').value
        self.centering_threshold_x = self.get_parameter('centering_threshold_x').value
        self.centering_threshold_y = self.get_parameter('centering_threshold_y').value
        self.default_fly_distance = self.get_parameter('default_fly_distance').value
        
        # ==========================================
        # STATE VARIABLES
        # ==========================================
        self.state = MissionState.IDLE
        self.prev_state = MissionState.IDLE
        self.move_start_time = None
        self.distance_traveled = 0.0
        self.search_count = 0
        self.advance_distance = 0.5
        self.calculated_fly_distance = 0.0 # Stores (1.5 * detected_z)
        
        # Sensor Data State
        self.latest_error = Vector3()
        self.last_detection_time = 0.0
        self.DETECTION_TIMEOUT = 0.5 
        
        self.log_counter = 0
        self.LOG_INTERVAL = 10 
        
        # ==========================================
        # COMMUNICATIONS
        # ==========================================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', qos_profile)
        self.mode_switch_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.error_sub = self.create_subscription(Vector3, '/aruco_error', self.error_callback, 10)
        
        self.current_vel_cmd = TwistStamped()
        
        self.mission_timer = self.create_timer(0.1, self.mission_loop)
        self.vel_timer = self.create_timer(0.05, self.velocity_command_loop)
        
        self.get_logger().info('=== Mission Node Started (4-Marker Lock + 50% Distance) ===')
        self.create_timer(2.0, self.start_mission)
        self.mission_started = False

    # ==========================================
    # LOGIC
    # ==========================================
    def error_callback(self, msg):
        self.latest_error = msg
        self.last_detection_time = time.time()

    def is_target_visible(self):
        # Returns False if Detector stops publishing (because < 4 markers)
        if self.last_detection_time == 0.0:
            return False
        return (time.time() - self.last_detection_time) < self.DETECTION_TIMEOUT

    def start_mission(self):
        if not self.mission_started:
            self.mission_started = True
            self.state = MissionState.SEARCHING_RIGHT
            self.move_start_time = time.time()
            self.get_logger().info('Starting mission! Searching right...')

    def velocity_command_loop(self):
        active_states = [
            MissionState.SEARCHING_RIGHT, MissionState.SEARCHING_LEFT,
            MissionState.ADVANCING, MissionState.CENTERING_X,
            MissionState.CENTERING_Y, MissionState.FLY_THROUGH,
            MissionState.CROSSING
        ]
        if self.state in active_states:
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
        # Force publish stop
        for _ in range(3):
            self.current_vel_cmd.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(self.current_vel_cmd)
            time.sleep(0.02)

    # ==========================================
    # STATE MACHINE
    # ==========================================
    def mission_loop(self):
        if self.state != self.prev_state:
            self.get_logger().info(f'>>> STATE: {self.state.name}')
            self.prev_state = self.state
        
        if self.state == MissionState.SEARCHING_LEFT:
            self.do_search(direction='left')
        elif self.state == MissionState.SEARCHING_RIGHT:
            self.do_search(direction='right')
        elif self.state == MissionState.ADVANCING:
            self.do_advancing()
        elif self.state == MissionState.CENTERING_X:
            self.do_centering_x()
        elif self.state == MissionState.CENTERING_Y:
            self.do_centering_y()
        elif self.state == MissionState.FLY_THROUGH:
            self.do_fly_through()
        elif self.state == MissionState.LANDING:
            self.do_landing()

    def do_search(self, direction='right'):
        # If detector sees < 4 markers, it stops publishing.
        # is_target_visible() becomes False.
        # We continue searching.
        if self.is_target_visible():
            self.get_logger().info('>>> GATE FOUND (4 Markers Locked)!')
            self.stop_movement()
            self.state = MissionState.CENTERING_X
            self.move_start_time = None
            return
        
        if self.move_start_time is None:
            self.move_start_time = time.time()
            self.distance_traveled = 0.0
        
        elapsed = time.time() - self.move_start_time
        self.distance_traveled = elapsed * self.search_velocity
        
        vy = self.search_velocity if direction == 'right' else -self.search_velocity
        self.set_velocity(0.0, vy, 0.0)
        
        if self.distance_traveled >= self.search_distance:
            self.stop_movement()
            self.search_count += 1
            if self.search_count >= 2:
                self.state = MissionState.ADVANCING
                self.search_count = 0
            else:
                self.state = MissionState.SEARCHING_LEFT if direction == 'right' else MissionState.SEARCHING_RIGHT
            self.move_start_time = None

    def do_advancing(self):
        if self.is_target_visible():
            self.stop_movement()
            self.state = MissionState.CENTERING_X
            return
            
        if self.move_start_time is None:
            self.move_start_time = time.time()
            
        elapsed = time.time() - self.move_start_time
        if (elapsed * self.centering_velocity) >= self.advance_distance:
            self.stop_movement()
            self.state = MissionState.SEARCHING_RIGHT
            self.move_start_time = None
            return
            
        self.set_velocity(self.centering_velocity, 0.0, 0.0)

    def do_centering_x(self):
        if not self.is_target_visible():
            self.get_logger().warn('Lost Lock (Markers < 4). Resuming Search...')
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
            
            # --- CALCULATE DISTANCE (Measured * 1.5) ---
            measured_z = self.latest_error.z
            if measured_z > 0.1:
                self.calculated_fly_distance = measured_z * 1.5
                self.get_logger().info(f'Measured: {measured_z:.2f}m. Target Fly Distance: {self.calculated_fly_distance:.2f}m')
            else:
                self.calculated_fly_distance = self.default_fly_distance
                self.get_logger().warn('Invalid distance. Using default.')
                
            self.state = MissionState.FLY_THROUGH
            self.move_start_time = None
            return

        error_x = self.latest_error.x
        vy = np.clip(0.5 * error_x, -0.1, 0.1)
        vz = -np.clip(0.8 * error_y, -self.centering_velocity, self.centering_velocity)
        self.set_velocity(0.0, vy, vz)

    def do_fly_through(self):
        if self.move_start_time is None:
            self.move_start_time = time.time()
            self.get_logger().info(f'>>> FLYING {self.calculated_fly_distance:.2f} meters')
            
        elapsed = time.time() - self.move_start_time
        distance = elapsed * self.fly_velocity
        
        if distance >= self.calculated_fly_distance:
            self.stop_movement()
            self.state = MissionState.LANDING
            return
            
        self.set_velocity(self.fly_velocity, 0.0, 0.0)

    def do_landing(self):
        self.stop_movement()
        if not self.mode_switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Mode switch service missing. Land manually.')
            self.state = MissionState.COMPLETED
            return
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