#!/usr/bin/env python3
"""
Simple square pattern mission node.

Author:AVFL Autonomy Brazilians Team - Pedro
Date: January 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped

import time
from enum import Enum


class MissionState(Enum):
    IDLE = 0
    MOVING_NORTH = 1
    MOVING_EAST = 2
    MOVING_SOUTH = 3
    MOVING_WEST = 4
    COMPLETED = 5


class SimpleMissionNode(Node):
    def __init__(self):
        super().__init__('simple_mission_node')
        
        # Parameters
        self.declare_parameter('square_side', 5.0)
        self.declare_parameter('velocity', 1.0)
        
        self.square_side = self.get_parameter('square_side').value
        self.velocity = self.get_parameter('velocity').valueSize of the square side in meters
        
        # Mission state
        self.state = MissionState.IDLE
        self.move_start_time = None
        self.move_duration = self.square_side / self.velocity
        
        # QoS profile for ArduPilot topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(
            TwistStamped, 
            '/ap/cmd_vel', 
            qos_profile
        )
        
        # Current velocity command
        self.current_vel_cmd = TwistStamped()
        
        # Main mission timer
        self.mission_timer = self.create_timer(0.1, self.mission_loop)
        
        # Timer to send velocity commands continuously (20 Hz)
        self.vel_timer = self.create_timer(0.05, self.velocity_command_loop)
        
        self.get_logger().info('=== Simple Mission Node Started ===')
        self.get_logger().info(f'Square side: {self.square_side}m')
        self.get_logger().info(f'Velocity: {self.velocity}m/s')
        self.get_logger().info(f'Duration per segment: {self.move_duration}s')
        
        # Start mission after small delay
        self.create_timer(2.0, self.start_mission)
        self.mission_started = False

    def start_mission(self):
        """Start the mission."""
        if not self.mission_started:
            self.mission_started = True
            self.state = MissionState.MOVING_NORTH
            self.get_logger().info('Starting square pattern mission!')

    def velocity_command_loop(self):
        """Loop to send velocity commands continuously."""
        if self.state in [MissionState.MOVING_NORTH, MissionState.MOVING_EAST, 
                          MissionState.MOVING_SOUTH, MissionState.MOVING_WEST]:
            self.current_vel_cmd.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(self.current_vel_cmd)

    def set_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """Set the current velocity command."""
        self.current_vel_cmd = TwistStamped()
        self.current_vel_cmd.header.stamp = self.get_clock().now().to_msg()
        self.current_vel_cmd.header.frame_id = 'base_link'
        self.current_vel_cmd.twist.linear.x = float(vx)
        self.current_vel_cmd.twist.linear.y = float(vy)
        self.current_vel_cmd.twist.linear.z = float(vz)
        self.current_vel_cmd.twist.angular.z = float(yaw_rate)

    def stop_movement(self):
        """Stop drone movement."""
        self.set_velocity(0.0, 0.0, 0.0, 0.0)
        # Send stop command multiple times
        for _ in range(5):
            self.current_vel_cmd.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(self.current_vel_cmd)
            time.sleep(0.05)

    def mission_loop(self):
        """Main mission state machine loop."""
        
        if self.state == MissionState.IDLE:
            pass
            
        elif self.state == MissionState.MOVING_NORTH:
            self.do_move_segment('NORTH', MissionState.MOVING_EAST)
            
        elif self.state == MissionState.MOVING_EAST:
            self.do_move_segment('EAST', MissionState.MOVING_SOUTH)
            
        elif self.state == MissionState.MOVING_SOUTH:
            self.do_move_segment('SOUTH', MissionState.MOVING_WEST)
            
        elif self.state == MissionState.MOVING_WEST:
            self.do_move_segment('WEST', MissionState.COMPLETED)
            
        elif self.state == MissionState.COMPLETED:
            self.get_logger().info('Mission completed successfully!')
            self.get_logger().info('Drone is holding position. Use mode_switch to LAND or RTL.')
            self.state = MissionState.IDLE

    def do_move_segment(self, direction, next_state):
        """Execute a movement segment."""
        
        # Initialize movement
        if self.move_start_time is None:
            self.move_start_time = time.time()
            
            # Set velocity based on direction
            # Frame: X = forward, Y = left, Z = up
            if direction == 'NORTH':
                self.set_velocity(self.velocity, 0.0, 0.0)
                self.get_logger().info(f'Moving NORTH for {self.square_side}m')
            elif direction == 'EAST':
                self.set_velocity(0.0, -self.velocity, 0.0)
                self.get_logger().info(f'Moving EAST for {self.square_side}m')
            elif direction == 'SOUTH':
                self.set_velocity(-self.velocity, 0.0, 0.0)
                self.get_logger().info(f'Moving SOUTH for {self.square_side}m')
            elif direction == 'WEST':
                self.set_velocity(0.0, self.velocity, 0.0)
                self.get_logger().info(f'Moving WEST for {self.square_side}m')
        
        # Check if segment is complete (time-based)
        elapsed = time.time() - self.move_start_time
        
        if elapsed >= self.move_duration:
            self.stop_movement()
            self.get_logger().info(f'Segment {direction} completed!')
            time.sleep(1.0)  # Pause between segments
            
            self.move_start_time = None
            self.state = next_state


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleMissionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user')
        node.stop_movement()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
