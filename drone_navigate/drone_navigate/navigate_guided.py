#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped, PoseStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
from drone_navigate.srv import Navigate 
import math
import time

class AVFLAutonomy(Node):
    def __init__(self):
        super().__init__('avfl_autonomy')

        # Create reentrant callback group for service calls
        self.callback_group = ReentrantCallbackGroup()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.current_pos = None
        self.current_yaw = 0.0
        self.ekf_received = False
        self.pose_sub = self.create_subscription(PoseStamped, '/ap/pose/filtered', self.pose_cb, qos)

        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)
        
        # Use callback group for service clients
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors', callback_group=self.callback_group)
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch', callback_group=self.callback_group)

        # Use callback group for service server
        self.srv = self.create_service(Navigate, 'avfl/navigate', self.handle_navigate, callback_group=self.callback_group)

        self.target_pos = None # {x, y, z}
        self.nav_speed = 0.5
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz

        self.get_logger().info("AVFL Autonomy Service Ready. Waiting for EKF...")

    def pose_cb(self, msg):
        if not self.ekf_received:
            self.get_logger().info("EKF Position received. Service ready.")
            self.ekf_received = True
        self.current_pos = msg.pose.position
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def handle_navigate(self, request, response):
        """ The High-Level Autonomy Logic """
        self.get_logger().info(f"Command: Go {request.x}m Fwd, {request.z}m Up. Auto-Arm: {request.auto_arm}")

        if self.current_pos is None:
            response.success = False
            response.message = "No EKF Position fix yet."
            return response

        # --- A. AUTO-ARMING SEQUENCE (Reordered: Guided -> Arm) ---
        if request.auto_arm:
            self.get_logger().info("Initiating Auto-Takeoff...")
            
            # 1. Switch to GUIDED (Mode 4)
            mode_req = ModeSwitch.Request()
            mode_req.mode = 4
            if not self.call_service_sync(self.mode_client, mode_req):
                response.success = False
                response.message = "Failed to switch to GUIDED"
                return response

            # 2. Arm Motors
            arm_req = ArmMotors.Request()
            arm_req.arm = True
            if not self.call_service_sync(self.arm_client, arm_req):
                response.success = False
                response.message = "Failed to ARM motors"
                return response
            
            # 3. Wait for Spin-up
            self.get_logger().info("Armed successfully. Waiting for spin-up...")
            time.sleep(1.0) 

        # --- B. CALCULATE TARGET (Body to Map Frame) ---
        tx, ty, tz = self.current_pos.x, self.current_pos.y, self.current_pos.z

        if request.frame_id == 'body':
            dx = request.x * math.cos(self.current_yaw) - request.y * math.sin(self.current_yaw)
            dy = request.x * math.sin(self.current_yaw) + request.y * math.cos(self.current_yaw)
            tx += dx
            ty += dy
        else:
            tx += request.x
            ty += request.y
        
        tz += request.z 

        # Update State
        self.nav_speed = request.speed if request.speed > 0 else 0.5
        self.target_pos = {'x': tx, 'y': ty, 'z': tz}

        response.success = True
        response.message = "Autonomy Engaged."
        return response

    def call_service_sync(self, client, req):
        """Synchronous service call"""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {client.srv_name} not available")
            return False
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            try:
                result = future.result()
                self.get_logger().info(f"Service call succeeded: {result}")
                return True
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return False
        else:
            self.get_logger().error("Service call timed out")
            return False

    def control_loop(self):
        """ The Pilot Loop """
        if self.target_pos is None or self.current_pos is None:
            return

        err_x = self.target_pos['x'] - self.current_pos.x
        err_y = self.target_pos['y'] - self.current_pos.y
        err_z = self.target_pos['z'] - self.current_pos.z
        dist = math.sqrt(err_x**2 + err_y**2 + err_z**2)

        self.get_logger().info(f"Distance to target: {dist:.2f}m, Target: ({self.target_pos['x']:.2f}, {self.target_pos['y']:.2f}, {self.target_pos['z']:.2f}), Current: ({self.current_pos.x:.2f}, {self.current_pos.y:.2f}, {self.current_pos.z:.2f})", throttle_duration_sec=1.0)

        if dist < 0.15: 
            self.target_pos = None
            self.stop_drone()
            self.get_logger().info("Target Reached. Hovering.")
            return

        vel_x = err_x * 1.0
        vel_y = err_y * 1.0
        vel_z = err_z * 1.0

        speed_xy = math.sqrt(vel_x**2 + vel_y**2)
        if speed_xy > self.nav_speed:
            scale = self.nav_speed / speed_xy
            vel_x *= scale
            vel_y *= scale

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = vel_x
        msg.twist.linear.y = vel_y
        msg.twist.linear.z = vel_z
        self.vel_pub.publish(msg)

    def stop_drone(self):
        self.vel_pub.publish(TwistStamped())

def main(args=None):
    try:
        rclpy.init(args=args)
        node = AVFLAutonomy()
        
        # Use MultiThreadedExecutor to handle service callbacks properly
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()