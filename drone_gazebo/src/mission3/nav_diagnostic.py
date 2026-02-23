#!/usr/bin/env python3
"""
Quick diagnostic tool for Mission 2 navigation issues
"""
import rclpy
from rclpy.node import Node
from drone_navigate.srv import NavigateGlobal, GetTelemetry

class NavDiagnostic(Node):
    def __init__(self):
        super().__init__('nav_diagnostic')
        self.nav_client = self.create_client(NavigateGlobal, 'avfl/navigate_global')
        self.telem_client = self.create_client(GetTelemetry, 'avfl/get_telemetry')
        
        self.get_logger().info("Navigation Diagnostic Tool")
        self.run_diagnostic()
        
    def run_diagnostic(self):
        # Check service availability
        self.get_logger().info("Checking navigate_global service...")
        if self.nav_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("✓ navigate_global service is available")
        else:
            self.get_logger().error("✗ navigate_global service NOT available")
            return
            
        self.get_logger().info("Checking get_telemetry service...")
        if self.telem_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("✓ get_telemetry service is available")
        else:
            self.get_logger().error("✗ get_telemetry service NOT available")
            return
            
        # Test service call
        self.get_logger().info("\nTesting navigate_global service call...")
        req = NavigateGlobal.Request()
        req.lat = -35.3632621
        req.lon = 149.1654
        req.z = 10.0
        req.yaw = 0.0
        req.yaw_rate = 0.0
        req.speed = 0.5
        req.frame_id = 'map'
        req.auto_arm = False
        
        future = self.nav_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            result = future.result()
            if result.success:
                self.get_logger().info(f"✓ Service call successful: {result.message}")
            else:
                self.get_logger().error(f"✗ Service call failed: {result.message}")
        else:
            self.get_logger().error("✗ Service call timed out")

def main():
    rclpy.init()
    node = NavDiagnostic()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
