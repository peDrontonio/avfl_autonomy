#!/usr/bin/env python3
"""
Mission 3 Launch Script
Starts both the YOLO detector and mission controller
"""
import subprocess
import sys
import os

def main():
    print("=" * 60)
    print("Mission 3: Mobile Base Landing with GPS Navigation (YOLO)")
    print("=" * 60)
    print("\nStarting YOLO Base Detector and Mission Controller...")
    print("\nUsage:")
    print("  Set base GPS coordinates as parameters:")
    print("  ros2 run ... --ros-args -p base_lat:=<LAT> -p base_lon:=<LON> -p base_alt:=<ALT>")
    print("\n" + "=" * 60)
    
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Start YOLO detector in background
    yolo_cmd = ["python3", os.path.join(script_dir, "yolo_base_detector.py")]
    
    # Start mission controller
    mission_cmd = ["python3", os.path.join(script_dir, "master_mission3.py")]
    
    try:
        print("\n[1/2] Starting YOLO Base Detector...")
        yolo_process = subprocess.Popen(yolo_cmd)
        
        print("[2/2] Starting Mission 3 Controller...")
        mission_process = subprocess.Popen(mission_cmd)
        
        print("\n✓ All nodes started successfully!")
        print("\nTo start the mission, call:")
        print("  ros2 service call /start_mission3 std_srvs/srv/Trigger")
        print("\nPress Ctrl+C to stop all nodes...")
        
        # Wait for processes
        yolo_process.wait()
        mission_process.wait()
        
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        if yolo_process:
            yolo_process.terminate()
        if mission_process:
            mission_process.terminate()
        sys.exit(0)

if __name__ == '__main__':
    main()
