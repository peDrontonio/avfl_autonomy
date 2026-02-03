#!/usr/bin/env python3
"""
Wrapper script to run the ArUco Detector node for Mission 1.

This script is installed as an executable and properly sets up the Python path
to import the mission1 modules.
"""

import sys
import os

# Add the mission1 directory to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
mission1_dir = os.path.join(script_dir, 'mission1')
if mission1_dir not in sys.path:
    sys.path.insert(0, mission1_dir)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Now import and run the aruco detector
from mission1.aruco_detector import main

if __name__ == '__main__':
    main()
