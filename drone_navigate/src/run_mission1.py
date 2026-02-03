#!/usr/bin/env python3
"""
Wrapper script to run Mission 1 - Gate Passing with ArUco Detection.

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

# Now import and run the mission
from mission1.master_mission1 import main

if __name__ == '__main__':
    main()
