"""
Mission 1 - Gate Passing with ArUco Detection

This package contains the FSM-based mission for passing through a gate
using ArUco marker detection for centering and alignment.

Modules:
    - FSM: Finite State Machine base classes
    - states: Mission state definitions (Searching, Aligning, Centering, etc.)
    - tools_mission1: Navigation and ArUco detection utilities
    - master_mission1: Main mission node
    - aruco_detector: ArUco marker detection node
"""

from .FSM import State
from .states import Searching, Aligning, Centering, Advancing, Landing
