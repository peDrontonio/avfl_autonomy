#!/usr/bin/env python3
"""
States for the ArUco Gate Passing Mission

Simplified State Machine:
1. Takeoff: Arm and take off
2. Searching: Lateral search until 2+ markers found
3. Aligning: Continuous movement until all 4 markers visible
4. Centering: Center loosely, approach to 3m, center tightly
5. Advancing: Fly through the gate
6. Landing: Land

Gate Layout (ArUco markers 0-3):
    2 --- 3
    |     |
    0 --- 1
"""

from FSM import State


class Takeoff(State):
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('takeoff_complete'):
            return Searching
        return Takeoff


class Searching(State):
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('markers_found'):
            return Aligning
        return Searching


class Aligning(State):
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('all_visible'):
            return Centering
        if self.tail('markers_lost'):
            return Searching
        return Aligning


class Centering(State):
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('ready_to_pass'):
            return Advancing
        if self.tail('markers_lost'):
            return Searching
        return Centering


class Advancing(State):
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('crossed'):
            return Landing
        return Advancing


class Landing(State):
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('landed'):
            return None
        return Landing