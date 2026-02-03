#!/usr/bin/env python3
"""
States for the ArUco Gate Passing Mission

State Machine Flow:
1. Searching: No/1 marker visible - search mode 
2. PartialCentering: 2-3 markers visible - center on partial, adjust to find rest
3. Aligning: After partial centering, adjust Y/Z based on marker layout
4. Centering: 4 markers visible - center on gate
5. Advancing: Centered - move forward through gate
6. Landing: Crossed gate - land

Gate Layout (ArUco markers 0-3):
    2 --- 3
    |     |
    0 --- 1

Alignment Logic (when 2 markers detected):
- {0, 1} (bottom pair) -> Move UP
- {2, 3} (top pair) -> Move DOWN
- {1, 3} (right pair) -> Move LEFT
- {0, 2} (left pair) -> Move RIGHT
"""

from FSM import State


class Searching(State):
    """
    State: SEARCHING (0-1 markers visible)
    
    No/insufficient markers detected - drone searches by moving laterally.
    Requires at least 2 markers to transition.
    
    Events:
        - 'partial_detected': 2-3 markers found -> PartialCentering
        - 'all_markers_visible': All 4 markers found -> Centering
    """
    
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('all_markers_visible'):
            return Centering
        if self.tail('partial_detected'):
            return PartialCentering
        return Searching


class PartialCentering(State):
    """
    State: PARTIAL_CENTERING (2-3 markers visible)
    
    Partial view of gate - center on visible markers first,
    then transition to Aligning to find remaining markers.
    
    Events:
        - 'partial_centered': Centered on partial markers -> Aligning
        - 'all_markers_visible': All 4 markers detected -> Centering
        - 'markers_lost': Lost markers (< 2) -> Searching
    """
    
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('all_markers_visible'):
            return Centering
        if self.tail('partial_centered'):
            return Aligning
        if self.tail('markers_lost'):
            return Searching
        return PartialCentering


class Aligning(State):
    """
    State: ALIGNING (2-3 markers visible, partially centered)
    
    After partial centering - adjust Y or Z based on which markers
    are visible to bring all 4 markers into view.
    
    Events:
        - 'all_markers_visible': All 4 markers detected -> Centering
        - 'markers_lost': Lost all markers -> Searching
        - 'partial_lost': Lost some but still have 2+ -> PartialCentering
    """
    
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('all_markers_visible'):
            return Centering
        if self.tail('markers_lost'):
            return Searching
        if self.tail('partial_lost'):
            return PartialCentering
        return Aligning


class Centering(State):
    """
    State: CENTERING (4 markers visible)
    
    All markers visible - calculate center and align drone.
    Only correct X and Y, no forward movement yet.
    
    Events:
        - 'centered': Drone is aligned with gate center -> Advancing
        - 'markers_lost': Lost some markers -> PartialCentering
        - 'all_markers_lost': Lost all markers -> Searching
    """
    
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('centered'):
            return Advancing
        if self.tail('markers_lost'):
            return PartialCentering
        if self.tail('all_markers_lost'):
            return Searching
        return Centering


class Advancing(State):
    """
    State: ADVANCING (centered, moving forward)
    
    Drone is centered - advance through the gate.
    
    Events:
        - 'crossed': Successfully crossed the gate -> Landing
        - 'lost_alignment': Lost centering -> Centering
    """
    
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('crossed'):
            return Landing
        if self.tail('lost_alignment'):
            return Centering
        return Advancing


class Landing(State):
    """
    State: LANDING
    
    Gate crossed - land the drone.
    
    Events:
        - 'landed': Drone has landed (FSM ends)
    """
    
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('landed'):
            return None  # FSM finished
        return Landing