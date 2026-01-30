#!/usr/bin/env python3
"""
States for the ArUco Gate Passing Mission

Simplified State Machine Flow:
1. Searching: No markers visible - search mode 
2. Aligning: 1-3 markers visible - move to bring all markers into view
3. Centering: 4 markers visible - center on gate
4. Advancing: Centered - move forward through gate
5. Landing: Crossed gate - land

"""

from FSM import State


class Searching(State):
    """
    State: SEARCHING (0 markers visible)
    
    No markers detected - drone searches by moving/rotating.
    
    Events:
        - 'markers_detected': At least one marker found -> Aligning
    """
    
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('markers_detected'):
            return Aligning
        return Searching


class Aligning(State):
    """
    State: ALIGNING (1-3 markers visible)
    
    Partial view of gate - move to bring all 4 markers into view.
    Movement direction is inferred from which markers are visible.
    
    Events:
        - 'all_markers_visible': All 4 markers detected -> Centering
        - 'markers_lost': Lost all markers -> Searching
    """
    
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('all_markers_visible'):
            return Centering
        if self.tail('markers_lost'):
            return Searching
        return Aligning


class Centering(State):
    """
    State: CENTERING (4 markers visible)
    
    All markers visible - calculate center and align drone.
    Only correct X and Y, no forward movement yet.
    
    Events:
        - 'centered': Drone is aligned with gate center -> Advancing
        - 'markers_lost': Lost some markers -> Aligning
        - 'all_markers_lost': Lost all markers -> Searching
    """
    
    def __init__(self, name: str = "") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('centered'):
            return Advancing
        if self.tail('markers_lost'):
            return Aligning
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