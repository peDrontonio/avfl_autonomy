from FSM import State
import colorful as cf

class Takeoff(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # transition to goingtobase when takeoff is complete
        if self.tail('takeoff_complete'):
            return GoingToBase
        return self

class GoingToBase(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # transition to search_base when reached base area
        if self.tail('reached_base_area'):
            return Search_Base
        return self
    
class Search_Base(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # transition to descend_and_centralize when base is found
        if self.tail('base_found'):
            return Descend_and_Centralize
        
        if self.tail('base_lost'):
            return Scan_for_Base
        
        # safety: return to base after max retries
        if self.tail('rtl'):
            return ReturnBase
        return self

class Scan_for_Base(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # transition to descend_and_centralize when base is centered
        if self.tail('centered'):
            return Descend_and_Centralize
        
        # safety: return to base after max retries
        if self.tail('rtl'):
            return ReturnBase
        return self

class Descend_and_Centralize(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # transition to landing when centered at low altitude
        if self.tail('centered_low'):
            return Landing
        
        # if base lost during descent, go back to scanning
        if self.tail('base_lost'):
            return Scan_for_Base
        
        # safety: return to base after max retries
        if self.tail('rtl'):
            return ReturnBase
        return self

class Landing(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # mission complete
        if self.tail('finished'):
            return None
        return self

class ReturnBase(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # rtl activated - mission ended for safety
        if self.tail('rtl_activated'):
            return None
        return self