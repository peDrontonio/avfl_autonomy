from FSM import State
import colorful as cf

class GoingToBase(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Transition to Search_Base when reached base area
        if self.tail('reached_base_area'):
            return Search_Base
        return self
    
class Search_Base(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Transition to Approach_Base when base is consistently detected
        if self.tail('base_found'):
            return Descent
        
        if self.tail('base_lost'):
            return Scan_for_Base
        return self

class Scan_for_Base(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Transition to Approach_Base when base is consistently detected
        if self.tail('centered'):
            return Descent
        return self

class Descent(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Mission complete
        if self.tail('ready_to_land'):
            return Landing
        return self

class Landing(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Mission complete
        if self.tail('finished'):
            return None
        return self