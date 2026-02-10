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
        # Transition to Landing when base is found
        if self.tail('base_found'):
            return Landing
        
        if self.tail('base_lost'):
            return Scan_for_Base
        return self

class Scan_for_Base(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Transition to Landing when base is centered
        if self.tail('centered'):
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