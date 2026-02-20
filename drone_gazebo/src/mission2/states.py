from FSM import State
import colorful as cf

class Takeoff(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Transition to GoingToBase when takeoff is complete
        if self.tail('takeoff_complete'):
            return GoingToBase
        return self

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
        # Transition to Descend_and_Centralize when base is found
        if self.tail('base_found'):
            return Descend_and_Centralize
        
        if self.tail('base_lost'):
            return Scan_for_Base
        
        # Safety: return to base after max retries
        if self.tail('rtl'):
            return ReturnBase
        return self

class Scan_for_Base(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Transition to Descend_and_Centralize when base is centered
        if self.tail('centered'):
            return Descend_and_Centralize
        
        # Safety: return to base after max retries
        if self.tail('rtl'):
            return ReturnBase
        return self

class Descend_and_Centralize(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Transition to Landing when centered at low altitude
        if self.tail('centered_low'):
            return Landing
        
        # If base lost during descent, go back to scanning
        if self.tail('base_lost'):
            return Scan_for_Base
        
        # Safety: return to base after max retries
        if self.tail('rtl'):
            return ReturnBase
        return self

class Landing(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # Mission complete
        if self.tail('finished'):
            return None
        return self

class ReturnBase(State):
    def __init__(self, name="") -> None:
        super().__init__(name)
    
    def event(self):
        # RTL acionado - missão encerrada por segurança
        if self.tail('rtl_activated'):
            return None
        return self