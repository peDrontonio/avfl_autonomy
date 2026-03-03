#!/usr/bin/python3

from typing import List

class State:

    __allowedOperations = [-1, 0, 1, 2, 3, 4, 5]
    __eventList = []


    def __init__(self, name="") -> None:
        self.name = name # todo: is this really useful?
        

    def __repr__(self) -> str:
        return 'none' 
    
    # state machine operations

    def add(self, data : str) -> None:
        # add the data (str) element to event list
        State.__eventOp(0, data)
    
    def remove(self, data : str) -> None:
        # remove the data (str) element to event list
        State.__eventOp(1, data)
    
    def contains(self, data : str) -> bool:
        # checks if data (str) element is contained within event list
        return State.__eventOp(2, data )

    def pop(self) -> None:
        # pops the last element from event list
        State.__eventOp(3)
    
    def clear(self) -> None:
        # clears all event list array
        State.__eventOp(4)

    def tail(self, data : str) -> bool:
        # check the last element of the event list
        return State.__eventOp(5, data )
    

    def getAll(self) -> List[str]:
        # return the event list as an array of str
        return State.__eventOp(-1)
    
    def updateEvent(self):
        # responsible for updating the states of the machine instance.
        new_class = self.event()
        if new_class is None:
            print("FSM finalizada")
            return
        # guard: if event() returned an instance (no transition), stay in current state
        if not isinstance(new_class, type):
            return
        self.__class__ = new_class
            
    @classmethod
    def __eq__(cls,comparison):
        # comparison class method that is called when compared with another element
        return cls.__name__ == comparison
    

    @staticmethod
    def __eventOp(operation : int, data :str = ""):
        '''
        TODO: does it need to be static? 
        Handles the event list operation and it's layed like:\n
            -1 -> return an array with all the events \n
            0 -> add operation; param: str (event)\n
            1 -> remove operation; param: str (event)\n
            2 -> checks if event (str) is in event list\n
            3 -> pop operation; para: str (event)\n
            4 -> clear operation; clear the entire array\n
            5 -> tail operation; checks if data is the last element of the list\n 

            >>> SomeState.eventOp(0, "read_all_bases") # adds the event\
                            read_all_bases to the event list
        '''

        if operation not in State.__allowedOperations:
            raise Exception("Operation not found")
        
        
        if operation == 0: # add
            State.__eventList.append(data)

        elif operation == 1: # remove
            if data in State.__eventList:
                State.__eventList.remove(data)
            
            else:
                print("Event not in list")
        
        elif operation == 2: # check if in list
            if data in State.__eventList:
                return True
            
            return False

        elif operation == 3: # pop
            if len(State.__eventList) > 0:
                State.__eventList.pop()
            else:
                print("Event list empty")

        elif operation == 4: # clear all
            State.__eventList = []

        elif operation == 5: # tail
            if data == State.__eventList[-1]:
                return True
            return False
            
        elif operation == -1:
            return State.__eventList

    @classmethod
    def __str__(cls) -> str:
        # returns the name of the state by getting its name.
        return cls.__name__
    
    def getName(self):
        return self.name