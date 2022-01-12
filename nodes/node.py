#!/usr/bin/env python3

from enum import Enum

class STATUS(Enum):
    """
    All possible return status' of a behavior.
    """

    SUCCESS = 0
    RUNNING = 1
    FAILURE = 2
    INVALID = 3


class Behavior:
    """
    Lowest-level class that makes up the building block of every other class.
    """

    def __init__(self):
        self.status = STATUS.INVALID

    def _on_initialise(self, blackboard: dict):
        """
        Setup the behavior prior to being run for the first time
        since being stopped.
        """
        pass

    def _on_terminate(self, status: STATUS, blackboard: dict):
        """
        Clean up the behavior after being stopped.
        """
        pass

    def _update(self, blackboard: dict):
        """
        Update the behavior.
        """
        pass

    def get_status(self):
        """
        Gets the behavior's status from the most recent tick.
        """
        return self.status

    def tick(self, blackboard: dict):
        """
        A wrapper to update the behavior while adhering to the behavior tree contract.
        Returns the status from being updated.
        """

        # If behavior is not running, initialise it
        if self.status != STATUS.RUNNING:
            self._on_initialise(blackboard)
        self.status = self._update(blackboard)
        # If behavior is not running, terminate it
        if self.status != STATUS.RUNNING:
            self._on_terminate(self.status, blackboard)
        return self.status


class Decorator(Behavior):
    """
    A behavior with a child behavior. Can be considered a wrapper for the child behavior.
    """

    def __init__(self, child: Behavior):
        super().__init__()
        self.child = child


class Inverter(Decorator):
    """
    Inverts the result unless RUNNING is returned.
    """

    def __init__(self, child: Behavior):
        super().__init__(child)
    
    def _update(self, blackboard: dict):
        status = self.child.tick(blackboard)
        if status == STATUS.SUCCESS:
            return STATUS.FAILURE
        elif status == STATUS.FAILURE:
            return STATUS.SUCCESS
        return status


class ForceSuccess(Decorator):
    """
    Forces the result to be SUCCESS after a single tick.
    """
    
    def __init__(self, child: Behavior):
        super().__init__(child)
    
    def _update(self, blackboard: dict):
        self.child.tick(blackboard)
        return STATUS.SUCCESS


class RepeatUntilSuccess(Decorator):
    """
    Repeatedly ticks the child until it returns SUCCESS.
    """

    def __init__(self, child: Behavior):
        super().__init__(child)

    def _update(self, blackboard: dict):
        while True:
            status = self.child.tick(blackboard)
            if status == STATUS.SUCCESS:
                return STATUS.SUCCESS
        # Unexpected exit
        return STATUS.INVALID


class Repeat(Decorator):
    """
    Repeatedly executes the child behavior n times.
    """

    repeats_counter = 0

    def __init__(self, child: Behavior, repeats: int):
        super().__init__(child)
        self.repeats = repeats

    def _on_initialise(self, blackboard: dict):
        super()._on_initialise(blackboard)
        self.repeats_counter = 0

    def _update(self, blackboard: dict):
        while True:
            status = self.child.tick(blackboard)
            if status == STATUS.RUNNING:
                return STATUS.RUNNING
            if status == STATUS.FAILURE:
                return STATUS.FAILURE
            if status == STATUS.SUCCESS:
                self.repeats_counter += 1
                if self.repeats_counter >= self.repeats:
                    return STATUS.SUCCESS
        # Unexpected exit
        return STATUS.INVALID


class Composite(Behavior):
    """
    A behavior with multiple child behaviors.
    """

    current_child = 0

    def __init__(self):
        super().__init__()
        self.children = []

    def addChild(self, child: Behavior):
        """
        Adds a new child behavior.
        """

        self.children.append(child)
        return self
    
    def removeChild(self, child: Behavior):
        """
        Removes a child behavior.
        """

        self.children.remove(child)
        return self
    
    def clearChildren(self):
        """
        Clear all child behaviors.
        """

        self.children.clear()
        return self
    

class Sequence(Composite):
    """
    A sequence executes each of its child behaviors in order until all of the
    children have executed successfully or until one of its child behaviors fail.
    """

    def __init__(self):
        super().__init__()

    def _on_initialise(self, blackboard: dict):
        super()._on_initialise(blackboard)
        self.current_child = 0
    
    def _update(self, blackboard: dict):
        while True:
            status = self.children[self.current_child].tick(blackboard)
            if status != STATUS.SUCCESS:
                return status
            self.current_child += 1
            if self.current_child >= len(self.children):
                return STATUS.SUCCESS
        # Unexpected exit
        return STATUS.INVALID


class Fallback(Composite):
    """
    A fallback executes each of its child behaviors in order until it
    finds a child that either succeeds or that returns a RUNNING status.
    """

    def __init__(self):
        super().__init__()
    
    def _on_initialise(self, blackboard: dict):
        super()._on_initialise(blackboard)
        self.current_child = 0

    def _update(self, blackboard: dict):
        # Keep going until a child behavior is running/succeeds.
        while True:
            status = self.children[self.current_child].tick(blackboard)
            if status == STATUS.SUCCESS or status == STATUS.RUNNING:
                return status
            self.current_child += 1
            if self.current_child >= len(self.children):
                return STATUS.FAILURE
        # Unexpected exit
        return STATUS.INVALID


class Filter(Sequence):
    """
    Conditions must return success to reach actions.
    """

    def __init__(self):
        super().__init__()
    
    def addCondition(self, condition: Behavior):
        """
        Adds a condition to the front of the array.
        """
        
        self.children.insert(0, condition)
        return self
    
    def addAction(self, action: Behavior):
        """
        Adds an action to the end of the array.
        """

        self.children.append(action)
        return self

