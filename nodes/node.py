from enum import Enum

class STATUS(Enum):
    SUCCESS = 0
    RUNNING = 1
    FAILURE = 2
    INVALID = 3

# Lowest-level class that makes up the building block of every other behavior else
class Behavior:

    def __init__(self):
        self.status = STATUS.INVALID

    # Initialises the behavior
    def _on_initialise(self, blackboard: dict):
        pass

    # Cleanup the behavior
    def _on_terminate(self, status: STATUS, blackboard: dict):
        pass

    # Update the behavior
    def _update(self, blackboard: dict):
        pass

    def get_status(self):
        return self.status

    # Wrapper to update the behavior while adhering to the behavior tree contract
    def tick(self, blackboard: dict):
        # If behavior is not running, initialise it
        if self.status != STATUS.RUNNING:
            self._on_initialise(blackboard)
        self.status = self._update(blackboard)
        # If behavior is not running, terminate it
        if self.status != STATUS.RUNNING:
            self._on_terminate(self.status, blackboard)
        return self.status

# A behavior with a child behavior. Can be considered a wrapper for the child behavior.
class Decorator(Behavior):

    def __init__(self, child: Behavior):
        super().__init__()
        self.child = child

# Repeatedly executes the child behavior n times.
class Repeat(Decorator):

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

# A behavior with multiple child behaviors.
class Composite(Behavior):

    current_child = 0

    def __init__(self):
        super().__init__()
        self.children = []

    def addChild(self, child: Behavior):
        self.children.append(child)
        return self
    
    def removeChild(self, child: Behavior):
        self.children.remove(child)
        return self
    
    def clearChildren(self):
        self.children.clear()
        return self
    

# A sequence executes each of its child behaviors in order until all of the
# children have executed successfully or until one of its child behaviors fail.
class Sequence(Composite):

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

# A fallback executes each of its child behaviors in order until it
# finds a child that either succeeds or that returns a RUNNING status.
class Fallback(Composite):

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

# Conditions must return success to reach actions.
class Filter(Sequence):

    def __init__(self):
        super().__init__()
    
    def addCondition(self, condition: Behavior):
        self.children.insert(0, condition)
        return self
    
    def addAction(self, action: Behavior):
        self.children.append(action)
        return self

