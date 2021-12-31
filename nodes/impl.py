from nodes.node import *

# Checks if the rover has a destination to move to.
class HasDestination(Behavior):

    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        print('has_dest')
        if blackboard['destination'] is not None:
            return STATUS.SUCCESS
        return STATUS.FAILURE

# Asks for a destination.
class GetDestination(Behavior):

    def __init__(self):
        super().__init__()
    
    def _on_terminate(self, status: STATUS, blackboard: dict):
        super()._on_terminate(status, blackboard)

    def _ask_for_pos(self, blackboard: dict):
        x, y = [int(x) for x in input('Enter a new x,y position: ').split()]
        # Sanitise pos... eg.
        if not isinstance(x, int) or not isinstance(y, int):
            blackboard['destination'] = None
        else:
            blackboard['destination'] = { 'x': x, 'y': y }
        
    def _update(self, blackboard: dict):
        print('get_dest')
        self._ask_for_pos(blackboard)
        if blackboard['destination'] is not None:
            return STATUS.SUCCESS
        return STATUS.FAILURE

# Checks if the rover is at the destination.
class AtDestination(Behavior):
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        print('at_dest')
        if blackboard['destination']['x'] == blackboard['current_pos']['x'] \
        and blackboard['destination']['y'] == blackboard['current_pos']['y']:
            return STATUS.SUCCESS
        return STATUS.FAILURE