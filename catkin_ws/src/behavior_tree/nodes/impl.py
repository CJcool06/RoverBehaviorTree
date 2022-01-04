#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from node import *

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

    def __init__(self, destination_controller):
        super().__init__()
        self.controller = destination_controller

    def _on_initialise(self, blackboard: dict):
        super()._on_initialise(blackboard)
        self.controller.destination = None
        self.controller.start_listening()
    
    def _on_terminate(self, status: STATUS, blackboard: dict):
        super()._on_terminate(status, blackboard)
        self.controller.stop_listening()
        
    def _update(self, blackboard: dict):
        print('get_dest')
        if self.controller.destination is not None:
            blackboard['destination'] = self.controller.destination
            return STATUS.SUCCESS
        return STATUS.RUNNING

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

# Moves the rover toward the destination.
class MoveTowardDestination(Behavior):
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        print('moving')
        blackboard['is_moving'] = True
        if blackboard['current_pos']['x'] < blackboard['destination']['x']:
            blackboard['current_pos']['x'] += 1
        elif blackboard['current_pos']['x'] > blackboard['destination']['x']:
            blackboard['current_pos']['x'] -= 1
        elif blackboard['current_pos']['y'] < blackboard['destination']['y']:
            blackboard['current_pos']['y'] += 1
        elif blackboard['current_pos']['y'] > blackboard['destination']['y']:
            blackboard['current_pos']['y'] -= 1
        
        return STATUS.SUCCESS

# Clears the destination
class ClearDestination(Behavior):
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        blackboard['destination'] = None
        return STATUS.SUCCESS

# Checks if the rover is currently moving.
class IsMoving(Behavior):
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        if blackboard['is_moving'] == True:
            return STATUS.SUCCESS
        return STATUS.FAILURE

# Stops the rover's movement.
class StopMoving(Behavior):
    def __init__(self):
        super().__init__()
    
    def _on_initialise(self, blackboard: dict):
        super()._on_initialise(blackboard)
        # Example implementation
        self.ticks_needed = 5
        self.ticks = 0
        #

    def _update(self, blackboard: dict):
        # Example implementation
        self.ticks += 1
        if self.ticks == self.ticks_needed:
            blackboard['is_moving'] = False
        #

        if blackboard['is_moving'] == False:
            return STATUS.SUCCESS
        return STATUS.RUNNING

# Checks if the rover has a current planned path.
class HasPath(Behavior):
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        if blackboard['path'] is not None:
            return STATUS.SUCCESS
        return STATUS.FAILURE

# Checks if the rover has a current planned path.
class PlanPath(Behavior):
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        blackboard['path'] = 'Go to destination'
        return STATUS.SUCCESS

# Resets the planned path.
class ResetPath(Behavior):
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        blackboard['path'] = None
        return STATUS.SUCCESS

# Checks if the current path has been blocked by an object.
class PathBlocked(Behavior):
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        return STATUS.FAILURE

# For one reason or another, a path was unable to be planned.
# Perform an emergency movement.
class EmergencyMovement(Behavior):
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        print('EMERGENCY!')
        # TODO
        return STATUS.SUCCESS