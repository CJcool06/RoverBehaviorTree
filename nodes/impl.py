#!/usr/bin/env python3

from LocalisationController import LocalisationController
from DestinationController import DestinationController

from node import *


class HasDestination(Behavior):
    """
    Checks if the rover has a destination to move to.

    SUCCESS: A destination exists in the blackboard.
    FAILURE: A destination does not exist in the blackboard.
    RUNNING: --
    INVALID: --
    """

    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        if blackboard['destination'] is not None:
            return STATUS.SUCCESS
        return STATUS.FAILURE


class GetDestination(Behavior):
    """
    Asks for a destination from the DestinationController.
    If one is received, the destination is added to the blackboard.

    SUCCESS: The controller has a destination.
    FAILURE: --
    RUNNING: Waiting for a destination from the controller.
    INVALID: --
    """

    def __init__(self, destination_controller: DestinationController):
        super().__init__()
        self.controller = destination_controller

    def _on_initialise(self, blackboard: dict):
        super()._on_initialise(blackboard)
        self.controller.destination = None
        self.controller.start_listening()   # Can be removed
    
    def _on_terminate(self, status: STATUS, blackboard: dict):
        super()._on_terminate(status, blackboard)
        self.controller.stop_listening()    # Can be removed
        
    def _update(self, blackboard: dict):
        if self.controller.destination is not None:
            blackboard['destination'] = self.controller.destination
            return STATUS.SUCCESS
        return STATUS.RUNNING


# TODO
class AtDestination(Behavior):
    """
    Checks if the rover is at the destination.

    SUCCESS: The rover is within TODO: (X cm) of the threshold.
    FAILURE: The rover is not at the destination.
    RUNNING: --
    INVALID: --
    """


    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        if blackboard['destination']['x'] == blackboard['current_pos']['x'] \
        and blackboard['destination']['y'] == blackboard['current_pos']['y']:
            return STATUS.SUCCESS
        return STATUS.FAILURE


# TODO
class MoveTowardDestination(Behavior):
    """
    Sends an instruction to the move controller.

    SUCCESS: The instruction was sent.
    FAILURE: There was a problem sending the instruction.
    RUNNING: --
    INVALID: --
    """

    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
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


class ClearDestination(Behavior):
    """
    Clears the destination from the blackboard.

    SUCCESS: The destination was cleared.
    FAILURE: --
    RUNNING: --
    INVALID: --
    """


    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        blackboard['destination'] = None
        return STATUS.SUCCESS


# TODO
class IsMoving(Behavior):
    """
    Checks if the rover is currently moving.

    SUCCESS: The rover is moving.
    FAILURE: The rover is not moving.
    RUNNING: --
    INVALID: --
    """

    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        if blackboard['is_moving'] == True:
            return STATUS.SUCCESS
        return STATUS.FAILURE


# TODO
class StopMoving(Behavior):
    """
    Stops the rover's movement.

    SUCCESS: The rover is no longer moving.
    FAILURE: --
    RUNNING: The rover is still moving.
    INVALID: --
    """

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


# TODO
class HasPath(Behavior):
    """
    Checks if the rover has a current planned path.

    SUCCESS: The rover has a path.
    FAILURE: The rover does not have a path.
    RUNNING: --
    INVALID: --
    """

    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        if blackboard['path'] is not None:
            return STATUS.SUCCESS
        return STATUS.FAILURE


# TODO
class PlanPath(Behavior):
    """
    Checks if the rover has a current planned path.

    SUCCESS: A path was successfully planned.
    FAILURE: A path was unsuccessfully planned.
    RUNNING: --
    INVALID: --
    """

    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        blackboard['path'] = 'Go to destination'
        return STATUS.SUCCESS


# TODO
class ResetPath(Behavior):
    """
    Resets the planned path.

    SUCCESS: The path was cleared from the blackboard.
    FAILURE: --
    RUNNING: --
    INVALID: --
    """

    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        blackboard['path'] = None
        return STATUS.SUCCESS


# TODO
class PathBlocked(Behavior):
    """
    Checks if the current path has been blocked by an object.

    SUCCESS: The path has been blocked.
    FAILURE: The path is clear.
    RUNNING: --
    INVALID: --
    """
    
    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        return STATUS.FAILURE


# TODO
class EmergencyMovement(Behavior):
    """
    For one reason or another, a path was unable to be planned.
    Perform an emergency movement.

    SUCCESS: The movement has completed.
    FAILURE: Something else went wrong.
    RUNNING: The movement is in-progress.
    INVALID: --
    """

    def __init__(self):
        super().__init__()

    def _update(self, blackboard: dict):
        print('EMERGENCY!')
        return STATUS.SUCCESS


class TagDetected(Behavior):
    """
    Checks if a tag is currently being detected.
    
    SUCCESS: A tag is detected.
    FAILURE: A tag is not detected.
    RUNNING: --
    INVALID: --
    """
    
    def __init__(self, localisation_controller: LocalisationController):
        super().__init__()
        self._controller = localisation_controller
    
    def _update(self, blackboard: dict):
        if self._controller.has_detection():
            return STATUS.SUCCESS
        return STATUS.FAILURE
    

class ShouldReLocalise(Behavior):
    """
    Checks if the rover would benefit from a re-localisation
    using the currently detected tag.
    
    SUCCESS: The rover should re-localise.
    FAILURE: The rover does not need to re-localise.
    RUNNING: --
    INVALID: --
    """
    
    def __init__(self, localisation_controller: LocalisationController):
        super().__init__()
        self._controller = localisation_controller
    
    def _update(self, blackboard: dict):
        if self._controller.should_re_localise():
            return STATUS.SUCCESS
        return STATUS.FAILURE


# TODO
class ReLocalise(Behavior):
    """
    Attempt to re-localise the rover using the
    currently detected tag(s).
    
    SUCCESS: Re-Localisation was successful.
    FAILURE: Re-Localisation failed.
    RUNNING: --
    INVALID: --
    """
    
    def __init__(self, localisation_controller: LocalisationController):
        super().__init__()
        self._controller = localisation_controller
    
    def _update(self, blackboard: dict):
        print('RELOCALISING...')
        if self._controller.re_localise():
            print('FINISHED.')
            return STATUS.SUCCESS
        return STATUS.FAILURE