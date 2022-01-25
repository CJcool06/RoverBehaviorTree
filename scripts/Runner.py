#!/usr/bin/env python3

# Adds all files to the import path.
import os, sys
from pathlib import Path
parent_dir = Path(os.path.dirname(__file__)).parent.absolute()
sys.path.append(os.path.join(parent_dir, "tree"))
sys.path.append(os.path.join(parent_dir, "nodes"))
sys.path.append(os.path.join(parent_dir, "controllers"))

import rospy
from tree_impl import Tree
from node import Fallback, Sequence, ForceSuccess, RepeatUntilSuccess, Inverter
from impl import *
from DestinationController import DestinationController
from LocalisationController import LocalisationController
import time

def Root(destination_controller: DestinationController, localisation_controller: LocalisationController):
    """
    Root of the behavior tree.

    Children:
    1. Ensures there's a destination.
    2. Performs checks and moves rover toward destination.
    """
    return (
        # Root of the tree.
        Sequence()
            # Success if there's a destination.
            .addChild(EnsureDestination(destination_controller))
            # --
            .addChild(ChecksAndMoveFallback(localisation_controller))
    )

def EnsureDestination(destination_controller: DestinationController):
    """
    If there is no destination, listen for input and report running.

    Children:
    1. Checks if the rover already has a destination.
    2. Get a new destination.
    """
    return (
        Fallback()
            # Success if there already exists a destination.
            .addChild(HasDestination())
            # Success when a destination is received, otherwise running.
            .addChild(GetDestination(destination_controller))
    )

def ChecksAndMoveFallback(localisation_controller: LocalisationController):
    """
    Ensures that the pre-requisites are fulfilled, then moves toward the destination.

    Children:
    1. Performs important checks before proceeding.
    2. Ensures we are localised properly.
    3. Ensures that a path exists to reach the destination.
    4. Enter the final checks, then move toward the destination.
    """
    return (
        Fallback()
            # Success if checks fail.
            .addChild(ImportantChecks())
            # Success if re-localised.
            .addChild(EnsureLocalisation(localisation_controller))
            # Success if a path is not ensured.
            .addChild(Inverter(EnsurePath()))
            # Success if the rover submitted a move command.
            .addChild(MoveFallback())
    )

def ImportantChecks():
    """
    Ensures that we're not already at the destination. If we are, then stop moving, clear the destination, and reset the planned path.

    Children:
    1. Checks if we are at the destination.
    2. Ensures that we are stopped.
    3. Clears the current destination
    4. Resets the planned path.
    """
    return (
        Sequence()
            # Success if the rover is at the destination
            .addChild(AtDestination())
            # Success if the rover is no longer moving.
            .addChild(EnsureNotMoving())
            # Success if the destination was cleared.
            .addChild(ClearDestination())
            # Resets the current path.
            .addChild(ResetPath())
    )

def EnsureNotMoving():
    """
    Ensures that the rover will no longer be moving.
    
    Children:
    1. Checks if we are moving.
    2. Stop moving.
    """
    return (
        ForceSuccess(
            Sequence()
                # Success if the rover is currently moving.
                .addChild(IsMoving())
                # Always success.
                .addChild(RepeatUntilSuccess(StopMoving()))
        )
    )
    
def EnsureLocalisation(localisation_controller: LocalisationController):
    """
    Ensures that we are localised as accurately as we can be.
    
    Children:
    1. Checks if a tag is being detected.
    2. Determine if it would be beneificial to re-localise.
    3. Re-Localise.
    """
    return (
        Sequence()
            .addChild(TagDetected(localisation_controller))
            .addChild(ShouldReLocalise(localisation_controller))
            .addChild(ForceSuccess(ReLocalisation(localisation_controller)))
    )

def ReLocalisation(localisation_controller: LocalisationController):
    """
    Re-Localises the rover's position.
    
    Children:
    1. Stop moving.
    2. Re-Localise.
    3. Reset the current planned path.
    """
    return (
        Sequence()
            .addChild(EnsureNotMoving())
            .addChild(ReLocalise(localisation_controller))
            .addChild(ResetPath())
    )

def EnsurePath():
    """
    Ensures that the rover has a planned path.

    Children:
    1. Checks if we already have a path.
    2. Plans a path.
    3. Perform an emergency movement.
    """
    return (
        Fallback()
            # Success when there is a path.
            .addChild(HasPath())
            # Success if there is a new planned path.
            .addChild(PlanPath())
            # Success if the emergency movement did not succeed.
            .addChild(Inverter(EmergencyMovement()))
    )

def MoveFallback():
    """
    Ensures the path is clear, then moves toward the destination.

    Children:
    1. Checks if the path is clear of obstacles.
    2. Move toward the destination.
    """
    return (
        Fallback()
            # Success if path is clear.
            .addChild(CheckPath())
            # Success if move command was successfully sent.
            .addChild(MoveTowardDestination())
    )

def CheckPath():
    """
    Checks the path for new obstacles. If the path is blocked, reset it and stop the rover.

    Children:
    1. Checks if the path is blocked.
    2. Resets the path.
    3. Stop the rover from moving.
    """
    return (
        Sequence()
            .addChild(
                Sequence()
                    # Success if the path is blocked.
                    .addChild(PathBlocked())
                    # Success if the path has been reset.
                    .addChild(ResetPath())
            )
            # Always success.
            .addChild(EnsureNotMoving())
    )

def Blackboard():
    """
    The blackboard given to each behavior in the tree.
    """
    return {
        'destination': None,                    # Target destination coordinates
        'current_pos': { 'x': 0.0, 'y': 0.0 },  # Current position coordinates
        'is_moving': False,                     # Is the rover currently moving
        'path': None                            # Current planned path
    }


if __name__ == '__main__':

    # Subscribes to the DestinationController.
    rospy.init_node('behavior_tree_runner', anonymous=False)
    localisation_controller = LocalisationController(10)
    destination_controller = DestinationController()
    destination_controller.open_subscriber()
    
    # Instantiates the tree.
    tree = Tree(Root(destination_controller, localisation_controller), Blackboard())

    try:
        # Tick the tree every interval.
        while True:
            tree.tick()
            print(tree.blackboard)
            time.sleep(1/2)
    except:
        print('Stopping behavior tree.')