#!/usr/bin/env python3

from node import Behavior

class Tree:
    """
    An implementation of the behavior tree.
    """

    def __init__(self, root: Behavior, blackboard: dict):
        self.root = root
        self.blackboard = blackboard

    """
    Ticks the root behavior of the tree.
    """
    def tick(self):
        self.root.tick(self.blackboard)
