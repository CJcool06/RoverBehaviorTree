from nodes.node import Behavior

class Tree:

    def __init__(self, root: Behavior, blackboard: dict):
        self.root = root
        self.blackboard = blackboard

    def tick(self):
        self.root.tick(self.blackboard)
