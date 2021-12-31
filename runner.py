from tree.behavior_tree import Tree
from nodes.node import Fallback, Sequence
from nodes.impl import *
import time

def Root():
    return (
        Sequence()
            .addChild(Destination())
            # .addChild(ImportantChecks())
    )

def Destination():
    return (
        Fallback()
            .addChild(HasDestination())
            .addChild(GetDestination())
    )
                

def ImportantChecks():
    return (
        Fallback()
            .addChild(AtDestination())
    )

def AtPosition():
    return (
        Sequence()
            .addChild()
    )

def Blackboard():
    return {
        'destination': None,
        'current_pos': { 'x': 0, 'y': 0 }
    }

tree = Tree(Root(), Blackboard())

try:
    while True:
        tree.tick()
        print(tree.blackboard)
        time.sleep(1/2)
except:
    print('Ending.')