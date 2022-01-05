#!/usr/bin/env python3

import os, sys
from pathlib import Path
parent_dir = Path(os.path.dirname(__file__)).parent.absolute()
sys.path.append(os.path.join(parent_dir, "tree"))
sys.path.append(os.path.join(parent_dir, "nodes"))
sys.path.append(os.path.join(parent_dir, "controllers"))

import rospy
from std_msgs.msg import String

from queue import Queue
import threading

from DestinationController import DestinationController

if __name__ == '__main__':

    # Init rospy
    rospy.init_node('behavior_tree_comms', anonymous=False)

    # Create controller
    controller = DestinationController()

    # Open & start publisher
    controller.open_publisher()
    controller.start_publishing()

    # Prevents exit unless node is stopped
    rospy.spin()

