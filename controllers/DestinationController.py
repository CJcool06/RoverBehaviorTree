#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import threading

class DestinationController():
    """
    The Destination Controller handles how the rover receives destinations from input.
    This class is meant to be used by both the behavior_tree_comms node (publisher) and
    the behavior_tree_runner node (listener).
    
    Nodes have a separate instance.
    """

    def __init__(self):
        self.destination = None
        self.publishing = False
        self.pub_thread = None
        self.listening = False

    def open_publisher(self):
        """
        Opens the publisher topic and starts the publisher thread.
        """

        publisher = rospy.Publisher('behavior_tree/controller/destination', String, queue_size=10)

        # Publish input on different thread.
        # This is prevent blocking by asking for input.
        pub_thread = threading.Thread(target=self._threaded_publisher, args=(publisher,), daemon=True)
        pub_thread.start()
    
    def open_subscriber(self):
        """
        Opens the subscriber topic.
        """

        # callback_lambda = lambda data : handle_input(data, controller)
        callback_lambda = lambda data : self._handle_input(data)
        subscriber = rospy.Subscriber('behavior_tree/controller/destination', String, callback_lambda)

    def start_publishing(self):
        """
        Indicates that the publisher should start publishing.
        """
        self.publishing =  True

    def stop_publishing(self):
        """
        Indicates that the publisher should stop publishing.
        """
        self.publishing = False

    def start_listening(self):
        """
        Indicates that the subscriber should start listening.
        """
        self.listening = True
    
    def stop_listening(self):
        """
        Indicates that the subscriber should stop listening.
        """
        self.listening = False
    
    def _handle_input(self, input):
        """
        If the subscriber should be listening, receive the published message.

        The message should be a string {x y} where x,y are floats.
        """

        if self.listening:
            nums = input.data.split()
            self.destination = {
                'x': float(nums[0]),
                'y': float(nums[1])
            }
    
    def _sanitise(self, inp_x, inp_y):
        """
        Sanitises the input before publishing it to the topic.

        Ensures that the two inputs are floats, then returns {x y}.
        Otherwise returns None
        """

        try:
            x = float(inp_x)
            y = float(inp_y)
            # TODO: Return dict and convert using rospy_message_converter package.
            return inp_x + ' ' + inp_y
        except:
            return None

    def _threaded_publisher(self, publisher):
        """
        If the publisher should be publishing, send the sanitised message to the topic.
        """

        try:
            while not rospy.is_shutdown():
                inp_x = input('Enter coordinate x: ')
                inp_y = input('Enter coordinate y: ')
                res = self._sanitise(inp_x, inp_y)
                if res is not None and self.publishing:
                    publisher.publish(res)
                elif res is not None:
                    print('Not currently publishing...')
                else:
                    print('Did not publish - incorrect input.')
        except:
            print('Error encountered in DestinationController publisher.')
    
    