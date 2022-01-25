import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from datetime import datetime

class LocalisationController():
    
    def __init__(self, min_re_localisation_interval):
        self._min_re_localisation_interval = min_re_localisation_interval # In seconds
        
        self._tag_detected = False
        self._last_re_localisation = None
        self._pose = None
        self._old_pose = None
        
        # Subscribers
        rospy.Subscriber('tag_detector/detections', String, self._handle_detections)
        rospy.Subscriber('pose_wrapper/pose', Pose, self._handle_new_pose)

        # Publishers
        self.re_loc_publisher = rospy.Publisher('behavior_tree/controllers/localisation/re_localise', Bool, queue_size=10)
    
    def has_detection(self):
        """
        Checks if a tag is being detected.
        """
        return self._tag_detected
    
    def should_re_localise(self):
        """
        Checks if it's beneficial to re-localise.
        """
        
        if self._last_re_localisation == None:
            return True
        else:
            return (datetime.now() - self._last_re_localisation).total_seconds() > self._min_re_localisation_interval
    
    def re_localise(self):
        """
        Ask the pose wrapper to calculate re-localisation and
        wait for the result.
        
        TODO: Add timeout to prevent hanging in-case of error.
        """
        
        self._old_pose = self._pose                 # Save current pose.
        self._pose = None                           # Reset current pose.
        self.re_loc_publisher.publish(True)         # Signal the pose wrapper to re-localise.
        while self._pose == None:                   # Wait for a reply from the wrapper.
            rospy.sleep(1/20)
        self._last_re_localisation = datetime.now() # Remember current time
        return True
    
    def _handle_detections(self, data):
        """
        Handles detection input.
        """
        
        if data.data == 'None':
            self._tag_detected = False
        else:
            self._tag_detected = True
            
    def _handle_new_pose(self, data):
        """
        Handles pose input.
        """
        self._pose = data
        