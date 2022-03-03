import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from datetime import datetime
import json

class LocalisationController():
    
    def __init__(self, min_re_localisation_interval):
        self._min_re_localisation_interval = min_re_localisation_interval # In seconds
        
        self._tag_detected = False
        self._last_re_localisation = None
        self._pose = None
        self._old_pose = None
        self._last_detections = None
        
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
        elif self._tag_detected:
            err_x, err_y = self._error_from_centre()
            print(f'X: {err_x}, Y: {err_y}')
            if err_x <= 0.3 and err_y <= 0.2:
                return (datetime.now() - self._last_re_localisation).total_seconds() > self._min_re_localisation_interval
        return False
    
    def re_localise(self):
        """
        Ask the pose wrapper to calculate re-localisation and
        wait for the result.
        
        TODO: Add timeout to prevent hanging in-case of error.
        """
        
        self._old_pose = self._pose                 # Save current pose.
        self._pose = None                           # Reset current pose.
        self.re_loc_publisher.publish(True)         # Signal the pose wrapper to re-localise.
        print('Published.')
        while self._pose == None:                   # Wait for a reply from the wrapper.
            rospy.sleep(1/20)
        self._last_re_localisation = datetime.now() # Remember current time
        return True
    
    def _error_from_centre(self):
        """
        The percentage (0-1) of pixels that the best tag is off-centre for both x and y.
        
        For example, if the image is 1000 pixels wide and the tag's centre is at pixel 300
        then we consider it 200 pixels off-centre, thus 200/(1000/2) = 0.4 = 40% error from the centre.
        """
        
        if self._last_detections == None:
            return 1
        
        best_x = 10000
        best_y = 10000
        for d in self._last_detections:
            best_x = min(best_x, abs((d['image-dimensions'][1]//2) - d['center'][0]))
            best_y = min(best_y, abs((d['image-dimensions'][0]//2) - d['center'][1]))
        return best_x / (d['image-dimensions'][1] / 2), best_y / (d['image-dimensions'][0] / 2)
    
    def _handle_detections(self, data):
        """
        Handles detection input.
        """
        
        if data.data == 'None':
            self._tag_detected = False
            self._last_detections = None
        else:
            self._tag_detected = True
            self._last_detections = json.loads(data.data)
            
    def _handle_new_pose(self, data):
        """
        Handles pose input.
        """
        self._pose = data
        