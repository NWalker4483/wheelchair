# This class controls all interactions with the motors of the wheelchair
from tf import Pose
class Driver(): 
    def __init__(self, base_frame = 'base_link'):
        self.gotManualInput = False
        self.last_update = None # time of last input 
        self.pose_estimate = None
        pass
    def update(self): # New command velocities and update your internal pose estimate
        pass