# This class controls all interactions with the motors of the wheelchair
# The driver controls the motors and stores an internal estimate of where the base is in 2D space
import RPi.GPIO as GPIO
from tf import Pose
import pins
class Driver(): 
    def __init__(self, base_frame = 'base_link'):
        self.gotManualInput = False
        self.last_update = None # time of last input 
        self.pose_estimate = Pose()
    
    def update(self, new_cmd): # New command velocities and update your internal pose estimate
        if new_cmd == None:
            pass