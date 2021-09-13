from serial import Serial
from utils import constrain
from utils.math import rotate_about, translate
import numpy as np
import os
import time
"""
The driver class contains functions for controlling the arduino joystick 
and high-level operations like facing right, left, stopping etc.
Once manual control is added back to also monitor the state of manual 
input versus the automated control"""

class Driver(): 
    def __init__(self, port = None):
        if port == None:
            os.environ.get("JOYSTICK_PORT", '/dev/ttyACM0')
        self.attach('/dev/ttyACM0') 
        self.linear = 0 # Upper Servo 
        self.angular = 0  # Lower Servo -100 : 100 - Right : Left
   
    def attach(self, serial_port):
        self.ser = Serial(serial_port, 9600)
          
    def send_cmd(self, linear, angular):      
        self.linear, self.angular = constrain(linear, -100, 100), constrain(angular, -100, 100)

        linear_cmd = int(translate(self.linear, -100, 100, 0, 127))
        angular_cmd = int(translate(self.angular, -100, 100, 0, 127))
        
        values = [ord('#'), linear_cmd, angular_cmd]
        self.ser.write(bytearray(values))
 
    def send_speed(self, x, r):
        # I'd like to be able to set a velocity once Detector.estimateVelocities is reliable 
        raise(NotImplementedError)
    
    def stop(self):
        self.send_cmd(0, 0)

if __name__ == '__main__':
    drive = Driver()
    try:
        while True:
            for angle in range(0, 360, 1):
                x, y = rotate_about((100, 0), (0, 0), np.deg2rad(angle))
                drive.send_cmd(x, y)
                time.sleep(1/60)
    finally:
        drive.stop()