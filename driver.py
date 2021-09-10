from serial import Serial
from utils import constrain, rotate_about
import numpy as np
import os
import time
"""
The driver class contains functions for controlling the arduino joystick 
and high-level operations like facing right, left, stopping etc.
Once manual control is added back to also monitor the state of manual 
input versus the automated control"""
def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)
 
class Driver(): 
    def __init__(self, port = None):
        if port == None:
            os.environ.get("JOYSTICK_PORT", '/dev/ttyACM0')
        self.attach(port) 
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
        # I'd like to be able to set
        raise(NotImplementedError)
    
    def stop(self):
        self.send_cmd(0, 0)

if __name__ == '__main__':
    drive = Driver(os.environ.get("JOYSTICK_PORT", '/dev/ttyACM0'))
    try:
        while True:
            for angle in range(0, 360, 1):
                x, y = rotate_about((100, 0), (0, 0), np.rad2deg(angle))
                drive.send_cmd(x, y)
                time.sleep(1/60)
    finally:
        drive.stop()