import serial
from serial import Serial
import time
from detector import Detector
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
    def __init__(self, port):
        # self.gotManualInput = False
        # self.last_update = None # time of last input
        self.attach(port) 
        self.linear = 0 # Upper Servo
        self.angular = 0  # Lower Servo
        self.rotation_speed = 30 # 0:100%
    
    def face(self, direction, detector, ID, tolerance = 3, max_det_gap = 200000):
        """
        direction {0: Top, 1: Bottom, 2: Left, 3: Right}

        max_det_gap: 
        """
        goal_rotations = {0: 270, 1: 90, 2: 180, 3: 0}
        current_rotation = 45
        gap = 0
        turn_val = 0
        while abs(current_rotation - goal_rotations[direction]) > tolerance:
            if gap >= max_det_gap:
                self.stop()
                # raise Exception("Tracking lost For too long")
            _, seen, local_marker_pose = detector.update()
            if seen == ID:
                gap = 0
                current_rotation = local_marker_pose.rot
                print(current_rotation,  goal_rotations[direction])
            else:
                gap += 1
            if current_rotation > goal_rotations[direction]:
                turn_val += 10
            else: 
                 turn_val -= 10
            print(turn_val)
            self.send_cmd(0, turn_val)

    def attach(self, serial_port):
        self.ser = Serial(serial_port, 9600)

    def send_cmd(self, linear, angular):
        linear = linear if linear >= -100 else -100
        angular = angular if angular >= -100 else -100
        linear = linear if linear <= 100 else 100
        angular = angular if angular <= 100 else 100
        
        self.linear, self.angular = linear, angular

        linear_cmd = int(translate(linear, -100, 100, 0, 127))
        angular_cmd = int(translate(angular, -100, 100, 0, 127))
        
        values = [ord('#'), linear_cmd, angular_cmd]
        self.ser.write(bytearray(values))

    def stop(self):
        self.send_cmd(0, 0)
if __name__ == '__main__':
    import numpy as np
    import math
    import math

    import numpy as np
    def rotate(point, origin, degrees):
        radians = np.deg2rad(degrees)
        x,y = point
        offset_x, offset_y = origin
        adjusted_x = (x - offset_x)
        adjusted_y = (y - offset_y)
        cos_rad = np.cos(radians)
        sin_rad = np.sin(radians)
        qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
        qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
        return qx, qy
    drive = Driver('/dev/ttyACM0')
    try:
        while True:
            for angle in range(0, 360, 1):
                x, y = rotate((0,100), (0, 0), angle)
                drive.send_cmd(x, y)
                time.sleep(1/60)
    finally:
        drive.stop()