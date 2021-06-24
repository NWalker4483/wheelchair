from tf import Pose
import serial
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
        self.rotation_speed = 50 # 0:100%
    
    def face(self, direction, detector, ID, tolerance = 10, max_det_gap = 120):
        """
        direction {0: Top, 1: Bottom, 2: Left, 3: Right}

        max_det_gap: 
        """
        goal_rotations = {0: 0, 1: 180, 2: 90, 3: 270}
        current_rotation = 77
        gap = 0 
        while abs(current_rotation - goal_rotations[direction]) > tolerance:
            if gap >= max_det_gap:
                break
            seen, local_marker_pose = detector.update()
            if seen == ID:
                gap = 0
                current_rotation = local_marker_pose.rot
            else:
                gap += 1
            if current_rotation > goal_rotations[direction]:
                self.send_cmd(0, self.rotation_speed)
            else: 
                self.send_cmd(0, -self.rotation_speed)

    def attach(self, serial_port):
        self.ser = serial.Serial(serial_port, 9600)

    def send_cmd(self, linear, angular):
        # TODO: Constrain 
        self.linear, self.angular = linear, angular

        linear_cmd = int(translate(linear, -100, 100, 0, 127))
        angular_cmd = int(translate(linear, -100, 100, 0, 127))

        values = [ord('#'), linear_cmd, angular_cmd]
        self.ser.write(bytearray(values))

    def stop(self):
        self.send_cmd(0, 0)
if __name__ == '__main__':
    drive = Driver()
    try:
        while True:
            linear, angular = [int(i) for i in input("{linear} {angular}\n")][:2]
            drive.send_cmd(linear, angular)
            time.sleep(1)
    finally:
        drive.stop()