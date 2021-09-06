import time
from simple_pid import PID
from gpiozero import Servo
import numpy as np

# For Visual Feedback 
from detector import Detector

"""
The driver class contains functions for controlling the joystick 
and high-level operations like facing right, left, stopping etc.
Once manual control is added back to also monitor the state of manual 
input versus the automated control
"""

def face_towards():
    pass

class Driver(): 
    def __init__(self):
        self.linear_servo = Servo(8, min_pulse_width = 1, max_pulse_width = 2)
        self.angular_servo = Servo(10, min_pulse_width = 1, max_pulse_width = 2)

        self.linear = 0 # Upper Servo 
        self.angular = 0  # Lower Servo -100 : 100 - Right : Left
        
    def face(self, direction, detector, ID, tolerance = 3, max_det_gap = 200):
        """
        direction {0: Top, 1: Bottom, 2: Left, 3: Right}

        max_det_gap: 
        """
        try:
            if type(direction) == str:
                direction = {"top": 0 , "bottom": 1, "left": 2, "right": 3}[direction]
            else:
                assert(type(direction) == int)
                assert(0 <= direction <= 3)
        except AssertionError as e:
            raise(e)

        goal_rotations = {0: 270, 1: 90, 2: 180, 3: 0}
        gap = 0
        min_ang_dist = lambda a, b: (b - a) if abs(a - b) < abs(360 - max(a,b) + min(a,b)) else (max(a,b) + min(a,b) - 360)
        angle_error = 10e5
        pid = PID(.125, .125, .25)
        pid.setpoint = 0
        pid.sample_time = 1/10 # 10 Hz
        pid.output_limits = (-100, 100)  # Output will always be above 0, but with no upper bound
        pid.error_map = lambda x: min_ang_dist(x,goal_rotations[direction]) 
        
        while (abs(angle_error) > tolerance):
            if gap >= max_det_gap:
                self.stop()
                raise Exception("Tracking lost For too long")
            _, seen, local_marker_pose = detector.update()
                
            if seen == ID:
                gap = 0
                # Flip Left and Right
                control = -pid(local_marker_pose.rot)
                self.send_cmd(0, control)
                # print(min_ang_dist(local_marker_pose.rot, goal_rotations[direction]), control)
            else:
                gap += 1

    def send_cmd(self, linear, angular):
        constrain = lambda x, min_, max_: min_ if x < min_ else (max_ if x > max_ else x)        
        self.linear, self.angular = constrain(linear, -1, 1), constrain(angular, -1, 1)
        
        self.linear_servo = Servo(8, min_pulse_width = 1, max_pulse_width = 2)
        self.angular_servo = Servo(10, min_pulse_width = 1, max_pulse_width = 2)
 
    def send_speed(self, x, y):
        pass
    
    def stop(self):
        self.send_cmd(0, 0)

if __name__ == '__main__':

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
            # for x,y in [(100,0),(-100,0),(0,100),(0,-100),(0,0)]:
            #    drive.send_cmd(x, y)
            #   time.sleep(1)
            for angle in range(0, 360, 1):
                x, y = rotate((0,100), (0, 0), angle)
                drive.send_cmd(x, y)
                time.sleep(1/60)
    finally:
        drive.stop()