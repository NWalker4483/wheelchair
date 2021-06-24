#!/usr/bin/env python3
from driver import Driver
from detector import Detector

import time
from tf import Pose
from map_tools import QrMap 

driver = Driver('/dev/ttyUSB0')
detector = Detector('data/green.mp4')

start, stop = 1, 3

m = QrMap()
a = 0
last_step = start
path = m.get_plan(start, stop)

def align(direction, detector, driver):
    # direction {0: Top, 1: Bottom, 2: Left, 3: Right}
    driver.stop()
    last_pose = None
    pass
def follow_line(detector, driver):
    pass

started = False

step = 0 
last_stop = path[0]
try:
    while True:  # loop over the frames from the video stream
        ID, local_marker_pose = detector.update()
        if step == len(path):
            break
        if ID != None: # Marker in frame
            if ID not in [path[step - 1 if step > 0 else 0], path[step], path[step + 1 if step < len(path) else 0]]: # We got lost
                print(3)
            # We're on the next step
            elif ID == path[step]: 
                print(1)
                direction = a.get_connection_direction(stop, path[step + 1]) # Face Direction 
                align(direction, detector, driver)
                step += 1
            elif last_stop == path[step]: # We still havent found the first marker
                continue
            else:
                follow_line(detector, driver)
except Exception as e:
    raise(e)
finally:
    print('[INFO] cleaning up...')
    driver.stop()
    # client.disconnect()
