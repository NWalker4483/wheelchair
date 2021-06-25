#!/usr/bin/env python3
from wheelchair.driver import Driver
from wheelchair.detector import Detector
from map_tools import QrMap 

driver = Driver('/dev/ttyUSB0')
detector = Detector('test_data/green.mp4')

def run_path(start, stop):
    pass

start, stop = 1, 3

map = QrMap()
last_step = start
path = map.get_plan(start, stop)


def follow_line(detector, driver):
    pass

def follow_line_until(detector, driver, ID):
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
                print("How'd I get here")
            elif ID == path[step]: # We're on the next step
                direction = map.get_connection_direction(path[step], path[step + 1]) # Face Direction 
                driver.face(direction, detector, ID)
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
