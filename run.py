#!/usr/bin/env python3
from sys import flags
from threading import current_thread
from driver import Driver
from detector import Detector
from map_tools import QrMap 
import time 
import socket
from driver import Driver
from detector import Detector

driver = Driver('/dev/ttyACM0')
detector = Detector(0)

control_update_topic = 'control_update'
goal_update_topic = 'goal_update'

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

port = 5005
host = "192.168.0.2"  
sock.bind((host, port))

map = QrMap()

start, stop = 1, 3
current_goal = 3
current_path = []
path_complete = False
last_step = start
path = map.get_plan(start, stop)
current_step = 0 
last_stop = path[0]

try:
    while True:
        raw_data = sock.recvfrom(1024)
        topic, data = raw_data.split('/')
        if topic == control_update_topic:
            if (data == 'q'):
                driver.send_cmd(100, -100)
            elif (data == 'w'):
                driver.send_cmd(100, 0)
            elif (data == 'e'):
                driver.send_cmd(100, 100)
            elif (data == 'a'):
                driver.send_cmd(0, -100)
            elif (data == 's'):
                driver.send_cmd(0, 0)
            elif (data == 'd'):
                driver.send_cmd(0, 100)
            elif (data == 'z'):
                driver.send_cmd(-100, -100)
            elif (data == 'x'):
                driver.send_cmd(-100, 0)
            elif (data == 'c'):
                driver.send_cmd(-100, 100)
        elif topic == goal_update_topic:
            if current_goal != int(data):
                current_goal = int(data)
        if not path_complete and len(current_path) > 0:
            local_line_pose, marker_id, local_marker_pose = detector.update()

            if current_goal != current_path[-1]:
                try:
                    current_path = map.get_plan(start, current_goal)
                except:
                    print(2)
                finally:
                    continue
            if current_step == len(path):
                path_complete = True
                continue
            elif (marker_id == current_path[0]):
                pass
            else:
                if marker_id != None: # Marker in frame
                    if marker_id not in [current_path[current_step - 1 if current_step > 0 else 0], current_path[current_step], current_path[current_step + 1 if current_step < len(current_path) else 0]]: # We got lost
                        print("How'd I get here")
                    elif marker_id == current_path[current_step]: # We're on the next step
                        direction = map.get_connection_direction(path[current_path], path[current_path + 1]) # Face Direction 
                        driver.face(direction, detector, marker_id)
                        current_step += 1
                    elif last_stop == path[current_step]: # We still havent found the first marker
                        continue    
                else:# detector
                    follow_line(detector, driver)
finally:
    sock.close()
    driver.stop()