#!/usr/bin/env python3
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

current_goal = -1
current_path = []
current_step = -1 
path_complete = True

try:
    while True:
        raw_data = sock.recv(1024) 
        topic, data = raw_data.decode().split('/')
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
                print(f"Current Goal: {current_goal}")
        if len(current_path) > 0:
            if current_goal != current_path[-1]:
                try:
                    current_path = map.get_plan(start, current_goal)
                    current_step = 0
                    last_stop = current_path[0]
                except:
                    print(2)
            if current_step == len(path) or marker_id == current_goal:
                current_path = []
                continue
                    
        if len(current_path) > 0:
            local_line_form, marker_id, local_marker_pose = detector.update()

            if (marker_id == current_path[0]):
                pass
            else:
                if marker_id != None: # Marker in frame
                    if marker_id == current_path[current_step]: # We're on the next step
                        direction = map.get_connection_direction(path[current_path], path[current_path + 1]) # Face Direction 
                        driver.face(direction, detector, marker_id)
                        current_step += 1
                    elif current_step == 0: # We still havent found the first marker
                        continue    
                    else:
                        if local_line_form[0] > 0:
                            driver.send_cmd(70, driver.angular + 2)
                        else:
                            driver.send_cmd(70, driver.angular - 2)
finally:
    sock.close()
    driver.stop()