#!/usr/bin/env python3

# Base Controller Script
# The One Script that runs automatically on startup 
# During development phase this will control just the driver but in the 
# final implementation this will launch both planning and driver code

from utils import UDPStream

from driver import Driver
from detector import Detector
from planner import Planner
from utils.map import QrMap

import time
import socket

import os

control_update_topic = 'c'
goal_update_topic = 'g'
idle_topic = 'i'

Map = QrMap()

driver = Driver()
detector = Detector()
planner = Planner(driver, detector, Map)

# Networking Code
port = os.getenv('CONTROL_PORT') # 5005
host = os.getenv('BASE_IP') # 192.168.0.2 

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

sock = UDPStream(_sock)
sock.start()

connected = False
while not connected:
    try:
        sock.sock.bind((host, port))
        connected = True
    except Exception as e:
        print(e)
        print("Failed to connect retrying in 3 seconds")
        time.sleep(3)

running = True

try:
    while running:
        try:
            # Grab Data from Listener
            if time.time() - sock.data["cap_time"] <= .5:
                raw_data = sock.data["last"]
            else:
                raw_data = "i/0"
                
            topic, data = raw_data.split('/')
            
            if topic == control_update_topic:
                planner.exit_plan()
                driver.send_cmd(*[int(i) for i in data.split(",")])

            elif topic == goal_update_topic:
                planner.set_goal(data)
            
            elif topic == "e": # For Exit
                running = False 
            
        except Exception as e:
            print(e)
            try:
                sock.sock.bind((host, port))
                print("Reconnected")
            except Exception as e:
                print(e, "Delaying for 5 seconds")
                time.sleep(5)
finally:
    planner.exit_plan()
    sock.close()
    driver.stop()