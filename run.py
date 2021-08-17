#!/usr/bin/env python3
from driver import Driver
from detector import Detector
from map_tools import QrMap 
import time 
import socket
from driver import Driver
import cv2
from detector import Detector
from threading import Thread

class UDPStream(Thread):

    def __init__(self, socket):
        Thread.__init__(self)
        self.daemon = True
        self.data = {"last": "i/0","cap_time": time.time()}
        self.__raw_data = ''
        self.sock = socket
        self.alive = True

    def run(self):
        while self.alive:
            recv_data = self.sock.recv(1024)
            if not recv_data:
                continue
            self.__raw_data += recv_data.decode()

            start, stop = -1, -1
            for i in range(len(self.__raw_data) - 1, -1, -1):
                if start == -1 and self.__raw_data[i] == ";":
                    stop = i
                if stop != -1 and self.__raw_data[i] == "#":
                    start = i
                    break
         
            if start >= 0 and stop >= 0 and stop > start:
                self.data["last"] = self.__raw_data[start + 1: stop]
                self.__raw_data = self.__raw_data[stop:]
                self.data["cap_time"] = time.time()

    def close(self):
        self.alive = False
        self.sock.close()

driver = Driver('/dev/ttyACM0')
detector = Detector()

control_update_topic = 'j'
goal_update_topic = 'g'
idle_topic = 'i'

# Networking Code
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 5005
host = "192.168.0.2"  
sock.bind((host, port))
sock = UDPStream(sock)
sock.start()

map = QrMap()

# Path Nav Code 
current_goal = -1
current_path = []
current_step = -1 
lost = True

running = True

s = time.time()

try:
    while running:
        # print("FPS: ", 1.0 / (time.time() - s))
        s = time.time()
        
        if time.time() - sock.data["cap_time"] > .5:
            raw_data = idle_topic + "/0"
        else:
            raw_data = sock.data["last"]
        topic, data = raw_data.split('/')
        if topic == control_update_topic:
            driver.send_cmd(*[int(i) for i in data.split(",")])
                
        elif topic == goal_update_topic:
            if current_goal != int(data):
                current_goal = int(data)
                print(f"Current Goal: {current_goal}")
                if len(current_path) == 0: 
                      lost = True
                      current_step = -1
                      
        elif topic == "e":
            running = False
            
        if (len(current_path) > 0) and (current_goal != current_path[-1]):
            try:
                current_path = map.get_plan(current_path[current_step], current_goal)
                current_step = 0
            except Exception as e:
                print("Invalid Goal Set")
                raise(e)
            
        if (len(current_path) > 0) or lost:
            local_line_form, marker_id, local_marker_pose = detector.update()
            
            if marker_id != None: # Marker in frame
                if (lost):
                      # Setup the other loop to calculate the set path
                      current_path = [marker_id]
                      current_step = 0
                elif marker_id == current_path[current_step]: # We're on the next step
                    current_step += 1
                    if current_step == len(current_path) or marker_id == current_goal:
                        current_path = []
                        current_goal = -1
                        current_step = -1
                        print("Goal Reached")
                        continue
                    direction = map.get_connection_direction(path[current_path], path[current_path + 1]) # Face Direction 
                    driver.face(direction, detector, marker_id)
                    
                      
                elif current_step == 0: # We still havent found the first marker
                    continue    
                else:
                    # TODO Maybe add filtering
                    if local_line_form[0] > 0:
                        driver.send_cmd(70, driver.angular + 2)
                    else:
                        driver.send_cmd(70, driver.angular - 2)
        if 1: # Blocks recvform for some reason
            cv2.imshow("Debug", detector.getDebugView())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
finally:
    sock.close()
    driver.stop()