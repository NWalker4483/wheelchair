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
        self.delimiter = ';'
    def run(self):
        while self.alive:
            recv_data = self.sock.recv(4)
            if not recv_data:
                continue
            if recv_data[:-1] == self.delimiter:
                self.data["last"] = self.__raw_data
            else:
                self.data["last"] += recv_data.decode()
                self.data["cap_time"] = time.time()
    def close(self):
        self.alive = False
        self.sock.close()

driver = Driver('/dev/ttyACM0')
detector = Detector()

control_update_topic = 'c'
goal_update_topic = 'g'
idle_topic = 'i'

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.setblocking(0)
port = 5005
host = "192.168.0.2"  
sock.bind((host, port))
sock = UDPStream(sock)
sock.start()
map = QrMap()

current_goal = -1
current_path = []
current_step = -1 
lost = True
last_sent = ""
running = True
s=time.time()
try:
    while running:
        print("FPS: ", 1.0 / (time.time() - s))
        s = time.time()
        
        if time.time() - sock.data["cap_time"] > .5:
            raw_data = idle_topic + "/0"
        else:
            raw_data = sock.data["last"]
        print(raw_data)
        topic, data = raw_data.split('/')
        if topic == control_update_topic:
            last_sent = data
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
            elif len(data.split(",")) == 2:
                lin, ang = [int(i) for i in data.split(",")]
                driver.send_cmd(lin, ang)
            else:
                print(data)
                
        elif topic == goal_update_topic:
            if current_goal != int(data):
                current_goal = int(data)
                print(f"Current Goal: {current_goal}")
                if len(current_path) == 0: 
                      lost = True
                      current_step = -1
        elif topic == "e":
            running = False
        
        if len(current_path) > 0:
            if current_goal != current_path[-1]:
                try:
                    current_path = map.get_plan(current_path[current_step], current_goal)
                    current_step = 0
                except Exception as e:
                    print("Invalid Goal Set")
                    raise(e)
            
        if (len(current_path) > 0) or lost:# THis Fucks or recv from
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
                    # An ID other than the intended starting ID was seen first
                    # IDK if its necessary but I wrote this part sober so imma levae it alone
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