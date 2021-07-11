#!/usr/bin/env python3
from driver import Driver
from detector import Detector

driver = Driver('/dev/ttyACM0')
detector = Detector(0)

control_update_topic = 'control_update'
goal_update_topic = 'goal'

# Import socket module 
import socket             
  
# Create a socket object 
s = socket.socket()         
  
# Define the port on which you want to connect 
port = 12345                
  
# connect to the server on local computer 
host = "192.168.0.4" # '127.0.0.1'
s.connect((host, port)) 
  
# receive data from the server 
print (s.recv(1024) )
# close the connection 
s.close()     

    if msg.topic == control_update_topic:
        payload = msg.payload.decode()
        payload = payload.lower()
        if (payload == 'q'):
            driver.send_cmd(100,-100)
        elif (payload == 'w'):
            driver.send_cmd(100,0)
        elif (payload == 'e'):
            driver.send_cmd(100,100)
        elif (payload == 'a'):
            driver.send_cmd(0,-100)
        elif (payload == 's'):
            driver.send_cmd(0,0)
        elif (payload == 'd'):
            driver.send_cmd(0,100)
        elif (payload == 'z'):
            driver.send_cmd(-100,-100)
        elif (payload == 'x'):
            driver.send_cmd(-100,0)
        elif (payload == 'c'):
            driver.send_cmd(-100,100)
        
        
    elif msg.topic == goal_update_topic:
        print(msg.payload.decode())



print("Starting Wheelchair Base")
running = False
try:
    while True:  # loop over the frames from the video stream

except Exception as e:
    raise(e)
finally:
    print('[INFO] cleaning up...')
    client.disconnect()
    driver.stop()
