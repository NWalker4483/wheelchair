#!/usr/bin/env python3
import paho.mqtt.client as mqtt  
import pygame
import utils.colors as colors
import numpy as np
import time


FPS = 30
pygame.init()
pygame.display.set_caption("Tracking System")


import socket               # Import socket module

s = socket.socket()         # Create a socket object
host = socket.gethostname() # Get local machine name
port = 12345                # Reserve a port for your service.
s.bind((host, port))        # Bind to the port

s.listen(5)                 # Now wait for client connection.
while True:
   c, addr = s.accept()     # Establish connection with client.
   print ('Got connection from', addr)
   c.send('Thank you for connecting')
   c.close()                # Close the connection

Length, Width, Height = 41.91, 55.88, 30  # cm

# Setup pygame
SCREEN_WIDTH = int(Width) * 10
W_R = Width / SCREEN_WIDTH
SCREEN_HEIGHT = int(Length) * 10
H_R = Length / SCREEN_HEIGHT

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

clock = pygame.time.Clock()

running = True

qr_map_file = "qr_map.txt"

# Reading the global poses of each QR code
global_qr_poses = dict()
markers = []

control_update_topic = 'control_update'

while running:
    client.loop(timeout=.1)
    # - events -
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    pressed = pygame.key.get_pressed()
    if pressed[pygame.K_q]:
        client.publish(control_update_topic, "q")
        print("q")
        time.sleep(1/24)
    elif pressed[pygame.K_w]:
        client.publish(control_update_topic, "w")
        print("w")
        time.sleep(1/24)
    elif pressed[pygame.K_e]:
        client.publish(control_update_topic, "e")
        print("e")
        time.sleep(1/24)
    elif pressed[pygame.K_a]:
        client.publish(control_update_topic, "a")
        print("a")
        time.sleep(1/24)
    elif pressed[pygame.K_s]:
        client.publish(control_update_topic, "s")
        print("s")
        time.sleep(1/24)
    elif pressed[pygame.K_d]:
        client.publish(control_update_topic, "d")
        print("d")
        time.sleep(1/24)
    elif pressed[pygame.K_z]:
        client.publish(control_update_topic, "z")
        print("z")
        time.sleep(1/24)
    elif pressed[pygame.K_x]:
        client.publish(control_update_topic, "x")
        print("x")
        time.sleep(1/24)
    elif pressed[pygame.K_c]:
        client.publish(control_update_topic, "c")
        print("c")
        time.sleep(1/24)
    
      
    # - draws (without updates) -

    screen.fill(colors.WHITE)

    # for pose in global_qr_poses.values():
    #     angle = pose.rot
    #     length = 50;

    #     x2 = int(round(pose.x + length * np.cos(angle * (3.14 / 180.0))))
    #     y2 = int(round(pose.y + length * np.sin(angle * (3.14 / 180.0))))

    #     x3 = int(round(pose.x + length * np.cos((angle + 90) * (3.14 / 180.0))))
    #     y3 = int(round(pose.y + length * np.sin((angle + 90) * (3.14 / 180.0))))
 
    #     pygame.draw.line(screen, (255,0,0), (pose.x, pose.y), (x2, y2), width = 4)
    #     pygame.draw.line(screen, (0,0,255), (pose.x, pose.y), (x3, y3), width = 4)

    # for pose in [manager.get_tf("map", child_frame) for child_frame in ["base_link"]]:
    #     angle = pose.rot
    #     length = 50;

    #     x2 = int(round(pose.x + length * np.cos(angle * (3.14 / 180.0))))
    #     y2 = int(round(pose.y + length * np.sin(angle * (3.14 / 180.0))))

    #     x3 = int(round(pose.x + length * np.cos((angle + 90) * (3.14 / 180.0))))
    #     y3 = int(round(pose.y + length * np.sin((angle + 90) * (3.14 / 180.0))))
 
    #     pygame.draw.line(screen, (255,125,0), (pose.x, pose.y), (x2, y2), width = 4)
    #     pygame.draw.line(screen, (0,125,255), (pose.x, pose.y), (x3, y3), width = 4)
    
    pygame.display.flip()

    # - constant game speed / FPS -

    clock.tick(FPS)

# - end -
pygame.quit()