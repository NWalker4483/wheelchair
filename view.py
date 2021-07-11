#!/usr/bin/env python3
import paho.mqtt.client as mqtt  
import pygame
import utils.colors as colors
import numpy as np
import time


FPS = 30
pygame.init()
pygame.display.set_caption("Tracking System")


import socket

UDP_IP = "192.168.0.2"
UDP_PORT = 5005
MESSAGE = b"Hello, World!"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
def publish(topic, data):
    sock.sendto(data.encode("utf-8"), (UDP_IP, UDP_PORT))

screen = pygame.display.set_mode((500, 500))

clock = pygame.time.Clock()

running = True

qr_map_file = "qr_map.txt"

# Reading the global poses of each QR code
global_qr_poses = dict()
markers = []

control_update_topic = 'control_update'

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    pressed = pygame.key.get_pressed()
    if pressed[pygame.K_q]:
        publish(control_update_topic, "q")
        print("q")
        time.sleep(1/24)
    elif pressed[pygame.K_w]:
        publish(control_update_topic, "w")
        print("w")
        time.sleep(1/24)
    elif pressed[pygame.K_e]:
        publish(control_update_topic, "e")
        print("e")
        time.sleep(1/24)
    elif pressed[pygame.K_a]:
        publish(control_update_topic, "a")
        print("a")
        time.sleep(1/24)
    elif pressed[pygame.K_s]:
        publish(control_update_topic, "s")
        print("s")
        time.sleep(1/24)
    elif pressed[pygame.K_d]:
        publish(control_update_topic, "d")
        print("d")
        time.sleep(1/24)
    elif pressed[pygame.K_z]:
        publish(control_update_topic, "z")
        print("z")
        time.sleep(1/24)
    elif pressed[pygame.K_x]:
        publish(control_update_topic, "x")
        print("x")
        time.sleep(1/24)
    elif pressed[pygame.K_c]:
        publish(control_update_topic, "c")
        print("c")
        time.sleep(1/24)
    
      
    # - draws (without updates) -

    screen.fill(colors.WHITE)

    pygame.display.flip()

    # - constant game speed / FPS -

    clock.tick(FPS)

# - end -
pygame.quit()