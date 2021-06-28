#!/usr/bin/env python3
import paho.mqtt.client as mqtt  
import pygame
import utils.colors as colors
import numpy as np

def on_connect(client, userdata, flags, rc):
    client.subscribe("tf")
          
def on_message(client, userdata, msg):
    if msg.topic == "tf":
        pass
FPS = 30
pygame.init()
pygame.display.set_caption("Tracking System")

# Connect to broker
broker_address = "192.168.0.4"
client = mqtt.Client("Master")
client.connect(broker_address)
client.on_connect = on_connect

client.on_message = on_message
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
    elif pressed[pygame.K_w]:
        client.publish(control_update_topic, "w")
        print("w")
    elif pressed[pygame.K_e]:
        client.publish(control_update_topic, "e")
        print("e")
    elif pressed[pygame.K_a]:
        client.publish(control_update_topic, "a")
        print("a")
    elif pressed[pygame.K_s]:
        client.publish(control_update_topic, "s")
        print("s")
    elif pressed[pygame.K_d]:
        client.publish(control_update_topic, "d")
        print("d")
    elif pressed[pygame.K_z]:
        client.publish(control_update_topic, "z")
        print("z")
    elif pressed[pygame.K_x]:
        client.publish(control_update_topic, "x")
        print("x")
    elif pressed[pygame.K_c]:
        client.publish(control_update_topic, "c")
        print("c")
    
      
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