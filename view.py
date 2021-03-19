#!/usr/bin/env python3
import paho.mqtt.client as mqtt  # import the client
import struct
import pygame
import colors
from tf import Pose

class QrMarker(pygame.rect.Rect):
    def __init__(self, left, top, width, height, ID):
        super().__init__(left, top, width, height)
        self.id = ID
        self.dragging = False
        
FPS = 30
pygame.init()
pygame.display.set_caption("Tracking System")

# Connect to broker
broker_address = "localhost"
client = mqtt.Client("Master")
client.connect(broker_address)

def distance(x1, y1, z1, x2, y2, z2):
    return round(((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)**.5, 2)

Length, Width, Height = 41.91, 55.88, 30  # cm

# Setup pygame
SCREEN_WIDTH = int(Width) * 10
W_R = Width / SCREEN_WIDTH
SCREEN_HEIGHT = int(Length) * 10
H_R = Length / SCREEN_HEIGHT
SCREEN_DEPTH = int(Height) * 10
D_R = Height / SCREEN_DEPTH

rectangle = pygame.rect.Rect(int(SCREEN_WIDTH/2), int(SCREEN_HEIGHT/2), 17, 17)
slider = pygame.rect.Rect(SCREEN_WIDTH - 17, 0, 17, 17)
slider_draging = False
rectangle_draging = False

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

clock = pygame.time.Clock()

running = True

qr_map_file = "qr_map.txt"

# Reading the global poses of each QR code
global_qr_poses = dict()
markers = []
print("[INFO] loading global marker position..")
with open(qr_map_file, "r") as fh:
    for line in fh:
        ID, pose = line.split(',', 1)
        x, y, rot = [int(i) for i in pose.strip()[1:-1].split(',')]
        global_qr_poses[int(ID)] = 1#Pose(x, y, rot)
        print(f"Marker {ID}: {global_qr_poses[int(ID)]}")


while running:
    # - events -
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                if rectangle.collidepoint(event.pos):
                    rectangle_draging = True
                    mouse_x, mouse_y = event.pos
                    offset_x = rectangle.x - mouse_x
                    offset_y = rectangle.y - mouse_y

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                rectangle_draging = False

        elif event.type == pygame.MOUSEMOTION:
            if rectangle_draging:
                mouse_x, mouse_y = event.pos
                rectangle.x = mouse_x + offset_x
                rectangle.y = mouse_y + offset_y
                client.publish('goal', f"{rectangle.x},{rectangle.y},90")
      
    # - draws (without updates) -

    screen.fill(colors.WHITE)

    pygame.draw.rect(screen, colors.RED, rectangle)
    pygame.draw.rect(screen, colors.BLACK, slider)

    pygame.display.flip()

    # - constant game speed / FPS -

    clock.tick(FPS)

# - end -
pygame.quit()