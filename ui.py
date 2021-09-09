#!/usr/bin/env python3
import socket
import utils.colors as colors
from tkinter import *
from functools import partial
import pygame

app = Tk()

## initialize pygame and joystick
pygame.init()
UDP_IP = "192.168.0.2"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

def publish(topic, data):
    global sock
    cmd_str = f"#{topic}/{data};"
    print(cmd_str)
    sock.sendto(cmd_str.encode("utf-8"), (UDP_IP, UDP_PORT))


control_update_topic = 'c'
goal_update_topic = 'g'

symbols = ["\\","|","/","<-","*","->","/","|","\\"]

dirs = [(100,100),(100,0),(100,-100),
        (0,100),(0,0),(0,-100),
        (100,-100),(-100,0),(-100,100)]

for i, (symbol, dir) in enumerate(zip(symbols, dirs)):
    x, y = dir
    Button(app, text=symbol,command=partial(publish, control_update_topic,"%d, %d" % (x, y))).grid(row=i//3, column=i%3)

for i in range(9):
    Button(app, text=str(i + 1),command=partial(publish, goal_update_topic,"%d" % (i + 1))).grid(row=i//3, column=((i%3) + 3))

joystick = None
def joystick_check():
    global joystick

    pygame.joystick.quit()
    pygame.joystick.init()
    # create a new joystick object from
    joystick_count = pygame.joystick.get_count()
    if joystick_count > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    if not joystick_count: 
        if not joystick:
            pass
            # print("reconnect")
        # joystick_check()
    else:
        joystick

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    app.update_idletasks()
    app.update()
    try:
        lvalue = joystick.get_axis(1) # Flipped Linear
        lvalue *= -100
        avalue = joystick.get_axis(0) # Flipped Angular
        avalue *= 100
        if (abs(avalue) > 5) or (abs(lvalue) > 5):
            publish(control_update_topic, f"{int(lvalue)}, {int(avalue)}")
    except Exception as e:
        print(type(e), e)
        joystick_check()