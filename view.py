#!/usr/bin/env python3
import socket
import pygame
import utils.colors as colors

def text_to_screen(screen, text, x, y, size = 50,
            color = (200, 000, 000), font_type = 'data/fonts/orecrusherexpand.ttf'):
    try:

        text = str(text)
        font = pygame.font.Font(font_type, size)
        text = font.render(text, True, color)
        screen.blit(text, (x, y))

    except Exception as e:
        print('Font Error, saw it coming')
        raise(e)

FPS = 30
pygame.init()
pygame.display.set_caption("Control System")
screen = pygame.display.set_mode((500, 500))
clock = pygame.time.Clock()

UDP_IP = "192.168.0.2"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

def publish(topic, data):
    sock.sendto(topic.encode("utf-8")+b"/" +
                data.encode("utf-8"), (UDP_IP, UDP_PORT))

running = True

control_update_topic = 'control_update'
goal_update_topic = 'goal_update'

joystick = pygame.joystick.Joystick(0)
joystick.init()
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if joystick != None:
        lvalue = joystick.get_axis(1) # Flipped Linear
        lvalue *= -100;
        avalue = joystick.get_axis(0) # Flipped Angular
        avalue *= 100;

        publish(control_update_topic, f"{int(lvalue)}, {int(avalue)}")

    pressed = pygame.key.get_pressed()
    pressed = [i for i in range(len(pressed)) if pressed[i]]
    active = False
    for key in pressed:
        key_val = chr(key)
        if key_val in "qweasdzxc":
            publish(control_update_topic, key_val)
            
        elif (key_val in "1234567890"):
            publish(goal_update_topic, key_val)
            print(f"Goal Set to : {key_val}")
    if not active:
        publish('idle', "0")
    # pygame.display.flip()
    # - constant game speed / FPS -
    clock.tick(FPS)

# - end -
pygame.quit()
