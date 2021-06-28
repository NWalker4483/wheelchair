#!/usr/bin/env python3
import paho.mqtt.client as mqtt
from driver import Driver
from detector import Detector

driver = Driver('/dev/ttyACM0')
detector = Detector(0)

control_update_topic = 'control_update'
goal_update_topic = 'goal'

def on_connect(client, userdata, flags, rc):
    client.subscribe(control_update_topic)
    client.subscribe(goal_update_topic)


def on_message(client, userdata, msg):
    global map_update_topic
    global driver
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

client = mqtt.Client()
client.connect('localhost', 1883, 60)

client.on_connect = on_connect
client.on_message = on_message
running = False
try:
    while True:  # loop over the frames from the video stream
        client.loop(timeout=.1)
        if running:
            pass
except Exception as e:
    raise(e)
finally:
    print('[INFO] cleaning up...')
    client.disconnect()
    driver.stop()
