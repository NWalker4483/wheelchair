#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import imutils
import time

from driver import Driver
from detector import Detector
from planner import Planner

from tf import TFManager, Pose

global_qr_poses = dict()

driver = Driver()
detector = Detector('data/green.mp4')
planner = Planner()

test = True
if test:
    qr_map_file = 'qr_map.txt'
    # Reading the global poses of each QR code
    print('[INFO] loading global marker position..')
    with open(qr_map_file, 'r') as file_handle:
        for line in file_handle:
            ID, pose = line.split(',', 1)
            x, y, rot = [int(i) for i in pose.strip()[1:-1].split(',')]
            global_qr_poses[int(ID)] = Pose(x, y, rot)
            print(f'Marker {ID}: {global_qr_poses[int(ID)]}')

odom_frame = 'odom'

map_update_topic = 'map_update'
goal_update_topic = 'goal'


def on_connect(client, userdata, flags, rc):
    client.subscribe(map_update_topic)
    client.subscribe(goal_update_topic)
    print('Connected with result code ' + str(rc))


def on_message(client, userdata, msg):
    global global_qr_poses
    global map_update_topic
    global planner
    if msg.topic == map_update_topic:
        print(msg.payload.decode())
    elif msg.topic == goal_update_topic:
        print(msg.payload.decode())
        planner.set_goal(Pose())

manager = TFManager()

manager.create_frame('map')
manager.create_frame('odom')
manager.create_frame('base_link')

manager.init_tf('map', 'odom')
manager.init_tf('odom', 'base_link')

client = mqtt.Client()
client.connect('localhost', 1883, 60)

client.on_connect = on_connect
client.on_message = on_message

try:
    a=0
    while True:  # loop over the frames from the video stream
        if a % 25 == 0:
            manager.publish(client=client)
        a+=1
        client.loop(timeout=.1)

        # [QR ID, Pose(), Guide Pose(), [Distance Senses...]]
        sense_states = detector.update()

        ID, local_marker_pose = detector.update()
        if local_marker_pose != None:
            # Convert the measurement to be within the map frame
            global_marker_pose_estimate = manager.get_tf(
                'map', 'base_link') 
            global_marker_pose_estimate.rot += local_marker_pose.rot
            # manager.set_tf(
            #     global_qr_poses[ID] - global_marker_pose_estimate, 'map', 'odom')
            pose = global_qr_poses[ID]
            pose.rot += local_marker_pose.rot
            pose.rot %= 360
            manager.set_tf(pose, 'odom', 'base_link')
            print(manager.get_tf('map', 'base_link'))

        new_cmd = planner.update(manager)
        driver.send_cmd(*new_cmd)

        
except Exception as e:
    raise(e)
finally:
    print('[INFO] cleaning up...')
    client.disconnect()
