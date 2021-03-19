#!/usr/bin/env python3
import paho.mqtt.client as mqtt
from imutils.video import VideoStream
import imutils
import time

from driver import Driver
from planner import Planner

from tf import TFManager, Pose
from detect import getGuideLinePosition, checkForMarker

global_qr_poses = dict()

planner = Planner()
driver = Driver()

test = True
if test:
    qr_map_file = "qr_map.txt"
    # Reading the global poses of each QR code
    print("[INFO] loading global marker position..")
    with open(qr_map_file, "r") as fh:
        for line in fh:
            ID, pose = line.split(',', 1)
            x, y, rot = [int(i) for i in pose.strip()[1:-1].split(',')]
            global_qr_poses[int(ID)] = Pose(x, y, rot)
            print(f"Marker {ID}: {global_qr_poses[int(ID)]}")

base_frame = 'base_link'
odom_frame = 'odom'
map_frame = 'map'

map_update_topic = 'map_update'
goal_update_topic = 'goal'

# This is the Subscriber
def on_connect(client, userdata, flags, rc):
    client.subscribe(map_update_topic)
    client.subscribe(goal_update_topic)
    print("Connected with result code " + str(rc))

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

manager.create_frame(map_frame)
manager.create_frame(odom_frame)
manager.create_frame(base_frame)

manager.init_tf(map_frame, odom_frame)
manager.init_tf(odom_frame, base_frame)

client = mqtt.Client()
client.connect("localhost", 1883, 60)

client.on_connect = on_connect
client.on_message = on_message

# initialize the video stream, sensors, etc
print("[INFO] starting video stream...")
vs = VideoStream(0).start()
time.sleep(2.0)

try:
    while True:  # loop over the frames from the video stream
        manager.publish(client=client)
        client.loop(timeout=.1)
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        manager.set_tf(driver.pose_estimate, odom_frame, base_frame)

        # Get the position rotation and ID for any markers within view
        # * The sensor readings are always in the base frame frame
        ID, local_marker_pose = checkForMarker(frame)
        if ID:
            # Convert the measurement to be within the map frame
            global_marker_pose_estimate = manager.get_tf(
                map_frame, base_frame) + local_marker_pose
            manager.set_tf(
                global_qr_poses[ID] - global_marker_pose_estimate, map_frame, odom_frame)

        # Update the planner with the most current global position estimate
        new_cmd = planner.update(frame, manager.get_tf(map_frame, base_frame))
        driver.update(new_cmd)
except Exception as e:
    raise(e)
finally:
    print("[INFO] cleaning up...")
    client.disconnect()
    vs.stop()
