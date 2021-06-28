#!/usr/bin/env python3
import paho.mqtt.client as mqtt

driver = Driver()
detector = Detector('test_data/green.mp4')

odom_frame = 'odom'

map_update_topic = 'map_update'
goal_update_topic = 'goal'


def on_connect(client, userdata, flags, rc):
    client.subscribe(map_update_topic)
    client.subscribe(goal_update_topic)


def on_message(client, userdata, msg):
    global global_qr_poses
    global map_update_topic
    global planner
    if msg.topic == map_update_topic:
        print(msg.payload.decode())
    elif msg.topic == goal_update_topic:
        print(msg.payload.decode())
        planner.set_goal(Pose())

client = mqtt.Client()
client.connect('localhost', 1883, 60)

client.on_connect = on_connect
client.on_message = on_message

try:
    a=0
    while True:  # loop over the frames from the video stream
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
    
        new_cmd = planner.update(manager)
        driver.send_cmd(*new_cmd)

        
except Exception as e:
    raise(e)
finally:
    print('[INFO] cleaning up...')
    client.disconnect()
