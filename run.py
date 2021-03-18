# import the necessary packages
from imutils.video import VideoStream
import imutils
import time
import cv2

from tf import TFManager, Pose
from detect import getGuideLinePosition, checkForMarker

qr_map_file = "qr_map.txt"

# Reading the global poses of each QR code
global_qr_poses = dict()
print("[INFO] loading global marker position..")
with open(qr_map_file, "r") as fh:
    for line in fh:
        ID, pose = line.split(',',1)
        x, y, rot = [int(i) for i in pose.strip("()").split(',')]
        global_qr_poses[int(ID)] = Pose(x, y, rot)
        print(f"Marker {ID}: {global_qr_poses[int(ID)]}")

from planner import Planner # The planner sends commands to the driver and stores the actual position of the robot
from driver import Driver # The driver controls the motors and stores an internal estimate of where the base is in 2D space

base_frame = 'base_link'
odom_frame = 'odom'
map_frame = 'map'

manager = TFManager()

manager.create_frame(map_frame)
manager.create_frame(odom_frame)
manager.create_frame(base_frame)

manager.init_tf(map_frame, odom_frame)
manager.init_tf(odom_frame, base_frame)

planner = Planner()
driver = Driver()

# initialize the video stream, sensors, etc
print("[INFO] starting video stream...")
vs = VideoStream(0).start()
time.sleep(2.0)

while True: # loop over the frames from the video stream
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    manager.set_tf(driver.pose_estimate, odom_frame, base_frame)

    # Get the position rotation and ID for any markers within view 
    # * The sensor readings are always in the base frame frame
    ID, local_marker_pose = checkForMarker(frame)
    if ID:
        # Convert the measurement to be within the map frame 
        global_marker_pose_estimate = manager.get_tf(map_frame, base_frame) + local_marker_pose
        manager.set_tf(global_qr_poses[ID] - global_marker_pose_estimate, map_frame, odom_frame)

    new_cmd = planner.update(frame, manager.get_tf(map_frame, base_frame)) # Update the planner with the most current global position estimate
    driver.update(new_cmd)
# close the output CSV file do a bit of cleanup
print("[INFO] cleaning up...")
cv2.destroyAllWindows()
vs.stop()
