import numpy as np
import cv2
#TODO: Draw some representation of rotation
from utils.math import rotate_about

def main(detector, explore_area = (1000, 1000), sample_freq = 10):
    detector.update_freq = sample_freq
    x, y = 0,0
    frame = np.zeros((*explore_area, 3))
    while True: 
        detector.update()
        cv2.circle(frame, (int(x),int(y)), 2, (255, 0, 0), -1) 
        x, y = detector.state_info["odom"]["px"], detector.state_info["odom"]["py"]
        x, y = x + (explore_area[0]//2), y+(explore_area[1]//2)
        detector.state_info["odom"]["r"]
        cv2.circle(frame, (int(x), int(y)), 2, (255, 255, 255), -1) 
        cv2.imshow("frame", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    from detector import Detector
    detector = Detector(filename = "L.avi", debug = True)
    main(detector)