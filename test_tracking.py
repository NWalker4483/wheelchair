import numpy as np
import cv2
from utils import rotate_about

def main(detector, explore_area = (500, 500), sample_freq = 10):
    x, y, r = explore_area[0]//2, explore_area[1]//2, 0
    detector.setOdom(x, y, r)
    detector.update_freq = sample_freq
    frame = np.zeros((*explore_area, 3))
    while True: 
        detector.update()
        cv2.circle(frame, (int(x),int(y)), 2, (255, 0, 0), -1) 
        x, y = detector.state_info["odom"]["px"], detector.state_info["odom"]["py"]
        #TODO: Draw some representation of rotation
        detector.state_info["odom"]["r"]
        cv2.circle(frame, (int(x), int(y)), 2, (255, 255, 255), -1) 
        cv2.imshow("frame", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    from detector import Detector
    detector = Detector(debug = True)
    main(detector)