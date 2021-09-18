import numpy as np
import cv2
#TODO: Draw some representation of rotation
from utils.math import rotate_about

def main(detector, explore_area = (1500, 1500), sample_freq = 10):
    detector.update_freq = sample_freq
    x, y = 0,0
    p1 = (explore_area[0]//2, explore_area[0]//2)
    p2 = (p1[0], p1[1] + 100)
    frame = np.zeros((*explore_area, 3))
    while True: 
        detector.update()
        cv2.line(frame, p1, p2, (5,5,5), 10)
        cv2.circle(frame, (int(x),int(y)), 8, (255, 0, 0), -1) 
        
        x, y = detector.state_info["odom"]["x"], detector.state_info["odom"]["y"]
        x, y = x + (explore_area[0]//2), y+(explore_area[1]//2)
        p1 = (x, y)
        p2 = (p1[0], p1[1] + 100)
        p2 = rotate_about(p2, p1, detector.state_info["odom"]["r"])
        p1, p2 = (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1]))
        cv2.line(frame, p1,p2, (0,0,255), 10)
        cv2.circle(frame, (int(x), int(y)), 8, (255, 255, 255), -1) 
        cv2.imshow("frame", cv2.resize(frame, (500,500)))
        cv2.waitKey(1)

if __name__ == "__main__":
    from detector import Detector
    detector = Detector(filename="R.avi", debug = True)
    main(detector)