import time
import numpy as np
import cv2
from utils import rotate_about
def main(detector, explore_area = (500, 500), sample_freq=10, reset_time = 5):
    x, y, r = explore_area[0]//2, explore_area[1]//2, 0
    frame = np.zeros((*explore_area, 3))
    last_sample_time = time.time()
    while True: 
        curr_time = time.time()
        detector.update()
        d_t = curr_time - last_sample_time  
        if d_t >= (1/sample_freq):
            dx = detector.state_info["velocity"]["px"] * d_t
            dy = detector.state_info["velocity"]["py"] * d_t
            dr = detector.state_info["velocity"]["r"] * d_t
            r += dr
            tdx, tdy = rotate_about((dx, dy), (0, 0), r)
            x += tdx
            y += tdy
            cv2.circle(frame, (int(x),int(y)), 2, (255, 0, 0), -1) 
            last_sample_time = curr_time
        if d_t >= reset_time:
            pass
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
if __name__ == "__main__":
    from detector import Detector
    detector = Detector(debug = True)

    main(detector)