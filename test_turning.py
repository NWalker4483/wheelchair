from utils.math import  min_ang_dist
from utils.map import parse_direction, direction2qr_rotation
from simple_pid import PID
import numpy as np

def main(driver, detector, marker_id = 0, direction = "bottom", tolerance = 5, max_det_gap = 500):
    pid = PID(.125, .025, .25)
    pid.setpoint = 0
    pid.sample_time = 1/10 # 10 Hz
    pid.output_limits = (-100, 100) 
    
    direction = parse_direction(direction)
    goal_rotation = direction2qr_rotation(direction)

    gap = 0
    
    # TODO: Use other visual ques to determine rotation 
    while abs(angle_error) > tolerance:
        detector.update()
        # if gap >= max_det_gap:
        #     driver.stop()
        #     raise Exception("Tracking lost For too long task failed")
        if detector.state_info.get("marker"):
            if detector.state_info.get("marker")["id"] == marker_id:
                gap = 0
                angle_error = -min_ang_dist(np.rad2deg(detector.state_info.get("marker")["r"]), goal_rotation)
                control = pid(angle_error)
                driver.send_cmd(control, 0)
            else:
                gap += 1
    driver.stop()

if __name__ == "__main__":
    from driver import Driver
    from detector import Detector

    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector)

