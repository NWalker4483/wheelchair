from utils.math import  min_ang_dist, min_rad_dist
from utils.map import parse_direction, direction2qr_rotation
from simple_pid import PID
import numpy as np

def main(driver, detector, marker_id = 0, direction = "bottom", tolerance = 5, hold_time = 1):
    pid = PID(10, .21285, 8)
    pid.setpoint = 0
    pid.sample_time = 1/10 # 10 Hz
    pid.output_limits = (-100, 100) 
    
    direction = parse_direction(direction)
    goal_rotation = direction2qr_rotation(direction)

    started = False
    current_rotation = 0
    held_for = 0
    
    est = 0
    # TODO: Use other visual ques to determine rotation

    angle_error = tolerance
    running = True
    
    while running:
        if abs(angle_error) < tolerance:
            #TODO Use time.time()
            held_for += 1
        if held_for > (hold_time * 24):
            running = False
            break
            
        detector.update()
        
        if detector.state_info.get("marker"):
            if detector.state_info.get("marker")["id"] == marker_id:
                last_pose_rotation = detector.state_info["odom"]["r"]
                marker_rot = detector.state_info.get("marker")["r"]
                current_rotation = marker_rot
                started = True
        else:
            if started:
                new_pose_rotation = detector.state_info["odom"]["r"]
                current_rotation = marker_rot #+ min_rad_dist(last_pose_rotation, new_pose_rotation
            else:
                print("waiting to start")
        if started:
            angle_error = -min_ang_dist(np.rad2deg(current_rotation), goal_rotation)
            print(np.rad2deg(current_rotation),angle_error)
            control = pid(angle_error)
            driver.send_cmd(0, control)
    driver.stop()

if __name__ == "__main__":
    from driver import Driver
    from detector import Detector

    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector, 4, "left")

