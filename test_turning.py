from utils.math import angle_between, rotate_about
from utils.map import parse_direction, direction2qr_rotation
from simple_pid import PID
import numpy as np

def main(driver, detector, marker_id = 0, direction = "bottom", tolerance = 5, hold_time = 1):
    print(f"Started QR Facing Test {marker_id}")
    pid = PID(140, 60, 30)
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
        if abs(angle_error) < np.deg2rad(tolerance):
            #TODO Use time.time()
            held_for += 1
        if held_for > (hold_time * 24):
            running = False
            break
            
        detector.update()
        marker_data = detector.state_info.get("marker")
        if not isinstance(marker_data, type(None)):
            if marker_data["id"] == marker_id:
                last_pose_rotation = detector.state_info["odom"]["r"]
                marker_rot = marker_data["r"]
                current_rotation = marker_rot
                started = True
        else:
            if started:
                new_pose_rotation = detector.state_info["odom"]["r"]
                current_rotation = marker_rot #+ min_rad_dist(last_pose_rotation, new_pose_rotation
            else:
                print("waiting to start")
        if started:
            v1 = rotate_about((0,1),(0,0),goal_rotation)
            v2 = rotate_about((0,1),(0,0),current_rotation)
            d1 = rotate_about((0,1),(0,0),goal_rotation + np.deg2rad(90))
            d2 = rotate_about((0,1),(0,0),goal_rotation - np.deg2rad(90))
            
            a1 = angle_between(v1, v2)
            if angle_between(v2,d1) > angle_between(v2,d2):
                a1 = -a1
            angle_error = a1
            control = pid(angle_error)
            driver.send_cmd(0, control)
    driver.stop()

if __name__ == "__main__":
    from driver import Driver
    from detector import Detector

    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector, 1, "right")

