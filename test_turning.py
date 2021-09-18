from utils.math import angle_between, rotate_about, sign, turn_clockwise
from utils.map import parse_direction, direction2qr_rotation
from simple_pid import PID
import time
import numpy as np

def main(driver, detector, marker_id = 0, direction = "bottom", tolerance = 7, hold_time = 1):
    print(f"Started QR Facing Test {marker_id}")
    pid = PID(170, 60, 45)
    pid.setpoint = 0
    pid.sample_time = 1/10 # 10 Hz
    pid.output_limits = (-100, 100) 
    
    direction = parse_direction(direction)
    goal_rotation = direction2qr_rotation(direction)

    started = False
    current_rotation = 0
    start_hold = 0

    error = tolerance # TODO: Use other visual ques to determine rotation
    last_error = 0
    running = True
    
    while running:
        if abs(error) <= np.deg2rad(tolerance):
            if (time.time() - start_hold) > hold_time:
                running = False
                break
        else:
            start_hold = time.time()
            
        detector.update()
        marker_data = detector.state_info.get("marker", None)
        if marker_data != None:
            if marker_data["id"] == marker_id:
                last_pose_rotation = detector.state_info["odom"]["r"]
                marker_rotation = marker_data["r"]
                current_rotation = marker_rotation
                started = True
        else:
            if started:
                # TODO Test ... like at all
                new_pose_rotation = detector.state_info["odom"]["r"]
                vector_1 = rotate_about((0,1), (0,0), last_pose_rotation)
                vector_2 = rotate_about((0,1), (0,0), new_pose_rotation)
                
                dr = angle_between(vector_1, vector_2)
                dr = dr if turn_clockwise(vector_1, vector_2) else -dr

                current_rotation = marker_rotation - dr
                print(f"Predicted Rotation {np.rad2deg(current_rotation)}")
            else:
                print("waiting to first start marker")
        if started:
            # TODO: Swap in utils.math.turn_clockwise function to consolidata 
            v1 = rotate_about((0,1),(0,0),goal_rotation)
            v2 = rotate_about((0,1),(0,0),current_rotation)
            d1 = rotate_about((0,1),(0,0),goal_rotation + np.deg2rad(90))
            d2 = rotate_about((0,1),(0,0),goal_rotation - np.deg2rad(90))
            
            a1 = angle_between(v1, v2)
            if angle_between(v2,d1) > angle_between(v2,d2):
                a1 = -a1
            error = a1
            
            if sign(last_error) != sign(error):
                pid.reset()
            last_error = error
            control = pid(error)
            driver.send_cmd(0, control)
            
    driver.stop()

if __name__ == "__main__":
    from driver import Driver
    from detector import Detector

    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector, 1, "right")

