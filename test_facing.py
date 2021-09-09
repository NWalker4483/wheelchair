from utils import get_direction, min_ang_dist
from simple_pid import PID

def main(driver, detector, marker_id = 0, direction = "bottom", tolerance = 5, max_det_gap = 500):
    direction = get_direction(direction)
    goal_rotations = {0: 270, 1: 90, 2: 180, 3: 0}
    angle_error = 10e5
    gap = 0

    pid = PID(.125, .025, .25)
    pid.setpoint = 0
    pid.sample_time = 1/10 # 10 Hz
    pid.output_limits = (-100, 100)  # Output will always be above 0, but with no upper bound

    while abs(angle_error) > tolerance:
        if gap >= max_det_gap:
            driver.stop()
            raise Exception("Tracking lost For too long task failed")
        detector.update()
        if detector.state_info.get("marker"):
            if detector.state_info.get("marker")["id"] == marker_id:
                gap = 0
                # Flip Left and Right
                angle_error = min_ang_dist(detector.state_info.get("marker")["pose"].rot, goal_rotations[direction])
                control = -pid(angle_error)
                driver.send_cmd(control, 0)
            else:
                gap += 1
    driver.stop()

if __name__ == "__main__":
    from driver import Driver
    from detector import Detector

    driver = Driver('/dev/ttyACM0')
    detector = Detector(debug = True)

    main(driver, detector)