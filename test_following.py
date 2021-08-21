from driver import Driver
from detector import Detector
import cv2
from simple_pid import PID

def main():
    driver = Driver('/dev/ttyACM0')
    detector = Detector(debug=True)
    
    first_seen = -1
    pid = PID(45, 27, 25.5)
    pid.setpoint = 0
    pid.sample_time = 1/10# 10 Hz
    
    pid.output_limits = (-100, 100)  # Output will always be above 0, but with no upper bound

    while True:c
        line_form, marker_id, _ = detector.update()
        if marker_id != None:
            if first_seen == -1:
                first_seen = marker_id
            if marker_id == 1:
                print("Line Complete")
                driver.stop()
                break
        if first_seen != -1:
            b = detector.debug_info["center"][0]
            b -= detector.low_res_view.shape[1]//2
            b /= detector.low_res_view.shape[1]//2
            t = pid(-b)
            deadzone = 10
            t += deadzone if t > 0 else -deadzone
            print(b,t)
            driver.send_cmd(80, t)
            
            #driver.adjust_to_line(*line_form, drive_speed = 80)
            print("Following Line...")
        else:
            print("Waitng to see start marker...")
if __name__ == "__main__":
    main()