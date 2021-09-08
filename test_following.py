from driver import Driver
from detector import Detector
import cv2
from simple_pid import PID

def main():
    driver = Driver('/dev/ttyACM0')
    detector = Detector(debug = True)
    
    first_seen = -1
    pid = PID(45, 27, 25.5)
    pid.setpoint = 0
    pid.sample_time = 1/10# 10 Hz
    
    pid.output_limits = (-100, 100)  # Output will always be above 0, but with no upper bound

    while True:
        detector.update()
        
        if detector.state_info.get("marker"):
            marker_id = detector.state_info["marker"]["id"]
            if first_seen == -1:
                first_seen = marker_id
            if marker_id == 1:
                print("Line Complete")
                driver.stop()
                break
        if first_seen != -1:
            if detector.state_info.get("line"):    
                m, b = detector.state_info.get("line")["slope"], detector.state_info.get("line")["bias"]
                i = m * detector.low_res_shape[0] + b
                i -= detector.low_res_shape[1]//2
                i /= detector.low_res_shape[1]//2
                print(m, b, i)
                s = 75
                t = 0
                if abs(i) > .30:
                    t = 75
                elif abs(i) > .20:
                    t = 60
                elif abs(i) > .10:
                    t = 50
                t = t if i > 0 else -t
                
                driver.send_cmd(t,s)
                
                #driver.adjust_to_line(*line_form, drive_speed = 80)
                print("Following Line...")
        else:
            print("Waitng to see start marker...")
if __name__ == "__main__":
    main()