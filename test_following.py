from simple_pid import PID

def main(driver, detector, start_marker = 1, stop_marker = 2, drive_speed = 70):
    pid = PID(45, 0, 25.5)
    pid.setpoint = 0
    pid.sample_time = 1/10 # 10 Hz

    deadzone = 30 
    overlap = 5
    started = False

    # def clip(intercept_error):
    #     return intercept_error if abs(intercept_error) > .1 else 0
       
    # pid.error_map = clip
    pid.output_limits = (deadzone, 100) 

    while True:
        detector.update()

        if detector.state_info.get("marker"):
            marker_id = detector.state_info["marker"]["id"]
            if not started and marker_id == (start_marker if start_marker != -1 else marker_id):
                started = True
            if started and marker_id == (stop_marker if stop_marker != -1 else marker_id):
                print("Line Complete")
                driver.stop()
                return

        if started:
            if detector.state_info.get("line_fused"):    
                m, b = detector.state_info.get("line_fused")["slope"], detector.state_info.get("line_fused")["bias"]
                i = m * detector.low_res_shape[0] + b

                i -= detector.low_res_shape[1]//2
                i /= detector.low_res_shape[1]//2

                output = pid(abs(i))
                output = output if output > (deadzone + overlap) else 0
                
                # s = 75
                # t = 0
                # if abs(i) > .30:
                #     t = 75
                # elif abs(i) > .20:
                #     t = 60
                # elif abs(i) > .10:
                #     t = 50
                # t = t if i > 0 else -t
                
                driver.send_cmd(output, drive_speed)
                print("Following Line...")
        else:
            print(f"Waitng to see start marker: ID = {start_marker}...")
if __name__ == "__main__":
    from driver import Driver
    from detector import Detector
    import os
    driver = Driver(os.environ.get("JOYSTICK_PORT", '/dev/ttyACM0'))
    detector = Detector(debug = True)

    main(driver, detector)