from simple_pid import PID

def main(driver, detector, start_marker = 1, stop_marker = 2, drive_speed = 80):
    pid = PID(.6,.0 ,.8,setpoint = 0)
    pid.sample_time = 1/10 # 10 Hz

    deadzone = 30
    overlap = 5
    started = False

    def clip(intercept_error):
         return intercept_error if abs(intercept_error) > 2 else 0
       
    pid.error_map = clip
    pid.output_limits = (0, 80) 

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
                m, b = detector.state_info.get("line_last")["slope"], detector.state_info.get("line_last")["bias"]
                i = m * (detector.low_res_shape[0]//1) + b

                i -= detector.low_res_shape[1]//2
                i /= detector.low_res_shape[1]//2
                i *= 100
                

                output = pid(-abs(i))

                """
                output = 0
                if abs(i) > 30:
                    output = 75
                elif abs(i) > 20:
                    output = 60
                elif abs(i) > 10:
                    output = 40
                """
                output = output if i > 0 else -output
                
                driver.send_cmd(-drive_speed, -output)
                
                print("Following Line...", output, abs(i))
        else:
            print(f"Waitng to see start marker: ID = {start_marker}...")
if __name__ == "__main__":
    from driver import Driver
    from detector import Detector
    import os
    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector, 1, 2)

"""

def main(driver, detector, start_marker = 1, stop_marker = 2, drive_speed = 80):
    started = False
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
                i = m * (detector.low_res_shape[0]//1) + b

                i -= detector.low_res_shape[1]//2
                i /= detector.low_res_shape[1]//2
                i *= 100

                output = 0
                def approx(x, samples):
                    for i in range(len(samples)):
                        x_, y_ = samples[i]
                        if x <= x_:
                            if i == 0:
                                return y_
                            else:
                                s = (y_ - samples[i - 1][1]) /  (x_ - samples[i - 1][0])
                                return samples[i - 1][1] + ((x_ - x) * s)
                    return samples[-1][1]
                            
                            
                output = approx(abs(i), [(2, 0), (5, 10), (10, 40),(15, 45), (20,60), (30, 75)])
                output = output if i < 0 else -output
                
                driver.send_cmd(-drive_speed, output)
                
                print("Following Line...", output, abs(i))
        else:
            print(f"Waitng to see start marker: ID = {start_marker}...")
if __name__ == "__main__":
    from driver import Driver
    from detector import Detector
    import os
    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector, 1, 2)

"""