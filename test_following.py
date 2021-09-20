from simple_pid import PID

def main(driver, detector, start_marker = 1, stop_marker = 2, drive_speed = 80, skip_start = False):
    pid = PID(.6,.0 ,.8,setpoint = 0)
    pid.sample_time = 1/10 # 10 Hz
    started = False
    if skip_start:
        started = True
    pid.output_limits = (-80, 80) 

    while True:
        detector.update()
        if detector.state_info.get("marker"):
            marker_id = detector.state_info["marker"]["id"]
            if not started and marker_id == (start_marker if start_marker != -1 else marker_id):
                started = True
                print(f"Started Line Following from {start_marker} to {stop_marker}...")
            
            if started and marker_id == (stop_marker if stop_marker != -1 else marker_id):
                print("Line Following Complete")
                driver.stop()
                break

        if started:
            line_info = detector.state_info.get("line_fused")
            if line_info != None:    
                m, b = line_info["slope"], line_info["bias"]
                
                i = m * (detector.low_res_shape[0]//1) + b
                i -= detector.low_res_shape[1]//2
                i /= detector.low_res_shape[1]//2
                i *= 100

                output = pid(i)
                
                driver.send_cmd(-drive_speed, output)
        else:
            print(f"Waiting to see start marker: ID = {start_marker}...")
if __name__ == "__main__":
    from driver import Driver
    from detector import Detector
    
    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector, 1, 2)