from simple_pid import PID

def main(driver, detector, start_marker = 1, stop_marker = 2, drive_speed = 80):
    pid = PID(.6,.0 ,.8,setpoint = 0)
    pid.sample_time = 1/10 # 10 Hz
    started = False
    pid.output_limits = (-80, 80) 

    while True:
        detector.update()
        if detector.state_info.get("marker"):
            marker_id = detector.state_info["marker"]["id"]
            if not started and marker_id == (start_marker if start_marker != -1 else marker_id):
                started = True
                print("Following Line...")
            if started and marker_id == (stop_marker if stop_marker != -1 else marker_id):
                print("Line Complete")
                driver.stop()
                break

        if started:
            if detector.state_info.get("line_fused"):    
                m, b = detector.state_info.get("line_fused")["slope"], detector.state_info.get("line_fused")["bias"]
                
                i = m * (detector.low_res_shape[0]//1) + b
                print(m,b,i)
                i -= detector.low_res_shape[1]//2
                i /= detector.low_res_shape[1]//2
                i *= 100

                output = pid(i)
                
                driver.send_cmd(-drive_speed, output)
        else:
            print(f"Waitng to see start marker: ID = {start_marker}...")
if __name__ == "__main__":
    from driver import Driver
    from detector import Detector
    
    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector, 1, 2)