import cv2

def main(driver, detector, record_frames= False, video_file = None, wait_for_trigger=True, lock_speed=0):
    """
      ## Connect
    while True:
        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            break
        except Exception as e:
            print("Failed to connect to joystick")
            print(e)
            pass
    if lock_speed != 0:
        if not wait_for_trigger:
            raise(AssertionError("In order to lock in a speed you must have wait for trigger set to true"))
    """
    if record_frames:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(video_file, fourcc, 30.0, detector.raw_res_shape[:2][::-1], True)    
    X = [] # Slope and Bias
    y = [] # Linear and angular control Input

    recording = True
    try:
        while recording:
            """
            lvalue = joystick.get_axis(1) # Flipped Linear
            lvalue *= -100
            
            avalue = joystick.get_axis(0) # Flipped Angular
            avalue *= 100
            """
            lvalue, avalue = 0, 0
            
            detector.update()
            slope, bias = 0,0#detector.state_info["line_fused"]
            check = True
            """
            # NOTE This line is required to grab joystick events like button presses
            pygame.event.get()
            if joystick.get_button(0) == 0 and wait_for_trigger:
                check = False
            """
            
            if check:
                if lock_speed != 0:
                    lvalue = lock_speed
                if record_frames:
                    frame = detector.raw_res_view
                    #frame = cv2.resize(frame, detector.raw_res_shape[:2][::-1])
                    out.write(frame)
                X.append([slope, bias])
                y.append([lvalue, avalue])
            # if joystick.get_button(1) > 0:
            #    recording = False
            driver.send_cmd(lvalue, avalue)
    finally:
        if record_frames:
            out.release()
            print("Saved")
    return X, y 
    
if __name__ == "__main__":
    from driver import Driver
    from detector import Detector

    driver = Driver()
    detector = Detector(debug = True)
    main(driver, detector, record_frames = True, video_file = 'out.avi', wait_for_trigger = False)