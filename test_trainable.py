from sklearn import tree
import cv2
import pygame
import numpy as np 
pygame.init()

def record(detector, driver, record_frames= False, wait_for_trigger=True, lock_speed=0):
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
    
    if record_frames:
        pass
        #out = cv2.VideoWriter()
    
    X = [] # Slope and Bias
    y = [] # Control Input

    recording = True 
    while recording:
        lvalue = joystick.get_axis(1) # Flipped Linear
        lvalue *= -100
        
        avalue = joystick.get_axis(0) # Flipped Angular
        avalue *= 100
        
        detector.update()
    
        slope, bias = detector.state_info["line_fused"]
        check = True
        # NOTE This line is required to grab joystick events like button presses
        pygame.event.get()
        if joystick.get_button(0) == 0 and wait_for_trigger:
            check = False
        
        if check:
            if lock_speed != 0:
                lvalue = lock_speed
            if record_frames:
                pass
            X.append([slope, bias])
            y.append([lvalue, avalue])
        if joystick.get_button(1) > 0:
            recording = False
        driver.send_cmd(lvalue, avalue)
    clf = tree.DecisionTreeRegressor()
    return X, y 
    
def train(X, y):
    clf = tree.DecisionTreeRegressor()
    clf = clf.fit(X, y)
    pass

def test(detector, driver, model):
    while True:
        detector.update()

def main(detector, driver):
    x, y = record(detector, driver)
    model = train(x, y)
    test(model)
    
if __name__ == "__main__":
    from driver import Driver
    from detector import Detector

    driver = Driver()
    detector = Detector(debug = True)
    main(driver, detector)