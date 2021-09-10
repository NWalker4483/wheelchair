from sklearn import tree
import pygame
pygame.init()

def record(detector, driver, sample_count = 500):
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

    X = [] # Slope and Bias
    y = [] # Control Input

    input("Press Enter to Start")
    while (len(y) < sample_count):
        lvalue = joystick.get_axis(1) # Flipped Linear
        lvalue *= -100
        
        avalue = joystick.get_axis(0) # Flipped Angular
        avalue *= 100
        
        _input = avalue
        detector.update()
    
        slope, bias = detector.state_info["line_fused"]
        # NOTE This line is required to grab joystick events like button presses
        pygame.event.get()
        if joystick.get_button(0) > 0:
            driver.send_cmd(80, avalue) # Keep Linear Speed consistent when recording
            if not (slope == 0 and bias == 0): # Good Detection and Trigger Pressed
                X.append([slope, bias])
                y.append(_input)
        else:
            driver.send_cmd(lvalue, avalue)
    clf = tree.DecisionTreeRegressor()
    clf = clf.fit(X, y)
    print(clf.predict([[0,0]]))
    
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