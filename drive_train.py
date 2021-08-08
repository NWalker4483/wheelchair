from detector import Detector
from sklearn import tree
import pygame
from driver import Driver
pygame.init()
def connect():
    pass

def record(sample_count = 500):
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
    ## Connect P2
    drive = Driver('/dev/ttyACM0') 
    X = [] # Slope and Bias
    y = [] # Control Input
    det = Detector(debug = True)
    input("Press Enter to Start")
    while (len(y) < sample_count):
        lvalue = joystick.get_axis(1) # Flipped Linear
        lvalue *= -100;
        
        avalue = joystick.get_axis(0) # Flipped Angular
        avalue *= 100;
        
        _input = avalue
        line, _, _ = det.update()
    
        slope, bias = line
        pygame.event.get()
        if joystick.get_button(0) > 0:
            print(avalue)
            drive.send_cmd(80, avalue) # Keep Linear Speed consistent when recording
            if not (slope == 0 and bias == 0): # Good Detection and Trigger Pressed
                X.append([slope, bias])
                y.append(_input)
        else:
            drive.send_cmd(lvalue, avalue)
    clf = tree.DecisionTreeRegressor()
    clf = clf.fit(X, y)
    print(clf.predict([[0,0]]))
    
def train():
    pass

def main():
    record()
    
if __name__ == "__main__":
    main()