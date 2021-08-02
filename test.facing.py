from driver import Driver
from detector import Detector
import cv2

driver = Driver('/dev/ttyACM0')
detector = Detector()# 'test_data/green.mp4')
while True:
    cv2.imshow("Debug", detector.getDebugView())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    driver.face(direction = 1, detector = detector, ID = 1)