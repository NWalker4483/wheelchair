from driver import Driver
from detector import Detector

driver = Driver('/dev/ttyACM0')
detector = Detector()# 'test_data/green.mp4')
while True:
    driver.face(direction = 1, detector = detector, ID = 2)