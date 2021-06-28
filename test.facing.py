from driver import Driver
from detector import Detector

driver = Driver('/dev/cu.usbmodem14301')
detector = Detector()# 'test_data/green.mp4')
while True:
    driver.face(0, detector, 1)