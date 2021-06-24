from wheelchair.driver import Driver
from wheelchair.detector import Detector

driver = Driver('/dev/ttyUSB0')
detector = Detector()
while True:
    driver.face(0, detector, 1)