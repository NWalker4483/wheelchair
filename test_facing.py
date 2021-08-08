from driver import Driver
from detector import Detector
import cv2
def main(ID = 0):
    driver = Driver('/dev/ttyACM0')
    detector = Detector(debug=True)
    while True:
        driver.face(direction = "bottom", detector = detector, ID = ID)
if __name__ == "__main__":
    main()