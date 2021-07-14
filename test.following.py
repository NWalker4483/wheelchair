from driver import Driver
from detector import Detector
import cv2
driver = Driver('/dev/ttyACM0')
detector = Detector()# 'test_data/green.mp4')
first_seen = -1
while True:
    line_form, marker_id, _ = detector.update()
    if marker_id != None:
        if first_seen == -1:
            first_seen = marker_id
        if marker_id != first_seen:
            print("Line Complete")
            break
    cv2.imshow("Debug", detector.getDebugView())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if first_seen != -1:
        driver.adjust_to_line(*line_form)
    else:
        print("Waitng to see start marker...")