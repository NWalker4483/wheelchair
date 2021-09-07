from driver import Driver
from detector import Detector
import cv2
driver = Driver()
detector = Detector()
first_seen = -1
while True:
    line_form, marker_id, _ = detector.update()
    if marker_id != None:
        if first_seen == -1:
            first_seen = marker_id
        if marker_id != first_seen:
            print("Line Complete")
            driver.stop()
            break
    cv2.imshow("Debug", detector.getDebugView())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if first_seen != -1:
        driver.adjust_to_line(*line_form, drive_speed = 80)
        print("Following Line...")
    else:
        print("Waitng to see start marker...")