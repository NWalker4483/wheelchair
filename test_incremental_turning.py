from utils.map import parse_direction, direction2qr_rotation
from simple_pid import PID
import time
import numpy as np

def main(driver, detector, marker_id = 0, turns = 1, direction = "right", turn_speed = 70):
    print(f"Started QR Facing Test {marker_id}")
    direction = parse_direction(direction)
    turn_speed = -turn_speed if direction == 2 else turn_speed
    running = True
    started = False
    turns_made = 0
    
    while running:
        detector.update()
        if not started:
            marker_data = detector.state_info.get("marker", None)
            if marker_data != None:
                if marker_data["id"] == marker_id:
                    started = True
        else:
            driver.send_cmd(0, turn_speed)
    driver.stop()

if __name__ == "__main__":
    from driver import Driver
    from detector import Detector

    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector, 1, "right")

