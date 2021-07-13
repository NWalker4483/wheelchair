import time
import cv2
from pyzbar.pyzbar import decode
from tf import Pose
from math import pi, acos
import numpy as np
import time
from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image

class Detector():
    def __init__(self, filename=None):

        self.camera = PiCamera()
        self.camera.start_preview()
        sleep(2)

        self.marker_pose = Pose()
        self.marker_id = None
        self.guide_pose = Pose()

        self.high_res_view = None
        self.low_res_view = None
        
        self.debug_info = dict()

    def getDebugView(self):
        frame = self.high_res_view

        if "marker_polygon" in self.debug_info:
            polygon = self.debug_info["marker_polygon"]
            # Draw Corners
            frame = cv2.circle(
                frame, (polygon[0].x, polygon[0].y), 2, (0, 255, 0), 5)
            frame = cv2.circle(
                frame, (polygon[1].x, polygon[1].y), 2, (0, 0, 255), 5)
            frame = cv2.circle(
                frame, (polygon[2].x, polygon[2].y), 2, (255, 0, 0), 5)
            frame = cv2.circle(
                frame, (polygon[3].x, polygon[3].y), 2, (255, 255, 0), 5)

        if "marker_pose" in self.debug_info and "marker_id" in self.debug_info: #TODO Rotate Marker ID to match marker rotation
            pose = self.debug_info["marker_pose"]
            # font
            font = cv2.FONT_HERSHEY_SIMPLEX
            # origin
            org = (int(pose.x), int(pose.y))
            fontScale = 3
            color = (0, 0, 255) # BGR
            # Using cv2.putText() method
            frame = cv2.putText(frame, str(self.debug_info["marker_id"]), org, font, fontScale,
                                color, 5, cv2.LINE_AA, False)
        return frame

    def update_views(self):
        # Create the in-memory stream
        stream = BytesIO()
        self.camera.capture(stream, format='jpeg')
        stream.seek(0)
        self.high_res_view = np.array(Image.open(stream));
        self.camera.capture(stream, resize=(320, 240))
        stream.seek(0)
        self.low_res_view = np.array(Image.open(stream));
        
    def update(self, debug = False):
        self.update_views()
        guide_pose = self.getGuideLinePosition()
        marker_id, marker_pose = self.checkForMarker()
        
        return guide_pose, marker_id, marker_pose

    def getGuideLinePosition(self):
        # Line Color Range
        lower_hsv = np.array([40, 46, 77])
        upper_hsv = np.array([144, 210, 227])
        
        frame = self.low_res_view
        frame = cv2.medianBlur(frame, 5)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        pnts = []
        self.avg_points = [(0,0)]
        self.mask_points = []
        # Separate the image into strips of height 20
        for y in range(0, mask.shape[0], 20):
            vals = []
            for i in range(0, 15, 5):  # Sample each of the strips four times
                for x in range(0, mask.shape[1], 5):
                    if mask[y ][x] > 125:
                        vals.append([x, y + i])
            if len(vals) > 0:
                pnts.append(np.mean(vals, axis=0))
            self.mask_points += vals
        self.avg_points += pnts
        print(self.avg_points)
        return Pose(*self.avg_points[0], rot=0)

    def checkForMarker(self):
        """find the two left side corners of a QR marker and return it to pose and ID"""
        def sss(d, e, f):
            """ This function solves the triangle and returns (d,e,f,D,E,F) """
            assert d + e > f and e + f > d and f + d > e
            F = acos((d**2 + e**2 - f**2) / (2 * d * e))
            E = acos((d**2 + f**2 - e**2) / (2 * d * f))
            D = pi - F - E
            return (d, e, f, D, E, F)

        def dist(x1, y1, x2, y2):
            return ((x1-x2)**2 + (y1-y2)**2)**.5
        frame = self.low_res_view
        barcodes = decode(frame)
        if len(barcodes) > 0:
            marker = barcodes[0]
            self.debug_info["marker_id"] = int(marker.data)
            self.debug_info["marker_polygon"] = marker.polygon
            p1 = marker.polygon[0]  # Upper Right
            p2 = marker.polygon[1]  # Lower Right
            p3 = marker.polygon[2]  # Upper Right
            p4 = marker.polygon[3]  # Lower Right
            a = dist(p1.x, p1.y, p2.x, p2.y) + 1e-5
            b = dist(p2.x, p2.y, frame.shape[1], p2.y) + 1e-5
            c = dist(p1.x, p1.y, frame.shape[1], p2.y) + 1e-5
            a, b, c, _, _, C = sss(a, b, c)
            rot = C * (180/pi)
            if p1.y > p2.y:  # Flip Quadrant if upside down
                rot = 180 + (180 - rot)
            return int(marker.data), Pose(*np.mean([p1, p2, p3, p4], axis=0), rot=rot)
        else:
            self.debug_info["marker_polygon"] = None
            return None, None

    def checkObstacleDetection(self):
        return [1000,1000,1000]
        # dists = []
        # for trigger_pin, echo_pin in zip(self.trigger_pins, self.echo_pins):
        #     # set Trigger to HIGH
        #     GPIO.output(trigger_pin, True)

        #     # set Trigger after 0.01ms to LOW
        #     time.sleep(0.00001)
        #     GPIO.output(trigger_pin, False)

        #     StartTime = time.time()
        #     StopTime = time.time()

        #     # save StartTime
        #     while GPIO.input(echo_pin) == 0:
        #         StartTime = time.time()

        #     # save time of arrival
        #     while GPIO.input(echo_pin) == 1:
        #         StopTime = time.time()

        #     # time difference between start and arrival
        #     TimeElapsed = StopTime - StartTime
        #     # multiply with the sonic speed (34300 cm/s)
        #     # and divide by 2, because there and back
        #     distance = (TimeElapsed * 34300) / 2

        #     dists.append(distance)
        # return dists


if __name__ == '__main__':
    det = Detector()
    while True:  # loop over the frames from the video stream
        det.update()
        cv2.imshow("Debug", det.getDebugView())
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
