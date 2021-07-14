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

        self.high_res = (512, 384)
        self.low_res = (320, 240)

        self.high_res_view = None
        self.low_res_view = None

        self.debug_info = dict()

        self.camera = PiCamera(sensor_mode = 5)
        sleep(2)  # Warm Up camera
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

        # TODO Rotate Marker ID to match marker rotation
        if "marker_pose" in self.debug_info and "marker_id" in self.debug_info:
            pose = self.debug_info["marker_pose"]
            # font
            font = cv2.FONT_HERSHEY_SIMPLEX
            # origin
            org = (int(pose.x), int(pose.y))
            fontScale = 3
            color = (0, 0, 255)  # BGR
            # Using cv2.putText() method
            frame = cv2.putText(frame, str(self.debug_info["marker_id"]), org, font, fontScale,
                                color, 5, cv2.LINE_AA, False)

        if "mask_points" in self.debug_info:
            for x, y in self.debug_info["mask_points"]:
                x, y = int((x/self.low_res[0]) * self.high_res[0]
                           ), int((y/self.low_res[1]) * self.high_res[1])
                frame = cv2.circle(
                    frame, (int(x/2) , int(y/2) ), 2, (255, 0, 255), 2)

        if "line_points" in self.debug_info:
            for x, y in self.debug_info["line_points"]:
                x, y = int((x/self.low_res[1]) * self.high_res[1]
                           ), int((y/self.low_res[0]) * self.high_res[0])
                x/=2
                y/=2
                frame = cv2.circle(
                    frame, (int(x), int(y)), 2, (0, 0, 255), 7)


        return frame

    def update_views(self):
        self.stream = BytesIO()
        self.camera.capture(self.stream, resize=self.high_res, format='jpeg', use_video_port=True)
        self.stream.seek(0)
        self.high_res_view = np.array(Image.open(self.stream))

        self.camera.capture(self.stream, resize=self.low_res, format='jpeg', use_video_port=True)
        self.stream.seek(0)
        self.low_res_view = np.array(Image.open(self.stream))

    def update(self):
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
        avg_points = [(0, 0)]
        mask_points = []
        # Separate the image into strips of height 20
        for y in range(0, mask.shape[0], 20):
            vals = []
            for i in range(0, 15, 5):  # Sample each of the strips four times
                for x in range(0, mask.shape[1], 5):
                    if mask[y][x] > 125:
                        vals.append([x, y + i])
            if len(vals) > 0:
                pnts.append(np.mean(vals, axis=0))
            mask_points += vals
        avg_points += pnts

        mask_points = np.array(mask_points)
        avg_points = np.array(avg_points)

        self.debug_info["mask_points"] = mask_points
        self.debug_info["line_points"] = avg_points
 
        m, b = np.polyfit(avg_points[:,1], avg_points[:,0], 1) # y, x
        print(m,b)
        return Pose(0,0, rot=0)

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
            if "marker_polygon" in self.debug_info:
                del self.debug_info["marker_polygon"]
            return None, None


if __name__ == '__main__':
    det = Detector()
    while True:  # loop over the frames from the video stream
        det.update()
        cv2.imshow("Debug", det.getDebugView())
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
