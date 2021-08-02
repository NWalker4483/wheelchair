from __future__ import print_function
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
from picamera.array import PiRGBArray
from PIL import Image
# import the necessary packages
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import cv2
class Detector():
    def __init__(self, filename=None):
        self.high_res = (640, 480)
        self.low_res = (640, 480)

        self.high_res_view = None
        self.low_res_view = None

        self.debug_info = dict()

        self.camera_stream = WebcamVideoStream(src=0).start()
        sleep(2)  # Warm Up camera
        
        self.update_views()
    def stop(self):
        self.camera_stream.stop()
    def getDebugView(self):
        frame = self.high_res_view
        if type(frame) == type(None):
            import numpy as np
            return np.zeros((10,10))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if "marker_polygon" in self.debug_info:
            polygon = self.debug_info["marker_polygon"]
            # Draw Corners
            frame = cv2.circle(
                frame, (polygon[0].x, polygon[0].y), 12, (0, 255, 0), 5)
            frame = cv2.circle(
                frame, (polygon[1].x, polygon[1].y), 12, (0, 0, 255), 5)
            frame = cv2.circle(
                frame, (polygon[2].x, polygon[2].y), 12, (255, 0, 0), 5)
            frame = cv2.circle(
                frame, (polygon[3].x, polygon[3].y), 12, (255, 255, 0), 5)
            # cv2.line()

        # TODO Rotate Marker ID to match marker rotation
        if all([i in self.debug_info for i in ["marker_pose","marker_id"]]):

            pose = self.debug_info["marker_pose"]
            # font
            font = cv2.FONT_HERSHEY_SIMPLEX
            # origin
            org = (int(pose.x), int(pose.y))
            fontScale = 5
            color = (0, 0, 255)  # BGR
            # Using cv2.putText() method
            frame = cv2.putText(frame, str(self.debug_info["marker_id"]), org, font, fontScale,
                                color, 5, cv2.LINE_AA, False)

        if "mask_points" in self.debug_info:
            for x, y in self.debug_info["mask_points"]:
                x, y = int((x/self.low_res[0]) * self.high_res[0]
                           ), int((y/self.low_res[1]) * self.high_res[1])
                frame = cv2.circle(
                    frame, (int(x) , int(y)), 2, (255, 0, 255), 2)

        if "line_points" in self.debug_info:
            for x, y in self.debug_info["line_points"]:
                x, y = int((x/self.low_res[1]) * self.high_res[1]
                           ), int((y/self.low_res[0]) * self.high_res[0])
              
                frame = cv2.circle(
                    frame, (int(x), int(y)), 2, (0, 0, 255), 7)
        if "line_form" in self.debug_info:
            m, b = self.debug_info["line_form"]
            p1 = (int(b),0)
            p2 = ( int(m*self.low_res[1] + b), self.low_res[1])
            
            p1 = (int((p1[0]/self.low_res[0]) * self.high_res[0]),0)
            p2 = (int((p2[0]/self.low_res[0]) * self.high_res[0]),self.high_res[1])
            
            frame = cv2.line(frame, p1, p2,(0,233,243),6)

        return frame

    def update_views(self):
        frame = self.camera_stream.read()
        frame = imutils.resize(frame, width=325)
        self.low_res_view = frame
        self.high_res_view = frame
        # self.first = True
    def update(self):
        
        self.update_views()
        marker_id, marker_pose = self.checkForMarker()
  
        guide_pose = self.getGuideLinePosition()

        return guide_pose, marker_id, marker_pose

    def getGuideLinePosition(self, clip_at = None):
        # Line Color Range
        lower_hsv = np.array([40, 46, 77])
        upper_hsv = np.array([144, 210, 227])

        frame = self.low_res_view

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        pnts = []
        avg_points = []
        mask_points = []
        # Separate the image into strips of height 20
        for y in range(0, mask.shape[0], 20):
            vals = []
            for i in range(0, 15, 5):  # Sample each of the strips four times
                for x in range(0, mask.shape[1], 5):
                    if mask[y][x] > 125:
                        if x > int(.1 * self.low_res[0]) and x <int(.8 * self.low_res[0]):
                            if clip_at != None:
                                pass
                            vals.append([x, y + i])
            if len(vals) > 0:
                pnts.append(np.mean(vals, axis=0))
            mask_points += vals
        avg_points += pnts

        mask_points = np.array(mask_points)
        avg_points = np.array(avg_points)

        self.debug_info["mask_points"] = mask_points
        self.debug_info["line_points"] = avg_points
        # avg_points = mask_points
        if len(avg_points) > 3:
            m, b = np.polyfit(avg_points[:,1], avg_points[:,0], 1)
        else:
            m, b = 0, 0
        self.debug_info["line_form"] = (m, b)
        return (m, b)

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
        frame = self.high_res_view
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
                
                
            self.debug_info["marker_pose"] = Pose(*np.mean([p1, p2, p3, p4], axis=0), rot=rot)
            return int(marker.data), self.debug_info["marker_pose"]
        else:
            if "marker_polygon" in self.debug_info:
                del self.debug_info["marker_polygon"]
                del self.debug_info["marker_pose"]
                del self.debug_info["marker_id"]
            return None, None


if __name__ == '__main__':
    import time
    det = Detector()
    try:
        while True:  # loop over the frames from the video stream
            s=time.time()
            det.update()
            print("FPS: ", 1.0 / (time.time() - s))
            cv2.imshow("Debug", det.getDebugView())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt as e:
        print("Manual ShutOff")
    finally:
        det.stop()
