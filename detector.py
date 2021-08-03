from __future__ import print_function
import time
import cv2
from pyzbar.pyzbar import decode
from tf import Pose
from math import pi, acos, atan
import numpy as np
import time
from io import BytesIO
from time import sleep
from picamera import PiCamera
from picamera.array import PiRGBArray
from PIL import Image
from imutils.video import VideoStream
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

        self.camera_stream = VideoStream(usePiCamera = True)
        
        #self.camera_stream.stream.shutter_speed = 10
        #self.camera_stream.stream.awb_mode = 'off'
        #self.camera_stream.stream.iso = 0
        #self.camera_stream.stream.framerate = 30
        
        #self.camera_stream.stream.sharpness = 100
        
        self.camera_stream.stream.camera.shutter_speed = 8000
        #self.camera_stream.stream.camera.exposure_mode = 'sports'
        self.camera_stream.start()
        
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

        if all([i in self.debug_info.get("marker", dict()) for i in ["pose","id","polygon"]]):
            polygon = self.debug_info["marker"]["polygon"]
            pose = self.debug_info["marker"]["pose"]
            ID = self.debug_info["marker"]["id"]
            
            # Draw Polygon Corners
            frame = cv2.circle(
                frame, (polygon[0].x, polygon[0].y), 12, (0, 255, 0), 5)
            frame = cv2.circle(
                frame, (polygon[1].x, polygon[1].y), 12, (0, 0, 255), 5)
            frame = cv2.circle(
                frame, (polygon[2].x, polygon[2].y), 12, (255, 0, 0), 5)
            frame = cv2.circle(
                frame, (polygon[3].x, polygon[3].y), 12, (255, 255, 0), 5)
            frame = cv2.circle(
                frame, (int(pose.x), int(pose.y)), 12, (255, 255, 255), 2)

            # TODO Rotate Marker ID to match marker rotation
          
            font = cv2.FONT_HERSHEY_SIMPLEX
            org = (int(pose.x), int(pose.y))
            fontScale = 5
            color = (0, 0, 255)
            frame = cv2.putText(frame, str(ID), org, font, fontScale,
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
        if "line" in self.debug_info:
            m, b = self.debug_info["line"]["slope"],self.debug_info["line"]["bias"]
            b += 1
            b *= self.high_res[1] // 2
            
            # Calculate Start and stop
            p1 = (int(b),0)
            p2 = (int(m*self.high_res[1] + b), self.high_res[1])
            
            bias_color = (2,2,255) if b > frame.shape[1]//2 else (255,2,0)
            
            frame = cv2.line(frame, (p1[0], 2), (frame.shape[1]//2, 2), bias_color, 6)
            
            frame = cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]),(0,233,0),2)
            
            frame = cv2.line(frame, p1, p2,(0,233,243),6)

        return frame

    def update_views(self):
        frame = self.camera_stream.read()
        frame = imutils.resize(frame, width=350)
        self.high_res = frame.shape
        self.high_res_view = frame
        
        frame = imutils.resize(frame, width=200)
        self.low_res = frame.shape
        self.low_res_view = frame
        
        
    def update(self):
        self.update_views()
        marker_id, marker_pose = self.checkForMarker()
        
        if type(marker_pose) != type(None):
            #TODO: fix logic
            self.debug_info["line"] = dict()
            angle = marker_pose.rot
            angle -= 180 if angle > 180 else 0
            angle = angle - 90 # TODO:
            slope = atan(angle * (3.14/180))
    
            self.debug_info["line"]["slope"] = slope
            bias = marker_pose.x - (slope * marker_pose.y)
            print(bias)
            #bias = 200
            self.debug_info["line"]["bias"] = (bias - (self.high_res_view.shape[1] // 2)) / (self.high_res_view.shape[1]//2)
            print(self.debug_info["line"]["bias"])
            guide_pose = (self.debug_info["line"]["slope"], self.debug_info["line"]["bias"])
        else:
            guide_pose = self.getGuideLinePosition()
        return guide_pose, marker_id, marker_pose

    def getGuideLinePosition(self, clip_at = None):
        # Line Color Range
        lower_hsv = np.array([27, 103, 103])
        upper_hsv = np.array([120, 202, 220])

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
                            vals.append([x, y + i])
            if len(vals) > 0:
                pnts.append(np.mean(vals, axis=0))
            mask_points += vals
        avg_points += pnts

        mask_points = np.array(mask_points)
        avg_points = np.array(avg_points)

        self.debug_info["mask_points"] = mask_points
        self.debug_info["line_points"] = avg_points
        if len(avg_points) > 3:
            m, b = np.polyfit(avg_points[:,1], avg_points[:,0], 1)
            b -= frame.shape[1]//2
            b /= frame.shape[1]//2
        else:
            m, b = 0, 0 
        self.debug_info["line"] = dict()
        self.debug_info["line"]["slope"], self.debug_info["line"]["bias"] = m, b
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
        marker = next(filter(lambda code: code.type == "QRCODE", barcodes), None)
        if marker != None:
            self.debug_info["marker"] = dict()
            self.debug_info["marker"]["id"] = int(marker.data)
            self.debug_info["marker"]["polygon"] = marker.polygon
            
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
                
            self.debug_info["marker"]["pose"] = Pose(*np.mean([p1, p2, p3, p4], axis=0), rot=rot)
            
            return self.debug_info["marker"]["id"], self.debug_info["marker"]["pose"]
        else:
            if "marker" in self.debug_info:
                del self.debug_info["marker"]
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
