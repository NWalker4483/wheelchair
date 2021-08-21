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
from threading import Thread

class Detector(Thread):
    def __init__(self, filename=None,debug=False):
        self.debug = debug
        super(Detector, self).__init__()
        self.daemeon = True
        
        self.lower_hsv = np.array([32, 42, 0])
        self.upper_hsv = np.array([60, 182, 255])
        self.high_res = (640, 480)
        self.low_res = (640, 480)
        
        self.high_res_view = None
        self.low_res_view = None

        self.debug_info = dict()

        self.camera_stream = VideoStream(usePiCamera = True)
        self.camera_stream.stream.camera.shutter_speed = 2000 # Drop to reduce Motion Blur
        self.camera_stream.start()
        
        sleep(2)  # Warm Up camera
        self.update_views()
    def run(self):
        while True:
            self.update()
    def stop(self):
        self.camera_stream.stop()
        
    def getDebugView(self):
        frame = self.high_res_view
  
        if all([i in self.debug_info.get("marker", dict()) for i in ["pose","id","polygon"]]):
            polygon = self.debug_info["marker"]["polygon"]
            pose = self.debug_info["marker"]["pose"]
            ID = self.debug_info["marker"]["id"]
            
            # Draw QR Code corners
            frame = cv2.circle(
                frame, (polygon[0].x, polygon[0].y), 12, (0, 255, 0), 5)
            frame = cv2.circle(
                frame, (polygon[1].x, polygon[1].y), 12, (0, 0, 255), 5)
            frame = cv2.circle(
                frame, (polygon[2].x, polygon[2].y), 12, (255, 0, 0), 5)
            frame = cv2.circle(
                frame, (polygon[3].x, polygon[3].y), 12, (255, 255, 0), 5)
            frame = cv2.circle(
                frame, (int(pose.x), int(pose.y)), 6, (255, 255, 255), 2)

            # TODO Rotate Marker ID to match marker rotation
          
            font = cv2.FONT_HERSHEY_SIMPLEX
            org = (int(pose.x), int(pose.y))
            fontScale = 2.5
            color = (0, 0, 255)
            frame = cv2.putText(frame, str(ID), org, font, fontScale,
                                color, 5, cv2.LINE_AA, False)

        if "mask_points" in self.debug_info:
            for x, y in self.debug_info["mask_points"]:
                x, y = int((x/self.low_res[0]) * self.high_res[0]
                           ), int((y/self.low_res[1]) * self.high_res[1])
                frame = cv2.circle(
                    frame, (int(x) , int(y)), 2, (165, 0, 55), 2)

        if "line_points" in self.debug_info:
            for x, y in self.debug_info["line_points"]:
                x, y = int((x/self.low_res[1]) * self.high_res[1]
                           ), int((y/self.low_res[0]) * self.high_res[0])
              
                frame = cv2.circle(
                    frame, (int(x), int(y)), 2, (0, 0, 255), 7)
        if "line" in self.debug_info:
            m, b = self.debug_info["line"]["slope"],self.debug_info["line"]["bias"]
            offset = abs(b) * (self.high_res[1] // 2)
            b = offset if b > 0 else -offset
            b += (self.high_res[1] // 2)
            
            # Calculate Start and stop
            p1 = (int(b),0)
            p2 = (int(m*self.high_res[1] + b), self.high_res[1])
            
            bias_color = (2,2,255) if b > frame.shape[1]//2 else (255,2,0)
            
            frame = cv2.line(frame, (p1[0], 2), (frame.shape[1]//2, 2), bias_color, 6)
            
            frame = cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]),(0,233,0),2)
            
            frame = cv2.line(frame, p1, p2,(0,233,243),6)
        if "zones" in self.debug_info:
            
            for p1, p2, is_green  in self.debug_info["zones"]:
                x,y = p1
                x, y = int((x/self.low_res[1]) * self.high_res[1]
                           ), int((y/self.low_res[0]) * self.high_res[0])
                p1 = (x,y )
                
                x,y = p2
                x, y = int((x/self.low_res[1]) * self.high_res[1]
                           ), int((y/self.low_res[0]) * self.high_res[0])
                p2 = (x,y )
                frame = cv2.rectangle(frame, p1, p2, color = (0,255,0) if is_green else (0,0,255), thickness = -1)
            
            x,y = self.debug_info["center"]
            x, y = int((x/self.low_res[1]) * self.high_res[1]
                           ), int((y/self.low_res[0]) * self.high_res[0])
            p = (x,y )
            frame = cv2.circle(
                frame, p, 6, (255, 0, 0), 2)
        return frame

    def update_views(self):
        frame = self.camera_stream.read()
        frame = imutils.resize(frame, width=450)
        self.high_res = frame.shape
        self.high_res_view = frame
        
        frame = imutils.resize(frame, width=200)
        self.low_res = frame.shape
        self.low_res_view = frame
        
        
    def update(self):
        self.update_views()
        marker_id, marker_pose = self.checkForMarker()
        
        if type(marker_pose) != type(None):
            angle = marker_pose.rot
            angle -= 180 if angle > 180 else 0
            
            min_ang_dist = lambda a, b: (b - a) if abs(a - b) < abs(360 - max(a,b) + min(a,b)) else (max(a,b) + min(a,b) - 360)
            
            base = min([0, 90, 180, 270], key = lambda x: abs(min_ang_dist(angle, x)))
            angle = -min_ang_dist(angle, base) 
            slope = atan(angle * (3.14/180))
            
            self.debug_info["line"] = dict()
            self.debug_info["line"]["slope"] = slope
            bias = marker_pose.x - (slope * marker_pose.y)
            self.debug_info["line"]["bias"] = (bias - (self.high_res_view.shape[1] // 2)) / (self.high_res_view.shape[1]//2)
            guide_pose = (self.debug_info["line"]["slope"], self.debug_info["line"]["bias"]) 
        else:
            guide_pose = self.getGuideLinePosition()
        self.checkZones()
        if self.debug:
            cv2.imshow("Debug", self.getDebugView())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass
        return guide_pose, marker_id, marker_pose
    
    def checkZones(self):

        frame = self.low_res_view

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        
        start = 0
        stop = 0
        zones = [] # ((x1,y1),(), bool)
        for stop in np.linspace(0, frame.shape[1], 25):
            stop = int(stop) 
            p1 = (start,int(mask.shape[0] * (1 - .15)) - 50)
            p2 = (stop, mask.shape[0] - 50)
            crop = mask[p1[1]:p2[1], p1[0]: p2[0]]
            val = np.sum(crop > 125) > (crop.shape[0]*crop.shape[1]) * .8
            
            if 30 > start or start > 180:
                val = False
            start = stop
            
            zones.append((p1, p2, val))
            
        self.debug_info["zones"] = zones
        start = 0
        stop = 0
        curr_start = 0
        best_val = 0
        curr_val = 0
        for i, zone in enumerate(self.debug_info["zones"] + [((None),(None),False)]):
            p1, p2, is_green = zone
            if is_green:
                if curr_val == 0:
                    curr_start = i
                curr_val += 1
            elif curr_val > best_val:
                stop = i - 1
                start = curr_start
                best_val = curr_val
            else:
                curr_val = 0
        start, stop = self.debug_info["zones"][start],self.debug_info["zones"][stop]
        a, _, _ = start
        _, b, _ = stop
        p = (a[0] + ((b[0] - a[0]) //2), a[1] + ((b[1] - a[1]) // 2))
        
        self.debug_info["center"] = p
            
        
    def getGuideLinePosition(self):

        frame = self.low_res_view

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        pnts = []
        avg_points = []
        mask_points = []
        # Separate the image into strips of height 20
        for y in range(0, mask.shape[0], 20):
            vals = []
            for i in range(0, 15, 5):  # Sample each of the strips four times
                for x in range(0, mask.shape[1], 5):
                    if mask[y][x] > 125:
                        #if x > int(.1 * self.low_res[0]) and x < int(.8 * self.low_res[0]):
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
    det = Detector(debug = True)
    try:
        while True:  # loop over the frames from the video stream
            s=time.time()
            det.update()
            print("FPS: ", 1.0 / (time.time() - s))
    except KeyboardInterrupt as e:
        print("Manual ShutOff")
    finally:
        det.stop()
