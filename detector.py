from imutils.video import VideoStream
from pyzbar.pyzbar import decode
from threading import Thread
from math import pi, acos, atan
from tf import Pose
import numpy as np
import imutils
import time
import cv2

def QrPose2LineForm(pose, high_res_shape):
    angle = pose.rot
    angle -= 180 if angle > 180 else 0
    
    min_ang_dist = lambda a, b: (b - a) if abs(a - b) < abs(360 - max(a,b) + min(a,b)) else (max(a,b) + min(a,b) - 360)
    
    base = min([0, 90, 180, 270], key = lambda x: abs(min_ang_dist(angle, x)))
    angle = -min_ang_dist(angle, base) 
    slope = atan(angle * (3.14/180))

    bias = pose.x - (slope * pose.y)
    bias = (bias - (high_res_shape[1] // 2)) / (high_res_shape[1]//2)
    return slope, bias

def scale_from(x, y, res_1, res_2):
    x, y = int((x/res_1[0]) * res_2[0]
            ), int((y/res_1[1]) * res_2[1])
    return x, y
class Detector(Thread):
    def __init__(self, debug=False):
        super(Detector, self).__init__()
        self.debug = debug
        ####### Image Masking
        self.lower_green = np.array([32, 42, 0])
        self.upper_green = np.array([60, 182, 255])
        ###### Motion Tracking 
        self.lk_params = dict(winSize  = (15, 15),
                maxLevel = 2,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.feature_params = dict( maxCorners = 500,
                    qualityLevel = 0.3,
                    minDistance = 7,
                    blockSize = 7)
        # NEW
        self.velocity_estimate = 0
        self.update_freq = 10 # Hz
        
        self.odom = [0, 0, 0] # x, y, rot
        self.s_len = 2.5 # meters
        self.pps = 680

        ########
        self.high_res_shape = None
        self.low_res_shape = None
        
        self.high_res_view = None
        self.low_res_view = None

        self.state_info = dict()

        self.camera_stream = VideoStream(usePiCamera = False)
        #self.camera_stream.stream.camera.shutter_speed = 2000 # Drop shutter speed to reduce motion blur
        self.camera_stream.start()
        
        time.sleep(2)  # Warm Up camera
        self.readCameraFeed()

        self.daemeon = True
        self.__alive = True
        # self.start()

    def run(self):
        while self.__alive:
            self.update()

    def stop(self):
        self.__alive = False
        self.camera_stream.stop()      

    def getDebugView(self):
        frame = self.high_res_view
  
        if all([i in self.state_info.get("marker", dict()) for i in ["pose","id","polygon"]]):
            polygon = self.state_info["marker"]["polygon"]
            pose = self.state_info["marker"]["pose"]
            ID = self.state_info["marker"]["id"]
            
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

        if "mask_points" in self.state_info:
            for x, y in self.state_info["mask_points"]:
                x, y = scale_from(x, y, self.low_res_shape, self.high_res_shape)
                frame = cv2.circle(
                    frame, (x , y), 2, (165, 0, 55), 2)

        if "line_points" in self.state_info:
            for x, y in self.state_info["line_points"]:
                x, y = scale_from(x, y, self.low_res_shape,self.high_res_shape)
      
                frame = cv2.circle(
                    frame, (x, y), 2, (0, 0, 255), 7)
        if "current_tracks" in self.state_info:
            cv2.polylines(frame, [np.int32(list(map(lambda p: scale_from(p[0], p[1], self.low_res_shape, self.high_res_shape), tr))) for tr in self.state_info["current_tracks"]], False, (0, 255, 0))
                
        if "line" in self.state_info:
            m, b = self.state_info["line"]["slope"], self.state_info["line"]["bias"]
            offset = abs(b) * (self.high_res_shape[1] // 2)
            b = offset if b > 0 else -offset
            b += (self.high_res_shape[1] // 2)
            
            # Calculate Start and stop
            p1 = (int(b),0)
            p2 = (int(m * self.high_res_shape[1] + b), self.high_res_shape[1])
            
            bias_color = (2,2,255) if b > frame.shape[1]//2 else (255,2,0)
            
            frame = cv2.line(frame, (p1[0], 2), (frame.shape[1]//2, 2), bias_color, 6)
            

            frame = cv2.line(frame, p1, p2,(0,233,243),6)
 
        if "zones" in self.state_info:
            
            for p1, p2, is_green  in self.state_info["zones"]:
                x, y = scale_from(p1[0], p1[1], self.low_res_shape, self.high_res_shape)
                p1 = (x,y)
                
                x,y = p2
                x, y = int((x/self.low_res_shape[1]) * self.high_res_shape[1]
                           ), int((y/self.low_res_shape[0]) * self.high_res_shape[0])
                p2 = (x,y )
                frame = cv2.rectangle(frame, p1, p2, color = (0,255,0) if is_green else (0,0,255), thickness = -1)
            
            x,y = self.state_info["center"]
            x, y = int((x/self.low_res_shape[1]) * self.high_res_shape[1]
                           ), int((y/self.low_res_shape[0]) * self.high_res_shape[0])
            p = (x,y )
            frame = cv2.circle(
                frame, p, 6, (255, 0, 0), 2)
        
        # Draw Center Line
        frame = cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]),(0,233,0),2)
            
        return frame

    def estimateLineForm(self, bias_only = False):
        if "line" in self.state_info:
            del self.state_info["line"]
        # Returns an estimate of the lines slope and Bias relative to the front of the camera frame
        frame = self.low_res_view

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        pnts = []
        avg_points = []
        mask_points = []
        # Separate the image into patches 
        patch_height = 50 #px
        patch_width = 25 #px
        sample_cnt = 5

        # https://stackoverflow.com/questions/12864445/how-to-convert-the-output-of-meshgrid-to-the-corresponding-array-of-points
        Y_, X_ = np.mgrid[0:mask.shape[0]:patch_height, 0:mask.shape[1]:patch_width]
        # patch_positions = zip(*np.vstack([X_.ravel(), Y_.ravel()]))
        
        y_, x_ = np.mgrid[0:patch_height:patch_height//sample_cnt, 0:patch_width:patch_width//sample_cnt]
        # sample_positions = zip(*np.vstack([x_.ravel(), y_.ravel()]))

        for X, Y in zip(*np.vstack([X_.ravel(), Y_.ravel()])):
            vals = []
            for x, y in zip(*np.vstack([x_.ravel(), y_.ravel()])):
                if Y+y >= mask.shape[0] or X+x >= mask.shape[1]:
                    break
                if mask[Y+y][X+x] == 255:
                    vals.append([X+x, Y+y])
            if len(vals) > 3:
                pnts.append(np.mean(vals, axis=0))
            mask_points += vals
        avg_points += pnts

        self.state_info["mask_points"] = mask_points
        self.state_info["line_points"] = avg_points

        if len(avg_points) > 3:
            avg_points = np.array(avg_points)
            m, b = np.polyfit(avg_points[:,1], avg_points[:,0], 1)
            b -= mask.shape[1]//2
            b /= mask.shape[1]//2

            self.state_info["line"] = dict()
            self.state_info["line"]["slope"], self.state_info["line"]["bias"] = m, b

    def estimateBiasOnly(self):
        frame = self.low_res_view
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
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
            
        self.state_info["zones"] = zones
        start = 0
        stop = 0
        curr_start = 0
        best_val = 0
        curr_val = 0
        for i, zone in enumerate(self.state_info["zones"] + [((None),(None),False)]):
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
        start, stop = self.state_info["zones"][start],self.state_info["zones"][stop]
        a, _, _ = start
        _, b, _ = stop
        p = (a[0] + ((b[0] - a[0]) //2), a[1] + ((b[1] - a[1]) // 2))
        
        self.state_info["center"] = p

    def estimateVelocity(self):
        frame = self.low_res_view
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.state_info.get("current_tracks"):
            img0, img1 = self.state_info["last_gray"], gray
            p_c = np.float32([tr[-1] for tr in self.state_info.get("current_tracks")]).reshape(-1, 1, 2)
            p_f, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p_c, None, **self.lk_params)
            p_r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p_f, None, **self.lk_params)
            d = abs(p_c - p_r).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            for tr, (x, y), good_flag in zip(self.state_info["current_tracks"], p_f.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > 10:
                    del tr[0] # PopLeft
                new_tracks.append(tr)
            if len(new_tracks) > 3:
                self.state_info["current_tracks"] = new_tracks
            else:
                del self.state_info["current_tracks"]
          
            self.state_info["last_gray"] = gray
        
            if len(self.state_info.get("current_tracks")) > 9:
                pass

        mask = np.zeros_like(gray)
        mask[:] = 255

        if self.state_info.get("current_tracks"):
            # Mark latest position in each track as an roi for features to track 
            for x, y in [np.int32(tr[-1]) for tr in self.state_info.get("current_tracks")]:
                cv2.circle(mask, (x, y), 5, 0, -1)
        else:
            self.state_info["current_tracks"] = [ ]

        p = cv2.goodFeaturesToTrack(gray, mask = mask, **self.feature_params)
        if p is not None: # New feature Positions
            for x, y in np.float32(p).reshape(-1, 2):
                self.state_info["current_tracks"].append([(x, y)])
        
        self.state_info["last_gray"] = gray
        
    def readCameraFeed(self):
        frame = self.camera_stream.read()
        frame = imutils.resize(frame, width=450)
        self.high_res_shape = frame.shape
        self.high_res_view = frame
        
        frame = imutils.resize(frame, width=200)
        self.low_res_shape = frame.shape
        self.low_res_view = frame
    
    def update(self):
        self.readCameraFeed()
        self.checkForMarker()

        if self.state_info.get("marker"):
            self.state_info["line"] = dict()
            self.state_info["line"]["slope"], self.state_info["line"]["bias"] = QrPose2LineForm(self.state_info["marker"]["pose"], self.high_res_shape)
        else:
            self.estimateLineForm()
        self.estimateVelocity()
        if False:
            self.estimateBiasOnly()

        if self.debug:
            cv2.imshow("Debug", self.getDebugView())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass

    def checkForMarker(self):
        if "marker" in self.state_info:
            del self.state_info["marker"]
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
            try:
                self.state_info["marker"] = dict()
                self.state_info["marker"]["polygon"] = marker.polygon
                self.state_info["marker"]["id"] = int(marker.data)
            except ValueError as ve:
                self.state_info["marker"]["id"] = 0
                print("Invalid code read defaulting to id = 0 for DEV")
            
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
            self.state_info["marker"]["pose"] = Pose(*np.mean([p1, p2, p3, p4], axis=0), rot=rot)
            

if __name__ == '__main__':
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
