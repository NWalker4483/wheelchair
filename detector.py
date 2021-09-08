from imutils.video import VideoStream
from pyzbar.pyzbar import decode
from threading import Thread
from math import pi, acos, atan
from tf import Pose
import numpy as np
import imutils
import time
import cv2
from test_fuse import predict_newline, draw_line, line_to_points, points_to_line

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
    x, y = int((x/res_1[0]) * res_2[0]), int((y/res_1[1]) * res_2[1])
    return x, y

class Detector(Thread):
    def __init__(self, debug=False):
        super(Detector, self).__init__()
        self.debug = debug
        self.state_info = dict()
        self.iter_num = 0 

        ####### Image Masking
        self.lower_green = np.array([22, 50 , 25])
        self.upper_green = np.array([70, 255, 255])
        
        ####### Line Estimation
        # Separate the image into patches 
        self.patch_height = 35 #px
        self.patch_width = 35 #px
        self.sample_cnt = 3 # * WARNING This value is squared 

        # https://stackoverflow.com/questions/12864445/how-to-convert-the-output-of-meshgrid-to-the-corresponding-array-of-points
        Y_, X_ = np.mgrid[0:149:self.patch_height, 0:200:self.patch_width]
        self.patch_positions = list(zip(*np.vstack([X_.ravel(), Y_.ravel()])))
        
        y_, x_ = np.mgrid[0:self.patch_height:self.patch_height//self.sample_cnt, 0:self.patch_width:self.patch_width//self.sample_cnt]
        self.sample_positions = list(zip(*np.vstack([x_.ravel(), y_.ravel()])))

        ###### Motion Tracking 
        self.lk_params = dict(winSize  = (15, 15),
                maxLevel = 2,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        
        self.feature_params = dict( maxCorners = 500,
                    qualityLevel = 0.3,
                    minDistance = 7,
                    blockSize = 7)
        
        self.state_info["velocity"] = dict()
        self.state_info["velocity"]["px"] = 0
        self.state_info["velocity"]["py"] = 0
        self.state_info["velocity"]["r"] = 0

        ###### Sensor Fusion
        self.initialized = False
        self.last_sample_time = time.time()
        self.alpha = .8
        
        self.state_info["odom"] = dict()
        self.state_info["odom"]["px"] = 0
        self.state_info["odom"]["py"] = 0
        self.state_info["odom"]["r"] = 0

        ######
        self.high_res_shape = None
        self.low_res_shape = None
        
        self.high_res_view = None
        self.low_res_view = None

        self.camera_stream = VideoStream(usePiCamera = False)
        # self.camera_stream.stream.camera.shutter_speed = 2000 # Drop shutter speed to reduce motion blur
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
  
        if "marker" in self.state_info:
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
            frame = cv2.putText(frame, str(ID), (int(pose.x), int(pose.y)),
                                cv2.FONT_HERSHEY_SIMPLEX, 2.5,
                                (0, 0, 255), 5, cv2.LINE_AA, False)

        if "mask_points" in self.state_info:
            for x, y in self.state_info["mask_points"]:
                x, y = scale_from(x, y, self.low_res_shape, self.high_res_shape)
                frame = cv2.circle(frame, (x , y), 2, (165, 0, 55), 2)

        if "line_points" in self.state_info:
            for x, y in self.state_info["line_points"]:
                x, y = scale_from(x, y, self.low_res_shape, self.high_res_shape)
                frame = cv2.circle(frame, (x, y), 2, (0, 0, 255), 7)

        if "current_tracks" in self.state_info:
            cv2.polylines(frame, [np.int32(list(map(lambda p: scale_from(p[0], p[1], self.low_res_shape, self.high_res_shape), tr))) for tr in self.state_info["current_tracks"]], False, (0, 255, 0))
        
        if "line" in self.state_info:
            m, b = self.state_info["line"]["slope"], self.state_info["line"]["bias"]
        
            p1, p2 = line_to_points(m, b)
            p1,p2 = list(p1), list(p2)
            #p1[0] += self.low_res_shape[1] //2
            #p2[0] += self.low_res_shape[1] //2 
            p1 = scale_from(*p1, self.low_res_shape,self.high_res_shape)
            p2 = scale_from(*p2, self.low_res_shape, self.high_res_shape)
            m, b = points_to_line(p1,p2)
            print(m,b)
            frame = draw_line(frame, m, b, color=(0,233,243))

        if "line_fused" in self.state_info:
            m, b = self.state_info["line_fused"]["slope"], self.state_info["line_fused"]["bias"]
        
            b = scale_from(int(b),0, self.low_res_shape, self.high_res_shape)[0]

            bias_color = (2,2,255) if b > frame.shape[1]//2 else (255,2,0)

            frame = cv2.line(frame, (b, 2), (frame.shape[1]//2, 2), bias_color, 6)

            frame = draw_line(frame, m, b, color=(255 ,233,243))
                     
        if "true_center" in self.state_info:
            p0 = self.state_info["true_center"]
            cv2.circle(
                frame,scale_from(*p0,self.low_res_shape,self.high_res_shape), 6, (255, 0, 0), 10)
        
        # Draw Center Line
        frame = cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]),(0,233,0),2)
        
        if "largest_contour" in self.state_info:
            c=self.state_info["largest_contour"]
            cv2.drawContours(frame, c, -1, (255,0,0),10)
            # determine the most extreme points along the contour
            extLeft = tuple(c[c[:, :, 0].argmin()][0])
            extRight = tuple(c[c[:, :, 0].argmax()][0])
            extTop = tuple(c[c[:, :, 1].argmin()][0])
            extBot = tuple(c[c[:, :, 1].argmax()][0])
            l_c =  ((extRight[0] + extLeft[0])//2, extBot[1])
            
            frame = cv2.circle(frame, extLeft, 2, (255, 0, 255), 7)
            frame = cv2.circle(frame, extRight, 2, (255, 0, 255), 7)
            frame = cv2.circle(frame, extTop, 2, (255, 0, 255), 7)
            frame = cv2.circle(frame, extBot, 2, (255, 0, 255), 7)
            frame = cv2.circle(frame, l_c, 2, (255, 255, 255), 7)
          
        return frame

    def estimateLineForm(self):
        # Returns an estimate of the lines slope and bias relative to the front of the camera frame
        if "line" in self.state_info:
            del self.state_info["line"]

        frame = self.low_res_view

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        avg_points = []
        mask_points = []

        for X, Y in self.patch_positions:
            green_spots = []
            for x, y in self.sample_positions:
                if Y+y >= mask.shape[0] or X+x >= mask.shape[1]:
                    break
                if mask[Y+y][X+x] == 255:
                    green_spots.append([X+x, Y+y])
            if len(green_spots) > 3:
                avg_points.append(np.mean(green_spots, axis=0))
            mask_points += green_spots
    
        self.state_info["mask_points"] = mask_points
        self.state_info["line_points"] = avg_points

        if len(avg_points) >= 3:
            avg_points = np.array(avg_points)
            # Add Notes (too low too high blah) 
            m, b = np.polyfit(avg_points[:,0], avg_points[:,1], 1)
            m, b = round(m,3), round(b,3)
            # print(m,b, frame.shape)
            self.state_info["line"] = dict()
            self.state_info["line"]["sample_time"] = time.time()
            self.state_info["line"]["slope"], self.state_info["line"]["bias"] = m, b

    def estimateVelocities(self):
        frame = self.low_res_view
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.state_info.get("current_tracks"):
            img0, img1 = self.state_info["last_gray"], gray
            p_c = np.float32([tr[-1][:-1] for tr in self.state_info.get("current_tracks")]).reshape(-1, 1, 2)
            
            p_f, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p_c, None, **self.lk_params)
            
            p_r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p_f, None, **self.lk_params)
            
            drift = abs(p_c - p_r).reshape(-1, 2).max(-1)
            good = drift < 1
            new_tracks = []

            curr_time = time.time()
            for tr, (x, y), good_flag in zip(self.state_info["current_tracks"], p_f.reshape(-1, 2), good):
                if not good_flag:
                    last_update = tr[-1][-1]
                    if (time.time() - last_update) > 5:
                        del tr
                    continue
                tr.append((x, y, curr_time))
                if len(tr) > 5:
                    tr = tr[1:6]
                new_tracks.append(tr)
            self.state_info["current_tracks"] = new_tracks
        else:
            self.state_info["current_tracks"] = []

        if len(self.state_info.get("current_tracks", [])) > 5: # Do Good Feautures Remain if not trigger search for new features
            # Estimate Velocity From Tracks 
            x_speeds = []
            y_speeds = []
            r_speeds = []
            
            for track in self.state_info.get("current_tracks", []): 
                track = np.array(track)
                # Shift the Plane of tracked point Away from the pixel 
                # Origin and toward the center of rotation for the robot
                # So that rotation can be calculated
                times = track[:,-1]
                track = track[:,:2]
                track -= self.state_info["true_center"] 

                time_deltas = times[1:] - times[:-1]
                track_deltas = track[1:] - track[:-1]
                track_speeds = track_deltas
                track_speeds[:,0] /= time_deltas
                track_speeds[:,1] /= time_deltas

                track_r_speeds = [] # average ang dist in radians
                for vector_1, vector_2, delta_t in zip(track[1:], track[:-1], time_deltas):
                    # While making the assumption that the distance traveled will be less than 2pi radians 
                    # Determine the minimum angular distance and direction travel between the start and stop at actor
                    # TODO: I have an actually tested the map below here so it might be giving me BS results or rotations in the wrong direction
                    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
                    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)

                    angle_1 = np.arccos(np.dot(unit_vector_1, [0,1]))
                    angle_2 = np.arccos(np.dot(unit_vector_2, [0,1]))
                    
                    min_rad_dist = lambda a, b: (b - a) if abs(a - b) < abs((2*np.pi) - max(a,b) + min(a,b)) else (max(a,b) + min(a,b) - (2*np.pi))
                    
                    track_r_speeds.append(min_rad_dist(angle_1, angle_2)/delta_t)
                
                v_x, v_y = np.mean(track_speeds, 0)
                v_r = np.mean(track_r_speeds)

                x_speeds.append(v_x)
                y_speeds.append(v_y)
                r_speeds.append(v_r)
                
            self.state_info["velocity"] = dict()
            self.state_info["velocity"]["px"] = np.mean(x_speeds)
            self.state_info["velocity"]["py"] = np.mean(y_speeds)
            self.state_info["velocity"]["r"] = np.mean(r_speeds)

        # TODO: I shouldn't need this every frame
        mask = np.zeros_like(gray)
        mask[:] = 255
            
        # Mark latest position in each track as an roi for features to track 
        for x, y, _ in [np.int32(tr[-1]) for tr in self.state_info.get("current_tracks",[])]:
            cv2.circle(mask, (x, y), 5, 0, -1)

        p = cv2.goodFeaturesToTrack(gray, mask = mask, **self.feature_params)
        if p is not None: # New feature Positions
            curr_time = time.time()
            for x, y in np.float32(p).reshape(-1, 2):
                self.state_info["current_tracks"].append([(x, y, curr_time)])
        self.state_info["last_gray"] = gray
    
    def largestForegoodObject(self):
        frame = self.high_res_view
        mask = cv2.inRange(frame, self.lower_green, self.upper_green)
        contours, hierarchy= cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        sorted_contours= sorted(contours, key=cv2.contourArea, reverse= True)
        largest_item = sorted_contours[0]
        self.state_info["largest_contour"] = largest_item
    def readCameraFeed(self):
        frame = self.camera_stream.read()
        frame = imutils.resize(frame, width=400)
        self.high_res_shape = frame.shape
        self.high_res_view = frame
        
        frame = imutils.resize(frame, width=200)
        self.low_res_shape = frame.shape
        self.state_info["true_center"] = (frame.shape[1]//2, frame.shape[0]//2)
        self.low_res_view = frame
    def updateFusedMeasurements(self):
        # Store whatever the most recent line detection is so that it's available between short gaps in detection
        if self.state_info.get("line"):
            self.state_info["line_last"] = self.state_info["line"]

        if not self.initialized:
            if self.state_info.get("line"):
                # initialized
                self.state_info["line_fused"] = self.state_info["line"]

                self.initialized = True
            return

        curr_time = time.time()

        self.tau = tau = 1/20

        d_t = curr_time - self.state_info["line_fused"]["sample_time"]
        if d_t >= self.tau:
            # Update the predicted value
            dx = self.state_info["velocity"]["px"] * d_t
            dy = self.state_info["velocity"]["py"] * d_t
            dr = self.state_info["velocity"]["r"] * d_t

            mf_1, bf_1 = self.state_info["line_fused"]["slope"], self.state_info["line_fused"]["bias"]
            mp, bp = predict_newline(mf_1, bf_1, dx, dy, dr, self.low_res_shape)

            m, b = self.state_info["line_last"]["slope"], self.state_info["line_last"]["bias"]
                
            # Apply filtering
            mf = mp * self.alpha + m * (1 - self.alpha)
            bf = bp * self.alpha + b * (1 - self.alpha)
            
            self.state_info["line_fused"]["slope"], self.state_info["line_fused"]["bias"] = mf, bf
            self.state_info["line_fused"]["sample_time"] = curr_time

    def update(self):     
        self.iter_num += 1
        if self.iter_num % (24*4) == 0:
            # Reinitialize
            self.initialized = False
        self.readCameraFeed()
        self.checkForMarker()
        self.estimateLineForm()
        self.estimateVelocities()
        self.updateFusedMeasurements()
        #self.largestForegoodObject()

        if False:# self.state_info.get("marker"): # Default to Absolute Measurement
            self.state_info["line"] = dict()
            self.state_info["line"]["sample_time"] = time.time()
            self.state_info["line"]["slope"], self.state_info["line"]["bias"] = QrPose2LineForm(self.state_info["marker"]["pose"], self.high_res_shape)
            self.state_info["line_init"] = self.state_info["line"]
            self.state_info["line_fused"] = self.state_info["line"]
        
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
                print("WARN: Invalid QR CODE READ: Defaulting to id = 0 for development")
            
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
            #print("FPS: ", 1.0 / (time.time() - s))
    except KeyboardInterrupt as e:
        print("Manual ShutOff")
    finally:
        det.stop()
