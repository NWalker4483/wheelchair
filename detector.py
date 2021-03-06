from typing import Sized
from utils.box_projection import predict_newline
from utils.math import points_to_line, rotate_about, midpoint, distance, angle_between, turn_clockwise
from imutils.video import VideoStream
from utils.draw import draw_line
from pyzbar.pyzbar import decode
from threading import Thread
import numpy as np
import imutils
import time
import cv2

def scale_from(x, y, res_1, res_2):
    x, y = int((x/res_1[0]) * res_2[0]), int((y/res_1[1]) * res_2[1])
    return x, y

class Detector(Thread):
    def __init__(self, filename = None, threaded = False,debug=False):
        super(Detector, self).__init__()
        self.debug = debug
        self.state_info = dict()

        ###### Image Capturing
        self.high_res_shape = None
        self.low_res_shape = None
        
        self.high_res_view = None
        self.low_res_view = None

        self.view_area_shape = () # meters

        self.hl_ratio = 3 # Not yet used 
        self.filename = filename
        if self.filename == None:
            self.camera_stream = VideoStream(usePiCamera = True)
            self.camera_stream.stream.camera.shutter_speed = 2000 # Drop shutter speed to reduce motion blur
            self.camera_stream.start()
        else:
            self.video_stream = cv2.VideoCapture(filename)
            self.file_fps = 30
            
        time.sleep(2)  # Warm Up camera
        self.readCameraFeed()

        ####### Image Masking
        self.lower_green = np.array([28, 50 , 25])
        self.upper_green = np.array([50, 255, 230])
        
        ####### Line Estimation
        # Separate the image into patches 
        self.patch_height = 35 #px
        self.patch_width = 35 #px
        self.sample_cnt = 3 # * WARNING This value is squared 
        self.swap_xy = True
        # ! Do NOT Hardcode Low Res Size ITF 
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
        
        # TODO: Convert to Pose Object
        self.state_info["velocity"] = dict()
        self.state_info["velocity"]["sample_time"] = time.time()
        self.state_info["velocity"]["x"] = 0
        self.state_info["velocity"]["y"] = 0
        self.state_info["velocity"]["r"] = 0

        ####### Image Matching
        # Camera internals
        size = (1, 1)
        focal_length = size[1]
        center = (size[1]/2, size[0]/2)
        self.camera_matrix = camera_matrix = np.array(
                                [[focal_length, 0, center[0]],
                                [0, focal_length, center[1]],
                                [0, 0, 1]], dtype = "double")

        print(f"Camera Matrix :\n {camera_matrix}")

        self.dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion

        # FLANN parameters
        # FLANN_INDEX_KDTREE = 0
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm = FLANN_INDEX_LSH, trees = 5)
        search_params = dict(checks=50)   # or pass empty dictionary

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.MIN_MATCHES = 50
        self.orb = cv2.ORB_create(nfeatures=250)

        self.update_freq = 5
        
        self.state_info["odom"] = dict()
        self.state_info["odom"]["sample_time"] = time.time()
        self.state_info["odom"]["x"] = 0
        self.state_info["odom"]["y"] = 0
        self.state_info["odom"]["r"] = 0

        ###### Sensor Fusion
        self.initialized = False
        self.alpha = .4
        self.tau = 1/20

        ###### MultiThreading
        self.daemeon = True
        self.__alive = True
        if threaded:
            self.start()

    def run(self):
        try:
            while self.__alive:
                self.update()
        finally:
            self.stop()

    def stop(self):
        self.__alive = False
        if self.filename == None:
            self.camera_stream.stop()  
        else:  
            self.video_stream.release()
        if self.debug:
            cv2.destroyAllWindows()  

    def getDebugView(self):
        frame = self.high_res_view
  
        if "marker" in self.state_info:
            
            x, y, r = self.state_info["marker"]["x"], self.state_info["marker"]["y"], self.state_info["marker"]["r"]
            ID = self.state_info["marker"]["id"]

            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)]
            polygon = self.state_info["marker"]["polygon"]
            # Draw QR Code corners
            for i, (point, color) in enumerate(zip(polygon, colors)):
                frame = cv2.circle(frame, (point.x, point.y), 9 + i, color , 5 - i)
            frame = cv2.circle(frame, (int(x), int(y)), 6, (255, 255, 255), 2)

            x, y = int(x), int(y)
            new_end = tuple([int(i) for i in rotate_about((x + distance((x,y), (polygon[0].x, polygon[0].y)), y),(x,y) , r)])
            frame = cv2.line(frame, (x,y), new_end,(255,0,0),9)
            # TODO Rotate Marker ID to match marker rotation
            frame = cv2.putText(frame, str(ID), (int(x), int(y)),
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
            if self.swap_xy:
                b_ = scale_from(abs(b), 0, self.low_res_shape, self.high_res_shape)[0]
            else:
                b_ = scale_from(0,abs(b), self.low_res_shape, self.high_res_shape)[1]
            b = b_ if b >= 0 else -b_

            frame = draw_line(frame, m, b, color=(0, 233, 243), swap_xy = self.swap_xy)

        if "line_fused" in self.state_info:
            m, b = self.state_info["line_fused"]["slope"], self.state_info["line_fused"]["bias"]
            if self.swap_xy:
                b_ = scale_from(abs(b), 0, self.low_res_shape, self.high_res_shape)[0]
            else:
                b_ = scale_from(0,abs(b), self.low_res_shape, self.high_res_shape)[1]
            b = b_ if b >= 0 else -b_

            frame = draw_line(frame, m, b, color=(255, 233, 243), swap_xy = self.swap_xy)
                     
        if "true_center" in self.state_info:
            p0 = self.state_info["true_center"]
            cv2.circle(
                frame,scale_from(*p0,res_1 = self.low_res_shape,res_2 =self.high_res_shape), 6, (255, 0, 0), 10)
        
        if "odom" in self.state_info:
            pass
        
        if "vmelocity" in self.state_info:
            scaler = 1
            vy, vx = self.state_info["velocity"]["y"], self.state_info["velocity"]["x"]
            vr = self.state_info["velocity"]["x"]
            p0 = self.state_info.get("true_center", (0,0))
            p1 = rotate_about((vx, vy), p0, vr)
            p2 = (int(p1[0] * scaler), int(p1[1] * scaler))
            cv2.line(frame, p0, p2, (0,0,255), 4)
        
        if "good_matches" in self.state_info:
            
            # Need to draw only good matches, so create a mask
            matchesMask = []

            #matchesMask[i]=[1,0]

            #matchesMask = [[0,0] for i in range(len(matches))]

            draw_params = dict(matchColor = (0,255,0),
                singlePointColor = (255,0,0),
                matchesMask = matchesMask,
                flags = 0)
            
            #display = cv2.drawMatchesKnn(last, kp1, frame, kp2, matches, None, **draw_params)

        # Draw Center Line
        frame = cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]),(0,233,0),2)
        
        return frame

    def estimateLineForm(self):
        # Returns an estimate of the lines slope and bias relative to the front of the camera frame
        if "line" in self.state_info:
            del self.state_info["line"]

        frame = self.low_res_view

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.state_info["mask"] = mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        avg_points = []
        mask_points = []

        for X, Y in self.patch_positions:
            green_spots = []
            for x, y in self.sample_positions:
                if Y+y >= mask.shape[0] or X+x >= mask.shape[1]:
                    continue
                if mask[Y+y][X+x] == 255:
                    green_spots.append([X+x, Y+y])
            if len(green_spots) > 3:
                mean = np.mean(green_spots, axis=0)
                avg_points.append(mean)
            mask_points += green_spots

        self.state_info["mask_points"] = mask_points
        self.state_info["line_points"] = avg_points

        if len(avg_points) >= 3:
            avg_points = np.array(avg_points)
            # Add Notes (too low too high blah) 
            if self.swap_xy:
                m, b = np.polyfit(avg_points[:,1], avg_points[:,0], 1)
            else:
                m, b = np.polyfit(avg_points[:,0], avg_points[:,1], 1)

            m, b = round(m,3), round(b,3)
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
                # Shift the Plane of tracked point Away from the pixel 
                # Origin and toward the center of rotation for the robot
                # So that rotation can be calculated
                track = np.array(track)
                times = track[:,-1]
                track = track[:,:2]
                #track -= self.state_info["true_center"] 

                time_delta = -1 * (times[0] - times[-1])
                track_delta = track[0] - track[-1]

                # Determine the minimum angular distance and direction travel between the start and stop
                vector_1, vector_2 = track[0], track[-1]
                vector_2[1] += track_delta[1]
                
                dr = angle_between(vector_1, vector_2)
                dr = dr if turn_clockwise(vector_1, vector_2) else -dr

                v_r = np.mean(dr/time_delta)

                x_speeds.append(track_delta[0]/ time_delta)
                y_speeds.append(track_delta[1]/ time_delta)
                r_speeds.append(v_r)
                
            self.state_info["velocity"]["x"] = 0 # np.mean(x_speeds) # WheelChair Cant Strafe so I'm setting this to zero expicitly for rn 
            self.state_info["velocity"]["y"] = np.mean(y_speeds)
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

    def updateOdometry(self):
        curr_time = time.time()
         
        frame = self.high_res_view
        last_keypoints = self.state_info.get("last_keypoints")

        if last_keypoints != None:
            d_t = curr_time - self.state_info["odom"]["sample_time"] 
            if d_t >= (1 / self.update_freq): 
                good_matches = []
                try:
                    kp1, des1 = last_keypoints
                    kp2, des2 = self.orb.detectAndCompute(frame, None)
                    self.state_info["last_keypoints"] = (kp2, des2)

                    matches = self.flann.knnMatch(des1, des2, k=2)
                
                    # ratio test as per Lowe's paper
                    # TODO: I do not understand this value error but it keeps coming up
                    for i, pair in enumerate(matches):
                        try:
                            m, n = pair
                            if m.distance < 0.7 * n.distance:
                                good_matches.append(m)
                        except ValueError:
                            #print("Match Failed", i)
                            pass
                    self.state_info["good_matches"] = good_matches 
                except Exception as e:
                    print(e)
                
                if len(good_matches) >= self.MIN_MATCHES:
                    old_points = np.float32([kp1[m.queryIdx].pt for m in good_matches])
                    curr_points = np.float32([kp2[m.trainIdx].pt for m in good_matches])

                    old_points = np.pad(old_points, [(0,0),(0,1)]) # Add Z axis of 0
                    (success, rotation_vector, translation_vector, inliers) = cv2.solvePnPRansac(old_points, curr_points, self.camera_matrix, self.dist_coeffs, flags = cv2.SOLVEPNP_ITERATIVE)
                    if success:
                        dx, dy, dz = translation_vector
                        dr = rotation_vector[2][0]
                        #print(dx, dy, dz, *rotation_vector)

                        self.state_info["odom"]["r"] += dr
                        tdx, tdy = rotate_about((dx, dy), (0, 0), self.state_info["odom"]["r"])
                        self.state_info["odom"]["x"] += tdx
                        self.state_info["odom"]["y"] += tdy
                        self.state_info["odom"]["sample_time"] = curr_time
        else:
            self.state_info["last_keypoints"] = (self.orb.detectAndCompute(frame, None))
        
    def readCameraFeed(self):
        if self.filename == None:
            frame = self.camera_stream.read()
        else:
            _, frame = self.video_stream.read() 
        self.raw_res_shape = frame.shape
        self.raw_res_view = frame
        
        frame = imutils.resize(frame, width=400)
        self.high_res_shape = frame.shape
        self.high_res_view = frame
        
        frame = imutils.resize(frame, width=200)
        self.low_res_shape = frame.shape
        self.low_res_view = frame
    
    def updateFusedMeasurements(self):
        # Store whatever the most recent line detection is so that it's available between short gaps in detection
        if self.state_info.get("line"):
            self.state_info["line_last"] = self.state_info["line"]
            self.state_info["line_last"]["odom"] = self.state_info["odom"]
        
        if self.state_info.get("marker"): # Default to Absolute Measurement
            self.state_info["line"] = dict()
            self.state_info["line"]["sample_time"] = time.time()
            self.state_info["line"]["slope"], self.state_info["line"]["bias"] = self.QrPose2LineForm()
            self.state_info["line_fused"] = self.state_info["line"]
            return
        
        if not self.initialized:
            if self.state_info.get("line"):
                # initialized
                self.state_info["line_fused"] = self.state_info["line"]
                self.initialized = True
            return

        curr_time = time.time()

        d_t = curr_time - self.state_info["line_fused"]["sample_time"]
        if d_t >= self.tau:
            # Update the predicted value
            dx = 0#self.state_info["odom"]
            dy = 0
            dr = 0
            mf_1, bf_1 = self.state_info["line_fused"]["slope"], self.state_info["line_fused"]["bias"]
            box_shape = self.low_res_shape[:2]
            if self.swap_xy:
                dx, dy = dy, dx
                box_shape = box_shape[::-1]

            # I think Because the camera is mounted in reverse the velocities Need to be negative
            # I don't know if this for sure but I'm making a 
            # guess based on how the prediction moves in the opposite direction of motion when I don't flip them
            
            # TODO: Check for or prevent nan predictions 
            mp, bp = predict_newline(mf_1, bf_1, -dx, -dy, dr, box_shape)

            m, b = self.state_info["line_last"]["slope"], self.state_info["line_last"]["bias"]
                
            # Apply filtering
            mf = mp * self.alpha + m * (1 - self.alpha)
            bf = bp * self.alpha + b * (1 - self.alpha)
            
            self.state_info["line_fused"]["slope"], self.state_info["line_fused"]["bias"] = mf, bf
            self.state_info["line_fused"]["sample_time"] = curr_time

    def update(self):
        self.readCameraFeed()
        self.checkForMarker()
        self.estimateLineForm()
        # self.estimateVelocities()
        self.updateOdometry()

        # TODO: Update to use odometry values instead of velocity
        self.updateFusedMeasurements()

        if self.debug:
            cv2.imshow("Debug", self.getDebugView())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass

    def QrPose2LineForm(self):
        polygon = self.state_info["marker"]["polygon"] 
        p1 = polygon[0]  # Upper Left 
        p2 = polygon[1]  # Lower Left 
        p3 = polygon[2]  # Lower Right
        p4 = polygon[3]  # Upper Right

        # If turned sideways
        if p1[1] > p3[1] or p1[0] > p3[0]:
            p1 = midpoint(p1, p2)
            p2 = midpoint(p3, p4)
        else:
            p1 = midpoint(p1, p4)
            p2 = midpoint(p2, p3)

        p1 = scale_from(*p1, self.high_res_shape, self.low_res_shape)
        p2 = scale_from(*p2, self.high_res_shape, self.low_res_shape)
        
        if self.swap_xy:
            p1 = p1[::-1]
            p2 = p2[::-1]
        try:
            m, b = points_to_line(p1, p2)
        except ZeroDivisionError as e:
            m, b = 1e-5, midpoint(p1,p2)[1]
        return m, b 
    
    def checkForMarker(self):
        if "marker" in self.state_info:
            del self.state_info["marker"]
        
        frame = self.high_res_view
        
        barcodes = decode(frame)
        marker = next(filter(lambda code: code.type == "QRCODE", barcodes), None)
        if marker != None:
            self.state_info["marker"] = dict()
            self.state_info["marker"]["polygon"] = marker.polygon
            try:
                self.state_info["marker"]["id"] = int(marker.data)
            except ValueError as ve:
                self.state_info["marker"]["id"] = 0
                # TODO: Swap in an actual logger
                print("WARNING: Invalid QR CODE READ: Defaulting to id = 0 for development")
            
            p1 = marker.polygon[0] # Upper Left 
            p2 = marker.polygon[1] # Lower Left 
            p3 = marker.polygon[2] # Lower Right
            p4 = marker.polygon[3] # Upper Right
            
            v1 = (p1[0] - p2[0], p1[1] - p2[1]) # midpoint(p1, p2)
  
            rot = angle_between(v1, [0,1])
            
            #TODO: Convert Q4 and Q3 into postive radian values
            #TODO: Rewrite above TODO Works but standardize Rotation Values
            if v1[0] < 0: # Q3 + Q4 
                rot = np.deg2rad(180) + (np.deg2rad(180) - rot)

            x, y = np.mean([p1, p2, p3, p4], axis=0)
            self.state_info["marker"]["x"] = x
            self.state_info["marker"]["y"] = y
            self.state_info["marker"]["r"] = rot
            #print(rot)
            
if __name__ == '__main__':
    det = Detector(debug = True)
    try:
        while True:  # loop over the frames from the video stream
            s = time.time()
            det.update()
            #print("FPS: ", 1.0 / (time.time() - s))
    except KeyboardInterrupt as e:
        print("Manual ShutOff")
    finally:
        
        det.stop()
# sudo py-spy record -o profile.svg -- python3.7 detector.py