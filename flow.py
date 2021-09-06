#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import cv2 as cv
from detector import Detector
from simple_pid import PID

lk_params = dict(winSize  = (15, 15),
                maxLevel = 2,
                criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7)

class PID_Driver():
    def __init__(self, detector, debug = False):
        self.track_len = 20
        self.detector = detector
        self.detect_interval = 1
        self.tracks = []
        self.frame_idx = 0
        self.debug = debug
        self.alive = True
        
        # NEW
        self.velocity_estimate = 0
        self.update_freq = 10 # Hz
        pid = PID()
        pid = PID(.125, .125, .25)
        pid.setpoint = 0
        pid.sample_time = 1/self.update_freq # 10 Hz
        
        pid.output_limits = (-100, 100)  # Output will always be above 0, but with no upper bound
        
        self.pid = pid
        
        self.odom = [0, 0, 0] # x, y, rot
        self.s_len = 2.5 # meters
        self.pps = 680
        
    def run(self):
        #self.pid.update()
        while self.alive:
            frame = self.detector.low_res_view
            frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            vis = frame.copy()

            if len(self.tracks) > 0:
                img0, img1 = self.prev_gray, frame_gray
                p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
                p1, _st, _err = cv.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
                p0r, _st, _err = cv.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
                d = abs(p0-p0r).reshape(-1, 2).max(-1)
                good = d < 1
                new_tracks = []
                for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                    if not good_flag:
                        continue
                    tr.append((x, y))
                    if len(tr) > self.track_len:
                        del tr[0] # PopLeft
                    new_tracks.append(tr)
                    cv.circle(vis, (int(x), int(y)), 2, (0, 255, 0), -1)
                
                
                
                self.tracks = new_tracks
                
                if  len(self.tracks) > 5:    
                    avg_delta_x = 0
                    avg_delta_y = 0
                    for track in self.tracks: 
                        local_avg_delta_x = track[0][0] - track[1][0]
                        local_avg_delta_y = track[0][1] - track[1][1]
                        for i, (x, y) in enumerate(track[1:]):
                            x2, y2 = track[i]
                            local_avg_delta_x += (x - x2)
                            local_avg_delta_x /= 2
                            local_avg_delta_y += (y - y2)
                            local_avg_delta_y /= 2
                        avg_delta_x += local_avg_delta_x
                        avg_delta_y += local_avg_delta_y
                    avg_delta_x /= len(self.tracks) 
                    avg_delta_y /= len(self.tracks)

                    #TODO: Account for rotation
                    self.odom[0] += round(avg_delta_x,3)
                    self.odom[1] += round(avg_delta_y,3)
                    
                    print(self.odom)
                    
                    cv.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
                
            if self.frame_idx % self.detect_interval == 0: # Grab New Keypoint Descriptor
                mask = np.zeros_like(frame_gray)
                mask[:] = 255
                for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                    cv.circle(mask, (x, y), 5, 0, -1)
                p = cv.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                if p is not None: # New feature Positions
                    for x, y in np.float32(p).reshape(-1, 2):
                        self.tracks.append([(x, y)])

            self.frame_idx += 1
            self.prev_gray = frame_gray
            if self.debug:
                cv.imshow('lk_track', vis)
                ch = cv.waitKey(1)
                if ch == 27:
                    break

def main():
    det = Detector(debug=False)
    det.start()
    a =  PID_Driver(det, debug = True)
    try:
        a.run()
    except Exception as e:
        raise(e)
    finally:
        a.alive = False
        print('Done')

if __name__ == '__main__':
    main()
    cv.destroyAllWindows()