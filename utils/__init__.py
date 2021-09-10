from threading import Thread
import time
import cv2
import numpy as np
min_ang_dist = lambda a, b: (b - a) if abs(a - b) < abs(360 - max(a,b) + min(a,b)) else (max(a,b) + min(a,b) - 360)
constrain = lambda x, min_, max_: min_ if x < min_ else (max_ if x > max_ else x)  
import time
def distance(p1,p2):
  return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**.5
def midpoint(p1,p2):
  x1,y1 = p1
  x2,y2 = p2
  x_m_point = (x1 + x2)/2
  y_m_point = (y1 + y2)/2
  return (x_m_point, y_m_point)

# https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
def find_line_intersection(m1, b1, m2, b2):
    if m1 == m2:
        print ("These lines are parallel!!!")
        return None
    # y = mx + b
    # Set both lines equal to find the intersection point in the x direction
    # m1 * x + b1 = m2 * x + b2
    # m1 * x - m2 * x = b2 - b1
    # x * (m1 - m2) = b2 - b1
    # x = (b2 - b1) / (m1 - m2)

    x = (b2 - b1) / (m1 - m2)
    # Now solve for y -- use either line, because they are equal here
    # y = mx + b
    y = m1 * x + b1
    return x, y

def points_to_line(p1, p2):
  # when given two points, return a slope and bias value which will intersect with those given points
  x1, y1 = p1
  x2, y2 = p2
  m = (y2 - y1) / (x2 - x1)
  b = y1 - (m * x1)
  return m, b

def line_to_points(m, b):
  # Return origin and unit vector
  return (0, 0), (1, m + b)

def rotate_about(point, origin, radians):
  x, y = point
  offset_x, offset_y = origin
  adjusted_x = (x - offset_x)
  adjusted_y = (y - offset_y)
  cos_rad = np.cos(radians)
  sin_rad = np.sin(radians)
  qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
  qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
  return qx, qy
    
def get_boxpoints(box_hw):
  return [(0, box_hw[0]), box_hw[::-1], (box_hw[1], 0), (0,0)]

def draw_boxpoints(frame, box_points, color = (255, 0, 0)):
  for i in range(4):
    cv2.line(frame, [int(i) for i in box_points[i]], [int(i) for i in box_points[i-1]], color, 3)
  return frame
  
def draw_line(frame, slope, bias, color = (0, 255, 0), thickness = 2, swap_xy = False):
  if swap_xy:
    p1 = (int(bias), 0)
    p2 = (int(slope * frame.shape[0] + bias), frame.shape[0])
  else:
    p1 = (0, int(bias))
    p2 = (frame.shape[1], int(slope * frame.shape[1] + bias))

  cv2.line(frame, p1, p2, color, thickness)
  return frame

class UDPStream(Thread):

    def __init__(self, socket):
        Thread.__init__(self)
        self.daemon = True
        self.data = {"last": "i/0","cap_time": time.time()}
        self.__raw_data = ''
        self.sock = socket
        self.alive = True

    def run(self):
        while self.alive:
            if type(self.sock) == type(None):
                continue
            recv_data = self.sock.recv(1024)
            if not recv_data:
                continue
            self.__raw_data += recv_data.decode()
            start, stop = -1, -1
            for i in range(len(self.__raw_data) - 1, -1, -1):
                if start == -1 and self.__raw_data[i] == ";":
                    stop = i
                if stop != -1 and self.__raw_data[i] == "#":
                    start = i
                    break
         
            if start >= 0 and stop >= 0 and stop > start:
                self.data["last"] = self.__raw_data[start + 1: stop]
                self.__raw_data = self.__raw_data[stop:]
                self.data["cap_time"] = time.time()

    def close(self):
        self.alive = False
        self.sock.close()

