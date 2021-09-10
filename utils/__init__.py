from threading import Thread
import time
import cv2
import numpy as np
constrain = lambda x, min_, max_: min_ if x < min_ else (max_ if x > max_ else x)  
    
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



import threading
import ctypes
import time
  
class thread_with_exception(threading.Thread):
  def __init__(self, target, args):
    threading.Thread.__init__(self)
    self.function = target
    self.args = args
          
  def run(self):
    # target function of the thread class
    try:
      self.function(*self.args)
    finally:
      print('ended')
      
  def get_id(self):
    # returns id of the respective thread
    if hasattr(self, '_thread_id'):
      return self._thread_id
    for id, thread in threading._active.items():
      if thread is self:
        return id

  def raise_exception(self):
    thread_id = self.get_id()
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id,
          ctypes.py_object(SystemExit))
    if res > 1:
      ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
      print('Exception raise failure')
    
# t1 = thread_with_exception('Thread 1')
# t1.start()
# time.sleep(2)
# t1.raise_exception()
# t1.join()


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

