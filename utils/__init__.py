from threading import Thread
import time

import numpy as np
constrain = lambda x, min_, max_: min_ if x < min_ else (max_ if x > max_ else x)  

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

