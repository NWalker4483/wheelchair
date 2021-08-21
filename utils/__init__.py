from threading import Thread
import time

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
