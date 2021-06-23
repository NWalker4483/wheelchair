from tf import Pose
import serial

class Driver(): 
    def __init__(self, port):
        self.speed_setting = 3 # 1 - 5
        self.gotManualInput = False
        self.last_update = None # time of last input
        self.attach(port) 
        self.facing = False
    
    def attach(self, serial_port):
  
        self.ser = serial.Serial(serial_port, 9600)

    def update(self, new_cmd = None): # New command velocities and update your internal pose estimate
        if new_cmd == None:
            pass
        else:
            pass

    def send_cmd(self, linear, angular):
        values = [ord('#'), linear, angular]
        self.ser.write(bytearray(values))

    def stop(self):
        self.send_cmd(0, 0)