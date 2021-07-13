import socket             
import time
# Create a socket object 
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)         

# Define the port on which you want to connect 
port = 5005                

# connect to the server on local computer 
host = '192.168.0.2'
s.bind((host,port))
"""
connected = False
while not connected:
    try:
        print("Attempting to connect to host...")
        #s.connect((host, port))
        connected = True
    except ConnectionRefusedError as e:
        time.sleep(1)
        print(e)
"""    
while True:
    # receive data from the server 
    print (s.recvfrom(1024) )
# close the connection 
s.close()     

    