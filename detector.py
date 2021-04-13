import cv2 
from pyzbar.pyzbar import decode
from tf import Pose
from math import pi, acos
import time 
from imutils.video import VideoStream
import RPi.GPIO as GPIO
import pins


def sss(d,e,f):
    """ This function solves the triangle and returns (d,e,f,D,E,F) """
    assert d + e > f and e + f > d and f + d > e
    F = acos((d**2 + e**2 - f**2) / (2 * d * e))
    E = acos((d**2 + f**2 - e**2) / (2 * d * f))
    D = pi - F - E
    return (d,e,f,D,E,F)

def dist(x1,y1,x2,y2):
	return ((x1-x2)**2 + (y1-y2)**2)**.5

class Detector():
	def __init__(self):
		self.vs = VideoStream(0).start()
		time.sleep(2.0)
		GPIO.set_mode(pins.MODE)
		self.trigger_pins = [pins.TRIGGER_RIGHT, pins.TRIGGER_MIDDLE, pins.TRIGGER_LEFT]
		self.echo_pins =    [pins.ECHO_RIGHT,    pins.ECHO_MIDDLE,    pins.ECHO_LEFT   ]
		for trigger_pin, echo_pin in zip(self.trigger_pins, self.echo_pins):
			GPIO.setup(trigger_pin, GPIO.OUT)
			GPIO.setup(echo_pin, GPIO.IN)
		
	def getFrame(self):
		frame = self.vs.read()
		frame = imutils.resize(frame, width=400)
		return frame

	def update(self):
		self.checkObstacleDetection()
		frame = self.getFrame()
		self.getGuideLinePosition(frame)
		self.checkForMarker(frame)
		pass

	def getGuideLinePosition(self, frame):
		pass

	def checkForMarker(self, frame, debug = False):
		"""find the two left side corners of a QR marker and return it to pose and ID"""
		barcodes = decode(frame)
		if len(barcodes) > 0:
			marker = barcodes[0] 
			p1 = marker.polygon[0] # Upper Right
			p2 = marker.polygon[1] # Lower Right
			p3 = marker.polygon[2] # Upper Right
			p4 = marker.polygon[3] # Lower Right
			if debug:
				image = cv2.circle(frame, (p1.x,p1.y), 2, (0,255,0), p1.y//40)
				image = cv2.circle(image, (p2.x,p2.y), 2, (0,0,255), 2)
				image = cv2.circle(image, (p3.x,p3.y), 2, (255,0,0), 2)
				frame = cv2.circle(image, (p4.x,p4.y), 2, (255,255,0), 2)
			a = dist(p1.x, p1.y, p2.x, p2.y) + 1e-5
			b = dist(p2.x, p2.y, frame.shape[1], p2.y) + 1e-5
			c = dist(p1.x, p1.y, frame.shape[1], p2.y) + 1e-5
			a,b,c, _, _, C = sss(a,b,c)
			rot = C * (180/pi)
			# Flip Quadrant
			if p1.y > p2.y: # If upside down 
				rot = 180 + (180 - rot)
			return marker.data, Pose(rot=rot)
		else:
			return None, None

	def checkObstacleDetection(self):
		dists = []
		for trigger_pin, echo_pin in zip(self.trigger_pins, self.echo_pins):
			# set Trigger to HIGH
			GPIO.output(trigger_pin, True)
		
			# set Trigger after 0.01ms to LOW
			time.sleep(0.00001)
			GPIO.output(trigger_pin, False)
		
			StartTime = time.time()
			StopTime = time.time()
		
			# save StartTime
			while GPIO.input(echo_pin) == 0:
				StartTime = time.time()
		
			# save time of arrival
			while GPIO.input(echo_pin) == 1:
				StopTime = time.time()
		
			# time difference between start and arrival
			TimeElapsed = StopTime - StartTime
			# multiply with the sonic speed (34300 cm/s)
			# and divide by 2, because there and back
			distance = (TimeElapsed * 34300) / 2
	
			dists.append(distance)

		return dists 

if __name__ == '__main__':
	from imutils.video import VideoStream
	import imutils
	import time
	import cv2

	# initialize the video stream, sensors, etc
	print("[INFO] starting video stream...")
	vs = VideoStream(0).start()
	time.sleep(2.0)

	while True: # loop over the frames from the video stream
		frame = vs.read()
		frame = imutils.resize(frame, width=400)
		ID, pose = checkForMarker(frame, True)
		if ID:
			print(ID, pose)
		cv2.imshow("d",frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break