import cv2 
from pyzbar.pyzbar import decode
from tf import Pose
from math import pi, acos
import numpy as np
import time 
import imutils
from imutils.video import VideoStream, FileVideoStream

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
	def __init__(self, filename = None):
		if filename == None:
			self.vs = VideoStream(0)
		else:
			self.vs = FileVideoStream(filename)

		self.vs.start()

		self.marker_polygon = None
		self.marker_pose = Pose()
		self.marker_id = None
		self.guide_pose = Pose()

		self.avg_points = [] 
		self.mask_points = []

	def drawDebug(self, frame):
		for x, y in self.mask_points:
			frame = cv2.circle(frame, (int(x) * 2,int(y) * 2), 2, (255,0,255), 2)
		for x, y in self.avg_points:
			frame = cv2.circle(frame, (int(x) * 2,int(y) * 2), 2, (0,0,255), 7)

		if self.marker_polygon != None:
			frame = cv2.circle(frame, (self.marker_polygon[0].x,self.marker_polygon[0].y), 2, (0,255,0), 5)
			frame = cv2.circle(frame, (self.marker_polygon[1].x,self.marker_polygon[1].y), 2, (0,0,255), 5)
			frame = cv2.circle(frame, (self.marker_polygon[2].x,self.marker_polygon[2].y), 2, (255,0,0), 5)
			frame = cv2.circle(frame, (self.marker_polygon[3].x,self.marker_polygon[3].y), 2, (255,255,0), 5)
		
		if self.marker_pose != None:
			# font
			font = cv2.FONT_HERSHEY_SIMPLEX
			# org
			org = (int(self.marker_pose.x), int(self.marker_pose.y))
			# fontScale
			fontScale = 3
			# Red color in BGR
			color = (0, 0, 255)
			# Using cv2.putText() method
			frame = cv2.putText(frame, str(self.marker_id), org, font, fontScale, 
							color, 5, cv2.LINE_AA, False)
		return frame

	def getFrame(self):
		frame = self.vs.read()
		frame = imutils.resize(frame, width=400)
		return frame

	def update(self, debug=False):
		# self.checkObstacleDetection()
		frame = self.getFrame()
		self.guide_pose = self.getGuideLinePosition(frame)
		self.marker_id, self.marker_pose = self.checkForMarker(frame)

		if debug:
			return self.drawDebug(frame)
		else:
			return self.marker_id, self.marker_pose
		
	def getGuideLinePosition(self, img):
		img = imutils.resize(img, width=200)
		lower_hsv = np.array([40,46,77]) 
		upper_hsv = np.array([144,210,227]) 
		img = cv2.medianBlur(img,5)
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
		pnts = []
		self.avg_points = []
		self.mask_points = []
		for y in range(0, mask.shape[0], 20): # Separate the image into strips of height 20
			vals = []
			for i in range(0, 15, 5): # Sample each of the strips four times
			
				for x in range(0, mask.shape[1], 5):
					if mask[y + i][x] > 125:
						vals.append([x,y + i])
			if len(vals) > 0:
				pnts.append(np.mean(vals, axis=0))
			self.mask_points+=vals
		self.avg_points += pnts
		return Pose(*self.avg_points[0],rot = 0) 

	def checkForMarker(self, frame):
		"""find the two left side corners of a QR marker and return it to pose and ID"""
		barcodes = decode(frame)
		if len(barcodes) > 0:
			marker = barcodes[0] 
			self.marker_polygon = marker.polygon
			p1 = self.marker_polygon[0] # Upper Right
			p2 = self.marker_polygon[1] # Lower Right
			p3 = self.marker_polygon[2] # Upper Right
			p4 = self.marker_polygon[3] # Lower Right
			a = dist(p1.x, p1.y, p2.x, p2.y) + 1e-5
			b = dist(p2.x, p2.y, frame.shape[1], p2.y) + 1e-5
			c = dist(p1.x, p1.y, frame.shape[1], p2.y) + 1e-5
			a,b,c, _, _, C = sss(a,b,c)
			rot = C * (180/pi)
			# Flip Quadrant
			if p1.y > p2.y: # If upside down 
				rot = 180 + (180 - rot)
			return int(marker.data), Pose(*np.mean([p1,p2,p3,p4], axis=0),rot=rot)
		else:
			self.marker_polygon = None
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
	from imutils.video import VideoStream, FileVideoStream
	import imutils
	import time
	import cv2

	# initialize the video stream, sensors, etc
	print("[INFO] starting video stream...")
	det = Detector("data/green.mp4")
	time.sleep(2.0)

	while True: # loop over the frames from the video stream
		cv2.imshow("d",det.update(True ))
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break