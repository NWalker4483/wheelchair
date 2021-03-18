import cv2 
import pyzbar
from tf import Pose
def getGuideLinePosition(frame):
    pass 
def getMarkerPose(frame):
	pass

def checkForMarker(frame):
	pass
# barcodes = pyzbar.decode(frame)

# # loop over the detected barcodes
# for barcode in barcodes:
# 	# extract the bounding box location of the barcode and draw
# 	# the bounding box surrounding the barcode on the image
# 	(x, y, w, h) = barcode.rect
# 	cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
# 	# the barcode data is a bytes object so if we want to draw it
# 	# on our output image we need to convert it to a string first
# 	barcodeData = barcode.data.decode("utf-8")
# 	barcodeType = barcode.type
# 	# draw the barcode data and barcode type on the image
# 	text = "{} ({})".format(barcodeData, barcodeType)
# 	cv2.putText(frame, text, (x, y - 10),
# 		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
if __name__ == '__main__':
	pass