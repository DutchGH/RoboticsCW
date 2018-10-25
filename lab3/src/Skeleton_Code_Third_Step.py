#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
		# Initialise a publisher to publish messages to the robot base
		# We covered which topic receives messages that move the robot in the 2nd Lab Session


		# Initialise any flags that signal a colour has been detected in view
		self.colorDetected = False
		self.blueDetected = False
		self.greenDetected = False

		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)

		# Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)




	def callback(self, data):
		sensitivity = 10
		# Set the upper and lower bounds for the two colours you wish to identify
		hsv_green_lower = np.array([60 - sensitivity, 100, 100])
		hsv_green_upper = np.array([60 + sensitivity, 255, 255])
		hsv_blue_lower = np.array([120 - sensitivity, 50, 50])
		hsv_blue_upper = np.array([120 + sensitivity, 255, 255])
		hsv_red_lower = np.array([10 - sensitivity, 100, 100])
		hsv_red_upper = np.array([10 + sensitivity, 255, 255])
		try:
			# Convert the received image into a opencv image
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		# Filter out everything but particular colours using the cv2.inRange() method
		Gmask = cv2.inRange(hsv, hsv_green_lower, hsv_green_upper)
		Bmask = cv2.inRange(hsv, hsv_blue_lower, hsv_blue_upper)
		Rmask = cv2.inRange(hsv, hsv_red_lower, hsv_red_upper)
		mask = Gmask + Bmask + Rmask
		result = cv2.bitwise_and(cv_image, cv_image, mask= mask)
		
		#find contours in the image
		#Make an individual find contours for each mask
		im1, Gcontours, Ghierarchy = cv2.findContours(Gmask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		im2, Bcontours, Bhierachy = cv2.findContours(Bmask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		#cv2.drawContours(result, contours, -1, (0,255,0), 3)
		
		if len(Gcontours) > 0:
			c = max(Gcontours, key = cv2.contourArea)
			if cv2.contourArea(c) > 1500:
				self.greenDetected = True
				(x,y),radius = cv2.minEnclosingCircle(c)
				center = (int(x),int(y))
				radius = int(radius)
				cv2.circle(result,center,radius,(0,255,0),2)
		else:
			self.greenDetected = False
				
		if len(Bcontours) > 0:
			c = max(Bcontours, key = cv2.contourArea)
			if cv2.contourArea(c) > 1500:
				self.greenDetected = True
				(x,y),radius = cv2.minEnclosingCircle(c)
				center = (int(x),int(y))
				radius = int(radius)
				cv2.circle(result,center,radius,(255,0,0),2)
		else:
			self.greenDetected = False


		cv2.namedWindow('Camera_Feed')
		cv2.namedWindow('Mask_Feed')
		cv2.imshow('Camera_Feed', cv_image)
		cv2.imshow('Mask_Feed', result)
		cv2.waitKey(3)



# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	# Instantiate your class
	# And rospy.init the entire node
	cI = colourIdentifier()

	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
	# Ensure that the node continues running with rospy.spin()
	# You may need to wrap it in an exception handler in case of KeyboardInterrupts

	# Remember to destroy all image windows before closing node


# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
