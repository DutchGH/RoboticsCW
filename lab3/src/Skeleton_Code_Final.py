#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

	def __init__(self, pub, rate):
		# Initialise a publisher to publish messages to the robot base
		# We covered which topic receives messages that move the robot in the 2nd Lab Session
		self.publisher = pub
		self.rate = rate
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
		self.desired_velocity = Twist()
		# Initialise a publisher to publish messages to the robot base
		# We covered which topic receives messages that move the robot in the 2nd Lab Session


		# Initialise any flags that signal a colour has been detected in view
		self.colorDetected = False
		self.blueDetected = False
		self.greenDetected = False
		self.moving = False


		# Initialise any flags that signal a colour has been detected in view


		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)


		# Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)

		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
	def followObject(self, c, result):
		area = cv2.contourArea(c)
		sizeGoal = 18000
		horTolerance = 300
		sizeTolerance = 2000
		(x,y),radius = cv2.minEnclosingCircle(c)
		center = (int(x),int(y))
		radius = int(radius)
		cv2.circle(result,center,radius,(0,0,255),2)

		self.desired_velocity.angular.z = 0
		self.desired_velocity.angular.x = 0


		if center[0] > (radius + horTolerance):
			self.desired_velocity.angular.z = -0.1
			print("X COORD: " + str(center[0]))
			print("TOLERANCE: " + str(radius + sizeTolerance))
			print("LFT")
		elif center[0] < (radius + horTolerance):
			self.desired_velocity.angular.z = 0.1
			print("X COORD: " + str(center[0]))
			print("TOLERANCE: " + str(radius + sizeTolerance))
			print("RT")
		else:
			self.desired_velocity.angular.z = 0


		if area > (sizeGoal + sizeTolerance):
			self.desired_velocity.linear.x = -0.2
		elif area < (sizeGoal - sizeTolerance):
			self.desired_velocity.linear.x = 0.2
		else:
			self.desired_velocity.linear.z = 0

		self.publisher.publish(self.desired_velocity)

	def stopMovement(self):
		self.desired_velocity.linear.x = 0
		self.desired_velocity.angular.z = 0
		self.publisher.publish(self.desired_velocity)


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
		Gcontours, Ghierarchy = cv2.findContours(Gmask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		Bcontours, Bhierachy = cv2.findContours(Bmask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		#cv2.drawContours(result, contours, -1, (0,255,0), 3)

		self.greenDetected = False
		self.blueDetected = False
		c = None
		if len(Gcontours) > 0:
			c = max(Gcontours, key = cv2.contourArea)
			area = cv2.contourArea(c)
			if area > 1500:
				self.greenDetected = True
			else:
				self.greenDetected = False

		if len(Bcontours) > 0:
			c = max(Bcontours, key = cv2.contourArea)
			area = cv2.contourArea(c)
			if cv2.contourArea(c) > 1500:
				self.blueDetected = True
			else:
				self.blueDetected = False


		if (self.greenDetected and self.blueDetected):
			#We have found 2 different colours - stop
			self.movement = False
		elif (self.greenDetected) or (self.blueDetected):
			self.movement = True
		else:
			self.movement = False

		if self.movement:
			self.followObject(c, result)
		else:
			self.stopMovement()



		cv2.namedWindow('Camera_Feed')
		cv2.namedWindow('Mask_Feed')
		cv2.imshow('Camera_Feed', cv_image)
		cv2.imshow('Mask_Feed', result)
		cv2.waitKey(3)

		# cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

		#Check if the area of the shape you want is big enough to be considered
		# If it is then change the flag for that colour to be True(1)
			# draw a circle on the contour you're identifying as a blue object as well
			# cv2.circle(<image>, (<center x>,<center y>), <radius>, <colour (rgb tuple)>, <thickness (defaults to 1)>)
			# Then alter the values of any flags

		# Be sure to do this for the other colour as well
		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
	rate = rospy.Rate(10) #10hz

	return pub, rate

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	rospy.init_node('image_converter', anonymous=True)
	pub, rate = publisher()
	cI = colourIdentifier(pub, rate)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
