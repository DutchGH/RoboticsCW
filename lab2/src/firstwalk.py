#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

shouldMove = True
def callback(data):
	global shouldMove
	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.state)
	if data.state :
		shouldMove = False


def publisher():
	global shouldMove
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz

	rospy.Subscriber('mobile_base/events/bumper', BumperEvent, callback)

	desired_velocity = Twist()
	# desired_velocity.linear.x = 0.2# Forward with 0.2 m/sec.
	# desired_velocity.angular.z = 0.5
	# 	pub.publish(desired_velocity)
	# 	rate.sleep()

	# for i in range (30):
	while not rospy.is_shutdown():
		desired_velocity.linear.x = 0.2 # Backward with 0.2 m/sec.
		for i in range(30):
			if (shouldMove):
				pub.publish(desired_velocity)
				rate.sleep()

		desired_velocity.linear.x = 0
		desired_velocity.angular.z = -1.5708
		for i in range(10):
			if (shouldMove):
				pub.publish(desired_velocity)
				rate.sleep()

		desired_velocity.linear.x = 0.2
		desired_velocity.angular.z = 0
		for i in range(30):
			if (shouldMove):
				pub.publish(desired_velocity)
				rate.sleep()

		desired_velocity.linear.x = 0
		desired_velocity.angular.z = -1.5708
		for i in range(10):
			if (shouldMove):
				pub.publish(desired_velocity)
				rate.sleep()

		desired_velocity.linear.x = 0.2
		desired_velocity.angular.z = 0
		for i in range(30):
			if (shouldMove):
				pub.publish(desired_velocity)
				rate.sleep()

		desired_velocity.linear.x = 0
		desired_velocity.angular.z = -1.5708
		for i in range(10):
			if (shouldMove):
				pub.publish(desired_velocity)
				rate.sleep()

		desired_velocity.linear.x = 0.2
		desired_velocity.angular.z = 0
		for i in range(30):
			if (shouldMove):
				pub.publish(desired_velocity)
				rate.sleep()

		desired_velocity.linear.x = 0
		desired_velocity.angular.z = -1.5708
		for i in range(10):
			if (shouldMove):
				pub.publish(desired_velocity)
				rate.sleep()


if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
