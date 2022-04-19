#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Jayson De La Vega
# Created Date: 4/5/22
# version = '1.0'
# ---------------------------------------------------------------------------
""" This program defines a ROS publisher and subscribe node """
# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

# TODO: Measure and adjust these values
WHEEL_DISTANCE 1
WHEEL_RAD 1

# ROS nodes to publish left and right motor encoder ticks
r_pub = rospy.Publisher('/right_ticks', Int16)
l_pub = rospy.Publisher('/left_ticks', Int16)

def control():
	"""
	Initializes ROS node for low level control

	Initializes a ROS subscriber and publisher node that publishes encoder data 
	as an Int16 message. Encoder messages are passed to a Raspberry Pi connected
	via USB to perform odometry calculations. Subscriber calls callback() when
	it receives a message on the 'cmd_vel' topic.

	See Also
	--------
	callback(data) : Callback function for subscriber node receiving data

	"""
	rospy.init_node('ev3_ctrl', anonymous=True)
	rate = rospy.Rate(10)

	while not rospy.is_shutdoown():
		r_pub.publish(right_motor.position)
		l_pub.publish(left_motor.position)
		rate.sleep()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass
