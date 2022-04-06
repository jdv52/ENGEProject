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
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B

# ROS nodes to publish left and right motor encoder ticks
r_pub = rospy.Publisher('/right_ticks', Int16)
l_pub = rospy.Publisher('/left_ticks', Int16)

encoder_data = 0

# TODO: write algorithm to calculate motor speeds from twist message
def callback(data):
	"""
	Callback function for subscriber node receiving data

	This function is called when the node subscribed to '/cmd_vel' receives
	a message. This function utilizes the linear x-direction velocity and
	angular z-direction velocity to calculate the robot's left and right
	wheel speeds.

	Parameters
	----------
	data : geometry_msgs.msg.Twist
		Message with two 3D Vectors defining linear and angular velocity.

	"""

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
	rospy.Subscriber('/cmd_vel', Twist, callback)

	# TODO: Obtain encoder measurements and publish msg

	r_pub.publish(encoder_data)
	l_pub.publish(encoder_data)

	rospy.spin()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass
