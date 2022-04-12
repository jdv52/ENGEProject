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
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_B

# TODO: Measure and adjust these values
WHEEL_DISTANCE 1
WHEEL_RAD 1

# ROS nodes to publish left and right motor encoder ticks
r_pub = rospy.Publisher('/right_ticks', Int16)
l_pub = rospy.Publisher('/left_ticks', Int16)

right_motor = LargeMotor(OUTPUT_A)
left_motor = LargeMoror(OUTPUT_B)
right_wheel_ticks = 0
left_wheel_ticks  = 0

'''def callback(data):
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
	linear_vel = data.linear.x
	angular_vel = data.angular.z
	
	# Algorithm based on kinematic model of non-holonomic 2-wheeled robot
	left_speed = (linear_vel - WHEEL_DISTANCE * angular_vel) / WHEEL_RAD 
	right_speed = (linear_vel + WHEEL_DISTANCE * angular_vel) / WHEEL_RAD
	
	# TODO: remap "speed" values to percentages of maximum wheel speed
	# TODO: drive motor at resulting speed percentage '''

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
	# rospy.Subscriber('/cmd_vel', Twist, callback) 

	r_pub.publish(right_motor.position)
	l_pub.publish(left_motor.position)

	rospy.spin()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass
