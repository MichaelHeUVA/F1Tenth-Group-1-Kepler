#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.5	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    # DONE: implement
	
	# print("angle min", data.angle_min)
	# print("angle max", data.angle_max)
	# print("angle increment", data.angle_increment)
	# print("ranges length", len(data.ranges))
	# print("angle in degrees", angle)
	angle += 30
	angle_radians = math.radians(angle)
	
	index = int((angle_radians) / data.angle_increment)
	# print("angle radians", angle_radians)
	#  print("index", index)

	if index >= 0 and index < len(data.ranges):
		value = data.ranges[index]
		if not math.isinf(value) and not math.isnan(value):
			past_range = value
			return value
		else:
			return 10 



def callback(data):
	global forward_projection
	
	theta = 75 # you need to try different values for theta
	a = getRange(data,theta) # obtain the ray distance for theta
	b = getRange(data,0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)

	if not a or not b:
		rospy.loginfo("Invalid LIDAR readings a: %s, b: %s", a, b)
                msg = pid_input()
                msg.pid_error = 0
                msg.pid_vel = vel
                pub.publish(msg)
		return

	swing = math.radians(theta)

	## Your code goes here to determine the projected error as per the alrorithm
	# Compute Alpha, AB, and CD..and finally the error.
	# DONE: implement
	alpha = math.atan2((a*math.cos(swing) - b), (a * math.sin(swing)))
	AB = b * math.cos(alpha)
	CD = AB + forward_projection * math.sin(alpha)
	error = desired_distance - CD
	# rospy.loginfo("Error: %f", error)

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	# print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# DONE: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_1/scan",LaserScan,callback)
	rospy.spin()
