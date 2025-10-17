#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
kp = 0.0
kd = 0.0
ki = 0.0
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
angle = 0


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 15.0

max_velocity = 40.0
min_velocity = 25.0
vel_scale_factor = 30.0		# need to tune this

# Publisher for moving the car.
# DONE: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle

	# print("PID Control Node is Listening to error")

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller

	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering
	
	# print("old angle is", angle)
	error = data.pid_error
	diff = error - prev_error
        steering_correction = kp * error + kd * diff
        angle = math.degrees(steering_correction)
	prev_error = error
	
	# print("error is", error)
	# print("steering correction is", steering_correction)
	# print("new angle is", angle)

	scaled_velocity = max_velocity - (vel_scale_factor * abs(error) * abs(error))

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# DONE: Make sure the steering value is within bounds [-100,100]
	if angle > 100:
		angle = 100
        elif angle < -100:
		angle = -100
	
	command.steering_angle = angle

	# DONE: Make sure the velocity is within bounds
	if scaled_velocity > max_velocity:
		scaled_velocity = max_velocity
	elif scaled_velocity < min_velocity:
		scaled_velocity = min_velocity

	# print("scaled velocity is", scaled_velocity)
	
	command.speed = scaled_velocity

        # Move the car autonomously
	command_pub.publish(command)

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	kp = float(input("Enter Kp Value: "))		# try 5 for kp and 0.09 for kd at first
	kd = float( input("Enter Kd Value: "))
	ki = float(input("Enter Ki Value: "))
	# max_velocity = float(input("Enter desired max velocity: "))
	 
	# kp = rospy.get_param('~kp', 5.0)
	# kd = rospy.get_param('~kd', 3.0)
	# ki = rospy.get_param('~ki', 0.0)
	# max_velocity = rospy.get_param('~vel_input', 45.0)

	rospy.loginfo("PID gains loaded: kp=%f, kd=%f, ki=%f", kp, kd, ki)
	rospy.loginfo("Max Velocity set to: %f", max_velocity)

	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
