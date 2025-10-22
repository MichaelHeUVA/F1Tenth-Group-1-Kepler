#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import math

disparity_threshold = 0.1
velocity = 15

command_pub = rospy.Publisher("/car_1/offboard/command", AckermannDrive, queue_size=10)

def callback(data):

    target_min_rad = math.radians(-90)
    target_max_rad = math.radians(90)

    angle_offset_min = target_min_rad - data.angle_min
    start_index = int(round(angle_offset_min / data.angle_increment))

    angle_offset_max = target_max_rad - data.angle_min
    end_index = int(round(angle_offset_max / data.angle_increment))
    
    # print(start_index, end_index)
    # print(angle_offset_max, angle_offset_min)
    # print(len(data.ranges))

    # Turn ranges tuple to list, remove nan, and set the range of the scan to [-90, 90]
    lidar_readings = [r if (math.isnan(r) or math.isinf(r)) else r for r in data.ranges[start_index: end_index + 1]]

    # Find the disparities and overwrite the values
    for i in range(len(lidar_readings) - 1): 
        if abs(lidar_readings[i] - lidar_readings[i + 1]) >= disparity_threshold:
            closer_dist = min(lidar_readings[i], lidar_readings[i + 1])

            num_samples_to_overwrite = 30 # TODO: calculate value

            if lidar_readings[i] < lidar_readings[i + 1]:
                start_idx = i + 1
                end_idx = min(i + 1 + num_samples_to_overwrite, len(lidar_readings))
                for j in range(start_idx, end_idx):
                    if lidar_readings[j] > closer_dist:
                        lidar_readings[j] = closer_dist
            else:
                start_idx = i + 1
                end_idx = min(i + 1 + num_samples_to_overwrite, len(lidar_readings))

                for j in range(start_idx, end_idx):
                    if lidar_readings[j] > closer_dist:
                        lidar_readings[j] = closer_dist

    # Find farthest gap, this doesn't really work because it chooses the first value it can find, this point might not be the best choice if there is a larger gap TODO: change to find the largest gap
    furthest_dist = max(lidar_readings)
    furthest_index = len(lidar_readings) - list(reversed(lidar_readings)).index(furthest_dist) - 1


    # Convert index to angle
    angle = math.degrees((furthest_index - (len(lidar_readings) / 2)) * data.angle_increment)

    print(angle)

    command = AckermannDrive()
    if angle > 100:
        angle = 100
    elif angle < -100:
        angle = -100

    command.steering_angle = angle
    command.speed = velocity

    command_pub.publish(command)
    
    # print(math.degrees(data.angle_min), math.degrees(data.angle_max))

if __name__ == '__main__':
    rospy.init_node("follow_the_gap_node", anonymous=True)
    rospy.Subscriber("/car_1/scan", LaserScan, callback)
    rospy.spin()




    