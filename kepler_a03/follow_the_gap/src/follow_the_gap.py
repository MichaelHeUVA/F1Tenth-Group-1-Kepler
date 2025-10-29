#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import math

disparity_threshold = 0.1
velocity = 15
width_car = 0.25
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
    lidar_readings = [
        r if (math.isnan(r) or math.isinf(r)) else r
        for r in data.ranges[start_index : end_index + 1]
    ]

    lidar_readings = [
        data.range_max if (math.isnan(r) or math.isinf(r)) else r
        for r in data.ranges[start_index : end_index + 1]
    ]
    i = 0
    # Find the disparities and overwrite the values
    while i < len(lidar_readings) - 1:
        if abs(lidar_readings[i] - lidar_readings[i + 1]) >= disparity_threshold:
            # Law of cosines => angle to rewrite for the disparity => num_samples_to_overwrite = angle / 0.25 (0.25 degree per lidar scan)
            angle_radian = math.acos((2 * lidar_readings[i] ** 2 - (width_car / 2) ** 2  /(2 * lidar_readings[i] ** 2)))
            angle_degree = math.degrees(angle_radian)
            num_samples_to_overwrite = math.ceil(angle_degree / 0.25)  # TODO: calculate value

            start_idx = i
            end_idx = min(i + num_samples_to_overwrite + 1, len(lidar_readings))
            j = i
            while j < end_idx:
                if lidar_readings[j] > lidar_readings[i]:
                    lidar_readings[j] = lidar_readings[i]
                j += 1
            #start the next disparity finding after the overwrite
            i = j
        i += 1

    # Find the largest gap:
    # potentially normalize values first? or idea, find average value, and then find any gap bigger than average?
    # arbtrially choose a distance, and if its greater than that distance, then choose that:
    #
    arbitary_distance = 1.5
    # find the start of the largest gap
    print(lidar_readings)
    start = -1
    largest_gap = 0
    largest_gap_start = 0
    end = -1
    i = 0
    while i < len(lidar_readings):
        start = i
        end = i
        while i < len(lidar_readings) and lidar_readings[i] > arbitary_distance:
            end = i
            i += 1
        if end - start + 1 > largest_gap:
            largest_gap = end - start + 1
            largest_gap_start = start
        i += 1

    # furthest_dist = max(lidar_readings)
    # furthest_index = len(lidar_readings) - list(reversed(lidar_readings)).index(furthest_dist) - 1
    furthest_index = (largest_gap_start + (largest_gap_start + largest_gap)) // 2
    print(furthest_index, largest_gap)

    # Convert index to angle
    angle = math.degrees(
        (furthest_index - (len(lidar_readings) / 2)) * data.angle_increment
    )

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


if __name__ == "__main__":
    rospy.init_node("follow_the_gap_node", anonymous=True)
    rospy.Subscriber("/car_1/scan", LaserScan, callback)
    rospy.spin()
