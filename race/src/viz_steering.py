#!/usr/bin/env python3
import rospy, math
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class SteeringViz:
    def __init__(self):
        # Keep markers in base_link so the arrow points relative to the car body
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/car_1/offboard/command")
        self.arrow_topic = rospy.get_param("~arrow_topic", "/viz/steering")

        self.arrow_len = rospy.get_param("~arrow_length", 0.8)
        self.max_vis_angle = rospy.get_param("~max_vis_angle_deg", 40.0)
        self.assume_degrees = rospy.get_param("~steering_cmd_in_degrees", False)

        self.pub = rospy.Publisher(self.arrow_topic, Marker, queue_size=1)
        rospy.Subscriber(self.cmd_topic, AckermannDrive, self.cb)

    def callback(self, msg):
        angle = msg.steering_angle
        if self.assume_degrees:
            angle = math.radians(angle)

        angle = max(
            -math.radians(self.max_vis_angle),
            min(math.radians(self.max_vis_angle), angle),
        )

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "steering_arrow"
        m.id = 1
        m.type = Marker.ARROW
        m.action = Marker.ADD

        p0 = Point(0.0, 0.0, 0.1)
        p1 = Point(
            self.arrow_len * math.cos(angle), self.arrow_len * math.sin(angle), 0.1
        )
        m.points = [p0, p1]

        m.scale.x = 0.03  # shaft diameter
        m.scale.y = 0.06  # head diameter
        m.scale.z = 0.12  # head length
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0)
        self.pub.publish(m)


if __name__ == "__main__":
    rospy.init_node("viz_steering")
    SteeringViz()
    rospy.spin()
