#!/usr/bin/env python
import rospy, math
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class SteeringViz(object):
    def __init__(self):
        # ---- params ----
        self.frame_id = rospy.get_param("~frame_id", "car_1/base_link")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/car_1/offboard/command")
        self.arrow_topic = rospy.get_param("~arrow_topic", "/car_1/steering_marker")
        self.arrow_len = float(rospy.get_param("~arrow_length", 0.35))  # meters
        self.use_degrees = bool(rospy.get_param("~use_degrees", False))
        self.clip_deg = float(
            rospy.get_param("~clip_degrees", 45.0)
        )  # for display only

        self.pub = rospy.Publisher(self.arrow_topic, Marker, queue_size=1)
        self.sub = rospy.Subscriber(
            self.cmd_topic, AckermannDrive, self.callback, queue_size=10
        )

        rospy.loginfo(
            "viz_steering listening on %s, publishing %s",
            self.cmd_topic,
            self.arrow_topic,
        )

    def callback(self, msg):
        # Pull steering angle from the drive command
        ang = msg.steering_angle  # expected in radians by ROS conventions
        if self.use_degrees:
            # If your control stack uses degrees, convert here once
            ang = math.radians(ang)

        # Optional: clip for visualization so the arrow doesn’t fold back on itself
        ang = max(min(ang, math.radians(self.clip_deg)), -math.radians(self.clip_deg))

        # Arrow from origin along the steering direction in base_link
        tip_x = self.arrow_len * math.cos(ang)
        tip_y = self.arrow_len * math.sin(ang)

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "steering"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # When using ARROW with points, scale.x = shaft diameter,
        # scale.y = head diameter, scale.z = head length
        m.scale.x = 0.04
        m.scale.y = 0.08
        m.scale.z = 0.10

        # white, fully opaque
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0

        p0 = Point()
        p0.x, p0.y, p0.z = 0.0, 0.0, 0.05
        p1 = Point()
        p1.x, p1.y, p1.z = tip_x, tip_y, 0.05
        m.points = [p0, p1]
        m.lifetime = rospy.Duration(0.0)

        self.pub.publish(m)


def main():
    rospy.init_node("viz_steering")
    SteeringViz()
    rospy.spin()


if __name__ == "__main__":
    main()
