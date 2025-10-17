#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PolygonStamped, Point32


def make_rect_polygon(length, width):
    L = length / 2.0
    W = width / 2.0
    return [
        Point32(+L, +W, 0.0),
        Point32(+L, -W, 0.0),
        Point32(-L, -W, 0.0),
        Point32(-L, +W, 0.0),
    ]


if __name__ == "__main__":
    rospy.init_node("footprint_publisher")
    pub = rospy.Publisher("/viz/footprint", PolygonStamped, queue_size=1, latch=True)

    length = rospy.get_param("~length", 0.52)
    width = rospy.get_param("~width", 0.30)
    frame_id = rospy.get_param("~frame_id", "base_link")

    poly = PolygonStamped()
    poly.header.frame_id = frame_id
    poly.polygon.points = make_rect_polygon(length, width)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        poly.header.stamp = rospy.Time.now()
        pub.publish(poly)
        rate.sleep()
