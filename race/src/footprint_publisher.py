#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PolygonStamped, Point32


def make_rect_polygon(length_m, width_m):
    L = length_m / 2.0
    W = width_m / 2.0
    # CCW rectangle around the origin (base_link)
    return [
        Point32(+L, +W, 0.0),
        Point32(+L, -W, 0.0),
        Point32(-L, -W, 0.0),
        Point32(-L, +W, 0.0),
    ]


def main():
    rospy.init_node("footprint_publisher")

    # ---- params (feel free to tweak) ----
    frame_id = rospy.get_param("~frame_id", "car_1/base_link")
    topic_out = rospy.get_param("~topic_out", "/car_1/footprint")
    length_m = float(rospy.get_param("~length", 0.52))  # car length (m)
    width_m = float(rospy.get_param("~width", 0.30))  # car width  (m)
    hz = float(rospy.get_param("~rate", 5.0))  # publish rate

    pub = rospy.Publisher(topic_out, PolygonStamped, queue_size=1)

    poly = PolygonStamped()
    poly.header.frame_id = frame_id
    poly.polygon.points = make_rect_polygon(length_m, width_m)

    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        poly.header.stamp = rospy.Time.now()
        pub.publish(poly)
        rate.sleep()


if __name__ == "__main__":
    main()
