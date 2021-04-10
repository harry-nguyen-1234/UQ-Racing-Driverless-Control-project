#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from fssim_messages.msg import Command, State, WheelSpeed
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt


class CarController:
    def __init__(self):
        rospy.init_node('car_controller', anonymous=True)
        self.pub_command = rospy.Publisher(
            "chicken/command", Command, latch=True, queue_size=1)

        self.pub_position = rospy.Publisher(
            "chicken/position", Marker, latch=True, queue_size=1)

        rospy.Subscriber("chicken/trajectory", PolygonStamped, self.callback_trajectory,
                         queue_size=1)

        rospy.Subscriber("chicken/state", State,
                         self.callback_state, queue_size=1)

    def callback_trajectory(self, data):
        x = np.array([point.x for point in data.polygon.points])
        y = np.array([point.y for point in data.polygon.points])

    def callback_state(self, data):
        x = data.position.x
        y = data.position.y

        marker = Marker()
        marker.ns = "position"
        marker.id = 0
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.a = 1.0

        self.pub_position.publish(marker)


if __name__ == '__main__':

    car_controller = CarController()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
