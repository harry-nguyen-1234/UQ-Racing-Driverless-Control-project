#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from fssim_messages.msg import Command, State, WheelSpeed
from geometry_msgs.msg import PolygonStamped, Point32
import matplotlib.pyplot as plt

fig = plt.figure()


def callback_trajectory(data):
    x = np.array([point.x for point in data.polygon.points])
    y = np.array([point.y for point in data.polygon.points])


def callback_state(data):
    x = data.position.x
    y = data.position.y
    fig.clear()
    plt.scatter(x, y)
    plt.xlim([-30, 50])
    plt.ylim([-80, 10])
    plt.axes().set_aspect('equal')
    plt.grid(True)
    plt.draw()
    plt.pause(0.01)


if __name__ == '__main__':

    rospy.init_node('car_controller', anonymous=True)

    rospy.Subscriber("chicken/trajectory", PolygonStamped, callback_trajectory,
                     queue_size=1)
    rospy.Subscriber("chicken/state", State, callback_state, queue_size=1)

    rospy.Publisher("chicken/command", Command, latch=True, queue_size=1)
    plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
