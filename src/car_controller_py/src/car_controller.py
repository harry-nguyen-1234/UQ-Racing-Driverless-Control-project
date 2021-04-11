#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from fssim_messages.msg import Command, State, WheelSpeed
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from tf.transformations import quaternion_about_axis
from scipy.spatial import KDTree


class CarController:
    def __init__(self):
        self.pub_command = rospy.Publisher(
            "chicken/command", Command, latch=True, queue_size=1)

        self.pub_position = rospy.Publisher(
            "chicken/position", Marker, latch=True, queue_size=1)

        rospy.Subscriber("chicken/trajectory", PolygonStamped, self.callback_trajectory,
                         queue_size=1)

        rospy.Subscriber("chicken/state", State,
                         self.callback_state, queue_size=1)

        self.car_position = None
        self.car_velocity = None
        self.car_acceleration = None
        self.traj_coords = None
        self.tree = None

    def draw_car_position(self, marker):
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.car_position.x
        marker.pose.position.y = self.car_position.y
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.pub_position.publish(marker)

    def draw_car_heading(self, marker):
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        q = quaternion_about_axis(
            self.car_position.theta + self.car_velocity.theta, (0, 0, 1))
        marker.pose.position.x = self.car_position.x
        marker.pose.position.y = self.car_position.y
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.scale.x = 2.0
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.pub_position.publish(marker)

    def draw_closest_traj_point(self, marker, closest_x, closest_y):
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = closest_x
        marker.pose.position.y = closest_y
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 0.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 1.0

        self.pub_position.publish(marker)

    def callback_trajectory(self, data):
        x = [point.x for point in data.polygon.points]
        y = [point.y for point in data.polygon.points]
        self.traj_coords = zip(x, y)
        self.tree = KDTree(self.traj_coords)

    def callback_state(self, state):
        self.car_position = state.position
        self.car_velocity = state.velocity
        self.car_acceleration = state.acceleration

        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "map"
        self.draw_car_position(marker)
        self.draw_car_heading(marker)

        if self.tree is not None:
            _, indexes = self.tree.query(
                [(self.car_position.x, self.car_position.y)])
            index_closest = indexes[0]
            closest_x = self.traj_coords[index_closest][0]
            closest_y = self.traj_coords[index_closest][1]
            self.draw_closest_traj_point(marker, closest_x, closest_y)


if __name__ == '__main__':

    rospy.init_node('car_controller', anonymous=True)

    car_controller = CarController()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
