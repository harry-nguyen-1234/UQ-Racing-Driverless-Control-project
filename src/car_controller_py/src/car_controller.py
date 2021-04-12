#!/usr/bin/env python

import numpy as np
import math
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

import rospy
from tf.transformations import quaternion_about_axis
from std_msgs.msg import String
from fssim_messages.msg import Command, State, WheelSpeed
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker


class CarController:
    def __init__(self):
        self.pub_command = rospy.Publisher(
            "chicken/cmd", Command, latch=True, queue_size=1)

        self.pub_position = rospy.Publisher(
            "chicken/position", Marker, latch=True, queue_size=1)

        rospy.Subscriber("chicken/trajectory", PolygonStamped,
                         self.callback_trajectory, queue_size=1)

        rospy.Subscriber("chicken/state", State,
                         self.callback_state, queue_size=1)

        self.car_position = None
        self.car_velocity = None
        self.car_acceleration = None

        self.traj_coords = None
        self.traj_closet_index = 0
        self.traj_closest_x = 0
        self.traj_closest_y = 0
        self.tree = None

        self.lookahead_dist = 1
        self.lookahead_index = 0
        self.lookahead_x = 0
        self.lookahead_y = 0

        self.MARKER_POSITION = 0
        self.MARKER_HEADING = 1
        self.MARKER_CLOSEST_TRAJ = 2
        self.MARKER_LOOKAHEAD = 3

    def draw_car_position(self, marker):
        marker.id = self.MARKER_POSITION
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.car_position.x
        marker.pose.position.y = self.car_position.y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
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
        marker.id = self.MARKER_HEADING
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        q = quaternion_about_axis(self.car_position.theta, (0, 0, 1))
        marker.pose.position.x = self.car_position.x
        marker.pose.position.y = self.car_position.y
        marker.pose.position.z = 0
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

    def draw_closest_traj_point(self, marker):
        marker.id = self.MARKER_CLOSEST_TRAJ
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.traj_closest_x
        marker.pose.position.y = self.traj_closest_y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 0.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 1.0

        self.pub_position.publish(marker)

    def draw_lookahead_point(self, marker):
        marker.id = self.MARKER_LOOKAHEAD
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.lookahead_x
        marker.pose.position.y = self.lookahead_y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 1.0

        self.pub_position.publish(marker)

    def update_closest_traj(self, marker):
        _, indexes = self.tree.query(
            [(self.car_position.x, self.car_position.y)])
        self.traj_closet_index = indexes[0]
        self.traj_closest_x = self.traj_coords[self.traj_closet_index][0]
        self.traj_closest_y = self.traj_coords[self.traj_closet_index][1]
        self.draw_closest_traj_point(marker)

    def update_lookahead(self, marker):
        self.lookahead_index = self.traj_closet_index
        while(True):
            self.lookahead_x = self.traj_coords[self.lookahead_index][0]
            self.lookahead_y = self.traj_coords[self.lookahead_index][1]
            chord_length = math.hypot((self.traj_closest_x - self.lookahead_x),
                                      (self.traj_closest_y - self.lookahead_y))
            if chord_length > self.lookahead_dist:
                break

            self.lookahead_index = (
                self.lookahead_index + 1) % len(self.traj_coords)

        self.draw_lookahead_point(marker)

    def drive(self, marker):
        car_velocity = math.hypot(self.car_velocity.x, self.car_velocity.y)
        velocity_cmd = 5 - car_velocity

        x_offset = self.car_position.x - self.lookahead_x
        y_offset = self.car_position.y - self.lookahead_y
        theta = math.atan2(y_offset, x_offset)
        car_heading = self.car_position.theta
        angle_offset = car_heading - theta

        dist_to_point = math.hypot(x_offset, y_offset)
        curvature = 2 * math.sin(angle_offset) / dist_to_point

        steering_angle = curvature
        if steering_angle > 1:
            steering_angle = 1
        elif steering_angle < -1:
            steering_angle = -1

        angle_cmd = steering_angle

        cmd = Command()
        cmd.header.stamp = rospy.get_rostime()
        cmd.header.frame_id = "map"
        cmd.throttle = velocity_cmd
        cmd.steering_angle = angle_cmd

        self.pub_command.publish(cmd)

    def pure_pursuit(self, marker):
        self.update_closest_traj(marker)
        self.update_lookahead(marker)
        self.drive(marker)

    def callback_trajectory(self, trajectory):
        x = [point.x for point in trajectory.polygon.points]
        y = [point.y for point in trajectory.polygon.points]
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
            self.pure_pursuit(marker)


if __name__ == '__main__':

    rospy.init_node('car_controller', anonymous=True)

    car_controller = CarController()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
