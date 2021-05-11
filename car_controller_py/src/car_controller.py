#!/usr/bin/env python

import numpy as np
import math
from scipy.spatial import KDTree
import colorsys
from matplotlib import animation
import matplotlib.pyplot as plt

import rospy
from tf.transformations import quaternion_about_axis
from std_msgs.msg import String, ColorRGBA
from fssim_messages.msg import Command, State
from geometry_msgs.msg import PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker

ENABLE_PLOTTING = False


class CarController:
    def __init__(self):
        self.pub_command = rospy.Publisher(
            "chicken/cmd", Command, latch=True, queue_size=1)

        self.pub_visualiser = rospy.Publisher(
            "chicken/visualiser", Marker, latch=True, queue_size=1)

        rospy.Subscriber("chicken/trajectory", PolygonStamped,
                         self.callback_trajectory, queue_size=1)

        rospy.Subscriber("chicken/state", State,
                         self.callback_state, queue_size=1)

        self.car_position = None
        self.car_velocity = None
        self.car_speed = 0.0
        self.car_acceleration = None

        self.traj_coords = None
        self.traj_closest_index = 0
        self.traj_closest_x = 0.0
        self.traj_closest_y = 0.0
        self.tree = None
        self.curvature_map = None
        self.min_curv = 0.0
        self.max_curv = 0.0
        self.max_lookahead_curv = 0.0

        self.lookahead_dist = 1.0
        self.lookahead_index = 0
        self.speed_lookahead_index = 0
        self.lookahead_x = 0.0
        self.lookahead_y = 0.0

        self.target_speed = 5.0

        self.LOOKAHEAD_FACTOR = 5.0

        self.MAX_SPEED = 25.0
        self.MIN_SPEED = 5.0

        self.MARKER_POSITION = 0
        self.MARKER_HEADING = 1
        self.MARKER_CLOSEST_TRAJ = 2
        self.MARKER_LOOKAHEAD = 3
        self.MARKER_CURVATURE = 4
        self.MARKER_STATE = 5
        self.MARKER_SPEED_LOOKAHEAD = 6
        self.MARKER_SPEED_SPLINE = 7

        if ENABLE_PLOTTING:
            self.fig, self.ax = plt.subplots()
            self.dot, = plt.plot([0], [0], 'o', markersize=10)
            self.dot.set_data([], [])

    def pure_pursuit(self):
        self.update_closest_traj()
        self.update_lookahead_dist()
        self.update_lookahead()
        self.update_speed_lookahead()
        self.update_target_speed()
        self.drive()

    # Find the point on the trajectory closest to the current car position.
    def update_closest_traj(self):
        _, indexes = self.tree.query(
            [(self.car_position.x, self.car_position.y)])
        self.traj_closest_index = indexes[0]
        self.traj_closest_x = self.traj_coords[self.traj_closest_index][0]
        self.traj_closest_y = self.traj_coords[self.traj_closest_index][1]

        self.draw_pp_marker(self.traj_closest_x, self.traj_closest_y, ColorRGBA(
            0, 0, 1, 1), self.MARKER_CLOSEST_TRAJ)

    # At the lookahead point is a certain curvature.
    # Based on that curvature, change the lookahead distance.
    # If there is high curvature, lower the lookahead distance to prevent
    # over shooting, and increase lookahead distance for the opposite case.
    def update_lookahead_dist(self):
        min_lookahead_dist = 1.0
        min_radius = 1.0 / self.max_curv
        max_lookahead_dist = min_radius

        # Scale the lookahead distance based on the current car velocity.
        new_dist = self.car_speed / (self.MAX_SPEED - self.MIN_SPEED) * \
            (max_lookahead_dist - min_lookahead_dist)

        if new_dist < min_lookahead_dist:
            new_dist = min_lookahead_dist
        elif new_dist > max_lookahead_dist:
            new_dist = max_lookahead_dist

        self.lookahead_dist = new_dist

    # Find the lookahead point that is a minimum length away from the
    # closest trajectory point.
    def update_lookahead(self):
        self.lookahead_index = (
            self.traj_closest_index + 1) % len(self.traj_coords)
        while(True):
            self.lookahead_x = self.traj_coords[self.lookahead_index][0]
            self.lookahead_y = self.traj_coords[self.lookahead_index][1]
            chord_length = math.hypot((self.traj_closest_x - self.lookahead_x),
                                      (self.traj_closest_y - self.lookahead_y))
            if chord_length > self.lookahead_dist:
                break

            self.lookahead_index = (
                self.lookahead_index + 1) % len(self.traj_coords)

        self.draw_pp_marker(self.lookahead_x, self.lookahead_y,
                            ColorRGBA(1, 0, 1, 1), self.MARKER_LOOKAHEAD)

    # Find the lookahead point that is twice the minimum length away from the
    # closest trajectory point to account for braking distance.
    def update_speed_lookahead(self):
        self.speed_lookahead_index = (
            self.traj_closest_index + 1) % len(self.traj_coords)
        while(True):
            lookahead_x = self.traj_coords[self.speed_lookahead_index][0]
            lookahead_y = self.traj_coords[self.speed_lookahead_index][1]
            chord_length = math.hypot((self.traj_closest_x - lookahead_x),
                                      (self.traj_closest_y - lookahead_y))
            if chord_length > self.lookahead_dist * self.LOOKAHEAD_FACTOR:
                break

            self.speed_lookahead_index = (
                self.speed_lookahead_index + 1) % len(self.traj_coords)

        self.draw_pp_marker(lookahead_x, lookahead_y, ColorRGBA(
            0, 1, 0, 1), self.MARKER_SPEED_LOOKAHEAD)

        self.draw_speed_control_spline()

    # At the lookahead point is a certain curvature.
    # Based on that curvature, change how fast the car should be going.
    # If there is high curvature, lower the speed to prevent
    # over shooting, and increase speed for the opposite case.
    def update_target_speed(self):
        self.max_lookahead_curv = self.get_max_lookahead_curv()

        speed = self.piecewise_linear(self.max_lookahead_curv)

        if speed < self.MIN_SPEED:
            speed = self.MIN_SPEED
        elif speed > self.MAX_SPEED:
            speed = self.MAX_SPEED

        self.target_speed = speed

    # In the interval between the closest trajectory point and the speed
    # lookahead point, find the maximum curvature. A higher curvature would
    # dictate a lower speed.
    def get_max_lookahead_curv(self):
        index = self.traj_closest_index

        curvs = []
        # Loop from the closest trajectory point to the speed lookahead point.
        while (index != self.speed_lookahead_index):
            curvs.append(self.curvature_map[index])
            index = (index + 1) % len(self.curvature_map)

        return np.max(curvs)

    # Send throttle and steering commands.
    def drive(self):
        throttle_cmd = self.set_throttle()
        steering_cmd = self.set_steering()

        cmd = Command()
        cmd.header.stamp = rospy.get_rostime()
        cmd.header.frame_id = "map"
        cmd.throttle = throttle_cmd
        cmd.steering_angle = steering_cmd

        self.pub_command.publish(cmd)

    def set_throttle(self):
        # Scale throttle command from -1 to 1.
        throttle = (self.target_speed - self.car_speed) / self.target_speed

        if throttle < -1:
            throttle = -1
        elif throttle > 1:
            throttle = 1

        return throttle

    def set_steering(self):
        x_offset = self.car_position.x - self.lookahead_x
        y_offset = self.car_position.y - self.lookahead_y
        theta = math.atan2(y_offset, x_offset)
        car_heading = self.car_position.theta
        angle_offset = car_heading - theta

        dist_to_point = math.hypot(x_offset, y_offset)
        curvature = 2.0 * math.sin(angle_offset) / dist_to_point

        steering = curvature
        if steering > 1.0:
            steering = 1.0
        elif steering < -1.0:
            steering = -1.0

        return steering

    # Given a trajectory, calculate its curvature and store as a map/list.
    def update_curvature_map(self):
        # Calculate curvature of trajectory using circumradiuses of three
        # iteratively sampled points.
        curv_map = []
        # Set the search interval to 1% the number of trajectory coordinates.
        search_interval = int(math.ceil(0.01 * len(self.traj_coords)))
        for index_this in range(len(self.traj_coords)):
            index_next = (index_this + search_interval) % len(self.traj_coords)
            index_prev = (index_this + len(self.traj_coords) -
                          search_interval) % len(self.traj_coords)

            point_ax, point_ay = self.traj_coords[index_this]
            point_bx, point_by = self.traj_coords[index_next]
            point_cx, point_cy = self.traj_coords[index_prev]

            dist_AB = math.hypot((point_ax - point_bx), (point_ay - point_by))
            dist_AC = math.hypot((point_ax - point_cx), (point_ay - point_cy))
            dist_BC = math.hypot((point_bx - point_cx), (point_by - point_cy))

            # Sort lengths for numerically stable Heron's formula.
            lengths = np.sort([dist_AB, dist_AC, dist_BC])[::-1]
            a = lengths[0]
            b = lengths[1]
            c = lengths[2]

            denominator = a * b * c
            if denominator == 0:
                curvature = 0
            else:
                # Calculate triangle area using Heron's formula.
                area = 1/4.0 * \
                    math.sqrt((a+(b+c))*(c-(a-b))*(c+(a-b))*(a+(b-c)))
                curvature = 4.0 * area / denominator

            curv_map.append(curvature)

        self.curvature_map = curv_map
        self.min_curv = np.amin(self.curvature_map)
        self.max_curv = np.amax(self.curvature_map)

    # Find a linear mapping between curvature and target speed.
    def linear(self, curv):
        speed = curv / (self.max_curv - self.min_curv) * (self.MAX_SPEED -
                                                          self.MIN_SPEED)

        # Take complement to map low curvature to high speed
        speed = self.MAX_SPEED - speed

        return speed

    # Find a piecewise linear mapping between curvature and target speed.
    def piecewise_linear(self, curv):
        # Perform speed control using piecewise linear functions
        # y = mx + c
        threshold = 0.2
        m_factor = 3.5

        m = (self.MAX_SPEED - self.MIN_SPEED) / \
            (self.min_curv - self.max_curv)
        x_stop = (self.max_curv - self.min_curv) * threshold + self.min_curv

        m1 = m_factor * m
        c1 = self.MAX_SPEED

        y_stop = m1 * x_stop + c1
        m2 = (y_stop - self.MIN_SPEED) / (x_stop - self.max_curv)
        c2 = y_stop - m2 * x_stop

        if curv < x_stop:
            speed = m1 * curv + c1
        else:
            speed = m2 * curv + c2

        return speed

    # Update the plot to show the current speed point on the speed profile.
    def animate(self, frame):
        if self.curvature_map:
            lookahead_curv = self.max_lookahead_curv
        else:
            lookahead_curv = 0

        self.dot.set_data(lookahead_curv, self.target_speed)
        return (self.dot, )

    # Plot the various speed profiles as curvature vs target speed.
    def plot_speed_profiles(self):
        self.ax.set_xlim(self.min_curv, self.max_curv)
        self.ax.set_ylim(self.MIN_SPEED, self.MAX_SPEED)
        self.ax.set_xlabel("Curvature")
        self.ax.set_ylabel("Speed m/s")

        x = np.linspace(self.min_curv, self.max_curv, 100)

        y = []
        for curv in x:
            y.append(self.linear(curv))
        plt.plot(x, y, 'g-', label='Linear')

        y = []
        for curv in x:
            y.append(self.piecewise_linear(curv))
        plt.plot(x, y, 'r-', label='Piecewise linear')

        self.ax.legend(fontsize=16)

    # Draw pure pursuit markers.
    def draw_pp_marker(self, x, y, color_rgba, marker_id):
        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "map"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color = color_rgba

        self.pub_visualiser.publish(marker)

    # Draw the direction the car is facing, with arrow length proportional to
    # current car speed.
    def draw_car_heading(self):
        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "map"
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

        max_arrow_length = 5.0
        arrow_length = self.car_speed / \
            (self.MAX_SPEED - self.MIN_SPEED) * max_arrow_length

        marker.scale.x = arrow_length
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.pub_visualiser.publish(marker)

    # Write the controller state as text.
    # Write current car speed and current lookahead distance.
    def write_car_state(self):
        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "map"
        marker.id = self.MARKER_STATE
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = self.car_position.x
        marker.pose.position.y = self.car_position.y
        marker.pose.position.z = 2.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.5

        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 1.0
        marker.color.a = 1.0

        marker.text = """\
                        Current: %.1f m/s
                        Target: %.1f m/s
                        Lookahead: %.1f m
                        Speed lookahead: %.1f m""" % (self.car_speed,
                                                      self.target_speed,
                                                      self.lookahead_dist,
                                                      self.lookahead_dist * self.LOOKAHEAD_FACTOR)

        self.pub_visualiser.publish(marker)

    # Draw the colour coded map of trajectory curvature.
    def draw_curv_map(self):
        HSV_min = 0
        HSV_max = 240 / 360.0

        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "map"
        marker.id = self.MARKER_CURVATURE
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1

        for i, v in enumerate(self.curvature_map):
            color = v / (self.max_curv - self.min_curv) * (HSV_max - HSV_min)
            # Invert so the largest curvature maps to smallest HSV
            color = HSV_max - HSV_min - color
            r, g, b = colorsys.hsv_to_rgb(color, 1, 1)
            x, y = self.traj_coords[i]

            marker.points.append(Point(x, y, 0))
            marker.colors.append(ColorRGBA(r, g, b, 1))

        self.pub_visualiser.publish(marker)

    # Draw the colour coded map of trajectory curvature.
    def draw_speed_control_spline(self):
        HSV_min = 0
        HSV_max = 240 / 360.0

        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "map"
        marker.id = self.MARKER_SPEED_SPLINE
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        color = self.max_lookahead_curv / \
            (self.max_curv - self.min_curv) * (HSV_max - HSV_min)
        # Invert so the largest curvature maps to smallest HSV
        color = HSV_max - HSV_min - color
        r, g, b = colorsys.hsv_to_rgb(color, 1, 1)
        marker.color = ColorRGBA(r, g, b, 1)

        marker.scale.x = 0.5
        index = self.traj_closest_index

        # Loop from the closest trajectory point to the speed lookahead point.
        while (index != self.speed_lookahead_index):
            x, y = self.traj_coords[index]
            marker.points.append(Point(x, y, 0))
            index = (index + 1) % len(self.traj_coords)

        self.pub_visualiser.publish(marker)

    # Called when ROS sends a new trajectory message.
    def callback_trajectory(self, trajectory):
        x = [point.x for point in trajectory.polygon.points]
        y = [point.y for point in trajectory.polygon.points]
        self.traj_coords = zip(x, y)
        self.tree = KDTree(self.traj_coords)

        self.update_curvature_map()

        if ENABLE_PLOTTING:
            self.plot_speed_profiles()

    # Called when ROS sends a new state message.
    def callback_state(self, state):
        self.car_position = state.position
        self.car_velocity = state.velocity
        self.car_speed = math.hypot(self.car_velocity.x, self.car_velocity.y)
        self.car_acceleration = state.acceleration

        self.draw_pp_marker(self.car_position.x, self.car_position.y, ColorRGBA(
            1, 0, 0, 1), self.MARKER_POSITION)
        self.draw_car_heading()
        self.write_car_state()

        if self.tree is not None:
            self.pure_pursuit()
            # Rviz uses lazy drawing, so continuously redraw curvature map.
            self.draw_curv_map()


if __name__ == '__main__':

    rospy.init_node('car_controller', anonymous=True)

    car_controller = CarController()

    if ENABLE_PLOTTING:
        anim = animation.FuncAnimation(
            car_controller.fig, car_controller.animate, interval=1/30.0 * 1000)

        plt.grid(True, which='both')
        plt.show(block=True)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
