#!/usr/bin/env python

# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    path_planner.py                                    :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: saeed <arabi@iastate.edu>                  +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2021/03/08 18:26:34 by saeed             #+#    #+#              #
#    Updated: 2021/03/08 18:26:34 by saeed            ###   ########.fr        #
#                                                                              #
# **************************************************************************** #


import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import tf
from nav_msgs.msg import Odometry
import math
from helper.path_planning_helper import SIMPLE_PATH_PLANNER, DUBINS_PATH_PLANNER


class PATH_PLANNER():

    def __init__(self):
        # node name
        rospy.init_node('path_planner', anonymous=False)
        self.x_current = 0
        self.y_current = 0
        self.yaw_current = 0
        self.x_goal = None
        self.y_goal = None
        self.yaw_goal = None
        self.path = []
        self.got_new_goal = False
        # subscribers
        self.sub_odom = rospy.Subscriber(
            '/odom', Odometry, self.callback_odom)
        self.sub_goal = rospy.Subscriber(
            "/goal", PoseStamped, self.callback_goal)
        # self.sub_temp = rospy.Subscriber('/topic_temp', Message_temp, self.callback_topic_temp)
        # publishers
        self.pub_path = rospy.Publisher(
            "/trajectory", MarkerArray, queue_size=5)
        # publishing rate
        self.pub_path_rate = 10
        # self.pub_temp = rospy.Publisher("/topic_temp", Message_temp, queue_size=queue_size_temp)
        # self.pub_temp = pub_temp_rate

    def planning_computation(self):
        """
        Your code goes here!
        inputs:
        1. robot and goal pose and orientation,  (self.x_current, self.y_current, self.yaw_current, self.x_goal, self.y_goal, self.yaw_goal)
        output:
        1. Path information: an array consists of tupels of referenced values, self.path: [(x1, y1, theta1, v1), (x2, y2, theta2, v2), ...]
        """
        # simple_path = SIMPLE_PATH_PLANNER(
        #     self.x_current, self.y_current, self.yaw_current, self.x_goal, self.y_goal, self.yaw_goal)
        # self.path = simple_path.get_path_info()

        dubins_path = DUBINS_PATH_PLANNER(
            self.x_current, self.y_current, self.yaw_current, self.x_goal, self.y_goal, self.yaw_goal)
        self.path = dubins_path.get_path_info()

    def callback_odom(self, data):
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        quaternion = (qx, qy, qz, qw)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw_current = euler[2]
        self.x_current = data.pose.pose.position.x
        self.y_current = data.pose.pose.position.y

    def callback_goal(self, data):
        rospy.loginfo("Getting a new path...")
        qx = data.pose.orientation.x
        qy = data.pose.orientation.y
        qz = data.pose.orientation.z
        qw = data.pose.orientation.w
        quaternion = (qx, qy, qz, qw)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw_goal = euler[2]
        self.x_goal = data.pose.position.x
        self.y_goal = data.pose.position.y
        self.got_new_goal = True

    def to_marker(self, x, y, theta):
        marker = Marker()
        marker.header.frame_id = "/odom"
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.scale.x = 0.3
        marker.scale.y = 0.05
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1
        marker.type = Marker.ARROW
        marker.action = marker.ADD
        return marker

    def get_maker_path(self):
        path = MarkerArray()
        pose_id = 0
        idx = 0
        while True:
            x = self.path[idx][0]
            y = self.path[idx][1]
            yaw = self.path[idx][2]
            pose_marker = self.to_marker(x, y, yaw)
            pose_marker.id = pose_id
            path.markers.append(pose_marker)
            pose_id += 1
            idx += 1
            if idx == len(self.path):
                break
        return path

    def pub_path_update(self, event=None):
        if self.got_new_goal:
            self.planning_computation()
            path = self.get_maker_path()
            self.pub_path.publish(path)
            self.got_new_goal = False

    def run(self):
        rospy.Timer(rospy.Duration(1. / self.pub_path_rate),
                    self.pub_path_update)
        # rospy.Time(rospy.Duration(1./self.pub_temp_rate), self.pub_temp_update)
        rospy.spin()


if __name__ == '__main__':
    try:
        path_planner = PATH_PLANNER()
        path_planner.run()
    except rospy.ROSInterruptException:
        pass
