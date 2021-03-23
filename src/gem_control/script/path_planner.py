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
from helper.path_planning_helper import SIMPLE_PATH_PLANNER, DUBINS_PATH_PLANNER, RRTStarDubins
from helper.callback_helper import callback_odom, callback_pose, callback_detected_objects_to_obstacle
from helper.visualizer import path_to_marker, pose_to_marker
from gem_perception.msg import DetectedObjectsArray, DetectedObject
# from gem_perception.script.helper.callback_helper import callback_detected_objects


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
        self.got_new_odom = False
        self.obstacle_list = []
        self.listener = tf.TransformListener()
        # subscribers
        self.sub_odom = rospy.Subscriber(
            '/odometry/filtered_map', Odometry, self.read)
        self.sub_goal = rospy.Subscriber(
            "/goal", PoseStamped, self.read)
        self.sub_detected_objects_3d_bbox = rospy.Subscriber(
            "/detected_objects_3d_bbox", MarkerArray, self.read)
        # publishers
        self.pub_path = rospy.Publisher(
            "/trajectory", MarkerArray, queue_size=5)
        # publishing rate
        self.pub_path_rate = 10

    def read(self, data):
        msg_type = str(data._type)
        if msg_type == 'nav_msgs/Odometry':
            self.x_current, self.y_current, self.yaw_current, self.v, self.w, self.got_new_odom = callback_odom(
                data)
        if msg_type == 'visualization_msgs/MarkerArray':
            self.obstacle_list = callback_detected_objects_to_obstacle(
                data, self.listener)
            print "self.obstacle_list", self.obstacle_list
        if msg_type == 'geometry_msgs/PoseStamped':
            self.x_goal, self.y_goal, self.yaw_goal, self.got_new_goal = callback_pose(
                data)

    def publish(self, event=None):
        if self.got_new_goal:
            self.compute()
            path = path_to_marker(self.path)
            self.pub_path.publish(path)
            self.got_new_goal = False

    def run(self):
        rospy.Timer(rospy.Duration(1. / self.pub_path_rate),
                    self.publish)
        rospy.spin()

    def compute(self):
        """
        Your code goes here!
        inputs:
        1. robot and goal pose and orientation,  (self.x_current, self.y_current, self.yaw_current, self.x_goal, self.y_goal, self.yaw_goal)
        output:
        1. Path information: an array consists of tupels of referenced values, self.path: [(x1, y1, theta1, v1), (x2, y2, theta2, v2), ...]
        """
        # ------------------------Simple path planner---------------------------------------

        # simple_path = SIMPLE_PATH_PLANNER(
        #     self.x_current, self.y_current, self.yaw_current, self.x_goal, self.y_goal, self.yaw_goal)
        # self.path = simple_path.get_path_info()
        
        #-------------------------Dubins path planner---------------------------------------
        dubins_path = DUBINS_PATH_PLANNER(
            self.x_current, self.y_current, self.yaw_current, self.x_goal, self.y_goal, self.yaw_goal)
        self.path = dubins_path.get_path_info()


        #-------------------------RRTstar path planner----------------------------------------------------------
        # rrtstartdubin = RRTStarDubins(
        #     [self.x_current, self.y_current, self.yaw_current], [self.x_goal, self.y_goal, self.yaw_goal], rand_area=[0.0, 20.0], obstacle_list=self.obstacle_list)
        # self.path = rrtstartdubin.get_path_info()


if __name__ == '__main__':
    try:
        path_planner = PATH_PLANNER()
        rospy.loginfo(
            "Perception: Waiting for 7 sec. to load the all the files")
        rospy.sleep(7)
        path_planner.run()
    except rospy.ROSInterruptException:
        pass
