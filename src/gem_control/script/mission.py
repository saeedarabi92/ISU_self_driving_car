#!/usr/bin/env python

# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    mission.py                                         :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: saeed <arabi@iastate.edu>                  +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2021/03/08 20:11:26 by saeed             #+#    #+#              #
#    Updated: 2021/03/08 20:11:26 by saeed            ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import rospy
from std_msgs.msg import String
import tf
import math
import subprocess
import os
import numpy as np
from helper.general import get_goal, distance_to_goal
from helper.callback_helper import callback_odom


class MISSION():
    def __init__(self):
        # node name
        rospy.init_node('mission', anonymous=False)
        self.goal_list = [(2, 0, 0), (2, 2, np.pi), (-2, 2, np.pi), (0, 0, 0)]
        self.laps_num = 1
        # publishers
        self.pub_goal = rospy.Publisher('/goal', PoseStamped, queue_size=1)
        # subscribers
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.read)
        self.pub_goal_rate = 10

    def read(self, data):
        msg_type = str(data._type)
        if msg_type == 'nav_msgs/Odometry':
            self.x_current, self.y_current, self.yaw_current, self.v, self.w, self.got_new_odom = callback_odom(
                data)

    def publish(self, goal):
        self.pub_goal.publish(goal)

    def run(self):
        rospy.loginfo("Waiting 5 seconds to load all the nodes...!")
        rospy.sleep(5)
        for j in range(self.laps_num):
            for i in range(len(self.goal_list)):
                self.computation(i)
        rospy.logwarn("Mission ended. Killing all the ROS nodes...!")
        os.system(
            "rosnode kill --all & killall -9 roscore & killall -9 rosmaster & killall -9 rviz & pkill -9 python")

    def computation(self, i):
        goal = get_goal(self.goal_list, i)
        x_goal = goal.pose.position.x
        y_goal = goal.pose.position.y
        self.publish(goal)
        distance = distance_to_goal(
            self.x_current, self.y_current, x_goal, y_goal)
        while distance > 1:
            distance = distance_to_goal(
                self.x_current, self.y_current, x_goal, y_goal)
        print "Approaching to the next goal..."
        rospy.sleep(.5)


if __name__ == '__main__':
    try:
        mission = MISSION()
        mission.run()
    except rospy.ROSInterruptException:
        pass
