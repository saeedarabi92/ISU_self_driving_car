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


global goal_list
goal_list = [(10,0), (10,10), (0,10), (0,0)]
def get_goal(idx):
    goal = PoseStamped()
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "/odom"
    goal.pose.position.x = goal_list[idx][0] 
    goal.pose.position.y = goal_list[idx][1]
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.0
    return goal

def odom_callback(data):
    global x, y, yaw
    qx=data.pose.pose.orientation.x
    qy=data.pose.pose.orientation.y
    qz=data.pose.pose.orientation.z
    qw=data.pose.pose.orientation.w
    quaternion = (qx,qy,qz,qw)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    
def distance_to_goal(x_c, y_c, x_g, y_g):
    return math.sqrt( (x_c - x_g)**2 + (y_c - y_g)**2 )


def mission():

    pub = rospy.Publisher('/goal', PoseStamped, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.init_node('mission', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    goal_idx = 0
    laps_num = 2
    rospy.loginfo("Waiting 5 seconds to load all the nodes...!")
    rospy.sleep(5)
    for j in range(laps_num):
        for i in range(len(goal_list)):
            goal = get_goal(i)
            x_g = goal.pose.position.x
            y_g = goal.pose.position.y
            pub.publish(goal)
            distance = distance_to_goal(x, y, x_g, y_g)
            while distance > 1:
                distance = distance_to_goal(x, y, x_g, y_g)
            print "Approaching to the next goal..."
            rospy.sleep(1)
    rospy.loginfo("Mission ended. Killing all the ROS nodes...!")
    os.system("rosnode kill --all & killall -9 roscore & killall -9 rosmaster & killall -9 rviz & pkill -9 python")

if __name__ == '__main__':
    try:
        mission()
    except rospy.ROSInterruptException:
        pass