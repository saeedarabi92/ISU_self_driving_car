#!/usr/bin/env python

# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    odom_to_baselink_tf.py                             :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: saeed <arabi@iastate.edu>                  +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2021/03/08 18:17:24 by saeed             #+#    #+#              #
#    Updated: 2021/03/08 18:17:24 by saeed            ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

import rospy
import math
import sys
import tf2_ros
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

global tf_pub
tf_pub = tf2_ros.TransformBroadcaster()


def odom_callback(data):

    odom = Odometry()
    odom.header.frame_id = '/odom'
    odom.child_frame_id = '/base_footprint'
    odom.header.stamp = rospy.Time.now()
    odom.pose = data.pose
    odom.pose.pose.position.x = odom.pose.pose.position.x
    odom.pose.pose.position.y = odom.pose.pose.position.y
    odom.twist = data.twist
    tf = TransformStamped(header=Header(
                          frame_id=odom.header.frame_id,
                          stamp=odom.header.stamp),
                          child_frame_id=odom.child_frame_id,
                          transform=Transform(
                          translation=odom.pose.pose.position,
                          rotation=odom.pose.pose.orientation))

    tf_pub.sendTransform(tf)


if __name__ == '__main__':
    try:
        rospy.init_node('odom_to_baselink_tf', anonymous=True)
        rospy.Subscriber('/odom', Odometry, odom_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
