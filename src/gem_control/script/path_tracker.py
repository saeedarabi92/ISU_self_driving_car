#!/usr/bin/env python

# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    path_tracker.py                                    :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: saeed <arabi@iastate.edu>                  +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2021/03/08 20:06:32 by saeed             #+#    #+#              #
#    Updated: 2021/03/08 20:06:32 by saeed            ###   ########.fr        #
#                                                                              #
# **************************************************************************** #


from helper.path_tracking_helper import PURE_PURSUIT
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from helper.callback_helper import callback_odom, callback_markerarray


class PATH_TRACKER():

    def __init__(self):
        # node name
        rospy.init_node('path_tracker', anonymous=False)
        # initial values
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0
        self.w = 0
        self.course = []
        self.cmd_v = 0
        self.cmd_w = 0
        # subscribers
        self.sub_odom = rospy.Subscriber(
            '/odom', Odometry, self.read)
        self.sub_marker = rospy.Subscriber(
            '/trajectory', MarkerArray, self.read)
        # Publishers
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        # publishing rate
        self.pub_cmd_vel_rate = 10

    def read(self, data):
        msg_type = str(data._type)
        if msg_type == 'nav_msgs/Odometry':
            self.x, self.y, self.yaw, self.v, self.w, self.got_new_odom = callback_odom(
                data)
        if msg_type == 'visualization_msgs/MarkerArray':
            self.course = callback_markerarray(
                data)

    def publish(self, event=None):
        if self.course != []:
            self.computation()
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = self.cmd_v, self.cmd_w
            self.pub_cmd_vel.publish(cmd)

    def run(self):
        rospy.Timer(rospy.Duration(1. / self.pub_cmd_vel_rate),
                    self.publish)
        rospy.spin()

    def computation(self):
        """
        Your code goes here!
        inputs:
        1. robot pose, orientation, linear, and angular velocities  (self.x, self.y, self.yaw, self.v, self.w)
        2. Path information: an array consists of tupels of referenced values, self.course: [(x1, y1, theta1, v1), (x2, y2, theta2, v2), ...]
        output:
        1. Twist linear and angular velocities: cmd_v, cmd_w
        """
        pure_persuit = PURE_PURSUIT(
            self.x, self.y, self.yaw, self.v, self.w, self.course)
        self.cmd_v, self.cmd_w = pure_persuit.get_velocity_comands()


if __name__ == '__main__':
    try:
        path_tracker = PATH_TRACKER()
        path_tracker.run()
    except rospy.ROSInterruptException:
        pass
