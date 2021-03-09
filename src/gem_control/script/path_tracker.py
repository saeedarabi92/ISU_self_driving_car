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


class PATH_TRACKER():

    def __init__(self):
        # node name
        rospy.init_node('path_tracker')
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
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.sub_marker = rospy.Subscriber(
            '/trajectory', MarkerArray, self.callback_marker)
        # self.sub_temp = rospy.Subscriber('/topic_temp', Message_temp, self.callback_topic_temp)
        # Publishers
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        # publishing rate
        self.pub_cmd_vel_rate = 10
        # self.pub_temp = rospy.Publisher("/topic_temp", Message_temp, queue_size=queue_size_temp)
        # self.pub_temp = pub_temp_rate

    def tracking_computation(self):
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

    def callback_marker(self, data):
        cx = []
        cy = []
        for marker in data.markers:
            cx.append(marker.pose.position.x)
            cy.append(marker.pose.position.y)
        self.course = [(i, j, None, None) for i, j in zip(cx, cy)]

    def callback_odom(self, data):
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        quaternion = (qx, qy, qz, qw)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.v = data.twist.twist.linear.x
        self.w = data.twist.twist.angular.z

    # def callback_topic_temp(self, data):
    #     self.callback_topic_temp_value = data.data

    # def pub_temp_update(self, event=None):
        # self.pub_temp_update.publish(self.message)

    def pub_cmd_vel_update(self, event=None):
        if self.course != []:
            self.tracking_computation()
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = self.cmd_v, self.cmd_w
            self.pub_cmd_vel.publish(cmd)

    def run(self):
        rospy.Timer(rospy.Duration(1. / self.pub_cmd_vel_rate),
                    self.pub_cmd_vel_update)
        # rospy.Time(rospy.Duration(1./self.pub_temp_rate), self.pub_temp_update)
        rospy.spin()


if __name__ == '__main__':
    try:
        path_tracker = PATH_TRACKER()
        path_tracker.run()
    except rospy.ROSInterruptException:
        pass
