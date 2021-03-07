#!/usr/bin/env python

"""
Path tracking with pure pursuit steering and PID speed control.
author: Saeed Arabi (saeed.arabi93@gmail.com)
"""

import numpy as np
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
import matplotlib.pyplot as plt



class Pure_pursuit:
    """docstring for Pure_pursuit."""

    def __init__(self):
    # Parameters
        self.k = 0.1  # look forward gain
        self.Lfc = .7 #.3 #2.0  # [m] look-ahead distance
        self.Kp = 1  # speed proportional gain
        self.dt = 0.1  # [s] time tick
        self.WB = 1.5  # [m] wheel base of vehicle
        self.v = 0     #linear_speed
        self.w = 0     #angular_speed
        self.max_linear = 1 #max_linear_speed
        self.max_angular = 10 #max_angular_speed
        self.alpha_plot = []
        self.yaw_plot = []
        self.at2_plot = []
        self.curvature_plot = []
        self.w_plot = []
        self.v_plot = []
        self.rear_x = 0.
        self.rear_y = 0.
        self.yaw = 0.
        self.old_nearest_point_index = None
        self.target_ind = 0
        self.ind = None
        self.lat = 0.
        self.long = 0.
        self.path_size = 5.
        # self.cx, self.cy = self.draw_line()
        self.cx, self.cy = [], []

    # Publisher for 'drive_parameters' (speed and steering angle)
        self.sub = rospy.Subscriber('/trajectory', MarkerArray, self.callback_trajectory)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback)


    
    def callback_trajectory(self, data):
        for marker in data.markers:
            self.cx.append(marker.pose.position.x)
            self.cy.append(marker.pose.position.y)

        # print([(i,j) for i, j in zip(self.cx,self.cy)])



    def callback(self,data):

        qx=data.pose.pose.orientation.x
        qy=data.pose.pose.orientation.y
        qz=data.pose.pose.orientation.z
        qw=data.pose.pose.orientation.w
        quaternion = (qx,qy,qz,qw)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.yaw = euler[2]
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.v = data.twist.twist.linear.x
        self.w = data.twist.twist.angular.z
        self.rear_x = x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = y - ((self.WB / 2) * math.sin(self.yaw))
        # self.rear_x = x 
        # self.rear_y = y 

        cmd_v = self.proportional_control(self.max_linear, self.v)
        curvature, self.target_ind = self.pure_pursuit_steer_control()
        # self.curv_plot.append(curvature)
        # print "curvature: ", curvature
        # print "self.v: ", self.v
        # print "self.w: ", self.w
        cmd_w = curvature * self.max_linear
        cmd_v = np.clip(cmd_v, 0, self.max_linear)
        cmd_w = np.clip(cmd_w, -self.max_angular, self.max_angular)
        # print "self.w: ", self.w
        cmd = Twist()
        if self.target_ind > len(self.cx):
            cmd.linear.x, cmd.angular.z = 0., 0.
        else:
            cmd.linear.x, cmd.angular.z = cmd_v, cmd_w
        #print "cmd_w: ", cmd.angular.z
        self.w_plot.append(cmd_w)
        self.v_plot.append(cmd_v)
        self.pub.publish(cmd)


    def draw_line(self):
        # c1 = {'lat': self.lat, 'lon': self.long}
        # c2 = {'lat': self.goal[0], 'lon': self.goal[1]}
        # yaw = cal_bearing_ENU(c1, c2)
        # self.yaw
        # lats = []
        # lons = []
        # c = {'lat': self.lat, 'lon': self.long}
		# for j in range(self.path_size * 100):
    	# 	c = getCoord(c,1.0/100,0.)
    	# 	lats.append(c['lat'])
    	# 	lons.append(c['lon'])

#        theta = np.linspace(0, 2*np.pi, 100)
        # the radius of the circle
#        r = np.sqrt(5)
        # compute x1 and x2
 #       cx = r*np.cos(theta)
  #      cy = r*np.sin(theta)
      
        cx = np.linspace(0,5, 1000)
        cy = np.linspace(0,1, 1000)

        return cx, cy

    def search_target_index(self):
        
        
        # print "self.old_nearest_point_index", self.old_nearest_point_index
        # print "self.ind", self.ind        
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [self.rear_x - icx for icx in self.cx]
            dy = [self.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            self.ind = np.argmin(d)
            self.old_nearest_point_index = self.ind

        if self.ind + 1 == len(self.cx):
            # print self.cx
            # print self.cy
            rospy.loginfo("Reaching end of path! Exiting the node")
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = 0., 0.
            self.pub.publish(cmd)
            # plt.figure("alpha")
            # plt.plot(self.alpha_plot)
            # plt.ylim((-7,7))
            # plt.figure("omega")
            # plt.plot(self.at2_plot)
            # plt.ylim((-7,7))
            # plt.figure("yaw")
            # plt.plot(self.yaw_plot)
            # plt.ylim((-7, 7))
            # plt.figure("curvature")
            # plt.plot(self.curvature_plot)
            # plt.ylim((-3, 3))
            # plt.figure("self.w_plot")
            # plt.plot(self.w_plot)
            # plt.ylim((-1, 1))
            # plt.figure("self.v_plot")
            # plt.plot(self.v_plot)
            # plt.ylim((-1, 1))
            # plt.show()
            rospy.signal_shutdown("End of path!")

        else:
            self.ind = self.old_nearest_point_index
            distance_this_index = self.calc_distance(self.cx[self.ind],
                                                      self.cy[self.ind])
            while True:
                distance_next_index = self.calc_distance(self.cx[self.ind + 1],
                                                          self.cy[self.ind + 1])
                if distance_this_index < distance_next_index:
                    break
                self.ind = self.ind + 1 if (self.ind + 1) < len(self.cx) else self.ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = self.ind
        
        Lf = self.k * self.v + self.Lfc  # update look ahead distance
        # print "Lf: ", Lf
        # search look ahead target point index
        while Lf > self.calc_distance(self.cx[self.ind], self.cy[self.ind]):
            if (self.ind + 1) >= len(self.cx):
                break  # not exceed goal
            self.ind += 1
#        ind = int(ind)
        # print "ind: ", self.ind
        return self.ind, Lf


    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


    def pure_pursuit_steer_control(self):
        self.ind, Lf = self.search_target_index()
#        ind = int(ind)
        if self.target_ind >= self.ind:
            self.ind = self.target_ind
        #print "ind"
        #print ind
 #       ind = int(ind)
        if self.ind < len(self.cx):
            tx = self.cx[self.ind]
            ty = self.cy[self.ind]
        else:  # toward goal
            tx = self.cx[-1]
            ty = self.cy[-1]
            self.ind = len(self.cx) - 1
        # print "self.rear_y: ", self.rear_y
        # print "self.rear_x: ", self.rear_x
        # print "self.yaw ", self.yaw
        
        #print "ty - self.rear_y: ", ty - self.rear_y
        #print "tx - self.rear_x: ", tx - self.rear_x
        if self.yaw < 0:
            self.yaw = self.yaw + 2*np.pi
        omega = math.atan2(ty - self.rear_y, tx - self.rear_x)
        if omega < 0:
            omega = omega + 2*np.pi
        alpha = omega - self.yaw
        self.at2_plot.append(omega)
        self.yaw_plot.append(self.yaw)
        self.alpha_plot.append(alpha)
        #print "omega: ", omega
        #print "yaw: ", self.yaw

        curvature = 2.0 * math.sin(alpha) / Lf
        #print "curvature: ", curvature

        self.curvature_plot.append(curvature)

        return curvature, self.ind

    def proportional_control(self, target, current):
        # a = self.Kp * (target - current)
        # linear_speed = current + a
        return self.max_linear

    def generate_path():
        pass


def main():
    print "Running pure_pursuit.py"
    rospy.init_node('pure_pursuit', anonymous=True)
    pp = Pure_pursuit()
    try:
        # print "THis is test"  # This only ran one time
        rospy.spin()
    except KeyboardInterrupt:
         print "Shutting down ROS Navigator module"

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
