#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray, Marker
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import subprocess
import os
import time
import math 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import numpy as np
import math



class Create_trajectory():

    def __init__(self):
       
        rate = rospy.Rate(1)
        self.yaw_current = None
        rospy.on_shutdown(self.shutdown)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal_subscriber = rospy.Subscriber("/goal", PoseStamped, self.new_goal_callback)
        self.path_publisher = rospy.Publisher("/trajectory", MarkerArray, queue_size=1)

    def get_path_array(self):
        path = MarkerArray()
        pose_id = 0
        idx = 0
        self.get_path() 
        while True:
            x = self.path[idx][0]
            y = self.path[idx][1]      
            pose_marker = self.to_marker(x, y)
            pose_marker.id = pose_id
            path.markers.append(pose_marker)
            pose_id += 1
            idx += 1
            if idx == len(self.path):
                break
        return path        


    def get_path(self, step=0.05):
        """
        bearing         value
            ^              0
            >            -pi/2
            <             pi/2
        downward arrow  -pi, pi
----------------------------------------
            yaw           value

            ^             -pi, pi
            >              pi/2
            <             -pi/2
        downward arrow      0 
        """
        dx = self.x_goal - self.x_current
        dy = self.y_goal - self.y_current
        bearing = math.atan2(dy, dx)
        distance_between_points = math.sqrt(dx ** 2 + dy ** 2)
        # print "distance_between_points: ", distance_between_points
        # print "self.x_goal", self.x_goal
        # print "self.y_goal", self.y_goal
        # print "self.x_current", self.x_current
        # print "self.y_current", self.y_current
        # print "dx", dx
        # print "dy", dy
        # print "self.yaw", self.yaw_current
        # print "bearning", bearing
        if (self.yaw_current > 0 and self.yaw_current < np.pi / 4) or (self.yaw_current < 0 and self.yaw_current > -np.pi / 4):
            if bearing < -np.pi / 4 and bearing > -3 * np.pi / 4:
                theta = np.linspace(-np.pi/2, np.pi/2, 1000)
                r = distance_between_points/2
                cx = r * np.cos(theta)
                cy = r * np.sin(theta)
                x_list = [self.x_current + i - cx[-1] for i in cx]   
                y_list = [self.y_current + i - cy[-1] for i in cy]
                x_list = list(reversed(x_list))
                y_list = list(reversed(y_list))
                
            elif bearing > np.pi / 4 and bearing < 3 * np.pi / 4:
                theta = np.linspace(-np.pi/2, np.pi/2, 1000)
                r = distance_between_points/2
                cx = r * np.cos(theta)
                cy = r * np.sin(theta)
                x_list = [self.x_current + i - cx[0] for i in cx]   
                y_list = [self.y_current + i - cy[0] for i in cy]

            else:
                x_list = []
                y_list = []
                for p in np.arange(0,distance_between_points, step):
                    x_list.append(self.x_current + p * math.cos(bearing))
                    y_list.append(self.y_current + p * math.sin(bearing))

                

        if (self.yaw_current > 3*np.pi/4 and self.yaw_current < np.pi ) or (self.yaw_current > -np.pi and self.yaw_current < -3*np.pi/4):

            if bearing < -np.pi / 4 and bearing > -3 * np.pi / 4:
                theta = np.linspace(-np.pi/2, np.pi/2, 1000)
                r = distance_between_points/2
                cx = r * np.cos(theta)
                cy = r * np.sin(theta)
                x_list = [-(-self.x_current + i - cx[0]) for i in cx]   
                y_list = [-(-self.y_current + i - cy[0]) for i in cy]
               
            elif bearing > np.pi / 4 and bearing < 3 * np.pi / 4:
                theta = np.linspace(-np.pi/2, np.pi/2, 1000)
                r = distance_between_points/2
                cx = r * np.cos(theta)
                cy = r * np.sin(theta)
                x_list = [-(-self.x_current + i - cx[-1]) for i in cx]   
                y_list = [-(-self.y_current + i - cy[-1]) for i in cy]
                x_list = list(reversed(x_list))
                y_list = list(reversed(y_list))

            else:
                x_list = []
                y_list = []
                for p in np.arange(0,distance_between_points, step):
                    x_list.append(self.x_current + p * math.cos(bearing))
                    y_list.append(self.y_current + p * math.sin(bearing))
                    # x_list = list(reversed(x_list))
                    # y_list = list(reversed(y_list))

        # dx = self.x_goal - self.x_current
        # dy = self.y_goal - self.y_current
        # bearing = math.atan2(dy, dx)
        # distance_between_points = math.sqrt(dx ** 2 + dy ** 2)
        # print "distance_between_points: ", distance_between_points
        # print "self.x_goal", self.x_goal
        # print "self.y_goal", self.y_goal
        # print "self.x_current", self.x_current
        # print "self.y_current", self.y_current
        # print "dx", dx
        # print "dy", dy

        # print "bearing", bearing
        # if bearing > 1 and bearing < 2: #turn right 
        #     print "turn right"
        #     print "self.x_current", self.x_current
        #     print "self.y_current", self.y_current

        #     theta = np.linspace(0, np.pi, 1000)
        #     # the radius of the circle
        #     r = distance_between_points/2
        #     cy = r * np.cos(theta)
        #     cx = r * np.sin(theta)
        #     print "cx[0]", cx[0]   #0
        #     print "cy[0]", cy[0]   #2.5
        #     print "cx[-1]", cx[-1]  #0
        #     print "cy[-1]", cy[-1]  #-2.5
        #     print "self.x_current", self.x_current
        #     print "self.y_current", self.y_current

        #     x_shift = self.x_current - cx[0]  # 5 - 0   = 5
        #     y_shift = self.y_current - cy[0]  # 0 - 2.5 = -2.5
        #     # print y_shift
        #     x_list = [i + x_shift for i in cx]   
        #     y_list = [i + y_shift for i in cy]
        #     print "x_list[0]", x_list[0]
        #     print "y_list[0]", y_list[0]
        #     print "x_list[-1]", x_list[-1]
        #     print "y_list[-1]", y_list[-1]

        # elif bearing > -2 and bearing < -1:  #turn left
        #     print "turn left"  
        #     theta = np.linspace(0, np.pi, 1000)
        #     # the radius of the circle
        #     r = distance_between_points/2
        #     cy = r * np.cos(theta)
        #     cx = r * np.sin(theta)
        #     print "cx[0]", cx[0]
        #     print "cy[0]", cy[0]
        #     print "cx[-1]", cx[-1]
        #     print "cy[-1]", cy[-1]
            
        #     x_shift = self.x_current - cx[0]
        #     y_shift = self.y_current - cy[0]
        #     # print y_shift
        #     x_list = [i + x_shift for i in cx]
        #     y_list = [-(i + y_shift) for i in cy]
        #     print "x_list[0]", x_list[0]
        #     print "y_list[0]", y_list[0]
        #     print "x_list[-1]", x_list[-1]
        #     print "y_list[-1]", y_list[-1]
        # else:  #straight line
        #     print "straight line"
        #     x_list = []
        #     y_list = []
        #     for p in np.arange(0,distance_between_points, step):
        #         x_list.append(self.x_current + p * math.cos(bearing))
        #         y_list.append(self.y_current + p * math.sin(bearing))

        # print "x_list", x_list
        # print "y_list", y_list
        print "x_list: ", x_list[0]
        print "y_list: ", y_list[0]
        self.path = zip(x_list, y_list)  




    def to_marker(self, x, y, theta = 0):
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
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1
        marker.type = Marker.CUBE
        marker.action = marker.ADD
        # marker.lifetime = rospy.Duration(0)
        return marker


    def new_goal_callback(self, data):
        rospy.loginfo("Getting a new path...")
        qx=data.pose.orientation.x
        qy=data.pose.orientation.y
        qz=data.pose.orientation.z
        qw=data.pose.orientation.w
        quaternion = (qx,qy,qz,qw)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw_goal = euler[2]
        self.x_goal = data.pose.position.x
        self.y_goal = data.pose.position.y
        try:
            self.path_array = self.get_path_array()
            rospy.loginfo("Path array generated...")
            self.path_publisher.publish(self.path_array)
        except:
            pass


    def odom_callback(self, data):
        qx=data.pose.pose.orientation.x
        qy=data.pose.pose.orientation.y
        qz=data.pose.pose.orientation.z
        qw=data.pose.pose.orientation.w
        quaternion = (qx,qy,qz,qw)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw_current = euler[2]
        # print "self.yaw_current :", self.yaw_current
        self.x_current = data.pose.pose.position.x
        self.y_current = data.pose.pose.position.y


    def shutdown(self):
        rospy.sleep(1)
    
    def loop(self):
        rospy.logwarn("Starting Loop...")
        rospy.spin()
        


if __name__ == "__main__": 
    rospy.init_node('planner', anonymous=False)
    my_subs = Create_trajectory()
    my_subs.loop()