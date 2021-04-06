#!/usr/bin/env python

# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    lidar_object_detection.py                          :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: saeed <arabi@iastate.edu>                  +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2021/04/05 11:43:46 by saeed             #+#    #+#              #
#    Updated: 2021/04/05 13:20:55 by saeed            ###   ########.fr        #
#                                                                              #
# **************************************************************************** #


import rospy
import tf
import math
from sensor_msgs.msg import PointCloud2, PointField, Image
from gem_perception.msg import DetectedObjectsArray, DetectedObject
from helper.plc_helper import *
from helper.callback_helper import callback_detected_objects_3d, detection_using_pointcloud, callback_pcl
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray



class Lidar_object_detection():

    def __init__(self):
        # node name
        rospy.init_node('lidar_object_detection', anonymous=False)
        self.pcl_clouds = None
        self.clusters = None
        self.detected_objects = None
        self.bboxes_3d = None
        # subscribers
        self.sub_lidar = rospy.Subscriber(
            '/velodyne_points', PointCloud2, self.read)

        # publishers
        self.pub_detected_objects_3d_bbox = rospy.Publisher(
            "/3d_objects_debug", MarkerArray, queue_size=1)
        self.pub_detected_objects = rospy.Publisher(
            "/3d_objects", DetectedObjectsArray, queue_size=1)
        self.pub_point = rospy.Publisher(
            "/cluster", PointCloud2, queue_size=1)
        # publishing rate
        self.pub_rate = 10

    def read(self, data):
        msg_type = str(data._type)
        if msg_type == 'sensor_msgs/PointCloud2':
            self.pcl_clouds = callback_pcl(data)

    def publish(self, event=None):
        self.compute()
        self.pub_point.publish(self.clusters)
        self.pub_detected_objects.publish(self.detected_objects)
        self.pub_detected_objects_3d_bbox.publish(
            self.bboxes_3d)

    def run(self):
        rospy.Timer(rospy.Duration(1. / self.pub_rate),
                    self.publish)
        rospy.spin()

    def compute(self):
        self.clusters, self.detected_objects = detection_using_pointcloud(
            self.pcl_clouds)
        self.bboxes_3d = callback_detected_objects_3d(self.detected_objects)

if __name__ == '__main__':
    try:
        lidar_object_detection = Lidar_object_detection()
        rospy.loginfo(
            "Running Lidar object detection")
        lidar_object_detection.run()
    except rospy.ROSInterruptException:
        pass
