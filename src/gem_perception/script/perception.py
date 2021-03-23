#!/usr/bin/env python

# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    perception.py                                      :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: saeed <arabi@iastate.edu>                  +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2021/03/15 15:53:24 by saeed             #+#    #+#              #
#    Updated: 2021/03/15 15:53:24 by saeed            ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

import rospy
import tf
import math
from sensor_msgs.msg import PointCloud2, PointField, Image
from gem_perception.msg import DetectedObjectsArray, DetectedObject
from helper.plc_helper import *
from helper.callback_helper import *
import sensor_msgs.point_cloud2 as pc2


class PERCEPTION():

    def __init__(self):
        # node name
        rospy.init_node('perception', anonymous=False)
        self.pcl_clouds = None
        self.clusters = None
        self.detected_objects = None
        self.bboxes_3d = None
        # subscribers
        self.sub_lidar = rospy.Subscriber(
            '/velodyne_points', PointCloud2, self.read)
        self.sub_image = rospy.Subscriber(
            "/gem/front_single_camera/front_single_camera/image_raw", Image, self.read)
        self.sub_image = rospy.Subscriber(
            "/detected_objects", DetectedObjectsArray, self.read)
        # publishers
        self.pub_detected_objects_3d_bbox = rospy.Publisher(
            "/detected_objects_3d_bbox", MarkerArray, queue_size=1)
        self.pub_detected_objects = rospy.Publisher(
            "/detected_objects", DetectedObjectsArray, queue_size=1)
        self.pub_point = rospy.Publisher(
            "/cluster", PointCloud2, queue_size=1)
        # publishing rate
        self.pub_rate = 10


    def read(self, data):
        msg_type = str(data._type)
        if msg_type == 'sensor_msgs/PointCloud2':  
            self.pcl_clouds = callback_pcl(data)
            # self.detected_objects, self.clusters = callback_pcl(data)
        if msg_type == "gem_perception/DetectedObjectsArray":
            self.bboxes_3d = callback_detected_objects(data)

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
        """
        Your code goes here!
        inputs:
        1. data with type "pcl._pcl.PointCloud_PointXYZRGB",  (the point cloud after preprocessing)
        output:
        1. Detected objects, pcl point cloud 
        2. Detected objects, ros point cloud 
        """
        self.clusters, self.detected_objects = detection_using_pointcloud(self.pcl_clouds)
        # print(self.detected_objects)
        

if __name__ == '__main__':
    try:
        perception = PERCEPTION()
        rospy.loginfo("Perception: Waiting for 5 sec. to load the all the files")
        rospy.sleep(5)
        perception.run()
    except rospy.ROSInterruptException:
        pass
