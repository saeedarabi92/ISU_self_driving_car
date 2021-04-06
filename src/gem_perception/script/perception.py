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
import sensor_msgs.point_cloud2 as pc2
from vision_msgs.msg import ObjectHypothesisWithPose, Detection3DArray, Detection3D, Detection2DArray
from visualization_msgs.msg import MarkerArray


class PERCEPTION():

    def __init__(self):
        # node name
        rospy.init_node('perception', anonymous=False)
        self.pcl_clouds = None
        self.clusters = None
        self.detected_objects = None
        self.bboxes_3d = None
        self.bboxes_2d = None
        # subscribers
        self.sub_3d_objects = rospy.Subscriber(
            "/3d_objects", DetectedObjectsArray, self.read)
        self.sub_2d_objects = rospy.Subscriber("/2d_objects", Detection2DArray, self.read)
        # publishers
        self.fused_objects_debug = rospy.Publisher(
            "/fused_objects_debug", MarkerArray, queue_size=1)
        self.fused_objects = rospy.Publisher(
            "/fused_objects", Detection3DArray, queue_size=1)
        # publishing rate
        self.pub_rate = 10

    def read(self, data):
        msg_type = str(data._type)
        if msg_type == "gem_perception/DetectedObjectsArray":
            self.bboxes_3d = data
        if msg_type == "vision_msgs/Detection2DArray":
            self.bboxes_2d = data

    def publish(self, event=None):
        self.compute()
        # self.fused_objects_debug.publish(self.bbox_3d_fused_debug)
        # self.fused_objects.publish(self.bbox_3d_fused)
        pass

    def run(self):
        rospy.Timer(rospy.Duration(1. / self.pub_rate),
                    self.publish)
        rospy.spin()


    def compute(self):
        """
        Your code goes here!
        inputs:
        1. 2d bbox array
        2. 3d bbox array
        output:
        1. 3d bbox array 
        2. 3d bbox marker array 
        """
        # self.bbox_3d_fused = fuse_detections(self.bboxes_3d, self.bboxes_2d)
        # self.bbox_3d_fused_debug = fused_detections_debug(self.bbox_3d_fused)
        pass

if __name__ == '__main__':
    try:
        perception = PERCEPTION()
        rospy.loginfo("Perception: Waiting for 5 sec. to load the all the files")
        rospy.sleep(5)
        perception.run()
    except rospy.ROSInterruptException:
        pass
