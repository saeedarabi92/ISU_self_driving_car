from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
import tf
import rospy
import numpy as np
from std_msgs.msg import Header



def callback_pose(data):
    # rospy.loginfo("Got a new odometry data...")
    qx = data.pose.orientation.x
    qy = data.pose.orientation.y
    qz = data.pose.orientation.z
    qw = data.pose.orientation.w
    quaternion = (qx, qy, qz, qw)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    x = data.pose.position.x
    y = data.pose.position.y
    got_new_data = True
    return x, y, yaw, got_new_data


def callback_odom(data):
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    quaternion = (qx, qy, qz, qw)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    v = data.twist.twist.linear.x
    w = data.twist.twist.angular.z
    got_new_data = True
    return x, y, yaw, v, w, got_new_data


def callback_markerarray(data):
    c_x = []
    c_y = []
    c_yaw = []
    for marker in data.markers:
        c_x.append(marker.pose.position.x)
        c_y.append(marker.pose.position.y)
        qx = marker.pose.orientation.x
        qy = marker.pose.orientation.y
        qz = marker.pose.orientation.z
        qw = marker.pose.orientation.w
        c_yaw.append(tf.transformations.euler_from_quaternion(
            (qx, qy, qz, qw))[2])
    return [(i, j, k, None) for i, j, k in zip(c_x, c_y, c_yaw)]


def callback_detected_objects_to_obstacle(data, listener):
    obstacle_x = []
    obstacle_y = []
    obstacle_radius = []
    listener.waitForTransform("/velodyne",
                         "/map", rospy.Time(0), rospy.Duration(5.0))
    for marker in data.markers:
        point_msg = PointStamped()
        point_msg.header.frame_id = '/velodyne'
        point_msg.point.x = marker.pose.position.x
        point_msg.point.y = marker.pose.position.y
        point_msg.point.z = 0
        transformed_coord = listener.transformPoint('/map', point_msg)      
        obstacle_x.append(transformed_coord.point.x)
        obstacle_y.append(transformed_coord.point.y)
        # print "x: ", obstacle_x[-1], " y: ", obstacle_y[-1]
        obstacle_radius.append( np.sqrt(marker.scale.x ** 2 + marker.scale.y ** 2) * 5)
    return [(i, j, k) for i, j, k in zip(obstacle_x, obstacle_y, obstacle_radius)]
