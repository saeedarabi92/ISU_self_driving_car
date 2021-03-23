from visualization_msgs.msg import MarkerArray, Marker
import rospy
import tf


def pose_to_marker(x, y, frame, theta = 0):
    marker = Marker()
    marker.header.frame_id = frame  # "/odom"
    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1
    marker.type = Marker.CUBE
    marker.action = marker.ADD
    return marker


def path_to_marker(path):
    Path = MarkerArray()
    pose_id = 0
    idx = 0
    while True:
        x = path[idx][0]
        y = path[idx][1]
        yaw = path[idx][2]
        pose_marker = pose_to_marker(x, y, theta = yaw, frame = "/map")
        pose_marker.id = pose_id
        Path.markers.append(pose_marker)
        pose_id += 1
        idx += 1
        if idx == len(path):
            break
    return Path
