from geometry_msgs.msg import PoseStamped
import tf
import math
import rospy


def get_goal(goal_list, idx):
    goal = PoseStamped()
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "/map"
    goal.pose.position.x = goal_list[idx][0]
    goal.pose.position.y = goal_list[idx][1]
    goal.pose.position.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(
        0, 0, goal_list[idx][2])
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    return goal


def distance_to_goal(x_c, y_c, x_g, y_g):
    return math.sqrt((x_c - x_g)**2 + (y_c - y_g)**2)
