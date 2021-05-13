from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from plc_helper import *
import tf
from gem_perception.msg import DetectedObjectsArray, DetectedObject
from visualization_msgs.msg import MarkerArray, Marker


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

    # self.course = [(i, j, k, None) for i, j, k in zip(c_x, c_y, c_yaw)]


def callback_pcl(data):

    # Convert a ROS PointCloud2 message to a pcl PointXYZRGB
    cloud_filtered = ros_to_pcl(data)

    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_filtered.make_voxel_grid_filter()
    # Choose a voxel (also known as leaf) size
    # 1 means 1mx1mx1m leaf size
    # Experiment and find the appropriate size!
    LEAF_SIZE = .1
    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # RANSAC plane segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    max_distance = .1
    seg.set_distance_threshold(max_distance)
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    # Extract outliers 
    extracted_objects = cloud_filtered.extract(inliers, negative=True)

    # print "type(extracted_objects)", type(extracted_objects) < type 'pcl._pcl.PointCloud_PointXYZRGB' >


    return extracted_objects


def pcl_clustering(extracted_objects):

    #----------------------------------------------------------------------------------
    # Euclidean Clustering
    #----------------------------------------------------------------------------------
    white_cloud = XYZRGB_to_XYZ(extracted_objects)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(5)
    ec.set_MinClusterSize(5)
    ec.set_MaxClusterSize(100000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # print "# of cluster", len(cluster_indices)

    #----------------------------------------------------------------------------------
    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    #----------------------------------------------------------------------------------
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # print "cluster_cloud: ", cluster_cloud  # <PointCloud of 204 points>
    #----------------------------------------------------------------------------------
    # Converts a pcl PointXYZRGB to a ROS PointCloud2 message
    #----------------------------------------------------------------------------------
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    return cluster_indices, ros_cluster_cloud


def pcl_to_3d_bbox(cluster_indices, extracted_objects):
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):

        #----------------------------------------------------------------------------------
        # Grab the points for the cluster from the extracted_objects
        #----------------------------------------------------------------------------------
        pcl_cluster = extracted_objects.extract(pts_list)
        # Convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        #----------------------------------------------------------------------------------
        # Add the detected object to the list of detected objects.
        #----------------------------------------------------------------------------------
        do = DetectedObject()
        do.label = "None"
        do.cloud = ros_cluster
        detected_objects.append(do)

    return detected_objects

def callback_detected_objects_3d(data):
    detection_bbox_3d = MarkerArray()
    for index, cluster in enumerate(data):
        x_l = []
        y_l = []
        z_l = []
        for idx in pc2.read_points(cluster.cloud, skip_nans=True):
            x_l.append(idx[0])
            y_l.append(idx[1])
            z_l.append(idx[2])
        x_center = sum(x_l) / len(x_l)
        y_center = sum(y_l) / len(y_l)
        z_center = sum(z_l) / len(z_l)
        x_scale = abs(max(x_l) - min(x_l))
        y_scale = abs(max(y_l) - min(y_l))
        z_scale = abs(max(z_l) - min(z_l))
        detection_marker = detection_to_marker(x_center, y_center, z_center,
                            x_scale, y_scale, z_scale, 0, "velodyne")
        detection_marker.id = index
        detection_bbox_3d.markers.append(detection_marker)
    return detection_bbox_3d


def callback_detected_objects_2d(data):
    
    pass

def detection_to_marker(x, y, z, x_scale, y_scale, z_scale, theta, frame):
    marker = Marker()
    marker.header.frame_id = frame  # "/odom"
    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    marker.scale.x = x_scale
    marker.scale.y = y_scale
    marker.scale.z = z_scale
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1
    marker.type = Marker.CUBE
    marker.action = marker.ADD
    # marker.Duration
    return marker


def detection_using_pointcloud(extracted_objects):
    
    cluster_indices, ros_cluster_cloud = pcl_clustering(extracted_objects)
    detected_objects = pcl_to_3d_bbox(cluster_indices, extracted_objects)
    return ros_cluster_cloud, detected_objects
