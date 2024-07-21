#!/usr/bin/env python3
from geometry_msgs.msg import Quaternion

import time
import numpy as np
import pcl
import random
import rospy
from nav_msgs.msg import OccupancyGrid
import struct
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import statistics

def pcl_to_ros(pcl_array, color):
    """Converts a PCL PointCloud into a ROS PointCloud2 message."""
    rospy.loginfo("Converting PCL to ROS PointCloud2")
    ros_msg = PointCloud2()
    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "base_link"
    ros_msg.height = 1
    ros_msg.width = len(pcl_array)
    ros_msg.is_dense = True
    ros_msg.is_bigendian = False
    ros_msg.fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1), pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1), pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1), pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1),
    ]

    ros_msg.point_step = 16

    ros_msg.row_step = ros_msg.point_step * ros_msg.width
    buffer = []
    r, g, b = color
    rgb = struct.pack('BBBB', b, g, r, 255)
    rgb = struct.unpack('I', rgb)[0]
    for point in pcl_array:
        buffer.append(struct.pack('fffI', point[0], point[1], point[2], rgb))

    ros_msg.data = b''.join(buffer)
    rospy.loginfo("PCL to ROS conversion complete")
    return ros_msg

def generate_color(index):
    """ Generate a unique RGB color based on the index. """
    import random
    random.seed(index)
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))


def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a PCL PointCloud """
    rospy.loginfo("Converting ROS PointCloud2 to PCL")
    points_list = []
    for data in pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z")):
        points_list.append([data[0], data[1], data[2], 0xFF0000])  # Default red color for visualization

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)
    rospy.loginfo("ROS to PCL conversion complete")
    return pcl_data

def cloud_callback(cloud_msg):
    rospy.loginfo("Received PointCloud2 message")
    # Convert ROS PointCloud2 message to PCL data
    cloud = ros_to_pcl(cloud_msg)

    # Voxel Grid Downsampling
    rospy.loginfo("Applying Voxel Grid Downsampling")
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(0.5, 0.5, 0.5)  # Adjust leaf size to prevent overflow
    cloud_filtered = vg.filter()
    rospy.loginfo("Voxel Grid Downsampling complete") # Segmentation using RANSAC
    rospy.loginfo("Performing RANSAC Plane Segmentation")
    segmenter = cloud_filtered.make_segmenter_normals(ksearch=50)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  # Use NORMAL_PLANE for segmentation
    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC)inliers
    segmenter.set_max_iterations(100)
    segmenter.set_distance_threshold(0.6)
    indices, coefficients = segmenter.segment()

    if len(indices) == 0:
        rospy.loginfo("No planes found.")
        return

    # Extract inliers and outliers
    inliers = cloud_filtered.extract(indices, negative=False)
    outliers = cloud_filtered.extract(indices, negative=True)
    # ransac_result = pcl_to_ros(inliers.to_list(), (255, 0, 0))  # Visualize outliers as red
    ransac_result = pcl_to_ros(outliers.to_list(), (255, 0, 0))  # Visualize outliers as red
    pub_ransac.publish(ransac_result)

    # Convert outliers to a simple XYZ point cloud for KD-tree operations
    rospy.loginfo("Converting outliers to XYZ format for KD-tree")
    cloud_xyz = pcl.PointCloud()
    cloud_xyz.from_list([(point[0], point[1], point[2]) for point in outliers])
    tree = cloud_xyz.make_kdtree()

    # Extract clusters from the outliers
    rospy.loginfo("Extracting clusters")
    ec = cloud_xyz.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.75)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(1000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    rospy.loginfo("Cluster extraction complete")
    rospy.loginfo("Number of clusters found: {}".format(len(cluster_indices)))

    for idx, indices in enumerate(cluster_indices):
        cluster_points = [outliers[i] for i in indices]  # Ensure indices match the dataset
        color = generate_color(idx)
        cluster_msg = pcl_to_ros(cluster_points,color)
        pub.publish(cluster_msg)  # Publish each cluster
        rospy.loginfo("Published cluster {} with {} points".format(idx + 1, len(cluster_points)))
    rospy.loginfo("Visualizing clusters")

# ROS node initialization
rospy.init_node('cluster_extraction')
pub = rospy.Publisher("/clustered_cloud", PointCloud2, queue_size=1)
pub_ransac = rospy.Publisher("/ransac_result", PointCloud2, queue_size=1)
rospy.Subscriber("/livox/lidar", PointCloud2, cloud_callback)
rospy.spin()
