#!/usr/bin/env python3
import cv2 import numpy as np
import pcl
import rospy
from sensor_msgs.msg import PointCloud2 
from utils import make_cv2, pcl_to_ros, ros_to_pcl

def do_passthrough(input_cloud, filter_axis, axis_min, axis_max):
    """Apply a PassThrough filter to a point cloud on a specified axis."""
    rospy.loginfo("Applying PassThrough filter on axis %s between %s and %s", filter_axis, axis_min, axis_max)
    passthrough = input_cloud.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

def cloud_callback(cloud_msg):
    rospy.loginfo("Received PointCloud2 message")
    # Convert ROS PointCloud2 message to PCL data
    cloud = ros_to_pcl(cloud_msg)
    filtered_cloud = do_passthrough(cloud, 'z', -0.2, 5.0)
    #front
    filtered_cloud = do_passthrough(filtered_cloud, 'x', -3.5, 7.5)
    ros_filtered_cloud = pcl_to_ros(filtered_cloud.to_list(), None)
    pub_filtered.publish(ros_filtered_cloud)
    rospy.loginfo("Filtered cloud published")

    # Voxel Grid Downsampling
    rospy.loginfo("Applying Voxel Grid Downsampling") 
    vg = filtered_cloud.make_voxel_grid_filter()
    vg.set_leaf_size(0.005, 0.005, 0.005)  # Adjust leaf size to prevent overflow
    cloud_filtered = vg.filter()
    rospy.loginfo("Voxel Grid Downsampling complete")

    cloud_xyz = pcl.PointCloud()
    cloud_xyz.from_list([(point[0], point[1], point[2]) for point in cloud_filtered])
    tree = cloud_xyz.make_kdtree()

    # Extract clusters from the outliers
    rospy.loginfo("Extracting clusters")
    ec = cloud_xyz.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.87)
    ec.set_MinClusterSize(2)
    ec.set_MaxClusterSize(70)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    rospy.loginfo("Cluster extraction complete")
    rospy.loginfo("Number of clusters found: {}".format(len(cluster_indices)))
    make_cv2(cluster_indices, cloud_filtered,pub)

cv2.destroyAllWindows()

# ROS node initialization
rospy.init_node('cluster_extraction')
pub = rospy.Publisher("/clustered_cloud", PointCloud2, queue_size=10,latch=True)
pub_filtered = rospy.Publisher("/filtered_cloud", PointCloud2, queue_size=1)
rospy.Subscriber("/livox/lidar", PointCloud2, cloud_callback)
rospy.spin()
