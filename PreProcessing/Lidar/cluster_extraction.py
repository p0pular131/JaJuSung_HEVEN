#!/usr/bin/env python3
import cv2
import pcl
import rospy
import sensor_msgs.point_cloud2 as pc2
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
    filtered_cloud = do_passthrough(cloud, 'z', -0.5, 0.0)
    filtered_cloud = do_passthrough(filtered_cloud, 'x', -0.0, 7.5)  # front

    # Flatten pointcloud 3D to 2D
    bev_points_list = []
    filtered_cloud_msg = pcl_to_ros(filtered_cloud.to_list())
    for data in pc2.read_points(filtered_cloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
        bev_points_list.append([data[0], data[1], 0]) # data[3]

    bev_cloud_msg = pcl_to_ros(bev_points_list, None)
    pub_filtered.publish(bev_cloud_msg)
    rospy.loginfo("ROI-filtered BEV points published")

    # Voxel Grid Downsampling
    bev_cloud_xy0 = pcl.PointCloud()
    bev_cloud_xy0.from_list(bev_points_list)

    rospy.loginfo("Applying Voxel Grid Downsampling")
    vg = bev_cloud_xy0.make_voxel_grid_filter()
    vg.set_leaf_size(0.01, 0.01, 0.01)  # Adjust leaf size to prevent overflow
    bev_cloud_xy0 = vg.filter()
    rospy.loginfo("Voxel Grid Downsampling complete")

    # Extract clusters from the outliers
    tree = bev_cloud_xy0.make_kdtree()
    rospy.loginfo("Extracting clusters")
    ec = bev_cloud_xy0.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.8)
    ec.set_MinClusterSize(2)
    ec.set_MaxClusterSize(70)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    rospy.loginfo("Cluster extraction complete")
    rospy.loginfo("Number of clusters found: {}".format(len(cluster_indices)))
    make_cv2(cluster_indices, bev_cloud_xy0, pub)

cv2.destroyAllWindows()

# ROS node initialization
rospy.init_node('cluster_extraction', anonymous=True)
pub = rospy.Publisher("/clustered_cloud", PointCloud2, queue_size=10, latch=True)
pub_filtered = rospy.Publisher("/filtered_cloud", PointCloud2, queue_size=1)
rospy.Subscriber("/livox/lidar", PointCloud2, cloud_callback)
rospy.spin()
