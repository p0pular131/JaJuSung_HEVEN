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


def pcl_to_ros(pcl_array):
    """Converts a PCL PointCloud into a ROS PointCloud2 message."""
    rospy.loginfo("Converting PCL to ROS PointCloud2")
    ros_msg = PointCloud2()
    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "base_link"
    ros_msg.height = 1
    ros_msg.width = pcl_array.size
    ros_msg.is_dense = True
    ros_msg.is_bigendian = False
    ros_msg.fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1),
    ]
    ros_msg.point_step = 16
    ros_msg.row_step = ros_msg.point_step * ros_msg.width
    buffer = []

    for point in pcl_array:
        x, y, z, rgb = point[0], point[1], point[2], int(point[3])
        r, g, b = (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF
        packed_rgb = struct.pack('BBBB', b, g, r, 255)
        buffer.append(struct.pack('fffI', x, y, z, struct.unpack('I', packed_rgb)[0]))

    ros_msg.data = b''.join(buffer)
    rospy.loginfo("PCL to ROS conversion complete")
    return ros_msg


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
    vg.set_leaf_size(0.05, 0.05, 0.05)  # Adjust leaf size to prevent overflow
    cloud_filtered = vg.filter()
    rospy.loginfo("Voxel Grid Downsampling complete")

    # Segmentation using RANSAC
    rospy.loginfo("Performing RANSAC Plane Segmentation")
    segmenter = cloud_filtered.make_segmenter_normals(ksearch=50)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  # Use NORMAL_PLANE for segmentation
    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_max_iterations(100)
    segmenter.set_distance_threshold(0.48)
    indices, coefficients = segmenter.segment()

    if len(indices) == 0:
        rospy.loginfo("No planes found.")
        return

    # Extract inliers and outliers
    inliers = cloud_filtered.extract(indices, negative=False)
    outliers = cloud_filtered.extract(indices, negative=True)

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
    ec.set_MaxClusterSize(150)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    rospy.loginfo("Cluster extraction complete")
    rospy.loginfo("Visualizing clusters")

    # Publish cylinders for each cluster
    publish_cylinders(cluster_indices, cloud_xyz, pub_marker)



def publish_cylinders(cluster_indices, cloud_filtered, pub_marker):
    marker_array = MarkerArray()
    for j, indices in enumerate(cluster_indices):
        points = [cloud_filtered[i] for i in indices]
        x_coords, y_coords, z_coords = zip(*[(point[0], point[1], point[2]) for point in points])

        centroid_x = statistics.median(x_coords)
        centroid_y = statistics.median(y_coords)
        centroid_z = statistics.median(z_coords)
        height = 0.1
        radius = 0.4
        marker = Marker()
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.header.frame_id = "base_link"
        marker.id = j
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.position.x = centroid_x
        marker.pose.position.y = centroid_y
        marker.pose.position.z = 0
        marker.scale.x = marker.scale.y = 2 * radius  # Ensure visible size
        marker.scale.z = height
        marker.color.a = 1.0  # Ensure visibility
        marker.color.r = random.random()
        marker.color.g = random.random()
        marker.color.b = random.random()
        # marker.lifetime = rospy.Duration()  # Make them persistent for troubleshooting
        marker.lifetime = rospy.Time(0.7)

        marker_array.markers.append(marker)

    rospy.loginfo("Publishing {} markers".format(len(marker_array.markers)))
    pub_marker.publish(marker_array)

def callback_cluster_processing(clustered_cloud_msg):
    rospy.loginfo("Received clustered PointCloud2 message")
    try:
        # Convert the ROS PointCloud2 message to PCL data
        pcl_cloud = ros_to_pcl(clustered_cloud_msg)

        # Assuming you have a function to extract clusters from pcl_cloud
        clusters = extract_clusters(pcl_cloud)
        if not clusters:
            rospy.loginfo("No clusters found in the received data.")
            return

        # Publish cylinders based on the clusters
        publish_cylinders(clusters, pcl_cloud, pub_marker)
        rospy.loginfo("Cylinder markers published for clusters.")

    except Exception as e:
        rospy.logerr("Error processing clustered cloud: %s", str(e))


# ROS node initialization
rospy.init_node('cluster_extraction')


pub = rospy.Publisher("/clustered_cloud", PointCloud2, queue_size=1)
pub_marker = rospy.Publisher("/cluster_markers", MarkerArray, queue_size=1)
pub_map = rospy.Publisher("/updated_2d_map", OccupancyGrid, queue_size=1)

rospy.Subscriber("/lidar3D", PointCloud2, cloud_callback)



rospy.spin()


