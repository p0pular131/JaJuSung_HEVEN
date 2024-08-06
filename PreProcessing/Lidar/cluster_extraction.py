#!/usr/bin/env python3
import pcl
import math
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from utils import make_cv2, pcl_to_ros, ros_to_pcl, list_to_multiarray


class LiDARProcessor:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/livox/lidar", PointCloud2, self.cloud_callback)
        self.cluster_publisher = rospy.Publisher("/clustered_cloud", PointCloud2, queue_size=10, latch=True)
        self.filtered_cloud_publisher = rospy.Publisher("/filtered_cloud", PointCloud2, queue_size=1)
        self.lane_angle_publisher = rospy.Publisher("/lane_angle", Float32MultiArray, queue_size=10)
        self.previous_lane_angle = [0, 0]
    
    def cloud_callback(self, cloud_msg):
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
        self.filtered_cloud_publisher.publish(bev_cloud_msg)
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

        # Visualization
        self.left_cluster_dots, self.right_cluster_dots = make_cv2(cluster_indices,
                                                                   bev_cloud_xy0,
                                                                   self.cluster_publisher)
        # cv2.detstroyAllWindows()
        
        # Calculate angles for control
        left_lane_angle = self.get_average_lane_angle("left")        
        right_lane_angle = self.get_average_lane_angle("right")
        lane_angle = [left_lane_angle, right_lane_angle]
        lane_angle_msg = list_to_multiarray(lane_angle)
        self.lane_angle_publisher.publish(lane_angle_msg)


    def is_right_lane_reliable(self):
        return self.right_cluster_dots.shape[0] > 1
    
    def is_left_lane_reliable(self):
        return self.left_cluster_dots.shape[0] > 1
    
    def get_average_lane_angle(self, side):
        """
        : Calculates angle of a lane from the positive x-axis.
          Each angle is calculated from two neighboring points and the angles are averaged lastly.
        Input : side of the lane with respect to the vehicle (str)
        Returns : mean angle from positive x-axis in degress (float) 
        """
        assert (side == "left") or (side == "right")
        if side == "left" and not self.is_left_lane_reliable():
            return self.previous_lane_angle[0]
        elif side == "right" and (not self.is_right_lane_reliable()):
            return self.previous_lane_angle[1]

        angles = []
        cluster_dots = self.left_cluster_dots if side == "left" else self.right_cluster_dots
        sorted_cluster_dots = cluster_dots[np.argsort(cluster_dots[:, 1])]

        # calculate angle from two neighboring points
        for i in range(sorted_cluster_dots.shape[0] - 1):
            dot_1 = sorted_cluster_dots[i]
            dot_2 = sorted_cluster_dots[i+1]
            angle = (-1) * math.atan2((dot_1[1] - dot_2[1]), (dot_1[0] - dot_2[0])) * 180 / np.pi
            angles.append(angle)

        average_lane_angle = sum(angles) / len(angles)
        if side == "left":
            self.previous_lane_angle[0] = average_lane_angle
        else:
            self.previous_lane_angle[1] = average_lane_angle
        
        return average_lane_angle


def do_passthrough(input_cloud, filter_axis, axis_min, axis_max):
    """Apply a PassThrough filter to a point cloud on a specified axis."""
    rospy.loginfo("Applying PassThrough filter on axis %s between %s and %s", filter_axis, axis_min, axis_max)
    passthrough = input_cloud.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()


if __name__  == "__main__":
    # ROS node initialization
    rospy.init_node('cluster_extraction', anonymous=True)
    lidar_processor = LiDARProcessor()
    rospy.spin()