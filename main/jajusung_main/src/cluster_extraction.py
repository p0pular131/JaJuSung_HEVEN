#!/usr/bin/env python3
import pcl
import math
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from utils import make_cv2, pcl_to_ros, ros_to_pcl, list_to_multiarray, clustering
from message_filters import Subscriber, ApproximateTimeSynchronizer


class LiDARProcessor:
    def __init__(self):
        self.subscriber_yellow = Subscriber("cloud_yellow", PointCloud2)
        self.subscriber_blue = Subscriber("cloud_blue", PointCloud2)
        self.ts = ApproximateTimeSynchronizer(
            [self.subscriber_yellow, self.subscriber_blue], queue_size=10, slop=1
        )
        self.ts.registerCallback(self.sync_callback)

        self.cluster_publisher = rospy.Publisher(
            "/clustered_cloud", PointCloud2, queue_size=10, latch=True
        )
        self.filtered_cloud_publisher = rospy.Publisher(
            "/filtered_cloud", PointCloud2, queue_size=10
        )
        self.lane_angle_publisher = rospy.Publisher(
            "/lane_angle", Float32MultiArray, queue_size=10
        )
        self.lane_offset_publisher = rospy.Publisher(
            "/lane_offset", Float32MultiArray, queue_size=10
        )

        self.previous_lane_angle = [90, 90]
        self.previous_lane_offset = [350, 450]

    def cloud_callback_yellow(self, cloud_msg):
        rospy.loginfo("Received PointCloud2 message from yellow cloud")
        self.process_cloud(cloud_msg, "yellow")

    def cloud_callback_blue(self, cloud_msg):
        rospy.loginfo("Received PointCloud2 message from blue cloud")
        self.process_cloud(cloud_msg, "blue")

    def sync_callback(self, cloud_yellow, cloud_blue):
        rospy.loginfo("Received PointCloud2 message")
        yellow_bev_points_list = []
        blue_bev_points_list = []

        # Convert ROS PointCloud2 message to PCL data
        for cloud_msg, color in [(cloud_yellow, "yellow"), (cloud_blue, "blue")]:
            cloud = ros_to_pcl(cloud_msg)

            bev_points_list = []
            for data in pc2.read_points(
                pcl_to_ros(cloud.to_list()),
                skip_nans=True,
                field_names=("x", "y", "z", "rgb"),
            ):
                if color == "yellow":
                    yellow_bev_points_list.append([data[0], data[1], 0])

                else:
                    blue_bev_points_list.append([data[0], data[1], 0])

            bev_cloud_msg = pcl_to_ros(bev_points_list, None)
            self.filtered_cloud_publisher.publish(bev_cloud_msg)
            rospy.loginfo("ROI-filtered BEV points published")
            yellow_clouds, yellow_cluster_indices = clustering(yellow_bev_points_list)
            blue_clouds, blue_cluster_indices = clustering(blue_bev_points_list)
            clouds = [
                [yellow_clouds, yellow_cluster_indices, "yellow"],
                [blue_clouds, blue_cluster_indices, "blue"],
            ]

            # Visualization
            self.left_cluster_dots, self.right_cluster_dots = make_cv2(clouds)
            # cv2.detstroyAllWindows()

            # Calculate angles and offsets for control
            left_lane_angle = self.get_average_lane_angle("left")
            right_lane_angle = self.get_average_lane_angle("right")
            lane_angle = [left_lane_angle, right_lane_angle]
            lane_angle_msg = list_to_multiarray(lane_angle)
            self.lane_angle_publisher.publish(lane_angle_msg)

            left_lane_offset = self.get_average_lane_offset("left")
            right_lane_offset = self.get_average_lane_offset("right")
            lane_offset = [left_lane_offset, right_lane_offset]
            lane_offset_msg = list_to_multiarray(lane_offset)
            self.lane_offset_publisher.publish(lane_offset_msg)

    def is_right_lane_reliable(self):
        return self.right_cluster_dots.shape[0] > 1

    def is_left_lane_reliable(self):
        return self.left_cluster_dots.shape[0] > 1

    def get_average_lane_angle(self, side):
        """
        Calculates angle of a lane from the positive x-axis.
        Each angle is calculated from two neighboring points and the angles are averaged lastly.
          - Input : side of the lane with respect to the vehicle (str)
          - Returns : mean angle from positive x-axis in degress (float)
        """
        assert (side == "left") or (side == "right")
        if side == "left" and not self.is_left_lane_reliable():
            return self.previous_lane_angle[0]
        elif side == "right" and (not self.is_right_lane_reliable()):
            return self.previous_lane_angle[1]

        angles = []
        cluster_dots = (
            self.left_cluster_dots if side == "left" else self.right_cluster_dots
        )
        sorted_cluster_dots = cluster_dots[np.argsort(cluster_dots[:, 1])]

        # calculate angle from two neighboring points
        for i in range(sorted_cluster_dots.shape[0] - 1):
            dot_1 = sorted_cluster_dots[i]
            dot_2 = sorted_cluster_dots[i + 1]
            angle = (
                (-1)
                * math.atan2((dot_1[1] - dot_2[1]), (dot_1[0] - dot_2[0]))
                * 180
                / np.pi
            )
            angles.append(angle)

        average_lane_angle = sum(angles) / len(angles)
        if side == "left":
            self.previous_lane_angle[0] = average_lane_angle
        else:
            self.previous_lane_angle[1] = average_lane_angle

        return average_lane_angle
    
    def get_average_lane_offset(self, side):
        """
        Calculates offset of a lane from the left side of image coordinate.
          - Input : side of the lane with respect to the vehicle (str)
          - Returns : offset from left side of image in pixels (float)
        """
        assert (side == "left") or (side == "right")
        if side == "left" and not self.is_left_lane_reliable():
            return self.previous_lane_offset[0]
        elif side == "right" and (not self.is_right_lane_reliable()):
            return self.previous_lane_offset[1]

        cluster_dots = (
            self.left_cluster_dots if side == "left" else self.right_cluster_dots
        )
        sorted_cluster_dots = cluster_dots[np.argsort(cluster_dots[:, 1])]

        # calculate offset
        offset_sum = 0
        for cluster_dot in sorted_cluster_dots:
            offset_sum += cluster_dot[0]

        average_lane_offset = offset_sum / sorted_cluster_dots.shape[0]
        if side == "left":
            self.previous_lane_offset[0] = average_lane_offset
        else:
            self.previous_lane_offset[1] = average_lane_offset

        return average_lane_offset


if __name__ == "__main__":
    # ROS node initialization
    rospy.init_node("cluster_extraction", anonymous=True)
    lidar_processor = LiDARProcessor()
    rospy.spin()
