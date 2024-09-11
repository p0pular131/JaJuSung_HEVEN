#!/usr/bin/env python3
import cv2
import math
import threading
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from time import time
from sensor_msgs.msg import PointCloud2
from jajusung_main.msg import lane_info
from std_msgs.msg import Float32MultiArray
from utils import make_cv2, pcl_to_ros, ros_to_pcl, list_to_multiarray, clustering, draw_polyfit_lane
from message_filters import Subscriber, ApproximateTimeSynchronizer


class LiDARProcessor:
    def __init__(self):
        rospy.init_node("cluster_extraction", anonymous=True)
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
        self.lane_info_publisher = rospy.Publisher(
            "lane_result", lane_info, queue_size=10
        )
        self.previous_lane_angle = [90, 90]
        self.previous_lane_offset = [350, 450]
        self.previous_timestamp = time()

        # bev frame
        self.frame = None
        self.frame_index = 0
        self.frame_size = (600, 800)  # (height, width)

        # multi threading
        self.lock = threading.Lock()
        self.display_thread = threading.Thread(target=self.display_bev_frame)
        self.display_thread.start()
        rospy.on_shutdown(self.clean_up)

    def clean_up(self):
        rospy.loginfo("Shutting down and closing OpenCV windows")
        cv2.destroyAllWindows()

    def check_fps(self, frame_interval=100):
        self.frame_index += 1
        if self.frame_index % frame_interval == 0:
            timestamp = time()
            fps = round(1 / (timestamp - self.previous_timestamp + 1e-6) * frame_interval, 1)
            rospy.loginfo(f"FPS = {fps} frames / sec")
            self.previous_timestamp = timestamp
            self.frame_index = 0

    def cloud_callback_yellow(self, cloud_msg):
        rospy.loginfo("Received PointCloud2 message from yellow cloud")
        self.process_cloud(cloud_msg, "yellow")

    def cloud_callback_blue(self, cloud_msg):
        rospy.loginfo("Received PointCloud2 message from blue cloud")
        self.process_cloud(cloud_msg, "blue")

    def sync_callback(self, cloud_yellow, cloud_blue):
        # rospy.loginfo("Received PointCloud2 message")
        with self.lock:
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
                # rospy.loginfo("ROI-filtered BEV points published")

            # Cluster pointclouds
            yellow_clouds, yellow_cluster_indices = clustering(yellow_bev_points_list)
            blue_clouds, blue_cluster_indices = clustering(blue_bev_points_list)
            clouds = [
                [yellow_clouds, yellow_cluster_indices, "yellow"],
                [blue_clouds, blue_cluster_indices, "blue"],
            ]

            # Visualization
            self.frame, self.left_cluster_dots, self.right_cluster_dots = make_cv2(clouds, self.frame_size)

            # Calculate angles and offsets for control
            left_lane_angle = self.get_average_lane_angle("left")
            right_lane_angle = self.get_average_lane_angle("right")
            left_lane_offset = self.get_average_lane_offset("left")
            right_lane_offset = self.get_average_lane_offset("right")

            # Exception handling for single side lane detection
            if self.is_left_lane_reliable() ^ self.is_right_lane_reliable():
                # rospy.loginfo("Sigle lane detected")
                if self.is_right_lane_reliable():
                    left_lane_offset = 200
                    left_lane_angle = right_lane_angle
                    self.previous_lane_angle[0] = left_lane_angle
                if self.is_left_lane_reliable():
                    right_lane_offset = 200
                    right_lane_angle = left_lane_angle
                    self.previous_lane_angle[1] = right_lane_angle

            # Publish
            lane_angle = [left_lane_angle, right_lane_angle]
            lane_angle_msg = list_to_multiarray(lane_angle)
            self.lane_angle_publisher.publish(lane_angle_msg)

            lane_offset = [left_lane_offset, right_lane_offset]
            lane_offset_msg = list_to_multiarray(lane_offset)
            self.lane_offset_publisher.publish(lane_offset_msg)

            curr_info = lane_info()
            curr_info.left_x = int(left_lane_offset)
            curr_info.right_x = int(right_lane_offset)
            curr_info.left_theta = left_lane_angle
            curr_info.right_theta = right_lane_angle

            self.lane_info_publisher.publish(curr_info)
            self.check_fps()

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
        if side == "left" and (not self.is_left_lane_reliable()):
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
    
    def get_average_lane_offset(self, side, center=400):
        """
        Calculates offset of a lane from the center of image coordinate.
          - Input : side of the lane with respect to the vehicle (str)
          - Returns : absolute value of offset from center of image in pixels (float)
        """
        assert (side == "left") or (side == "right")
        if side == "left" and (not self.is_left_lane_reliable()):
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
            offset = abs(center-cluster_dot[0])
            offset_sum += offset

        average_lane_offset = offset_sum / sorted_cluster_dots.shape[0]
        if side == "left":
            self.previous_lane_offset[0] = average_lane_offset
        else:
            self.previous_lane_offset[1] = average_lane_offset

        return average_lane_offset
    
    def display_bev_frame(self):
        height, width = self.frame_size

        while not rospy.is_shutdown():
            with self.lock:
                if self.frame is not None:
                    if self.is_left_lane_reliable():
                        draw_polyfit_lane(self.frame, self.left_cluster_dots)

                    if self.is_right_lane_reliable():
                        draw_polyfit_lane(self.frame, self.right_cluster_dots)

                    for i in range(0, width, 80):
                        cv2.line(self.frame, (i, 0), (i, height), (0, 0, 0), 1)
                    for j in range(0, height, 80):
                        cv2.line(self.frame, (0, j), (width, j), (0, 0, 0), 1)

                    cv2.imshow("BEV frame", self.frame)
                    cv2.waitKey(1)


if __name__ == "__main__":
    LiDARProcessor()
    rospy.spin()
