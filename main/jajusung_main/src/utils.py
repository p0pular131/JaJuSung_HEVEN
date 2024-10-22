import statistics
import cv2
import rospy
import pcl
import struct
import random
import numpy as np
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import PointCloud2


def split_centroids_by_color(centroids):
    left_centroids = []
    right_centroids = []

    for centroid, color in centroids:
        centroid_list = list(centroid)
        if color == "yellow":
            left_centroids.append(centroid_list)
        elif color == "blue":
            right_centroids.append(centroid_list)
        else:
            raise NotImplementedError()

    return (np.array(left_centroids), np.array(right_centroids))


def ros_to_pcl(ros_cloud):
    """Converts a ROS PointCloud2 message to a PCL PointCloud"""
    points_list = []
    for data in pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z")):
        points_list.append(
            [data[0], data[1], data[2], 0xFF0000]
        )  # Default red color for visualization

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)
    return pcl_data

def ros_to_list(ros_cloud):
    """Converts a ROS PointCloud2 message to a list"""
    points_list = []
    for data in pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z")):
        points_list.append([data[0], data[1], data[2]])

    return points_list

def pcl_to_ros(pcl_array, color=None):
    """Converts a PCL PointCloud into a ROS PointCloud2 message. Uses a default color if none is provided."""
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"
    ros_msg = PointCloud2()
    ros_msg.header = header
    ros_msg.height = 1
    ros_msg.width = len(pcl_array)
    ros_msg.is_dense = True
    ros_msg.is_bigendian = False
    ros_msg.fields = [
        pc2.PointField("x", 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField("y", 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField("z", 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField("rgb", 12, pc2.PointField.UINT32, 1),
    ]
    ros_msg.point_step = 16
    ros_msg.row_step = ros_msg.point_step * ros_msg.width

    buffer = []
    default_color = (255, 255, 255)  # White color
    rgb = color if color is not None else default_color
    packed_rgb = struct.pack("BBBB", rgb[2], rgb[1], rgb[0], 255)
    rgb_int = struct.unpack("I", packed_rgb)[0]

    for point in pcl_array:
        buffer.append(struct.pack("fffI", point[0], point[1], point[2], rgb_int))

    ros_msg.data = b"".join(buffer)
    return ros_msg


def draw_lane(frame, clustered_dots):
    clustered_dots = clustered_dots[np.argsort(clustered_dots[:, 1])]
    x_coords = clustered_dots[:, 0]
    y_coords = clustered_dots[:, 1]
    previous_point = None
    for point in zip(x_coords, y_coords):
        if previous_point is not None:
            cv2.line(frame, previous_point, point, (255, 0, 0), 2)
        previous_point = point


def list_to_multiarray(list_value):
    msg = Float32MultiArray()
    msg.data = list_value
    return msg


def clustering(bev_points_list):
    bev_cloud_xy0 = pcl.PointCloud()
    bev_cloud_xy0.from_list(bev_points_list)

    # rospy.loginfo("Applying Voxel Grid Downsampling")
    vg = bev_cloud_xy0.make_voxel_grid_filter()
    vg.set_leaf_size(0.01, 0.01, 0.01)  # Adjust leaf size to prevent overflow
    bev_cloud_xy0 = vg.filter()
    # rospy.loginfo("Voxel Grid Downsampling complete")

    # Extract clusters from the outliers
    tree = bev_cloud_xy0.make_kdtree()
    # rospy.loginfo("Extracting clusters")
    ec = bev_cloud_xy0.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.8)
    ec.set_MinClusterSize(2)
    ec.set_MaxClusterSize(70)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    # rospy.loginfo("Cluster extraction complete")
    rospy.loginfo("Number of clusters found: {}".format(len(cluster_indices)))
    # rospy.loginfo("Number of BEV points: {}".format(str(len(bev_points_list))))
    return bev_cloud_xy0, cluster_indices


def make_cv2(clouds, bev_frame_size):
    HEIGHT, WIDTH = bev_frame_size
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype="uint8") + 255
    centroids_image_coord = []

    for cloud in clouds:
        cloud_filtered, cluster_indices, color = cloud
        for idx, indices in enumerate(cluster_indices):
            points = [cloud_filtered[i] for i in indices]
            x_coords, y_coords, z_coords = zip(
                *[(point[0], point[1], point[2]) for point in points]
            )
            color_rgb = (0, 255, 255) if color == "yellow" else (255, 0, 0)

            # print(statistics.median(x_coords), (statistics.median(y_coords)
            center_x, center_y = WIDTH // 2, HEIGHT // 2

            # rospy.loginfo(
            #     f"x_coords:{statistics.median(x_coords)}, y_coords:{statistics.median(y_coords)}"
            # )
            centroid_y = center_y + (-statistics.median(x_coords) * 60) + 400
            centroid_x = center_x + (-statistics.median(y_coords) * 50)
            centroid_image_coord = (int(centroid_x), int(centroid_y))
            cv2.circle(frame, centroid_image_coord, 10, color_rgb, -1)
            centroids_image_coord.append((centroid_image_coord, color))

    left_lane_centroids, right_lane_centroids = split_centroids_by_color(centroids_image_coord)
    return frame, left_lane_centroids, right_lane_centroids

