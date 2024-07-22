#!/usr/bin/env python3
import rospy
import rosbag
import pcl
import numpy as np

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import Header


def pointcloud_to_laserscan(bev_point_cloud, frame_id, angle_min=-np.pi, angle_max=np.pi, angle_increment=np.pi/360):
    ranges = [float('inf')] * int((angle_max - angle_min) / angle_increment)

    for point in point_cloud2.read_points(bev_point_cloud, field_names=("x", "y"), skip_nans=True):
        x, y = point[:2]
        angle = np.arctan2(y, x)
        distance = np.sqrt(x**2 + y**2)
        if angle_min <= angle <= angle_max:
            index = int((angle - angle_min) / angle_increment)
            if 0 <= index < len(ranges) and distance < ranges[index]:
                ranges[index] = distance

    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = frame_id
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.range_min = 0.0
    scan.range_max = max(ranges)
    scan.ranges = ranges
    return scan

def cloud_callback(cloud_msg, callback_args):
    bag, bev_publisher, scan_publisher = callback_args
    
    LIDAR_HEIGHT = 0.8
    cloud = pcl.PointCloud()
    points_list = []

    for data in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        points_list.append([data[0], data[1], data[2]])

    cloud.from_list(points_list)

    # Apply RANSAC
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.1)
    inliers, coefficients = seg.segment()

    # Filter ground points
    cloud_non_ground = cloud.extract(inliers, negative=True)

    # Convert to BEV w/o ground points
    bev_points = []
    for point in cloud_non_ground:
        if point[2] > - LIDAR_HEIGHT:
            bev_points.append([point[0], point[1], 0.0]) 

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = cloud_msg.header.frame_id
    # rospy.loginfo(header.frame_id)

    bev_point_cloud = point_cloud2.create_cloud(header, [PointField('x', 0, PointField.FLOAT32, 1),
                                                         PointField('y', 4, PointField.FLOAT32, 1),
                                                         PointField('z', 8, PointField.FLOAT32, 1)], bev_points)

    bev_publisher.publish(bev_point_cloud)
    scan = pointcloud_to_laserscan(bev_point_cloud, header.frame_id)
    scan_publisher.publish(scan)
    bag.write('/bev_pointcloud', bev_point_cloud)


def listener():
    rospy.init_node('velodyne_to_bev_node', anonymous=True)
    bag = rosbag.Bag('bev_output.bag', 'w')
    bev_publisher = rospy.Publisher('/bev_pointcloud', PointCloud2, queue_size=1)
    scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=1)
    rospy.Subscriber('/lidar3D', PointCloud2, cloud_callback, callback_args=(bag, bev_publisher, scan_publisher))
    rospy.spin()
    bag.close()

if __name__ == '__main__':
    listener()
