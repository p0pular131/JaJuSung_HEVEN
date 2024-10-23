#!/usr/bin/env python3

import rospy
import lidar_pcl_helper as pcl_helper
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_geometry import LaserProjection
import math

def do_voxel_grid_downsampling(pcl_data, leaf_size):
    # Voxel Grid Filter를 사용한 다운샘플링
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)  # 리프 크기가 클수록 정보가 적게 유지됨
    return vox.filter()

def do_passthrough(pcl_data, filter_axis, axis_min, axis_max):
    # 축 기반 필터링
    passthrough = pcl_data.make_passthrough_filter() #pc데이터를 축 기반 필터링을 위한 객체로 변환
    passthrough.set_filter_field_name(filter_axis) #필터링할 축 지정
    passthrough.set_filter_limits(axis_min, axis_max) #해당 축에서 필터링 범위 설정
    return passthrough.filter()

class Point_to_scan():
    def __init__(self):
        rospy.init_node("Fusion_node", anonymous=False)
        # LiDAR 데이터 구독
        self.lidar_sub = rospy.Subscriber("/livox/lidar", PointCloud2, self.lidar_callback)  #pc메시지 구독, 수신되면 lidar_callback함수 실행
        self.roi_pub = rospy.Publisher('/roi_pointcloud', PointCloud2, queue_size=10)
        self.laserscan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.laser_projector = LaserProjection() #pc를 2d레이저 스캔으로 변환함....

    def lidar_callback(self, data):
        # PointCloud2 메시지를 PCL 포맷으로 변환
        pcl_cloud = pcl_helper.ros_to_pcl(data)
        # ROI 설정 및 필터링
        filter_axis = 'y'
        axis_min = -3.0
        axis_max = 3.0
        cloud = do_passthrough(pcl_cloud, filter_axis, axis_min, axis_max)
        filter_axis = 'x'
        axis_min = 0.0
        axis_max = 8.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
        filter_axis = 'z'
        axis_min = -0.7
        axis_max = 0.5
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
        # 필터링된 포인트 클라우드를 PointCloud2 메시지로 변환
        roi_cloud = pcl_helper.pcl_to_ros(cloud)
        roi_cloud.header.frame_id = "livox_frame"
        self.roi_pub.publish(roi_cloud)
        laserscan_msg = self.pointcloud_to_laserscan(roi_cloud)
        if laserscan_msg is not None:
            self.laserscan_pub.publish(laserscan_msg)

    def pointcloud_to_laserscan(self, cloud_msg):
        """
        PointCloud2 데이터를 LaserScan으로 변환하는 함수
        """
        scan_msg = LaserScan()
        # LaserScan 기본 정보 설정
        scan_msg.header = cloud_msg.header
        scan_msg.header.frame_id = "livox_frame"
        scan_msg.angle_min = -math.pi / 2  # 시작 각도
        scan_msg.angle_max = math.pi / 2   # 종료 각도
        scan_msg.angle_increment = math.radians(0.3)  # 각도 증가량
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.2  # 최소 거리 (m)
        scan_msg.range_max = 15.0  # 최대 거리 (m)
        # ranges 리스트 생성 (초기화)- 각도 범위에 따른 스캔 포인트 개수 계산
        num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
        scan_msg.ranges = [float('inf')] * num_ranges
        # LaserScan ranges 리스트에 값 채우기-y와 z축을 기준으로 2d평면에서의 거리 계산
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[:3]
            range_val = math.sqrt(x ** 2 + y ** 2)  # 2D 거리 계산
            # 스캔 범위 내에 있는 점들만 추가
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                angle = math.atan2(y, x)  # 각도 계산
                if scan_msg.angle_min <= angle <= scan_msg.angle_max:
                    # 각도를 인덱스로 변환
                    idx = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
                    if 0 <= idx < num_ranges:
                        # 기존에 기록된 값보다 더 가까운 거리를 기록
                        if range_val < scan_msg.ranges[idx]:
                            scan_msg.ranges[idx] = range_val
        return scan_msg
    
if __name__ == "__main__":
    if not rospy.is_shutdown():
        Point_to_scan()
        rospy.spin()