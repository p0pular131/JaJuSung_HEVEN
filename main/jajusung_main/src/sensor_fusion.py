#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pcl  # Python PCL 라이브러리
import time
import torch
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
import std_msgs.msg
import lidar_pcl_helper as pcl_helper
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from math import radians, cos, sin

# 카메라의 내부 파라미터 행렬
intrinsic_matrix = np.array([[611.768234, 0.000000, 306.164069],
                             [0.000000, 613.154786, 233.896019],
                             [0.000000, 0.000000, 1.000000]])

# 외부 파라미터 행렬 정의
extrinsic_matrix = np.array(
[[ 0.06060704, -0.99802871,  0.01629351, -0.04498142],
 [ 0.05040392, -0.01324265, -0.99864112,  0.05205786],
 [ 0.99688827,  0.06134594,  0.04950196,  0.51905197]]
)

def do_voxel_grid_downsampling(pcl_data, leaf_size):
    # Voxel Grid Filter를 사용한 다운샘플링
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)  # 리프 크기가 클수록 정보가 적게 유지됨
    return vox.filter()

def do_passthrough(pcl_data, filter_axis, axis_min, axis_max):
    # 축 기반 필터링
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

class Fusion():
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None

        rospy.init_node("Fusion_node", anonymous=False)

        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.camera_callback)
        self.lidar_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)
        self.cone_sub = rospy.Subscriber("cone_result", Image, self.cone_callback)
        self.roi_pub = rospy.Publisher('/roi_pointcloud', PointCloud2, queue_size=10)
        self.blue_pub = rospy.Publisher('/cloud_blue',PointCloud2, queue_size=10)
        self.yellow_pub = rospy.Publisher('/cloud_yellow',PointCloud2, queue_size=10)

        self.yellow_cloud = PointCloud2()
        self.blue_cloud = PointCloud2()
        self.cone_seg = torch.zeros((480, 640, 3))  # [H, W, C]
        # 각 라바콘 영역 내의 LiDAR 포인트 저장
        self.blue_cone_points = None
        self.yellow_cone_points = None
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

    def camera_callback(self, data):
        # 카메라 데이터를 받아와서 이미지를 저장
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # 라바콘을 감지하고 그 좌표를 업데이트
        self.blue_cones, self.yellow_cones = self.process_frame(self.image)

    def cone_callback(self, data):
        cone_seg = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.cone_seg = torch.from_numpy(cone_seg).to(self.device)

    def lidar_callback(self, data):
        global intrinsic_matrix
        global extrinsic_matrix

        # PointCloud2 메시지를 PCL 포맷으로 변환
        pcl_cloud = pcl_helper.ros_to_pcl(data)
        
        # downsampling leaf size는 tuning 필요. 
        pcl_cloud = do_voxel_grid_downsampling(pcl_cloud, 0.5)

        # ROI 설정 및 필터링
        filter_axis = 'x'
        axis_min = 0.0
        axis_max = 12.0
        cloud = do_passthrough(pcl_cloud, filter_axis, axis_min, axis_max)

        filter_axis = 'y'
        axis_min = -3.0
        axis_max = 3.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        # 필터링된 포인트 클라우드를 PointCloud2 메시지로 변환
        roi_cloud = pcl_helper.pcl_to_ros(cloud)
        roi_cloud.header.frame_id = "velodyne"
        self.roi_pub.publish(roi_cloud)

        points = np.array(list(pc2.read_points(roi_cloud, field_names=("x", "y", "z"), skip_nans=True)))
        points = torch.from_numpy(points).to(self.device)

        ones_tensor = torch.ones((points.shape[0], 1), device=self.device)
        world_points = torch.cat((points, ones_tensor), dim=1)

        intrinsic_matrix = torch.tensor(intrinsic_matrix, device=self.device)
        extrinsic_matrix = torch.tensor(extrinsic_matrix, device=self.device)

        with torch.no_grad():
            projected_points = intrinsic_matrix @ extrinsic_matrix @ world_points.T
            image_points = projected_points[:2] / projected_points[2]
            image_points = image_points.T.to(torch.int)

            x_coords = image_points[:, 0]
            y_coords = image_points[:, 1]
            x_min, x_max, y_min, y_max = 0, 640, 0, 480

            in_bounds = (x_coords >= x_min) & (x_coords < x_max) & (y_coords >= y_min) & (y_coords < y_max)
            valid_points = image_points[in_bounds]
            valid_world_points = points[in_bounds]

            valid_x_coords = valid_points[:, 0]
            valid_y_coords = valid_points[:, 1]
        
            blue_channel = self.cone_seg[valid_y_coords, valid_x_coords, 0]
            yellow_channel = self.cone_seg[valid_y_coords, valid_x_coords, 1]

            blue_mask = blue_channel > 100
            yellow_mask = yellow_channel > 100

            self.blue_cone_points = valid_world_points[blue_mask].cpu().numpy()
            self.yellow_cone_points = valid_world_points[yellow_mask].cpu().numpy()

        valid_points_numpy = valid_points.cpu().numpy()
        blue_coords = valid_points_numpy[blue_mask.cpu().numpy()]
        yellow_coords = valid_points_numpy[yellow_mask.cpu().numpy()]

        for coord in blue_coords:
            cv2.circle(self.image, tuple(coord), 2, (255, 0, 0), -1)  # Blue
        for coord in yellow_coords:
            cv2.circle(self.image, tuple(coord), 2, (0, 0, 0), -1)  # Black

        # Display the result
        cv2.imshow("Result", self.image)
        cv2.waitKey(1)

        # Publish clouds
        self.publish_clouds()
    
    def publish_clouds(self) :
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "livox_frame"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        if self.blue_cone_points.size > 0:
            self.blue_cloud = pc2.create_cloud(header, fields, self.blue_cone_points)
            self.blue_pub.publish(self.blue_cloud)

        if self.yellow_cone_points.size > 0:
            self.yellow_cloud = pc2.create_cloud(header, fields, self.yellow_cone_points)
            self.yellow_pub.publish(self.yellow_cloud)

if __name__ == "__main__":
    if not rospy.is_shutdown():
        Fusion()
        rospy.spin()
