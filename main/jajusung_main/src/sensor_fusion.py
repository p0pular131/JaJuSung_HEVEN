#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pcl  # Python PCL 라이브러리
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
import std_msgs.msg
import lidar_pcl_helper as pcl_helper
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from math import radians, cos, sin

# 각 x, y, z축 기준 회전각도 (카메라를 LiDAR와 맞추기 위한 회전각)
ALPHA = 5
BETA = 2
GAMMA = 0

# 카메라의 내부 파라미터 행렬
intrinsic_matrix = np.array([[611.768234, 0.000000, 306.164069],
                             [0.000000, 613.154786, 233.896019],
                             [0.000000, 0.000000, 1.000000]])

# 회전 행렬 정의
R_x = np.array([[1, 0, 0], 
                [0, cos(radians(ALPHA)), -sin(radians(ALPHA))], 
                [0, sin(radians(ALPHA)), cos(radians(ALPHA))]])

R_y = np.array([[cos(radians(BETA)), 0, sin(radians(BETA))],
                [0, 1, 0],
                [-sin(radians(BETA)), 0, cos(radians(BETA))]])

R_z = np.array([[cos(radians(GAMMA)), -sin(radians(GAMMA)), 0],
                [sin(radians(GAMMA)), cos(radians(GAMMA)), 0],
                [0, 0, 1]])

R_axis = np.array([[0, -1, 0],
                   [0, 0, -1],
                   [1, 0, 0]])

# 최종 회전 행렬
R = R_z @ R_y @ R_x @ R_axis

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
        self.roi_pub = rospy.Publisher('/roi_pointcloud', PointCloud2, queue_size=10)
        self.blue_pub = rospy.Publisher('/cloud_blue',PointCloud2, queue_size=10)
        self.yellow_pub = rospy.Publisher('/cloud_yellow',PointCloud2, queue_size=10)

        self.yellow_cloud = PointCloud2()
        self.blue_cloud = PointCloud2()

        # 각 라바콘 영역 내의 LiDAR 포인트 저장
        self.blue_cone_points = []
        self.yellow_cone_points = []

    def camera_callback(self, data):
        # 카메라 데이터를 받아와서 이미지를 저장
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # 라바콘을 감지하고 그 좌표를 업데이트
        self.blue_cones, self.yellow_cones = self.process_frame(self.image)

    def process_frame(self, frame):
        # 이미지 데이터를 HSV 색상 공간으로 변환
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 색상 범위 정의
        # lower_blue = np.array([90, 100, 40])    # 파란색 범위
        lower_blue = np.array([90, 190, 100])    # 파란색 범위
        upper_blue = np.array([150, 255, 255])

        lower_yellow = np.array([5, 70, 120])  # 노란색 범위
        # lower_yellow = np.array([5, 130, 160])  # 노란색 범위
        upper_yellow = np.array([30, 255, 255])

        
        # HSV 이미지에서 파란색과 노란색만 추출
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        
        # 각 색상의 마스크에서 컨투어 찾기
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        blue_coords = []
        for contour in contours_blue:
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                blue_coords.append((cx, cy))
        
        yellow_coords = []
        for contour in contours_yellow:
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                yellow_coords.append((cx, cy))

        return blue_coords, yellow_coords

    def lidar_callback(self, data):
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
    
        # 각 라바콘 영역 내의 LiDAR 포인트 저장 리스트 초기화
        self.blue_cone_points.clear()
        self.yellow_cone_points.clear()

        for point in pc2.read_points(roi_cloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[:3]

            # 4x1 벡터 생성
            world_point = np.array([[x], [y], [z], [1]]) 

            # 3x1 벡터로 변환
            projected_point = intrinsic_matrix @ extrinsic_matrix @ world_point

            scaling = projected_point[2][0]

            if scaling == 0:
                continue

            image_point = projected_point / scaling

            coord = (int(image_point[0][0]), int(image_point[1][0]))

            # 이미지 내의 좌표 체크
            if 0 <= coord[0] < 640 and 0 <= coord[1] < 480:
                # 파란색 라바콘 영역 내의 포인트
                if self.is_point_in_cone(coord, self.blue_cones):
                    self.blue_cone_points.append((x, y, z))
                    cv2.circle(self.image, coord, 2, (255, 0, 0), -1)  # 파란색으로 표시
                # 노란색 라바콘 영역 내의 포인트
                elif self.is_point_in_cone(coord, self.yellow_cones):
                    self.yellow_cone_points.append((x, y, z))
                    cv2.circle(self.image, coord, 2, (0, 255, 255), -1)  # 노란색으로 표시

        # 결과 출력
        # print("Blue cone LiDAR points:", self.blue_cone_points)
        # print("Yellow cone LiDAR points:", self.yellow_cone_points)
                    
        cv2.imshow("Result", self.image)
        cv2.waitKey(1)
                    
        self.publish_clouds()
    
    def is_point_in_cone(self, coord, cone_coords, radius=20):
        """
        2D 이미지 좌표에서 특정 라바콘 영역 내의 포인트인지 체크
        """
        for (cx, cy) in cone_coords:
            if (coord[0] - cx) ** 2 + (coord[1] - cy) ** 2 <= radius ** 2:
                return True
        return False
    
    def publish_clouds(self) :
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "livox_frame"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        if self.blue_cone_points:
            self.blue_cloud = pc2.create_cloud(header, fields, self.blue_cone_points)
            self.blue_pub.publish(self.blue_cloud)

        if self.yellow_cone_points:
            self.yellow_cloud = pc2.create_cloud(header, fields, self.yellow_cone_points)
            self.yellow_pub.publish(self.yellow_cloud)

if __name__ == "__main__":
    if not rospy.is_shutdown():
        Fusion()
        rospy.spin()
