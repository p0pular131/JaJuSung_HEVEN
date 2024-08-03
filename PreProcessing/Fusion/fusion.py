#! /usr/bin/env python3

import rospy
import cv2
import numpy as np

import lidar_pcl_helper as pcl_helper
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from math import *
"""
LiDAR 좌표 -> x, y, z축
Camera 좌표 -> u, v, w축
LiDAR 좌표를 Camera 좌표로 투영시키는 Calibration 진행
"""
# 각각 x, y, z축 기준 회전각도 (카메라를 LiDAR와 align하기 위한 LiDAR 축 기준 회전각)
ALPHA = 5
BETA = 2
GAMMA = 0
# 640, 480
# intrinsic_matrix = np.array([[791.096231, 0.000000,   306.054697], 
#                              [0.000000,   790.812314, 178.231744], 
#                              [0.000000,   0.000000,   1.000000]])

intrinsic_matrix = np.array([[611.768234, 0.000000, 306.164069],
                            [0.000000, 613.154786, 233.896019],
                            [0.000000, 0.000000, 1.000000]])

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

R = R_z @ R_y @ R_x @ R_axis

# lidar 기준으로 카메라는 lidar와 x축 -0.12, z축 0.28만큼 떨어져 있음. 
T = np.array([[0.0], [0.07], [0.-0.04]])
# T = R_axis @ T

extrinsic_matrix = np.hstack((R,T))

# extrinsic_matrix = np.array([[ 1.54781499e-01,  9.84256401e-01,  8.53347814e-02 , 5.09525906e-01],
#                             [ -3.67614971e-01, -2.27945052e-02,  9.29698684e-01 , 5.07544055e+01],
#                             [ 9.17007045e-01, -1.75270499e-01 , 3.58299221e-01 ,-2.58618128e+02]])

extrinsic_matrix = np.array(
[[ 0.05295328, -0.99799799, -0.03458266,  0.0826162 ],
 [ 0.04563755,  0.03701367, -0.99827211, -0.24284274],
 [ 0.99755359,  0.05128351,  0.04750618, -0.36089244]]
)

def do_voxel_grid_downssampling(pcl_data,leaf_size):
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
    return vox.filter()

def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
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
        self.lidar_sub = rospy.Subscriber("/livox/lidar", PointCloud2, self.lidar_callback)
        self.roi_pub = rospy.Publisher('/roi_pointcloud', PointCloud2, queue_size=10)

    def camera_callback(self, data):

        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        self.image_time = data.header.stamp.to_sec()

    def lidar_callback(self, data):
        
        cloud = pcl_helper.ros_to_pcl(data)

        # cloud = do_voxel_grid_downssampling(cloud_orin, 0.1)
        # ROI
        filter_axis = 'z'
        axis_min = 0.0
        axis_max = 2.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        # filter_axis = 'y'
        # axis_min = -1.0
        # axis_max = 1.0
        # cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
        
        # pcd -> 3.5, 3.8
        filter_axis = 'x'
        axis_min = 3.0
        axis_max = 12.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        roi_cloud = pcl_helper.pcl_to_ros(cloud)
        roi_cloud.header.frame_id = "velodyne"
        self.roi_pub.publish(roi_cloud)
    
        if cloud.size > 0:

            for point in cloud:
                x = point[0]
                y = point[1]
                z = point[2]

                # 4x1 vector
                world_point = np.array([[x], [y], [z], [1]]) 

                # 3x1 vector
                projected_point = intrinsic_matrix @ extrinsic_matrix @ world_point

                scaling = projected_point[2][0]

                if scaling==0 :
                    continue

                image_point = projected_point / scaling

                coord = (int(image_point[0][0]), int(image_point[1][0]))

                if 0 <= coord[0] < 640 and 0 <= coord[1] < 480:
                    cv2.circle(self.image, coord, 2, (255, 0, 255), -1)
                
                '''
                나중에 센퓨 완성되면 라바콘 영역들안에 있는지 체크하고 그 3d point들만 따로 list에 추가해서
                pointcloud_to_laserscan으로 넘기는 코드 작성하면 될 것 같습니다. 
                '''
            
            cv2.imshow("Result", self.image)
            cv2.waitKey(1)
            
        else:
            cv2.imshow("Result", self.image)
            cv2.waitKey(1)
            pass
    

if __name__ == "__main__":

    if not rospy.is_shutdown():
        Fusion()
        rospy.spin()