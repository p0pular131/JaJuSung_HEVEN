#!/usr/bin/env python3

import rospy
import lidar_pcl_helper as pcl_helper
from sensor_msgs.msg import PointCloud2 
from jajusung_main.msg import HevenCtrlCmd

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

class Jeddong():
    def __init__(self):
        rospy.init_node("Fusion_node", anonymous=False)
        # LiDAR 데이터 구독
        self.lidar_sub = rospy.Subscriber("/livox/lidar", PointCloud2, self.lidar_callback)  #pc메시지 구독, 수신되면 lidar_callback함수 실행
        self.stanley_sub = rospy.Subscriber("/drive_stanley", HevenCtrlCmd, self.stanley_cb)
        self.vel_pub = rospy.Publisher("/drive",HevenCtrlCmd)
        self.braking = False
        self.stanley_cmd = HevenCtrlCmd()
        self.braking_dixtance = 5.0
        self.point_count_threshold = 4

    def stanley_cb(self, data) :
        self.stanley_cmd = data

    def lidar_callback(self, data):
        # PointCloud2 메시지를 PCL 포맷으로 변환
        pcl_cloud = pcl_helper.ros_to_pcl(data)
        # ROI 설정 및 필터링
        filter_axis = 'y'
        axis_min = -0.5
        axis_max = 0.5
        cloud = do_passthrough(pcl_cloud, filter_axis, axis_min, axis_max)
        filter_axis = 'x'
        axis_min = 2.0
        axis_max = 15.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
        filter_axis = 'z'
        axis_min = -1.1
        axis_max = 0.5
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        points_within_distance = [point for point in cloud if point[0] <= self.braking_dixtance]
        
        if len(points_within_distance) >= self.point_count_threshold :
            self.braking = True
            # print(points_within_distance)

        m = self.stanley_cmd
        if(self.braking) :
            m.brake = 1
            m.velocity = 0
            rospy.loginfo("=============ESTOP!!")
        else :
            rospy.loginfo("Drive . . . ")
        self.vel_pub.publish(m)
        

if __name__ == "__main__":
    if not rospy.is_shutdown():
        Jeddong()
        rospy.spin() 