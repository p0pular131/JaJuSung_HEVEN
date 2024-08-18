#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import pcl
import ros_numpy
import numpy as np

class PointCloudSaver:
    def __init__(self):
        # 최초 토픽 수신 여부 플래그
        self.first_received = False

        # 노드 초기화
        rospy.init_node('pointcloud_saver', anonymous=True)

        # 토픽 구독
        rospy.Subscriber("/livox/lidar", PointCloud2, self.callback)

    def callback(self, msg):
        if not self.first_received:
            rospy.loginfo("PointCloud2 메시지를 수신했습니다. PCD 파일로 저장 중...")

            # PointCloud2 메시지를 NumPy 배열로 변환
            pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

            # x, y, z 필드만 추출
            points = np.zeros((pc_np.shape[0], 3), dtype=np.float32)
            points[:, 0] = pc_np['x']
            points[:, 1] = pc_np['y']
            points[:, 2] = pc_np['z']

            # PCL 포인트 클라우드 생성
            cloud = pcl.PointCloud()
            cloud.from_array(points)

            # PCD 파일로 저장
            pcl.save(cloud, "output.pcd")
            rospy.loginfo("PCD 파일이 저장되었습니다: output.pcd")

            # 최초 토픽 수신으로 플래그 설정
            self.first_received = True

    def run(self):
        # ROS 루프 실행
        rospy.spin()

if __name__ == '__main__':
    saver = PointCloudSaver()
    saver.run()
