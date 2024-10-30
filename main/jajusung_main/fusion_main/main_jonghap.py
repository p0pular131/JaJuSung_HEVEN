#!/usr/bin/env python3

import rospy
import lidar_pcl_helper as pcl_helper
from sensor_msgs.msg import PointCloud2 
from jajusung_main.msg import HevenCtrlCmd

class Jonghap():
    def __init__(self):
        rospy.init_node("Fusion_node", anonymous=False)
        # LiDAR 데이터 구독
        self.lidar_sub = rospy.Subscriber("/livox/lidar", PointCloud2, self.lidar_callback)  #pc메시지 구독, 수신되면 lidar_callback함수 실행
        self.stanley_sub = rospy.Subscriber("/drive_stanley", HevenCtrlCmd, self.stanley_cb)
        self.vel_pub = rospy.Publisher("/drive",HevenCtrlCmd)
        self.stanley_cmd = HevenCtrlCmd()

    def stanley_cb(self, data) :
        self.stanley_cmd = data

    def lidar_callback(self, data):
        m = self.stanley_cmd

        self.vel_pub.publish(m)
        

if __name__ == "__main__":
    if not rospy.is_shutdown():
        Jonghap()
        rospy.spin() 