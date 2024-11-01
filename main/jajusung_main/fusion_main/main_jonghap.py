#!/usr/bin/env python3

import rospy
import lidar_pcl_helper as pcl_helper
from sensor_msgs.msg import PointCloud2 
from jajusung_main.msg import HevenCtrlCmd

class Jonghap():
    def __init__(self):
        rospy.init_node("Fusion_node", anonymous=False)
        
        self.vel_pub = rospy.Publisher("/drive",HevenCtrlCmd)
        self.stanley_cmd = HevenCtrlCmd()
        self.init_time = rospy.Time.now()
        while True :
            current_time = rospy.Time.now() 
            if (current_time - self.init_time).to_sec() > 5 :
                for i in range(100) :
                    rospy.loginfo("Start ! ! ")
                    self.stanley_cmd.brake = -1
                    self.stanley_cmd.steering = 0
                    self.stanley_cmd.velocity = 0
                    self.vel_pub.publish(self.stanley_cmd)       
                break
            else :
                rospy.loginfo("Start Line ESTOP")
                self.stanley_cmd.brake = 1
                self.stanley_cmd.steering = 0
                self.stanley_cmd.velocity = 0
                self.vel_pub.publish(self.stanley_cmd)         
        
        self.lidar_sub = rospy.Subscriber("/livox/lidar", PointCloud2, self.lidar_callback)  #pc메시지 구독, 수신되면 lidar_callback함수 실행
        self.stanley_sub = rospy.Subscriber("/drive_stanley", HevenCtrlCmd, self.stanley_cb)

    def stanley_cb(self, data) :
        self.stanley_cmd.velocity = 300

        self.stanley_cmd = data

    def lidar_callback(self, data):
        m = self.stanley_cmd

        self.vel_pub.publish(m)
        

if __name__ == "__main__":
    if not rospy.is_shutdown():
        Jonghap()
        rospy.spin() 