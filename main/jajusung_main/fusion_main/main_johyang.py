#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2 
from jajusung_main.msg import HevenCtrlCmd

class Johyang():
    def __init__(self):
        rospy.init_node("Johyang_node", anonymous=False)
        
        self.vel_pub = rospy.Publisher("/drive",HevenCtrlCmd)
        self.stanley_cmd = HevenCtrlCmd()

        self.lidar_sub = rospy.Subscriber("/livox/lidar", PointCloud2, self.lidar_callback)  #pc메시지 구독, 수신되면 lidar_callback함수 실행
        self.stanley_sub = rospy.Subscriber("/drive_stanley", HevenCtrlCmd, self.stanley_cb)
        self.midpoint_path_sub = rospy.Subscriber("/midpoint_path", PointCloud2, self.midpoint_path_callback)


        self.last_midpoint_time = None 

    def midpoint_path_callback(self, data): 
        if data.data != bytes(b'') :
            self.last_midpoint_time = rospy.Time.now()

    def stanley_cb(self, data) :
        self.stanley_cmd = data

        current_time = rospy.Time.now()
        
        if self.last_midpoint_time is None or (current_time - self.last_midpoint_time).to_sec() > 3:
            rospy.loginfo("ESTOP")
            self.stanley_cmd.velocity = 0
            self.stanley_cmd.brake = 1

    def lidar_callback(self, data):
        m = self.stanley_cmd
        self.vel_pub.publish(m)
        
if __name__ == "__main__":
    if not rospy.is_shutdown():
        Johyang()
        rospy.spin() 