#!/usr/bin/env python3

import rospy
import csv
from sensor_msgs.msg import NavSatFix
from math import *

csv_file_path = 'gps_test.csv'

data_list = []

CNT = 10

class GpsTracker():
    def __init__(self):
        self.cnt = 0
        self.idx_cnt = 0
        
        rospy.init_node("recording_node")
        rospy.sleep(1)
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self._gps_callback)
        rospy.loginfo("Gps Tracker Node on. Waiting for topics...")

        rospy.spin()

    # Functions
    def _gps_callback(self, data=NavSatFix):
        if self.cnt % CNT == 0:
            data_list.append([data.latitude, data.longitude])
            rospy.loginfo("Get row #%3d.", self.idx_cnt / CNT)
        self.idx_cnt += 1
        self.cnt = (self.cnt + 1) % CNT


if __name__ == "__main__":
    try:
        GpsTracker()
    except rospy.ROSInterruptException:
        pass

    with open(csv_file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        for data in data_list:
            a_data, b_data = data  # 데이터를 a와 b 열로 분리
            csv_writer.writerow([a_data, b_data])  # CSV 파일에 추가하기

    print("Write done!")