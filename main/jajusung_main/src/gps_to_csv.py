#!/usr/bin/env python3

import rospy
import csv
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import euler_from_quaternion
from math import *

# CSV file path
csv_file_path = 'gps_test.csv'

# List to store data
data_list = []

# Frequency for saving data
CNT = 10

class GpsTracker():
    def __init__(self):
        self.cnt = 0
        self.idx_cnt = 0

        rospy.init_node("recording_node")
        
        # Subscribers
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self._gps_callback)
        rospy.loginfo("Gps Tracker Node on. Waiting for topics...")

    # GPS Callback function
    def _gps_callback(self, data):
        if self.cnt % CNT == 0:
            data_list.append([data.latitude, data.longitude])
            rospy.loginfo("Get row #%3d with Yaw.", self.idx_cnt / CNT)
        self.idx_cnt += 1
        self.cnt = (self.cnt + 1) % CNT


if __name__ == "__main__":
    try:
        GpsTracker()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    # Write data to CSV file
    with open(csv_file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        for data in data_list:
            a_data, b_data = data  # Separate GPS and yaw data
            csv_writer.writerow([a_data, b_data])  # Write to CSV

    print("Write done!")
