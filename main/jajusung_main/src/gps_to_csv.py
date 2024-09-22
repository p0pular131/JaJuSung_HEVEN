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
        self.current_yaw = None  # To store the current yaw value from IMU

        rospy.init_node("recording_node")
        rospy.sleep(1)
        
        # Subscribers
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self._gps_callback)
        rospy.Subscriber("/imu", Imu, self._imu_callback)
        rospy.loginfo("Gps Tracker Node on. Waiting for topics...")

        rospy.spin()

    # GPS Callback function
    def _gps_callback(self, data):
        if self.cnt % CNT == 0:
            if self.current_yaw is not None:  # Ensure IMU data has been received
                data_list.append([data.latitude, data.longitude, self.current_yaw])
                rospy.loginfo("Get row #%3d with Yaw.", self.idx_cnt / CNT)
        self.idx_cnt += 1
        self.cnt = (self.cnt + 1) % CNT

    # IMU Callback function
    def _imu_callback(self, data):
        # Convert quaternion to Euler angles and extract yaw
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # Store the yaw value
        self.current_yaw = yaw

if __name__ == "__main__":
    try:
        GpsTracker()
    except rospy.ROSInterruptException:
        pass

    # Write data to CSV file
    with open(csv_file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        for data in data_list:
            a_data, b_data, yaw_data = data  # Separate GPS and yaw data
            csv_writer.writerow([a_data, b_data, yaw_data])  # Write to CSV

    print("Write done!")
