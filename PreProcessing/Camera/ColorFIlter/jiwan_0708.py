#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def read_bag_file(bag_file_path, topic_name):
    bridge = CvBridge()

    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            h_channel, s_channel, v_channel = cv2.split(hsv_image)


            # Define range for blue color in HSV
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])
            # Define range for yellow color in HSV
            lower_yellow = np.array([5, 130, 160])     # s 보다 v가 더 커도 됨
            upper_yellow = np.array([30, 255, 255])
            # Threshold the HSV image to get only blue and yellow colors
            mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
            mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
            # Combine masks
            mask = cv2.bitwise_or(mask_blue, mask_yellow)
            # Bitwise-AND mask and original image
            filtered_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            # Display the resulting frame

            # cv2.imshow('Hue Channel', h_channel)
            # cv2.imshow('Saturation Channel', s_channel)
            # cv2.imshow('Value Channel', v_channel)
            cv2.imshow('Original Image', cv_image)
            cv2.imshow('Filtered Image', filtered_image)

            cv2.waitKey(15)

if __name__ == "__main__":
    bag_file_path = '/home/handh0405/colorfilter1.bag'  # 여기에 자신의 bag 파일 경로를 입력하세요
    topic_name = '/usb_cam/image_raw'  # 확인한 토픽 이름으로 수정
    read_bag_file(bag_file_path, topic_name)
    cv2.destroyAllWindows()
