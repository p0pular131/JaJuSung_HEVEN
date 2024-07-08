#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from sklearn.cluster import DBSCAN


def process_frame(frame):
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 190, 100])    # H값 차이 적음 S 명확함
    upper_blue = np.array([150, 255, 255])
    lower_yellow = np.array([5, 130, 160])  # S가 V보다 작음
    upper_yellow = np.array([30, 255, 255])
    
    # Threshold the HSV image to get only blue and yellow colors
    mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    
    # Find contours for blue and yellow masks
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    blue_coords = []
    for contour in contours_blue:
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            blue_coords.append((cx, cy))
    
    yellow_coords = []
    for contour in contours_yellow:
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            yellow_coords.append((cx, cy))
    
    h_channel, s_channel, v_channel = cv2.split(hsv_image)
    # cv2.imshow('Hue Channel', h_channel)
    # cv2.imshow('Saturation Channel', s_channel)
    # cv2.imshow('Value Channel', v_channel)


    # Apply clustering to reduce multiple points per cone to a single point
    blue_coords = cluster_coordinates(blue_coords)
    yellow_coords = cluster_coordinates(yellow_coords)
    print("blue",blue_coords)
    print("yellow",yellow_coords)

    return blue_coords, yellow_coords

def cluster_coordinates(coords):
    if len(coords) == 0:
        return coords
    coords_array = np.array(coords)
    
    # Sort points by x-coordinate
    coords_array = coords_array[np.argsort(coords_array[:, 0])]
    
    clustered_coords = []
    cluster = [coords_array[0]]
    
    for point in coords_array[1:]:
        if abs(point[0] - cluster[-1][0]) < 10:  # Threshold for x-coordinate clustering
            cluster.append(point)
        else:
            # Calculate the average y-coordinate for the cluster
            centroid_x = np.mean([p[0] for p in cluster])
            centroid_y = np.mean([p[1] for p in cluster])
            clustered_coords.append((int(centroid_x), int(centroid_y)))
            cluster = [point]
    # Add the last cluster
    if cluster:
        centroid_x = np.mean([p[0] for p in cluster])
        centroid_y = np.mean([p[1] for p in cluster])
        clustered_coords.append((int(centroid_x), int(centroid_y)))


    return clustered_coords


def read_bag_file(bag_file_path, topic_name):

    lower_blue = np.array([90, 190, 100])    # H값 차이 적음 S 명확함
    upper_blue = np.array([150, 255, 255])
    lower_yellow = np.array([5, 130, 160])  # S가 V보다 작음
    upper_yellow = np.array([30, 255, 255])

    bridge = CvBridge()
    pub = rospy.Publisher('cone_coordinates', Float32MultiArray, queue_size=10)
    rospy.init_node('cone_detector', anonymous=True)
    
    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            blue_coords, yellow_coords = process_frame(cv_image)
            
            coords = Float32MultiArray()
            coords.data = [item for sublist in (blue_coords + yellow_coords) for item in sublist]
            
            pub.publish(coords)
            
            # Combine masks
            mask_blue = cv2.inRange(cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV), lower_blue, upper_blue)
            mask_yellow = cv2.inRange(cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV), lower_yellow, upper_yellow)
            mask = cv2.bitwise_or(mask_blue, mask_yellow)
            # Bitwise-AND mask and original image
            filtered_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            
            # Display the resulting frame
            cv_image_copy = cv_image.copy()
            for coord in blue_coords:
                cv2.circle(cv_image_copy, coord, 5, (0, 255, 0), -1)
            for coord in yellow_coords:
                cv2.circle(cv_image_copy, coord, 5, (255, 255, 0), -1)   # 점 반지름 5
            
            #cv2.imshow('Original Image', cv_image)
            cv2.imshow('Filtered Image', filtered_image)
            cv2.imshow('Detected Cones', cv_image_copy)
            cv2.waitKey(50) # 영상 재생 속도 조절

            



if __name__ == "__main__":
    bag_file_path = '/home/handh0405/colorfilter1.bag'  # 여기에 자신의 bag 파일 경로를 입력하세요
    topic_name = '/usb_cam/image_raw'  # 확인한 토픽 이름으로 수정
    read_bag_file(bag_file_path, topic_name)
    cv2.destroyAllWindows()