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


    # BEV 변환 적용
    #bev_image = bev_transform(frame, src_points, dst_points, size)
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
        if abs(point[0] - cluster[-1][0]) < 7:  # Threshold for x-coordinate clustering
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
    
def draw_lines(image, coords, color):
    """
    Draw lines connecting the given coordinates on the image.

    Args:
        image: The image on which to draw the lines.
        coords: List of (x, y) tuples representing the coordinates.
        color: The color of the lines (BGR tuple).
    """
    if len(coords) > 1:
        for i in range(len(coords) - 1):
            cv2.line(image, coords[i], coords[i + 1], color, 8) 
            
def smooth_line(coords, window_size=3):
    """
    Smooth the line connecting the coordinates by averaging adjacent points.
    
    Args:
        coords: List of (x, y) tuples representing the coordinates.
        smoothing_factor: The factor to blend points, 0 means no change, 1 means full average.
        
    Returns:
        List of smoothed (x, y) tuples.
    """
    if len(coords) < 2:
        return coords

    smoothed_coords = []
    for i in range(len(coords)):
        start_idx = max(0, i - window_size // 2)
        end_idx = min(len(coords), i + window_size // 2 + 1)
        window_coords = coords[start_idx:end_idx]
        
        avg_x = int(np.mean([p[0] for p in window_coords]))
        avg_y = int(np.mean([p[1] for p in window_coords]))
        smoothed_coords.append((avg_x, avg_y))
        
    return smoothed_coords
    
def ema_smooth(coords, alpha=0.3):
    """
    Apply Exponential Moving Average (EMA) smoothing to the coordinates.
    
    Args:
        coords: List of (x, y) tuples representing the coordinates.
        alpha: Smoothing factor between 0 and 1. Higher alpha gives more weight to recent points.
        
    Returns:
        List of smoothed (x, y) tuples.
    """
    if len(coords) < 2:
        return coords

    smoothed_coords = [coords[0]]  # Start with the first coordinate
    for i in range(1, len(coords)):
        prev_x, prev_y = smoothed_coords[-1]
        curr_x, curr_y = coords[i]
        new_x = int(alpha * curr_x + (1 - alpha) * prev_x)
        new_y = int(alpha * curr_y + (1 - alpha) * prev_y)
        smoothed_coords.append((new_x, new_y))
    
    return smoothed_coords

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
            #blue_coords, yellow_coords = process_frame(cv_image)
            height, width = cv_image.shape[:2]
            print(f"Image Size: Width={width}, Height={height}")
            
            size = (480, 800)
            src_points = np.float32([
            [0, height * 0.6],
            [width, height * 0.6],
            [width, height*0.85],
            [0, height*0.85]])  
            dst_points = np.float32([
            [0, 0],
            [width*0.8, 0],
            [width*0.8, height*1.4],
            [0, height*1.4]])
            # BEV 변환 적용 및 시각화
            bev_image = bev_transform(cv_image, src_points, dst_points, size)
            #bev_image_resized = cv2.resize(bev_image, (480,800))
            blue_coords, yellow_coords = process_frame(bev_image)
            
            coords = Float32MultiArray()
            coords.data = [item for sublist in (blue_coords + yellow_coords) for item in sublist]
            
            pub.publish(coords)
            
            # Combine masks
            mask_blue = cv2.inRange(cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV), lower_blue, upper_blue)
            mask_yellow = cv2.inRange(cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV), lower_yellow, upper_yellow)
            mask = cv2.bitwise_or(mask_blue, mask_yellow)
            # Bitwise-AND mask and original image
            filtered_image = cv2.bitwise_and(bev_image, bev_image, mask=mask)
            
            # Display the resulting frame
            cv_image_copy = cv_image.copy()
            for coord in blue_coords:
                cv2.circle(cv_image_copy, coord, 5, (0, 255, 0), -1)
            for coord in yellow_coords:
                cv2.circle(cv_image_copy, coord, 5, (255, 255, 0), -1)   # 점 반지름 5
            
            smooth_blue_coords = smooth_line(blue_coords)
            smooth_yellow_coords = smooth_line(yellow_coords)
            
            ema_blue_coords = ema_smooth(smooth_blue_coords, alpha=0.3)
            ema_yellow_coords = ema_smooth(smooth_yellow_coords, alpha=0.3)
            
            draw_lines(filtered_image, ema_blue_coords, (255, 0, 0))  # Blue color
            draw_lines(filtered_image, ema_yellow_coords, (0, 255, 255))  # Yellow color
            #cv2.imshow('Original Image', cv_image)
            cv2.imshow('BEV Image', bev_image)
            cv2.imshow('Filtered Image', filtered_image)
            cv2.imshow('Detected Cones', cv_image_copy)
            cv2.imshow('Detected Cones with Lines', cv_image_copy)
            cv2.waitKey(50) # 영상 재생 속도 조절
           
def bev_transform(image, src_points, dst_points, size):
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        bev_img = cv2.warpPerspective(image, matrix, size)
        cv2.imshow('bev', bev_img)
        return bev_img

            



if __name__ == "__main__":
    bag_file_path = '/home/kwonnakyeong/Downloads/colorfilter1.bag'  # 여기에 자신의 bag 파일 경로를 입력하세요
    topic_name = '/usb_cam/image_raw'  # 확인한 토픽 이름으로 수정
    read_bag_file(bag_file_path, topic_name)
    cv2.destroyAllWindows()
