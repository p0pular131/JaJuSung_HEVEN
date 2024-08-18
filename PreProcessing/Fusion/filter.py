#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray

def process_frame(frame):
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 100, 40])    # H값 차이 적음 S 명확함
    upper_blue = np.array([150, 255, 255])
    lower_yellow = np.array([5, 130, 90])  # S가 V보다 작음
    upper_yellow = np.array([30, 255, 255])
    
    mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    blue_coords = extract_coordinates(contours_blue)
    yellow_coords = extract_coordinates(contours_yellow)

    return blue_coords, yellow_coords, mask_blue, mask_yellow  # 마스크 반환 추가
def extract_coordinates(contours):
    coords = []
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            coords.append((cx, cy))
    return coords

def image_callback(msg, bridge):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        blue_coords, yellow_coords, mask_blue, mask_yellow = process_frame(cv_image)

        coords = Float32MultiArray()
        coords.data = [item for sublist in (blue_coords + yellow_coords) for item in sublist]
        #pub.publish(coords)
        
        # Optionally, visualize the results
        for coord in blue_coords:
            cv2.circle(cv_image, coord, 5, (0, 255, 0), -1)
        for coord in yellow_coords:
            cv2.circle(cv_image, coord, 5, (255, 255, 0), -1)
            
    
        cv2.imshow('mask_blue',mask_blue)
        cv2.imshow('maks_yellow',mask_yellow)
        cv2.imshow('Detected Cones', cv_image)
        cv2.waitKey(3)

    except CvBridgeError as e:
        print(e)

def main():
    rospy.init_node('cone_detector', anonymous=True)
    bridge = CvBridge()
    pub = rospy.Publisher('cone_coordinates', Float32MultiArray, queue_size=10)
    
    # Subscribe to the image topic from usb_cam
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback, bridge)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
