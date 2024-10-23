#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from heven_ev_auto.msg import HevenCtrlCmd

# Don't change
MAX_DISTANCE = 1200
OBSTACLE_RADIUS = 30
ACT_RAD = 300
# Arc angle for detection
DANGER_MIN_ARC_ANGLE = 87
DANGER_MAX_ARC_ANGLE = 92
MIN_ARC_ANGLE = 70
MAX_ARC_ANGLE = 110
SECOND_MIN_ARC_ANGLE = 50
SECOND_MAX_ARC_ANGLE = 130
ANGLE_INCREMENT = 4
# DETECTION_RANGE
DETECT_RANGE = 100
DANGER_DETECT_RANGE = 500
# Y offset
Y_OFFSET = 0


class SteeringCtrl:
    def __init__(self, default_vel):
        rospy.Subscriber("/scan", LaserScan, self.callback)
        self.DEFAULT_VAL = default_vel
        self.prev_angle = 0
        self.lidar_raw_data = np.array([])
        self.calculate_dis = False
        self.avg_left_dis = 0
        self.avg_right_dis = 0
        self.avg_danger_dis = 0
    
    def callback(self, data):
        self.lidar_raw_data = data.ranges

    def main(self):
        # Filter 'inf' value
        lidar_raw_data = np.array([self.lidar_raw_data[i] for i in range(len(self.lidar_raw_data)) if i % 7 == 0]) * 40
        current_frame = np.zeros((ACT_RAD * 2, ACT_RAD, 3), dtype=np.uint8)
        measured_points = []
        available_angles = []
        middle_angle = 0
        left_dis_list = []
        right_dis_list = []
        danger_dis_list = []

        # Steering
        # ================================================================================================
        for i in range(len(lidar_raw_data)):
            if i <= 25 or i > len(lidar_raw_data) - 25:
                continue
            # Skip empty points
            if lidar_raw_data[i] > MAX_DISTANCE:
                continue
            # Calculate xy points
            xy = [lidar_raw_data[i] * np.cos(np.deg2rad(180 - i)), lidar_raw_data[i] * np.sin(np.deg2rad(180 - i))]
            rospy.loginfo(xy)
            measured_points.append(xy)

        # Mark points on the map
        for point in measured_points:
            cv2.circle(current_frame, (int(ACT_RAD / 2 - np.round(point[0])), int(ACT_RAD * 2 - np.round(point[1]))), OBSTACLE_RADIUS, (255, 255, 255), -1)

        # Draw a line to danger
        for theta in range(DANGER_MIN_ARC_ANGLE - 90, DANGER_MAX_ARC_ANGLE - 90):
            # Find maximum length of line
            r = 1
            while r < DANGER_DETECT_RANGE:
                if current_frame[int(ACT_RAD * 2 - 1 - Y_OFFSET - np.round(r * np.cos(np.deg2rad(theta))))][int(ACT_RAD / 2 - 1 - np.round(r * np.sin(np.deg2rad(theta))))][0] == 255:
                    break
                r += 5
            danger_dis_list.append(r)

            # draw a gray line (not detected)
            cv2.line(current_frame, 
                    (int(ACT_RAD / 2 - 1), int(ACT_RAD * 2 - 1 - Y_OFFSET)), 
                    (int(ACT_RAD / 2 - 1 - np.round(DANGER_DETECT_RANGE * np.sin(np.deg2rad(theta)))), 
                    int(ACT_RAD * 2 - 1 - np.round(DANGER_DETECT_RANGE * np.cos(np.deg2rad(theta)) - Y_OFFSET))), 
                    (200, 200, 200), 1)
        
        if danger_dis_list:
            self.avg_danger_dis = sum(danger_dis_list) / len(danger_dis_list)

        # Draw a line to obstacles
        for theta in range(MIN_ARC_ANGLE - 90, MAX_ARC_ANGLE - 90, ANGLE_INCREMENT):
            # Find maximum length of line
            r = 1
            while r < DETECT_RANGE:
                if current_frame[int(ACT_RAD * 2 - 1 - Y_OFFSET - np.round(r * np.cos(np.deg2rad(theta))))][int(ACT_RAD / 2 - 1 - np.round(r * np.sin(np.deg2rad(theta))))][0] == 255:
                    break
                r += 2
            if theta > 0:
                right_dis_list.append(r)
            else:
                left_dis_list.append(r)

            if r <= DETECT_RANGE:
                # draw a red line (detected)
                cv2.line(current_frame, 
                         (int(ACT_RAD / 2 - 1), int(ACT_RAD * 2 - 1 - Y_OFFSET)), 
                         (int(ACT_RAD / 2 - 1 - np.round(r * np.sin(np.deg2rad(theta)))), 
                         int(ACT_RAD * 2 - 1 - np.round(r * np.cos(np.deg2rad(theta)) - Y_OFFSET))), 
                         (0, 0, 255), 1)
            else:
                # draw a gray line (not detected)
                cv2.line(current_frame, 
                         (int(ACT_RAD / 2 - 1), int(ACT_RAD * 2 - 1 - Y_OFFSET)), 
                         (int(ACT_RAD / 2 - 1 - np.round(DETECT_RANGE * np.sin(np.deg2rad(theta)))), 
                         int(ACT_RAD * 2 - 1 - np.round(DETECT_RANGE * np.cos(np.deg2rad(theta)) - Y_OFFSET))), 
                         (0, 255, 0), 1)
                available_angles.append(theta)

        if left_dis_list:
            self.avg_left_dis = sum(left_dis_list) / len(left_dis_list)
        if right_dis_list:
            self.avg_right_dis = sum(right_dis_list) / len(right_dis_list)

        # Control angle
        if len(available_angles) == 0:
            for theta in range(SECOND_MIN_ARC_ANGLE - 90, MIN_ARC_ANGLE - 90, ANGLE_INCREMENT):
                # Find maximum length of line
                r = 1
                while r < DETECT_RANGE:
                    if current_frame[int(ACT_RAD * 2 - 1 - Y_OFFSET - np.round(r * np.cos(np.deg2rad(theta))))][int(ACT_RAD / 2 - 1 - np.round(r * np.sin(np.deg2rad(theta))))][0] == 255:
                        break
                    r += 2
                right_dis_list.append(r)

                if r <= DETECT_RANGE:
                    # draw a red line (detected)
                    cv2.line(current_frame, 
                             (int(ACT_RAD / 2 - 1), int(ACT_RAD * 2 - 1 - Y_OFFSET)), 
                             (int(ACT_RAD / 2 - 1 - np.round(r * np.sin(np.deg2rad(theta)))), 
                             int(ACT_RAD * 2 - 1 - np.round(r * np.cos(np.deg2rad(theta)) - Y_OFFSET))), 
                             (0, 0, 255), 1)
                else:
                    # draw a gray line (not detected)
                    cv2.line(current_frame, 
                             (int(ACT_RAD / 2 - 1), int(ACT_RAD * 2 - 1 - Y_OFFSET)), 
                             (int(ACT_RAD / 2 - 1 - np.round(DETECT_RANGE * np.sin(np.deg2rad(theta)))), 
                             int(ACT_RAD * 2 - 1 - np.round(DETECT_RANGE * np.cos(np.deg2rad(theta)) - Y_OFFSET))), 
                             (0, 255, 0), 1)
                    available_angles.append(theta)

            if len(available_angles) == 0:
                middle_angle = self.prev_angle
            else:
                middle_angle = np.median(np.array(available_angles), axis=0)
        else:
            middle_angle = np.median(np.array(available_angles), axis=0)

        self.prev_angle = middle_angle
        cv2.line(current_frame, 
                 (int(ACT_RAD / 2 - 1), int(ACT_RAD * 2 - 1)), 
                 (int(ACT_RAD / 2 - 1 - np.round(DETECT_RANGE * np.sin(np.deg2rad(middle_angle)))), 
                 int(ACT_RAD * 2 - 1 - np.round(DETECT_RANGE * np.cos(np.deg2rad(middle_angle))))), 
                 (255, 255, 255), 1)
        
        # Show the result
        cv2.imshow("result", current_frame)
        cv2.waitKey(1)

        # Make control message
        m = HevenCtrlCmd()
        m.velocity = self.DEFAULT_VAL
        m.steering = -self.determ_angle(middle_angle)
        m.brake = 0

        return m
    
    def determ_angle(self, before_ang):
        if before_ang >= 40:
            return 40
        elif before_ang <= -40:
            return -40
        else:
            return before_ang


if __name__ == "__main__":
    rospy.init_node('steering_ctrl_node')
    p = SteeringCtrl(default_vel=20)  # Example default velocity
    rospy.spin()
