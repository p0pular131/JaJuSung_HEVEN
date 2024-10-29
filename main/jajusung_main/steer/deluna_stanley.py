#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32MultiArray
from jajusung_main.msg import HevenCtrlCmd

# Initialize constants
k_stanley = 0.2 # steering결정 / target_yaw 비례상수


# Current state of the vehicle (always (0,0,pi/2))
current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0}

def target_callback(msg):
    rospy.loginfo("Control cb")
    data = msg.data
    # Extract target pose (x, y, yaw)
    target_x = data[0]
    target_y = data[1]
    target_yaw = data[2]
    
    # Compute errors
    error_x = target_x - current_pose['x']
    error_y = target_y - current_pose['y']
    distance_error = math.sqrt(error_x**2 + error_y**2)
    # Compute the target direction and heading error
    target_direction = math.atan2(error_y, error_x)
    heading_error = target_yaw - current_pose['yaw'] # 0 직진, 음수 우측, 양수 좌측 ref_yaw
    
    # Normalize heading error to [-pi, pi]
    while heading_error > math.pi:
        heading_error -= 2 * math.pi
    while heading_error < -math.pi:
        heading_error += 2 * math.pi


    # ===================================================================================
    # torc 
    v = 400
    # ===================================================================================

    # ===================================================================================
    '조향각 제어: Stanley'
    lateral_error = distance_error * math.sin(target_direction)  # 경로와의 측면 거리 

    # Stanley control law for steering angle
    # k = 이득 상수 (Stanley gain), v는 차량 속도
    delta = heading_error + math.atan2(k_stanley * lateral_error, v)  # Stanley control 공식
    # ===================================================================================

    # Print debugging information
    print("delta:", math.degrees(delta))
    print("velocity:", v)
    print("lat_err: ",lateral_error)
    print("heading_err: ",heading_error)
    print("target_yaw: ",target_yaw)

    # final drive pub
    cmd_pub = rospy.Publisher('/drive_stanley', HevenCtrlCmd, queue_size=10)
    drive_cmd = HevenCtrlCmd()
    drive_cmd.velocity = v 
    drive_cmd.steering = int(math.degrees(delta))
    drive_cmd.brake = 0
    cmd_pub.publish(drive_cmd)


if __name__ == '__main__':
    rospy.init_node('path_following_controller')

    # Subscribe to the target pose topic
    rospy.Subscriber('/midpoint_gradients', Float32MultiArray, target_callback)

    rospy.spin()