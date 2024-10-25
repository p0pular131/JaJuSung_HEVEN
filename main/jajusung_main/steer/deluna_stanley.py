#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32MultiArray
from jajusung_main.msg import HevenCtrlCmd

# Initialize constants
K_v_dist = 1.0  # velocity결정 / diatance_error 비례상수
k_stanley = 0.2 # steering결정 / target_yaw 비례상수

# Speed and steering angle limits
MIN_SPEED = 1.38889  # 5 km/h in m/s
MAX_SPEED = 6.944  # 25 km/h in m/s
MIN_STEERING = -0.488  # -28 degrees in radians
MAX_STEERING = 0.488  # 28 degrees in radians

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
    '속력 제어: 거리 비례'
    '최대한 빠르게 바꿀 필요있음'
    v = 8 / 3.6
    # ===================================================================================

    # ===================================================================================
    '조향각 제어: Stanley'
    lateral_error = distance_error * math.sin(target_direction)  # 경로와의 측면 거리 

    # Stanley control law for steering angle
    # k = 이득 상수 (Stanley gain), v는 차량 속도
    delta = heading_error + math.atan2(k_stanley * lateral_error, v)  # Stanley control 공식
    # ===================================================================================

    # Clamp the steering angle to the defined limits
    delta = max(MIN_STEERING + math.pi/2, min(MAX_STEERING + math.pi/2, delta + math.pi/2))

    delta -= math.pi/2

    # Clamp velocity to defined limits
    v = max(MIN_SPEED, min(MAX_SPEED, v))
    
    # Print debugging information
    print("delta:", math.degrees(delta))
    print("velocity:", v)
    print("lat_err: ",lateral_error)
    print("heading_err: ",heading_error)
    print("target_yaw: ",target_yaw)

    # final drive pub
    cmd_pub = rospy.Publisher('/drive', HevenCtrlCmd, queue_size=10)
    drive_cmd = HevenCtrlCmd()
    drive_cmd.velocity = 400 
    drive_cmd.steering = int(math.degrees(delta))
    dirve_cmd.brake = 0
    cmd_pub.publish(drive_cmd)


if __name__ == '__main__':
    rospy.init_node('path_following_controller')

    # Subscribe to the target pose topic
    rospy.Subscriber('/midpoint_gradients', Float32MultiArray, target_callback)

    rospy.spin()