#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

class EKF:
    def __init__(self):
        # EKF 초기화
        self.ekf = ExtendedKalmanFilter(dim_x=3, dim_z=1)  # 상태 벡터: [yaw, vyaw, yaw_dot]
        self.ekf.x = np.zeros(3)  # 초기 상태 [yaw, vyaw, yaw_dot]
        self.ekf.P *= 1000.  # 초기 불확실성
        self.ekf.R = np.array([[0.1]])  # 측정 잡음 공분산
        self.ekf.Q = np.eye(3) * 0.01  # 프로세스 잡음 공분산
        self.initialized = False


    def initialize_yaw(self, orientation):
        # 오리엔테이션으로 초기 yaw 값 설정
        yaw = np.arctan2(2 * (orientation.z * orientation.w + orientation.x * orientation.y),
                         1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
        self.ekf.x[0] = yaw  # yaw 초기화
        self.initialized = True

    def predict(self, dt, angular_velocity):
        # 상태 예측 (동역학 모델)
        self.ekf.x[2] = angular_velocity  # yaw_dot 업데이트 (각 속도)
        self.ekf.x[0] += self.ekf.x[2] * dt  # yaw 업데이트

        # 예측 단계
        self.ekf.predict()

        self.ekf.x[0] = self.normalize_angle(self.ekf.x[0])

    # def normalize_angle(self, angle):
    # # 각도를 -pi에서 pi 범위로 정규화
    #     while angle > np.pi:
    #         angle -= 2 * np.pi
    #     while angle < -np.pi:
    #         angle += 2 * np.pi
    #     return angle

    def normalize_angle(self, angle):
        # 각도를 0에서 2*pi 범위로 정규화
        angle %= 2 * np.pi
        return angle

    def update(self, orientation):
        # 오리엔테이션으로 상태 업데이트
        yaw = np.arctan2(2 * (orientation.z * orientation.w + orientation.x * orientation.y),
                         1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
        
        yaw = np.rad2deg(yaw) % 360
        yaw_rad = np.deg2rad(yaw)

        # Jacobian 및 hx 계산
        self.ekf.update(np.array([yaw_rad]), self.HJacobian, self.hx)

    def HJacobian(self, x):
        # Jacobian 계산
        return np.array([[0, 0, 1]])

    def hx(self,  x):
        # 상태를 측정값으로 변환
        return np.array([x[0]])  # yaw만 반환

    def get_yaw(self):
        return self.ekf.x[0]  # yaw 값 반환

def imu_callback(msg, ekf):
    if not ekf.initialized:
        # 초기 yaw 설정
        ekf.initialize_yaw(msg.orientation)
    else:
    # IMU로부터 각 속도를 가져옴
        angular_velocity = msg.angular_velocity.z  # yaw에 대한 각 속도 (z 축)

        ekf.predict(0.01, angular_velocity)  # 예측 단계
        ekf.update(msg.orientation)  # 업데이트 단계
        corrected_yaw = np.rad2deg(ekf.get_yaw())

        # offset
        # offset = -30
        # corrected_yaw += offset
    
        rospy.loginfo(f'Corrected Yaw: {corrected_yaw} degrees')  # 보정된 yaw 출력
        yaw_pub.publish(Float64(corrected_yaw))

yaw_pub = rospy.Publisher('/corrected_yaw', Float64, queue_size=10)

def main():
    rospy.init_node('ekf_imu_node')
    ekf = EKF()
    rospy.Subscriber('/imu', Imu, imu_callback, ekf)

    rospy.spin()
    

if __name__ == '__main__':
    main()