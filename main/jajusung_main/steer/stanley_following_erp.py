import rospy
import numpy as np
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from erp42_msgs.msg import DriveCmd, ModeCmd
import math


class StanleyController:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('stanley_controller', anonymous=True)
        
        # 경로 구독
        rospy.Subscriber("/planned_path_spline", Path, self.path_callback)
        
        # 속도와 조향각, 목표점 퍼블리셔
        self.cmd_pub = rospy.Publisher('/drive', DriveCmd, queue_size=10)
        self.mode_pub = rospy.Publisher('/mode', ModeCmd, queue_size=10)
        self.nearest_pub = rospy.Publisher("/target_point_stanley", Marker, queue_size=10)


        # Stanley 제어 관련 변수
        self.path = []
        self.current_index = 0
        self.max_steer = np.radians(28)  # 최대 조향각 제한 (라디안)
        self.distance_to_goal = 0 # 속도 p 제어에 쓰일 기준
        self.current_speed = 0.0  # 초기 속도
        self.new_speed = 0.0 # P 제어로 나온 target 속도
        self.start_x, self.start_y, self.start_yaw = -0.7, 0, 0  # 초기 위치 (앞바퀴 기준)
        
        '''
        ============================================================================================
        Steering 튜닝 parameter
        ============================================================================================
        '''
        self.goal_tolerance = 0.3  # 목표 점 도달 거리 (m)
        self.k = 0.5  # dist_error에 곱해지는 조향각 parameter

        '''
        ============================================================================================
        Speed 튜닝 parameter
        ============================================================================================
        '''
        self.speed_gain = 0.7 # (목표속도 - 현재속도)에 곱해지는 파라미터
        self.target_speed = 4 # 속도 p제어에 쓰이는 목표(최대)속도 (m/s) (1.4 ~ 6.9 m/s) (*3.6 = km/h) 
        self.speed_dec_rate = 0.7 # 조향이 일정 각도 이상일때, 감속을 위해 속도에 곱해주는 파라미터
        self.steer_slowdown_threshold = np.radians(10)  # 감속을 시작할 조향각 기준 

    def path_callback(self, msg):
        # 경로 데이터를 업데이트
        self.path = [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses]
        self.current_index = 0  # 새 경로 시작 시 인덱스 초기화

    def publish_nearest_marker(self, point):
        # Publishes a marker for the nearest point
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1  # Identity quaternion
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        self.nearest_pub.publish(marker)

    def compute_steering(self, current_x, current_y, current_yaw):
        # Stanley 제어로 steering 계산
        if not self.path:
            rospy.logwarn("경로가 비어 있습니다.")
            return 0.0

        # 1. 목표 점까지의 거리 계산
        nearest_point = self.path[self.current_index]
        self.distance_to_goal = math.hypot(nearest_point[0] - current_x, nearest_point[1] - current_y)

        # 2. 목표 점 근처에 도달하면 다음 점으로 이동
        if self.distance_to_goal < self.goal_tolerance and self.current_index < len(self.path) - 1:
            self.current_index += 1
            rospy.loginfo(f"Reached point {self.current_index}, moving to the next point.")

        # 3. 크로스 트랙 오차 계산
        nearest_point = self.path[self.current_index]
        error_front_axle = self.distance_to_goal  # 전방 축의 오차
        self.publish_nearest_marker(nearest_point)

        # 4. 헤딩 오차 계산 (경로의 목표 방향과 현재 방향 차이)
        target_yaw = math.atan2(nearest_point[1] - current_y, nearest_point[0] - current_x)
        yaw_error = target_yaw - current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # -pi ~ pi 범위로 조정

        # 5. Stanley 제어 입력 계산
        if self.current_speed == 0.0:  
            self.current_speed = 0.1
        cross_track_term = math.atan2(self.k * error_front_axle, self.current_speed)
        steer_angle = yaw_error + cross_track_term
        steer_angle = max(-self.max_steer, min(self.max_steer, steer_angle))  # 조향각 제한

        return steer_angle

    def compute_speed(self, current_speed, steer_angle):
        # P 제어를 이용한 속도 제어 및 조향각에 따른 감속
        # 1. 목표점과의 오차를 계산
        speed_error = self.target_speed - current_speed

        # 2. P 제어 기반 속도 계산
        new_speed = current_speed + self.speed_gain * speed_error

        # 3. 조향각에 따라 점진적으로 감속
        steer_angle_abs = abs(steer_angle)  # 조향각의 절댓값

        # 감속 비율 계산 (0에서 target_speed까지)
        if steer_angle_abs > self.steer_slowdown_threshold:
            # 조향각에 비례한 감속 비율을 결정 (최대 속도 제한을 감안)
            slowdown_factor = max(0.0, (self.max_steer - steer_angle_abs) / self.max_steer)
            new_speed *= (1.0 - (1.0 - self.speed_dec_rate) * (1.0 - slowdown_factor))

        # 4. 최대 속도 제한
        new_speed = min(new_speed, self.target_speed)

        return new_speed

    def run(self):
        # 주 제어 루프
        rate = rospy.Rate(10)  # 10Hz 주기로 실행

        while not rospy.is_shutdown():
            # 차량의 현재 위치와 방향
            current_x, current_y, current_yaw = self.start_x, self.start_y, self.start_yaw  

            # 제어 계산
            steer_angle = self.compute_steering(current_x, current_y, current_yaw)
            self.new_speed = self.compute_speed(self.current_speed, steer_angle)
            self.current_speed = self.new_speed

            # 속도 및 조향각 메시지 생성
            drive_cmd = DriveCmd()
            drive_cmd.KPH = int(self.new_speed*3.6)
            drive_cmd.Deg = int(math.degrees(steer_angle))

            # 컨트롤러 메시지 생성
            mode_cmd = ModeCmd()
            mode_cmd.MorA = 0x01
            mode_cmd.EStop = 0x00
            mode_cmd.Gear = 0x00

            # 퍼블리시
            self.cmd_pub.publish(drive_cmd)
            self.mode_pub.publish(mode_cmd)

            rospy.loginfo(f"Speed: {self.current_speed:.2f} m/s, Steering Angle: {math.degrees(steer_angle):.2f} deg")

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = StanleyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
