#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from scipy.interpolate import splprep, splev

class PathPlanner:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('path_planner', anonymous=True)
        
        # Delaunay midpoints 구독
        self.subscriber = rospy.Subscriber('/delaunay_midpoints', MarkerArray, self.callback)

        # 경로를 출판하기 위한 퍼블리셔
        self.publisher = rospy.Publisher('/planned_path_spline', Path, queue_size=10)

        self.goal_points = []  # 목표점 정보 저장
        self.spline_s = 100 # spline path 생성시, 목표점과 멀어져도 되는 거리(커지면 더 smooth해짐)
        self.spline_parm = 50 # 커지면 spline path 생성시, 더 smooth해짐
        '''
        ============================================================================================
        튜닝 parameter
        ============================================================================================
        '''
        self.start_x, self.start_y = -0.5, 0.0  # 경로생성 시작 위치(앞바퀴 위치)
        self.resolution = 5 # 1m의 path에 생성될 점의 개수


    def callback(self, msg):
        # Marker.DELETE(3) 메시지는 필터링
        valid_markers = [
            marker for marker in msg.markers if marker.action != 3
        ]

        # 유효한 마커들의 좌표를 추출
        self.goal_points = np.array([
            (marker.pose.position.x, marker.pose.position.y) for marker in valid_markers
        ])
        
        if not self.goal_points.any():
            rospy.loginfo("No goal points")
            return


        rospy.loginfo(f"Extracted Goal Points: {self.goal_points}")

        # 경로 계획 (스플라인 사용)
        path = self.plan_path(self.goal_points)

        # 경로를 퍼블리시
        self.publish_path(path)

    # 1m당 resolution만큼의 점이 들어가서 path 생성
    def generate_evenly_spaced_points(self, points):
        """
        Generates evenly spaced points between the given points 
        at the specified resolution (points per meter).
        """
        points = np.array(points)

        # 인접한 두 점간의 거리
        distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
        # path의 총 거리
        total_distance = np.sum(distances)

        # Calculate the total number of points required (self.resolution points per meter)
        num_points = int(total_distance * self.resolution)

        # Generate cumulative distances for interpolation
        cumulative_distances = np.insert(np.cumsum(distances), 0, 0) # insert 0 at index 0 (since starting point is included in path)

        # Create the target distance values for interpolation
        target_distances = np.linspace(0, total_distance, num_points) # resolution을 포함하여 각각의 점이 가지고 있어야할 거리값

        # Interpolate x and y coordinates along the cumulative distances
        x_interp = np.interp(target_distances, cumulative_distances, points[:, 0]) # np.interp(x, xp, fp)
        y_interp = np.interp(target_distances, cumulative_distances, points[:, 1])

        return np.vstack((x_interp, y_interp)).T


    def plan_path(self, midpoints):
        # Convert midpoints to a NumPy array for processing
        midpoints = np.array(midpoints)
        # print("Midpoints shape:", midpoints.shape)
        
        # Include the origin point
        if not np.any(np.all(midpoints == [self.start_x, self.start_y], axis=1)):
            midpoints = np.vstack(([self.start_x, self.start_y], midpoints))

        # Check for valid points and filter out NaN or infinite values
        midpoints = midpoints[np.isfinite(midpoints).all(axis=1)]

        # Calculate the Euclidean distance from (self.start_x, self.start_y) for each point
        distances_from_start = np.sqrt(
            (midpoints[:, 0] - self.start_x) ** 2 + (midpoints[:, 1] - self.start_y) ** 2
        )

        # Sort the midpoints based on the computed distances
        midpoints = midpoints[np.argsort(distances_from_start)]

        # Initialize planned_path as an empty array(only consider x and y axis)
        planned_path = np.empty((0, 2))  


        # Check the number of valid points and handle accordingly
        if len(midpoints) <= 3:
            # Simple fallback strategy: straight line with the same resolution
            planned_path = self.generate_evenly_spaced_points(midpoints)
        else:
            # Fit a B-spline to the midpoints
            try:
                tck, u = splprep(midpoints.T, s=self.spline_s)
                spline_points = np.array(splev(np.linspace(0, 1, self.spline_parm), tck)).T

                # Ensure same number of points per meter on the B-spline path
                planned_path = self.generate_evenly_spaced_points(spline_points)

            except ValueError as e:     
                # Fallback to the evenly spaced straight-line path
                planned_path = self.generate_evenly_spaced_points(midpoints)

        # Publish the planned path
        self.publish_path(planned_path)


    def publish_path(self, planned_path):
        if planned_path is None or planned_path.size == 0:
                # rospy.logwarn("No valid path to publish.")
                return
        
        # Path 메시지 초기화
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'velodyne'  # 적절한 프레임으로 설정

        for point in planned_path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'velodyne'  # 적절한 프레임으로 설정
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0  # z 좌표는 0으로 설정
            path_msg.poses.append(pose)

        self.publisher.publish(path_msg)
        rospy.loginfo("Publishing planned_path with topic /planned_path_spline!")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = PathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
