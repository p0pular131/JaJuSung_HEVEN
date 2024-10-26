#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
import lidar_pcl_helper as pcl_helper
from sensor_msgs.msg import PointCloud2
import numpy as np
import math
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import Delaunay

def do_passthrough(pcl_data, filter_axis, axis_min, axis_max):
    # 축 기반 필터링
    passthrough = pcl_data.make_passthrough_filter() #pc데이터를 축 기반 필터링을 위한 객체로 변환
    passthrough.set_filter_field_name(filter_axis) #필터링할 축 지정
    passthrough.set_filter_limits(axis_min, axis_max) #해당 축에서 필터링 범위 설정
    return passthrough.filter()

def do_voxel_grid_downsampling(pcl_data, leaf_size):
    # Voxel Grid Filter를 사용한 다운샘플링
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)  # 리프 크기가 클수록 정보가 적게 유지됨
    return vox.filter()

class PointCloudProcessor:
    def __init__(self):
        self.sub = rospy.Subscriber("/livox/lidar", PointCloud2, self.callback) # Raw 라이다 데이터 구독

        self.pub_clusters = rospy.Publisher("/clustered_centroids", PointCloud2, queue_size=1) # 라바콘 clusters 발행
        self.pub_triangles = rospy.Publisher("/delaunay_triangles", MarkerArray, queue_size=1) # delaunay 삼각형 발행
        self.pub_midpoints = rospy.Publisher("/delaunay_midpoints", MarkerArray, queue_size=1) # delaunay 삼각형으로 추출한 target points 발행 
        self.pub_path = rospy.Publisher("/midpoint_gradients", Float32MultiArray, queue_size=1)
        # marker namespace 정의
        self.marker_ns_tri = "delaunay"
        self.marker_ns_avg = "avg_points"

        '''
        =========================================================================
        튜닝 파라미터
        =========================================================================
        '''
        # ROI 범위 설정 
        self.x_min_roi, self.x_max_roi = 2.5, 20.0 # x축 최대 최소
        self.y_min_roi, self.y_max_roi = -4.0, 6.0 # y축 최대 최소
        self.z_min_roi, self.z_max_roi = -1.2, 0.0 # z축 최대 최소

        self.angle_constraints = 120 # 고려할 삼각형이 가질 수 있는 최대 각도\

        self.length_thre = 1.0

       
    def callback(self, msg):

        pcl_cloud = pcl_helper.ros_to_pcl(msg)
        do_voxel_grid_downsampling(pcl_cloud, 0.5)
        filter_axis = 'y'
        axis_min = self.y_min_roi
        axis_max = self.y_max_roi 
        cloud = do_passthrough(pcl_cloud, filter_axis, axis_min, axis_max)
        filter_axis = 'x'
        axis_min = self.x_min_roi 
        axis_max = self.x_max_roi
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
        filter_axis = 'z'
        axis_min = self.z_min_roi
        axis_max = self.z_max_roi
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        cloud = np.array(cloud)
            
        if len(cloud) == 0:
            rospy.logwarn("No points received")
            return

        # DBSCAN 클러스터링 수행
        db = DBSCAN(eps=0.8, min_samples=3).fit(cloud[:, :3])
        labels = db.labels_

        unique_labels = set(labels)

        # clusters들의 centroid 저장 리스트
        centroids_clusters = []

        # 각 클러스터의 중심점 계산
        for label in unique_labels:
            if label == -1:
                continue  # 노이즈는 건너뜀
            class_member_mask = (labels == label)
            cluster_points = cloud[class_member_mask]
            cluster_point = np.mean(cluster_points, axis=0)
            centroids_clusters.append([cluster_point[0], cluster_point[1], 0.0])  # z를 0으로 설정

        # 퍼블리시할 PointCloud2 메시지 생성
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id
        output_cloud = pc2.create_cloud_xyz32(header, centroids_clusters)

        # 결과 퍼블리시
        self.pub_clusters.publish(output_cloud)
        rospy.loginfo(f"Publishing {len(centroids_clusters)} clusters with topic /clustered_centroids")

        centroids_clusters = np.array(centroids_clusters)


        # Delaunay Triangulation 실행 (2D - x, y 좌표만 사용)
        if len(centroids_clusters) > 2:
            tri = Delaunay(centroids_clusters[:, :2])
            # Midpoints of internal edges x,y for문 돌면서 일정 가까운 점만 publish
            self.publish_midpoints(centroids_clusters, tri)
        '''
        ex) 
        for simplex in tri.simplices:
            # 삼각형의 정점
            p1 = centroids_clusters[simplex[0]]
            p2 = centroids_clusters[simplex[1]]
            p3 = centroids_clusters[simplex[2]]
        ex) 
        simplex: [1 0 7], simplex:  [4 6 7], simplex:  [6 1 7] -> 삼각형을 만드는 centroids_clusters들의 idx로 이루어진 리스트
        '''

    def calculate_angle(self, a, b, c):
        """Calculate angle at point A using cosine law."""
        ab = math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
        ac = math.sqrt((c[0] - a[0]) ** 2 + (c[1] - a[1]) ** 2)
        bc = math.sqrt((c[0] - b[0]) ** 2 + (c[1] - b[1]) ** 2)

        # Calculate angles using cosine law
        # Clipping because of collinear cases
        angle_a = math.acos(np.clip((ab**2 + ac**2 - bc**2) / (2 * ab * ac), -1.0, 1.0))
        angle_b = math.acos(np.clip((ac**2 + bc**2 - ab**2) / (2 * ac * ab), -1.0, 1.0))
        angle_c = math.acos(np.clip((ab**2 + bc**2 - ac**2) / (2 * ab * bc), -1.0, 1.0))

        return math.degrees(angle_a), math.degrees(angle_b), math.degrees(angle_c)

    # simplex를 이루는 삼각형의 무게중심을 구하는 함수
    def calculate_centroid(self, simplex, centroids_clusters):
        p1, p2, p3 = centroids_clusters[simplex[0]], centroids_clusters[simplex[1]], centroids_clusters[simplex[2]]
        centroid_x = (p1[0] + p2[0] + p3[0]) / 3
        centroid_y = (p1[1] + p2[1] + p3[1]) / 3
        return centroid_x, centroid_y

    # 원점으로부터의 유클리드 거리 계산 함수
    def distance_from_origin(self, point):
        return (point[0]**2 + point[1]**2)**0.5

    # 다른 삼각형과 한변만 접해있는 삼각형중에서 가장 멀리 떨어진 삼각형 하나를 제거하는 함수
    def remove_farthest_single_shared_triangle(self, simplices_with_index, centroids_clusters):
        # 삼각형의 개수가 홀수일 때만 처리
        if len(simplices_with_index) % 2 != 0:
            # 각각의 변과 해당 변이 등장하는 횟수를 저장할 딕셔너리
            edge_count = {}

            # 모든 삼각형의 변을 정렬 후, edges에 넣음
            for idx, simplex in simplices_with_index:
                edges = [
                    tuple(sorted([simplex[0], simplex[1]])),
                    tuple(sorted([simplex[1], simplex[2]])),
                    tuple(sorted([simplex[2], simplex[0]]))
                ]

                # 각 변이 몇 번 등장하는지 기록
                for edge in edges:
                    if edge in edge_count:
                        edge_count[edge] += 1
                    else:
                        edge_count[edge] = 1

            # 공유하는 변이 1개인 삼각형들을 모을 리스트
            single_shared_simplices = []
            for idx, simplex in simplices_with_index:
                edges = [
                    tuple(sorted([simplex[0], simplex[1]])),
                    tuple(sorted([simplex[1], simplex[2]])),
                    tuple(sorted([simplex[2], simplex[0]]))
                ]

                # 공유 변이 1개인 삼각형을 확인
                shared_edges = sum(1 for edge in edges if edge_count[edge] == 1)
                if shared_edges == 1:
                    single_shared_simplices.append((idx, simplex))  

            # 공유 변이 1개인 삼각형이 2개일 경우
            if len(single_shared_simplices) == 2:
                # 두 삼각형의 중심점을 계산하고 원점에서의 거리를 비교
                farthest_simplex_idx = None
                max_distance = -1
                for idx, simplex in single_shared_simplices:
                    # 삼각형의 중심점 (centroid) 계산
                    centroid_tri = self.calculate_centroid(simplex, centroids_clusters)

                    # 원점으로부터의 거리 계산
                    distance = self.distance_from_origin(centroid_tri)

                    # 가장 먼 삼각형을 선택
                    if distance > max_distance:
                        max_distance = distance
                        farthest_simplex_idx = (idx, simplex)  # 인덱스와 삼각형을 함께 저장

                # 원점에서 가장 먼 삼각형을 simplices 리스트에서 제거
                simplices_with_index = [(idx, simplex) for idx, simplex in simplices_with_index if idx != farthest_simplex_idx[0]]  # 인덱스로 삼각형 제거

        return simplices_with_index

    def publish_midpoints(self, centroids_clusters, tri):
        """Publish average midpoints for all pairs of triangles sharing two vertices."""

        marker_array = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = self.marker_ns_tri
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.pub_midpoints.publish(marker_array)
        # 
        path = Float32MultiArray()
        path_candi = []

        # simplex: 삼각형을 이루는 점의 centroids_cluster들의 idx들을 모은 리스트 
        simplices = []
        for i in range(len(tri.simplices)):
            simplex = tri.simplices[i]
            p1 = centroids_clusters[simplex[0]]
            p2 = centroids_clusters[simplex[1]]
            p3 = centroids_clusters[simplex[2]]

            # 각도 계산
            angle_a, angle_b, angle_c = self.calculate_angle(p1, p2, p3)

            # 모든 각이 일정각도 이하인 경우만 추가(path 외부에 생기는 삼각형 제외)
            if angle_a <= self.angle_constraints and angle_b <= self.angle_constraints and angle_c <= self.angle_constraints:
                simplices.append(simplex)

        # 거리 기준으로 삼각형을 나열, index추가
        simplices.sort(key=lambda simplex: self.distance_from_origin(self.calculate_centroid(simplex, centroids_clusters)))
        simplices_with_index = [(i, simplices[i]) for i in range(len(simplices))]

        # 홀수 개 삼각형일 경우 가장 외곽 삼각형 제외
        simplices_with_index = self.remove_farthest_single_shared_triangle(simplices_with_index, centroids_clusters)

        print("numbers of triangles: ", len(simplices_with_index))
        processed_pairs = set()  # 이미 처리된 삼각형 저장

        for i, (orig_idx1, simplex1) in enumerate(simplices_with_index):
            if orig_idx1 in processed_pairs:
                continue

            # 첫 번째 삼각형의 정점
            p1, p2, p3 = [centroids_clusters[idx] for idx in simplex1]
        
            for j in range(orig_idx1 + 1, len(simplices_with_index)):
                orig_idx2, simplex2 = simplices_with_index[j]
                if orig_idx2 in processed_pairs or orig_idx1 == orig_idx2:
                    continue

                # 두 번째 삼각형의 정점
                q1, q2, q3 = [centroids_clusters[idx] for idx in simplex2]

                # 공유 정점 찾기
                shared_points = set([tuple(p1), tuple(p2), tuple(p3)]) & set([tuple(q1), tuple(q2), tuple(q3)])
                if len(shared_points) == 2:  # 두 개 정점 공유 시 처리
                    shared_points = list(shared_points)

                    # 공유된 변의 중점 계산
                    midpoint = [
                        (shared_points[0][0] + shared_points[1][0]) / 2,  # avg_x
                        (shared_points[0][1] + shared_points[1][1]) / 2   # avg_y
                    ]

                    avg_x, avg_y = midpoint  # 결과 할당
                    # rospy.loginfo(f"Midpoint: ({avg_x}, {avg_y})")

                    # 처리된 삼각형 쌍 추가 (각각의 절대 인덱스를 추가)
                    processed_pairs.add(orig_idx1)
                    processed_pairs.add(orig_idx2)

                    # Create a marker for the average midpoint
                    marker = Marker()
                    marker.pose.orientation.x = 0
                    marker.pose.orientation.y = 0
                    marker.pose.orientation.z = 0
                    marker.pose.orientation.w = 1  # Identity quaternion
                    marker.header.frame_id = "livox_frame"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = self.marker_ns_avg
                    marker.id = len(marker_array.markers) + 1  # Unique ID
                    marker.type = Marker.SPHERE  # Use sphere for the average midpoint
                    marker.action = Marker.ADD
                    marker.scale.x = 0.20  # Sphere diameter
                    marker.scale.y = 0.20
                    marker.scale.z = 0.20
                    marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red for average midpoint
                    # Average midpoint position
                    marker.pose.position.x = avg_x
                    marker.pose.position.y = avg_y
                    marker.pose.position.z = 0  # Keep z as 0 for 2D triangulation

                    marker_array.markers.append(marker)

                    gradient = np.arctan2(avg_y, avg_x)
                    path_candi.append([avg_x, avg_y, gradient])
                    # rospy.loginfo("markers: ", marker_array.markers)

        min_dist_point = None
        min_dist = float('inf')

        # path_candi 안의 각 원소들에 대해 반복
        for point in path_candi:
            x, y, gradient = point
            dist = (x**2 + y**2)**0.5  # 원점으로부터의 거리 계산

            # 거리 threshold 이상이며 최소 거리 조건을 만족하는 경우 갱신
            if dist > self.length_thre and dist < min_dist:
                min_dist = dist
                min_dist_point = point

        path.data = min_dist_point   
        self.pub_path.publish(path)
        # Avg midpoint 마커 퍼블리시    
        self.pub_midpoints.publish(marker_array)
        rospy.loginfo(f"Published {len(marker_array.markers)-1} average midpoint markers with topic /delaunay_midpoints")


if __name__ == "__main__":
    rospy.init_node("pointcloud_processor")
    processor = PointCloudProcessor()
    rospy.spin()
