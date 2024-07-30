import numpy as np
import cv2

# LiDAR 포인트 클라우드에서 추출한 점들 (3D 좌표)
points_3d = np.array([
    [3.7, 0.55, 0.76],  # 왼쪽 상단
    [3.7, -0.31, 0.76],  # 오른쪽 상단
    [3.7, 0.53, 0.2],  # 왼쪽 하단
    [3.7, -0.33, 0.19],   # 오른쪽 하단
], dtype=np.float32)

# Camera 이미지에서 추출한 점들 (2D 좌표)
points_2d = np.array([
    [226, 68],  # 왼쪽 상단
    [380, 70],  # 오른쪽 상단
    [227, 171],  # 왼쪽 하단
    [381, 172],   # 오른쪽 하단
], dtype=np.float32)

# Camera의 내부 파라미터 행렬 (calibration matrix)
camera_matrix = np.array([
    [611.768234, 0.000000, 306.164069],
    [0.000000, 613.154786, 233.896019],
    [0.000000, 0.000000, 1.000000]
], dtype=np.float32)

# 왜곡 계수 (distortion coefficients)
dist_coeffs = np.array([-0.007568, -0.058246, 0.000966, -0.006965, 0.000000], dtype=np.float32)

# 외부행렬 구하기
success, rotation_vector, translation_vector = cv2.solvePnP(points_3d, points_2d, camera_matrix, dist_coeffs)

if success:
    # 회전벡터를 회전행렬로 변환
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    extrinsic_matrix = np.hstack((rotation_matrix, translation_vector))

    print("Extrinsic Matrix: ")
    print(extrinsic_matrix)
else:
    print("Could not solve PnP problem.")