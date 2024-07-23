import numpy as np
import cv2

# LiDAR 포인트 클라우드에서 추출한 점들 (3D 좌표)
points_3d = np.array([
    [3.8, -0.32, 0.76],  # 오른쪽 하단
    [3.7, -0.32, 0.21],  # 오른쪽 상단
    [3.8, -0.33, 0.5],   # 오른쪽 중간
    [3.7, 0.51, 0.76],   # 왼쪽 하단
    [3.7, 0.54, 0.21],   # 왼쪽 상단
    [3.7, 0.54, 0.46]    # 왼쪽 중간
], dtype=np.float32)

# Camera 이미지에서 추출한 점들 (2D 좌표)
points_2d = np.array([

    [374, 166],  # 오른쪽 하단
    [376, 77],   # 오른쪽 상단
    [378, 124],  # 오른쪽 중간
    
    [233, 170],  # 왼쪽 하단
    [232, 72],   # 왼쪽 상단
    [232, 123]   # 왼쪽 중간
], dtype=np.float32)

# Camera의 내부 파라미터 행렬 (calibration matrix)
camera_matrix = np.array([
    [791.096231, 0.000000, 306.054697],
    [0.000000, 790.812314, 178.231744],
    [0.000000, 0.000000, 1.000000]
], dtype=np.float32)

# 왜곡 계수 (distortion coefficients)
dist_coeffs = np.array([0.172951, -0.553452, -0.009440, 0.001628, 0.000000], dtype=np.float32)

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
