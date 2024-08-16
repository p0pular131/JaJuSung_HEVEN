import statistics
import cv2
import rospy
import pcl
import struct 
import random
import numpy as np
import sensor_msgs.point_cloud2 as pc2 

from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import PointCloud2 

def generate_color(index):
    """ Generate a unique RGB color based on the index. """
    random.seed(index)
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

def plot_dots(coordinates, height, width):
    frame = np.zeros((600, 800, 3), dtype=np.uint8)
    for x, y in coordinates:
        cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
    cv2.circle(frame, (width//2, height//2), 5, (0,255,255), -1)

    return frame

def split_dots_by_origin(coordinates, origin):  # TODO: requires camera fusion
    origin_x, origin_y = origin
    left_dots_array = []
    right_dots_array = []
        
    for coordinate in coordinates:
        x, y = int(coordinate[0]), int(coordinate[1])
        coordinate_array = [x, y]
        if x <= origin_x and y <= origin_y:
            left_dots_array.append(coordinate_array)
        elif x > origin_x and y <= origin_y:
            right_dots_array.append(coordinate_array)

    return (np.array(left_dots_array), np.array(right_dots_array))

def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a PCL PointCloud """
    rospy.loginfo("Converting ROS PointCloud2 to PCL")
    points_list = []
    for data in pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z")):
        points_list.append([data[0], data[1], data[2], 0xFF0000])  # Default red color for visualization

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)
    rospy.loginfo("ROS to PCL conversion complete")
    return pcl_data


def pcl_to_ros(pcl_array, color=None):
    """Converts a PCL PointCloud into a ROS PointCloud2 message. Uses a default color if none is provided."""
    rospy.loginfo("Converting PCL to ROS PointCloud2")
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"
    ros_msg = PointCloud2()
    ros_msg.header = header
    ros_msg.height = 1
    ros_msg.width = len(pcl_array)
    ros_msg.is_dense = True
    ros_msg.is_bigendian = False
    ros_msg.fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1),
    ]
    ros_msg.point_step = 16
    ros_msg.row_step = ros_msg.point_step * ros_msg.width

    buffer = []
    default_color = (255, 255, 255)  # White color
    rgb = color if color is not None else default_color
    packed_rgb = struct.pack('BBBB', rgb[2], rgb[1], rgb[0], 255)
    rgb_int = struct.unpack('I', packed_rgb)[0]

    for point in pcl_array:
        buffer.append(struct.pack('fffI', point[0], point[1], point[2], rgb_int))

    ros_msg.data = b''.join(buffer)
    rospy.loginfo("PCL to ROS conversion complete")
    return ros_msg

def draw_polyfit_lane(frame, clustered_dots, dim=2):
    x_coords = clustered_dots[:, 0]
    y_coords = clustered_dots[:, 1]
    coeffecients = np.polyfit(x_coords, y_coords, dim)
    polynomial = np.poly1d(coeffecients)

    x_dense = np.linspace(min(x_coords), max(x_coords), 100).astype(int)
    y_dense = polynomial(x_dense).astype(int)

    previous_point = None
    for point in zip(x_dense, y_dense):
        if previous_point is not None:
            cv2.line(frame, previous_point, point, (255, 0, 0), 2)
        previous_point = point

def list_to_multiarray(list_value):
    msg = Float32MultiArray()
    msg.data = list_value
    return msg

def make_cv2(cluster_indices, cloud_filtered, pub, image_size=(800, 600)):
    WIDTH, HEIGHT = image_size
    ROAD_WIDTH = 80  # in pixels
    coordinates = [(WIDTH//2 - ROAD_WIDTH, HEIGHT//2),
                   (WIDTH//2 + ROAD_WIDTH, HEIGHT//2)]
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (WIDTH, HEIGHT))

    for idx, indices in enumerate(cluster_indices):
        points = [cloud_filtered[i] for i in indices]
        x_coords, y_coords, z_coords = zip(*[(point[0], point[1], point[2]) for point in points])
        color = generate_color(idx)
        cluster_msg = pcl_to_ros(points,color)
        pub.publish(cluster_msg)

        # print(statistics.median(x_coords), (statistics.median(y_coords)
        center_x, center_y = WIDTH // 2, HEIGHT // 2
        centroid_y = center_y - (statistics.median(x_coords)*40)
        centroid_x = center_x - (statistics.median(y_coords)*40)
        coordinates.append((centroid_x, centroid_y))
    
    frame = plot_dots(coordinates, HEIGHT, WIDTH)
    left_dots_array, right_dots_array = split_dots_by_origin(coordinates=coordinates,
                                                             origin=(WIDTH//2, HEIGHT//2))
    if left_dots_array.shape[0] > 1:
        draw_polyfit_lane(frame, left_dots_array)
    
    if right_dots_array.shape[0] > 1:
        draw_polyfit_lane(frame, right_dots_array)

    for i in range(0, WIDTH, 40):
        cv2.line(frame, (i, 0), (i, HEIGHT), (255, 255, 255), 1)
    for j in range(0, HEIGHT, 40):
        cv2.line(frame, (0, j), (WIDTH, j), (255, 255, 255), 1)
    cv2.imshow('Frame with Multiple Red Dots', frame)
    # out.write(frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

    cv2.waitKey(1)
    # out.release()

    return left_dots_array, right_dots_array