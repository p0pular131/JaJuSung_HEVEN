#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>  
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>
#include <math.h>
#include <unordered_set>
#include <functional>

class Fusion {
public:
    double first_image_time = 0.0, first_scan_time = 0.0;
    double current_image_time = 0.0, current_scan_time = 0.0;
    double time_offset = 0.0;
    double lane_width = 6.0;
    Fusion() {
        // ROS 노드 초기화
        ros::NodeHandle nh;
        left_image_sub_ = nh.subscribe("/cameraLeft/usb_cam1/image_raw", 1, &Fusion::LeftcameraCallback, this);
        right_image_sub_ = nh.subscribe("/cameraRight/usb_cam2/image_raw", 1, &Fusion::RightcameraCallback, this);
        lidar_sub_ = nh.subscribe("/livox/lidar", 1, &Fusion::lidarCallback, this);
        cone_sub_ = nh.subscribe("/cone_result", 1, &Fusion::coneCallback, this);
        roi_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/roi_pointcloud", 10);
        blue_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_blue", 10);
        yellow_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_yellow", 10);
        path_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/midpoint_path", 10);

        blue_cone_points_.clear();
        yellow_cone_points_.clear();

        intrinsic_matrix_left = (cv::Mat_<double>(3, 3) << 675.759248, 0.000000, 319.4499363,
                                                      0.000000, 679.678618, 231.999081,
                                                      0.000000, 0.000000, 1.000000);

        intrinsic_matrix_right = (cv::Mat_<double>(3, 3) << 658.640475, 0.000000, 347.282338,
                                                      0.000000, 661.438151, 243.753564,
                                                      0.000000, 0.000000, 1.000000);

        extrinsic_matrix_left = (cv::Mat_<double>(3, 4) << 0.32409061, -0.94543155, -0.03353304, -0.02130857,
                                                        0.10725113,  0.0719368,  -0.99162608, -0.26748812,
                                                        0.93992685,  0.31778024,  0.12471264,  0.54786644);

        extrinsic_matrix_right = (cv::Mat_<double>(3, 4) << -0.38588106, -0.92234947, -0.01916385, -0.11465837,
                                                0.07988828, -0.01271354, -0.99672274,  0.03751879,
                                                0.91908306, -0.3861474,  0.07859082,  0.21283645);

    }

    void LeftcameraCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            left_image_ = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void RightcameraCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            right_image_ = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void coneCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            if(first_image_time == 0.0) {
                first_image_time = ros::Time::now().toSec();
            }
            current_image_time = ros::Time::now().toSec();

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cone_seg_ = cv_ptr->image;

            if (cone_seg_.cols == 1280 && cone_seg_.rows == 480) {
                cone_seg_left = cone_seg_(cv::Rect(0, 0, 640, 480));  // 왼쪽 절반
                cone_seg_right = cone_seg_(cv::Rect(640, 0, 640, 480));  // 오른쪽 절반
            } 
            else if (cone_seg_.cols == 1280 && cone_seg_.rows == 480) {
                cone_seg_left = cone_seg_(cv::Rect(0, 0, 640, 480));  // 왼쪽 절반
                cone_seg_right = cone_seg_(cv::Rect(640, 0, 640, 480));  // 오른쪽 절반
            } else {
                ROS_ERROR("Unexpected image size: %d x %d", cone_seg_.cols, cone_seg_.rows);
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        if(cone_seg_.empty() || left_image_.empty() || right_image_.empty()) {
            ROS_INFO("No Image. Pass this scan.");
            return;
        }
        if(first_scan_time == 0.0) {
            first_scan_time = msg->header.stamp.toSec();
        }
        time_offset = first_image_time - first_scan_time;

        current_scan_time = msg->header.stamp.toSec();

        double diff_time = fabs(current_image_time - current_scan_time - time_offset);

        // if( diff_time > 0.2) {
        //     ROS_INFO("Too diff time : %.6f. skip this scan.",diff_time);
        //     return;
        // }
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Voxel Grid Filter를 사용한 다운샘플링
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(pcl_cloud);
        vox.setLeafSize(0.1f, 0.1f, 0.1f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        vox.filter(*filtered_cloud);

        // ROI 필터링
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(filtered_cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(2.5, 13.0);
        pass.filter(*filtered_cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-6.0, 6.0);
        pass.filter(*filtered_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.1, 0.0);
        pass.filter(*filtered_cloud);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = "livox_frame";
        roi_pub_.publish(output);
        processPointCloud(filtered_cloud);
    }

    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        std::vector<cv::Point2i> left_image_points;
        std::vector<pcl::PointXYZ> left_valid_world_points;
        std::vector<cv::Point2i> right_image_points;
        std::vector<pcl::PointXYZ> right_valid_world_points;

        // 포인트 클라우드의 각 포인트를 카메라 좌표계로 변환
        for (const auto& point : cloud->points) {
            cv::Mat world_point = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1.0);
            cv::Mat left_camera_point = extrinsic_matrix_left * world_point;
            cv::Mat right_camera_point = extrinsic_matrix_right * world_point;

            // 이미지 평면으로 투영
            cv::Mat left_projected_point = intrinsic_matrix_left * left_camera_point.rowRange(0, 3);
            cv::Point2i left_img_point;
            left_img_point.x = static_cast<int>(left_projected_point.at<double>(0) / left_projected_point.at<double>(2));
            left_img_point.y = static_cast<int>(left_projected_point.at<double>(1) / left_projected_point.at<double>(2));

            // 이미지 좌표가 유효한 경우에만 추가
            if (left_img_point.x >= 0 && left_img_point.x < left_image_.cols && left_img_point.y >= 0 && left_img_point.y < left_image_.rows) {
                left_image_points.push_back(left_img_point);
                left_valid_world_points.push_back(point);
            }
            // 이미지 평면으로 투영
            cv::Mat right_projected_point = intrinsic_matrix_right * right_camera_point.rowRange(0, 3);
            cv::Point2i right_img_point;
            right_img_point.x = static_cast<int>(right_projected_point.at<double>(0) / right_projected_point.at<double>(2));
            right_img_point.y = static_cast<int>(right_projected_point.at<double>(1) / right_projected_point.at<double>(2));

            // 이미지 좌표가 유효한 경우에만 추가
            if (right_img_point.x >= 0 && right_img_point.x < right_image_.cols && right_img_point.y >= 0 && right_img_point.y < right_image_.rows) {
                right_image_points.push_back(right_img_point);
                right_valid_world_points.push_back(point);
            }

        }

        blue_cone_points_.clear();
        yellow_cone_points_.clear();

        // 왼쪽 이미지에서 라바콘 색상 필터링 -> 파란색 추출
        pcl::PointXYZ left_flattened;
        pcl::PointXYZ right_flattened;
        
        for (size_t i = 0; i < left_image_points.size(); ++i) {
            const cv::Point2i& pt = left_image_points[i];
            if (pt.x >= 0 && pt.x < cone_seg_left.cols && pt.y >= 0 && pt.y < cone_seg_left.rows) {
                cv::Vec3b color = cone_seg_left.at<cv::Vec3b>(pt); 
                if (pt.x < left_image_.cols && pt.y < left_image_.rows && !left_image_.empty())
                // cv::circle(left_image_, pt, 2, cv::Scalar(255, 0, 0), -1);
                if (color[0] > 100) {
                    left_flattened = {left_valid_world_points[i].x, left_valid_world_points[i].y, 0};
                    blue_cone_points_.push_back(left_flattened);
                    // cv::circle(cone_seg_left, pt, 2, cv::Scalar(255, 0, 0), -1);
                }
            } else {
                ROS_WARN("Invalid point access in cone_seg_left: (%d, %d)", pt.x, pt.y);
                ROS_WARN("Seg boundary: (%d, %d)", cone_seg_left.cols, cone_seg_left.rows);
            }
        }

        for (size_t i = 0; i < right_image_points.size(); ++i) {
            const cv::Point2i& pt = right_image_points[i];
            if (pt.x >= 0 && pt.x < cone_seg_right.cols && pt.y >= 0 && pt.y < cone_seg_right.rows) {
                cv::Vec3b color = cone_seg_right.at<cv::Vec3b>(pt); 
                if (pt.x < right_image_.cols && pt.y < right_image_.rows && !right_image_.empty())
                // cv::circle(right_image_, pt, 2, cv::Scalar(255, 0, 0), -1);
                if (color[1] > 100) {
                    right_flattened = {right_valid_world_points[i].x, right_valid_world_points[i].y, 0};
                    yellow_cone_points_.push_back(right_flattened);
                    // cv::circle(cone_seg_right, pt, 2, cv::Scalar(0, 255, 255), -1);
                }
            } else {
                ROS_WARN("Invalid point access in cone_seg_right: (%d, %d)", pt.x, pt.y);
                ROS_WARN("Seg boundary: (%d, %d)", cone_seg_right.cols, cone_seg_right.rows);
            }
        }

        // cv::imshow("Result Left", cone_seg_left);
        // cv::moveWindow("Result Left", 0, 0);  

        // cv::imshow("Result Right", cone_seg_right);
        // cv::moveWindow("Result Right", 650, 0);  

        // cv::imshow("Calib Result Left", left_image_);
        // cv::moveWindow("Calib Result Left", 0, 550);  

        // cv::imshow("Calib Result Right", right_image_);
        // cv::moveWindow("Calib Result Right", 650, 550);  

        // cv::waitKey(1);

        processMidPoint();
    }

/*
===================================================================================================
================================== Calculate Midpoints ============================================
===================================================================================================
*/

    void processMidPoint() {
        // 클러스터링 및 중점 계산 후 퍼블리시
        std::vector<pcl::PointXYZ> blue_centroids = clusterCones(blue_cone_points_, true);
        std::vector<pcl::PointXYZ> yellow_centroids = clusterCones(yellow_cone_points_, false);

        // 각 파란색 라바콘에 대해 가장 가까운 노란색 라바콘을 찾고 중점을 생성
        std::vector<pcl::PointXYZ> midpoints;

        for (const auto& blue : blue_centroids) {
            pcl::PointXYZ nearest_yellow;
            if (findNearestCone(blue, yellow_centroids, nearest_yellow, lane_width)) {
                pcl::PointXYZ midpoint = calculateMidPoint(blue, nearest_yellow);
                    midpoints.push_back(midpoint);
            } else {
                // 유효한 쌍이 없는 경우 오른쪽에 중점 생성
                pcl::PointXYZ adjacent_blue;
                if (findAdjacentCone(blue, blue_centroids, adjacent_blue)) {
                    pcl::PointXYZ midpoint = calculatePerpendicularPoint(blue, adjacent_blue, lane_width / 2.0, true);
                        midpoints.push_back(midpoint);
                }
            }
        }

        // 남은 노란색 라바콘에 대해 인접한 같은 색의 라바콘을 활용하여 왼쪽에 중점 생성
        for (const auto& yellow : yellow_centroids) {
            if (!isPaired(yellow, midpoints)) {
                pcl::PointXYZ adjacent_yellow;
                if (findAdjacentCone(yellow, yellow_centroids, adjacent_yellow)) {
                    pcl::PointXYZ midpoint = calculatePerpendicularPoint(yellow, adjacent_yellow, lane_width / 2.0, false);
                        midpoints.push_back(midpoint);
                }
            }
        }

        // 중점 결과 퍼블리시
        publishMidpoints(midpoints);
        publishClusters(blue_centroids, yellow_centroids);
    }

    std::vector<pcl::PointXYZ> clusterCones(const std::vector<pcl::PointXYZ>& points, bool is_blue) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.reserve(points.size());  // 필요한 크기만큼 미리 할당
        std::copy(points.begin(), points.end(), std::back_inserter(cloud->points)); // copy 사용

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(3);
        ec.setMaxClusterSize(100);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        std::vector<pcl::PointXYZ> centroids;
        for (const auto& indices : cluster_indices) {
            pcl::PointXYZ centroid;
            for (const auto& idx : indices.indices) {
                centroid.x += cloud->points[idx].x;
                centroid.y += cloud->points[idx].y;
                centroid.z += cloud->points[idx].z;
            }
            centroid.x /= indices.indices.size();
            centroid.y /= indices.indices.size();
            centroid.z /= indices.indices.size();
            if(is_blue == false) { // yellow 
                if(centroid.y > 1.0) {
                    continue;
                }
            }
            else if(is_blue == true) { // blue
                if(centroid.y < -1.0) {
                    continue;
                }
            }
            centroids.push_back(centroid);
        }
        return centroids;
    }

    bool findNearestCone(const pcl::PointXYZ& cone, const std::vector<pcl::PointXYZ>& targets, pcl::PointXYZ& nearest, double max_distance) {
        double min_distance = max_distance;
        bool found = false;
        for (const auto& target : targets) {
            double distance = calculateDistance(cone, target);
            if (distance < min_distance) {
                min_distance = distance;
                nearest = target;
                found = true;
            }
        }
        return found;
    }

    bool findAdjacentCone(const pcl::PointXYZ& ref_point, const std::vector<pcl::PointXYZ>& centroids, pcl::PointXYZ& adjacent_point) {
        double min_distance = std::numeric_limits<double>::max();
        bool found = false;

        for (const auto& point : centroids) {
            if (point.x == ref_point.x && point.y == ref_point.y && point.z == ref_point.z) {
                continue; // 동일한 점은 제외
            }

            double distance = std::sqrt(std::pow(point.x - ref_point.x, 2) + std::pow(point.y - ref_point.y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                adjacent_point = point;
                found = true;
            }
        }

        return found;
    }


    pcl::PointXYZ calculateMidPoint(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
        pcl::PointXYZ mid;
        mid.x = (p1.x + p2.x) / 2;
        mid.y = (p1.y + p2.y) / 2;
        mid.z = (p1.z + p2.z) / 2;
        return mid;
    }

    pcl::PointXYZ calculatePerpendicularPoint(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, double distance, bool isBlueCone) {
        // 항상 p1.x < p2.x가 되도록 두 점을 정렬하여 일관된 기울기 계산
        const pcl::PointXYZ& point_a = (p1.x > p2.x) ? p1 : p2;
        const pcl::PointXYZ& point_b = (p1.x > p2.x) ? p2 : p1;

        double angle = atan2(point_b.y - point_a.y, point_b.x - point_a.x) + (isBlueCone ? M_PI / 2 : -M_PI / 2);
        pcl::PointXYZ perpendicular_point;
        perpendicular_point.x = point_a.x + distance * cos(angle);
        perpendicular_point.y = point_a.y + distance * sin(angle);
        perpendicular_point.z = 0;

        return perpendicular_point;
    }

    double calculateDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
        return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    }

    bool isPaired(const pcl::PointXYZ& cone, const std::vector<pcl::PointXYZ>& midpoints) {
        for (const auto& midpoint : midpoints) {
            if (calculateDistance(cone, midpoint) < 1.0) {
                return true;
            }
        }
        return false;
    }

    struct PointHash {
        std::size_t operator()(const pcl::PointXYZ& point) const {
            return std::hash<float>()(point.x) ^ std::hash<float>()(point.y) ^ std::hash<float>()(point.z);
        }
    };

    void publishMidpoints(const std::vector<pcl::PointXYZ>& midpoints) {
        pcl::PointCloud<pcl::PointXYZ> midpoint_cloud;
        midpoint_cloud.points.reserve(midpoints.size()); // 공간 예약
        std::copy(midpoints.begin(), midpoints.end(), std::back_inserter(midpoint_cloud.points)); // copy 사용

        sensor_msgs::PointCloud2 midpoint_msg;
        pcl::toROSMsg(midpoint_cloud, midpoint_msg);
        midpoint_msg.header.frame_id = "livox_frame";
        midpoint_msg.header.stamp = ros::Time::now();
        path_pub_.publish(midpoint_msg);
    }

    void publishClusters(const std::vector<pcl::PointXYZ>& blue_centroids, const std::vector<pcl::PointXYZ>& yellow_centroids) {
        pcl::PointCloud<pcl::PointXYZ> blue_cluster_cloud;
        pcl::PointCloud<pcl::PointXYZ> yellow_cluster_cloud;

        // blue_centroids의 각 포인트를 blue_cluster_cloud.points에 추가
        blue_cluster_cloud.points.reserve(blue_centroids.size());
        for (const auto& point : blue_centroids) {
            blue_cluster_cloud.points.push_back(point);
        }

        // yellow_centroids의 각 포인트를 yellow_cluster_cloud.points에 추가
        yellow_cluster_cloud.points.reserve(yellow_centroids.size());
        for (const auto& point : yellow_centroids) {
            yellow_cluster_cloud.points.push_back(point);
        } 

        sensor_msgs::PointCloud2 blue_cluster_msg;
        sensor_msgs::PointCloud2 yellow_cluster_msg;

        pcl::toROSMsg(blue_cluster_cloud, blue_cluster_msg);
        pcl::toROSMsg(yellow_cluster_cloud, yellow_cluster_msg);

        blue_cluster_msg.header.frame_id = "livox_frame";
        yellow_cluster_msg.header.frame_id = "livox_frame";

        blue_cluster_msg.header.stamp = ros::Time::now();
        yellow_cluster_msg.header.stamp = ros::Time::now();

        blue_pub_.publish(blue_cluster_msg);
        yellow_pub_.publish(yellow_cluster_msg);
    }



private:
    ros::Subscriber left_image_sub_, right_image_sub_, lidar_sub_, cone_sub_;
    ros::Publisher roi_pub_, blue_pub_, yellow_pub_, path_pub_;

    cv::Mat left_image_, right_image_, cone_seg_, cone_seg_left, cone_seg_right;
    cv::Mat intrinsic_matrix_left, intrinsic_matrix_right, extrinsic_matrix_left, extrinsic_matrix_right;
    std::vector<pcl::PointXYZ> blue_cone_points_, yellow_cone_points_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_node");
    Fusion fusion;
    ros::spin();
    return 0;
}
