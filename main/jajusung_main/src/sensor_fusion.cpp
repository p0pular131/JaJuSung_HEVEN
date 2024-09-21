#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>

class Fusion {
public:
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

        blue_cone_points_.clear();
        yellow_cone_points_.clear();

        intrinsic_matrix_ = (cv::Mat_<double>(3, 3) << 611.768234, 0.000000, 306.164069,
                                                      0.000000, 613.154786, 233.896019,
                                                      0.000000, 0.000000, 1.000000);

        extrinsic_matrix_left = (cv::Mat_<double>(3, 4) << 0.06060704, -0.99802871, 0.01629351, -0.04498142,
                                                       0.05040392, -0.01324265, -0.99864112, 0.05205786,
                                                       0.99688827, 0.06134594, 0.04950196, 0.51905197);

        extrinsic_matrix_right = (cv::Mat_<double>(3, 4) << 0.06060704, -0.99802871, 0.01629351, -0.04498142,
                                                0.05040392, -0.01324265, -0.99864112, 0.05205786,
                                                0.99688827, 0.06134594, 0.04950196, 0.51905197);

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
        if(cone_seg_.rows == 0) {
            ROS_INFO("No Image. Pass this scan.");
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Voxel Grid Filter를 사용한 다운샘플링
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(pcl_cloud);
        vox.setLeafSize(0.5f, 0.5f, 0.5f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        vox.filter(*filtered_cloud);

        // ROI 필터링
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(filtered_cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0, 4.0);
        pass.filter(*filtered_cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-3.0, 3.0);
        pass.filter(*filtered_cloud);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = "velodyne";
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
            cv::Mat left_projected_point = intrinsic_matrix_ * left_camera_point.rowRange(0, 3);
            cv::Point2i left_img_point;
            left_img_point.x = static_cast<int>(left_projected_point.at<double>(0) / left_projected_point.at<double>(2));
            left_img_point.y = static_cast<int>(left_projected_point.at<double>(1) / left_projected_point.at<double>(2));

            // 이미지 좌표가 유효한 경우에만 추가
            if (left_img_point.x >= 0 && left_img_point.x < left_image_.cols && left_img_point.y >= 0 && left_img_point.y < left_image_.rows) {
                left_image_points.push_back(left_img_point);
                left_valid_world_points.push_back(point);
            }
            // 이미지 평면으로 투영
            cv::Mat right_projected_point = intrinsic_matrix_ * right_camera_point.rowRange(0, 3);
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
        for (size_t i = 0; i < left_image_points.size(); ++i) {
            const cv::Point2i& pt = left_image_points[i];
            if (pt.x >= 0 && pt.x < cone_seg_left.cols && pt.y >= 0 && pt.y < cone_seg_left.rows) {
                cv::Vec3b color = cone_seg_left.at<cv::Vec3b>(pt); 
                if (color[0] > 100) {  
                    blue_cone_points_.push_back(left_valid_world_points[i]);
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
                if (color[1] > 100) {  
                    yellow_cone_points_.push_back(right_valid_world_points[i]);
                    // cv::circle(cone_seg_right, pt, 2, cv::Scalar(0, 255, 255), -1);
                }
            } else {
                ROS_WARN("Invalid point access in cone_seg_right: (%d, %d)", pt.x, pt.y);
                ROS_WARN("Seg boundary: (%d, %d)", cone_seg_right.cols, cone_seg_right.rows);
            }
        }

        cv::imshow("Result Left", cone_seg_left);
        cv::imshow("Result Right", cone_seg_right);
        cv::waitKey(1);

        publishClouds();
    }

    void publishClouds() {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "livox_frame";

        if (!blue_cone_points_.empty()) {
            pcl::PointCloud<pcl::PointXYZ> blue_cloud_pcl;
            blue_cloud_pcl.points.reserve(blue_cone_points_.size());  // 크기 설정
            blue_cloud_pcl.points.insert(blue_cloud_pcl.points.end(), blue_cone_points_.begin(), blue_cone_points_.end());  // 데이터를 복사
            sensor_msgs::PointCloud2 blue_cloud;
            pcl::toROSMsg(blue_cloud_pcl, blue_cloud);  // 변환된 pcl::PointCloud를 사용
            blue_cloud.header = header;
            blue_pub_.publish(blue_cloud);
        }

        if (!yellow_cone_points_.empty()) {
            pcl::PointCloud<pcl::PointXYZ> yellow_cloud_pcl;
            yellow_cloud_pcl.points.reserve(yellow_cone_points_.size());  // 크기 설정
            yellow_cloud_pcl.points.insert(yellow_cloud_pcl.points.end(), yellow_cone_points_.begin(), yellow_cone_points_.end());  // 데이터를 복사
            sensor_msgs::PointCloud2 yellow_cloud;
            pcl::toROSMsg(yellow_cloud_pcl, yellow_cloud);  // 변환된 pcl::PointCloud를 사용
            yellow_cloud.header = header;
            yellow_pub_.publish(yellow_cloud);
        }
    }


private:
    ros::Subscriber left_image_sub_, right_image_sub_, lidar_sub_, cone_sub_;
    ros::Publisher roi_pub_, blue_pub_, yellow_pub_;

    cv::Mat left_image_, right_image_, cone_seg_, cone_seg_left, cone_seg_right;
    cv::Mat intrinsic_matrix_, extrinsic_matrix_left, extrinsic_matrix_right;
    std::vector<pcl::PointXYZ> blue_cone_points_, yellow_cone_points_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_node");
    Fusion fusion;
    ros::spin();
    return 0;
}
