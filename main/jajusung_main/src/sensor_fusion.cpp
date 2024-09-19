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
#include <torch/torch.h>  

class Fusion {
public:
    Fusion() {
        // ROS 노드 초기화
        ros::NodeHandle nh;
        image_sub_ = nh.subscribe("usb_cam/image_raw", 1, &Fusion::cameraCallback, this);
        lidar_sub_ = nh.subscribe("/livox_frame", 1, &Fusion::lidarCallback, this);
        cone_sub_ = nh.subscribe("cone_result", 1, &Fusion::coneCallback, this);
        roi_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/roi_pointcloud", 10);
        blue_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_blue", 10);
        yellow_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_yellow", 10);

        // 초기화
        blue_cone_points_.clear();
        yellow_cone_points_.clear();

        // 장치 설정
        device_ = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;

        torch::Tensor intrinsic_matrix = torch::tensor({
            {611.768234, 0.000000, 306.164069},
            {0.000000, 613.154786, 233.896019},
            {0.000000, 0.000000, 1.000000}
        }, torch::TensorOptions().device(device_));

        torch::Tensor extrinsic_matrix = torch::tensor({
            {0.06060704, -0.99802871, 0.01629351, -0.04498142},
            {0.05040392, -0.01324265, -0.99864112, 0.05205786},
            {0.99688827, 0.06134594, 0.04950196, 0.51905197}
        }, torch::TensorOptions().device(device_));
    }

    void cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image_ = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void coneCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat cone_seg_mat = cv_ptr->image;
            // Torch로 변환
            cone_seg_ = torch::from_blob(cone_seg_mat.data, {cone_seg_mat.rows, cone_seg_mat.cols, 3}, torch::kByte).to(device_);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
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
        pass.setFilterLimits(0.0, 12.0);
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
        // 3D 포인트를 Torch 텐서로 변환
        torch::Tensor points = torch::from_blob(cloud->points.data(), {static_cast<long>(cloud->points.size()), 3}, torch::kFloat).to(device_);
        torch::Tensor ones_tensor = torch::ones({points.size(0), 1}, torch::TensorOptions().device(device_));
        torch::Tensor world_points = torch::cat({points, ones_tensor}, 1);

        torch::Tensor projected_points = intrinsic_matrix.matmul(extrinsic_matrix).matmul(world_points.transpose(0, 1));
        torch::Tensor image_points = projected_points.slice(0, 0, 2).div(projected_points.slice(0, 2, 3));
        image_points = image_points.t().to(torch::kInt32);

        // 이미지 내의 유효한 좌표 필터링
        auto in_bounds = (image_points.slice(1, 0, 1) >= 0) & (image_points.slice(1, 0, 1) < 640) &
                         (image_points.slice(1, 1, 2) >= 0) & (image_points.slice(1, 1, 2) < 480);

        torch::Tensor valid_points = image_points.index(in_bounds);
        torch::Tensor valid_world_points = points.index(in_bounds);

        // 라바콘 색상 필터링
        auto blue_mask = cone_seg_.index({valid_points.slice(1, 1), valid_points.slice(1, 0), 0}) > 100;
        auto yellow_mask = cone_seg_.index({valid_points.slice(1, 1), valid_points.slice(1, 0), 1}) > 100;

        blue_cone_points_ = valid_world_points.index(blue_mask).cpu().numpy();
        yellow_cone_points_ = valid_world_points.index(yellow_mask).cpu().numpy();

        // 결과 표시
        for (int i = 0; i < valid_points.size(0); i++) {
            cv::Point pt(valid_points[i][0].item<int>(), valid_points[i][1].item<int>());
            if (blue_mask[i].item<bool>()) {
                cv::circle(image_, pt, 2, cv::Scalar(255, 0, 0), -1);  // Blue
            } else if (yellow_mask[i].item<bool>()) {
                cv::circle(image_, pt, 2, cv::Scalar(0, 0, 0), -1);   // Black
            }
        }

        cv::imshow("Result", image_);
        cv::waitKey(1);

        publishClouds();
    }

    void publishClouds() {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "livox_frame";

        if (!blue_cone_points_.empty()) {
            sensor_msgs::PointCloud2 blue_cloud;
            pcl::toROSMsg(blue_cone_points_, blue_cloud);
            blue_cloud.header = header;
            blue_pub_.publish(blue_cloud);
        }

        if (!yellow_cone_points_.empty()) {
            sensor_msgs::PointCloud2 yellow_cloud;
            pcl::toROSMsg(yellow_cone_points_, yellow_cloud);
            yellow_cloud.header = header;
            yellow_pub_.publish(yellow_cloud);
        }
    }

private:
    ros::Subscriber image_sub_, lidar_sub_, cone_sub_;
    ros::Publisher roi_pub_, blue_pub_, yellow_pub_;

    cv::Mat image_;
    torch::Tensor cone_seg_;
    torch::Device device_;
    std::vector<pcl::PointXYZ> blue_cone_points_, yellow_cone_points_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_node");
    Fusion fusion;
    ros::spin();
    return 0;
}
