#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <jajusung_main/HevenCtrlCmd.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>

class StanleyController {
public:
    StanleyController() {
        // ROS 노드 초기화
        ros::NodeHandle nh;
        path_sub_ = nh.subscribe("/midpoint_path", 1, &StanleyController::pathCallback, this);
        control_pub_ = nh.advertise<jajusung_main::HevenCtrlCmd>("/drive", 10);

    }

    void pathCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (cloud.empty()) {
            ROS_WARN("Received empty PointCloud2 data on /midpoint_path");
            return;
        }

        // 가장 가까운 점을 찾아 목표점으로 설정
        pcl::PointXYZ closest_point;
        double min_distance = std::numeric_limits<double>::max();
        for (const auto& point : cloud) {
            double distance = std::sqrt(point.x * point.x + point.y * point.y);
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = point;
            }
        }

        // Stanley 제어기를 통한 조향각 계산
        double delta = calculateSteeringAngle(closest_point);
        delta = delta * 180 / M_PI;  // rad2deg
        int delta_int = static_cast<int>(delta); // int casting

        if(delta_int > 35) delta_int = 35;
        else if(delta_int < -35) delta_int = -35;

        // 조향각을 포함한 제어 명령 퍼블리시
        jajusung_main::HevenCtrlCmd cmd_vel;
        cmd_vel.velocity = trc;
        cmd_vel.steering = delta_int;
        control_pub_.publish(cmd_vel);
    }

private:
    double calculateSteeringAngle(const pcl::PointXYZ& target_point) {
        // 측면 오차 계산
        double cross_track_error = target_point.y;

        // Stanley 제어기 조향각 계산
        double delta = atan2(k_ * cross_track_error, trc / 9 / 3.6) + atan2(target_point.y, target_point.x);

        return delta;
    }

    ros::Subscriber path_sub_;
    ros::Publisher control_pub_;

    double k_ = 0.1;  // Stanley 제어기 이득
    int trc = 500;  // 토크
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stanley_controller");
    StanleyController controller;
    ros::spin();
    return 0;
}
