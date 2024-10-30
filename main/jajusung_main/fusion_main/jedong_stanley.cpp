#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <jajusung_main/HevenCtrlCmd.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <deque>

class StanleyController {
public:
    StanleyController() {
        // ROS 노드 초기화
        ros::NodeHandle nh;
        path_sub_ = nh.subscribe("/midpoint_path", 1, &StanleyController::pathCallback, this);
        control_pub_ = nh.advertise<jajusung_main::HevenCtrlCmd>("/drive_stanley", 10);

        // 가중치 초기화 (최근 값에 더 높은 가중치를 주도록 설정)
        weights_ = {1, 2, 3, 4, 5};
        normalizeWeights();
    }

    void pathCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (cloud.empty()) {
            ROS_WARN("Received empty PointCloud2 data on /midpoint_path");
            jajusung_main::HevenCtrlCmd cmd_vel;
            cmd_vel.velocity = trc;
            cmd_vel.steering = delta_prev * 1.1;
            updateMovingAverage(delta_prev * 1.1);
            control_pub_.publish(cmd_vel);
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

        // 가중 이동 평균 필터를 적용하기 위해 delta 값을 추가
        updateMovingAverage(delta);

        // 필터링된 delta 값을 사용하여 제어 명령 결정
        delta = getFilteredSteering();
        if (delta > 35.0) delta = 35.0;
        else if (delta < -35.0) delta = -35.0;

        // ROS_INFO("Filtered delta: %d", delta);

        delta_prev = delta;

        // 조향각을 포함한 제어 명령 퍼블리시
        jajusung_main::HevenCtrlCmd cmd_vel;
        cmd_vel.velocity = trc;
        cmd_vel.steering = delta;
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

    void updateMovingAverage(double new_delta) {
        // delta 값을 deque에 추가
        delta_history_.push_back(new_delta);
        if (delta_history_.size() > weights_.size()) {
            delta_history_.pop_front();  // 가장 오래된 값 제거
        }
    }

    double getFilteredSteering() const {
        // 가중 이동 평균 계산
        double weighted_sum = 0.0;
        double weight_sum = 0.0;
        int weight_idx = 0;

        for (int i = delta_history_.size() - weights_.size(); i < delta_history_.size(); ++i) {
            if (i >= 0) {  // 유효한 인덱스만 사용
                weighted_sum += delta_history_[i] * weights_[weight_idx];
                weight_sum += weights_[weight_idx];
                weight_idx++;
            }
        }

        return weight_sum > 0 ? weighted_sum / weight_sum : 0.0;
    }

    void normalizeWeights() {
        // 가중치의 합이 1이 되도록 정규화
        double sum = 0.0;
        for (double w : weights_) {
            sum += w;
        }
        for (double& w : weights_) {
            w /= sum;
        }
    }

    ros::Subscriber path_sub_;
    ros::Publisher control_pub_;

    double k_ = 0.05;  // Stanley 제어기 이득
    int trc = 400;  // 토크
    double delta_prev = 0.0;

    // 가중 이동 평균 필터 변수
    std::deque<double> delta_history_;
    std::vector<double> weights_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stanley_controller");
    StanleyController controller;
    ros::spin();
    return 0;
}
