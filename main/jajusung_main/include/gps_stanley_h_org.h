#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <map>
#include <Eigen/Dense>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
// #include <erp42_msgs/DriveCmd.h>
// #include <erp42_msgs/ModeCmd.h>
#include <jajusung_main/HevenCtrlCmd.h>

#include "ros/ros.h"

using namespace std;
using namespace Eigen;

class EKF_YawCorrector {
    public:
    EKF_YawCorrector() {
        // 초기 상태 벡터 (yaw, yaw_rate)
        x << 0, 0;
        
        // 초기 공분산 행렬
        P << 1, 0,
             0, 1;
        
        // 상태 전이 행렬
        F << 1, delta_t,
             0, 1;
        
        // 관측 행렬
        H << 1, 0;
        
        // 프로세스 잡음 행렬
        Q << 0.1, 0,
             0, 0.1;
        
        // 측정 잡음 행렬
        R << 0.8;
    }

    // 예측 단계
    void predict(double measured_yaw_rate) {
        x[1] = measured_yaw_rate; // yaw_rate 값을 IMU 측정치로 업데이트
        x = F * x; // 예측 방정식 적용
        P = F * P * F.transpose() + Q;
    }

    // 측정 업데이트 단계
    void update(double measured_yaw) {
        VectorXd z(1); // 측정값 벡터
        z << measured_yaw;

        VectorXd y = z - (H * x); // 잔차
        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse(); // 칼만 이득 계산

        x = x + (K * y);       // 상태 벡터 업데이트
        P = (MatrixXd::Identity(2, 2) - K * H) * P; // 공분산 행렬 업데이트
    }

    // 결과 출력
    double getCorrectedYaw() const {
        return x(0);
    }

    double getYawRate() const {
        return x(1);
    }

private:
    VectorXd x = VectorXd(2); // 상태 벡터
    MatrixXd P = MatrixXd(2, 2); // 공분산 행렬
    MatrixXd F = MatrixXd(2, 2); // 상태 전이 행렬
    MatrixXd H = MatrixXd(1, 2); // 관측 행렬
    MatrixXd Q = MatrixXd(2, 2); // 프로세스 잡음 행렬
    MatrixXd R = MatrixXd(1, 1); // 측정 잡음 행렬
    double delta_t = 1/100; // 시간 간격 
};

class GPS_STANLEY
{
public:
    GPS_STANLEY()
    {
        sub_1 = nh.subscribe<sensor_msgs::NavSatFix>("/ublox_gps/fix", 10, &GPS_STANLEY::chatterCallback_1, this);
        sub_2 = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &GPS_STANLEY::chatterCallback_2, this);

        drive_pub = nh.advertise<jajusung_main::HevenCtrlCmd>("/drive", 10);
        
        init_dict();
        ref_yaw_dict();
    }
    
    double gps_stanley();
    
    void init_dict();
    void ref_yaw_dict();

    // double calculate_targ_angle();

    // double calculate_gps(const std::pair<double, double>& prev, const std::pair<double, double>& curr);

    double find_angle_error(double car_angle, double ref_angle);

    double _constraint(double steer_angle);

    double rad2deg(double value)
    {
        return value * (180.0/M_PI);
    }
    double deg2rad(double value) 
    {
        return value * (M_PI/180.0);
    }
    void chatterCallback_1(const sensor_msgs::NavSatFix::ConstPtr& msg_1);
    void chatterCallback_2(const sensor_msgs::Imu::ConstPtr& msg_2);


private:
    ros::NodeHandle nh; 
    ros::Publisher drive_pub;
    // ros::Publisher mode_pub;
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    
    double LANE_K = 0.05;
    double VEL = 5;

    double METER_X_CONST = 110000;
    double METER_Y_CONST = 88800;
    int GPS_TRACKING_SPEED = 7;
    int TRACK_IDX = 0;

    double DISTANCE_SQUARE = 4;
    std::vector<std::pair<double, double>> TRACK_DICT;
    std::vector<double> ref_yaw;
    // double ref_yaw;
    int TRACK_NO = 0; // initialized as 0, used to find TRACK_DICT key 
    
    bool GPS_TRACK_END = false;

    double curr_angle_error = 0;
    double final_steer_angle = 0;
    double lateral_error = 0;

    double latitude;
    double longitude;
    double car_angle = 0.0;

    double enu_imu_yaw = 0;
    double imu_yaw = 0;
    double yaw_rate;
    double imu_yaw_modified = 0;

    // std::pair<double,double> prev_position;
    // double prev_car_angle;
    // double prev_delta_x, prev_delta_y;

    double prev_latitude = 0.0;
    double prev_longitude = 0.0;
    bool is_first_point = true;
};