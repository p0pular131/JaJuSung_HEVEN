#include "gps_stanley_usingekf_h.h"
#include <iostream>
#include <utility>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <geometry_msgs/Vector3.h>
#include <jajusung_main/HevenCtrlCmd.h>

void GPS_STANLEY::init_dict()
/* INITIALIZE DICTIONARY!!! */
{
    // std::string assets_path = "/home/heven/erp_ws/src/heven_m_m/lane_ctrl/src/";

    std::vector<std::string> csv_file_path = {
        // "/home/heven/erp_ws/src/JaJuSung_HEVEN/main/jajusung_main/src/gps_test.csv"
        // "/home/pcy028x2/catkin_ws/src/JaJuSung_HEVEN/main/jajusung_main/src/gps_test_pcy.csv"
        // "/home/pcy028x2/catkin_ws/src/JaJuSung_HEVEN/main/jajusung_main/src/gps_test_1026.csv"
        "/home/heven/jajusung_ws/src/JaJuSung_HEVEN/main/jajusung_main/src/gps_test_1031.csv"
    };

    for (int i = 0; i < csv_file_path.size(); ++i) {
        std:: ifstream csvfile(csv_file_path[i]);

        if (!csvfile.is_open()) {
            std::cerr << "Failed to open file: " << csv_file_path[i] << std::endl;
            continue;
        }

        std::string line;
        while (std::getline(csvfile, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> row;

            while (std::getline(ss, token, ',')) {
                row.push_back(token);
            }

            if (row.size() >= 2) {
                double first = std::stod(row[0]);
                double second = std::stod(row[1]);
                // double third = std::stod(row[2]);
                // ref_yaw.push_back(third);
                // ref_yaw 는 imu 센서를 이용해서 구하고 있음
                TRACK_DICT.emplace_back(first, second);
            }
        }
        csvfile.close();
        std::cout<<"=======csv close !"<<std::endl;
    }
}

void GPS_STANLEY::ref_yaw_dict()
{
    for (size_t i = 0; i < TRACK_DICT.size() - 1; ++i) {
        double first1 = TRACK_DICT[i].first;
        double second1 = TRACK_DICT[i].second;
        double first2 = TRACK_DICT[i+1].first;
        double second2 = TRACK_DICT[i+1].second;

        double delta_lat = (first2 - first1) * METER_X_CONST;
        double delta_lon = (second2 - second1) * METER_Y_CONST;

        double global_yaw = fmod((atan2(delta_lon, delta_lat) * (180.0 / M_PI)) + 360.0, 360.0);
        ref_yaw.push_back(global_yaw);
    }
}

double GPS_STANLEY::gps_stanley()
// SHOULD RETURN DELTA USING STANLEY
{
    // changes p_curr gps latitude, altitude to meterscale
    std::pair<double,double> p_curr = {latitude * METER_X_CONST, longitude * METER_Y_CONST};

    double distance = pow((p_curr.first - METER_X_CONST * TRACK_DICT[TRACK_IDX].first), 2) + pow((p_curr.second - METER_Y_CONST * TRACK_DICT[TRACK_IDX].second), 2);
    car_angle = imu_yaw_modified;
    if (distance < DISTANCE_SQUARE) {
        TRACK_IDX += 1;
    }

    std::pair<double,double> p_targ = TRACK_DICT[TRACK_IDX];

    // curr_angle_error = GPS_STANLEY::find_angle_error(car_angle, p_curr, p_targ);
    curr_angle_error = GPS_STANLEY::find_angle_error(car_angle, ref_yaw[TRACK_IDX]);

    // HEADING ERROR
    final_steer_angle = GPS_STANLEY::_constraint(curr_angle_error);
    
    // LATERAL ERROR
    double targetdir_x = METER_Y_CONST * p_targ.second - p_curr.second;
    double targetdir_y = METER_X_CONST * p_targ.first - p_curr.first;
    
    distance = sqrt(distance);

    lateral_error = atan2(targetdir_y, targetdir_x);

    lateral_error = -sin(lateral_error)*distance;

    ROS_INFO("%f", lateral_error);

    double stanley_delta = rad2deg(deg2rad(final_steer_angle) + atan(LANE_K*lateral_error/((VEL/3.6))));


    jajusung_main::HevenCtrlCmd drive_cmd;
    drive_cmd.velocity = 300;
    drive_cmd.steering = (int)stanley_delta;
    // erp42_msgs::DriveCmd drive_cmd;
    // drive_cmd.KPH = VEL;
    // drive_cmd.Deg = (int)stanley_delta;
    // // if(drive_cmd.Deg > 25) drive_cmd.Deg = 25;
    // // else if(drive_cmd.Deg < -25) drive_cmd.Deg = -25;

    // erp42_msgs::ModeCmd mode_cmd;
    // mode_cmd.MorA = 0x01;
    // mode_cmd.EStop = 0x00;
    // mode_cmd.Gear = 0x00;



    std::cout<<"================lateral_error : "<<rad2deg(atan(LANE_K*lateral_error/((VEL/3.6))))<<'\n';
    std::cout<<"================heading_error : "<<final_steer_angle<<'\n';
    std::cout<<"================result_drive : "<<stanley_delta<<'\n';
    std::cout<<"================target_idx : "<<TRACK_IDX<<'\n';
    drive_pub.publish(drive_cmd);

    return stanley_delta;
}

double GPS_STANLEY::find_angle_error(double car_angle, double ref_angle)
{
    // targetdir_x could be positive or negative value
    // targetedir_y might be always positive 
    // double targetdir_x = METER_Y_CONST * position_targ.second - position_curr.second;
    // double targetdir_y = METER_X_CONST * position_targ.first - position_curr.first;

    double target_angle = ref_angle;
    // target_angle = - target_angle + 90.0;

    // double target_angle_modified = 0;

    // if (target_angle <= 0) {
    //     target_angle_modified = 180.0 + (180.0 + target_angle);
    // }
    // else { target_angle_modified = target_angle; }

    ROS_INFO("Target angle : %.1f", target_angle);
    ROS_INFO("Car angle : %.2f", car_angle);

    double angle_error = car_angle - target_angle;

    if (angle_error >= 180) {
        angle_error -= 360;
    }
    if (angle_error <= -180) {
        angle_error += 360;
    }

    ROS_INFO("Angle error : %.2f", angle_error);

    return angle_error;
}

double GPS_STANLEY::_constraint(double steer_angle)
// TO GIVE A SATURATION EFFECT ON STEERING ANGLE
{
    if (steer_angle >= 2000.0/71.0) {
        steer_angle = 2000.0/71.0;
    }
    else if (steer_angle <= -2000.0/71.0) {
        steer_angle = -2000.0/71.0;
    }

    return steer_angle;
}

void GPS_STANLEY::chatterCallback_1(const sensor_msgs::NavSatFix::ConstPtr& msg_1)
{
  latitude = msg_1->latitude;
  longitude = msg_1->longitude;
  gps_stanley();
}

// void GPS_STANLEY::chatterCallback_2(const std_msgs::Float64::ConstPtr& msg_2)
// {

//     imu_yaw = msg_2->data;

//     // ROS_INFO("Corrected Yaw: %f degrees", imu_yaw);

// }


void GPS_STANLEY::chatterCallback_2(const sensor_msgs::Imu::ConstPtr& msg_2)
{

    // 메시지 복사본을 생성하여 타임스탬프를 설정합니다.
    sensor_msgs::Imu imu_msg = *msg_2;  // 메시지 복사
    imu_msg.header.stamp = ros::Time::now();  // 현재 시간으로 타임스탬프 설정

    tf::Quaternion q(
        msg_2->orientation.x,
        msg_2->orientation.y,
        msg_2->orientation.z,
        msg_2->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, imu_yaw);

    // imu_yaw = fmod((- rad2deg(imu_yaw) + 90.0), 360.0);

    imu_yaw = rad2deg(imu_yaw);

    yaw_rate = msg_2->angular_velocity.z;

    // // Use `yaw_rate` and `delta_time` to update `global_yaw`
    // double delta_time = (msg_2->header.stamp - last_timestamp_).toSec(); // Calculate time delta
    // yaw_from_rate += yaw_rate * delta_time; // Integrate yaw rate
    // yaw_from_rate = fmod(yaw_from_rate + 360.0, 360.0); // Keep yaw in 0-360 range


    // if (!last_timestamp_.isZero()) {
    //     double delta_time = (imu_msg.header.stamp - last_timestamp_).toSec(); // 시간 차이 계산
    //     yaw_rate = rad2deg(imu_msg.angular_velocity.z);  // rad/s에서 deg/s로 변환
    //     yaw_from_rate += yaw_rate * delta_time;  // yaw rate 적분
    //     yaw_from_rate = fmod(yaw_from_rate + 360.0, 360.0); // 초기 yaw offset 추가 및 0-360 범위 유지
    // }

    // last_timestamp_ = imu_msg.header.stamp;  // 마지막 시간 업데이트

    // // last_timestamp_ = msg_2->header.stamp; // Update last timestamp

    if (imu_yaw <= 0) {
        imu_yaw_modified = 360 + imu_yaw;
    }
    else { imu_yaw_modified = imu_yaw; }

    // ROS_INFO("%f", imu_yaw_modified);

    // imu_yaw_rate = msg_2->angular_velocity.z;
}