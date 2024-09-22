#include "gps_stanley_h.h"
#include <iostream>
#include <utility>
#include <erp42_msgs/DriveCmd.h>
#include <erp42_msgs/ModeCmd.h>
#include <tf/transform_datatypes.h>
#include <cmath>

void GPS_STANLEY::init_dict()
/* INITIALIZE DICTIONARY!!! */
{
    // std::string assets_path = "/home/heven/erp_ws/src/heven_m_m/lane_ctrl/src/";

    std::vector<std::string> csv_file_path = {
        "/home/popular/catkin_ws/src/JaJuSung_HEVEN/main/jajusung_main/src/gps_test.csv"
        // "/home/pcy028x2/catkin_ws/src/JaJuSung_HEVEN/main/jajusung_main/src/gps_test_pcy.csv"
        // "/home/pcy028x2/catkin_ws/src/JaJuSung_HEVEN/main/jajusung_main/src/gps_test.csv"
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
                double third = std::stod(row[2]);
                ref_yaw.push_back(third);
                TRACK_DICT.emplace_back(first, second);
            }
        }
        csvfile.close();
        std::cout<<"=======csv close !"<<std::endl;
    }
}

double GPS_STANLEY::gps_stanley()
// SHOULD RETURN DELTA USING STANLEY
{
    // changes p_curr gps latitude, altitude to meterscale
    std::pair<double,double> p_curr = {latitude * METER_X_CONST, longitude * METER_Y_CONST};

    double distance = pow((p_curr.first - METER_X_CONST * TRACK_DICT[TRACK_IDX].first), 2) + pow((p_curr.second - METER_Y_CONST * TRACK_DICT[TRACK_IDX].second), 2);
    car_angle = imu_yaw;
    if (distance < DISTANCE_SQUARE) {
        TRACK_IDX += 1;
    }

    std::pair<double,double> p_targ = TRACK_DICT[TRACK_IDX];

    // curr_angle_error = GPS_STANLEY::find_angle_error(car_angle, p_curr, p_targ);
    curr_angle_error = GPS_STANLEY::find_angle_error(car_angle, ref_yaw[TRACK_IDX]);

    // HEADING ERROR
    final_steer_angle = GPS_STANLEY::_constraint(curr_angle_error);
    
    // LATERAL ERROR
    // lateral_error = pow((p_curr.first - METER_X_CONST * TRACK_DICT[TRACK_IDX].first), 2);
    // lateral_error = pow((p_curr.second - METER_Y_CONST * p_targ.second), 2);
    double targetdir_x = METER_Y_CONST * p_targ.second - p_curr.second;
    double targetdir_y = METER_X_CONST * p_targ.first - p_curr.first;
    
    distance = sqrt(distance);

    lateral_error = atan2(targetdir_y, targetdir_x);

    lateral_error = -sin(lateral_error)*distance;

    double stanley_delta = rad2deg(deg2rad(final_steer_angle) + atan(LANE_K*lateral_error/((VEL/3.6))));

    erp42_msgs::DriveCmd drive_cmd;
    drive_cmd.KPH = VEL;
    drive_cmd.Deg = (int)stanley_delta;
    // if(drive_cmd.Deg > 25) drive_cmd.Deg = 25;
    // else if(drive_cmd.Deg < -25) drive_cmd.Deg = -25;
    erp42_msgs::ModeCmd mode_cmd;
    mode_cmd.MorA = 0x01;
    mode_cmd.EStop = 0x00;
    mode_cmd.Gear = 0x00;
    std::cout<<"================lateral_error : "<<rad2deg(atan(LANE_K*lateral_error/((VEL/3.6))))<<'\n';
    std::cout<<"================heading_error : "<<final_steer_angle<<'\n';
    // std::cout<<"================heading_error not saturated : "<<curr_angle_error<<'\n';
    std::cout<<"================result_drive : "<<stanley_delta<<'\n';
    std::cout<<"================target_idx : "<<TRACK_IDX<<'\n';
    std::cout << imu_yaw << std::endl;
    drive_pub.publish(drive_cmd);
    mode_pub.publish(mode_cmd);

    return stanley_delta;
}

double GPS_STANLEY::find_angle_error(double car_angle, double ref_angle)
{
    // targetdir_x could be positive or negative value
    // targetedir_y might be always positive 
    // double targetdir_x = METER_Y_CONST * position_targ.second - position_curr.second;
    // double targetdir_y = METER_X_CONST * position_targ.first - position_curr.first;

    double target_angle = rad2deg(ref_angle);

    target_angle = - target_angle - 90; // 음???
    // target_angle = fmod(450.0 - target_angle, 360.0);
    ROS_INFO("Target angle : %.1f", target_angle);

    double angle_error = car_angle - target_angle;

    // if (angle_error >= 180) {
    //     angle_error -= 360;
    // }
    // if (angle_error <= -180) {
    //     angle_error += 360;
    // }

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

void GPS_STANLEY::chatterCallback_2(const sensor_msgs::Imu::ConstPtr& msg_2)
{
    tf::Quaternion q(
        msg_2->orientation.x,
        msg_2->orientation.y,
        msg_2->orientation.z,
        msg_2->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, imu_yaw);
    imu_yaw = - rad2deg(imu_yaw) - 90;
}