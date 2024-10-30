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
#include <std_msgs/Float64.h>

#include "ros/ros.h"

using namespace std;
using namespace Eigen;

class GPS_STANLEY
{
public:
    GPS_STANLEY()
    {
        sub_1 = nh.subscribe<sensor_msgs::NavSatFix>("/ublox_gps/fix", 10, &GPS_STANLEY::chatterCallback_1, this);
        sub_2 = nh.subscribe<std_msgs::Float64>("/corrected_yaw", 10, &GPS_STANLEY::chatterCallback_2, this);

        // drive_pub = nh.advertise<erp42_msgs::DriveCmd>("/drive", 10);
        // mode_pub = nh.advertise<erp42_msgs::ModeCmd>("/mode", 10);
        
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
    void chatterCallback_2(const std_msgs::Float64::ConstPtr& msg_2);


private:
    ros::NodeHandle nh; 
    ros::Publisher drive_pub;
    ros::Publisher mode_pub;
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