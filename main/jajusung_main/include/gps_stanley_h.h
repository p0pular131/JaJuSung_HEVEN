#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <map>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <erp42_msgs/DriveCmd.h>
#include <erp42_msgs/ModeCmd.h>

#include "ros/ros.h"


class GPS_STANLEY
{
public:
    GPS_STANLEY()
    {
        sub_1 = nh.subscribe<sensor_msgs::NavSatFix>("/ublox_gps/fix", 10, &GPS_STANLEY::chatterCallback_1, this);
        sub_2 = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &GPS_STANLEY::chatterCallback_2, this);

        drive_pub = nh.advertise<erp42_msgs::DriveCmd>("/drive", 10);
        mode_pub = nh.advertise<erp42_msgs::ModeCmd>("/mode", 10);
        
        init_dict();
    }
    
    double gps_stanley();
    
    void init_dict();

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
    ros::Publisher mode_pub;
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    
    double LANE_K = 0.05;
    double VEL = 5;

    double METER_X_CONST = 110000;
    double METER_Y_CONST = 88800;
    int GPS_TRACKING_SPEED = 7;
    int TRACK_IDX = 0;

    double DISTANCE_SQUARE = 1;
    std::vector<std::pair<double, double>> TRACK_DICT;
    std::vector<double> ref_yaw;
    int TRACK_NO = 1; // initialized as 1, used to find TRACK_DICT key 
    
    bool GPS_TRACK_END = false;

    double curr_angle_error = 0;
    double final_steer_angle = 0;
    double lateral_error = 0;

    double latitude=0;
    double longitude=0;
    double car_angle=0.0;

    double enu_imu_yaw = 0;
    double imu_yaw = 0;
};