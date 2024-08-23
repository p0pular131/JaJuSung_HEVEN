#ifndef TRACKER
#define TRACKER

#include <ros/ros.h>
#include <EKFH.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <erp42_msgs/DriveCmd.h>
#include <erp42_msgs/ModeCmd.h>
#include <lane_ctrl/lane_info.h>
#include <tf/transform_datatypes.h>
#include <vector>

#define WIDTH 3 // 차선폭
#define WB 1.04
#define PI 3.1415926
#define deg2rad(x) x*PI/180.0
#define rad2deg(x) x*180.0/PI

struct _Point
{
    double x;
    double y;
    double yaw;
};

struct _State
{
    double x;
    double y;
    double yaw;
    double velocity;
};

class path_tracker
{
public:
    path_tracker(ros::NodeHandle &nh);
    int VEL;
    int LAD;
    double PATH_K;
    std::string Path_path;
    std::vector<_Point> Global_Path;
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

private:
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher drive_pub_;

    _State car_pose_;
    int idx_;
    double error_front_axle_;

    void loadPathFromFile(const std::string &file_path); 
    void selectTarget();
    void calcError();
    double normalizeAngle(double angle);
    double stanley();
    void publishDriveCmd();
};

class Lane_Controller 
{
public : 
    int VEL;
    double LANE_K;
    Lane_Controller(EKF* input, ros::NodeHandle &nh);
    int normalize_delta(int delta);
private :
    ros::Publisher cmd_pub;
    ros::Publisher mod_pub;
    ros::Subscriber lane_sub;

    double this_left, this_right;
    double left_theta, right_theta;
        
    int stanley_delta;
    double lat_err, heading_err;

    int ekf_mode;
    bool drive_start = false;
    
    EKF* lane_ekf = NULL;

    void lane_cb(const lane_ctrl::lane_info::ConstPtr& data);
    void stanley();
};

#endif 
